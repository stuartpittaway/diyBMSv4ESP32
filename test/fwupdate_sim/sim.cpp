/*
 * PC simulation: firmware update over the RX/TX module chain with up to 16 modules
 *
 * Uses the REAL code of
 *   - bootloader core          ATTINYBootloader/src/bl_core.c
 *   - module application       ATTINYCellModule/src/fwupdate_module.cpp
 *   - ESP32 state machine      ESPController/src/ModuleFirmwareUpdate.cpp (without the ARDUINO part)
 *
 * Models: packet time per baud rate, forwarding module by module, processing time (CRC32
 * during the version check), CPU halt during flash writes (UART overrun), 64 byte RX buffer,
 * packet loss/corruption per hop, application start up time, bootloader watchdog, power loss,
 * broken flash cell, modules with old firmware, modules without bootloader, normal voltage
 * requests in parallel to the update.
 */
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <deque>
#include <queue>
#include <random>
#include <string>
#include <functional>

extern "C"
{
#include "bl_core.h"
}
#include "fwupdate_module.h"
#include "ModuleFirmwareUpdate.h"

typedef uint64_t us_t;
static us_t g_now = 0;
static std::mt19937 rng(1);
static double urand() { return std::uniform_real_distribution<double>(0, 1)(rng); }

static uint16_t crc16x(const uint8_t *p, int n)
{
    uint16_t crc = 0;
    while (n--)
    {
        crc ^= (uint16_t)(*p++) << 8;
        for (int i = 0; i < 8; i++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}
static void setcrc(uint8_t *p) { uint16_t c = crc16x(p, 38); p[38] = c & 0xFF; p[39] = c >> 8; }
static bool crcok(const uint8_t *p) { return crc16x(p, 38) == (uint16_t)(p[38] | (p[39] << 8)); }

// ------------------------------------------------------------------ module
struct Module
{
    int idx = 0;
    uint8_t flash[16384];
    uint8_t pagebuf[64];
    uint16_t latched = 0;
    uint8_t bootend = 6;
    bool legacy = false; // old firmware without update support
    enum Mode { APP, BOOT, STARTING } mode = APP;
    us_t startingUntil = 0;
    bl_state_t bl;
    uint32_t magic = 0;
    struct InPkt { us_t arrival; uint8_t d[40]; };
    std::deque<InPkt> inbox;
    us_t busyUntil = 0;
    us_t lastMine = 0;
    std::vector<std::pair<us_t, us_t>> halts;
    int stuckAddr = -1;
    int pagewritesThisPacket = 0;
    int resets = 0, pagewrites = 0, dropsOverflow = 0, dropsStarting = 0, overruns = 0;
    bool illegalWrite = false;
    bool appValid() const { return !(flash[FW_APP_START] == 0xFF && flash[FW_APP_START + 1] == 0xFF); }
};
static Module *g_cur = nullptr;

// ---- HAL for the bootloader core
extern "C" uint8_t bl_hal_flash_read(uint16_t a) { return g_cur->flash[a]; }
extern "C" void bl_hal_flash_fill(uint16_t a, const uint8_t *s, uint8_t n)
{
    for (int i = 0; i < n; i++)
    {
        g_cur->pagebuf[(a + i) & 63] = s[i];
        g_cur->latched = (a + i) & ~63;
    }
}
static void page_op(bool write)
{
    Module *m = g_cur;
    if (m->latched < FW_APP_START)
        m->illegalWrite = true; // the hardware would prevent this - must never happen
    for (int i = 0; i < 64; i++)
    {
        uint8_t v = write ? m->pagebuf[i] : 0xFF;
        if (m->latched + i == m->stuckAddr)
            v &= 0xFE; // broken cell
        m->flash[m->latched + i] = v;
    }
    memset(m->pagebuf, 0xFF, 64);
    m->pagewrites++;
    m->pagewritesThisPacket++;
}
extern "C" void bl_hal_flash_erase_write(uint16_t) { page_op(true); }
extern "C" void bl_hal_flash_erase(uint16_t a)
{
    g_cur->latched = a & ~63;
    page_op(false);
}
extern "C" void bl_hal_flash_bufclr(void) { memset(g_cur->pagebuf, 0xFF, 64); }
extern "C" uint16_t bl_hal_crc16(const uint8_t *p, uint8_t len) { return crc16x(p, len); }

// ---- HAL for the module application
uint8_t fw_hal_flash_read(uint16_t a) { return g_cur->flash[a]; }
uint8_t fw_hal_bootend() { return g_cur->bootend; }
uint16_t fw_hal_board_version() { return 450; }
static uint16_t g_own_len = 0;
uint16_t fw_hal_own_length() { return g_own_len; }

// ------------------------------------------------------------------ Simulation
struct Config
{
    const char *name;
    int modules = 16;
    uint32_t baud = 5000;
    double drop = 0.0, corrupt = 0.0;
    int alreadyNew = 0;     // this many modules (from the end) already have the new image
    int legacyModule = -1;  // module with old firmware without update support
    int noBootModule = -1;  // module without bootloader fuse
    int powerCutModule = -1;
    int stuckModule = -1;
    int abortAtModule = -1; // abort during the transfer to this module, then restart
    double limitSeconds = 4 * 3600;
    double pollSeconds = 6;
};

struct Result
{
    bool ok;
    double seconds;
};

static Result run(const Config &cfg, bool verbose = false)
{
    rng.seed(12345);
    g_now = 0;
    const int N = cfg.modules;
    const us_t pkt_us = (44ULL * 10ULL * 1000000ULL) / cfg.baud;

    std::vector<uint8_t> imgA(8000), imgB(9118);
    for (auto &b : imgA) b = rng() & 0xFF;
    for (auto &b : imgB) b = rng() & 0xFF;

    std::vector<Module> mods(N);
    for (int i = 0; i < N; i++)
    {
        Module &m = mods[i];
        m.idx = i;
        memset(m.flash, 0xFF, sizeof(m.flash));
        memset(m.flash, 0xAB, FW_APP_START); // Bootloader
        const std::vector<uint8_t> &img = (i >= N - cfg.alreadyNew) ? imgB : imgA;
        memcpy(m.flash + FW_APP_START, img.data(), img.size());
        memset(m.pagebuf, 0xFF, 64);
        m.legacy = (i == cfg.legacyModule);
        m.bootend = (i == cfg.noBootModule) ? 0 : 6;
        if (i == cfg.stuckModule)
            m.stuckAddr = FW_APP_START + 4000;
    }

    // ---- ESP32
    ModuleFwUpdateCore core;
    std::deque<PacketStruct> txq;  // normal requests (request_q)
    std::deque<PacketStruct> fwq;  // update packets (fwupd_tx_q, depth 2, priority)
    struct Ctx { std::deque<PacketStruct> *q; } ctx{&fwq};
    core.setHooks(
        [](void *c, PacketStruct *p) {
            auto *q = ((Ctx *)c)->q;
            if (q->size() >= 2) return false;
            if ((p->command & 0x0F) == FW_CMD_CONTROL) q->push_front(*p); else q->push_back(*p);
            return true;
        },
        [](void *) { return (uint32_t)(g_now / 1000); },
        [](void *, const char *msg) { if (getenv("SIMLOG")) printf("   [%8.1fs] %s\n", g_now / 1e6, msg); },
        &ctx);
    core.setFlushHook([](void *c) { ((Ctx *)c)->q->clear(); });

    // events
    enum EvType { ARRIVE, SERVICE, RESET, TICK, POLL };
    struct Ev { us_t t; int type; int target; uint8_t d[40]; uint32_t arg; uint64_t seq; };
    auto cmp = [](const Ev &a, const Ev &b) { return a.t != b.t ? a.t > b.t : a.seq > b.seq; };
    std::priority_queue<Ev, std::vector<Ev>, decltype(cmp)> ev(cmp);
    uint64_t evseq = 0;
    auto push = [&](us_t t, int type, int target, const uint8_t *d = nullptr, uint32_t arg = 0) {
        Ev e{t, type, target, {0}, arg, evseq++};
        if (d) memcpy(e.d, d, 40);
        ev.push(e);
    };

    // hop with loss/corruption
    auto hop = [&](us_t arrival, int target, uint8_t *d) {
        if (urand() < cfg.drop) return;
        if (urand() < cfg.corrupt) d[rng() % 40] ^= (1 << (rng() % 8));
        push(arrival, ARRIVE, target, d);
    };

    us_t espNextTx = 0;
    uint16_t sequence = 0;
    int normalReplies = 0, normalBadValues = 0, fwReplies = 0;

    auto doReset = [&](Module &m, bool requestBoot) {
        m.resets++;
        m.inbox.clear();
        m.halts.clear();
        bool req = requestBoot && m.magic == FW_BOOT_MAGIC;
        m.magic = 0;
        if (m.bootend == 6 && (req || !m.appValid()))
        {
            m.mode = Module::BOOT;
            bl_init(&m.bl);
            m.lastMine = g_now;
            m.busyUntil = g_now;
        }
        else
        {
            m.mode = Module::STARTING;
            m.startingUntil = g_now + 1300000; // setup(): LED flashes 4x
            m.busyUntil = m.startingUntil;
        }
    };

    // module processes a packet, returns the processing time
    auto process = [&](Module &m, uint8_t *d, bool &resetAfter, bool &enterBoot) -> us_t {
        g_cur = &m;
        resetAfter = enterBoot = false;
        if (m.mode == Module::BOOT)
        {
            m.pagewritesThisPacket = 0;
            uint8_t act = bl_handle_packet(&m.bl, d);
            if (act & BL_ACT_MINE) m.lastMine = g_now;
            if (act & BL_ACT_RESET) resetAfter = true;
            us_t t = 1000 + (us_t)m.pagewritesThisPacket * 4000;
            // CRC32 check in the bootloader (bitwise, ~0.3 s for 9 KB)
            if ((d[2] & 0x0F) == FW_CMD_CONTROL && d[6] == FW_FINISH) t += 600000;
            if (m.pagewritesThisPacket) m.halts.push_back({g_now, g_now + (us_t)m.pagewritesThisPacket * 4000});
            return t;
        }
        // application (like PacketProcessor::onPacketReceived)
        PacketStruct *p = (PacketStruct *)d;
        if (!crcok(d)) return 2000;
        uint8_t me = p->hops;
        p->hops++;
        us_t t = 2000;
        if (p->start_address <= me && p->end_address >= me)
        {
            uint8_t cmd = p->command & 0x0F;
            bool processed = false;
            if (cmd == FW_CMD_CONTROL && !m.legacy)
            {
                bool eb = false;
                if ((p->moduledata[0] & 0xFF) == FW_ANNOUNCE) t += 110000;
                processed = FwUpdate_HandleControl(p, me, &eb);
                enterBoot = eb;
            }
            else if (cmd == 1)
            {
                p->moduledata[me % 16] = 3300;
                processed = true;
            }
            if (processed) p->command |= 0x80;
        }
        setcrc(d);
        return t;
    };

    // ---- Start
    bool started = core.begin(imgB.data(), imgB.size(), 450, 0x12345678, N, cfg.baud);
    if (!started) { printf("begin() failed: %s\n", core.message()); return {false, 0}; }
    push(0, TICK, 0);
    push(6000000, POLL, 0);
    if (cfg.powerCutModule >= 0) push(0, RESET, -100 - cfg.powerCutModule); // placeholder, triggered when needed

    bool powerCutDone = false, abortDone = false, restarted = false;
    const us_t limit = (us_t)(cfg.limitSeconds * 1e6);

    while (!ev.empty())
    {
        Ev e = ev.top();
        ev.pop();
        g_now = e.t;
        if (g_now > limit) break;

        switch (e.type)
        {
        case TICK:
        {
            // power loss in the middle of the transfer
            if (cfg.powerCutModule >= 0 && !powerCutDone && core.phase() == FwUpdPhase::Data &&
                core.currentModule() == cfg.powerCutModule && core.chunksAcked() > core.chunksTotal() / 2)
            {
                powerCutDone = true;
                if (verbose) printf("   ** power loss module %d at block %u\n", cfg.powerCutModule, core.chunksAcked());
                doReset(mods[cfg.powerCutModule], false);
            }
            // abort and later restart
            if (cfg.abortAtModule >= 0 && !abortDone && core.phase() == FwUpdPhase::Data &&
                core.currentModule() == cfg.abortAtModule && core.chunksAcked() > 50)
            {
                abortDone = true;
                core.abort();
                if (verbose) printf("   ** abort at module %d\n", cfg.abortAtModule);
            }
            if (abortDone && !restarted && core.phase() == FwUpdPhase::Aborted && g_now > 30000000ULL + 0)
            {
                restarted = true;
                txq.clear();
                fwq.clear();
                core.begin(imgB.data(), imgB.size(), 450, 0x12345678, N, cfg.baud);
                if (verbose) printf("   ** update restarted\n");
            }

            core.poll();

            // ESP32 transmit_task
            if (g_now >= espNextTx && (!fwq.empty() || !txq.empty()))
            {
                PacketStruct p;
                if (!fwq.empty()) { p = fwq.front(); fwq.pop_front(); }
                else { p = txq.front(); txq.pop_front(); }
                uint8_t cmd = p.command & 0x0F;
                bool fw = cmd == FW_CMD_CONTROL || cmd == FW_CMD_DATA;
                if (!fw) p.sequence = ++sequence;
                else if (cmd != FW_CMD_DATA) p.sequence = 0;
                setcrc((uint8_t *)&p);
                uint8_t d[40];
                memcpy(d, &p, 40);
                hop(g_now + pkt_us, 0, d);
                us_t delay = fw ? core.packetIntervalMs() * 1000ULL : (core.busy() ? 300000ULL : 700000ULL);
                if (cmd == FW_CMD_CONTROL && (p.moduledata[0] & 0xFF) == FW_ANNOUNCE) delay = 1000000;
                espNextTx = g_now + delay;
            }

            // bootloader watchdog
            for (auto &m : mods)
            {
                if (m.mode != Module::BOOT) continue;
                if (!m.appValid()) m.lastMine = g_now; // wdr in the main loop
                if (g_now - m.lastMine > 8000000ULL) doReset(m, false);
            }

            bool keepGoing = core.busy() || (cfg.abortAtModule >= 0 && !restarted);
            if (keepGoing) push(g_now + 5000, TICK, 0);
            break;
        }
        case POLL:
        {
            // normal voltage request every 6 s (enqueue_task)
            if (txq.size() < 30)
            {
                PacketStruct p;
                memset(&p, 0, sizeof(p));
                p.start_address = 0;
                p.end_address = N - 1;
                p.command = 1;
                for (int i = 0; i < 16; i++) p.moduledata[i] = 0xFFFF;
                txq.push_back(p);
            }
            if (core.busy() || (cfg.abortAtModule >= 0 && !restarted)) push(g_now + (us_t)(cfg.pollSeconds * 1e6), POLL, 0);
            break;
        }
        case RESET:
        {
            if (e.target >= 0) doReset(mods[e.target], e.arg == 1);
            break;
        }
        case ARRIVE:
        {
            if (e.target == N)
            {
                // ESP32 receives
                PacketStruct p;
                memcpy(&p, e.d, 40);
                if (!crcok(e.d)) break;
                uint8_t cmd = p.command & 0x0F;
                if (cmd == FW_CMD_CONTROL || cmd == FW_CMD_DATA)
                {
                    fwReplies++;
                    core.onReply(&p);
                }
                else if (cmd == 1 && (p.command & 0x80))
                {
                    normalReplies++;
                    int skip = core.moduleInBootloader();
                    for (int i = 0; i < N; i++)
                        if (i != skip && i != cfg.stuckModule && (p.moduledata[i] != 3300)) normalBadValues++;
                }
                break;
            }
            Module &m = mods[e.target];
            if (m.mode == Module::STARTING)
            {
                if (g_now < m.startingUntil) { m.dropsStarting++; break; }
                m.mode = Module::APP;
            }
            // UART overrun while the bootloader writes flash (CPU halted)
            if (m.mode == Module::BOOT)
            {
                for (auto &h : m.halts)
                    if (h.first < g_now && h.second > g_now - pkt_us)
                    {
                        e.d[rng() % 40] ^= 0x10;
                        m.overruns++;
                        break;
                    }
            }
            size_t waiting = 0;
            for (auto &x : m.inbox) (void)x, waiting++;
            if (waiting >= 2) { m.dropsOverflow++; break; }
            Module::InPkt ip;
            ip.arrival = g_now;
            memcpy(ip.d, e.d, 40);
            m.inbox.push_back(ip);
            push(std::max(g_now, m.busyUntil), SERVICE, e.target);
            break;
        }
        case SERVICE:
        {
            Module &m = mods[e.target];
            if (m.inbox.empty()) break;
            if (g_now < m.busyUntil) { push(m.busyUntil, SERVICE, e.target); break; }
            if (m.mode == Module::STARTING) { m.inbox.clear(); break; }
            Module::InPkt ip = m.inbox.front();
            m.inbox.pop_front();
            bool resetAfter, enterBoot;
            us_t proc = process(m, ip.d, resetAfter, enterBoot);
            us_t txStart = g_now + proc;
            us_t arrivalNext = txStart + pkt_us;
            m.busyUntil = arrivalNext;
            hop(arrivalNext, e.target + 1, ip.d);
            if (resetAfter) push(arrivalNext, RESET, e.target, nullptr, 0);
            if (enterBoot) { m.magic = FW_BOOT_MAGIC; push(arrivalNext, RESET, e.target, nullptr, 1); }
            if (!m.inbox.empty()) push(m.busyUntil, SERVICE, e.target);
            break;
        }
        }
        if (!core.busy() && !(cfg.abortAtModule >= 0 && !restarted) && ev.empty()) break;
    }

    // ---- evaluation
    bool ok = true;
    int updated = 0;
    for (int i = 0; i < N; i++)
    {
        Module &m = mods[i];
        bool hasB = memcmp(m.flash + FW_APP_START, imgB.data(), imgB.size()) == 0;
        bool expectB = !(i == cfg.legacyModule || i == cfg.noBootModule || i == cfg.stuckModule);
        bool expectFailed = (i == cfg.stuckModule);
        FwModResult r = core.result(i);
        bool good;
        if (expectFailed)
            good = r == FwModResult::Failed && m.mode == Module::BOOT;
        else if (expectB)
            good = hasB && m.appValid() && (r == FwModResult::Updated || r == FwModResult::UpToDate) && m.mode != Module::BOOT;
        else
            good = !hasB && (r == FwModResult::Unknown || r == FwModResult::NoBootloader);
        if (m.illegalWrite) good = false;
        if (r == FwModResult::Updated) updated++;
        if (!good || verbose)
            printf("   module %2d: %-14s flash=%s mode=%s resets=%d pages=%d overruns=%d %s\n", i,
                   ModuleFwUpdateCore::resultName(r), hasB ? "new" : "old",
                   m.mode == Module::BOOT ? "BOOT" : "APP", m.resets, m.pagewrites, m.overruns, good ? "" : "<-- ERROR");
        ok = ok && good;
    }
    // check the checksum report (FW_CMD_REPORT) of the updated modules against the image
    {
        uint32_t want = ModuleFwUpdateCore::crc32(imgB.data(), imgB.size());
        for (int i = 0; i < N; i++)
        {
            if (core.result(i) != FwModResult::Updated) continue;
            g_cur = &mods[i];
            g_own_len = imgB.size();
            FwUpdate_InitOwnCrc();
            PacketStruct p;
            memset(&p, 0, sizeof(p));
            p.start_address = (i / 8) * 8;
            p.end_address = p.start_address + 7;
            p.command = FW_CMD_REPORT;
            if (!FwUpdate_HandleReport(&p, i)) { printf("   module %d: report refused\n", i); ok = false; continue; }
            int rel = i - p.start_address;
            uint32_t got = (uint32_t)p.moduledata[2 * rel] | ((uint32_t)p.moduledata[2 * rel + 1] << 16);
            if (got != want) { printf("   module %d: checksum %08x instead of %08x\n", i, got, want); ok = false; }
        }
    }
    if (normalBadValues) { printf("   %d wrong readings in normal replies\n", normalBadValues); ok = false; }
    double secs = core.elapsedMs() / 1000.0;
    printf("%-58s %s  %6.0f s  updated=%2d  retransmissions=%5lu  normal replies=%d  [%s]\n", cfg.name,
           ok ? "OK    " : "ERROR ", secs, updated, (unsigned long)core.retransmissions(), normalReplies, core.message());
    return {ok, secs};
}

int main(int argc, char **)
{
    bool verbose = argc > 1;
    std::vector<Config> tests;
    Config c;

    c = Config(); c.name = "16 modules, 5000 baud, no errors"; tests.push_back(c);
    c = Config(); c.name = "16 modules, 10000 baud, no errors"; c.baud = 10000; tests.push_back(c);
    c = Config(); c.name = "16 modules, 5 already up to date"; c.alreadyNew = 5; tests.push_back(c);
    c = Config(); c.name = "16 modules, all already up to date"; c.alreadyNew = 16; tests.push_back(c);
    c = Config(); c.name = "2% loss + 1% bit errors per hop"; c.drop = 0.02; c.corrupt = 0.01; tests.push_back(c);
    c = Config(); c.name = "1% loss + 0.5% bit errors per hop, 10000 baud"; c.drop = 0.01; c.corrupt = 0.005; c.baud = 10000; tests.push_back(c);
    c = Config(); c.name = "power loss module 7 during transfer"; c.powerCutModule = 7; tests.push_back(c);
    c = Config(); c.name = "module 3 old firmware (no update support)"; c.legacyModule = 3; tests.push_back(c);
    c = Config(); c.name = "module 12 without bootloader fuse"; c.noBootModule = 12; tests.push_back(c);
    c = Config(); c.name = "module 9 broken flash cell (CRC error)"; c.stuckModule = 9; tests.push_back(c);
    c = Config(); c.name = "abort at module 4, restarted later"; c.abortAtModule = 4; tests.push_back(c);
    c = Config(); c.name = "16 modules, 10000 baud, polling every 2 s, 1% loss"; c.baud = 10000; c.pollSeconds = 2; c.drop = 0.01; tests.push_back(c);
    c = Config(); c.name = "1 module, 10000 baud"; c.modules = 1; c.baud = 10000; tests.push_back(c);
    c = Config(); c.name = "4 modules, 2400 baud"; c.modules = 4; c.baud = 2400; tests.push_back(c);

    int failed = 0;
    const char *only = getenv("SIMONLY");
    int count = 0;
    for (auto &t : tests)
    {
        if (only && !strstr(t.name, only)) continue;
        count++;
        if (!run(t, verbose).ok) failed++;
    }
    printf("\n%d of %d scenarios passed\n", count - failed, count);
    return failed ? 1 : 0;
}
