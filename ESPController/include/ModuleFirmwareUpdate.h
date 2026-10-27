/*
 * DIYBMS ESP32 - firmware update of the cell modules over the RX/TX module chain
 *
 * ModuleFwUpdateCore is the pure state machine without FreeRTOS/Arduino dependencies
 * (it is also tested in the PC simulation test/fwupdate_sim).
 * The glue to queues, task and web server is in ModuleFirmwareUpdate.cpp
 * (section "ESP32 glue").
 *
 * Sequence:
 *   Announce (one packet per 16 modules) -> every module compares itself (CRC32 of its flash)
 *   for every module that differs, one after the other:
 *     EnterBoot -> WaitBoot -> Start -> Data (Go-Back-N, window W) -> Finish -> WaitApp -> Verify
 *   Packets use the normal transmit path (transmit_task) and therefore pass through all
 *   modules in front of the target module, which forward them unchanged.
 */
#ifndef ModuleFirmwareUpdate_H_
#define ModuleFirmwareUpdate_H_

#include <stdint.h>
#include <string.h>

#if defined(ARDUINO)
#include "defines.h"
#else
#include "packet_struct_host.h"
#endif

#include "fwupdate_protocol.h"

#define FWUPD_MAX_MODULES 128
// Retries per request (packet loss in the chain) and attempts per module
#define FWUPD_TRIES 8
#define FWUPD_MODULE_ATTEMPTS 4
// Poll interval while the module waits in the bootloader for START (bootloader watchdog 8 s)
#define FWUPD_KEEPALIVE_MS 2000

enum class FwUpdPhase : uint8_t
{
    Idle = 0,
    Announce,
    EnterBoot,
    WaitBoot,
    Start,
    Data,
    Finish,
    WaitApp,
    Verify,
    Done,
    Aborted
};

enum class FwModResult : uint8_t
{
    Unknown = 0,  // no reply (old firmware without update support)
    NoBootloader, // firmware without bootloader / wrong hardware
    UpToDate,     // same firmware, nothing to do
    Pending,      // will be updated, waiting
    InProgress,
    Updated,
    Failed
};

class ModuleFwUpdateCore
{
public:
    typedef bool (*SendFn)(void *ctx, PacketStruct *p); // false = queue full
    typedef uint32_t (*NowFn)(void *ctx);
    typedef void (*LogFn)(void *ctx, const char *msg);

    typedef void (*FlushFn)(void *ctx);
    // optional: called on a retransmission to drop blocks that are still queued (outdated)
    void setFlushHook(FlushFn flush) { _flush = flush; }

    void setHooks(SendFn send, NowFn now, LogFn log, void *ctx)
    {
        _send = send;
        _now = now;
        _log = log;
        _ctx = ctx;
    }

    // Starts the update. image points to the complete application image (from 0x0600),
    // it has to stay valid until the end.
    bool begin(const uint8_t *image, uint16_t len, uint16_t boardVersion, uint32_t gitVersion,
               uint8_t numModules, uint32_t baudRate);
    void abort();

    // Reply packet with command 12/13 (from any task -> pass it via a queue to the poll() task)
    void onReply(const PacketStruct *p);
    // call regularly (e.g. every 20 ms)
    void poll();

    bool busy() const { return _phase != FwUpdPhase::Idle && _phase != FwUpdPhase::Done && _phase != FwUpdPhase::Aborted; }
    // Module that is currently in the bootloader (ignore its readings), otherwise -1
    int16_t moduleInBootloader() const;
    // Interval between two update packets for the transmit task
    uint32_t packetIntervalMs() const { return _intervalMs; }

    // ---- status for the web interface ----
    FwUpdPhase phase() const { return _phase; }
    uint8_t currentModule() const { return _current; }
    uint8_t numModules() const { return _numModules; }
    FwModResult result(uint8_t m) const { return m < FWUPD_MAX_MODULES ? _result[m] : FwModResult::Unknown; }
    uint16_t chunksTotal() const { return _nchunks; }
    uint16_t chunksAcked() const { return _base; }
    uint8_t modulesTodo() const { return _todoCount; }
    uint8_t modulesUpdated() const { return _updatedCount; }
    uint8_t modulesFailed() const { return _failedCount; }
    uint32_t elapsedMs() const;
    const char *message() const { return _msg; }
    uint32_t imageCrc() const { return _crc; }
    uint16_t imageLength() const { return _len; }
    uint32_t retransmissions() const { return _retransmissions; }
    uint16_t moduleSeconds(uint8_t m) const { return m < FWUPD_MAX_MODULES ? _modSeconds[m] : 0; }
    uint16_t moduleRetransmissions(uint8_t m) const { return m < FWUPD_MAX_MODULES ? _modRetrans[m] : 0; }
    // Remaining time in seconds (estimate), 0 if unknown
    uint32_t remainingSeconds() const;

    static const char *phaseName(FwUpdPhase p);
    static const char *resultName(FwModResult r);
    static uint32_t crc32(const uint8_t *data, uint32_t len);

private:
    SendFn _send = nullptr;
    FlushFn _flush = nullptr;
    NowFn _now = nullptr;
    LogFn _log = nullptr;
    void *_ctx = nullptr;

    const uint8_t *_image = nullptr;
    uint16_t _len = 0;
    uint16_t _nchunks = 0;
    uint32_t _crc = 0;
    uint16_t _board = 0;
    uint32_t _git = 0;
    uint8_t _numModules = 0;

    // Times (ms)
    uint32_t _intervalMs = 250;
    uint32_t _replyTimeoutMs = 5000;
    uint32_t _announceTimeoutMs = 8000;
    uint16_t _window = 8;

    FwUpdPhase _phase = FwUpdPhase::Idle;
    FwModResult _result[FWUPD_MAX_MODULES];
    uint8_t _current = 0;
    uint8_t _todoCount = 0, _updatedCount = 0, _failedCount = 0;
    uint8_t _moduleAttempts = 0;

    // general request/reply state
    uint32_t _phaseStartMs = 0;
    uint32_t _lastTxMs = 0;
    uint8_t _tries = 0;
    bool _sent = false;
    bool _gotReply = false;

    // Announce
    uint8_t _groupStart = 0;
    uint16_t _maskNeeds = 0, _maskBad = 0, _maskBoot = 0;
    bool _verifyOk = false, _verifyAnswered = false;

    // Data (Go-Back-N)
    uint16_t _base = 0;
    uint16_t _sendPtr = 0;
    uint16_t _lastGoBack = 0xFFFF;
    uint32_t _lastGoBackMs = 0;
    uint32_t _goBackHoldoffMs = 3000;
    uint32_t _lastProgressMs = 0;
    uint8_t _noProgress = 0;
    uint32_t _retransmissions = 0;
    uint16_t _modSeconds[FWUPD_MAX_MODULES];
    uint16_t _modRetrans[FWUPD_MAX_MODULES];
    uint32_t _modStartMs = 0;
    uint32_t _doneMsSum = 0;
    uint8_t _doneCount = 0;
    void addRetrans(uint32_t n)
    {
        _retransmissions += n;
        if (_current < FWUPD_MAX_MODULES)
            _modRetrans[_current] = (uint16_t)((_modRetrans[_current] + n) > 0xFFFF ? 0xFFFF : _modRetrans[_current] + n);
    }

    // last bootloader reply

    uint16_t _blNext = 0;
    uint16_t _blStatus = 0;
    bool _blProcessed = false;
    uint16_t _appAck = 0;

    uint32_t _startedMs = 0, _finishedMs = 0;
    char _msg[96] = {0};

    uint32_t now() { return _now ? _now(_ctx) : 0; }
    void log(const char *fmt, ...);
    void setPhase(FwUpdPhase p);
    bool sendControl(uint8_t first, uint8_t last, uint8_t sub);
    bool sendChunk(uint16_t idx);
    bool sendAnnounceGroup();
    void processAnnounceMasks();
    bool selectNextModule();
    void moduleRetry(const char *why);
    void moduleDone(bool ok);
    void startModule();
};

#endif
