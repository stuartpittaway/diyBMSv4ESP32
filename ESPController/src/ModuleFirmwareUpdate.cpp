/*
 * DIYBMS ESP32 - firmware update of the cell modules over the RX/TX module chain
 * (description see include/ModuleFirmwareUpdate.h and ../common/fwupdate_protocol.h)
 */

#if defined(ARDUINO)
#define USE_ESP_IDF_LOG 1
static constexpr const char *const TAG = "diybms-modfw";
#endif

#include "ModuleFirmwareUpdate.h"
#include <stdarg.h>
#include <stdio.h>

// =====================================================================================
//  State machine (platform independent)
// =====================================================================================

static uint16_t crc16_xmodem(const uint8_t *p, uint16_t len)
{
    uint16_t crc = 0;
    while (len--)
    {
        crc ^= (uint16_t)(*p++) << 8;
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

uint32_t ModuleFwUpdateCore::crc32(const uint8_t *data, uint32_t len)
{
    uint32_t c = 0xFFFFFFFFUL;
    for (uint32_t i = 0; i < len; i++)
        c = fw_crc32_update(c, data[i]);
    return ~c;
}

const char *ModuleFwUpdateCore::phaseName(FwUpdPhase p)
{
    switch (p)
    {
    case FwUpdPhase::Idle: return "Ready";
    case FwUpdPhase::Announce: return "Version check";
    case FwUpdPhase::EnterBoot: return "Requesting bootloader";
    case FwUpdPhase::WaitBoot: return "Waiting for bootloader";
    case FwUpdPhase::Start: return "Start";
    case FwUpdPhase::Data: return "Transfer";
    case FwUpdPhase::Finish: return "Verify/finish";
    case FwUpdPhase::WaitApp: return "Module restart";
    case FwUpdPhase::Verify: return "Verification";
    case FwUpdPhase::Done: return "Done";
    case FwUpdPhase::Aborted: return "Aborted";
    }
    return "?";
}

const char *ModuleFwUpdateCore::resultName(FwModResult r)
{
    switch (r)
    {
    case FwModResult::Unknown: return "no reply";
    case FwModResult::NoBootloader: return "no bootloader";
    case FwModResult::UpToDate: return "up to date";
    case FwModResult::Pending: return "waiting";
    case FwModResult::InProgress: return "running";
    case FwModResult::Updated: return "updated";
    case FwModResult::Failed: return "failed";
    }
    return "?";
}

void ModuleFwUpdateCore::log(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(_msg, sizeof(_msg), fmt, ap);
    va_end(ap);
    if (_log)
        _log(_ctx, _msg);
}

uint32_t ModuleFwUpdateCore::elapsedMs() const
{
    if (_startedMs == 0)
        return 0;
    if (_finishedMs != 0)
        return _finishedMs - _startedMs;
    return (_now ? _now(_ctx) : 0) - _startedMs;
}

uint32_t ModuleFwUpdateCore::remainingSeconds() const
{
    if (!busy() || _phase == FwUpdPhase::Announce)
        return 0;
    // Time per module: average of the finished modules, otherwise estimated from the packet interval
    uint32_t perModule = _doneCount ? _doneMsSum / _doneCount : (uint32_t)_nchunks * _intervalMs + 8000;
    uint32_t pending = 0;
    for (uint16_t m = 0; m < _numModules; m++)
        if (_result[m] == FwModResult::Pending)
            pending++;
    uint32_t t = _now ? _now(_ctx) : 0;
    uint32_t inCurrent = t - _modStartMs;
    uint32_t currentLeft = perModule > inCurrent ? perModule - inCurrent : 2000;
    return (pending * perModule + currentLeft + 999) / 1000;
}

int16_t ModuleFwUpdateCore::moduleInBootloader() const
{
    switch (_phase)
    {
    case FwUpdPhase::WaitBoot:
    case FwUpdPhase::Start:
    case FwUpdPhase::Data:
    case FwUpdPhase::Finish:
    case FwUpdPhase::WaitApp:
        return _current;
    case FwUpdPhase::EnterBoot:
        // after the first request the module may already be in the bootloader
        return _sent ? _current : -1;
    case FwUpdPhase::Aborted:
        // aborted during the transfer: the module stays in the bootloader
        return _result[_current] == FwModResult::InProgress ? _current : -1;
    default:
        return -1;
    }
}

void ModuleFwUpdateCore::setPhase(FwUpdPhase p)
{
    _phase = p;
    _phaseStartMs = now();
    _tries = 0;
    _sent = false;
    _gotReply = false;
    _blProcessed = false;
}

bool ModuleFwUpdateCore::begin(const uint8_t *image, uint16_t len, uint16_t boardVersion, uint32_t gitVersion,
                               uint8_t numModules, uint32_t baudRate)
{
    if (busy())
    {
        log("Update is already running");
        return false;
    }
    if (image == nullptr || len == 0 || len > FW_APP_MAX)
    {
        log("Invalid image (%u bytes, max. %u)", (unsigned)len, (unsigned)FW_APP_MAX);
        return false;
    }
    if (numModules == 0 || numModules > FWUPD_MAX_MODULES || baudRate == 0)
    {
        log("Invalid number of modules / baud rate");
        return false;
    }

    _image = image;
    _len = len;
    _nchunks = (len + FW_CHUNK_SIZE - 1) / FW_CHUNK_SIZE;
    _crc = crc32(image, len);
    _board = boardVersion;
    _git = gitVersion;
    _numModules = numModules;

    // Timing from baud rate and chain length
    // one packet = start byte + 40 bytes + a few escape bytes, 10 bits per byte
    uint32_t packetMs = (44UL * 10UL * 1000UL + baudRate - 1) / baudRate;
    _intervalMs = packetMs + FW_PACING_GAP_MS;
    uint32_t ringMs = (uint32_t)(numModules + 1) * (packetMs + 20) + 300;
    uint32_t w = ringMs / _intervalMs + 2;
    _window = (uint16_t)(w < 3 ? 3 : (w > 24 ? 24 : w));
    // normal requests (700..900 ms interval) can be ahead in the transmit queue;
    // control packets have priority, at most one normal request (<= 900 ms) can run before them
    _replyTimeoutMs = 2 * ringMs + 2500;
    // a retransmitted block needs queue + ring time before its acknowledge can arrive
    _goBackHoldoffMs = ringMs + 4 * _intervalMs + 300;
    _announceTimeoutMs = _replyTimeoutMs + (uint32_t)numModules * 250;

    for (uint16_t i = 0; i < FWUPD_MAX_MODULES; i++)
    {
        _result[i] = FwModResult::Unknown;
        _modSeconds[i] = 0;
        _modRetrans[i] = 0;
    }
    _doneMsSum = 0;
    _doneCount = 0;

    _todoCount = _updatedCount = _failedCount = 0;
    _retransmissions = 0;
    _groupStart = 0;
    _maskNeeds = _maskBad = _maskBoot = 0;
    _base = _sendPtr = 0;
    _startedMs = now();
    if (_startedMs == 0)
        _startedMs = 1;
    _finishedMs = 0;

    log("Start: %u bytes, CRC32 %08lx, %u modules, packet interval %lu ms, window %u",
        (unsigned)len, (unsigned long)_crc, (unsigned)numModules, (unsigned long)_intervalMs, (unsigned)_window);
    setPhase(FwUpdPhase::Announce);
    return true;
}

void ModuleFwUpdateCore::abort()
{
    if (!busy())
        return;
    // If the module is in the bootloader and the old application is still valid, send it back.
    // During the transfer (application already erased) the bootloader refuses FW_EXIT and
    // waits for a new attempt.
    if (moduleInBootloader() >= 0)
    {
        sendControl(_current, _current, FW_EXIT);
    }
    log("Aborted at module %u (%s)", (unsigned)_current, phaseName(_phase));
    _finishedMs = now();
    _phase = FwUpdPhase::Aborted;
}

bool ModuleFwUpdateCore::sendControl(uint8_t first, uint8_t last, uint8_t sub)
{
    PacketStruct p;
    memset(&p, 0, sizeof(p));
    p.start_address = first;
    p.end_address = last;
    p.command = FW_CMD_CONTROL;
    p.moduledata[FW_MD_SUB] = sub;
    p.moduledata[FW_MD_LEN] = _len;
    p.moduledata[FW_MD_CRC_LO] = (uint16_t)(_crc & 0xFFFF);
    p.moduledata[FW_MD_CRC_HI] = (uint16_t)(_crc >> 16);
    p.moduledata[FW_MD_BOARD] = _board;
    p.moduledata[FW_MD_GIT_HI] = (uint16_t)(_git >> 16);
    p.moduledata[FW_MD_GIT_LO] = (uint16_t)(_git & 0xFFFF);
    if (_send == nullptr || !_send(_ctx, &p))
        return false;
    _sent = true;
    _tries++;
    _lastTxMs = now();
    return true;
}

bool ModuleFwUpdateCore::sendChunk(uint16_t idx)
{
    PacketStruct p;
    memset(&p, 0, sizeof(p));
    p.start_address = _current;
    p.end_address = _current;
    p.command = FW_CMD_DATA;
    p.sequence = idx; // block number (the transmit task does not overwrite it for FW_CMD_DATA)
    uint8_t buf[FW_CHUNK_SIZE];
    memset(buf, 0xFF, sizeof(buf));
    uint32_t off = (uint32_t)idx * FW_CHUNK_SIZE;
    uint32_t n = (_len - off) < FW_CHUNK_SIZE ? (_len - off) : FW_CHUNK_SIZE;
    memcpy(buf, _image + off, n);
    memcpy(p.moduledata, buf, FW_CHUNK_SIZE); // little endian on ESP32 and AVR
    if (_send == nullptr || !_send(_ctx, &p))
        return false;
    _lastTxMs = now();
    return true;
}

bool ModuleFwUpdateCore::sendAnnounceGroup()
{
    uint8_t last = _groupStart + 15;
    if (last >= _numModules)
        last = _numModules - 1;
    _maskNeeds = _maskBad = _maskBoot = 0;
    return sendControl(_groupStart, last, FW_ANNOUNCE);
}

void ModuleFwUpdateCore::onReply(const PacketStruct *p)
{
    if (!busy() || p == nullptr)
        return;
    if (crc16_xmodem((const uint8_t *)p, sizeof(PacketStruct) - 2) != p->crc)
        return;

    const uint8_t cmd = p->command & 0x0F;
    const bool processed = (p->command & 0x80) != 0;
    const uint8_t sub = (uint8_t)(p->moduledata[FW_MD_SUB] & 0xFF);

    if (cmd == FW_CMD_CONTROL && sub == FW_ANNOUNCE)
    {
        if (_phase == FwUpdPhase::Announce && p->start_address == _groupStart && !_gotReply)
        {
            uint8_t last = _groupStart + 15;
            if (last >= _numModules)
                last = _numModules - 1;
            for (uint8_t m = _groupStart; m <= last; m++)
            {
                uint16_t bit = (uint16_t)1 << (m & 0x0F);
                if (!(p->moduledata[FW_MD_SEEN_MASK] & bit))
                    _result[m] = FwModResult::Unknown;
                else if (p->moduledata[FW_MD_BADHW_MASK] & bit)
                    _result[m] = FwModResult::NoBootloader;
                else if ((p->moduledata[FW_MD_NEEDS_MASK] & bit) || (p->moduledata[FW_MD_INBOOT_MASK] & bit))
                    _result[m] = FwModResult::Pending;
                else
                    _result[m] = FwModResult::UpToDate;
            }
            _gotReply = true;
        }
        else if (_phase == FwUpdPhase::Verify && p->start_address == _current && p->end_address == _current)
        {
            uint16_t bit = (uint16_t)1 << (_current & 0x0F);
            _verifyAnswered = processed && (p->moduledata[FW_MD_SEEN_MASK] & bit);
            _verifyOk = _verifyAnswered && !(p->moduledata[FW_MD_NEEDS_MASK] & bit) && !(p->moduledata[FW_MD_INBOOT_MASK] & bit) &&
                        !(p->moduledata[FW_MD_BADHW_MASK] & bit);
            _gotReply = true;
        }
        return;
    }

    // from here only replies of the current module
    if (p->start_address != _current || p->end_address != _current)
        return;

    if (cmd == FW_CMD_CONTROL)
    {
        bool match = (sub == FW_ENTER_BOOT && _phase == FwUpdPhase::EnterBoot) ||
                     (sub == FW_STATUS && _phase == FwUpdPhase::WaitBoot) ||
                     (sub == FW_START && _phase == FwUpdPhase::Start) ||
                     (sub == FW_FINISH && _phase == FwUpdPhase::Finish);
        if (match)
        {
            _gotReply = true;
            _blProcessed = processed;
            _appAck = p->moduledata[FW_MD_APP_ACK];
            _blNext = p->moduledata[FW_MD_BL_NEXT];
            _blStatus = p->moduledata[FW_MD_BL_STATUS];
        }
        return;
    }

    if (cmd == FW_CMD_DATA && _phase == FwUpdPhase::Data && processed)
    {
        uint16_t idx = p->sequence;
        uint16_t next = p->moduledata[FW_MD_BL_NEXT];
        uint16_t st = p->moduledata[FW_MD_BL_STATUS];
        if ((st & 0xFF00) != FW_BL_SIGNATURE)
            return;
        uint8_t state = st & FW_BL_STATE_MASK;
        if (state == FW_BL_IDLE)
        {
            // the bootloader has restarted (watchdog) and lost the transfer
            _blStatus = st;
            _gotReply = true; // evaluated in poll()
            return;
        }
        if (state != FW_BL_RECEIVING)
            return;
        if (next > _base && next <= _nchunks)
        {
            _base = next;
            _lastProgressMs = now();
            _noProgress = 0;
        }
        // gap: the module has dropped a later block -> resend from "next"
        // (for the same block only again once the retransmission should have arrived)
        uint32_t t = now();
        if (idx > next && _sendPtr > next && (_lastGoBack != next || t - _lastGoBackMs > _goBackHoldoffMs))
        {
            addRetrans(_sendPtr - next);
            _sendPtr = next;
            if (_flush)
                _flush(_ctx);
            _lastGoBack = next;
            _lastGoBackMs = t;
        }
    }
}

bool ModuleFwUpdateCore::selectNextModule()
{
    for (uint16_t m = 0; m < _numModules; m++)
    {
        if (_result[m] == FwModResult::Pending)
        {
            _current = (uint8_t)m;
            _moduleAttempts = 0;
            _modStartMs = now();
            _result[m] = FwModResult::InProgress;
            startModule();
            return true;
        }
    }
    return false;
}

void ModuleFwUpdateCore::startModule()
{
    log("Module %u: requesting bootloader (attempt %u)", (unsigned)_current, (unsigned)(_moduleAttempts + 1));
    setPhase(FwUpdPhase::EnterBoot);
}

void ModuleFwUpdateCore::moduleRetry(const char *why)
{
    _moduleAttempts++;
    if (_moduleAttempts >= FWUPD_MODULE_ATTEMPTS)
    {
        log("Module %u: %s - giving up", (unsigned)_current, why);
        moduleDone(false);
        return;
    }
    log("Module %u: %s - retrying", (unsigned)_current, why);
    startModule();
}

void ModuleFwUpdateCore::moduleDone(bool ok)
{
    _result[_current] = ok ? FwModResult::Updated : FwModResult::Failed;
    {
        uint32_t d = now() - _modStartMs;
        _modSeconds[_current] = (uint16_t)((d + 500) / 1000 > 0xFFFF ? 0xFFFF : (d + 500) / 1000);
        if (ok)
        {
            _doneMsSum += d;
            _doneCount++;
        }
    }
    if (ok)
    {
        _updatedCount++;
        log("Module %u: updated", (unsigned)_current);
    }
    else
    {
        _failedCount++;
    }

    if (!selectNextModule())
    {
        _finishedMs = now();
        _phase = FwUpdPhase::Done;
        log("Done: %u updated, %u failed, %lu retransmissions",

            (unsigned)_updatedCount, (unsigned)_failedCount, (unsigned long)_retransmissions);
    }
}

void ModuleFwUpdateCore::poll()
{
    if (!busy())
        return;

    const uint32_t t = now();
    const uint32_t sinceTx = t - _lastTxMs;
    const uint32_t inPhase = t - _phaseStartMs;

    switch (_phase)
    {
    case FwUpdPhase::Announce:
    {
        if (_gotReply)
        {
            _groupStart += 16;
            if (_groupStart < _numModules)
            {
                setPhase(FwUpdPhase::Announce);
                break;
            }
            // evaluation
            _todoCount = 0;
            uint8_t uptodate = 0, unknown = 0, noboot = 0;
            for (uint16_t m = 0; m < _numModules; m++)
            {
                switch (_result[m])
                {
                case FwModResult::Pending: _todoCount++; break;
                case FwModResult::UpToDate: uptodate++; break;
                case FwModResult::NoBootloader: noboot++; break;
                default: unknown++; break;
                }
            }
            log("Comparison: %u differ, %u up to date, %u without bootloader, %u no reply",
                (unsigned)_todoCount, (unsigned)uptodate, (unsigned)noboot, (unsigned)unknown);
            if (!selectNextModule())
            {
                _finishedMs = t;
                _phase = FwUpdPhase::Done;
            }
            break;
        }
        if (!_sent || sinceTx > _announceTimeoutMs)
        {
            if (_tries >= FWUPD_TRIES)
            {
                log("No reply to the version check (modules %u-%u) - check the module chain",
                    (unsigned)_groupStart, (unsigned)(_groupStart + 15));
                _finishedMs = t;
                _phase = FwUpdPhase::Aborted;
                break;
            }
            sendAnnounceGroup();
        }
        break;
    }

    case FwUpdPhase::EnterBoot:
    {
        if (_gotReply)
        {
            bool fromApp = _blProcessed && _appAck == FW_APP_ACK_ENTERING;
            bool fromBl = _blProcessed && (_blStatus & 0xFF00) == FW_BL_SIGNATURE;
            if (fromApp || fromBl)
            {
                setPhase(FwUpdPhase::WaitBoot);
            }
            else
            {
                moduleRetry("module refuses the bootloader");
            }
            break;
        }
        if (!_sent || sinceTx > _replyTimeoutMs)
        {
            if (_tries >= FWUPD_TRIES)
            {
                moduleRetry("no reply to the bootloader request");
                break;
            }
            sendControl(_current, _current, FW_ENTER_BOOT);
        }
        break;
    }

    case FwUpdPhase::WaitBoot:
    {
        if (_gotReply && _blProcessed && (_blStatus & 0xFF00) == FW_BL_SIGNATURE)
        {
            setPhase(FwUpdPhase::Start);
            break;
        }
        if (_gotReply && !_blProcessed && inPhase > 3000)
        {
            // the application answers again: bootloader watchdog expired -> request again
            moduleRetry("module returned to the application");
            break;
        }
        _gotReply = false;
        // the module needs a moment after the reset
        if (inPhase < 800)
            break;
        // While the old application is valid, the bootloader returns to it after 8 s without
        // a packet for this module -> poll at short intervals
        if (!_sent || sinceTx > FWUPD_KEEPALIVE_MS)
        {
            if (_tries >= 2 * FWUPD_TRIES)
            {
                moduleRetry("bootloader does not answer");
                break;
            }
            sendControl(_current, _current, FW_STATUS);
        }
        break;
    }

    case FwUpdPhase::Start:
    {
        if (_gotReply)
        {
            uint8_t state = _blStatus & FW_BL_STATE_MASK;
            if (_blProcessed && (_blStatus & 0xFF00) == FW_BL_SIGNATURE && state == FW_BL_RECEIVING && _blNext == 0)
            {
                _base = 0;
                _sendPtr = 0;
                _lastGoBack = 0xFFFF;
                _noProgress = 0;
                setPhase(FwUpdPhase::Data);
                _lastProgressMs = t;
                log("Module %u: transferring %u blocks", (unsigned)_current, (unsigned)_nchunks);
            }
            else if (state == FW_BL_ERR_LEN)
            {
                _moduleAttempts = FWUPD_MODULE_ATTEMPTS;
                moduleRetry("image too large for the bootloader");
            }
            else
            {
                _gotReply = false; // unexpected reply, wait for the timeout
            }
            break;
        }
        if (!_sent || sinceTx > FWUPD_KEEPALIVE_MS)
        {
            if (_tries >= 2 * FWUPD_TRIES)
            {
                moduleRetry("no reply to START");
                break;
            }
            sendControl(_current, _current, FW_START);
        }
        break;
    }

    case FwUpdPhase::Data:
    {
        if (_gotReply && (_blStatus & FW_BL_STATE_MASK) == FW_BL_IDLE)
        {
            // the bootloader lost the transfer -> start again
            moduleRetry("bootloader restarted");
            break;
        }
        _gotReply = false;

        if (_base >= _nchunks)
        {
            setPhase(FwUpdPhase::Finish);
            break;
        }

        while (_sendPtr < _nchunks && _sendPtr < _base + _window)
        {
            if (!sendChunk(_sendPtr))
                break;
            _sendPtr++;
        }

        uint32_t dataTimeout = _replyTimeoutMs + (uint32_t)_window * _intervalMs;
        if (t - _lastProgressMs > dataTimeout)
        {
            _noProgress++;
            if (_noProgress > FWUPD_TRIES)
            {
                moduleRetry("no acknowledges");
                break;
            }
            addRetrans(_sendPtr - _base);
            _sendPtr = _base;
            if (_flush)
                _flush(_ctx);
            _lastGoBack = _base;
            _lastGoBackMs = t;
            _lastProgressMs = t;
        }
        break;
    }

    case FwUpdPhase::Finish:
    {
        if (_gotReply)
        {
            uint8_t state = _blStatus & FW_BL_STATE_MASK;
            bool sig = (_blStatus & 0xFF00) == FW_BL_SIGNATURE;
            if (!_blProcessed)
            {
                // not answered by the bootloader -> module probably already runs the new application
                setPhase(FwUpdPhase::WaitApp);
            }
            else if (sig && state == FW_BL_DONE)
            {
                setPhase(FwUpdPhase::WaitApp);
            }
            else if (sig && state == FW_BL_ERR_CRC)
            {
                moduleRetry("CRC error in the module");
            }
            else if (sig && state == FW_BL_RECEIVING && _blNext < _nchunks)
            {
                _base = _sendPtr = _blNext;
                setPhase(FwUpdPhase::Data);
                _lastProgressMs = t;
            }
            else
            {
                _gotReply = false;
            }
            break;
        }
        if (!_sent || sinceTx > _replyTimeoutMs)
        {
            if (_tries >= FWUPD_TRIES)
            {
                // reply was lost - the verification decides
                setPhase(FwUpdPhase::WaitApp);
                break;
            }
            sendControl(_current, _current, FW_FINISH);
        }
        break;
    }

    case FwUpdPhase::WaitApp:
        // bootloader -> reset -> application (LED flashes 4x at start up)
        if (inPhase > 3500)
        {
            _verifyOk = _verifyAnswered = false;
            setPhase(FwUpdPhase::Verify);
        }
        break;

    case FwUpdPhase::Verify:
    {
        if (_gotReply)
        {
            if (_verifyOk)
            {
                moduleDone(true);
                break;
            }
            if (_verifyAnswered)
            {
                moduleRetry("verification: flash still differs");
                break;
            }
            // no reply from the application yet -> ask again after the timeout
            _gotReply = false;
        }
        if (!_sent || sinceTx > _replyTimeoutMs + 1000)
        {
            if (_tries >= FWUPD_TRIES)
            {
                moduleRetry("application does not start");
                break;
            }
            sendControl(_current, _current, FW_ANNOUNCE);
        }
        break;
    }

    default:
        break;
    }
}

// =====================================================================================
//  ESP32 glue (FreeRTOS task, queues, LittleFS)
// =====================================================================================
#if defined(ARDUINO)

#include <Arduino.h>
#include "esp32_ota.h"
#include <LittleFS.h>
#include <ArduinoJson.h>
#include "ModuleFirmwareUpdateTask.h"

ModuleFwUpdateCore moduleFwUpdate;

static QueueHandle_t fwupd_reply_q = nullptr;
static QueueHandle_t fwupd_request_q = nullptr;
static TaskHandle_t fwupd_task_handle = nullptr;
static uint8_t *fwupd_image = nullptr;
static SemaphoreHandle_t fwupd_mutex = nullptr;
static char fwupd_filename[48] = {0};
static uint32_t fwupd_ctrl_baud = 0;

void ModuleFwUpdate_SetControllerBaud(uint32_t baud) { fwupd_ctrl_baud = baud; }

// Separate transmit queue for update packets, the transmit task serves it first.
// It is deliberately short (FWUPD_TXQ_DEPTH): after a retransmission ("go back") no
// outdated blocks should be left in the queue.
#define FWUPD_TXQ_DEPTH 2
static QueueHandle_t fwupd_tx_q = nullptr;

static bool fwupd_send(void *, PacketStruct *p)
{
    if (fwupd_tx_q == nullptr)
        return false;
    // control packets (only one in flight at a time) go to the front
    if ((p->command & 0x0F) == FW_CMD_CONTROL)
        return xQueueSendToFront(fwupd_tx_q, p, 0) == pdPASS;
    return xQueueSendToBack(fwupd_tx_q, p, 0) == pdPASS;
}

bool ModuleFwUpdate_NextPacket(PacketStruct *p)
{
    return fwupd_tx_q != nullptr && xQueueReceive(fwupd_tx_q, p, 0) == pdPASS;
}
static uint32_t fwupd_now(void *) { return millis(); }
static void fwupd_log(void *, const char *msg) { ESP_LOGI(TAG, "%s", msg); }

// ---- firmware checksums of the modules (FW_CMD_REPORT) ----
static uint32_t fwcrc[FWUPD_MAX_MODULES];
static bool fwcrc_seen[FWUPD_MAX_MODULES];
static uint8_t fwcrc_total = 0;     // number of modules of the last query
static uint8_t fwcrc_next = 0;      // next module a packet still has to be sent for
static uint32_t fwcrc_time = 0;     // millis() of the last query

static void fwcrc_send_pending()
{
    while (fwcrc_next < fwcrc_total && uxQueueSpacesAvailable(fwupd_tx_q) > 0)
    {
        PacketStruct p;
        memset(&p, 0, sizeof(p));
        p.start_address = fwcrc_next;
        uint16_t last = fwcrc_next + FW_REPORT_MODULES_PER_PACKET - 1;
        if (last >= fwcrc_total)
            last = fwcrc_total - 1;
        p.end_address = (uint8_t)last;
        p.command = FW_CMD_REPORT;
        if (xQueueSendToBack(fwupd_tx_q, &p, 0) != pdPASS)
            break;
        fwcrc_next = (uint8_t)(last + 1);
    }
}

void ModuleFwUpdate_RequestCrcCheck(uint8_t numModules)
{
    if (numModules == 0 || numModules > FWUPD_MAX_MODULES)
        return;
    for (uint16_t i = 0; i < FWUPD_MAX_MODULES; i++)
    {
        fwcrc[i] = 0;
        fwcrc_seen[i] = false;
    }
    fwcrc_total = numModules;
    fwcrc_next = 0;
    fwcrc_time = millis();
    if (fwupd_task_handle)
        xTaskNotifyGive(fwupd_task_handle);
}

static void fwcrc_on_reply(const PacketStruct *p)
{
    if (!(p->command & 0x80))
        return; // no module answered (old firmware / bootloader)
    for (uint16_t m = p->start_address; m <= p->end_address && m < FWUPD_MAX_MODULES; m++)
    {
        uint8_t rel = m - p->start_address;
        if (rel >= FW_REPORT_MODULES_PER_PACKET)
            break;
        uint32_t v = (uint32_t)p->moduledata[2 * rel] | ((uint32_t)p->moduledata[2 * rel + 1] << 16);
        if (v != 0)
        {
            fwcrc[m] = v;
            fwcrc_seen[m] = true;
        }
    }
}

static void fwupd_task(void *)
{
    bool wasBusy = false;
    for (;;)
    {
        if (!moduleFwUpdate.busy())
        {
            if (wasBusy)
            {
                // update finished: read the checksums of all modules again
                wasBusy = false;
                vTaskDelay(pdMS_TO_TICKS(1000));
                ModuleFwUpdate_RequestCrcCheck(moduleFwUpdate.numModules());
            }
            fwcrc_send_pending();
            // sleep until an update is started or a query is requested
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(fwcrc_next < fwcrc_total ? 50 : 1000));
            continue;
        }
        wasBusy = true;

        PacketStruct ps;
        while (xQueueReceive(fwupd_reply_q, &ps, pdMS_TO_TICKS(20)) == pdPASS)
        {
            xSemaphoreTake(fwupd_mutex, portMAX_DELAY);
            moduleFwUpdate.onReply(&ps);
            xSemaphoreGive(fwupd_mutex);
        }

        xSemaphoreTake(fwupd_mutex, portMAX_DELAY);
        moduleFwUpdate.poll();
        bool stillBusy = moduleFwUpdate.busy();
        xSemaphoreGive(fwupd_mutex);

        if (!stillBusy && fwupd_image != nullptr)
        {
            // free the image only once nothing is sent any more

            vTaskDelay(pdMS_TO_TICKS(5000));
            xSemaphoreTake(fwupd_mutex, portMAX_DELAY);
            if (!moduleFwUpdate.busy())
            {
                free(fwupd_image);
                fwupd_image = nullptr;
            }
            xSemaphoreGive(fwupd_mutex);
        }
    }
}

void ModuleFwUpdate_Init(QueueHandle_t request_q)
{
    fwupd_request_q = request_q;
    fwupd_reply_q = xQueueCreate(16, sizeof(PacketStruct));
    fwupd_tx_q = xQueueCreate(FWUPD_TXQ_DEPTH, sizeof(PacketStruct));
    fwupd_mutex = xSemaphoreCreateMutex();
    moduleFwUpdate.setHooks(&fwupd_send, &fwupd_now, &fwupd_log, nullptr);
    moduleFwUpdate.setFlushHook([](void *) { xQueueReset(fwupd_tx_q); });
    xTaskCreate(fwupd_task, "modfwupd", 4096, nullptr, configMAX_PRIORITIES - 4, &fwupd_task_handle);
}

bool ModuleFwUpdate_IsFirmwarePacket(const PacketStruct *p)
{
    uint8_t cmd = p->command & 0x0F;
    return cmd == FW_CMD_CONTROL || cmd == FW_CMD_DATA || cmd == FW_CMD_REPORT;
}

void ModuleFwUpdate_OnReply(const PacketStruct *p)
{
    if ((p->command & 0x0F) == FW_CMD_REPORT)
    {
        fwcrc_on_reply(p);
        return;
    }
    if (fwupd_reply_q != nullptr && moduleFwUpdate.busy())
    {
        xQueueSendToBack(fwupd_reply_q, p, 0);
    }
}

bool ModuleFwUpdate_Busy() { return fwupd_mutex != nullptr && moduleFwUpdate.busy(); }

// Refuse a controller update (esp32_ota.cpp) while a module update is running
const char *ESP32Ota_BlockedReason()
{
    return ModuleFwUpdate_Busy() ? "A module firmware update is running - update the controller afterwards" : nullptr;
}

int16_t ModuleFwUpdate_ModuleInBootloader() { return moduleFwUpdate.moduleInBootloader(); }

uint32_t ModuleFwUpdate_PacketIntervalMs() { return moduleFwUpdate.packetIntervalMs(); }

bool ModuleFwUpdate_Start(uint16_t manifestIndex, uint8_t numModules, uint32_t baudRate, char *message, size_t messageSize)
{
    if (fwupd_mutex == nullptr)
    {
        snprintf(message, messageSize, "not initialised");
        return false;
    }
    if (moduleFwUpdate.busy())
    {
        snprintf(message, messageSize, "Update is already running");
        return false;
    }

    File mf = LittleFS.open("/avr/manifest.json");
    if (!mf)
    {
        snprintf(message, messageSize, "manifest.json not found");
        return false;
    }
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, mf);
    mf.close();
    if (err != DeserializationError::Ok)
    {
        snprintf(message, messageSize, "manifest.json is invalid");
        return false;
    }

    JsonArray list = doc["avrprog"];
    if (manifestIndex >= list.size())
    {
        snprintf(message, messageSize, "Entry %u missing", manifestIndex);
        return false;
    }
    JsonObject e = list[manifestIndex];
    if ((e["chain"] | 0) != 1)
    {
        snprintf(message, messageSize, "%s is not an image for the chain update (select a bootloader variant)",
                 e["board"].as<const char *>());
        return false;
    }
    uint32_t imgBaud = e["baud"] | 0;
    if (imgBaud != baudRate)
    {
        snprintf(message, messageSize, "Baud rate mismatch: image %lu, controller %lu",
                 (unsigned long)imgBaud, (unsigned long)baudRate);
        return false;
    }
    uint16_t boardVer = e["boardver"] | 0;
    uint32_t git = strtoul(e["ver"] | "0", nullptr, 16);
    uint32_t expectedCrc = strtoul(e["crc32"] | "0", nullptr, 16);

    snprintf(fwupd_filename, sizeof(fwupd_filename), "/avr/%s", e["name"].as<const char *>());
    File f = LittleFS.open(fwupd_filename);
    if (!f)
    {
        snprintf(message, messageSize, "File %s missing", fwupd_filename);
        return false;
    }
    size_t len = f.size();
    if (len == 0 || len > FW_APP_MAX)
    {
        f.close();
        snprintf(message, messageSize, "Image has %u bytes (max. %u)", (unsigned)len, (unsigned)FW_APP_MAX);
        return false;
    }

    xSemaphoreTake(fwupd_mutex, portMAX_DELAY);
    if (fwupd_image != nullptr)
    {
        free(fwupd_image);
        fwupd_image = nullptr;
    }
    fwupd_image = (uint8_t *)malloc(len);
    bool ok = fwupd_image != nullptr && f.read(fwupd_image, len) == len;
    f.close();

    if (ok && expectedCrc != 0 && ModuleFwUpdateCore::crc32(fwupd_image, len) != expectedCrc)
    {
        snprintf(message, messageSize, "CRC32 of the file does not match manifest.json");
        ok = false;
    }
    else if (!ok)
    {
        snprintf(message, messageSize, "File could not be read");
    }

    if (ok)
    {
        // discard replies from an earlier run
        xQueueReset(fwupd_reply_q);
        ok = moduleFwUpdate.begin(fwupd_image, (uint16_t)len, boardVer, git, numModules, baudRate);
        snprintf(message, messageSize, "%s", moduleFwUpdate.message());
    }
    if (!ok && fwupd_image != nullptr)
    {
        free(fwupd_image);
        fwupd_image = nullptr;
    }
    xSemaphoreGive(fwupd_mutex);

    if (ok)
    {
        xTaskNotifyGive(fwupd_task_handle);
    }
    return ok;
}

void ModuleFwUpdate_Abort()
{
    if (fwupd_mutex == nullptr)
        return;
    xSemaphoreTake(fwupd_mutex, portMAX_DELAY);
    moduleFwUpdate.abort();
    xSemaphoreGive(fwupd_mutex);
}

int ModuleFwUpdate_StatusJson(char *buf, size_t size)
{
    if (fwupd_mutex == nullptr)
        return snprintf(buf, size, "{\"busy\":0,\"phase\":\"-\",\"modules\":[]}");
    JsonDocument doc;
    xSemaphoreTake(fwupd_mutex, portMAX_DELAY);
    doc["busy"] = moduleFwUpdate.busy() ? 1 : 0;
    doc["phase"] = ModuleFwUpdateCore::phaseName(moduleFwUpdate.phase());
    doc["module"] = moduleFwUpdate.currentModule();
    doc["chunks"] = moduleFwUpdate.chunksTotal();
    doc["acked"] = moduleFwUpdate.chunksAcked();
    doc["todo"] = moduleFwUpdate.modulesTodo();
    doc["updated"] = moduleFwUpdate.modulesUpdated();
    doc["failed"] = moduleFwUpdate.modulesFailed();
    doc["retrans"] = moduleFwUpdate.retransmissions();
    doc["elapsed"] = moduleFwUpdate.elapsedMs();
    doc["len"] = moduleFwUpdate.imageLength();
    char crc[12];
    snprintf(crc, sizeof(crc), "%08lx", (unsigned long)moduleFwUpdate.imageCrc());
    doc["crc"] = crc;
    doc["file"] = fwupd_filename;
    doc["msg"] = moduleFwUpdate.message();
    doc["ctrlbaud"] = fwupd_ctrl_baud;
    doc["eta"] = moduleFwUpdate.remainingSeconds();
    // -1 = not checked since start up (int32: the expression must not become unsigned)
    doc["fwcrcage"] = (fwcrc_time != 0 && fwcrc_total > 0) ? (int32_t)((millis() - fwcrc_time) / 1000UL) : (int32_t)-1;
    doc["fwcrcbusy"] = fwcrc_next < fwcrc_total ? 1 : 0;
    JsonArray fc = doc["fwcrc"].to<JsonArray>();
    for (uint8_t m = 0; m < fwcrc_total; m++)
    {
        char h[10] = "";
        if (fwcrc_seen[m])
            snprintf(h, sizeof(h), "%08lx", (unsigned long)fwcrc[m]);
        fc.add(h);
    }
    JsonArray mods = doc["modules"].to<JsonArray>();
    for (uint8_t m = 0; m < moduleFwUpdate.numModules(); m++)
    {
        // 0=no reply 1=no bootloader 2=up to date 3=waiting 4=running 5=updated 6=failed
        mods.add((uint8_t)moduleFwUpdate.result(m));
    }
    JsonArray secs = doc["secs"].to<JsonArray>();
    JsonArray rtx = doc["rtx"].to<JsonArray>();
    for (uint8_t m = 0; m < moduleFwUpdate.numModules(); m++)
    {
        secs.add(moduleFwUpdate.moduleSeconds(m));
        rtx.add(moduleFwUpdate.moduleRetransmissions(m));
    }
    xSemaphoreGive(fwupd_mutex);
    return serializeJson(doc, buf, size);
}


// =====================================================================================
//  Upload (web interface) and remove firmware files
// =====================================================================================
#include <esp_http_server.h>

extern char CookieValue[20 + 1];
extern bool validateXSS(httpd_req_t *req);
extern void url_decode(char *encoded, char *decoded);

// Read and discard the rest of the request body so the browser receives the reply
// (otherwise the connection is reset and the browser only reports "failed")
static void modfw_drain(httpd_req_t *req, size_t already)
{
    char tmp[256];
    size_t left = req->content_len > already ? req->content_len - already : 0;
    int timeouts = 0;
    while (left > 0)
    {
        int r = httpd_req_recv(req, tmp, left < sizeof(tmp) ? left : sizeof(tmp));
        if (r == HTTPD_SOCK_ERR_TIMEOUT && ++timeouts < 5)
            continue;
        if (r <= 0)
            break;
        left -= r;
    }
}

static const char *MANIFEST = "/avr/manifest.json";

static esp_err_t modfw_reply(httpd_req_t *req, bool ok, const char *msg, size_t bodyRead = 0)
{
    modfw_drain(req, bodyRead);
    char buf[200];
    JsonDocument d;
    d["ok"] = ok ? 1 : 0;
    d["message"] = msg;
    size_t n = serializeJson(d, buf, sizeof(buf));
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, buf, n);
}

static bool load_manifest(JsonDocument &doc)
{
    if (LittleFS.exists(MANIFEST))
    {
        File f = LittleFS.open(MANIFEST);
        if (f)
        {
            DeserializationError e = deserializeJson(doc, f);
            f.close();
            if (e == DeserializationError::Ok)
            {
                if (!doc["avrprog"].is<JsonArray>())
                    doc["avrprog"].to<JsonArray>();
                return true;
            }
            return false;
        }
    }
    doc["avrprog"].to<JsonArray>();
    return true;
}

static bool save_manifest(JsonDocument &doc)
{
    if (!LittleFS.exists("/avr"))
        LittleFS.mkdir("/avr");
    File f = LittleFS.open(MANIFEST, "w");
    if (!f)
        return false;
    size_t n = serializeJsonPretty(doc, f);
    f.close();
    return n > 0;
}

// POST /upload/modfw?xss=..&name=..&baud=..   body: application image (binary, from 0x0600)
esp_err_t ModuleFwUpdate_UploadHandler(httpd_req_t *req)
{
    // check the cookie (like validateXSS, but with a JSON reply)
    char cookie[sizeof(CookieValue) + 2] = {0};
    size_t cookielen = sizeof(cookie);
    if (httpd_req_get_cookie_val(req, "DIYBMS", cookie, &cookielen) != ESP_OK ||
        strncmp(cookie, CookieValue, sizeof(CookieValue)) != 0)
        return modfw_reply(req, false, "Web page is out of date (controller has restarted) - refresh the page (F5) and select the file again");

    char query[200] = {0};
    char enc[3 * sizeof(CookieValue) + 4] = {0};
    char xss[3 * sizeof(CookieValue) + 4] = {0};
    char nameenc[100] = {0};
    char name[100] = {0};
    char baudstr[12] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "xss", enc, sizeof(enc)) != ESP_OK)
        return modfw_reply(req, false, "Incomplete request");
    // the key can contain $ # @ -> URL encoded
    url_decode(enc, xss);
    if (strncmp(xss, CookieValue, sizeof(CookieValue)) != 0)
        return modfw_reply(req, false, "Web page is out of date (controller has restarted) - refresh the page (F5) and select the file again");
    if (httpd_query_key_value(query, "name", nameenc, sizeof(nameenc)) == ESP_OK)
        url_decode(nameenc, name);
    httpd_query_key_value(query, "baud", baudstr, sizeof(baudstr));
    uint32_t baud = strtoul(baudstr, nullptr, 10);
    if (baud == 0)
        baud = fwupd_ctrl_baud;

    if (moduleFwUpdate.busy())
        return modfw_reply(req, false, "An update is running - please wait");

    size_t len = req->content_len;
    if (len < 64 || len > FW_APP_MAX)
    {
        char m[96];
        snprintf(m, sizeof(m), "Invalid file size %u bytes (max. %u)", (unsigned)len, (unsigned)FW_APP_MAX);
        return modfw_reply(req, false, m);
    }

    uint8_t *img = (uint8_t *)malloc(len);
    if (img == nullptr)
        return modfw_reply(req, false, "Out of memory");

    size_t got = 0;
    int timeouts = 0;
    while (got < len)
    {
        int r = httpd_req_recv(req, (char *)img + got, len - got);
        if (r == HTTPD_SOCK_ERR_TIMEOUT && ++timeouts < 5)
            continue;
        if (r <= 0)
        {
            free(img);
            return modfw_reply(req, false, "Transfer aborted", req->content_len);
        }
        got += r;
    }

    // plausibility: the application vector table starts with "jmp" (0x940C, little endian 0C 94)
    if (img[0] != 0x0C || img[1] != 0x94)
    {
        free(img);
        return modfw_reply(req, false, "Not a module firmware linked at 0x0600 (first instruction is not JMP). "
                                       "Use the .bin or .hex file of a V450_BL_* variant.", len);
    }

    uint32_t crc = ModuleFwUpdateCore::crc32(img, len);
    char crcs[12];
    snprintf(crcs, sizeof(crcs), "%08lx", (unsigned long)crc);
    char fname[40];
    snprintf(fname, sizeof(fname), "fw_up_%s.bin", crcs);
    char path[48];
    snprintf(path, sizeof(path), "/avr/%s", fname);

    // sanitise the display name
    char label[28];
    size_t j = 0;
    for (size_t i = 0; name[i] && j < sizeof(label) - 1; i++)
    {
        char c = name[i];
        if (c == '+')
            c = ' ';
        if (isalnum((unsigned char)c) || c == '_' || c == '-' || c == '.' || c == ' ')
            label[j++] = c;
    }
    label[j] = 0;
    if (j == 0)
        snprintf(label, sizeof(label), "Upload %s", crcs);

    if (!LittleFS.exists("/avr"))
        LittleFS.mkdir("/avr");
    File f = LittleFS.open(path, "w");
    bool ok = f && f.write(img, len) == len;
    if (f)
        f.close();
    free(img);
    if (!ok)
    {
        LittleFS.remove(path);
        return modfw_reply(req, false, "File could not be saved (file system full?)", len);
    }

    JsonDocument doc;
    if (!load_manifest(doc))
        return modfw_reply(req, false, "manifest.json is invalid", len);
    JsonArray list = doc["avrprog"];
    for (size_t i = 0; i < list.size(); i++)
    {
        if (strcmp(list[i]["name"] | "", fname) == 0)
        {
            list.remove(i);
            break;
        }
    }
    JsonObject e = list.add<JsonObject>();
    e["board"] = label;
    e["name"] = fname;
    e["ver"] = crcs;
    e["mcu"] = "1e942a";
    e["efuse"] = 0;
    e["hfuse"] = 0;
    e["lfuse"] = 0;
    e["chain"] = 1;
    e["boardver"] = 450;
    e["baud"] = baud;
    e["len"] = (uint32_t)len;
    e["crc32"] = crcs;
    e["upload"] = 1;
    if (doc.overflowed() || !save_manifest(doc))
        return modfw_reply(req, false, "manifest.json could not be written", len);

    char m[120];
    snprintf(m, sizeof(m), "Saved: %s, %u bytes, CRC32 %s, %lu baud", label, (unsigned)len, crcs, (unsigned long)baud);
    ESP_LOGI(TAG, "%s", m);
    return modfw_reply(req, true, m, len);
}

bool ModuleFwUpdate_DeleteEntry(uint16_t manifestIndex, char *message, size_t messageSize)
{
    if (moduleFwUpdate.busy())
    {
        snprintf(message, messageSize, "An update is running");
        return false;
    }
    JsonDocument doc;
    if (!load_manifest(doc))
    {
        snprintf(message, messageSize, "manifest.json is invalid");
        return false;
    }
    JsonArray list = doc["avrprog"];
    if (manifestIndex >= list.size())
    {
        snprintf(message, messageSize, "Entry missing");
        return false;
    }
    char path[48];
    snprintf(path, sizeof(path), "/avr/%s", list[manifestIndex]["name"] | "");
    list.remove(manifestIndex);
    if (!save_manifest(doc))
    {
        snprintf(message, messageSize, "manifest.json could not be written");
        return false;
    }
    // only delete the file if no other entry still uses it
    bool used = false;
    for (JsonObject o : list)
        if (strcmp(path + 5, o["name"] | "") == 0)
            used = true;
    if (!used)
        LittleFS.remove(path);
    snprintf(message, messageSize, "Removed");

    return true;
}

#endif
