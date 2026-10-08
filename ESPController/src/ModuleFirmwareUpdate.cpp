/*
 * DIYBMS ESP32 - Firmware-Update der Zellmodule ueber die RX/TX-Kette
 * (Beschreibung siehe include/ModuleFirmwareUpdate.h und ../common/fwupdate_protocol.h)
 */

#if defined(ARDUINO)
#define USE_ESP_IDF_LOG 1
static constexpr const char *const TAG = "diybms-modfw";
#endif

#include "ModuleFirmwareUpdate.h"
#include <stdarg.h>
#include <stdio.h>

// =====================================================================================
//  Ablaufsteuerung (plattformunabhaengig)
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
    case FwUpdPhase::Idle: return "Bereit";
    case FwUpdPhase::Announce: return "Versionsvergleich";
    case FwUpdPhase::EnterBoot: return "Bootloader anfordern";
    case FwUpdPhase::WaitBoot: return "Warte auf Bootloader";
    case FwUpdPhase::Start: return "Start";
    case FwUpdPhase::Data: return "Uebertragung";
    case FwUpdPhase::Finish: return "Pruefen/Abschliessen";
    case FwUpdPhase::WaitApp: return "Neustart Modul";
    case FwUpdPhase::Verify: return "Kontrolle";
    case FwUpdPhase::Done: return "Fertig";
    case FwUpdPhase::Aborted: return "Abgebrochen";
    }
    return "?";
}

const char *ModuleFwUpdateCore::resultName(FwModResult r)
{
    switch (r)
    {
    case FwModResult::Unknown: return "keine Antwort";
    case FwModResult::NoBootloader: return "ohne Bootloader";
    case FwModResult::UpToDate: return "aktuell";
    case FwModResult::Pending: return "wartet";
    case FwModResult::InProgress: return "laeuft";
    case FwModResult::Updated: return "aktualisiert";
    case FwModResult::Failed: return "Fehler";
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
        // nach der ersten Anfrage kann das Modul bereits im Bootloader sein
        return _sent ? _current : -1;
    case FwUpdPhase::Aborted:
        // Abbruch mitten in der Uebertragung: Modul bleibt im Bootloader
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
        log("Update laeuft bereits");
        return false;
    }
    if (image == nullptr || len == 0 || len > FW_APP_MAX)
    {
        log("Image ungueltig (%u Byte, max. %u)", (unsigned)len, (unsigned)FW_APP_MAX);
        return false;
    }
    if (numModules == 0 || numModules > FWUPD_MAX_MODULES || baudRate == 0)
    {
        log("Modulanzahl/Baudrate ungueltig");
        return false;
    }

    _image = image;
    _len = len;
    _nchunks = (len + FW_CHUNK_SIZE - 1) / FW_CHUNK_SIZE;
    _crc = crc32(image, len);
    _board = boardVersion;
    _git = gitVersion;
    _numModules = numModules;

    // Zeitverhalten aus Baudrate und Kettenlaenge
    // ein Paket = Startbyte + 40 Byte + wenige Escape-Bytes, 10 Bit je Byte
    uint32_t packetMs = (44UL * 10UL * 1000UL + baudRate - 1) / baudRate;
    _intervalMs = (packetMs * FW_PACING_FACTOR_X10) / 10 + 10;
    uint32_t ringMs = (uint32_t)(numModules + 1) * (packetMs + 20) + 300;
    uint32_t w = ringMs / _intervalMs + 3;
    _window = (uint16_t)(w < 4 ? 4 : (w > 16 ? 16 : w));
    // normale Abfragen (700..900 ms Abstand) koennen in der Sendewarteschlange davor liegen
    _replyTimeoutMs = 2 * ringMs + 3000;
    // ein wiederholter Block braucht Warteschlange + Ring, bis seine Quittung da sein kann
    _goBackHoldoffMs = ringMs + (uint32_t)_window * _intervalMs + 500;
    _announceTimeoutMs = _replyTimeoutMs + (uint32_t)numModules * 250;

    for (uint16_t i = 0; i < FWUPD_MAX_MODULES; i++)
        _result[i] = FwModResult::Unknown;

    _todoCount = _updatedCount = _failedCount = 0;
    _retransmissions = 0;
    _groupStart = 0;
    _maskNeeds = _maskBad = _maskBoot = 0;
    _base = _sendPtr = 0;
    _startedMs = now();
    if (_startedMs == 0)
        _startedMs = 1;
    _finishedMs = 0;

    log("Start: %u Byte, CRC32 %08lx, %u Module, Paketabstand %lu ms, Fenster %u",
        (unsigned)len, (unsigned long)_crc, (unsigned)numModules, (unsigned long)_intervalMs, (unsigned)_window);
    setPhase(FwUpdPhase::Announce);
    return true;
}

void ModuleFwUpdateCore::abort()
{
    if (!busy())
        return;
    // Steht das Modul im Bootloader und ist die alte App noch gueltig, zurueckschicken.
    // Mitten in der Uebertragung (App schon geloescht) lehnt der Bootloader FW_EXIT ab
    // und wartet auf einen neuen Versuch.
    if (moduleInBootloader() >= 0)
    {
        sendControl(_current, _current, FW_EXIT);
    }
    log("Abgebrochen bei Modul %u (%s)", (unsigned)_current, phaseName(_phase));
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
    p.sequence = idx; // Blocknummer (der Sende-Task ueberschreibt sie bei FW_CMD_DATA nicht)
    uint8_t buf[FW_CHUNK_SIZE];
    memset(buf, 0xFF, sizeof(buf));
    uint32_t off = (uint32_t)idx * FW_CHUNK_SIZE;
    uint32_t n = (_len - off) < FW_CHUNK_SIZE ? (_len - off) : FW_CHUNK_SIZE;
    memcpy(buf, _image + off, n);
    memcpy(p.moduledata, buf, FW_CHUNK_SIZE); // Little Endian auf ESP32 und AVR
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

    // ab hier nur Antworten des aktuellen Moduls
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
            // Bootloader hat neu gestartet (Watchdog) und den Empfang verloren
            _blStatus = st;
            _gotReply = true; // wird in poll() ausgewertet
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
        // Luecke: das Modul hat einen spaeteren Block verworfen -> ab "next" erneut senden
        // (fuer denselben Block erst wieder, wenn die Wiederholung angekommen sein muesste)
        uint32_t t = now();
        if (idx > next && _sendPtr > next && (_lastGoBack != next || t - _lastGoBackMs > _goBackHoldoffMs))
        {
            _retransmissions += _sendPtr - next;
            _sendPtr = next;
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
            _result[m] = FwModResult::InProgress;
            startModule();
            return true;
        }
    }
    return false;
}

void ModuleFwUpdateCore::startModule()
{
    log("Modul %u: Bootloader anfordern (Versuch %u)", (unsigned)_current, (unsigned)(_moduleAttempts + 1));
    setPhase(FwUpdPhase::EnterBoot);
}

void ModuleFwUpdateCore::moduleRetry(const char *why)
{
    _moduleAttempts++;
    if (_moduleAttempts >= FWUPD_MODULE_ATTEMPTS)
    {
        log("Modul %u: %s - aufgegeben", (unsigned)_current, why);
        moduleDone(false);
        return;
    }
    log("Modul %u: %s - neuer Versuch", (unsigned)_current, why);
    startModule();
}

void ModuleFwUpdateCore::moduleDone(bool ok)
{
    _result[_current] = ok ? FwModResult::Updated : FwModResult::Failed;
    if (ok)
    {
        _updatedCount++;
        log("Modul %u: aktualisiert", (unsigned)_current);
    }
    else
    {
        _failedCount++;
    }

    if (!selectNextModule())
    {
        _finishedMs = now();
        _phase = FwUpdPhase::Done;
        log("Fertig: %u aktualisiert, %u Fehler, %lu Wiederholungen",
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
            // Auswertung
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
            log("Vergleich: %u abweichend, %u aktuell, %u ohne Bootloader, %u ohne Antwort",
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
                log("Keine Antwort auf den Versionsvergleich (Module %u-%u) - Kette pruefen",
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
                moduleRetry("Modul lehnt Bootloader ab");
            }
            break;
        }
        if (!_sent || sinceTx > _replyTimeoutMs)
        {
            if (_tries >= FWUPD_TRIES)
            {
                moduleRetry("keine Antwort auf Bootloader-Anforderung");
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
            // die Anwendung antwortet wieder: Bootloader-Watchdog abgelaufen -> neu anfordern
            moduleRetry("Modul ist zur Anwendung zurueckgekehrt");
            break;
        }
        _gotReply = false;
        // Modul braucht nach dem Reset einen Moment
        if (inPhase < 800)
            break;
        // Solange die alte App gueltig ist, kehrt der Bootloader nach 8 s ohne Paket an ihn
        // zur App zurueck -> in kurzen Abstaenden nachfragen
        if (!_sent || sinceTx > FWUPD_KEEPALIVE_MS)
        {
            if (_tries >= 2 * FWUPD_TRIES)
            {
                moduleRetry("Bootloader meldet sich nicht");
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
                log("Modul %u: Uebertragung %u Bloecke", (unsigned)_current, (unsigned)_nchunks);
            }
            else if (state == FW_BL_ERR_LEN)
            {
                _moduleAttempts = FWUPD_MODULE_ATTEMPTS;
                moduleRetry("Image zu gross fuer den Bootloader");
            }
            else
            {
                _gotReply = false; // unpassende Antwort, Zeitueberschreitung abwarten
            }
            break;
        }
        if (!_sent || sinceTx > FWUPD_KEEPALIVE_MS)
        {
            if (_tries >= 2 * FWUPD_TRIES)
            {
                moduleRetry("keine Antwort auf START");
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
            // Bootloader hat den Empfang verloren -> neu beginnen
            moduleRetry("Bootloader neu gestartet");
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
                moduleRetry("keine Quittungen");
                break;
            }
            _retransmissions += _sendPtr - _base;
            _sendPtr = _base;
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
                // nicht vom Bootloader beantwortet -> Modul laeuft vermutlich schon mit neuer App
                setPhase(FwUpdPhase::WaitApp);
            }
            else if (sig && state == FW_BL_DONE)
            {
                setPhase(FwUpdPhase::WaitApp);
            }
            else if (sig && state == FW_BL_ERR_CRC)
            {
                moduleRetry("CRC-Fehler im Modul");
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
                // Antwort ging verloren - Kontrolle entscheidet
                setPhase(FwUpdPhase::WaitApp);
                break;
            }
            sendControl(_current, _current, FW_FINISH);
        }
        break;
    }

    case FwUpdPhase::WaitApp:
        // Bootloader -> Reset -> Anwendung (4x LED-Blinken beim Start)
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
                moduleRetry("Kontrolle: Flash weicht noch ab");
                break;
            }
            // noch keine Antwort der Anwendung -> nach Zeitablauf erneut fragen
            _gotReply = false;
        }
        if (!_sent || sinceTx > _replyTimeoutMs + 1000)
        {
            if (_tries >= FWUPD_TRIES)
            {
                moduleRetry("Anwendung startet nicht");
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
//  ESP32-Anbindung (FreeRTOS-Task, Warteschlangen, LittleFS)
// =====================================================================================
#if defined(ARDUINO)

#include <Arduino.h>
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

static bool fwupd_send(void *, PacketStruct *p)
{
    return fwupd_request_q != nullptr && xQueueSendToBack(fwupd_request_q, p, 0) == pdPASS;
}
static uint32_t fwupd_now(void *) { return millis(); }
static void fwupd_log(void *, const char *msg) { ESP_LOGI(TAG, "%s", msg); }

static void fwupd_task(void *)
{
    for (;;)
    {
        if (!moduleFwUpdate.busy())
        {
            // schlafen bis ein Update gestartet wird
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
            continue;
        }

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
            // Image erst freigeben, wenn nichts mehr gesendet wird
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
    fwupd_mutex = xSemaphoreCreateMutex();
    moduleFwUpdate.setHooks(&fwupd_send, &fwupd_now, &fwupd_log, nullptr);
    xTaskCreate(fwupd_task, "modfwupd", 4096, nullptr, configMAX_PRIORITIES - 4, &fwupd_task_handle);
}

bool ModuleFwUpdate_IsFirmwarePacket(const PacketStruct *p)
{
    uint8_t cmd = p->command & 0x0F;
    return cmd == FW_CMD_CONTROL || cmd == FW_CMD_DATA;
}

void ModuleFwUpdate_OnReply(const PacketStruct *p)
{
    if (fwupd_reply_q != nullptr && moduleFwUpdate.busy())
    {
        xQueueSendToBack(fwupd_reply_q, p, 0);
    }
}

bool ModuleFwUpdate_Busy() { return fwupd_mutex != nullptr && moduleFwUpdate.busy(); }

int16_t ModuleFwUpdate_ModuleInBootloader() { return moduleFwUpdate.moduleInBootloader(); }

uint32_t ModuleFwUpdate_PacketIntervalMs() { return moduleFwUpdate.packetIntervalMs(); }

bool ModuleFwUpdate_Start(uint16_t manifestIndex, uint8_t numModules, uint32_t baudRate, char *message, size_t messageSize)
{
    if (fwupd_mutex == nullptr)
    {
        snprintf(message, messageSize, "nicht initialisiert");
        return false;
    }
    if (moduleFwUpdate.busy())
    {
        snprintf(message, messageSize, "Update laeuft bereits");
        return false;
    }

    File mf = LittleFS.open("/avr/manifest.json");
    if (!mf)
    {
        snprintf(message, messageSize, "manifest.json nicht gefunden");
        return false;
    }
    DynamicJsonDocument doc(6000);
    DeserializationError err = deserializeJson(doc, mf);
    mf.close();
    if (err != DeserializationError::Ok)
    {
        snprintf(message, messageSize, "manifest.json fehlerhaft");
        return false;
    }

    JsonArray list = doc["avrprog"];
    if (manifestIndex >= list.size())
    {
        snprintf(message, messageSize, "Eintrag %u fehlt", manifestIndex);
        return false;
    }
    JsonObject e = list[manifestIndex];
    if ((e["chain"] | 0) != 1)
    {
        snprintf(message, messageSize, "%s ist kein Image fuer das Ketten-Update (Bootloader-Variante waehlen)",
                 e["board"].as<const char *>());
        return false;
    }
    uint32_t imgBaud = e["baud"] | 0;
    if (imgBaud != baudRate)
    {
        snprintf(message, messageSize, "Baudrate passt nicht: Image %lu, Controller %lu",
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
        snprintf(message, messageSize, "Datei %s fehlt", fwupd_filename);
        return false;
    }
    size_t len = f.size();
    if (len == 0 || len > FW_APP_MAX)
    {
        f.close();
        snprintf(message, messageSize, "Image hat %u Byte (max. %u)", (unsigned)len, (unsigned)FW_APP_MAX);
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
        snprintf(message, messageSize, "CRC32 der Datei stimmt nicht mit manifest.json ueberein");
        ok = false;
    }
    else if (!ok)
    {
        snprintf(message, messageSize, "Datei konnte nicht gelesen werden");
    }

    if (ok)
    {
        // Antworten aus einem frueheren Lauf verwerfen
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
    DynamicJsonDocument doc(3072);
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
    JsonArray mods = doc.createNestedArray("modules");
    for (uint8_t m = 0; m < moduleFwUpdate.numModules(); m++)
    {
        // 0=keine Antwort 1=ohne Bootloader 2=aktuell 3=wartet 4=laeuft 5=aktualisiert 6=Fehler
        mods.add((uint8_t)moduleFwUpdate.result(m));
    }
    xSemaphoreGive(fwupd_mutex);
    return serializeJson(doc, buf, size);
}

#endif
