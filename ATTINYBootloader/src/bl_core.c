/*
 * DIYBMS ATtiny-Bootloader - Paketverarbeitung und Flash-Logik
 *
 * Sicherheitsprinzip:
 *  - FW_START loescht zuerst die ERSTE Seite der Anwendung. Ab dann ist die App
 *    ungueltig (erstes Wort = 0xFFFF) und der Bootloader bleibt nach jedem Reset
 *    oder Stromausfall aktiv, bis ein vollstaendiges Image angekommen ist.
 *  - Die erste Seite (Blocks 0 und 1, Reset-Vektor) wird nur im RAM gehalten und
 *    erst geschrieben, wenn die CRC32 ueber das ganze Image stimmt.
 *  - Bloecke werden nur in Reihenfolge angenommen; die Antwort meldet immer den
 *    naechsten erwarteten Block (kumulative Quittung, der ESP32 sendet ab dort neu).
 */
#include <string.h>
#include "bl_core.h"

static inline uint16_t get16(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
static inline void put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}
#define MDP(p, i) ((p) + BL_MD + 2 * (i))

void bl_init(bl_state_t *s)
{
    memset(s, 0, sizeof(bl_state_t));
    s->state = FW_BL_IDLE;
}

uint8_t bl_app_valid(void)
{
    return !(bl_hal_flash_read(FW_APP_START) == 0xFF && bl_hal_flash_read(FW_APP_START + 1) == 0xFF);
}

static uint32_t image_crc(const bl_state_t *s, uint8_t use_ram_page0)
{
    uint32_t c = 0xFFFFFFFFUL;
    for (uint16_t i = 0; i < s->len; i++)
    {
        uint8_t b = (use_ram_page0 && i < FW_PAGE_SIZE) ? s->page0[i] : bl_hal_flash_read(FW_APP_START + i);
        c = fw_crc32_update(c, b);
    }
    return ~c;
}

static void do_start(bl_state_t *s, const uint8_t *p)
{
    uint16_t len = get16(MDP(p, FW_MD_LEN));
    if (len == 0 || len > FW_APP_MAX)
    {
        s->state = FW_BL_ERR_LEN;
        return;
    }
    s->len = len;
    s->crc = (uint32_t)get16(MDP(p, FW_MD_CRC_LO)) | ((uint32_t)get16(MDP(p, FW_MD_CRC_HI)) << 16);
    s->next = 0;
    memset(s->page0, 0xFF, FW_PAGE_SIZE);
    bl_hal_flash_bufclr();
    /* Anwendung ungueltig machen: ab jetzt bleibt der Bootloader aktiv */
    bl_hal_flash_erase(FW_APP_START);
    s->state = FW_BL_RECEIVING;
}

static void do_data(bl_state_t *s, uint16_t idx, const uint8_t *src)
{
    if (s->state != FW_BL_RECEIVING || idx != s->next)
        return;
    uint16_t off = idx * FW_CHUNK_SIZE;
    if (off >= s->len)
        return;
    if (off < FW_PAGE_SIZE)
    {
        memcpy(&s->page0[off], src, FW_CHUNK_SIZE);
    }
    else
    {
        bl_hal_flash_fill(FW_APP_START + off, src, FW_CHUNK_SIZE);
        if (idx & 1)
        {
            /* zweite Haelfte der Seite -> Seite schreiben */
            bl_hal_flash_erase_write(FW_APP_START + off);
        }
    }
    s->next++;
}

/* Rueckgabe 1 = fertig, Anwendung starten */
static uint8_t do_finish(bl_state_t *s)
{
    if (s->state == FW_BL_DONE)
        return 1;
    if (s->state != FW_BL_RECEIVING || (uint32_t)s->next * FW_CHUNK_SIZE < s->len)
        return 0;

    /* letzte, nur halb gefuellte Seite noch schreiben */
    if ((s->next & 1) && s->next > 2)
    {
        bl_hal_flash_erase_write(FW_APP_START + (uint16_t)(s->next - 1) * FW_CHUNK_SIZE);
    }

    if (image_crc(s, 1) != s->crc)
    {
        s->state = FW_BL_ERR_CRC;
        return 0;
    }

    /* erste Seite (Reset-Vektor) zuletzt schreiben -> App wird gueltig */
    bl_hal_flash_fill(FW_APP_START, s->page0, FW_PAGE_SIZE);
    bl_hal_flash_erase_write(FW_APP_START);

    /* Kontrolle direkt aus dem Flash */
    if (image_crc(s, 0) != s->crc)
    {
        bl_hal_flash_erase(FW_APP_START);
        s->state = FW_BL_ERR_CRC;
        return 0;
    }

    s->state = FW_BL_DONE;
    return 1;
}

uint8_t bl_handle_packet(bl_state_t *s, uint8_t *p)
{
    uint8_t act = 0;

    if (bl_hal_crc16(p, BL_PACKET_SIZE - 2) != get16(p + BL_PACKET_SIZE - 2))
    {
        /* defektes Paket unveraendert weiterleiten (der Controller zaehlt CRC-Fehler) */
        return 0;
    }
    act = BL_ACT_VALID;

    uint8_t me = p[3];
    p[3] = me + 1; /* hops */

    if (p[0] <= me && me <= p[1])
    {
        uint8_t cmd = p[2] & 0x0F;
        uint8_t processed = 1;

        if (cmd == FW_CMD_DATA)
        {
            do_data(s, get16(p + 4), MDP(p, 0));
        }
        else if (cmd == FW_CMD_CONTROL)
        {
            switch (p[BL_MD + 2 * FW_MD_SUB])
            {
            case FW_START:
                do_start(s, p);
                break;
            case FW_FINISH:
                if (do_finish(s))
                    act |= BL_ACT_RESET;
                break;
            case FW_EXIT:
                if (s->state != FW_BL_RECEIVING && bl_app_valid())
                    act |= BL_ACT_RESET;
                break;
            case FW_ANNOUNCE:
            {
                uint16_t bit = (uint16_t)1 << (me & 0x0F);
                put16(MDP(p, FW_MD_SEEN_MASK), get16(MDP(p, FW_MD_SEEN_MASK)) | bit);
                put16(MDP(p, FW_MD_INBOOT_MASK), get16(MDP(p, FW_MD_INBOOT_MASK)) | bit);
                put16(MDP(p, FW_MD_NEEDS_MASK), get16(MDP(p, FW_MD_NEEDS_MASK)) | bit);
                break;
            }
            default:
                /* FW_STATUS, FW_ENTER_BOOT: nur Status melden */
                break;
            }
        }
        else
        {
            /* normale BMS-Kommandos beantwortet der Bootloader nicht */
            processed = 0;
        }

        if (processed)
        {
            act |= BL_ACT_MINE;
            p[2] |= 0x80;
            if (cmd != FW_CMD_CONTROL || p[BL_MD] != FW_ANNOUNCE)
            {
                put16(MDP(p, FW_MD_BL_NEXT), s->next);
                put16(MDP(p, FW_MD_BL_STATUS), FW_BL_SIGNATURE | s->state);
                put16(MDP(p, FW_MD_BL_VERSION), FW_BL_PROTOCOL_VERSION);
            }
        }
    }

    put16(p + BL_PACKET_SIZE - 2, bl_hal_crc16(p, BL_PACKET_SIZE - 2));
    return act;
}
