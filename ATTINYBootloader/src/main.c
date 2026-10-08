/*
 * DIYBMS ATtiny1624 Seriell-Bootloader (1,5 KB, Boot-Bereich 0x0000-0x05FF)
 *
 * Startet die Anwendung (0x0600) sofort, ausser
 *   a) die Anwendung hat per Software-Reset mit FW_BOOT_MAGIC den Bootloader angefordert, oder
 *   b) die Anwendung ist ungueltig (erste Seite leer, z.B. nach abgebrochenem Update).
 *
 * Im Bootloader:
 *   - USART0 an PB2 (TXD) / PB3 (RXD), gleiche Baudrate wie die Modulfirmware
 *   - JEDES empfangene Paket wird (mit hops+1) an das naechste Modul weitergeleitet,
 *     die Kette bleibt also fuer alle anderen Module intakt
 *   - Pakete an dieses Modul (FW_CMD_CONTROL/FW_CMD_DATA) werden ausgewertet und quittiert
 *   - Watchdog 8 s: solange die alte App noch gueltig ist, laden ihn nur Pakete an dieses
 *     Modul nach -> meldet sich der ESP32 nicht mehr, startet nach 8 s wieder die alte App.
 *     Nach FW_START (App geloescht) bleibt der Bootloader aktiv, bis ein Image komplett ist.
 *   - LED (PA6) blinkt bei jedem Paket an dieses Modul
 *
 * Fuses: BOOTEND = 0x06 (1,5 KB Bootloader), APPEND = 0x00, OSCCFG = 20 MHz (wie Modulfirmware).
 */
#include <avr/io.h>
#include <avr/xmega.h>
#include <util/crc16.h>
#include <string.h>
#include "bl_core.h"

#ifndef BL_BAUD
#error BL_BAUD muss gesetzt sein (gleiche Baudrate wie DIYBMSBAUD der Modulfirmware)
#endif
#ifndef F_CPU
#define F_CPU 5000000UL
#endif

#define FRAME_START 0xAA
#define FRAME_ESCAPE 0x88

/* ---------------- Hardware-Abstraktion ---------------- */

#define MAPPED(a) (*(volatile uint8_t *)(MAPPED_PROGMEM_START + (a)))

static void nvm_cmd(uint8_t cmd)
{
    _PROTECTED_WRITE_SPM(NVMCTRL.CTRLA, cmd);
    while (NVMCTRL.STATUS & (NVMCTRL_FBUSY_bm | NVMCTRL_EEBUSY_bm))
        ;
}

uint8_t bl_hal_flash_read(uint16_t addr) { return MAPPED(addr); }

void bl_hal_flash_fill(uint16_t addr, const uint8_t *src, uint8_t len)
{
    while (len--)
        MAPPED(addr++) = *src++;
}

void bl_hal_flash_erase_write(uint16_t addr)
{
    (void)addr; /* Adresse wurde mit dem letzten Schreibzugriff auf den Seitenpuffer gelatcht */
    nvm_cmd(NVMCTRL_CMD_PAGEERASEWRITE_gc);
}

void bl_hal_flash_erase(uint16_t addr)
{
    MAPPED(addr) = 0xFF; /* Seite auswaehlen */
    nvm_cmd(NVMCTRL_CMD_PAGEERASE_gc);
}

void bl_hal_flash_bufclr(void) { nvm_cmd(NVMCTRL_CMD_PAGEBUFCLR_gc); }

uint16_t bl_hal_crc16(const uint8_t *p, uint8_t len)
{
    uint16_t crc = 0;
    while (len--)
        crc = _crc_xmodem_update(crc, *p++);
    return crc;
}

/* ---------------- serielle Schnittstelle ---------------- */

static uint8_t rxring[64];
static uint8_t rxhead, rxtail;

static void poll_rx(void)
{
    while (USART0.STATUS & USART_RXCIF_bm)
    {
        rxring[rxhead & 63] = USART0.RXDATAL;
        rxhead++;
    }
}

static void tx_byte(uint8_t b)
{
    while (!(USART0.STATUS & USART_DREIF_bm))
        poll_rx();
    USART0.TXDATAL = b;
}

static void send_packet(const uint8_t *p)
{
    USART0.STATUS = USART_TXCIF_bm; /* Flag loeschen */
    tx_byte(FRAME_START);
    for (uint8_t i = 0; i < BL_PACKET_SIZE; i++)
    {
        uint8_t v = p[i];
        if (v == FRAME_START || v == FRAME_ESCAPE)
        {
            tx_byte(FRAME_ESCAPE);
            v &= 0x7F;
        }
        tx_byte(v);
    }
}

static void wait_tx_done(void)
{
    while (!(USART0.STATUS & USART_TXCIF_bm))
        poll_rx();
}

static inline void start_app(void) __attribute__((noreturn));
static inline void start_app(void)
{
    asm volatile("jmp %0" ::"i"(FW_APP_START));
    __builtin_unreachable();
}

static inline void sw_reset(void) __attribute__((noreturn));
static inline void sw_reset(void)
{
    _PROTECTED_WRITE(RSTCTRL.SWRR, RSTCTRL_SWRE_bm);
    for (;;)
        ;
}

/* ---------------- Start ---------------- */

static bl_state_t st;
static uint8_t pkt[BL_PACKET_SIZE];

int main(void) __attribute__((OS_main, noreturn, used, externally_visible));

void _start(void) __attribute__((naked, used, section(".init0")));
void _start(void)
{
    asm volatile("clr r1");
    asm volatile("rjmp main");
}

int main(void)
{
    /* Zuerst die Uebergabe der Anwendung pruefen (liegt am SRAM-Anfang, bevor
       irgendeine Variable dort beschrieben wird). RSTFR wird NICHT geloescht,
       die Anwendung (megaTinyCore) wertet es selbst aus. */
    volatile uint32_t *magic = (volatile uint32_t *)RAMSTART;
    uint8_t requested = (RSTCTRL.RSTFR & RSTCTRL_SWRF_bm) && (*magic == FW_BOOT_MAGIC);
    *magic = 0;

    if (!requested && bl_app_valid())
        start_app();

    /* ---- Bootloader-Modus ---- */
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_4X_gc | CLKCTRL_PEN_bm); /* 20 MHz / 4 = 5 MHz */
    _PROTECTED_WRITE(WDT.CTRLA, WDT_PERIOD_8KCLK_gc);

    PORTA.DIRSET = PIN6_bm; /* LED */
    PORTB.OUTSET = PIN2_bm; /* TX Ruhepegel high */
    PORTB.DIRSET = PIN2_bm;
    USART0.BAUD = (uint16_t)((4UL * F_CPU) / BL_BAUD);
    USART0.CTRLB = USART_RXEN_bm | USART_TXEN_bm;

    rxhead = rxtail = 0;
    bl_init(&st);

    uint8_t idx = 0, inframe = 0, esc = 0;

    for (;;)
    {
        /* Ist die Anwendung ungueltig (Update laeuft/abgebrochen), gibt es nur den Bootloader:
           der Watchdog schuetzt dann nur vor Haengern. Ist die alte Anwendung noch gueltig,
           laden nur Pakete an dieses Modul den Watchdog nach -> nach 8 s Ruhe zurueck zur App. */
        if (!bl_app_valid())
            asm volatile("wdr");

        poll_rx();
        if (rxhead == rxtail)
            continue;

        uint8_t b = rxring[rxtail & 63];
        rxtail++;

        if (b == FRAME_START)
        {
            inframe = 1;
            idx = 0;
            esc = 0;
            continue;
        }
        if (!inframe)
            continue;
        if (b == FRAME_ESCAPE)
        {
            esc = 1;
            continue;
        }
        if (esc)
        {
            b |= 0x80;
            esc = 0;
        }
        pkt[idx++] = b;
        if (idx < BL_PACKET_SIZE)
            continue;

        /* komplettes Paket */
        inframe = 0;
        uint8_t act = bl_handle_packet(&st, pkt);
        if (act & BL_ACT_MINE)
        {
            asm volatile("wdr");
            PORTA.OUTTGL = PIN6_bm;
        }
        send_packet(pkt);

        if (act & BL_ACT_RESET)
        {
            wait_tx_done();
            sw_reset(); /* Magic ist geloescht, App ist gueltig -> App startet */
        }
    }
}
