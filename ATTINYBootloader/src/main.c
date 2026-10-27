/*
 * DIYBMS ATtiny1624 serial bootloader (1.5 KB, boot section 0x0000-0x05FF)
 *
 * Starts the application (0x0600) immediately, unless
 *   a) the application requested the bootloader via software reset with FW_BOOT_MAGIC, or
 *   b) the application is invalid (first page empty, e.g. after an aborted update).
 *
 * In the bootloader:
 *   - USART0 on PB2 (TXD) / PB3 (RXD), same baud rate as the module firmware
 *   - EVERY received packet is forwarded (with hops+1) to the next module, so the
 *     chain stays intact for all other modules
 *   - packets for this module (FW_CMD_CONTROL/FW_CMD_DATA) are processed and acknowledged
 *   - watchdog 8 s: while the old application is still valid, only packets for this module
 *     reset it -> if the ESP32 stops talking, the old application starts again after 8 s.
 *     After FW_START (application erased) the bootloader stays active until an image is complete.
 *   - LED (PA6) toggles on every packet for this module
 *
 * Fuses: BOOTEND = 0x06 (1.5 KB bootloader), APPEND = 0x00, OSCCFG = 20 MHz (as module firmware).
 */
#include <avr/io.h>
#include <avr/xmega.h>
#include <util/crc16.h>
#include <string.h>
#include "bl_core.h"

#ifndef BL_BAUD
#error BL_BAUD must be defined (same baud rate as DIYBMSBAUD of the module firmware)
#endif
#ifndef F_CPU
#define F_CPU 5000000UL
#endif

#define FRAME_START 0xAA
#define FRAME_ESCAPE 0x88

/* ---------------- hardware abstraction ---------------- */

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
    (void)addr; /* the address was latched by the last write to the page buffer */
    nvm_cmd(NVMCTRL_CMD_PAGEERASEWRITE_gc);
}

void bl_hal_flash_erase(uint16_t addr)
{
    MAPPED(addr) = 0xFF; /* select the page */
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

/* ---------------- serial port ---------------- */

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
    USART0.STATUS = USART_TXCIF_bm; /* clear flag */
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
    /* First check the handover from the application (at the start of SRAM, before any
       variable is written there). RSTFR is NOT cleared, the application (megaTinyCore)
       evaluates it itself. */
    volatile uint32_t *magic = (volatile uint32_t *)RAMSTART;
    uint8_t requested = (RSTCTRL.RSTFR & RSTCTRL_SWRF_bm) && (*magic == FW_BOOT_MAGIC);
    *magic = 0;

    if (!requested && bl_app_valid())
        start_app();

    /* ---- bootloader mode ---- */
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_4X_gc | CLKCTRL_PEN_bm); /* 20 MHz / 4 = 5 MHz */
    _PROTECTED_WRITE(WDT.CTRLA, WDT_PERIOD_8KCLK_gc);

    PORTA.DIRSET = PIN6_bm; /* LED */
    PORTB.OUTSET = PIN2_bm; /* TX idle level high */
    PORTB.DIRSET = PIN2_bm;
    USART0.BAUD = (uint16_t)((4UL * F_CPU) / BL_BAUD);
    USART0.CTRLB = USART_RXEN_bm | USART_TXEN_bm;

    rxhead = rxtail = 0;
    bl_init(&st);

    uint8_t idx = 0, inframe = 0, esc = 0;

    for (;;)
    {
        /* If the application is invalid (update running/aborted) there is only the bootloader:
           the watchdog then only protects against hangs. If the old application is still valid,
           only packets for this module reset the watchdog -> after 8 s of silence back to the app. */
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

        /* complete packet */
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
            sw_reset(); /* magic is cleared, application is valid -> application starts */

        }
    }
}
