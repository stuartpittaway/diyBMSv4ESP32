# Module firmware update over the RX/TX chain

New firmware for the cell modules (V450, ATtiny1624) is written into the modules by the ESP32
controller over the existing RX/TX communication chain. The modules stay connected and installed.

## What happens

1. **Version check inside the module.** The ESP32 sends an announce packet (length + CRC32 of the
   new image) to all modules. Every module calculates the CRC32 over its own program memory and
   reports in one bit whether it differs. A module that already has the same firmware does nothing.
2. **One after the other.** For every module *k* that differs:
   * `FW_ENTER_BOOT` → module *k* starts its bootloader via software reset.
   * `FW_START` → the bootloader erases the first page of the old application (from now on it
     stays active, even after a power loss).
   * `FW_DATA` → the image in blocks of 32 bytes. Modules 0 … *k*-1 forward the packets, module *k*
     writes them to flash and acknowledges, modules *k*+1 … *n*-1 forward the acknowledge back to
     the ESP32. Lost blocks are resent from the missing block onwards (Go-Back-N).
   * `FW_FINISH` → check the CRC32 over the whole image, write the first page (reset vector) last,
     restart.
   * Verification: the module compares again by itself and has to report "up to date".
3. Then the next module, until all modules are done.

While module *k* is being updated only **its** readings are frozen; all other modules keep measuring
and balancing, and the controller keeps polling the voltages.

## Hardware

**No changes to the PCB are needed.** The opto-isolated RX/TX chain carries the update packets like
normal packets (same 40 byte format, same baud rate). The ATtiny1624 has enough flash:
bootloader 1.5 KB + application approx. 9.1 KB of 14.5 KB.

The bootloader is written once per module via **UPDI** (header `UPDI1`: VCC / UPDI / GND).

## Restart without power cycle

All restarts are software resets (`RSTCTRL.SWRR`): application → bootloader and, after the update,
bootloader → new application. The modules stay connected to the cell, the ESP32 keeps running.

## Old modules with ATtiny841 (V400–V440)

Not supported. The ATtiny841 has only 8 KB flash, the existing firmware almost fills it, and it has
no protected boot section. For these modules the new code is not compiled in (build size unchanged).
In a mixed chain they forward the update packets and show up as "no reply"; they are still
programmed via ISP (AVR programmer on the Utilities page).

## First programming (once per module, via UPDI)

Select the baud rate that matches the controller (`V450_BL_5K` or `V450_BL_10K`):

```
cd ATTINYBootloader && make              # only if the bootloader was changed (ready HEX files are in prebuilt/)
cd ATTINYCellModule
pio run -e V450_BL_5K -t fuses          # BOOTEND = 0x06 (1.5 KB bootloader), OSCCFG 20 MHz, BOD 1.8 V, EESAVE
pio run -e V450_BL_5K -t uploadall      # bootloader + application
```

Without PlatformIO, e.g. with SerialUPDI and avrdude 7+:

```
avrdude -p t1624 -c serialupdi -P COM5 -e -U fuse0:w:0x00:m -U fuse1:w:0x14:m -U fuse2:w:0x02:m \
        -U fuse5:w:0xC5:m -U fuse6:w:0x06:m -U fuse7:w:0x00:m -U fuse8:w:0x06:m \
        -U flash:w:module_fw_V450_BL_5K_..._with_bootloader.hex:i
```

After power up the LED flashes 4 times (application running). From now on UPDI is not needed any more.

Note: `V450_BL_*` use `LOAD_RESISTANCE=3.30` like the standard V450 environments. Adjust it for
boards with a different balance load.

## Running an update

1. Build the module firmware: `pio run -e V450_BL_10K` (or `_5K`). `GenerateBinaryFile.py` puts the
   image into `ATTINYCellModule/firmware_upload/<env>_<sha>.bin`.
2. Web interface → **Utilities** → **ATtiny1624 – update all modules**:
   1. Select the file and the baud rate → **Upload** (`.bin` or `.hex`, also `_with_bootloader.hex`).
   2. Select the firmware with its radio button.
   3. **Start update**. Progress, duration (in seconds) and retransmissions per module are shown,
      **Abort** is possible at any time, **Check modules** reads the firmware checksum of every module.

The AVR programmer (ATtiny841 via ISP) is next to it, the controller firmware upgrade above.

REST: `POST /upload/modfw?xss=..&name=..&baud=..`, `POST /post/modfwstart` (`file=<index in manifest>`),
`POST /post/modfwabort`, `POST /post/modfwdelete`, `POST /post/modfwcheck`, `GET /api/modfwstatus`.

## Duration (simulation, 16 modules, no errors)

| Baud rate | per module | 16 modules |
|---|---|---|
| 5000 | approx. 45 s | approx. 12 min |
| 10000 | approx. 27 s | approx. 7 min |

Measured on real hardware: 1 module at 10000 baud in 26 s without retransmissions.
Modules that already have the same firmware only cost the comparison (approx. 3 s for all).

## Error handling

| Case | Behaviour |
|---|---|
| Packet lost/corrupted | CRC16 per packet; resend from the missing block |
| Power loss or reset during the transfer | application is invalid → bootloader stays active, the ESP32 restarts this module |
| ESP32 stops talking before `FW_START` | after 8 s (watchdog) the old firmware starts again |
| CRC32 wrong after the transfer | first page is not written, new attempt (max. 4 per module), then "failed" and continue with the next module |
| Module with old firmware without update support | "no reply", skipped, but still forwards |
| Firmware without bootloader / wrong hardware / image too large | "no bootloader", skipped |
| Image for a different baud rate | start is refused |

A module in the bootloader forwards all packets - the chain always stays intact for the other modules.

## Files

| Path | Content |
|---|---|
| `common/fwupdate_protocol.h` | protocol (commands 12/13/14, fields, memory layout) |
| `ATTINYBootloader/` | bootloader (C, avr-gcc), `prebuilt/` ready HEX files for 5000 and 10000 baud |
| `ATTINYCellModule/src/fwupdate_module.cpp` | version check + bootloader entry in the module firmware |
| `ATTINYCellModule/buildscript_bootloader.py` | link from 0x0600, combined HEX, target `uploadall` |
| `ESPController/src/ModuleFirmwareUpdate.cpp` | state machine + FreeRTOS task + JSON status + upload |
| `test/fwupdate_sim/` | PC simulation with up to 16 modules (`make && ./sim`) |

## ATtiny1624 memory layout

```
0x0000 - 0x05FF   bootloader  (BOOT, 1.5 KB)       FUSE.BOOTEND = 0x06
0x0600 - 0x3FFF   application (APPCODE, 14.5 KB)   FUSE.APPEND  = 0x00
```

Handover application → bootloader: `0xB0071624` in the first 4 bytes of SRAM + software reset.
