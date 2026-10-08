# Modul-Firmware-Update über die RX/TX-Kette

Neue Firmware für die Zellmodule (ModuleV450 / Bolt123 Rev B, ATtiny1624) wird vom ESP32-Controller
über die vorhandenen RX/TX-Leitungen in die Module geschrieben. Die Module bleiben dabei
angeschlossen und eingebaut.

## Was passiert

1. **Versionsvergleich im Modul.** Der ESP32 schickt ein Ankündigungspaket (Länge + CRC32 des neuen
   Images) an alle 16 Module. Jedes Modul berechnet die CRC32 über seinen eigenen Programmspeicher und
   meldet in einem Bit, ob es abweicht. Ein Modul mit gleicher Firmware macht nichts.
2. **Eins nach dem anderen.** Für jedes abweichende Modul *k* (0 … 15):
   * `FW_ENTER_BOOT` → Modul *k* startet per Software-Reset seinen Bootloader.
   * `FW_START` → der Bootloader löscht die erste Seite der alten Anwendung (ab jetzt bleibt er aktiv,
     auch nach Stromausfall).
   * `FW_DATA` → das Image in Blöcken zu 32 Byte. Die Module 0 … *k*-1 leiten die Pakete weiter,
     Modul *k* schreibt sie ins Flash und quittiert, die Module *k*+1 … 15 leiten die Quittung zurück
     zum ESP32. Verlorene Blöcke werden ab dem fehlenden Block neu gesendet.
   * `FW_FINISH` → CRC32 über das ganze Image prüfen, erste Seite (Reset-Vektor) zuletzt schreiben,
     neu starten.
   * Kontrolle: das Modul vergleicht noch einmal selbst und muss „aktuell“ melden.
3. Danach das nächste Modul, bis alle 16 fertig sind.

Während Modul *k* aktualisiert wird (ca. 1,5 min bei 5000 Baud), sind nur **seine** Messwerte
eingefroren; alle anderen Module messen und balancieren normal weiter, und der Controller fragt die
Spannungen weiter ab.

## Hardware

**An der Platine muss nichts geändert werden.** Die Optokoppler-Kette RX/TX überträgt die
Update-Pakete wie normale Pakete (gleiches 40-Byte-Format, gleiche Baudrate). Der ATtiny1624
(JLCPCB C5160891, lagernd) hat genug Flash: Bootloader 1,5 KB + Anwendung 9,1 KB von 14,5 KB.

Einmalig pro Modul wird der Bootloader per **UPDI** geschrieben. Dafür reicht am Steckplatz `UPDI1`
(VCC / UPDI / GND, 2,54 mm) eine eingesteckte Stiftleiste oder ein Pogo-Adapter, löten ist nicht nötig.

Der ATtiny3224 (doppelter Flash) wäre pinkompatibel, ist bei JLCPCB aber kaum lagernd und wird nicht
gebraucht.

## Erstinbetriebnahme (je Modul einmal, per UPDI)

Baudrate passend zum Controller wählen (`V450_BL_5K` oder `V450_BL_10K`):

```
cd ATTINYBootloader && make              # nur wenn der Bootloader geändert wurde (fertige HEX liegen in prebuilt/)
cd ATTINYCellModule
pio run -e V450_BL_5K -t fuses          # u.a. BOOTEND = 0x06 (1,5 KB Bootloader), OSCCFG 20 MHz, BOD 1,8 V
pio run -e V450_BL_5K -t uploadall      # Bootloader + Anwendung
```

Ohne PlatformIO, z.B. mit SerialUPDI / avrdude 7+:

```
avrdude -p t1624 -c serialupdi -P COM5 -U fuse2:w:0x02:m -U fuse8:w:0x06:m -U fuse7:w:0x00:m
avrdude -p t1624 -c serialupdi -P COM5 -U flash:w:module_fw_V450_BL_5K_..._mit_bootloader.hex:i
```

Nach dem Einschalten blinkt die LED 4× (Anwendung läuft). Ab jetzt nie wieder UPDI nötig.

## Update durchführen

1. Modulfirmware bauen: `pio run -e V450_BL_5K` (bzw. `_10K`). Das Skript `GenerateBinaryFile.py`
   legt das Image und einen Eintrag in `ESPController/data/avr/manifest.json` an
   (Felder `chain`, `len`, `crc32`, `baud`, `boardver`).
2. ESP32-Dateisystem hochladen (`ESPController`: *Upload Filesystem Image*).
3. Weboberfläche → **AVR Programmer** → Abschnitt **Modul-Firmware über die RX/TX-Kette**:
   Image (z.B. `V450_BL_5K`) anklicken → **Update starten**. Fortschritt und Status je Modul werden
   alle 2 s aktualisiert. **Abbrechen** ist jederzeit möglich.

REST: `POST /post/modfwstart` (`file=<Index im Manifest>`), `POST /post/modfwabort`,
`GET /api/modfwstatus`.

## Dauer (Simulation, 16 Module, ohne Störungen)

| Baudrate | je Modul | 16 Module |
|---|---|---|
| 5000 | ca. 1,5 min | ca. 25 min |
| 10000 | ca. 50 s | ca. 13 min |

Module mit gleicher Firmware kosten nur den Vergleich (ca. 3 s für alle).

## Fehlerfälle

| Fall | Verhalten |
|---|---|
| Paket verloren/verfälscht | CRC16 je Paket; ab dem fehlenden Block neu senden |
| Stromausfall oder Reset während der Übertragung | Anwendung ist ungültig → Bootloader bleibt aktiv, der ESP32 beginnt das Modul neu |
| ESP32 meldet sich nicht mehr, bevor `FW_START` kam | nach 8 s (Watchdog) startet wieder die alte Firmware |
| CRC32 nach der Übertragung falsch | erste Seite wird nicht geschrieben, neuer Versuch (max. 4 je Modul), dann „Fehler“ und weiter mit dem nächsten Modul |
| Modul mit alter Firmware ohne Update-Funktion | „keine Antwort“, wird übersprungen, leitet aber weiter |
| Firmware ohne Bootloader / falsche Hardware / Image zu groß | „ohne Bootloader“, wird übersprungen |
| Image für andere Baudrate | Start wird abgelehnt |

Ein Modul, das im Bootloader steht, leitet alle Pakete weiter – die Kette bleibt für die anderen
Module immer intakt.

## Dateien

| Pfad | Inhalt |
|---|---|
| `common/fwupdate_protocol.h` | Protokoll (Kommandos 12/13, Felder, Speicheraufteilung) |
| `ATTINYBootloader/` | Bootloader (C, avr-gcc), `prebuilt/` fertige HEX für 5000 und 10000 Baud |
| `ATTINYCellModule/src/fwupdate_module.cpp` | Versionsvergleich + Bootloader-Aufruf in der Modulfirmware |
| `ATTINYCellModule/buildscript_bootloader.py` | Linken ab 0x0600, kombinierte HEX, Ziel `uploadall` |
| `ESPController/src/ModuleFirmwareUpdate.cpp` | Ablaufsteuerung + FreeRTOS-Task + JSON-Status |
| `test/fwupdate_sim/` | PC-Simulation mit 16 Modulen (`make && ./sim`) |

Geänderte vorhandene Dateien: `ATTINYCellModule/{platformio.ini, GenerateBinaryFile.py, include/defines.h,
include/packet_processor.h, src/packet_processor.cpp, src/main.cpp}`,
`ESPController/src/{main.cpp, PacketReceiveProcessor.cpp, webserver_json_post.cpp,
webserver_json_requests.cpp}`, `ESPController/web_src/{default.htm, pagecode.js}`.

Hinweis: In `V450_BL_*` ist `LOAD_RESISTANCE=1.20` (16 × 1R2 der Bolt123-Platine) eingetragen.

## Speicheraufteilung ATtiny1624

```
0x0000 ─ 0x05FF   Bootloader (BOOT, 1,5 KB)      FUSE.BOOTEND = 0x06
0x0600 ─ 0x3FFF   Anwendung  (APPCODE, 14,5 KB)  FUSE.APPEND  = 0x00
```

Übergabe Anwendung → Bootloader: `0xB0071624` an den ersten 4 Byte des SRAM + Software-Reset.
