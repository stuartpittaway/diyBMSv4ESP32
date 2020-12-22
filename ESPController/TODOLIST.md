# diyBMS v4
## New Controller To-Do-List

### Things To Check

(in no particular order)

1. ESP32 BOOT button for WIFI RESET

GPIO 0 pin connected to interrupt, hold BOOT button on ESP32 for more than 4 seconds to factory reset the WIFI settings stored in EEPROM.
Once reset, the LED lights CYAN, at this point reset the controller to enter either the terminal based WIFI configuration (by pressing 
SPACE bar) or WIFI access point configuration (default).  Connect to Wifi SSID DIY_BMS_CONTROLLER and IP address 192.168.4.1 to ensure
set up pages.

2. Relay 1
3. Relay 2
4. Relay 3
5. RGB LED
6. TFT Screen
7. SD CARD
8. TX1/RX1

Uses GPIO2 for RX and 32 for TX.  Works as per ESP8266 modules using hardware based UART.

9. I/O ports
10. CANBUS
11. RS485
12. ATTINY ISP Programming
13. External 5v power supply
14. SD Card on TFT display
15. TFT Touch
16. TX2/RX2 ?
17. USB Debugging/Console

USB Serial is connected to second UART on ESP32 allowing full console access and debug serial port via USB cable.
Also used for terminal based WIFI configuration.