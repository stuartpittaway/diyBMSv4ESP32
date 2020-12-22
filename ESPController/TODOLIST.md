# diyBMS v4
## New Controller To-Do-List

## Things To Check

(in no particular order)

### ESP32 BOOT button for WIFI RESET

GPIO 0 pin connected to interrupt, hold BOOT button on ESP32 for more than 4 seconds to factory reset the WIFI settings stored in EEPROM.
Once reset, the LED lights CYAN, at this point reset the controller to enter either the terminal based WIFI configuration (by pressing 
SPACE bar) or WIFI access point configuration (default).  Connect to Wifi SSID DIY_BMS_CONTROLLER and IP address 192.168.4.1 to ensure
set up pages.

### USB Debugging/Console

USB Serial is connected to second UART on ESP32 allowing full console access and debug serial port via USB cable.
Also used for terminal based WIFI configuration.

### TCA9534A
Controls RGB LED, TFT display LED and AVR ISP reset line

### TCA6408AQPWRQ1
Controls relay's and external IO A/B/C/D/E

### Relay 1
### Relay 2
### Relay 3
### RGB LED
### TFT Screen
### SD CARD
### TX1/RX1

Uses GPIO2 for RX and 32 for TX.  Works as per ESP8266 modules using hardware based UART.

### I/O ports
### CANBUS
### RS485
### ATTINY ISP Programming
### External 5v power supply
### SD Card on TFT display
### TFT Touch
### TX2/RX2 ?
