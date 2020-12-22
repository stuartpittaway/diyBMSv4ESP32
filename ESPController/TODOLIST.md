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
Driven from pin 9/P4 of TCA6408AQPWRQ1, confirmed working.

### Relay 2
Driven from pin 10/P5 of TCA6408AQPWRQ1

### Relay 3 (SSR)
Driven from pin 11/P6 of TCA6408AQPWRQ1

### TX1/RX1
Uses GPIO2 for RX and 32 for TX.  Works as per ESP8266 modules using hardware based UART.

### I/O ports
Driven from pin 4/5/6/7/12 (P0/1/2/3/7) of TCA6408AQPWRQ1

### External 5v power supply input
Confirmed working, 3.3v regulator working, reverse polarity protection working
Over voltage zener diode (ZMM5V6) not working as expected, incorrectly positioned in circuit diagram (fixed, but not tested in new revision)

### RGB LED
Confirmed working, driven from TCA9534A pins 4/5/6 (P0/P1/P2) BLUE, RED, GREEN.

### TFT Screen
Confirmed LED backlight working, driven from TCA9534A pin 7 (P3).

### SD CARD
### CANBUS
### RS485
### ATTINY ISP Programming

### SD Card on TFT display
### TFT Touch
### TX2/RX2 ?
