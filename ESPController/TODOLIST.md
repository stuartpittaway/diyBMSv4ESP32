# diyBMS v4
## New Controller To-Do-List

## Things To Check

(in no particular order)

### ESP32 BOOT button for WIFI RESET

GPIO0 pin connected to interrupt, hold BOOT button on ESP32 for more than 4 seconds to factory reset the WIFI settings stored in EEPROM.
Once reset, the LED lights CYAN, at this point reset the controller to enter either the terminal based WIFI configuration (by pressing 
SPACE bar) or WIFI access point configuration (default).  Connect to WIFI SSID "DIY_BMS_CONTROLLER" and IP address 192.168.4.1 to ensure
set up pages.

### USB Debugging/Console
USB Serial is connected to second UART on ESP32 allowing full console access and debug serial port via USB cable.
Also used for terminal based WIFI configuration.

### TCA9534A
Controls RGB LED, TFT display LED and AVR ISP reset line

### TCA6408AQPWRQ1
Controls relay's and external IO A/B/C/D/E

### Relay 1
Driven from pin 9/P4 of TCA6408AQPWRQ1, confirmed working.  Relay SRD-05VDC-SL-C, rated 10A @ 250VAC/ 28VDC

### Relay 2
Driven from pin 10/P5 of TCA6408AQPWRQ1, relay SRD-05VDC-SL-C, rated 10A @ 250VAC/ 28VDC
confirmed working

### Relay 3 (SSR)
Driven from pin 11/P6 of TCA6408AQPWRQ1, uses Panasonic AQY212GSZ.  Rated 60V @ 1A
confirmed working

### Relay 4 (SSR)
Driven from pin 12/P7 of TCA6408AQPWRQ1, uses Panasonic AQY212GSZ.  Rated 60V @ 1A
confirmed working

### TX1/RX1
Uses GPIO2 for RX and 32 for TX.  Works as per ESP8266 modules using hardware based UART, and EL3H7(B)(TA)-G isolator.

### I/O ports
Driven from pin 4/5/6/7 (P0/1/2/3) of TCA6408AQPWRQ1.  Maps to header socket marked A/B/C/D.

### External 5v power supply input
Confirmed working, 3.3v regulator working, reverse polarity protection working
Over voltage zener diode (ZMM5V6) not working as expected, incorrectly positioned in circuit diagram (fixed, but not tested in new revision)

### RGB LED
Confirmed working, driven from TCA9534A pins 4/5/6 (P0/P1/P2) BLUE, RED, GREEN.

### TFT Screen
Confirmed LED backlight working, driven from TCA9534A pin 7 (P3).

Display uses ILI9341 driver and is 240x320 pixels, with touch and SD Card interface (on seperate pins).

https://uk.banggood.com/2_8-Inch-ILI9341-240x320-SPI-TFT-LCD-Display-Touch-Panel-SPI-Serial-Port-Module-p-1206782.html

Around £9 UK GBP. Has two header pins, one for the touch and display, the other for the SD Card.

Looking top down onto the TFT screen (screen header pins on left marked J2) pins are

* VCC
* GND
* CS
* RESET
* DC
* MOSI
* SCK
* LED backlight
* MISO
* T_CLK (touch)
* T_CS (touch)
* T_DIN (touch)
* T_DO (touch)
* T_IRQ (touch)

### TFT Touch
Uses GPIO4 for chip select and VSPI interface for communication with XPT2046 driver
http://grobotronics.com/images/datasheets/xpt2046-datasheet.pdf

### SD CARD
Uses GPIO5 for chip select
Micro SD card onto controller PCB see B.O.M for part numbers

### CANBUS

Using TJA1051T/3 CAN Bus Transceiver

120ohm terminator resistor included on controller board (jumper to enable)

TX=GPIO16, RX=GPIO17 and RS=connected to P4 of TCA9534A (normally low, full speed CAN)

Confirmed working

### RS485
Using SN65HVD75DR,  3.3-V Supply RS-485 With IEC ESD protection.

Driven using Hardware Serial port Serial1, TX=GPIO22, RX=GPIO21, ENABLE=GPIO25

Confirmed working

### ATTINY ISP Programming
Connected to VSPI interface and uses P4 output on  TCA9534A to drive reset line.

VSPI should be disabled/not used whilst IVR programmer in use

### TX2/RX2 
Removed in newer revision of board
