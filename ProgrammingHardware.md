# DIYBMS v4 Programming The Hardware

## Programming the controller (ESP32)

ESP32 controller board uses "Dev Kit C" compatible controller boards using the ESP32-WROOM-32D (4MB)

Suitable boards are available from Amazon 
* https://amzn.to/3aTc3cS
* https://amzn.to/3a4GOfK

(affiliate links)

### Code

First step is to upload the main controller code to the ESP32.

You can flash the ESP32 whilst it is connected to the diyBMS controller board, no need to remove.

1. Unplug the ESP32 from its power supply/USB cable
1. If its not already, insert the ESP32 into the controller board, ensure that the ESP's USB connector faces towards the bottom of the board, facing the black ISP connector
1. Download the latest diyBMS ESP32 "Compiled Firmware" ZIP file from [Git Hub Releases](https://github.com/stuartpittaway/diyBMSv4ESP32/releases)
1. Open the ZIP file and extract the file named "esp32-controller-firmware-complete.bin" to your local drive
1. Connect the ESP32 to the computer using a micro USB cable
1. Download the [NodeMCU flasher tool](https://github.com/marcelstoer/nodemcu-pyflasher/releases/) for your operating system
1. Run the program once it has downloaded
1. Select the correct serial port from the list for the ESP32
1. Click Browse and select the file "esp32-controller-firmware-complete.bin"
1. Select the fastest baud rate (pick a slower one if the steps below fail)
1. Select "Dual Output DOUT" for Flash Mode
1. Select "YES" to Erase Flash
1. Click "Flash NodeMCU" and wait

You should see output similar to below:

```
Command: esptool.py --port COM4 --baud 921600 --after no_reset write_flash --flash_mode dout 0x00000 esp32-controller-firmware-complete.bin --erase-all

esptool.py v2.6
Serial port COM4
Connecting....
Detecting chip type... ESP32
Chip is ESP32D0WDQ6 (revision 1)
Features: WiFi, BT, Dual Core, 240MHz, VRef calibration in efuse, Coding Scheme None
MAC: 10:52:XX:XX:XX:XX
Uploading stub...
Running stub...
Stub running...
Changing baud rate to 921600
Changed.
Configuring flash size...
Auto-detected Flash size: 4MB
Erasing flash (this may take a while)...
Chip erase completed successfully in 6.6s

Compressed 4194304 bytes to 2177001...
Wrote 4194304 bytes (2177001 compressed) at 0x00000000 in 28.7 seconds (effective 1170.5 kbit/s)...

Hash of data verified.

Leaving...
Staying in bootloader.

Firmware successfully flashed. Unplug/replug or reset device 
to switch back to normal boot mode.
```

1. Once you see the message "Firmware successfully flashed.", reset the ESP device using the button marked "EN"
1. After a few seconds the LED on the controller board should turn SOLID WHITE, this means the WIFI configuration access point has started


## Connecting To WiFi

diyBMS presents a serial console to configure a WiFi connection to your local router/internet.

### Terminal/Serial Port - using USB cable
1. Download a terminal emulator like Putty or similar, which can be used to communicate over serial ports.  The Arduino IDE can also be used, in Serial Monitor mode.
1. Make a note of the serial port used whilst programming the device (COM4 in the example above)
1. Open the terminal emulator (Putty for example) and type in the name of the serial port COM4, and the settings baud rate=115200, 8 data bits, No parity, 1 Stop bit
1. Click Open/Connect
1. On the ESP32, press the "EN" button to reboot it
1. On the terminal emulator, you should see messages across the screen, like below:

```

                _          __
    _|  o      |_)  |\/|  (_
   (_|  |  \/  |_)  |  |  __)
           /
[I][main.cpp:2096] setup(): CONTROLLER - ver:e2a1c57f8dd12286fe83501c16c102c31dfd12c0 compiled 2021-02-10T13:54:49.608Z
[I][main.cpp:2101] setup(): ESP32 Chip model = 1, Rev 1, Cores=2, Features=50
[I][HAL_ESP32.cpp:168] ConfigureI2C(): Configure I2C
[I][HAL_ESP32.cpp:226] ConfigureI2C(): Found TCA9534A
[I][HAL_ESP32.cpp:256] ConfigureI2C(): Found TCA6408
[I][main.cpp:752] SetControllerState(): ** Controller changed state from Unknown to PowerUp **

Press SPACE BAR to enter terminal based configuration..............
```

1. Quickly tap the SPACE BAR to start configuration.  You have about 3 seconds to do this.  The controller will scan the local WiFi access points and present a list, like below.

```
DIYBMS CONTROLLER - Scanning Wifi

 0:myhomewifi                           1:WIFI
 2:private-wifi                         3:Virgin Media
 4:OpenReach                            5:BT
 6:Sky                                  7:TALKTALK
 8:VM1                                  9:VM2
10:VM11122
Enter the NUMBER of the Wifi network to connect to:
```

1. Type the number (not name) of the access point to connect to
1. When prompted, enter the access point password (asterisk characters will appear on screen)

```
Enter the password to use when connecting to 'TALKTALK': ******************
```

1. After the settings are saved, the controller reboots
1. A lot of messages will be shown on screen, however "Wi-Fi status=3" means that the controller has connected to the access point
1. You can also navigate using a web browser to the DIYBMS interface using the IP address listed in the log files - example below is http://192.168.0.68

```
Press SPACE BAR to enter terminal based configuration................skipped
[I][main.cpp:2355] setup(): Connecting to WIFI
[I][main.cpp:752] SetControllerState(): ** Controller changed state from PowerUp to Stabilizing **
[I][main.cpp:1164] connectToWifi(): Hostname: DIYBMS-001AAAAA
[D][WiFiGeneric.cpp:381] _eventCallback(): STA IP: 192.168.0.68, MASK: 255.255.255.0, GW: 192.168.0.1
[I][main.cpp:1372] onWifiConnect(): Wi-Fi status=3
[I][main.cpp:1374] onWifiConnect(): Request NTP from time.google.com
[D][DIYBMSServer.cpp:1953] StartServer(): Start Web Server complete
[I][ArduinoOTA.cpp:130] begin(): OTA server at: DIYBMS-001AAAAA.local:3232
[I][main.cpp:1416] onWifiConnect(): mDNS responder started
[I][main.cpp:1422] onWifiConnect(): You can access DIYBMS interface at http://DIYBMS-001AAAAA.local or http://192.168.0.68
```

1. If the controller fails to connect, you can repeat the whole process to reconfigure as needed

Note when the BMS asks for the wifi details, you have 30 seconds to enter them before the controller reboots


## Programming the modules

### Modules up to v4.40
The ESP32 controller supports programming the cell monitoring modules directly (only those supporting AVRISP 6 pin header). You no longer need to use a computer or USBASP style cable/adapter.

### Module v4.50 and newer
Modules after v4.50 require the use of an UPDI programming adapter.  Take a look at this website for help. https://teddywarner.org/Projects/SerialUPDI/#serial-programming

Download the pre-compiled files (inside the ZIP file) for programming from the releases section on GITHUB https://github.com/stuartpittaway/diyBMSv4ESP32/releases/

Look in the "Modules" folder of the release ZIP, for a file with a name similar to "module_fw_V450_10K_ATtiny1624_450_e0_h0_l0.hex"


# License

This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 2.0 UK: England & Wales License.

https://creativecommons.org/licenses/by-nc-sa/2.0/uk/

You are free to:
* Share — copy and redistribute the material in any medium or format
* Adapt — remix, transform, and build upon the material
The licensor cannot revoke these freedoms as long as you follow the license terms.

Under the following terms:
* Attribution — You must give appropriate credit, provide a link to the license, and indicate if changes were made. You may do so in any reasonable manner, but not in any way that suggests the licensor endorses you or your use.
* Non-Commercial — You may not use the material for commercial purposes.
* ShareAlike — If you remix, transform, or build upon the material, you must distribute your contributions under the same license as the original.
* No additional restrictions — You may not apply legal terms or technological measures that legally restrict others from doing anything the license permits.

Notices:
You do not have to comply with the license for elements of the material in the public domain or where your use is permitted by an applicable exception or limitation.

No warranties are given. The license may not give you all of the permissions necessary for your intended use. For example, other rights such as publicity, privacy, or moral rights may limit how you use the material.

