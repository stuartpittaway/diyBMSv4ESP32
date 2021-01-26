# diyBMS v4

Version 4 of the diyBMS.  Do-it-yourself battery management system for Lithium ion battery packs and cells

If you are looking for version 3 of this project take a look here https://github.com/stuartpittaway/diyBMS

# Support the project

If you find the BMS useful, please consider buying me a beer, check out [Patreon](https://www.patreon.com/StuartP) for more information.

You can also send beer tokens via Paypal - [paypal.me/stuart2222](https://www.paypal.com/paypalme/my/profile)

Any donations go towards the on going development and prototype costs of the project.

# Videos on how to use and build

https://www.youtube.com/stuartpittaway

### Video on how to program the devices
https://youtu.be/wTqDMg_Ql98

### Video on how to order from JLCPCB
https://youtu.be/E1OS0ZOmOT8


# Help

If you need help, ask over at the [forum](https://community.openenergymonitor.org/t/diybms-v4)

If you discover a bug or want to make a feature suggestion, open a Github issue

# How to use the code

![Master Branch](https://github.com/stuartpittaway/diyBMSv4Code/workflows/PlatformIO%20CI/badge.svg?branch=master)

This release removes the need to manually compile the code yourself, instead GITHUB Actions are now used to build the code for you automatically.

The files you will need are held as ZIP files in [Releases](https://github.com/stuartpittaway/diyBMSv4Code/releases)

Download the ZIP file and *extract its contents* and inside the folder you should find:

*Files for the controller (ESP8266)*
* diybms_controller_firmware_espressif8266_esp8266_d1mini.bin
* diybms_controller_filesystemimage_espressif8266_esp8266_d1mini.bin

*Files for the modules (ATTINY841)*
* module_fw_V400_attiny841_400_eF4_hD6_l62.hex
* module_fw_V410_attiny841_410_eF4_hD6_l62.hex
* module_fw_V420_attiny841_420_eF4_hD6_l62.hex
* module_fw_V420_SWAPR19R20_attiny841_420_SWAPR19R20_eF4_hD6_l62.hex
* module_fw_V421_attiny841_421_eF4_hD6_l62.hex
* module_fw_V421_LTO_attiny841_421_eF4_hD6_l62.hex

You can ignore the "filesystemimage" for the esp8266, this is no longer required.

You will need to determine which module HEX file to use (see "Identify which module/board you have" below for help).  Most people will have a V4.00 or V4.21 board.

## Programming the controller

Both [Wemos D1 Mini](https://amzn.to/3i1gPIz) and Wemos D1 Mini & Pro are supported - minimum of 4MB flash memory, ESP32 version coming soon.

1. Connect the WEMOS D1 to the computer using a USB cable
1. Download the [esphome-flasher](https://github.com/esphome/esphome-flasher/releases) tool for your operating system
1. Run the program once downloaded
1. Select the correct serial port from the list for the Wemos D1
1. Click Browse and select the file "diybms_controller_firmware_espressif8266_esp8266_d1mini.bin"
1. Click "Flash ESP" and wait

You should see output similar to below, this is a WeMos D1 Mini Pro (16MB Flash)

```
Using 'COM3' as serial port.
Connecting....
Detecting chip type... ESP8266
Connecting....

Chip Info:
 - Chip Family: ESP8266
 - Chip Model: ESP8266EX
 - Chip ID: 00123456
 - MAC Address: AA:BB:CC:DD:EE:FF
Uploading stub...
Running stub...
Stub running...
Changing baud rate to 460800
Changed.
 - Flash Size: 16MB
 - Flash Mode: dout
 - Flash Frequency: 40MHz
Erasing flash (this may take a while)...

Writing at 0x000b0000... (100 %)
Wrote 872784 bytes (728955 compressed) at 0x00000000 in 17.2 seconds...
Hash of data verified.
Leaving...
Hard Resetting...
Done! Flashing is complete!
```

## Programming the modules

Module code runs on the ATTINY841 micro controller, it is important to program the chip with the correct version of code depending on your PCB version.

You will need a programming device capable of programming ATMEL AVR chips - like the [USBASP programmer](https://amzn.to/2JZRp1h)

### Setup the programmer

1. Connect the [USBASP programmer](https://amzn.to/2JZRp1h) to the computer
1. On the programmer, move the jumper pin (normally marked JP1) to use 3.3 volt programming settings (instead of 5 volt)
1. Completely disconnect the module from any battery/cell and the TX/RX connectors should also be unconnected.
1. Connect the programmer to the module using the 6 pin ISP connector on the module - take great care to ensure PIN 1 is aligned to PIN 1 of the programmer.  PIN 1 is marked on the PCB.
1. Download AVRDUDE 6.3 or newer, for [Windows](http://download.savannah.gnu.org/releases/avrdude/avrdude-6.3-mingw32.zip) other versions are [here](http://download.savannah.gnu.org/releases/avrdude/)
1. Extract the AVRDUDE zip file
1. Open a console/command window and change to the folder where you extracted the AVRDUDE program, on Windows this looks similar to this
```
cd C:\temp\avrdude-6.3-mingw32
```
9. The standard avrdude tool doesn't include support for ATTINY841 chips.  So download and overwrite the file avrdude.conf using the file from [here](https://raw.githubusercontent.com/SpenceKonde/ATTinyCore/master/avr/avrdude.conf)
1. Lets test connectivity to the programmer and module. Back in the console window, run the command below.  On Linux and Mac operating systems, you may need to use a different port insteoad of "usb" - for example /dev/tty1 but this will vary depending on the computer.  Note that the parameters ARE case sensitive.
```
avrdude -C avrdude.conf -P usb -c usbasp -p t841
```
11. All being well, it should report something similar to the below.  If not, check the wiring and ensure you are using the correct COM port.
```
avrdude: set SCK frequency to 187500 Hz
avrdude: AVR device initialized and ready to accept instructions
Reading | ################################################## | 100% 0.02s
avrdude: Device signature = 0x1e9315 (probably t841)
avrdude: safemode: Fuses OK (E:F4, H:D6, L:E2)
avrdude done.  Thank you.
```

### Programming the module

Programming the module takes around 12 seconds.

1. Identify which module/board you have using the details found at the end of this document.
1. Copy the required ".hex" file to the same folder where you extracted the avrdude tool to.
1. Now we shall program the module, run the command line similar to below, replacing the "diybms_module_firmware_400" filename where applicable.
1. The fuse settings are important, and are in the filename for example "eF4_hD6_l62" - means efuse=0xF4, hfuse=0xD6, lfuse=0x62
```
avrdude -C avrdude.conf -P usb -c usbasp -p t841 -e -B 8 -U efuse:w:0xF4:m -U hfuse:w:0xD6:m -U lfuse:w:0xE2:m -U flash:w:diybms_module_firmware_400.hex:i
```
it should output
```
avrdude: set SCK frequency to 187500 Hz
avrdude: AVR device initialized and ready to accept instructions
avrdude: Device signature = 0x1e9315 (probably t841)
avrdude: erasing chip
avrdude: set SCK frequency to 187500 Hz
avrdude: reading input file "0xF4"
avrdude: writing efuse (1 bytes):
Writing | ################################################## | 100% 0.00s
avrdude: 1 bytes of efuse written
avrdude: verifying efuse memory against 0xF4:
avrdude: load data efuse data from input file 0xF4:
avrdude: input file 0xF4 contains 1 bytes
avrdude: reading on-chip efuse data:
Reading | ################################################## | 100% 0.00s
avrdude: verifying ...
avrdude: writing hfuse (1 bytes):
Writing | ################################################## | 100% 0.00s
avrdude: 1 bytes of hfuse written
avrdude: reading on-chip hfuse data:
Reading | ################################################## | 100% 0.00s
avrdude: verifying ...
avrdude: writing lfuse (1 bytes):
Writing | ################################################## | 100% 0.00s
avrdude: 1 bytes of lfuse written
avrdude: verifying lfuse memory against 0xE2:
avrdude: reading on-chip lfuse data:
Reading | ################################################## | 100% 0.00s
avrdude: verifying ...
avrdude: writing flash (7718 bytes):
Writing | ################################################## | 100% 6.74s
avrdude: 7718 bytes of flash written
avrdude: verifying flash memory against diybms_module_firmware_XXX.hex:
Reading | ################################################## | 100% 3.43s
avrdude: verifying ...
avrdude: 7718 bytes of flash verified
avrdude: safemode: Fuses OK (E:F4, H:D6, L:E2)
avrdude done.  Thank you.
```
1. If programming fails, but the programmer appears to be communicating, try increasing the value of the "B" setting from 8 to 16 to slow down the USBASP device.
1. Check that the fuses report as "OK" and read E:F4, H:D6, L:E2
1. That module can now be disconnected from the USBASP programmer, connect the next module and repeat the avrdude command to program the next one.



# Hardware

Hardware for this code is in a seperate repository, and consists of a controller (you need 1 of these) and modules (one per series cell in your battery)

https://github.com/stuartpittaway/diyBMSv4


## Identify which module/board you have
* V400 = Original board (marked DIYBMS v4 on silkscreen) - has 8 large resistors (marked 2R20) and likely handsoldered using 0805 sized parts [4.0 boards do have TP2 near the ATTINY841 chip]

* V410 = JLCPCB built board (marked DIYBMS v4 on silkscreen) - has 8 large resistors (marked 2R00) and machine soldered using 0603 sized parts [4.1 boards do not have TP2 near the ATTINY841 chip]

* V420 = JLCPCB built board (marked DIYBMS v4.2 on silkscreen) - has 20 small resistors (marked 6R20) and machine soldered using 0603 sized parts (R20 is in middle of resistor array)

* V420_SWAPR19R20 = JLCPCB built board (marked DIYBMS v4.2 on silkscreen) - has 20 small resistors (marked 6R20) and machine soldered using 0603 sized parts [you have manually resoldered R19 and R20 to swap the positions on PCB to move the thermistor inside the resistor array]

* V421 = JLCPCB built board (marked DIYBMS v4.21 on silkscreen) - has 20 small resistors (marked 6R20) and machine soldered using 0603 sized parts (R19 is in middle of resistor array)

* V430 = JLCPCB built board (marked DIYBMS v4.3 on silkscreen) - not released to public - experimental version - DO NOT USE THIS WITH A LOWER VERSION BOARD!!

Open the module code, navigate to platformio environment "env:attiny841_VXXX", (where XXX is the version from above).  Connect your USBASP programmer to the module and select "Upload"


# WARNING

This is a DIY product/solution so don’t use this for safety critical systems or in any situation where there could be a risk to life.  

There is no warranty, it may not work as expected or at all.

The use of this project is done so entirely at your own risk.  It may involve electrical voltages which could kill - if in doubt, seek help.

The use of this project may not be compliant with local laws or regulations - if in doubt, seek help.


# How to compile the code yourself

The code uses [PlatformIO](https://platformio.org/) to build the code.  There isn't any need to compile the code if you simply want to use it, see "How to use the code" above.

If you want to make changes, fix bugs or poke around, use platformio editor to open the workspace named "diybms_workspace.code-workspace"


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

