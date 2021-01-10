# diyBMS v4

Version 4 of the diyBMS.  Do-it-yourself battery management system for Lithium ion battery packs and cells

If you are looking for version 3 of this project take a look here https://github.com/stuartpittaway/diyBMS

# Support the project

If you find the BMS useful, please consider buying me a beer, check out [Patreon](https://www.patreon.com/StuartP) for more information.

Any donations go towards the on going development and prototype costs of the project.

# Videos on how to use and build

https://www.youtube.com/stuartpittaway


# How to use the code

![Master Branch](https://github.com/stuartpittaway/diyBMSv4Code/workflows/PlatformIO%20CI/badge.svg?branch=master)

This release removes the need to manually compile the code yourself, instead GITHUB Actions are now used to build the code for you automatically.

The files you will need are held as ZIP files in [Releases](https://github.com/stuartpittaway/diyBMSv4Code/releases)



# Hardware

Hardware for this code is in a seperate repository, and consists of a controller (you need 1 of these) and modules (one per series cell in your battery)

https://github.com/stuartpittaway/diyBMSv4

## Controller Code
ESP8266 (recommend Wemos D1 Mini) is currently supported, ESP32 version coming soon.

Both [Wemos D1 Mini](https://amzn.to/3i1gPIz) and Wemos D1 Mini Pro are supported - minimum of 4MB flash memory.


## Module Code
Module code runs on the ATTINY841 micro controller, it is important to program the chip with the correct version of code depending on your PCB version.

You will need a programming device capable of programming ATMEL AVR chips - like the [USBASP programmer](https://amzn.to/2JZRp1h)


* V400 = Original board (marked DIYBMS v4 on silkscreen) - has 8 large resistors (marked 2R20) and likely handsoldered using 0805 sized parts [4.0 boards do have TP2 near the ATTINY841 chip]

* V410 = JLCPCB built board (marked DIYBMS v4 on silkscreen) - has 8 large resistors (marked 2R00) and machine soldered using 0603 sized parts [4.1 boards do not have TP2 near the ATTINY841 chip]

* V420 = JLCPCB built board (marked DIYBMS v4.2 on silkscreen) - has 20 small resistors (marked 6R20) and machine soldered using 0603 sized parts (R20 is in middle of resistor array)

* V420_SWAPR19R20 = JLCPCB built board (marked DIYBMS v4.2 on silkscreen) - has 20 small resistors (marked 6R20) and machine soldered using 0603 sized parts [you have manually resoldered R19 and R20 to swap the positions on PCB to move the thermistor inside the resistor array]

* V421 = JLCPCB built board (marked DIYBMS v4.21 on silkscreen) - has 20 small resistors (marked 6R20) and machine soldered using 0603 sized parts (R19 is in middle of resistor array)

* V430 = JLCPCB built board (marked DIYBMS v4.3 on silkscreen) - not released to public - experimental version.

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

