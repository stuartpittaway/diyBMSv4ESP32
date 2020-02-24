BMS Module Serial Protocol
==========================

Serial protocol documentation for *diyBMSv4* BMS cell modules by
GitHub user [stuartpittaway](https://github.com/stuartpittaway).

Hardware repo: [diyBMSv4](https://github.com/stuartpittaway/diyBMSv4)  
Code repo: [diyBMSv4Code](https://github.com/stuartpittaway/diyBMSv4Code)

Hardware
--------

The cell modules use standard UART serial communication, but are wired in a
daisy chain with the TX output of one module going to the RX of the next in
the chain.  The controller ties the ends together. This scheme allows
individual modules to be addressed with a specific command, or for them all
to respond to a single command. The protocol allows space in the packet for
each module to provide a response.

**Serial Parameters:** 2400, N81

```
                               CONTROLLER
                                +------+
                                |      |
 +--<--------------------<------|TX  RX|<------------------<----------+
 |                              |      |                              |
 |                              +------+                              |
 |                                                                    |
 |  +------+      +------+      +------+      +------+      +------+  |
 |  |      |      |      |      |      |      |      |      |      |  |
 +->|RX  TX|----->|RX  TX|-...->|RX  TX|----->|RX  TX|----->|RX  TX|--+
    |      |      |      |      |      |      |      |      |      |
    +------+      +------+      +------+      +------+      +------+
     MODULE        MODULE        MODULE        MODULE        MODULE
     BANK 0        BANK 0        BANK 0        BANK 1        BANK 1
     ADDR 0        ADDR 1  ...   ADDR N        ADDR 0        ADDR 1  ...
```

Packet Framing
--------------

To simplify packet framing, the protocol utilizes
[Consistent Overhead Byte Stuffing (COBS)](
https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing).
While the module firmware handles packet encoding and decoding, it is important
to understand how this works if you intend to examine the data on the wire with
a serial trace tool (such as bus pirate).

Packets are 0-terminated. The COBS encoding takes care of any actual zeroes in
the packet contents.

Packet Format
-------------

The packet has a header consisting of an address, a command code, and a packet
sequence number. The payload is a data array with one slot for each module in
a bank. By default, there are 16-data slots to support 16 modules in a bank.
The packet ends with a CRC for error detection.

### Packet Structure

| Field  | Bits | Definition             |
|--------|------|------------------------|
|Address |   8  | device address         |
|Command |   8  | command code           |
|Sequence|  16  | packet sequence number |
|Data[0] |  16  | device 0 data item (varies based on command) |
|Data[1] |  16  | device 1 data item     |
|Data[N] |  16  | device N data item     |
|CRC     |  16  | packet checksum        |

#### Address Field

| Bit | Definition       |
|-----|------------------|
|  7  | broadcast flag   |
|  6  | reserved         |
| 5:4 | bank (0-3)       |
| 3:0 | address (0-15)   |

* _Broadcast Flag_ - if set then all modules of the specified bank will process
  the command, else only the addressed module processes the command.
* _Bank_ - A single chain of modules can have 4 banks of 16 modules each. A
  module will ignore any packet that does not have a matching bank number.
  A broadcast command applies to all modules in the chain that match the bank,
  but not any modules on the chain that are a different bank.

#### Command Field

| Bit | Definition            |
|-----|-----------------------|
|  7  | packet processed flag |
| 6:4 | reserved              |
| 3:0 | command code          |

* _Packet Processed Flag_ - will be set by the module if it processed the
  command. The controller can detect this in the response packet.
* _Command Code_ - number indicating the kind of command. See the command
  descriptions in the following sections.

Address Provisioning
--------------------

The BMS cell module devices store their bank assignment in non-volatile memory.
So a BMS module remembers its bank once assigned (default is 0). However the
device address is not remembered across a power cycle or cold reset. Therefore,
the first operation when talking to a string of devices should be to set the
address.

The broadcast flag is used to set the address. If the broadcast flag is set in
a command, then the device will assign itself the address from the command, and
then increment the address field which is passed along to the next BMS module
in the chain.

For example, if there are 4 devices, all in the same bank, then send an
"Identify" command with device address 0 and the broadcast flag. This will
cause the 4 devices to be assigned addresses 0, 1, 2, and 3, in sequence.

Before a BMS module is assigned an address this way, it cannot process any
command.

Commands
--------

|ID| Command      | Input        |Output                       |
|--|--------------|--------------|-----------------------------|
|0 |Set Bank      |bank num (0-3)|0xFFFF                       |
|1 |Volt/Status   | N/A          |Status/Voltage (see detail)  |
|2 |Identify      | N/A          |0xFFFF                       |
|3 |Temperature   | N/A          |Temperature (see detail)     |
|4 |Bad packets   | N/A          |count of bad packets received|
|5 |Read Settings | N/A          |device settings (see detail) |
|6 |Write Settings| new settings |device settings              |

Note: in the command details below, "module data field" means the 16-bit
data field in the response packet payload for the addressed module. The payload
has 16 data fields, one for each possible device in a bank. For example, if the
device address is `3`, then the module data field for that device is `Data[3]`.

### Set Bank (0)

This command is used to set the module bank. The bank value is specified in
bits 1:0 of the module data field and can be 0-3. The module will update its
bank number and save the new value in its internal EEPROM. The module will
return `0xFFFF` in the module data field.

### Read Voltage and Status (1)

This command is used to read the cell voltage and module status. The values
are returned through the 16-bit module data field according to the following
format:

| Bit | Definition                             |
|-----|----------------------------------------|
| 15  | bypass active                          |
| 14  | bypass overheated                      |
| 13  | reserved                               |
| 12:0| cell voltage in millivolts (0-8192 mV) |

### Identify (2)

This command causes the module to turn on the LEDs so it can be visually
identified. The LEDs will remain turned on for the next 10 packets received
by the module. The module will return `0xFFFF` in the data field.

### Read Temperature (3)

This command returns the internal and external temperature readings as two
8-bit unsigned values in the module data field, according to the following
format:

| Bit | Definition                                      |
|-----|-------------------------------------------------|
|15:8 | internal temperature (onboard bypass resistors) |
| 7:0 | external temperature sensor (offboard)          |

#### Temperature Units

The temperature is in units of Celcius. However, it is offset by 40 degrees C.
Subtract 40 from the returned value to get the temperature in C. This allows a
range of -40 to +216 C.

### Bad Packets (4)

The BMS cell module maintains a counter that tracks the number of bad packets
that it sees on the bus. This command will return the count of bad packets. It
can be used for diagnostics.

### Read Settings (5)

This command is used to read and return the BMS cell module internal settings.
Unlike other commands that use only the module data field, this command uses
multiple slots in the data field. Therefore it can only be used for one device
at a time (not broadcast).

The device settings are returned in multiple data fields according to the
following foramt:

|Index| Default | Definition                                |
|-----|---------|-------------------------------------------|
|  0  | 4.40    | load resistance, float[0]                 |
|  1  | ...     | load resistance, float [1]                |
|  2  | 2.21    | voltage calibration coefficient, float[0] |
|  3  | ...     | voltage calibration coefficient, float[1] |
|  4  | 2.0     | scale, mV/bit, float[0]                   |
|  5  | ...     | scale, mV/bit, float[1]                   |
|  6  | 70      | over temperature limit in C               |
|  7  | 4100    | bypass threshold millivolts               |
|  8  | 4150    | internal thermistor B coefficient         |
|  9  | 4150    | external thermistor B coefficient         |
| 10  | 4       | Hardware version of the module (4/4.1/4.2 are reported as 4  |

### Write Settings (6)

This command is used to program new settings to the device. It uses the same
format in the data fields as the "Read Settings" command, above. The new
settings will be stored in device non-volatile memory.

