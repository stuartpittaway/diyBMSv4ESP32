/*
 ____  ____  _  _  ____  __  __  ___    _  _  __
(  _ \(_  _)( \/ )(  _ \(  \/  )/ __)  ( \/ )/. |
 )(_) )_)(_  \  /  ) _ < )    ( \__ \   \  /(_  _)
(____/(____) (__) (____/(_/\/\_)(___/    \/   (_)

DIYBMS V4.0
CELL MODULE FOR ATTINY841

(c)2019/2020 Stuart Pittaway

COMPILE THIS CODE USING PLATFORM.IO

LICENSE
Attribution-NonCommercial-ShareAlike 2.0 UK: England & Wales (CC BY-NC-SA 2.0 UK)
https://creativecommons.org/licenses/by-nc-sa/2.0/uk/

* Non-Commercial — You may not use the material for commercial purposes.
* Attribution — You must give appropriate credit, provide a link to the license, and indicate if changes were made.
  You may do so in any reasonable manner, but not in any way that suggests the licensor endorses you or your use.
* ShareAlike — If you remix, transform, or build upon the material, you must distribute your
  contributions under the same license as the original.
* No additional restrictions — You may not apply legal terms or technological measures
  that legally restrict others from doing anything the license permits.
*/
/*
IMPORTANT

You need to configure the correct DIYBMSMODULEVERSION in defines.h file to build for your module

*/

#define RX_BUFFER_SIZE 64

#include <Arduino.h>

#if !(F_CPU == 8000000)
#error Processor speed should be 8 Mhz internal
#endif

#if !defined(ATTINY_CORE)
#error Expected ATTINYCORE
#endif



//Our project code includes
#include "defines.h"
#include "settings.h"
#include <FastPID.h>

#include <SerialEncoder.h>
#include "diybms_attiny841.h"
#include "packet_processor.h"

uint8_t SerialPacketReceiveBuffer[8 + sizeof(PacketStruct)];

SerialEncoder myPacketSerial;

//Default values which get overwritten by EEPROM on power up
CellModuleConfig myConfig;

//DiyBMSATTiny841 hardware;

PacketProcessor PP(&myConfig);

volatile bool wdt_triggered = false;

void DefaultConfig()
{

  //Default bank zero
  //myConfig.mybank = 0;

  //About 2.2007 seems about right
  myConfig.Calibration = 2.2007;

  //2mV per ADC resolution
  //myConfig.mVPerADC = 2.0; //2048.0/1024.0;

#if defined(DIYBMSMODULEVERSION) && (DIYBMSMODULEVERSION == 420 && !defined(SWAPR19R20))
  //Keep temperature low for modules with R19 and R20 not swapped
  myConfig.BypassTemperatureSetPoint = 45;
#else
  //Stop running bypass if temperature over 65 degrees C
  myConfig.BypassTemperatureSetPoint = 65;
#endif

  //Start bypass at 4.1V
  myConfig.BypassThresholdmV = 4100;

  //#if defined(DIYBMSMODULEVERSION) && (DIYBMSMODULEVERSION == 430 || DIYBMSMODULEVERSION == 420 || DIYBMSMODULEVERSION == 421)
  //Murata Electronics NCP18WB473J03RB = 47K ±5% 4050K ±2% 100mW 0603 NTC Thermistors RoHS
  //myConfig.Internal_BCoefficient = 4050;
  //#else
  //4150 = B constant (25-50℃)
  //myConfig.Internal_BCoefficient = 4150;
  //#endif

  //4150 = B constant (25-50℃)
  //myConfig.External_BCoefficient = 4150;

  // Resistance @ 25℃ = 47k, B Constant 4150, 0.20mA max current
  //Using https://www.thinksrs.com/downloads/programs/therm%20calc/ntccalibrator/ntccalculator.html
}

ISR(WDT_vect)
{
  //This is the watchdog timer - something went wrong and no serial activity received in over 8 seconds
  wdt_triggered = true;
  PP.IncrementWatchdogCounter();
}

ISR(ADC_vect)
{
  // when ADC completed, take an interrupt and process result
  PP.ADCReading(DiyBMSATTiny841::ReadADC());
}

void onPacketReceived()
{
  DiyBMSATTiny841::EnableSerial0TX();

  //A data packet has just arrived, process it and forward the results to the next module
  if (PP.onPacketReceived((PacketStruct *)SerialPacketReceiveBuffer))
  {
    //Only light green if packet is good
    DiyBMSATTiny841::GreenLedOn();
  }

  //Send the packet (fixed length!) (even if it was invalid so controller can count crc errors)
  myPacketSerial.sendBuffer(SerialPacketReceiveBuffer);

  //PLATFORM ATTINYCORE version is 1.3.2 which is old, and SERIAL.FLUSH simply clears the buffer (bad)
  //Therefore we use 1.4.1 which has the correct code to wait until the buffer is empty.
  DiyBMSATTiny841::FlushSerial0();

  //Replace flush with a simple delay - we have 35+ bytes to transmit at 2400 baud + COBS encoding
  //At 2400bits per second, = 300 bytes per second = 1000ms/300bytes/sec= 3ms per byte
  //delay(10);

  DiyBMSATTiny841::GreenLedOff();
}

ISR(USART0_START_vect)
{
  //Needs to be here!
  asm("NOP");
}

//Kp: Determines how aggressively the PID reacts to the current amount of error (Proportional)
//Ki: Determines how aggressively the PID reacts to error over time (Integral)
//Kd: Determines how aggressively the PID reacts to the change in error (Derivative)

//6Hz rate - number of times we call this code in Loop
//Kp, Ki, Kd, Hz, output_bits, output_signed);
//Settings for V4.00 boards with 2R2 resistors = (4.0, 0.5, 0.2, 6, 8, false);
FastPID myPID(4.0, 0.5, 0.2, 6, 8, false);

void ValidateConfiguration()
{
  if (myConfig.Calibration < 1.9)
  {
    myConfig.Calibration = 2.21000;
  }

  if (myConfig.BypassTemperatureSetPoint > DIYBMS_MODULE_SafetyTemperatureCutoff)
  {
    myConfig.BypassTemperatureSetPoint = 55;
  }

#if defined(DIYBMSMODULEVERSION) && (DIYBMSMODULEVERSION == 420 && !defined(SWAPR19R20))
  //Keep temperature low for modules with R19 and R20 not swapped
  if (myConfig.BypassTemperatureSetPoint > 45)
  {
    myConfig.BypassTemperatureSetPoint = 45;
  }
#endif
}

void setup()
{
  //Must be first line of code
  wdt_disable();
  wdt_reset();

  //8 second between watchdogs
  DiyBMSATTiny841::SetWatchdog8sec();

  //Setup IO ports
  DiyBMSATTiny841::ConfigurePorts();

  //More power saving changes
  DiyBMSATTiny841::EnableSerial0();

  DiyBMSATTiny841::DisableSerial1();

  //Check if setup routine needs to be run
  if (!Settings::ReadConfigFromEEPROM((uint8_t *)&myConfig, sizeof(myConfig), EEPROM_CONFIG_ADDRESS))
  {
    DefaultConfig();
    //No need to save here as the default config will load every time if the CRC is wrong
    //Settings::WriteConfigToEEPROM((uint8_t *)&myConfig, sizeof(myConfig), EEPROM_CONFIG_ADDRESS);
  }

  ValidateConfiguration();

  DiyBMSATTiny841::double_tap_green_led();

#if defined(DIYBMSMODULEVERSION) && DIYBMSMODULEVERSION < 430
  DiyBMSATTiny841::double_tap_blue_led();
#endif

  //The PID can vary between 0 and 100
  myPID.setOutputRange(0, 100);

  //Set up data handler
  Serial.begin(COMMS_BAUD_RATE, SERIAL_8N1);

  myPacketSerial.begin(&Serial, &onPacketReceived, sizeof(PacketStruct), SerialPacketReceiveBuffer, sizeof(SerialPacketReceiveBuffer));
}

void StopBalance()
{
  PP.WeAreInBypass = false;
  PP.bypassCountDown = 0;
  PP.bypassHasJustFinished = 0;
  PP.pwmrunning = false;
  PP.PWMValue = 0;
  PP.SettingsHaveChanged = false;

  DiyBMSATTiny841::StopTimer2();
  DiyBMSATTiny841::DumpLoadOff();
}

void loop()
{
  //This loop runs around 3 times per second when the module is in bypass
  wdt_reset();

  //if (bypassHasJustFinished>0)  {    DiyBMSATTiny841::BlueLedOn();  }else {    DiyBMSATTiny841::BlueLedOff();  }
  //if (hztiming) {  DiyBMSATTiny841::SparePinOn();} else {  DiyBMSATTiny841::SparePinOff();}hztiming=!hztiming;

  if (PP.identifyModule > 0)
  {
    DiyBMSATTiny841::GreenLedOn();
#if defined(DIYBMSMODULEVERSION) && DIYBMSMODULEVERSION < 430
    DiyBMSATTiny841::BlueLedOn();
#endif
    PP.identifyModule--;

    if (PP.identifyModule == 0)
    {
      DiyBMSATTiny841::GreenLedOff();
#if defined(DIYBMSMODULEVERSION) && DIYBMSMODULEVERSION < 430
      DiyBMSATTiny841::BlueLedOff();
#endif
    }
  }

  if (PP.SettingsHaveChanged)
  {
    //The configuration has just been modified so stop balancing if we are and reset our status
    StopBalance();
  }

  if (!PP.WeAreInBypass && PP.bypassHasJustFinished == 0)
  {
    //hardware.BlueLedOff();

    //Go to SLEEP, we are not in bypass anymore

    //Switch of TX - save power
    DiyBMSATTiny841::DisableSerial0TX();

    //Wake up on Serial port RX
    DiyBMSATTiny841::EnableStartFrameDetection();

    //Program stops here until woken by watchdog or Serial port ISR
    DiyBMSATTiny841::Sleep();

    //hardware.BlueLedOn();
  }

  //We are awake....

  if (wdt_triggered)
  {
#if defined(DIYBMSMODULEVERSION) && DIYBMSMODULEVERSION < 430
    //Flash blue LED twice after a watchdog wake up
    DiyBMSATTiny841::double_tap_blue_led();
#else
    //Flash green LED twice after a watchdog wake up
    DiyBMSATTiny841::double_tap_green_led();
#endif

    //Setup IO ports
    //DiyBMSATTiny841::ConfigurePorts();

    //If we have just woken up, we shouldn't be in balance
    //safety check that we are not
    StopBalance();
  }

  //We always take a voltage and temperature reading on every loop cycle to check if we need to go into bypass
  //this is also triggered by the watchdog should comms fail or the module is running standalone

  //Disable the PWM/load during voltage readings
  if (PP.bypassCountDown > 0)
  {
    if (PP.pwmrunning)
    {
      DiyBMSATTiny841::DisableTOCPMCOE();
    }
    else
    {
      DiyBMSATTiny841::DumpLoadOff();
    }
  }

  DiyBMSATTiny841::ReferenceVoltageOn();

  //allow reference voltage to stabalize
  delay(1);

  //Internal temperature
  PP.TakeAnAnalogueReading(ADC_INTERNAL_TEMP);
  //External temperature
  PP.TakeAnAnalogueReading(ADC_EXTERNAL_TEMP);
  //Do voltage reading last to give as much time for voltage to settle
  PP.TakeAnAnalogueReading(ADC_CELL_VOLTAGE);

  DiyBMSATTiny841::ReferenceVoltageOff();

  if (wdt_triggered)
  {
    //We got here because the watchdog (after 8 seconds) went off - we didn't receive a packet of data
    wdt_triggered = false;
  }
  else
  {
    //Loop here processing any packets then go back to sleep

    //NOTE this loop size is dependant on the size of the packet buffer (40 bytes)
    //     too small a loop will prevent anything being processed as we go back to Sleep
    //     before packet is received correctly
    for (size_t i = 0; i < 20; i++)
    {
      // Call update to receive, decode and process incoming packets.
      myPacketSerial.checkInputStream();
      //Allow data to be received in buffer (delay must be AFTER) checkInputStream and before DisableSerial0TX
      delay(1);
    }
  }

  //Switch balance PWM back on if needed
  if (PP.bypassCountDown > 0)
  {
    if (PP.pwmrunning)
    {
      DiyBMSATTiny841::EnableTOCPMCOE();
    }
    else
    {
      DiyBMSATTiny841::DumpLoadOn();
    }
  }

  uint8_t internal_temperature = PP.InternalTemperature() & 0xFF;

  if (internal_temperature > DIYBMS_MODULE_SafetyTemperatureCutoff)
  {
    //Force shut down if temperature is too high
    //although this does run the risk that the voltage on the cell will go high
    //but the BMS controller should shut off the charger in this situation
    StopBalance();
  }

  //Only enter bypass if the board temperature is below safety
  if (PP.BypassCheck() && internal_temperature < DIYBMS_MODULE_SafetyTemperatureCutoff)
  {
    //Our cell voltage is OVER the voltage setpoint limit, start draining cell using bypass resistor

    if (!PP.WeAreInBypass)
    {
      //We have just entered the bypass code
      PP.WeAreInBypass = true;

      //This controls how many cycles of loop() we make before re-checking the situation
      //about every 30 seconds
      PP.bypassCountDown = 200;
      PP.bypassHasJustFinished = 0;

      //Reset PID to defaults
      myPID.clear();
    }
  }

  if (PP.bypassCountDown > 0)
  {
    if (internal_temperature < (myConfig.BypassTemperatureSetPoint - 10))
    {
      //Full power if we are nowhere near the setpoint (more than 10 degrees C away)
      DiyBMSATTiny841::StopTimer2();
      DiyBMSATTiny841::DumpLoadOn();
      PP.PWMValue = 100;
      PP.pwmrunning = false;
    }
    else
    {
      if (!PP.pwmrunning)
      {
        //We have approached the set point, enable PWM
        DiyBMSATTiny841::DumpLoadOff();
        //Start timer2 with zero value
        DiyBMSATTiny841::StartTimer2();
        PP.pwmrunning = true;
        //myPID.clear();
      }

      //Compare the real temperature against max setpoint, we want the PID to keep at this temperature
      PP.PWMValue = myPID.step(myConfig.BypassTemperatureSetPoint, internal_temperature);

      //Scale PWM up to 0-10000
      DiyBMSATTiny841::SetTimer2Value(PP.PWMValue * 100);
    }

    PP.bypassCountDown--;

    if (PP.bypassCountDown == 0)
    {
      //Switch everything off for this cycle
      StopBalance();
      //On the next iteration of loop, don't sleep so we are forced to take another
      //cell voltage reading without the bypass being enabled, and we can then
      //evaluate if we need to stay in bypass mode, we do this a few times
      //as the cell has a tendancy to float back up in voltage once load resistor is removed
      PP.bypassHasJustFinished = 200;
    }
  }

  if (PP.bypassHasJustFinished > 0)
  {
    PP.bypassHasJustFinished--;
  }

  if (PP.bypassCountDown > 0)
  {
    //If we are trying to drain the cell/balance, then we need
    //to wait around a bit here whilst the balancing works
    //otherwise the loop() will start again and switch balance off!
    //wait 500ms before continuing, or we exit when some Serial data arrives
    uint16_t i = 500 / 5;
    while (i > 0 && Serial.available() < 8)
    {
      delay(5);
      i--;
    }
  }
}
