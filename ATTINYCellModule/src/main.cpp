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
#include <Arduino.h>

#if !(F_CPU == 8000000)
#error Processor speed should be 8 Mhz internal
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

DiyBMSATTiny841 hardware;

PacketProcessor PP(&hardware, &myConfig);

volatile bool wdt_triggered = false;

void DefaultConfig()
{

  //Default bank zero
  //myConfig.mybank = 0;

  //About 2.2100 seems about right
  myConfig.Calibration = 2.21000;

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
  //This is the watchdog timer - something went wrong and no activity recieved in a while
  wdt_triggered = true;
  PP.IncrementWatchdogCounter();
}

ISR(ADC_vect)
{
  // when ADC completed, take an interrupt and process result
  PP.ADCReading(hardware.ReadADC());
}

void onPacketReceived()
{
  //if (len > 0)
  //{

  //A data packet has just arrived, process it and forward the results to the next module
  if (PP.onPacketReceived((PacketStruct *)SerialPacketReceiveBuffer))
  {
    //Only light green if packet is good
    hardware.GreenLedOn();
  }

  if (PP.pwmrunning)
  {
    //Disable the PWM during Serial transmission to avoid CRC errors
    hardware.DisableTOCPMCOE();
  }

  hardware.EnableSerial0TX();

  //Wake up the connected cell module from sleep, send a framingmarker
  //byte which the receiver will ignore
  //myPacketSerial.sendStartFrame();
  Serial.write((uint8_t)0x00);
  //Let connected module wake up
  hardware.FlushSerial0();
  //delay(1);

  //Send the packet (fixed length!) (even if it was invalid so controller can count crc errors)
  myPacketSerial.sendBuffer(SerialPacketReceiveBuffer);

  //DEBUG: Are there any known issues with Serial Flush causing a CPU to hang?
  hardware.FlushSerial0();

  //Replace flush with a simple delay - we have 35+ bytes to transmit at 2400 baud + COBS encoding
  //delay(10);

  //At 2400bits per second, = 300 bytes per second = 1000ms/300bytes/sec= 3ms per byte

  hardware.DisableSerial0TX();
  //}
  if (PP.pwmrunning)
  {
    hardware.EnableTOCPMCOE();
  }

  hardware.GreenLedOff();
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
  hardware.SetWatchdog8sec();

  //Setup IO ports
  hardware.ConfigurePorts();

  //More power saving changes
  hardware.EnableSerial0();

  hardware.DisableSerial1();

  //Check if setup routine needs to be run
  if (!Settings::ReadConfigFromEEPROM((uint8_t *)&myConfig, sizeof(myConfig), EEPROM_CONFIG_ADDRESS))
  {
    DefaultConfig();
    //No need to save here as the default config will load every time if the CRC is wrong
    //Settings::WriteConfigToEEPROM((uint8_t *)&myConfig, sizeof(myConfig), EEPROM_CONFIG_ADDRESS);
  }

  ValidateConfiguration();

  hardware.double_tap_green_led();

#if defined(DIYBMSMODULEVERSION) && DIYBMSMODULEVERSION < 430
  hardware.double_tap_blue_led();
#endif

  //The PID can vary between 0 and 100
  myPID.setOutputRange(0, 100);

  //Set up data handler
  Serial.begin(COMMS_BAUD_RATE, SERIAL_8N1);

  myPacketSerial.begin(&Serial, &onPacketReceived, sizeof(PacketStruct), SerialPacketReceiveBuffer, sizeof(SerialPacketReceiveBuffer));

  //myPacketSerial.setStream(&Serial);
  //myPacketSerial.setPacketHandler(&onPacketReceived);
}

void StopBalance()
{
  PP.WeAreInBypass = false;
  PP.bypassCountDown = 0;
  PP.bypassHasJustFinished = 0;
  PP.pwmrunning = false;
  PP.PWMValue = 0;
  PP.SettingsHaveChanged = false;

  hardware.StopTimer2();
  hardware.DumpLoadOff();
}

void loop()
{
  //This loop runs around 3 times per second when the module is in bypass
  wdt_reset();

  //if (bypassHasJustFinished>0)  {    hardware.BlueLedOn();  }else {    hardware.BlueLedOff();  }
  //if (hztiming) {  hardware.SparePinOn();} else {  hardware.SparePinOff();}hztiming=!hztiming;

  if (PP.identifyModule > 0)
  {
    hardware.GreenLedOn();
    PP.identifyModule--;

    if (PP.identifyModule == 0)
    {
      hardware.GreenLedOff();
    }
  }

  if (PP.SettingsHaveChanged)
  {
    //The configuration has just been modified so stop balancing if we are and reset our status
    StopBalance();
  }

  //#ifndef DIYBMS_DEBUG
  if (!PP.WeAreInBypass && PP.bypassHasJustFinished == 0)
  {
    //We don't sleep if we are in bypass mode or just after completing bypass
    hardware.EnableStartFrameDetection();

    //Program stops here until woken by watchdog or pin change interrupt
    hardware.Sleep();
  }
  //#endif

  //We are awake....

  if (wdt_triggered)
  {
#if defined(DIYBMSMODULEVERSION) && DIYBMSMODULEVERSION < 430
    //Flash blue LED twice after a watchdog wake up
    hardware.double_tap_blue_led();
#else
    //Flash green LED twice after a watchdog wake up
    hardware.double_tap_green_led();
#endif

    //Setup IO ports
    hardware.ConfigurePorts();

    //More power saving changes
    hardware.EnableSerial0();

    //If we have just woken up, we shouldn't be in balance
    //safety check that we are not
    StopBalance();
  }

  //We always take a voltage and temperature reading on every loop cycle to check if we need to go into bypass
  //this is also triggered by the watchdog should comms fail or the module is running standalone

  if (PP.bypassCountDown > 0)
  {
    if (PP.pwmrunning)
    {
      //Disable the PWM during voltage readings
      hardware.DisableTOCPMCOE();
    }
    else
    {
      hardware.DumpLoadOff();
    }
  }

  hardware.ReferenceVoltageOn();

  //allow reference voltage to stabalize
  delay(4);

  PP.TakeAnAnalogueReading(ADC_CELL_VOLTAGE);
  //Internal temperature
  PP.TakeAnAnalogueReading(ADC_INTERNAL_TEMP);
  //External temperature
  PP.TakeAnAnalogueReading(ADC_EXTERNAL_TEMP);

  hardware.ReferenceVoltageOff();

  //Switch balance back on if needed
  if (PP.bypassCountDown > 0)
  {
    if (PP.pwmrunning)
    {
      hardware.EnableTOCPMCOE();
    }
    else
    {
      hardware.DumpLoadOn();
    }
  }

  uint8_t temp = PP.InternalTemperature() & 0xFF;

  if (temp > DIYBMS_MODULE_SafetyTemperatureCutoff)
  {
    //Force shut down if temperature is too high
    //although this does run the risk that the voltage on the cell will go high
    //but the BMS controller should shut off the charger in this situation
    StopBalance();
  }

  //Only enter bypass if the board temperature is below safety
  if (PP.BypassCheck() && temp < DIYBMS_MODULE_SafetyTemperatureCutoff)
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
    if (temp < (myConfig.BypassTemperatureSetPoint - 10))
    {
      //Full power if we are nowhere near the setpoint (more than 10 degrees C away)
      hardware.StopTimer2();
      hardware.DumpLoadOn();
      PP.PWMValue = 100;
      PP.pwmrunning = false;
    }
    else
    {
      if (!PP.pwmrunning)
      {
        //We have approached the set point, enable PWM
        hardware.DumpLoadOff();
        //Start timer2 with zero value
        hardware.StartTimer2();
        PP.pwmrunning = true;
        //myPID.clear();
      }

      //Compare the real temperature against max setpoint, we want the PID to keep at this temperature
      PP.PWMValue = myPID.step(myConfig.BypassTemperatureSetPoint, temp);

      //Scale PWM up to 0-10000
      hardware.SetTimer2Value(PP.PWMValue * 100);
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

  if (wdt_triggered)
  {
    //We got here because the watchdog (after 8 seconds) went off - we didn't receive a packet of data
    wdt_triggered = false;
  }
  else
  {
    //Loop here processing any packets then go back to sleep

    //NOTE this loop size is dependant on the size of the packet buffer (34 bytes)
    //     too small a loop will prevent anything being processed as we go back to Sleep
    //     before packet is received correctly
    for (size_t i = 0; i < 15; i++)
    {
      //Allow data to be received in buffer
      delay(10);

      // Call update to receive, decode and process incoming packets.
      myPacketSerial.checkInputStream();
    }
  }

  if (PP.bypassHasJustFinished > 0)
  {
    PP.bypassHasJustFinished--;
  }
}
