/*
 ____  ____  _  _  ____  __  __  ___    _  _  __
(  _ \(_  _)( \/ )(  _ \(  \/  )/ __)  ( \/ )/. |
 )(_) )_)(_  \  /  ) _ < )    ( \__ \   \  /(_  _)
(____/(____) (__) (____/(_/\/\_)(___/    \/   (_)

DIYBMS V4.0
CELL MODULE FOR ATTINY841

(c)2019 to 2021 Stuart Pittaway

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

ATTINY chip frequency dropped to 2Mhz to comply with datasheet at low voltages (<2V)
Baud rate changed to 5000bits/second from 26 Jan 2021, 5000 chosen due to 2Mhz frequency and ATTINY bad freq regulation
https://trolsoft.ru/en/uart-calc

*/

#define RX_BUFFER_SIZE 64

#include <Arduino.h>

#if !(F_CPU == 2000000)
#error Processor speed should be 2Mhz
#endif

#if !defined(ATTINY_CORE)
#error Expected ATTINYCORE
#endif

#if !defined(BAUD)
#error Expected BAUD define
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

//Interrupt counter
volatile uint8_t InterruptCounter = 0;
volatile uint16_t PulsePeriod = 0;
volatile uint16_t OnPulseCount = 0;
//volatile bool PacketProcessed = false;

void DefaultConfig()
{
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

  DiyBMSATTiny841::GreenLedOff();

  //PacketProcessed = true;
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
FastPID myPID(5.0, 1.0, 0.1, 3, 8, false);

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

void StopBalance()
{
  PP.WeAreInBypass = false;
  PP.bypassCountDown = 0;
  PP.bypassHasJustFinished = 0;
  PP.PWMSetPoint = 0;
  PP.SettingsHaveChanged = false;

  OnPulseCount = 0;
  PulsePeriod = 0;

  DiyBMSATTiny841::StopTimer1();
  DiyBMSATTiny841::DumpLoadOff();
}

void setup()
{
  //Must be first line of code
  wdt_disable();
  wdt_reset();

  //Boot up will be in 1Mhz CKDIV8 mode, swap to /4 to change speed to 2Mhz
  //CCP – Configuration Change Protection Register
  CCP = 0xD8;
  //CLKPR – Clock Prescale Register  
  CLKPR = _BV(CLKPS1);

  //below 2Mhz is required for running ATTINY at low voltages (less than 2V)

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

  //The PID can vary between 0 and 255 (1 byte)
  myPID.setOutputRange(0, 255);

  StopBalance();

  //Set up data handler
  Serial.begin(BAUD, SERIAL_8N1);

  myPacketSerial.begin(&Serial, &onPacketReceived, sizeof(PacketStruct), SerialPacketReceiveBuffer, sizeof(SerialPacketReceiveBuffer));
}

ISR(TIMER1_COMPA_vect)
{
  // This ISR is called every 1 millisecond when TIMER1 is enabled

  // when v=1, the duration between on and off is 2.019ms (1.0095ms per interrupt) - on for 2.019ms off for 255.7ms, 0.7844% duty
  // when v=128 (50%), the duration between on and off is 130.8ms (1.0218ms per interrupt) - on for 130.8ms, off for 127.4ms 50.67% duty
  // when v=192 (75%), the duration between on and off is 195.6ms (1.0187ms per interrupt) - on for 62.63ms, off for 30.83ms, 75.74% duty
  InterruptCounter++;
  PulsePeriod++;
  //Reset at top
  if (InterruptCounter == 255)
  {
    InterruptCounter = 0;
  }

  //Switch the load on if the counter is below the SETPOINT
  if (InterruptCounter <= PP.PWMSetPoint)
  {
    //Enable the pin
    //DiyBMSATTiny841::SparePinOn();
    DiyBMSATTiny841::DumpLoadOn();

    //Count the number of "on" periods, so we can calculate the amount of energy consumed
    //over time
    OnPulseCount++;
  }
  else
  {
    //Off
    //DiyBMSATTiny841::SparePinOff();
    DiyBMSATTiny841::DumpLoadOff();
  }

  if (PulsePeriod == 1000)
  {

    // Floats are not good on ATTINY/8bit controllers, need to look at moving to fixed decimal/integer calculations

    //CellVoltage is in millivolts, so we get milli-amp current reading out.
    //For example 4000mV / 4.4R = 909.0909mA
    float CurrentmA = ((float)PP.CellVoltage() / (float)LOAD_RESISTANCE);

    //Scale down to the number of "ON" pulses
    //Assuming 100% on, 909.0909mA * (1000/1000) * 0.0002777 = 0.2525 milli amp hours (or 0.000252 amp hours)
    float milliAmpHours = (CurrentmA * ((float)OnPulseCount / (float)1000.0)) * (1.0 / 3600.0);

    //0.2525 * 3600 / 1000 = 0.909Ah

    //Keep running total
    PP.MilliAmpHourBalanceCounter += milliAmpHours;

    OnPulseCount = 0;
    PulsePeriod = 0;
  }
}

inline void identifyModule()
{
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
}

void loop()
{
  //This loop runs around 3 times per second when the module is in bypass
  wdt_reset();
  identifyModule();

  if (PP.SettingsHaveChanged)
  {
    //The configuration has just been modified so stop balancing if we are and reset our status
    StopBalance();
  }

  if (!PP.WeAreInBypass && PP.bypassHasJustFinished == 0 && Serial.available() == 0)
  {
    //Go to SLEEP, we are not in bypass anymore and no serial data waiting...

    //Reset PID to defaults, in case we want to start balancing
    myPID.clear();

    //Switch of TX - save power
    DiyBMSATTiny841::DisableSerial0TX();

    //Wake up on Serial port RX
    DiyBMSATTiny841::EnableStartFrameDetection();

    //Program stops here until woken by watchdog or Serial port ISR
    DiyBMSATTiny841::Sleep();
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

    //If we have just woken up, we shouldn't be in balance safety check that we are not
    StopBalance();
  }

  //We always take a voltage and temperature reading on every loop cycle to check if we need to go into bypass
  //this is also triggered by the watchdog should comms fail or the module is running standalone
  DiyBMSATTiny841::ReferenceVoltageOn();

  //allow reference voltage to stabilize
  //delay(1);

  //Internal temperature
  PP.TakeAnAnalogueReading(ADC_INTERNAL_TEMP);

  //Disable the PWM/load during voltage readings
  //if (PP.bypassCountDown > 0) { DiyBMSATTiny841::PausePWM(); }

  //Only take these readings when we are NOT in bypass....
  //this causes the voltage and temperature to "freeze" during bypass cycles
  if (PP.bypassCountDown == 0)
  {
//Just for debug purposes, shows when voltage is read
//#if defined(DIYBMSMODULEVERSION) && DIYBMSMODULEVERSION < 430
//    DiyBMSATTiny841::BlueLedOn();
//#endif    

    //External temperature
    PP.TakeAnAnalogueReading(ADC_EXTERNAL_TEMP);

    //Do voltage reading last to give as much time for voltage to settle
    PP.TakeAnAnalogueReading(ADC_CELL_VOLTAGE);

//#if defined(DIYBMSMODULEVERSION) && DIYBMSMODULEVERSION < 430
//    DiyBMSATTiny841::BlueLedOff();
//#endif    
  }

  //Switch balance PWM back on if needed
  //if (PP.bypassCountDown > 0) { DiyBMSATTiny841::ResumePWM(); }

  //Switch reference off if we are not in bypass (otherwise leave on)
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
    //PacketProcessed = false;
    for (size_t i = 0; i < 200; i++)
    {
      // Call update to receive, decode and process incoming packets.
      myPacketSerial.checkInputStream();

      //Abort loop if we just processed a packet
      //if (PacketProcessed) break;

      //Allow data to be received in buffer (delay must be AFTER) checkInputStream and before DisableSerial0TX
      delay(1);
    }
  }

  //We should probably check for invalid InternalTemperature ranges here and throw error (shorted or unconnecter thermistor for example)
  int16_t internal_temperature = PP.InternalTemperature();

  if (internal_temperature > DIYBMS_MODULE_SafetyTemperatureCutoff || internal_temperature > (myConfig.BypassTemperatureSetPoint + 10))
  {
    //Force shut down if temperature is too high although this does run the risk that the voltage on the cell will go high
    //but the BMS controller should shut off the charger in this situation
    myPID.clear();
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
      PP.bypassCountDown = 50;
      PP.bypassHasJustFinished = 0;

      //Start PWM
      DiyBMSATTiny841::StartTimer1();
      PP.PWMSetPoint = 0;
    }
  }

  if (PP.bypassCountDown > 0)
  {
    if (internal_temperature < (myConfig.BypassTemperatureSetPoint - 6))
    {
      //Full power if we are no where near the setpoint (more than 6 degrees C away)
      PP.PWMSetPoint = 0xFF;
    }
    else
    {
      //Compare the real temperature against max setpoint, we want the PID to keep at this temperature
      PP.PWMSetPoint = myPID.step(myConfig.BypassTemperatureSetPoint, internal_temperature);

      if (myPID.err())
      {
        //Clear the error and stop balancing
        myPID.clear();
        StopBalance();
        //Just for debug...
#if defined(DIYBMSMODULEVERSION) && DIYBMSMODULEVERSION < 430
        DiyBMSATTiny841::BlueLedOn();
#endif
      }
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

/*
  if (PP.bypassCountDown > 0)
  {
    //If we are trying to drain the cell/balance, then we need
    //to wait around a bit here whilst the balancing works
    //otherwise the loop() will start again and switch balance off!
    //wait 500ms before continuing, or we exit when some Serial data arrives
    uint16_t i = 500 / 5;
    while (i > 0)
    {
      // Call update to receive, decode and process incoming packets (if any)
      myPacketSerial.checkInputStream();
      delay(5);
      i--;
    }
  }
*/  
}
