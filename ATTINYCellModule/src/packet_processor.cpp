/*
____  ____  _  _  ____  __  __  ___
(  _ \(_  _)( \/ )(  _ \(  \/  )/ __)
)(_) )_)(_  \  /  ) _ < )    ( \__ \
(____/(____) (__) (____/(_/\/\_)(___/

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

#include "packet_processor.h"

// Enable this for debug/testing a single module will pretend to be an entire bank of 16 modules
// you are likely to get OOS errors when these are running in a string as the timings will be wrong
//#define FAKE_16_CELLS

// Returns TRUE if the internal thermistor is hotter than the required setting (or over max limit)
bool PacketProcessor::BypassOverheatCheck()
{
  int16_t temp = InternalTemperature();
  return (temp > _config->BypassTemperatureSetPoint || temp > DIYBMS_MODULE_SafetyTemperatureCutoff);
}

// Returns an integer byte indicating the internal thermistor temperature in degrees C
// uses basic B Coefficient Steinhart calculaton to give rough approximation in temperature
int16_t PacketProcessor::InternalTemperature()
{
  return Steinhart::ThermistorToCelcius(INT_BCOEFFICIENT, raw_adc_onboard_temperature, MAXIUMUM_ATTINY_ADC_SCALE);
}

// Returns TRUE if the cell voltage is greater than the required setting
bool PacketProcessor::BypassCheck()
{
  return (CellVoltage() > _config->BypassThresholdmV);
}

// Records an ADC reading after the interrupt has finished
void PacketProcessor::ADCReading(uint16_t value)
{
  switch (adcmode)
  {
  case ADC_CELL_VOLTAGE:
  {

#if (SAMPLEAVERAGING == 1)
    raw_adc_voltage = value;
#else
    // Multiple samples - keep history for sample averaging

    // subtract the last reading:
    total = total - readings[readIndex];
    // read from the sensor:
    readings[readIndex] = value;
    // add the reading to the total:
    total = total + value;
    // advance to the next position in the array:
    readIndex++;

    // if we're at the end of the array...
    if (readIndex >= SAMPLEAVERAGING)
    {
      // ...wrap around to the beginning:
      readIndex = 0;
    }

    // JB: Keep Full total and divide later in float variables
    raw_adc_voltage = total;

#endif

    break;
  }
  case ADC_INTERNAL_TEMP:
  {
#if (defined(DIYBMSMODULEVERSION) && (DIYBMSMODULEVERSION == 420 && defined(SWAPR19R20)))
    // R19 and R20 swapped on V4.2 board, invert the thermistor reading
    // Reverted back to 1000 base value to fix issue https://github.com/stuartpittaway/diyBMSv4Code/issues/95
    raw_adc_onboard_temperature = 1000 - value;
#elif (defined(DIYBMSMODULEVERSION) && (DIYBMSMODULEVERSION == 430 && defined(SWAPR19R20)))
    // R19 and R20 swapped on V4.3 board (never publically released), invert the thermistor reading
    raw_adc_onboard_temperature = 1000 - value;
#else
    raw_adc_onboard_temperature = value;
#endif
    break;
  }
  case ADC_EXTERNAL_TEMP:
  {
    raw_adc_external_temperature = value;
    break;
  }
  }
}

PacketProcessor::PacketProcessor()
{
#if (SAMPLEAVERAGING > 1)
  for (int thisReading = 0; thisReading < SAMPLEAVERAGING; thisReading++)
  {
    readings[thisReading] = 0;
  }
#endif
};

// Start an ADC reading via Interrupt
void PacketProcessor::TakeAnAnalogueReading(uint8_t _mode)
{
  adcmode = _mode;

  switch (_mode)
  {
  case ADC_CELL_VOLTAGE:
  {
    diyBMSHAL::SelectCellVoltageChannel();
    break;
  }
  case ADC_INTERNAL_TEMP:
  {
    diyBMSHAL::SelectInternalTemperatureChannel();
    break;
  }
  case ADC_EXTERNAL_TEMP:
  {
    diyBMSHAL::SelectExternalTemperatureChannel();
    break;
  }
  default:
    // Avoid taking a reading if we get to here
    return;
  }

#if defined(__AVR_ATtinyx24__)
  // For tiny2 devices we don't use the interrupt method as there is no benefit
  // ATTiny841 ADC readings are less noisy when using sleep mode and interrupts
  ADCReading(diyBMSHAL::BeginADCReading(_mode));
#else
  diyBMSHAL::BeginADCReading(_mode);
#endif
}

// Run when a new packet is received over serial
bool PacketProcessor::onPacketReceived(PacketStruct *receivebuffer)
{
  bool commandProcessed = false;
  // Temporary debug counter, see where packets get lost
  PacketReceivedCounter++;

  // Calculate the CRC and compare to received
  uint16_t validateCRC = CRC16::CalculateArray((unsigned char *)receivebuffer, sizeof(PacketStruct) - 2);

  if (validateCRC == receivebuffer->crc)
  {

#if defined(FAKE_16_CELLS)
    uint8_t start = receivebuffer->hops;
    uint8_t end = start + 16;
    for (size_t i = start; i < end; i++)
    {
#endif

      // TODO: We can probably get rid of mymoduleaddress
      mymoduleaddress = receivebuffer->hops;

      bool isPacketForMe = receivebuffer->start_address <= mymoduleaddress && receivebuffer->end_address >= mymoduleaddress;

      // Increment the hops no matter what (on valid CRC)
      receivebuffer->hops++;

      commandProcessed = false;
      // It's a good packet
      if (isPacketForMe)
      {
        commandProcessed = processPacket(receivebuffer);

        if (commandProcessed)
        {
          // Set flag to indicate we processed packet (other modules may also do this)
          receivebuffer->command = receivebuffer->command | B10000000;
        }
      }
#if defined(FAKE_16_CELLS)
    }
#endif
    // Calculate new checksum over whole buffer (as hops as increased)
    receivebuffer->crc = CRC16::CalculateArray((unsigned char *)receivebuffer, sizeof(PacketStruct) - 2);

    // Return false the packet was not for me (but still a valid packet)...
    return commandProcessed;
  }

  // The packet received was not correct, failed CRC check
  badpackets++;
  return false;
}

// Read cell voltage and return millivolt reading (16 bit unsigned)
uint16_t PacketProcessor::CellVoltage()
{
// TODO: Get rid of the need for float variables?

// JB: Divide by number of samples here to get more precision from the averaging.
#if (SAMPLEAVERAGING > 1)
  float v = (((float)raw_adc_voltage / (float)SAMPLEAVERAGING) * (float)MV_PER_ADC) * _config->Calibration;
#else
  float v = (float)((1250L * 65535L) / raw_adc_voltage) * _config->Calibration;
#endif
  return (uint16_t)v;
}

// Process the request in the received packet
// command byte
// RRRR CCCC
// X    = 1 bit indicate if packet processed
// R    = 3 bits reserved not used
// C    = 4 bits command (16 possible commands)
bool PacketProcessor::processPacket(PacketStruct *buffer)
{
  uint8_t moduledata_index = mymoduleaddress % maximum_cell_modules;
  switch (buffer->command & 0x0F)
  {
  case COMMAND::ResetBadPacketCounter:
    badpackets = 0;
    PacketReceivedCounter = 0;
    return true;

  case COMMAND::ReadVoltageAndStatus:
  {
    // Read voltage of VCC
    // Maximum voltage 8191mV
    buffer->moduledata[moduledata_index] = CellVoltage() & 0x1FFF;

    // 3 top bits
    // X = In bypass
    // Y = Bypass over temperature
    // Z = Not used

    if (BypassOverheatCheck())
    {
      // Set bit
      buffer->moduledata[moduledata_index] = buffer->moduledata[moduledata_index] | 0x4000;
    }

    if (IsBypassActive())
    {
      // Set bit
      buffer->moduledata[moduledata_index] = buffer->moduledata[moduledata_index] | 0x8000;
    }

    return true;
  }

  case COMMAND::Timing:
  {
    // Do nothing just accept and pass on the packet
    return true;
  }

  case COMMAND::Identify:
  {
    // identify module
    // For the next 10 received packets - keep the LEDs lit up
    identifyModule = 10;
    return true;
  }

  case COMMAND::ReadTemperature:
  {
    // Return the last known temperature values recorded by the ADC (both internal and external)
    buffer->moduledata[moduledata_index] = TemperatureMeasurement();
    return true;
  }

  case COMMAND::ReadBalancePowerPWM:
  {
    // Read the last PWM value
    // Use WeAreInBypass instead of IsByPassActive() as the later also includes the "settle" time
    buffer->moduledata[moduledata_index] = WeAreInBypass ? PWMSetPoint : 0;
    return true;
  }

  case COMMAND::ReadBadPacketCounter:
  {
    // Report number of bad packets
    buffer->moduledata[moduledata_index] = badpackets;
    return true;
  }

  case COMMAND::ReadSettings:
  {
    // Report settings/configuration, fills whole moduledata buffer up

    FLOATUNION_t myFloat;
    myFloat.number = (float)LOAD_RESISTANCE;
    buffer->moduledata[0] = myFloat.word[0];
    buffer->moduledata[1] = myFloat.word[1];

    myFloat.number = _config->Calibration;
    buffer->moduledata[2] = myFloat.word[0];
    buffer->moduledata[3] = myFloat.word[1];

    myFloat.number = (float)MV_PER_ADC;
    buffer->moduledata[4] = myFloat.word[0];
    buffer->moduledata[5] = myFloat.word[1];

    buffer->moduledata[6] = _config->BypassTemperatureSetPoint;
    buffer->moduledata[7] = _config->BypassThresholdmV;
    buffer->moduledata[8] = INT_BCOEFFICIENT;
    buffer->moduledata[9] = EXT_BCOEFFICIENT;
    buffer->moduledata[10] = DIYBMSMODULEVERSION;

    // Version of firmware (taken automatically from GIT)
    buffer->moduledata[14] = GIT_VERSION_B1;
    buffer->moduledata[15] = GIT_VERSION_B2;
    return true;
  }

  case COMMAND::WriteSettings:
  {
    FLOATUNION_t myFloat;

    myFloat.word[0] = buffer->moduledata[0];
    myFloat.word[1] = buffer->moduledata[1];
    // if (myFloat.number < 0xFFFF)
    //{
    //       _config->LoadResistance = myFloat.number;
    // }

    myFloat.word[0] = buffer->moduledata[2];
    myFloat.word[1] = buffer->moduledata[3];
    if (myFloat.number < 0xFFFF)
    {
      _config->Calibration = myFloat.number;
    }

    // myFloat.word[0] = buffer.moduledata[4];
    // myFloat.word[1] = buffer.moduledata[5];
    // if (myFloat.number < 0xFFFF)
    //{
    //   _config->mVPerADC = (float)MV_PER_ADC;
    // }

    if (buffer->moduledata[6] != 0xFF)
    {
      _config->BypassTemperatureSetPoint = buffer->moduledata[6];

#if defined(DIYBMSMODULEVERSION) && (DIYBMSMODULEVERSION == 420 && !defined(SWAPR19R20))
      // Keep temperature low for modules with R19 and R20 not swapped
      if (_config->BypassTemperatureSetPoint > 45)
      {
        _config->BypassTemperatureSetPoint = 45;
      }
#endif
    }

    if (buffer->moduledata[7] != 0xFFFF)
    {
      _config->BypassThresholdmV = buffer->moduledata[7];
    }
    // if (buffer.moduledata[8] != 0xFFFF)
    //{
    //       _config->Internal_BCoefficient = buffer.moduledata[8];
    // }

    // if (buffer.moduledata[9] != 0xFFFF)
    //{
    //       _config->External_BCoefficient = buffer.moduledata[9];
    // }

    // Save settings
    Settings::WriteConfigToEEPROM((uint8_t *)_config, sizeof(CellModuleConfig), EEPROM_CONFIG_ADDRESS);

    SettingsHaveChanged = true;

    return true;
  }

  case COMMAND::ReadBalanceCurrentCounter:
  {
    buffer->moduledata[moduledata_index] = (uint16_t)MilliAmpHourBalanceCounter;
    return true;
  }

  case COMMAND::ReadPacketReceivedCounter:
  {
    buffer->moduledata[moduledata_index] = PacketReceivedCounter;
    return true;
  }

  case COMMAND::ResetBalanceCurrentCounter:
  {
    MilliAmpHourBalanceCounter = 0;
    return true;
  }
  }

  return false;
}

uint16_t PacketProcessor::TemperatureMeasurement()
{
  return (Steinhart::TemperatureToByte(Steinhart::ThermistorToCelcius(INT_BCOEFFICIENT, raw_adc_onboard_temperature, MAXIUMUM_ATTINY_ADC_SCALE)) << 8) +
         Steinhart::TemperatureToByte(Steinhart::ThermistorToCelcius(EXT_BCOEFFICIENT, raw_adc_external_temperature, MAXIUMUM_ATTINY_ADC_SCALE));
}
