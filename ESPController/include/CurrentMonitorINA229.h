/*
 ____  ____  _  _  ____  __  __  ___    _  _  __
(  _ \(_  _)( \/ )(  _ \(  \/  )/ __)  ( \/ )/. |
 )(_) )_)(_  \  /  ) _ < )    ( \__ \   \  /(_  _)
(____/(____) (__) (____/(_/\/\_)(___/    \/   (_)

DIYBMS V4.0
INA229 CURRENT/ENERGY MONITOR CHIP (SPI INTERFACE)

(c)2023 Stuart Pittaway

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

** COMMERCIAL USE AND RESALE PROHIBITED **
*/

/*
DATASHEET
INA229-Q1 AEC-Q100, 85-V, 20-Bit, Ultra-Precise Power/Energy/Charge Monitor With SPI Interface
https://www.ti.com/lit/ds/symlink/ina229-q1.pdf?ts=1599057970767
*/

#define CONFIG_DISABLE_HAL_LOCKS 1

#pragma once

#include <Arduino.h>
#include <SPI.h>

#ifndef CURRENTMONITORINA229_H_
#define CURRENTMONITORINA229_H_

class CurrentMonitorINA229
{
    // This structure is held in EEPROM, it has the same register/values
    // as the INA229 chip and is used to set the INA229 chip to the correct parameters on power up
    // On initial power up (or EEPROM clear) these parameters are read from the INA229 chip
    // to provide defaults.  Some values are overridden in code (like ADC_CONFIG and CONFIG)
    // to configure to our prescribed needs.
    struct eeprom_regs
    {
        uint16_t R_CONFIG;
        uint16_t R_ADC_CONFIG;
        uint16_t R_SHUNT_CAL;    // Shunt Calibration
        uint16_t R_SHUNT_TEMPCO; // Shunt Temperature Coefficient
        uint16_t R_DIAG_ALRT;
        uint16_t R_SOVL;
        uint16_t R_SUVL;
        uint16_t R_BOVL;
        uint16_t R_BUVL;
        uint16_t R_TEMP_LIMIT;
        uint16_t R_PWR_LIMIT;

        // LSB step size for the CURRENT register where the current in Amperes is stored
        float CURRENT_LSB;
        // Resistance of SHUNT in OHMS
        float RSHUNT;

        uint16_t shunt_max_current;
        uint16_t shunt_millivolt;
        uint16_t batterycapacity_amphour;
        float fully_charged_voltage;
        float tail_current_amps;
        float charge_efficiency_factor;
    };

    enum INA_REGISTER : uint8_t
    {
        CONFIG = 0,
        ADC_CONFIG = 1,
        // Shunt Calibration
        SHUNT_CAL = 2,
        // Shunt Temperature Coefficient
        SHUNT_TEMPCO = 3,
        // Shunt Voltage Measurement 24bit
        VSHUNT = 4,
        // Bus Voltage Measurement 24bit
        VBUS = 5,
        DIETEMP = 6,
        // Current Result 24bit
        CURRENT = 7,
        // Power Result 24bit
        POWER = 8,
        // Energy Result 40bit
        ENERGY = 9,
        // Charge Result 40bit
        CHARGE = 0x0A,
        DIAG_ALRT = 0x0b,
        // Shunt Overvoltage Threshold
        SOVL = 0x0c,
        // Shunt Undervoltage Threshold
        SUVL = 0x0d,
        // Bus Overvoltage Threshold
        BOVL = 0x0e,
        // Bus Undervoltage Threshold
        BUVL = 0x0f,
        // Temperature Over-Limit Threshold
        TEMP_LIMIT = 0x10,
        // Power Over-Limit Threshold
        PWR_LIMIT = 0x11,
        // Manufacturer ID
        MANUFACTURER_ID = 0x3E,
        // Device ID
        DEVICE_ID = 0x3F

    };

    enum DIAG_ALRT_FIELD : uint16_t
    {
        ALATCH = 15,
        CNVR = 14,
        SLOWALERT = 13,
        APOL = 12,
        ENERGYOF = 11,
        CHARGEOF = 10,
        MATHOF = 9,
        RESERVED = 8,
        TMPOL = 7,
        SHNTOL = 6,
        SHNTUL = 5,
        BUSOL = 4,
        BUSUL = 3,
        POL = 2,
        CNVRF = 1,
        MEMSTAT = 0
    };

public:
    CurrentMonitorINA229()
    {
        INA229Installed = false;
        SPI_Ptr = NULL;
        chipselectpin = 0;

        milliamphour_out_lifetime = 0;
        milliamphour_in_lifetime = 0;

        daily_milliamphour_out = 0;
        daily_milliamphour_in = 0;

        milliamphour_out = 0;
        milliamphour_in = 0;

        milliamphour_out_offset = 0;
        milliamphour_in_offset = 0;

        // Clear structure
        memset(&registers, 0, sizeof(eeprom_regs));

        // Conversion times for voltage, current  temperature
        // 128 times sample averaging
        // MODE  = 1111 = Continuous bus, shunt voltage and temperature
        // VBUSCT= 111 = 6h = 4120µs BUS VOLT
        //  VSHCT= 111 = 6h = 4120µs CURRENT
        //   VTCT= 010 = 2h =  150µs TEMPERATURE
        //    AVG= 100 = 4h = 128 ADC sample averaging count
        // B1111 111 111 010 100 = 0xFFD4
        registers.R_ADC_CONFIG = 0xFFD4;

        registers.R_CONFIG = _BV(4); // ADCRANGE = 40.96mV scale

        // Defaults for battery capacity/voltages
        registers.batterycapacity_amphour = 280;
        registers.fully_charged_voltage = 3.50 * 16;
        registers.tail_current_amps = 20;
        registers.charge_efficiency_factor = 99.5;

        // Default 150A shunt @ 50mV scale
        registers.shunt_max_current = 150;
        registers.shunt_millivolt = 50;

        // SLOWALERT = Wait for full sample averaging time before triggering alert (about 1.5 seconds)
        registers.R_DIAG_ALRT = bit(DIAG_ALRT_FIELD::SLOWALERT);

        // This is not enabled by default
        // The 16 bit register provides a resolution of 1ppm/°C/LSB
        registers.R_SHUNT_TEMPCO = 15;

        // Use the defaults from the INA229 chip as a starting point
        registers.R_SOVL = 0x7FFF;
        registers.R_SUVL = 0x8000;
        // 85volt max
        registers.R_BOVL = 0x6A40;
        registers.R_BUVL = 0;
        registers.R_TEMP_LIMIT = 0x2800; // 80 degrees C

        CalculateLSB();

        // Default Power limit = 5kW
        registers.R_PWR_LIMIT = (uint16_t)((5000.0 / registers.CURRENT_LSB / 3.2) / 256.0); // 5kW
    }

    bool Available()
    {
        return INA229Installed;
    }

    bool Initialise(SPIClass *SPI, uint8_t cs_pin);

    bool Configure(uint16_t shuntmv,
                   uint16_t shuntmaxcur,
                   uint16_t batterycapacity,
                   uint16_t fullchargevolt,
                   uint16_t tailcurrent,
                   uint16_t chargeefficiency,
                   uint16_t shuntcal,
                   int16_t temperaturelimit,
                   int16_t overvoltagelimit,
                   int16_t undervoltagelimit,
                   int32_t overcurrentlimit,
                   int32_t undercurrentlimit,
                   uint32_t overpowerlimit,
                   uint16_t shunttempcoefficient);

    void GuessSOC();
    void TakeReadings();

    float BusOverVolt;
    float BusUnderVolt;
    float ShuntOverCurrentLimit;
    float ShuntUnderCurrentLimit;
    float PowerLimit;
    uint16_t ShuntTemperatureCoefficient;

    uint32_t calc_milliamphour_out() { return milliamphour_out - milliamphour_out_offset; }
    uint32_t calc_milliamphour_in() { return milliamphour_in - milliamphour_in_offset; }
    uint32_t calc_daily_milliamphour_out() { return daily_milliamphour_out; }
    uint32_t calc_daily_milliamphour_in() { return daily_milliamphour_in; }
    float charge_efficiency_factor() { return registers.charge_efficiency_factor; }
    float state_of_charge() { return SOC / 100.0; }
    float calc_voltage() { return voltage; }
    float calc_current() { return current; }
    float calc_power() { return power; }
    uint16_t calc_shuntcalibration() { return registers.R_SHUNT_CAL; }
    int16_t calc_temperature() { return (int16_t)temperature; }
    uint16_t calc_shunttempcoefficient() { return registers.R_SHUNT_TEMPCO; }
    float calc_tailcurrentamps() { return registers.tail_current_amps; }
    float calc_fullychargedvoltage() { return registers.fully_charged_voltage; }
    float calc_shuntresistance() { return 1000 * registers.RSHUNT; }

    uint16_t calc_shuntmillivolt() { return registers.shunt_millivolt; }
    uint16_t calc_shuntmaxcurrent() { return registers.shunt_max_current; }
    uint16_t calc_batterycapacityAh() { return registers.batterycapacity_amphour; }

    int16_t calc_temperaturelimit()
    {
        // Case unsigned to int16 to cope with negative temperatures
        return (int16_t)registers.R_TEMP_LIMIT * (float)0.0078125;
    }
    float calc_overpowerlimit()
    {
        return registers.R_PWR_LIMIT * 256 * 3.2F * registers.CURRENT_LSB;
    }
    float calc_overvoltagelimit() { return (float)registers.R_BOVL * 0.003125F; }
    float calc_undervoltagelimit() { return (float)registers.R_BUVL * 0.003125F; }
    float calc_overcurrentlimit() { return ((float)registers.R_SOVL / 1000 * 1.25) / (full_scale_adc * registers.shunt_max_current); }
    float calc_undercurrentlimit() { return ((float)registers.R_SUVL / 1000 * 1.25) / (full_scale_adc * registers.shunt_max_current); }

private:
    uint16_t SOC = 0;
    float voltage = 0;
    float current = 0;
    float power = 0;
    float temperature = 0;

    const float full_scale_adc = 40.96;
    // const float CoulombsToAmpHours = 1.0 / 3600.0;
    const float CoulombsToMilliAmpHours = 1.0 / 3.6;

    uint8_t max_soc_reset_counter = 0;
    uint8_t soc_reset_counter = 0;
    int32_t last_charge_coulombs = 0;

    // Pointer to SPI object class  NOTE: MUTEX OVER SPI PORT MUST BE HANDLED EXTERNALLY TO THIS CLASS
    SPIClass *SPI_Ptr;
    // True is chip is installed
    bool INA229Installed;
    // Chip select pin
    uint8_t chipselectpin;
    // Settings used for SPI comms (10Mhz, MSB, Mode 1)
    SPISettings _spisettings = SPISettings(10000000, MSBFIRST, SPI_MODE1);

    eeprom_regs registers;

    uint32_t milliamphour_out_lifetime;
    uint32_t milliamphour_in_lifetime;

    uint32_t daily_milliamphour_out;
    uint32_t daily_milliamphour_in;

    uint32_t milliamphour_out;
    uint32_t milliamphour_in;

    uint32_t milliamphour_out_offset;
    uint32_t milliamphour_in_offset;

    volatile uint16_t diag_alrt_value = 0;

    uint8_t readRegisterValue(INA_REGISTER r);
    uint8_t writeRegisterValue(INA_REGISTER r);
    void CalculateLSB();
    void SetSOC(uint16_t value);
    uint16_t read16bits(INA_REGISTER r);
    uint16_t write16bits(INA_REGISTER r, uint16_t value);
    void SetINA229Registers();
    float BusVoltage();
    float Energy();
    float ShuntVoltage();

    int32_t ChargeInCoulombsAsInt();

    int32_t readInt20(INA_REGISTER r);
    uint32_t readUInt24(INA_REGISTER r);
    uint32_t spi_readUint24(INA_REGISTER r);
    uint64_t spi_readUint40(INA_REGISTER r);
    int64_t spi_readInt40(INA_REGISTER r);

    void CalculateAmpHourCounts();
    uint16_t CalculateSOC();

    // Calculated power output.  Output value in watts. Unsigned representation. Positive value.
    float Power()
    {
        // POWER Power [W] = 3.2 x CURRENT_LSB x POWER
        return (float)spi_readUint24(INA_REGISTER::POWER) * (float)3.2 * registers.CURRENT_LSB;
    }

    // The INA228 device has an internal temperature sensor which can measure die temperature from –40 °C to +125°C.
    float DieTemperature()
    {
        // The accuracy of the temperature sensor is ±2 °C across the operational temperature range. The temperature
        // value is stored inside the DIETEMP register
        // Internal die temperature measurement.
        // Case unsigned to int16 to cope with negative temperatures
        // Two's complement value. Conversion factor: 7.8125 m°C/LSB
        float dietemp = (int16_t)read16bits(INA_REGISTER::DIETEMP);
        return dietemp * (float)0.0078125;
    }

    float TemperatureLimit()
    {
        // Case unsigned to int16 to cope with negative temperatures
        float temp = (int16_t)read16bits(INA_REGISTER::TEMP_LIMIT);
        return temp * (float)0.0078125;
    }

    // Calculated current output in Amperes.
    // In the way this circuit is designed, NEGATIVE current indicates DISCHARGE of the battery
    // POSITIVE current indicates CHARGE of the battery
    float Current()
    {
        // Current. Two's complement value.
        return -(registers.CURRENT_LSB * (float)readInt20(INA_REGISTER::CURRENT));
    }

    void ResetChargeEnergyRegisters()
    {
        // BIT 14
        // RSTACC
        // Resets the contents of accumulation registers ENERGY and CHARGE to 0
        // 0h = Normal Operation
        // 1h = Clears registers to default values for ENERGY and CHARGE registers
        write16bits(INA_REGISTER::CONFIG, registers.R_CONFIG | (uint16_t)_BV(14));
    }

    void SetINA229ConfigurationRegisters()
    {
        write16bits(INA_REGISTER::CONFIG, registers.R_CONFIG);
        write16bits(INA_REGISTER::ADC_CONFIG, registers.R_ADC_CONFIG);
    }
};
#endif
