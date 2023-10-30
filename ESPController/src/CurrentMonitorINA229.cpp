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

#define USE_ESP_IDF_LOG 1
static constexpr const char *const TAG = "curmon";

#include "CurrentMonitorINA229.h"

void CurrentMonitorINA229::CalculateLSB()
{
    // Take a look at these for information on how it works!
    // https://dev.ti.com/gallery/view/4910879/INA228_229_237_238_239EVM_GUI/ver/2.0.0/
    // in above - click COG icon top left.
    // https://e2e.ti.com/support/amplifiers-group/amplifiers/f/amplifiers-forum/1034569/ina228-accumulated-energy-and-charge-is-wrong

    // 150A/50mV shunt =   full_scale_current= 150.00A / 50.00 * 40.96 = 122.88 AMPS
    //                     RSHUNT = (50 / 1000) / 150 = 0.00033333333
    //                     CURRENT_LSB = 150/ 524288 = 0.000286102294921875
    //                     R_SHUNT_CAL = 52428800000*0.000286102294921875*0.00033333333 = 4999.999 = 5000

    // 300A/75mV shunt =   full_scale_current= 300.00A / 75.00 * 40.96 = 163.84 AMPS
    //                     RSHUNT = (75 / 1000) / 163.84 = 0.000457763671875
    //                     CURRENT_LSB = 163.84/ 524288 = 2.648162841796875e-4
    //                     R_SHUNT_CAL = 52428800000*0.00057220458984375*0.00025 = 7499.9922688 = 7500

    // Calculate CURRENT_LSB and R_SHUNT_CAL values
    // registers.full_scale_current = ((float)registers.shunt_max_current / (float)registers.shunt_millivolt) * full_scale_adc;
    registers.RSHUNT = ((float)registers.shunt_millivolt / 1000.0F) / (float)registers.shunt_max_current;
    registers.CURRENT_LSB = registers.shunt_max_current / (float)0x80000;
    registers.R_SHUNT_CAL = 4L * (13107200000L * registers.CURRENT_LSB * registers.RSHUNT);

    ESP_LOGI(TAG, "RSHUNT=%.8f, CURRENT_LSB=%.8f, R_SHUNT_CAL=%u", registers.RSHUNT, registers.CURRENT_LSB, registers.R_SHUNT_CAL);

    // Will the full scale current be over the 40.96mV range the ADC can handle?
    if (((float)registers.shunt_max_current * registers.RSHUNT * 1000.0) > full_scale_adc)
    {
        float true_max_current = ((float)registers.shunt_max_current / (float)registers.shunt_millivolt) * full_scale_adc;
        ESP_LOGW(TAG, "WARNING: True Max Current Measurable %.2f Amps", true_max_current);

        registers.CURRENT_LSB = true_max_current / (float)0x80000;
        registers.R_SHUNT_CAL = 4L * (13107200000L * registers.CURRENT_LSB * registers.RSHUNT);
        ESP_LOGI(TAG, "RECALC: RSHUNT=%.8f, CURRENT_LSB=%.8f, R_SHUNT_CAL=%u", registers.RSHUNT, registers.CURRENT_LSB, registers.R_SHUNT_CAL);
    }

    // Deliberately reduce calibration by 2.5%, which appears to be the loses seen in the current monitor circuit design
    // (or shunt resistance tolerance)
    // You can always configure this value through the web gui - "Calibration" value.
    // registers.R_SHUNT_CAL = ((uint32_t)registers.R_SHUNT_CAL * 985) / 1000;
}

// Sets SOC by setting "fake" in/out amphour counts
// value=8212 = 82.12%
void CurrentMonitorINA229::SetSOC(uint16_t value)
{
    // Assume battery is fully charged
    milliamphour_in = 1000 * (uint32_t)registers.batterycapacity_amphour;
    // And we have consumed this much...
    milliamphour_out = (uint32_t)((1.0F - ((float)value / 10000.0F)) * milliamphour_in);

    // Zero out readings using the offsets
    milliamphour_out_offset = milliamphour_out;
    milliamphour_in_offset = milliamphour_in;
}

uint8_t CurrentMonitorINA229::readRegisterValue(INA_REGISTER r) const
{
    // These are not really registers, but shape the SPI frame to indicate read/write
    // page 19 of documentation, Table 7-2. First 8-MSB Bits of SPI Frame.
    // Read register
    return (uint8_t)((r << 2U) | B00000001);
}
uint8_t CurrentMonitorINA229::writeRegisterValue(INA_REGISTER r) const
{
    // These are not really registers, but shape the SPI frame to indicate read/write
    // page 19 of documentation, Table 7-2. First 8-MSB Bits of SPI Frame.
    // Write register
    return (uint8_t)((r << 2U) | (uint8_t)B00000000);
}

uint16_t CurrentMonitorINA229::read16bits(INA_REGISTER r)
{
    SPI_Ptr->beginTransaction(_spisettings);
    digitalWrite(chipselectpin, LOW);
    // The transfers are always a step behind, so the transfer reads the previous value/command
    SPI_Ptr->write(readRegisterValue(r));
    uint16_t value = SPI_Ptr->transfer16(0);
    digitalWrite(chipselectpin, HIGH);
    SPI_Ptr->endTransaction();

    // ESP_LOGD(TAG, "Read register 0x%02x = 0x%04x", r, value);
    return value;
}

// Write a register in INA229
// During an SPI write frame, while new data is shifted into the INA229-Q1
// register, the old data from the same register is shifted out on the MISO line.
uint16_t CurrentMonitorINA229::write16bits(INA_REGISTER r, uint16_t value)
{
    SPI_Ptr->beginTransaction(_spisettings);
    digitalWrite(chipselectpin, LOW);
    // The transfers are always a step behind, so the transfer reads the previous value/command
    SPI_Ptr->write(writeRegisterValue(r));
    uint16_t retvalue = SPI_Ptr->transfer16(value);
    digitalWrite(chipselectpin, HIGH);
    SPI_Ptr->endTransaction();

    // ESP_LOGD(TAG, "Write register 0x%02x = 0x%04x (old value=0x%04x)", r, value, retvalue);
    //  retvalue is the PREVIOUS value stored in the register
    return retvalue;
}

// Reads 3 bytes from SPI bus, and returns them in a uint32 (lower 24 bits)
uint32_t CurrentMonitorINA229::spi_readUint24(INA_REGISTER r)
{
    SPI_Ptr->beginTransaction(_spisettings);
    digitalWrite(chipselectpin, LOW);
    SPI_Ptr->write(readRegisterValue(r));
    uint8_t a = SPI_Ptr->transfer(0);
    uint8_t b = SPI_Ptr->transfer(0);
    uint8_t c = SPI_Ptr->transfer(0);
    digitalWrite(chipselectpin, HIGH);
    SPI_Ptr->endTransaction();

    uint32_t value = ((uint32_t)a << 16) | ((uint32_t)b << 8) | ((uint32_t)c);

    // ESP_LOGD(TAG, "Read register 0x%02x = 0x%08x", r, value);
    return value;
}

int64_t CurrentMonitorINA229::spi_readInt40(INA_REGISTER r)
{
    SPI_Ptr->beginTransaction(_spisettings);
    digitalWrite(chipselectpin, LOW);
    SPI_Ptr->write(readRegisterValue(r));
    uint8_t a = SPI_Ptr->transfer(0);
    uint8_t b = SPI_Ptr->transfer(0);
    uint8_t c = SPI_Ptr->transfer(0);
    uint8_t d = SPI_Ptr->transfer(0);
    uint8_t e = SPI_Ptr->transfer(0);
    digitalWrite(chipselectpin, HIGH);
    SPI_Ptr->endTransaction();

    // Check if a twos compliment negative number
    uint64_t reply = (a & 0x80) ? (uint64_t)0xFFFFFF0000000000 : 0;

    reply += (uint64_t)a << 32;
    reply += (uint64_t)b << 24;
    reply += (uint64_t)c << 16;
    reply += (uint64_t)d << 8;
    reply += (uint64_t)e;

    // ESP_LOGD(TAG, "Read register 0x%02x");

    // Cast to signed integer (which also sorts out the negative sign if applicable)
    return (int64_t)reply;
}

uint64_t CurrentMonitorINA229::spi_readUint40(INA_REGISTER r)
{
    SPI_Ptr->beginTransaction(_spisettings);
    digitalWrite(chipselectpin, LOW);
    SPI_Ptr->write(readRegisterValue(r));
    uint8_t a = SPI_Ptr->transfer(0);
    uint8_t b = SPI_Ptr->transfer(0);
    uint8_t c = SPI_Ptr->transfer(0);
    uint8_t d = SPI_Ptr->transfer(0);
    uint8_t e = SPI_Ptr->transfer(0);
    digitalWrite(chipselectpin, HIGH);
    SPI_Ptr->endTransaction();

    uint64_t value = (uint64_t)a << 32;
    value += (uint64_t)b << 24;
    value += (uint64_t)c << 16;
    value += (uint64_t)d << 8;
    value += (uint64_t)e;

    // ESP_LOGD(TAG, "Read register 0x%02x", r);
    return value;
}

// Read a 24 bit (3 byte) unsigned integer into a uint32 (including right shift 4 bits)
uint32_t CurrentMonitorINA229::readUInt24(INA_REGISTER r)
{
    return spi_readUint24(r) >> 4;
}

// Energy in JOULES
float CurrentMonitorINA229::Energy()
{
    uint64_t energy = spi_readUint40(INA_REGISTER::ENERGY);
    return 16.0F * 3.2F * registers.CURRENT_LSB * energy;
}

// Charge in Coulombs.
// The raw value is 40 bit, but we frequently reset this back to zero so it prevents
// overflow and keeps inside the int32 limits
// NEGATIVE value means battery is being charged
int32_t CurrentMonitorINA229::ChargeInCoulombsAsInt()
{
    // Calculated charge output. Output value is in Coulombs.Two's complement value.  40bit number stored in int64
    return registers.CURRENT_LSB * (float)spi_readInt40(INA_REGISTER::CHARGE);
}

// Bus voltage output. Two's complement value, however always positive.  Value in bits 23 to 4
float CurrentMonitorINA229::BusVoltage()
{
    uint32_t busVoltage = readUInt24(INA_REGISTER::VBUS) & 0x000FFFFF;
    // The accuracy is 20bits and 195.3125uV is the LSB
    uint32_t busVoltage_mV = (uint64_t)busVoltage * (uint64_t)0x1DCD65 / (uint64_t)0x989680; // conversion to get mV

    // ESP_LOGD(TAG, "busVoltage mV=%u", busVoltage_mV);

    //  Return VOLTS
    return (float)busVoltage_mV / (float)1000.0;
}

// Read a 20 bit (3 byte) TWOS COMPLIMENT integer from an INA register
// Used to read VSHUNT value
int32_t CurrentMonitorINA229::readInt20(INA_REGISTER r)
{
    // Returns
    uint32_t value = readUInt24(r);

    // The number is two's complement, check for negative at bit 20
    if (value & 0x80000)
    {
        // Invert
        value = ~value;
        // Trim to 20 bits
        value = value & 0x000FFFFFU;
        // Add 1
        value = value + 1;
        // Return NEGATIVE of the value
        return -(int32_t)value;
    }

    return (int32_t)value;
}

// Shunt voltage in MILLIVOLTS mV
float CurrentMonitorINA229::ShuntVoltage()
{
    // 78.125 nV/LSB when ADCRANGE = 1
    // Differential voltage measured across the shunt output. Two's complement value.
    // 20 bit value max = 1048575
    int32_t vshunt = readInt20(INA_REGISTER::VSHUNT);
    return (float)(((int64_t)vshunt) * 78125UL) / 1000000000.0;
}

void CurrentMonitorINA229::TakeReadings()
{
    voltage = BusVoltage();
    current = Current();
    power = Power();

    // https://github.com/stuartpittaway/diyBMSv4ESP32/issues/240
    // INA229 is reported to have poor calculation of power at low levels
    // this workaround overrides the power value below 200W
    auto pow_val = voltage * abs(current);
    ESP_LOGD(TAG, "V=%.4f, I=%.4f, P=%.2f, Calc_P=%.2f", voltage, current, power, pow_val);
    if (pow_val < 200) { power = pow_val; }

    temperature = DieTemperature();
    SOC = CalculateSOC();
    CalculateAmpHourCounts();
}

uint16_t CurrentMonitorINA229::CalculateSOC() const
{
    float milliamphour_in_scaled = ((float)milliamphour_in / 100.0F) * registers.charge_efficiency_factor;
    float milliamphour_batterycapacity = 1000.0F * (uint32_t)registers.batterycapacity_amphour;
    float difference = milliamphour_in_scaled - (float)milliamphour_out;

    float answer = 100 * (difference / milliamphour_batterycapacity);
    if (answer < 0)
    {
        // We have taken more out of the battery than put in, so must be zero SoC (or more likely out of calibration)
        return 0;
    }

    // Store result as fixed point decimal
    uint16_t SOC = 100 * answer;

    // Add a hard upper limit 999.99%
    if (SOC > 99999)
    {
        SOC = (uint16_t)99999;
    }

    return SOC;
}

void CurrentMonitorINA229::CalculateAmpHourCounts()
{
    // We amp-hour count using units of 18 coulombs = 5mAh, to avoid rounding issues

    // If we don't have a voltage reading, ignore the coulombs - also means
    // Ah counting won't work without voltage reading on the INA228 chip
    if (voltage > 0)
    {
        int32_t charge_coulombs = ChargeInCoulombsAsInt();
        int32_t difference = charge_coulombs - last_charge_coulombs;

        // Have we used up more than 5mAh of energy?
        // if not, ignore for now and await next cycle
        if (abs(difference) >= 18)
        {
            if (difference > 0)
            {
                // Amp hour out
                // Integer divide (18 coulombs)
                int32_t integer_divide = (difference / 18);
                // Subtract remainder
                last_charge_coulombs = charge_coulombs - (difference - (integer_divide * 18));
                // Chunks of 5mAh
                uint32_t a = integer_divide * 5;
                milliamphour_out += a;
                milliamphour_out_lifetime += a;
                daily_milliamphour_out += a;
            }
            else
            {
                // Make it positive, for counting amp hour in
                difference = abs(difference);
                int32_t integer_divide = (difference / 18);
                // Add on remainder
                last_charge_coulombs = charge_coulombs + (difference - (integer_divide * 18));
                // chunks of 5mAh
                uint32_t a = integer_divide * 5;
                milliamphour_in += a;
                milliamphour_in_lifetime += a;
                daily_milliamphour_in += a;
            }
        }
    }

    // Periodically we need to reset the energy register to prevent it overflowing
    // if we do this too frequently we get incorrect readings over the long term
    // 360000 = 100Amp Hour
    if (abs(last_charge_coulombs) > (int32_t)360000)
    {
        ResetChargeEnergyRegisters();
        last_charge_coulombs = 0;
    }

    // Now to test if we need to reset SOC to 100% ?
    // Check if voltage is over the fully_charged_voltage and current UNDER tail_current_amps
    if (voltage >= registers.fully_charged_voltage && current > 0 && current < registers.tail_current_amps)
    {
        // Battery has reached fully charged so wait for time counter to elapse
        ESP_LOGI(TAG,"Battery has reached charging tail period");

        if (esp_timer_get_time() > soc_reset_time)
        {
            // Now we reset the SOC, by clearing the registers, at this point SOC returns to 100%
            // This does have an annoying "feature" of clearing down Ah counts :-(
            // TODO: FIX THIS - probably need a set of shadow variables to hold the internal SOC and AH counts
            //                  but then when/how do we reset the Ah counts?
            ResetChargeEnergyRegisters();
            last_charge_coulombs = 0;
            SetSOC(10000);
        }
    }
    else
    {
        // Voltage or current is out side of monitoring limits, so set timer to now + 3 minutes
        int64_t soc_reset_time = esp_timer_get_time() + (3 * 60000000L);
    }
}

// Guess the SoC % based on battery voltage - not accurate, just a guess!
void CurrentMonitorINA229::GuessSOC()
{
    // Default SOC% at 60%
    uint16_t soc = 6000;

    // We apply a "guestimate" to SoC based on voltage - not really accurate, but somewhere to start
    // only applicable to 24V/48V (16S) LIFEPO4 setups. These voltages should be the unloaded (no current flowing) voltage.
    // Assumption that its LIFEPO4 cells we are using...
    float v = BusVoltage();

    if (v > 20 && v < 30)
    {
        // Scale up 24V battery to use the 48V scale
        v = v * 2;
    }

    if (v > 40 && v < 60)
    {
        // 16S LIFEPO4...
        if (v >= 40.0)
            soc = 500;
        if (v >= 48.0)
            soc = 900;
        if (v >= 50.0)
            soc = 1400;
        if (v >= 51.2)
            soc = 1700;
        if (v >= 51.6)
            soc = 2000;
        if (v >= 52.0)
            soc = 3000;
        if (v >= 52.4)
            soc = 4000;
        if (v >= 52.8)
            soc = 7000;
        if (v >= 53.2)
            soc = 9000;
    }

    SetSOC(soc);

    // Reset the daily counters
    daily_milliamphour_in = 0;
    daily_milliamphour_out = 0;
}

bool CurrentMonitorINA229::Configure(uint16_t shuntmv,
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
                                     int32_t overpowerlimit,
                                     uint16_t shunttempcoefficient,
                                     bool TemperatureCompEnabled)
{
    registers.shunt_millivolt = shuntmv;
    registers.shunt_max_current = shuntmaxcur;
    registers.fully_charged_voltage = fullchargevolt / 100.0;
    registers.batterycapacity_amphour = batterycapacity;
    registers.tail_current_amps = tailcurrent / 100.0;
    registers.charge_efficiency_factor = chargeefficiency / 100.0;

    CalculateLSB();

    if (shuntcal != 0 && registers.R_SHUNT_CAL != shuntcal)
    {
        ESP_LOGI(TAG, "Override SHUNT_CAL from %u to %u", registers.R_SHUNT_CAL, shuntcal);
        // Are we trying to override the default SHUNT_CAL value?
        registers.R_SHUNT_CAL = shuntcal;
    }

    // This is not enabled by default
    // The 16 bit register provides a resolution of 1ppm/°C/LSB
    // Shunt Temperature Coefficient - 3FFF is maximum
    registers.R_SHUNT_TEMPCO = shunttempcoefficient & 0x3FFF;

    // Shunt Over Limit (current limit) = overcurrent protection
    registers.R_SOVL = ConvertTo2sComp((((float)overcurrentlimit / 100.0F) * 1000.0F / 1.25F) * full_scale_adc / registers.shunt_max_current);
    // Shunt UNDER Limit (under current limit) = undercurrent protection
    registers.R_SUVL = ConvertTo2sComp((((float)undercurrentlimit / 100.0F) * 1000.0F / 1.25F) * full_scale_adc / registers.shunt_max_current);
    // Bus Overvoltage (overvoltage protection).
    registers.R_BOVL = ConvertTo2sComp(((float)overvoltagelimit / 100.0F) / 0.003125F);
    // Bus under voltage protection
    registers.R_BUVL = ConvertTo2sComp(((float)undervoltagelimit / 100.0F) / 0.003125F);
    // temperature limit
    registers.R_TEMP_LIMIT = (int16_t)temperaturelimit / (float)0.0078125;

    // Default Power limit = 5kW
    registers.R_PWR_LIMIT = (uint16_t)((float)overpowerlimit / 256.0F / 3.2F / registers.CURRENT_LSB);

    // ESP_LOGI(TAG, "undercurrentlimit=%f, R_SUVL=%u", ((float)undercurrentlimit / 100.0F), registers.R_SUVL);

    registers.R_CONFIG = _BV(4); // ADCRANGE = 40.96mV scale

    if (TemperatureCompEnabled)
    {
        // Set bit
        registers.R_CONFIG |= bit(5);
    }

    // Configure other registers
    write16bits(INA_REGISTER::CONFIG, registers.R_CONFIG);
    write16bits(INA_REGISTER::ADC_CONFIG, registers.R_ADC_CONFIG);
    write16bits(INA_REGISTER::SHUNT_CAL, registers.R_SHUNT_CAL);
    write16bits(INA_REGISTER::DIAG_ALRT, registers.R_DIAG_ALRT);

    // Check MEMSTAT=1 which proves the INA chip is not corrupt
    diag_alrt_value = read16bits(INA_REGISTER::DIAG_ALRT);
    if ((diag_alrt_value & bit(DIAG_ALRT_FIELD::MEMSTAT)) == 0)
    {
        // MEMSTAT error
        ESP_LOGE(TAG, "INA229 chip MEMSTAT error");
        INA229Installed = false;
        return false;
    }

    SetINA229Registers();

    return true;
}

// Initialise INA229 chip and class object
// fullchargevolt, tailcurrent and chargeefficiency are fixed point values (x100, example 1234=12.34)
bool CurrentMonitorINA229::Initialise(SPIClass *SPI, uint8_t cs_pin)
{
    SPI_Ptr = SPI;
    chipselectpin = cs_pin;
    assert(SPI != nullptr);

    uint16_t value = read16bits(INA_REGISTER::DEVICE_ID);
    if ((value >> 4) == 0x229)
    {
        ESP_LOGI(TAG, "Chip=INA%02x, Revision=%u", value >> 4, value & B1111);
        INA229Installed = true;
    }
    else
    {
        // Stop here - no chip found
        ESP_LOGI(TAG, "INA229 chip absent");
        return false;
    }

    // Now we know the INA chip is connected, reset to power on defaults
    // 15=Reset Bit. Setting this bit to '1' generates a system reset that is the same as power-on reset.
    write16bits(INA_REGISTER::CONFIG, (uint16_t)_BV(15));
    // Allow the reset to work
    delay(100);

    return true;
}

void CurrentMonitorINA229::SetINA229Registers()
{
    write16bits(INA_REGISTER::SHUNT_CAL, registers.R_SHUNT_CAL);
    write16bits(INA_REGISTER::SHUNT_TEMPCO, registers.R_SHUNT_TEMPCO);
    write16bits(INA_REGISTER::SOVL, registers.R_SOVL);
    write16bits(INA_REGISTER::SUVL, registers.R_SUVL);
    write16bits(INA_REGISTER::BOVL, registers.R_BOVL);
    write16bits(INA_REGISTER::BUVL, registers.R_BUVL);
    write16bits(INA_REGISTER::TEMP_LIMIT, registers.R_TEMP_LIMIT);
    write16bits(INA_REGISTER::PWR_LIMIT, registers.R_PWR_LIMIT);
}
