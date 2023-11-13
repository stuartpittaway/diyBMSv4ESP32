#include <Arduino.h>

#ifndef DIYBMS_CELL_H // include guard
#define DIYBMS_CELL_H

class Cell
{
public:
    // Returns TRUE if the cell voltage is greater than the required setting
    bool BypassCheck() const
    {
        return (getCellVoltage() > BypassThresholdmV);
    }

    /// @brief returns cell voltage with parasitic voltage removed
    uint16_t getCellVoltage() const
    {
        return cellVoltage;
    }

    uint16_t CombineTemperatures() const
    {
        return (uint16_t)(TemperatureToByte(getInternalTemperature()) << 8) + TemperatureToByte(getExternalTemperature());
    }

    // Returns TRUE if the internal thermistor is hotter than the required setting (or over max safety limit)
    bool BypassOverheatCheck() const
    {
        auto temp = getInternalTemperature();
        return (temp > BypassTemperatureSetPoint || temp > Cell::SafetyTemperatureCutoff || Cell::getOverTemperature());
    }

    /// @brief Set Cell voltage
    /// @param v voltage in millivolts
    void setCellVoltage(uint16_t v)
    {
        cellVoltage = v;
    }

    /// @brief Set external temperature measurement
    /// @param t Temperature in degrees C
    void setExternalTemperature(int16_t t)
    {
        externalTemperature = t;
    }
    int16_t getExternalTemperature() const
    {
        return externalTemperature;
    }
    /// @brief Onboard temperature sensor (for balancing monitoring)
    /// @param t temperature in C
    void setInternalTemperature(int16_t t)
    {
        internalTemperature = t;
    }
    /// @brief Onboard temperature sensor (for balancing monitoring)
    /// @return temperature in C
    int16_t getInternalTemperature() const
    {
        return internalTemperature;
    }
    // This function reduces the scale of temperatures from int16_t type to a single byte (unsigned)
    // We have an artifical floor at 40oC, anything below +40 is considered negative (below freezing)
    // Gives range of -39 to +216 degrees C
    uint8_t TemperatureToByte(int16_t TempInCelcius) const
    {
        if (TempInCelcius == -999)
        {
            // Not fitted...
            return 0;
        }

        TempInCelcius += 40;
        // Set the limits and convert from signed to unsigned
        if (TempInCelcius < 0)
        {
            // Limit -39
            TempInCelcius = 1;
        }
        if (TempInCelcius > 255)
        {
            TempInCelcius = 255;
        }
        return (uint8_t)TempInCelcius;
    }

    bool IsBypassActive() const
    {
        return CellIsInBypass;
    }

    void StartBypass()
    {
        if (!IsBypassActive())
        {
            // Record when the bypass started
            CellIsInBypass = true;
        }
    }
    void StopBypass()
    {
        if (!IsBypassActive())
            return;

        // We don't have an accurate way to calculate energy burnt, so just increment
        // the counter to show we have actually balanced something on this cell
        MilliAmpHourBalanceCounter += 1;
        CellIsInBypass = false;
    }

    auto getMilliAmpHourBalanceCounter() const { return MilliAmpHourBalanceCounter; }
    void setMilliAmpHourBalanceCounter(float v)
    {
        MilliAmpHourBalanceCounter = v;
    }
    auto changesAllowed() const { return ChangesAllowed; }
    void disableChanges()
    {
        ChangesAllowed = false;
    }

    static auto getBypassThresholdmV() { return BypassThresholdmV; }
    static void setBypassThresholdmV(uint16_t v)
    {
        BypassThresholdmV = v;
    }
    static auto getBypassTemperatureSetPoint() { return BypassTemperatureSetPoint; }
    static auto getBypassTemperatureHysteresis() { return BypassTemperatureHysteresis; }

    static void setBypassTemperatureSetPoint(uint16_t v)
    {
        if (v > Cell::SafetyTemperatureCutoff)
        {
            v = Cell::SafetyTemperatureCutoff;
        }
        if (v > 20)
        {
            BypassTemperatureSetPoint = (uint8_t)v;

            // Set back hysteresis is set point minus 5 degrees C
            BypassTemperatureHysteresis = (uint8_t)v - 5;
        }

        // Revalidate fan temperature after bypass temperature change
        setFanSwitchOnTemperature(getFanSwitchOnTemperature());
    }

    static uint8_t getFanSwitchOnTemperature() { return fanswitchontemperature; }
    static void setFanSwitchOnTemperature(uint8_t v)
    {
        // Sanity check values
        if (v > getBypassTemperatureHysteresis() || v < 15)
        {
            v = getBypassTemperatureHysteresis();
        }

        fanswitchontemperature = v;
    }
    static auto getRelayMinmV() { return relay_minimummv; }
    static void setRelayMinmV(uint16_t v)
    {
        relay_minimummv = v;
    }
    static auto getRelayRange() { return relay_range; }
    static void setRelayRange(uint16_t v)
    {
        relay_range = v;
    }

    static auto getCalibration() { return Calibration; }
    static void setCalibration(float v)
    {
        Calibration = v;
    }
    static auto getSafetyTemperatureCutoff()
    {
        return SafetyTemperatureCutoff;
    }
    /// @brief Indicator if we have exceeded temperature, and waiting for cooldown
    /// @return
    static bool getOverTemperature()
    {
        return OverTemperature;
    }
    static void setOverTemperature(bool v)
    {
        OverTemperature = v;
    }

private:
    bool ChangesAllowed{true};
    uint16_t cellVoltage{0};
    int16_t externalTemperature{-999};
    int16_t internalTemperature{-999};
    bool CellIsInBypass{false};

    float MilliAmpHourBalanceCounter{0};

    // Defined in cell.cpp
    /// @brief Define maximum allowed temperature as safety cut off
    static const int16_t SafetyTemperatureCutoff;

    /// @brief Max temperature during balance
    static uint8_t BypassTemperatureSetPoint;
    /// @brief Balance over this voltage
    static uint16_t BypassThresholdmV;
    /// @brief Voltage calibration multiplier
    static float Calibration;
    /// @brief Temperature set back once limit is reached
    static uint8_t BypassTemperatureHysteresis;
    /// @brief Temperature has exceeded BypassTemperatureSetPoint, waiting for drop below BypassTemperatureHysteresis
    static bool OverTemperature;

    static uint16_t relay_range;
    static uint16_t relay_minimummv;
    /// @brief Temperature at which FAN output is enabled
    static uint8_t fanswitchontemperature;
};
#endif
