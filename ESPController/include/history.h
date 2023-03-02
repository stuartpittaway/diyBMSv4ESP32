#ifndef History_H_
#define History_H_

#pragma once
/*
#define USE_ESP_IDF_LOG 1
static constexpr const char *const TAG = "diybms-hist";
*/
#include "defines.h"
#include "Rules.h"
#include "circular_buffer.hpp"

class History
{
private:
    static const int NUMBER_OF_HOURS_HISTORY = 36;
    struct history_values
    {
        time_t historic_time;
        // Highest cell voltage range (mV) across all banks
        uint16_t highestBankRange;

        // Highest cell voltage in the whole system (millivolts)
        uint16_t highestCellVoltage;
        // Lowest cell voltage in the whole system (millivolts)
        uint16_t lowestCellVoltage;

        // Highest pack voltage (millivolts)
        uint32_t highestBankVoltage;
        // Lowest pack voltage (millivolts)
        uint32_t lowestBankVoltage;

        int8_t highestExternalTemp;
        int8_t lowestExternalTemp;

        float voltage;
        float current;
        float stateofcharge;
        uint32_t milliamphour_in;
        uint32_t milliamphour_out;
    };

    circular_buffer<history_values, NUMBER_OF_HOURS_HISTORY> historic_readings;

public:
    void Clear()
    {
        // Zero out memory for holding historic data
        memset(&historic_readings, 0, sizeof(historic_readings));
    }

    // Capture important statistics as a snapshot in time into circular buffer
    void SnapshotHistory(time_t now, Rules *rules, currentmonitoring_struct *currentMonitor)
    {
        history_values hr;
        history_values *ptr = &hr;

        // Zero all values
        memset(ptr, 0, sizeof(history_values));
        // Capture the values
        ptr->historic_time = now;
        ptr->highestBankRange = rules->highestBankRange;
        ptr->highestCellVoltage = rules->highestCellVoltage;
        ptr->lowestCellVoltage = rules->lowestCellVoltage;
        ptr->highestBankVoltage = rules->highestBankVoltage;
        ptr->lowestBankVoltage = rules->lowestBankVoltage;
        ptr->highestExternalTemp = rules->highestExternalTemp;
        ptr->lowestExternalTemp = rules->lowestExternalTemp;

        if (currentMonitor->validReadings)
        {
            ptr->voltage = currentMonitor->modbus.voltage;
            ptr->current = currentMonitor->modbus.current;
            ptr->milliamphour_in = currentMonitor->modbus.milliamphour_in;
            ptr->milliamphour_out = currentMonitor->modbus.milliamphour_out;
            ptr->stateofcharge = currentMonitor->stateofcharge;
        }

        historic_readings.put(hr);
        /*
                ESP_LOGD(TAG, "cap=%u size=%u", historic_readings.capacity(), historic_readings.size());

                for (uint16_t i = 0; i < historic_readings.size(); i++)
                {
                    hr = historic_readings.peek(i);
                    ESP_LOGD(TAG, "%u:time=%ld", i, hr.historic_time);
                }*/
    }

    int GenerateJSON(char buffer[], int bufferLenMax)
    {
        int bufferused = 0;

        bufferused += snprintf(&buffer[bufferused], BUFSIZE - bufferused, "{");
        bufferused += snprintf(&buffer[bufferused], BUFSIZE - bufferused, "\"time\":[");

        size_t size = historic_readings.size();
        for (uint16_t i = 0; i < size; i++)
        {
            history_values hr = historic_readings.peek(i);
            bufferused += snprintf(&buffer[bufferused], BUFSIZE - bufferused, "%u", hr.historic_time);
            if (i != (size - 1))
            {
                bufferused += snprintf(&buffer[bufferused], BUFSIZE - bufferused, ",");
            }
        }

        bufferused += snprintf(&buffer[bufferused], BUFSIZE - bufferused, "]");
        bufferused += snprintf(&buffer[bufferused], BUFSIZE - bufferused, "}");

        return bufferused;
    }
};

#endif