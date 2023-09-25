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
#include <esp_http_server.h>

class History
{
private:
    static const int NUMBER_OF_HISTORY_POINTS = 36;
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

        uint8_t address_LowCellVoltage;
        uint8_t address_HighCellVoltage;

        float voltage;
        float current;
        float stateofcharge;
        uint32_t milliamphour_in;
        uint32_t milliamphour_out;
    };

    circular_buffer<history_values, NUMBER_OF_HISTORY_POINTS> historic_readings;

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
        ptr->address_LowCellVoltage = rules->address_LowestCellVoltage;
        ptr->address_HighCellVoltage = rules->address_HighestCellVoltage;

        if (currentMonitor->validReadings)
        {
            ptr->voltage = currentMonitor->modbus.voltage;
            ptr->current = currentMonitor->modbus.current;
            ptr->milliamphour_in = currentMonitor->modbus.milliamphour_in;
            ptr->milliamphour_out = currentMonitor->modbus.milliamphour_out;
            ptr->stateofcharge = currentMonitor->stateofcharge;
        }

        historic_readings.put(hr);
    }

    esp_err_t GenerateJSON(httpd_req_t *req, char buffer[], int bufferLenMax)
    {
        // allocate & clear memory for copy of history values
        auto h = (history_values *)calloc(NUMBER_OF_HISTORY_POINTS, sizeof(history_values));

        if (h == nullptr)
        {
            // malloc failed
            return httpd_resp_set_status(req, HTTPD_500);
        }

        // Take a copy of all the historic readings, this allows the loops further on to work optimaly in creating JSON.
        size_t size = historic_readings.size();
        for (uint16_t i = 0; i < size; i++)
        {
            h[i] = historic_readings.peek(i);
        }

        int bufferused = 0;

        // TIME
        bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, "{\"time\":[");
        for (uint16_t i = 0; i < size; i++)
        {
            bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, "%ld", h[i].historic_time);
            if (i != (size - 1))
            {
                bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, ",");
            }
        }

        // stateofcharge
        bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, "],\"stateofcharge\":[");
        for (uint16_t i = 0; i < size; i++)
        {
            bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, "%.2f", h[i].stateofcharge);
            if (i != (size - 1))
            {
                bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, ",");
            }
        }

        // voltage
        bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, "],\"voltage\":[");
        for (uint16_t i = 0; i < size; i++)
        {
            bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, "%.2f", h[i].voltage);
            if (i != (size - 1))
            {
                bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, ",");
            }
        }
        //  Send it...
        httpd_resp_send_chunk(req, buffer, bufferused);

        // voltrange
        bufferused = 0;

        // milliamphour_in
        bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, "],\"milliamphour_in\":[");
        for (uint16_t i = 0; i < size; i++)
        {
            bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, "%u", h[i].milliamphour_in);
            if (i != (size - 1))
            {
                bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, ",");
            }
        }

        // milliamphour_out
        bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, "],\"milliamphour_out\":[");
        for (uint16_t i = 0; i < size; i++)
        {
            bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, "%u", h[i].milliamphour_out);
            if (i != (size - 1))
            {
                bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, ",");
            }
        }

        // current
        bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, "],\"current\":[");
        for (uint16_t i = 0; i < size; i++)
        {
            bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, "%.4f", h[i].current);
            if (i != (size - 1))
            {
                bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, ",");
            }
        }
        //  Send it...
        httpd_resp_send_chunk(req, buffer, bufferused);

        // voltrange
        bufferused = 0;

        // highestExternalTemp
        bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, "],\"highestExternalTemp\":[");
        for (uint16_t i = 0; i < size; i++)
        {
            bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, "%i", h[i].highestExternalTemp);
            if (i != (size - 1))
            {
                bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, ",");
            }
        }

        // lowestExternalTemp
        bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, "],\"lowestExternalTemp\":[");
        for (uint16_t i = 0; i < size; i++)
        {
            bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, "%i", h[i].lowestExternalTemp);
            if (i != (size - 1))
            {
                bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, ",");
            }
        }

        // lowestBankVoltage
        bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, "],\"lowestBankVoltage\":[");
        for (uint16_t i = 0; i < size; i++)
        {
            bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, "%u", h[i].lowestBankVoltage);
            if (i != (size - 1))
            {
                bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, ",");
            }
        }

        //  Send it...
        httpd_resp_send_chunk(req, buffer, bufferused);

        // voltrange
        bufferused = 0;

        // HighestBankVoltage
        bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, "],\"highestBankVoltage\":[");
        for (uint16_t i = 0; i < size; i++)
        {
            bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, "%u", h[i].highestBankVoltage);
            if (i != (size - 1))
            {
                bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, ",");
            }
        }

        // HighestBankRange
        bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, "],\"highestBankRange\":[");
        for (uint16_t i = 0; i < size; i++)
        {
            bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, "%u", h[i].highestBankRange);
            if (i != (size - 1))
            {
                bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, ",");
            }
        }
        // highestCellVoltage
        bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, "],\"highestCellVoltage\":[");
        for (uint16_t i = 0; i < size; i++)
        {
            bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, "%u", h[i].highestCellVoltage);
            if (i != (size - 1))
            {
                bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, ",");
            }
        }
        // Address highestCellVoltage
        bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, "],\"address_HighCellV\":[");
        for (uint16_t i = 0; i < size; i++)
        {
            bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, "%u", h[i].address_HighCellVoltage);
            if (i != (size - 1))
            {
                bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, ",");
            }
        }
        //  Send it...
        httpd_resp_send_chunk(req, buffer, bufferused);

        // voltrange
        bufferused = 0;

        // lowestCellVoltage
        bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, "],\"lowestCellVoltage\":[");
        for (uint16_t i = 0; i < size; i++)
        {
            bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, "%u", h[i].lowestCellVoltage);
            if (i != (size - 1))
            {
                bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, ",");
            }
        }

        // Address highestCellVoltage
        bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, "],\"address_LowCellV\":[");
        for (uint16_t i = 0; i < size; i++)
        {
            bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, "%u", h[i].address_LowCellVoltage);
            if (i != (size - 1))
            {
                bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, ",");
            }
        }

        // Closing tag
        bufferused += snprintf(&buffer[bufferused], bufferLenMax - bufferused, "]}");

        //  Send it...
        httpd_resp_send_chunk(req, buffer, bufferused);

        free(h);

        // Indicate last chunk (zero byte length)
        return httpd_resp_send_chunk(req, buffer, 0);
    }
};

#endif