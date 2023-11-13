
/*
  ____  ____  _  _  ____  __  __  ___
 (  _ \(_  _)( \/ )(  _ \(  \/  )/ __)
  )(_) )_)(_  \  /  ) _ < )    ( \__ \
 (____/(____) (__) (____/(_/\/\_)(___/

DIYBMS V4.0

(c)2019-2022 Stuart Pittaway

COMPILE THIS CODE USING PLATFORM.IO

LICENSE
Attribution-NonCommercial-ShareAlike 2.0 UK: England & Wales (CC BY-NC-SA 2.0 UK)
https://creativecommons.org/licenses/by-nc-sa/2.0/uk/

* Non-Commercial — You may not use the material for commercial purposes.
* Attribution — You must give appropriate credit, provide a link to the license, and indicate if changes were made.
  You may do so in any reasonable manner, but not in any way that suggests the licensor endorses you or your use.
* ShareAlike — If you remix, transform, or build upon the material, you must distribute your   contributions under the same license as the original.
* No additional restrictions — You may not apply legal terms or technological measures that legally restrict others from doing anything the license permits.
*/

#define USE_ESP_IDF_LOG 1
static constexpr const char *const TAG = "diybms-tft";

#define CONFIG_DISABLE_HAL_LOCKS 1

#include "defines.h"
#include "HAL_ESP32.h"
#include <esp_wifi.h>
#include <SPI.h>

#include "TFT_eSPI.h"
TFT_eSPI tft = TFT_eSPI();

#include "Rules.h"

#include "tft.h"
#include "tft_splash_image.h"

bool _tft_screen_available = false;
volatile bool _screen_awake = false;
volatile uint32_t _interrupt_triggered = 0;
bool force_tft_wake = false;
TimerHandle_t tftwake_timer;
int8_t tftsleep_timer = 0;

ScreenTemplateToDisplay _lastScreenToDisplay = ScreenTemplateToDisplay::NotInstalled;
uint8_t _ScreenToDisplayDelay = 0;
int8_t _ScreenPageCounter = 0;

int16_t fontHeight_2;
int16_t fontHeight_4;

TouchScreenValues _lastTouch;

void ResetScreenSequence()
{
    _ScreenToDisplayDelay = 0;
    _ScreenPageCounter = 0;
    _lastScreenToDisplay = ScreenTemplateToDisplay::None;
    tftsleep_timer = 120;
}

void IRAM_ATTR TFTScreenTouchInterrupt()
{
    // Keep track of interrupts
    uint32_t _interrupt_triggered_on_entry = _interrupt_triggered;
    _interrupt_triggered++;

    // Avoid multiple touches/ISR until the last one has been processed
    if (_interrupt_triggered_on_entry > 0)
        return;

    if (!_tft_screen_available)
        return;

    // if (_screen_awake)
    // return;

    // Trigger timer to wake up the screen
    if (tftwake_timer != NULL)
    {
        BaseType_t xHigherPriorityTaskWoken;
        xHigherPriorityTaskWoken = pdFALSE;
        xTimerStartFromISR(tftwake_timer, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken == pdTRUE)
        {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

void TFTDrawWifiDetails()
{
    tft.setTextDatum(TL_DATUM);

    int16_t y = tft.height() - fontHeight_2;

    tft.fillRect(0, y, tft.width(), tft.height() - y, TFT_DARKGREY);
    tft.setTextFont(2);
    tft.setTextColor(TFT_BLACK, TFT_DARKGREY);
    int16_t x = 2;

    if (wifi_isconnected)
    {
        x += tft.drawString(hostname.c_str(), x, y);
        x += 10;
        x += tft.drawString(ip_string.c_str(), x, y);

        // Draw RSSI on bottom right corner
        // Received Signal Strength in dBm
        wifi_ap_record_t ap;
        esp_wifi_sta_get_ap_info(&ap);
        x += 10;
        x += tft.drawNumber(ap.rssi, x, y);
        x += tft.drawString("dBm", x, y);
    }
    else
    {
        x += tft.drawString("WIFI not connected", x, y);
    }

    DrawClock();
}

void DrawClock()
{
    struct tm timeinfo;
    time_t now;
    time(&now);
    localtime_r(&now, &timeinfo);
    if (timeinfo.tm_year > (2016 - 1900))
    {
        tft.setTextDatum(TL_DATUM);

        // Draw the time in bottom right corner of screen
        int16_t y = tft.height() - fontHeight_2;
        int16_t x = tft.width() - 38;

        std::string clock;

        if (timeinfo.tm_hour < 10)
        {
            clock.append("0");
        }
        clock.append(std::to_string(timeinfo.tm_hour)).append(":");
        if (timeinfo.tm_min < 10)
        {
            clock.append("0");
        }
        clock.append(std::to_string(timeinfo.tm_min));

        x += tft.drawString(clock.c_str(), x, y);
    }
}

void PrepareTFT_NoWiFi()
{
    // Assumes the Mutex is already obtained by caller
    tft.fillScreen(TFT_BLACK);
    tft.fillRoundRect(16, 16, tft.width() - 32, tft.height() - 48, 8, TFT_BLUE);
    // White outline
    tft.drawRoundRect(16 + 2, 16 + 2, tft.width() - 36, tft.height() - 52, 8, TFT_LIGHTGREY);
}

void PrepareTFT_Error()
{
    // Assumes the Mutex is already obtained by caller
    tft.fillScreen(TFT_BLACK);
    // Errors have priority, draw filled red box
    tft.fillRoundRect(16, 16, tft.width() - 32, tft.height() - 48, 8, TFT_RED);

    // White outline
    tft.drawRoundRect(16 + 2, 16 + 2, tft.width() - 36, tft.height() - 52, 8, TFT_LIGHTGREY);

    TFTDrawWifiDetails();
}

void PrepareTFT_ControlState()
{
    // Assumes the Mutex is already obtained by caller
    tft.fillScreen(TFT_BLACK);

    tft.setTextColor(TFT_WHITE, TFT_DARKCYAN);
    tft.setTextFont(2);
    uint16_t x = tft.width() / 2;
    uint16_t y = tft.height() / 2 - fontHeight_4 * 2;
    // Centre/middle text
    tft.setTextDatum(TC_DATUM);

    switch (_controller_state)
    {
    case ControllerState::Running:
    case ControllerState::Unknown:
    {
        // Errors have priority, draw filled red box
        tft.fillRoundRect(16, 16, tft.width() - 32, tft.height() - 48, 8, TFT_DARKCYAN);
        // White outline
        tft.drawRoundRect(16 + 2, 16 + 2, tft.width() - 36, tft.height() - 52, 8, TFT_WHITE);
        tft.drawNumber(_controller_state, x, y, 4);
        break;
    }
    case ControllerState::NoWifiConfiguration:
    {
        tft.setTextColor(TFT_WHITE, TFT_BLUE);
        tft.fillRoundRect(16, 16, tft.width() - 32, tft.height() - 48, 8, TFT_BLUE);
        tft.drawRoundRect(16 + 2, 16 + 2, tft.width() - 36, tft.height() - 52, 8, TFT_WHITE);
        tft.drawCentreString("Configure", x, y, 4);
        y += fontHeight_4;
        tft.drawCentreString("WiFi", x, y, 4);
        y += fontHeight_4;
        tft.drawCentreString("Settings", x, y, 4);
        break;
    }
    case ControllerState::PowerUp:
    case ControllerState::Stabilizing:
    {
        // Draw box in same colour as background of logo/image
        tft.fillRoundRect(8, 8, tft.width() - 16, tft.height() - 48, 8, SplashLogoPalette[3]);
        // White border
        tft.drawRoundRect(8 + 2, 8 + 2, tft.width() - 20, tft.height() - 52, 8, TFT_WHITE);

        TFT_eSprite spr = TFT_eSprite(&tft);
        spr.setColorDepth(4);
        spr.createSprite(SplashLogoGraphic_Width, SplashLogoGraphic_Height);
        spr.pushImage(0, 0,
                      SplashLogoGraphic_Width,
                      SplashLogoGraphic_Height,
                      (uint16_t *)SplashLogoGraphic);

        spr.createPalette(SplashLogoPalette, 16);
        spr.pushSprite(tft.width() / 2 - SplashLogoGraphic_Width / 2, 16);
        spr.deleteSprite();

        y = 100;
        tft.setTextColor(TFT_WHITE, SplashLogoPalette[3]);
        tft.setTextDatum(MR_DATUM);
        tft.drawString("Version: ", x, y, 2);

        tft.setTextDatum(ML_DATUM);
        tft.setTextColor(TFT_YELLOW, SplashLogoPalette[3]);
        tft.drawString(GIT_VERSION_SHORT, x, y, 2);

        y += 2 * fontHeight_2;
        tft.setTextColor(TFT_WHITE, SplashLogoPalette[3]);
        tft.setTextDatum(MR_DATUM);
        tft.drawString("Build Date: ", x, y, 2);
        tft.setTextDatum(ML_DATUM);
        tft.setTextColor(TFT_YELLOW, SplashLogoPalette[3]);
        tft.drawString(COMPILE_DATE_TIME_SHORT, x, y, 2);
        y += fontHeight_2;

        break;
    }

    } // end switch

    TFTDrawWifiDetails();
}

void PrepareTFT_VoltageOneBank()
{
    // Assumes the Mutex is already obtained by caller

    tft.fillScreen(TFT_BLACK);
    // We have a single bank/pack
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);

    int16_t w = tft.width();
    // Take off the wifi banner height
    int16_t h = tft.height() - fontHeight_2 - 100;

    const int16_t xoffset = 32;

    tft.setTextFont(2);
    // Need to think about multilingual strings in the longer term
    tft.drawString("Bank voltage", xoffset + 0, 0);
    tft.drawString("External temp", xoffset + 0, h);
    tft.drawString("Module temp", xoffset + w / 2, h);
    tft.drawString("Cell voltage", xoffset + 0, 44 + h);
    tft.drawString("Modules balancing", xoffset + w / 2, 44 + h);

    TFTDrawWifiDetails();
}

void PageForward()
{
    //"Right" touch or delay counter has expired
    _ScreenPageCounter++;

    if (_ScreenPageCounter == 1 && mysettings.currentMonitoringEnabled == false)
    {
        // Don't show current if its not fitted/installed
        // Skip to next page
        _ScreenPageCounter++;
    }

    if (_ScreenPageCounter == 2 && mysettings.currentMonitoringEnabled == false)
    {
        // Don't show current if its not fitted/installed
        // Skip to next page
        _ScreenPageCounter++;
    }

    if (_ScreenPageCounter > 3)
    {
        // Loop back to first page
        _ScreenPageCounter = 0;
    }

    // Trigger a refresh of the screen
    if (updatetftdisplay_task_handle != NULL)
    {
        xTaskNotify(updatetftdisplay_task_handle, 0, eNotifyAction::eSetValueWithOverwrite);
    }
}

void PageBackward()
{
    //"Left" touch or delay counter has expired
    _ScreenPageCounter--;

    if (_ScreenPageCounter == 2 && mysettings.currentMonitoringEnabled == false)
    {
        // Don't show current if its not fitted/installed
        // Skip to next page
        _ScreenPageCounter--;
    }

    if (_ScreenPageCounter == 1 && mysettings.currentMonitoringEnabled == false)
    {
        // Don't show current if its not fitted/installed
        // Skip to next page
        _ScreenPageCounter--;
    }

    if (_ScreenPageCounter < 0)
    {
        // Loop back to last page
        _ScreenPageCounter = 3;
    }

    // Trigger a refresh of the screen
    if (updatetftdisplay_task_handle != NULL)
    {
        xTaskNotify(updatetftdisplay_task_handle, 0, eNotifyAction::eSetValueWithOverwrite);
    }
}

// This gets called by the "periodic" task in main.cpp, every second.
void IncreaseDelayCounter()
{
    _ScreenToDisplayDelay++;

    // Switch pages if rotation delay exceeded
    // 15 seconds between pages
    if (_ScreenToDisplayDelay > 15)
    {
        // Move to the next page after the "_ScreenToDisplayDelay" delay
        _ScreenToDisplayDelay = 0;
        PageForward();
    }
}

// Determine what screen to show on the TFT based on priority/severity
ScreenTemplateToDisplay WhatScreenToDisplay()
{
    ScreenTemplateToDisplay reply = ScreenTemplateToDisplay::None;

    // Forced screen depending on mode/errors etc.
    if (!_tft_screen_available)
    {
        return ScreenTemplateToDisplay::NotInstalled;
    }
    else if (_avrsettings.inProgress || _avrsettings.programmingModeEnabled)
    {
        return ScreenTemplateToDisplay::AVRProgrammer;
    }
    else if (_controller_state != ControllerState::Running)
    {
        return ScreenTemplateToDisplay::State;
    }
    else if (rules.numberOfActiveErrors > 0)
    {
        return ScreenTemplateToDisplay::Error;
    }

    switch (_ScreenPageCounter)
    {
    case 0:
        // Voltage page
        if (mysettings.totalNumberOfBanks == 1)
        {
            reply = ScreenTemplateToDisplay::VoltageOneBank;
        }
        else
        {
            reply = ScreenTemplateToDisplay::VoltageFourBank;
        }
        break;
    case 1:
        // Show the current monitor
        reply = ScreenTemplateToDisplay::CurrentMonitor;
        break;
    case 2:
        // Show the current monitor SoC
        reply = ScreenTemplateToDisplay::SoCBarGraph;
        break;
    case 3:
        // System Information
        reply = ScreenTemplateToDisplay::SystemInformation;
        break;
    default:
        reply = ScreenTemplateToDisplay::None;
        break;
    }

    if (reply != _lastScreenToDisplay)
    {
        // Its a new type of screen/page, so reset count
        _ScreenToDisplayDelay = 0;
    }

    return reply;
}
void tftsleep()
{
    if (!_tft_screen_available)
        return;

    // Switch screen off
    hal.TFTScreenBacklight(false);
    _screen_awake = false;
    _lastScreenToDisplay = ScreenTemplateToDisplay::None;
    _ScreenToDisplayDelay = 0;
    ESP_LOGI(TAG, "TFT switched off");
}

void init_tft_display()
{
    if (!_tft_screen_available)
        return;

    ESP_LOGD(TAG, "Configure TFT display");

    tft.init();
    tft.initDMA(); // Initialise the DMA engine (tested with STM32F446 and STM32F767)
    tft.getSPIinstance().setHwCs(false);
    tft.setRotation(3);

    fontHeight_2 = tft.fontHeight(2);
    fontHeight_4 = tft.fontHeight(4);

    PrepareTFT_ControlState();
    DrawTFT_ControlState();

    hal.TFTScreenBacklight(true);
}

// This task switches on/off the TFT screen, and triggers a redraw of its contents
void tftwakeup(TimerHandle_t)
{
    // Use parameter to force a refresh (used when realtime events occur like wifi disconnect)
    if (_tft_screen_available)
    {
        if (_screen_awake && _interrupt_triggered > 0)
        {
            // If the screen is already awake, take a reading of the touch position
            // the first touch simply wakes up the screen, second touch can drive an action
            _lastTouch = hal.TouchScreenUpdate();

            // Screen is already awake, so can we process a touch command?
            // ESP_LOGD(TAG, "touched=%u, pressure=%u, X=%u, Y=%u", _lastTouch.touched, _lastTouch.pressure, _lastTouch.X, _lastTouch.Y);

            // X range is 0-4096
            if (_lastTouch.touched && _lastTouch.X < 1000)
            {
                ESP_LOGD(TAG, "Touched LEFT");
                PageBackward();
            }
            else if (_lastTouch.touched && _lastTouch.X > 3000)
            {
                ESP_LOGD(TAG, "Touched RIGHT");
                PageForward();
            }
        }

        if (_screen_awake == false || force_tft_wake == true)
        {
            ESP_LOGI(TAG, "Wake up screen");
            _screen_awake = true;

            // Always start on the same screen/settings
            ResetScreenSequence();

            hal.TFTScreenBacklight(true);
        }

        // Trigger a refresh of the screen
        if (updatetftdisplay_task_handle != nullptr)
        {
            xTaskNotify(updatetftdisplay_task_handle, force_tft_wake ? 1 : 0, eNotifyAction::eSetValueWithOverwrite);
        }
    }

    // Reset force flag value
    force_tft_wake = false;

    // Allow interrupt to trigger again
    _interrupt_triggered = 0;
}

void DrawTFT_ControlState()
{
    TFTDrawWifiDetails();

    tft.setTextColor(TFT_WHITE, SplashLogoPalette[3]);
    tft.setTextFont(4);
    uint16_t x = tft.width() / 2;
    uint16_t y = tft.height() - 72;
    // Centre/middle text
    tft.setTextDatum(TC_DATUM);

    switch (_controller_state)
    {
    case ControllerState::Running:
    {
        tft.drawCentreString("Running", x, y, 4);
        break;
    }
    case ControllerState::Unknown:
    {
        tft.drawCentreString("???", x, y, 4);
        break;
    }
    case ControllerState::PowerUp:
    {
        tft.drawCentreString("Powering up...", x, y, 4);
        break;
    }
    case ControllerState::Stabilizing:
    {
        tft.drawCentreString("Waiting for modules...", x, y, 4);
        break;
    }
    } // end switch
}

void PrepareTFT_CurrentMonitor()
{
    tft.fillScreen(TFT_BLACK);

    int16_t w = tft.width();
    // Take off the wifi banner height
    int16_t h = tft.height() - fontHeight_2;
    // int16_t yhalfway = h / 2;

    int16_t y_row0 = 0;
    int16_t y_row1 = h / 3;
    int16_t y_row2 = y_row1 * 2;

    // Grid lines
    tft.drawLine(w / 2, 0, w / 2, h, TFT_DARKGREY);
    tft.drawLine(0, y_row1, w, y_row1, TFT_DARKGREY);
    tft.drawLine(0, y_row2, w, y_row2, TFT_DARKGREY);
    tft.drawLine(0, h, w, h, TFT_DARKGREY);

    // Skip over horizontal line
    y_row1 += 2;
    y_row2 += 2;

    tft.setTextFont(2);
    // Need to think about multilingual strings in the longer term
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    tft.drawString("Current (A)", 0, y_row0);
    tft.drawString("Power (W)", 2 + (w / 2), y_row0);
    tft.drawString("Voltage", 0, y_row1);
    tft.drawString("State of charge %", 2 + (w / 2), y_row1);
    tft.drawString("Amp/hour Out", 0, y_row2);
    tft.drawString("Amp/hour In", 2 + (w / 2), y_row2);
    TFTDrawWifiDetails();
}

void PrepareTFT_SocBarGraph()
{
    tft.fillScreen(TFT_BLACK);

    int16_t w = tft.width();
    // Take off the wifi banner height
    int16_t h = tft.height() - fontHeight_2;
    int16_t yhalfway = h / 2;
    int16_t xhalfway = w / 2;

    tft.drawCentreString("State of Charge %", xhalfway, 10, 4);

    tft.drawRoundRect(xhalfway - 102, yhalfway - 26, 204, 52, 4, TFT_GREEN);
    tft.drawRoundRect(xhalfway - 103, yhalfway - 27, 2 + 204, 2 + 52, 4, TFT_GREEN);

    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);

    // The bar graph
    int16_t SoC = (int16_t)currentMonitor.stateofcharge;

    if (SoC > 100)
    {
        SoC = 100;
    }

    tft.fillRectHGradient(xhalfway - 100, yhalfway - 22, 200, 44, TFT_RED, TFT_GREEN);

    if (SoC != 100)
    {
        // Clear between SoC and 100%
        tft.fillRect((xhalfway - 100) + (2 * SoC), yhalfway - 22, 200 - (2 * SoC), 44, TFT_BLACK);
    }

    // Stripe lines
    for (int16_t i = (xhalfway - 94); i < (xhalfway + 94); i += 6)
    {
        tft.fillRect(i, yhalfway - 22, 2, 44, TFT_BLACK);
    }

    // Single bank, large font
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setTextDatum(TC_DATUM);
    tft.setTextFont(7);
    tft.drawFloat(currentMonitor.stateofcharge, 1, xhalfway, yhalfway + 55);

    TFTDrawWifiDetails();
}

void DrawTFT_SoCBarGraph()
{
    // Do nothing here - screen refreshes on page change
}

void DrawTFT_CurrentMonitor()
{
    int16_t w = tft.width();
    // Take off the wifi banner height
    int16_t h = tft.height() - fontHeight_2;
    // int16_t yhalfway = h / 2;
    int16_t half_w = w / 2;

    int16_t y_row0 = 0;
    int16_t y_row1 = h / 3;
    int16_t y_row2 = y_row1 * 2;

    // Skip over horizontal line and font titles
    y_row0 += 0 + fontHeight_2;
    y_row1 += 2 + fontHeight_2;
    y_row2 += 2 + fontHeight_2;

    // Position of text in X axis
    int16_t middle_x = 2 + w / 2;

    tft.setTextColor(TFT_GREEN, TFT_BLACK);

    // Top left
    tft.setTextDatum(TL_DATUM);
    tft.setTextFont(7);
    // Skip over title
    int16_t y = y_row0;
    int16_t x = 0;

    uint8_t decimals = 2;

    if (currentMonitor.modbus.current > 99)
    {
        decimals = 1;
    }

    x += tft.drawFloat(currentMonitor.modbus.current, decimals, x, y);
    // Clear out surrounding background (to black)
    tft.fillRect(x, y, half_w - x, tft.fontHeight(), TFT_BLACK);

    y = y_row1;
    x = middle_x;
    x += tft.drawFloat(currentMonitor.stateofcharge, 1, x, y);
    x += tft.drawString("%", x, y);
    // Clear out surrounding background (to black)
    tft.fillRect(x, y, w - x, tft.fontHeight(), TFT_BLACK);

    decimals = 2;
    if (currentMonitor.modbus.voltage > 99)
    {
        decimals = 1;
    }
    y = y_row1;
    x = 0;
    x += tft.drawFloat(currentMonitor.modbus.voltage, decimals, x, y);
    tft.fillRect(x, y, half_w - x, tft.fontHeight(), TFT_BLACK);

    y = y_row0;
    x = middle_x;
    x += tft.drawNumber(currentMonitor.modbus.power, x, y);
    tft.fillRect(x, y, w - x, tft.fontHeight(), TFT_BLACK);

    y = y_row2;
    x = 0;
    decimals = 1;
    tft.setTextFont(4);
    float ahout = (float)currentMonitor.modbus.milliamphour_out / 1000.0;
    if (ahout < 100)
    {
        decimals = 2;
    }
    x += tft.drawFloat(ahout, decimals, x, y);
    tft.fillRect(x, y, half_w - x, tft.fontHeight(), TFT_BLACK);

    // Amp hour in
    y = y_row2;
    x = middle_x;
    decimals = 1;
    float ahin = (float)currentMonitor.modbus.milliamphour_in / 1000.0;

    if (ahin < 100)
    {
        decimals = 2;
    }
    x += tft.drawFloat(ahin, decimals, x, y);
    tft.fillRect(x, y, w - x, tft.fontHeight(), TFT_BLACK);
}

void PrepareTFT_SystemInfo()
{
    tft.fillScreen(TFT_BLACK);

    int16_t w = tft.width();
    // Take off the wifi banner height
    int16_t h = tft.height() - fontHeight_2;
    int16_t yhalfway = h / 2;

    int16_t column0 = 0;
    int16_t column1 = w / 3;
    int16_t column2 = column1 * 2;

    int16_t row0 = 0;
    int16_t row1 = h / 4;
    int16_t row2 = row1 * 2;
    int16_t row3 = row1 * 3;

    // Grid lines
    tft.drawLine(column1, 0, column1, row3, TFT_DARKGREY);
    tft.drawLine(column2, 0, column2, row3, TFT_DARKGREY);
    tft.drawLine(column0, row1, w, row1, TFT_DARKGREY);
    tft.drawLine(column0, row2, w, row2, TFT_DARKGREY);
    tft.drawLine(column0, row3, w, row3, TFT_DARKGREY);

    column1 += 2;
    column2 += 2;
    row1 += 2;
    row2 += 2;
    row3 += 2;

    tft.setTextFont(2);
    // Need to think about multilingual strings!
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    tft.drawString("Packets sent", column0, row0);
    tft.drawString("Packets rec'd", column1, row0);
    tft.drawString("Round trip (ms)", column2, row0);

    tft.drawString("Error - OOS", column0, row1);
    tft.drawString("Error - CRC", column1, row1);
    tft.drawString("Error - Ignored", column2, row1);

    tft.drawString("CAN - Sent", column0, row2);
    tft.drawString("CAN - Received", column1, row2);
    tft.drawString("CAN - Send fail", column2, row2);

    tft.drawString("Uptime", column0, row3);

    TFTDrawWifiDetails();
}

void DrawTFT_SystemInfo()
{
    // Split screen for multiple banks, maximum of 4 banks on the display

    int16_t w = tft.width();
    int16_t h = tft.height() - fontHeight_2;
    int16_t halfway = h / 2;

    int16_t column0 = 0;
    int16_t column1 = 2 + (w / 3);
    int16_t column2 = 2 + 2 * (w / 3);

    int16_t row0 = 2 + fontHeight_2;
    int16_t row1 = 2 + fontHeight_2 + (h / 4);
    int16_t row2 = 2 + fontHeight_2 + 2 * (h / 4);
    int16_t row3 = 2 + fontHeight_2 + 3 * (h / 4);

    int16_t x = 0;

    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setTextFont(4);
    x += tft.drawNumber(prg.packetsGenerated, column0, row0);
    x += tft.drawNumber(receiveProc.packetsReceived, column1, row0);
    x += tft.drawNumber(receiveProc.packetTimerMillisecond, column2, row0);

    x += tft.drawNumber(receiveProc.totalOutofSequenceErrors, column0, row1);
    x += tft.drawNumber(receiveProc.totalCRCErrors, column1, row1);
    x += tft.drawNumber(receiveProc.totalNotProcessedErrors, column2, row1);

    x += tft.drawNumber(canbus_messages_sent, column0, row2);
    x += tft.drawNumber(canbus_messages_received, column1, row2);
    x += tft.drawNumber(canbus_messages_failed_sent, column2, row2);

    uint32_t uptime = (uint32_t)(esp_timer_get_time() / (uint64_t)1e+6);

    std::string uptime_string;
    uptime_string.reserve(20);
    uptime_string.append(std::to_string(uptime / (3600 * 24))).append("d ");
    uptime_string.append(std::to_string(uptime % (3600 * 24) / 3600)).append("h ");
    uptime_string.append(std::to_string(uptime % 3600 / 60)).append("m ");
    uptime_string.append(std::to_string(uptime % 60)).append("s");

    x += tft.drawString(uptime_string.c_str(), column0, row3);
}

void PrepareTFT_VoltageFourBank()
{
    tft.fillScreen(TFT_BLACK);

    int16_t w = tft.width();
    // Take off the wifi banner height
    int16_t h = tft.height() - fontHeight_2 - 68;
    int16_t yhalfway = h / 2;

    // Grid lines
    tft.drawLine(w / 2, 0, w / 2, h, TFT_DARKGREY);

    tft.drawLine(0, yhalfway, w, yhalfway, TFT_DARKGREY);

    tft.drawLine(0, h, w, h, TFT_DARKGREY);

    tft.setTextFont(2);
    // Need to think about multilingual strings in the longer term
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    tft.drawString("Bank voltage", 0, 0);
    tft.drawString("External temp", 0, h + 2);
    tft.drawString("Module temp", 2 + w / 2, h + 2);
    tft.drawString("Cell voltages", 0, fontHeight_2 + fontHeight_2 + h + 2);
    tft.drawString("Modules balancing", 2 + w / 2, fontHeight_2 + fontHeight_2 + h + 2);

    uint8_t banks = mysettings.totalNumberOfBanks > 4 ? 4 : mysettings.totalNumberOfBanks;

    for (uint8_t i = 0; i < banks; i++)
    {
        int16_t y = 0;
        int16_t x = 0;

        if (i == 1 || i == 3)
        {
            x += 4 + w / 2;
        }

        if (i > 1)
        {
            y += 2 + yhalfway;
        }

        // Need to think about multilingual strings in the longer term
        x += tft.drawString("Bank voltage ", x, y);
        x += tft.drawNumber(i, x, y);
    }

    TFTDrawWifiDetails();
}

void DrawTFT_VoltageFourBank()
{
    // Split screen for multiple banks, maximum of 4 banks on the display

    int16_t w = tft.width();
    int16_t h = tft.height() - fontHeight_2 - 68;
    int16_t halfway = h / 2;

    uint8_t banks = mysettings.totalNumberOfBanks > 4 ? 4 : mysettings.totalNumberOfBanks;

    for (uint8_t i = 0; i < banks; i++)
    {
        // ESP_LOGD(TAG, "Drawing bank %u", i);

        // We could probably do this with tft.setViewport...

        tft.setTextDatum(TL_DATUM);

        int16_t y = 1 + fontHeight_2;
        int16_t x = 0;

        // Half way across screen, minus vertical line
        int16_t limitx = (w / 2) - 1;

        if (i == 1 || i == 3)
        {
            x += 4 + w / 2;
            limitx = w;
        }

        if (i > 1)
        {
            y += halfway;
        }

        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.setTextFont(7);
        float value = rules.bankvoltage.at(i) / 1000.0F;
        x += tft.drawFloat(value, 2, x, y);

        // Clear right hand side of display
        tft.fillRect(x, y, limitx - x, tft.fontHeight(), TFT_BLACK);
    }

    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    // Tiny font
    tft.setTextFont(2);

    int16_t x = 0;
    int16_t y = fontHeight_2 + h + 2;
    if (rules.moduleHasExternalTempSensor)
    {
        x += tft.drawNumber(rules.lowestExternalTemp, x, y);
        x += tft.drawString(" / ", x, y);
        x += tft.drawNumber(rules.highestExternalTemp, x, y);
    }
    else
    {
        x += tft.drawString("Not fitted", x, y);
    }
    // blank out gap between numbers
    tft.fillRect(x, y, (w / 2) - 1 - x, fontHeight_2, TFT_BLACK);

    x = 2 + w / 2;
    y = fontHeight_2 + h + 2;
    x += tft.drawNumber(rules.lowestInternalTemp, x, y);
    x += tft.drawString(" / ", x, y);
    x += tft.drawNumber(rules.highestInternalTemp, x, y);
    tft.fillRect(x, y, w - x, fontHeight_2, TFT_BLACK);

    x = 0;
    y = fontHeight_2 + fontHeight_2 + fontHeight_2 + h + 2;
    float value = rules.lowestCellVoltage / 1000.0;
    x += tft.drawFloat(value, 3, x, y);
    x += tft.drawString(" / ", x, y);
    value = rules.highestCellVoltage / 1000.0;
    x += tft.drawFloat(value, 3, x, y);
    tft.fillRect(x, y, (w / 2) - 1 - x, fontHeight_2, TFT_BLACK);

    x = 2 + w / 2;
    y = fontHeight_2 + fontHeight_2 + fontHeight_2 + h + 2;
    x += tft.drawNumber(rules.numberOfBalancingModules, x, y);
    tft.fillRect(x, y, w - x, fontHeight_2, TFT_BLACK);
}

void DrawTFT_VoltageOneBank()
{
    // Single bank, large font
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    // Large FONT 8 - 75 pixel high (only numbers available)
    // Top centre
    tft.setTextDatum(TC_DATUM);
    tft.setTextFont(8);

    const int16_t xoffset = 32;
    int16_t y = fontHeight_2;
    int16_t x = tft.width() / 2;
    float value = rules.bankvoltage.at(0) / 1000.0F;
    x += tft.drawFloat(value, 2, x, y);
    // Clear right hand side of display
    tft.fillRect(x, y, tft.width() - x, tft.fontHeight(), TFT_BLACK);

    // Top left
    tft.setTextDatum(TL_DATUM);
    tft.setTextFont(4);

    // Cell temperatures and stuff
    int16_t h = tft.height() - fontHeight_2 - 100;

    y = h + fontHeight_2;
    x = xoffset + 0;
    if (rules.moduleHasExternalTempSensor)
    {
        x += tft.drawNumber(rules.lowestExternalTemp, x, y);
        x += tft.drawString(" / ", x, y);
        x += tft.drawNumber(rules.highestExternalTemp, x, y);
    }
    else
    {
        x += tft.drawString("Not fitted", x, y);
    }
    // blank out gap between numbers
    tft.fillRect(x, y, (tft.width() / 2) - x, fontHeight_4, TFT_BLACK);

    x = xoffset + tft.width() / 2;
    y = h + fontHeight_2;
    x += tft.drawNumber(rules.lowestInternalTemp, x, y);
    x += tft.drawString(" / ", x, y);
    x += tft.drawNumber(rules.highestInternalTemp, x, y);
    // blank out gap between numbers
    tft.fillRect(x, y, tft.width() - x, fontHeight_4, TFT_BLACK);

    // Cell voltage ranges
    y = h + fontHeight_4 + fontHeight_2 + fontHeight_2 + 2;
    x = xoffset + 0;
    value = rules.lowestCellVoltage / 1000.0;
    x += tft.drawFloat(value, 3, x, y);
    x += tft.drawString(" / ", x, y);
    value = rules.highestCellVoltage / 1000.0;
    x += tft.drawFloat(value, 3, x, y);
    // blank out gap between numbers
    tft.fillRect(x, y, tft.width() / 2 - x, fontHeight_4, TFT_BLACK);

    y = h + fontHeight_4 + fontHeight_2 + fontHeight_2 + 2;
    x = xoffset + tft.width() / 2;
    x += tft.drawNumber(rules.numberOfBalancingModules, x, y);
    // blank out gap between numbers
    tft.fillRect(x, y, tft.width() - x, fontHeight_4, TFT_BLACK);
}

void DrawTFT_NoWiFi()
{
    tft.setTextColor(TFT_WHITE, TFT_RED);
    // Centre screen
    tft.setTextFont(2);
    uint16_t x = tft.width() / 2;
    uint16_t y = tft.height() / 2 - fontHeight_4 * 2;
    // Centre/middle text
    tft.setTextDatum(TC_DATUM);

    tft.drawCentreString("Module", x, y, 4);
    y += fontHeight_4;
    tft.drawCentreString("communications", x, y, 4);
    y += fontHeight_4;
    tft.drawCentreString("error", x, y, 4);
}

void DrawTFT_Error()
{
    tft.setTextColor(TFT_WHITE, TFT_RED);

    for (size_t i = 0; i < rules.ErrorCodes.size(); i++)
    {
        if (rules.ErrorCodes.at(i) != InternalErrorCode::NoError)
        {
            // Centre screen
            tft.setTextFont(2);
            uint16_t x = tft.width() / 2;
            uint16_t y = tft.height() / 2 - fontHeight_4 * 2;
            // Centre/middle text
            tft.setTextDatum(TC_DATUM);

            switch (rules.ErrorCodes.at(i))
            {
            case InternalErrorCode::CommunicationsError:
            {
                tft.drawCentreString("Module or RS485", x, y, 4);
                y += fontHeight_4;
                tft.drawCentreString("communications", x, y, 4);
                y += fontHeight_4;
                tft.drawCentreString("error", x, y, 4);
                break;
            }
            case InternalErrorCode::ModuleCountMismatch:
            {
                tft.drawCentreString("Module count", x, y, 4);
                y += fontHeight_4;
                tft.drawCentreString("mismatch", x, y, 4);
                break;
            }
            case InternalErrorCode::TooManyModules:
            {
                tft.drawCentreString("Too many", x, y, 4);
                y += fontHeight_4;
                tft.drawCentreString("modules", x, y, 4);
                break;
            }
            case InternalErrorCode::WaitingForModulesToReply:
            {
                tft.drawCentreString("Waiting for", x, y, 4);
                y += fontHeight_4;
                tft.drawCentreString("modules to", x, y, 4);
                y += fontHeight_4;
                tft.drawCentreString("reply", x, y, 4);
                break;
            }
            case InternalErrorCode::ZeroVoltModule:
            {
                tft.drawCentreString("Module returned", x, y, 4);
                y += fontHeight_4;
                tft.drawCentreString("zero volt", x, y, 4);
                y += fontHeight_4;
                tft.drawCentreString("reading", x, y, 4);
                break;
            }
            case InternalErrorCode::ControllerMemoryError:
            {
                tft.drawCentreString("Controller", x, y, 4);
                y += fontHeight_4;
                tft.drawCentreString("memory error", x, y, 4);
                break;
            }
            case InternalErrorCode::ErrorEmergencyStop:
            {
                tft.drawCentreString("Emergency", x, y, 4);
                y += fontHeight_4;
                tft.drawCentreString("STOP", x, y, 4);
                break;
            }
            default:
                tft.drawCentreString("Error with", x, y, 4);
                y += fontHeight_4;
                tft.drawCentreString("no text", x, y, 4);
                break;
            }

            // Only show first error
            break;
        }
    }
}

void PrepareTFT_AVRProgrammer()
{
    tft.fillScreen(TFT_BLACK);

    tft.setTextFont(2);
    uint16_t x = tft.width() / 2;
    uint16_t y = 32;
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(TFT_BLACK, TFT_LIGHTGREY);

    tft.fillRoundRect(16, 16, tft.width() - 32, tft.height() - 48, 8, TFT_LIGHTGREY);
    tft.drawRoundRect(16 + 2, 16 + 2, tft.width() - 36, tft.height() - 52, 8, TFT_DARKGREY);
    tft.drawCentreString("AVR", x, y, 4);
    y += fontHeight_4;
    tft.drawCentreString("Programming", x, y, 4);

    y += fontHeight_4 + fontHeight_2;
    tft.drawCentreString(_avrsettings.filename, x, y, 2);

    y = 140;
    x = 58;
    tft.drawRect(x, y, 202, 20, TFT_DARKGREY);
    tft.drawRect(x + 1, y + 1, 202 - 2, 20 - 2, TFT_DARKGREY);
}

void tftdisplay_avrprogrammer_progress(uint8_t programingMode, size_t current, size_t maximum)
{
    // programingMode=0 for programming, 1=verify
    if (_tft_screen_available)
    {
        // ESP_LOGD(TAG, "%u %u", maximum, current);

        uint16_t percent = round(((float)current / (float)maximum) * (float)100);

        uint32_t color = programingMode == 0 ? TFT_BLUE : TFT_DARKGREEN;

        if (hal.GetDisplayMutex())
        {
            uint16_t x = 60;
            uint16_t y = 140 + 2;
            tft.fillRect(x, y, 2 * percent, 16, color);
            hal.ReleaseDisplayMutex();
        }
    }
}

void updatetftdisplay_task(void *param)
{
    for (;;)
    {
        uint32_t force = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (_tft_screen_available && (_screen_awake == true || force == 1))
        {
            ESP_LOGD(TAG, "Update TFT display");

            // Set default to top left
            tft.setTextDatum(TL_DATUM);

            ScreenTemplateToDisplay screenToDisplay = WhatScreenToDisplay();

            // Force will "force" a redraw of the full screen regardless
            // of previous state.

            // Step 1: Prepare the screen layout (boxes, bars, titles etc.)
            //         stuff which doesn't change (static)
            if (_lastScreenToDisplay != screenToDisplay || force == 1)
            {
                if (hal.GetDisplayMutex())
                {
                    // The screen type has changed, so prepare the background
                    switch (screenToDisplay)
                    {
                    case ScreenTemplateToDisplay::NotInstalled:
                    case ScreenTemplateToDisplay::None:
                        break;
                    case ScreenTemplateToDisplay::AVRProgrammer:
                        PrepareTFT_AVRProgrammer();
                        break;
                    case ScreenTemplateToDisplay::Error:
                        PrepareTFT_Error();
                        break;
                    case ScreenTemplateToDisplay::VoltageOneBank:
                        PrepareTFT_VoltageOneBank();
                        break;
                    case ScreenTemplateToDisplay::VoltageFourBank:
                        PrepareTFT_VoltageFourBank();
                        break;
                    case ScreenTemplateToDisplay::State:
                        PrepareTFT_ControlState();
                        break;
                    case ScreenTemplateToDisplay::CurrentMonitor:
                        PrepareTFT_CurrentMonitor();
                        break;
                    case ScreenTemplateToDisplay::SoCBarGraph:
                        PrepareTFT_SocBarGraph();
                        break;
                    case ScreenTemplateToDisplay::SystemInformation:
                        PrepareTFT_SystemInfo();
                        break;
                    }
                    hal.ReleaseDisplayMutex();
                }
                _lastScreenToDisplay = screenToDisplay;
            }

            // Step2: Draw the variable screen layouts
            if (hal.GetDisplayMutex())
            {
                switch (screenToDisplay)
                {
                case ScreenTemplateToDisplay::AVRProgrammer:
                    // Update is done via call back, so don't do anything here
                    break;
                case ScreenTemplateToDisplay::NotInstalled:
                    break;
                case ScreenTemplateToDisplay::None:
                    break;
                case ScreenTemplateToDisplay::State:
                    DrawTFT_ControlState();
                    break;
                case ScreenTemplateToDisplay::Error:
                    DrawTFT_Error();
                    break;
                case ScreenTemplateToDisplay::VoltageOneBank:
                    DrawTFT_VoltageOneBank();
                    break;
                case ScreenTemplateToDisplay::VoltageFourBank:
                    DrawTFT_VoltageFourBank();
                    break;
                case ScreenTemplateToDisplay::CurrentMonitor:
                    DrawTFT_CurrentMonitor();
                    break;
                case ScreenTemplateToDisplay::SoCBarGraph:
                    DrawTFT_SoCBarGraph();
                    break;
                case ScreenTemplateToDisplay::SystemInformation:
                    DrawTFT_SystemInfo();
                    break;
                }

                hal.ReleaseDisplayMutex();
            } // endif mutex
        }
    } // end for
}