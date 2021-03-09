
/*
  ____  ____  _  _  ____  __  __  ___
 (  _ \(_  _)( \/ )(  _ \(  \/  )/ __)
  )(_) )_)(_  \  /  ) _ < )    ( \__ \
 (____/(____) (__) (____/(_/\/\_)(___/

DIYBMS V4.0

(c)2019-2021 Stuart Pittaway

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

#include "defines.h"
#include "HAL_ESP32.h"
#include <WiFi.h>
#include <esp_wifi.h>
#include <SPI.h>

#include "TFT_eSPI.h"
TFT_eSPI tft = TFT_eSPI();

#include "Rules.h"

#include "tft.h"
#include "tft_splash_image.h"

bool _tft_screen_available = false;
volatile bool _screen_awake = false;

ScreenTemplateToDisplay _lastScreenToDisplay = ScreenTemplateToDisplay::NotInstalled;
int16_t fontHeight_2;
int16_t fontHeight_4;

void IRAM_ATTR TFTScreenTouchInterrupt()
{
    if (!_tft_screen_available)
        return;

    if (_screen_awake)
    {
        ESP_LOGD(TAG, "Ignored touch, screen already on");
    }
    else
    {
        ESP_LOGD(TAG, "Touch");
        xTaskNotifyFromISR(tftwakeup_task_handle, 0x00, eNotifyAction::eNoAction, pdFALSE);
    }
}

void SwitchTFTBacklight(bool value)
{
    if (!_tft_screen_available)
        return;
    //Queue up i2c message
    i2cQueueMessage m;
    //4 = TFT backlight LED
    m.command = 0x04;
    //Lowest 3 bits are RGB led GREEN/RED/BLUE
    m.data = value;
    xQueueSendToBack(queue_i2c, &m, 10 / portTICK_PERIOD_MS);
}

void TFTDrawWifiDetails()
{
    tft.setTextDatum(TL_DATUM);

    int16_t y = tft.height() - fontHeight_2;
    ;
    tft.fillRect(0, y, tft.width(), tft.height() - y, TFT_DARKGREY);
    tft.setTextFont(2);
    tft.setTextColor(TFT_BLACK, TFT_DARKGREY);
    int16_t x = 2;

    if (WiFi.isConnected())
    {
        x += tft.drawString(WiFi.getHostname(), x, y);
        x += 10;
        x += tft.drawString(WiFi.localIP().toString(), x, y);

        //Draw RSSI on bottom right corner
        //Received Signal Strength in dBm
        x += 10;
        x += tft.drawNumber(WiFi.RSSI(), x, y);
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

        //Draw the time in bottom right corner of screen
        int16_t y = tft.height() - fontHeight_2;
        ;
        int16_t x = tft.width() - 38;
        if (timeinfo.tm_hour < 10)
        {
            x += tft.drawString("0", x, y);
        }
        x += tft.drawNumber(timeinfo.tm_hour, x, y);
        x += tft.drawString(":", x, y);
        if (timeinfo.tm_min < 10)
        {
            x += tft.drawString("0", x, y);
        }
        x += tft.drawNumber(timeinfo.tm_min, x, y);
    }
}

void PrepareTFT_Error()
{
    //Assumes the Mutex is already obtained by caller
    tft.fillScreen(TFT_BLACK);
    // Errors have priority, draw filled red box
    tft.fillRoundRect(16, 16, tft.width() - 32, tft.height() - 48, 8, TFT_RED);

    // White outline
    tft.drawRoundRect(16 + 2, 16 + 2, tft.width() - 36, tft.height() - 52, 8, TFT_LIGHTGREY);

    TFTDrawWifiDetails();
}

void PrepareTFT_ControlState()
{
    //Assumes the Mutex is already obtained by caller
    tft.fillScreen(TFT_BLACK);

    tft.setTextColor(TFT_WHITE, TFT_DARKCYAN);
    tft.setTextFont(2);
    uint16_t x = tft.width() / 2;
    uint16_t y = tft.height() / 2 - fontHeight_4 * 2;
    //Centre/middle text
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
    case ControllerState::ConfigurationSoftAP:
    {
        // Errors have priority, draw filled red box
        tft.fillRoundRect(16, 16, tft.width() - 32, tft.height() - 48, 8, TFT_DARKCYAN);
        tft.drawRoundRect(16 + 2, 16 + 2, tft.width() - 36, tft.height() - 52, 8, TFT_WHITE);
        tft.drawCentreString("Configure", x, y, 4);
        y += fontHeight_4;
        tft.drawCentreString("Access", x, y, 4);
        y += fontHeight_4;
        tft.drawCentreString("Point", x, y, 4);
        break;
    }
    case ControllerState::PowerUp:
    case ControllerState::Stabilizing:
    {
        // Draw box in same colour as background of logo/image
        tft.fillRoundRect(16, 16, tft.width() - 32, tft.height() - 48, 8, SplashLogoPalette[0]);
        // White border
        tft.drawRoundRect(16 + 2, 16 + 2, tft.width() - 36, tft.height() - 52, 8, TFT_WHITE);

        TFT_eSprite spr = TFT_eSprite(&tft);
        spr.setColorDepth(4);
        spr.createSprite(SplashLogoGraphic_Width, SplashLogoGraphic_Height);
        spr.pushImage(0, 0,
                      SplashLogoGraphic_Width,
                      SplashLogoGraphic_Height,
                      (uint16_t *)SplashLogoGraphic);

        spr.createPalette(SplashLogoPalette, 16);
        spr.pushSprite(tft.width() / 2 - SplashLogoGraphic_Width / 2, 24);
        spr.deleteSprite();

        y = 85;
        tft.setTextColor(TFT_WHITE, SplashLogoPalette[0]);
        tft.drawCentreString("Version:", x, y, 2);
        y += fontHeight_2;
        ;
        tft.setTextColor(TFT_YELLOW, SplashLogoPalette[0]);
        tft.drawCentreString(GIT_VERSION_SHORT, x, y, 2);
        y += fontHeight_2;
        ;
        tft.setTextColor(TFT_WHITE, SplashLogoPalette[0]);
        tft.drawCentreString("Build Date:", x, y, 2);
        y += fontHeight_2;
        ;
        tft.setTextColor(TFT_YELLOW, SplashLogoPalette[0]);
        tft.drawCentreString(COMPILE_DATE_TIME_SHORT, x, y, 2);
        y += fontHeight_2;
        ;

        break;
    }

    } //end switch

    TFTDrawWifiDetails();
}

void PrepareTFT_VoltageOneBank()
{
    //Assumes the Mutex is already obtained by caller

    tft.fillScreen(TFT_BLACK);
    //We have a single bank/pack
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    tft.setTextFont(2);
    //Need to think about multilingual strings in the longer term
    tft.drawString("Bank voltage", 0, 0);
    tft.drawString("External temp", 0, tft.height() / 2);
    tft.drawString("Module temp", tft.width() / 2, tft.height() / 2);
    tft.drawString("Cell voltage", 0, 44 + tft.height() / 2);
    tft.drawString("Modules balancing", tft.width() / 2, 44 + tft.height() / 2);

    TFTDrawWifiDetails();
}

//Determine what screen to show on the TFT based on priority/severity
ScreenTemplateToDisplay WhatScreenToDisplay()
{
    if (!_tft_screen_available)
        return ScreenTemplateToDisplay::NotInstalled;

    if (rules.numberOfActiveErrors > 0)
        return ScreenTemplateToDisplay::Error;

    if (mysettings.totalNumberOfBanks == 1 && _controller_state == ControllerState::Running)
        return ScreenTemplateToDisplay::VoltageOneBank;

    if (mysettings.totalNumberOfBanks > 1 && _controller_state == ControllerState::Running)
        return ScreenTemplateToDisplay::VoltageFourBank;

    if (_controller_state != ControllerState::Running)
        return ScreenTemplateToDisplay::State;

    return ScreenTemplateToDisplay::None;
}

void init_tft_display()
{
    if (!_tft_screen_available)
        return;

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

//Put the TFT to sleep after a set period of time
void tftsleep_task(void *param)
{
    for (;;)
    {
        //Wait until this task is triggered
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        //Now we go back to sleep to provide a delay
        //Delay 2 minutes, or forever if an error is active
        do
        {
            vTaskDelay(30000 / portTICK_PERIOD_MS);
            vTaskDelay(30000 / portTICK_PERIOD_MS);
            vTaskDelay(30000 / portTICK_PERIOD_MS);
            vTaskDelay(30000 / portTICK_PERIOD_MS);
        } while (WhatScreenToDisplay() == ScreenTemplateToDisplay::Error);

        //Switch screen back off
        SwitchTFTBacklight(false);
        _screen_awake = false;
        _lastScreenToDisplay = ScreenTemplateToDisplay::None;
    }
}
//This task switches on/off the TFT screen, and triggers a redraw of its contents
void tftwakeup_task(void *param)
{
    for (;;)
    {
        //Wait until this task is triggered
        uint32_t force = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        //Use parameter to force a refresh (used when realtime events occur like wifi disconnect)
        if (_tft_screen_available)
        {

            if (!_screen_awake || force == 1)
            {
                ESP_LOGI(TAG, "Wake up screen");

                if (hal.GetDisplayMutex())
                {
                    //Fill screen with a grey colour, to let user know
                    //we have responded to touch (may may be a few seconds until the display task runs)
                    tft.fillScreen(TFT_LIGHTGREY);
                    hal.ReleaseDisplayMutex();
                }

                _screen_awake = true;
                SwitchTFTBacklight(true);

                xTaskNotify(tftsleep_task_handle, 0, eNotifyAction::eNoAction);
            }

            //Trigger a refresh of the screen
            xTaskNotify(updatetftdisplay_task_handle, force, eNotifyAction::eSetValueWithOverwrite);
        }
    }
}

void DrawTFT_ControlState()
{
    TFTDrawWifiDetails();

    tft.setTextColor(TFT_WHITE, SplashLogoPalette[0]);
    tft.setTextFont(4);
    uint16_t x = tft.width() / 2;
    uint16_t y = tft.height() - 64;
    //Centre/middle text
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
    case ControllerState::ConfigurationSoftAP:
    {
        //Don't draw anything
        break;
    }
    case ControllerState::PowerUp:
    {
        tft.drawCentreString("Powering up...", x, y, 4);
        break;
    }
    case ControllerState::Stabilizing:
    {
        tft.drawCentreString("Stabilizing...", x, y, 4);

        break;
    }
    } //end switch
}

void PrepareTFT_VoltageFourBank()
{
    //Assumes the Mutex is already obtained by caller
    //Show the first 4 banks in a grid
    tft.fillScreen(TFT_BLACK);
    //Grid lines
    tft.drawLine(tft.width() / 2, 0, tft.width() / 2, tft.height(), TFT_DARKGREY);
    tft.drawLine(0, tft.height() / 2, tft.width(), tft.height() / 2, TFT_DARKGREY);

    TFTDrawWifiDetails();
}

void DrawTFT_VoltageFourBank()
{
    //Split screen for multiple banks
    for (uint8_t i = 0; i < mysettings.totalNumberOfBanks; i++)
    {
        //Smaller font
        tft.setCursor(0, 0, 7);
        float value = rules.packvoltage[i] / 1000.0;
        tft.printf("%2.2fV", value);
    }
}

void DrawTFT_VoltageOneBank()
{
    //Single bank, large font
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    //Large FONT 8 - 75 pixel high (only numbers available)
    //Top centre
    tft.setTextDatum(TC_DATUM);
    tft.setTextFont(8);

    int16_t y = fontHeight_2;
    int16_t x = tft.width() / 2;
    float value = rules.packvoltage[0] / 1000.0;
    x += tft.drawFloat(value, 2, x, y);
    //Clear right hand side of display
    tft.fillRect(x, y, tft.width() - x, tft.fontHeight(), TFT_BLACK);

    //Top left
    tft.setTextDatum(TL_DATUM);
    tft.setTextFont(4);

    //Cell temperatures
    y = 16 + tft.height() / 2;
    x = 0;
    x += tft.drawNumber(rules.lowestExternalTemp, x, y);
    x += tft.drawString(" / ", x, y);
    x += tft.drawNumber(rules.highestExternalTemp, x, y);
    //blank out gap between numbers
    tft.fillRect(x, y, (tft.width() / 2) - x, fontHeight_4, TFT_BLACK);

    x = tft.width() / 2;
    y = fontHeight_2 + tft.height() / 2;
    x += tft.drawNumber(rules.lowestInternalTemp, x, y);
    x += tft.drawString(" / ", x, y);
    x += tft.drawNumber(rules.highestInternalTemp, x, y);
    //blank out gap between numbers
    tft.fillRect(x, y, tft.width() - x, fontHeight_4, TFT_BLACK);

    //Cell voltage ranges
    y = fontHeight_2 + 44 + tft.height() / 2;
    x = 0;
    value = rules.lowestCellVoltage / 1000.0;
    x += tft.drawFloat(value, 3, x, y);
    x += tft.drawString(" / ", x, y);
    value = rules.highestCellVoltage / 1000.0;
    x += tft.drawFloat(value, 3, x, y);
    //blank out gap between numbers
    tft.fillRect(x, y, tft.width() / 2 - x, fontHeight_4, TFT_BLACK);

    y = fontHeight_2 + 44 + tft.height() / 2;
    x = tft.width() / 2;
    x += tft.drawNumber(rules.numberOfBalancingModules, x, y);
    //blank out gap between numbers
    tft.fillRect(x, y, tft.width() - x, fontHeight_4, TFT_BLACK);
}

void DrawTFT_Error()
{
    tft.setTextColor(TFT_WHITE, TFT_RED);
    //uint16_t y = 16 + 6;

    for (size_t i = 0; i < sizeof(rules.ErrorCodes); i++)
    {
        if (rules.ErrorCodes[i] != InternalErrorCode::NoError)
        {
            //Centre screen
            tft.setTextFont(2);
            uint16_t x = tft.width() / 2;
            uint16_t y = tft.height() / 2 - fontHeight_4 * 2;
            //Centre/middle text
            tft.setTextDatum(TC_DATUM);

            switch (rules.ErrorCodes[i])
            {
            case InternalErrorCode::CommunicationsError:
            {
                tft.drawCentreString("Module", x, y, 4);
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
            default:
                tft.drawCentreString("Error with", x, y, 4);
                y += fontHeight_4;
                tft.drawCentreString("no text", x, y, 4);
                break;
            }

            //Only show first error
            break;
        }
    }
}

void tftdisplay_avrprogrammer_start()
{
    if (_tft_screen_available)
    {
        SwitchTFTBacklight(true);

        if (hal.GetDisplayMutex())
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

            y = 140;
            x = 58;
            tft.drawRect(x, y, 202, 20, TFT_DARKGREY);
            tft.drawRect(x + 1, y + 1, 202 - 2, 20 - 2, TFT_DARKGREY);

            hal.ReleaseDisplayMutex();
        }

        _lastScreenToDisplay = ScreenTemplateToDisplay::AVRProgrammer;
    }
}

void tftdisplay_avrprogrammer_progress(uint8_t programingMode, size_t current, size_t maximum)
{
    //programingMode=0 for programming, 1=verify
    if (_tft_screen_available)
    {
        //ESP_LOGD(TAG, "%u %u", maximum, current);

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

void tftdisplay_avrprogrammer_stop()
{
    if (_tft_screen_available)
    {
        SwitchTFTBacklight(false);
    }
    _screen_awake = false;
}

void updatetftdisplay_task(void *param)
{
    for (;;)
    {
        uint32_t force = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (_tft_screen_available && _screen_awake)
        {
            //Set default to top left
            tft.setTextDatum(TL_DATUM);

            ScreenTemplateToDisplay screenToDisplay = WhatScreenToDisplay();

            if (_lastScreenToDisplay != screenToDisplay || force == 1)
            {
                if (hal.GetDisplayMutex())
                {
                    //The screen type has changed, so prepare the background
                    switch (screenToDisplay)
                    {
                    case ScreenTemplateToDisplay::AVRProgrammer:
                    case ScreenTemplateToDisplay::NotInstalled:
                    case ScreenTemplateToDisplay::None:
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
                    }
                    hal.ReleaseDisplayMutex();
                }
                _lastScreenToDisplay = screenToDisplay;
            }

            if (hal.GetDisplayMutex())
            {
                switch (screenToDisplay)
                {
                case ScreenTemplateToDisplay::AVRProgrammer:
                case ScreenTemplateToDisplay::NotInstalled:
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
                }

                hal.ReleaseDisplayMutex();
            } //endif mutex
        }
    } //end for
}