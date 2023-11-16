
#ifndef DIYBMS_TFT_H_
#define DIYBMS_TFT_H_

#include "PacketRequestGenerator.h"
#include "PacketReceiveProcessor.h"

/*
#define USER_SETUP_LOADED
#define USE_DMA_TO_TFT
// Color depth has to be 16 bits if DMA is used to render image
#define COLOR_DEPTH 16
#define ILI9341_DRIVER
//#define SPI_FREQUENCY 40000000
//#define SPI_READ_FREQUENCY 20000000
//#define SPI_TOUCH_FREQUENCY 2500000
//#define LOAD_GLCD  // Font 1. Original Adafruit 8 pixel font needs ~1820 bytes in FLASH
//#define LOAD_FONT2 // Font 2. Small 16 pixel high font, needs ~3534 bytes in FLASH, 96 characters
//#define LOAD_FONT4 // Font 4. Medium 26 pixel high font, needs ~5848 bytes in FLASH, 96 characters
//#define LOAD_FONT6 // Font 6. Large 48 pixel font, needs ~2666 bytes in FLASH, only characters 1234567890:-.apm
//#define LOAD_FONT7 // Font 7. 7 segment 48 pixel font, needs ~2438 bytes in FLASH, only characters 1234567890:.
//#define LOAD_FONT8 // Font 8. Large 75 pixel font needs ~3256 bytes in FLASH, only characters 1234567890:-.
//#define LOAD_GFXFF // FreeFonts. Include access to the 48 Adafruit_GFX free fonts FF1 to FF48 and custom fonts
//#define SMOOTH_FONT
*/

enum ScreenTemplateToDisplay : uint8_t
{
  NotInstalled = 0,
  None = 1,
  Error = 2,
  VoltageOneBank = 3,
  VoltageFourBank = 4,
  State = 5,
  AVRProgrammer=6,
  CurrentMonitor=7,
  SoCBarGraph=8,
  SystemInformation=9
};


void tftwakeup_task(void *param);
void updatetftdisplay_task(void *param);
void TFTDrawWifiDetails();
void PrepareTFT_Error();
void PrepareTFT_VoltageFourBank();
void PrepareTFT_ControlState();
void PrepareTFT_VoltageOneBank();
void tftsleep();
void init_tft_display();
ScreenTemplateToDisplay WhatScreenToDisplay();
void SwitchTFTBacklight(bool value);
void IRAM_ATTR TFTScreenTouchInterrupt();
void DrawTFT_ControlState();
void DrawClock();
void PrepareTFT_AVRProgrammer();
void tftdisplay_avrprogrammer_progress(uint8_t programingMode,size_t current, size_t maximum);
void tftdisplay_avrprogrammer_stop();
void IncreaseDelayCounter();
void PageForward();
void PageBackward();
void ResetScreenSequence();


//I hate EXTERN....
extern Rules rules;
extern diybms_eeprom_settings mysettings;
extern HAL_ESP32 hal;
extern ControllerState _controller_state;
extern TaskHandle_t updatetftdisplay_task_handle;
extern avrprogramsettings _avrsettings;
extern currentmonitoring_struct currentMonitor;
extern bool wifi_isconnected;
extern std::string hostname;
extern std::string ip_string;
extern PacketRequestGenerator prg;
extern PacketReceiveProcessor receiveProc;
extern uint32_t canbus_messages_received;
extern uint32_t canbus_messages_sent;
extern uint32_t canbus_messages_failed_sent;


#endif