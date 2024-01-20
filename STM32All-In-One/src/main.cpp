/*
____  ____  _  _  ____  __  __  ___
(  _ \(_  _)( \/ )(  _ \(  \/  )/ __)
)(_) )_)(_  \  /  ) _ < )    ( \__ \
(____/(____) (__) (____/(_/\/\_)(___/

DIYBMS V4

V490 UP TO 16S CELL MONITORING MODULES

STM32F030K6T6

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
*/

#define RX_BUFFER_SIZE 64

#include <Arduino.h>

extern "C"
{
#include "stm32_flash.h"
  void SystemClock_Config(void);
}

#include <IWatchdog.h>

#include "SPI.h"
#include <SerialEncoder.h>
#include "packet_processor.h"

// Our project code includes
#include "defines.h"
#include "cell.h"
#include <SerialEncoder.h>
#include "packet_processor.h"

#if !defined(HAL_UART_MODULE_ENABLED)
#error Expected HAL_UART_MODULE_ENABLED
#endif

// Two unused jumpers which could be used to enable features/comms address etc.
const auto ADDRESS0_JP3 = PA11;

const auto AFE_EN = PB0;
const auto RELAY = PB1;
const auto AFE_CS = PB5;
const auto ADC_CS_CNVST = PB6;
const auto SAMPLE_AFE = PB4;
const auto THERM_ENABLE = PA8;
const auto THERM_J11_T1 = PA_4;
const auto THERM_J12_T2 = PA_0;
const auto THERM_J13_T3 = PA_1;
const auto BALANCE_DETECT = PA12;
const auto FAN = PB3;

// Prototype functions...
[[noreturn]] void ErrorFlashes(int number);
uint32_t takeRawMCP33151ADCReading();
uint32_t MAX14921Command(uint8_t b1, uint8_t b2, uint8_t b3);
void DecimateRawADCCellVoltage(std::array<uint32_t, 16> &rawADC, CellData &cd, uint8_t numberCells);
void TakeExternalTempMeasurements(CellData &cd);
uint16_t DecimateValue(uint64_t val);
void EnableThermistorPower();
void DisableThermistorPower();
int16_t ReadThermistor(uint32_t);
int16_t ReadTH();
uint32_t MAX14921Command(uint16_t b1_2, uint8_t b3);
uint16_t CalculateCellBalanceRequirement(CellData &cd, uint8_t number_of_active_cells);

uint32_t fan_timer = 0;
uint32_t relay_timer = 0;

/// @brief Global variable to delay balance function on first power up
uint8_t waitbeforebalance = 20;

/// @brief Global variable to hold cell level data as its collected
CellData celldata{};

/// @brief Number of cells actually in use by the MAX chip (between 4 and 16)
uint8_t number_of_active_cells = 0;

uint8_t SerialPacketReceiveBuffer[8 + sizeof(PacketStruct)];

/// @brief Serial encoder
SerialEncoder myPacketSerial;

PacketProcessor PP;

/// @brief  Bitmap of which cells are balancing (maps into MAX chip register)
uint16_t cell_balancing = 0;

/// @brief   Is serial baud rate scanning active?
bool serialBaudScanning = true;
/// @brief  Index to current SerialBaudRates array
int8_t serialBaudIndex = 0;
/// @brief  Wait X iterations of loop() before swapping baud rates
int8_t serialBaudCountDown = 15;

// Baud rates that we can use
constexpr std::array<uint16_t, 4> SerialBaudRates = {10000, 9600, 5000, 2400};

const uint32_t LEVEL_SHIFTING_DELAY_MAX = 50; // μs
const uint32_t T_SETTLING_TIME_MAX = 10;      // μs

// Command byte 1
#define AFE_CB1 bit(7)
#define AFE_CB2 bit(6)
#define AFE_CB3 bit(5)
#define AFE_CB4 bit(4)
#define AFE_CB5 bit(3)
#define AFE_CB6 bit(2)
#define AFE_CB7 bit(1)
#define AFE_CB8 bit(0)

// Command byte 2
#define AFE_CB9 bit(7)
#define AFE_CB10 bit(6)
#define AFE_CB11 bit(5)
#define AFE_CB12 bit(4)
#define AFE_CB13 bit(3)
#define AFE_CB14 bit(2)
#define AFE_CB15 bit(1)
#define AFE_CB16 bit(0)
// Command byte 3
//  Enable Cell Selection
#define AFE_ECS bit(7)
// Selects the cell for voltage readout during hold phase
#define AFE_SC0 bit(6)
#define AFE_SC1 bit(5)
#define AFE_SC2 bit(4)
#define AFE_SC3 bit(3)
// 0: Device in sample phase if SAMPL input is logic-high, 1: Device in hold phase
// Not used by this code - using SAMPLE pin instead
#define AFE_SMPLB bit(2)
// Diagnostic enable (normally zero)
#define AFE_DIAG bit(1)
// Low-power mode enabled (normally zero)
#define AFE_LOPW bit(0)

#define BUFFER_CALIBRATION AFE_SC0

#define ANALOG_BUFFERED_T1 AFE_SC0 | AFE_SC2 | AFE_SC3
#define ANALOG_BUFFERED_T2 AFE_SC1 | AFE_SC2 | AFE_SC3
#define ANALOG_BUFFERED_T3 AFE_SC0 | AFE_SC1 | AFE_SC2 | AFE_SC3

#define ANALOG_CELL1 0
#define ANALOG_CELL2 AFE_SC0
#define ANALOG_CELL3 AFE_SC1
#define ANALOG_CELL4 AFE_SC0 | AFE_SC1
#define ANALOG_CELL5 AFE_SC2
#define ANALOG_CELL6 AFE_SC0 | AFE_SC2
#define ANALOG_CELL7 AFE_SC1 | AFE_SC2
#define ANALOG_CELL8 AFE_SC0 | AFE_SC1 | AFE_SC2
#define ANALOG_CELL9 AFE_SC3
#define ANALOG_CELL10 AFE_SC0 | AFE_SC3
#define ANALOG_CELL11 AFE_SC1 | AFE_SC3
#define ANALOG_CELL12 AFE_SC0 | AFE_SC1 | AFE_SC3
#define ANALOG_CELL13 AFE_SC2 | AFE_SC3
#define ANALOG_CELL14 AFE_SC0 | AFE_SC2 | AFE_SC3
#define ANALOG_CELL15 AFE_SC1 | AFE_SC2 | AFE_SC3
#define ANALOG_CELL16 AFE_SC0 | AFE_SC1 | AFE_SC2 | AFE_SC3

constexpr std::array<uint8_t, 16> CellTable =
    {
        AFE_ECS | ANALOG_CELL1,
        AFE_ECS | ANALOG_CELL2,
        AFE_ECS | ANALOG_CELL3,
        AFE_ECS | ANALOG_CELL4,
        AFE_ECS | ANALOG_CELL5,
        AFE_ECS | ANALOG_CELL6,
        AFE_ECS | ANALOG_CELL7,
        AFE_ECS | ANALOG_CELL8,
        AFE_ECS | ANALOG_CELL9,
        AFE_ECS | ANALOG_CELL10,
        AFE_ECS | ANALOG_CELL11,
        AFE_ECS | ANALOG_CELL12,
        AFE_ECS | ANALOG_CELL13,
        AFE_ECS | ANALOG_CELL14,
        AFE_ECS | ANALOG_CELL15,
        AFE_ECS | ANALOG_CELL16};

extern "C"
{
  void SystemClock_Config(void)
  {
    RCC_OscInitTypeDef RCC_OscInitStruct = {};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {};

    /* Initializes the RCC Oscillators according to the specified parameters in the RCC_OscInitTypeDef structure.  */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Initializes the CPU, AHB and APB buses clocks  */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
      Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* Enables the Clock Security System */
    HAL_RCC_EnableCSS();
  }
}

uint32_t SPITransfer24(uint8_t byte1, uint8_t byte2, uint8_t byte3)
{
  uint32_t data;

  data = SPI.transfer(byte1);
  data <<= 8;

  data |= SPI.transfer(byte2);
  data <<= 8;

  data |= SPI.transfer(byte3);

  return data;
}

/// @brief Query MAX14921 chip (analog front end=AFE) and determine its ready status
/// @return number of connected battery cells (up to 16)
uint8_t queryAFE()
{
  uint8_t cellCount;
  uint8_t DeviceIsNotReady = 1;
  uint8_t countDown = 20;

  while (countDown > 0)
  {
    // MAX14921
    digitalWrite(SAMPLE_AFE, HIGH);              // Sample cell voltages (all 16 cells at same time)
    delay(60);                                   // Wait 60ms for capacitors to fill and equalize against cell voltages
    digitalWrite(SAMPLE_AFE, LOW);               // Disable sampling - HOLD mode
    delayMicroseconds(LEVEL_SHIFTING_DELAY_MAX); // Wait to settle

    // Tell AFE to switch AOUT channel to zero, all balancing off
    auto reply = MAX14921Command(0, 0, CellTable.at(0));

    // Bad reply - chip not enabled or connected?
    if (reply == 0)
    {
      ErrorFlashes(7);
    }

    DeviceIsNotReady = (reply & B00000010) >> 1;

    // Should be ZERO = MAX14921, 2= MAX14920
    // uint8_t productIdentifier = (reply & B11000000) >> 6;
    // uint8_t dieVersion = (reply & B00110000) >> 4;

    // 1: UV_VA = VA is below UV_VAVTH (4.7V)
    // VA (Analog Supply Voltage) below UV_VAVTH (4.7V)
    if (((reply & B00001000) >> 3) != 0)
    {
      ErrorFlashes(4);
    }
    // 1: VP is below UV_VPVTH (6V). If LOPW = 1, VP UVLO circuit is disabled and this bit is always set to 1
    if (((reply & B00000100) >> 2) != 0)
    {
      ErrorFlashes(5);
    }

    // thermal shutdown
    if ((reply & B00000001) != 0)
    {
      ErrorFlashes(2);
    }

    // Calculate how many cells are valid (based on reply from MAX chip)
    // Start with 16 cells
    cellCount = 16;
    uint16_t cells = (reply & 0x00FFFF00) >> 8;
    // Subtract 1 from cell count for each bit thats "1" (out of voltage range)
    // bit ="1" if under voltage of UV_VCVTH or over 5V
    while (cells)
    {
      cellCount -= cells & 1;
      cells >>= 1;
    }

    if (DeviceIsNotReady == 0)
    {
      // Everything is good - exit loop
      break;
    }

    // Loop again if needed until device is ready
    delay(250);
    countDown--;
  }

  // Device not ready after 5 seconds
  if (countDown == 0)
  {
    ErrorFlashes(6);
  }

  return cellCount;
}

/// @brief Loop through battery cells and take ADC measurement and store the raw (un-decimated) value in an array
/// @param cellCount number of cells to sample
/// @param rawADC array to hold output result
void ADCSampleCellVoltages(uint8_t cellCount, std::array<uint32_t, 16> &rawADC)
{
  // Scan active cell voltages (starting at highest)
  for (int8_t cellid = (cellCount - 1); cellid >= 0; cellid--)
  {
    MAX14921Command(0, CellTable.at(cellid));
    // Delay required for AOUT to settle
    delayMicroseconds(T_SETTLING_TIME_MAX);
    // Read cell voltage
    rawADC.at(cellid) = takeRawMCP33151ADCReading();
  }
}

/// @brief Calibrate internal op-amp buffer
void BufferAmplifierOffsetCalibration()
{
  // MAX14921
  digitalWrite(SAMPLE_AFE, HIGH); // Sample mode

  // Parasitic Capacitance Charge Injection Error Calibration
  MAX14921Command(0, BUFFER_CALIBRATION);

  // Takes 8ms to complete
  delay(8);
}

void NotificationLedOn()
{
  digitalWrite(PB7, HIGH);
}
void NotificationLedOff()
{
  digitalWrite(PB7, LOW);
}

void configureModules()
{
  CellModuleConfig config;

  // Default values for cells
  float Calibration = 1.0F;
  uint8_t BypassTemperatureSetPoint = 55;
  uint16_t BypassThresholdmV = 4250;
  uint16_t relay_range = 5;
  uint16_t relay_minimummv = 3400;
  int16_t fanswitchontemperature = BypassTemperatureSetPoint - 8;

  uint16_t RunAwayCellMinimumVoltage = 3400;
  uint16_t RunAwayCellDifferential = 75;

  flash_copy_buffer((uint8_t *)&config, sizeof(CellModuleConfig));
  // If we have a FLASH configuration, use that...
  if (config.magicnumber == 0x1234 && config.sizeofconfig == sizeof(CellModuleConfig))
  {
    Calibration = config.Calibration;
    BypassTemperatureSetPoint = config.BypassTemperatureSetPoint;
    BypassThresholdmV = config.BypassThresholdmV;
    relay_range = config.relay_range;
    relay_minimummv = config.relay_minimummv;
    fanswitchontemperature = config.fanswitchontemperature;
    RunAwayCellMinimumVoltage = config.RunAwayCellMinimumVoltage;
    RunAwayCellDifferential = config.RunAwayCellDifferential;
  }

  // These values are shared/static between cell instances
  Cell::setCalibration(Calibration);
  Cell::setBypassTemperatureSetPoint(BypassTemperatureSetPoint);
  Cell::setBypassThresholdmV(BypassThresholdmV);
  Cell::setRelayMinmV(relay_minimummv);
  Cell::setRelayRange(relay_range);
  Cell::setFanSwitchOnTemperature(fanswitchontemperature);

  PP.setRunAwayCellMinimumVoltage(RunAwayCellMinimumVoltage);
  PP.setRunAwayCellDifferential(RunAwayCellDifferential);

  // All cells excluding ZERO are prevented from having config changes made in this all-in-one board setup
  for (auto i = 1; i < celldata.size(); i++)
  {
    celldata.at(i).disableChanges();
  }
}

void onPacketReceived()
{
  // diyBMSHAL::EnableSerial0TX();

  // A data packet has just arrived, process it and forward the results to the
  // next module

  if (PP.onPacketReceived((PacketStruct *)SerialPacketReceiveBuffer, number_of_active_cells, celldata))
  {
    // Only light Notification if packet is good
    NotificationLedOn();
  }

  // Send the packet (fixed length!) (even if it was invalid so controller can count crc errors)
  myPacketSerial.sendBuffer(SerialPacketReceiveBuffer);

  Serial1.flush();

  NotificationLedOff();

  if (PP.getSettingsHaveChanged())
  {
    // We need to update the settings stored in FLASH
    // As this hardware pretends to be a 16S module - we copy all the settings from cell zero, everything else is ignored/wiped out

    // Save new values into flash - important to align this buffer "aligned(8)" for correct write into FLASH
    CellModuleConfig config __attribute__((aligned(8))) = {
        0x1234,
        sizeof(CellModuleConfig),
        Cell::getBypassTemperatureSetPoint(),
        Cell::getBypassThresholdmV(),
        Cell::getCalibration(),
        Cell::getRelayRange(),
        Cell::getRelayMinmV(),
        Cell::getFanSwitchOnTemperature(),
        PP.getRunAwayCellMinimumVoltage(),
        PP.getRunAwayCellDifferential()};

    // This function uses the top 1K of FLASH memory to emulate EEPROM
    flash_erase_and_write((uint8_t *)&config, sizeof(config));

    configureModules();

    PP.clearSettingsHaveChanged();
  }
}
void configurePins()
{
  // LED
  pinMode(PB7, OUTPUT);
  NotificationLedOn();

  pinMode(SAMPLE_AFE, OUTPUT);
  pinMode(AFE_CS, OUTPUT);
  pinMode(ADC_CS_CNVST, OUTPUT);
  pinMode(THERM_ENABLE, OUTPUT);
  pinMode(RELAY, OUTPUT);
  pinMode(ADDRESS0_JP3, INPUT_FLOATING);
  pinMode(AFE_EN, OUTPUT);
  pinMode(THERM_J11_T1, INPUT_ANALOG);
  pinMode(THERM_J12_T2, INPUT_ANALOG);
  pinMode(THERM_J13_T3, INPUT_ANALOG);
  pinMode(FAN, OUTPUT);

  // RELAY off
  digitalWrite(RELAY, LOW);
  // FAN off
  digitalWrite(FAN, LOW);
  digitalWrite(ADC_CS_CNVST, HIGH);
  digitalWrite(AFE_CS, HIGH);

  // Track cell voltages (LOW=HOLD SAMPLE)
  digitalWrite(SAMPLE_AFE, HIGH);

  // STM32 use 12 bit ADC resolution
  analogReadResolution(12);
  // STM32 default voltage reference (3.3v)
  analogReference(AR_DEFAULT);
}

void setup()
{
  SystemClock_Config();

  // Initialize the IWDG with 4 seconds timeout.  This would cause a CPU reset if the IWDG timer is not reloaded in approximately 4 seconds.
  IWatchdog.begin(4000000);

  configurePins();
  DisableThermistorPower();

  // Check if balance board is fitted? (pin pulled up to 3.3v by daughter board)
  pinMode(BALANCE_DETECT, INPUT_PULLDOWN);
  PP.BalanceBoardInstalled = digitalRead(BALANCE_DETECT) == HIGH;
  // Disable pull down after checking
  pinMode(BALANCE_DETECT, INPUT_FLOATING);

  // Set up data handler
  Serial1.setTx(PA_9);
  Serial1.setRx(PA_10);
  // Start using the first serial baud rate (10k)
  Serial1.begin(SerialBaudRates.at(serialBaudIndex), SERIAL_8N1);
  myPacketSerial.begin(&Serial1, &onPacketReceived, sizeof(PacketStruct), SerialPacketReceiveBuffer, sizeof(SerialPacketReceiveBuffer));

  SPI.begin();

  // Restart MAX chip before we crack on...
  digitalWrite(AFE_EN, LOW);
  delay(100);
  digitalWrite(AFE_EN, HIGH);

  // Warm up ADC
  for (size_t i = 0; i < 128; i++)
  {
    takeRawMCP33151ADCReading();
  }

  // Sometimes on power up, the chip returns 1 cell more than actually connected, so take a few
  // samples and return the lowest cell count
  number_of_active_cells = 255;
  for (size_t i = 0; i < 4; i++)
  {
    auto x = queryAFE();

    if (x < number_of_active_cells)
    {
      number_of_active_cells = x;
    }
  }

  // We need at least 4 cells with correct voltages for this to work
  if (number_of_active_cells < 4)
  {
    ErrorFlashes(3);
  }

  BufferAmplifierOffsetCalibration();

  configureModules();

  NotificationLedOff();
}

uint32_t takeRawMCP33151ADCReading()
{
  /* ADC READING */
  // INPUT ACQUISITION PHASE (STANDBY), TOGGLE CONVERT START
  auto MCP33151settings = SPISettings(50000000, MSBFIRST, SPI_MODE0);

  // Take a reading and throw away - reset ADC accumulator to zero
  SPI.beginTransaction(MCP33151settings);
  digitalWrite(ADC_CS_CNVST, LOW);
  SPI.transfer(0xFF);
  SPI.transfer(0xFF);
  digitalWrite(ADC_CS_CNVST, HIGH);
  SPI.endTransaction();

  // Use MCP33151 Accumulator to over sample input voltage
  // Pulse CNVST for each sample we want to take and average
  // 256x over sample (last sample is taken by SPITransfer24 below)
  for (size_t i = 0; i < 256 - 1; i++)
  {
    digitalWrite(ADC_CS_CNVST, LOW);
    digitalWrite(ADC_CS_CNVST, HIGH);
    // This is probably not required...
    delayMicroseconds(1);
  }

  // Take a reading
  SPI.beginTransaction(MCP33151settings);
  // ADC conversion stops when CNVST goes LOW
  digitalWrite(ADC_CS_CNVST, LOW);
  auto val = SPITransfer24(0xFF, 0xFF, 0xFF);
  digitalWrite(ADC_CS_CNVST, HIGH);
  SPI.endTransaction();

  return val;
}

/// @brief Decimates an ADC reading after oversampling
/// @param val Takes in a raw ADC value which has been oversampled giving ENOB of 16.5-17 bits
/// @return Corrected value
uint16_t DecimateValue(uint64_t val)
{
  // decimate result by right shifting 2 places (raw ADC is 14 bits)
  val = val >> 2;
  // 4096 or 4500 voltage reference
  val = val * ((uint64_t)DIYBMSREFMILLIVOLT);
  // 4194304 = 2^22 effective resolution bits
  val = val / 4194304UL;

  return (uint16_t)val;
}

// Take the raw oversampled readings and decimate
void DecimateRawADCCellVoltage(std::array<uint32_t, 16> &rawADC, CellData &cells, uint8_t numberCells)
{
  for (int cellid = 0; cellid < numberCells; cellid++)
  {
    cells.at(cellid).setCellVoltage(DecimateValue(rawADC[cellid]));
  }
}

/// @brief Send a 24bit command to MAX14921 chip over SPI
/// @param b1_2 byte1 and 2 as uint16
/// @param b3 byte3
/// @return 24 bit answer
uint32_t MAX14921Command(uint16_t b1_2, uint8_t b3)
{
  uint8_t b1 = (b1_2 & 0xFF00) >> 8;
  uint8_t b2 = b1_2 & 0x00FF;
  return MAX14921Command(b1, b2, b3);
}

/// @brief Send a 24bit command to MAX14921 chip over SPI
/// @param b1 byte1
/// @param b2 byte2
/// @param b3 byte3
/// @return 24 bit answer
uint32_t MAX14921Command(uint8_t b1, uint8_t b2, uint8_t b3)
{
  // Tell AFE to switch AOUT channel to this cell for the ADC to measure
  const auto MAX14921SPISettings = SPISettings(40000000, MSBFIRST, SPI_MODE0);
  SPI.beginTransaction(MAX14921SPISettings);
  digitalWrite(AFE_CS, LOW);
  auto reply = SPITransfer24(b1, b2, b3);
  digitalWrite(AFE_CS, HIGH);
  SPI.endTransaction();
  return reply;
}

// NTC THERMISTOR CMFB103F3950FANT
// ADC MAPPING TO TEMPERATURE (DEGREES C)
constexpr std::array<int8_t, 256> thermistorTable =
    {
        -40, -40, -40, -40, -40, -40, -40, -40, -40, -39, -37, -36, -34, -33, -31, -30, -29,
        -28, -27, -26, -25, -24, -23, -22, -21, -20, -19, -19, -18, -17, -16, -16, -15, -14,
        -14, -13, -13, -12, -11, -11, -10, -10, -9, -9, -8, -8, -7, -6, -6, -5, -5, -4, -4,
        -4, -3, -3, -2, -2, -1, -1, 0, 0, 0, 1, 1, 2, 2, 3, 3, 3, 4, 4, 5, 5, 5, 6, 6, 7, 7,
        7, 8, 8, 8, 9, 9, 10, 10, 10, 11, 11, 11, 12, 12, 12, 13, 13, 14, 14, 14, 15, 15, 15,
        16, 16, 16, 17, 17, 17, 18, 18, 19, 19, 19, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23,
        24, 24, 24, 24, 25, 25, 26, 26, 26, 27, 27, 27, 28, 28, 28, 29, 29, 30, 30, 30, 31, 31,
        31, 32, 32, 33, 33, 33, 34, 34, 35, 35, 35, 36, 36, 37, 37, 37, 38, 38, 39, 39, 39, 40,
        40, 41, 41, 42, 42, 42, 43, 43, 44, 44, 45, 45, 46, 46, 47, 47, 48, 48, 49, 49, 50, 50,
        51, 51, 52, 52, 53, 54, 54, 55, 55, 56, 57, 57, 58, 59, 59, 60, 61, 61, 62, 63, 63, 64,
        65, 66, 67, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 80, 81, 82, 83, 85, 86, 88,
        89, 91, 93, 95, 97, 99, 101, 104, 107, 110, 113, 117, 120, 120, 120, 120, 120, 120, 120,
        120, 120, 120};

/// @brief Read external temperature sensors, connected to STM32 (12 bit ADC)
/// @return Temperature in celcius. A value of -999 means not-connected (shorted to ground)
int16_t ReadThermistor(uint32_t pin)
{
  auto value = (uint16_t)analogRead(pin);

  // Ignore extreme ranges - assume not connected
  if (value < 15 || value > 4000)
  {
    return (int16_t)-999;
  }

  // Scale 12 bit to 8 bit (TODO: we can probably just change the resolution of analogRead instead)
  auto byte_value = (uint8_t)map(value, 0, 4096, 0, 255);
  return thermistorTable.at(byte_value);
}

/// @brief Read external temperature sensors (connected to board J11/J12/J13 sockets)
/// @param cd Cell data array
void TakeExternalTempMeasurements(CellData &cd)
{
  // Set cell 0/1/2 to map to external temperature sensors (socket on pcb)
  cd.at(0).setExternalTemperature(ReadThermistor(THERM_J11_T1));
  cd.at(1).setExternalTemperature(ReadThermistor(THERM_J12_T2));
  cd.at(2).setExternalTemperature(ReadThermistor(THERM_J13_T3));
}

void EnableThermistorPower()
{
  digitalWrite(THERM_ENABLE, LOW);
}

void DisableThermistorPower()
{
  digitalWrite(THERM_ENABLE, HIGH);
}
/// @brief Read onboard thermistor sensor connected to the MCP33151
/// @return Celcius temperature reading
int16_t ReadTH()
{
  // 14 bit reply...
  auto value = DecimateValue(takeRawMCP33151ADCReading());
  // THx is connected to 3.3V max via 10K resistors - scale 3.3V to 4.096V reference
  // 3.600 is used as temperature appears to be over read by 2 celcius

  // Ignore extreme ranges - assume not connected
  if (value == 0)
  {
    return (int16_t)-999;
  }

  // Scale to 8 bit
  auto byte_value = (uint8_t)map(value, 0, long((3600.0F / ((float)DIYBMSREFMILLIVOLT)) * 4095.0F), 0, 255);
  return thermistorTable.at(byte_value);
}

/// @brief internal temperature sensors (on board)
/// @param cd Cell Data array
void TakeOnboardInternalTempMeasurements(CellData &cd)
{
  // internal temperature sensors on balance board
  if (PP.BalanceBoardInstalled)
  {
    // Read temperature sensors
    MAX14921Command(cell_balancing, ANALOG_BUFFERED_T1);
    auto t1 = ReadTH();
    MAX14921Command(cell_balancing, ANALOG_BUFFERED_T2);
    auto t2 = ReadTH();

    // Spread the two internal temperature sensor values across all 16 cells (even though some may not be connected)
    // Odd cells are on the left of the balance board, near TH1/T1, even on right side TH2/T2
    // but this doesn't matter, as we use the highest recorded temperature from either sensor in safety checks
    for (size_t i = 0; i < 16; i += 2)
    {
      cd.at(i).setInternalTemperature(t1);
      cd.at(i + 1).setInternalTemperature(t2);
    }
  }

  MAX14921Command(cell_balancing, ANALOG_BUFFERED_T3);
  auto t3 = ReadTH();

  // Cell 0 internal temperature is the on-board (PCB) sensor (marked TH6)
  cd.at(0).setInternalTemperature(t3);

  if (!PP.BalanceBoardInstalled)
  {
    // Populate all cells with an internal temperature to prevent controller error messages
    for (size_t i = 0; i < 16; i += 2)
    {
      cd.at(i).setInternalTemperature(t3);
      cd.at(i + 1).setInternalTemperature(t3);
    }
  }
}

[[noreturn]] void ErrorFlashes(int number)
{
  NotificationLedOff();
  delay(500);
  while (true)
  {
    // make sure the code in this loop is executed in less than 2 seconds to leave 50% headroom for the timer reload.
    IWatchdog.reload();

    for (size_t i = 0; i < number; i++)
    {
      NotificationLedOn();
      delay(220);
      NotificationLedOff();
      delay(220);
    }

    // make sure the code in this loop is executed in less than 2 seconds to leave 50% headroom for the timer reload.
    IWatchdog.reload();
    delay(2000);
  }
}

/// @brief Calculate bit pattern for cell passive balancing (MOSFET switches)
/// @param cd Cell data array
/// @param num_cells total number of cells
/// @param runawaycell_index index of cell identified as run away (or -1 if none)
/// @return bit pattern
uint16_t CalculateCellBalanceRequirement(CellData &cd, uint8_t num_cells, int runawaycell_index)
{
  // if balance daughter board is not installed, always return zero - no balance
  if (PP.BalanceBoardInstalled == false)
  {
    return 0;
  }

  uint16_t reply = 0;

  for (auto i = 0; i < num_cells; i++)
  {
    // Get reference to cell object
    Cell &cell = cd.at(i);

    // Check if bypass is needed, or if runawaycell is identified
    if (cell.BypassCheck() == true || runawaycell_index == i)
    {
      // Our cell voltage is OVER the voltage setpoint limit, start draining cell using bypass resistor
      cell.StartBypass();

      // Enable balancing bit pattern - this enables the MOSFET and balance resistor
      reply = reply | (uint16_t)(1U << (15 - i));
    }
    else
    {
      // We've just ended bypass....
      cell.StopBypass();
    }
  }
  return reply;
}

/// @brief Calculate various functions against cell voltages
/// @param cd Cell data
/// @param num_cells number of active cells
/// @param lowestmV (reference) Lowest cell voltage (millivolt)
/// @param highestmV (reference) Highest cell voltage (millivolt)
/// @param range (reference) Range between highest and lowest cell voltages (millivolt)
/// @param highest_index Cell index with the highest voltage
/// @param total Total voltage of all cells (millivolt)
void CalculateCellVoltageMinMaxAvg(CellData &cd,
                                   uint8_t num_cells,
                                   uint16_t &lowestmV,
                                   uint16_t &highestmV,
                                   uint16_t &range,
                                   uint8_t &highest_index,
                                   uint32_t &total)
{
  lowestmV = 0xFFFF;
  highestmV = 0;
  // Index of cell with highest voltage
  highest_index = -1;
  total = 0;

  // Determine highest/lowest cell and average voltage
  for (uint8_t i = 0; i < num_cells; i++)
  {
    // Get reference to cell object
    const Cell &cell = cd.at(i);
    auto v = cell.getCellVoltage();
    total += v;
    if (v < lowestmV)
    {
      lowestmV = v;
    }
    if (v > highestmV)
    {
      highestmV = v;
      highest_index = i;
    }
  }

  // Determine voltage differential/range (highest - lowest)
  range = highestmV - lowestmV;
}

/// @brief Determine what (if any) cells need balancing, also controls relay output
/// @param highestTemp Highest temperature of on-board sensors (balance temperature)
/// @return bit pattern for which MOSFETs need enabling
uint16_t DoCellBalancing(const int16_t highestTemp)
{
  uint16_t lowestmV;
  uint16_t highestmV;
  uint8_t highest_index;
  uint16_t range;
  uint32_t total;

  CalculateCellVoltageMinMaxAvg(celldata, number_of_active_cells, lowestmV, highestmV, range, highest_index, total);

  // Switch relay off
  if (HAL_GetTick() > relay_timer)
  {
    relay_timer = 0;
    digitalWrite(RELAY, LOW);
  }

  // Enable external relay outputs, if both range and minimum voltage have been met
  // Setup a timer, to enable this output for a minimum of 1 minute over-run after balance condition is no longer met
  if (range > Cell::getRelayRange() && highestmV > Cell::getRelayMinmV())
  {
    digitalWrite(RELAY, HIGH);
    // This might run into problems if the HAL_GetTick wraps around - approx. every 50 days
    relay_timer = HAL_GetTick() + 60000;
  }

  // Daughter board not installed - stop here, no balancing possible (or fan control)
  if (!PP.BalanceBoardInstalled)
  {
    return 0;
  }

  // Now check the temperatures to ensure we remain safe
  if (highestTemp > Cell::getBypassTemperatureSetPoint() || highestTemp > Cell::getSafetyTemperatureCutoff())
  {
    // Temperature is over limit (or safety limit)
    Cell::setOverTemperature(true);
    // Disable all MOSFETs, allow to cool
    return 0;
  }

  if (Cell::getOverTemperature() == true)
  {
    // We went over temperature, so disable balancing until temperature drops below Hysteresis value
    if (highestTemp > Cell::getBypassTemperatureHysteresis())
    {
      // Still too hot - Disable all MOSFETs, allow to cool
      return 0;
    }
    else
    {
      // Temperature dropped below limit, so return to normal, allow balancing
      Cell::setOverTemperature(false);
    }
  }

  // Calculate the average of all the cells
  auto averagemv = (uint16_t)(total / number_of_active_cells);

  // Calculate the differential between highest cell voltage and the average
  uint16_t highest_average_diff = highestmV - averagemv;

  // Now determine if we should balance the highest "run away" cell.
  // If the highest cell is above average cell voltage by X millivolts, then begin balancing until it no longer is.
  // Ensure the cell voltage is above a minimum - LIFEPO4 cells need to be over 3400mV for this function to be useful.
  auto runawaycellindex = (highestmV > PP.getRunAwayCellMinimumVoltage() && highest_average_diff > PP.getRunAwayCellDifferential()) ? highest_index : -1;

  // Should any cells require balancing - check if they have gone over the threshold
  return CalculateCellBalanceRequirement(celldata, number_of_active_cells, runawaycellindex);
}

// Checks the serial port for about 60ms and processes any requests
// auto checks for serial baud rate scanning
void ServiceSerialPort()
{
  // Service the serial port/queue
  for (size_t i = 0; i < 60; i++)
  {
    // Call update to receive, decode and process incoming packets.
    myPacketSerial.checkInputStream();

    // Allow data to be received in buffer (delay must be AFTER) checkInputStream
    delay(1);
  }

  if (serialBaudScanning)
  {
    NotificationLedOff();
    serialBaudCountDown--;

    if (PP.getPacketReceivedCounter() > 0)
    {
      // We have found our baud rate - and processed at least 1 packet successfully, so stop scanning
      serialBaudScanning = false;
    }
    else if (serialBaudCountDown <= 0)
    {
      // If we got this far, we have not yet found/received a valid packet of serial data, so try another baud rate
      serialBaudCountDown = 15;
      serialBaudIndex++;

      if (serialBaudIndex >= SerialBaudRates.size())
      {
        serialBaudIndex = 0;
      }

      Serial1.end();
      Serial1.begin(SerialBaudRates.at(serialBaudIndex), SERIAL_8N1);
    }
  }
}

void loop()
{

  // make sure the code in this loop is executed in less than 2 seconds to leave 50% headroom for the timer reload.
  IWatchdog.reload();

  // Temperature sensor readings...
  EnableThermistorPower();
  TakeExternalTempMeasurements(celldata);
  DisableThermistorPower();

  if (cell_balancing != 0)
  {
    // Switch off any bypass balancing before taking voltage readings
    MAX14921Command(0, ANALOG_BUFFERED_T1);
    // Allow cell voltages to bounce back
    delay(10);
  }

  digitalWrite(SAMPLE_AFE, HIGH); // Sample cell voltages (all 16 cells at same time)

  if (waitbeforebalance == 0)
  {
    // This also takes about 60ms, but allows request packets to be processed quicker than waiting in a blocking delay call
    ServiceSerialPort();
  }
  else
  {
    // Delay until we have been through this loop a few times (received a good packet)
    // so we don't return zero voltage readings to controller
    delay(60); // Wait 60ms for capacitors to fill and equalize against cell voltages (1uF caps)
  }

  // delay(60); // Wait 60ms for capacitors to fill and equalize against cell voltages (1uF caps)

  digitalWrite(SAMPLE_AFE, LOW);               // Disable sampling - HOLD mode
  delayMicroseconds(LEVEL_SHIFTING_DELAY_MAX); // Wait to settle

  // Array to hold raw ADC measurements (over sampled)
  std::array<uint32_t, 16> rawADC;
  rawADC.fill(0);
  // Scan active cell voltages (starting at highest cell)
  ADCSampleCellVoltages(number_of_active_cells, rawADC);

  // Once all cells are measured, we enable the sampling to resume...
  // this links the sample capacitors with the cells to equalise the voltages in them
  digitalWrite(SAMPLE_AFE, HIGH);

  // Read T1/T2/T3
  // This also re-enables balancing MOSFET if required....
  EnableThermistorPower();
  TakeOnboardInternalTempMeasurements(celldata);
  DisableThermistorPower();

  // Now process the ADC readings we have taken...
  DecimateRawADCCellVoltage(rawADC, celldata, number_of_active_cells);

  // Three temperature sensors are recorded over cells 0/1/2, use the highest value
  auto highestTemp = max(celldata.at(0).getInternalTemperature(), celldata.at(1).getInternalTemperature());
  highestTemp = max(celldata.at(2).getInternalTemperature(), highestTemp);

  // If fan timer has expired, switch off FAN
  // This has the side effect of the fan switching off and on (not physically noticable) should the temperature still be too hot.
  auto millis = HAL_GetTick();
  if (millis > fan_timer)
  {
    fan_timer = 0;
    digitalWrite(FAN, LOW);
  }

  // Enable fan if temperature is OVER Hysteresis value (target minus 5C), or FAN value, and set a 45 second over-run timer
  if (highestTemp > Cell::getBypassTemperatureHysteresis() || highestTemp > Cell::getFanSwitchOnTemperature())
  {
    digitalWrite(FAN, HIGH);
    fan_timer = millis + 45000;
    // Might have problems when HAL_GetTick rolls over - approx. 50days, fan could get stuck on
    // in this case, the fan_timer would have overflowed (unsigned) so would be less than millis.
    if (fan_timer < millis)
    {
      fan_timer = millis;
    }
  }

  // Don't immediately start balance on power up, wait a few cycles to settle
  if (waitbeforebalance == 0)
  {
    cell_balancing = DoCellBalancing(highestTemp);
    MAX14921Command(cell_balancing, ANALOG_BUFFERED_T1);
  }
  else
  {
    waitbeforebalance--;
  }

  // This needs to be below all other cell checking code
  ServiceSerialPort();

  // Every so often, we should call this to calibrate the op-amp as it changes in ambient temperature (takes 8ms to complete)
  if (PP.getPacketReceivedCounter() % 8192 == 0)
  {
    BufferAmplifierOffsetCalibration();
  }

  if (serialBaudScanning)
  {
    NotificationLedOn();
  }

  // Sleep for 800ms
  // TODO:ideally, this would be a true CPU sleep with RTC + UART wake up, but running out of FLASH code space
  uint16_t countdown = 400;
  while (Serial1.available() == 0 && countdown > 0)
  {
    delay(2);
    countdown--;
  }
}
