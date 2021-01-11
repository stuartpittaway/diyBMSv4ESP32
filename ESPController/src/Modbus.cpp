#include <Arduino.h>

#include "defines.h"
#include "HAL_ESP32.h"
#include <SDM.h>
#include "Modbus.h"

//SDM sdm(SERIAL_RS485, 9600, NOT_A_PIN, SERIAL_8N1, 16, 17);    //esp32 default pins for Serial1 => RX pin 16, TX pin 17
SDM sdm(SERIAL_RS485, 9600, NOT_A_PIN, SERIAL_8N1, RS485_RX, RS485_TX);      // pins for DIYBMS => RX pin 21, TX pin 22


ModbusInfo ModBus[MODBUS_NUM];
ModbusVal ModBusVal[MODBUS_NUM];

void setModbusName(int dev, char* cp){ strncpy(ModBus[dev].name, cp, MODBUS_NAME_LEN); }
void setModbusUnit(int dev, char* cp){ strncpy(ModBus[dev].unit, cp, MODBUS_UNIT_LEN); }
void setModbusDesc(int dev, char* cp){ strncpy(ModBus[dev].desc, cp, MODBUS_DESC_LEN); }

void setModbus(int dev, uint8_t addr, uint32_t min, uint32_t max, uint16_t reg, char *name, char* unit, char* desc) {

//  static int ModbusNum = 0;

  ModBus[dev].addr = addr;
  ModBus[dev].op = MB_READ_REGISTER;
  ModBus[dev].min = min;
  ModBus[dev].max = max;
  ModBus[dev].reg = reg;
/*
  strncpy(ModBus[dev].name, name, MODBUS_NAME_LEN);
  strncpy(ModBus[dev].unit, unit, MODBUS_UNIT_LEN);
  strncpy(ModBus[dev].desc, desc, MODBUS_DESC_LEN);
*/
  setModbusName(dev, name);
  setModbusUnit(dev, unit);
  setModbusDesc(dev, desc);

//  SERIAL_DEBUG.printf("%d %s %s %s\n", dev, (char*) ModBus[dev].name, (char*) ModBus[dev].unit, (char*) ModBus[dev].desc);
  dev++;
}

void InitModbus() {
  sdm.begin();

  memset(ModBusVal, 0, sizeof(ModbusVal)*MODBUS_NUM);                                                     //initialize SDM communication
  delay(1000);                                                                  //wait a while before next loop


  setModbus(0, 31, 60, 3600, SDM_TOTAL_ACTIVE_ENERGY, (char*) "BAT_IN_E",  (char*) "kWh",  (char*) "Powersupply Energy");
  setModbus(1, 31, 10, 3600, SDM_PHASE_1_POWER,       (char*) "BAT_IN_P",  (char*) "W",    (char*) "Powersupply Power");
  setModbus(2, 31, 10, 3600, SDM_PHASE_1_VOLTAGE,     (char*) "BAT_IN_U",  (char*) "V",    (char*) "Powersupply Voltage");
  setModbus(3, 31, 60, 3600, SDM_PHASE_1_CURRENT,     (char*) "BAT_IN_I",  (char*) "A",    (char*) "Powersupply Current");
  setModbus(4, 31, 10, 3600, SDM_FREQUENCY,           (char*) "BAT_IN_F",  (char*) "Hz",   (char*) "Powersupply Frequency");
  setModbus(5, 31, 60, 3600, SDM_TOTAL_ACTIVE_ENERGY, (char*) "BAT_OUT_E", (char*) "kWh",  (char*) "Powerwall AC Energy");
  setModbus(6, 31, 10, 3600, SDM_PHASE_1_POWER,       (char*) "BAT_OUT_P", (char*) "W",    (char*) "Powerwall AC Power");
  setModbus(7, 31,  1, 3600, SDM_PHASE_1_VOLTAGE,     (char*) "BAT_OUT_U", (char*) "V",    (char*) "Powerwall AC Voltage");
  setModbus(8, 31, 60, 3600, SDM_PHASE_1_CURRENT,     (char*) "BAT_OUT_I", (char*) "A",    (char*) "Powerwall AC Current");
  setModbus(9, 31,  5, 3600, SDM_FREQUENCY,           (char*) "BAT_OUT_F", (char*) "Hz",   (char*) "Powerwall AC Freqency");

}


void ReadModbus () {

  static uint8_t ind = 0;
  uint32_t ts = xTaskGetTickCount();

  if(MODBUS_NUM){

    if(ModBus[ind].min < (ts - ModBusVal[ind].last)/1000) {

      ModBusVal[ind].val = sdm.readVal(ModBus[ind].reg, ModBus[ind].addr);
      ModBusVal[ind].last = ts;

//      SERIAL_DEBUG.printf("Read Modbus: %d %s: %f\n", ind, ModBus[ind].name, ModBusVal[ind].val);
    }

    if(++ind >= MODBUS_NUM)
      ind = 0;

  }

}
