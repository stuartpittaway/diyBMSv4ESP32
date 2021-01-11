
#ifndef DIYBMS_MODBUS_H_
#define DIYBMS_MODBUS_H_

#define MODBUS_NUM 10
#define MODBUS_NAME_LEN 20
#define MODBUS_UNIT_LEN 10
#define MODBUS_DESC_LEN 40

#define MB_READ_REGISTER 0x04

struct ModbusInfo
{
  uint8_t addr;
  uint16_t op;
  uint32_t min;
  uint32_t max;
  uint16_t reg;
  bool rule;
  bool mqtt;
  char name[MODBUS_NAME_LEN];
  char unit[MODBUS_UNIT_LEN];
  char desc[MODBUS_DESC_LEN];
};

struct ModbusVal
{
  volatile float val;
  volatile uint32_t last;
};



#endif