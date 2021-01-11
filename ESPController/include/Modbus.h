#include <SDM.h>

#define MODBUS_NUM   10
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
  uint8_t rule;
  uint8_t mqtt;
  char name[MODBUS_NAME_LEN];
  char unit[MODBUS_UNIT_LEN];
  char desc[MODBUS_DESC_LEN];
};

struct ModbusVal
{
  volatile float val;
  volatile uint32_t last;
};

extern ModbusInfo ModBus[MODBUS_NUM];
extern ModbusVal ModBusVal[MODBUS_NUM];
// extern int ModbusNum;

void InitModbus();
void ReadModbus();

void setModbus(int dev, uint8_t addr, uint32_t min, uint32_t max, uint16_t reg, char *name, char* unit, char* desc);

void setModbusName(int dev, char* cp);
void setModbusUnit(int dev, char* cp);
void setModbusDesc(int dev, char* cp);
