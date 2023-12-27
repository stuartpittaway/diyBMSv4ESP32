#ifndef DIYBMS_PYLON_RS485_H_
#define DIYBMS_PYLON_RS485_H_

#include "defines.h"
#include "Rules.h"
#include "HAL_ESP32.h"


class PylonRS485 {
   public:
      /**
       * @brief Constructor of the PylonRS485 class
       */
      PylonRS485(uart_port_t portNum, diybms_eeprom_settings& settings, Rules& rules, currentmonitoring_struct& currentMonitor,
                 ControllerState& controllerState, HAL_ESP32& hal);

      /**
       * @brief Call this to periodically check queries from inverter and to form a reply
       */
      void handle_rx();

   private:
      typedef struct {
         uint8_t soh;
         uint16_t ver;
         uint16_t addr;
         uint16_t cid1;
         uint16_t cid2;
         char length[4];
      } __attribute__((packed)) THeader;

      uart_port_t uart_num;
      diybms_eeprom_settings& settings;
      Rules& rules;
      currentmonitoring_struct& current_monitor;
      ControllerState& controller_state;
      HAL_ESP32& hal;

      uint16_t pack_voltage;
      uint16_t charge_voltage;
      uint16_t discharge_voltage;
      uint16_t charge_current_limit;
      uint16_t discharge_current_limit;
      bool stop_charging;
      bool stop_discharging;
      uint8_t flags;
      char tmp_buf[150];

      uint32_t hex2int(char *hex, char len);
      void insertLength(char *buf, int payload_len);
      int appendChecksum(char *buf, int buf_size, int payload_len);
};

#endif