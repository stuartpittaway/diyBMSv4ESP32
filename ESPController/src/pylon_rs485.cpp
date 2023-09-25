/*------------------------------------------------------------------------
 *
 *   Project: PylonTech protocol emulation library for diyBMS
 *            Using RS485 @ 9600 baud
 *
 *   Author:  Michal Ruzek (ruza87)
 *
 * -----------------------------------------------------------------------
 */


#include "pylon_rs485.h"

static constexpr const char * const TAG = "diybms-pylon485";

#define PYL_VERSION 0x3832    // '28' in ASCII
#define PYL_ADDR    0x3230    // '02' in ASCII
#define PYL_CID1    0x3634    // '46' in ASCII

#define CMD_VERSION     0x4634    // '4F' in ASCII
#define CMD_ANALOG_VAL  0x3234    // '42' in ASCII
#define CMD_CHARGE_MGMT 0x3239    // '92' in ASCII


PylonRS485::PylonRS485(uart_port_t portNum, diybms_eeprom_settings& settings, Rules& rules, currentmonitoring_struct& currentMonitor,
                 ControllerState& controllerState, HAL_ESP32& hal)
   : uart_num(portNum),
     settings(settings),
     rules(rules),
     current_monitor(currentMonitor),
     controller_state(controllerState),
     hal(hal) {

}


uint32_t PylonRS485::hex2int(char *hex, char len) {
    uint32_t val = 0;
    char count = 0;
    while (count < len) {
        // get current character then increment
        uint8_t byte = *hex++;
        count++;
        // transform hex character to the 4bit equivalent number, using the ascii table indexes
        if (byte >= '0' && byte <= '9') byte = byte - '0';
        else if (byte >= 'a' && byte <='f') byte = byte - 'a' + 10;
        else if (byte >= 'A' && byte <='F') byte = byte - 'A' + 10;
        // shift 4 to make space for new digit, and add the 4 bits of the new digit
        val = (val << 4) | (byte & 0xF);
    }
    return val;
}


void PylonRS485::insertLength(char *buf, int payload_len) {
   //do not include header into payload len
   payload_len -= sizeof(THeader);
   uint8_t sum = payload_len & 0x0F; //first 4 digits
   sum += (payload_len >> 4) & 0x0F; //second 4 digits
   sum += (payload_len >> 8) & 0x0F; //third 4 digits
   sum = sum % 16;         // modulo 16
   sum = ~sum;             // invert bits
   sum = (sum + 1) & 0x0F; // add one.

   //print into temporary buffer (trailing \0 present)
   char tmp[6];
   snprintf(tmp, sizeof(tmp), "%01X%03X", sum, payload_len);

   //insert into payload on proper position
   THeader* hdr_ptr = (THeader*)buf;
   for (char i=0; i<4; i++) {
      hdr_ptr->length[i] = tmp[i];
   }
}


int PylonRS485::appendChecksum(char *buf, int buf_size, int payload_len) {
   uint16_t sum = 0;
   //do not include SOF into chksum (start at 1)
   for (int i=1; i<payload_len; i++) {
      sum += buf[i];
   }
   sum = ~sum;             // invert bits, implicit modulo 65536
   sum = sum + 1;          // add one.

   //print sum into the end of buffer, add trailer, return new payload len (increased by the sum)
   return snprintf(buf+payload_len, buf_size, "%04X\r", sum) + payload_len;
}


void PylonRS485::handle_rx() {
   int rx_avail = 0;
   int rx_read  = 0;
   int response_len = 0;
   bool error = false;
   THeader hdr;

   if (hal.GetRS485Mutex()) {
      // Got mutex. Would be better to wrap it closely to uart_* methods, but since the PylonRS485 is the only thing
      // accessing the RS485, keep the lock for the whole time.

      ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t*)&rx_avail));
      if (rx_avail > sizeof(hdr)) {
         // at least size of header, read it out
         rx_read = uart_read_bytes(uart_num, (uint8_t*)&hdr, sizeof(hdr), pdMS_TO_TICKS(500));
         if (rx_read != sizeof(hdr)) {
            //wrong size, bail out
            uart_flush_input(uart_num);
            hal.ReleaseRS485Mutex();
            return;
         }

         // parse header, check values
         if (hdr.soh != '~' || hdr.cid1 != PYL_CID1) {
            // invalid message format, drop
            uart_flush_input(uart_num);
            hal.ReleaseRS485Mutex();
            return;
         }

         // read the rest of the message (optional INFO + 4B of ascii chksum + terminator)
         uint32_t body_len = (hex2int(hdr.length, 4)) & 0x0FFF; // mask to remove lchksum
         body_len += 5; //add length of message checksum (4 chars in ASCII) and terminator '\r'
         if (body_len > sizeof(tmp_buf) || body_len + rx_read > rx_avail) {
            // requested more data than available, fail
            uart_flush_input(uart_num);
            hal.ReleaseRS485Mutex();
            return;
         }
         if (uart_read_bytes(uart_num, tmp_buf, body_len, pdMS_TO_TICKS(500)) != body_len) {
            //wrong size, bail out
            uart_flush_input(uart_num);
            hal.ReleaseRS485Mutex();
            return;
         }

         //  TODO: check payload checksum in tmp_buf??

         // skip if not for us
         if (hdr.addr != PYL_ADDR) {
            hal.ReleaseRS485Mutex();
            return;
         }

         // header seems good, parse command
         switch (hdr.cid2) {
            case CMD_VERSION:
               //do not check version for this command
               response_len = snprintf(tmp_buf, sizeof(tmp_buf),"~"     // SOH
                                                               "28"    // VER
                                                               "02"    // ADDR
                                                               "4600"  // CID1 + RET of zero
                                                               "0000"  // Zero LENGTH
                                    );
            break;

            case CMD_ANALOG_VAL:
               //skip if version doesn't match
               if (hdr.ver != PYL_VERSION) break;

               // If current shunt is installed, use the voltage from that as it should be more accurate
               if (settings.currentMonitoringEnabled && current_monitor.validReadings) {
                  pack_voltage = current_monitor.modbus.voltage * 1000.0;
               }
               else {
                  // Use highest bank voltage calculated by controller and modules
                  pack_voltage = rules.highestBankVoltage;
               }

               response_len = snprintf(tmp_buf, sizeof(tmp_buf),"~"      // SOH
                                                               "28"     // VER
                                                               "02"     // ADDR
                                                               "4600"   // CID1 + RET of zero
                                                               "XXXX"   // Placeholder for length (computed later)
                                                               "11"     // Info flags
                                                               "02"     // Command == ADDR
                                                               "10"     // Cell count = 16 (10h)
                                                               "0D200D200D200D200D200D200D200D200D200D200D200D200D200D200D200D20" // 16x cell voltage 3360mv as hexa ascii (taken from doc example)
                                                               "010BC30BC30BC30BCD0BCD" // 5x temperature sensor with value in kelvins (taken from doc example)
                                                               "0000"   // current = 0 (resolution = tenths of mA, realValue = thisValue * 100)
                                                               "%04X"   // pack voltage (mV)
                                                               "FFFF02" // remain capacity, user def = 02
                                                               "FFFF"   // total capacity
                                                               "0002"   // num of cycles
                                    , pack_voltage);
               //add length
               insertLength(tmp_buf, response_len);
            break;

            case CMD_CHARGE_MGMT:
               //skip if version doesn't match
               if (hdr.ver != PYL_VERSION) break;

               // Defaults (do nothing)
               charge_current_limit = 1;
               discharge_current_limit = 1;
               charge_voltage = rules.DynamicChargeVoltage() * 100; //convert to mV scale
               discharge_voltage = settings.dischargevolt * 100;    //convert to mV scale
               if (rules.IsChargeAllowed(&settings)) {
                  if (rules.numberOfBalancingModules > 0 && settings.stopchargebalance == true) {
                     // Balancing is active, so stop charging
                     charge_current_limit = 1;
                  }
                  else {
                     // Default - normal behaviour
                     charge_current_limit = rules.DynamicChargeCurrent();
                  }
               }
               if (rules.IsDischargeAllowed(&settings)) {
                  discharge_current_limit = settings.dischargecurrent;
               }

               // My inverter doesn't read "Alarm message 0x44h" -> use status field of this message to stop chrg/dischrg
               // when alarm occurs.
               stop_charging = !rules.IsChargeAllowed(&settings);
               stop_discharging = !rules.IsDischargeAllowed(&settings);

               // The IsChargeAllowed / IsDischargeAllowed methods already cover:
               //  - Internal errors, Emergency stop
               //  - Low & High module temperature alarms (set in Rules, stops both chrg & dischrg)
               //  - Bank overvoltage & undervoltage alarm (set in Rules, stops both chrg & dischrg)
               //  - External temperature alarm (set on Charging page, stops both chrg & dischrg)
               //  - Low bank voltage (set on Charging page, stops discharging)
               //  - High bank voltage (set on Charging page, stops charging)
               //  - Cell undervoltage (set on Charging page, stops discharging)
               //  - Cell overvoltage (set on Charging page, stops charging)

               // Battery high voltage alarm from Current monitor -> stop charging
               stop_charging |= rules.ruleOutcome(Rule::CurrentMonitorOverVoltage);

               // Battery low voltage alarm from Current monitor -> stop discharging
               stop_discharging |= rules.ruleOutcome(Rule::CurrentMonitorUnderVoltage);

               // Place bolean flags into byte status field
               flags = 0;
               if (!stop_charging) {
                  flags |= 0x80;
               }
               if (!stop_discharging) {
                  flags |= 0x40;
               }

               response_len = snprintf(tmp_buf, sizeof(tmp_buf),"~"      // SOH
                                                               "28"     // VER
                                                               "02"     // ADDR
                                                               "4600"   // CID1 + RET of zero
                                                               "XXXX"   // Placeholder for length (computed later)
                                                               "02"     // Command == ADDR
                                                               "%04X"   // Charge limit
                                                               "%04X"   // Discharge limit
                                                               "%04X"   // Charge current
                                                               "%04X"   // Discharge current
                                                               "%02X"   // Status flags
                                    , charge_voltage, discharge_voltage, charge_current_limit, discharge_current_limit, flags);
               //add length
               insertLength(tmp_buf, response_len);
            break;

            default:
               //ignore unsupported commands
               ESP_LOGD(TAG, "Unsupported command 0x%04X", hdr.cid2);
            break;
         }

         if (response_len > 0) {
            //append checksum and trailer to the response (leading SOH is skipped during computation)
            response_len = appendChecksum(tmp_buf, sizeof(tmp_buf), response_len);

            //send through UART
            uart_write_bytes(uart_num, tmp_buf, response_len);
         }

         hal.ReleaseRS485Mutex();
      }
      else {
         // Got nothing or buffer too small, suspend the task
         hal.ReleaseRS485Mutex();
         vTaskDelay(pdMS_TO_TICKS(200));
      }

   }
}

