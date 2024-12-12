#ifndef DIYBMS_CONTROLLER_CANBUS_H_
#define DIYBMS_CONTROLLER_CANBUS_H_

#include "defines.h"
#include "Rules.h"
#include <driver/twai.h>
//#include "queue.h"



class ControllerCAN
{
public:
	ControllerCAN()
	{
		master = 0;
		online_controller_count = 0;
		
		memset(&data, 0, sizeof(data));
		memset(&DIYBMS_TIMESTAMP, 0, sizeof(DIYBMS_TIMESTAMP));
	}


	void c2c_DVCC();
	void c2c_ALARMS();
	void c2c_DIYBMS_MSGS();
	void c2c_MODULES();
	void c2c_SOC();
	void c2c_CAP();
	void c2c_HOST();
	void c2c_MINMAX_CELL_V_T();
	void c2c_CELL_IDS();
	void c2c_VIT();

	void clearvalues();

	void who_is_master();

	uint8_t master;

	uint8_t controllerNetwork_status();

	// this array holds all the CAN identifiers
	const static uint32_t id[MAX_CAN_PARAMETERS][MAX_NUM_CONTROLLERS];

	// storage array for each parameter  FIRST 2 DIMENSIONS MUST MATCH DIMENSIONS OF id[] ARRAY!
	uint8_t data[MAX_CAN_PARAMETERS][MAX_NUM_CONTROLLERS][TWAI_FRAME_MAX_DLC];

	// storage array for BITMSGS timestamp (microseconds)
	int64_t DIYBMS_TIMESTAMP[MAX_NUM_CONTROLLERS];

	// # of controllers currently online
	uint8_t online_controller_count;
	
	// Get the # of controllers participating in DVCC
	uint8_t DVCC_count() const {return DVCC_controllers;}

	// returns the heartbeat status of a networked controller
	bool controller_heartbeat(uint8_t controllerAddress);

	// Disconnect the bms from canbus
	void CAN_Networking_disconnect(TimerHandle_t error_debounce_timer);

private:
	void SetBankAndModuleText(char* buffer, uint8_t cellid);

	// # of controllers online and not Isolated (on the DC bus and free to participate in DVCC)
	uint8_t DVCC_Controllers;
};


extern uint8_t TotalNumberOfCells();
extern Rules rules;
extern currentmonitoring_struct currentMonitor;
extern diybms_eeprom_settings mysettings;
extern std::string hostname;
extern ControllerState _controller_state;
extern QueueHandle_t CANtx_q_handle;

#endif
