#ifndef DIYBMS_CONTROLLER_CANBUS_H_
#define DIYBMS_CONTROLLER_CANBUS_H_

#include "defines.h"
#include "Rules.h"
#include <driver/twai.h>

class ControllerCAN
{

public:
	// This table is for easy visualization of all the CAN parameters. It's size must match the first two dimensions of the data matrix!!
	const uint16_t id[MAX_CAN_PARAMETERS][MAX_NUM_CONTROLLERS] = {
	/*                                                                                                                                                                                     __INDUSTRY STANDARD ID'S__    */
	/*  0       DVCC*/                    {0x258 /*600*/   ,0x259 /*601*/   ,0x25a /*602*/   ,0x25b /*603*/   ,0x25c /*604*/   ,0x25d /*605*/   ,0x25e /*606*/   ,0x25f /*607*/   },         /*__0x351 (849)__*/
	/*  1       ALARMS*/                  {0x26c /*620*/   ,0x26d /*621*/   ,0x26e /*622*/   ,0x26f /*623*/   ,0x270 /*624*/   ,0x271 /*625*/   ,0x272 /*626*/   ,0x273 /*627*/   },         /*__0x35a (858)__*/
	/*  2       DIYBMS_MSGS*/             {0x280 /*640*/   ,0x281 /*641*/   ,0x282 /*642*/   ,0x283 /*643*/   ,0x284 /*644*/   ,0x285 /*645*/   ,0x286 /*646*/   ,0x287 /*647*/   },
	/*  3       #MODULES OK*/             {0x294 /*660*/   ,0x295 /*661*/   ,0x296 /*662*/   ,0x297 /*663*/   ,0x298 /*664*/   ,0x299 /*665*/   ,0x29a /*666*/   ,0x29b /*667*/   },         /*__0X372 (882)__*/
	/*  4       SOC/SOH*/                 {0x3e8 /*1000*/  ,0x3e9 /*1001*/  ,0x3ea /*1002*/  ,0x3eb /*1003*/  ,0x3ec /*1004*/  ,0x3ed /*1005*/  ,0x3ee /*1006*/  ,0x3ef /*1007*/  },         /*__0x355 (853)__*/
	/*  5       CAP & FIRMWARE*/          {0x3fc /*1020*/  ,0x3fd /*1021*/  ,0x3fe /*1022*/  ,0x3ff /*1023*/  ,0x400 /*1004*/  ,0x401 /*1005*/  ,0x402 /*1006*/  ,0x403 /*1007*/  },         /*__0x35f (863)__*/
	/*  6       V-I-T*/                   {0x410 /*1040*/  ,0x411 /*1041*/  ,0x412 /*1042*/  ,0x413 /*1043*/  ,0x414 /*1044*/  ,0x415 /*1045*/  ,0x416 /*1046*/  ,0x417 /*1047*/  },         /*__0x356 (854)__*/
	/*  7       HOSTNAME*/                {0x424 /*1060*/  ,0x425 /*1061*/  ,0x426 /*1062*/  ,0x427 /*1063*/  ,0x428 /*1064*/  ,0x429 /*1065*/  ,0x42a /*1066*/  ,0x42b /*1067*/  },         /*__0x35e (862)__*/
	/*  8       MIN_MAX CELL V_T*/        {0x438 /*1080*/  ,0x439 /*1081*/  ,0x43a /*1082*/  ,0x43b /*1083*/  ,0x43c /*1084*/  ,0x43d /*1085*/  ,0x43e /*1086*/  ,0x43f /*1087*/  },         /*__0x373 (883)__*/
	/*  9       MIN CELL V I.D.*/         {0x44c /*1100*/  ,0x44d /*1101*/  ,0x44e /*1102*/  ,0x44f /*1103*/  ,0x450 /*1104*/  ,0x451 /*1105*/  ,0x452 /*1106*/  ,0x453 /*1107*/  },         /*__0x374 (884)__*/
	/*  10      MAX CELL V I.D.*/         {0x460 /*1120*/  ,0x461 /*1121*/  ,0x462 /*1122*/  ,0x463 /*1123*/  ,0x464 /*1124*/  ,0x465 /*1125*/  ,0x466 /*1126*/  ,0x467 /*1127*/  },         /*__0x375 (885)__*/
	/*  11      MIN CELL T I.D.*/         {0x474 /*1140*/  ,0x475 /*1141*/  ,0x476 /*1142*/  ,0x477 /*1143*/  ,0x478 /*1144*/  ,0x479 /*1145*/  ,0x47a /*1146*/  ,0x47b /*1147*/  },         /*__0x376 (886)__*/
	/*  12      MAX CELL T I.D.*/         {0x488 /*1160*/  ,0x489 /*1161*/  ,0x48a /*1162*/  ,0x48b /*1163*/  ,0x48c /*1164*/  ,0x48d /*1165*/  ,0x48e /*1166*/  ,0x48f /*1167*/  }          /*__0x377 (887)__*/
	};

	ControllerCAN()
	{
		master = 0;
		online_controller_count = 1;
		canDisconnect = false;

		memset(&data, 0, sizeof(data));
		memset(&timestampBuffer, 0, sizeof(timestampBuffer));
		init_hash_table();

		// create a Mutex array for each parameter (row) in the data array
		for (uint8_t i =0; i < MAX_CAN_PARAMETERS; i++)
		{
			dataMutex[i] = xSemaphoreCreateMutex();
			assert(dataMutex[i]);
		}
	}
	
	// Define range of CAN parameters we care about
	const uint16_t ID_LBOUND = 600;
	const uint16_t ID_UBOUND = 1167;

	// Create pseudo hash table for ID matrix to be able to quickly navigate data table by indices 
	void init_hash_table();


	// retrieve data row from message ID
	uint8_t hash_i (uint32_t ID) {return hash_table[ID - ID_LBOUND] / 100;} 
	// retrieve data column from message ID
	uint8_t hash_j (uint32_t ID) {return hash_table[ID - ID_LBOUND] % 100;}	

	// ensure this message has a valid entry in our table (only message 0x258 has a valid zero value)
	bool hash_valid (uint32_t ID) {
		if ((ID_LBOUND <= ID <=ID_UBOUND) && ((ID == 0x258) || hash_table[ID - ID_LBOUND]))
		{return true;}
		else
		{return false;}
	}
	
	uint16_t hash_table[570]; // array large enough to represent range of CAN id's (600 - 1167 ). largest stored value == 12 * 100 + 8 = 1208

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

	// storage array for each parameter  FIRST 2 DIMENSIONS MUST MATCH DIMENSIONS OF id[] ARRAY!
	uint8_t data[MAX_CAN_PARAMETERS][MAX_NUM_CONTROLLERS][TWAI_FRAME_MAX_DLC] __attribute__ ((aligned (4)));

	// storage array that will hold the DIY_MSGS timestamp (microseconds)
	int64_t timestampBuffer[MAX_NUM_CONTROLLERS];

	// This controller is online
	bool controller_is_online[MAX_NUM_CONTROLLERS];

	// This controller is integrated
	bool controller_is_integrated[MAX_NUM_CONTROLLERS];

	// # of controllers currently online
	uint8_t online_controller_count;

	// # of controllers not isolated & participating in aggregation
	uint8_t integrated_count;
	
	//void CAN_Networking_disconnect(TimerHandle_t error_debounce_timer);

	bool canDisconnect;
	 
	TimerHandle_t error_debounce_timer;

    bool NetworkedControllerRules();

	SemaphoreHandle_t dataMutex[MAX_CAN_PARAMETERS];
private:
	void SetBankAndModuleText(char* buffer, uint8_t cellid);

};


extern uint8_t TotalNumberOfCells();
extern Rules rules;
extern currentmonitoring_struct currentMonitor;
extern diybms_eeprom_settings mysettings;
extern std::string hostname;
extern ControllerState _controller_state;
extern QueueHandle_t CANtx_q_handle;
extern TaskHandle_t canbus_rx_task_handle, canbus_tx_task_handle;
extern void send_canbus_message(CANframe *canframe);

#endif
