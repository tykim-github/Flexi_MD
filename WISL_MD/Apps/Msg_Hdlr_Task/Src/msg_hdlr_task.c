/**
 * @file msg_hdlr_task.c
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics FW Team
 * @brief 
 * @ref 
 */

// TODO : refactoring!!
#include "../../Apps/Msg_Hdlr_Task/Inc/msg_hdlr_task.h"

/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */




/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Variables accessible throughout the application.
 */

TaskStruct msg_hdlr_task;
uint8_t MD_node_id;

extern uint8_t act_type;
extern uint16_t packed_risk;

//uint32_t packed_risk;
/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */
//#define ACT_HIP_ABD
//#define ACT_HIP_ROT
//#define ACT_HIP_SAG
//#define ACT_KNEE
//#define ACT_ANK

static cvector_vector_type(MsgDataObjectHeader) pdo_send_list;
static cvector_vector_type(MsgDataObjectHeader) sdo_req_list;
static cvector_vector_type(MsgDataObjectHeader) sdo_res_list;

static TrajectoryBuffer trajectory_buffer;
static COMMType comm_type;

static uint8_t GUI_onoff;
static uint8_t GUI_command;

//static uint8_t MD_node_id;
static uint8_t ori_node;
static uint32_t fnc_code;
static uint32_t err_code;
static uint8_t fdcanTxData[64];
static uint8_t fdcanRxData[64];
static uint8_t usbRxData[64];
static int comm_loop_cnt;

static int32_t test_dummy[10];

static float msgTimeElap;

static WS5_MD_Transmit_Data_16BIT WS5_MD_Transmit_Data_16bit;
static WS5_MD_Transmit_Data_32BIT WS5_MD_Transmit_Data_32bit;

/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

/* ------------------- STATE FUNCTION ------------------- */
static void StateOff_Ent();

static void StateStandby_Ent();
static void StateStandby_Run();
static void StateStandby_Ext();

static void StateEnable_Ent();
static void StateEnable_Run();
static void StateEnable_Ext();

static void StateError_Run();

/* ------------------- READ NODE ID ------------------- */
static uint8_t Read_Node_ID();

/* ------------------- CONVERT BYTE TO LENGTH ------------------- */
static MsgDataObjectHeader Get_Header(uint8_t* t_byte_arr);

/* ------------------- EMCY ------------------- */
void Send_EMCY(uint32_t* t_err_code);
static void Recv_EMCY(uint8_t* t_byte_arr, uint32_t* t_err_code);
static MsgSDOargs Convert_Bytes_to_SDO_req(uint8_t* t_byte_arr, uint16_t *t_byte_len);

/* ------------------- SDO RX ------------------- */
static int Read_SDO(uint8_t* t_byte_arr);
static int Unpack_SDO(uint8_t* t_byte_arr);
static int Convert_SDOres_to_Bytes(MsgDataObjectHeader* t_header, uint8_t* t_byte_arr);

/* ------------------- SDO TX ------------------- */
static int Pack_SDO(uint8_t* t_byte_arr, uint8_t* t_byte_len);
static int Send_SDO(uint8_t t_dest_node);
static int Convert_Bytes_to_PDO(uint8_t* t_byte_arr);
static int Unpack_PDO(uint8_t* t_byte_arr);

/* ------------------- PDO RX ------------------- */
static int Convert_PDO_to_Bytes(MsgDataObjectHeader* t_header, uint8_t* t_byte_arr);
static int Pack_PDO(uint8_t* t_byte_arr, uint8_t* t_byte_len);
static int Run_Send_PDO();

/* ------------------- PDO TX ------------------- */
static int Ext_Send_PDO();
static int Set_PDO_Dummy();

/* ------------------- TRAJECTORY ------------------- */
static int Check_Trajectory_Error(uint16_t t_frame_idx);
static int Assemble_Trajectory(uint8_t* t_byte_arr);

/* ------------------- LIST MANAGEMENT ------------------- */
static void Add_PDO_to_Send(uint8_t t_dod_id, uint8_t t_obj_id);
static void Send_USB_Trick(uint8_t* t_in_buf, uint32_t t_in_len, uint8_t* t_out_buf);

/* ------------------- MSG HANDLER ------------------- */
int Send_MSG(uint16_t t_COB_ID, uint32_t t_len, uint8_t* t_tx_data);
static int USB_Rx_Hdlr(uint8_t* t_Buf, uint32_t* t_Len);
static int Fdcan_Rx_Hdlr(uint16_t t_wasp_id, uint8_t* t_rx_data);

/* ------------------- SDO CALLBACK ------------------- */
static void Set_Send_PDO_List(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_MS_Enum(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_GUI_COMM_OnOff(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_GUI_COMM_Command(MsgSDOargs* t_req, MsgSDOargs* t_res);

/************ WS5 SPECIAL FUNCTIONS ************/
static int Init_Encoding_WS5_Dataset(void);
static int Run_Encoding_WS5_Dataset(void);
/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

MSG_COMMON_SDO_CALLBACK(msg_hdlr_task)

void InitMsgHdlr()
{
    /*Task Init*/
    Init_Task(&msg_hdlr_task);

    MD_node_id = Read_Node_ID();
    act_type   = MD_node_id;

    ori_node = 0x00;

    IOIF_SetUSBCB();

    /* Communication Init */
    // TODO : Error Handle!
    comm_type = IOIF_InitUSB(IOIF_TIM6);
    
    if (comm_type == e_FDCAN) {
        // TODO : Error Handle!
        IOIF_InitFDCAN1(MD_node_id);
    }


	/* State Definition */
	TASK_CREATE_STATE(&msg_hdlr_task, e_State_Off,      StateOff_Ent,       NULL,    			NULL,               false);
	TASK_CREATE_STATE(&msg_hdlr_task, e_State_Standby,  StateStandby_Ent,   StateStandby_Run,	StateStandby_Ext,   true);
	TASK_CREATE_STATE(&msg_hdlr_task, e_State_Enable,   StateEnable_Ent,   	StateEnable_Run, 	StateEnable_Ext,   	false);
	TASK_CREATE_STATE(&msg_hdlr_task, e_State_Error,    NULL,   			StateError_Run,     NULL,   	        false);

    TASK_CREATE_ROUTINE(&msg_hdlr_task,  ROUTINE_ID_MSG_PDO_SEND,           NULL,                        Run_Send_PDO,      Ext_Send_PDO);
    TASK_CREATE_ROUTINE(&msg_hdlr_task,  ROUTINE_ID_MSG_PDO_DUMMY_TEST,     NULL,                        Set_PDO_Dummy, NULL);
    TASK_CREATE_ROUTINE(&msg_hdlr_task,  ROUTINE_ID_MSG_WS5_PDO_PACK,       Init_Encoding_WS5_Dataset,   Run_Encoding_WS5_Dataset,      NULL);

    ////PCAN ///////
//    Push_Routine(&msg_hdlr_task.routine, ROUTINE_ID_MSG_PDO_SEND);
    ////////////////
	/* Data Object Definition */
    Create_DOD(TASK_ID_MSG);

	MSG_COMMON_SDO_CREATE(TASK_ID_MSG)
    Create_SDO(TASK_ID_MSG, SDO_ID_MSG_PDO_LIST, 				e_UInt16, Set_Send_PDO_List);
    Create_SDO(TASK_ID_MSG, SDO_ID_MSG_MS_ENUM,  				e_UInt8,  Set_MS_Enum);
    Create_SDO(TASK_ID_MSG, SDO_ID_MSG_GUI_COMM_ONOFF,  		e_UInt8,  Set_GUI_COMM_OnOff);
    Create_SDO(TASK_ID_MSG, SDO_ID_MSG_GUI_COMM_COMMAND,  		e_UInt8,  Set_GUI_COMM_Command);

	Create_PDO(TASK_ID_MSG, PDO_ID_MSG_TEST1, 					e_Int32, 1, &test_dummy[0]);
	Create_PDO(TASK_ID_MSG, PDO_ID_MSG_TEST2, 					e_Int32, 1, &test_dummy[1]);
	Create_PDO(TASK_ID_MSG, PDO_ID_MSG_TEST3, 					e_Int32, 1, &test_dummy[2]);
	Create_PDO(TASK_ID_MSG, PDO_ID_MSG_TEST4, 					e_Int32, 1, &test_dummy[3]);
	Create_PDO(TASK_ID_MSG, PDO_ID_MSG_TEST5, 					e_Int32, 1, &test_dummy[4]);
	Create_PDO(TASK_ID_MSG, PDO_ID_MSG_TEST6, 					e_Int32, 1, &test_dummy[5]);
	Create_PDO(TASK_ID_MSG, PDO_ID_MSG_TEST7, 					e_Int32, 1, &test_dummy[6]);
	Create_PDO(TASK_ID_MSG, PDO_ID_MSG_TEST8, 					e_Int32, 1, &test_dummy[7]);
	Create_PDO(TASK_ID_MSG, PDO_ID_MSG_TEST9, 					e_Int32, 1, &test_dummy[8]);
	Create_PDO(TASK_ID_MSG, PDO_ID_MSG_TEST10, 					e_Int32, 1, &test_dummy[9]);

	Create_PDO(TASK_ID_MSG, PDO_ID_WS5_MD_TRANSMIT_16BIT,  		e_Int16,  7, &WS5_MD_Transmit_Data_16bit);
	Create_PDO(TASK_ID_MSG, PDO_ID_WS5_MD_TRANSMIT_32BIT,  		e_UInt32, 1, &WS5_MD_Transmit_Data_32bit);

	/* Callback Allocation */
    if (comm_type == e_FDCAN) {
    	IOIF_SetFDCANRxCB(IOIF_FDCAN1, IOIF_FDCAN_RXFIFO0CALLBACK, Fdcan_Rx_Hdlr);
    } else {
    	ioif_usbRxCBPtr = USB_Rx_Hdlr;
    }

	/* Timer 6 Callback Allocation */
	if(IOIF_StartTimIT(IOIF_TIM6) > 0){
		//TODO: ERROR PROCESS
	}

	IOIF_SetTimCB(IOIF_TIM6, IOIF_TIM_PERIOD_ELAPSED_CALLBACK, RunMsgHdlr, NULL);
}

void RunMsgHdlr(void* params)
{	
    /* Loop Start Time Check */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	/* Run Device */
	Run_Task(&msg_hdlr_task);
	
	/* Elapsed Time Check */
	msgTimeElap = DWT->CYCCNT/480;	// in microsecond
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/* ------------------- STATE FUNCTION ------------------- */
static void StateOff_Ent()
{
	GUI_onoff = 0;
	GUI_command = 0;
    Transition_State(&msg_hdlr_task.state_machine, e_State_Standby);
}

static void StateStandby_Ent()
{
	// PCAN 주석 제거
//	if (comm_type == e_FDCAN) {
//		Add_PDO_to_Send(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_LOOP_CNT);
//		Add_PDO_to_Send(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_REF_POSITION);
//		Add_PDO_to_Send(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_ACTUAL_POSITION);
//	//	Add_PDO_to_Send(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_ABSENCODER1_POSITION);
//		Add_PDO_to_Send(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_CURRENT_OUTPUT);
//		Add_PDO_to_Send(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_ABSENCODER2_POSITION);
//		Add_PDO_to_Send(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_DOB_DISTURABNCE);
//	//	Add_PDO_to_Send(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_SPRING_INIT);
//	//	Add_PDO_to_Send(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_LS_CTRL_INPUT);
//		Add_PDO_to_Send(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_MID_CTRL_INPUT);
//		Add_PDO_to_Send(TASK_ID_WIDM, PDO_ID_WIDM_GAIT_PHASE);
//		Add_PDO_to_Send(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_POS_PID_CTRL_INPUT);
//	//	Add_PDO_to_Send(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_FF_INPUT);
//	//	Add_PDO_to_Send(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_POSITION);
//		Add_PDO_to_Send(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_VELOCITY);
//
//	}

//	if (comm_type == e_FDCAN) {
//		Add_PDO_to_Send(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_VELOCITY_ESTIMATED);
//		Add_PDO_to_Send(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_SYSTEM_ID_SBS_FREQ);
//		Add_PDO_to_Send(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_CURRENT_OUTPUT);
//		Add_PDO_to_Send(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_LOOP_CNT);
//		Add_PDO_to_Send(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_MID_ID_PROCESS_INPUT);
//		Add_PDO_to_Send(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_LOADCELL_FORCE);
//		Add_PDO_to_Send(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_SBS_ID_DONE);
//	}
}

static void StateStandby_Run()
{

	// PCAN 주석 제거
//	if (comm_type == e_FDCAN) {
//		Transition_State(&msg_hdlr_task.state_machine, e_State_Enable);
//	}
	///
}

static void StateStandby_Ext()
{
}

static void StateEnable_Ent()
{
	comm_loop_cnt = 0;
	Ent_Routines(&msg_hdlr_task.routine);
}

static void StateEnable_Run()
{


	Run_Routines(&msg_hdlr_task.routine);

	if (low_level_triggered_msg_stop) {Transition_State(&msg_hdlr_task.state_machine, e_State_Standby); low_level_triggered_msg_stop = 0;}
	if (mid_level_triggered_msg_stop) {Transition_State(&msg_hdlr_task.state_machine, e_State_Standby); mid_level_triggered_msg_stop = 0;}

    comm_loop_cnt++;
}

static void StateEnable_Ext()
{
	Ext_Routines(&msg_hdlr_task.routine);

	GUI_onoff = 0;
	GUI_command = 0;
}

static void StateError_Run()
{

}

/* ------------------- WS5 SPECIAL FUNCTIONS (BEGIN) ------------------- */
static int Init_Encoding_WS5_Dataset(void)
{
//	WS5_MD_Transmit_Data_16bit.K_actual_current     = 65536.0f/(ACTUAL_CURRENT_MAX  - ACTUAL_CURRENT_MIN);
//	WS5_MD_Transmit_Data_16bit.K_incenc_position    = 65536.0f/(INCENC_POSITION_MAX - INCENC_POSITION_MIN);
//	WS5_MD_Transmit_Data_16bit.K_absenc_position    = 65536.0f/(ABSENC_POSITION_MAX - ABSENC_POSITION_MIN);
//	WS5_MD_Transmit_Data_16bit.K_reference_position = 65536.0f/(REFERENCE_POSITION_MAX - REFERENCE_POSITION_MIN);
	WS5_MD_Transmit_Data_16bit.K_bus_current        = 65536.0f/(BUS_CURRENT_MAX - BUS_CURRENT_MIN);
	WS5_MD_Transmit_Data_16bit.K_temperature        = 255.0f  /(TEMPERATURE_MAX - TEMPERATURE_MIN);
	//WS5_MD_Transmit_Data_16bit.K_stator_temp        = 65536.0f/(STATOR_TEMP_MAX - STATOR_TEMP_MIN);

	if ((MD_node_id == 2) || (MD_node_id == 3)) {
		WS5_MD_Transmit_Data_16bit.K_actual_current     = 65536.0f/30;
		WS5_MD_Transmit_Data_16bit.K_incenc_position    = 65536.0f/10000;
		WS5_MD_Transmit_Data_16bit.K_absenc_position    = 65536.0f/100;
		WS5_MD_Transmit_Data_16bit.K_reference_position = 65536.0f/10000;
	}
	else if ((MD_node_id == 4) || (MD_node_id == 5)) {
		WS5_MD_Transmit_Data_16bit.K_actual_current     = 65536.0f/30;
		WS5_MD_Transmit_Data_16bit.K_incenc_position    = 65536.0f/100;
		WS5_MD_Transmit_Data_16bit.K_absenc_position    = 65536.0f/100;
		WS5_MD_Transmit_Data_16bit.K_reference_position = 65536.0f/100;
	}
	else if ((MD_node_id == 6) || (MD_node_id == 7)) {
		WS5_MD_Transmit_Data_16bit.K_actual_current     = 65536.0f/200;
		WS5_MD_Transmit_Data_16bit.K_incenc_position    = 65536.0f/350;
		WS5_MD_Transmit_Data_16bit.K_absenc_position    = 65536.0f/350;
		WS5_MD_Transmit_Data_16bit.K_reference_position = 65536.0f/350;
	}
	else if ((MD_node_id == 8) || (MD_node_id == 9)) {
		WS5_MD_Transmit_Data_16bit.K_actual_current     = 65536.0f/200;
		WS5_MD_Transmit_Data_16bit.K_incenc_position    = 65536.0f/300;
		WS5_MD_Transmit_Data_16bit.K_absenc_position    = 65536.0f/300;
		WS5_MD_Transmit_Data_16bit.K_reference_position = 65536.0f/300;
	}
	else if ((MD_node_id == 10) || (MD_node_id == 11) || (MD_node_id == 12) || (MD_node_id == 13)) {
		WS5_MD_Transmit_Data_16bit.K_actual_current     = 65536.0f/30;
		WS5_MD_Transmit_Data_16bit.K_incenc_position    = 65536.0f/20000;
		WS5_MD_Transmit_Data_16bit.K_absenc_position    = 65536.0f/100;
		WS5_MD_Transmit_Data_16bit.K_reference_position = 65536.0f/20000;
	}

//#ifdef ACT_HIP_ABD
//	WS5_MD_Transmit_Data_16bit.K_actual_current     = 65536.0f/20;
//	WS5_MD_Transmit_Data_16bit.K_incenc_position    = 65536.0f/10000;
//	WS5_MD_Transmit_Data_16bit.K_absenc_position    = 65536.0f/100;
//	WS5_MD_Transmit_Data_16bit.K_reference_position = 65536.0f/10000;
//#endif
//#ifdef ACT_HIP_ROT
//	WS5_MD_Transmit_Data_16bit.K_actual_current     = 65536.0f/20;
//	WS5_MD_Transmit_Data_16bit.K_incenc_position    = 65536.0f/100;
//	WS5_MD_Transmit_Data_16bit.K_absenc_position    = 65536.0f/100;
//	WS5_MD_Transmit_Data_16bit.K_reference_position = 65536.0f/100;
//#endif
//#ifdef ACT_HIP_SAG
//	WS5_MD_Transmit_Data_16bit.K_actual_current     = 65536.0f/200;
//	WS5_MD_Transmit_Data_16bit.K_incenc_position    = 65536.0f/350;
//	WS5_MD_Transmit_Data_16bit.K_absenc_position    = 65536.0f/350;
//	WS5_MD_Transmit_Data_16bit.K_reference_position = 65536.0f/350;
//#endif
//#ifdef ACT_KNEE
//	WS5_MD_Transmit_Data_16bit.K_actual_current     = 65536.0f/200;
//	WS5_MD_Transmit_Data_16bit.K_incenc_position    = 65536.0f/300;
//	WS5_MD_Transmit_Data_16bit.K_absenc_position    = 65536.0f/300;
//	WS5_MD_Transmit_Data_16bit.K_reference_position = 65536.0f/300;
//#endif
//#ifdef ACT_ANK
//	WS5_MD_Transmit_Data_16bit.K_actual_current     = 65536.0f/20;
//	WS5_MD_Transmit_Data_16bit.K_incenc_position    = 65536.0f/20000;
//	WS5_MD_Transmit_Data_16bit.K_absenc_position    = 65536.0f/100;
//	WS5_MD_Transmit_Data_16bit.K_reference_position = 65536.0f/20000;
//#endif

	return 0;
}

static int Run_Encoding_WS5_Dataset(void)
{
	float t_incenc_pos_deg    = mid_level_state.position * 57.295779513082323;
	float t_absenc_pos_deg    = AbsObj1.posDeg;
	float t_reference_pos_deg = posCtrl.ref    * 57.295779513082323;

	WS5_MD_Transmit_Data_16bit.actual_current     = (int16_t) (motor_out.current_act      * WS5_MD_Transmit_Data_16bit.K_actual_current);
	WS5_MD_Transmit_Data_16bit.incenc_position    = (int16_t) (t_incenc_pos_deg           * WS5_MD_Transmit_Data_16bit.K_incenc_position);
	WS5_MD_Transmit_Data_16bit.absenc_position    = (int16_t) (t_absenc_pos_deg           * WS5_MD_Transmit_Data_16bit.K_absenc_position);
	WS5_MD_Transmit_Data_16bit.reference_position = (int16_t) (t_reference_pos_deg        * WS5_MD_Transmit_Data_16bit.K_reference_position);
	WS5_MD_Transmit_Data_16bit.bus_current        = (int16_t) (batData.batCurr            * WS5_MD_Transmit_Data_16bit.K_bus_current);
	WS5_MD_Transmit_Data_16bit.error_code         = packed_risk;

	WS5_MD_Transmit_Data_16bit.actuator_temperature = (uint8_t)fminf(fmaxf((temp_sense.stator_filtered - TEMPERATURE_MIN) * WS5_MD_Transmit_Data_16bit.K_temperature, 0.0f), 255.0f);
	WS5_MD_Transmit_Data_16bit.board_temperature    = (uint8_t)fminf(fmaxf((batData.brdTemp - TEMPERATURE_MIN)            * WS5_MD_Transmit_Data_16bit.K_temperature, 0.0f), 255.0f);
	WS5_MD_Transmit_Data_16bit.temperature          = (int16_t)(WS5_MD_Transmit_Data_16bit.actuator_temperature | (WS5_MD_Transmit_Data_16bit.board_temperature << 8));

	//WS5_MD_Transmit_Data_16bit.stator_temp        = (int16_t) (temp_sense.stator_filtered * WS5_MD_Transmit_Data_16bit.K_stator_temp);

	WS5_MD_Transmit_Data_32bit.loop_time  = mid_level_loop_cnt;
//	WS5_MD_Transmit_Data_32bit.error_code = packed_risk;

	return 0;
}



/* ------------------- WS5 SPECIAL FUNCTIONS (END) ------------------- */



/* ------------------- READ NODE ID ------------------- */
static uint8_t Read_Node_ID()
{
    uint8_t temp1 = 0, temp2 = 0, temp3 = 0, temp4 = 0;
    
    #if defined(WALKON5_MD_ENABLED) || defined(L30_MD_REV06_ENABLED) || \
        defined(L30_MD_REV07_ENABLED) || defined(L30_MD_REV08_ENABLED)
    temp1 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, BSP_GPIO_PIN_8);
    temp2 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, BSP_GPIO_PIN_9);
    temp3 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, BSP_GPIO_PIN_10);
    temp4 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, BSP_GPIO_PIN_11);
    #endif

    #if defined(SUIT_MD_ENABLED)
    temp1 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_F, IOIF_GPIO_PIN_2);
    temp2 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_F, IOIF_GPIO_PIN_3);
    temp3 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_F, IOIF_GPIO_PIN_4);
    temp4 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_F, IOIF_GPIO_PIN_5);
    #endif

    return ((temp1<<3)|(temp2<<2)|(temp3<<1)|(temp4));
//    return 6;
}

/* ------------------- CONVERT BYTE TO LENGTH ------------------- */

static MsgDataObjectHeader Get_Header(uint8_t* t_byte_arr)
{
	MsgDataObjectHeader t_header = {0};
    memcpy(&t_header, t_byte_arr, sizeof(MsgDataObjectHeader));
    return t_header;
}

/* ------------------- EMCY ------------------- */
void Send_EMCY(uint32_t* t_err_code)
{
    uint8_t* t_tx_data = malloc(sizeof(uint8_t)*4);
    uint16_t t_identifier = EMCY|(MD_node_id<<4);

    memcpy(t_tx_data, t_err_code, ERR_CODE_SIZE);

    if(Send_MSG(t_identifier, 4, t_tx_data) != 0){
        //TODO: MSG TX ERROR
    }

    free(t_tx_data);
    t_tx_data = NULL;
}

static void Recv_EMCY(uint8_t* t_byte_arr, uint32_t* t_err_code)
{
    memcpy(t_err_code, t_byte_arr, ERR_CODE_SIZE);
}

/* ------------------- SDO RX ------------------- */
static MsgSDOargs Convert_Bytes_to_SDO_req(uint8_t* t_byte_arr, uint16_t *t_byte_len)
{
	MsgSDOargs t_req = {0};
    *t_byte_len = 0;

    int t_idx = sizeof(t_req.status);
    int t_len = sizeof(t_req.size);

    memcpy(&t_req.size, &t_byte_arr[t_idx], t_len);
    *t_byte_len += t_len;

    t_req.data = &t_byte_arr[t_idx + t_len];

    t_req.status = t_byte_arr[0];
    *t_byte_len += 1;

    return t_req;
}

static int Read_SDO(uint8_t* t_byte_arr)
{
    int t_byte_read = 0;
    MsgDataObjectHeader t_header = Get_Header(t_byte_arr);
    t_byte_read += sizeof(MsgDataObjectHeader);

    MsgSDOStruct* t_sdo = Find_SDO(t_header.dod_id, t_header.obj_id);
    if (t_sdo == NULL) {
        //TODO: Cannot Find SDO ERROR
        return -2;
    }

    uint16_t t_req_bytes = 0;
    MsgSDOargs t_req = Convert_Bytes_to_SDO_req(t_byte_arr + t_byte_read, &t_req_bytes);
    t_req.data_size = t_sdo->args.data_size; // Copy SDO info
    t_byte_read += t_req_bytes;

    uint16_t t_n_bytes = 0;
    if (t_req.status == DATA_OBJECT_SDO_REQU) {
    	t_n_bytes = Call_SDO(t_sdo, &t_req);
        cvector_push_back(sdo_res_list, t_header); // Assign Response
    } else if(t_req.status == DATA_OBJECT_SDO_SUCC || t_req.status == DATA_OBJECT_SDO_FAIL) {
    	t_n_bytes = Set_SDO_args(t_sdo, &t_req);
        if (t_n_bytes < 0) {
            //TODO: Set SDO Argument ERROR
            return -1;
        }
    } else {
        //TODO: Read SDO Status ERROR
        return -1;
    }

    t_byte_read += t_n_bytes;
    return t_byte_read;
}

static int Unpack_SDO(uint8_t* t_byte_arr)
{
    int t_cursor = 0;

    // Get # of SDOs
    uint16_t t_n_sdo = 0;
    memcpy(&t_n_sdo, &t_byte_arr[t_cursor], OBJ_NUMS_SIZE);
    t_cursor += OBJ_NUMS_SIZE;

    // Call & Respond SDOs
    if (t_n_sdo > 0) {
        for (int i = 0; i < t_n_sdo; ++i) {
            int temp_cursor = Read_SDO(&t_byte_arr[t_cursor]);
            if (temp_cursor > 0) {
            	t_cursor += temp_cursor;
            } else if (temp_cursor < 0) {
                //TODO: Unpack SDO ERROR
                return MSG_SDO_FAULT;
            }
        }
    }

    return MSG_DO_SUCCESS;
}

/* ------------------- SDO TX ------------------- */
static int Convert_SDOres_to_Bytes(MsgDataObjectHeader* t_header, uint8_t* t_byte_arr)
{
    int t_byte_written = 0;
    // Set SDO Header
    memcpy(t_byte_arr, t_header, sizeof(MsgDataObjectHeader));
    t_byte_written += sizeof(MsgDataObjectHeader);

    // Return Response
    MsgSDOStruct* t_sdo = Find_SDO(t_header->dod_id, t_header->obj_id);
    if (t_sdo == NULL) {
        //TODO: Cannot Find SDO ERROR
        return -2;
    }

    memcpy(t_byte_arr + t_byte_written, &t_sdo->args.status, sizeof(t_sdo->args.status));
    t_byte_written += sizeof(t_sdo->args.status);
    memcpy(t_byte_arr + t_byte_written, &t_sdo->args.size,   sizeof(t_sdo->args.size));
    t_byte_written += sizeof(t_sdo->args.size);

    int t_data_len = t_sdo->args.size * t_sdo->args.data_size;
    memcpy(t_byte_arr + t_byte_written, t_sdo->args.data, t_data_len);

    t_byte_written += t_data_len;

    return t_byte_written;
}

static int Pack_SDO(uint8_t* t_byte_arr, uint8_t* t_byte_len)
{
	// check send list whether these are empty or not 
	if ((sdo_res_list == NULL) && (sdo_req_list == NULL)){
		return MSG_SDO_NOTHING;
	}

	// Message Packaging
    int t_cursor = 0;

    // Res SDOs
    int t_n_sdo_cursor = t_cursor;
    t_cursor += OBJ_NUMS_SIZE;

    uint8_t t_n_sdo = 0;
    
    if (sdo_res_list != NULL) {
        for(int i = 0; i < cvector_size(sdo_res_list); ++i) {
            int temp_cursor = Convert_SDOres_to_Bytes(&sdo_res_list[i], &t_byte_arr[t_cursor]);
            if (temp_cursor > 0) {
            	t_cursor += temp_cursor;
                ++t_n_sdo;
            } else if (temp_cursor < 0) {
                //TODO: Pack Response SDO Error
                return MSG_SDO_FAULT;
            }
        }
        cvector_free(sdo_res_list);
        sdo_res_list = NULL;
    }

    // Req SDOs
    if (sdo_req_list != NULL) {
        for(int i = 0; i < cvector_size(sdo_req_list); ++i) {
            int temp_cursor = Convert_SDOres_to_Bytes(&sdo_req_list[i], &t_byte_arr[t_cursor]);
            if (temp_cursor > 0) {
            	t_cursor += temp_cursor;
                ++t_n_sdo;
            } else if (temp_cursor < 0) {
                //TODO: Pack Request SDO Error
                return MSG_SDO_FAULT;
            }
        }
        cvector_free(sdo_req_list);
        sdo_req_list = NULL;
    }

    // Set # of SDOs
    memcpy(&t_byte_arr[t_n_sdo_cursor], &t_n_sdo, OBJ_NUMS_SIZE);

    *t_byte_len = t_cursor;

    return MSG_DO_SUCCESS;
}

static int Send_SDO(uint8_t t_dest_node)
{
    uint8_t t_byte_len = 0;
    uint16_t t_identifier = SDO|(MD_node_id<<4)|t_dest_node;

    int t_check = Pack_SDO(fdcanRxData, &t_byte_len);

    if(t_check < 0){
        //TODO: Send SDO Error
    	return t_check;
    } else if(t_check){
    	return t_check;
    }

    if (t_byte_len > 64) {
        //TODO: TX MESSAGE TOO LONG ERROR 
    }

    if(Send_MSG(t_identifier, t_byte_len, fdcanRxData) != 0){
        return t_check;
        //TODO: MSG TX ERROR
    }

    return t_check;
}

/* ------------------- PDO RX ------------------- */
static int Convert_Bytes_to_PDO(uint8_t* t_byte_arr)
{
    int t_byte_read = 0;

    MsgDataObjectHeader t_header = Get_Header(t_byte_arr);
    t_byte_read += sizeof(MsgDataObjectHeader);

    MsgPDOStruct* t_pdo = Find_PDO(t_header.dod_id, t_header.obj_id);
    if (t_pdo == NULL) {
        //TODO: Cannot Find PDO Error
        return -2;
    }

    uint16_t t_n_bytes = Get_PDO(t_pdo, (void*)(t_byte_arr + t_byte_read));
    if (t_n_bytes < 0) {
        //TODO: Copy PDO to Receive Error
        return -1;
    }
    t_byte_read += t_n_bytes;

    return t_byte_read;
}

static int Unpack_PDO(uint8_t* t_byte_arr)
{
    int t_cursor = 0;

    // Get # of PDOs
    uint8_t t_n_pdo = 0;
    memcpy(&t_n_pdo, &t_byte_arr[t_cursor], OBJ_NUMS_SIZE);
    t_cursor += OBJ_NUMS_SIZE;

    if (t_n_pdo > 0) {
        for (int i = 0; i < t_n_pdo; ++i) {
            int temp_cursor = Convert_Bytes_to_PDO(&t_byte_arr[t_cursor]);
            if (temp_cursor > 0) {
            	t_cursor += temp_cursor;
            } else if (temp_cursor < 0) {
                //TODO: Unpack PDO Error
                return MSG_PDO_FAULT;
            }
        }
    }

    return MSG_DO_SUCCESS;
}

/* ------------------- PDO TX ------------------- */
static int Convert_PDO_to_Bytes(MsgDataObjectHeader* t_header, uint8_t* t_byte_arr)
{
    int t_header_size = sizeof(MsgDataObjectHeader);
    // Publish PDO
    MsgPDOStruct* t_pdo = Find_PDO(t_header->dod_id, t_header->obj_id);
    if (t_pdo == NULL) {
        //TODO: Cannot Find PDO
        return -2;
    }

    uint16_t t_n_bytes = Set_PDO(t_pdo, t_byte_arr + t_header_size);
    if (t_n_bytes < 0) {
        //TODO: Copy PDO to Send 
        return -1;
    } else if (t_n_bytes == 0) { // Nothing to publish
        return 0;
    }

    memcpy(t_byte_arr, t_header, t_header_size);
    return t_header_size + t_n_bytes;
}

static int Pack_PDO(uint8_t* t_byte_arr, uint8_t* t_byte_len)
{
	// check send list whether these are empty or not 
    if (pdo_send_list == NULL){
        return 0;
    }

    int t_cursor = 0;

    // Pub PDO
    int t_n_pdo_cursor = t_cursor;
    t_cursor += OBJ_NUMS_SIZE;

    uint8_t t_n_pdo = 0;

    if (pdo_send_list != NULL) {
        for(int i = 0; i < cvector_size(pdo_send_list); ++i) {

            int temp_cursor = Convert_PDO_to_Bytes(&pdo_send_list[i], &t_byte_arr[t_cursor]);
            if (temp_cursor > 0) {
            	t_cursor += temp_cursor;
                ++t_n_pdo;
            } else if (temp_cursor < 0) {
                //TODO: Pack PDO Error
                return temp_cursor;
            }
        }
    }

    // Set # of PDOs
    memcpy(&t_byte_arr[t_n_pdo_cursor], &t_n_pdo, OBJ_NUMS_SIZE);

    *t_byte_len = t_cursor;

    return MSG_DO_SUCCESS;
}

static int Run_Send_PDO()
{
    uint8_t t_byte_len = 0;
//    uint8_t t_dest_node = NODE_ID_CM;
    uint8_t t_dest_node = 1;   // 12
    uint16_t t_identifier;

    if(GUI_onoff)	{	t_identifier = GUI_SYNC|GUI_command;	}
    else 			{	t_identifier = PDO|(MD_node_id<<4)|t_dest_node;	}

    int t_check = Pack_PDO(fdcanTxData, &t_byte_len);

    if(t_check != 0){
        //TODO: Send PDO Error
    	return t_check;
    } else if(t_check){
    	return t_check;
    }

    if (t_byte_len != 1){
		if(Send_MSG(t_identifier, t_byte_len, fdcanTxData) == 0){
			return t_check;
			//TODO: MSG TX ERROR
		}
    }

	return t_check;
}

static int Ext_Send_PDO()
{
	if(GUI_command == GET_DIRECTION_SET_DATA){
		Send_MSG((uint16_t)(GUI_SYNC|GET_DIRECTION_SET_DONE), 1, (uint8_t*)0);
	}

	return 0;
}

static int Set_PDO_Dummy()
{
	static int t_count = 0;

	test_dummy[0] = comm_loop_cnt;
	test_dummy[1] = comm_loop_cnt;
	test_dummy[2] = comm_loop_cnt;
	test_dummy[3] = comm_loop_cnt;
	test_dummy[4] = comm_loop_cnt;
	test_dummy[5] = comm_loop_cnt;
	test_dummy[6] = comm_loop_cnt;
	test_dummy[7] = comm_loop_cnt;
	test_dummy[8] = comm_loop_cnt;
	test_dummy[9] = comm_loop_cnt;

	t_count++;

	return 0;
}

/* ------------------- TRAJECTORY ------------------- */
static int Check_Trajectory_Error(uint16_t t_frame_idx)
{
	if((t_frame_idx % D10_TRAJECTORY_ELEMENT_NUMBER) != 0)		{return -1;}
	if( (t_frame_idx - trajectory_buffer.frame_idx) != 0)		{return -2;}

	trajectory_buffer.frame_idx += D10_TRAJECTORY_ELEMENT_NUMBER;

	return 0;
}

static int Assemble_Trajectory(uint8_t* t_byte_arr)
{
	uint8_t t_cursor = 0, t_check = 0, t_buf = 0;
	uint16_t t_index = 0;
	int8_t t_ack = 0;
    uint16_t t_identifier = TRAJECTORY|(MD_node_id<<4)|NODE_ID_CM;

	/* Get index */
	t_cursor = 0;
	memcpy(&t_index, &t_byte_arr[t_cursor], 2);
	t_cursor += 2;

	/* Check Error*/
	t_check = Check_Trajectory_Error(t_index);

	if(t_check != 0) {
		trajectory_buffer.frame_idx = 0;
		t_ack = -2;
	} else {

		/* Save Buffer */
		for(int i = 0; i < D10_TRAJECTORY_ELEMENT_NUMBER; ++i){
			memcpy(&trajectory_buffer.buff[t_index++], &t_byte_arr[t_cursor], 4);
			t_cursor += 4;
		}

		/* Check End of Trajectory */
		if(t_index >= D10_TRAJECTORY_TOTAL_LENGTH){
			t_ack = -1;
			trajectory_buffer.frame_idx = 0;
		} else {
			t_ack = 0;
		}
	}

	/* Send Acknowledgement */
	memcpy(&t_buf, &t_ack, 1);
	Send_MSG(t_identifier, 1, &t_buf);

	return t_check;
}

/* ------------------- LIST MANAGEMENT ------------------- */
static void Add_PDO_to_Send(uint8_t t_dod_id, uint8_t t_obj_id)
{
	MsgPDOStruct* temp_pdo = Find_PDO(t_dod_id, t_obj_id);
    if (temp_pdo == NULL) {
        //TODO: Cannot Find PDO Error
        return;
    }

    MsgDataObjectHeader t_pdo = {t_dod_id, t_obj_id};

    for (int i = 0; i < cvector_size(pdo_send_list); ++i) {
        if ((pdo_send_list[i].dod_id == t_dod_id) && (pdo_send_list[i].obj_id == t_obj_id)){
            return;
        }
    }
    cvector_push_back(pdo_send_list, t_pdo);
}

static void Clear_PDO_to_Send()
{
    cvector_free(pdo_send_list);
    pdo_send_list = NULL;
}


/* ------------------- MSG HANDLER ------------------- */
static void Send_USB_Trick(uint8_t* t_in_buf, uint32_t t_in_len, uint8_t* t_out_buf)
{
	/*
	 * This function is designed to prevent
	 * the continuous array of 'CR/LF' used
	 * as a terminal signal in matlab gui.
	 *
	 * Sometimes, when sending a float signal,
	 * a continuous array of 'CR/LF' is created by coincidence,
	 * which interrupts the USB communication between the GUI
	 * and MD and breaks the sequence of the GUI.
	 *
	 * Therefore, 0x00 is inserted between every byte and sent.
	 * */

	for(int i = 0; i < t_in_len; ++i){
		*(t_out_buf + (2*i)) = *(t_in_buf + i);
		*(t_out_buf + (2*i+1)) = 0;
	}
}

int Send_MSG(uint16_t t_COB_ID, uint32_t t_len, uint8_t* t_tx_data)
{
	static uint8_t t_fnc_code = 0, t_node_id = 0;
	int t_check = 0;
	uint8_t t_txBuf[67] = {0};
	uint8_t t_usb_txBuf[137] = {0};

	if(comm_type == e_FDCAN){
		if (IOIF_TransmitFDCAN1(t_COB_ID, t_len, t_tx_data) != 0) {
			return t_check;
			//TODO: MSG TX ERROR
		}
	} else if(comm_type == e_USB){

		t_fnc_code = (t_COB_ID & 0xF00) >> 8;
		t_node_id = (t_COB_ID & 0xFF);

		memcpy(&t_txBuf[2], t_tx_data, t_len);
		memcpy(t_txBuf, &t_fnc_code, 1);			t_len++;
		memcpy(&t_txBuf[1], &t_node_id, 1);			t_len++;


		Send_USB_Trick(t_txBuf, t_len, t_usb_txBuf);
		t_len *= 2;

		t_usb_txBuf[t_len++] = '\r';
		t_usb_txBuf[t_len++] = '\n';

		if(CDC_Transmit_FS(t_usb_txBuf, t_len) != 0){
			return t_check;
			//TODO: MSG TX ERROR
		}
	}

	return -1;
}

static int USB_Rx_Hdlr(uint8_t* t_Buf, uint32_t* t_Len)
{
	uint32_t t_cursor = 0;

	fnc_code = ((uint16_t)*t_Buf) << 8;
	t_cursor++;

	ori_node = ((*(t_Buf+t_cursor)) & 0xF0)>>4;
	t_cursor++;

	memcpy(usbRxData, &t_Buf[t_cursor], *t_Len);

	switch(fnc_code) {

		case EMCY:
			Recv_EMCY(usbRxData, &err_code);
			// TODO: ERROR Process
			break;

		case SDO:
			if (Unpack_SDO(usbRxData) < 0) {
				return SDO_RX_ERR;
			} else{
				Send_SDO(ori_node);
			}
			break;

		case PDO:
			if (Unpack_PDO(usbRxData) < 0) {
				return PDO_RX_ERR;
			} else{
				Run_Send_PDO(ori_node);
			}
			break;

		default: break;
	}

	return 0;
}

static int Fdcan_Rx_Hdlr(uint16_t t_wasp_id, uint8_t* t_rx_data)
{
    fnc_code = t_wasp_id & 0x700;
    ori_node = (t_wasp_id & 0x0F0)>>4;

    switch(fnc_code) {

        case EMCY:
            Recv_EMCY(t_rx_data, &err_code);
            // TODO: ERROR Process
            break;

        case SDO:
            if (Unpack_SDO(t_rx_data) < 0) {
                return SDO_RX_ERR;
            } else{
                Send_SDO(ori_node);
            }
            break;

        case PDO:
            if (Unpack_PDO(t_rx_data) < 0) {
                return PDO_RX_ERR;
            } else{
            	Run_Send_PDO(ori_node);
            }
            break;

        case TRAJECTORY:
			Assemble_Trajectory(t_rx_data);
        	break;
        default: break;
    }

    return 0;
}

/* ------------------- SDO CALLBACK ------------------- */
static void Set_Send_PDO_List(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	Clear_PDO_to_Send();

    int t_cursor = 0;
    uint8_t* t_ids = (uint8_t*)t_req->data;
    while (t_cursor < 2*t_req->size) {
        uint8_t t_dod_id = t_ids[t_cursor++];
        uint8_t t_obj_id = t_ids[t_cursor++];
        Add_PDO_to_Send(t_dod_id, t_obj_id);
    }

    t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_MS_Enum(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&MS_enum, t_req->data, 1);

	t_res->size = 0;
	t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_GUI_COMM_OnOff(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&GUI_onoff, t_req->data, 1);

	t_res->size = 1;
	t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_GUI_COMM_Command(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&GUI_command, t_req->data, 1);

	t_res->size = 1;
	t_res->status = DATA_OBJECT_SDO_SUCC;
}

