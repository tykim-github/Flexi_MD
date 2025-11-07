

#include "../../Risk_Mngr_Task/Inc/risk_mngr.h"

// revised to 100ms timer interrupt!

/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */

#define temp_sensor 1

#define MAX_ERR_ABD 1.74532925199
#define MAX_ERR_ROT 0.087266462599716
#define MAX_ERR_FLX 0.087266462599716
#define MAX_ERR_ANK 1.74532925199

#define MAX_CURR1_ABD 7
#define MAX_CURR1_ROT 6
#define MAX_CURR1_FLX 50
#define MAX_CURR1_ANK 8

#define MAX_CURR2_ABD 10.5   //10
#define MAX_CURR2_ROT 9      // 9
#define MAX_CURR2_FLX 75     //70
#define MAX_CURR2_ANK 12     //10

#define MAX_TEMP_L 80
#define MAX_TEMP_H 100

#define MAX_TEMP_BD_L 90
#define MAX_TEMP_BD_H 100

/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Variables accessible throughout the application.
 */

TaskStruct risk_mngr_task;

uint16_t packed_risk;
RiskObj  Risk[N_risk];
uint8_t  act_type;
uint32_t overtemp_cnt;

uint32_t risk_cnt;

extern float lowTimeElap;
extern float midTimeElap;

static uint8_t testTimRes = IOIF_TIM_OK;

/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */


/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

/* ------------------- STATE FUNCTION ------------------- */
static void StateOff_Run();
static void StateStandby_Run();
static void StateEnable_Ent();
static void StateEnable_Run();
static void StateEnable_Ext();

/* ------------------- SDO CALLBACK ------------------- */


/* ------------------- ROUTINE ------------------- */


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

static void Set_Risk(RiskObj *Obj, uint8_t riskcode, void (*det_fnc)(void), void (*hdl_fnc)(void));
static void Assign_Risk(uint8_t riskcode, void (*det_fnc)(void), void (*hdl_fnc)(void));

static void Detect_Act_OverCurr_Low();
static void Detect_Act_OverCurr_High();
static void Detect_Act_OverCurr_Peak();
static void Detect_Act_ROM_Lim();
static void Detect_Act_OverTemp_Low();
static void Detect_Act_OverTemp_High();
static void Detect_MD_RT_Break();
static void Detect_Act_Error_Lim();
static void Detect_MD_Main_Off();
static void Detect_Act_IncEnc_Error();
static void Detect_Act_AbsEnc_Error();
static void Detect_Act_TempSensor_Error();
static void Detect_Act_HallSensor_Error();
static void Detect_MD_OverTemp_Low();
static void Detect_MD_OverTemp_High();
static void Detect_MD_3Phase_OverCurr_Peak();

static void Handle_Act_OverCurr_Low();
static void Handle_Act_OverCurr_High();
static void Handle_Act_OverCurr_Peak();
static void Handle_Act_ROM_Lim();
static void Handle_Act_OverTemp_Low();
static void Handle_Act_OverTemp_High();
static void Handle_MD_RT_Break();
static void Handle_Act_Error_Lim();
static void Handle_MD_Main_Off();
static void Handle_Act_IncEnc_Error();
static void Handle_Act_AbsEnc_Error();
static void Handle_Act_TempSensor_Error();
static void Handle_Act_HallSensor_Error();
static void Handle_MD_OverTemp_Low();
static void Handle_MD_OverTemp_High();
static void Handle_MD_3Phase_OverCurr_Peak();

//MSG_COMMON_SDO_CALLBACK(risk_mngr_task)

void InitRiskMngrCtrl(void)
{
	/* Init */
	Init_Task(&risk_mngr_task);

	/* State Definition */
	TASK_CREATE_STATE(&risk_mngr_task, e_State_Off,      NULL,   				StateOff_Run,       NULL,         		 true);
	TASK_CREATE_STATE(&risk_mngr_task, e_State_Standby,  NULL,   				StateStandby_Run,	NULL,         		 false);
	TASK_CREATE_STATE(&risk_mngr_task, e_State_Enable,   StateEnable_Ent,       StateEnable_Run, 	StateEnable_Ext,     false);
	TASK_CREATE_STATE(&risk_mngr_task, e_State_Error,    NULL,   				NULL,             	NULL,         		 false);

	/* Routine Definition */
//#if defined(L30_MD_REV06_ENABLED) || defined (L30_MD_REV07_ENABLED) || defined (L30_MD_REV08_ENABLED)
//	TASK_CREATE_ROUTINE(&ext_dev_ctrl_task, ROUTINE_ID_EXTDEV_FOOT,             NULL,   GetFootVal, 			NULL);
//	TASK_CREATE_ROUTINE(&ext_dev_ctrl_task, ROUTINE_ID_EXTDEV_DC_LENGTH_UPDATE, NULL,   GetUprightLength,       NULL);
//	TASK_CREATE_ROUTINE(&ext_dev_ctrl_task, ROUTINE_ID_EXTDEV_DC_LENGTH_CMD, 	NULL,   RunUprightByLen, 		NULL);
//	TASK_CREATE_ROUTINE(&ext_dev_ctrl_task, ROUTINE_ID_EXTDEV_DC_BUTTON_CMD, 	NULL,   RunUprightByBT, 		NULL);
//#endif
//
//	TASK_CREATE_ROUTINE(&ext_dev_ctrl_task, ROUTINE_ID_EXTDEV_NTC, 				NULL,   GetMotorTemp,           NULL);

	TASK_CREATE_ROUTINE(&risk_mngr_task, ROUTINE_ID_RISK_DETECT_RISK, Init_Risk, Detect_Risk, NULL);
	TASK_CREATE_ROUTINE(&risk_mngr_task, ROUTINE_ID_RISK_HANDLE_RISK, NULL,      Handle_Risk, NULL);
	TASK_CREATE_ROUTINE(&risk_mngr_task, ROUTINE_ID_RISK_PACK_RISK,   NULL,      Pack_Risk,   NULL);
    /* DOD Definition */
    // DOD
    Create_DOD(TASK_ID_RISK);

    // PDO
//#if defined(L30_MD_REV06_ENABLED) || defined (L30_MD_REV07_ENABLED) || defined (L30_MD_REV08_ENABLED)
//    Create_PDO(TASK_ID_RISK, PDO_ID_RISK_FSR, 					e_Float32, 	1, &fsrObj.fsrVolt);
//    Create_PDO(TASK_ID_RISK, PDO_ID_RISK_LP,  					e_Float32, 	1, &lpObj.lpVolt);
//    Create_PDO(TASK_ID_RISK, PDO_ID_RISK_DC_LENGTH_REF,  		e_Float32, 	1, &uprightObj.lengthRef);
//    Create_PDO(TASK_ID_RISK, PDO_ID_RISK_DC_DIRECTION_CMD,  	e_UInt8, 	1, &uprightObj.cmd); // TODO : need to change uint16_t
//    Create_PDO(TASK_ID_RISK, PDO_ID_RISK_DC_LENGTH_ACT,  		e_Float32, 	1, &uprightObj.lengthAct);
//    Create_PDO(TASK_ID_RISK, PDO_ID_RISK_DC_DIRECTION_ACT,  	e_UInt8, 	1, &uprightObj.state);
//#endif
//	// SDO
//    MSG_COMMON_SDO_CREATE(TASK_ID_RISK)
//
//#if defined(L30_MD_REV06_ENABLED) || defined (L30_MD_REV07_ENABLED) || defined (L30_MD_REV08_ENABLED)
//    Create_SDO(TASK_ID_RISK, SDO_ID_RISK_DC_SET_LENGTH,    e_Float32,  Set_DcMotor_Length_Cmd);
//    Create_SDO(TASK_ID_RISK, SDO_ID_RISK_DC_SET_DIRECT,    e_UInt8,    Set_DcMotor_Direction_Cmd);
//#endif

    /* Timer 16 Callback Allocation */
	if(IOIF_StartTimIT(IOIF_TIM16) > 0){
		//TODO: ERROR PROCESS
		testTimRes = IOIF_TIM_ERROR;
	}

	IOIF_SetTimCB(IOIF_TIM16, IOIF_TIM_PERIOD_ELAPSED_CALLBACK, RunRiskMngrCtrl, NULL);
}

void RunRiskMngrCtrl(void* params)
{
	Run_Task(&risk_mngr_task);
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/* ------------------- STATE FUNCTION ------------------- */
static void StateOff_Run()
{
    Transition_State(&risk_mngr_task.state_machine, e_State_Standby);
}

static void StateStandby_Run()
{
    Transition_State(&risk_mngr_task.state_machine, e_State_Enable);
//	Push_Routine(&risk_mngr_task.routine, ROUTINE_ID_RISK_DETECT_RISK);
}

static void StateEnable_Ent()
{
	Init_Risk();
	//    Ent_Routines(&risk_mngr_task.routine);
}

static void StateEnable_Run()
{
//    Run_Routines(&risk_mngr_task.routine);
	Detect_Risk();
	Pack_Risk();
	Handle_Risk();

}

static void StateEnable_Ext()
{
    Clear_Routines(&risk_mngr_task.routine);
}

//********************Functions********************//

void Init_Risk()
{
	Assign_Risk(MD_Act_overcurrent_low,  Detect_Act_OverCurr_Low         , Handle_Act_OverCurr_Low     );
	Assign_Risk(MD_Act_overcurrent_high, Detect_Act_OverCurr_High        , Handle_Act_OverCurr_High    );
	Assign_Risk(MD_Act_overcurrent_peak, Detect_Act_OverCurr_Peak        , Handle_Act_OverCurr_Peak    );
	Assign_Risk(MD_Act_ROM_lim,          Detect_Act_ROM_Lim              , Handle_Act_ROM_Lim          );
	Assign_Risk(MD_Act_overtemp_low,     Detect_Act_OverTemp_Low         , Handle_Act_OverTemp_Low     );
	Assign_Risk(MD_Act_overtemp_high,    Detect_Act_OverTemp_High        , Handle_Act_OverTemp_High    );
	Assign_Risk(MD_RT_breakage,          Detect_MD_RT_Break              , Handle_MD_RT_Break          );
	Assign_Risk(MD_Act_error_lim,        Detect_Act_Error_Lim            , Handle_Act_Error_Lim        );
	Assign_Risk(MD_main_off,             Detect_MD_Main_Off        	     , Handle_MD_Main_Off          );
	Assign_Risk(MD_Act_incenc_error,     Detect_Act_IncEnc_Error         , Handle_Act_IncEnc_Error     );
	Assign_Risk(MD_Act_absenc_error,     Detect_Act_AbsEnc_Error         , Handle_Act_AbsEnc_Error     );
	Assign_Risk(MD_Act_tempsensor_error, Detect_Act_TempSensor_Error     , Handle_Act_TempSensor_Error );
	Assign_Risk(MD_Act_hallsensor_error, Detect_Act_HallSensor_Error     , Handle_Act_HallSensor_Error );
	Assign_Risk(MD_overtemp_low,         Detect_MD_OverTemp_Low          , Handle_MD_OverTemp_Low      );
	Assign_Risk(MD_overtemp_high,        Detect_MD_OverTemp_High         , Handle_MD_OverTemp_High     );
	Assign_Risk(MD_3Phase_overcurrent,   Detect_MD_3Phase_OverCurr_Peak  , Handle_MD_3Phase_OverCurr_Peak );

}

void Detect_Risk(void)
{
	for (int i = 0 ; i < N_risk ; i++) {
		if (Risk[i].det_fnc != NULL){
			Risk[i].det_fnc();
		}
	}
}

void Handle_Risk(void)
{
	for (int i = 0 ; i < N_risk ; i++) {
		if (Risk[i].risk_detected == ON){
			Risk[i].hdl_fnc();
			}
		}
}

void Set_Risk(RiskObj *Obj, uint8_t riskcode, void (*det_fnc)(void), void (*hdl_fnc)(void))
{
	Obj->errortype = riskcode;
	Obj->risk_detected = OFF;
	Obj->det_fnc   = det_fnc;
	Obj->hdl_fnc   = hdl_fnc;
}

void Assign_Risk(uint8_t riskcode, void (*det_fnc)(void), void (*hdl_fnc)(void))
{
	static uint8_t index = 0;
	Set_Risk(&Risk[index++], riskcode, det_fnc, hdl_fnc);
}

void Pack_Risk(void)
{
	packed_risk = 0;

	for (int i = 0 ; i < N_risk; i++) {
		packed_risk |= (Risk[i].risk_detected & 0x01) << i;
	}
}

////////////Detection Code///////////

static void Detect_Act_OverCurr_Low(void)
{
	static uint32_t over_curr_cnt1  = 0;
	static uint32_t under_curr_cnt1 = 0;

	if ((act_type == 2) || (act_type == 3)) {     // hip abductor
		if ((motor_in.total_current_input > MAX_CURR1_ABD) || (motor_in.total_current_input < -MAX_CURR1_ABD)) {
			over_curr_cnt1 ++;
			under_curr_cnt1 = 0;
		}
		else {
			under_curr_cnt1 ++;
			over_curr_cnt1 = 0;
		}
	}

	else if ((act_type == 4) || (act_type == 5)) {     // hip rotator
		if ((motor_in.total_current_input > MAX_CURR1_ROT) || (motor_in.total_current_input < -MAX_CURR1_ROT)) {
			over_curr_cnt1 ++;
			under_curr_cnt1 = 0;
		}
		else {
			under_curr_cnt1 ++;
			over_curr_cnt1 = 0;
		}
	}

	else if ((act_type == 6) || (act_type == 7) || (act_type == 8) || (act_type == 9)) {      // hip & knee saggital
		if ((motor_in.total_current_input > MAX_CURR1_FLX) || (motor_in.total_current_input < -MAX_CURR1_FLX)) {
			over_curr_cnt1 ++;
			under_curr_cnt1 = 0;
		}
		else {
			under_curr_cnt1 ++;
			over_curr_cnt1 = 0;
		}
	}

	else if ((act_type == 10) || (act_type == 11) || (act_type == 12) || (act_type == 13)) {  // ankle
		if ((motor_in.total_current_input > MAX_CURR1_ANK) || (motor_in.total_current_input < -MAX_CURR1_ANK)) {
			over_curr_cnt1 ++;
			under_curr_cnt1 = 0;
		}
		else {
			under_curr_cnt1 ++;
			over_curr_cnt1 = 0;
		}
	}

	if (over_curr_cnt1 > 30) {
		over_curr_cnt1 = 0;
		Risk[MD_Act_overcurrent_low].risk_detected = ON;
	}

	if (under_curr_cnt1 > 30) {
		under_curr_cnt1 = 0;
		Risk[MD_Act_overcurrent_low].risk_detected = OFF;
	}
}

static void Detect_Act_OverCurr_High(void)
{
	static uint32_t over_curr_cnt2  = 0;
	static uint32_t under_curr_cnt2 = 0;
	static uint32_t ema = 0;
	static uint32_t ema_cont = 0;
	static float alpha = 0.99;

	ema = (1-alpha)*ema + alpha*motor_in.total_current_input*motor_in.total_current_input;

	if ((act_type == 2) || (act_type == 3)) {     // hip abductor
		ema_cont = (1-alpha)*ema_cont + alpha*MAX_CURR1_ABD*MAX_CURR1_ABD;
	}
	else if ((act_type == 4) || (act_type == 5)) {
		ema_cont = (1-alpha)*ema_cont + alpha*MAX_CURR1_ROT*MAX_CURR1_ROT;
	}
	else if ((act_type == 6) || (act_type == 7) || (act_type == 8) || (act_type == 9)) {
		ema_cont = (1-alpha)*ema_cont + alpha*MAX_CURR1_FLX*MAX_CURR1_FLX;
	}
	else if ((act_type == 10) || (act_type == 11) || (act_type == 12) || (act_type == 13)) {
		ema_cont = (1-alpha)*ema_cont + alpha*MAX_CURR1_ANK*MAX_CURR1_ANK;
	}

	if (ema > ema_cont) {
		over_curr_cnt2 ++;
		under_curr_cnt2 = 0;
	}
	else {
		under_curr_cnt2 ++;
		over_curr_cnt2 = 0;
	}

	if (over_curr_cnt2 > 30) {
		over_curr_cnt2 = 0;
		Risk[MD_Act_overcurrent_high].risk_detected = ON;
	}

	if (under_curr_cnt2 > 30) {
		under_curr_cnt2 = 0;
		Risk[MD_Act_overcurrent_high].risk_detected = OFF;
	}
}

static void Detect_Act_OverCurr_Peak(void)
{
	static uint32_t over_curr_cnt3  = 0;
	static uint32_t under_curr_cnt3 = 0;

	if ((act_type == 2) || (act_type == 3)) {     // hip abductor
		if ((motor_in.total_current_input > MAX_CURR2_ABD) || (motor_in.total_current_input < -MAX_CURR2_ABD)) {
			over_curr_cnt3 ++;
			under_curr_cnt3 = 0;
		}
		else {
			under_curr_cnt3 ++;
			over_curr_cnt3 = 0;
		}
	}

	else if ((act_type == 4) || (act_type == 5)) {     // hip rotator
		if ((motor_in.total_current_input > MAX_CURR2_ROT) || (motor_in.total_current_input < -MAX_CURR2_ROT)) {
			over_curr_cnt3 ++;
			under_curr_cnt3 = 0;
		}
		else {
			under_curr_cnt3 ++;
			over_curr_cnt3 = 0;
		}
	}

	else if ((act_type == 6) || (act_type == 7) || (act_type == 8) || (act_type == 9)) {      // hip & knee saggital
		if ((motor_in.total_current_input > MAX_CURR2_FLX) || (motor_in.total_current_input < -MAX_CURR2_FLX)) {
			over_curr_cnt3 ++;
			under_curr_cnt3 = 0;
		}
		else {
			under_curr_cnt3 ++;
			over_curr_cnt3 = 0;
		}
	}

	else if ((act_type == 10) || (act_type == 11) || (act_type == 12) || (act_type == 13)) {  // ankle
		if ((motor_in.total_current_input > MAX_CURR2_ANK) || (motor_in.total_current_input < -MAX_CURR2_ANK)) {
			over_curr_cnt3 ++;
			under_curr_cnt3 = 0;
		}
		else {
			under_curr_cnt3 ++;
			over_curr_cnt3 = 0;
		}
	}

	if (over_curr_cnt3 > 3) {
		over_curr_cnt3 = 0;
		Risk[MD_Act_overcurrent_peak].risk_detected = ON;
	}

	if (under_curr_cnt3 > 30) {
		under_curr_cnt3 = 0;
		Risk[MD_Act_overcurrent_peak].risk_detected = OFF;
	}
}

static void Detect_Act_ROM_Lim(void)
{
	//static uint32_t rom_lim_cnt = 0;
}

static void Detect_Act_OverTemp_Low(void)
{
// ON  for actuators with    temperature sensor
// OFF for actuators without temperature sensor

	if (temp_sensor == 1) {
		static uint32_t overtemp1_cnt = 0;
		static uint32_t undertemp1_cnt = 0;

		if (temp_sense.stator_filtered > MAX_TEMP_L)
		{
			overtemp1_cnt ++;
			undertemp1_cnt = 0;
		}
		else
		{
			overtemp1_cnt = 0;
			undertemp1_cnt ++;
		}

		if (overtemp1_cnt > 10)
		{
			overtemp1_cnt = 0;
			Risk[MD_Act_overtemp_low].risk_detected = ON;
		}

		if (undertemp1_cnt > 30)
		{
			undertemp1_cnt = 0;
			Risk[MD_Act_overtemp_low].risk_detected = OFF;
		}
	}
}

static void Detect_Act_OverTemp_High(void)
{
	// ON  for actuators with    temperature sensor
	// OFF for actuators without temperature sensor

		if (temp_sensor == 1) {
			static uint32_t overtemp2_cnt = 0;
			static uint32_t undertemp2_cnt = 0;

			if (temp_sense.stator_filtered > MAX_TEMP_H)
			{
				overtemp2_cnt ++;
				undertemp2_cnt = 0;
			}
			else
			{
				overtemp2_cnt = 0;
				undertemp2_cnt ++;
			}

			if (overtemp2_cnt > 10)
			{
				overtemp2_cnt = 0;
				Risk[MD_Act_overtemp_high].risk_detected = ON;
			}

			if (undertemp2_cnt > 50)
			{
				undertemp2_cnt = 0;
				Risk[MD_Act_overtemp_high].risk_detected = OFF;
			}
		}
}


static void Detect_MD_3Phase_OverCurr_Peak(void)
{
	static uint32_t phaseU_over_curr_cnt  = 0;
	static uint32_t phaseU_under_curr_cnt = 0;

	static uint32_t phaseV_over_curr_cnt  = 0;
	static uint32_t phaseV_under_curr_cnt = 0;

	static uint32_t phaseW_over_curr_cnt  = 0;
	static uint32_t phaseW_under_curr_cnt = 0;

//	float margin = 1.4;
	float margin = 2;
//	float margin = 0.6667;

	float D2A = motor_setting.sensor_setting.curr_sensor_d2a;

	// Phase U Check
	if (((motor_out.I_U*D2A) > margin*motor_setting.contCurr_limit) || ((motor_out.I_U*D2A) < -margin*motor_setting.contCurr_limit))
	{
		phaseU_over_curr_cnt++;
		phaseU_under_curr_cnt = 0;
	}
	else
	{
		phaseU_under_curr_cnt++;
		phaseU_over_curr_cnt = 0;
	}

	// Phase V Check
	if (((motor_out.I_V*D2A) > margin*motor_setting.contCurr_limit) || ((motor_out.I_V*D2A) < -margin*motor_setting.contCurr_limit))
	{
		phaseV_over_curr_cnt++;
		phaseV_under_curr_cnt = 0;
	}
	else
	{
		phaseV_under_curr_cnt++;
		phaseV_over_curr_cnt = 0;
	}

	// Phase W Check
	if (((motor_out.I_W*D2A) > margin*motor_setting.contCurr_limit) || ((motor_out.I_W*D2A) < -margin*motor_setting.contCurr_limit)) {
		phaseW_over_curr_cnt++;
		phaseW_under_curr_cnt = 0;
	}
	else
	{
		phaseW_under_curr_cnt++;
		phaseW_over_curr_cnt = 0;
	}


	if (phaseU_over_curr_cnt > 3) {
		phaseU_over_curr_cnt = 0;
		Risk[MD_3Phase_overcurrent].risk_detected = ON;
	}
	if (phaseU_under_curr_cnt > 3) {
		phaseU_under_curr_cnt = 0;
		Risk[MD_3Phase_overcurrent].risk_detected = OFF;
	}

	if (phaseV_over_curr_cnt > 3) {
		phaseV_over_curr_cnt = 0;
		Risk[MD_3Phase_overcurrent].risk_detected = ON;
	}
	if (phaseV_under_curr_cnt > 3) {
		phaseV_under_curr_cnt = 0;
		Risk[MD_3Phase_overcurrent].risk_detected = OFF;
	}

	if (phaseW_over_curr_cnt > 3) {
		phaseW_over_curr_cnt = 0;
		Risk[MD_3Phase_overcurrent].risk_detected = ON;
	}
	if (phaseW_under_curr_cnt > 3) {
		phaseW_under_curr_cnt = 0;
		Risk[MD_3Phase_overcurrent].risk_detected = OFF;
	}


}

static void Detect_MD_RT_Break(void)
{
	if ((lowTimeElap > 50) | (midTimeElap > 50)) {
		Risk[MD_RT_breakage].risk_detected = ON;
	}
	else {
		Risk[MD_RT_breakage].risk_detected = OFF;
	}
}

static void Detect_Act_Error_Lim(void)
{
	static uint32_t over_error_cnt = 0;
	static uint32_t under_error_cnt = 0;

	if ((act_type == 2) || (act_type == 3)) {     // hip abductor
//		if ((posCtrl.err > MAX_ERR_ABD) || (posCtrl.err < -MAX_ERR_ABD)) {
//			over_error_cnt ++;
//			under_error_cnt = 0;
//		}
//		else {
//			under_error_cnt ++;
//			over_error_cnt = 0;
//		}
	}

	else if ((act_type == 4) || (act_type == 5)) {     // hip rotator
//		if ((posCtrl.err > MAX_ERR_ROT) || (posCtrl.err < -MAX_ERR_ROT)) {
//			over_error_cnt ++;
//			under_error_cnt = 0;
//		}
//		else {
//			under_error_cnt ++;
//			over_error_cnt = 0;
//		}
	}

	else if ((act_type == 6) || (act_type == 7) || (act_type == 8) || (act_type == 9)) {      // hip & knee saggital
		if ((posCtrl.err > MAX_ERR_FLX) || (posCtrl.err < -MAX_ERR_FLX)) {
			over_error_cnt ++;
			under_error_cnt = 0;
		}
		else {
			under_error_cnt ++;
			over_error_cnt = 0;
		}
	}

	else if ((act_type == 10) || (act_type == 11) || (act_type == 12) || (act_type == 13)) {  // ankle
//		if ((posCtrl.err > MAX_ERR_ANK) || (posCtrl.err < -MAX_ERR_ANK)) {
//			over_error_cnt ++;
//			under_error_cnt = 0;
//		}
//		else {
//			under_error_cnt ++;
//			over_error_cnt = 0;
//		}
	}

	if (over_error_cnt > 30) {
		over_error_cnt = 0;
		Risk[MD_Act_error_lim].risk_detected = ON;
	}

	if (under_error_cnt > 30) {
		under_error_cnt = 0;
		Risk[MD_Act_error_lim].risk_detected = OFF;
	}
}

static void Detect_MD_Main_Off(void)
{

}

static void Detect_Act_IncEnc_Error(void)
{

}

static void Detect_Act_AbsEnc_Error(void)
{

}

static void Detect_Act_TempSensor_Error(void)
{

}

static void Detect_Act_HallSensor_Error(void)
{

	Read_Hall_Sensors(&hallObj);

	if (hallObj.error_cnt_risk != 0)
	{
		Risk[MD_Act_hallsensor_error].risk_detected = ON;
	}
	else
	{
		Risk[MD_Act_hallsensor_error].risk_detected = OFF;
	}
}

static void Detect_MD_OverTemp_Low(void)
{
	static uint32_t bd_overtemp1_cnt = 0;
	static uint32_t bd_undertemp1_cnt = 0;

	if (batData.brdTemp > MAX_TEMP_BD_L)
	{
		bd_overtemp1_cnt ++;
		bd_undertemp1_cnt = 0;
	}
	else
	{
		bd_overtemp1_cnt = 0;
		bd_undertemp1_cnt ++;
	}

	if (bd_overtemp1_cnt > 10)
	{
		bd_overtemp1_cnt = 0;
		Risk[MD_overtemp_low].risk_detected = ON;
	}

	if (bd_undertemp1_cnt > 30)
	{
		bd_undertemp1_cnt = 0;
		Risk[MD_overtemp_low].risk_detected = OFF;
	}
}

static void Detect_MD_OverTemp_High(void)
{
	static uint32_t bd_overtemp2_cnt = 0;
	static uint32_t bd_undertemp2_cnt = 0;

	if (batData.brdTemp > MAX_TEMP_BD_H)
	{
		bd_overtemp2_cnt ++;
		bd_undertemp2_cnt = 0;
	}
	else
	{
		bd_overtemp2_cnt = 0;
		bd_undertemp2_cnt ++;
	}

	if (bd_overtemp2_cnt > 10)
	{
		bd_overtemp2_cnt = 0;
		Risk[MD_overtemp_high].risk_detected = ON;
	}

	if (bd_undertemp2_cnt > 30)
	{
		bd_undertemp2_cnt = 0;
		Risk[MD_overtemp_high].risk_detected = OFF;
	}
}

////////////Handling Code/////////////

static void Handle_Act_OverCurr_Low(void)
{

}

static void Handle_Act_OverCurr_High(void)
{

}

static void Handle_Act_OverCurr_Peak(void)
{

}

static void Handle_Act_ROM_Lim(void)
{

}

static void Handle_Act_OverTemp_Low(void)
{

}

static void Handle_Act_OverTemp_High(void)
{

}

static void Handle_MD_RT_Break(void)
{

}

static void Handle_Act_Error_Lim(void)
{

}

static void Handle_MD_Main_Off(void)
{

}

static void Handle_Act_IncEnc_Error(void)
{

}

static void Handle_Act_AbsEnc_Error(void)
{

}

static void Handle_Act_TempSensor_Error(void)
{

}

static void Handle_Act_HallSensor_Error(void)
{

}

static void Handle_MD_OverTemp_Low(void)
{

}

static void Handle_MD_OverTemp_High(void)
{

}

static void Handle_MD_3Phase_OverCurr_Peak(void)
{
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_E, LED_TEST_Pin,     IOIF_GPIO_PIN_RESET);
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_D, LED_BOOT_RED_Pin, IOIF_GPIO_PIN_SET);

	Transition_State(&low_level_ctrl_task.state_machine, e_State_Standby);
	Transition_State(&mid_level_ctrl_task.state_machine, e_State_Standby);
}

