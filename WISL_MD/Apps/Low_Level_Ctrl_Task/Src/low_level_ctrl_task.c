/**
 * @file low_level_ctrl_task.c
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics FW Team
 * @brief 
 * @ref 
 */

// TODO : refactoring!!
#include "../../../Low_Level_Ctrl_Task/Inc/low_level_ctrl_task.h"

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

TaskStruct 				low_level_ctrl_task;
MotorProperties 		motor_properties;
MotorSetting			motor_setting;
MotorIn 				motor_in;
MotorOut				motor_out;
KFObject        		kf_current_object;
IOIF_IncEnc_t			inc25KhzObj;
IOIF_HallSensor_t       hallObj;
Advanced_Friction_ID 	advanced_friction_id;
TempSense               temp_sense;
uint8_t                 low_level_triggered_msg_stop;

/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */

static MotorElecSystemID 	elec_system_id;
// static CurrentCtrlVerify 	current_ctrl_verify;

static PIDObject 			pid_d_axis_ctrl;       // d-axis current controller for FOC
static PIDObject 			pid_q_axis_ctrl;       // q-axis current controller for FOC
static PIDObject			pid_trape_ctrl;

static ClarkeIn 			clarke_in;
static ClarkeOut 			clarke_out;
// static InvClarkeIn		inv_clarke_in;
// static InvClarkeOut 		inv_clarke_out;
static ParkIn 	    		park_in;
static ParkOut 	    		park_out;
static InvParkIn 	    	inv_park_in;
// static InvParkOut		inv_park_out;
static Phasor 				svm_phasor;
static PeriodicSignal 	    ll_cur_periodic_sig;

static TrapezoidalControl trape_ctrl;

static SharedArrayBuffer shared_array_buffer;

static uint32_t low_level_ctrl_loop_cnt;

static uint16_t RT_Broken;
float lowTimeElap;

static float batch_result_1, batch_result_2;
static float rms_err, mean_vel;
static int N;

static uint16_t* rawCurr = {0};	// 0,1,2 UVW Phase Current & 3,4,5 UVW BEMF Voltage
static uint16_t* tempbuffer = {0};


static float ss_friction_gain = 1;
/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

/* ------------------- STATE FUNCTION ------------------- */
static void StateOff_Run();

static void StateStandby_Ent();
static void StateStandby_Run();
static void StateStandby_Ext();

static void StateEnable_Ent();
static void StateEnable_Run();
static void StateEnable_Ext();

static void StateError_Run();

/* ------------------- INITIALIZATION ------------------- */
static void Init_Kalman_Filter(KFObject * t_KF_obj, float t_A_KF, float t_B_KF, float t_C_KF, float t_Q_KF, float t_R_KF, float t_P_KF);
static void Init_Commutation();
static void Init_SensorUsage();
static void Init_Current_Ctrl();
static void Init_Position_Velocity();
static void Check_OverCurrent();

/* ------------------- FOC ------------------- */
static void Get_3Phase_Current();
static void Get_3Phase_Voltage();
static void Get_Position_Velocity(IOIF_IncEnc_t* t_inc_enc, MotorOut* t_motor_out);
static void Cal_Trape_Total_Current();
static void Cal_Elec_Angle();
static void Cal_Elec_Angle_with_Hall();
static void Run_Kalman_Filter();
static void Run_FOC_Ctrl(float t_current_in);

/* ------------------- TRAPEZOIDAL ------------------- */
static void Run_Trape_Ctrl(float t_currentRef);

/* ------------------- SDO CALLBACK ------------------- */
static void Set_Name(MsgSDOargs* req, MsgSDOargs* res);
static void Set_Pole_Pair(MsgSDOargs* req, MsgSDOargs* res);
static void Set_Enc_Resolution(MsgSDOargs* req, MsgSDOargs* res);
static void Set_Gear_Ratio(MsgSDOargs* req, MsgSDOargs* res);
static void Set_Torque_Constant(MsgSDOargs* req, MsgSDOargs* res);
static void Set_Velocity_Constant(MsgSDOargs* req, MsgSDOargs* res);
static void Set_Peak_Limit(MsgSDOargs* req, MsgSDOargs* res);
static void Set_Continu_Limit(MsgSDOargs* req, MsgSDOargs* res);
static void Set_Max_Velocity(MsgSDOargs* req, MsgSDOargs* res);
static void Set_Commutation_Duty(MsgSDOargs* req, MsgSDOargs* res);
static void Set_User_Direction(MsgSDOargs* req, MsgSDOargs* res);

static void Set_Elec_Sys_ID_Mag(MsgSDOargs* req, MsgSDOargs* res);
static void Set_Resistance(MsgSDOargs* req, MsgSDOargs* res);
static void Set_Inductance(MsgSDOargs* req, MsgSDOargs* res);
static void Set_BEMF_ID_velocity(MsgSDOargs* req, MsgSDOargs* res);
static void Set_BEMF_ID_Gain_PCTG(MsgSDOargs* req, MsgSDOargs* res);
static void Set_Curr_Ctrl_BW(MsgSDOargs* req, MsgSDOargs* res);

static void Set_Inertia(MsgSDOargs* req, MsgSDOargs* res);
static void Set_Damping_Coef(MsgSDOargs* req, MsgSDOargs* res);
static void Set_Mech_Model_a(MsgSDOargs* req, MsgSDOargs* res);
static void Set_Mech_Model_b(MsgSDOargs* req, MsgSDOargs* res);
static void Set_Friction_ID_Info(MsgSDOargs* req, MsgSDOargs* res);
static void Set_Friction_LUT_Info(MsgSDOargs* req, MsgSDOargs* res);

static void Set_Commutation_Sensor(MsgSDOargs* req, MsgSDOargs* res);
static void Set_Position_Feedback_Sensor(MsgSDOargs* req, MsgSDOargs* res);
static void Set_Elec_Angle_Homing_Sensor(MsgSDOargs* req, MsgSDOargs* res);
static void Set_Mech_Angle_Homing_Sensor(MsgSDOargs* req, MsgSDOargs* res);
static void Set_Sensor_Usage(MsgSDOargs* req, MsgSDOargs* res);
static void Set_Current_Periodic_Sig_Info(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_MD_Version(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_IncEncoder_Prescaler(MsgSDOargs* t_req, MsgSDOargs* t_res);

static void Set_BEMF_Comp_Gain(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_Friction_Comp_Gain_SS(MsgSDOargs* t_req, MsgSDOargs* t_res);

// Update 2024-10-10 by KHJ
static void Set_Current_KF_ONOFF(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_Current_KF_A(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_Current_KF_B(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_Current_KF_Q(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_Current_KF_R(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_BEMF_Constant(MsgSDOargs* t_req, MsgSDOargs* t_res);


/* ----------- HALL SENSOR INTERRUPT ROUTINE ----------- */
 void RunHallITCallback();

/* ------------------- ROUTINE ------------------- */
static int Ent_Elec_Angle_Homing();
static int Run_Elec_Angle_Homing();

static int Ent_Set_Commutation();
static int Run_Set_Commutation();

static int Ent_Electrical_System_ID();
static int Run_Electrical_System_ID();

static int Ent_BEMF_ID();
static int Run_BEMF_ID();
static int Ext_BEMF_ID();

static int Ent_Current_Control();
static int Run_Current_Control();

static int Ent_LL_Generate_Current_Sine();
static int Run_LL_Generate_Current_Sine();

static int Ent_LL_Generate_Current_Rec();
static int Run_LL_Generate_Current_Rec();

static int Ent_LL_Generate_Current_Tanh();
static int Run_LL_Generate_Current_Tanh();

static int Ent_Check_Current_Controller_Bandwidth();
static int Run_Check_Current_Controller_Bandwidth();
static int Ext_Check_Current_Controller_Bandwidth();

static int Ent_Advanced_Friction_ID();
static int Run_Advanced_Friction_ID();
static int Ext_Advanced_Friction_ID();

static int Ent_Evaluation_Friction_ID_Velocity_Ctrl();
static int Run_Evaluation_Friction_ID_Velocity_Ctrl();

static int Ent_Adv_Friction_Compensation();
static int Run_Adv_Friction_Compensation();

static int Ent_Adv_Friction_Compensation_SS();
static int Run_Adv_Friction_Compensation_SS();

static int Ent_Adv_Friction_Compensation_FF();
static int Run_Adv_Friction_Compensation_FF();

static int Ent_Check_Current_Controller_Tracking();
static int Run_Check_Current_Controller_Tracking();

static void Ent_Get_Temperature();
static void Run_Get_Temperature();

static void Run_Hall_Homing();
static void Elec_Angle_Calib();
static uint16_t Check_Elec_Angle_Offset(uint16_t elec_angle);

/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

MSG_COMMON_SDO_CALLBACK(low_level_ctrl_task)

void InitLowLvCtrl(void)
{
	/* init*/
	Init_SensorUsage();
   	Init_Commutation();
   	Init_Position_Velocity();

   	motor_properties.md_version = e_R08;

	/* Start DMA ADC1 for Current Sensor */
	if(IOIF_StartADCDMA(IOIF_ADC1, &rawCurr, IOIF_ADC1_BUFFER_LENGTH)) {
		//TODO: Error Process
	}

//	if(IOIF_StartADCDMA(IOIF_ADC3, &tempbuffer, IOIF_ADC3_BUFFER_LENGTH)){
//		//TODO: Error Process
//	}


//	Init_Kalman_Filter(&kf_current_object, kf_current_object.kf_A, kf_current_object.kf_B, kf_current_object.kf_C, 1, 10, 1);	// A, B, C, Q, R, P (Q = 1, R = 10)

	if (motor_properties.md_version == e_R08H)
		Init_Kalman_Filter(&kf_current_object, kf_current_object.kf_A, kf_current_object.kf_B, kf_current_object.kf_C, 0.1, 10, 1); // high power MD
	else
		Init_Kalman_Filter(&kf_current_object, kf_current_object.kf_A, kf_current_object.kf_B, kf_current_object.kf_C, 1, 10, 1);	// A, B, C, Q, R, P (Q = 1, R = 10)

	//Init_Kalman_Filter(&kf_current_object, kf_current_object.kf_A, kf_current_object.kf_B, 48.323, 1, 10, 1);	// A, B, C, Q, R, P (Q = 1, R = 10)

	uint8_t t_md_ver = motor_properties.md_version;
	if ((t_md_ver == 0) | (t_md_ver == 1) | (t_md_ver == 2) | (t_md_ver == 3)) {
		Init_PID(&pid_d_axis_ctrl, motor_properties.L * 0.4272 * (motor_setting.currCtrl_BW_radPsec), motor_properties.R * 0.4272 * (motor_setting.currCtrl_BW_radPsec), 0);
		Init_PID(&pid_q_axis_ctrl, motor_properties.L * 0.4272 * (motor_setting.currCtrl_BW_radPsec), motor_properties.R * 0.4272 * (motor_setting.currCtrl_BW_radPsec), 0);
		Init_PID(&pid_trape_ctrl,  motor_properties.L * 0.4272 * (motor_setting.currCtrl_BW_radPsec), motor_properties.R * 0.4272 * (motor_setting.currCtrl_BW_radPsec), 0);
	}
	else if (t_md_ver == 4)
	{
		Init_PID(&pid_d_axis_ctrl, motor_properties.L * 3.5355 * (motor_setting.currCtrl_BW_radPsec), motor_properties.R * 3.5355 * (motor_setting.currCtrl_BW_radPsec), 0);
		Init_PID(&pid_q_axis_ctrl, motor_properties.L * 3.5355 * (motor_setting.currCtrl_BW_radPsec), motor_properties.R * 3.5355 * (motor_setting.currCtrl_BW_radPsec), 0);
		Init_PID(&pid_trape_ctrl,  motor_properties.L * 3.5355 * (motor_setting.currCtrl_BW_radPsec), motor_properties.R * 3.5355 * (motor_setting.currCtrl_BW_radPsec), 0);
	}

	/* State Definition*/
	TASK_CREATE_STATE(&low_level_ctrl_task, e_State_Off,     NULL,     		 	StateOff_Run,   	NULL, 	            true);
	TASK_CREATE_STATE(&low_level_ctrl_task, e_State_Standby, StateStandby_Ent, 	StateStandby_Run,   StateStandby_Ext, 	false);
	TASK_CREATE_STATE(&low_level_ctrl_task, e_State_Enable,  StateEnable_Ent,  	StateEnable_Run, 	StateEnable_Ext, 	false);
	TASK_CREATE_STATE(&low_level_ctrl_task, e_State_Error,   NULL,             	StateError_Run,  	NULL, 				false);

	/* Routine Definition*/
    TASK_CREATE_ROUTINE(&low_level_ctrl_task,  ROUTINE_ID_LOWLEVEL_COMMUTATION_SET,  				Ent_Set_Commutation, 			     		Run_Set_Commutation, 						NULL);
    TASK_CREATE_ROUTINE(&low_level_ctrl_task,  ROUTINE_ID_LOWLEVEL_ELECTRICAL_SYS_ID, 				Ent_Electrical_System_ID, 					Run_Electrical_System_ID, 					NULL);
    TASK_CREATE_ROUTINE(&low_level_ctrl_task,  ROUTINE_ID_LOWLEVEL_ELECTRICAL_BEMF_ID, 				Ent_BEMF_ID,								Run_BEMF_ID, 								Ext_BEMF_ID);
    TASK_CREATE_ROUTINE(&low_level_ctrl_task,  ROUTINE_ID_LOWLEVEL_CURRENT_CTRL, 					Ent_Current_Control, 						Run_Current_Control, 						NULL);
    TASK_CREATE_ROUTINE(&low_level_ctrl_task,  ROUTINE_ID_LOWLEVEL_CURRENT_CTRL_BANDWIDTH_CHECK,	Ent_Check_Current_Controller_Bandwidth,		Run_Check_Current_Controller_Bandwidth,		Ext_Check_Current_Controller_Bandwidth);
    TASK_CREATE_ROUTINE(&low_level_ctrl_task,  ROUTINE_ID_LOWLEVEL_ADVANCED_FRICTION_ID,        	Ent_Advanced_Friction_ID,					Run_Advanced_Friction_ID, 					Ext_Advanced_Friction_ID);
    TASK_CREATE_ROUTINE(&low_level_ctrl_task,  ROUTINE_ID_LOWLEVEL_ADV_FRICTION_COMPENSATION,       Ent_Adv_Friction_Compensation,   			Run_Adv_Friction_Compensation,   			NULL);
    TASK_CREATE_ROUTINE(&low_level_ctrl_task,  ROUTINE_ID_LOWLEVEL_ADV_FRICTION_ID_VEL_CTRL_EVAL,   Ent_Evaluation_Friction_ID_Velocity_Ctrl,   Run_Evaluation_Friction_ID_Velocity_Ctrl,   NULL);
    TASK_CREATE_ROUTINE(&low_level_ctrl_task,  ROUTINE_ID_LOWLEVEL_ADV_FRICTION_COMPENSATION_FF,    Ent_Adv_Friction_Compensation_FF,  			Run_Adv_Friction_Compensation_FF,  			NULL);
	TASK_CREATE_ROUTINE(&low_level_ctrl_task,  ROUTINE_ID_LOWLEVEL_CURRENT_SINE_REF,  	            Ent_LL_Generate_Current_Sine, 	            Run_LL_Generate_Current_Sine, 	            NULL);
	TASK_CREATE_ROUTINE(&low_level_ctrl_task,  ROUTINE_ID_LOWLEVEL_CURRENT_REC_REF,	                Ent_LL_Generate_Current_Rec,                Run_LL_Generate_Current_Rec,                NULL);
	TASK_CREATE_ROUTINE(&low_level_ctrl_task,  ROUTINE_ID_LOWLEVEL_CURRENT_TANH_REF,	            Ent_LL_Generate_Current_Tanh,               Run_LL_Generate_Current_Tanh,               NULL);
	TASK_CREATE_ROUTINE(&low_level_ctrl_task,  ROUTINE_ID_LOWLEVEL_CURRENT_CTRL_TRACKING_CHECK,	    Ent_Check_Current_Controller_Tracking,      Run_Check_Current_Controller_Tracking,      NULL);
	TASK_CREATE_ROUTINE(&low_level_ctrl_task,  ROUTINE_ID_LOWLEVEL_ADV_FRICTION_COMPENSATION_SS,    Ent_Adv_Friction_Compensation_SS,           Run_Adv_Friction_Compensation_SS,			NULL);

	/* DOD Definition*/
	// DOD
	Create_DOD(TASK_ID_LOWLEVEL);

	// PDO
	Create_PDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_3PHASES_CURRENT_RAW,			e_Int32,  	3, &motor_out.I_U);
	Create_PDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_3PHASES_CURRENT_KF,			e_Float32, 	3, &motor_out.I_U_KF);
	Create_PDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_3PHASES_VOLTAGE_RAW,			e_Int32,  	3, &motor_out.V_U);
	Create_PDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_3PHASES_VOLTAGE_KF,			e_Float32,  3, &motor_out.V_U);
	Create_PDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_POSITION, 						e_Float32, 	1, &motor_out.position);
	Create_PDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_VELOCITY, 						e_Float32, 	1, &motor_out.velocity);
	Create_PDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_CLARKE_OUT, 					e_Int32, 	2, &clarke_out.Ia);
	Create_PDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_PARK_OUT, 						e_Int32, 	2, &park_out.Id);
	Create_PDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_VOLTAGE_IN, 					e_Float32, 	3, &motor_in.V_U_input);
	Create_PDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_ELEC_ANGLE,					e_UInt16,  	1, &park_in.theta_e);//BLDC_Motor.elec_angle);
	Create_PDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_PRBS_DATA,						e_Float32, 	2, &elec_system_id.f_signal);

	Create_PDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_TOTAL_CURRENT_INPUT, 			e_Float32, 	1, &motor_in.total_current_input);
	Create_PDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_CURRENT_OUTPUT,				e_Float32, 	1, &motor_out.current_act);

	Create_PDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_AUXILIARY_INPUT, 				e_Float32, 	1, &motor_in.auxiliary_input);
	Create_PDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_F_VECTOR_INPUT, 				e_Float32, 	1, &motor_in.f_vector_input);
	Create_PDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_LOW_ID_PROCESS_INPUT, 			e_Float32, 	1, &motor_in.low_id_process_input);
	Create_PDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_FRICTION_COMPENSATOR_INPUT, 	e_Float32, 	1, &motor_in.friction_input);
	Create_PDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_MID_ID_PROCESS_INPUT, 			e_Float32, 	1, &motor_in.mid_id_process_input);
	Create_PDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_IRC_INPUT,					 	e_Float32, 	1, &motor_in.irc_input);
	Create_PDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_MID_CTRL_INPUT,			 	e_Float32, 	1, &motor_in.mid_ctrl_input);
	Create_PDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_ANALYZER_INPUT, 				e_Float32, 	1, &motor_in.analysis_input);

	Create_PDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_COMMUTATION_STEP, 				e_UInt8, 	1, &motor_setting.commutation_set.comm_step);
	Create_PDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_FRICTION_ID_REF, 				e_Float32, 	1, &advanced_friction_id.friction_ref);

	Create_PDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_HALL_SENSOR_SIG, 			    e_UInt8, 	3, &hallObj.H1);
	Create_PDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_HALL_SENSOR_LOGIC, 			e_UInt8, 	1, &hallObj.hall_logic);

	Create_PDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_STATOR_TEMP,					e_Float32,  1, &temp_sense.stator_filtered);
	Create_PDO(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_ADV_ELEC_ANGLE,                e_Int16,    1, &motor_out.avelec_angle_U);

	// SDO
	MSG_COMMON_SDO_CREATE(TASK_ID_LOWLEVEL)

	Create_SDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_NAME, 							e_String10, Set_Name);
	Create_SDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_POLE_PAIR, 					e_UInt8, 	Set_Pole_Pair);
	Create_SDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_ENCODER_RESOLUTION, 			e_UInt32, 	Set_Enc_Resolution);
	Create_SDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_GEAR_RATIO, 					e_Float32, 	Set_Gear_Ratio);
	Create_SDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_TORQUE_CONSTANT,				e_Float32, 	Set_Torque_Constant);
	Create_SDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_VELOCITY_CONSTANT,				e_Float32, 	Set_Velocity_Constant);
	Create_SDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_PEAK_CURRENT_LIMIT, 			e_Float32, 	Set_Peak_Limit);
	Create_SDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_CONTINUOUS_CURRENT_LIMIT, 		e_Float32, 	Set_Continu_Limit);
	Create_SDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_MAX_VELOCITY,					e_Float32, 	Set_Max_Velocity);

	Create_SDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_COMMUTATION_DUTY, 				e_UInt16, 	Set_Commutation_Duty);
	Create_SDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_USER_DIRECTION, 				e_Int8, 	Set_User_Direction);

	Create_SDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_ELEC_SYSTEM_ID_MAG,			e_Float32, 	Set_Elec_Sys_ID_Mag);
	Create_SDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_TERMINAL_RESISTANCE, 			e_Float32, 	Set_Resistance);
	Create_SDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_TERMINAL_INDUCTANCE, 			e_Float32, 	Set_Inductance);
	Create_SDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_BEMF_ID_VELOCITY, 				e_Float32, 	Set_BEMF_ID_velocity);
	Create_SDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_BEMF_ID_GAIN_PCTG, 			e_UInt8, 	Set_BEMF_ID_Gain_PCTG);
	Create_SDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_CURRENT_CTRL_BW_RAD, 			e_Float32, 	Set_Curr_Ctrl_BW);

	Create_SDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_INERTIA, 						e_Float32, 	Set_Inertia);
	Create_SDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_DAMPING_COEF, 					e_Float32, 	Set_Damping_Coef);
	Create_SDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_MECH_MODEL_A, 					e_Float32, 	Set_Mech_Model_a);
	Create_SDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_MECH_MODEL_B, 					e_Float32, 	Set_Mech_Model_b);

	Create_SDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_FRICTION_ID_INFO,				e_Float32, 	Set_Friction_ID_Info);
	Create_SDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_FRICTION_LUT_INFO,				e_Float32, 	Set_Friction_LUT_Info);

	Create_SDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_SET_COMMUTATION_SENSOR,		e_UInt8, 	Set_Commutation_Sensor);
	Create_SDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_SET_POS_FEEDBACK_SENSOR,		e_UInt8, 	Set_Position_Feedback_Sensor);
	Create_SDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_SET_E_ANGLE_HOMING_SENSOR,		e_UInt8, 	Set_Elec_Angle_Homing_Sensor);
	Create_SDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_SET_M_ANGLE_HOMING_SENSOR,		e_UInt8, 	Set_Mech_Angle_Homing_Sensor);
	Create_SDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_SET_SENSOR_USAGE,		        e_UInt8, 	Set_Sensor_Usage);
	Create_SDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_CURRENT_PERIODIC_SIG_INFO,	    e_Float32, 	Set_Current_Periodic_Sig_Info);
	Create_SDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_MD_VERSION,	                e_UInt8, 	Set_MD_Version);
	Create_SDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_INC_ENCODER_PRESCALER,	        e_UInt16, 	Set_IncEncoder_Prescaler);

	Create_SDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_BEMF_COMP_GAIN, 		     	e_Float32, 	Set_BEMF_Comp_Gain);
	Create_SDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_FRICTION_COMP_GAIN_SS,         e_Float32,  Set_Friction_Comp_Gain_SS);

	Create_SDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_CURRENT_KF_ONOFF,              e_UInt8,    Set_Current_KF_ONOFF);
	Create_SDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_CURRENT_KF_A,                  e_Float32,  Set_Current_KF_A);
	Create_SDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_CURRENT_KF_B,                  e_Float32,  Set_Current_KF_B);
	Create_SDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_CURRENT_KF_C,                  e_Float32,  Set_BEMF_Constant);
	Create_SDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_CURRENT_KF_Q,                  e_Float32,  Set_Current_KF_Q);
	Create_SDO(TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_CURRENT_KF_R,                  e_Float32,  Set_Current_KF_R);


	/* Timer 1 Callback Allocation */
    if(IOIF_StartTimIT(IOIF_TIM1) > 0){					//25kHz
        //TODO: ERROR PROCESS
    }
    IOIF_StartTimOCIT(IOIF_TIM1, IOIF_TIM_CHANNEL_4);

	IOIF_SetTimCB(IOIF_TIM1, IOIF_TIM_OC_DELAY_ELAPSED_CALLBACK, RunLowLvCtrl, NULL);

	// Hall Sensor Interrupt callback
	IOIF_SetGPIOCB(IOIF_GPIO_PIN_6, IOIF_GPIO_EXTI_CALLBACK, RunHallITCallback);
	IOIF_SetGPIOCB(IOIF_GPIO_PIN_7, IOIF_GPIO_EXTI_CALLBACK, RunHallITCallback);
	IOIF_SetGPIOCB(IOIF_GPIO_PIN_8, IOIF_GPIO_EXTI_CALLBACK, RunHallITCallback);

	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_D ,IOIF_GPIO_PIN_13, IOIF_GPIO_PIN_SET);
}

void RunLowLvCtrl(void* params)
{
	 // Loop Start Time Check
	 CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	 DWT->CYCCNT = 0;
	 DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_B, IOIF_GPIO_PIN_9, IOIF_GPIO_PIN_SET); // TP6_Pin

	// Run Device
	Run_Task(&low_level_ctrl_task);

	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_B, IOIF_GPIO_PIN_9, IOIF_GPIO_PIN_RESET); // TP6_Pin

	// Elapsed Time Check
	lowTimeElap = DWT->CYCCNT/480;	// in microsecond

	if(lowTimeElap > 40){
		RT_Broken++;
		IOIF_WriteGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_13, IOIF_GPIO_PIN_SET); // LED_BOOT_RED_Pin
	}
}

void RunHallITCallback()
{
	if (motor_setting.sensor_setting.e_angle_homing_sensor == e_EHoming_Sensor_Hall)	Run_Elec_Angle_Homing();

	//uint16_t Virtual_Elec_Angle = Cal_Virtual_Elec_Angle();


	if (motor_setting.sensor_setting.commutation_sensor == e_Commute_Sensor_Inc_Encoder_Hall)
	{

		if (motor_setting.sensor_setting.hall_sensor_usage == 1) Read_Hall_Sensors(&hallObj);

		Cal_Elec_Angle();
		uint16_t Elec_Angle_Difference = Check_Elec_Angle_Offset(motor_out.elec_angle);

		int16_t Diff;
		if (Elec_Angle_Difference > 32768) Diff = Elec_Angle_Difference - 65536;
		else                               Diff = Elec_Angle_Difference;

		if ((Diff > 2000) || (Diff < -2000))
		{
			Elec_Angle_Calib();
		}
	}

	hallObj.hall_logic_f = hallObj.hall_logic;

}

/* ------------------- MAIN SEQUENCE FUNCTION ------------------- */
void Cal_Elec_System_ID_Batch()
{
	double t_i = 0.0, t_i_prev = 0.0;
	double t_v = 0.0, t_v_prev = 0.0;
	double phi_11 = 0.0, phi_12 = 0.0, phi_22 = 0.0;
	double phi_sum_11 = 0.0, phi_sum_12 = 0.0, phi_sum_22=0;
	double out_sum_1 = 0.0, out_sum_2 = 0.0;
	double det_phi_sum = 0.0, inv_phi_sum_11 = 0.0, inv_phi_sum_12 = 0.0, inv_phi_sum_22 = 0.0;


	for(int i = 0; i < CHIRP_ARRAY_SIZE; ++i) {
		t_v = (double)(shared_array_buffer.buf_1st[i]); // voltage, unit: duty(0~4799)
		t_i = (double)(shared_array_buffer.buf_2nd[i]); // current, unit: -32767 ~ 32768

		phi_11 = (t_i_prev)*(t_i_prev);
		phi_12 = (t_i_prev)*(t_v_prev);
		phi_22 = (t_v_prev)*(t_v_prev);

		phi_sum_11 += phi_11;
		phi_sum_12 += phi_12;
		phi_sum_22 += phi_22;

		out_sum_1 += (t_i_prev)*(t_i);
		out_sum_2 += (t_v_prev)*(t_i);

		t_i_prev = t_i;
		t_v_prev = t_v;
	}

	det_phi_sum = (phi_sum_11*phi_sum_22) - (phi_sum_12*phi_sum_12);
	inv_phi_sum_11 = phi_sum_22/det_phi_sum;
	inv_phi_sum_12 = -phi_sum_12/det_phi_sum;
	inv_phi_sum_22 = phi_sum_11/det_phi_sum;

	batch_result_1 = (inv_phi_sum_11*out_sum_1) + (inv_phi_sum_12*out_sum_2);
	batch_result_2 = (inv_phi_sum_12*out_sum_1) + (inv_phi_sum_22*out_sum_2);

	kf_current_object.kf_A = batch_result_1;		// a
	kf_current_object.kf_B = batch_result_2;		// b

	// i(k) = A*i(k-1) + B*u(k-1)

	/* Send Data for Synchronization With UI */
	uint16_t t_identifier;
	float t_temp_arr[2] = {0};

	memcpy(&t_temp_arr[0], &kf_current_object.kf_A, 4);
	memcpy(&t_temp_arr[1], &kf_current_object.kf_B, 4);

	for(int i = 0; i<25000; ++i){}

	t_identifier = GUI_SYNC|E_SYS_BATCH;
    Send_MSG(t_identifier, 8, (uint8_t*)t_temp_arr);
	return;
}

void Send_Elec_BEMF_Value()
{
	static uint16_t t_identifier = 0;
	static float t_temp_arr[2] = {0};

	/* Send Data for Synchronization With UI */
	memcpy(&t_temp_arr[0], &kf_current_object.kf_C, 4);

	t_identifier = GUI_SYNC|E_SYS_BEMF;
	Send_MSG(t_identifier, 4, (uint8_t*)t_temp_arr);

	MS_enum = IDLE;
}

void Send_Elec_Bandwidth_Data()
{
	uint16_t t_identifier = 0;
	float t_buf[15] = {0};
	int cnt = BW_CHECK_ARRAY_SIZE/7;

	for(int i = 0; i < cnt; ++i){
		t_buf[0] = (float)i;

		memcpy(&t_buf[1], &shared_array_buffer.buf_1st[i*7], 4);
		memcpy(&t_buf[2], &shared_array_buffer.buf_2nd[i*7], 4);

		memcpy(&t_buf[3], &shared_array_buffer.buf_1st[i*7+1], 4);
		memcpy(&t_buf[4], &shared_array_buffer.buf_2nd[i*7+1], 4);

		memcpy(&t_buf[5], &shared_array_buffer.buf_1st[i*7+2], 4);
		memcpy(&t_buf[6], &shared_array_buffer.buf_2nd[i*7+2], 4);

		memcpy(&t_buf[7], &shared_array_buffer.buf_1st[i*7+3], 4);
		memcpy(&t_buf[8], &shared_array_buffer.buf_2nd[i*7+3], 4);

		memcpy(&t_buf[9], &shared_array_buffer.buf_1st[i*7+4], 4);
		memcpy(&t_buf[10], &shared_array_buffer.buf_2nd[i*7+4], 4);

		memcpy(&t_buf[11], &shared_array_buffer.buf_1st[i*7+5], 4);
		memcpy(&t_buf[12], &shared_array_buffer.buf_2nd[i*7+5], 4);

		memcpy(&t_buf[13], &shared_array_buffer.buf_1st[i*7+6], 4);
		memcpy(&t_buf[14], &shared_array_buffer.buf_2nd[i*7+6], 4);

		for(int j = 0; j<1000000; ++j){}

		t_identifier = GUI_SYNC|BW_CHECK;
		Send_MSG(t_identifier, 60, (uint8_t*)t_buf);
	}


/*	uint16_t t_identifier = 0;
	float t_buf[3] = {0};

	for (int i = 0; i < BW_CHECK_ARRAY_SIZE; i++)
	{
		t_buf[0] = (float)i;

		memcpy(&t_buf[1], &shared_array_buffer.buf_1st[i], 4);
		memcpy(&t_buf[2], &shared_array_buffer.buf_2nd[i], 4);

		for(int j = 0; j<200000; ++j){}

		t_identifier = GUI_SYNC|BW_CHECK;
		Send_MSG(t_identifier, 12, (uint8_t*)t_buf);
	}*/

	MS_enum = IDLE;
}

float send_buf[3] = {0};

void Send_Elec_Tracking_Data()
{
	uint16_t t_identifier = 0;

	float t_buf[3] = {0};
	for (int i = 0; i < CURRENT_CTRL_EVAL_ARRAY_SIZE; i++)
	{
		t_buf[0] = (float)i;
		memcpy(&t_buf[1], &shared_array_buffer.buf_1st[i], 4);
		memcpy(&t_buf[2], &shared_array_buffer.buf_2nd[i], 4);

		for(int j = 0; j<100000; ++j){}

		t_identifier = GUI_SYNC|GET_CURRENT_TRACKING_CHECK;
		Send_MSG(t_identifier, 12, (uint8_t*)t_buf);
	}
	MS_enum = IDLE;
}

void Send_Advanced_Friction_ID_Data()
{
	uint16_t t_identifier = 0;
	float t_buf[3] = {0};

	for(int i = 0; i < 4*(advanced_friction_id.vel_num1+advanced_friction_id.vel_num2); ++i){
		t_buf[0] = (float)i;

		memcpy(&t_buf[1], &shared_array_buffer.buf_1st[i], 4);
		memcpy(&t_buf[2], &shared_array_buffer.buf_2nd[i], 4);

		for(int j = 0; j<500000; ++j){}

		t_identifier = GUI_SYNC|ADV_FRICTION_ID_DATA;
	    Send_MSG(t_identifier, 12, (uint8_t*)t_buf);
	}

	for(int i = 0; i<500000; ++i){}

	Send_MSG((uint16_t)(GUI_SYNC|ADV_FRICTION_ID_DONE), 1, (uint8_t*)0);

	MS_enum = IDLE;
}

void Cal_Friction_LUT()
{
	double t_temp_act     = 0.0;
	double t_temp_desired = 0.0;
	double t_max_vel = 0.0, t_vel = 0.0;

	t_max_vel = ((48 - motor_setting.peakCurr_limit*motor_properties.R)/(motor_properties.Ke)/(motor_properties.gear_ratio))*0.9;
	advanced_friction_id.scaling_factor = (2*t_max_vel)/(FRICTION_LUT_SIZE-1);

    for(int i = 0; i < FRICTION_LUT_SIZE; ++i){
    	t_vel = (i - ((FRICTION_LUT_SIZE-1)/2))*advanced_friction_id.scaling_factor;

    	if      (advanced_friction_id.lut_mdl == 1) // Coulomb Viscous Model
    	{
    		double Fc = advanced_friction_id.lut_p1;
    		double Fv = advanced_friction_id.lut_p2;

    		t_temp_act = Fc*tanh((M_PI/2)*(t_vel)/advanced_friction_id.lut_epsilon) + Fv*t_vel;
    	}
    	else if (advanced_friction_id.lut_mdl == 2) // Stribeck 1
    	{
    		double Fs = advanced_friction_id.lut_p1;
    		double Fc = advanced_friction_id.lut_p2;
    		double vs = advanced_friction_id.lut_p3;
    	    double b  = advanced_friction_id.lut_p4;

    		t_temp_act = (Fs-(Fs-Fc)*exp(-(t_vel/vs)*(t_vel/vs)))*tanh((M_PI/2)*(t_vel)/advanced_friction_id.lut_epsilon) + b*t_vel;
    	}


    	else if (advanced_friction_id.lut_mdl == 3) // Stribeck 2
    	{
    		double Fs = advanced_friction_id.lut_p1;
    		double Fc = advanced_friction_id.lut_p2;
    		double vs = advanced_friction_id.lut_p3;
    		double b  = advanced_friction_id.lut_p4;
    		double c  = advanced_friction_id.lut_p5;

    		double t_sign = 0;
    	    if (t_vel >= 0) t_sign = +1;
    	    else            t_sign = -1;
    		t_temp_act = (Fs-(Fs-Fc)*exp(-(t_vel/vs)*(t_vel/vs)))*tanh((M_PI/2)*(t_vel)/advanced_friction_id.lut_epsilon) + b*t_vel + c*t_vel*t_vel*t_sign;
    	}

    	t_temp_desired = advanced_friction_id.lut_d_mu_v * t_vel;
        advanced_friction_id.adv_friction_compensator_LUT[i] = advanced_friction_id.gain * (t_temp_act - t_temp_desired);
	}

	MS_enum = IDLE;
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
	static uint32_t calib_cnt = 0;
	static float resistance_initial = 0;

	if(calib_cnt < 1000) {

			GateDriver_ONOFF(DISABLE);

			motor_in.total_current_input = 0.0;

			motor_out.I_U_offset += rawCurr[0];
			motor_out.I_V_offset += rawCurr[1];
			motor_out.I_W_offset += rawCurr[2];

			motor_out.V_U_offset += rawCurr[3];
			motor_out.V_V_offset += rawCurr[4];
			motor_out.V_W_offset += rawCurr[5];

//			temp_sense.voltage_initial += tempbuffer[0] * 0.00005035477226;

		} else if(calib_cnt == 1000){

			GateDriver_ONOFF(DISABLE);
			motor_in.total_current_input = 0.0;

			motor_out.I_U_offset = motor_out.I_U_offset / 1000;
			motor_out.I_V_offset = motor_out.I_V_offset / 1000;
			motor_out.I_W_offset = motor_out.I_W_offset / 1000;

			motor_out.V_U_offset = motor_out.V_U_offset / 1000;
			motor_out.V_V_offset = motor_out.V_V_offset / 1000;
			motor_out.V_W_offset = motor_out.V_W_offset / 1000;

			motor_out.I_U_offset = motor_out.I_U_offset - (ADC1_RESOLUTION>>1);
			motor_out.I_V_offset = motor_out.I_V_offset - (ADC1_RESOLUTION>>1);
			motor_out.I_W_offset = motor_out.I_W_offset - (ADC1_RESOLUTION>>1);

			temp_sense.voltage_initial = temp_sense.voltage_initial / 1000;
			resistance_initial = 3.3 * 51000 / temp_sense.voltage_initial - 51000;
			temp_sense.stator_initial = 1 / (0.00335401643468 + 0.00023320895522388*log(resistance_initial*0.00002)) - 273.15;

			Transition_State(&low_level_ctrl_task.state_machine, e_State_Standby);
		}

	calib_cnt++;
}

static void StateStandby_Ent()
{
	GateDriver_ONOFF(DISABLE);
	Init_Current_Ctrl();
}

static void StateStandby_Run()
{
	Get_3Phase_Current();
	Get_3Phase_Voltage();
	IOIF_ReadIncCnt(IOIF_TIM5, &inc25KhzObj);
	Get_Position_Velocity(&inc25KhzObj, &motor_out);
	Cal_Elec_Angle();

//	Transition_State(&low_level_ctrl_task.state_machine, e_State_Enable);
}

static void StateStandby_Ext()
{
	GateDriver_ONOFF(DISABLE);
}

static void StateEnable_Ent()
{
	Ent_Elec_Angle_Homing();
	Ent_Routines(&low_level_ctrl_task.routine);
	low_level_ctrl_loop_cnt = 0;
	GateDriver_ONOFF(ENABLE);
	Stop_PWM();
	Ent_Get_Temperature();
}

static void StateEnable_Run()
{

	if (motor_setting.sensor_setting.e_angle_homing_sensor != e_EHoming_Sensor_Hall)	Run_Elec_Angle_Homing();

	// Safety Condition for Enable Run
	if ((motor_setting.commutation_set.start == 1) |  \
	    (motor_setting.sensor_setting.e_angle_homing_sensor == e_EHoming_Sensor_Hall) | \
		((motor_setting.commutation_set.done == 1) & (motor_setting.elec_angle_homing.done == 1)))

	{
		Get_3Phase_Current();
		Get_3Phase_Voltage();
		IOIF_ReadIncCnt(IOIF_TIM5, &inc25KhzObj);
		Get_Position_Velocity(&inc25KhzObj, &motor_out);
//		Run_Get_Temperature();


		Run_Routines(&low_level_ctrl_task.routine);

		low_level_ctrl_loop_cnt++;
	}
}

static void StateEnable_Ext()
{
	Ext_Routines(&low_level_ctrl_task.routine);
	GateDriver_ONOFF(DISABLE);
	Stop_PWM();
}

static void StateError_Run()
{}

/* ------------------- INITIALIZATION ------------------- */
static void Init_Kalman_Filter(KFObject * t_KF_obj, float t_A_KF, float t_B_KF, float t_C_KF, float t_Q_KF, float t_R_KF, float t_P_KF)
{
   //memset(t_KF_obj, 0, sizeof(KFObject));

   t_KF_obj->kf_A = t_A_KF;
   t_KF_obj->kf_B = t_B_KF;
   t_KF_obj->kf_C = t_C_KF;
   t_KF_obj->kf_Q = t_Q_KF;
   t_KF_obj->kf_R = t_R_KF;
   t_KF_obj->kf_P = t_P_KF;
}

static void Init_SensorUsage()
{
	if       (motor_setting.sensor_setting.commutation_sensor == e_Commute_Sensor_Inc_Encoder) {
		motor_setting.sensor_setting.incremetnal_encoder_usage = 1;
	}else if (motor_setting.sensor_setting.commutation_sensor == e_Commute_Sensor_Hall) {
		motor_setting.sensor_setting.hall_sensor_usage = 1;
	}else if (motor_setting.sensor_setting.commutation_sensor == e_Commute_Sensor_Inc_Encoder_Hall) {
		motor_setting.sensor_setting.incremetnal_encoder_usage = 1;
		motor_setting.sensor_setting.hall_sensor_usage = 1;
	}

	if       (motor_setting.sensor_setting.pos_feedback_sensor == e_Pos_Sensor_Inc_Encoder) {
		motor_setting.sensor_setting.incremetnal_encoder_usage = 1;
	}else if (motor_setting.sensor_setting.pos_feedback_sensor == e_Pos_Sensor_Abs_Encoder1) {
		motor_setting.sensor_setting.absolute_encoder1_usage = 1;
	}else if (motor_setting.sensor_setting.pos_feedback_sensor == e_Pos_Sensor_Abs_Encoder2) {
		motor_setting.sensor_setting.absolute_encoder2_usage = 1;
	}

	if       (motor_setting.sensor_setting.e_angle_homing_sensor == e_EHoming_Sensor_Hall) {
		motor_setting.sensor_setting.hall_sensor_usage = 1;
	}else if (motor_setting.sensor_setting.e_angle_homing_sensor == e_EHoming_Sensor_Abs_Encoder1) {
		motor_setting.sensor_setting.absolute_encoder1_usage = 1;
	}else if (motor_setting.sensor_setting.e_angle_homing_sensor == e_EHoming_Sensor_Abs_Encoder2) {
		motor_setting.sensor_setting.absolute_encoder2_usage = 1;
	}

	if       (motor_setting.sensor_setting.m_angle_homing_sensor == e_MHoming_Sensor_Abs_Encoder1) {
		motor_setting.sensor_setting.absolute_encoder1_usage = 1;
	}else if (motor_setting.sensor_setting.m_angle_homing_sensor == e_MHoming_Sensor_Abs_Encoder2) {
		motor_setting.sensor_setting.absolute_encoder2_usage = 1;
	}else if (motor_setting.sensor_setting.m_angle_homing_sensor == e_MHoming_Sensor_MD_Calculation) {
		motor_setting.sensor_setting.incremetnal_encoder_usage = 1;
	}

	if (AbsObj1.location != IOIF_ABS_ENC_NONE)
		motor_setting.sensor_setting.absolute_encoder1_usage = 1;

	if (AbsObj2.location != IOIF_ABS_ENC_NONE)
		motor_setting.sensor_setting.absolute_encoder2_usage = 1;

	if ((motor_setting.sensor_setting.incremetnal_encoder_usage != 1) &&
		(motor_setting.sensor_setting.incremetnal_encoder_usage != 0))
	{
		motor_setting.sensor_setting.incremetnal_encoder_usage = 0;
	}

	if ((motor_setting.sensor_setting.absolute_encoder1_usage != 1) &&
		(motor_setting.sensor_setting.absolute_encoder1_usage != 0))
	{
		motor_setting.sensor_setting.absolute_encoder1_usage = 0;
	}

	if ((motor_setting.sensor_setting.absolute_encoder2_usage != 1) &&
		(motor_setting.sensor_setting.absolute_encoder2_usage != 0))
	{
		motor_setting.sensor_setting.absolute_encoder2_usage = 0;
	}

	if ((motor_setting.sensor_setting.hall_sensor_usage != 1) &&
		(motor_setting.sensor_setting.hall_sensor_usage != 0))
	{
		motor_setting.sensor_setting.hall_sensor_usage = 0;
	}

	if ((motor_setting.sensor_setting.temperature_sensor_usage != 1) &&
		(motor_setting.sensor_setting.temperature_sensor_usage != 0))
	{
		motor_setting.sensor_setting.temperature_sensor_usage = 0;
	}

	if ((motor_setting.sensor_setting.imu_6axis_usage != 1) &&
		(motor_setting.sensor_setting.imu_6axis_usage != 0))
	{
		motor_setting.sensor_setting.imu_6axis_usage = 0;
	}

	if ((motor_setting.sensor_setting.imu_3axis_usage != 1) &&
		(motor_setting.sensor_setting.imu_3axis_usage != 0))
	{
		motor_setting.sensor_setting.imu_3axis_usage = 0;
	}

	uint8_t md_ver = motor_properties.md_version;
	if (md_ver == e_None)
	{
		motor_setting.sensor_setting.curr_sensor_range = 29;
	}
	else if ((md_ver == e_R06) | (md_ver == e_R07) | (md_ver == e_R08))
	{
		motor_setting.sensor_setting.curr_sensor_range = 29;
	}
	else if (md_ver == e_R08H)
	{
		motor_setting.sensor_setting.curr_sensor_range = 240;
	}
	else
	{
		motor_setting.sensor_setting.curr_sensor_range = 29;
	}

	motor_setting.sensor_setting.curr_sensor_a2d = ADC1_RESOLUTION / motor_setting.sensor_setting.curr_sensor_range;
	motor_setting.sensor_setting.curr_sensor_d2a = 1/motor_setting.sensor_setting.curr_sensor_a2d;


	if (isnanf(inc25KhzObj.prescaler) | (inc25KhzObj.prescaler < 1) | (inc25KhzObj.prescaler > 8192))
	{
		inc25KhzObj.prescaler = 1;
		inc1KhzObj.prescaler  = 1;
	}
	inc1KhzObj.prescaler = inc25KhzObj.prescaler;

}

static void Init_Commutation()
{
	if(motor_setting.commutation_set.done != 1){

		motor_setting.commutation_set.done   = 0;
		motor_setting.commutation_set.ea_dir = +1;
		motor_setting.commutation_set.ma_dir = +1;
		motor_setting.commutation_set.cc_dir = +1;
	}
	motor_setting.commutation_set.max_duty           = 300;
	motor_setting.commutation_set.state              = 0;
	motor_setting.commutation_set.time_stamp         = 0;
	motor_setting.commutation_set.comm_step          = 0;
	motor_setting.commutation_set.user_desired_dir   = 0;

	if((motor_setting.sensor_setting.commutation_sensor != e_Commute_Sensor_Inc_Encoder) &&
	   (motor_setting.sensor_setting.commutation_sensor != e_Commute_Sensor_Hall) &&
	   (motor_setting.sensor_setting.commutation_sensor != e_Commute_Sensor_Inc_Encoder_Hall))
	{
		motor_setting.sensor_setting.commutation_sensor = e_Commute_Sensor_Inc_Encoder;
	}

	if((motor_setting.sensor_setting.pos_feedback_sensor != e_Pos_Sensor_None) &&
	   (motor_setting.sensor_setting.pos_feedback_sensor != e_Pos_Sensor_Inc_Encoder) &&
	   (motor_setting.sensor_setting.pos_feedback_sensor != e_Pos_Sensor_Abs_Encoder1) &&
	   (motor_setting.sensor_setting.pos_feedback_sensor != e_Pos_Sensor_Abs_Encoder2))
	{
		motor_setting.sensor_setting.pos_feedback_sensor = e_Pos_Sensor_None;
	}

	if((motor_setting.sensor_setting.e_angle_homing_sensor != e_EHoming_Sensor_Forced) &&
	   (motor_setting.sensor_setting.e_angle_homing_sensor != e_EHoming_Sensor_Hall) &&
	   (motor_setting.sensor_setting.e_angle_homing_sensor != e_EHoming_Sensor_Abs_Encoder1) &&
	   (motor_setting.sensor_setting.e_angle_homing_sensor != e_EHoming_Sensor_Abs_Encoder2))
	{
		motor_setting.sensor_setting.e_angle_homing_sensor = e_EHoming_Sensor_Forced;
	}

	if((motor_setting.sensor_setting.m_angle_homing_sensor != e_MHoming_Sensor_Zero) &&
	   (motor_setting.sensor_setting.m_angle_homing_sensor != e_MHoming_Sensor_Abs_Encoder1) &&
	   (motor_setting.sensor_setting.m_angle_homing_sensor != e_MHoming_Sensor_Abs_Encoder2) &&
	   (motor_setting.sensor_setting.m_angle_homing_sensor != e_MHoming_Sensor_MD_Calculation) &&
	   (motor_setting.sensor_setting.m_angle_homing_sensor != e_MHoming_Sensor_CM_Setting))
	{
		motor_setting.sensor_setting.m_angle_homing_sensor = e_MHoming_Sensor_Zero;
	}

	motor_setting.elec_angle_homing.offset = 0;
	motor_setting.elec_angle_homing.forced_homing_cnt = 0;

	if (motor_setting.sensor_setting.hall_sensor_usage == 1) Read_Hall_Sensors(&hallObj);
}

static void Init_Current_Ctrl()
{
	pid_d_axis_ctrl.err = 0;
	pid_d_axis_ctrl.err_sum = 0;
	pid_q_axis_ctrl.err = 0;
	pid_q_axis_ctrl.err_sum = 0;
	pid_trape_ctrl.err = 0;
	pid_trape_ctrl.err_sum = 0;
}

static void Init_Position_Velocity()
{
	motor_out.position = 0;
	motor_out.position_f = 0;
	motor_out.velocity = 0;
	motor_out.velocity_f = 0;
}

static void Check_OverCurrent()
{
	static uint32_t overcurrent_cnt = 0;

	if((motor_out.current_act > motor_setting.contCurr_limit) || (motor_out.current_act < -motor_setting.contCurr_limit)) {
		overcurrent_cnt++;
	} else {
		overcurrent_cnt = 0;
	}
	// TODO: Error Process
	if (overcurrent_cnt > 75000) { // 3 second
		overcurrent_cnt = 0;
		Transition_State(&low_level_ctrl_task.state_machine, e_State_Standby);
		low_level_ctrl_task.err_code = ERROR_PHASE_OVER_CURRENT;
	}
}

/* ------------------- FOC ------------------- */
static void Get_3Phase_Current()
{
	motor_out.I_U = rawCurr[0] - 32768 - motor_out.I_U_offset;
	motor_out.I_V = rawCurr[1] - 32768 - motor_out.I_V_offset;
	motor_out.I_W = rawCurr[2] - 32768 - motor_out.I_W_offset;
}

static void Get_3Phase_Voltage()
{
	motor_out.V_U = rawCurr[3] - motor_out.V_U_offset;
	motor_out.V_V = rawCurr[4] - motor_out.V_V_offset;
	motor_out.V_W = rawCurr[5] - motor_out.V_W_offset;
}


static void Get_Position_Velocity(IOIF_IncEnc_t* t_inc_enc,  MotorOut* t_motor_out)
{
	t_motor_out->position = motor_setting.commutation_set.ma_dir * (t_inc_enc->userCnt / (t_inc_enc->resolution*motor_properties.gear_ratio) * 2* M_PI); 		// in rad

	/*Low Pass Filter*/
	float temp = (t_motor_out->position - t_motor_out->position_f)*LOW_LEVEL_CONTROL_FREQUENCY;

	if(isnan(t_motor_out->velocity)){
		t_motor_out->velocity = temp;
	}else{
		//t_motor_out->velocity = 0.817862*t_motor_out->velocity + 0.182138*temp;		// 800Hz low-pass filter

		if (motor_properties.md_version != e_R08H)
			t_motor_out->velocity = 0.963*t_motor_out->velocity + 0.037*temp;   // 300Hz low-pass filter
			//t_motor_out->velocity = 0.9962*t_motor_out->velocity + 0.0038*temp; // 30Hz low-pass filter
		else

			t_motor_out->velocity = 0.9752*t_motor_out->velocity + 0.02482*temp; // 100Hz low-pass filter

	}

	t_motor_out->position_f = t_motor_out->position;
	t_motor_out->velocity_f = t_motor_out->velocity;
}

static void Cal_Trape_Total_Current()
{
	float total_current = 0;
	float temp_current;

	if      (hallObj.hall_logic == motor_setting.commutation_set.hall_sensor_table[0]) {total_current = (motor_out.I_V - motor_out.I_W)*0.5; }
	else if (hallObj.hall_logic == motor_setting.commutation_set.hall_sensor_table[1]) {total_current = (motor_out.I_V - motor_out.I_U)*0.5; }
	else if (hallObj.hall_logic == motor_setting.commutation_set.hall_sensor_table[2]) {total_current = (motor_out.I_W - motor_out.I_U)*0.5; }
	else if (hallObj.hall_logic == motor_setting.commutation_set.hall_sensor_table[3]) {total_current = (motor_out.I_W - motor_out.I_V)*0.5; }
	else if (hallObj.hall_logic == motor_setting.commutation_set.hall_sensor_table[4]) {total_current = (motor_out.I_U - motor_out.I_V)*0.5; }
	else if (hallObj.hall_logic == motor_setting.commutation_set.hall_sensor_table[5]) {total_current = (motor_out.I_U - motor_out.I_W)*0.5; }

	temp_current = total_current * 0.0004425049; // 29/65535
	motor_out.current_act = motor_out.current_act * 0.99 + temp_current * 0.01;
}


static void Cal_Elec_Angle()
{

	uint16_t t_elec_angle;


    /*
	temp = ((uint32_t)inc25KhzObj.userCnt)  *  ((uint32_t)(65536/(double)inc25KhzObj.resolution));

	t_elec_angle         = motor_setting.commutation_set.ea_dir* ((temp * motor_properties.pole_pair) - (((temp * motor_properties.pole_pair) >> 16) << 16));
	motor_out.elec_angle = t_elec_angle + motor_setting.elec_angle_homing.offset;
	*/

	//temp = (uint32_t)(inc25KhzObj.userCnt * (65536/(double)inc25KhzObj.resolution));


	uint32_t temp_e = 0;

	float int_cal = (inc25KhzObj.userCnt * (65536/(float)inc25KhzObj.resolution));
	if (int_cal >= 0)
		temp_e = (uint32_t)int_cal;
	else
		temp_e = -(uint32_t)(-int_cal);


	t_elec_angle         = motor_setting.commutation_set.ea_dir* ((temp_e * motor_properties.pole_pair) - (((temp_e * motor_properties.pole_pair) >> 16) << 16));
	motor_out.elec_angle = t_elec_angle + motor_setting.elec_angle_homing.offset;


}


static void Cal_Elec_Angle_with_Hall()
{
	if (motor_setting.sensor_setting.hall_sensor_usage == 1) Read_Hall_Sensors(&hallObj);

	if (motor_setting.commutation_set.cc_dir * motor_in.total_current_input >= 0)
	{
		if      (hallObj.hall_logic == motor_setting.commutation_set.hall_sensor_table[0]) motor_out.elec_angle = +5461;
		else if (hallObj.hall_logic == motor_setting.commutation_set.hall_sensor_table[1]) motor_out.elec_angle = +16384;
		else if (hallObj.hall_logic == motor_setting.commutation_set.hall_sensor_table[2]) motor_out.elec_angle = +27307;
		else if (hallObj.hall_logic == motor_setting.commutation_set.hall_sensor_table[3]) motor_out.elec_angle = +38229;
		else if (hallObj.hall_logic == motor_setting.commutation_set.hall_sensor_table[4]) motor_out.elec_angle = +49152;
		else if (hallObj.hall_logic == motor_setting.commutation_set.hall_sensor_table[5]) motor_out.elec_angle = +60075;
		else { /* Error Handler */ }
	}

	else
	{
		if      (hallObj.hall_logic == motor_setting.commutation_set.hall_sensor_table[0]) motor_out.elec_angle = +60075;
		else if (hallObj.hall_logic == motor_setting.commutation_set.hall_sensor_table[1]) motor_out.elec_angle = +5461;
		else if (hallObj.hall_logic == motor_setting.commutation_set.hall_sensor_table[2]) motor_out.elec_angle = +16384;
		else if (hallObj.hall_logic == motor_setting.commutation_set.hall_sensor_table[3]) motor_out.elec_angle = +27307;
		else if (hallObj.hall_logic == motor_setting.commutation_set.hall_sensor_table[4]) motor_out.elec_angle = +38229;
		else if (hallObj.hall_logic == motor_setting.commutation_set.hall_sensor_table[5]) motor_out.elec_angle = +49152;
		else { /* Error Handler */ }
	}
}

static void Run_Kalman_Filter()
{
   static float u_A = 0.0, u_B = 0.0, u_C = 0.0, u_N = 0.0;
   uint16_t e_angle_u = 0, e_angle_v = 0, e_angle_w = 0;
   static float t_vel = 0.0;
   //static float ScalingK = 0.00036; // 0.036 * 0.01

   u_N = (motor_in.V_U_input + motor_in.V_V_input + motor_in.V_W_input)/3; // Neutral-point voltage
   u_A = motor_in.V_U_input - u_N;           // phase-U voltage (range: 0~4799) (=0~48V)
   u_B = motor_in.V_V_input - u_N;           // phase-V voltage (range: 0~4799) (=0~48V)
   u_C = motor_in.V_W_input - u_N;           // phase-W voltage (range: 0~4799) (=0~48V)

//   u_A = ScalingK*u_A; // voltage (unit: V)
//   u_B = ScalingK*u_B;
//   u_C = ScalingK*u_C;

   e_angle_u = park_in.theta_e + ((uint16_t)((svm_phasor.phase-(0xFFFF >> 2))) >> 4);
   if (e_angle_u > 4095) e_angle_u = e_angle_u - 4095;
   if (e_angle_u < 0)    e_angle_u = e_angle_u + 4095;

   e_angle_v = e_angle_u + 2730;
   e_angle_w = e_angle_u + 1365;

   if (e_angle_v > 4095)	{ e_angle_v = e_angle_v - 4095;}
   if (e_angle_v > 4095)	{ e_angle_v = e_angle_v - 4095;}
   if (e_angle_w > 4095)	{ e_angle_w = e_angle_w - 4095;}


   if(motor_out.velocity > 0) t_vel = motor_out.velocity;
   else t_vel = -motor_out.velocity;

   // (STEP2) model-based estimation
   float bemf_gain = kf_current_object.BEMF_Comp_Gain;
   motor_out.I_U_KF = (kf_current_object.kf_A * motor_out.I_U_KF) + (kf_current_object.kf_B * u_A) + (kf_current_object.kf_C * bemf_gain * t_vel * sine_signal[e_angle_u]);		// (duty_scaler) * ke * w * sin() --> duty
   motor_out.I_V_KF = (kf_current_object.kf_A * motor_out.I_V_KF) + (kf_current_object.kf_B * u_B) + (kf_current_object.kf_C * bemf_gain * t_vel * sine_signal[e_angle_v]);
   motor_out.I_W_KF = (kf_current_object.kf_A * motor_out.I_W_KF) + (kf_current_object.kf_B * u_C) + (kf_current_object.kf_C * bemf_gain * t_vel * sine_signal[e_angle_w]);

   motor_out.avelec_angle_U = e_angle_u;
   motor_out.avelec_angle_V = e_angle_v;
   motor_out.avelec_angle_W = e_angle_w;

   // (STEP3) P matrix calculation
   kf_current_object.kf_P = kf_current_object.kf_A * kf_current_object.kf_P * kf_current_object.kf_A + kf_current_object.kf_Q;

   // (STEP4) Kalman gain update
   kf_current_object.kf_K = kf_current_object.kf_P/(kf_current_object.kf_P + kf_current_object.kf_R);

   // (STEP5) Kalman filtering
   motor_out.I_U_KF = motor_out.I_U_KF + kf_current_object.kf_K * (motor_out.I_U - motor_out.I_U_KF);
   motor_out.I_V_KF = motor_out.I_V_KF + kf_current_object.kf_K * (motor_out.I_V - motor_out.I_V_KF);
   motor_out.I_W_KF = motor_out.I_W_KF + kf_current_object.kf_K * (motor_out.I_W - motor_out.I_W_KF);

   // (STEP6) P matrix update
   kf_current_object.kf_P = (1 - kf_current_object.kf_K) * kf_current_object.kf_P;

}

static void Run_FOC_Ctrl(float t_current_in)
{
	int32_t t_iu = 0, t_iv = 0, t_iw = 0;

	float A2D = motor_setting.sensor_setting.curr_sensor_a2d;

	float t_currentRef = motor_setting.commutation_set.cc_dir * t_current_in * A2D;

	if		(t_currentRef > +A2D * motor_setting.contCurr_limit) { t_currentRef = +A2D * motor_setting.contCurr_limit; }
	else if	(t_currentRef < -A2D * motor_setting.contCurr_limit) { t_currentRef = -A2D * motor_setting.contCurr_limit; }

	Run_Kalman_Filter();

	if(motor_setting.low_level_kalman_on == 1){
		t_iu =  motor_out.I_U_KF;
		t_iv =  motor_out.I_V_KF;
		t_iw =  motor_out.I_W_KF;
	}else{
		t_iu =  motor_out.I_U;
		t_iv =  motor_out.I_V;
		t_iw =  motor_out.I_W;
	}

	clarke_in.Iu = t_iu;
	clarke_in.Iv = t_iv;
	clarke_in.Iw = t_iw;
	ClarkeTransform(&clarke_in, &clarke_out);

	park_in.Ia = clarke_out.Ia;
	park_in.Ib = clarke_out.Ib;
	park_in.theta_e = (motor_out.elec_angle>>4);		// sine & cosine table range: 0~4096
	ParkTransform(&park_in, &park_out);

	Run_PID_Control(&pid_d_axis_ctrl,			   0, park_out.Id, LOW_LEVEL_CONTROL_PERIOD);
	Run_PID_Control(&pid_q_axis_ctrl,	t_currentRef, park_out.Iq, LOW_LEVEL_CONTROL_PERIOD); // ParkOut.Iq : Total Current

	inv_park_in.Vd = pid_d_axis_ctrl.control_input;
	inv_park_in.Vq = pid_q_axis_ctrl.control_input;
	inv_park_in.theta_e = park_in.theta_e;
	InvParkInputToPhasor(&inv_park_in, &svm_phasor);

	motor_out.current_act = motor_setting.commutation_set.cc_dir * \
							((float)park_out.Iq * motor_setting.sensor_setting.curr_sensor_d2a);

	Check_OverCurrent();

	motor_out.actual_elec_angle = motor_out.elec_angle + svm_phasor.phase;
	SVM(svm_phasor.magnitude, motor_out.actual_elec_angle, &motor_in.V_U_input, &motor_in.V_V_input, &motor_in.V_W_input);
}


/* ------------------- TRAPEZOIDAL ------------------- */
static void Run_Trape_Ctrl(float t_currentRef)
{
	float A2D = motor_setting.sensor_setting.curr_sensor_a2d;

	if		(t_currentRef > +motor_setting.contCurr_limit) { t_currentRef =  A2D * motor_setting.contCurr_limit; }
	else if	(t_currentRef < -motor_setting.contCurr_limit) { t_currentRef = -A2D * motor_setting.contCurr_limit; }

	Run_PID_Control(&pid_trape_ctrl, t_currentRef, motor_out.current_act, LOW_LEVEL_CONTROL_PERIOD);


	/*Float Duty to Uint Duty*/
	if (pid_trape_ctrl.control_input >= 0) {
		trape_ctrl.duty = (uint16_t)pid_trape_ctrl.control_input;
		trape_ctrl.direction = CCW;
	} else if (pid_trape_ctrl.control_input < 0) {
		trape_ctrl.duty = (uint16_t)(-pid_trape_ctrl.control_input);
		trape_ctrl.direction = CW;
	}

	/* Duty Limit */
	if (abs(trape_ctrl.duty) > MAX_DUTY) {
		trape_ctrl.duty = MAX_DUTY;
	} else {
		trape_ctrl.duty = abs(trape_ctrl.duty);
	} 

	if (trape_ctrl.duty < MIN_DUTY) {
		trape_ctrl.duty = 0;
	} else {
		trape_ctrl.duty = 1*trape_ctrl.duty;
	} 
}

/* ------------------- SDO CALLBACK ------------------- */
static void Set_Name(MsgSDOargs* req, MsgSDOargs* res)
{
	motor_properties.name_length = (uint8_t)(((uint8_t*)req->data)[0]);

	memset(&motor_properties.name, 0, sizeof(motor_properties.name));
	memcpy(&motor_properties.name, &(((char*)req->data)[1]), motor_properties.name_length);

    res->size = 0;
    res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Pole_Pair(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&motor_properties.pole_pair, req->data, 1);

    res->size = 0;
    res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Enc_Resolution(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&inc25KhzObj.resolution, req->data, 4);
//	memcpy(&inc1KhzObj.resolution, req->data, 4);

    res->size = 0;
    res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Gear_Ratio(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&motor_properties.gear_ratio, req->data, 4);

    res->size = 0;
    res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Torque_Constant(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&motor_properties.Kt, req->data, 4);

    res->size = 0;
    res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Velocity_Constant(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&motor_properties.Ke, req->data, 4);

    res->size = 0;
    res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Peak_Limit(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&motor_setting.peakCurr_limit, req->data, 4);

    res->size = 0;
    res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Continu_Limit(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&motor_setting.contCurr_limit, req->data, 4);

    res->size = 0;
    res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Max_Velocity(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&motor_setting.max_velocity_rpm, req->data, 4);

	motor_setting.max_velocity_radPsec = motor_setting.max_velocity_rpm*M_PI/30;
    res->size = 0;
    res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Commutation_Duty(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&motor_setting.commutation_set.max_duty, req->data, 2);

    res->size = 0;
    res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_User_Direction(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&motor_setting.commutation_set.user_desired_dir, (int8_t*)req->data, 1);
	res->size = 0;
    res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Elec_Sys_ID_Mag(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&elec_system_id.level, req->data, 4);

    res->size = 0;
    res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Resistance(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&motor_properties.R, req->data, 4);

    res->size = 0;
    res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Inductance(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&motor_properties.L, req->data, 4);

    res->size = 0;
    res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_BEMF_ID_velocity(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&motor_setting.bemf_id_vel, req->data, 4);

    res->size = 0;
    res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_BEMF_ID_Gain_PCTG(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&motor_setting.bemf_id_gain_pctg, req->data, 1);

    res->size = 0;
    res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Curr_Ctrl_BW(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&motor_setting.currCtrl_BW_radPsec, req->data, 4);

    res->size = 0;
    res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Inertia(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&motor_properties.J, req->data, 4);

    res->size = 0;
    res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Damping_Coef(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&motor_properties.B, req->data, 4);

    res->size = 0;
    res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Mech_Model_a(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&motor_properties.a, req->data, 4);

    res->size = 0;
    res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Mech_Model_b(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&motor_properties.b, req->data, 4);

    res->size = 0;
    res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Friction_ID_Info(MsgSDOargs* req, MsgSDOargs* res)
{
	float t_tps = 0.0, t_ct = 0.0, t_P_pctg = 0.0, t_I_pctg = 0.0;

	memcpy(&advanced_friction_id.max_vel1, 	req->data, 4);
	memcpy(&advanced_friction_id.max_vel2, 	((float*)req->data)+1, 4);
	memcpy(&advanced_friction_id.vel_num1, 	((float*)req->data)+2, 4);
	memcpy(&advanced_friction_id.vel_num2, 	((float*)req->data)+3, 4);
	memcpy(&t_tps, 							((float*)req->data)+4, 4);
	memcpy(&t_ct, 							((float*)req->data)+5, 4);
	memcpy(&t_P_pctg,					    ((float*)req->data)+6, 4);
	memcpy(&t_I_pctg,						((float*)req->data)+7, 4);

	advanced_friction_id.time_per_vel = LOW_LEVEL_CONTROL_FREQUENCY*t_tps;
	advanced_friction_id.cut_time = LOW_LEVEL_CONTROL_FREQUENCY*t_ct;

	// TODO : Reassgin!!
//	advanced_friction_id.P_gain = (motor_setting.contCurr_limit)*t_P_pctg/20;
//	advanced_friction_id.I_gain = (motor_setting.contCurr_limit)*t_I_pctg/20;	
	advanced_friction_id.P_gain = (motor_setting.contCurr_limit)*t_P_pctg/100;
	advanced_friction_id.I_gain = (motor_setting.contCurr_limit)*t_I_pctg/100;

    res->size = 0;
    res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Friction_LUT_Info(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&advanced_friction_id.lut_mdl, 					req->data, 4);
	memcpy(&advanced_friction_id.lut_p1, 		((float*)req->data)+1, 4);
	memcpy(&advanced_friction_id.lut_p2, 		((float*)req->data)+2, 4);
	memcpy(&advanced_friction_id.lut_p3, 		((float*)req->data)+3, 4);
	memcpy(&advanced_friction_id.lut_p4, 		((float*)req->data)+4, 4);
	memcpy(&advanced_friction_id.lut_p5, 		((float*)req->data)+5, 4);
	memcpy(&advanced_friction_id.gain,	        ((float*)req->data)+6, 4);
	memcpy(&advanced_friction_id.lut_d_mu_v, 	((float*)req->data)+7, 4);
	memcpy(&advanced_friction_id.lut_epsilon,	((float*)req->data)+8, 4);

    MS_enum = CAL_FRICTION_LUT;

    res->size = 0;
    res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Commutation_Sensor(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&motor_setting.sensor_setting.commutation_sensor, req->data, 1);

    res->size = 0;
    res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Position_Feedback_Sensor(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&motor_setting.sensor_setting.pos_feedback_sensor, req->data, 1);

    res->size = 0;
    res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Elec_Angle_Homing_Sensor(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&motor_setting.sensor_setting.e_angle_homing_sensor, req->data, 1);

    res->size = 0;
    res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Mech_Angle_Homing_Sensor(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&motor_setting.sensor_setting.m_angle_homing_sensor, req->data, 1);

    res->size = 0;
    res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Sensor_Usage(MsgSDOargs* req, MsgSDOargs* res)
{

	memcpy(&motor_setting.sensor_setting.temperature_sensor_usage,           req->data, 1);
	memcpy(&motor_setting.sensor_setting.imu_6axis_usage, 		  ((char*)req->data)+1, 1);
	memcpy(&motor_setting.sensor_setting.imu_3axis_usage, 		  ((char*)req->data)+2, 1);

    res->size = 0;
    res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Current_Periodic_Sig_Info(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&ll_cur_periodic_sig.amp, 		t_req->data, 			 4);
	memcpy(&ll_cur_periodic_sig.freq, 		((float*)t_req->data)+1, 4);

   t_res->size = 0;
   t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_MD_Version(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&motor_properties.md_version, 		t_req->data, 			 1);

	uint8_t t_ver = motor_properties.md_version;
	if ((t_ver == e_None) | (t_ver == e_R06) | (t_ver == e_R07) | (t_ver == e_R08))
	{
		motor_setting.sensor_setting.curr_sensor_range = 29; // default
	}
	if (t_ver == e_R08H)
	{
		motor_setting.sensor_setting.curr_sensor_range = 240;
	}

	motor_setting.sensor_setting.curr_sensor_a2d = ADC1_RESOLUTION / motor_setting.sensor_setting.curr_sensor_range;
	motor_setting.sensor_setting.curr_sensor_d2a = 1/motor_setting.sensor_setting.curr_sensor_a2d;

   t_res->size = 0;
   t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_IncEncoder_Prescaler(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	uint16_t t_inc_enc_prescaler;
	memcpy(&t_inc_enc_prescaler, 		t_req->data, 			 2);

	if (t_inc_enc_prescaler < 0)
	{
		t_inc_enc_prescaler = 1;
	}
	else if (t_inc_enc_prescaler > 8192)
	{
		t_inc_enc_prescaler = 8192;
	}
	else if (isnanf(t_inc_enc_prescaler))
	{
		t_inc_enc_prescaler = 1;
	}

	inc25KhzObj.prescaler = t_inc_enc_prescaler;
	inc1KhzObj.prescaler  = t_inc_enc_prescaler;

	t_res->size = 0;
	t_res->status = DATA_OBJECT_SDO_SUCC;
}



static void Set_BEMF_Comp_Gain(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	float t_gain;
	memcpy(&t_gain, 		t_req->data, 			 4);

	if (t_gain > 1) t_gain = 1;
	if (t_gain < 0) t_gain = 0;

	kf_current_object.BEMF_Comp_Gain = t_gain;

	t_res->size = 0;
	t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Friction_Comp_Gain_SS(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	float t_gain;
	memcpy(&t_gain, 		t_req->data, 			 4);

	ss_friction_gain = t_gain;

	t_res->size = 0;
	t_res->status = DATA_OBJECT_SDO_SUCC;

}

static void Set_Current_KF_ONOFF(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&motor_setting.low_level_kalman_on, 		t_req->data, 			 1);

	t_res->size = 0;
	t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Current_KF_Q(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&kf_current_object.kf_Q, 		t_req->data, 			 4);

	t_res->size = 0;
	t_res->status = DATA_OBJECT_SDO_SUCC;
}


static void Set_Current_KF_R(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&kf_current_object.kf_R, 		t_req->data, 			 4);

	t_res->size = 0;
	t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Current_KF_A(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&kf_current_object.kf_A, 		t_req->data, 			 4);

	t_res->size = 0;
	t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Current_KF_B(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&kf_current_object.kf_B, 		t_req->data, 			 4);

	t_res->size = 0;
	t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_BEMF_Constant(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&kf_current_object.kf_C, 		t_req->data, 			 4);

	t_res->size = 0;
	t_res->status = DATA_OBJECT_SDO_SUCC;
}


/* ------------------- ROUTINE ------------------- */
/*ROUTINE_ID_BLDC_DIRECTION_SET*/
static int Ent_Set_Commutation()
{
	motor_setting.commutation_set.duration1 = 0.2; // unit (s)
	motor_setting.commutation_set.duration2 = 0.1; // unit (s)

	if (isnanf(motor_setting.commutation_set.max_duty))
		motor_setting.commutation_set.max_duty = 300;
	else if (motor_setting.commutation_set.max_duty <= 0)
		motor_setting.commutation_set.max_duty = 300;

	motor_setting.commutation_set.duty_gap = motor_setting.commutation_set.max_duty / (motor_setting.commutation_set.duration1 * LOW_LEVEL_CONTROL_FREQUENCY);

	motor_setting.commutation_set.start              = 1;
	motor_setting.commutation_set.done               = 0;
	motor_setting.commutation_set.state              = 0;
	motor_setting.commutation_set.abs_encoder_offset = 0;
	motor_setting.commutation_set.abs_encoder_cnt    = 0;
	motor_setting.commutation_set.duty               = 0;
	motor_setting.commutation_set.user_desired_dir   = 0;
	motor_setting.commutation_set.run_cnt            = 0;
	IOIF_ReadIncCnt(IOIF_TIM5, &inc25KhzObj);
	motor_setting.commutation_set.initial_cnt        = inc25KhzObj.userCnt;
	return 0;
}

static int Run_Set_Commutation()
{
	int32_t inc_after_commutation = 0;

	/*(STEP1) Start Forced-Commutation */
	if (motor_setting.commutation_set.state == 0)
	{
		if (motor_setting.commutation_set.run_cnt == 0)
		{
			motor_setting.commutation_set.duty = (uint16_t)(motor_setting.commutation_set.duty_gap*(float)motor_setting.commutation_set.time_stamp);
			if (motor_setting.commutation_set.duty > motor_setting.commutation_set.max_duty)
				motor_setting.commutation_set.duty = motor_setting.commutation_set.max_duty;
		}
		else
		{
			motor_setting.commutation_set.duty = motor_setting.commutation_set.max_duty;
		}

		if (motor_setting.commutation_set.user_desired_dir == 0){

			if      (motor_setting.commutation_set.comm_step == 0)		excitate_phase_UVnWn(motor_setting.commutation_set.duty);
			else if (motor_setting.commutation_set.comm_step == 1)		excitate_phase_UVWn(motor_setting.commutation_set.duty);
			else if (motor_setting.commutation_set.comm_step == 2)		excitate_phase_UnVWn(motor_setting.commutation_set.duty);
			else if (motor_setting.commutation_set.comm_step == 3)		excitate_phase_UnVW(motor_setting.commutation_set.duty);
			else if (motor_setting.commutation_set.comm_step == 4)		excitate_phase_UnVnW(motor_setting.commutation_set.duty);
			else if (motor_setting.commutation_set.comm_step == 5)		excitate_phase_UVnW(motor_setting.commutation_set.duty);

			motor_setting.commutation_set.time_stamp++;

			if (motor_setting.commutation_set.time_stamp >= (motor_setting.commutation_set.duration1 + motor_setting.commutation_set.duration2)*LOW_LEVEL_CONTROL_FREQUENCY)
			{
				if (motor_setting.sensor_setting.hall_sensor_usage == 1)
				{
					Read_Hall_Sensors(&hallObj);
					motor_setting.commutation_set.hall_sensor_table[motor_setting.commutation_set.comm_step] = hallObj.hall_logic;
				}

				if (motor_setting.commutation_set.comm_step == 0)
				{
					float t_position = 0;
					uint16_t t_x;
					uint16_t t_abs_encoder_e_angle_offset;

					if (motor_setting.sensor_setting.e_angle_homing_sensor == e_EHoming_Sensor_Abs_Encoder1) {
						IOIF_GetPosVelDeg(IOIF_SPI1, &AbsObj1);
						if (AbsObj1.location == IOIF_ABS_ENC_ACTUATOR_INPUT) {
							t_position = AbsObj1.posDeg;
						}
					}
					else if (motor_setting.sensor_setting.e_angle_homing_sensor == e_EHoming_Sensor_Abs_Encoder2) {
						IOIF_GetPosVelDeg(IOIF_SPI3, &AbsObj2);
						if (AbsObj2.location == IOIF_ABS_ENC_ACTUATOR_INPUT) {
							t_position = AbsObj2.posDeg;
						}
					}

					if (t_position < 0) t_position = t_position + 360; // 0~360
					t_x = t_position * 182.0444444444445;              // 360 degree to 65535

					t_abs_encoder_e_angle_offset = t_x - ((motor_properties.pole_pair * t_x)>>16)*(65536/motor_properties.pole_pair);

					// recursive average
					motor_setting.commutation_set.abs_encoder_offset = (t_abs_encoder_e_angle_offset + motor_setting.commutation_set.abs_encoder_offset*motor_setting.commutation_set.abs_encoder_cnt)/(motor_setting.commutation_set.abs_encoder_cnt + 1);
					motor_setting.commutation_set.abs_encoder_cnt++;
				}

				motor_setting.commutation_set.time_stamp   = 0;
				motor_setting.commutation_set.duty         = 0;
				motor_setting.commutation_set.comm_step    = (motor_setting.commutation_set.comm_step + 1) % 6;
				motor_setting.commutation_set.run_cnt++;
			}
		}
		else
		{
			motor_setting.commutation_set.time_stamp = 0;
			motor_setting.commutation_set.comm_step = 0;
			motor_setting.commutation_set.run_cnt   = 0;
			motor_setting.commutation_set.abs_encoder_cnt = 0;
			motor_setting.commutation_set.state = 1;
			excitate_phase_UnVnWn(); // 0-voltage
		}
	}

	/* (STEP2) Decision Rotation Direction */
	else if (motor_setting.commutation_set.state == 1)
	{
		if (motor_setting.sensor_setting.commutation_sensor == e_Commute_Sensor_Inc_Encoder){
			IOIF_ReadIncCnt(IOIF_TIM5, &inc25KhzObj);
			inc_after_commutation = inc25KhzObj.userCnt - motor_setting.commutation_set.initial_cnt;
		}
		else
		{
			return -1; // (error): please set proper commutation sensor
		}

		/* Set commutation direction */
		if(inc_after_commutation > 0) {motor_setting.commutation_set.ea_dir = +1; }
		else                          {motor_setting.commutation_set.ea_dir = -1; }

		/* Set current control direction */
		if      (motor_setting.commutation_set.user_desired_dir == 1) {motor_setting.commutation_set.cc_dir = +1; motor_setting.commutation_set.hall_sensor_dir = +1; hallObj.direction_set = +1; }
		else if (motor_setting.commutation_set.user_desired_dir == 2) {motor_setting.commutation_set.cc_dir = -1; motor_setting.commutation_set.hall_sensor_dir = -1; hallObj.direction_set = -1;}

		/* Set mechanical angle direction */
		if     ((inc_after_commutation > 0) && (motor_setting.commutation_set.user_desired_dir == 1))
		{motor_setting.commutation_set.ma_dir = +1;}
		else if((inc_after_commutation > 0) && (motor_setting.commutation_set.user_desired_dir == 2))
		{motor_setting.commutation_set.ma_dir = -1;}
		else if((inc_after_commutation < 0) && (motor_setting.commutation_set.user_desired_dir == 1))
		{motor_setting.commutation_set.ma_dir = -1;}
		else if((inc_after_commutation < 0) && (motor_setting.commutation_set.user_desired_dir == 2))
		{motor_setting.commutation_set.ma_dir = +1;}

		motor_setting.commutation_set.state = 2;
	}

	/* (STEP3) Check Hall sensor table */
	if (motor_setting.commutation_set.state == 2) {

		if (motor_setting.sensor_setting.hall_sensor_usage == 1)
		{
			float t_sum = 0;
			float t_mul = 1;
			for (int i = 0; i < 6; i++){
				t_sum = t_sum + motor_setting.commutation_set.hall_sensor_table[i];
				t_mul = t_mul * motor_setting.commutation_set.hall_sensor_table[i];
			}

			if ((t_sum != 21) || (t_mul != 720)) {
				motor_setting.commutation_set.state = 0;
				motor_setting.commutation_set.done = 0;
				motor_setting.commutation_set.user_desired_dir = 0;
				Transition_State(&low_level_ctrl_task.state_machine, e_State_Standby);
				//Transition_State(&msg_hdlr_task.state_machine,       e_State_Standby);
				low_level_triggered_msg_stop = 1;
				low_level_ctrl_task.err_code = ERROR_HALL_TABLE_GENERATION_FAILED;
				return -1; // (error): please set proper commutation sensor
				}
			else
				motor_setting.commutation_set.state = 3;
		}
		else
			motor_setting.commutation_set.state = 3;
	}

	/* (STEP4) Finish Driver Commutation Setting */
	if (motor_setting.commutation_set.state == 3){
		motor_setting.commutation_set.state = 0;
		motor_setting.commutation_set.start = 0;
		motor_setting.commutation_set.done = 1;
		motor_setting.commutation_set.user_desired_dir = 0;
		Transition_State(&low_level_ctrl_task.state_machine, e_State_Standby);
		low_level_triggered_msg_stop = 1;
		//Transition_State(&msg_hdlr_task.state_machine,       e_State_Standby);
		return 0;
	}

	return 1;

}

/*ROUTINE_ID_BLDC_ELECTRICAL_SYS_ID*/
static int Ent_Electrical_System_ID()
{
	static int init_index = 0;

	if(init_index == 0){
		elec_system_id.flag = 0;
		elec_system_id.cnt = 0;
	    init_index++;
	}

	elec_system_id.input = 0;
	elec_system_id.signal = 0;
	elec_system_id.f_signal = 0;
	elec_system_id.output = 0.0;	//in amphere
    elec_system_id.flag = 0;
    elec_system_id.cnt = 0;

    memset(shared_array_buffer.buf_1st, 0, sizeof(shared_array_buffer.buf_1st));
    memset(shared_array_buffer.buf_2nd, 0, sizeof(shared_array_buffer.buf_2nd));

    motor_setting.low_level_kalman_on = 0;

	return 0;
}

static int Run_Electrical_System_ID()
{
	if(elec_system_id.flag == 0){

		/* Phase Align */
		if(elec_system_id.cnt < 4000){

			excitate_phase_UVn(500);

		} else if((4000<=elec_system_id.cnt) && (elec_system_id.cnt <= 8000)){

			excitate_phase_UnVnWn();

			/* System ID by Chirp signal */
		} else if(8000<elec_system_id.cnt){

			/* chirp */
			if(elec_system_id.cnt >= (8000+CHIRP_ARRAY_SIZE)) {
				elec_system_id.flag = 1;
				excitate_phase_UnVnWn();
			} else {

				elec_system_id.f_signal = elec_system_id.level*chirp_freq[elec_system_id.cnt-8001]*VBUS2DUTY_RATIO;			// 100~500hz		mag: 0~4800 (= 0~48V)


				if(elec_system_id.f_signal >= 0){
					excitate_phase_UVn((uint16_t)elec_system_id.f_signal);
				} else {
					excitate_phase_UnV((uint16_t)(-elec_system_id.f_signal));
				}

				elec_system_id.output = (motor_out.I_U - motor_out.I_V);					// mag: -32768 ~ 32767

				shared_array_buffer.buf_1st[elec_system_id.cnt-8001] = elec_system_id.f_signal/2;
				shared_array_buffer.buf_2nd[elec_system_id.cnt-8001] = elec_system_id.output/2;
			}
		}

		elec_system_id.cnt++;
	}

	if(elec_system_id.flag){
	    Transition_State(&low_level_ctrl_task.state_machine, e_State_Standby);
	    MS_enum = ELECTRICAL_SYSTEM_ID;
		return 0;
	}

	return 0;

}

/*ROUTINE_ID_BLDC_ELECTRICAL_BEMF*/
static int Ent_BEMF_ID()
{
	motor_setting.currCtrl_BW_radPsec = 1000*2*M_PI;
	motor_setting.bemf_id_gain = (motor_setting.contCurr_limit/10)*motor_setting.bemf_id_gain_pctg/VBUS2DUTY_RATIO;

	return 0;
}
static int Run_BEMF_ID()
{
	static int t_flag = 0;
	static float t_error = 0.0;

	t_error = motor_setting.bemf_id_vel - motor_out.velocity;

	if(low_level_ctrl_loop_cnt == 0){

		if(motor_setting.low_level_kalman_on == 1){
			motor_setting.low_level_kalman_on = 0;
			t_flag = 1;
		} else {
			t_flag = 0;
		}
	}

	if(low_level_ctrl_loop_cnt <= 25000){	// 1 sec

		motor_in.low_id_process_input = motor_setting.bemf_id_gain*t_error;
		rms_err = 0;
		mean_vel = 0;

		kf_current_object.kf_C = 0;
		kf_current_object.kf_Q = 0;

	}else if((low_level_ctrl_loop_cnt > 25000) && (low_level_ctrl_loop_cnt < 100000)){
		N = low_level_ctrl_loop_cnt - 25000;

		motor_in.low_id_process_input = motor_setting.bemf_id_gain*t_error;

        rms_err = sqrt(((motor_out.I_U_KF - motor_out.I_U)*(motor_out.I_U_KF - motor_out.I_U) + (rms_err*rms_err) * (N-1))/N);
        mean_vel = (motor_out.velocity + mean_vel * (N-1))/N;

	}else{
		kf_current_object.kf_C = rms_err * 1.4142 * (1-kf_current_object.kf_A)/mean_vel;
		motor_in.low_id_process_input = 0;
		kf_current_object.kf_Q = 1;

		if(t_flag)		motor_setting.low_level_kalman_on = 1;

		Transition_State(&low_level_ctrl_task.state_machine, e_State_Standby);
		MS_enum = BEMF_ID;
		return 0;
	}

	return 0;
}
static int Ext_BEMF_ID()
{
	// Added
	if(low_level_ctrl_task.err_code == ERROR_PHASE_OVER_CURRENT){
		static uint8_t temp_arr = 0;
		static uint16_t identifier = 0;

	    identifier = GUI_SYNC|BEMF_ID_OVER_CURRENT;
		Send_MSG(identifier, 0, &temp_arr);
	}

	RT_Broken = 0;
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_D, LED_BOOT_RED_Pin, IOIF_GPIO_PIN_RESET);

    motor_setting.low_level_kalman_on = 1;
	return 0;
}

/*ROUTINE_ID_BLDC_CURRENT_CTRL*/
static int Ent_Current_Control()
{
	motor_out.I_U_KF = 0;
	motor_out.I_V_KF = 0;
	motor_out.I_W_KF = 0;
	motor_out.velocity = 0;

	uint8_t t_md_ver = motor_properties.md_version;
	if ((t_md_ver == 0) | (t_md_ver == 1) | (t_md_ver == 2) | (t_md_ver == 3)) {
		Init_PID(&pid_d_axis_ctrl, motor_properties.L * 0.4272 * (motor_setting.currCtrl_BW_radPsec), motor_properties.R * 0.4272 * (motor_setting.currCtrl_BW_radPsec), 0);
		Init_PID(&pid_q_axis_ctrl, motor_properties.L * 0.4272 * (motor_setting.currCtrl_BW_radPsec), motor_properties.R * 0.4272 * (motor_setting.currCtrl_BW_radPsec), 0);
		Init_PID(&pid_trape_ctrl,  motor_properties.L * 0.4272 * (motor_setting.currCtrl_BW_radPsec), motor_properties.R * 0.4272 * (motor_setting.currCtrl_BW_radPsec), 0);
	}
	else if (t_md_ver == 4)
	{
		Init_PID(&pid_d_axis_ctrl, motor_properties.L * 3.5355 * (motor_setting.currCtrl_BW_radPsec), motor_properties.R * 3.5355 * (motor_setting.currCtrl_BW_radPsec), 0);
		Init_PID(&pid_q_axis_ctrl, motor_properties.L * 3.5355 * (motor_setting.currCtrl_BW_radPsec), motor_properties.R * 3.5355 * (motor_setting.currCtrl_BW_radPsec), 0);
		Init_PID(&pid_trape_ctrl,  motor_properties.L * 3.5355 * (motor_setting.currCtrl_BW_radPsec), motor_properties.R * 3.5355 * (motor_setting.currCtrl_BW_radPsec), 0);
	}

	motor_in.auxiliary_input = 0;
	motor_in.f_vector_input = 0;
	motor_in.low_id_process_input = 0;
	motor_in.friction_input = 0;
	motor_in.mid_id_process_input = 0;
	motor_in.irc_input = 0;
	motor_in.mid_ctrl_input = 0;
	motor_in.analysis_input = 0;

	Init_Current_Ctrl();

	return 0;
}


static int Run_Current_Control()
{
	motor_in.total_current_input = motor_in.auxiliary_input 		+ \
			                       motor_in.f_vector_input          + \
								   motor_in.low_id_process_input 	+ \
								   motor_in.friction_input 			+ \
								   motor_in.irc_input 				+ \
								   motor_in.mid_ctrl_input 			+ \
								   motor_in.analysis_input;

	//TODO
	if       ((motor_setting.sensor_setting.commutation_sensor == e_Commute_Sensor_Inc_Encoder)
		   || (motor_setting.sensor_setting.commutation_sensor == e_Commute_Sensor_Inc_Encoder_Hall))
	{
		if (motor_setting.sensor_setting.e_angle_homing_sensor == e_EHoming_Sensor_Hall)
		{
			if (motor_setting.elec_angle_homing.done == 1)			Cal_Elec_Angle();
			else                                    				Cal_Elec_Angle_with_Hall();
		}
		else														Cal_Elec_Angle();

		Run_FOC_Ctrl(motor_in.total_current_input);

	} else if(motor_setting.sensor_setting.commutation_sensor == e_Commute_Sensor_Hall) {

		Get_3Phase_Current();
		if (motor_setting.sensor_setting.hall_sensor_usage == 1) Read_Hall_Sensors(&hallObj);
		Cal_Trape_Total_Current();
		Run_Trape_Ctrl(motor_in.total_current_input);

	}

	return 0;
}

/*ROUTINE_ID_LOWLEVEL_CURRENT_SINE_REF*/
static int Ent_LL_Generate_Current_Sine() {return 0;}
static int Run_LL_Generate_Current_Sine()
{
	motor_in.low_id_process_input = Generate_Sine(ll_cur_periodic_sig.amp, 0, ll_cur_periodic_sig.freq, low_level_ctrl_loop_cnt, LOW_LEVEL_CONTROL_PERIOD, 0);

	return 0;
}

/*ROUTINE_ID_LOWLEVEL_CURRENT_TANH_REF*/
static int Ent_LL_Generate_Current_Tanh() {return 0;}
static int Run_LL_Generate_Current_Tanh()
{
	//velCtrl.ref = Generate_Rectangle(vel_periodic_sig.amp, 0, vel_periodic_sig.freq, 0.5, mid_level_loop_cnt, MID_LEVEL_CONTROL_PERIOD, 1);
	motor_in.low_id_process_input = Generate_Rectangle_tanh(ll_cur_periodic_sig.amp, 0, ll_cur_periodic_sig.freq, low_level_ctrl_loop_cnt, LOW_LEVEL_CONTROL_PERIOD, 0);

	return 0;
}

/*ROUTINE_ID_LOWLEVEL_CURRENT_REC_REF*/
static int Ent_LL_Generate_Current_Rec() {return 0;}
static int Run_LL_Generate_Current_Rec()
{
	motor_in.low_id_process_input = Generate_Rectangle(ll_cur_periodic_sig.amp, 0, ll_cur_periodic_sig.freq, 0.5, low_level_ctrl_loop_cnt, LOW_LEVEL_CONTROL_PERIOD, 1);

	return 0;
}

static int Ent_Check_Current_Controller_Tracking()
{
	memset(shared_array_buffer.buf_1st, 0, sizeof(shared_array_buffer.buf_1st));
	memset(shared_array_buffer.buf_2nd, 0, sizeof(shared_array_buffer.buf_2nd));
	return 0;
}

static int Run_Check_Current_Controller_Tracking()
{
	if(low_level_ctrl_loop_cnt <= CURRENT_CTRL_EVAL_ARRAY_SIZE){

		shared_array_buffer.buf_1st[low_level_ctrl_loop_cnt] = motor_out.current_act;
		shared_array_buffer.buf_2nd[low_level_ctrl_loop_cnt] = motor_in.low_id_process_input;

	}else{

		motor_in.low_id_process_input = 0;
		Transition_State(&low_level_ctrl_task.state_machine, e_State_Standby);
	    MS_enum = CURRENT_TRACKING_CHECK;

	}
	return 0;
}


/*ROUTINE_ID_BLDC_CURRENT_CTRL_BANDWIDTH_CHECK*/
static int Ent_Check_Current_Controller_Bandwidth()
{
	memset(shared_array_buffer.buf_1st, 0, sizeof(shared_array_buffer.buf_1st));
	memset(shared_array_buffer.buf_2nd, 0, sizeof(shared_array_buffer.buf_2nd));
	return 0;
}
static int Run_Check_Current_Controller_Bandwidth()
{
	if(low_level_ctrl_loop_cnt <= BW_CHECK_ARRAY_SIZE){

		motor_in.low_id_process_input = current_bandwidth[low_level_ctrl_loop_cnt];
		shared_array_buffer.buf_1st[low_level_ctrl_loop_cnt] = motor_out.current_act;
		shared_array_buffer.buf_2nd[low_level_ctrl_loop_cnt] = motor_in.low_id_process_input;

	}else{

		motor_in.low_id_process_input = 0;
		Transition_State(&low_level_ctrl_task.state_machine, e_State_Standby);
	    MS_enum = CURRENT_BANDWIDTH_CHECK;

	}

	return 0;
}
static int Ext_Check_Current_Controller_Bandwidth()
{
	RT_Broken = 0;
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_D, LED_BOOT_RED_Pin, IOIF_GPIO_PIN_RESET);
	return 0;
}

static int Ent_Advanced_Friction_ID()
{
	if((advanced_friction_id.time_per_vel == 0) ||
	   (advanced_friction_id.section_time == 0) ||
	   (advanced_friction_id.max_vel1 == 0) ||
	   (advanced_friction_id.max_vel2 == 0) ||
	   (advanced_friction_id.vel_num1 == 0) ||
	   (advanced_friction_id.vel_num2 == 0) ||
	   (advanced_friction_id.P_gain == 0)){

		Transition_State(&low_level_ctrl_task.state_machine, e_State_Standby);

		return 0;
	}

	/* Entry */
	advanced_friction_id.gap1 = advanced_friction_id.max_vel1 / advanced_friction_id.vel_num1;
	advanced_friction_id.gap2 = (advanced_friction_id.max_vel2 - advanced_friction_id.max_vel1) / advanced_friction_id.vel_num2;


    memset(shared_array_buffer.buf_1st, 0, sizeof(shared_array_buffer.buf_1st));
    memset(shared_array_buffer.buf_2nd, 0, sizeof(shared_array_buffer.buf_2nd));

	advanced_friction_id.state = 0;
	advanced_friction_id.friction_cnt = 1;
	advanced_friction_id.friction_index = 0;

	advanced_friction_id.section_time[0] = advanced_friction_id.time_per_vel * advanced_friction_id.vel_num1;
	advanced_friction_id.section_time[1] = advanced_friction_id.time_per_vel * (advanced_friction_id.vel_num1 + advanced_friction_id.vel_num2);
	advanced_friction_id.section_time[2] = advanced_friction_id.time_per_vel * (advanced_friction_id.vel_num1 + 2*advanced_friction_id.vel_num2);
	advanced_friction_id.section_time[3] = advanced_friction_id.time_per_vel * (2*advanced_friction_id.vel_num1 + 2*advanced_friction_id.vel_num2);
	advanced_friction_id.section_time[4] = advanced_friction_id.time_per_vel * (3*advanced_friction_id.vel_num1 + 2*advanced_friction_id.vel_num2);
	advanced_friction_id.section_time[5] = advanced_friction_id.time_per_vel * (3*advanced_friction_id.vel_num1 + 3*advanced_friction_id.vel_num2);
	advanced_friction_id.section_time[6] = advanced_friction_id.time_per_vel * (3*advanced_friction_id.vel_num1 + 4*advanced_friction_id.vel_num2);
	advanced_friction_id.section_time[7] = advanced_friction_id.time_per_vel * (4*advanced_friction_id.vel_num1 + 4*advanced_friction_id.vel_num2);
	advanced_friction_id.e_sum           = 0;

	return 0;
}

static int Run_Advanced_Friction_ID()
{
	static uint8_t save = 0;
	advanced_friction_id.state = 1;

	if((low_level_ctrl_loop_cnt % advanced_friction_id.time_per_vel) == 0){ // @ every 3 sec
		advanced_friction_id.friction_cnt = 1;
		if(low_level_ctrl_loop_cnt == 0){   // Init
			advanced_friction_id.friction_index = 0;
		} else{
			if (save) advanced_friction_id.friction_index++;
		}
	}

	int i = (low_level_ctrl_loop_cnt/advanced_friction_id.time_per_vel) + 1;


	if(low_level_ctrl_loop_cnt < advanced_friction_id.section_time[0]){
		advanced_friction_id.friction_ref = i*advanced_friction_id.gap1;
		save = 1;

	} else if(low_level_ctrl_loop_cnt < advanced_friction_id.section_time[1]){
		advanced_friction_id.friction_ref = advanced_friction_id.max_vel1 + (i - advanced_friction_id.vel_num1)*advanced_friction_id.gap2;
		save = 1;

	} else if(low_level_ctrl_loop_cnt < advanced_friction_id.section_time[2]){
		advanced_friction_id.friction_ref = advanced_friction_id.max_vel2 - (i - advanced_friction_id.vel_num1 - advanced_friction_id.vel_num2)*advanced_friction_id.gap2;
		save = 1;

	} else if(low_level_ctrl_loop_cnt < advanced_friction_id.section_time[3]){
		advanced_friction_id.friction_ref = advanced_friction_id.max_vel1 - (i - advanced_friction_id.vel_num1 - 2*advanced_friction_id.vel_num2)*advanced_friction_id.gap1;
		save = 1;

	} else if(low_level_ctrl_loop_cnt < advanced_friction_id.section_time[4]){
		advanced_friction_id.friction_ref = -(i - 2*advanced_friction_id.vel_num1 - 2*advanced_friction_id.vel_num2)*advanced_friction_id.gap1;
		save = 1;

	} else if(low_level_ctrl_loop_cnt < advanced_friction_id.section_time[5]){
		advanced_friction_id.friction_ref = -advanced_friction_id.max_vel1 - (i - 3*advanced_friction_id.vel_num1 - 2*advanced_friction_id.vel_num2)*advanced_friction_id.gap2;
		save = 1;

	} else if(low_level_ctrl_loop_cnt < advanced_friction_id.section_time[6]){
		advanced_friction_id.friction_ref = -advanced_friction_id.max_vel2 + (i - 3*advanced_friction_id.vel_num1 - 3*advanced_friction_id.vel_num2)*advanced_friction_id.gap2;
		save = 1;

	} else if(low_level_ctrl_loop_cnt < advanced_friction_id.section_time[7]){
		advanced_friction_id.friction_ref = -advanced_friction_id.max_vel1 + (i - 3*advanced_friction_id.vel_num1 - 4*advanced_friction_id.vel_num2)*advanced_friction_id.gap1;
		save = 1;
	} else{

		advanced_friction_id.friction_ref = 0;
		motor_in.low_id_process_input  = 0;
		MS_enum = ADV_FRICTION_ID;
		Transition_State(&low_level_ctrl_task.state_machine, e_State_Standby);
	}

	if (save == 1)
	{
		advanced_friction_id.e        = advanced_friction_id.friction_ref - motor_out.velocity;
		advanced_friction_id.e_sum    = advanced_friction_id.e_sum + advanced_friction_id.e*LOW_LEVEL_CONTROL_PERIOD;
		motor_in.low_id_process_input = advanced_friction_id.P_gain*advanced_friction_id.e + advanced_friction_id.I_gain*advanced_friction_id.e_sum;
	}

	/* Moving Averaging */
/*	if((advanced_friction_id.friction_cnt >= advanced_friction_id.cut_time) && (save == 1)){
		shared_array_buffer.buf_1st[advanced_friction_id.friction_index] = ((advanced_friction_id.friction_cnt - advanced_friction_id.cut_time) * \
																				shared_array_buffer.buf_1st[advanced_friction_id.friction_index] + \
																				motor_out.velocity) / \
																				(advanced_friction_id.friction_cnt - (advanced_friction_id.cut_time-1));
		shared_array_buffer.buf_2nd[advanced_friction_id.friction_index] = ((advanced_friction_id.friction_cnt - advanced_friction_id.cut_time) *\
																				shared_array_buffer.buf_2nd[advanced_friction_id.friction_index] + \
																				motor_in.low_id_process_input) / \
																				(advanced_friction_id.friction_cnt - (advanced_friction_id.cut_time-1));
	}*/

	if((advanced_friction_id.friction_cnt >= advanced_friction_id.cut_time) && (save == 1)){
		shared_array_buffer.buf_1st[advanced_friction_id.friction_index] = ((advanced_friction_id.friction_cnt - advanced_friction_id.cut_time) * \
																				shared_array_buffer.buf_1st[advanced_friction_id.friction_index] + \
																				motor_out.velocity) / \
																				(advanced_friction_id.friction_cnt - (advanced_friction_id.cut_time-1));
		shared_array_buffer.buf_2nd[advanced_friction_id.friction_index] = ((advanced_friction_id.friction_cnt - advanced_friction_id.cut_time) *\
																				shared_array_buffer.buf_2nd[advanced_friction_id.friction_index] + \
																				motor_out.current_act) / \
																				(advanced_friction_id.friction_cnt - (advanced_friction_id.cut_time-1));
	}

	advanced_friction_id.friction_cnt++;

	return 0;
}

static int Ext_Advanced_Friction_ID()
{
	RT_Broken = 0;
	advanced_friction_id.state = 0;
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_13, IOIF_GPIO_PIN_RESET);

	return 0;
}

static int Ent_Evaluation_Friction_ID_Velocity_Ctrl()
{
	advanced_friction_id.e        = 0;
	advanced_friction_id.e_sum    = 0;
	return 0;
}

static int Run_Evaluation_Friction_ID_Velocity_Ctrl()
{
	advanced_friction_id.e        = advanced_friction_id.max_vel1 - motor_out.velocity;
	advanced_friction_id.e_sum    = advanced_friction_id.e_sum + advanced_friction_id.e*LOW_LEVEL_CONTROL_PERIOD;
	motor_in.low_id_process_input = advanced_friction_id.P_gain*advanced_friction_id.e + advanced_friction_id.I_gain*advanced_friction_id.e_sum;
	advanced_friction_id.state = 1;
	return 0;
}

static int Ent_Adv_Friction_Compensation()
{
	return 0;
}

static int Run_Adv_Friction_Compensation()
{
	int16_t index = 0;

	index = (int16_t)(4096 + motor_out.velocity / advanced_friction_id.scaling_factor);

	if (index < 0)         index = 0;
	else if (index > 8192) index = 8192;

	float t_control_input;

	t_control_input = advanced_friction_id.adv_friction_compensator_LUT[index];

	if (advanced_friction_id.overall_gain_trigger == 1)
	{
		advanced_friction_id.overall_gain_curr = advanced_friction_id.overall_gain_curr + advanced_friction_id.overall_gain_gap;
		if (advanced_friction_id.overall_gain_time_stamp == LOW_LEVEL_CONTROL_FREQUENCY * advanced_friction_id.overall_gain_transition_time)
		{
			advanced_friction_id.overall_gain_gap        = 0;
			advanced_friction_id.overall_gain_time_stamp = 0;
			advanced_friction_id.overall_gain_trigger    = 0;
			advanced_friction_id.overall_gain_curr       = advanced_friction_id.overall_gain_des;
		}
		advanced_friction_id.overall_gain_time_stamp++;

		if (advanced_friction_id.overall_gain_curr > 1) advanced_friction_id.overall_gain_curr = 1;
		if (advanced_friction_id.overall_gain_curr < 0) advanced_friction_id.overall_gain_curr = 0;
	}
	motor_in.friction_input = t_control_input * advanced_friction_id.overall_gain_curr;

	return 0;
}

static int Ent_Adv_Friction_Compensation_SS()
{
	return 0;
}

static int Run_Adv_Friction_Compensation_SS()
{
	int16_t index = 0;

	index = (int16_t)(4096 + motor_out.velocity / advanced_friction_id.scaling_factor);

	if (index < 0)         index = 0;
	else if (index > 8192) index = 8192;

	float t_control_input;

	t_control_input = advanced_friction_id.adv_friction_compensator_LUT[index];

	if (advanced_friction_id.overall_gain_trigger == 1)
	{
		advanced_friction_id.overall_gain_curr = advanced_friction_id.overall_gain_curr + advanced_friction_id.overall_gain_gap;
		if (advanced_friction_id.overall_gain_time_stamp == LOW_LEVEL_CONTROL_FREQUENCY * advanced_friction_id.overall_gain_transition_time)
		{
			advanced_friction_id.overall_gain_gap        = 0;
			advanced_friction_id.overall_gain_time_stamp = 0;
			advanced_friction_id.overall_gain_trigger    = 0;
			advanced_friction_id.overall_gain_curr       = advanced_friction_id.overall_gain_des;
		}
		advanced_friction_id.overall_gain_time_stamp++;

		if (advanced_friction_id.overall_gain_curr > 1) advanced_friction_id.overall_gain_curr = 1;
		if (advanced_friction_id.overall_gain_curr < 0) advanced_friction_id.overall_gain_curr = 0;
	}

	if (motor_out.velocity >= 0)
	{
		motor_in.friction_input = t_control_input * advanced_friction_id.overall_gain_curr;
	}
	else
	{
		motor_in.friction_input = ss_friction_gain * (t_control_input * advanced_friction_id.overall_gain_curr);
	}

	return 0;
}


static int Ent_Adv_Friction_Compensation_FF()
{
	return 0;
}

static int Run_Adv_Friction_Compensation_FF()
{
	int16_t index = 0;

	index = (int16_t)(4096 + advanced_friction_id.friction_ref / advanced_friction_id.scaling_factor);

	if (index < 0)         index = 0;
	else if (index > 8192) index = 8192;

	motor_in.friction_input = advanced_friction_id.adv_friction_compensator_LUT[index];

	return 0;
}

/* Electrical Angle Homing Algorithm*/
static void Run_Forced_Homing()
{
	if      (motor_setting.elec_angle_homing.forced_homing_cnt == 0)
	{
		motor_setting.elec_angle_homing.forced_homing_dy = (float)motor_setting.elec_angle_homing.forced_homing_duty / (float)motor_setting.elec_angle_homing.forced_homing_duration;
		GateDriver_ONOFF(ENABLE);
		htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
	}

	else if (motor_setting.elec_angle_homing.forced_homing_cnt <= motor_setting.elec_angle_homing.forced_homing_duration)
	{
		motor_setting.elec_angle_homing.forced_homing_duty = motor_setting.elec_angle_homing.forced_homing_duty + motor_setting.elec_angle_homing.forced_homing_dy;

		if (motor_setting.elec_angle_homing.forced_homing_duty > 1000) motor_setting.elec_angle_homing.forced_homing_duty = 1000;

		excitate_phase_UVnWn(motor_setting.elec_angle_homing.forced_homing_duty);
	}

	else if (motor_setting.elec_angle_homing.forced_homing_cnt == motor_setting.elec_angle_homing.forced_homing_duration + 5000)
	{
		IOIF_ReadIncCnt(IOIF_TIM5, &inc25KhzObj);
		Cal_Elec_Angle();
		motor_setting.elec_angle_homing.offset = -motor_out.elec_angle;
		excitate_phase_UnVnWn();
	}

	else if (motor_setting.elec_angle_homing.forced_homing_cnt == motor_setting.elec_angle_homing.forced_homing_duration + 15000)
	{
		motor_setting.elec_angle_homing.forced_homing_cnt = 0;
		motor_setting.elec_angle_homing.done = 1;
	}
	motor_setting.elec_angle_homing.forced_homing_cnt++;
}
uint32_t Hall_Matching_IDX = 0;

static void Run_Hall_Homing()
{
	if (motor_setting.sensor_setting.hall_sensor_usage == 1) Read_Hall_Sensors(&hallObj);

	uint8_t t_hall_logic, t_hall_logic_f;
	uint8_t *t_hall_sensor_table;
	t_hall_logic        = hallObj.hall_logic;
	t_hall_logic_f      = hallObj.hall_logic_f;
	t_hall_sensor_table = motor_setting.commutation_set.hall_sensor_table;

	IOIF_ReadIncCnt(IOIF_TIM5, &inc25KhzObj);
	Cal_Elec_Angle();


	if      ((t_hall_logic_f == t_hall_sensor_table[0]) && (t_hall_logic == t_hall_sensor_table[1]))
	{
		motor_setting.elec_angle_homing.offset = -motor_out.elec_angle + 5461;
		motor_setting.elec_angle_homing.done = 1;
	}
	else if ((t_hall_logic_f == t_hall_sensor_table[1]) && (t_hall_logic == t_hall_sensor_table[0]))
	{
		motor_setting.elec_angle_homing.offset = -motor_out.elec_angle + 5461;
		motor_setting.elec_angle_homing.done = 1;
	}
	else if ((t_hall_logic_f == t_hall_sensor_table[1]) && (t_hall_logic == t_hall_sensor_table[2]))
	{
		motor_setting.elec_angle_homing.offset = -motor_out.elec_angle + 16384;
		motor_setting.elec_angle_homing.done = 1;
	}
	else if ((t_hall_logic_f == t_hall_sensor_table[2]) && (t_hall_logic == t_hall_sensor_table[1]))
	{
		motor_setting.elec_angle_homing.offset = -motor_out.elec_angle + 16384;
		motor_setting.elec_angle_homing.done = 1;
	}
	else if ((t_hall_logic_f == t_hall_sensor_table[2]) && (t_hall_logic == t_hall_sensor_table[3]))
	{
		motor_setting.elec_angle_homing.offset = -motor_out.elec_angle + 27307;
		motor_setting.elec_angle_homing.done = 1;
	}
	else if ((t_hall_logic_f == t_hall_sensor_table[3]) && (t_hall_logic == t_hall_sensor_table[2]))
	{
		motor_setting.elec_angle_homing.offset = -motor_out.elec_angle + 27307;
		motor_setting.elec_angle_homing.done = 1;
	}
	else if ((t_hall_logic_f == t_hall_sensor_table[3]) && (t_hall_logic == t_hall_sensor_table[4]))
	{
		motor_setting.elec_angle_homing.offset = -motor_out.elec_angle + 38229;
		motor_setting.elec_angle_homing.done = 1;
	}
	else if ((t_hall_logic_f == t_hall_sensor_table[4]) && (t_hall_logic == t_hall_sensor_table[3]))
	{
		motor_setting.elec_angle_homing.offset = -motor_out.elec_angle + 38229;
		motor_setting.elec_angle_homing.done = 1;
	}
	else if ((t_hall_logic_f == t_hall_sensor_table[4]) && (t_hall_logic == t_hall_sensor_table[5]))
	{
		motor_setting.elec_angle_homing.offset = -motor_out.elec_angle + 49152;
		motor_setting.elec_angle_homing.done = 1;
	}
	else if ((t_hall_logic_f == t_hall_sensor_table[5]) && (t_hall_logic == t_hall_sensor_table[4]))
	{
		motor_setting.elec_angle_homing.offset = -motor_out.elec_angle + 49152;
		motor_setting.elec_angle_homing.done = 1;
	}
	else if ((t_hall_logic_f == t_hall_sensor_table[5]) && (t_hall_logic == t_hall_sensor_table[0]))
	{
		motor_setting.elec_angle_homing.offset = -motor_out.elec_angle + 60075;
		motor_setting.elec_angle_homing.done = 1;
	}
	else if ((t_hall_logic_f == t_hall_sensor_table[0]) && (t_hall_logic == t_hall_sensor_table[5]))
	{
		motor_setting.elec_angle_homing.offset = -motor_out.elec_angle + 60075;
		motor_setting.elec_angle_homing.done = 1;
	}
	else
	{
		low_level_ctrl_task.err_code = ERROR_ELEC_ANGLE_HOMING_FAILED;
	}

	hallObj.hall_logic_f = hallObj.hall_logic;
}

static void Elec_Angle_Calib()
{
	uint8_t t_hall_logic, t_hall_logic_f;
	uint8_t *t_hall_sensor_table;
	t_hall_logic        = hallObj.hall_logic;
	t_hall_logic_f      = hallObj.hall_logic_f;
	t_hall_sensor_table = motor_setting.commutation_set.hall_sensor_table;

	IOIF_ReadIncCnt(IOIF_TIM5, &inc25KhzObj);
	Cal_Elec_Angle();


	if      ((t_hall_logic_f == t_hall_sensor_table[0]) && (t_hall_logic == t_hall_sensor_table[1]))
	{
		motor_setting.elec_angle_homing.offset += -motor_out.elec_angle + 5461;
		motor_setting.commutation_set.elec_angle_reset_idx++;
	}
	else if ((t_hall_logic_f == t_hall_sensor_table[1]) && (t_hall_logic == t_hall_sensor_table[0]))
	{
		motor_setting.elec_angle_homing.offset += -motor_out.elec_angle + 5461;
		motor_setting.commutation_set.elec_angle_reset_idx++;
	}
	else if ((t_hall_logic_f == t_hall_sensor_table[1]) && (t_hall_logic == t_hall_sensor_table[2]))
	{
		motor_setting.elec_angle_homing.offset += -motor_out.elec_angle + 16384;
		motor_setting.commutation_set.elec_angle_reset_idx++;
	}
	else if ((t_hall_logic_f == t_hall_sensor_table[2]) && (t_hall_logic == t_hall_sensor_table[1]))
	{
		motor_setting.elec_angle_homing.offset += -motor_out.elec_angle + 16384;
		motor_setting.commutation_set.elec_angle_reset_idx++;
	}
	else if ((t_hall_logic_f == t_hall_sensor_table[2]) && (t_hall_logic == t_hall_sensor_table[3]))
	{
		motor_setting.elec_angle_homing.offset += -motor_out.elec_angle + 27307;
		motor_setting.commutation_set.elec_angle_reset_idx++;
	}
	else if ((t_hall_logic_f == t_hall_sensor_table[3]) && (t_hall_logic == t_hall_sensor_table[2]))
	{
		motor_setting.elec_angle_homing.offset += -motor_out.elec_angle + 27307;
		motor_setting.commutation_set.elec_angle_reset_idx++;
	}
	else if ((t_hall_logic_f == t_hall_sensor_table[3]) && (t_hall_logic == t_hall_sensor_table[4]))
	{
		motor_setting.elec_angle_homing.offset += -motor_out.elec_angle + 38229;
		motor_setting.commutation_set.elec_angle_reset_idx++;
	}
	else if ((t_hall_logic_f == t_hall_sensor_table[4]) && (t_hall_logic == t_hall_sensor_table[3]))
	{
		motor_setting.elec_angle_homing.offset += -motor_out.elec_angle + 38229;
		motor_setting.commutation_set.elec_angle_reset_idx++;
	}
	else if ((t_hall_logic_f == t_hall_sensor_table[4]) && (t_hall_logic == t_hall_sensor_table[5]))
	{
		motor_setting.elec_angle_homing.offset += -motor_out.elec_angle + 49152;
		motor_setting.commutation_set.elec_angle_reset_idx++;
	}
	else if ((t_hall_logic_f == t_hall_sensor_table[5]) && (t_hall_logic == t_hall_sensor_table[4]))
	{
		motor_setting.elec_angle_homing.offset += -motor_out.elec_angle + 49152;
		motor_setting.commutation_set.elec_angle_reset_idx++;
	}
	else if ((t_hall_logic_f == t_hall_sensor_table[5]) && (t_hall_logic == t_hall_sensor_table[0]))
	{
		motor_setting.elec_angle_homing.offset += -motor_out.elec_angle + 60075;
		motor_setting.commutation_set.elec_angle_reset_idx++;
	}
	else if ((t_hall_logic_f == t_hall_sensor_table[0]) && (t_hall_logic == t_hall_sensor_table[5]))
	{
		motor_setting.elec_angle_homing.offset += -motor_out.elec_angle + 60075;
		motor_setting.commutation_set.elec_angle_reset_idx++;
	}
	else
	{
		low_level_ctrl_task.err_code = ERROR_ELEC_ANGLE_HOMING_FAILED;
	}
}

static uint16_t Check_Elec_Angle_Offset(uint16_t elec_angle)
{
	uint8_t t_hall_logic, t_hall_logic_f;
	uint8_t *t_hall_sensor_table;
	t_hall_logic        = hallObj.hall_logic;
	t_hall_logic_f      = hallObj.hall_logic_f;
	t_hall_sensor_table = motor_setting.commutation_set.hall_sensor_table;

	uint16_t Difference = 0;

	if      ((t_hall_logic_f == t_hall_sensor_table[0]) && (t_hall_logic == t_hall_sensor_table[1]))
	{
		Difference = elec_angle - 5461;
	}
	else if ((t_hall_logic_f == t_hall_sensor_table[1]) && (t_hall_logic == t_hall_sensor_table[0]))
	{
		Difference = elec_angle - 5461;
	}
	else if ((t_hall_logic_f == t_hall_sensor_table[1]) && (t_hall_logic == t_hall_sensor_table[2]))
	{
		Difference = elec_angle - 16384;
	}
	else if ((t_hall_logic_f == t_hall_sensor_table[2]) && (t_hall_logic == t_hall_sensor_table[1]))
	{
		Difference = elec_angle - 16384;
	}
	else if ((t_hall_logic_f == t_hall_sensor_table[2]) && (t_hall_logic == t_hall_sensor_table[3]))
	{
		Difference = elec_angle - 27307;
	}
	else if ((t_hall_logic_f == t_hall_sensor_table[3]) && (t_hall_logic == t_hall_sensor_table[2]))
	{
		Difference = elec_angle - 27307;
	}
	else if ((t_hall_logic_f == t_hall_sensor_table[3]) && (t_hall_logic == t_hall_sensor_table[4]))
	{
		Difference = elec_angle - 38229;
	}
	else if ((t_hall_logic_f == t_hall_sensor_table[4]) && (t_hall_logic == t_hall_sensor_table[3]))
	{
		Difference = elec_angle - 38229;
	}
	else if ((t_hall_logic_f == t_hall_sensor_table[4]) && (t_hall_logic == t_hall_sensor_table[5]))
	{
		Difference = elec_angle - 49152;
	}
	else if ((t_hall_logic_f == t_hall_sensor_table[5]) && (t_hall_logic == t_hall_sensor_table[4]))
	{
		Difference = elec_angle - 49152;
	}
	else if ((t_hall_logic_f == t_hall_sensor_table[5]) && (t_hall_logic == t_hall_sensor_table[0]))
	{
		Difference = elec_angle - 60075;
	}
	else if ((t_hall_logic_f == t_hall_sensor_table[0]) && (t_hall_logic == t_hall_sensor_table[5]))
	{
		Difference = elec_angle - 60075;
	}
	else
	{
		low_level_ctrl_task.err_code = ERROR_ELEC_ANGLE_HOMING_FAILED;
	}


	return Difference;
}

static void Run_Abs_Encoder1_Homing()
{
	float    t_abs_encoder_m_angle_deg = 0;
	uint16_t t_abs_encoder_m_angle_cnt = 0;
	uint16_t t_m_angle_offset;
	uint16_t t_abs_e_angle;


	IOIF_GetPosVelDeg(IOIF_SPI1, &AbsObj1);

	if        (AbsObj1.location == IOIF_ABS_ENC_ACTUATOR_INPUT) {
		t_abs_encoder_m_angle_deg = AbsObj1.posDeg;
	}
	else{
		low_level_ctrl_task.err_code = ERROR_ELEC_ANGLE_HOMING_NO_ABS_ENCODER;
	}

	if (t_abs_encoder_m_angle_deg < 0) t_abs_encoder_m_angle_deg = t_abs_encoder_m_angle_deg + 360;
	t_abs_encoder_m_angle_cnt = t_abs_encoder_m_angle_deg * 182.0444444444445;

	int8_t t_sign;
	t_sign = motor_setting.commutation_set.ea_dir * motor_setting.commutation_set.ma_dir;
	t_m_angle_offset = t_sign * (t_abs_encoder_m_angle_cnt - motor_setting.commutation_set.abs_encoder_offset);
	// get (remainder of (t_m_angle_offset) devide to (65536/P)) * P
	t_abs_e_angle    = (t_m_angle_offset - ((motor_properties.pole_pair*t_m_angle_offset)>>16)*(65536/motor_properties.pole_pair))*motor_properties.pole_pair;

	IOIF_ReadIncCnt(IOIF_TIM5, &inc25KhzObj);
	Cal_Elec_Angle();

	motor_setting.elec_angle_homing.offset = -motor_out.elec_angle + t_abs_e_angle;
	motor_setting.elec_angle_homing.done = 1;
}




static void Run_Abs_Encoder2_Homing()
{

	float    t_abs_encoder_m_angle_deg = 0;
	uint16_t t_abs_encoder_m_angle_cnt = 0;
	uint16_t t_m_angle_offset;
	uint16_t t_abs_e_angle;

	IOIF_GetPosVelDeg(IOIF_SPI3, &AbsObj2);

	if        (AbsObj2.location == IOIF_ABS_ENC_ACTUATOR_INPUT) {
		t_abs_encoder_m_angle_deg = AbsObj2.posDeg;
	}
	else{
		low_level_ctrl_task.err_code = ERROR_ELEC_ANGLE_HOMING_NO_ABS_ENCODER;
	}

	if (t_abs_encoder_m_angle_deg < 0) t_abs_encoder_m_angle_deg = t_abs_encoder_m_angle_deg + 360;
	t_abs_encoder_m_angle_cnt = t_abs_encoder_m_angle_deg * 182.0444444444445;

	int8_t t_sign;
	t_sign = motor_setting.commutation_set.ea_dir * motor_setting.commutation_set.ma_dir;
	t_m_angle_offset = t_sign * (t_abs_encoder_m_angle_cnt - motor_setting.commutation_set.abs_encoder_offset);
	// get (remainder of (t_m_angle_offset) devide to (65536/P)) * P
	t_abs_e_angle    = (t_m_angle_offset - ((motor_properties.pole_pair*t_m_angle_offset)>>16)*(65536/motor_properties.pole_pair))*motor_properties.pole_pair;


	IOIF_ReadIncCnt(IOIF_TIM5, &inc25KhzObj);
	Cal_Elec_Angle();

	motor_setting.elec_angle_homing.offset = -motor_out.elec_angle + t_abs_e_angle;
	motor_setting.elec_angle_homing.done = 1;

	/*
	if (t_abs_encoder_m_angle < 0) t_abs_encoder_m_angle = t_abs_encoder_m_angle + 360;
	t_abs_encoder_e_angle = t_abs_encoder_m_angle * (65536/360) * motor_properties.pole_pair;

	t_offset = motor_setting.commutation_set.ea_dir * (t_abs_encoder_e_angle - motor_setting.commutation_set.abs_encoder_offset);

	IOIF_ReadIncCnt(IOIF_TIM5, &inc25KhzObj);
	Cal_Elec_Angle();

	motor_setting.elec_angle_homing.offset = -motor_out.elec_angle + t_offset;
	motor_setting.elec_angle_homing.done = 1;
	*/
}

static int Ent_Elec_Angle_Homing()
{
	motor_setting.elec_angle_homing.forced_homing_duration = 25000;

	float t_peak_voltage;
	if (!isnan(motor_properties.R) & !isnan(motor_setting.peakCurr_limit)) // if R, peakCurr is not nan
		t_peak_voltage = motor_setting.peakCurr_limit * motor_properties.R;
	else                                                                   // if R or peakCurr is nan, default duty = 5V
		t_peak_voltage = 2;

//	motor_setting.elec_angle_homing.forced_homing_duty     = t_peak_voltage * VBUS2DUTY_RATIO;
	motor_setting.elec_angle_homing.forced_homing_duty     = 2 * VBUS2DUTY_RATIO;
	motor_setting.elec_angle_homing.forced_homing_cnt      = 0;

	if (motor_setting.sensor_setting.hall_sensor_usage == 1) Read_Hall_Sensors(&hallObj);

	return 0;
}

static int Run_Elec_Angle_Homing()
{
	if (motor_setting.elec_angle_homing.done == 0)  // if electrical angle homing is not done
	{
		if (motor_setting.commutation_set.done == 0)
		{
			// error_handler: You must do commutation_setting on GUI
		}
		else
		{
			if      (motor_setting.sensor_setting.e_angle_homing_sensor == e_EHoming_Sensor_Forced)          Run_Forced_Homing();
			else if (motor_setting.sensor_setting.e_angle_homing_sensor == e_EHoming_Sensor_Hall)            Run_Hall_Homing();
			else if (motor_setting.sensor_setting.e_angle_homing_sensor == e_EHoming_Sensor_Abs_Encoder1)    Run_Abs_Encoder1_Homing();
			else if (motor_setting.sensor_setting.e_angle_homing_sensor == e_EHoming_Sensor_Abs_Encoder2)    Run_Abs_Encoder2_Homing();

			if (motor_setting.elec_angle_homing.done == 1)		BSP_WriteGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_13, BSP_GPIO_PIN_RESET);
		}
	}

	return 0;
}

static void Ent_Get_Temperature()
{
	temp_sense.stator_raw =      temp_sense.stator_initial;
	temp_sense.stator_raw_pre =  temp_sense.stator_initial;
	temp_sense.stator_filtered = temp_sense.stator_initial;

	temp_sense.max_temp = 100;
}

static void Run_Get_Temperature()
{
	float stator_voltage;
	float stator_resistance;

	float cutoff_stator = 0.1;
	float alpha_stator;

	stator_voltage = tempbuffer[0] * 0.00005035477226;
	stator_resistance = 3.3 * 51000 / stator_voltage - 51000;
	temp_sense.stator_raw = 1 / (0.00335401643468 + 0.00023320895522388*log(stator_resistance*0.00002)) - 273.15;

	if ((temp_sense.stator_raw - temp_sense.stator_raw_pre > 30) || (temp_sense.stator_raw - temp_sense.stator_raw_pre < -30)) {
		temp_sense.stator_raw = temp_sense.stator_raw_pre;
	}

	alpha_stator = 2*3.1415926535*0.00004*cutoff_stator / (2*3.1415926535*0.00004*cutoff_stator + 1);
	temp_sense.stator_filtered = (1-alpha_stator)*temp_sense.stator_filtered + alpha_stator*temp_sense.stator_raw;

	temp_sense.stator_raw_pre = temp_sense.stator_raw;
}

