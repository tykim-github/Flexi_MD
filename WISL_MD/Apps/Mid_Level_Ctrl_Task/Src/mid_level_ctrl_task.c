/**
 * @file mid_level_ctrl_task.c
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics FW Team
 * @brief 
 * @ref 
 */

// TODO : refactoring!!
#include "mid_level_ctrl_task.h"

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

TaskStruct 		        mid_level_ctrl_task;
float mid_ctrl_saturation;
float k_emf=0.15;
float saturation;
float vel_limit;


uint32_t                mid_level_loop_cnt;
PIDObject 		        posCtrl;
PIDObject 		        velCtrl;
PIDObject 		        forceCtrl;
ImpedanceReductionCtrl 	IRC;
ImpedanceCtrl           impedanceCtrl;
ProportionalCtrl		proportionalCtrl;
DOBObject	            posDOB;
FFObject	            posFF;
FCObject                FrictionCompObj;
VirtualSpringDamper     VSD;

VelocityEstObject       veObj;

IOIF_IncEnc_t       inc1KhzObj;
IOIF_AbsEnc_t       AbsObj1;
IOIF_AbsEnc_t       AbsObj2;

MidLevelState 	    mid_level_state;

uint8_t          mid_level_triggered_msg_stop;

//////////////////      Flexi-SEA          ///////////////////////////////
//////////////////////////////////////////////////////////////////////////
static MidLevelState spring_state;
PIDObject 		        posCtrlAnk;
LinearizeStiffness	LS;
RefTanh				refTanh;
RefAnk				refAnk;
Risk_flexi			RISK_Flexi;
WIDM_GaitData		widmGaitDataObj2;
LoadCell			load_cell;
FlexiAnkle			flexi_ankle;
AnkleComp			ankle_comp;
uint16_t step = 0;
float shift_test=0;
static float gaitphase;
static uint16_t* LCbuffer = {0};


pMMGSense           pMMG_sense;
///////////////////////////////////////////////////////
static float t_gaitphase = 0;
/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */

static PeriodicSignal 	cur_periodic_sig;
static PeriodicSignal 	vel_periodic_sig;
static PeriodicSignal 	pos_periodic_sig;

static Backlash_Test    backlash_test;
static System_ID_SBS    sys_id_sbs;
static TVCF_Ver         TVCF_ver;
static Trapezoidal_ID_Obj  TrapeIDObj;

static P_Vector_Decoder pvectorObj;
static F_Vector_Decoder fvectorObj;
static Cos3_Vector_Decoder cos3vectorObj;


float midTimeElap;
static uint16_t RT_Broken;
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

/* ------------------- KALMAN FILTER ------------------- */
static void Init_Kalman_Filter(KFObject * t_KF_obj, float t_kf_A, float t_kf_B, float t_kf_Q, float t_kf_R);
static int Run_Kalman_Filter(KFObject * t_KF_obj, float t_u, float t_y);

/* ------------------- GET POS & VEL ------------------- */
static void Init_Position_Velocity();
static void Get_Position_Velocity(IOIF_IncEnc_t* t_inc_enc, MidLevelState* t_mid_level_state);

/* ------------------- INIT TORQUE MODE ------------------- */
static void Init_Controller_Overall_Gain(float init_gain);
static void Init_F_Vector_Modes();

/* ------------------- SDO CALLBACK ------------------- */
static void Set_IRC_Numerator_Length(MsgSDOargs* req, MsgSDOargs* res);
static void Set_IRC_Denominator_Length(MsgSDOargs* req, MsgSDOargs* res);
static void Set_IRC_Numerator(MsgSDOargs* req, MsgSDOargs* res);
static void Set_IRC_Denominator(MsgSDOargs* req, MsgSDOargs* res);
static void Set_IRC_Saturation(MsgSDOargs* req, MsgSDOargs* res);

static void Set_Vel_Ctrl_BW(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_Vel_Ctrl_PGain(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_Vel_Ctrl_IGain(MsgSDOargs* t_req, MsgSDOargs* t_res);

static void Set_Pos_Ctrl_Input_Penalty(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_Pos_Ctrl_PGain(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_Pos_Ctrl_DGain(MsgSDOargs* t_req, MsgSDOargs* t_res);

static void Set_Mid_Ctrl_Saturation(MsgSDOargs* t_req, MsgSDOargs* t_res);

static void Set_IOIF_IncEnc_t_Offset(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_IOIF_AbsEnc1_t_Offset(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_IOIF_AbsEnc1_t_Sign(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_IOIF_AbsEnc2_t_Offset(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_IOIF_AbsEnc2_t_Sign(MsgSDOargs* t_req, MsgSDOargs* t_res);

static void Set_DOB_Q_BW(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_DOB_GQ_Num(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_DOB_GQ_Den(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_DOB_Q_Num(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_DOB_Q_Den(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_DOB_Saturation(MsgSDOargs* t_req, MsgSDOargs* t_res);

static void Set_Current_Periodic_Sig_Info(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_Velocity_Periodic_Sig_Info(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_Position_Periodic_Sig_Info(MsgSDOargs* t_req, MsgSDOargs* t_res);

static void Set_VSD_Stiffness(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_VSD_Damper(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_VSD_Damped_Range(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_VSD_Stiff_Range(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_VSD_Upper_Limit(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_VSD_Lower_Limit(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_VSD_Saturation(MsgSDOargs* t_req, MsgSDOargs* t_res);

static void Set_Feedforward_Num(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_Feedforward_Den(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_Velocity_Estimator(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_Velocity_Estimator_LeadLag(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_1khz_Enc_Resolution(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_System_ID_SBS_Info(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_System_ID_Verification_Mag(MsgSDOargs* t_req, MsgSDOargs* t_res);

static void Set_C_Vector_FF_Gain(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_C_Vector_PD_Gain(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_C_Vector_IC_Gain(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_C_Vector_DOB_Gain(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_C_Vector_IRC_Gain(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_C_Vector_FC_Gain(MsgSDOargs* t_req, MsgSDOargs* t_res);

static void Set_P_Vector_Yd(MsgSDOargs* req, MsgSDOargs* res);
static void Set_P_Vector_L(MsgSDOargs* req, MsgSDOargs* res);
static void Set_P_Vector_S0(MsgSDOargs* req, MsgSDOargs* res);
static void Set_P_Vector_Sd(MsgSDOargs* req, MsgSDOargs* res);

static void Set_F_Vector_ModeIDX(MsgSDOargs* req, MsgSDOargs* res);
static void Set_F_Vector_TauMax(MsgSDOargs* req, MsgSDOargs* res);
static void Set_F_Vector_Delay(MsgSDOargs* req, MsgSDOargs* res);

static void Set_I_Vector_Epsilon(MsgSDOargs* req, MsgSDOargs* res);
static void Set_I_Vector_Kp(MsgSDOargs* req, MsgSDOargs* res);
static void Set_I_Vector_Kd(MsgSDOargs* req, MsgSDOargs* res);
static void Set_I_Vector_Lambda(MsgSDOargs* req, MsgSDOargs* res);
static void Set_I_Vector_Duration(MsgSDOargs* req, MsgSDOargs* res);
static void Set_I_Vector_Kp_Max(MsgSDOargs* req, MsgSDOargs* res);
static void Set_I_Vector_Kd_Max(MsgSDOargs* req, MsgSDOargs* res);
static void Set_I_Vector_Option(MsgSDOargs* req, MsgSDOargs* res);

static void Set_Desired_Mech_Angle(MsgSDOargs* req, MsgSDOargs* res);
static void Set_IOIF_AbsEnc1_t_Location(MsgSDOargs* req, MsgSDOargs* res);
static void Set_IOIF_AbsEnc2_t_Location(MsgSDOargs* req, MsgSDOargs* res);
static void Set_IOIF_AbsEnc1_t_Module_ID(MsgSDOargs* req, MsgSDOargs* res);
static void Set_IOIF_AbsEnc2_t_Module_ID(MsgSDOargs* req, MsgSDOargs* res);

static void Set_Overall_Gain_Transition_Time(MsgSDOargs* req, MsgSDOargs* res);

static void Set_Freeze(MsgSDOargs* req, MsgSDOargs* res);
static void Set_Position_Reference_Small_Offset(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_TVCF_Verification_Info(MsgSDOargs* t_req, MsgSDOargs* t_res);

static void Set_Trape_ID_Info(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_FrictionComp_Param(MsgSDOargs* t_req, MsgSDOargs* t_res);

/* ------------------- ROUTINE ------------------- */
static int Ent_Position_Ctrl();
static int Run_Position_Ctrl();

static int Ent_Velocity_Ctrl();
static int Run_Velocity_Ctrl();

static int Ent_Impedance_Reduction();
static int Run_Impedance_Reduction();

static int Ent_Compressional_SpringDamper();
static int Run_Compressional_SpringDamper();
static int Ext_Compressional_SpringDamper();

static int Run_Backlash_Test();

static int Ent_Disturbance_Obs();
static int Run_Disturbance_Obs();

static int Ent_Mech_SystemID_SBS();
static int Run_Mech_SystemID_SBS();
static int Ext_Mech_SystemID_SBS();

static int Ent_P_Vector_Decoder();
static int Run_P_Vector_Decoder();
static int Ext_P_Vector_Decoder();

static int Run_F_Vector_Decoder();
static int Ext_F_Vector_Decoder();

static int Ent_Feedforward_Filter();
static int Run_Feedforward_Filter();
static int Ext_Feedforward_Filter();

static int Run_System_ID_Verify();

static void error_filter2(ImpedanceCtrl *t_impedanceCtrl);
static int Ent_Corridor_Impedance_Control();
static int Run_Corridor_Impedance_Control();

static int Ent_Generate_Current_Sine();
static int Run_Generate_Current_Sine();

static int Ent_Generate_Velocity_Sine();
static int Run_Generate_Velocity_Sine();

static int Ent_Generate_Position_Sine();
static int Run_Generate_Position_Sine();

static int Ent_Generate_Current_Tanh();
static int Run_Generate_Current_Tanh();

static int Ent_Generate_Current_Rec();
static int Run_Generate_Current_Rec();

static int Ent_Generate_Velocity_Rectangle();
static int Run_Generate_Velocity_Rectangle();

static int Ent_Generate_Position_Rectangle();
static int Run_Generate_Position_Rectangle();

static int Ent_Generate_Position_Trape();
static int Run_Generate_Position_Trape();

static int Run_Send_IOIF_Hall_t_Values_to_GUI();
static int Run_Send_IOIF_IncEnc_t_Values_to_GUI();
static int Run_Send_IOIF_AbsEnc1_t_Values_to_GUI();
static int Run_Send_IOIF_AbsEnc2_t_Values_to_GUI();

static int Ent_Generate_Position_Sine_without_Offset();
static int Run_Generate_Position_Sine_without_Offset();

static int Ent_TVCF_Verification();
static int Run_TVCF_Verification();
static int Ext_TVCF_Verification();

static int Run_Cos3_Ref_Decoder();

static int Ent_Trape_SystemID();
static int Run_Trape_SystemID();
static int Ext_Trape_SystemID();

static int Ent_Mech_SystemID_SBS_Position();
static int Run_Mech_SystemID_SBS_Position();
static int Ext_Mech_SystemID_SBS_Position();

static int Ent_Friction_Compensation();
static int Run_Friction_Compensation();
static int Ext_Friction_Compensation();

////////////////////// Flexi-SEA        //////////////////////
//////////////////////////////////////////////////////////////
static void Set_Position_Tanh_Sig_Info(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_Ankle_Onoff(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_Ankle_REF_Sig_Info(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_Ankle_REF_Sig_Info2(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_Ankle_REF_Sig_Info3(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_Risk_Param(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_Init_Torque();
static void Set_Phase_Shift(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_ImpedanceCtrl_Info(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void Set_ProportionalCtrl_Info(MsgSDOargs* t_req, MsgSDOargs* t_res);
////////////////////////////////////////////////////////////////
static int Ent_Linearize_Stiffness();
static int Run_Linearize_Stiffness();
static int Ext_Linearize_Stiffness();
static int Ent_Generate_Position_Tanh();
static int Run_Generate_Position_Tanh();
static int Ext_Generate_Position_Tanh();
static int Ent_Generate_Position_Ankle();
static int Run_Generate_Position_Ankle();
static int Ext_Generate_Position_Ankle();
static int Ent_Generate_Position_Ankle_Periodic();
static int Run_Generate_Position_Ankle_Periodic();
static int Ext_Generate_Position_Ankle_Periodic();


static int Ent_Risk_Manage();
static int Run_Risk_Manage();

static int Ent_Position_Ctrl_Ankle();
static int Run_Position_Ctrl_Ankle();
static float Gait_Phase_Shift(float gp, float s);
static void Ent_Get_Loadcell();
static void Run_Get_Loadcell();
static void Ent_T2F();
static void T2F();
float Generate_Ref_Dorsi(float amp_p, float amp_t,float gc_i, float gc_p, float gc_t, uint16_t T_gait, uint32_t t_k, float t_T, int32_t t_phase_lead_index);
static int Ent_Ankle_Compensator();
static int Run_Ankle_Compensator();
static int Ent_Proportional_Assist();
static int Run_Proportional_Assist();

//////////////////////////////////////////////////////////////////////////////////////
/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

MSG_COMMON_SDO_CALLBACK(mid_level_ctrl_task)

void InitMidLvCtrl(void)
{
   	Init_Task(&mid_level_ctrl_task);

   	Init_Controller_Overall_Gain(1);
   	Init_F_Vector_Modes();
   	////////////////  Flexi-Ankle  ///////////////////////////////////
	if(IOIF_StartADCDMA(IOIF_ADC3, &LCbuffer, IOIF_ADC3_BUFFER_LENGTH)){
		//TODO: Error Process
	}
    /**/
   	//IOIF_InitAbsEnc(IOIF_SPI1, 1, &AbsObj1, 0x00007FFF, MID_LEVEL_CONTROL_FREQUENCY);

//	IOIF_InitAbsEnc(IOIF_SPI1, 1, &AbsObj1, MID_LEVEL_CONTROL_FREQUENCY);
//	IOIF_InitAbsEnc(IOIF_SPI3, 2, &AbsObj2, MID_LEVEL_CONTROL_FREQUENCY);
	IOIF_InitAbsEnc(IOIF_SPI1, 1, &AbsObj1, MID_LEVEL_CONTROL_FREQUENCY);
//	IOIF_InitAbsEnc(IOIF_SPI3, 2, &AbsObj2, MID_LEVEL_CONTROL_FREQUENCY);
	AbsObj1.del_threshold=1;
	AbsObj2.del_threshold=1;
	Init_Kalman_Filter(&veObj.kf_obj, -motor_properties.a, motor_properties.b, 1, 5);

	/* State Definition*/
	TASK_CREATE_STATE(&mid_level_ctrl_task, e_State_Off,     NULL,     			StateOff_Run,           NULL, 				true);
	TASK_CREATE_STATE(&mid_level_ctrl_task, e_State_Standby, StateStandby_Ent, 	StateStandby_Run,		StateStandby_Ext, 	false);
	TASK_CREATE_STATE(&mid_level_ctrl_task, e_State_Enable,  StateEnable_Ent, 	StateEnable_Run, 		StateEnable_Ext, 	false);
	TASK_CREATE_STATE(&mid_level_ctrl_task, e_State_Error,   NULL,             	StateError_Run,  		NULL, 				false);

	/* Routine Definition*/
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_SYS_ID_SBS,  		            Ent_Mech_SystemID_SBS,      	 Run_Mech_SystemID_SBS,                 Ext_Mech_SystemID_SBS);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_IRC, 				            Ent_Impedance_Reduction,		 Run_Impedance_Reduction,		        NULL);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_IMPEDANCE_CONTROL,  	            Ent_Corridor_Impedance_Control,  Run_Corridor_Impedance_Control,        NULL);
	//TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_IMPEDANCE_CONTROL, 	        Ent_Impedance_Control,			 Run_Impedance_Control,			        NULL);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_POSITION_REC_REF,	            Ent_Generate_Position_Rectangle, Run_Generate_Position_Rectangle,       NULL);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_POSITION_SINE_REF,  	            Ent_Generate_Position_Sine, 	 Run_Generate_Position_Sine, 	        NULL);
//	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_POSITION_TRAPE_REF, 	            Ent_Generate_Position_Trape, 	 Run_Generate_Position_Trape, 	        NULL);

	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_POSITION_CTRL, 		            Ent_Position_Ctrl, 				 Run_Position_Ctrl, 			        NULL);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_VELOCITY_CTRL, 		            Ent_Velocity_Ctrl, 				 Run_Velocity_Ctrl, 				    NULL);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_VELOCITY_SINE_REF,  	            Ent_Generate_Velocity_Sine, 	 Run_Generate_Velocity_Sine, 	        NULL);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_VELOCITY_REC_REF,	            Ent_Generate_Velocity_Rectangle, Run_Generate_Velocity_Rectangle,       NULL);

	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_CURRENT_SINE_REF,  	            Ent_Generate_Current_Sine, 	     Run_Generate_Current_Sine, 	        NULL);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_CURRENT_REC_REF,	                Ent_Generate_Current_Rec,        Run_Generate_Current_Rec,              NULL);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_CURRENT_TANH_REF,	            Ent_Generate_Current_Tanh,       Run_Generate_Current_Tanh,             NULL);

	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_GET_HALL_SENSOR_VALUE,           NULL,				 			 Run_Send_IOIF_Hall_t_Values_to_GUI,    NULL);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_GET_INCENCODER_VALUE,            NULL,				 			 Run_Send_IOIF_IncEnc_t_Values_to_GUI,  NULL);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_GET_ABSENCODER1_VALUE, 			NULL,				 			 Run_Send_IOIF_AbsEnc1_t_Values_to_GUI, NULL);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_GET_ABSENCODER2_VALUE, 			NULL,				 			 Run_Send_IOIF_AbsEnc2_t_Values_to_GUI, NULL);

	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_COMPRESSIONAL_VSD,				Ent_Compressional_SpringDamper,	 Run_Compressional_SpringDamper,	Ext_Compressional_SpringDamper);

	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_BACKLASH_TEST,        			NULL,   	                     Run_Backlash_Test, 			    NULL);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_DISTURBANCE_OBS,      			Ent_Disturbance_Obs,             Run_Disturbance_Obs,          	    NULL);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_P_VECTOR_DECODER,  		        Ent_P_Vector_Decoder,            Run_P_Vector_Decoder,  	        Ext_P_Vector_Decoder);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_F_VECTOR_DECODER,   			    NULL,                            Run_F_Vector_Decoder,              Ext_F_Vector_Decoder);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_FEEDFORWARD_FILTER,  	        Ent_Feedforward_Filter,          Run_Feedforward_Filter,       	    Ext_Feedforward_Filter);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_SYS_ID_SBS_VERIFY,  	            NULL,         					 Run_System_ID_Verify,       	    NULL);

//	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_COS3_REF_DECODER,  	            NULL, 			                 Run_Cos3_Ref_Decoder,       	    NULL);
//
//	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_POSITION_SINE_REF_WITHOUT_OFFSET,  	Ent_Generate_Position_Sine_without_Offset,        Run_Generate_Position_Sine_without_Offset,       	    NULL);
//
//	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_TVCF_VERIFICATION,  		        Ent_TVCF_Verification,      	 Run_TVCF_Verification,             Ext_TVCF_Verification);
//	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_TRAPEZOIDAL_ID,  		        Ent_Trape_SystemID,      	     Run_Trape_SystemID,                Ext_Trape_SystemID);
//	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_SYS_ID_SBS_POSITION,  		    Ent_Mech_SystemID_SBS_Position,  Run_Mech_SystemID_SBS_Position,    Ext_Mech_SystemID_SBS_Position);
//
//	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_FRICTION_COMPENSATION,  		    Ent_Friction_Compensation,       Run_Friction_Compensation,         Ext_Friction_Compensation);

	////////////////////////////////////	 Flexi-SEA 		 ////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_LINEARIZE_STIFFNESS,  	        Ent_Linearize_Stiffness,    	 Run_Linearize_Stiffness,       	Ext_Linearize_Stiffness);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_POSITION_TANH_REF,  	        	Ent_Generate_Position_Tanh,    	 Run_Generate_Position_Tanh,       	Ext_Generate_Position_Tanh);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_ANKLE_REF,  	        			Ent_Generate_Position_Ankle,     Run_Generate_Position_Ankle,       Ext_Generate_Position_Ankle);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_RISK_MANAGEMENT,  	        	Ent_Risk_Manage,     			 Run_Risk_Manage,       			NULL);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_POSITION_CTRL_ANKLE,				Ent_Position_Ctrl_Ankle,		 Run_Position_Ctrl_Ankle,			NULL);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_ANKLE_REF_PERIODIC,				Ent_Generate_Position_Ankle_Periodic,		 Run_Generate_Position_Ankle_Periodic,       Ext_Generate_Position_Ankle_Periodic);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_ANKLE_COMPENSATOR,				Ent_Ankle_Compensator,		 Run_Ankle_Compensator,       NULL);
	TASK_CREATE_ROUTINE(&mid_level_ctrl_task,  ROUTINE_ID_MIDLEVEL_PROPORTIONAL_ASSIST,				Ent_Proportional_Assist,		 Run_Proportional_Assist,       NULL);
	/////////////////////////////////////////////////////////////////////////////////////////////////////////


	/* DOD Definition*/
	// DOD
	Create_DOD(TASK_ID_MIDLEVEL);

	// PDO
	Create_PDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_LOOP_CNT,						e_UInt32,   1, &mid_level_loop_cnt);
	Create_PDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_REF_POSITION, 					e_Float32,  1, &posCtrl.ref);
	Create_PDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_REF_VELOCITY, 					e_Float32,  1, &velCtrl.ref);
	Create_PDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_ACTUAL_POSITION, 				e_Float32,  1, &mid_level_state.position); // rad
	Create_PDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_ACTUAL_VELOCITY_RAW, 			e_Float32,  1, &mid_level_state.velocity_raw);
	Create_PDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_SYSTEM_ID_SBS_FREQ,			e_Float32,  1, &sys_id_sbs.current_f);

	Create_PDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_IMP_CTRL_INPUT,		 		e_Float32,  1, &impedanceCtrl.control_input);
	Create_PDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_POS_PID_CTRL_INPUT,			e_Float32,  1, &posCtrl.control_input);
	Create_PDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_VEL_PID_CTRL_INPUT, 			e_Float32,  1, &velCtrl.control_input);
	Create_PDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_VSD_INPUT,					 	e_Float32, 	1, &VSD.control_input);



	Create_PDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_ABSENCODER1_POSITION,  		e_Float32,  1, &AbsObj1.posDeg);
	Create_PDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_ABSENCODER2_POSITION,  		e_Float32,  1, &AbsObj2.posDeg);
	Create_PDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_DOB_DISTURABNCE, 				e_Float32,  1, &posDOB.disturbance);
	Create_PDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_DOB_INPUT, 					e_Float32,  1, &posDOB.control_input);
	Create_PDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_FF_INPUT, 						e_Float32,  1, &posFF.control_input);
	Create_PDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_VELOCITY_ESTIMATED, 			e_Float32,  1, &mid_level_state.velocity_final);

	Create_PDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_IMP_EPSILON, 			        e_Float32,  1, &impedanceCtrl.epsilon);
	Create_PDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_IMP_KP, 	 			        e_Float32,  1, &impedanceCtrl.Kp);
	Create_PDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_IMP_KD,    			        e_Float32,  1, &impedanceCtrl.Kd );
	Create_PDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_IMP_LAMDA,   		         	e_Float32,  1, &impedanceCtrl.lambda);
	Create_PDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_REF_POSITION1,    			    e_Float32,  1, &posCtrl.ref1 );
	Create_PDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_REF_POSITION2,   		        e_Float32,  1, &posCtrl.ref2);
	Create_PDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_TOTAL_REF_POSITION,            e_Float32,  1, &posCtrl.total_ref);

	Create_PDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_TVCF_VER_FREQUENCY,			e_Float32,  1, &TVCF_ver.current_f);

	Create_PDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_TRAPE_ID_DONE,	        		e_UInt8,    1, &TrapeIDObj.Done);
	Create_PDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_SBS_ID_DONE,	        		e_UInt8,    1, &sys_id_sbs.done);

	////////////////////////////////////	 Flexi-SEA 		 ////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
//	Create_PDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_SPRING_INIT,   		        e_Float32,  1, &spring_state.initial_pos);
	Create_PDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_LOADCELL_FORCE,   		        e_Float32,  1, &load_cell.loadcell_filtered);
	Create_PDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_LS_CTRL_INPUT,   		        e_Float32,  1, &motor_in.mid_ctrl_input);
	Create_PDO(TASK_ID_WIDM,	 PDO_ID_WIDM_GAIT_PHASE, 	 	 				e_Float32,  1, &widmGaitDataObj2.gaitPhase);
	Create_PDO(TASK_ID_WIDM,	 PDO_ID_WIDM_GAIT_PERIOD, 	 	 				e_UInt16,   1, &widmGaitDataObj2.gaitPeriod);
	Create_PDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_REF_IMPEDANCE, 				e_Float32,  1, &impedanceCtrl.ref);
	Create_PDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_AC_CTRL_INPUT, 				e_Float32,  1, &ankle_comp.control_input);
	Create_PDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_LOADCELL_TORQUE, 				e_Float32,  1, &load_cell.torque);

	Create_PDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_PMMG1, 			         	e_Float32,  1, &pMMG_sense.pMMG1);
	Create_PDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_PMMG2, 			         	e_Float32,  1, &pMMG_sense.pMMG2);
	Create_PDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_PMMG3, 			         	e_Float32,  1, &pMMG_sense.pMMG3);
	Create_PDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_F_VECTOR_INPUT,	         	e_Float32,  1, &pMMG_sense.pMMG4);
	/////////////////////////////////////////////////////////////////////////////////////////////////////////

	//Create_PDO(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_RISK,   		                e_UInt32,   1, &packed_risk);

	// SDO
	MSG_COMMON_SDO_CREATE(TASK_ID_MIDLEVEL)

	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_IRC_NUMERATOR_LENGTH,     		e_UInt8,  	Set_IRC_Numerator_Length);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_IRC_DENOMINATOR_LENGTH,	    e_UInt8,  	Set_IRC_Denominator_Length);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_IRC_NUMERATOR,	       			e_Float32,  Set_IRC_Numerator);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_IRC_DENOMINATOR,	       		e_Float32,  Set_IRC_Denominator);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_IRC_SATURATION,	       		e_Float32,  Set_IRC_Saturation);

    Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_CURRENT_PERIODIC_SIG_INFO,	    e_Float32, 	Set_Current_Periodic_Sig_Info);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_VELOCITY_PERIODIC_SIG_INFO,	e_Float32, 	Set_Velocity_Periodic_Sig_Info);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_POSITION_PERIODIC_SIG_INFO,	e_Float32, 	Set_Position_Periodic_Sig_Info);

	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_DOB_Q_BW,	       				e_Float32,  Set_DOB_Q_BW);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_DOB_GQ_NUM,	       			e_Float32,  Set_DOB_GQ_Num);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_DOB_GQ_DEN,	       			e_Float32,  Set_DOB_GQ_Den);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_DOB_Q_NUM,	       				e_Float32,  Set_DOB_Q_Num);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_DOB_Q_DEN,	       				e_Float32,  Set_DOB_Q_Den);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_DOB_SATURATION,	       		e_Float32,  Set_DOB_Saturation);

	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_VELOCITY_CTRL_BW, 				e_Float32, 	Set_Vel_Ctrl_BW);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_VELOCITY_CTRL_P_GAIN, 			e_Float32, 	Set_Vel_Ctrl_PGain);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_VELOCITY_CTRL_I_GAIN, 			e_Float32, 	Set_Vel_Ctrl_IGain);

	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_POSITION_CTRL_INPUT_PENALTY,	e_Float32, 	Set_Pos_Ctrl_Input_Penalty);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_POSITION_CTRL_P_GAIN, 			e_Float32, 	Set_Pos_Ctrl_PGain);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_POSITION_CTRL_D_GAIN, 			e_Float32, 	Set_Pos_Ctrl_DGain);

	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_MID_CTRL_SATURATION, 			e_Float32, 	Set_Mid_Ctrl_Saturation);

	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_INCENCODER_SET_OFFSET, 		e_UInt8, 	Set_IOIF_IncEnc_t_Offset);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_ABSENCODER1_SET_OFFSET,        e_UInt8, 	Set_IOIF_AbsEnc1_t_Offset);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_ABSENCODER1_CHANGE_DIRECTION,	e_UInt8, 	Set_IOIF_AbsEnc1_t_Sign);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_ABSENCODER2_SET_OFFSET,        e_UInt8, 	Set_IOIF_AbsEnc2_t_Offset);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_ABSENCODER2_CHANGE_DIRECTION,	e_UInt8, 	Set_IOIF_AbsEnc2_t_Sign);

	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_VSD_STIFFNESS,					e_Float32, 	Set_VSD_Stiffness);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_VSD_DAMPER,					e_Float32, 	Set_VSD_Damper);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_VSD_DAMPED_RANGE,				e_Float32, 	Set_VSD_Damped_Range);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_VSD_STIFF_RANGE,				e_Float32, 	Set_VSD_Stiff_Range);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_VSD_UPPER_LIMIT,			e_Float32, 	Set_VSD_Upper_Limit);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_VSD_LOWER_LIMIT,			e_Float32, 	Set_VSD_Lower_Limit);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_VSD_SATURATION,				e_Float32, 	Set_VSD_Saturation);

	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_FEEDFORWARD_NUM,				e_Float32,  Set_Feedforward_Num);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_FEEDFORWARD_DEN,				e_Float32,  Set_Feedforward_Den);

	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_VELOCITY_ESTIMATOR,			e_UInt8,  	Set_Velocity_Estimator);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_VELOCITY_ESTIMATOR_LEAD_LAG,	e_Float32,  Set_Velocity_Estimator_LeadLag);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_ENCODER_RESOLUTION,			e_UInt32,  	Set_1khz_Enc_Resolution);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SYSTEM_ID_SBS_INFO,			e_Float32,  Set_System_ID_SBS_Info);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SYSTEM_ID_VERIFICATION_MAG,	e_Float32,  Set_System_ID_Verification_Mag);

	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_C_VECTOR_FF_GAIN,	            e_UInt8,    Set_C_Vector_FF_Gain);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_C_VECTOR_PD_GAIN,	            e_UInt8,    Set_C_Vector_PD_Gain);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_C_VECTOR_IC_GAIN,	            e_UInt8,    Set_C_Vector_IC_Gain);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_C_VECTOR_DOB_GAIN,	            e_UInt8,    Set_C_Vector_DOB_Gain);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_C_VECTOR_IRC_GAIN,	            e_UInt8,    Set_C_Vector_IRC_Gain);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_C_VECTOR_FC_GAIN,	            e_UInt8,    Set_C_Vector_FC_Gain);

	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_P_VECTOR_YD,	                e_Int16,    Set_P_Vector_Yd);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_P_VECTOR_L, 	                e_UInt16,   Set_P_Vector_L);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_P_VECTOR_S0,	                e_UInt8,    Set_P_Vector_S0);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_P_VECTOR_SD,	                e_UInt8,    Set_P_Vector_Sd);

	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_F_VECTOR_MODE_IDX,             e_UInt8,    Set_F_Vector_ModeIDX);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_F_VECTOR_TMAX,	                e_Int16,    Set_F_Vector_TauMax);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_F_VECTOR_DELAY,	            e_UInt16,   Set_F_Vector_Delay);

	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_I_VECTOR_EPSILON,	            e_UInt8,    Set_I_Vector_Epsilon);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_I_VECTOR_KP, 	                e_UInt8,    Set_I_Vector_Kp);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_I_VECTOR_KD,	                e_UInt8,    Set_I_Vector_Kd);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_I_VECTOR_LAMBDA,	            e_UInt8,    Set_I_Vector_Lambda);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_I_VECTOR_DURATION,	            e_UInt16,   Set_I_Vector_Duration);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_I_VECTOR_KP_MAX,	            e_Float32,  Set_I_Vector_Kp_Max); // Actual Kp @ 100%
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_I_VECTOR_KD_MAX,	            e_Float32,  Set_I_Vector_Kd_Max); // Actual Kd @ 100%
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_I_VECTOR_OPTION,	            e_UInt8,    Set_I_Vector_Option); // Actual Kd @ 100%
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_DESIRED_MECH_ANGLE,            e_Float32,  Set_Desired_Mech_Angle);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_ABSENCODER1_LOCATION,          e_UInt8,    Set_IOIF_AbsEnc1_t_Location);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_ABSENCODER2_LOCATION,          e_UInt8,    Set_IOIF_AbsEnc2_t_Location);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_ABSENCODER1_MODULE_ID,         e_UInt8,    Set_IOIF_AbsEnc1_t_Module_ID);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_ABSENCODER2_MODULE_ID,         e_UInt8,    Set_IOIF_AbsEnc2_t_Module_ID);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_TVCF_VERIFICATION_INFO,        e_Float32,  Set_TVCF_Verification_Info);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_GAIN_TRANSITION_TIME,          e_Float32,  Set_Overall_Gain_Transition_Time);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_FREEZE,                        e_UInt8,    Set_Freeze);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_POSITION_REFERENCE_OFFSET,	    e_Float32, 	Set_Position_Reference_Small_Offset); // 2024-07-04 (KHJ)
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_TRAPE_ID_INFO,	                e_Float32, 	Set_Trape_ID_Info); // 2024-07-04 (KHJ)
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_FRICTION_COMP_PARAM,           e_Float32, 	Set_FrictionComp_Param); // 2024-07-04 (KHJ)

	////////////////////////////////////	 Flexi-SEA 		 ////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_POSITION_Tanh_INFO,	e_Float32, 	Set_Position_Tanh_Sig_Info);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_ANKLE_REF_INFO,		e_Float32, 	Set_Ankle_REF_Sig_Info);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_ANKLE_REF_INFO2,		e_Float32, 	Set_Ankle_REF_Sig_Info2);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_ANKLE_REF_INFO3,		e_Float32, 	Set_Ankle_REF_Sig_Info3);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_ASSIST_ONOFF,		e_UInt8, 	Set_Ankle_Onoff);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_RISK_PARAM,		e_Float32, 	Set_Risk_Param);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_INIT_TORQUE,		e_Float32, 	Set_Init_Torque);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_PHASE_SHIFT,		e_Float32, 	Set_Phase_Shift);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_IMPEDANCECTRL_INFO,		e_Float32, 	Set_ImpedanceCtrl_Info);
	Create_SDO(TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_PROPORTIONALCTRL_INFO,		e_Float32, 	Set_ProportionalCtrl_Info);
	/////////////////////////////////////////////////////////////////////////////////////////////////////////

	/* Timer 7 Callback Allocation */
	if(IOIF_StartTimIT(IOIF_TIM7) > 0) {
		//TODO: ERROR PROCESS
	}
	IOIF_SetTimCB(IOIF_TIM7, IOIF_TIM_PERIOD_ELAPSED_CALLBACK, RunMidLvCtrl, NULL);

	/* PWM Input Setting*/
	if(motor_setting.input_info.input_method == e_PWM){
		// Set_GPIOE_State(PWM_INPUT_Pin, GPIO_PIN_RESET);
		// HAL_TIM_IC_Start_IT(&htim15, TIM_CHANNEL_2);

		// ioif_tim15_callback_ptr = Get_PWM_Control_Reference;
	}
}

void RunMidLvCtrl(void* params)
{


	/* Loop Start Time Check */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	//TODO: REMOVE
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_B, IOIF_GPIO_PIN_8, IOIF_GPIO_PIN_SET);

	/*Run Device */
	Run_Task(&mid_level_ctrl_task);

	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_B, IOIF_GPIO_PIN_8, IOIF_GPIO_PIN_RESET);

	/* Elapsed Time Check */
	midTimeElap = DWT->CYCCNT/480;	// in microsecond

	if (midTimeElap > 100)	RT_Broken++;
}

void Tune_Gain()
{
	static float t_wvc = 0.0, t_num = 0.0, t_den = 0.0, t_R = 0.0;

	t_wvc = velCtrl.Ctrl_BW_Hz * 2 * M_PI;
	t_R = posCtrl.R;

	velCtrl.Kp = 0;
	velCtrl.Ki = 0;
	posCtrl.Kp = 0;
	posCtrl.Kd = 0;

	velCtrl.Kp = motor_properties.J * t_wvc;
	velCtrl.Ki = motor_properties.B * t_wvc;


	posCtrl.R = t_R;
	t_num = 2*sqrtf(posCtrl.R*motor_properties.J*motor_properties.J);
	t_den = posCtrl.R;


	posCtrl.Kp = sqrtf(1/posCtrl.R);
	posCtrl.Kd = -motor_properties.B + sqrtf(motor_properties.B*motor_properties.B + ( t_num/t_den ));

	/* Send to GUI */
	uint16_t t_identifier;
	float t_buf[4] = {0};

	memcpy(&t_buf[0], &velCtrl.Kp, 4);
	memcpy(&t_buf[1], &velCtrl.Ki, 4);
	memcpy(&t_buf[2], &posCtrl.Kp, 4);
	memcpy(&t_buf[3], &posCtrl.Kd, 4);

	for(int j = 0; j<100000; ++j){}

	t_identifier = GUI_SYNC|GAIN_TUNER;
	Send_MSG(t_identifier, 16, (uint8_t*)t_buf);

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
	Transition_State(&mid_level_ctrl_task.state_machine, e_State_Standby);
	IOIF_InitIncEnc(IOIF_TIM5, IOIF_TIM_CHANNEL_ALL , &inc25KhzObj);
	Init_Position_Velocity();
	posCtrl.Kp = 0; posCtrl.Kd = 0; posCtrl.Ki=0;
	refAnk.gaitPeriod = 2000;
	RISK_Flexi.max_ank_cnt = 0; RISK_Flexi.min_ank_cnt = 0; RISK_Flexi.max_error_cnt = 0;
	RISK_Flexi.max_error = 10; RISK_Flexi.max_ank_ang = 90; RISK_Flexi.min_ank_ang = -90;

	if (MD_node_id == 7) { // RIGHT
		AbsObj1.offset = 143.5;
	}
	if (MD_node_id == 6) { // LEFT
		AbsObj1.offset = 0.0;
	}

	posDOB.gain = 1;
	Ent_Get_Loadcell();
	Ent_T2F();
	inc1KhzObj.resolution = 16384;
	velCtrl.Kp = 0.9;
	velCtrl.Ki = 0;
	velCtrl.Kd = 0;
	ankle_comp.Jm = 0.003;
	ankle_comp.Bm = 0.2;
	ankle_comp.Kp=0.0378;
	ankle_comp.alpha = ankle_comp.Jm/ankle_comp.Kp;
	ankle_comp.beta = 1 + ankle_comp.Bm/ankle_comp.Kp;
	ankle_comp.gain = 0.1;
	ankle_comp.cutoff = 6;
}

static void StateStandby_Ent( )
{
	mid_level_loop_cnt = 0;

	IOIF_GetPosVelDeg(IOIF_SPI1, &AbsObj1);
	IOIF_ReadIncCnt(IOIF_TIM5, &inc1KhzObj);
	Get_Position_Velocity(&inc1KhzObj, &mid_level_state);

	if        (motor_setting.sensor_setting.m_angle_homing_sensor == e_MHoming_Sensor_Zero) {
		mid_level_state.position_offset = 0;
	} else if (motor_setting.sensor_setting.m_angle_homing_sensor == e_MHoming_Sensor_Abs_Encoder1) {
		mid_level_state.init_position = AbsObj1.posDeg * 0.017453292519943;
		mid_level_state.position_offset += mid_level_state.init_position - mid_level_state.position;
	} else if (motor_setting.sensor_setting.m_angle_homing_sensor == e_MHoming_Sensor_Abs_Encoder2) {
		mid_level_state.init_position = AbsObj2.posDeg * 0.017453292519943;
		mid_level_state.position_offset += mid_level_state.init_position - mid_level_state.position;
	}

	if (isnan(mid_level_state.position_offset))
		mid_level_state.position_offset = 0;
	////////////////////////////////////	 Flexi-SEA 		 ////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////


	////////////////////////////////////////////////////////////////////////////////////////////

}

static void StateStandby_Run( )
{
	IOIF_GetPosVelDeg(IOIF_SPI1, &AbsObj1);
	IOIF_ReadIncCnt(IOIF_TIM5, &inc1KhzObj);
	Get_Position_Velocity(&inc1KhzObj, &mid_level_state);
	///////////////////////////////////        Flexi-SEA       /////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////
	T2F();
	Run_Get_Loadcell();

	////////////////////////////////////////////////////////////////////////////////////////////

	mid_level_loop_cnt++;
//	Transition_State(&mid_level_ctrl_task.state_machine, e_State_Enable);  //주석처리 필요
}

static void StateStandby_Ext( )
{}


static void StateEnable_Ent( )
{
	mid_level_loop_cnt = 0;

	impedanceCtrl.control_input = 0;        // impedance controller
	posCtrl.control_input = 0;
	velCtrl.control_input = 0;
	posFF.control_input = 0;
	posDOB.control_input = 0;

	IOIF_GetPosVelDeg(IOIF_SPI1, &AbsObj1);
	IOIF_GetPosVelDeg(IOIF_SPI3, &AbsObj2);
	IOIF_ReadIncCnt(IOIF_TIM5, &inc1KhzObj);
	Get_Position_Velocity(&inc1KhzObj, &mid_level_state);
	Run_Get_Loadcell();
	///////////////////////////////////        Flexi-SEA       /////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////
	posCtrl.ref = 0;

	////////////////////////////////////////////////////////////////////////////////////////////////

	Ent_Routines(&mid_level_ctrl_task.routine);
	//Init_Risk();
}

static void StateEnable_Run( )
{
	static float t_input = 0;
	static float t_input_2 = 0;


	IOIF_GetPosVelDeg(IOIF_SPI1, &AbsObj1);               // Get Absolute Encoder CH1 Angle
	IOIF_ReadIncCnt(IOIF_TIM5, &inc1KhzObj);              // Get Incremental Encoder Angle
	Get_Position_Velocity(&inc1KhzObj, &mid_level_state); // Calculate Angle and Velocity



	////////////////////////////////           Flexi-SEA        ////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////
	T2F();
	Run_Get_Loadcell();



	/////////////////////////////////////////////////////////////////////////////////////////////

	Run_Routines(&mid_level_ctrl_task.routine);


//	t_input = motor_in.mid_id_process_input + /*control input for model identification */            \
			  impedanceCtrl.control_input   + /*control input of impedance controller */             \
			  posCtrl.control_input         + /*control input of PID position controller */          \
			  velCtrl.control_input         + /*control input of PID velocity controller */          \
			  posFF.control_input           + /*control input of position feed-forward controller */ \
			  VSD.control_input				+ /*control input of virtual-spring-damper */            \
			  -posDOB.control_input;          /*control input of disturbance observer */
	t_input = velCtrl.control_input;

	t_input_2 = t_input-LS.control_input;
	/* Input Saturation */
//	if (motor_out.velocity>vel_limit)		{saturation = mid_ctrl_saturation-k_emf*(motor_out.velocity-vel_limit);}
//	else if (motor_out.velocity<-vel_limit){saturation = mid_ctrl_saturation+k_emf*(motor_out.velocity+vel_limit);}
//	else 							{saturation = mid_ctrl_saturation;}
//
//	if (saturation<0) 	{saturation = 0;}
//	else 				{saturation = 1*saturation;}
	saturation = mid_ctrl_saturation;
	if     (t_input > +mid_ctrl_saturation)		{motor_in.ls_ctrl_input = +mid_ctrl_saturation;}
	else if(t_input < -mid_ctrl_saturation)		{motor_in.ls_ctrl_input = -mid_ctrl_saturation;}
	else										{motor_in.ls_ctrl_input = t_input;}

	//////////////        Flexi-SEA            /////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////

	if     (t_input_2 > +mid_ctrl_saturation)		{motor_in.mid_ctrl_input = +mid_ctrl_saturation;}
	else if(t_input_2 < -mid_ctrl_saturation)		{motor_in.mid_ctrl_input = -mid_ctrl_saturation;}
	else										{motor_in.mid_ctrl_input = t_input_2;}

	//////////////////////////////////////////////////////////////////////

	mid_level_loop_cnt++;
}

static void StateEnable_Ext( )
{
	Ext_Routines(&mid_level_ctrl_task.routine);

	mid_level_loop_cnt = 0;
}

static void StateError_Run( )
{
}

/* ------------------- KALMNA FILTER ------------------- */
static void Init_Kalman_Filter(KFObject * t_KF_obj, float t_kf_A, float t_kf_B, float t_kf_Q, float t_kf_R)
{
  	memset(t_KF_obj, 0, sizeof(KFObject));
	t_KF_obj->kf_A = t_kf_A;
	t_KF_obj->kf_B = t_kf_B;
	t_KF_obj->kf_Q = t_kf_Q;
	t_KF_obj->kf_R = t_kf_R;
	t_KF_obj->kf_P = 1;
	t_KF_obj->kf_y = 0;
}

static int Run_Kalman_Filter(KFObject * t_KF_obj, float t_u, float t_y)
{
	if(isnan(t_KF_obj->kf_y)) t_KF_obj->kf_y = t_y;

	// (STEP1) Model-based Prediction
	t_KF_obj->kf_y = t_KF_obj->kf_A * t_KF_obj->kf_y + t_KF_obj->kf_B * t_u;

	// (STEP2) Project the error covariance ahead
	t_KF_obj->kf_P = t_KF_obj->kf_A * t_KF_obj->kf_P + t_KF_obj->kf_A + t_KF_obj->kf_Q;

	// (STEP3) Update the Kalman Gain
	t_KF_obj->kf_K = t_KF_obj->kf_P / (t_KF_obj->kf_P + t_KF_obj->kf_R);

	// (STEP4) Update the estimate via 'y'
	t_KF_obj->kf_y = t_KF_obj->kf_y + t_KF_obj->kf_K * (t_y - t_KF_obj->kf_y);

	// (STEP5) Update the Error Covariance
	t_KF_obj->kf_P = (1 - t_KF_obj->kf_K) * t_KF_obj->kf_P;

	/* (end) implement here */

	return 0;
}

/* ------------------- GET POS & VEL ------------------- */
static void Init_Position_Velocity()
{
	memset(&mid_level_state, 0, sizeof(mid_level_state));

}

static void Get_Position_Velocity(IOIF_IncEnc_t* t_inc_enc, MidLevelState* t_mid_level_state)
{
	/* STEP 1: Get Position */
	float t_position;
	t_position = motor_setting.commutation_set.ma_dir * (t_inc_enc->userCnt / (t_inc_enc->resolution*motor_properties.gear_ratio) * 2* M_PI); 		// in rad
	t_mid_level_state->position = t_position + t_mid_level_state->position_offset;

	/*Low Pass Filter*/
	t_mid_level_state->velocity_raw = (t_mid_level_state->position - t_mid_level_state->position_f)*MID_LEVEL_CONTROL_FREQUENCY;

	t_mid_level_state->position_f = t_mid_level_state->position;

	/* Velocity Estimator */
	if (advanced_friction_id.state == 1) {
		// Friction ID -> 30Hz LPF
		t_mid_level_state->velocity_final = 0.8282*t_mid_level_state->velocity_final + 0.1718*t_mid_level_state->velocity_raw;
	}
	else if(veObj.type == e_VE_RAW){

		veObj.velocity = t_mid_level_state->velocity_raw;
		t_mid_level_state->velocity_final = veObj.velocity;

	}else if((veObj.type >= e_VE_LPF_300) && (veObj.type <= e_VE_LPF_500)){

		veObj.vel_save[0] = t_mid_level_state->velocity_raw;
		veObj.velocity = veObj.lpf_a*veObj.vel_save[1] + veObj.lpf_b*veObj.vel_save[0];
		veObj.vel_save[1] = veObj.velocity;
		t_mid_level_state->velocity_final = veObj.velocity;

	}else if(veObj.type == e_VE_MOVING_AVR){

		veObj.pos_save[0] = t_mid_level_state->position;
		veObj.velocity = (veObj.pos_save[0] - veObj.pos_save[veObj.masking_size])/(veObj.masking_size*MID_LEVEL_CONTROL_PERIOD);
		for(int i = veObj.masking_size; i > 0; --i){
			veObj.pos_save[i] = veObj.pos_save[i-1];
		}
		t_mid_level_state->velocity_final = veObj.velocity;

	} else if(veObj.type == e_VE_KALMAN){
		Run_Kalman_Filter(&veObj.kf_obj, (motor_in.mid_ctrl_input + motor_in.irc_input + motor_in.auxiliary_input + motor_in.f_vector_input), t_mid_level_state->velocity_raw);
		veObj.velocity = veObj.kf_obj.kf_y;
		t_mid_level_state->velocity_final = veObj.kf_obj.kf_y;
	} else {
//		t_mid_level_state->velocity_final = t_mid_level_state->velocity_raw;
		// 500Hz //
		veObj.lpf_a = exp(-50*2*M_PI*MID_LEVEL_CONTROL_PERIOD);
		veObj.lpf_b = 1-veObj.lpf_a;
		veObj.vel_save[0] = t_mid_level_state->velocity_raw;
		veObj.velocity = veObj.lpf_a*veObj.vel_save[1] + veObj.lpf_b*veObj.vel_save[0];
		veObj.vel_save[1] = veObj.velocity;
		t_mid_level_state->velocity_final = veObj.velocity;
	}
}

/* ------------------- INIT F VECTOR MODES ------------------- */
static void Init_F_Vector_Modes()
{
	static double t_temp1 = 0.0, t_temp2 = 0.0, t_temp3 = 0.0;

	float t_tp[10] = {0.1, 0.2, 0.3, 0.5, 0.75, 1, 2, 3, 4, 5};
	memcpy(fvectorObj.tp, t_tp, sizeof(t_tp));

	for(int i = 0; i < F_MODE_NUM; ++i){

		fvectorObj.mode_param[i].tp = t_tp[i];
		fvectorObj.mode_param[i].wn = 1 / fvectorObj.mode_param[i].tp;

		t_temp1 = (fvectorObj.mode_param[i].wn * MID_LEVEL_CONTROL_PERIOD + 2);
		t_temp2 = (fvectorObj.mode_param[i].wn * MID_LEVEL_CONTROL_PERIOD - 2);
		t_temp3 = MID_LEVEL_CONTROL_PERIOD * fvectorObj.mode_param[i].wn * exp(1);

		fvectorObj.mode_param[i].b0 = t_temp1*t_temp1;
		fvectorObj.mode_param[i].b1 = -2*t_temp2/t_temp1;
		fvectorObj.mode_param[i].b2 = -(t_temp2*t_temp2) / fvectorObj.mode_param[i].b0;
		fvectorObj.mode_param[i].a0 = t_temp3 / fvectorObj.mode_param[i].b0;
		fvectorObj.mode_param[i].a1 = 2 * t_temp3 / fvectorObj.mode_param[i].b0;
		fvectorObj.mode_param[i].a2 = fvectorObj.mode_param[i].a0;
	}
}

static void Init_Controller_Overall_Gain(float init_gain)
{
	float t_init_gain = 0;
	if      (init_gain > 1) t_init_gain = 1;
	else if (init_gain < 0)	t_init_gain = 0;
	else                    t_init_gain = init_gain;

	// Feed-Forward Controller (FF)
	posFF.overall_gain_curr = t_init_gain;
	posFF.overall_gain_des  = t_init_gain;
	posFF.overall_gain_trigger = 0;
	posFF.overall_gain_time_stamp = 0;
	posFF.overall_gain_gap = 0;
	posFF.overall_gain_transition_time = 1; //s

	// PD Position Controller (PD)
	posCtrl.overall_gain_curr = t_init_gain;
	posCtrl.overall_gain_des  = t_init_gain;
	posCtrl.overall_gain_trigger = 0;
	posCtrl.overall_gain_time_stamp = 0;
	posCtrl.overall_gain_gap = 0;
	posCtrl.overall_gain_transition_time = 1; //s

	// Impedance Controller (IC)
	impedanceCtrl.overall_gain_curr = t_init_gain;
	impedanceCtrl.overall_gain_des  = t_init_gain;
	impedanceCtrl.overall_gain_trigger = 0;
	impedanceCtrl.overall_gain_time_stamp = 0;
	impedanceCtrl.overall_gain_gap = 0;
	impedanceCtrl.overall_gain_transition_time = 1; //s

	// Disturbance Observer (DOB)
	posDOB.overall_gain_curr = t_init_gain;
	posDOB.overall_gain_des  = t_init_gain;
	posDOB.overall_gain_trigger = 0;
	posDOB.overall_gain_time_stamp = 0;
	posDOB.overall_gain_gap = 0;
	posDOB.overall_gain_transition_time = 1; //s

	// Impedance Reduction Controller (IRC)
	IRC.overall_gain_curr = t_init_gain;
	IRC.overall_gain_des  = t_init_gain;
	IRC.overall_gain_trigger = 0;
	IRC.overall_gain_time_stamp = 0;
	IRC.overall_gain_gap = 0;
	IRC.overall_gain_transition_time = 1; //s

	// Friction Compensator (FC)
	advanced_friction_id.overall_gain_curr = t_init_gain;
	advanced_friction_id.overall_gain_des  = t_init_gain;
	advanced_friction_id.overall_gain_trigger = 0;
	advanced_friction_id.overall_gain_time_stamp = 0;
	advanced_friction_id.overall_gain_gap = 0;
	advanced_friction_id.overall_gain_transition_time = 1; //s
}
/* ------------------- SDO CALLBACK ------------------- */
static void Set_IRC_Numerator_Length(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&IRC.numerator_length, req->data, 1);

	memset(&IRC.irc_num, 0, sizeof(IRC.irc_num));

	res->size = 0;
	res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_IRC_Denominator_Length(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&IRC.denominator_length, req->data, 1);

	memset(&IRC.irc_den, 0, sizeof(IRC.irc_den));

	res->size = 0;
	res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_IRC_Numerator(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&IRC.irc_num, req->data, 4*IRC.numerator_length);

	res->size = 0;
	res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_IRC_Denominator(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&IRC.irc_den, req->data, 4*IRC.denominator_length);

	res->size = 0;
	res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_IRC_Saturation(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&IRC.saturation, req->data, 4);

	res->size = 0;
	res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Vel_Ctrl_BW(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&velCtrl.Ctrl_BW_Hz, t_req->data, 4);

	t_res->size = 0;
	t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Vel_Ctrl_PGain(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&velCtrl.Kp, t_req->data, 4);

	t_res->size = 0;
	t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Vel_Ctrl_IGain(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&velCtrl.Ki, t_req->data, 4);

	t_res->size = 0;
	t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Pos_Ctrl_Input_Penalty(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&posCtrl.R, t_req->data, 4);

	t_res->size = 0;
	t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Pos_Ctrl_PGain(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&posCtrl.Kp, t_req->data, 4);

	t_res->size = 0;
	t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Pos_Ctrl_DGain(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&posCtrl.Kd, t_req->data, 4);

	t_res->size = 0;
	t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Mid_Ctrl_Saturation(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&mid_ctrl_saturation, t_req->data, 4);

	t_res->size = 0;
	t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_IOIF_IncEnc_t_Offset(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	IOIF_ReadIncCnt(IOIF_TIM5,   &inc1KhzObj);
	IOIF_SetIncOffset(IOIF_TIM5, &inc1KhzObj);

	t_res->size = 0;
	t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_IOIF_AbsEnc1_t_Offset(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	float t_des_angle;
	memcpy(&t_des_angle, t_req->data, 4);

	IOIF_GetPosVelDeg(IOIF_SPI1, &AbsObj1);

	AbsObj1.offset = AbsObj1.offset + AbsObj1.sign*(AbsObj1.posDeg - t_des_angle);
	if (AbsObj1.offset > 360)  AbsObj1.offset = AbsObj1.offset - 360;
	if (AbsObj1.offset < -360) AbsObj1.offset = AbsObj1.offset + 360;

	t_res->size = 0;
   	t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_IOIF_AbsEnc1_t_Sign(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	IOIF_SetAbsSign(&AbsObj1);

	t_res->size = 0;
   	t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_IOIF_AbsEnc2_t_Offset(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	float t_des_angle;
	memcpy(&t_des_angle, t_req->data, 4);

	IOIF_GetPosVelDeg(IOIF_SPI3, &AbsObj2);

	AbsObj2.offset = AbsObj2.offset + AbsObj2.sign*(AbsObj2.posDeg - t_des_angle);
	if (AbsObj2.offset > 360)  AbsObj2.offset = AbsObj2.offset - 360;
	if (AbsObj2.offset < -360) AbsObj2.offset = AbsObj2.offset + 360;

	t_res->size = 0;
   	t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_IOIF_AbsEnc2_t_Sign(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	IOIF_SetAbsSign(&AbsObj2);

	t_res->size = 0;
   	t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_DOB_Q_BW(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&posDOB.wc_Q, t_req->data, 4);

	t_res->size = 0;
	t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_DOB_GQ_Num(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memset(posDOB.gq_num, 0, sizeof(posDOB.gq_num));

	memcpy(&posDOB.gq_num_length, t_req->data, 4);
	memcpy(posDOB.gq_num, ((float*)t_req->data)+1, posDOB.gq_num_length*4);

   t_res->size = 0;
   t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_DOB_GQ_Den(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memset(posDOB.gq_den, 0, sizeof(posDOB.gq_den));

	memcpy(&posDOB.gq_den_length, t_req->data, 4);
	memcpy(posDOB.gq_den, ((float*)t_req->data)+1, posDOB.gq_den_length*4);

   t_res->size = 0;
   t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_DOB_Q_Num(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memset(posDOB.q_num, 0, sizeof(posDOB.q_num));

	memcpy(&posDOB.q_num_length, t_req->data, 4);
	memcpy(posDOB.q_num, ((float*)t_req->data)+1, posDOB.q_num_length*4);

   t_res->size = 0;
   t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_DOB_Q_Den(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memset(posDOB.q_den, 0, sizeof(posDOB.q_den));

	memcpy(&posDOB.q_den_length, t_req->data, 4);
	memcpy(posDOB.q_den, ((float*)t_req->data)+1, posDOB.q_den_length*4);

   t_res->size = 0;
   t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_DOB_Saturation(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&posDOB.saturation, t_req->data, 4);

   t_res->size = 0;
   t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Current_Periodic_Sig_Info(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&cur_periodic_sig.amp, 		t_req->data, 			 4);
	memcpy(&cur_periodic_sig.freq, 		((float*)t_req->data)+1, 4);
	memcpy(&cur_periodic_sig.offset,    ((float*)t_req->data)+2, 4);

   t_res->size = 0;
   t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Velocity_Periodic_Sig_Info(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&vel_periodic_sig.amp, 		t_req->data, 			 4);
	memcpy(&vel_periodic_sig.freq, 		((float*)t_req->data)+1, 4);

   t_res->size = 0;
   t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Position_Periodic_Sig_Info(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&pos_periodic_sig.amp, 		t_req->data, 4);
	memcpy(&pos_periodic_sig.freq, 		((float*)t_req->data)+1, 4);
	memcpy(&pos_periodic_sig.offset,	((float*)t_req->data)+2, 4);


   t_res->size = 0;
   t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_VSD_Stiffness(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&VSD.lower_stiffness, t_req->data, 8);

   t_res->size = 0;
   t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_VSD_Damper(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&VSD.lower_damper, t_req->data, 8);

   t_res->size = 0;
   t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_VSD_Damped_Range(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&VSD.lower_damped_range, t_req->data, 8);

	VSD.lower_damper_origin = VSD.lower_limit + VSD.lower_damped_range;
	VSD.upper_damper_origin = VSD.upper_limit - VSD.upper_damped_range;

   t_res->size = 0;
   t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_VSD_Stiff_Range(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&VSD.lower_stiff_range, t_req->data, 8);

	VSD.lower_spring_origin = VSD.lower_limit + VSD.lower_stiff_range;
	VSD.upper_spring_origin = VSD.upper_limit - VSD.upper_stiff_range;

   t_res->size = 0;
   t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_VSD_Upper_Limit(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	static uint16_t t_identifier = 0;

	IOIF_GetPosVelDeg(IOIF_SPI1, &AbsObj1);
	IOIF_GetPosVelDeg(IOIF_SPI3, &AbsObj2);

	if (AbsObj1.location == IOIF_ABS_ENC_ACTUATOR_OUTPUT)
		VSD.upper_limit = AbsObj1.posDeg;
	else if (AbsObj2.location == IOIF_ABS_ENC_ACTUATOR_OUTPUT)
		VSD.upper_limit = AbsObj2.posDeg;

	VSD.upper_damper_origin = VSD.upper_limit - VSD.upper_damped_range;
	VSD.upper_spring_origin = VSD.upper_limit - VSD.upper_stiff_range;

	t_identifier = GUI_SYNC|GET_VSD_UPPER_LIMIT;
	Send_MSG(t_identifier, 4, (uint8_t*)(&VSD.upper_limit));

   t_res->size = 0;
   t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_VSD_Lower_Limit(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	static uint16_t t_identifier = 0;

	IOIF_GetPosVelDeg(IOIF_SPI1, &AbsObj1);
	IOIF_GetPosVelDeg(IOIF_SPI3, &AbsObj2);

	if (AbsObj1.location == IOIF_ABS_ENC_ACTUATOR_OUTPUT)
		VSD.lower_limit = AbsObj1.posDeg;
	else if (AbsObj2.location == IOIF_ABS_ENC_ACTUATOR_OUTPUT)
		VSD.lower_limit = AbsObj2.posDeg;

	VSD.lower_damper_origin = VSD.lower_limit + VSD.lower_damped_range;
	VSD.lower_spring_origin = VSD.lower_limit + VSD.lower_stiff_range;

	t_identifier = GUI_SYNC|GET_VSD_LOWER_LIMIT;
	Send_MSG(t_identifier, 4, (uint8_t*)(&VSD.lower_limit));

   t_res->size = 0;
   t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_VSD_Saturation(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&VSD.saturation, t_req->data, 4);

   t_res->size = 0;
   t_res->status = DATA_OBJECT_SDO_SUCC;
}

/******************** C Vector Setting ********************/
static void Set_C_Vector_FF_Gain(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	uint8_t t_gain;
	memcpy(&t_gain, 	         t_req->data,               1);
	posFF.overall_gain_des = (float)t_gain/255;

	posFF.overall_gain_gap = (posFF.overall_gain_des - posFF.overall_gain_curr)/(MID_LEVEL_CONTROL_FREQUENCY * posFF.overall_gain_transition_time);
	posFF.overall_gain_trigger = 1;
	posFF.overall_gain_time_stamp = 0;

	t_res->size = 0;
	t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_C_Vector_PD_Gain(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	uint8_t t_gain;
	memcpy(&t_gain, 	         t_req->data,               1);
	posCtrl.overall_gain_des = (float)t_gain/255;

	posCtrl.overall_gain_gap = (posCtrl.overall_gain_des - posCtrl.overall_gain_curr)/(MID_LEVEL_CONTROL_FREQUENCY * posCtrl.overall_gain_transition_time);
	posCtrl.overall_gain_trigger = 1;
	posCtrl.overall_gain_time_stamp = 0;

	t_res->size = 0;
	t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_C_Vector_IC_Gain(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	uint8_t t_gain;
	memcpy(&t_gain, 	         t_req->data,               1);
	impedanceCtrl.overall_gain_des = (float)t_gain/255;

	impedanceCtrl.overall_gain_gap = (impedanceCtrl.overall_gain_des - impedanceCtrl.overall_gain_curr)/(MID_LEVEL_CONTROL_FREQUENCY * impedanceCtrl.overall_gain_transition_time);
	impedanceCtrl.overall_gain_trigger = 1;
	impedanceCtrl.overall_gain_time_stamp = 0;

	t_res->size = 0;
	t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_C_Vector_DOB_Gain(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	uint8_t t_gain;
	memcpy(&t_gain, 	         t_req->data,               1);
	posDOB.overall_gain_des = (float)t_gain/255;

	posDOB.overall_gain_gap = (posDOB.overall_gain_des - posDOB.overall_gain_curr)/(MID_LEVEL_CONTROL_FREQUENCY * posDOB.overall_gain_transition_time);
	posDOB.overall_gain_trigger = 1;
	posDOB.overall_gain_time_stamp = 0;

	t_res->size = 0;
	t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_C_Vector_IRC_Gain(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	uint8_t t_gain;
	memcpy(&t_gain, 	         t_req->data,               1);
	IRC.overall_gain_des = (float)t_gain/255;

	IRC.overall_gain_gap = (IRC.overall_gain_des - IRC.overall_gain_curr)/(MID_LEVEL_CONTROL_FREQUENCY * IRC.overall_gain_transition_time);
	IRC.overall_gain_trigger = 1;
	IRC.overall_gain_time_stamp = 0;

	t_res->size = 0;
	t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_C_Vector_FC_Gain(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	uint8_t t_gain;
	memcpy(&t_gain, 	         t_req->data,               1);
	advanced_friction_id.overall_gain_des = (float)t_gain/255;

	advanced_friction_id.overall_gain_gap = (advanced_friction_id.overall_gain_des - advanced_friction_id.overall_gain_curr)/(LOW_LEVEL_CONTROL_FREQUENCY * advanced_friction_id.overall_gain_transition_time);
	advanced_friction_id.overall_gain_trigger = 1;
	advanced_friction_id.overall_gain_time_stamp = 0;

	t_res->size = 0;
	t_res->status = DATA_OBJECT_SDO_SUCC;
}
/******************** P Vector Setting ********************/
static void Set_P_Vector_Yd(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&pvectorObj.p_buffer[pvectorObj.N].yd , req->data, 2);

   res->size = 0;
   res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_P_Vector_L(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&pvectorObj.p_buffer[pvectorObj.N].L , req->data, 2);

	res->size = 0;
	res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_P_Vector_S0(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&pvectorObj.p_buffer[pvectorObj.N].s0 , req->data, 1);

	res->size = 0;
	res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_P_Vector_Sd(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&pvectorObj.p_buffer[pvectorObj.N].sd , req->data, 1);

	pvectorObj.N++;
	res->size = 0;
	res->status = DATA_OBJECT_SDO_SUCC;
}

/******************** F Vector Setting ********************/
static void Set_F_Vector_ModeIDX(MsgSDOargs* req, MsgSDOargs* res)
{
	for (int8_t t_idx = 0; t_idx < F_VECTOR_BUFF_SIZE; t_idx++)
	{
		if (fvectorObj.f_buffer[t_idx].is_full == 0)
		{
			fvectorObj.temp_idx = t_idx;
			break;
		}
	}
	memcpy(&fvectorObj.f_buffer[fvectorObj.temp_idx].mode_idx , req->data, 1);

	float t_tp = fvectorObj.mode_param[fvectorObj.f_buffer[fvectorObj.temp_idx].mode_idx].tp;
	fvectorObj.f_buffer[fvectorObj.temp_idx].t_end = (uint32_t)(15 * t_tp * MID_LEVEL_CONTROL_FREQUENCY);

	res->size = 0;
	res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_F_Vector_TauMax(MsgSDOargs* req, MsgSDOargs* res)
{

	memcpy(&fvectorObj.f_buffer[fvectorObj.temp_idx].tau_max , req->data, 2);

	res->size = 0;
	res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_F_Vector_Delay(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&fvectorObj.f_buffer[fvectorObj.temp_idx].delay , req->data, 2);
	fvectorObj.f_buffer[fvectorObj.temp_idx].time_stamp = 0;
	fvectorObj.f_buffer[fvectorObj.temp_idx].is_full = 1;

	res->size = 0;
	res->status = DATA_OBJECT_SDO_SUCC;
}

/******************** I Vector Setting ********************/

static void Set_I_Vector_Epsilon(MsgSDOargs* req, MsgSDOargs* res)
{
	if (impedanceCtrl.option == 0) {
		memcpy(&impedanceCtrl.i_buffer[impedanceCtrl.N].epsilon_target , req->data, 1);
	} else if (impedanceCtrl.option == 1)
	{
		memcpy(&impedanceCtrl.opt1_i_buffer.epsilon_target , req->data, 1);
	}

   res->size = 0;
   res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_I_Vector_Kp(MsgSDOargs* req, MsgSDOargs* res)
{
	if (impedanceCtrl.option == 0) {
		memcpy(&impedanceCtrl.i_buffer[impedanceCtrl.N].Kp_target , req->data, 1);
	} else if (impedanceCtrl.option == 1)
	{
		memcpy(&impedanceCtrl.opt1_i_buffer.Kp_target , req->data, 1);
	}

   res->size = 0;
   res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_I_Vector_Kd(MsgSDOargs* req, MsgSDOargs* res)
{
	if (impedanceCtrl.option == 0) {
		memcpy(&impedanceCtrl.i_buffer[impedanceCtrl.N].Kd_target , req->data, 1);
	} else if (impedanceCtrl.option == 1)
	{
		memcpy(&impedanceCtrl.opt1_i_buffer.Kd_target , req->data, 1);
	}

   res->size = 0;
   res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_I_Vector_Lambda(MsgSDOargs* req, MsgSDOargs* res)
{
	if (impedanceCtrl.option == 0) {
		memcpy(&impedanceCtrl.i_buffer[impedanceCtrl.N].lambda_target, req->data, 1);
	} else if (impedanceCtrl.option == 1)
	{
		memcpy(&impedanceCtrl.opt1_i_buffer.lambda_target , req->data, 1);
	}

   res->size = 0;
   res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_I_Vector_Duration(MsgSDOargs* req, MsgSDOargs* res)
{
	if (impedanceCtrl.option == 0) {
		memcpy(&impedanceCtrl.i_buffer[impedanceCtrl.N].duration , req->data, 2);
	} else if (impedanceCtrl.option == 1)
	{
		memcpy(&impedanceCtrl.opt1_i_buffer.duration , req->data, 2);
	}

	impedanceCtrl.N++;

	res->size = 0;
   	res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_I_Vector_Kp_Max(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&impedanceCtrl.Kp_max , req->data, 4);

	res->size = 0;
   	res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_I_Vector_Kd_Max(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&impedanceCtrl.Kd_max , req->data, 4);

	res->size = 0;
   	res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_I_Vector_Option(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&impedanceCtrl.option , req->data, 4);

	res->size = 0;
   	res->status = DATA_OBJECT_SDO_SUCC;
}

// input: yd -> update offset to set y = yd
static void Set_Desired_Mech_Angle(MsgSDOargs* req, MsgSDOargs* res)
{
	//float t_current_angle;
	//t_current_angle = *(float*)(req->data);

	memcpy(&mid_level_state.init_position, req->data, 4);

	IOIF_ReadIncCnt(IOIF_TIM5, &inc1KhzObj);
	Get_Position_Velocity(&inc1KhzObj, &mid_level_state);
	mid_level_state.position_offset += mid_level_state.init_position - mid_level_state.position;

	res->size = 0;
   	res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_IOIF_AbsEnc1_t_Location(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&AbsObj1.location, req->data, 1);

	res->size = 0;
   	res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_IOIF_AbsEnc2_t_Location(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&AbsObj2.location, req->data, 1);

	res->size = 0;
   	res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_IOIF_AbsEnc1_t_Module_ID(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&AbsObj1.module_id, req->data, 1);

	res->size = 0;
   	res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_IOIF_AbsEnc2_t_Module_ID(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&AbsObj2.module_id, req->data, 1);

	res->size = 0;
   	res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Overall_Gain_Transition_Time(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&posFF.overall_gain_transition_time,                req->data, 4);
	memcpy(&posCtrl.overall_gain_transition_time,              req->data, 4);
	memcpy(&posDOB.overall_gain_transition_time,  	           req->data, 4);
	memcpy(&posFF.overall_gain_transition_time,                req->data, 4);
	memcpy(&impedanceCtrl.overall_gain_transition_time,        req->data, 4);
	memcpy(&advanced_friction_id.overall_gain_transition_time, req->data, 4);

	res->size = 0;
   	res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Feedforward_Num(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memset(posFF.num, 0, sizeof(posFF.num));

	memcpy(&posFF.num_length, t_req->data, 4);
	memcpy(posFF.num, ((float*)t_req->data)+1, 4*(uint8_t)(posFF.num_length));

	t_res->size = 0;
	t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Feedforward_Den(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memset(posFF.den, 0, sizeof(posFF.den));

	memcpy(&posFF.den_length, t_req->data, 4);
	memcpy(posFF.den, ((float*)t_req->data)+1, 4*(uint8_t)(posFF.den_length));

	t_res->size = 0;
	t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Velocity_Estimator(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memset(&veObj.lpf_a, 0, sizeof(float));
	memset(&veObj.lpf_b, 0, sizeof(float));
	memset(veObj.vel_save, 0, sizeof(veObj.vel_save));
	memset(veObj.pos_save, 0, sizeof(veObj.pos_save));
	memset(&veObj.kf_obj, 0, sizeof(veObj.kf_obj));
	veObj.leadlag_a = 0;
	veObj.leadlag_b = 0;
	veObj.masking_size = 0;

	memcpy(&veObj.type, t_req->data, 1);
	memcpy(&veObj.masking_size, ((uint8_t*)t_req->data)+1, 1);

	if(veObj.type == e_VE_LPF_30){
		veObj.lpf_a = exp(-30*2*M_PI*MID_LEVEL_CONTROL_PERIOD);
		veObj.lpf_b = 1-veObj.lpf_a;
	} else if(veObj.type == e_VE_LPF_300){
		veObj.lpf_a = exp(-300*2*M_PI*MID_LEVEL_CONTROL_PERIOD);
		veObj.lpf_b = 1-veObj.lpf_a;
	} else if(veObj.type == e_VE_LPF_400){
		veObj.lpf_a = exp(-400*2*M_PI*MID_LEVEL_CONTROL_PERIOD);
		veObj.lpf_b = 1-veObj.lpf_a;
	} else if(veObj.type == e_VE_LPF_500){
		veObj.lpf_a = exp(-500*2*M_PI*MID_LEVEL_CONTROL_PERIOD);
		veObj.lpf_b = 1-veObj.lpf_a;
	} else if(veObj.type == e_VE_KALMAN){

		if(isnan(motor_properties.a) || isnan(motor_properties.b)){
			Send_MSG((uint16_t)(GUI_SYNC|VE_KF_SETTING_ERROR), 1, (uint8_t*)0);
		} else if(motor_properties.a == 0 || motor_properties.b == 0){
			Send_MSG((uint16_t)(GUI_SYNC|VE_KF_SETTING_ERROR), 1, (uint8_t*)0);
		} else {
			Init_Kalman_Filter(&veObj.kf_obj, -motor_properties.a, motor_properties.b, 1, 5);
		}
	}

	t_res->size = 0;
	t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Velocity_Estimator_LeadLag(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&veObj.leadlag_a, t_req->data, 4);
	memcpy(&veObj.leadlag_b, ((float*)t_req->data)+1, 4);

	t_res->size = 0;
	t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_1khz_Enc_Resolution(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&inc1KhzObj.resolution , t_req->data, 4);

	t_res->size = 0;
	t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_System_ID_SBS_Info(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&sys_id_sbs.fmin, 		t_req->data, 4);
	memcpy(&sys_id_sbs.fmax, 		((float*)t_req->data)+1, 4);
	memcpy(&sys_id_sbs.N_samples, 	((float*)t_req->data)+2, 4);
	memcpy(&sys_id_sbs.N_iter, 		((float*)t_req->data)+3, 4);
	memcpy(&sys_id_sbs.amp, 		((float*)t_req->data)+4, 4);
	memcpy(&sys_id_sbs.offset, 		((float*)t_req->data)+5, 4);

	t_res->size = 0;
	t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_System_ID_Verification_Mag(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&sys_id_sbs.verify_mag , t_req->data, 4);

	t_res->size = 0;
	t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Freeze(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&mid_level_state.enable_freeze, t_req->data, 1);

	t_res->size = 0;
	t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Position_Reference_Small_Offset(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	if (cos3vectorObj.ON == 0)
	{
		memcpy(&cos3vectorObj.Amplitude, 		t_req->data, 4);
		memcpy(&cos3vectorObj.Period, 		    ((float*)t_req->data)+1, 4);

		cos3vectorObj.ON = 1;
		cos3vectorObj.cnt = 0;
	}
	t_res->size = 0;
	t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_TVCF_Verification_Info(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&TVCF_ver.fmin, 		t_req->data, 4);
	memcpy(&TVCF_ver.fmax, 		((float*)t_req->data)+1, 4);
	memcpy(&TVCF_ver.N_samples, ((float*)t_req->data)+2, 4);
	memcpy(&TVCF_ver.N_iter, 	((float*)t_req->data)+3, 4);
	memcpy(&TVCF_ver.amp, 		((float*)t_req->data)+4, 4);
	memcpy(&TVCF_ver.offset, 	((float*)t_req->data)+5, 4);

	t_res->size = 0;
	t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Trape_ID_Info(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&TrapeIDObj.Amp, 	             	t_req->data, 4);
	memcpy(&TrapeIDObj.Delta, 		((float*)t_req->data)+1, 4);
	memcpy(&TrapeIDObj.Offset, 	    ((float*)t_req->data)+2, 4);
	memcpy(&TrapeIDObj.V_min, 		((float*)t_req->data)+3, 4);
	memcpy(&TrapeIDObj.V_max, 		((float*)t_req->data)+4, 4);
	memcpy(&TrapeIDObj.N_Samples,   ((float*)t_req->data)+5, 4);
	memcpy(&TrapeIDObj.N_Iter,      ((float*)t_req->data)+6, 4);

	t_res->size = 0;
	t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_FrictionComp_Param(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&FrictionCompObj.mu_C, 	             	t_req->data, 4);
	memcpy(&FrictionCompObj.mu_V, 	    ((float*)t_req->data)+1, 4);
	memcpy(&FrictionCompObj.Epsilon, 	((float*)t_req->data)+2, 4);

	t_res->size = 0;
	t_res->status = DATA_OBJECT_SDO_SUCC;
}
/* ------------------- ROUTINE ------------------- */
/*ROUTINE_ID_MIDLEVEL_POSITION_CTRL*/

//static int Ent_Position_Ctrl()
//{
//	posCtrl.err = 0;
//	posCtrl.err_sum = 0;
//	posCtrl.err_diff = 0;
//	mid_level_state.initial_pos = mid_level_state.position;
//
//	return 0;
//}
//static int Run_Position_Ctrl()
//{
//	posCtrl.total_ref = posCtrl.ref + cos3vectorObj.y;
//
//	Run_PID_Control(&posCtrl, posCtrl.total_ref, mid_level_state.position, MID_LEVEL_CONTROL_PERIOD);
//
//	if (posCtrl.overall_gain_trigger == 1)
//	{
//		posCtrl.overall_gain_curr = posCtrl.overall_gain_curr + posCtrl.overall_gain_gap;
//		if (posCtrl.overall_gain_time_stamp == MID_LEVEL_CONTROL_FREQUENCY * posCtrl.overall_gain_transition_time)
//		{
//			posCtrl.overall_gain_gap        = 0;
//			posCtrl.overall_gain_time_stamp = 0;
//			posCtrl.overall_gain_trigger    = 0;
//			posCtrl.overall_gain_curr       = posCtrl.overall_gain_des;
//		}
//		posCtrl.overall_gain_time_stamp++;
//
//		if (posCtrl.overall_gain_curr > 1) posCtrl.overall_gain_curr = 1;
//		if (posCtrl.overall_gain_curr < 0) posCtrl.overall_gain_curr = 0;
//	}
//	posCtrl.control_input = posCtrl.control_input * posCtrl.overall_gain_curr;
//
//	return 0;
//}

/*ROUTINE_ID_MIDLEVEL_VELOCITY_CTRL*/
static int Ent_Velocity_Ctrl()
{

	velCtrl.err = 0;
	velCtrl.err_sum = 0;
	velCtrl.err_diff = 0;

	return 0;
}
static int Run_Velocity_Ctrl()
{
	velCtrl.ref = posCtrl.control_input         + /*control input of PID position controller */          \
				  motor_in.mid_id_process_input +														 \
				  posFF.control_input           + /*control input of position feed-forward controller */ \
				  -posDOB.control_input;          /*control input of disturbance observer */
//	velCtrl.t_ref = velCtrl.ref - ankle_comp.control_input;
	Run_PID_Control(&velCtrl, velCtrl.ref, mid_level_state.velocity_final, MID_LEVEL_CONTROL_PERIOD);

	return 0;
}

/*ROUTINE_ID_MIDLEVEL_IRC*/
static int Ent_Impedance_Reduction()
{
	memset(IRC.irc_input, 0, sizeof(IRC.irc_input));
	memset(IRC.irc_output, 0, sizeof(IRC.irc_output));
	motor_in.irc_input = 0;
	return 0;
}
static int Run_Impedance_Reduction()
{
	double t_vel_term = 0.0, t_cur_term = 0.0, t_input = 0.0;

	/* k-th input (velocity) */
	IRC.irc_input[0] = mid_level_state.velocity_final;

	/* Calculation */
	for(int i = 0; i < IRC.numerator_length; ++i){
		t_vel_term += IRC.irc_num[i]*IRC.irc_input[i];
	}
	for(int i = 1; i < IRC.denominator_length; ++i){
		t_cur_term += IRC.irc_den[i]*IRC.irc_output[i];
	}

	t_input = -t_cur_term + t_vel_term;

	/* Saturation */
	if(t_input > IRC.saturation)		{t_input = IRC.saturation;}
	else if(t_input < -IRC.saturation)	{t_input = -IRC.saturation;}

	/* Transfer to Low-level Task*/
	if (IRC.overall_gain_trigger == 1)
	{
		IRC.overall_gain_curr = IRC.overall_gain_curr + IRC.overall_gain_gap;
		if (IRC.overall_gain_time_stamp == MID_LEVEL_CONTROL_FREQUENCY * IRC.overall_gain_transition_time)
		{
			IRC.overall_gain_gap        = 0;
			IRC.overall_gain_time_stamp = 0;
			IRC.overall_gain_trigger    = 0;
			IRC.overall_gain_curr       = IRC.overall_gain_des;
		}
		IRC.overall_gain_time_stamp++;

		if (IRC.overall_gain_curr > 1) IRC.overall_gain_curr = 1;
		if (IRC.overall_gain_curr < 0) IRC.overall_gain_curr = 0;
	}
	motor_in.irc_input = t_input * IRC.overall_gain_curr;

	IRC.irc_output[0] = motor_in.irc_input;

	/* Array Shifting */
	for(int i = IRC.numerator_length-1; i > 0; --i){
		IRC.irc_input[i] = IRC.irc_input[i-1];
	}
	for(int i = IRC.denominator_length-1; i > 0; --i){
		IRC.irc_output[i] = IRC.irc_output[i-1];
	}

	return 0;
}

/*ROUTINE_ID_MIDLEVEL_COMPRESSIONAL_VSD*/
static int Ent_Compressional_SpringDamper()
{
	VSD.lower_damper_origin = VSD.lower_limit + VSD.lower_damped_range;
	VSD.lower_spring_origin = VSD.lower_limit + VSD.lower_stiff_range;
	VSD.upper_damper_origin = VSD.upper_limit - VSD.upper_damped_range;
	VSD.upper_spring_origin = VSD.upper_limit - VSD.upper_stiff_range;

	VSD.control_input = 0;
	return 0;
}

static int Run_Compressional_SpringDamper()
{
	VSD.control_input = 0;

	float t_posDeg = 0;
	float t_velDeg = 0;

	if      (AbsObj1.location == IOIF_ABS_ENC_ACTUATOR_OUTPUT) {
		t_posDeg = AbsObj1.posDeg;
	    t_velDeg = AbsObj1.velDeg;
	}
	else if (AbsObj2.location == IOIF_ABS_ENC_ACTUATOR_OUTPUT) {
		t_posDeg = AbsObj2.posDeg;
	    t_velDeg = AbsObj2.velDeg;
	}

	/* Lower Spring */
	if(t_posDeg < VSD.lower_spring_origin){
		VSD.control_input += VSD.lower_stiffness * (VSD.lower_spring_origin - t_posDeg);
	}

	/* Lower Damper */
	if(t_posDeg < VSD.lower_damper_origin){
		if(t_velDeg < 0){
					VSD.control_input  += -VSD.lower_damper*t_velDeg;
		}
	}

	/* Upper Spring */
	if(t_posDeg > VSD.upper_spring_origin){
		VSD.control_input += VSD.upper_stiffness * (VSD.upper_spring_origin - t_posDeg);
	}

	/* Upper Damper */
	if(t_posDeg > VSD.upper_damper_origin){
		if(t_velDeg > 0){
			VSD.control_input  += -VSD.upper_damper*t_velDeg;
		}
	}

	/* Saturation */
	if      (VSD.control_input > +VSD.saturation)	{VSD.control_input = VSD.saturation;}
	else if (VSD.control_input < -VSD.saturation)	{VSD.control_input = -VSD.saturation;}

	return 0;
}

static int Ext_Compressional_SpringDamper()
{
	VSD.control_input = 0;
	return 0;
}

/*ROUTINE_ID_MIDLEVEL_COMPRESSIONAL_VSD*/
static int Run_Backlash_Test()
{
	static int flag = 0.0;

	if (mid_level_state.position > abs(backlash_test.range))
		flag = 1;
	else if (mid_level_state.position < -abs(backlash_test.range))
		flag = 0;

	if      (flag == 0) motor_in.analysis_input = +backlash_test.amplitude;
	else if (flag == 1) motor_in.analysis_input = -backlash_test.amplitude;

	return 0;
}



// Advanced System ID SBS
static int Ent_Mech_SystemID_SBS()
{
	// make frequency samples
	float f_sample = 0.0;
	float log_fmin = log10(sys_id_sbs.fmin);
	float log_fmax = log10(sys_id_sbs.fmax);
	float gap = (log_fmax - log_fmin)/(sys_id_sbs.N_samples - 1);

	// frequency initialization
	sys_id_sbs.current_f = sys_id_sbs.fmin;

	// make frequency samples that evenly distributed in log-scale
	sys_id_sbs.f_samples = (float*)malloc(sizeof(float) * sys_id_sbs.N_samples);

	if (sys_id_sbs.f_samples != NULL)
	{
		for (int i = 0; i < sys_id_sbs.N_samples; i++)
		{
			f_sample = log_fmin + gap * i;
			sys_id_sbs.f_samples[i] = powf(10, f_sample);
		}
	}
	sys_id_sbs.sys_id_cnt = 0;
	sys_id_sbs.f_cnt = 0;
	sys_id_sbs.done = 0;

	mid_level_triggered_msg_stop = 0;
	return 0;
}

static int Run_Mech_SystemID_SBS()
{
	/* End of Frequency Index */

	sys_id_sbs.current_f = sys_id_sbs.f_samples[sys_id_sbs.f_cnt];

//	motor_in.mid_id_process_input = sys_id_sbs.amp * sin(2 * M_PI * sys_id_sbs.f_samples[sys_id_sbs.f_cnt] * sys_id_sbs.sys_id_cnt * MID_LEVEL_CONTROL_PERIOD) + sys_id_sbs.offset;
	motor_in.mid_id_process_input = sys_id_sbs.amp * sin(2 * M_PI * sys_id_sbs.f_samples[sys_id_sbs.f_cnt] * sys_id_sbs.sys_id_cnt * MID_LEVEL_CONTROL_PERIOD) + sys_id_sbs.offset;

	if (sys_id_sbs.sys_id_cnt >= MID_LEVEL_CONTROL_FREQUENCY * sys_id_sbs.N_iter * (1/sys_id_sbs.current_f)){
		sys_id_sbs.sys_id_cnt = 0;
		sys_id_sbs.f_cnt++;
	}else{
		sys_id_sbs.sys_id_cnt++;
	}

	if (sys_id_sbs.f_cnt == sys_id_sbs.N_samples){
//
//		motor_in.mid_id_process_input = 0;
		velCtrl.ref = 0;
		sys_id_sbs.done = 1;
		mid_level_triggered_msg_stop = 1;
		Transition_State(&mid_level_ctrl_task.state_machine, e_State_Standby);
		Transition_State(&low_level_ctrl_task.state_machine, e_State_Standby);
	}

	return 0;
}

static int Ext_Mech_SystemID_SBS()
{
	sys_id_sbs.done = 0;
	sys_id_sbs.sys_id_cnt = 0;
	sys_id_sbs.f_cnt = 0;

	free(sys_id_sbs.f_samples);

	Transition_State(&mid_level_ctrl_task.state_machine, e_State_Standby);
	Transition_State(&low_level_ctrl_task.state_machine, e_State_Standby);

	return 0;
}
/***********************************************************************************************************/
// Advanced System ID SBS
static int Ent_Mech_SystemID_SBS_Position()
{
	// make frequency samples
	float f_sample = 0.0;
	float log_fmin = log10(sys_id_sbs.fmin);
	float log_fmax = log10(sys_id_sbs.fmax);
	float gap = (log_fmax - log_fmin)/(sys_id_sbs.N_samples - 1);

	// frequency initialization
	sys_id_sbs.current_f = sys_id_sbs.fmin;

	// make frequency samples that evenly distributed in log-scale
	sys_id_sbs.f_samples = (float*)malloc(sizeof(float) * sys_id_sbs.N_samples);

	if (sys_id_sbs.f_samples != NULL)
	{
		for (int i = 0; i < sys_id_sbs.N_samples; i++)
		{
			f_sample = log_fmin + gap * i;
			sys_id_sbs.f_samples[i] = powf(10, f_sample);
		}
	}
	sys_id_sbs.sys_id_cnt = 0;
	sys_id_sbs.f_cnt = 0;
	sys_id_sbs.done = 0;

	mid_level_triggered_msg_stop = 0;
	return 0;
}

static int Run_Mech_SystemID_SBS_Position()
{
	/* End of Frequency Index */

	sys_id_sbs.current_f = sys_id_sbs.f_samples[sys_id_sbs.f_cnt];

	posCtrl.ref  = sys_id_sbs.amp * sin(2 * M_PI * sys_id_sbs.f_samples[sys_id_sbs.f_cnt] * sys_id_sbs.sys_id_cnt * MID_LEVEL_CONTROL_PERIOD) + sys_id_sbs.offset;
	posCtrl.ref1 = sys_id_sbs.amp * sin(2 * M_PI * sys_id_sbs.f_samples[sys_id_sbs.f_cnt] * (sys_id_sbs.sys_id_cnt+1) * MID_LEVEL_CONTROL_PERIOD) + sys_id_sbs.offset;
	posCtrl.ref2 = sys_id_sbs.amp * sin(2 * M_PI * sys_id_sbs.f_samples[sys_id_sbs.f_cnt] * (sys_id_sbs.sys_id_cnt+2) * MID_LEVEL_CONTROL_PERIOD) + sys_id_sbs.offset;

	if (sys_id_sbs.sys_id_cnt >= MID_LEVEL_CONTROL_FREQUENCY * sys_id_sbs.N_iter * (1/sys_id_sbs.current_f)){
		sys_id_sbs.sys_id_cnt = 0;
		sys_id_sbs.f_cnt++;
	}else{
		sys_id_sbs.sys_id_cnt++;
	}

	if (sys_id_sbs.f_cnt == sys_id_sbs.N_samples){

		sys_id_sbs.done = 1;
		mid_level_triggered_msg_stop = 1;
		Transition_State(&mid_level_ctrl_task.state_machine, e_State_Standby);
		Transition_State(&low_level_ctrl_task.state_machine, e_State_Standby);
	}

	return 0;
}
static int Ext_Mech_SystemID_SBS_Position()
{
	sys_id_sbs.done = 0;
	sys_id_sbs.sys_id_cnt = 0;
	sys_id_sbs.f_cnt = 0;

	free(sys_id_sbs.f_samples);

	Transition_State(&mid_level_ctrl_task.state_machine, e_State_Standby);
	Transition_State(&low_level_ctrl_task.state_machine, e_State_Standby);

	return 0;
}

//
static int Ent_Friction_Compensation()
{

}
static int Run_Friction_Compensation()
{
	float sign = 0;

	//if (mid_level_state.velocity_final >= 0)	sign = +1;
	//else                                		sign = -1;

	sign = tanh(mid_level_state.velocity_final * FrictionCompObj.Epsilon);

	motor_in.friction_input = FrictionCompObj.mu_C * sign + FrictionCompObj.mu_V * mid_level_state.velocity_final;
}
static int Ext_Friction_Compensation()
{

}

// On-Link Friction ID
static int Ent_Trape_SystemID()
{
	// Make Velocity Samples
	float V_sample = 0.0;
	float log_vmin = log10(TrapeIDObj.V_min);
	float log_vmax = log10(TrapeIDObj.V_max);
	float gap = (log_vmax - log_vmin)/(TrapeIDObj.N_Samples - 1);

	// Velocity Initialization
	TrapeIDObj.Current_Velocity = TrapeIDObj.V_min;

	// make frequency samples that evenly distributed in log-scale
	TrapeIDObj.Vel_Samples     = (float*)malloc(sizeof(float)    * TrapeIDObj.N_Samples);
	TrapeIDObj.Durations       = (uint16_t*)malloc(sizeof(uint16_t) * TrapeIDObj.N_Samples);
	TrapeIDObj.SineDurations   = (uint16_t*)malloc(sizeof(uint16_t) * TrapeIDObj.N_Samples);

	if (TrapeIDObj.Vel_Samples != NULL)
	{
		for (int i = 0; i < TrapeIDObj.N_Samples; i++)
		{
			V_sample = log_vmin + gap * i;
			TrapeIDObj.Vel_Samples[i]   = powf(10, V_sample);
			TrapeIDObj.Durations[i]     = (TrapeIDObj.Amp*1000) / TrapeIDObj.Vel_Samples[i]; // 1msec count
			TrapeIDObj.SineDurations[i] = 1000* M_PI * TrapeIDObj.Delta/TrapeIDObj.Vel_Samples[i];
		}
	}
	TrapeIDObj.Cnt    = 0;
	TrapeIDObj.VelIdx = 0;
	TrapeIDObj.ON     = 1;
	TrapeIDObj.State  = 0;
	TrapeIDObj.Iter   = 0;

	TrapeIDObj.isInit = 0;
	TrapeIDObj.InitCnt = 0;
	TrapeIDObj.StartGain = 0;
	TrapeIDObj.Done = 0;
	//mid_level_triggered_msg_stop = 0;
	return 0;
}

static int Run_Trape_SystemID()
{
	/* End of Frequency Index */

	float PosRef;

	TrapeIDObj.Current_Velocity = TrapeIDObj.Vel_Samples[TrapeIDObj.VelIdx];

	if      (TrapeIDObj.State == 0) // Ascending
	{
		float t = TrapeIDObj.Cnt*0.001;

		PosRef = TrapeIDObj.Current_Velocity * t;

		if (TrapeIDObj.Cnt == TrapeIDObj.Durations[TrapeIDObj.VelIdx])
		{
			TrapeIDObj.Cnt   = 1;
			TrapeIDObj.State = 1;
		}
		else
		{
			TrapeIDObj.Cnt++;
		}
	}
	else if (TrapeIDObj.State == 1) // None-Jerk 4th Plolynomial
	{

		float t = TrapeIDObj.Cnt*0.001; // t

		float omega = TrapeIDObj.Current_Velocity/TrapeIDObj.Delta;
		PosRef = TrapeIDObj.Amp + TrapeIDObj.Delta*sin(omega*t);

		if (TrapeIDObj.Cnt == TrapeIDObj.SineDurations[TrapeIDObj.VelIdx])
		{
			TrapeIDObj.Cnt   = 1;
			TrapeIDObj.State = 2;
		}
		else
		{
			TrapeIDObj.Cnt++;
		}
	}

	else if (TrapeIDObj.State == 2) // Descending
	{
		float t = TrapeIDObj.Cnt*0.001;

		PosRef = TrapeIDObj.Amp - TrapeIDObj.Current_Velocity * t;

		if (TrapeIDObj.Cnt == 2*TrapeIDObj.Durations[TrapeIDObj.VelIdx])
		{
			TrapeIDObj.Cnt   = 1;
			TrapeIDObj.State = 3;
		}
		else
		{
			TrapeIDObj.Cnt++;
		}
	}

	else if (TrapeIDObj.State == 3) // None-Jerk 4th Plolynomial
	{
		float t = TrapeIDObj.Cnt*0.001; // t

		float omega = TrapeIDObj.Current_Velocity/TrapeIDObj.Delta;
		PosRef = -TrapeIDObj.Amp - TrapeIDObj.Delta*sin(omega*t);

		if (TrapeIDObj.Cnt == TrapeIDObj.SineDurations[TrapeIDObj.VelIdx])
		{
			TrapeIDObj.Cnt   = 1;
			TrapeIDObj.State = 4;
		}
		else
		{
			TrapeIDObj.Cnt++;
		}
	}

	else if (TrapeIDObj.State == 4) // Re-Ascending
	{
		float t = TrapeIDObj.Cnt*0.001;

		PosRef = -TrapeIDObj.Amp + TrapeIDObj.Current_Velocity * t;

		if (TrapeIDObj.Cnt == TrapeIDObj.Durations[TrapeIDObj.VelIdx])
		{
			TrapeIDObj.Cnt   = 1;
			TrapeIDObj.Iter++;

			if (TrapeIDObj.Iter == TrapeIDObj.N_Iter) {

				TrapeIDObj.VelIdx++;

				TrapeIDObj.Iter  = 0;
				TrapeIDObj.Cnt   = 0;
				TrapeIDObj.State = 0;

				if (TrapeIDObj.VelIdx == TrapeIDObj.N_Samples)
				{
					TrapeIDObj.ON = 0;
					TrapeIDObj.Done = 1;
					mid_level_triggered_msg_stop = 1;
					Transition_State(&mid_level_ctrl_task.state_machine, e_State_Standby);
					Transition_State(&low_level_ctrl_task.state_machine, e_State_Standby);
				}
			}
			else
			{
				TrapeIDObj.State = 0;
				TrapeIDObj.Cnt   = 0;
			}
		}
		else
		{
			TrapeIDObj.Cnt++;
		}
	}

	if (TrapeIDObj.isInit == 0)
	{
		TrapeIDObj.InitCnt++;

		TrapeIDObj.StartGain = (float)TrapeIDObj.InitCnt / 3000;
		if (TrapeIDObj.InitCnt == 3000)
		{
			TrapeIDObj.isInit = 1;
			TrapeIDObj.StartGain = 1;
		}
	}

	TrapeIDObj.PosRef = (PosRef + TrapeIDObj.Offset) * TrapeIDObj.StartGain;
	posCtrl.ref       = TrapeIDObj.PosRef * 0.017453292519943;

	return 0;
}

static int Ext_Trape_SystemID()
{
	TrapeIDObj.ON = 0;
	TrapeIDObj.VelIdx = 0;
	TrapeIDObj.Iter = 0;
	TrapeIDObj.Cnt   = 0;
	TrapeIDObj.State = 0;
	TrapeIDObj.isInit = 0;
	TrapeIDObj.InitCnt = 0;
	TrapeIDObj.StartGain = 0;

	free(TrapeIDObj.Durations);
	free(TrapeIDObj.SineDurations);
	free(TrapeIDObj.Vel_Samples);

	/* Done Signal */
	Transition_State(&mid_level_ctrl_task.state_machine, e_State_Standby);
	Transition_State(&low_level_ctrl_task.state_machine, e_State_Standby);


	return 0;
}
/***********************************************************************************************************/
static int Ent_P_Vector_Decoder()
{
	pvectorObj.yd_f = mid_level_state.position;

	return 0;
}

static int Run_P_Vector_Decoder()
{
	if (mid_level_state.enable_freeze == 0) {

		if ((pvectorObj.N > 0) && (pvectorObj.ON == 0))
		{
			// FIFO process
			double yd = (double)pvectorObj.p_buffer[0].yd*0.002;
			double y0 = pvectorObj.yd_f;
			double s0 =  (double)pvectorObj.p_buffer[0].s0;
			double sd = -(double)pvectorObj.p_buffer[0].sd;
			pvectorObj.yd_f = yd;

			double e  = yd - y0;

			pvectorObj.a0 = y0;
			pvectorObj.a2 =                 0.5*s0 * e;
			pvectorObj.a3 = (10 - 1.5*s0 + 0.5*sd) * e;
			pvectorObj.a4 =    (-15 + 1.5*s0 - sd) * e;
			pvectorObj.a5 =  (6 - 0.5*s0 + 0.5*sd) * e;

			pvectorObj.L     = (float)pvectorObj.p_buffer[0].L;
			pvectorObj.L_inv = 1/pvectorObj.L;

			pvectorObj.count = 0;
			pvectorObj.ON    = 1;

			if (pvectorObj.N == 1)
			{
				y0 = (double)pvectorObj.p_buffer[0].yd*0.002;
				pvectorObj.b0 = y0;
				pvectorObj.b2 = 0;
				pvectorObj.b3 = 0;
				pvectorObj.b4 = 0;
				pvectorObj.b5 = 0;

			}
			else if (pvectorObj.N > 1)
			{
				yd = (double)pvectorObj.p_buffer[1].yd*0.002;
				y0 = (double)pvectorObj.p_buffer[0].yd*0.002;
				s0 =  (double)pvectorObj.p_buffer[1].s0;
				sd = -(double)pvectorObj.p_buffer[1].sd;
				e  = yd - y0;

				pvectorObj.b0 = y0;
				pvectorObj.b2 =                 0.5*s0 * e;
				pvectorObj.b3 = (10 - 1.5*s0 + 0.5*sd) * e;
				pvectorObj.b4 =    (-15 + 1.5*s0 - sd) * e;
				pvectorObj.b5 =  (6 - 0.5*s0 + 0.5*sd) * e;
			}
	   }

		if (pvectorObj.ON == 1)
		{
			pvectorObj.t1 = (float)pvectorObj.count*pvectorObj.L_inv;    // tau^1
			if (pvectorObj.t1 > 1) pvectorObj.t1 = 1;                    // saturation for safety
			pvectorObj.t2 = pvectorObj.t1 * pvectorObj.t1;               // tau^2
			pvectorObj.t3 = pvectorObj.t2 * pvectorObj.t1;               // tau^3
			pvectorObj.t4 = pvectorObj.t3 * pvectorObj.t1;               // tau^4
			pvectorObj.t5 = pvectorObj.t4 * pvectorObj.t1;               // tau^5

			posCtrl.ref = pvectorObj.a0 + //
					      pvectorObj.a2*pvectorObj.t2 + //
					      pvectorObj.a3*pvectorObj.t3 + //
					      pvectorObj.a4*pvectorObj.t4 + //
					      pvectorObj.a5*pvectorObj.t5;  //

			if (pvectorObj.count < (pvectorObj.L - 2))
			{
				double t_t1, t_t2, t_t3, t_t4, t_t5;
				t_t1 = ((double)pvectorObj.count + 1)*pvectorObj.L_inv;
				if (t_t1 > 1) t_t1 = 1;
				t_t2 = t_t1 * t_t1;
				t_t3 = t_t2 * t_t1;
				t_t4 = t_t3 * t_t1;
				t_t5 = t_t4 * t_t1;

				posCtrl.ref1 = pvectorObj.a0 + //
						       pvectorObj.a2*t_t2 + //
						       pvectorObj.a3*t_t3 + //
						       pvectorObj.a4*t_t4 + //
						       pvectorObj.a5*t_t5;

				t_t1 = ((double)pvectorObj.count + 2)*pvectorObj.L_inv;
				if (t_t1 > 1) t_t1 = 1;
				t_t2 = t_t1 * t_t1;
				t_t3 = t_t2 * t_t1;
				t_t4 = t_t3 * t_t1;
				t_t5 = t_t4 * t_t1;

				posCtrl.ref2 = pvectorObj.a0 + //
						       pvectorObj.a2*t_t2 + //
						       pvectorObj.a3*t_t3 + //
						       pvectorObj.a4*t_t4 + //
						       pvectorObj.a5*t_t5;  //

				posFF.in[0] = posCtrl.ref1;
			}
			else if (pvectorObj.count == (pvectorObj.L - 2))
			{
				double t_t1, t_t2, t_t3, t_t4, t_t5;
				t_t1 = ((double)pvectorObj.count + 1)*pvectorObj.L_inv;
				if (t_t1 > 1) t_t1 = 1;
				t_t2 = t_t1 * t_t1;
				t_t3 = t_t2 * t_t1;
				t_t4 = t_t3 * t_t1;
				t_t5 = t_t4 * t_t1;

				posCtrl.ref1 = pvectorObj.a0 + //
						       pvectorObj.a2*t_t2 + //
						       pvectorObj.a3*t_t3 + //
						       pvectorObj.a4*t_t4 + //
						       pvectorObj.a5*t_t5; //

				posCtrl.ref2 = (double)pvectorObj.p_buffer[0].yd*0.002;

				posFF.in[0] = posCtrl.ref1;

			}
			else if (pvectorObj.count == (pvectorObj.L - 1))
			{
				posCtrl.ref1 = (double)pvectorObj.p_buffer[0].yd*0.002;

				double t_t1, t_t2, t_t3, t_t4, t_t5;
				t_t1 = (double)1/pvectorObj.p_buffer[1].L;
				if (t_t1 > 1) t_t1 = 1;
				t_t2 = t_t1 * t_t1;
				t_t3 = t_t2 * t_t1;
				t_t4 = t_t3 * t_t1;
				t_t5 = t_t4 * t_t1;

				posCtrl.ref2 = pvectorObj.b0 + //
						       pvectorObj.b2*t_t2 + //
						       pvectorObj.b3*t_t3 + //
						       pvectorObj.b4*t_t4 + //
						       pvectorObj.b5*t_t5;  //

				posFF.in[0] = posCtrl.ref1;
			}

			pvectorObj.count++;

			if (pvectorObj.count >= pvectorObj.L)
			{
				pvectorObj.ON = 0;
				pvectorObj.count  = 0;

				// FIFO shifting
				for (int i = 0; i < pvectorObj.N; i++)
				{
					memcpy(&pvectorObj.p_buffer[i], &pvectorObj.p_buffer[i+1], sizeof(P_Vector));
				}
				pvectorObj.N--;
			}
		}
	}

	return 0;
}

static int Ext_P_Vector_Decoder()
{
	for(int i = 0; i < P_VECTOR_BUFF_SIZE; ++i){
		pvectorObj.p_buffer[i].yd = 0;
		pvectorObj.p_buffer[i].L = 0;
		pvectorObj.p_buffer[i].s0 = 0;
		pvectorObj.p_buffer[i].sd = 0;
	}
	return 0;
}
/* F Vector-based Torque Generation */
static int Run_F_Vector_Decoder()
{
	fvectorObj.input = 0;

	for (int i = 0; i < F_VECTOR_BUFF_SIZE; i++)
	{
		// Step 1. Check time delay
		if (fvectorObj.f_buffer[i].is_full)
		{
			if (fvectorObj.f_buffer[i].time_stamp == fvectorObj.f_buffer[i].delay)
			{
				fvectorObj.f_buffer[i].u = fvectorObj.f_buffer[i].tau_max * 0.01;
				//fvectorObj.f_buffer[i].time_stamp = 0; //reset time stamp, and restart to measure duration
			}
		}

		// (Step2) Calculate torque and sum
		uint8_t t_idx = fvectorObj.f_buffer[i].mode_idx;

		fvectorObj.f_buffer[i].tau = fvectorObj.mode_param[t_idx].b1 * fvectorObj.f_buffer[i].tau_old1 + \
				                     fvectorObj.mode_param[t_idx].b2 * fvectorObj.f_buffer[i].tau_old2 + \
									 fvectorObj.mode_param[t_idx].a0 * fvectorObj.f_buffer[i].u + \
									 fvectorObj.mode_param[t_idx].a1 * fvectorObj.f_buffer[i].u_old1 + \
									 fvectorObj.mode_param[t_idx].a2 * fvectorObj.f_buffer[i].u_old2;

		fvectorObj.f_buffer[i].tau_old2 = fvectorObj.f_buffer[i].tau_old1;
		fvectorObj.f_buffer[i].tau_old1 = fvectorObj.f_buffer[i].tau;
		fvectorObj.f_buffer[i].u_old2   = fvectorObj.f_buffer[i].u_old1;
		fvectorObj.f_buffer[i].u_old1   = fvectorObj.f_buffer[i].u;
		fvectorObj.f_buffer[i].u = 0;

		fvectorObj.input += fvectorObj.f_buffer[i].tau;


		// Step 3. Update times
		if (fvectorObj.f_buffer[i].is_full)
		{
			fvectorObj.f_buffer[i].time_stamp++;
		}

		// Step 4. Check F-vector erase condition
		if (fvectorObj.f_buffer[i].time_stamp >= (fvectorObj.f_buffer[i].t_end + fvectorObj.f_buffer[i].delay))
		{
			memset(&fvectorObj.f_buffer[i], 0, sizeof(F_Vector));
			fvectorObj.f_buffer[i].is_full = 0;
		}
	}
	motor_in.f_vector_input = fvectorObj.input;

	return 0;
}

static int Ext_F_Vector_Decoder()
{
	motor_in.f_vector_input = 0;

	for(int i = 0; i < F_VECTOR_BUFF_SIZE; ++i){
		fvectorObj.f_buffer[i].u = 0;
		fvectorObj.f_buffer[i].u_old1 = 0;
		fvectorObj.f_buffer[i].u_old2 = 0;
		fvectorObj.f_buffer[i].tau = 0;
		fvectorObj.f_buffer[i].tau_old1 = 0;
		fvectorObj.f_buffer[i].tau_old2 = 0;
	}
	return 0;
}


static int Run_Cos3_Ref_Decoder()
{
	if (cos3vectorObj.ON == 1)
	{

		cos3vectorObj.y  = Generate_Cosine_3(cos3vectorObj.Amplitude, cos3vectorObj.Period, cos3vectorObj.cnt, MID_LEVEL_CONTROL_PERIOD, 0);
		cos3vectorObj.y1 = Generate_Cosine_3(cos3vectorObj.Amplitude, cos3vectorObj.Period, cos3vectorObj.cnt, MID_LEVEL_CONTROL_PERIOD, 1);

		if (cos3vectorObj.cnt > MID_LEVEL_CONTROL_FREQUENCY * cos3vectorObj.Period)
		{
			cos3vectorObj.cnt = 0;
			cos3vectorObj.ON  = 0;
		}
		else
		{
			cos3vectorObj.cnt++;
		}
	}
	else
	{
		cos3vectorObj.y = 0;
		cos3vectorObj.y1 = 0;
	}

	return 0;
}


//static int Ent_Feedforward_Filter()
//{
//	memset(posFF.in, 0, sizeof(posFF.in));
//	memset(posFF.out, 0, sizeof(posFF.out));
//	posFF.diff = posFF.num_length - posFF.den_length;
//	posFF.in[0] = mid_level_state.position;
//	posFF.in[1] = mid_level_state.position;
//	posFF.in[2] = mid_level_state.position;
//	posFF.in[3] = mid_level_state.position;
//
//	posFF.control_input = 0;
//
//	return 0;
//}
//
//static int Run_Feedforward_Filter()
//{
//	double t_in_term = 0.0, t_out_term = 0.0;
//
//	for(int i = 0; i < posFF.num_length; ++i){
//		t_in_term += (double)posFF.num[i]*(double)posFF.in[i];
//	}
//	for(int i = 1; i < posFF.den_length; ++i){
//		t_out_term += (double)posFF.den[i]*(double)posFF.out[i];
//	}
//	float t_control_input = -t_out_term + t_in_term;
//
//	//
//	if (posFF.overall_gain_trigger == 1)
//	{
//		posFF.overall_gain_curr = posFF.overall_gain_curr + posFF.overall_gain_gap;
//		if (posFF.overall_gain_time_stamp == MID_LEVEL_CONTROL_FREQUENCY * posFF.overall_gain_transition_time)
//		{
//			posFF.overall_gain_gap        = 0;
//			posFF.overall_gain_time_stamp = 0;
//			posFF.overall_gain_trigger    = 0;
//			posFF.overall_gain_curr       = posFF.overall_gain_des;
//		}
//		posFF.overall_gain_time_stamp++;
//
//		if (posFF.overall_gain_curr > 1) posFF.overall_gain_curr = 1;
//		if (posFF.overall_gain_curr < 0) posFF.overall_gain_curr = 0;
//	}
//	posFF.control_input = t_control_input * posFF.overall_gain_curr;
//	/*if (pos)
//	posFF.overall_gain_curr*/
//
//	posFF.out[0] = posFF.control_input;
//
//	/* Array Shifting */
//	for(int i = posFF.num_length-1; i > 0; --i){
//		posFF.in[i] = posFF.in[i-1];
//	}
//	for(int i = posFF.den_length-1; i > 0; --i){
//		posFF.out[i] = posFF.out[i-1];
//	}
//
//	return 0;
//}
//
//static int Ext_Feedforward_Filter()
//{
//	posFF.diff = 0;
//	return 0;
//}

static int Run_System_ID_Verify()
{
	static uint16_t t_identifier = 0;
	static float t_temp_arr[3] = {0};

	if(mid_level_loop_cnt <= 2000){
		motor_in.mid_id_process_input = sys_id_sbs.verify_mag*rgs_input[mid_level_loop_cnt];

		memcpy(&t_temp_arr[0], &mid_level_loop_cnt, 4);
		memcpy(&t_temp_arr[1], &motor_in.mid_id_process_input, 4);
		memcpy(&t_temp_arr[2], &mid_level_state.velocity_raw, 4);

		t_identifier = GUI_SYNC|GET_SYSTEM_ID_VERIFY;
		Send_MSG(t_identifier, 12, (uint8_t*)t_temp_arr);
	} else {
		Transition_State(&mid_level_ctrl_task.state_machine, e_State_Standby);
		Transition_State(&low_level_ctrl_task.state_machine, e_State_Standby);
		motor_in.mid_id_process_input = 0;
	}

	return 0;
}

/*ROUTINE_ID_CORRIDOR_IMPEDANCE_CONTROL */
//static void error_filter2(ImpedanceCtrl *t_impedanceCtrl)
//{
//	// f2(e,t) = lambda * e + (1 - lambda)*sign(e)*max(|e| - epsilon, 0)
//	float t_abs_e = 0.0;
//	float t_sign_e = 0.0;
//	float t_max = 0.0;
//	float t_diff = 0.0;
//	float y_ef = 0.0;
//
//	/* Calculate 'sign(e) & |e|' */
//	if (t_impedanceCtrl->e > 0) {t_abs_e = +t_impedanceCtrl->e; t_sign_e = +1; }
//	else                        {t_abs_e = -t_impedanceCtrl->e; t_sign_e = -1; }
//
//	/* Calculate 'max(|e| - epsilon, 0)' */
//	t_diff = t_abs_e - t_impedanceCtrl->epsilon;
//	if (t_diff > 0) {t_max = t_diff;}
//	else            {t_max = 0;}
//
//	y_ef = (t_impedanceCtrl->lambda * t_impedanceCtrl->e) + (1 - t_impedanceCtrl->lambda)*t_sign_e*t_max;
//
//	t_impedanceCtrl->ef_diff = (y_ef - t_impedanceCtrl->ef) * MID_LEVEL_CONTROL_FREQUENCY;
//
//	t_impedanceCtrl->ef   = y_ef;
//}
//
//static int Ent_Corridor_Impedance_Control()
//{
//	if (impedanceCtrl.Kp_max == 0)
//		impedanceCtrl.Kp_max = 10;
//
//	if (impedanceCtrl.Kd_max == 0)
//		impedanceCtrl.Kd_max = 10;
//
//	if (isnan(impedanceCtrl.Kp_max))
//		impedanceCtrl.Kp_max = 10;
//
//	if (isnan(impedanceCtrl.Kd_max))
//		impedanceCtrl.Kd_max = 10;
//
//	if (isnanf(impedanceCtrl.option))
//		impedanceCtrl.option = 0;
//
//	if ((impedanceCtrl.option != 0) && (impedanceCtrl.option != 1))
//		impedanceCtrl.option = 0;
//
//
//	if (impedanceCtrl.option == 1)
//	{
//		impedanceCtrl.epsilon = impedanceCtrl.opt1_i_buffer.epsilon_target * 0.001745329252;
//		impedanceCtrl.Kp = impedanceCtrl.opt1_i_buffer.Kp_target * impedanceCtrl.Kp_max * 0.392156862745098;
//		impedanceCtrl.Kd = impedanceCtrl.opt1_i_buffer.Kd_target * impedanceCtrl.Kd_max * 0.392156862745098;
//		impedanceCtrl.lambda = impedanceCtrl.opt1_i_buffer.lambda_target * 0.01;
//	}
//
//
//	return 0;
//}
//static int Run_Corridor_Impedance_Control()
//{
//	float t_epsilon = 0.0;
//	float t_Kp = 0.0;
//	float t_Kd = 0.0;
//	float t_lambda = 0.0;
//
//	if (impedanceCtrl.option == 0) // Variable I-Vector
//	{
//		if ((impedanceCtrl.N > 0) && (impedanceCtrl.ON == 0))
//		{
//			// FIFO process
//			t_epsilon = (float)impedanceCtrl.i_buffer[0].epsilon_target * 0.1745329252; // unit: rad   (0.001745329252 = 0.1 * pi/180)
//			t_Kp      = (float)impedanceCtrl.i_buffer[0].Kp_target * impedanceCtrl.Kp_max * 0.003921568627451; //0.57295779513;      // unit: A/rad (0.57295779513 = 0.01 * 180/pi)
//			t_Kd      = (float)impedanceCtrl.i_buffer[0].Kd_target * impedanceCtrl.Kd_max * 0.003921568627451; //     // unit: A/rad (0.57295779513 = 0.01 * 180/pi)
//			t_lambda  = (float)impedanceCtrl.i_buffer[0].lambda_target * 0.01;
//			impedanceCtrl.L = (float)impedanceCtrl.i_buffer[0].duration;
//
//			if (impedanceCtrl.L > 0)
//			{
//				float invT      = 1/impedanceCtrl.L;
//
//				impedanceCtrl.gap_epsilon = (t_epsilon - impedanceCtrl.epsilon) * invT;
//				impedanceCtrl.gap_Kp      = (t_Kp      - impedanceCtrl.Kp)      * invT;
//				impedanceCtrl.gap_Kd      = (t_Kd      - impedanceCtrl.Kd)      * invT;
//				impedanceCtrl.gap_lambda  = (t_lambda  - impedanceCtrl.lambda)  * invT;
//			}
//
//			impedanceCtrl.i  = 0; // initialize 1ms counter
//			impedanceCtrl.ON = 1;
//		}
//
//		if (impedanceCtrl.ON == 1)
//		{
//			if (impedanceCtrl.L == 0)
//			{
//				impedanceCtrl.epsilon = t_epsilon;
//				impedanceCtrl.Kp      = t_Kp;
//				impedanceCtrl.Kd      = t_Kd;
//				impedanceCtrl.lambda  = t_lambda;
//			}
//			else
//			{
//				impedanceCtrl.epsilon = impedanceCtrl.epsilon + impedanceCtrl.gap_epsilon;
//				impedanceCtrl.Kp      = impedanceCtrl.Kp      + impedanceCtrl.gap_Kp;
//				impedanceCtrl.Kd      = impedanceCtrl.Kd      + impedanceCtrl.gap_Kd;
//				impedanceCtrl.lambda  = impedanceCtrl.lambda  + impedanceCtrl.gap_lambda;
//				impedanceCtrl.i++;
//			}
//
//			if (impedanceCtrl.i >= impedanceCtrl.L)
//			{
//				impedanceCtrl.ON = 0;
//				impedanceCtrl.i = 0;
//
//				// FIFO shifting
//				for (int i = 0; i < impedanceCtrl.N; i++)
//				{
//					memcpy(&impedanceCtrl.i_buffer[i], &impedanceCtrl.i_buffer[i+1], sizeof(I_Vector));
//				}
//				impedanceCtrl.N--;
//			}
//		}
//	}
//
//	/* Impedance Controller */
//	posCtrl.total_ref = posCtrl.ref + cos3vectorObj.y;
//	impedanceCtrl.e = posCtrl.total_ref - mid_level_state.position;
//
//	error_filter2(&impedanceCtrl);
//
//	float t_ef_diff = 0.0;
//
//	if (((impedanceCtrl.ef > 0) & (impedanceCtrl.ef_diff > 0)) | ((impedanceCtrl.ef <= 0) & (impedanceCtrl.ef_diff <= 0)))
//	{
//		t_ef_diff = +impedanceCtrl.ef_diff;
//	}
//	else
//	{
//		//t_ef_diff = -impedanceCtrl.ef_diff;
//		t_ef_diff = +impedanceCtrl.ef_diff;
//	}
//
//
//
//	float t_control_input;
//	t_control_input = impedanceCtrl.Kp * impedanceCtrl.ef + impedanceCtrl.Kd * t_ef_diff;
//
//	if (impedanceCtrl.overall_gain_trigger == 1)
//	{
//		impedanceCtrl.overall_gain_curr = impedanceCtrl.overall_gain_curr + impedanceCtrl.overall_gain_gap;
//		if (impedanceCtrl.overall_gain_time_stamp == MID_LEVEL_CONTROL_FREQUENCY * impedanceCtrl.overall_gain_transition_time)
//		{
//			impedanceCtrl.overall_gain_gap        = 0;
//			impedanceCtrl.overall_gain_time_stamp = 0;
//			impedanceCtrl.overall_gain_trigger    = 0;
//			impedanceCtrl.overall_gain_curr       = impedanceCtrl.overall_gain_des;
//		}
//		impedanceCtrl.overall_gain_time_stamp++;
//
//		if (impedanceCtrl.overall_gain_curr > 1) impedanceCtrl.overall_gain_curr = 1;
//		if (impedanceCtrl.overall_gain_curr < 0) impedanceCtrl.overall_gain_curr = 0;
//	}
//	impedanceCtrl.control_input = t_control_input * impedanceCtrl.overall_gain_curr;
//
//	return 0;
//}

/*ROUTINE_ID_MIDLEVEL_CURRENT_SINE_REF*/
static int Ent_Generate_Current_Sine() {return 0;}
static int Run_Generate_Current_Sine()
{
	motor_in.mid_id_process_input = Generate_Sine(cur_periodic_sig.amp, cur_periodic_sig.offset, cur_periodic_sig.freq, mid_level_loop_cnt, MID_LEVEL_CONTROL_PERIOD, 0);

	return 0;
}

/*ROUTINE_ID_MIDLEVEL_VELOCITY_SINE_REF*/
static int Ent_Generate_Velocity_Sine() {return 0;}
static int Run_Generate_Velocity_Sine()
{
	velCtrl.ref = Generate_Sine(vel_periodic_sig.amp, 0, vel_periodic_sig.freq, mid_level_loop_cnt, MID_LEVEL_CONTROL_PERIOD, 0);

	return 0;
}

/*ROUTINE_ID_MIDLEVEL_POSITION_SINE_REF*/
static int Ent_Generate_Position_Sine()
{
	posCtrl.t_ref=0;
	return 0;
}
static int Run_Generate_Position_Sine()
{
	/* For PD controller */
	posCtrl.t_ref = Generate_Sine(pos_periodic_sig.amp, 0, pos_periodic_sig.freq, \
								mid_level_loop_cnt, MID_LEVEL_CONTROL_PERIOD, 0);

	/*
	posCtrl.ref = Generate_Sine(pos_periodic_sig.amp,  pos_periodic_sig.offset, pos_periodic_sig.freq, \
								mid_level_loop_cnt, MID_LEVEL_CONTROL_PERIOD, 0); */

	/* For Feedforward */
	if(posFF.diff != 0){
		for(int i = 0; i < posFF.diff; ++i){
			posFF.in[i] = Generate_Sine(pos_periodic_sig.amp, 0, pos_periodic_sig.freq, \
										mid_level_loop_cnt, MID_LEVEL_CONTROL_PERIOD, posFF.diff-i);
		}
		posFF.in[posFF.diff] = posCtrl.t_ref;
	}

	/* For PD controller */
//	velCtrl.ref = Generate_Sine(pos_periodic_sig.amp, 0, pos_periodic_sig.freq, \
//								mid_level_loop_cnt, MID_LEVEL_CONTROL_PERIOD, 0);



	return 0;
}

static int Ent_Generate_Position_Sine_without_Offset()
{
	posCtrl.time_stamp = 0;
	return 0;
}
static int Run_Generate_Position_Sine_without_Offset()
{
	float Gain = ((float)posCtrl.time_stamp)*0.0002;
	if (Gain > 1) Gain = 1;
	if (posCtrl.time_stamp < 5000)
		posCtrl.time_stamp++;

	posCtrl.ref = Gain*Generate_Sine(pos_periodic_sig.amp, pos_periodic_sig.offset, pos_periodic_sig.freq, \
									mid_level_loop_cnt, MID_LEVEL_CONTROL_PERIOD, 0);

	posFF.in[0] = Gain*Generate_Sine(pos_periodic_sig.amp, pos_periodic_sig.offset, pos_periodic_sig.freq, \
			                    mid_level_loop_cnt, MID_LEVEL_CONTROL_PERIOD, 1);

	return 0;
}

/*ROUTINE_ID_MIDLEVEL_POSITION_SINE_REF*/
static int Ent_Generate_Position_Trape()
{
	//mid_level_state.initial_pos = mid_level_state.position;
		return 0;
}
static int Run_Generate_Position_Trape()
{
	float gain;
	static int stamp;

	gain = (float)stamp/2000;
	if (gain > 1) gain = 1;

	if (stamp < 2000)
	{
		stamp++;
	}

	posCtrl.ref = gain * Generate_Trapezoidal(pos_periodic_sig.amp, pos_periodic_sig.offset, pos_periodic_sig.freq, \
								mid_level_loop_cnt, MID_LEVEL_CONTROL_PERIOD, 0);


	posFF.in[0] = gain * Generate_Trapezoidal(pos_periodic_sig.amp, pos_periodic_sig.offset, pos_periodic_sig.freq, \
			                    mid_level_loop_cnt, MID_LEVEL_CONTROL_PERIOD, 1)	\
			                     - mid_level_state.initial_pos;
	/* For PD controller */
/*	posCtrl.ref = gain * Generate_Trapezoidal(pos_periodic_sig.amp, mid_level_state.initial_pos + pos_periodic_sig.offset, pos_periodic_sig.freq, \
								mid_level_loop_cnt, MID_LEVEL_CONTROL_PERIOD, 0);


	posFF.in[0] = gain * Generate_Trapezoidal(pos_periodic_sig.amp, mid_level_state.initial_pos + pos_periodic_sig.offset, pos_periodic_sig.freq, \
			                    mid_level_loop_cnt, MID_LEVEL_CONTROL_PERIOD, 1)	\
			                     - mid_level_state.initial_pos;*/


	return 0;
}

/*ROUTINE_ID_MIDLEVEL_CURRENT_TANH_REF*/
static int Ent_Generate_Current_Tanh() {return 0;}
static int Run_Generate_Current_Tanh()
{
	//velCtrl.ref = Generate_Rectangle(vel_periodic_sig.amp, 0, vel_periodic_sig.freq, 0.5, mid_level_loop_cnt, MID_LEVEL_CONTROL_PERIOD, 1);
	motor_in.mid_id_process_input = Generate_Rectangle_tanh(cur_periodic_sig.amp, cur_periodic_sig.offset, cur_periodic_sig.freq, mid_level_loop_cnt, MID_LEVEL_CONTROL_PERIOD, 0);

	return 0;
}

/*ROUTINE_ID_MIDLEVEL_CURRENT_REC_REF*/
static int Ent_Generate_Current_Rec() {return 0;}
static int Run_Generate_Current_Rec()
{
	motor_in.mid_id_process_input = Generate_Rectangle(cur_periodic_sig.amp, cur_periodic_sig.offset, cur_periodic_sig.freq, 0.5, mid_level_loop_cnt, MID_LEVEL_CONTROL_PERIOD, 1);

	return 0;
}

/*ROUTINE_ID_MIDLEVEL_VELOCITY_REC_REF*/
static int Ent_Generate_Velocity_Rectangle() {return 0;}
static int Run_Generate_Velocity_Rectangle()
{
	//velCtrl.ref = Generate_Rectangle(vel_periodic_sig.amp, 0, vel_periodic_sig.freq, 0.5, mid_level_loop_cnt, MID_LEVEL_CONTROL_PERIOD, 1);
	velCtrl.ref = Generate_Rectangle_tanh(vel_periodic_sig.amp, 0, vel_periodic_sig.freq, mid_level_loop_cnt, MID_LEVEL_CONTROL_PERIOD, 0);

	return 0;
}

/*ROUTINE_ID_MIDLEVEL_POSITION_REC_REF*/
static int Ent_Generate_Position_Rectangle()
{
	mid_level_state.initial_pos = mid_level_state.position;
	return 0;
}

int32_t t_lead = 0;

static int Run_Generate_Position_Rectangle()
{
	/* For PD controller */
	posCtrl.ref = Generate_Rectangle_tanh(pos_periodic_sig.amp, mid_level_state.initial_pos + pos_periodic_sig.offset, pos_periodic_sig.freq, \
										  mid_level_loop_cnt, MID_LEVEL_CONTROL_PERIOD, 0);

	/* For Feedforward */

	posFF.in[0] = Generate_Rectangle_tanh(pos_periodic_sig.amp, mid_level_state.initial_pos + pos_periodic_sig.offset, pos_periodic_sig.freq,\
			                              mid_level_loop_cnt, MID_LEVEL_CONTROL_PERIOD, 1)	\
				                          - mid_level_state.initial_pos;

	return 0;
}

/*ROUTINE_ID_MIDLEVEL_GET_HALL_SENSOR_VALUE*/
static int Run_Send_IOIF_Hall_t_Values_to_GUI()
{
	Read_Hall_Sensors(&hallObj);

	static uint16_t t_identifier = 0;
	static uint32_t t_temp_arr[5] = {0};

	memcpy(&t_temp_arr[0], &mid_level_loop_cnt, 4);
	memcpy(&t_temp_arr[1], &hallObj.H1, 1);
	memcpy(&t_temp_arr[2], &hallObj.H2, 1);
	memcpy(&t_temp_arr[3], &hallObj.H3, 1);
	memcpy(&t_temp_arr[4], &hallObj.hall_logic, 1);

	t_identifier = GUI_SYNC|GET_HALLSENSOR;
	Send_MSG(t_identifier, 20, (uint8_t*)t_temp_arr);

	return 0;
}


/*ROUTINE_ID_MIDLEVEL_GET_ENCODER_VALUE*/
static int Run_Send_IOIF_IncEnc_t_Values_to_GUI()
{
	static uint16_t t_identifier = 0;
	static float t_temp_arr[3] = {0};

	memcpy(&t_temp_arr[0], &mid_level_loop_cnt, 4);
	memcpy(&t_temp_arr[1], &inc1KhzObj.userCnt, 4);
	memcpy(&t_temp_arr[2], &mid_level_state.position, 4);

	t_identifier = GUI_SYNC|GET_INCENCODER;
	Send_MSG(t_identifier, 12, (uint8_t*)t_temp_arr);

	return 0;
}

/*ROUTINE_ID_MIDLEVEL_GET_ABSENCODER1_VALUE*/
static int Run_Send_IOIF_AbsEnc1_t_Values_to_GUI()
{
	static uint16_t t_identifier = 0;
	static float t_temp_arr[3] = {0};

	memcpy(&t_temp_arr[0], &mid_level_loop_cnt, 4);
	memcpy(&t_temp_arr[1], &AbsObj1.posDeg, 4);
	memcpy(&t_temp_arr[2], &AbsObj1.offset, 4);

	t_identifier = GUI_SYNC|GET_ABSENCODER1;
	Send_MSG(t_identifier, 12, (uint8_t*)t_temp_arr);

	return 0;
}

/*ROUTINE_ID_MIDLEVEL_GET_ABSENCODER2_VALUE*/
static int Run_Send_IOIF_AbsEnc2_t_Values_to_GUI()
{
	static uint16_t t_identifier = 0;
	static float t_temp_arr[3] = {0};

	memcpy(&t_temp_arr[0], &mid_level_loop_cnt, 4);
	memcpy(&t_temp_arr[1], &AbsObj2.posDeg, 4);
	memcpy(&t_temp_arr[2], &AbsObj2.offset, 4);

	t_identifier = GUI_SYNC|GET_ABSENCODER2;
	Send_MSG(t_identifier, 12, (uint8_t*)t_temp_arr);

	return 0;
}

static void Run_Convert_BatCurr_Value_to_UINT16()
{
	static uint32_t out_def_cnt = 0;

}

static int Ent_TVCF_Verification()
{
	mid_level_state.initial_pos = mid_level_state.position;

	TVCF_ver.delta_freq = (TVCF_ver.fmax - TVCF_ver.fmin) / (TVCF_ver.N_samples - 1);
	TVCF_ver.current_f = TVCF_ver.fmin;

	TVCF_ver.f_samples = (float*)malloc(sizeof(float) * TVCF_ver.N_samples);

	if (TVCF_ver.f_samples != NULL)
	{
		for (int i = 0; i < TVCF_ver.N_samples; i++)
		{
			TVCF_ver.f_samples[i] = TVCF_ver.delta_freq * i + TVCF_ver.fmin;
		}
	}
	return 0;

	TVCF_ver.ver_cnt = 0;
	TVCF_ver.f_cnt = 0;
	TVCF_ver.done = 0;
}

static int Run_TVCF_Verification()
{
	TVCF_ver.current_f = TVCF_ver.f_samples[TVCF_ver.f_cnt];

	posCtrl.ref = TVCF_ver.amp * sin(2 * M_PI * TVCF_ver.f_samples[TVCF_ver.f_cnt] * TVCF_ver.ver_cnt * MID_LEVEL_CONTROL_PERIOD) + TVCF_ver.offset;

	if (TVCF_ver.ver_cnt >= MID_LEVEL_CONTROL_FREQUENCY * TVCF_ver.N_iter * (1/TVCF_ver.current_f)) {
		TVCF_ver.ver_cnt = 0;
		TVCF_ver.f_cnt ++;
	}
	else {
		TVCF_ver.ver_cnt ++;
	}

	if (TVCF_ver.f_cnt == TVCF_ver.N_samples) {
		motor_in.mid_ctrl_input = 0;
		TVCF_ver.done ++;
	}

	if (TVCF_ver.done >= 1) {
		Transition_State(&mid_level_ctrl_task.state_machine, e_State_Standby);
		Transition_State(&low_level_ctrl_task.state_machine, e_State_Standby);
	}

	posFF.in[0] = Generate_Sine(TVCF_ver.amp, mid_level_state.initial_pos + TVCF_ver.offset, TVCF_ver.current_f, \
			                    mid_level_loop_cnt, MID_LEVEL_CONTROL_PERIOD, 1)	\
			                    - mid_level_state.initial_pos;

	return 0;
}

static int Ext_TVCF_Verification()
{
	TVCF_ver.ver_cnt = 0;
	TVCF_ver.f_cnt = 0;

	free(TVCF_ver.f_samples);
	// Send_MSG((uint16_t)(GUI_SYNC|TVCF_VERIFICATION_DONE), 1, (uint8_t*)0);
	Transition_State(&mid_level_ctrl_task.state_machine, e_State_Standby);
	Transition_State(&low_level_ctrl_task.state_machine, e_State_Standby);
	mid_level_triggered_msg_stop = 1;

	return 0;
}

//////////////////////  Flexi-SEA   //////////////////////
///////////////////////////////////////////////////////////
static void Set_Position_Tanh_Sig_Info(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&refTanh.amp, 		t_req->data, 			 4);
	memcpy(&refTanh.a, 		((float*)t_req->data)+1, 4);
	memcpy(&refTanh.td, 		((float*)t_req->data)+2, 4);
   t_res->size = 0;
   t_res->status = DATA_OBJECT_SDO_SUCC;
}
static void Set_Ankle_Onoff(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&refAnk.onoff, req->data, 1);

	res->size = 0;
	res->status = DATA_OBJECT_SDO_SUCC;
}
static void Set_Ankle_REF_Sig_Info(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&refAnk.amp_P, 		t_req->data, 			 4);
	memcpy(&refAnk.gc[0], 		((float*)t_req->data)+1, 4);
	memcpy(&refAnk.gc[1], 		((float*)t_req->data)+2, 4);
	memcpy(&refAnk.gc[2], 		((float*)t_req->data)+3, 4);
   t_res->size = 0;
   t_res->status = DATA_OBJECT_SDO_SUCC;
}
static void Set_Ankle_REF_Sig_Info2(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&refAnk.amp_D, 		((float*)t_req->data), 4);
	memcpy(&refAnk.amp_D2, 		((float*)t_req->data)+1, 4);
	memcpy(&refAnk.gc[3], 		((float*)t_req->data)+2, 4);
	memcpy(&refAnk.gc[4], 		((float*)t_req->data)+3, 4);

   t_res->size = 0;
   t_res->status = DATA_OBJECT_SDO_SUCC;
}
static void Set_Ankle_REF_Sig_Info3(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&refAnk.gc[5], 		((float*)t_req->data), 4);
	memcpy(&refAnk.gc[6], 		((float*)t_req->data)+1, 4);

   t_res->size = 0;
   t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_Risk_Param(MsgSDOargs* t_req, MsgSDOargs* t_res){
	memcpy(&RISK_Flexi.max_error, 		((float*)t_req->data), 4);
	memcpy(&RISK_Flexi.max_ank_ang, 		((float*)t_req->data)+1, 4);
	memcpy(&RISK_Flexi.min_ank_ang, 		((float*)t_req->data)+2, 4);
}
static void Set_Init_Torque(){
	spring_state.initial_pos = spring_state.position;
}
static void Set_Phase_Shift(MsgSDOargs* t_req, MsgSDOargs* t_res){
	memcpy(&refAnk.phase_shift, 		((float*)t_req->data), 			 4);
}

static void Set_ImpedanceCtrl_Info(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&impedanceCtrl.Kp, 		((float*)t_req->data), 4);
	memcpy(&impedanceCtrl.Kd, 		((float*)t_req->data)+1, 4);
	memcpy(&impedanceCtrl.lambda, 		((float*)t_req->data)+2, 4);
	memcpy(&impedanceCtrl.epsilon, 		((float*)t_req->data)+3, 4);

	   t_res->size = 0;
	   t_res->status = DATA_OBJECT_SDO_SUCC;
}

static void Set_ProportionalCtrl_Info(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&proportionalCtrl.K_torque, 			((float*)t_req->data), 4);
	memcpy(&proportionalCtrl.max_torque, 		((float*)t_req->data)+1, 4);
	memcpy(&proportionalCtrl.power_PF, 			((float*)t_req->data)+2, 4);
	memcpy(&proportionalCtrl.power_DF, 			((float*)t_req->data)+3, 4);

	   t_res->size = 0;
	   t_res->status = DATA_OBJECT_SDO_SUCC;
}
/********************************Routine****************************************/
static int Ent_Position_Ctrl()
{
	posCtrl.err = 0;
	posCtrl.err_sum = 0;
	posCtrl.err_diff = 0;


	return 0;
}
static int Run_Position_Ctrl()
{
	posCtrl.ref = posCtrl.t_ref;
	Run_PID_Control(&posCtrl, posCtrl.ref, load_cell.loadcell_filtered, MID_LEVEL_CONTROL_PERIOD);
	return 0;
}

static int Ent_Position_Ctrl_Ankle()
{
	posCtrlAnk.err = 0;
	posCtrlAnk.err_sum = 0;
	posCtrlAnk.err_diff = 0;

	return 0;
}
static int Run_Position_Ctrl_Ankle()
{
	Run_PID_Control(&posCtrlAnk, posCtrlAnk.ref, spring_state.position-spring_state.initial_pos, MID_LEVEL_CONTROL_PERIOD);
	return 0;
}
static int Ent_Disturbance_Obs()
{
	memset(posDOB.q_in, 0, sizeof(posDOB.q_in));
	memset(posDOB.gq_in, 0, sizeof(posDOB.gq_in));
	memset(posDOB.q_out, 0, sizeof(posDOB.q_out));
	memset(posDOB.gq_out, 0, sizeof(posDOB.gq_out));

//	posDOB.initial_pos = spring_state.position;
	posDOB.time_stamp = 0;
	posDOB.transition_time = CONTROLLER_TRANSITION_DURATION;

	return 0;
}
static int Run_Disturbance_Obs()
{

	/* k-th input */
//	posDOB.saturation=mid_ctrl_saturation;
//	posDOB.q_in[0] = motor_in.mid_ctrl_input; //tau_m
//	posDOB.gq_in[0] = load_cell.loadcell_filtered; // force

	///// 6Hz /////
//	posDOB.q_out[0] =+1.9260053067939*posDOB.q_out[1]-0.92737411044961*posDOB.q_out[2]+0.012135677106153*posDOB.gq_in[0]-0.023957462278146*posDOB.gq_in[1]+0.011861436320903*posDOB.gq_in[2]-0.0013688036556644*posDOB.q_in[2];
	///// 10Hz /////
//	posDOB.q_out[0] =+1.8782027348486*posDOB.q_out[1]-0.88191137829818*posDOB.q_out[2]+0.032880464060595*posDOB.gq_in[0]-0.064910467749698*posDOB.gq_in[1]+0.032137434709659*posDOB.gq_in[2]-0.003708643449591*posDOB.q_in[2];

////	posDOB.q_out[0] =+1.9260053067939*posDOB.q_out[1]-0.92737411044961*posDOB.q_out[2]+0.0096669844834378*posDOB.gq_in[0]-0.019167112573304*posDOB.gq_in[1]+0.0095298664515479*posDOB.gq_in[2]-0.0013688036556644*posDOB.q_in[2];
//	posDOB.q_out[0] =+1.9260053067939*posDOB.q_out[1]-0.92737411044961*posDOB.q_out[2]+0.012889312644584*posDOB.gq_in[0]-0.025556150097739*posDOB.gq_in[1]+0.012706488602064*posDOB.gq_in[2]-0.0013688036556644*posDOB.q_in[2];
//	/* Get Disturbance */
//	posDOB.disturbance = posDOB.gain*posDOB.q_out[0];
//
//	/* Saturation */
//	if(posDOB.disturbance > posDOB.saturation)			{posDOB.control_input = +posDOB.saturation;}
//	else if(posDOB.disturbance < -posDOB.saturation)	{posDOB.control_input = -posDOB.saturation;}
//	else 												{posDOB.control_input = +posDOB.disturbance;}
	/// vel ctrl
	posDOB.q_in[0] = velCtrl.ref; //tau_m
	posDOB.gq_in[0] = load_cell.loadcell_filtered; // force

//	15Hz //
	if (MD_node_id == 7){ // RIGHT
		posDOB.q_out[0] =+1.820114481352*posDOB.q_out[1]-0.82820418130686*posDOB.q_out[2]+0.32817844327731*posDOB.gq_in[0]-0.60512870178618*posDOB.gq_in[1]+0.27717733359458*posDOB.gq_in[2]-0.0080896999548105*posDOB.q_in[2];
	}
	if (MD_node_id == 6){ // LEFT
		posDOB.q_out[0] =+1.820114481352*posDOB.q_out[1]-0.82820418130686*posDOB.q_out[2]+0.32817844327731*posDOB.gq_in[0]-0.60512870178618*posDOB.gq_in[1]+0.27717733359458*posDOB.gq_in[2]-0.0080896999548105*posDOB.q_in[2];

	}

	posDOB.disturbance = posDOB.gain*posDOB.q_out[0];
	posDOB.control_input = +posDOB.disturbance;
	/* Array Shifting */
	for(int i = 2; i > 0; --i){
		posDOB.gq_out[i] = posDOB.gq_out[i-1];
	}
	for(int i = 2; i > 0; --i){
		posDOB.gq_in[i] = posDOB.gq_in[i-1];
	}
	for(int i = 2; i > 0; --i){
		posDOB.q_out[i] = posDOB.q_out[i-1];
	}
	for(int i = 2; i > 0; --i){
		posDOB.q_in[i] = posDOB.q_in[i-1];
	}

	return 0;
}
static int Ent_Feedforward_Filter()
{
	memset(posFF.in, 0, sizeof(posFF.in));
	memset(posFF.out, 0, sizeof(posFF.out));
//	posFF.diff = posFF.num_length - posFF.den_length;
	posFF.diff = 2;
	posFF.control_input = 0;

	return 0;
}
static int Run_Feedforward_Filter()
{


//	posFF.control_input = 0.001893*posFF.in[1] + 0.001815*posFF.in[2];

//	posFF.control_input=+9170.5467851218*posFF.in[0]-18210.033136289*posFF.in[1]+9043.0471354804*posFF.in[2];
	//New system// 240408
//	posFF.control_input=20449.131400037*posFF.in[0]-40709.558792614*posFF.in[1]+20265.90552229*posFF.in[2];
	//241007//
//	posFF.control_input=19640.624755283*posFF.in[0]-39137.764017877*posFF.in[1]+19503.613415892*posFF.in[2];


//	posFF.control_input=+8.8659005664784*posFF.in[0]-17.50248268187*posFF.in[1]+8.6655498557575*posFF.in[2];
//	posFF.control_input=+9.4164802901024*posFF.in[0]-18.670427998919*posFF.in[1]+9.2829154491821*posFF.in[2];

	//vel ctrl

//	posFF.control_input = +54.131464031295*posFF.in[0]-104.01954010138*posFF.in[1]+49.917050486966*posFF.in[2];
	posFF.control_input = +40.567443182137*posFF.in[0]-74.802366610191*posFF.in[1]+34.26299308292*posFF.in[2];
	posFF.out[0] = posFF.control_input;
	/* Array Shifting */
	for(int i = 2; i > 0; --i){
		posFF.in[i] = posFF.in[i-1];
	}
	for(int i = 2; i > 0; --i){
		posFF.out[i] = posFF.out[i-1];
	}

	return 0;
}

static int Ext_Feedforward_Filter()
{
	posFF.diff = 0;
	return 0;
}
/////////// 	Flexi-SEA 	/////////////////////////////
/////////////////////////////////////////////////////////
static int Ent_Linearize_Stiffness()
{
	LS.k=0.085*5; // torque constant amplified by gears in motor
//	spring_state.initial_pos = spring_state.position;

	return 0;
}
static int Run_Linearize_Stiffness()
{
	if(LS.theta_f>0){
		LS.tau_f=0.232*powf(LS.theta_f,3)-0.062*powf(LS.theta_f,2)-0.426*LS.theta_f;
	}
	else{
		LS.tau_f=-0.246*powf(LS.theta_f,3)-0.773*powf(LS.theta_f,2)-0.610*LS.theta_f;
	}
	LS.control_input = LS.tau_f/LS.k;
	if (LS.control_input>1){LS.control_input=1;}
	else if(LS.control_input<-1){LS.control_input=-1;}
	return 0;
}
static int Ext_Linearize_Stiffness()
{
	LS.tau_f=0;
	LS.control_input=0;
	return 0;
}
//// Tanh reference generation
static int Ent_Generate_Position_Tanh()
{
//	spring_state.initial_pos = spring_state.position;
	posCtrl.ref=0;
	return 0;
}
static int Run_Generate_Position_Tanh()
{
	/* For PD controller */
	posCtrl.t_ref= Generate_Ref_tanh(refTanh.amp, refTanh.a, refTanh.td, 0,\
			mid_level_loop_cnt, MID_LEVEL_CONTROL_PERIOD, 0);


	/* For Feedforward */
	if(posFF.diff != 0){
		for(int i = 0; i < posFF.diff; ++i){
			posFF.in[i] = Generate_Ref_tanh(refTanh.amp, refTanh.a, refTanh.td, 0,\
					mid_level_loop_cnt, MID_LEVEL_CONTROL_PERIOD, posFF.diff-i);

		}
		posFF.in[posFF.diff] = posCtrl.t_ref;
	}

	return 0;
}

static int Ext_Generate_Position_Tanh()
{
	posCtrl.ref=0;
//	refTanh.amp=0;
	return 0;
}

//// Ankle reference generation
static int Ent_Generate_Position_Ankle()
{
	step=0;
	return 0;
}
static int Run_Generate_Position_Ankle()
{

	/* For PD controller */


	gaitphase = Gait_Phase_Shift(widmGaitDataObj2.gaitPhase, refAnk.phase_shift);
	shift_test=gaitphase-widmGaitDataObj2.gaitPhase;
	refAnk.gaitPeriod=widmGaitDataObj2.gaitPeriod;

	if(refAnk.onoff==1){
		if(gaitphase<5 && t_gaitphase>95){
			refAnk.gait_cnt=0;
			step ++;

		}else if(gaitphase > 0){
			refAnk.gait_cnt++;
		}
		t_gaitphase= gaitphase;
		if(step>3){
			if(refAnk.gait_cnt)
			posCtrl.t_ref= Generate_Ref_Plantar(-1*refAnk.amp_P/36.32, refAnk.gc[0], refAnk.gc[1], refAnk.gc[2], refAnk.gc[3], refAnk.gaitPeriod, refAnk.offset,\
					refAnk.gait_cnt, MID_LEVEL_CONTROL_PERIOD, 0);
			Generate_Ref_Dorsi(refAnk.amp_D, refAnk.amp_D2,refAnk.gc[4],refAnk.gc[5],refAnk.gc[6], refAnk.gaitPeriod, refAnk.gait_cnt, MID_LEVEL_CONTROL_PERIOD, 0);
			Run_Corridor_Impedance_Control();
			/* For Feedforward */
			if(posFF.diff != 0){
				for(int i = 0; i < posFF.diff; ++i){
					posFF.in[i]= Generate_Ref_Plantar(-1*refAnk.amp_P/36.32, refAnk.gc[0], refAnk.gc[1], refAnk.gc[2], refAnk.gc[3], refAnk.gaitPeriod, refAnk.offset,\
							refAnk.gait_cnt, MID_LEVEL_CONTROL_PERIOD, posFF.diff-i);
				}
				posFF.in[posFF.diff] = posCtrl.t_ref;
			}
		}
	}
	else if(refAnk.onoff==0){
		posCtrl.t_ref=0;
		impedanceCtrl.ref=AbsObj1.posDeg;
		if(posFF.diff != 0){
			for(int i = 0; i < posFF.diff; ++i){
				posFF.in[i] = 0;

			}
			posFF.in[posFF.diff] = posCtrl.t_ref;
		}
		refAnk.gait_cnt=0;
		step=0;
	}
	posCtrl.total_t_ref = posCtrl.t_ref+impedanceCtrl.control_input;




	return 0;
}

static int Ext_Generate_Position_Ankle()
{
	posCtrl.ref=0;
	refTanh.amp=0;
	return 0;
}

static int Ent_Generate_Position_Ankle_Periodic()
{

	return 0;
}
static int Run_Generate_Position_Ankle_Periodic()
{

	/* For PD controller */
	refAnk.gait_cnt = mid_level_loop_cnt % refAnk.gaitPeriod;
	posCtrl.t_ref= Generate_Ref_Plantar(refAnk.amp_P, refAnk.gc[0], refAnk.gc[1], refAnk.gc[2], refAnk.gc[3], refAnk.gaitPeriod, refAnk.offset,\
			refAnk.gait_cnt, MID_LEVEL_CONTROL_PERIOD, 0);
	/* For Feedforward */
	if(posFF.diff != 0){
		for(int i = 0; i < posFF.diff; ++i){
			posFF.in[i]= Generate_Ref_Plantar(refAnk.amp_P, refAnk.gc[0], refAnk.gc[1], refAnk.gc[2], refAnk.gc[3], refAnk.gaitPeriod, refAnk.offset,\
					refAnk.gait_cnt, MID_LEVEL_CONTROL_PERIOD, posFF.diff-i);
		}
		posFF.in[posFF.diff] = posCtrl.t_ref;
	}




	return 0;
}

static int Ext_Generate_Position_Ankle_Periodic()
{
	posCtrl.ref=0;
	refTanh.amp=0;
	return 0;
}

static int Ent_Risk_Manage(){
	RISK_Flexi.max_ank_cnt=0;
	RISK_Flexi.min_ank_cnt=0;
	RISK_Flexi.max_error_cnt=0;
	return 0;
}
static int Run_Risk_Manage(){
	// Torque error management
	if((posCtrl.err>RISK_Flexi.max_error) || (posCtrl.err< (-1 * RISK_Flexi.max_error))){
		RISK_Flexi.max_error_cnt=RISK_Flexi.max_error_cnt+1;
	}else{
		RISK_Flexi.max_error_cnt=0;
	}
	// Min ankle ROM management
	if(AbsObj1.posDeg<RISK_Flexi.min_ank_ang){
		RISK_Flexi.min_ank_cnt=RISK_Flexi.min_ank_cnt+1;
	}else{
		RISK_Flexi.min_ank_cnt=0;
	}
	// Torque error management
	if(AbsObj1.posDeg>RISK_Flexi.max_ank_ang){
		RISK_Flexi.max_ank_cnt=RISK_Flexi.max_ank_cnt+1;
	}else{
		RISK_Flexi.max_ank_cnt=0;
	}
	if(RISK_Flexi.max_error_cnt>50 || RISK_Flexi.max_ank_cnt>50 || RISK_Flexi.min_ank_cnt>50){
		Transition_State(&low_level_ctrl_task.state_machine, e_State_Standby);
		Transition_State(&mid_level_ctrl_task.state_machine, e_State_Standby);
	}
	return 0;
}
static float Gait_Phase_Shift(float gp, float s){
	float gp_s;
	if (gp ==-100){
		gp_s=-100;
	}
	else{
		gp_s = gp - s;
		if (gp_s>100){
			gp_s = gp_s-100;
		}
		else if(gp_s<0){
			gp_s = gp_s+100;
		}
		else{
			gp_s = +gp_s;
		}
	}
	return gp_s;
}
static void Ent_Get_Loadcell()
{
/////////////// AD620 Module: high noise from motor drive /////////////////
//	load_cell.scale = -420.72;
//	load_cell.offset=1.652;

/////////////// ST01A/////////////////
	//// LEFT ////////////////
	if (MD_node_id == 6) {
		load_cell.scale =  195.38;
		load_cell.offset=  1.0;
	}
	//// RIGHT  ////////////////
	if (MD_node_id == 7) {
		load_cell.scale =  210.21;
		load_cell.offset=  1.0;
	}

	load_cell.cutoff=50;
}

static void Run_Get_Loadcell()
{
	float rawdata;
	rawdata = LCbuffer[0]*0.00005035477226; // Voltage
	load_cell.loadcell_raw=(rawdata-load_cell.offset)*load_cell.scale;
//
//	if ((load_cell.loadcell_raw - load_cell.loadcell_raw_pre > 30) || (load_cell.loadcell_raw - load_cell.loadcell_raw_pre < -30)) {
//		load_cell.loadcell_raw = load_cell.loadcell_raw_pre;
//	}

	load_cell.alpha = 2*3.1415926535*0.001*load_cell.cutoff / (2*3.1415926535*0.001*load_cell.cutoff + 1);
	load_cell.loadcell_filtered = (1-load_cell.alpha)*load_cell.loadcell_filtered + load_cell.alpha*load_cell.loadcell_raw;

	load_cell.loadcell_raw_pre = load_cell.loadcell_raw;
	load_cell.torque = load_cell.loadcell_filtered * flexi_ankle.ratio;
}
static void Ent_T2F()
{
	flexi_ankle.xa = 60; flexi_ankle.ya = 7;
	flexi_ankle.xs = 110; flexi_ankle.ys = 160;
	flexi_ankle.la=sqrt(pow(flexi_ankle.xa,2)+pow(flexi_ankle.ya,2));
	flexi_ankle.ls=sqrt(pow(flexi_ankle.xs,2)+pow(flexi_ankle.ys,2));
	flexi_ankle.angle_bias = atan(flexi_ankle.ya/flexi_ankle.xa)+atan(flexi_ankle.xs/flexi_ankle.ys);
}

static void T2F()
{
	flexi_ankle.alpha = M_PI/180*AbsObj1.posDeg + M_PI_2 - flexi_ankle.angle_bias;
	flexi_ankle.alpha_deg = 180/M_PI*flexi_ankle.alpha;
	flexi_ankle.ratio = (flexi_ankle.la*flexi_ankle.ls*sin(flexi_ankle.alpha))/sqrt(pow(flexi_ankle.la,2)+pow(flexi_ankle.ls,2)-2*flexi_ankle.la*flexi_ankle.ls*cos(flexi_ankle.alpha))*0.001;
	flexi_ankle.ratio_inv = 1/flexi_ankle.ratio;
	flexi_ankle.ratio_torque = flexi_ankle.ratio*2*M_PI/0.006;
//	posCtrl.ref=flexi_ankle.ratio*posCtrl.total_t_ref;
	posCtrl.ref=posCtrl.t_ref;
	// TODO : Transform the torque reference to the force reference (=posCtrl.ref)
}

float Generate_Ref_Dorsi(float amp_p, float amp_t,float gc_i, float gc_p, float gc_t, uint16_t T_gait, uint32_t t_k, float t_T, int32_t t_phase_lead_index)
{
	/* section time = settle time = rising time = falling time */

	float t_result, t_time, t_p, t_i, t_t;		// t_period = rising time + settle time + falling time

	static float t_result_f = 0.0;
	t_p=T_gait * gc_p * 0.01*0.001;
	t_i=T_gait * gc_i * 0.01*0.001;
	t_t=T_gait * gc_t * 0.01*0.001;
	t_time = (t_k + t_phase_lead_index)* t_T ;		//sec

	if (t_time<t_i){
		impedanceCtrl.on=0;
		impedanceCtrl.init_pos=AbsObj1.posDeg;
		impedanceCtrl.ref=impedanceCtrl.init_pos;
	}
	else if((t_time>=t_i) && (t_time<t_p)){
		impedanceCtrl.ref=((3*powf((t_p-t_i),-2))*powf((t_time-t_i),2)+(-2*powf((t_p-t_i),-3))*powf((t_time-t_i),3))*(amp_p-impedanceCtrl.init_pos)+impedanceCtrl.init_pos;
		impedanceCtrl.on=1;
	}
	else if((t_time>=t_p) && (t_time<t_t)){
		impedanceCtrl.ref=((3*powf((t_t-t_p),-2))*powf((t_time-t_p),2)+(-2*powf((t_t-t_p),-3))*powf((t_time-t_p),3))*(amp_t-amp_p)+amp_p;
		impedanceCtrl.on=1;
	}
	else if(t_time>=t_t){
		impedanceCtrl.on=0;
		impedanceCtrl.init_pos=AbsObj1.posDeg;
		impedanceCtrl.ref=impedanceCtrl.init_pos;
	}

	return 0;
}
static void error_filter2(ImpedanceCtrl *t_impedanceCtrl)
{
	// f2(e,t) = lambda * e + (1 - lambda)*sign(e)*max(|e| - epsilon, 0)
	float t_abs_e = 0.0;
	float t_sign_e = 0.0;
	float t_max = 0.0;
	float t_diff = 0.0;
	float y_ef = 0.0;

	/* Calculate 'sign(e) & |e|' */
	if (t_impedanceCtrl->e > 0) {t_abs_e = +t_impedanceCtrl->e; t_sign_e = +1; }
	else                        {t_abs_e = -t_impedanceCtrl->e; t_sign_e = -1; }

	/* Calculate 'max(|e| - epsilon, 0)' */
	t_diff = t_abs_e - t_impedanceCtrl->epsilon;
	if (t_diff > 0) {t_max = t_diff;}
	else            {t_max = 0;}


	y_ef = (t_impedanceCtrl->lambda * t_impedanceCtrl->e) + (1 - t_impedanceCtrl->lambda)*t_sign_e*t_max;

	t_impedanceCtrl->ef_diff = (y_ef - t_impedanceCtrl->ef) * MID_LEVEL_CONTROL_FREQUENCY;

	t_impedanceCtrl->ef   = y_ef;
}

static int Ent_Corridor_Impedance_Control()
{
	return 0;
}
static int Run_Corridor_Impedance_Control()
{

	/* Impedance Controller */
	impedanceCtrl.e = impedanceCtrl.ref - AbsObj1.posDeg;

	error_filter2(&impedanceCtrl);

	float t_control_input;
	t_control_input = impedanceCtrl.Kp * impedanceCtrl.ef + impedanceCtrl.Kd * impedanceCtrl.ef_diff;
	impedanceCtrl.control_input = t_control_input * impedanceCtrl.on;

	return 0;
}
static int Ent_Ankle_Compensator()
{
	ankle_comp.theta_dot = 0;
	ankle_comp.theta_dot_prev = 0;
	ankle_comp.theta_ddot = 0;
	ankle_comp.theta = M_PI/180*AbsObj1.posDeg;
	ankle_comp.theta_prev = ankle_comp.theta;
	return 0;
}
static int Run_Ankle_Compensator()
{
	ankle_comp.theta = M_PI/180*AbsObj1.posDeg;
	ankle_comp.a = 2*3.1415926535*0.001*ankle_comp.cutoff / (2*3.1415926535*0.001*ankle_comp.cutoff + 1);
	ankle_comp.theta_dot = (ankle_comp.theta-ankle_comp.theta_prev) * MID_LEVEL_CONTROL_FREQUENCY;
	ankle_comp.theta_dot_final = (1-ankle_comp.a)*ankle_comp.theta_dot_final + ankle_comp.a*ankle_comp.theta_dot;

	ankle_comp.theta_ddot = (ankle_comp.theta_dot_final-ankle_comp.theta_dot_prev) * MID_LEVEL_CONTROL_FREQUENCY;
	ankle_comp.theta_ddot_final = (1-ankle_comp.a)*ankle_comp.theta_ddot_final + ankle_comp.a*ankle_comp.theta_ddot;

	ankle_comp.theta_prev = ankle_comp.theta;
	ankle_comp.theta_dot_prev = ankle_comp.theta_dot_final;

	ankle_comp.control_input = ankle_comp.gain * (0.1 * ankle_comp.alpha * ankle_comp.theta_ddot_final + ankle_comp.beta * ankle_comp.theta_dot_final) * flexi_ankle.ratio_torque;
	return 0;
}

static int Ent_Proportional_Assist()
{
	posCtrl.err = 0;
	posCtrl.err_sum = 0;
	posCtrl.err_diff = 0;

}

static int Run_Proportional_Assist()
{
	proportionalCtrl.torque_ref = proportionalCtrl.K_torque * pow((pMMG_sense.pMMG3 - 102), proportionalCtrl.power_PF);
	proportionalCtrl.force_ref  = proportionalCtrl.torque_ref * flexi_ankle.ratio_inv;

	posCtrl.t_ref = proportionalCtrl.force_ref;
	return 0;
}
