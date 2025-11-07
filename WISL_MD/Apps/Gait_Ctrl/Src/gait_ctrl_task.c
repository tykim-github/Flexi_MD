/*
 * gait_ctrl_task.c
 *
 *  Created on: Oct 11, 2023
 *      Author: INVINCIBLENESS
 */

#include "gait_ctrl_task.h"

/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.z
 */

/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Variables accessible throughout the application.
 */

VQF_MagCalib_t vqfMagCalibObj;
extern uint8_t act_type;
TaskStruct gait_ctrl_task;
IMU_Params_t imu_params;
static uint8_t status;
uint8_t MD_Node_ID;
uint8_t Cali_flag;

#ifdef QUATERNION
// For Quaternion //
VQF_MagCalib_t vqfMagCalibObj;
VQF_t vqfObj;

//static IOIF_MagData_t   magData;
//static IOIF_MagData_t   magData_calib;

float magValueCal[3];
VQF_Real_t vqfGyr[3];
VQF_Real_t vqfAcc[3];
VQF_Real_t vqfMag[3];
VQF_Real_t vqfQuat[4];
int16_t q_send[4];

float RealMagX;
float RealMagY;
float RealMagZ;

float tilt_angle;
/////////////////////////////////////////////////////////////////////////////////
#endif

// For Triggering //
WIDM_GaitData_t		widmGaitDataObj;
WIDM_AngleData_t 	widmAngleDataObj;
WIDM_QuatData_t     widmQuatDataObj;
WIDM_AttachCase_t	widmAttachCaseObj;

#ifdef SUIT_MD_ENABLED
// Prof - 11.02 version //
static uint32_t a = 0;
static uint32_t k = 0;
static uint16_t Period = 1000;
static uint32_t timeStampPrev = 0;
static double cutoffFreq = 6.0;
static double cutoffFreqSmooth = 6.0;
static float modeCheck = 0;
static double PeakAmp = 10;
static double PeakAmpSmooth = 10;
static double PeakWAmp = 70;
static double PeakWAmpSmooth = 70;
static double NeutralPosture = 0;
static uint8_t firstPeriodCheck = 1;
static double velLPF = 0;
static double degLPF0ABS = 0;

static uint8_t mode[2] = {0, 0};
static double velLPF2[2] = {0, 0};
static double velLPF0ABS = 0;
static double degLPF[2] = {0, 0};
static double gyroLPF[2] = {0, 0};
static double degBPF[3] = {0, 0, 0};		// center frequency BPF
static double degBPF0ABS = 0;
static double velBPF = 0;
static double velBPF0ABS = 0;
static double angleNorm = 0;
static double velocityNorm = 0;
static double degUnbiased[2] = {0, 0};
static double gaitPhase = 0;
static double gaitPhasePrev = 0;
uint32_t gaitCount = 0;

float prevIncDeg = 0.0;
float incDeg = 0.0;
float incDegTrig[3] = {0};
float prevIncVel = 0.0;
float incVel = 0.0;
float incVelTrig[3] = {0};

float filteredIncDeg = 0.0;
float filteredIncVel = 0.0;

uint8_t B1Flag = 0;	// walking
uint8_t B2Flag = 0; // 10%
uint8_t B3Flag = 0; // 20%
uint8_t B4Flag = 0; // 30%
uint8_t B5Flag = 0; // 40%
uint8_t B6Flag = 0; // 50%
uint8_t B7Flag = 0; // ~~ 0%  FB Transition
uint8_t B8Flag = 0; // ~~ 50% BF Transition
uint8_t B9Flag = 0; // ~~ 75% BF Moving
uint8_t B10Flag = 0; // ~~ 25% FB Moving
uint8_t B11Flag = 0; // 60%
uint8_t B12Flag = 0; // 70%
uint8_t B13Flag = 0; // 80%
uint8_t B14Flag = 0; // 90%
uint8_t B15Flag = 0; // B8 -> for extension incVel == 0

static uint8_t B7Finished = 0;
static uint8_t B8Finished = 0;

static uint8_t phaseThreshold1 = 10;
static uint8_t phaseThreshold2 = 20;
static uint8_t phaseThreshold3 = 30;
static uint8_t phaseThreshold4 = 40;
static uint8_t phaseThreshold5 = 50;
static uint8_t phaseThreshold6 = 60;
static uint8_t phaseThreshold7 = 88;
static uint8_t phaseThreshold8 = 80;
static uint8_t phaseThreshold9 = 90;

// H10 //
static double velLPF_H10[2] = {0, 0};


// For ISI check //
//static uint8_t B2_chk[ISIchkNum] = {0};
//static uint8_t B3_chk[ISIchkNum] = {0};
//static uint8_t B4_chk[ISIchkNum] = {0};
//static uint8_t B5_chk[ISIchkNum] = {0};
//static uint8_t B6_chk[ISIchkNum] = {0};
//static uint8_t B7_chk[ISIchkNum] = {0};
//static uint8_t B8_chk[ISIchkNum] = {0};
//static uint8_t B9_chk[ISIchkNum] = {0};
//static uint8_t B10_chk[ISIchkNum] = {0};

static uint8_t B2stack = 0;
static uint8_t B3stack = 0;
static uint8_t B4stack = 0;
static uint8_t B5stack = 0;
static uint8_t B6stack = 0;
static uint16_t B7stack = 0;
static uint16_t B8stack = 0;
static uint16_t B9stack = 0;
static uint16_t B10stack = 0;
static uint16_t B15stack = 0;

uint32_t RTBrokenFlag = 0;
////////////////////////////////////////////////////////////////////////////
#endif


/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */

static IOIF_6AxisData_t 	imu6AxisDataObj;
static IOIF_MagData_t 		magDataObj;
static IOIF_MagData_t 		magCalibDataObj;

static WIDM_SensorData_t 	widmSensorDataObj;
WIDM_FuzzyData_t		widmFuzzyDataObj;
static WIDM_NormData_t 		widmNormDataObj;
static WIDM_ThresData_t		widmThresDataObj;
static WIDM_Module_t		widmModuleObj;

// Loop Time Count //
static uint32_t gaitCtrlLoopCnt;
static float gaitCtrlTimeElap;

// For Debug //
static uint8_t testImu6AxisRes 	= IOIF_I2C_STATUS_OK;
static uint8_t testImu3AxisRes 	= IOIF_I2C_STATUS_OK;
static float wcDebug 			= 0.0;

static float deg_diff;
static float vel_diff;

//static uint8_t absOffsetCmd = 0;
uint8_t gaitCtrlState = 0;

static uint16_t stopCnt = 0;

// SELECTION !!!  For ALL Model //
static WIDM_AttachCase_t 	ATTACH_CASE_SEL;
static WIDM_Module_t	    MODULE_SEL;
static uint8_t WIDM_MODULE_NODE_ID;


/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

/* ------------------- STATE FUNCTION ------------------- */
static void StateOff_Run(void);

static void StateStandby_Run(void);

static void StateEnable_Ent(void);
static void StateEnable_Run(void);
static void StateEnable_Ext(void);

static void StateError_Run(void);

/* ------------------- ROUTINE ------------------- */

static int Run_WIDM_6Axis_GetValue(void);
static int Ent_TVCF_Quaternion_WalkONS(void);
static int Run_TVCF_Quaternion_WalkONS(void);
static int Ent_Gyr_Calibration_WalkONS (void);
static int Run_Gyr_Calibration_WalkONS (void);

static int Run_WIDM_3Axis_GetValue(void);

static int RunGetIMUFunction(void);
static int RunTotalGaitFunction(void);
// Prof //
static int RunTotalGaitFunction_Prof(void);
//static void CheckWalkingState_Prof(void);
//static void GaitFunction_Prof_1029_K10(WIDM_AngleData_t* widmAngleData, WIDM_SensorData_t* widmSensorData);
//static void GaitFunction_Prof_1029_H10(WIDM_AngleData_t* widmAngleData);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static int Init_IMU(void);

/* Functions for ALL cases(General) */
static int NodeIDCheck(uint8_t directionSet);
static void ModelSelection(uint8_t nodeID);
static void InitializeIMU(void);
static void InitValueSetting(WIDM_FuzzyData_t* widmFuzzyData, WIDM_NormData_t* widmNormData, WIDM_GaitData_t* widmGaitData, WIDM_ThresData_t* widmThresData, WIDM_Module_t widmModule);
static void SetInitialAngle(WIDM_AngleData_t* widmAngleData, WIDM_NormData_t* widmNormData, WIDM_QuatData_t* widmQuatData, float initialAngle);
static void GetInitialAngle_IMU(IOIF_6AxisData_t* imu6AxisData, WIDM_SensorData_t* widmSensorData, WIDM_AngleData_t* widmAngleData, WIDM_NormData_t* widmNormData, WIDM_AttachCase_t widmAttachCase, WIDM_QuatData_t* widmQuatData);
static void GetInitialAngle(WIDM_Module_t widmModule);
static void UpdateSensorRawData(WIDM_SensorData_t* widmSensorData, IOIF_6AxisData_t* imu6AxisData, WIDM_AttachCase_t widmAttachCase);
static void RunTvcfFilter(WIDM_SensorData_t* widmSensorData, WIDM_AngleData_t* widmAngleData, WIDM_FuzzyData_t* widmFuzzyData, float samplingPeriod, WIDM_AttachCase_t widmAttachCase);
static void SetUsedDegVel(WIDM_AngleData_t* widmAngleData, WIDM_Module_t widmModule);
static void NoiseReduction(WIDM_AngleData_t* widmAngleData, WIDM_GaitData_t* widmGaitData, WIDM_Module_t widmModule);
static void GetGaitPhase(WIDM_AngleData_t* widmAngleData, WIDM_NormData_t* widmNormData, WIDM_GaitData_t* widmGaitData);
static float GetPhaseRadius(float degDiff, float degTh, float velDiff, float velTh);
static void UpdateWalkingState(WIDM_GaitData_t* widmGaitData, float phaseRadiusStart, float phaseRadiusStop, int16_t sumIter);
static void CheckWalkingState(WIDM_AngleData_t* widmAngleData, WIDM_NormData_t* widmNormData, WIDM_ThresData_t* widmThresData, WIDM_GaitData_t* widmGaitData);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/* Functions for only CM */
#if defined(WALKON5_CM_ENABLED) || defined(L30_CM_ENABLED) || defined(SUIT_MINICM_ENABLED)

#endif

/* Functions for only MD */
#if defined(WALKON5_MD_ENABLED) || defined(L30_MD_REV06_ENABLED) || defined(L30_MD_REV07_ENABLED) || defined(L30_MD_REV08_ENABLED) || defined(SUIT_MD_ENABLED)
static void GetABSLinkData(IOIF_AbsEnc_t* absEnc, WIDM_AngleData_t* widmAngleData);
static void GetINCLinkData(WIDM_AngleData_t* widmAngleData);

#ifdef IMUABS_MODE
static void CompensateIMUABS(WIDM_AngleData_t* widmAngleData);
static void GetInitialAngle_IMUABS(IOIF_6AxisData_t* imu6AxisData, IOIF_AbsEnc_t* absEnc, WIDM_SensorData_t* widmSensorData, WIDM_AngleData_t* widmAngleData, WIDM_NormData_t* widmNormData, WIDM_AttachCase_t widmAttachCase);
#endif

#ifdef IMUINC_MODE
static void CompensateIMUINC(WIDM_AngleData_t* widmAngleData);
static void GetInitialAngle_IMUINC(IOIF_6AxisData_t* imu6AxisData, WIDM_SensorData_t* widmSensorData, WIDM_AngleData_t* widmAngleData, WIDM_NormData_t* widmNormData, WIDM_AttachCase_t widmAttachCase);
#endif

//static void Send_BFlag(uint8_t nodeID);
#endif

/* Functions for only WIDM */
#if defined(WIDM_ENABLED)

#endif


/* Functions for [IMU + Absolute encdoer] mode */
#ifdef IMUABS_MODE

#endif /* SAM_IMUABS_MODE */


/* Functions for [IMU + Incremental encoder] mode */
#ifdef IMUINC_MODE

#endif /* SAM_IMUINC_MODE */



#ifdef QUATERNION
// Quaternion //
static void SetMagInvAInfo(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void SetMagIronErrorInfo(MsgSDOargs* t_req, MsgSDOargs* t_res);
static void InitMagInfo(void);
static int EntGetQuaternion(void);
static int RunGetQuaternion(void);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#endif


/* ------------------- SDO CALLBACK ------------------- */
//static void SetAbsOffsetCmd(MsgSDOargs* req, MsgSDOargs* res);
static void SetAccScaleFactor_X(MsgSDOargs* req, MsgSDOargs* res);
static void SetAccScaleFactor_Y(MsgSDOargs* req, MsgSDOargs* res);
static void SetAccScaleFactor_Z(MsgSDOargs* req, MsgSDOargs* res);
static void SetGyrScaleFactor_X(MsgSDOargs* req, MsgSDOargs* res);
static void SetGyrScaleFactor_Y(MsgSDOargs* req, MsgSDOargs* res);
static void SetGyrScaleFactor_Z(MsgSDOargs* req, MsgSDOargs* res);
static void SetAccBias_X(MsgSDOargs* req, MsgSDOargs* res);
static void SetAccBias_Y(MsgSDOargs* req, MsgSDOargs* res);
static void SetAccBias_Z(MsgSDOargs* req, MsgSDOargs* res);
static void SetGyrBias_X(MsgSDOargs* req, MsgSDOargs* res);
static void SetGyrBias_Y(MsgSDOargs* req, MsgSDOargs* res);
static void SetGyrBias_Z(MsgSDOargs* req, MsgSDOargs* res);
static void SetGyrSign_X(MsgSDOargs* req, MsgSDOargs* res);
static void SetGyrSign_Y(MsgSDOargs* req, MsgSDOargs* res);
static void SetGyrSign_Z(MsgSDOargs* req, MsgSDOargs* res);
static void SetFuzzy_wcl(MsgSDOargs* req, MsgSDOargs* res);
static void SetFuzzy_wch(MsgSDOargs* req, MsgSDOargs* res);
static void SetFuzzy_alpha(MsgSDOargs* req, MsgSDOargs* res);
static void SetFuzzy_s(MsgSDOargs* req, MsgSDOargs* res);
static void SetFuzzy_m0(MsgSDOargs* req, MsgSDOargs* res);
static void SetFuzzy_acc_mean(MsgSDOargs* req, MsgSDOargs* res);
static void SetFuzzy_wc_m(MsgSDOargs* req, MsgSDOargs* res);
static void SetFuzzy_wc_debug(MsgSDOargs* req, MsgSDOargs* res);
static void Start_IMU_Calibration(MsgSDOargs* req, MsgSDOargs* res);

/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

MSG_COMMON_SDO_CALLBACK(gait_ctrl_task)

void InitGaitCtrl(void)
{
    Init_Task(&gait_ctrl_task);

    /* Checking Node ID */
    WIDM_MODULE_NODE_ID = NodeIDCheck(0);		// 0:LEFT, 1:RIGHT

	/* State Definition */
	TASK_CREATE_STATE(&gait_ctrl_task, e_State_Off,      NULL,   			StateOff_Run,       NULL,         		 true);
	TASK_CREATE_STATE(&gait_ctrl_task, e_State_Standby,  NULL,   			StateStandby_Run,	NULL,         		 false);
	TASK_CREATE_STATE(&gait_ctrl_task, e_State_Enable,   StateEnable_Ent,  	StateEnable_Run, 	StateEnable_Ext,	 false);
	TASK_CREATE_STATE(&gait_ctrl_task, e_State_Error,    NULL,			  	StateError_Run,    	NULL,				 false);

	/* Routine Definition */
////	TASK_CREATE_ROUTINE(&gait_ctrl_task, ROUTINE_ID_WIDM_6AXIS_GETVALUE, 		NULL, Run_WIDM_6Axis_GetValue, 		NULL);
//	TASK_CREATE_ROUTINE(&gait_ctrl_task, ROUTINE_ID_WIDM_6AXIS_GETVALUE, 	NULL,                         Run_WIDM_6Axis_GetValue, 	     NULL);
//	TASK_CREATE_ROUTINE(&gait_ctrl_task, ROUTINE_ID_WIDM_3AXIS_GETVALUE, 	NULL,                         Run_WIDM_3Axis_GetValue,       NULL);
//	TASK_CREATE_ROUTINE(&gait_ctrl_task, ROUTINE_ID_WIDM_QUATERNION, 		EntGetQuaternion,             RunGetQuaternion, 		     NULL);
////	TASK_CREATE_ROUTINE(&gait_ctrl_task, ROUTINE_ID_WIDM_FUZZY_DEBUG,       NULL,                         Run_WIDM_Fuzzy_Debug_WalkONS,  NULL);
//
//	TASK_CREATE_ROUTINE(&gait_ctrl_task, ROUTINE_ID_TVCF_QUATERNION,   	    Ent_TVCF_Quaternion_WalkONS,  Run_TVCF_Quaternion_WalkONS,   NULL);
//	TASK_CREATE_ROUTINE(&gait_ctrl_task, ROUTINE_ID_GYR_CALIBRATION,        Ent_Gyr_Calibration_WalkONS,  Run_Gyr_Calibration_WalkONS,   NULL);
	/* DOD Definition */
	// DOD
	Create_DOD(TASK_ID_WIDM);
	Create_PDO(TASK_ID_WIDM,	 PDO_ID_WIDM_GAIT_PHASE, 	 	 e_Float32,   	1,    &widmGaitDataObj.gaitPhase);
	Create_PDO(TASK_ID_WIDM,	 PDO_ID_WIDM_GAIT_PERIOD, 	 	 e_UInt16,   	1,    &widmGaitDataObj.gaitPeriod);
	Create_PDO(TASK_ID_WIDM,	 PDO_ID_WIDM_DEG_TVCF, 	 	     e_Float32,   	1,    &widmAngleDataObj.degFinal);
	// PDO
//	Create_PDO(TASK_ID_WIDM,	 PDO_ID_WIDM_ANGLE_IMUABS, 	 		 e_Float32,   	1,    &widmAngleDataObj.degFinal);
//	Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_GAITPHASE_IMUABS, 	 	 e_Float32,   	1,    &widmGaitDataObj.gaitPhase);
//	Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_WALKINGSTATE_IMUABS, 	 e_UInt8, 		1,    &widmGaitDataObj.walkingState);
//	Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_GAITCOUNT,		e_UInt32, 		1,    &gaitCount);
//	Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_STATE,			e_UInt8, 		1,    &gaitCtrlState);
//	Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_INCDEG,		e_Float32, 		1,    &incDeg);
//	Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_INCVEL,		e_Float32, 		1,    &incVel);
//	Create_PDO(TASK_ID_WIDM,	 PDO_ID_WIDM_LOOP_CNT, 	 		 e_UInt32,   	1,    &gaitCtrlLoopCnt);
//	Create_PDO(TASK_ID_WIDM,	 PDO_ID_WIDM_ACC_X, 	 		 e_Float32,   	1,    &widmSensorDataObj.accX);
//	Create_PDO(TASK_ID_WIDM,	 PDO_ID_WIDM_ACC_Y, 	 		 e_Float32,   	1,    &widmSensorDataObj.accY);
//	Create_PDO(TASK_ID_WIDM,	 PDO_ID_WIDM_ACC_Z, 	 		 e_Float32,   	1,    &widmSensorDataObj.accZ);
//	Create_PDO(TASK_ID_WIDM,	 PDO_ID_WIDM_GYR_X, 	 		 e_Float32,   	1,    &widmSensorDataObj.gyrX);
//	Create_PDO(TASK_ID_WIDM,	 PDO_ID_WIDM_GYR_Y, 	 		 e_Float32,   	1,    &widmSensorDataObj.gyrY);
//	Create_PDO(TASK_ID_WIDM,	 PDO_ID_WIDM_GYR_Z, 	 		 e_Float32,   	1,    &widmSensorDataObj.gyrZ);
//	Create_PDO(TASK_ID_WIDM,	 PDO_ID_WIDM_MAG_X, 	 		 e_Float32,   	1,    &magDataObj.magX);
//	Create_PDO(TASK_ID_WIDM,	 PDO_ID_WIDM_MAG_Y, 	 		 e_Float32,   	1,    &magDataObj.magY);
//	Create_PDO(TASK_ID_WIDM,	 PDO_ID_WIDM_MAG_Z, 	 		 e_Float32,   	1,    &magDataObj.magZ);
//	Create_PDO(TASK_ID_WIDM,	 PDO_ID_WIDM_ACC_XYZ, 	 		 e_Float32,   	3,    &imu6AxisDataObj.accX);
//	Create_PDO(TASK_ID_WIDM,	 PDO_ID_WIDM_GYR_XYZ, 	 		 e_Float32,   	3,    &imu6AxisDataObj.gyrX);
//	Create_PDO(TASK_ID_WIDM,	 PDO_ID_WIDM_MAG_XYZ, 	 		 e_Float32,   	3,    &magDataObj.magX);
//	Create_PDO(TASK_ID_WIDM,	 PDO_ID_WIDM_MAG_CALIB_XYZ,  	 e_Float32,   	3,    &magCalibDataObj.magX);
//	Create_PDO(TASK_ID_WIDM,	 PDO_ID_WIDM_ACC_GYR_XYZ,        e_Float32,   	6,    &imu6AxisDataObj.accX);
//	Create_PDO(TASK_ID_WIDM,	 PDO_ID_WIDM_TILT_ANGLE,         e_Float32,   	1,    &tilt_angle);
//	Create_PDO(TASK_ID_WIDM,	 PDO_ID_WIDM_DEG_ACC_FILTERED,   e_Float32,   	1,    &widmAngleDataObj.degAccFiltered);
//	Create_PDO(TASK_ID_WIDM,	 PDO_ID_WIDM_DEG_GYR_FILTERED,   e_Float32,   	1,    &widmAngleDataObj.degGyrFiltered);
//	Create_PDO(TASK_ID_WIDM,	 PDO_ID_WIDM_FUZZY_M_ACC,        e_Float32,   	1,    &widmFuzzyDataObj.m_acc);
//	Create_PDO(TASK_ID_WIDM,	 PDO_ID_WIDM_FUZZY_M_GYR,        e_Float32,   	1,    &widmFuzzyDataObj.m_gyr);
//	Create_PDO(TASK_ID_WIDM,	 PDO_ID_WIDM_FUZZY_M,            e_Float32,   	1,    &widmFuzzyDataObj.m_filtered);
//	Create_PDO(TASK_ID_WIDM,	 PDO_ID_WIDM_FUZZY_WC,           e_Float32,   	1,    &widmFuzzyDataObj.wc);
//	Create_PDO(TASK_ID_WIDM,	 PDO_ID_WIDM_TVCF_ROLL,          e_Float32,   	1,    &widmQuatDataObj.roll);
//	Create_PDO(TASK_ID_WIDM,	 PDO_ID_WIDM_TVCF_PITCH,         e_Float32,   	1,    &widmQuatDataObj.pitch);
//	Create_PDO(TASK_ID_WIDM,	 PDO_ID_WIDM_TVCF_YAW,           e_Float32,   	1,    &widmQuatDataObj.yaw);
//
//	Create_PDO(TASK_ID_WIDM,	 PDO_ID_WIDM_SO3_GYR_FILTERED_X, e_Float32,   	1,    &widmQuatDataObj.so3GyrFiltered[0].x);
//	Create_PDO(TASK_ID_WIDM,	 PDO_ID_WIDM_SO3_GYR_FILTERED_Y, e_Float32,   	1,    &widmQuatDataObj.so3GyrFiltered[0].y);
//	Create_PDO(TASK_ID_WIDM,	 PDO_ID_WIDM_SO3_GYR_FILTERED_Z, e_Float32,   	1,    &widmQuatDataObj.so3GyrFiltered[0].z);
//	Create_PDO(TASK_ID_WIDM,	 PDO_ID_WIDM_SO3_ACC_FILTERED_X, e_Float32,   	1,    &widmQuatDataObj.so3AccFiltered[0].x);
//	Create_PDO(TASK_ID_WIDM,	 PDO_ID_WIDM_SO3_ACC_FILTERED_Y, e_Float32,   	1,    &widmQuatDataObj.so3AccFiltered[0].y);
//	Create_PDO(TASK_ID_WIDM,	 PDO_ID_WIDM_SO3_ACC_FILTERED_Z, e_Float32,   	1,    &widmQuatDataObj.so3AccFiltered[0].z);

	/* For MD_MODULE */
//	Send_BFlag(WIDM_MODULE_NODE_ID);

#ifdef QUATERNION
    Create_PDO(TASK_ID_WIDM, PDO_ID_WIDM_QUATERNION,          e_Int16,   4, &q_send);
    Create_SDO(TASK_ID_WIDM, SDO_ID_WIDM_MAG_INVA,        e_Float32, SetMagInvAInfo);
    Create_SDO(TASK_ID_WIDM, SDO_ID_WIDM_MAG_IRON_ERROR,  e_Float32, SetMagIronErrorInfo);
#endif
	//////////////////////////////////////////////////////////////////////////////////////////////////////////


	// SDO
	MSG_COMMON_SDO_CREATE(TASK_ID_WIDM)

//	Create_SDO(TASK_ID_WIDM, 	SDO_ID_WIDM_SET_ABSOFFSET_CMD, 	e_UInt8, 	SetAbsOffsetCmd);
    Create_SDO(TASK_ID_WIDM, 	SDO_ID_WIDM_SET_ACC_SCALE_FACTOR_X, 	e_Float32, 	SetAccScaleFactor_X);
	Create_SDO(TASK_ID_WIDM, 	SDO_ID_WIDM_SET_ACC_SCALE_FACTOR_Y, 	e_Float32, 	SetAccScaleFactor_Y);
	Create_SDO(TASK_ID_WIDM, 	SDO_ID_WIDM_SET_ACC_SCALE_FACTOR_Z, 	e_Float32, 	SetAccScaleFactor_Z);
	Create_SDO(TASK_ID_WIDM, 	SDO_ID_WIDM_SET_ACC_BIAS_X, 	        e_Float32, 	SetAccBias_X);
	Create_SDO(TASK_ID_WIDM, 	SDO_ID_WIDM_SET_ACC_BIAS_Y, 	        e_Float32, 	SetAccBias_Y);
	Create_SDO(TASK_ID_WIDM, 	SDO_ID_WIDM_SET_ACC_BIAS_Z, 	        e_Float32, 	SetAccBias_Z);
	Create_SDO(TASK_ID_WIDM, 	SDO_ID_WIDM_SET_GYR_SCALE_FACTOR_X, 	e_Float32, 	SetGyrScaleFactor_X);
	Create_SDO(TASK_ID_WIDM, 	SDO_ID_WIDM_SET_GYR_SCALE_FACTOR_Y, 	e_Float32, 	SetGyrScaleFactor_Y);
	Create_SDO(TASK_ID_WIDM, 	SDO_ID_WIDM_SET_GYR_SCALE_FACTOR_Z, 	e_Float32, 	SetGyrScaleFactor_Z);
	Create_SDO(TASK_ID_WIDM, 	SDO_ID_WIDM_SET_GYR_BIAS_X, 	        e_Float32, 	SetGyrBias_X);
	Create_SDO(TASK_ID_WIDM, 	SDO_ID_WIDM_SET_GYR_BIAS_Y, 	        e_Float32, 	SetGyrBias_Y);
	Create_SDO(TASK_ID_WIDM, 	SDO_ID_WIDM_SET_GYR_BIAS_Z, 	        e_Float32, 	SetGyrBias_Z);
	Create_SDO(TASK_ID_WIDM, 	SDO_ID_WIDM_SET_GYR_SIGN_X, 	        e_Int16, 	SetGyrSign_X);
	Create_SDO(TASK_ID_WIDM, 	SDO_ID_WIDM_SET_GYR_SIGN_Y, 	        e_Int16, 	SetGyrSign_Y);
	Create_SDO(TASK_ID_WIDM, 	SDO_ID_WIDM_SET_GYR_SIGN_Z, 	        e_Int16, 	SetGyrSign_Z);
	Create_SDO(TASK_ID_WIDM, 	SDO_ID_WIDM_SET_FUZZY_WCL, 	          e_Float32, 	SetFuzzy_wcl);
	Create_SDO(TASK_ID_WIDM, 	SDO_ID_WIDM_SET_FUZZY_WCH, 	          e_Float32, 	SetFuzzy_wch);
	Create_SDO(TASK_ID_WIDM, 	SDO_ID_WIDM_SET_FUZZY_ALPHA, 	        e_Float32, 	SetFuzzy_alpha);
	Create_SDO(TASK_ID_WIDM, 	SDO_ID_WIDM_SET_FUZZY_S, 	            e_Float32, 	SetFuzzy_s);
	Create_SDO(TASK_ID_WIDM, 	SDO_ID_WIDM_SET_FUZZY_M0, 	          e_Float32, 	SetFuzzy_m0);
	Create_SDO(TASK_ID_WIDM, 	SDO_ID_WIDM_SET_FUZZY_ACC_MEAN, 	    e_Float32, 	SetFuzzy_acc_mean);
	Create_SDO(TASK_ID_WIDM, 	SDO_ID_WIDM_SET_FUZZY_WC_M, 	        e_Float32, 	SetFuzzy_wc_m);
	Create_SDO(TASK_ID_WIDM, 	SDO_ID_WIDM_SET_FUZZY_WC_DEBUG, 	    e_Float32, 	SetFuzzy_wc_debug);
	Create_SDO(TASK_ID_WIDM,    SDO_ID_WIDM_START_CALIBRATION,          e_UInt8,    Start_IMU_Calibration);

	/* Init 6axis & 3axis IMU */
	InitializeIMU();

#ifdef QUATERNION
	// Quaternion //
	InitMagInfo();
	/////////////////////////////////////////////////////////////////////////////////////////////////////
#endif

	/* Timer Callback Allocation */
	if(IOIF_StartTimIT(IOIF_TIM2) > 0){
		//TODO: ERROR PROCESS
	}
	IOIF_SetTimCB(IOIF_TIM2, IOIF_TIM_PERIOD_ELAPSED_CALLBACK, RunGaitCtrl, NULL);


}

void RunGaitCtrl(void* params)
{
	/* Loop Start Time Check */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	/* Run Device */
	Run_Task(&gait_ctrl_task);

	/* Elapsed Time Check */
#ifndef WIDM_ENABLED
	gaitCtrlTimeElap = DWT->CYCCNT / 480;	// in microsecond
#else
	gaitCtrlTimeElap = DWT->CYCCNT / 160;	// in microsecond
#endif
}



/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */
static void StateOff_Run(void)
{
	gaitCtrlState = 0;
	Transition_State(&gait_ctrl_task.state_machine, e_State_Standby);
}

static void StateStandby_Run(void)
{
	gaitCtrlState = 1;
	Transition_State(&gait_ctrl_task.state_machine, e_State_Enable);
}

static void StateEnable_Ent(void)
{
	gaitCtrlLoopCnt = 0;

	// Quaternion //
	//	EntGetQuaternion();

		Ent_Routines(&gait_ctrl_task.routine);

//	if ((act_type == 10) || (act_type ==11)) {
//		Init_IMU();
//		if ((widmSensorDataObj.Calibration_ON == 1) & (widmSensorDataObj.Calibration_DONE == 0)) {
//			Cali_flag = 1;
//			Ent_Gyr_Calibration_WalkONS();
//		}
//
//		if ((widmSensorDataObj.Calibration_ON == 0) & (widmSensorDataObj.Calibration_DONE == 1)) {
//			Ent_TVCF_Quaternion_WalkONS();
//			widmSensorDataObj.Calibration_DONE = 0;
//		}
//	}
}

static void StateEnable_Run(void)
{
	gaitCtrlState = 2;

//	Run_WIDM_6Axis_GetValue();
//	Run_WIDM_6Axis_GetValue_WalkONS();
//	Run_WIDM_Fuzzy_Debug_WalkONS();

	// Quaternion //
//	RunGetQuaternion();

	/* Select the function */
//	RunTotalGaitFunction();
//  RunGetIMUFunction();
//	RunTotalGaitFunction_Prof();	// For Suit H10 & K10

	Run_Routines(&gait_ctrl_task.routine);

//	if ((act_type == 10) || (act_type ==11)) {
//		if (widmSensorDataObj.start_calibration == 1) {
//			widmSensorDataObj.Calibration_ON = 1;
//			widmSensorDataObj.start_calibration = 0;
//		}
//
//		if ((widmSensorDataObj.Calibration_ON == 1) & (Cali_flag == 0)) {
//			Transition_State(&gait_ctrl_task.state_machine, e_State_Standby);
//		}
//
//		if (widmSensorDataObj.Calibration_ON == 1) {
//			if (widmSensorDataObj.Calibration_DONE == 0) {
//				Run_Gyr_Calibration_WalkONS();
//			}
//			else {
//				widmSensorDataObj.Calibration_ON = 0;
//				Cali_flag = 0;
//				Transition_State(&gait_ctrl_task.state_machine, e_State_Standby);
//			}
//		}
//
//	//	if ((widmSensorDataObj.Calibration_ON == 0) & (widmSensorDataObj.Calibration_DONE == 1)) {
//		if (widmSensorDataObj.Calibration_ON == 0){
//			Run_TVCF_Quaternion_WalkONS();
//		}
//	}

	gaitCtrlLoopCnt++;
}

static void StateEnable_Ext(void)
{
    //Ext_Routines(&gait_ctrl_task.routine);
    gaitCtrlLoopCnt = 0;
}

static void StateError_Run(void)
{

}


/* ------------------- ROUTINE ------------------- */

/* Module & Mode Selection Functions */
static int NodeIDCheck(uint8_t directionSet)
{
#if defined(WALKON5_CM_ENABLED) || defined(L30_CM_ENABLED) || defined(SUIT_MINICM_ENABLED)
	int nodeID = 1;
	return nodeID;
#endif

#if defined(WALKON5_MD_ENABLED) || defined(L30_MD_REV06_ENABLED) || defined(L30_MD_REV07_ENABLED) || defined(L30_MD_REV08_ENABLED)
	int temp1, temp2, temp3, temp4;
	temp1 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, BSP_GPIO_PIN_8);
	temp2 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, BSP_GPIO_PIN_9);
	temp3 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, BSP_GPIO_PIN_10);
	temp4 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, BSP_GPIO_PIN_11);
	return ((temp1<<3)|(temp2<<2)|(temp3<<1)|(temp4));
#endif

#if defined(SUIT_MD_ENABLED)
	int temp1, temp2, temp3, temp4;
	temp1 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_F, IOIF_GPIO_PIN_2);
	temp2 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_F, IOIF_GPIO_PIN_3);
	temp3 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_F, IOIF_GPIO_PIN_4);
	temp4 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_F, IOIF_GPIO_PIN_5);
	return ((temp1<<3)|(temp2<<2)|(temp3<<1)|(temp4));
#endif

#ifdef WIDM_ENABLED
	int nodeID = 0;
	if (directionSet == 0){			// RIGHT WIDM
		nodeID = 14;
	}
	else if (directionSet == 1){	// LEFT WIDM
		nodeID = 15;
	}
	return nodeID;
#endif
}


/* Select the Joint and Sensor method */
//static void ModelSelection(uint8_t nodeID)
//{
//	switch (nodeID){
//		case (NODE_ID_LH_SAG):	// LH
//			ATTACH_CASE_SEL   = WIDM_H10_LEFT;
//			MODULE_SEL 		  = WIDM_IMUABS_SAM;
//			break;
//		case (NODE_ID_RH_SAG):	// RH
//			ATTACH_CASE_SEL   = WIDM_H10_RIGHT;
//			MODULE_SEL 		  = WIDM_IMUABS_SAM;
//			break;
//		case (NODE_ID_LK):		// LK
//			ATTACH_CASE_SEL   = WIDM_K10_LEFT;
//			MODULE_SEL 		  = WIDM_IMU_SAM;
//			break;
//		case (NODE_ID_RK):		// RK
//			ATTACH_CASE_SEL   = WIDM_K10_RIGHT;
//			MODULE_SEL 		  = WIDM_IMU_SAM;
//			break;
//		case (NODE_ID_WIDM_R):
//			ATTACH_CASE_SEL   = WIDM_RIGHT_U5;
//			MODULE_SEL 		  = WIDM_IMU_U5;
//			break;
//		case (NODE_ID_WIDM_L):
//			ATTACH_CASE_SEL   = WIDM_LEFT_U5;
//			MODULE_SEL 		  = WIDM_IMU_U5;
//			break;
//	}
//}

/* Initialize 6Axis & 3Axis IMU */
static void InitializeIMU(void)
{
#ifdef WALKON5_CM_ENABLED
	if(IOIF_Init6Axis(IOIF_I2C3) > 0) {
	    //TODO: ERROR PROCESS
	    testImu6AxisRes = IOIF_IMU6AXIS_STATUS_ERROR;
	}
	if(IOIF_InitMag(IOIF_I2C2) > 0) {
	    //TODO: ERROR PROCESS
	    testImu3AxisRes = IOIF_IMU3AXIS_STATUS_ERROR;
	}
#endif

#ifdef L30_CM_ENABLED
	if(IOIF_Init6Axis(IOIF_I2C3) > 0) {
	    //TODO: ERROR PROCESS
	    testImu6AxisRes = IOIF_IMU6AXIS_STATUS_ERROR;
	}
	if(IOIF_InitMag(IOIF_I2C2) > 0) {
	    //TODO: ERROR PROCESS
	    testImu3AxisRes = IOIF_IMU3AXIS_STATUS_ERROR;
	}
#endif

#ifdef SUIT_MINICM_ENABLED
	if(IOIF_Init6Axis(IOIF_I2C1) > 0) {
	    //TODO: ERROR PROCESS
	    testImu6AxisRes = IOIF_IMU6AXIS_STATUS_ERROR;
	}
	if(IOIF_InitMag(IOIF_I2C2) > 0) {
	    //TODO: ERROR PROCESS
	    testImu3AxisRes = IOIF_IMU3AXIS_STATUS_ERROR;
	}
#endif

#ifdef L30_MD_REV06_ENABLED
	if(IOIF_Init6Axis(IOIF_I2C1) > 0) {
	    //TODO: ERROR PROCESS
	    testImu6AxisRes = IOIF_IMU6AXIS_STATUS_ERROR;
	}
	if(IOIF_InitMag(IOIF_I2C1) > 0) {
	    //TODO: ERROR PROCESS
	    testImu3AxisRes = IOIF_IMU3AXIS_STATUS_ERROR;
	}
#endif

#if defined(L30_MD_REV07_ENABLED) || defined(L30_MD_REV08_ENABLED)
	if(IOIF_Init6Axis(IOIF_I2C3) > 0) {
	    //TODO: ERROR PROCESS
	    testImu6AxisRes = IOIF_IMU6AXIS_STATUS_ERROR;
	}
	if(IOIF_InitMag(IOIF_I2C1) > 0) {
	    //TODO: ERROR PROCESS
	    testImu3AxisRes = IOIF_IMU3AXIS_STATUS_ERROR;
	}
#endif

#ifdef SUIT_MD_ENABLED
	if (widmModuleObj == WIDM_IMUABS_SAM || widmModuleObj == WIDM_IMUINC_SAM || widmModuleObj == WIDM_IMU_SAM){
	    if(IOIF_Init6Axis(IOIF_I2C2) > 0) {
	        //TODO: ERROR PROCESS
	    	testImu6AxisRes = IOIF_IMU6AXIS_STATUS_ERROR;
	    }
	    if(IOIF_InitMag(IOIF_I2C1) > 0) {
	        //TODO: ERROR PROCESS
	    	testImu6AxisRes = IOIF_IMU3AXIS_STATUS_ERROR;
	    }
	}
#endif

#ifdef WIDM_ENABLED
	if (widmModuleObj == WIDM_IMU_U5){
	    if(IOIF_Init6Axis(IOIF_I2C1) > 0) {
	        //TODO: ERROR PROCESS
	    	testImu6AxisRes = IOIF_IMU6AXIS_STATUS_ERROR;
	    }
	    if(IOIF_InitMag(IOIF_I2C3) > 0) {
	        //TODO: ERROR PROCESS
	    	testImu3AxisRes = IOIF_IMU3AXIS_STATUS_ERROR;
	    }
	}
#endif
}

#ifdef SUIT_MD_ENABLED
/* Setting for the B-Flag PDO */
static void Send_BFlag(uint8_t nodeID)
{
	if ( nodeID == NODE_ID_LH_SAG ||  nodeID == NODE_ID_LK ||  nodeID == NODE_ID_LA_MED) {
		 // LH, LK, LA
//		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_L_B1FLAG,	e_UInt8,	1,    &B1Flag);
//		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_L_B2FLAG,	e_UInt8,	1,    &B2Flag);
//		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_L_B3FLAG,	e_UInt8,	1,    &B3Flag);
//		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_L_B4FLAG,	e_UInt8,	1,    &B4Flag);
//		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_L_B5FLAG,	e_UInt8,	1,    &B5Flag);
//		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_L_B6FLAG,	e_UInt8,	1,    &B6Flag);
//		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_L_B7FLAG,	e_UInt8,	1,    &B7Flag);
//		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_L_B8FLAG,	e_UInt8,	1,    &B8Flag);
//		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_L_B9FLAG,	e_UInt8,	1,    &B9Flag);
//		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_L_B10FLAG,	e_UInt8,	1,    &B10Flag);
//		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_L_B11FLAG,	e_UInt8,	1,    &B11Flag);
//		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_L_B12FLAG,	e_UInt8,	1,    &B12Flag);
//		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_L_B13FLAG,	e_UInt8,	1,    &B13Flag);
//		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_L_B14FLAG,	e_UInt8,	1,    &B14Flag);
//		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_L_B15FLAG,	e_UInt8,	1,    &B15Flag);
  	} else if ( nodeID == NODE_ID_RH_SAG ||  nodeID == NODE_ID_RK ||  nodeID == NODE_ID_RA_MED) {
  		 // RH, RK, RA
//		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_R_B1FLAG,	e_UInt8,	1,    &B1Flag);
//		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_R_B2FLAG,	e_UInt8,	1,    &B2Flag);
//		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_R_B3FLAG,	e_UInt8,	1,    &B3Flag);
//		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_R_B4FLAG,	e_UInt8,	1,    &B4Flag);
//		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_R_B5FLAG,	e_UInt8,	1,    &B5Flag);
//		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_R_B6FLAG,	e_UInt8,	1,    &B6Flag);
//		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_R_B7FLAG,	e_UInt8,	1,    &B7Flag);
//		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_R_B8FLAG,	e_UInt8,	1,    &B8Flag);
//		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_R_B9FLAG,	e_UInt8,	1,    &B9Flag);
//		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_R_B10FLAG,	e_UInt8,	1,    &B10Flag);
//		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_R_B11FLAG,	e_UInt8,	1,    &B11Flag);
//		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_R_B12FLAG,	e_UInt8,	1,    &B12Flag);
//		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_R_B13FLAG,	e_UInt8,	1,    &B13Flag);
//		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_R_B14FLAG,	e_UInt8,	1,    &B14Flag);
//		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_R_B15FLAG,	e_UInt8,	1,    &B15Flag);
  	}
}
#endif

/* Setting for Initial values of "angle" variables after get initial thigh angle */
static void SetInitialAngle(WIDM_AngleData_t* widmAngleData, WIDM_NormData_t* widmNormData, WIDM_QuatData_t* widmQuatData, float initialAngle)
{
	widmAngleData->degAccFiltered 	= initialAngle;
	widmAngleData->degGyrFiltered	= 0.0;

	widmAngleData->degTvcfFiltered = initialAngle; // Assume the initial state is no-motion or slow-motion

	widmAngleData->degLPF1st[0] 	= initialAngle;
	widmAngleData->degLPF1st[1] 	= initialAngle;
	widmAngleData->degLPF2nd[0] 	= initialAngle;
	widmAngleData->degLPF2nd[1] 	= initialAngle;
	widmNormData->degOri 	 		= initialAngle;

	widmQuatData->quatGyr[1].w = 1;
	widmQuatData->quatGyr[1].x = 0;
	widmQuatData->quatGyr[1].y = 0;
	widmQuatData->quatGyr[1].z = 0;

	widmQuatData->quatGyr[2].w = 1;
	widmQuatData->quatGyr[2].x = 0;
	widmQuatData->quatGyr[2].y = 0;
	widmQuatData->quatGyr[2].z = 0;
}

/*
 *Function to calculate the initial thigh angle - IMU case
*/
static void GetInitialAngle_IMU(IOIF_6AxisData_t* imu6AxisData, WIDM_SensorData_t* widmSensorData, WIDM_AngleData_t* widmAngleData, WIDM_NormData_t* widmNormData, WIDM_AttachCase_t widmAttachCase, WIDM_QuatData_t* widmQuatData)
{
	uint8_t tTotalSamples 	= 100;
	uint8_t tDataCheck_IMU	= 0;
	uint8_t tRealSamples	= 0;
	float tAccumulatedAngle = 0.0;
	float tInitThighAngle	= 0.0;
	float tImuAngle			= 0.0;

	for (uint8_t i = 1; i <= tTotalSamples; i++){
        for (uint16_t j = 0; j < 30000; j++){
        	// For delay of DMA reading
        }

        tDataCheck_IMU = IOIF_Get6AxisValue(&imu6AxisDataObj, &imu_params);

        if (tDataCheck_IMU == 0){
        	widmSensorData->accX[0] = imu6AxisData->accX;
        	widmSensorData->accY[0] = imu6AxisData->accY;
        	widmSensorData->gyrZ[0] = imu6AxisData->gyrZ;

			tImuAngle = WIDM_AttachCaseSetting_WalkONS(widmSensorData, widmAttachCase);
			tAccumulatedAngle += tImuAngle;

			tRealSamples++;
        }
    }

	tInitThighAngle = tAccumulatedAngle / tRealSamples;
	widmAngleData->initAngle = tInitThighAngle;
  SetInitialAngle(widmAngleData, widmNormData, widmQuatData, tInitThighAngle);
}


static void GetInitialAngle(WIDM_Module_t widmModule)
{
#ifdef IMU_MODE
	/* [IMU only] case */
	if (widmModule == WIDM_IMU_U5 || widmModule == WIDM_IMU_SAM){
		GetInitialAngle_IMU(&imu6AxisDataObj, &widmSensorDataObj, &widmAngleDataObj, &widmNormDataObj, widmAttachCaseObj, &widmQuatDataObj);
	}
#endif

	/* [IMU + ABS] case */
#ifdef IMUABS_MODE
	if (widmModule == WIDM_IMUABS_SAM){
		IOIF_SetAbsOffset(IOIF_SPI1, &AbsObj1);		// 3times -> Prevent the case that it is not properly offset
		IOIF_SetAbsOffset(IOIF_SPI1, &AbsObj1);
		IOIF_SetAbsOffset(IOIF_SPI1, &AbsObj1);
		GetInitialAngle_IMUABS(&imu6AxisDataObj, &AbsObj1, &widmSensorDataObj, &widmAngleDataObj, &widmNormDataObj, widmAttachCaseObj);
	}
#endif

	/* [IMU + INC] case */
#ifdef IMUINC_MODE
	if (widmModule == WIDM_IMUINC_SAM){
		// Maybe Incremental encoder's initial offset is completed in "low_level_ctrl_task".
		GetInitialAngle_IMUINC(&imu6AxisDataObj, &widmSensorDataObj, &widmAngleDataObj, &widmNormDataObj, widmAttachCaseObj);
	}
#endif
}


/* Just get 6Axis & 3Axis IMU values */
static int RunGetIMUFunction(void)
{
	testImu6AxisRes = IOIF_Get6AxisValue(&imu6AxisDataObj, &imu_params);
	testImu3AxisRes = IOIF_GetMagValue(&magDataObj);
	return 0;
}


/* Old version of WIDM algorithm */
//static int RunTotalGaitFunction(void)
//{
//	WIDM_UpdateBuffer(&widmSensorDataObj, &widmAngleDataObj, &widmGaitDataObj, &widmNormDataObj);
//
//	uint8_t t6AxisRes = IOIF_Get6AxisValue(&imu6AxisDataObj, &imu_params);;
//	if (t6AxisRes != IOIF_IMU6AXIS_STATUS_OK) {
//		return t6AxisRes;
//	}
//
//	UpdateSensorRawData(&widmSensorDataObj, &imu6AxisDataObj, widmAttachCaseObj);
//
//	WIDM_CalculateFuzzyInput(&widmSensorDataObj, &widmFuzzyDataObj);
//	wcDebug = WIDM_CalculateFuzzyWc(&widmFuzzyDataObj);
//
//	RunTvcfFilter(&widmSensorDataObj, &widmAngleDataObj, &widmFuzzyDataObj, WIDM_CONTROL_PERIOD, widmAttachCaseObj);
//
//#ifdef IMUABS_MODE
//	// For IMU + Encoder(ABS or INC) case //
//	if (widmModuleObj == WIDM_IMUABS_SAM){
//		// [Get Angle by using ABS] //
//		GetABSLinkData(&AbsObj1, &widmAngleDataObj);
//		// [Combining the results of IMU & ABS] //
//		CompensateIMUABS(&widmAngleDataObj);
//	}
//#endif
//
//#ifdef IMUINC_MODE
//	if (widmModuleObj == WIDM_IMUINC_SAM){
//		// [Get Angle by using INC] //
//		GetINCLinkData(&widmAngleDataObj);
//		// [Combining the results of IMU & INC] //
//		CompensateIMUINC(&widmAngleDataObj);
//	}
//#endif
//
//	SetUsedDegVel(&widmAngleDataObj, widmModuleObj);
//
//	NoiseReduction(&widmAngleDataObj, &widmGaitDataObj, widmModuleObj);
//
//	GetGaitPhase(&widmAngleDataObj, &widmNormDataObj, &widmGaitDataObj);
//
//	CheckWalkingState(&widmAngleDataObj, &widmNormDataObj, &widmThresDataObj, &widmGaitDataObj);
//
//	if (widmGaitDataObj.gaitPeriod > 3000){
//		widmGaitDataObj.gaitPeriod = 3000;
//	}
//
//	return 0;
//}


/* Prof version */
//static int RunTotalGaitFunction_Prof(void)
//{
//	WIDM_UpdateBuffer(&widmSensorDataObj, &widmAngleDataObj, &widmGaitDataObj, &widmNormDataObj);
//
//	uint8_t t6AxisRes = IOIF_Get6AxisValue(&imu6AxisDataObj, &imu_params);
//	if (t6AxisRes != IOIF_IMU6AXIS_STATUS_OK) {
//		return t6AxisRes;
//	}
//
//	UpdateSensorRawData(&widmSensorDataObj, &imu6AxisDataObj, widmAttachCaseObj);
//
//	WIDM_CalculateFuzzyInput(&widmSensorDataObj, &widmFuzzyDataObj);
//	wcDebug = WIDM_CalculateFuzzyWc(&widmFuzzyDataObj);
//
//	RunTvcfFilter(&widmSensorDataObj, &widmAngleDataObj, &widmFuzzyDataObj, WIDM_CONTROL_PERIOD, widmAttachCaseObj);
//
//#ifdef IMUABS_MODE
//	// For IMU + Encoder(ABS or INC) case //
//	if (widmModuleObj == WIDM_IMUABS_SAM){
//		// [Get Angle by using ABS] //
//		GetABSLinkData(&AbsObj1, &widmAngleDataObj);
//		// [Combining the results of IMU & ABS] //
//		CompensateIMUABS(&widmAngleDataObj);
//	}
//#endif
//
//#ifdef IMUINC_MODE
//	if (widmModuleObj == WIDM_IMUINC_SAM){
//		// [Get Angle by using INC] //
//		GetINCLinkData(&widmAngleDataObj);
//		// [Combining the results of IMU & INC] //
//		CompensateIMUINC(&widmAngleDataObj);
//	}
//#endif
//
//	SetUsedDegVel(&widmAngleDataObj, widmModuleObj);
//
//	if (WIDM_MODULE_NODE_ID == NODE_ID_LK || WIDM_MODULE_NODE_ID == NODE_ID_RK){
//		GaitFunction_Prof_1029_K10(&widmAngleDataObj, &widmSensorDataObj);
//	}
//	else if (WIDM_MODULE_NODE_ID == NODE_ID_LK || WIDM_MODULE_NODE_ID == NODE_ID_RK){
//		GaitFunction_Prof_1029_H10(&widmAngleDataObj);
//	}
//
//
////	CheckWalkingState_Prof();
//
//	// Convert Data format //
////	saveDegRaw = widmAngleDataObj.degFinal * 100;
////	saveVelRaw = widmAngleDataObj.velFinal * 10;
////	savePeriodCheck = cutoffFreqSmooth * 1000;
////	saveGyrZ = widmSensorDataObj.gyrZ[0] * 10;
////	saveDegNorm = angleNorm * 1000;
//
//	return 0;
//}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Setting for initial parameters */
//static void InitValueSetting(WIDM_FuzzyData_t* widmFuzzyData, WIDM_NormData_t* widmNormData, WIDM_GaitData_t* widmGaitData, WIDM_ThresData_t* widmThresData, WIDM_Module_t widmModule)
//{
//	widmFuzzyData->var[0] 			= 8.0;
//	widmFuzzyData->var[1] 			= 30.0;
//	widmFuzzyData->var[2] 			= 5.8;
//	widmFuzzyData->var[3] 			= 320.0;
//
//	widmNormData->ampDeg 			= 30.0; 	//30
//	widmNormData->ampDegLPF[0]		= 30.0;
//	widmNormData->ampDegLPF[1]		= 30.0;
//
//	widmNormData->ampVel 			= 400.0; 	//400
//	widmNormData->ampVelLPF[0]		= 400.0;
//	widmNormData->ampVelLPF[1]		= 400.0;
//
//
//	widmGaitData->gaitPeriod 		= 1000;
//	widmGaitData->gaitPhase 		= -100.0;
//	widmGaitData->gaitPhasePre   	= -100.0;
//
//
//	if (widmModule == WIDM_IMU_U5){
//		widmThresData->degThStart	= 5.0;
//		widmThresData->velThStart	= 20.0;
//		widmThresData->degThStop 	= 5.0;
//		widmThresData->velThStop 	= 3.0;
//	}
//	else if (widmModule == WIDM_IMU_SAM){
//		widmThresData->degThStart	= 12.0;
//		widmThresData->velThStart	= 20.0;
//		widmThresData->degThStop 	= 3.0;
//		widmThresData->velThStop 	= 12.0;
//	}
//	else if (widmModule == WIDM_IMUABS_SAM){
//		widmThresData->degThStart	= 10.0;		//15.0
//		widmThresData->velThStart	= 40.0;		//60.0
//		widmThresData->degThStop 	= 8.0;		//5.0
//		widmThresData->velThStop 	= 10.0;		//25.0
//	}
//	else if (widmModule == WIDM_IMUINC_SAM){
//		widmThresData->degThStart	= 10.0;		//15.0
//		widmThresData->velThStart	= 40.0;		//60.0
//		widmThresData->degThStop 	= 8.0;		//5.0
//		widmThresData->velThStop 	= 10.0;		//25.0
//	}
//}

/*
*The function UpdateSensorRawData updates the IMU raw values.
*/
static void UpdateSensorRawData(WIDM_SensorData_t* widmSensorData, IOIF_6AxisData_t* imu6AxisData, WIDM_AttachCase_t widmAttachCase)
{
	widmSensorData->accX[0] = imu6AxisData->accX;
	widmSensorData->accY[0] = imu6AxisData->accY;
	widmSensorData->gyrZ[0] = imu6AxisData->gyrZ;

	if (widmAttachCase < 8){
		widmSensorData->gyrZ[0] = (-1) * (imu6AxisData->gyrZ); 	// For Negative Gyro case (Maybe LEFT case)
	}
	else if (widmAttachCase >= 8){
		widmSensorData->gyrZ[0] = imu6AxisData->gyrZ; 			// For Positive Gyro case (Maybe RIGHT case)
	}
}

#if defined(WALKON5_MD_ENABLED) || defined(L30_MD_REV06_ENABLED) || defined(L30_MD_REV07_ENABLED) || defined(L30_MD_REV08_ENABLED) || defined(SUIT_MD_ENABLED)
/*
*The function GetABSLinkData considers the Absolute Encoder's raw values.
*/
//static void GetABSLinkData(IOIF_AbsEnc_t* absEnc, WIDM_AngleData_t* widmAngleData)
//{
//	// Left Leg //
//	if (widmAttachCaseObj < 8){
//		widmAngleData->degABS 	= absEnc->posDegMultiTurn * 90 / 168.75;
//		widmAngleData->velABS   = absEnc->velDeg * 90 / 168.75;
//	}
//
//	// Right Leg //
//	else if (widmAttachCaseObj >= 8){
//		widmAngleData->degABS 	= absEnc->posDegMultiTurn * 90 / 168.75 * (-1);
//		widmAngleData->velABS   = absEnc->velDeg * 90 / 168.75 * (-1);
//	}
//}

#ifdef IMUABS_MODE
/*
 *Function to compensate IMU+ABS case
*/
static void CompensateIMUABS(WIDM_AngleData_t* widmAngleData)
{
	widmAngleData->degIMUABS = widmAngleData->degTvcf[0] + widmAngleData->degABS;
	widmAngleData->velIMUABS = widmAngleData->velRaw[0]  + widmAngleData->velABS;
}

/*
 *Function to calculate the initial thigh angle - IMU + ABS case
*/
static void GetInitialAngle_IMUABS(IOIF_6AxisData_t* imu6AxisData, IOIF_AbsEnc_t* absEnc, WIDM_SensorData_t* widmSensorData, WIDM_AngleData_t* widmAngleData, WIDM_NormData_t* widmNormData, WIDM_AttachCase_t widmAttachCase)
{
	uint8_t tTotalSamples 			= 100;
	uint8_t tDataCheck_IMU 			= 0;
	uint8_t tRealSamples			= 0;
	float tAccumulatedAngle_IMU 	= 0.0;
	float tAccumulatedAngle_ABS 	= 0.0;
	float tAccumulatedAngle_IMUABS 	= 0.0;

	float tInitAngle_IMU 			= 0.0;
	float tInitAngle_ABS 			= 0.0;
	float tInitAngle_IMUABS 		= 0.0;
	float tInitThighAngle			= 0.0;


	for (uint8_t i = 1; i <= tTotalSamples; i++){
		for (int j = 0; j < 100000; j++){
			// For Delay of DMA reading
		}
        tDataCheck_IMU = IOIF_Get6AxisValue(imu6AxisData);
        if (tDataCheck_IMU == 0){
        	widmSensorData->accX[0] = imu6AxisData->accX;
        	widmSensorData->accY[0] = imu6AxisData->accY;
        	widmSensorData->gyrZ[0] = imu6AxisData->gyrZ;

        	float tImuAngle = WIDM_AttachCaseSetting(widmSensorData, widmAttachCase);
        	tAccumulatedAngle_IMU += tImuAngle;

        	GetABSLinkData(absEnc, widmAngleData);
        	tAccumulatedAngle_ABS += widmAngleData->degABS;
            tAccumulatedAngle_IMUABS += (tImuAngle + widmAngleData->degABS);

        	tRealSamples++;
        }
    }

	tInitAngle_IMU 		= tAccumulatedAngle_IMU / tRealSamples;
	tInitAngle_ABS 		= tAccumulatedAngle_ABS / tRealSamples;
	tInitAngle_IMUABS 	= tAccumulatedAngle_IMUABS / tRealSamples;

	widmAngleData->initAngle = tInitAngle_IMUABS;
	tInitThighAngle			 = tInitAngle_IMUABS;

	SetInitialAngle(widmAngleData, widmNormData, tInitThighAngle);
}
#endif

/*
*The function GetINCLinkData considers the Incremental Encoder's raw values.
*/
//static void GetINCLinkData(WIDM_AngleData_t* widmAngleData)
//{
//	// Left Leg //
//	if (widmAttachCaseObj < 8){
//		widmAngleData->degINC 	= motor_out.position / WIDM_PI * 180.0;		// If gear ratio is correctly selected (Suit = 18.75:1)
//		widmAngleData->velINC   = (widmAngleData->degINC - widmAngleData->degINCPrev) / 0.001;
//	}
//
//	// Right Leg //
//	else if (widmAttachCaseObj >= 8){
//		widmAngleData->degINC 	= motor_out.position / WIDM_PI * 180.0 * (-1);
//		widmAngleData->velINC   = (widmAngleData->degINC - widmAngleData->degINCPrev) / 0.001;
//	}
//
//	widmAngleData->degINCPrev = widmAngleData->degINC;		// 1ms Update Inc degree value
//}

#ifdef IMUINC_MODE
/*
 *Function to compensate IMU + INC case
*/
static void CompensateIMUINC(WIDM_AngleData_t* widmAngleData)
{
	widmAngleData->degIMUINC = widmAngleData->degTvcf[0] + widmAngleData->degINC;
	widmAngleData->velIMUINC = widmAngleData->velRaw[0]  + widmAngleData->velINC;
}

/*
 *Function to calculate the initial thigh angle - IMU + INC case
*/
static void GetInitialAngle_IMUINC(IOIF_6AxisData_t* imu6AxisData, WIDM_SensorData_t* widmSensorData, WIDM_AngleData_t* widmAngleData, WIDM_NormData_t* widmNormData, WIDM_AttachCase_t widmAttachCase)
{
	uint8_t tTotalSamples 			= 100;
	uint8_t tDataCheck_IMU 			= 0;
	uint8_t tRealSamples			= 0;
	float tAccumulatedAngle_IMU 	= 0.0;
	float tAccumulatedAngle_INC 	= 0.0;
	float tAccumulatedAngle_IMUINC 	= 0.0;

	float tInitAngle_IMU 			= 0.0;
	float tInitAngle_INC 			= 0.0;
	float tInitAngle_IMUINC 		= 0.0;
	float tInitThighAngle			= 0.0;


	for (uint8_t i = 1; i <= tTotalSamples; i++){
		for (int j = 0; j < 100000; j++){
			// For Delay of DMA reading
		}
        tDataCheck_IMU = IOIF_Get6AxisValue(imu6AxisData);
        if (tDataCheck_IMU == 0){
        	widmSensorData->accX[0] = imu6AxisData->accX;
        	widmSensorData->accY[0] = imu6AxisData->accY;
        	widmSensorData->gyrZ[0] = imu6AxisData->gyrZ;

        	float tImuAngle = WIDM_AttachCaseSetting(widmSensorData, widmAttachCase);
        	tAccumulatedAngle_IMU += tImuAngle;

        	GetINCLinkData(widmAngleData);
        	tAccumulatedAngle_INC += widmAngleData->degINC;
            tAccumulatedAngle_IMUINC += (tImuAngle + widmAngleData->degINC);

        	tRealSamples++;
        }
    }

	tInitAngle_IMU 		= tAccumulatedAngle_IMU / tRealSamples;
	tInitAngle_INC 		= tAccumulatedAngle_INC / tRealSamples;
	tInitAngle_IMUINC 	= tAccumulatedAngle_IMUINC / tRealSamples;

	widmAngleData->initAngle = tInitAngle_IMUINC;
	tInitThighAngle			 = tInitAngle_IMUINC;

	SetInitialAngle(widmAngleData, widmNormData, tInitThighAngle);
}
#endif

#endif

/*
 *Function to execute the time-varying complementary filter (with Fuzzy Logic - wc)
*/
static void RunTvcfFilter(WIDM_SensorData_t* widmSensorData, WIDM_AngleData_t* widmAngleData, WIDM_FuzzyData_t* widmFuzzyData, float samplingPeriod, WIDM_AttachCase_t widmAttachCase)
{
	/* Apply time-varying complementary filter on the sensor data using fuzzy logic(wc) and update the thigh angle parameters */
	WIDM_RunTVCF(widmSensorData, widmAngleData, widmFuzzyData->wc, samplingPeriod, widmAttachCase);

	/* Update the unfiltered thigh angle to be the same as the filtered thigh angle */
	widmAngleData->degTvcf[0] = widmAngleData->degTvcfFiltered;
	widmAngleData->velRaw[0] = (widmAngleData->degTvcf[0] - widmAngleData->degTvcf[1]) / WIDM_CONTROL_PERIOD;
}

/*
 *Function to select finally used deg&vel value before filtering
*/
static void SetUsedDegVel(WIDM_AngleData_t* widmAngleData, WIDM_Module_t widmModule)
{
	if (widmModule == WIDM_IMU_U5 || widmModule == WIDM_IMU_SAM){
		widmAngleData->degFinal = widmAngleData->degTvcf[0];
		widmAngleData->velFinal = widmAngleData->velRaw[0];
	}
	else if (widmModule == WIDM_IMUABS_SAM){
		widmAngleData->degFinal = widmAngleData->degIMUABS;
		widmAngleData->velFinal = widmAngleData->velIMUABS;
	}
	else if (widmModule == WIDM_IMUINC_SAM){
		widmAngleData->degFinal = widmAngleData->degIMUINC;
		widmAngleData->velFinal = widmAngleData->velIMUINC;
	}
}

/*
 *Function to reduce noise in sensor data
*/
static void NoiseReduction(WIDM_AngleData_t* widmAngleData, WIDM_GaitData_t* widmGaitData, WIDM_Module_t widmModule)
{
	float dt = 1000.0;
	float wTarget = WIDM_GetMaxValue(0.3, dt/widmGaitData->gaitPeriod);
	float freqLPF = 1.2 * wTarget * 2 * WIDM_PI;

	/* First LPF filter on angle data */
	widmAngleData->degLPF1st[0] = WIDM_LPF(
		widmAngleData->degFinal,
		widmAngleData->degLPF1st[1],
		freqLPF,
		WIDM_CONTROL_PERIOD
	);

	/* Second LPF filter on angle data */
	widmAngleData->degLPF2nd[0] = WIDM_LPF(
		widmAngleData->degLPF1st[0],
		widmAngleData->degLPF2nd[1],
		freqLPF,
		WIDM_CONTROL_PERIOD
	);

	/* First LPF filter on velocity data */
	widmAngleData->velLPF1st[0] = WIDM_LPF(
		widmAngleData->velFinal,
		widmAngleData->velLPF1st[1],
		freqLPF,
		WIDM_CONTROL_PERIOD
	);

	/* Second LPF filter on velocity data */
	widmAngleData->velLPF2nd[0] = WIDM_LPF(
		widmAngleData->velLPF1st[0],
		widmAngleData->velLPF2nd[1],
		freqLPF,
		WIDM_CONTROL_PERIOD
	);
}

/*
 *Function to normalize sensor data and calculate the current phase of the gait
*/
static void GetGaitPhase(WIDM_AngleData_t* widmAngleData, WIDM_NormData_t* widmNormData, WIDM_GaitData_t* widmGaitData)
{
	/* Prepare for circular normalization by finding the center point of the original ellipse */
	WIDM_Normalization(widmAngleData, widmNormData, widmGaitData);

	float dt = 0.001;
	widmNormData->degOriLPF[0] = WIDM_LPF(widmNormData->degOri, widmNormData->degOriLPF[1], NORM_CUTOFF_FREQ, dt);
	widmNormData->velOriLPF[0] = WIDM_LPF(widmNormData->velOri, widmNormData->velOriLPF[1], NORM_CUTOFF_FREQ, dt);
	widmNormData->ampDegLPF[0] = WIDM_LPF(widmNormData->ampDeg, widmNormData->ampDegLPF[1], NORM_CUTOFF_FREQ, dt);
	widmNormData->ampVelLPF[0] = WIDM_LPF(widmNormData->ampVel, widmNormData->ampVelLPF[1], NORM_CUTOFF_FREQ, dt);

	/* Normalize degree and velocity data based on calculated origin and amplitude */
	widmNormData->degNorm = (widmAngleData->degLPF2nd[0] - widmNormData->degOriLPF[0]) / widmNormData->ampDegLPF[0];
	widmNormData->velNorm = (widmAngleData->velLPF2nd[0] - widmNormData->velOriLPF[0]) / widmNormData->ampVelLPF[0];

	/* Calculate and update the current phase of the gait */
	widmGaitData->gaitPhase = WIDM_GetGaitPhase(widmNormData, widmGaitData); // Current phase (0 ~ 100%)
}

/*
 *This function calculates and returns the phase radius
*/
static float GetPhaseRadius(float degDiff, float degTh, float velDiff, float velTh)
{
    /* Calculate degree ratio */
//    float degRatio = degDiff / degTh;

    /* Calculate velocity ratio */
//    float velRatio = velDiff / velTh;

    /* Calculate and return the phase radius */
//    return WIDM_SquareRootSum(degRatio, velRatio);
}

/*
 *This function updates the walking state based on the phase radii and sum_i
*/
static void UpdateWalkingState(WIDM_GaitData_t* widmGaitData, float phaseRadiusStart, float phaseRadiusStop, int16_t sumIter)
{
    /* The walking state is updated based on the current walking state, phase radii, and t_sum_i */
    switch (widmGaitData->walkingState)
    {
        case WIDM_STOP:
            /* If the start phase radius is greater than 1, set the walking state to 1 */
            if (phaseRadiusStart > 1){
            	widmGaitData->walkingState = WIDM_WALKING_START;
            }
            break;
        case WIDM_WALKING_START:
            /* If sum_i is greater than 1000, set the walking state to 2 */
            if (sumIter > 700){
            	widmGaitData->walkingState = WIDM_WALKING_HALF_CYCLE;
            }
            break;
        case WIDM_WALKING_HALF_CYCLE:
            /* If sum_i is 0, set the walking state to 3 */
            if (sumIter == 0){
            	widmGaitData->walkingState = WIDM_WALKING_ONE_CYCLE;
            }
            break;
        default:
            /* If the stop phase radius is less than 1, set the walking state to 0 */
//            if (phaseRadiusStop < 1){
//            	widmGaitData->walkingState = WIDM_STOP;
//            }
            if (phaseRadiusStop < 1){
            	stopCnt++;
            	if (stopCnt >= 250){
            		widmGaitData->walkingState = WIDM_STOP;
            		stopCnt = 0;
            	}
            }
            else{
            	stopCnt = 0;
            }
            break;
    }
}

/*
*This function checks the walking state using the walking parameters and IMU system information
*/
static void CheckWalkingState(WIDM_AngleData_t* widmAngleData, WIDM_NormData_t* widmNormData, WIDM_ThresData_t* widmThresData, WIDM_GaitData_t* widmGaitData)
{
	float degDiff = 0.0;
	float velDiff = 0.0;
	float phaseRadiusStart = 0.0;
	float phaseRadiusStop = 0.0;

    /* Get the relevant values from the walking parameters and IMU system */
    degDiff = widmAngleData->degLPF2nd[0] - widmNormData->degOri;
    velDiff = widmAngleData->velLPF2nd[0] - widmNormData->velOri;

    // For Debugging Threshold values //
    deg_diff = degDiff;
    vel_diff = velDiff;

    /* Calculate the start and stop phase radii */
    phaseRadiusStart = GetPhaseRadius(degDiff, widmThresData->degThStart, velDiff, widmThresData->velThStart);
    phaseRadiusStop  = GetPhaseRadius(degDiff, widmThresData->degThStop, velDiff, widmThresData->velThStop);

    /* Update the walking state based on the phase radii and sum_i */
    UpdateWalkingState(widmGaitData, phaseRadiusStart, phaseRadiusStop, widmNormData->sumIter);

    /* If the walking state is 0 or 1, set the gait phase to -100 */
    if (widmGaitData->walkingState == WIDM_STOP || widmGaitData->walkingState == WIDM_WALKING_START){
        widmGaitData->gaitPhase = -100;
    }
}


///* ------------------- SDO CALLBACK ---------------------*/
//static void SetAbsOffsetCmd(MsgSDOargs* req, MsgSDOargs* res)
//{
//	memcpy(&absOffsetCmd, req->data, 1);
//
//	res->size = 0;
//	res->status = DATA_OBJECT_SDO_SUCC;
//}

static void SetAccScaleFactor_X(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&imu_params.accScaleFactor[0], req->data, 4);

	res->size = 0;
	res->status = DATA_OBJECT_SDO_SUCC;
}

static void SetAccScaleFactor_Y(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&imu_params.accScaleFactor[1], req->data, 4);

	res->size = 0;
	res->status = DATA_OBJECT_SDO_SUCC;
}

static void SetAccScaleFactor_Z(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&imu_params.accScaleFactor[2], req->data, 4);

	res->size = 0;
	res->status = DATA_OBJECT_SDO_SUCC;
}

static void SetGyrScaleFactor_X(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&imu_params.gyrScaleFactor[0], req->data, 4);

	res->size = 0;
	res->status = DATA_OBJECT_SDO_SUCC;
}

static void SetGyrScaleFactor_Y(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&imu_params.gyrScaleFactor[1], req->data, 4);

	res->size = 0;
	res->status = DATA_OBJECT_SDO_SUCC;
}

static void SetGyrScaleFactor_Z(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&imu_params.gyrScaleFactor[2], req->data, 4);

	res->size = 0;
	res->status = DATA_OBJECT_SDO_SUCC;
}

static void SetAccBias_X(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&imu_params.accBias[0], req->data, 4);

	res->size = 0;
	res->status = DATA_OBJECT_SDO_SUCC;
}

static void SetAccBias_Y(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&imu_params.accBias[1], req->data, 4);

	res->size = 0;
	res->status = DATA_OBJECT_SDO_SUCC;
}

static void SetAccBias_Z(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&imu_params.accBias[2], req->data, 4);

	res->size = 0;
	res->status = DATA_OBJECT_SDO_SUCC;
}

static void SetGyrBias_X(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&imu_params.gyrBias[0], req->data, 4);

	res->size = 0;
	res->status = DATA_OBJECT_SDO_SUCC;
}

static void SetGyrBias_Y(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&imu_params.gyrBias[1], req->data, 4);

	res->size = 0;
	res->status = DATA_OBJECT_SDO_SUCC;
}

static void SetGyrBias_Z(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&imu_params.gyrBias[2], req->data, 4);

	res->size = 0;
	res->status = DATA_OBJECT_SDO_SUCC;
}

static void SetGyrSign_X(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&imu_params.gyrSign[0], req->data, 2);

	res->size = 0;
	res->status = DATA_OBJECT_SDO_SUCC;
}

static void SetGyrSign_Y(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&imu_params.gyrSign[1], req->data, 2);

	res->size = 0;
	res->status = DATA_OBJECT_SDO_SUCC;
}

static void SetGyrSign_Z(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&imu_params.gyrSign[2], req->data, 2);

	res->size = 0;
	res->status = DATA_OBJECT_SDO_SUCC;
}

static void SetFuzzy_wcl(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&widmFuzzyDataObj.wl, req->data, 4);

	res->size = 0;
	res->status = DATA_OBJECT_SDO_SUCC;
}

static void SetFuzzy_wch(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&widmFuzzyDataObj.wh, req->data, 4);

	res->size = 0;
	res->status = DATA_OBJECT_SDO_SUCC;
}

static void SetFuzzy_alpha(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&widmFuzzyDataObj.alpha, req->data, 4);

	res->size = 0;
	res->status = DATA_OBJECT_SDO_SUCC;
}

static void SetFuzzy_s(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&widmFuzzyDataObj.s, req->data, 4);

	res->size = 0;
	res->status = DATA_OBJECT_SDO_SUCC;
}

static void SetFuzzy_m0(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&widmFuzzyDataObj.m0, req->data, 4);

	res->size = 0;
	res->status = DATA_OBJECT_SDO_SUCC;
}

static void SetFuzzy_acc_mean(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&widmFuzzyDataObj.acc_mean, req->data, 4);

	res->size = 0;
	res->status = DATA_OBJECT_SDO_SUCC;
}

static void SetFuzzy_wc_m(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&widmFuzzyDataObj.wc_m, req->data, 4);

	res->size = 0;
	res->status = DATA_OBJECT_SDO_SUCC;
}

static void SetFuzzy_wc_debug(MsgSDOargs* req, MsgSDOargs* res)
{
	memcpy(&widmFuzzyDataObj.wc_debug, req->data, 4);

	res->size = 0;
	res->status = DATA_OBJECT_SDO_SUCC;
}

static void Start_IMU_Calibration(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&widmSensorDataObj.start_calibration, t_req->data, 1);

	t_res->size = 0;
	t_res->status = DATA_OBJECT_SDO_SUCC;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#ifdef SUIT_MD_ENABLED
// 10.29 version //
static void GaitFunction_Prof_1029_K10(WIDM_AngleData_t* widmAngleData, WIDM_SensorData_t* widmSensorData)
{
	k++; 			// Check this Loop Count
	a = k/1000;		// For Debug, check each 1sec

	velLPF = WIDM_LPF_walking_Prof((double)widmAngleData->velFinal);			// velLPF : velRaw -> Low-pass filtering
	velLPF0ABS = WIDM_Abs_double(velLPF);
	double velLPFAbs = WIDM_GetMaxValue_double(0, velLPF0ABS-15);


	GetINCLinkData(&widmAngleDataObj);
	incDegTrig[0] = widmAngleDataObj.degINC;
	incVel = widmAngleDataObj.velINC;

	filteredIncDeg = WIDM_Abs_double(widmAngleDataObj.degINC);
	filteredIncDeg = WIDM_LPF_walking_Prof(widmAngleDataObj.degINC);
	incDegTrig[0] = filteredIncDeg;

	filteredIncVel = WIDM_Abs_double(incVel);
	filteredIncVel = WIDM_LPF_walking_Prof(incVel);
	incVelTrig[0] = filteredIncVel;

	modeCheck = 0.993 * modeCheck + 0.007 * velLPFAbs;

	if (k > 1){
		degLPF[0] = 0.98 * degLPF[1] + 0.02 * widmAngleData->degFinal;

		mode[0] = mode[1];

		if (k > 1000){
			if (mode[1] == 0 && modeCheck > 4){									// mode = 1 : Walking, mode = 0 : Stop  // modeCheck = 4
				mode[0] = 1;
			}
			else if (mode[1] == 1 && modeCheck < 1){
				mode[0] = 0;
			}
		}

		gyroLPF[0] = 0.98 * gyroLPF[1] + 0.02 * widmSensorData->gyrZ[0];
		velLPF2[0] = 0.9997 * velLPF2[1] + 0.0003 * gyroLPF[0];
	}

	if (k > 10){
		if (mode[0] == 1){
			if (velLPF2[1] > 0 && velLPF2[0] < 0){								// Leg front->back moving point
				if (firstPeriodCheck == 0){
					Period = k - timeStampPrev;
					if (Period > 2000){
						Period = 2000;
					}
					else if (Period < 200){
						Period = 200;
					}
				}
				else{
					firstPeriodCheck = 0;
				}

				cutoffFreq = 2 * WIDM_PI / (Period * 0.001);
				timeStampPrev = k;
			}
		}
		else if (mode[0] == 0){
			firstPeriodCheck = 1;
		}
	}

	cutoffFreqSmooth = 0.992 * cutoffFreqSmooth + 0.008 * cutoffFreq;
	degBPF[0] = WIDM_BPF_Peak_Prof_1((double)widmAngleData->degFinal, cutoffFreqSmooth);
	degBPF0ABS = WIDM_Abs_double(degBPF[0]);

	if (k > 3){
		if (mode[0] == 1 && firstPeriodCheck == 0 && degBPF[1] < degBPF[2] && degBPF[0] > degBPF[1]){
			PeakAmp = degBPF0ABS;			// At Leg Backward->Forward transition, get Amplitude of angle(deg)
		}

		velBPF = WIDM_BPF_Peak_Prof_2(widmAngleData->velFinal, cutoffFreqSmooth);
		velBPF0ABS = WIDM_Abs_double(velBPF);

		if (mode[0] == 1 && firstPeriodCheck == 0 && degBPF[1] < 0 && degBPF[0] > 0){
			PeakWAmp = velBPF0ABS;			// At maximum slope of degBPF, get Amplitude of velocity(deg/s)
		}
	}

	if (mode[0] == 1){
		PeakAmpSmooth = 0.998 * PeakAmpSmooth + 0.002 * PeakAmp;
		PeakWAmpSmooth = 0.998 * PeakWAmpSmooth + 0.002 * PeakWAmp;
	}

	if (k > 1000){
		angleNorm = degBPF[0] / PeakAmpSmooth;
		velocityNorm = - velBPF / PeakWAmpSmooth;
	}

	gaitPhase = mode[0] * (atan2( (-1)*velocityNorm, (-1)*angleNorm)) / (2*WIDM_PI) * 100 + 50;  // 0~100%		// "1" : 50~75%, "2" : 75~100%, "3" : 0~25%, "4" : 25~50% ....

	B2Flag = 0;			// 10%
	B3Flag = 0;			// 20%
	B4Flag = 0;			// 30%
	B5Flag = 0;			// 40%
	B6Flag = 0;			// 50%

	B7Flag = 0;			// 0%	 Leg front->back	&   Gait Count ++
	B8Flag = 0;			// 50%	 Leg back->front
	B9Flag = 0;			// 75%	 Leg middle->front
	B10Flag = 0;		// 25%	 Leg middle->back

	B11Flag = 0;		// 60%
	B12Flag = 0;		// 70%
	B13Flag = 0;		// 80%
	B14Flag = 0;		// 90%

	B15Flag = 0;		// after B8 Flag on, swing for extension


	if (k > 1000 && mode[0] == 1){
		if (gaitPhase > phaseThreshold1 && gaitPhasePrev < phaseThreshold1){
			B2Flag = 1;
		}
		if (gaitPhase > phaseThreshold2 && gaitPhasePrev < phaseThreshold2){
			B3Flag = 1;
		}
		if (gaitPhase > phaseThreshold3 && gaitPhasePrev < phaseThreshold3){
			B4Flag = 1;
		}
		if (gaitPhase > phaseThreshold4 && gaitPhasePrev < phaseThreshold4){
			B5Flag = 1;
		}
		if (gaitPhase > phaseThreshold5 && gaitPhasePrev < phaseThreshold5){
			B6Flag = 1;
		}
		if (gaitPhase > phaseThreshold6 && gaitPhasePrev < phaseThreshold6){
			B11Flag = 1;
		}
		if (gaitPhase > phaseThreshold7 && gaitPhasePrev < phaseThreshold7){
			B12Flag = 1;
		}
		if (gaitPhase > phaseThreshold8 && gaitPhasePrev < phaseThreshold8){
			B13Flag = 1;
		}
		if (gaitPhase > phaseThreshold9 && gaitPhasePrev < phaseThreshold9){
			B14Flag = 1;
		}
	}


	degLPF0ABS = WIDM_Abs_double(degLPF[0]);

	if (degLPF0ABS < 20 && mode[0] == 0){
		NeutralPosture = 0.999 * NeutralPosture + 0.001 * degLPF[0];				// Get (Stop & Standing) Posture's Angle
	}

	degUnbiased[0] = degLPF[0] - NeutralPosture;

	if (k > 1000){
		if (degUnbiased[0] > 10 && gyroLPF[1] > 0 && gyroLPF[0] < 0 && B7Finished == 0 && mode[0] == 1){
			B7Flag = 1;
			B7Finished = 1;
			gaitCount++;
		}
		if (degUnbiased[0] < 3 && gyroLPF[1] < 0 && gyroLPF[0] > 0 && B8Finished == 0 && mode[0] == 1){
			B8Flag = 1;
			B8Finished = 1;
			B8stack++;
		}
		if (degUnbiased[0] < 3 && degUnbiased[1] > 3){
			B7Finished = 0;
		}
		if (degUnbiased[0] > 5 && degUnbiased[1] < 5){
			B8Finished = 0;
		}

        if (gyroLPF[0] > 5){
            B9Flag = 1;
        }
        if (gyroLPF[0] < -5){
            B10Flag = 1;
        }

		if (B8Finished == 1) {
			if (incDegTrig[0] < incDegTrig[1] && incDegTrig[1] > incDegTrig[2] && incDegTrig[0] > 40) {
			// if (incVelTrig[0] < incVelTrig[1] && incVelTrig[1] > incVelTrig[2]) {
			// if (incDegTrig[0] < incDegTrig[1] && incDegTrig[1] > incDegTrig[2]) {
				// if (incVelTrig[0] < 0 && incVelTrig[1] > 0 && incVelTrig[1] > incVelTrig[2]) {
        			B15Flag = 1;
					B15stack++;
    			// }
				// B15Flag = 1;
				// B15stack++;
			}
		}
	}

	// Update previous values//
	mode[1] = mode[0];
	B1Flag = mode[0];
	gaitPhasePrev = gaitPhase;
	degLPF[1] = degLPF[0];
	gyroLPF[1] = gyroLPF[0];
	velLPF2[1] = velLPF2[0];
	degBPF[2] = degBPF[1];
	degBPF[1] = degBPF[0];
	degUnbiased[1] = degUnbiased[0];

	incDegTrig[2] = incDegTrig[1];
	incDegTrig[1] = incDegTrig[0];

	incVelTrig[2] = incVelTrig[1];
	incVelTrig[1] = incVelTrig[0];
}


static void GaitFunction_Prof_1029_H10(WIDM_AngleData_t* widmAngleData)
{
	k++; 			// Check this Loop Count
	a = k/1000;		// For Debug, check each 1sec

	velLPF_H10[0] = WIDM_LPF_walking_Prof((double)widmAngleData->velFinal);			// velLPF : velRaw -> Low-pass filtering
	if (k < 1000){
		velLPF_H10[0] = 0;
	}

	velLPF0ABS = WIDM_Abs_double(velLPF_H10[0]);
	double velLPFAbs = WIDM_GetMaxValue_double(0, velLPF0ABS-15);
	modeCheck = 0.993 * modeCheck + 0.007 * velLPFAbs;

	if (k > 1){
		degLPF[0] = 0.98 * degLPF[1] + 0.02 * widmAngleData->degFinal;

		mode[0] = mode[1];

		if (k > 1000){
			if (mode[1] == 0 && modeCheck > 10){									// mode = 1 : Walking, mode = 0 : Stop
				mode[0] = 1;
			}
			else if (mode[1] == 1 && modeCheck < 1){
				mode[0] = 0;
			}
		}

		velLPF2[0] = 0.9997 * velLPF2[1] + 0.0003 * velLPF_H10[0];
	}

	if (k > 10){
		if (mode[0] == 1){
			if (velLPF2[1] > 0 && velLPF2[0] < 0){								// Leg front->back moving point
				if (firstPeriodCheck == 0){
					Period = k - timeStampPrev;
					if (Period > 2000){
						Period = 2000;
					}
					else if (Period < 200){
						Period = 200;
					}
				}
				else{
					firstPeriodCheck = 0;
				}

				cutoffFreq = 2 * WIDM_PI / (Period * 0.001);
				timeStampPrev = k;
			}
		}
		else if (mode[0] == 0){
			firstPeriodCheck = 1;
		}
	}

	cutoffFreqSmooth = 0.992 * cutoffFreqSmooth + 0.008 * cutoffFreq;
	degBPF[0] = WIDM_BPF_Peak_Prof_1((double)widmAngleData->degFinal, cutoffFreqSmooth);
	degBPF0ABS = WIDM_Abs_double(degBPF[0]);

	if (k > 3){
		if (mode[0] == 1 && firstPeriodCheck == 0 && degBPF[1] < degBPF[2] && degBPF[0] > degBPF[1]){
			PeakAmp = degBPF0ABS;			// At Leg Backward->Forward transition, get Amplitude of angle(deg)
		}

		velBPF = WIDM_BPF_Peak_Prof_2(widmAngleData->velFinal, cutoffFreqSmooth);
		velBPF0ABS = WIDM_Abs_double(velBPF);

		if (mode[0] == 1 && firstPeriodCheck == 0 && degBPF[1] < 0 && degBPF[0] > 0){
			PeakWAmp = velBPF0ABS;			// At maximum slope of degBPF, get Amplitude of velocity(deg/s)
		}
	}

	if (mode[0] == 1){
		PeakAmpSmooth = 0.998 * PeakAmpSmooth + 0.002 * PeakAmp;
		PeakWAmpSmooth = 0.998 * PeakWAmpSmooth + 0.002 * PeakWAmp;
	}

	if (k > 1000){
		angleNorm = degBPF[0] / PeakAmpSmooth;
		velocityNorm = - velBPF / PeakWAmpSmooth;
	}

	gaitPhase = mode[0] * (atan2( (-1)*velocityNorm, (-1)*angleNorm)) / (2*WIDM_PI) * 100 + 50;  // 0~100%		// "1" : 50~75%, "2" : 75~100%, "3" : 0~25%, "4" : 25~50% ....

	B2Flag = 0;			// 10%
	B3Flag = 0;			// 20%
	B4Flag = 0;			// 30%
	B5Flag = 0;			// 40%
	B6Flag = 0;			// 50%

	B7Flag = 0;			// 0%	 Leg front->back	&   Gait Count ++
	B8Flag = 0;			// 50%	 Leg back->front
	B9Flag = 0;			// 75%	 Leg middle->front
	B10Flag = 0;		// 25%	 Leg middle->back

	B11Flag = 0;		// 60%
	B12Flag = 0;		// 70%
	B13Flag = 0;		// 80%
	B14Flag = 0;		// 90%


	if (k > 1000 && mode[0] == 1){
		if (gaitPhase > phaseThreshold1 && gaitPhasePrev < phaseThreshold1){
			B2Flag = 1;
		}
		if (gaitPhase > phaseThreshold2 && gaitPhasePrev < phaseThreshold2){
			B3Flag = 1;
		}
		if (gaitPhase > phaseThreshold3 && gaitPhasePrev < phaseThreshold3){
			B4Flag = 1;
		}
		if (gaitPhase > phaseThreshold4 && gaitPhasePrev < phaseThreshold4){
			B5Flag = 1;
		}
		if (gaitPhase > phaseThreshold5 && gaitPhasePrev < phaseThreshold5){
			B6Flag = 1;
		}
		if (gaitPhase > phaseThreshold6 && gaitPhasePrev < phaseThreshold6){
			B11Flag = 1;
		}
		if (gaitPhase > phaseThreshold7 && gaitPhasePrev < phaseThreshold7){
			B12Flag = 1;
		}
		if (gaitPhase > phaseThreshold8 && gaitPhasePrev < phaseThreshold8){
			B13Flag = 1;
		}
		if (gaitPhase > phaseThreshold9 && gaitPhasePrev < phaseThreshold9){
			B14Flag = 1;
		}
	}


	degLPF0ABS = WIDM_Abs_double(degLPF[0]);

	if (degLPF0ABS < 20 && mode[0] == 0){
		NeutralPosture = 0.999 * NeutralPosture + 0.001 * degLPF[0];				// Get (Stop & Standing) Posture's Angle
	}

	degUnbiased[0] = degLPF[0] - NeutralPosture;

	if (k > 1000){
		if (degUnbiased[0] > 10 && velLPF_H10[1] > 0 && velLPF_H10[0] < 0 && B7Finished == 0 && mode[0] == 1){
			B7Flag = 1;
			B7Finished = 1;
			gaitCount++;
		}
		if (degUnbiased[0] < 3 && velLPF_H10[1] < 0 && velLPF_H10[0] > 0 && B8Finished == 0 && mode[0] == 1){
			B8Flag = 1;
			B8Finished = 1;
		}
		if (degUnbiased[0] < 3 && degUnbiased[1] > 3){
			B7Finished = 0;
		}
		if (degUnbiased[0] > 5 && degUnbiased[1] < 5){
			B8Finished = 0;
		}

        if (velLPF_H10[0] > 8){
            B9Flag = 1;
        }
        if (velLPF_H10[0] < -8){
            B10Flag = 1;
        }
	}

	// Update previous values//
	mode[1] = mode[0];
	B1Flag = mode[0];
	gaitPhasePrev = gaitPhase;
	velLPF_H10[1] = velLPF_H10[0];
	degLPF[1] = degLPF[0];
	gyroLPF[1] = gyroLPF[0];
	velLPF2[1] = velLPF2[0];
	degBPF[2] = degBPF[1];
	degBPF[1] = degBPF[0];
	degUnbiased[1] = degUnbiased[0];
}


/*
 *Function to reduce noise in sensor data : Professor's solution
*/
static void CheckWalkingState_Prof(void)
{
		if (B2Flag == 1){
//			B2_chk[B2stack] = (uint8_t)gaitPhase;		// 10%
			B2stack++;
		}
		// B3Flag //
		if (B3Flag == 1){
//			B3_chk[B3stack] = (uint8_t)gaitPhase;		// 20%
			B3stack++;
		}
		// B4Flag //
		if (B4Flag == 1){
//			B4_chk[B4stack] = (uint8_t)gaitPhase;		// 30%
			B4stack++;
		}
		// B5Flag//
		if (B5Flag == 1){
//			B5_chk[B5stack] = (uint8_t)gaitPhase;		// 40%
			B5stack++;
		}
		// B6Flag //
		if (B6Flag == 1){
//			B6_chk[B6stack] = (uint8_t)gaitPhase;		// 50%
			B6stack++;
		}
		// B7Flag //
		if (B7Flag == 1){
//			B7_chk[B7stack] = (uint8_t)gaitPhase;		// ~~ 0% FB transition
			B7stack++;
		}
		// B8Flag //
		if (B8Flag == 1){
//			B8_chk[B8stack] = (uint8_t)gaitPhase;		// ~~ 50% BF transition
			B8stack++;
		}
		// B9Flag //
		if (B9Flag == 1){
//			B9_chk[B9stack] = (uint8_t)gaitPhase;		// Backward -> Forward
			B9stack++;
		}
		// B10Flag //
		if (B10Flag == 1){
//			B10_chk[B10stack] = (uint8_t)gaitPhase;		// Forward -> Backward
			B10stack++;
		}
}
#endif



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef QUATERNION
// Quaternion //
static void SetMagInvAInfo(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&vqfMagCalibObj.a11, t_req->data, 9*4);

	vqfMagCalibObj.A_inv[0][0] = vqfMagCalibObj.a11;
	vqfMagCalibObj.A_inv[0][1] = vqfMagCalibObj.a12;
	vqfMagCalibObj.A_inv[0][2] = vqfMagCalibObj.a13;
	vqfMagCalibObj.A_inv[1][0] = vqfMagCalibObj.a21;
	vqfMagCalibObj.A_inv[1][1] = vqfMagCalibObj.a22;
	vqfMagCalibObj.A_inv[1][2] = vqfMagCalibObj.a23;
	vqfMagCalibObj.A_inv[2][0] = vqfMagCalibObj.a31;
	vqfMagCalibObj.A_inv[2][1] = vqfMagCalibObj.a32;
	vqfMagCalibObj.A_inv[2][2] = vqfMagCalibObj.a33;

   t_res->size = 0;
   t_res->status = DATA_OBJECT_SDO_SUCC;
}


static void SetMagIronErrorInfo(MsgSDOargs* t_req, MsgSDOargs* t_res)
{
	memcpy(&vqfMagCalibObj.b1, t_req->data, 3*4);

	vqfMagCalibObj.ironErr[0] = vqfMagCalibObj.b1;
	vqfMagCalibObj.ironErr[1] = vqfMagCalibObj.b2;
	vqfMagCalibObj.ironErr[2] = vqfMagCalibObj.b3;

   t_res->size = 0;
   t_res->status = DATA_OBJECT_SDO_SUCC;
}


static void InitMagInfo(void)
{
	vqfMagCalibObj.A_inv[0][0] = vqfMagCalibObj.a11;
	vqfMagCalibObj.A_inv[0][1] = vqfMagCalibObj.a12;
	vqfMagCalibObj.A_inv[0][2] = vqfMagCalibObj.a13;
	vqfMagCalibObj.A_inv[1][0] = vqfMagCalibObj.a21;
	vqfMagCalibObj.A_inv[1][1] = vqfMagCalibObj.a22;
	vqfMagCalibObj.A_inv[1][2] = vqfMagCalibObj.a23;
	vqfMagCalibObj.A_inv[2][0] = vqfMagCalibObj.a31;
	vqfMagCalibObj.A_inv[2][1] = vqfMagCalibObj.a32;
	vqfMagCalibObj.A_inv[2][2] = vqfMagCalibObj.a33;

	vqfMagCalibObj.ironErr[0] = vqfMagCalibObj.b1;
	vqfMagCalibObj.ironErr[1] = vqfMagCalibObj.b2;
	vqfMagCalibObj.ironErr[2] = vqfMagCalibObj.b3;
}


static int Run_WIDM_6Axis_GetValue(void)
{
	status = IOIF_Get6AxisValue(&imu6AxisDataObj, &imu_params);

	return 0;
}

static int Ent_TVCF_Quaternion_WalkONS(void)
{
	//ResetDataObj(&imu_params, &widmSensorDataObj, &widmQuatDataObj, &widmFuzzyDataObj);

	SetInitialPosture_WalkONS(&imu6AxisDataObj, &imu_params);            // updating default posture of IMU on WalkONS
	SetFilterInitialValue_WalkONS(&imu6AxisDataObj, &widmQuatDataObj, &imu_params);

	widmFuzzyDataObj.wc_m = 1;            // cutoff frequency setting for 3rd order butterworth filter

	widmFuzzyDataObj.wl = 0.5;
	widmFuzzyDataObj.wh = 5;
	widmFuzzyDataObj.alpha = 30;
	widmFuzzyDataObj.s = 0.05;
	widmFuzzyDataObj.m0 = 5;

	for (uint8_t i = 0; i < 4; i++) {
		widmFuzzyDataObj.m[i] = 0;
		widmFuzzyDataObj.m_filtered[i] = 0;
	}

	return 0;
}

static int Run_TVCF_Quaternion_WalkONS(void)
{
	/* To deal with i2c busy situation (temporal)*/
	if (status == IOIF_IMU6AXIS_STATUS_BUSY){
		HAL_I2C_DeInit(&hi2c3);
		HAL_I2C_Init(&hi2c3);
	}

	status = IOIF_Get6AxisValue(&imu6AxisDataObj, &imu_params);
	UpdateSensorRawData_WalkONS(&widmSensorDataObj, &imu6AxisDataObj, &imu_params);

	WIDM_CalculateFuzzyM_WalkONS(&widmFuzzyDataObj, &widmSensorDataObj, WIDM_CONTROL_PERIOD);
	RunTvcfFilter_WalkONS(&widmSensorDataObj, &widmQuatDataObj, &widmFuzzyDataObj, WIDM_CONTROL_PERIOD);
	WIDM_UpdateBuffer_WalkONS(&widmSensorDataObj, &widmQuatDataObj, &widmFuzzyDataObj);

	WIDM_quat2eul(&widmQuatDataObj);

	return 0;
}

static int Ent_Gyr_Calibration_WalkONS (void)
{
	SetInitialPosture_WalkONS(&imu6AxisDataObj, &imu_params);            // updating default posture of IMU on WalkONS
	Ent_GyroCalibration_WalkONS(&widmSensorDataObj, &imu_params);

	return 0;
}

static int Run_Gyr_Calibration_WalkONS (void)
{
	Run_GyroCalibration_WalkONS(&imu6AxisDataObj, &imu_params, &widmSensorDataObj, &widmQuatDataObj);

	return 0;
}

static int Run_WIDM_3Axis_GetValue(void)
{
	IOIF_GetMagValue(&magDataObj);

	return 0;
}


static int EntGetQuaternion(void)
{
	VQF_Params_t params = { 0 };
	vqfObj.params = params;

	VQF_Init(&vqfObj, 0.001, 0.001, 0.001);
	return 0;
}


static int RunGetQuaternion(void)
{
	//uint8_t t6AxisRes
	//uint8_t t3AxisRes

	IOIF_Get6AxisValue(&imu6AxisDataObj, &imu_params);
	IOIF_GetMagValue(&magDataObj);

	//if (t6AxisRes != IOIF_IMU6AXIS_STATUS_OK) { return t6AxisRes; }
	//if (t3AxisRes != IOIF_IMU3AXIS_STATUS_OK) { return t3AxisRes; }

	float accX = imu6AxisDataObj.accX;
	float accY = imu6AxisDataObj.accY;
	float accZ = imu6AxisDataObj.accZ;
	float gyrX = imu6AxisDataObj.gyrX * 0.017453f; //degree to radian
	float gyrY = imu6AxisDataObj.gyrY * 0.017453f; //degree to radian
	float gyrZ = imu6AxisDataObj.gyrZ * 0.017453f; //degree to radian

	float magX_r = -magDataObj.magY;
	float magY_r = magDataObj.magX;
	float magZ_r = magDataObj.magZ;

	RealMagX = magX_r;
	RealMagY = magY_r;
	RealMagZ = magZ_r;

	float magValueRaw[3] = {magX_r, magY_r, magZ_r};

	for (int i = 0; i < 3; i++) {
		magValueCal[i] = 0;
		for (int j = 0; j < 3; j++) {
			magValueCal[i] += vqfMagCalibObj.A_inv[i][j] * (magValueRaw[j] - vqfMagCalibObj.ironErr[j]);
//			magValueCal[i] += vqfMagCalibInfo.A_inv[i][j] * (magValueRaw[j] - mag_calib_info.iron_err[j]);
		}
	}
	float magnorm = sqrt(magValueCal[0]*magValueCal[0] + magValueCal[1]*magValueCal[1] + magValueCal[2]*magValueCal[2]);
	magValueCal[0] = magValueCal[0] / magnorm;
	magValueCal[1] = magValueCal[1] / magnorm;
	magValueCal[2] = magValueCal[2] / magnorm;

	float magX = magValueCal[0];
	float magY = magValueCal[1];
	float magZ = magValueCal[2];

	magCalibDataObj.magX = magX;
	magCalibDataObj.magY = magY;
	magCalibDataObj.magZ = magZ;

    VQF_Real_t gyr[3] = {gyrX, gyrY, gyrZ};
    VQF_Real_t acc[3] = {accX, accY, accZ};
    VQF_Real_t mag[3] = {magX, magY, magZ};

	VQF_UpdateWithMag(&vqfObj, gyr, acc, mag);
	VQF_GetQuat9D(&vqfObj, vqfQuat);
	int i;
	for (i = 0; i < 4; i++) { q_send[i] = (int16_t)(vqfQuat[i] * 32768); }

	float t_tilt_angle = 57.2958 * atan(accY/accX);
	tilt_angle = t_tilt_angle * 0.02 + tilt_angle * 0.98;


	return 0;
}
#endif

static int Init_IMU(void)
{
	for (uint8_t i = 0; i < 3; i++) {
		imu_params.accScaleFactor[i] = 1;
		imu_params.gyrScaleFactor[i] = 1;
		imu_params.accBias[i] = 0;
		imu_params.gyrBias[i] = 0;
		imu_params.gyrSign[i] = 1;
		for (uint8_t j = 0; j < 3; j++) {
			if (i == j) {
				imu_params.R_Matrix[i][j] = 1;
			}
			else {
				imu_params.R_Matrix[i][j] = 0;
			}
		}
	}

	widmFuzzyDataObj.wc = 0;
	widmFuzzyDataObj.wl = 0;
	widmFuzzyDataObj.wh = 0;

	return 0;
}
