/**
 * @file system_ctrl_task.c
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics FW Team
 * @brief 
 * @ref 
 */

#include "system_ctrl_task.h"

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

TaskStruct systemCtrlTask;

#if defined(L30_MD_REV06_ENABLED) || defined (L30_MD_REV07_ENABLED) || defined (L30_MD_REV08_ENABLED)
IOIF_BatData_t batData;

uint8_t IsRecoverI2C2 = false;
#endif


/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */

static float sysTimeElap;

#if defined(L30_MD_REV06_ENABLED) || defined (L30_MD_REV07_ENABLED) || defined (L30_MD_REV08_ENABLED)
static uint8_t bmInitRes = IOIF_BAT_STATUS_OK;
static uint8_t bmRes = IOIF_BAT_STATUS_OK;
#endif


/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

/* ------------------- STATE FUNCTIONS ------------------- */
static void StateOff_Run(void);

static void StateStandby_Run(void);

static void StateEnable_Ent(void);
static void StateEnable_Run(void);
static void StateEnable_Ext(void);

static void StateError_Run(void);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

/* ------------------- SDO CALLBACK ------------------- */
// MSG_COMMON_SDO_CALLBACK(systemCtrlTask)

void InitSysMngtTask(void)
{
    // init
    Init_Task(&systemCtrlTask);

#if defined(L30_MD_REV06_ENABLED) || defined (L30_MD_REV07_ENABLED) || defined (L30_MD_REV08_ENABLED)
	// bmInitRes = IOIF_InitBat(IOIF_I2C2);
	// 	if (bmInitRes != IOIF_BAT_STATUS_OK) {
	// 	// TODO : Error Handling
	// 	return;
	// }

	bmInitRes = IOIF_InitBat(IOIF_I2C2);
	if (bmInitRes != IOIF_BAT_STATUS_OK) { //i2c revocery
		// TODO : Error Handling
		__HAL_RCC_I2C2_CLK_DISABLE();
		__HAL_I2C_DISABLE(&hi2c2);
		HAL_I2C_DeInit(&hi2c2);
		HAL_I2C_MspDeInit(&hi2c2);
		HAL_I2C_BusReset(&hi2c2, GPIOB, LTC2944_I2C_SDA_Pin, LTC2944_I2C_SCL_Pin);

		HAL_I2C_Init(&hi2c2);
		HAL_I2C_MspInit(&hi2c2);
		__HAL_I2C_ENABLE(&hi2c2);
		__HAL_RCC_I2C2_CLK_ENABLE();

		IsRecoverI2C2 = true;

		bmInitRes = IOIF_InitBat(IOIF_I2C2);
	}

	if (bmInitRes == IOIF_BAT_STATUS_OK) {
		// TODO : Next Sequence
	}
#endif

	/* State Definition */
	TASK_CREATE_STATE(&systemCtrlTask, e_State_Off,      NULL,   				StateOff_Run,       NULL,       		false);
	TASK_CREATE_STATE(&systemCtrlTask, e_State_Standby,  NULL,   				StateStandby_Run,	NULL,       		true);
	TASK_CREATE_STATE(&systemCtrlTask, e_State_Enable,   StateEnable_Ent,   	StateEnable_Run, 	StateEnable_Ext,  	false);
	TASK_CREATE_STATE(&systemCtrlTask, e_State_Error,    NULL,   				StateError_Run,    	NULL,				false);

	/* Routine Definition */
	// TASK_CREATE_ROUTINE(&systemCtrlTask, ROUTINE_ID_GET_POWER_VALUE, 		NULL, Run_Get_Power_Value, 		NULL);
	// TASK_CREATE_ROUTINE(&systemCtrlTask, ROUTINE_ID_CHECK_POWER_STATE, 	NULL, Run_Check_Power_State, 	NULL);

	/* DOD Definition */
	// DOD
	Create_DOD(TASK_ID_SYSMNGT);

	// PDO

#if defined(L30_MD_REV06_ENABLED) || defined (L30_MD_REV07_ENABLED) || defined (L30_MD_REV08_ENABLED)
	Create_PDO(TASK_ID_SYSMNGT, PDO_ID_SYSTEM_VOLT, e_Float32, 1, &batData.batVol);
    Create_PDO(TASK_ID_SYSMNGT, PDO_ID_SYSTEM_CURR, e_Float32, 1, &batData.batCurr);
	Create_PDO(TASK_ID_SYSMNGT, PDO_ID_SYSTEM_TEMP, e_Float32, 1, &batData.brdTemp);
#endif
	// SDO
	// MSG_COMMON_SDO_CREATE(TASK_ID_SYSMNGT)

	/* Timer 15 Callback Allocation */
	if(IOIF_StartTimIT(IOIF_TIM15) > 0){
		//TODO: ERROR PROCESS
	}

	IOIF_SetTimCB(IOIF_TIM15, IOIF_TIM_PERIOD_ELAPSED_CALLBACK, RunSysMngtTask, NULL);
}

void RunSysMngtTask(void* params)
{	
	/* Loop Start Time Check */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	/* Run Device */
	Run_Task(&systemCtrlTask);
	
	/* Elapsed Time Check */
	sysTimeElap = DWT->CYCCNT/480;	// in microsecond
}

/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */
/* ------------------- STATE FUNCTIONS ------------------- */
static void StateOff_Run(void)
{

}

static void StateStandby_Run(void)
{
	Transition_State(&systemCtrlTask.state_machine, e_State_Enable);
}

static void StateEnable_Ent(void)
{
	Ent_Routines(&systemCtrlTask.routine);
}

static void StateEnable_Run(void)
{
#if defined(L30_MD_REV06_ENABLED) || defined (L30_MD_REV07_ENABLED) || defined (L30_MD_REV08_ENABLED)
	bmRes = IOIF_GetBatValue(&batData);
#endif
	Run_Routines(&systemCtrlTask.routine);
}

static void StateEnable_Ext(void)
{
	Ext_Routines(&systemCtrlTask.routine);
}

static void StateError_Run(void)
{
	// TODO : Error Handle -> Indicate LED RED Blinking, LED BLUE OFF
}

/* ------------------- ROUTINES ------------------- */

