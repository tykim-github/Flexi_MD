/**
 * @file msg_hdlr_task.h
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics FW Team
 * @brief 
 * @ref 
 */

#ifndef APPS_MSG_HDLR_TASK_INC_MSG_HDLR_H_
#define APPS_MSG_HDLR_TASK_INC_MSG_HDLR_H_

#include "task_mngr.h"
#include "cvector.h"

#include "error_dictionary.h"
#include "ioif_fdcan_common.h"
#include "ioif_tim_common.h"
#include "ioif_gpio_common.h"
#include "ioif_usb_common.h"

#include "msg_common.h"
#include "BumbleBee.h"

#include "module.h"

#include "low_level_ctrl_task.h"
#include "mid_level_ctrl_task.h"
#include "risk_mngr.h"
#include "usbd_cdc_if.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define FDCAN_DLC_BYTES_0  ((uint32_t)0x00000000U) /*!< 0 bytes data field  */
#define FDCAN_DLC_BYTES_1  ((uint32_t)0x00010000U) /*!< 1 bytes data field  */
#define FDCAN_DLC_BYTES_2  ((uint32_t)0x00020000U) /*!< 2 bytes data field  */
#define FDCAN_DLC_BYTES_3  ((uint32_t)0x00030000U) /*!< 3 bytes data field  */
#define FDCAN_DLC_BYTES_4  ((uint32_t)0x00040000U) /*!< 4 bytes data field  */
#define FDCAN_DLC_BYTES_5  ((uint32_t)0x00050000U) /*!< 5 bytes data field  */
#define FDCAN_DLC_BYTES_6  ((uint32_t)0x00060000U) /*!< 6 bytes data field  */
#define FDCAN_DLC_BYTES_7  ((uint32_t)0x00070000U) /*!< 7 bytes data field  */
#define FDCAN_DLC_BYTES_8  ((uint32_t)0x00080000U) /*!< 8 bytes data field  */
#define FDCAN_DLC_BYTES_12 ((uint32_t)0x00090000U) /*!< 12 bytes data field */
#define FDCAN_DLC_BYTES_16 ((uint32_t)0x000A0000U) /*!< 16 bytes data field */
#define FDCAN_DLC_BYTES_20 ((uint32_t)0x000B0000U) /*!< 20 bytes data field */
#define FDCAN_DLC_BYTES_24 ((uint32_t)0x000C0000U) /*!< 24 bytes data field */
#define FDCAN_DLC_BYTES_32 ((uint32_t)0x000D0000U) /*!< 32 bytes data field */
#define FDCAN_DLC_BYTES_48 ((uint32_t)0x000E0000U) /*!< 48 bytes data field */
#define FDCAN_DLC_BYTES_64 ((uint32_t)0x000F0000U) /*!< 64 bytes data field */

#define MEMORY_SECOND_HAND_CHECK	1

/******************** WS5 SPECIAL DEFINITIONS *******************/
#define ACTUAL_CURRENT_MAX   +200      // unit: A
#define ACTUAL_CURRENT_MIN   -200      // unit: A

#define INCENC_POSITION_MAX +6000      // unit: Deg
#define INCENC_POSITION_MIN -6000      // unit: Deg

#define ABSENC_POSITION_MAX  +180      // unit: Deg
#define ABSENC_POSITION_MIN  -180      // unit: Deg

#define REFERENCE_POSITION_MAX +6000  // unit: Deg
#define REFERENCE_POSITION_MIN -6000  // unit: Deg

#define BUS_CURRENT_MAX  +100          // unit: A
#define BUS_CURRENT_MIN  -100          // unit: A

#define TEMPERATURE_MAX  +256          // unit: Deg
#define TEMPERATURE_MIN  0          // unit: Deg

/******************** WS5 SPECIAL DEFINITIONS *******************/


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

/*typedef enum{
	FIRST_USE = 0x01,
	SECOND_USE,
	MEMORY_UI_MOTOR_PROPERTIES,
	MEMORY_UI_SENSOR_SETTING,
	MEMORY_UI_ELECTRICAL_PROPERTIES,
	MEMORY_UI_MECHANICAL_PROPERTIES,
	MEMORY_UI_CONTROL_PARAMETERS1,
	MEMORY_UI_CONTROL_PARAMETERS2,
	MEMORY_UI_ADDITIONAL_FUNCTION_PARAMETERS,
	MEMORY_UI_IMU_PARAMETERS,

	E_SYS_BATCH,
	E_SYS_BEMF,
	BEMF_ID_OVER_CURRENT,
	BW_CHECK,
	GET_CURRENT_TRACKING_CHECK,
	FRICTION_ID_RAW_DATA,
	FRICTION_ID_AVERAGED_DATA,
	FRICTION_ID_DONE,
	FRICTION_COMPENSATOR_VERIFICATION,

	MECH_SYS_ID_SBS_RAW_DATA,
	MECH_SYS_ID_SBS_DONE,

	IRC_VERIFICATION,

	GET_IMPEDANCE_SINE_CTRL,
	GET_IMPEDANCE_REC_CTRL,

	GAIN_TUNER,
	GET_VELOCITY_CTRL,
	GET_POSITION_CTRL,

	GET_HALLSENSOR,
	GET_INCENCODER,
	GET_ABSENCODER1,
	GET_ABSENCODER2,

	GET_VSD_UPPER_LIMIT,
	GET_VSD_LOWER_LIMIT,

	VSD_VERIFICATION_DATA,

	GET_BACKLASH_TEST,
	GET_DOB_DATA,

	GET_DIRECTION_SET_DATA,
	GET_DIRECTION_SET_DONE,

	SAVE_DONE,
	GET_VE_TEST_DATA,
	GET_SYSTEM_ID_VERIFY,
	VE_KF_SETTING_ERROR,
	GET_FF_CTRL,
	GET_TOTAL_CTRL,

	ADV_FRICTION_ID_DATA,
	ADV_FRICTION_ID_DONE,

	GET_GAIT_PHASE,
	GET_ACC_GYRO,
	GET_QUATERNION,
	GET_MAG,

	EXPORT_SETTING_PROPERTIES
} GUISequence_Enum;

typedef enum{
	IDLE,
	UPLOAD_PROPERTIES,
	SAVE_PROPERTIES,
	DOWNLOAD_PROPERTIES,
	ELECTRICAL_SYSTEM_ID,
	BEMF_ID,
	CURRENT_BANDWIDTH_CHECK,
	CURRENT_TRACKING_CHECK,
	AUTO_TUNING,
	ADV_FRICTION_ID,
	CAL_FRICTION_LUT,
} MainSequence_Enum;*/

typedef enum _COMMType{
	e_FDCAN = 0U,
	e_USB
} COMMType;

typedef struct _WS5_MD_Transmit_Data_16BIT{

	int16_t  actual_current;
	int16_t  incenc_position;
	int16_t  absenc_position;
	int16_t  reference_position;
	int16_t  bus_current;
	int16_t  error_code;
	int16_t  temperature;

	float    K_actual_current;
	float    K_incenc_position;
	float    K_absenc_position;
	float    K_reference_position;
	float    K_bus_current;
	float    K_temperature;

	uint8_t actuator_temperature;
	uint8_t board_temperature;

}WS5_MD_Transmit_Data_16BIT;

typedef struct _WS5_MD_Transmit_Data_32BIT{

	uint32_t loop_time;
//	uint32_t error_code;

}WS5_MD_Transmit_Data_32BIT;

/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

/**************** WS5 Special Functions *********************/



/**************** WS5 Special Functions *********************/
extern TaskStruct        msg_hdlr_task;
extern uint8_t           MD_node_id;

/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void Send_EMCY(uint32_t* f_err_code);
int Send_MSG(uint16_t COB_ID, uint32_t len, uint8_t* tx_data);

void InitMsgHdlr(void);
void RunMsgHdlr(void* params);


#endif /* APPS_MSG_HDLR_TASK_INC_MSG_HDLR_H_ */
