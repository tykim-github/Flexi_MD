

#ifndef APPS_RISK_MNGR_TASK_INC_RISK_MNGR_H_
#define APPS_RISK_MNGR_TASK_INC_RISK_MNGR_H_

#include "task_mngr.h"
#include "low_level_ctrl_task.h"
#include "mid_level_ctrl_task.h"
#include "system_ctrl_task.h"     //added
#include "ioif_tim_common.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define POLL_CONV_TIMEOUT 100

/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef enum{
	MD_Act_overcurrent_low = 0x00,
	MD_Act_overcurrent_high,
	MD_Act_overcurrent_peak,
	MD_Act_ROM_lim,          // actuator ROM limit
	MD_Act_overtemp_low,     // actuator overtemperature
	MD_Act_overtemp_high,
	MD_RT_breakage,          // MD real-time breakage
	MD_Act_error_lim,        // actuator control error limit
	MD_main_off,             // MD main power off
	MD_Act_incenc_error,     // actuator incremental encoder issue
	MD_Act_absenc_error,     // actuator absolute encoder issue
	MD_Act_tempsensor_error, // actuator temperature sensor issue
	MD_Act_hallsensor_error, // actuator hall sensor issue
	MD_overtemp_low,         // MD board overtemperature
	MD_overtemp_high,
	MD_3Phase_overcurrent,

	N_risk              // for counting errors
}ErrorCode_Enum;

typedef enum{
	OFF = 0x00,
	ON,
}Risk_Detected_Enum;

typedef void (*Risk_Det_FncPtr) (void);
typedef void (*Risk_Hdl_FncPtr) (void);

typedef struct _RiskObj {
	ErrorCode_Enum errortype;
	Risk_Det_FncPtr det_fnc;
	Risk_Hdl_FncPtr hdl_fnc;
	Risk_Detected_Enum risk_detected;
} RiskObj;

/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

extern TaskStruct risk_mngr_task;
extern uint16_t packed_risk;
extern uint8_t act_type;

/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void InitRiskMngrCtrl(void);
void RunRiskMngrCtrl(void* params);
void Init_Risk();
void Detect_Risk();
void Handle_Risk();
void Pack_Risk();

#endif /* APPS_RISK_MNGR_TASK_INC_RISK_MNGR_H_ */
