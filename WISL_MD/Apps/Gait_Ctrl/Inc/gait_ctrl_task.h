/*
 * gait_ctrl_task.h
 *
 *  Created on: Oct 11, 2023
 *      Author: INVINCIBLENESS
 */

#ifndef APPS_GAIT_CTRL_INC_GAIT_CTRL_TASK_H_
#define APPS_GAIT_CTRL_INC_GAIT_CTRL_TASK_H_


/* Select WIDM ANGLE CHECK MODE */
#define IMU_MODE
//#define IMUABS_MODE
//#define IMUINC_MODE
//////////////////////

/* Select when you want to do Quaternion test */
//#define QUATERNION
//////////////////////


#include <stdbool.h>

#include "ioif_tim_common.h"
#include "msg_hdlr_task.h"
#include "msg_common.h"
#include "error_dictionary.h"

// For IMU //
#include "module.h"
#include "ioif_icm20608g.h"
#include "ioif_bm1422agmv.h"
#include "widm_algorithms.h"

#if defined(WALKON5_MD_ENABLED) || defined(L30_MD_REV06_ENABLED) || defined(L30_MD_REV07_ENABLED) || defined(L30_MD_REV08_ENABLED) || defined(SUIT_MD_ENABLED)
#include "mid_level_ctrl_task.h"
#include "low_level_ctrl_task.h"
#endif

#ifdef IMUABS_MODE
#include "mid_level_ctrl_task.h"
#endif

#ifdef IMUINC_MODE
#include "low_level_ctrl_task.h"
#endif

#include "tvcf.h"
#include "vqf.h"
// For Quaternion //
#ifdef QUATERNION
#include "vqf.h"
#endif

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define NORM_CUTOFF_FREQ	3

/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

extern IMU_Params_t imu_params;
extern WIDM_FuzzyData_t	widmFuzzyDataObj;

extern WIDM_GaitData_t widmGaitDataObj;
extern WIDM_AngleData_t widmAngleDataObj;
extern WIDM_AttachCase_t widmAttachCaseObj;

extern uint8_t B7Flag;
extern uint8_t B8Flag;
extern uint8_t B9Flag;
extern uint8_t B10Flag;
extern uint8_t B11Flag;
extern uint8_t B12Flag;
extern uint8_t B13Flag;
extern uint8_t B14Flag;
extern uint32_t gaitCount;

extern float incDeg;
extern float incVel;

extern VQF_MagCalib_t vqfMagCalibObj;
#ifdef QUATERNION
extern VQF_MagCalib_t vqfMagCalibObj;
#endif

extern uint8_t MD_Node_ID;

/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void InitGaitCtrl(void);
void RunGaitCtrl(void* params);


#endif /* APPS_GAIT_CTRL_INC_GAIT_CTRL_TASK_H_ */
