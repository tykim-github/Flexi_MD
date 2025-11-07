#ifndef COMMON_MODULE_H_
#define COMMON_MODULE_H_

/* ------------------- Module Enable/Disable ------------------- */
//******* IF CM *******//
// #define WALKON5_CM_ENABLED
// #define L30_CM_ENABLED
// #define SUIT_MINICM_ENABLED

//******* IF MD *******//
//#define WALKON5_MD_ENABLED
//#define L30_MD_REV06_ENABLED
//#define L30_MD_REV07_ENABLED
#define L30_MD_REV08_ENABLED
//#define SUIT_MD_ENABLED

//******* IF WIDM *******//
// #define WIDM_ENABLED
//#define QUATERNION
// TODO : 
/* ------------------- RTOS Enable/Disable ------------------- */
// #define _USE_OS_RTOS_BSP
// #define _USE_CMSISV1
// #define _USE_CMSISV2

/* HW Device Semaphore Timeout Definition */
// #define _USE_SEMAPHORE

/* Task(Thread) Definition Handler */
// #define _USE_OS_RTOS
#define _USE_BAREMETAL

/* HW Platform */
// #define _HW_PLATFORM_WKON5
// #define _HW_PLATFORM_MINICM

/* ------------------- Select CM or MD or WIDM For WIDM(Gait Ctrl) ------------------- */
#if defined(WALKON5_CM_ENABLED) || defined(L30_CM_ENABLED) || defined(SUIT_MINICM_ENABLED)
#define CM_MODULE
#endif
#if defined(WALKON5_MD_ENABLED) || defined(L30_MD_REV06_ENABLED) || defined(L30_MD_REV07_ENABLED) || defined(L30_MD_REV08_ENABLED) || defined(SUIT_MD_ENABLED)
#define MD_MODULE
#endif
#if defined(WIDM_ENABLED)
#define WIDM_MODULE
#endif

/* ------------------- WALKON5_CM_ENABLED ------------------- */
#ifdef WALKON5_CM_ENABLED

#define IOIF_LTC2944_ENABLED
#define IOIF_BM1422AGMV_ENABLED
#define IOIF_ICM20608G_ENABLED
#define BUS_VOLTAGE 48
#define VBUS2DUTY_RATIO 100

#endif /* WALKON5_CM_ENABLED */

/* ------------------- L30_CM_ENABLED ------------------- */
#ifdef L30_CM_ENABLED

#define IOIF_LTC2944_ENABLED
#define IOIF_BM1422AGMV_ENABLED
#define IOIF_ICM20608G_ENABLED
#define IOIF_NZRLED_ENABLED
#define IOIF_RMB30SC_ENABLED
#define IOIF_THINPOT_LS_ENABLED
#define IOIF_TB67H450FNGEL_ENABLED
#define IOIF_BUZZER_ENABLED
#define IOIF_BATTERYLED_ENABLED
#define BUS_VOLTAGE 48
#define VBUS2DUTY_RATIO 100

#endif /* L30_CM_ENABLED */

/* ------------------- SUIT_MINICM_ENABLED ------------------- */
#ifdef SUIT_MINICM_ENABLED

#define IOIF_LTC2944_ENABLED
#define IOIF_BM1422AGMV_ENABLED
#define IOIF_ICM20608G_ENABLED
#define BUS_VOLTAGE 48
#define VBUS2DUTY_RATIO 100

#endif /* SUIT_MINICM_ENABLED */

/* ------------------- L30_MD_REV06_ENABLED ------------------- */
#ifdef L30_MD_REV06_ENABLED

#define IOIF_LTC2944_ENABLED
#define IOIF_BM1422AGMV_ENABLED
#define IOIF_ICM20608G_ENABLED
#define IOIF_TMCS1100A2_ENABLED
#define IOIF_FSR_SZH_HWS004_ENABLED
#define IOIF_LINEAR_POTEN_ENABLED
#define IOIF_503NTC_ENABLED
#define IOIF_THINPOT_LS_ENABLED
#define IOIF_TB67H450FNGEL_ENABLED
#define IOIF_RMB20SC_ENABLED
#define IOIF_RMB20IC_ENABLED
#define IOIF_NZRLED_ENABLED
#define BUS_VOLTAGE 48
#define VBUS2DUTY_RATIO 100

#endif /* L30_MD_REV06_ENABLED */

/* ------------------- L30_MD_REV07_ENABLED ------------------- */
#ifdef L30_MD_REV07_ENABLED

#define IOIF_LTC2944_ENABLED
#define IOIF_BM1422AGMV_ENABLED
#define IOIF_ICM20608G_ENABLED
#define IOIF_TMCS1100A2_ENABLED
#define IOIF_FSR_SZH_HWS004_ENABLED
#define IOIF_LINEAR_POTEN_ENABLED   
#define IOIF_503NTC_ENABLED
#define IOIF_THINPOT_LS_ENABLED
#define IOIF_TB67H450FNGEL_ENABLED
#define IOIF_RMB20SC_ENABLED
#define IOIF_RMB20IC_ENABLED
#define IOIF_NZRLED_ENABLED
#define BUS_VOLTAGE 48
#define VBUS2DUTY_RATIO 100

#endif /* L30_MD_REV07_ENABLED */

/* ------------------- L30_MD_REV08_ENABLED ------------------- */
#ifdef L30_MD_REV08_ENABLED

#define IOIF_LTC2944_ENABLED
#define IOIF_BM1422AGMV_ENABLED
#define IOIF_ICM20608G_ENABLED
#define IOIF_TMCS1100A2_ENABLED
#define IOIF_FSR_SZH_HWS004_ENABLED
#define IOIF_LINEAR_POTEN_ENABLED   
#define IOIF_503NTC_ENABLED
#define IOIF_THINPOT_LS_ENABLED
#define IOIF_TB67H450FNGEL_ENABLED
#define IOIF_RMB20SC_ENABLED
#define IOIF_RMB20IC_ENABLED
#define IOIF_NZRLED_ENABLED
#define BUS_VOLTAGE 48
#define VBUS2DUTY_RATIO 100

#endif /* L30_MD_REV08_ENABLED */

/* ------------------- SUIT_MD_ENABLED ------------------- */
#ifdef SUIT_MD_ENABLED

#define IOIF_BM1422AGMV_ENABLED
#define IOIF_ICM20608G_ENABLED
#define IOIF_TMCS1100A3_ENABLED
#define IOIF_503NTC_ENABLED
#define IOIF_GRFSENSOR_ENABLED
#define IOIF_RMB20SC_ENABLED
#define IOIF_RMB20IC_ENABLED
#define BUS_VOLTAGE 24
#define VBUS2DUTY_RATIO 200

#endif /* SUIT_MD_ENABLED */

/* ------------------- WIDM_ENABLED ------------------- */
#ifdef WIDM_ENABLED

#define IOIF_BM1422AGMV_ENABLED
#define IOIF_ICM20608G_ENABLED

#endif /* WIDM_ENABLED */

#endif /* COMMON_MODULE_H_ */
