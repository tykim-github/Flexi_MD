/*
 * tvcf.h
 *
 *  Created on: Aug 22, 2024
 *      Author: my036
 */

#ifndef APPS_GAIT_CTRL_INC_TVCF_H_
#define APPS_GAIT_CTRL_INC_TVCF_H_

#include "ioif_icm20608g.h"
#include "msg_hdlr_task.h"
#include "risk_mngr.h"

#define WIDM_CONTROL_PERIOD		0.001
#define WIDM_PI					3.141592653589793
#define Deg2Rad                 0.01745329251994329577


/**
 * @brief Structure to save (current & previous) IMU sensor data
 */
typedef struct _WIDM_SensorData_t {
	float accX[2];			// [0] is current value, [1] is previous value (Accelerometer)
	float accY[2];
	float accZ[2];
	float gyrX[2];          // [0] is current value, [1] is previous value (Gyroscope)
	float gyrY[2];
	float gyrZ[2];

	float gyr_sum[3];
	float acc_sum[3];

	uint16_t tTotalSamples;
	uint16_t tRealSamples;

	uint8_t tDataCheck_IMU;
	uint8_t Calibration_ON;
	uint8_t Calibration_DONE;

	uint8_t start_calibration;

	uint8_t flush_flag;
	uint8_t current_flush_flag;

} WIDM_SensorData_t;

/**
 * @brief Structure to hold quaternion data
 */
typedef struct _WIDM_Quat_t {
	float w;
	float x;
	float y;
	float z;
} WIDM_Quat_t;

/**
 * @brief Structure to hold lie algebra of quaternion data
 */
typedef struct _WIDM_so3_t {
	float x;
	float y;
	float z;
} WIDM_so3_t;

/**
 * @brief Structure to hold filtered quaternion data
 */
typedef struct _WIDM_QuatData_t {

	WIDM_Quat_t quatAcc[3];

	WIDM_so3_t so3Acc[3];
	WIDM_so3_t so3AccFiltered[3];

	WIDM_Quat_t quatGyr[3];

	WIDM_so3_t so3Gyr[3];
	WIDM_so3_t so3GyrFiltered[3];

	WIDM_Quat_t quatTVCF;

	float roll;
	float pitch;
	float yaw;

} WIDM_QuatData_t;

/**
 * @brief Structure to hold Fuzzy Logic parameters
 */
typedef struct _WIDM_FuzzyData_t {
	float fuzzyInput[4];	// Acc, Jerk, Gyro, Wdot(measurement value)
	float wc;				// Cut off Frequency
	float wl;				// Low Frequency
	float wh;				// High Frequency
	float var[4];			// Acc, Jerk, Gyro, Wdot-variance(initially set velue)
	float wc_debug;

	float alpha;    // The scaling constant to set the magnitude between acc. and gyr.
	float s;        // determine the slope of Fuzzy logic membership function
	float m0;       // reference value to distinguish whether m(k) is big or small
	float acc_mean;
	float wc_m;     // mitigate the noise in m
	float m_acc;
	float m_gyr;
	double m[4];
	double m_filtered[4];

} WIDM_FuzzyData_t;

/* Basic Calculation Function */
float WIDM_SquareRootSum(float x, float y, float z);
float WIDM_AbsoluteValue(float value);

/* Quaternion-related Function */
void WIDM_quatAdd(WIDM_Quat_t* quat_1, WIDM_Quat_t* quat_2, WIDM_Quat_t* quat_out);
void WIDM_quatScalarMul(float multiple, WIDM_Quat_t* quat, WIDM_Quat_t* quat_out);
void WIDM_quatMul(WIDM_Quat_t* quat_1, WIDM_Quat_t* quat_2, WIDM_Quat_t* quat_out);
void WIDM_quatNormalization(WIDM_Quat_t* quat, WIDM_Quat_t* quat_out);
void WIDM_quatConjugate(WIDM_Quat_t* quat, WIDM_Quat_t* quat_out);
void WIDM_quatLog(WIDM_Quat_t* quat, WIDM_so3_t* so3_out);
void WIDM_quatExp(WIDM_so3_t* so3_in, WIDM_Quat_t* quat_out);
void WIDM_GetquatGyr(WIDM_SensorData_t* sensorData, WIDM_QuatData_t* quatData, float samplingPeriod);
void WIDM_GetquatAcc(WIDM_SensorData_t* sensorData, WIDM_QuatData_t* quatData);
void WIDM_quat2eul(WIDM_QuatData_t* quatData);
void Ent_GyroCalibration_WalkONS(WIDM_SensorData_t* widmSensorDataObj, IMU_Params_t* imu_params);
void Run_GyroCalibration_WalkONS(IOIF_6AxisData_t* imu6AxisDataObj, IMU_Params_t* imu_params, WIDM_SensorData_t* widmSensorDataObj, WIDM_QuatData_t* widmQuatDataObj);

/* Filter-related Function */
void get2ndFilterCoefficient_LPF(float f_cut, float ts, float H0, float Q, double b[], double a[]);
void get3rdFilterCoefficient_LPF(float f_cut, float ts, float H0, float Q, double b[], double a[]);
void get2ndFilterCoefficient_HPF(float f_cut, float ts, float H0, float Q, double b[], double a[]);
float WIDM_LPF_WalkONS(float data, float data_f, float data_ff, float dataFiltered_f, float dataFiltered_ff, float cutoffFreq, float samplingPeriod);
double WIDM_LPF_WalkONS_3rd(double data, double data_f, double data_ff, double data_fff, double dataFiltered_f, double dataFiltered_ff, double dataFiltered_fff, float cutoffFreq, float samplingPeriod);
float WIDM_HPF_WalkONS(float data, float data_f, float data_ff, float dataFiltered_f, float dataFiltered_ff, float cutoffFreq, float samplingPeriod);

/* TVCF Algorithm Function*/
/* Reset Value Zero */
void ResetDataObj(IMU_Params_t* imu_params, WIDM_SensorData_t* widmSensorDataObj, WIDM_QuatData_t* widmQuatDataObj, WIDM_FuzzyData_t* widmFuzzyDataObj);
void GyroCalibration_WalkONS(IOIF_6AxisData_t* imu6AxisDataObj, IMU_Params_t* imu_params);
int SetInitialPosture_WalkONS(IOIF_6AxisData_t* imu6AxisDataObj, IMU_Params_t* imu_params);
void SetFilterInitialValue_WalkONS(IOIF_6AxisData_t* imu6AxisDataObj, WIDM_QuatData_t* quatData, IMU_Params_t* imu_params);
void UpdateSensorRawData_WalkONS(WIDM_SensorData_t* widmSensorData, IOIF_6AxisData_t* imu6AxisData, IMU_Params_t* imu_params);
void WIDM_CalculateFuzzyM_WalkONS(WIDM_FuzzyData_t* fuzzyData, WIDM_SensorData_t* sensorData, float samplingPeriod);
void RunTvcfFilter_WalkONS(WIDM_SensorData_t* widmSensorData, WIDM_QuatData_t* widmQuatData, WIDM_FuzzyData_t* widmFuzzyData, float samplingPeriod);
void WIDM_RunTVCF_WalkONS(WIDM_SensorData_t* sensorData, WIDM_QuatData_t* quatData, float cutoffFreq, float samplingPeriod);
void WIDM_UpdateBuffer_WalkONS(WIDM_SensorData_t* sensorData, WIDM_QuatData_t* quatData, WIDM_FuzzyData_t* fuzzyData);
void WIDM_FlushBuffer_WalkONS(WIDM_QuatData_t* quatData);

#endif /* APPS_GAIT_CTRL_INC_TVCF_H_ */
