/*
 * tvcf.c
 *
 *  Created on: Aug 22, 2024
 *      Author: my036
 */

#include <math.h>

#include "tvcf.h"

extern uint8_t act_type;

/*
    ____             _         ______      __           __      __  _
   / __ )____ ______(_)____   / ____/___ _/ /______  __/ /___ _/ /_(_)___  ____
  / __  / __ `/ ___/ / ___/  / /   / __ `/ / ___/ / / / / __ `/ __/ / __ \/ __ \
 / /_/ / /_/ (__  ) / /__   / /___/ /_/ / / /__/ /_/ / / /_/ / /_/ / /_/ / / / /
/_____/\__,_/____/_/\___/   \____/\__,_/_/\___/\__,_/_/\__,_/\__/_/\____/_/ /_/
*/

float WIDM_SquareRootSum(float x, float y, float z)
{
	return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
}

float WIDM_AbsoluteValue(float value)
{
	return fabs(value);
}

/*
   ____              __                  _                             __      __           __
  / __ \__  ______ _/ /____  _________  (_)___  ____        ________  / /___ _/ /____  ____/ /
 / / / / / / / __ `/ __/ _ \/ ___/ __ \/ / __ \/ __ \______/ ___/ _ \/ / __ `/ __/ _ \/ __  /
/ /_/ / /_/ / /_/ / /_/  __/ /  / / / / / /_/ / / / /_____/ /  /  __/ / /_/ / /_/  __/ /_/ /
\___\_\__,_/\__,_/\__/\___/_/  /_/ /_/_/\____/_/ /_/     /_/   \___/_/\__,_/\__/\___/\__,_/
*/

void WIDM_quatAdd(WIDM_Quat_t* quat_1, WIDM_Quat_t* quat_2, WIDM_Quat_t* quat_out)
{
	quat_out->w = quat_1->w + quat_2->w;
	quat_out->x = quat_1->x + quat_2->x;
	quat_out->y = quat_1->y + quat_2->y;
	quat_out->z = quat_1->z + quat_2->z;
}

void WIDM_quatScalarMul(float multiple, WIDM_Quat_t* quat, WIDM_Quat_t* quat_out)
{
	quat_out->w = quat->w*multiple;
	quat_out->x = quat->x*multiple;
	quat_out->y = quat->y*multiple;
	quat_out->z = quat->z*multiple;
}

void WIDM_quatMul(WIDM_Quat_t* quat_1, WIDM_Quat_t* quat_2, WIDM_Quat_t* quat_out)
{
	quat_out->w = quat_1->w*quat_2->w - quat_1->x*quat_2->x - quat_1->y*quat_2->y - quat_1->z*quat_2->z;
	quat_out->x = quat_1->w*quat_2->x + quat_1->x*quat_2->w + quat_1->y*quat_2->z - quat_1->z*quat_2->y;
	quat_out->y = quat_1->w*quat_2->y - quat_1->x*quat_2->z + quat_1->y*quat_2->w + quat_1->z*quat_2->x;
	quat_out->z = quat_1->w*quat_2->z + quat_1->x*quat_2->y - quat_1->y*quat_2->x + quat_1->z*quat_2->w;
}

void WIDM_quatNormalization(WIDM_Quat_t* quat, WIDM_Quat_t* quat_out)
{
	float norm = sqrt(quat->w*quat->w + quat->x*quat->x + quat->y*quat->y + quat->z*quat->z);

	if (norm == 0)
	{
		quat_out->w = 1;
		quat_out->x = 0;
		quat_out->y = 0;
		quat_out->z = 0;
	}
	else
	{
		quat_out->w = quat->w/norm;
		quat_out->x = quat->x/norm;
		quat_out->y = quat->y/norm;
		quat_out->z = quat->z/norm;
	}
}

void WIDM_quatConjugate(WIDM_Quat_t* quat, WIDM_Quat_t* quat_out)
{
	quat_out->w = quat->w;
	quat_out->x = -quat->x;
	quat_out->y = -quat->y;
	quat_out->z = -quat->z;
}

void WIDM_quatLog(WIDM_Quat_t* quat, WIDM_so3_t* so3_out)
{

	if (quat->w == 1)
	{
		so3_out->x = 0;
		so3_out->y = 0;
		so3_out->z = 0;
	}
	else
	{
		float temp1 = acos(quat->w)/sqrt(1 - quat->w*quat->w);

		so3_out->x = quat->x*temp1;
		so3_out->y = quat->y*temp1;
		so3_out->z = quat->z*temp1;
	}

}

void WIDM_quatExp(WIDM_so3_t* so3_in, WIDM_Quat_t* quat_out)
{

	if (so3_in->x == 0 && so3_in->y == 0 && so3_in->z == 0)
	{
		quat_out->w = 1;
		quat_out->x = 0;
		quat_out->y = 0;
		quat_out->z = 0;
	}
	else
	{
		float norm = sqrt(so3_in->x*so3_in->x + so3_in->y*so3_in->y + so3_in->z*so3_in->z);

		quat_out->w = cos(norm);
		quat_out->x = so3_in->x*sin(norm)/norm;
		quat_out->y = so3_in->y*sin(norm)/norm;
		quat_out->z = so3_in->z*sin(norm)/norm;
	}
}

void WIDM_GetquatGyr(WIDM_SensorData_t* sensorData, WIDM_QuatData_t* quatData, float samplingPeriod)
{
	WIDM_Quat_t q_gyr = {0, Deg2Rad*sensorData->gyrX[0], Deg2Rad*sensorData->gyrY[0], Deg2Rad*sensorData->gyrZ[0]};
	WIDM_Quat_t qDot;

	WIDM_quatMul(&(quatData->quatGyr[1]), &q_gyr, &qDot);
	WIDM_quatScalarMul(0.5*samplingPeriod, &qDot, &qDot);
	WIDM_quatAdd(&(quatData->quatGyr[1]), &qDot, &(quatData->quatGyr[0]));
	WIDM_quatNormalization(&(quatData->quatGyr[0]), &(quatData->quatGyr[0]));
}

void WIDM_GetquatAcc(WIDM_SensorData_t* sensorData, WIDM_QuatData_t* quatData)
{
	// TODO: Estimating yaw angle using encoder sensor fusion

//	float yaw;
	float roll;
	float pitch;

	if (sensorData->accX[0] == 0 && sensorData->accX[0] == 0 && sensorData->accX[0] == 0)
	{
		quatData->quatAcc[0].w = 1;
		quatData->quatAcc[0].x = 0;
		quatData->quatAcc[0].y = 0;
		quatData->quatAcc[0].z = 0;
	}
	else
	{
		float accNorm = sqrt(sensorData->accX[0]*sensorData->accX[0] + sensorData->accY[0]*sensorData->accY[0] + sensorData->accZ[0]*sensorData->accZ[0]);

		float accX_normalized = sensorData->accX[0]/accNorm;
		float accY_normalized = sensorData->accY[0]/accNorm;
//		float accZ_normalized = sensorData->accZ[0]/accNorm;

//		if (accZ_normalized > 1)
//		{
//			accZ_normalized = 1;
//		}

		roll = asin(accY_normalized);
		pitch = -asin(accX_normalized)/cos(roll);

		quatData->quatAcc[0].w = cos(pitch/2)*cos(roll/2);
		quatData->quatAcc[0].x = cos(pitch/2)*sin(roll/2);
		quatData->quatAcc[0].y = sin(pitch/2)*cos(roll/2);
		quatData->quatAcc[0].z = -sin(pitch/2)*sin(roll/2);
	}
}

void WIDM_quat2eul(WIDM_QuatData_t* quatData)
{
	float w = quatData->quatTVCF.w;
	float x = quatData->quatTVCF.x;
	float y = quatData->quatTVCF.y;
	float z = quatData->quatTVCF.z;

	// Euler angle (ZXY order -> X is gimbal lock axis)

	double t0 = 2.0 * (w * x - y * z);
	t0 = fmax(fmin(t0, 1.0), -1.0);
	quatData->roll = asin(t0) * 57.29577951308232;

	double t1 = 2.0 * (w * y + z * x);
	double t2 = 1.0 - 2.0 * (x * x + y * y);
	quatData->pitch = atan2(t1, t2) * 57.29577951308232;

	double t3 = 2.0 * (w * z + x * y);
	double t4 = 1.0 - 2.0 * (y * y + z * z);
	quatData->yaw = atan2(t3, t4) * 57.29577951308232;

}

void Ent_GyroCalibration_WalkONS(WIDM_SensorData_t* widmSensorDataObj, IMU_Params_t* imu_params)
{
	// Temporal initial setting
	for (int i = 0; i < 3; i++) {           // for raw data savingen getting initial angles
		imu_params->accScaleFactor[i] = 1;
		imu_params->accBias[i] = 0;
		imu_params->gyrScaleFactor[i] = 1;
		imu_params->gyrBias[i] = 0;
		imu_params->gyrSign[i] = 1;
	}

	widmSensorDataObj->tTotalSamples  = 5000;
	widmSensorDataObj->tDataCheck_IMU = 0;
	widmSensorDataObj->tRealSamples   = 0;

	for (uint8_t i = 0; i <3; i++){
		widmSensorDataObj->gyr_sum[i] = 0;
		widmSensorDataObj->acc_sum[i] = 0;
	}
}

void Run_GyroCalibration_WalkONS(IOIF_6AxisData_t* imu6AxisDataObj, IMU_Params_t* imu_params, WIDM_SensorData_t* widmSensorDataObj, WIDM_QuatData_t* widmQuatDataObj)
{
	widmSensorDataObj->tDataCheck_IMU = IOIF_Get6AxisValue(imu6AxisDataObj, imu_params);

	if (widmSensorDataObj->tDataCheck_IMU == 0) {
		widmSensorDataObj->gyr_sum[0] += imu6AxisDataObj->gyrX;
		widmSensorDataObj->gyr_sum[1] += imu6AxisDataObj->gyrY;
		widmSensorDataObj->gyr_sum[2] += imu6AxisDataObj->gyrZ;
		widmSensorDataObj->acc_sum[0] += imu6AxisDataObj->accX;
		widmSensorDataObj->acc_sum[1] += imu6AxisDataObj->accY;
		widmSensorDataObj->acc_sum[2] += imu6AxisDataObj->accZ;

    	widmSensorDataObj->tRealSamples++;
	}
	else {
		HAL_I2C_DeInit(&hi2c3);
		HAL_I2C_Init(&hi2c3);
	}

	if (widmSensorDataObj->tRealSamples == widmSensorDataObj->tTotalSamples) {
		for (uint8_t i = 0; i < 3; i++) {
			imu_params->gyrBias[i] = widmSensorDataObj->gyr_sum[i] / widmSensorDataObj->tRealSamples;

			if (i == 0) { imu6AxisDataObj->accX = widmSensorDataObj->acc_sum[i] / widmSensorDataObj->tRealSamples; }
			else if (i == 1) { imu6AxisDataObj->accY = widmSensorDataObj->acc_sum[i] / widmSensorDataObj->tRealSamples; }
			else if (i == 2) { imu6AxisDataObj->accZ = widmSensorDataObj->acc_sum[i] / widmSensorDataObj->tRealSamples; }
		}

		UpdateSensorRawData_WalkONS(widmSensorDataObj, imu6AxisDataObj, imu_params);
		WIDM_GetquatAcc(widmSensorDataObj, widmQuatDataObj);
		WIDM_quatLog(&(widmQuatDataObj->quatAcc[0]), &(widmQuatDataObj->so3Acc[1]));
		WIDM_quatLog(&(widmQuatDataObj->quatAcc[0]), &(widmQuatDataObj->so3Acc[2]));
		WIDM_quatLog(&(widmQuatDataObj->quatAcc[0]), &(widmQuatDataObj->so3AccFiltered[1]));
		WIDM_quatLog(&(widmQuatDataObj->quatAcc[0]), &(widmQuatDataObj->so3AccFiltered[2]));

		widmSensorDataObj->Calibration_DONE = 1;
	}
}

/*
    _______ ____                            __      __           __
   / ____(_) / /____  _____      ________  / /___ _/ /____  ____/ /
  / /_  / / / __/ _ \/ ___/_____/ ___/ _ \/ / __ `/ __/ _ \/ __  /
 / __/ / / / /_/  __/ /  /_____/ /  /  __/ / /_/ / /_/  __/ /_/ /
/_/   /_/_/\__/\___/_/        /_/   \___/_/\__,_/\__/\___/\__,_/

*/

void get2ndFilterCoefficient_LPF(float f_cut, float ts, float H0, float Q, double b[], double a[])
{
    float w0 = 2*3.14159265358979323846*f_cut;

    double a0 = 4/ts/ts + 2*w0/(Q*ts) + w0*w0;
    double a1 = 2*w0*w0 - 8/ts/ts;
    double a2 = 4/ts/ts - 2*w0/(Q*ts) + w0*w0;

    double b0 = H0 * w0*w0 + w0/Q/ts;
    double b1 = 2 * H0 * w0*w0;
    double b2 = H0 * w0*w0 - w0/Q/ts;

    a[0] = 1;
    a[1] = a1/a0;
    a[2] = a2/a0;

    b[0] = b0/a0;
    b[1] = b1/a0;
    b[2] = b2/a0;
}

void get3rdFilterCoefficient_LPF(float f_cut, float ts, float H0, float Q, double b[], double a[])
{
//    float w0 = 2 * M_PI * f_cut;
//
//    float a0 = 8   / pow(ts,3) + 4 * w0 / (Q * pow(ts,2)) + 2 * pow(w0,2) / ts + pow(w0,3);
//    float a1 = -24 / pow(ts,3) + 2 * pow(w0,2) / ts + 4 * pow(w0,2) / ts - 3 * pow(w0,3);
//    float a2 = 24  / pow(ts,3) - 4 * w0 / (Q * pow(ts,2)) + 2 * pow(w0,2) / ts + pow(w0,3);
//    float a3 = -8  / pow(ts,3) + 4 * w0 / (Q * pow(ts,2)) - 2 * pow(w0,2) / ts + pow(w0,3);
//
//    float b0 = 1 * H0 * pow(w0,3) + 3 * w0 / (Q * pow(ts,2)) + 3 * pow(w0,2) / ts + pow(w0,3);
//    float b1 = 3 * H0 * pow(w0,3) + 3 * w0 / (Q * pow(ts,2)) + 3 * pow(w0,2) / ts;
//    float b2 = 3 * H0 * pow(w0,3) + 3 * w0 / (Q * pow(ts,2)) + 3 * pow(w0,2) / ts;
//    float b3 = 1 * H0 * pow(w0,3) - 3 * w0 / (Q * pow(ts,2)) - 3 * pow(w0,2) / ts + pow(w0,3);
//
//    a[0] = 1;
//    a[1] = a1 / a0;
//    a[2] = a2 / a0;
//    a[3] = a3 / a0;
//
//    b[0] = b0 / a0;
//    b[1] = b1 / a0;
//    b[2] = b2 / a0;
//    b[3] = b3 / a0;

	  a[0] = 1;
	  a[1] = -2.993716817276653;
	  a[2] = 2.987453358242849;
	  a[3] = -0.993736510057099;

	  b[0] = 0.0000000038636370830198;
	  b[1] = 0.0000000115909112490594;
	  b[2] = 0.0000000115909112490594;
	  b[3] = 0.0000000038636370830198;
}

void get2ndFilterCoefficient_HPF(float f_cut, float ts, float H0, float Q, double b[], double a[])
{
	float w0 = 2*3.14159265358979323846*f_cut;

	double a0 = 4/ts/ts + 2*w0/(Q*ts) + w0*w0;
	double a1 = 2*w0*w0 - 8/ts/ts;
	double a2 = 4/ts/ts - 2*w0/(Q*ts) + w0*w0;

	double b0 = 4 * H0 / ts/ts + w0/Q/ts;
	double b1 = -8 * H0 / ts/ts;
	double b2 = 4 * H0 / ts/ts - w0/Q/ts;

    a[0] = 1;
    a[1] = a1/a0;
    a[2] = a2/a0;

    b[0] = b0/a0;
    b[1] = b1/a0;
    b[2] = b2/a0;
}

float WIDM_LPF_WalkONS(float data, float data_f, float data_ff, float dataFiltered_f, float dataFiltered_ff, float cutoffFreq, float samplingPeriod)
{
	double b[3];
	double a[3];

	get2ndFilterCoefficient_LPF(cutoffFreq, samplingPeriod, 1, 1/sqrt(2), b, a);

	return b[0]*data + b[1]*data_f + b[2]*data_ff - a[1]*dataFiltered_f - a[2]*dataFiltered_ff;
}

double WIDM_LPF_WalkONS_3rd(double data, double data_f, double data_ff, double data_fff, double dataFiltered_f, double dataFiltered_ff, double dataFiltered_fff, float cutoffFreq, float samplingPeriod)
{
//	static uint32_t flag = 0;
//	static double b[4];
//	static double a[4];
//
//	if (flag == 0) {
//		get3rdFilterCoefficient_LPF(cutoffFreq, samplingPeriod, 1, 1/sqrt(2), b, a);
//		flag ++;
//		return b[0]*data + b[1]*data_f + b[2]*data_ff + b[3]*data_fff - a[1]*dataFiltered_f - a[2]*dataFiltered_ff - a[3]*dataFiltered_fff;
//	}
//	else {
//		flag ++;
//		return b[0]*data + b[1]*data_f + b[2]*data_ff + b[3]*data_fff - a[1]*dataFiltered_f - a[2]*dataFiltered_ff - a[3]*dataFiltered_fff;
//	}

	double b[4] = {0.0000000038636370830198,
				   0.0000000115909112490594,
				   0.0000000115909112490594,
				   0.0000000038636370830198};
	double a[4] = {1,
			       -2.993716817276653,
			       2.987453358242849,
			       -0.993736510057099};

	//get3rdFilterCoefficient_LPF(cutoffFreq, samplingPeriod, 1, 1/sqrt(2), b, a);

	return b[0]*data + b[1]*data_f + b[2]*data_ff + b[3]*data_fff - a[1]*dataFiltered_f - a[2]*dataFiltered_ff - a[3]*dataFiltered_fff;

}

float WIDM_HPF_WalkONS(float data, float data_f, float data_ff, float dataFiltered_f, float dataFiltered_ff, float cutoffFreq, float samplingPeriod)
{
	double b[3];
	double a[3];

	get2ndFilterCoefficient_HPF(cutoffFreq, samplingPeriod, 1, 1/sqrt(2), b, a);

	return b[0]*data + b[1]*data_f + b[2]*data_ff - a[1]*dataFiltered_f - a[2]*dataFiltered_ff;
}


/*
  _______    ______________         __                 _ __  __
 /_  __/ |  / / ____/ ____/  ____ _/ /___ _____  _____(_) /_/ /_  ____ ___
  / /  | | / / /   / /_     / __ `/ / __ `/ __ \/ ___/ / __/ __ \/ __ `__ \
 / /   | |/ / /___/ __/    / /_/ / / /_/ / /_/ / /  / / /_/ / / / / / / / /
/_/    |___/\____/_/       \__,_/_/\__, /\____/_/  /_/\__/_/ /_/_/ /_/ /_/
                                  /____/
*/

/* Reset Value Zero */
void ResetDataObj(IMU_Params_t* imu_params, WIDM_SensorData_t* widmSensorDataObj, WIDM_QuatData_t* widmQuatDataObj, WIDM_FuzzyData_t* widmFuzzyDataObj)
{
	if (imu_params != NULL) {
		*imu_params = (IMU_Params_t){0};
	}
	if (widmSensorDataObj != NULL) {
		for (uint8_t i = 0; i < 3; i++){
			widmSensorDataObj->gyr_sum[i] = 0;
			widmSensorDataObj->acc_sum[i] = 0;
		}
		for (uint8_t i = 0; i < 2; i++){
			widmSensorDataObj->accX[i] = 0;
			widmSensorDataObj->accY[i] = 0;
			widmSensorDataObj->accZ[i] = 0;
			widmSensorDataObj->gyrX[i] = 0;
			widmSensorDataObj->gyrY[i] = 0;
			widmSensorDataObj->gyrZ[i] = 0;
		}
		widmSensorDataObj->gyr_sum[0] = 0;



	}
	if (widmQuatDataObj != NULL) {
		*widmQuatDataObj = (WIDM_QuatData_t){0};
	}
	if (widmFuzzyDataObj != NULL) {
		*widmFuzzyDataObj = (WIDM_FuzzyData_t){0};
	}
}

int SetInitialPosture_WalkONS(IOIF_6AxisData_t* imu6AxisDataObj, IMU_Params_t* imu_params)
{

	if (act_type == 10)
	{
		imu_params->R_Matrix[0][0] =  0;
		imu_params->R_Matrix[0][1] =  0;
		imu_params->R_Matrix[0][2] =  1;
		imu_params->R_Matrix[1][0] =  0.70710678;
		imu_params->R_Matrix[1][1] = -0.70710678;
		imu_params->R_Matrix[1][2] =  0;
		imu_params->R_Matrix[2][0] =  0.70710678;
		imu_params->R_Matrix[2][1] =  0.70710678;
		imu_params->R_Matrix[2][2] =  0;

	}
	else if (act_type == 11)
	{
		imu_params->R_Matrix[0][0] =  0;
		imu_params->R_Matrix[0][1] =  0;
		imu_params->R_Matrix[0][2] =  1;
		imu_params->R_Matrix[1][0] =  -0.70710678;
		imu_params->R_Matrix[1][1] =  -0.70710678;
		imu_params->R_Matrix[1][2] =  0;
		imu_params->R_Matrix[2][0] =  0.70710678;
		imu_params->R_Matrix[2][1] =  -0.70710678;
		imu_params->R_Matrix[2][2] =  0;
	}

	return 0;
}

void SetFilterInitialValue_WalkONS(IOIF_6AxisData_t* imu6AxisDataObj, WIDM_QuatData_t* quatData, IMU_Params_t* imu_params)
{

	// Set initial value of quatGyr

	for (uint8_t i = 0; i < 3; i++) {
		quatData->quatGyr[i].w = 1;
		quatData->quatGyr[i].x = 0;
		quatData->quatGyr[i].y = 0;
		quatData->quatGyr[i].z = 0;
		quatData->quatAcc[i].w = 1;
		quatData->quatAcc[i].x = 0;
		quatData->quatAcc[i].y = 0;
		quatData->quatAcc[i].z = 0;
	}

	for (uint8_t i = 0; i < 3; i++) {
		quatData->so3GyrFiltered[i].x = 0;
		quatData->so3GyrFiltered[i].y = 0;
		quatData->so3GyrFiltered[i].z = 0;
		quatData->so3AccFiltered[i].x = 0;
		quatData->so3AccFiltered[i].y = 0;
		quatData->so3AccFiltered[i].z = 0;
	}

}

void UpdateSensorRawData_WalkONS(WIDM_SensorData_t* widmSensorData, IOIF_6AxisData_t* imu6AxisData, IMU_Params_t* imu_params)
{
//	widmSensorData->accX[0] = imu6AxisData->accX;
//	widmSensorData->accY[0] = imu6AxisData->accY;
//	widmSensorData->accZ[0] = imu6AxisData->accZ;
//	widmSensorData->gyrX[0] = imu6AxisData->gyrX;
//	widmSensorData->gyrY[0] = imu6AxisData->gyrY;
//	widmSensorData->gyrZ[0] = imu6AxisData->gyrZ;

	widmSensorData->accX[0] = imu_params->R_Matrix[0][0]*imu6AxisData->accX + imu_params->R_Matrix[0][1]*imu6AxisData->accY + imu_params->R_Matrix[0][2]*imu6AxisData->accZ;
	widmSensorData->accY[0] = imu_params->R_Matrix[1][0]*imu6AxisData->accX + imu_params->R_Matrix[1][1]*imu6AxisData->accY + imu_params->R_Matrix[1][2]*imu6AxisData->accZ;
	widmSensorData->accZ[0] = imu_params->R_Matrix[2][0]*imu6AxisData->accX + imu_params->R_Matrix[2][1]*imu6AxisData->accY + imu_params->R_Matrix[2][2]*imu6AxisData->accZ;
	widmSensorData->gyrX[0] = imu_params->R_Matrix[0][0]*imu6AxisData->gyrX + imu_params->R_Matrix[0][1]*imu6AxisData->gyrY + imu_params->R_Matrix[0][2]*imu6AxisData->gyrZ;
	widmSensorData->gyrY[0] = imu_params->R_Matrix[1][0]*imu6AxisData->gyrX + imu_params->R_Matrix[1][1]*imu6AxisData->gyrY + imu_params->R_Matrix[1][2]*imu6AxisData->gyrZ;
	widmSensorData->gyrZ[0] = imu_params->R_Matrix[2][0]*imu6AxisData->gyrX + imu_params->R_Matrix[2][1]*imu6AxisData->gyrY + imu_params->R_Matrix[2][2]*imu6AxisData->gyrZ;
}

void WIDM_CalculateFuzzyM_WalkONS(WIDM_FuzzyData_t* fuzzyData, WIDM_SensorData_t* sensorData, float samplingPeriod)
{
	fuzzyData->wc = 0;
	fuzzyData->m_acc = WIDM_AbsoluteValue(WIDM_SquareRootSum(sensorData->accX[0], sensorData->accY[0], sensorData->accZ[0]) - 1);
	fuzzyData->m_gyr = WIDM_SquareRootSum(sensorData->gyrZ[0], sensorData->gyrY[0], sensorData->gyrZ[0]);
	fuzzyData->m[0] = fuzzyData->alpha*fuzzyData->m_acc + fuzzyData->m_gyr;

	if (fuzzyData->wc_m == 0)      // wc_m: determined in Ent Coder
	{
		fuzzyData->m_filtered[0] = fuzzyData->m[0];
//		fuzzyData->wc = (fuzzyData->wh - fuzzyData->wl)*(1 - tanh(fuzzyData->s*(fuzzyData->m[0] - fuzzyData->m0)))*0.5 + fuzzyData->wl;
//		fuzzyData->wc = 1;
	}
	else
	{
		fuzzyData->m_filtered[0] = WIDM_LPF_WalkONS_3rd(fuzzyData->m[0], fuzzyData->m[1], fuzzyData->m[2], fuzzyData->m[3], fuzzyData->m_filtered[1], fuzzyData->m_filtered[2], fuzzyData->m_filtered[3], fuzzyData->wc_m, samplingPeriod);
//		fuzzyData->wc = (fuzzyData->wh - fuzzyData->wl)*(1 - tanh(fuzzyData->s*(fuzzyData->m_filtered[0] - fuzzyData->m0)))*0.5 + fuzzyData->wl;
	}
}

void RunTvcfFilter_WalkONS(WIDM_SensorData_t* widmSensorData, WIDM_QuatData_t* widmQuatData, WIDM_FuzzyData_t* widmFuzzyData, float samplingPeriod)
{
	/* TVCF algorithm with flushing former values */
	if (widmFuzzyData->m_filtered[0] > widmFuzzyData->m0)                 widmSensorData->current_flush_flag = 1;
	else                                                                  widmSensorData->current_flush_flag = 0;

	if (widmSensorData->current_flush_flag == widmSensorData->flush_flag)
	{
		if (widmSensorData->current_flush_flag == 1)                      widmFuzzyData->wc = 0.0001; // fixed wc or variable wc?
		else                                                              widmFuzzyData->wc = 1; // fixed wc or variable wc?

		WIDM_RunTVCF_WalkONS(widmSensorData, widmQuatData, widmFuzzyData->wc, samplingPeriod);
	}
	else
	{
		WIDM_FlushBuffer_WalkONS(widmQuatData);
	}

	widmSensorData->flush_flag = widmSensorData->current_flush_flag;
}

void WIDM_RunTVCF_WalkONS(WIDM_SensorData_t* sensorData, WIDM_QuatData_t* quatData, float cutoffFreq, float samplingPeriod)
{
	/* Calculate the quat using gyroscope measurements and convert it to quaternion -> quatGyr[0] */
	WIDM_GetquatGyr(sensorData, quatData, samplingPeriod);

	/* Logarithm operator to quatGyr[0] */
	WIDM_quatLog(&(quatData->quatGyr[0]), &(quatData->so3Gyr[0]));

	/* Calculate the quat using accelerometer measurements and convert it to quaternion -> quatAcc[0] */
	WIDM_GetquatAcc(sensorData, quatData);

	/* Logarithm operator to quatAcc[0] */
	WIDM_quatLog(&(quatData->quatAcc[0]), &(quatData->so3Acc[0]));

	/* Apply High Pass Filter (HPF) on gyroscope measurements */
	quatData->so3GyrFiltered[0].x = WIDM_HPF_WalkONS(quatData->so3Gyr[0].x,
													 quatData->so3Gyr[1].x,
													 quatData->so3Gyr[2].x,
													 quatData->so3GyrFiltered[1].x,
													 quatData->so3GyrFiltered[2].x,
													 cutoffFreq,
													 samplingPeriod);
	quatData->so3GyrFiltered[0].y = WIDM_HPF_WalkONS(quatData->so3Gyr[0].y,
													 quatData->so3Gyr[1].y,
													 quatData->so3Gyr[2].y,
													 quatData->so3GyrFiltered[1].y,
													 quatData->so3GyrFiltered[2].y,
													 cutoffFreq,
													 samplingPeriod);
	quatData->so3GyrFiltered[0].z = WIDM_HPF_WalkONS(quatData->so3Gyr[0].z,
													 quatData->so3Gyr[1].z,
													 quatData->so3Gyr[2].z,
													 quatData->so3GyrFiltered[1].z,
													 quatData->so3GyrFiltered[2].z,
													 cutoffFreq,
													 samplingPeriod);

	/* Apply Low Pass Filter (LPF) on accelerometer angle */
	quatData->so3AccFiltered[0].x = WIDM_LPF_WalkONS(quatData->so3Acc[0].x,
													 quatData->so3Acc[1].x,
													 quatData->so3Acc[2].x,
													 quatData->so3AccFiltered[1].x,
													 quatData->so3AccFiltered[2].x,
													 cutoffFreq,
													 samplingPeriod);
	quatData->so3AccFiltered[0].y = WIDM_LPF_WalkONS(quatData->so3Acc[0].y,
													 quatData->so3Acc[1].y,
													 quatData->so3Acc[2].y,
													 quatData->so3AccFiltered[1].y,
													 quatData->so3AccFiltered[2].y,
													 cutoffFreq,
													 samplingPeriod);
	quatData->so3AccFiltered[0].z = WIDM_LPF_WalkONS(quatData->so3Acc[0].z,
													 quatData->so3Acc[1].z,
													 quatData->so3Acc[2].z,
													 quatData->so3AccFiltered[1].z,
													 quatData->so3AccFiltered[2].z,
													 cutoffFreq,
													 samplingPeriod);

	/* Combine filtered accelerometer and gyroscope measurements */
	WIDM_so3_t combined_so3 = {quatData->so3GyrFiltered[0].x + quatData->so3AccFiltered[0].x,
							   quatData->so3GyrFiltered[0].y + quatData->so3AccFiltered[0].y,
							   quatData->so3GyrFiltered[0].z + quatData->so3AccFiltered[0].z};

	WIDM_quatExp(&combined_so3, &(quatData->quatTVCF));
}

void WIDM_UpdateBuffer_WalkONS(WIDM_SensorData_t* sensorData, WIDM_QuatData_t* quatData, WIDM_FuzzyData_t* fuzzyData)
{
	sensorData->gyrX[1] = sensorData->gyrX[0];
	sensorData->gyrY[1] = sensorData->gyrY[0];
	sensorData->gyrZ[1] = sensorData->gyrZ[0];

	sensorData->accX[1] = sensorData->accX[0];
	sensorData->accY[1] = sensorData->accY[0];
	sensorData->accZ[1] = sensorData->accZ[0];

	quatData->quatGyr[2] = quatData->quatGyr[1];
	quatData->quatGyr[1] = quatData->quatGyr[0];

	quatData->quatAcc[2] = quatData->quatAcc[1];
	quatData->quatAcc[1] = quatData->quatAcc[0];

	quatData->so3Gyr[2] = quatData->so3Gyr[1];
	quatData->so3Gyr[1] = quatData->so3Gyr[0];

	quatData->so3GyrFiltered[2] = quatData->so3GyrFiltered[1];
	quatData->so3GyrFiltered[1] = quatData->so3GyrFiltered[0];

	quatData->so3Acc[2] = quatData->so3Acc[1];
	quatData->so3Acc[1] = quatData->so3Acc[0];

	quatData->so3AccFiltered[2] = quatData->so3AccFiltered[1];
	quatData->so3AccFiltered[1] = quatData->so3AccFiltered[0];

	fuzzyData->m[3] = fuzzyData->m[2];
	fuzzyData->m[2] = fuzzyData->m[1];
	fuzzyData->m[1] = fuzzyData->m[0];

	fuzzyData->m_filtered[3] = fuzzyData->m_filtered[2];
	fuzzyData->m_filtered[2] = fuzzyData->m_filtered[1];
	fuzzyData->m_filtered[1] = fuzzyData->m_filtered[0];
}

void WIDM_FlushBuffer_WalkONS(WIDM_QuatData_t* quatData)
{
	quatData->so3Acc[0] = quatData->so3Acc[1];
	quatData->so3Acc[2] = quatData->so3Acc[1];

	quatData->so3AccFiltered[0] = quatData->so3AccFiltered[1];
	quatData->so3AccFiltered[2] = quatData->so3AccFiltered[1];

	quatData->so3Gyr[0] = quatData->so3Gyr[1];
	quatData->so3Gyr[2] = quatData->so3Gyr[1];

	quatData->so3GyrFiltered[0] = quatData->so3GyrFiltered[1];
	quatData->so3GyrFiltered[2] = quatData->so3GyrFiltered[1];
}
