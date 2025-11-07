/**
 * @file low_level_ctrl_task.h
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics FW Team
 * @brief 
 * @ref 
 */

#ifndef APPS_LOW_LEVEL_CTRL_TASK_INC_LOW_LEVEL_CTRL_TASK_H_
#define APPS_LOW_LEVEL_CTRL_TASK_INC_LOW_LEVEL_CTRL_TASK_H_

#include <string.h>
#include <stdint.h>
#include <stdlib.h>

#include "ioif_adc_common.h"
#include "ioif_gpio_common.h"
#include "ioif_fdcan_common.h"
#include "ioif_wwdg_common.h"
#include "ioif_flash_common.h"
#include "ioif_tmcs1100a2.h"

#include "ioif_rmb20ic.h"
#include "ioif_hall_sensor.h"

#include "mid_level_ctrl_task.h"
#include "system_ctrl_task.h"

#include "signal_generator.h"
#include "motor_controller.h"
#include "Bumblebee.h"
#include "msg_common.h"
#include "../../../Low_Level_Ctrl_Task/Commutation_Ctrl/Inc/inverter.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define LOW_LEVEL_CONTROL_FREQUENCY      		25000
#define LOW_LEVEL_CONTROL_PERIOD      			0.00004
#define FRICTION_ID_STEP_NUM					30

#define ADC1_BUFFER_LENGTH          6
#define CURRENT_SENSOR_VOLTAGE      3.3
#define CURRENT_SENSOR_SENSITIVITY  0.0138 //0.1//0.110    	// V/A
#define CURRENT_SENSOR_RANGE		240//29//25			// in Amphere
#define ADC1_RESOLUTION        		65536
#define CURRENT_CTRL_PERFORMANCE_CHECK_CNT	2000

#define VISCOUS_COMPENSATION_RATIO	0.9
#define COULOMB_COMPENSATION_RATIO	1.0

#define FRICTION_LUT_SIZE	8193


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

/* Commutation Feedback Sensor */
typedef enum _SensorCommutation {
	e_Commute_Sensor_Inc_Encoder = (uint8_t)0,
	e_Commute_Sensor_Hall,
	e_Commute_Sensor_Inc_Encoder_Hall
} SensorCommutation;

/* Position Feedback Sensor */
typedef enum _SensorPosFeedback {
	e_Pos_Sensor_None = (uint8_t)0,
	e_Pos_Sensor_Inc_Encoder,
	e_Pos_Sensor_Abs_Encoder1,
	e_Pos_Sensor_Abs_Encoder2
} SensorPosFeedback;

/* Electrical Angle Homing Sensor */
typedef enum _SensorElecAngleHoming{
	e_EHoming_Sensor_Forced= (uint8_t)0,
	e_EHoming_Sensor_Hall,
	e_EHoming_Sensor_Abs_Encoder1,
	e_EHoming_Sensor_Abs_Encoder2
}SensorElecAngleHoming;

/* Mechanical Angle Homing Sensor*/
typedef enum _SensorMechAngleHoming{
	e_MHoming_Sensor_Zero= (uint8_t)0,  // default: position(t=0) = 0;
	e_MHoming_Sensor_Abs_Encoder1,      //          position(t=0) = abs_encoder1_position(t=0);
	e_MHoming_Sensor_Abs_Encoder2,      //          position(t=0) = abs_encoder2_position(t=0);
	e_MHoming_Sensor_MD_Calculation,    //          position(t=0) = MD_calculated_value;
	e_MHoming_Sensor_CM_Setting,        //          position(t=0) = CM_setting_value;
}SensorMechAngleHoming;

typedef struct _MotorProperties {
	uint8_t name_length;
	char name[31];

	uint8_t hall_sensor_table[6]; // calibrated hall sensor table (CCW)

	float J; // moment of inertia
	float B; // friction coefficient
	float a, b, c, d;

	float Vbus;     // bus voltage (V)
	float R; 		// resistance (ohm)
	float L; 		// inductance (mH)
	uint8_t pole_pair;
	float Ke; // bEMF constant (V/kRPM)
	float Kt; // torque constant (Nm/A)
	float gear_ratio;
	uint8_t md_version;
} MotorProperties;

typedef struct _SensorSetting {

	uint8_t incremetnal_encoder_usage;
	uint8_t absolute_encoder1_usage;
	uint8_t absolute_encoder2_usage;
	uint8_t hall_sensor_usage;

	SensorElecAngleHoming e_angle_homing_sensor;
	SensorMechAngleHoming m_angle_homing_sensor;
	SensorCommutation commutation_sensor;
	SensorPosFeedback pos_feedback_sensor;
	uint8_t temperature_sensor_usage;
	uint8_t imu_6axis_usage;
	uint8_t imu_3axis_usage;

	float   curr_sensor_range;
	float   curr_sensor_d2a;
	float   curr_sensor_a2d;

} SensorSetting;

typedef struct _MotorSetting {
	ModeOfOperation 	mode_of_operation;
	SensorSetting       sensor_setting;
	CommutationSetting  commutation_set;
	ElecAngleHoming     elec_angle_homing;
	InputInfo 			input_info;

	uint8_t  low_level_kalman_on;
	float currCtrl_BW_radPsec;
	float bemf_id_vel;
	float peakCurr_limit;
	float contCurr_limit;
	float max_velocity_rpm;
	float max_velocity_radPsec;		//in rad/s
	float bemf_id_gain;
	uint8_t bemf_id_gain_pctg;

} MotorSetting;

typedef struct _MotorOut {
	uint16_t elec_angle;
	uint16_t actual_elec_angle;
    int32_t I_U, I_V, I_W;                  		// 3-phase & low-side universal current
    int32_t I_U_offset, I_V_offset, I_W_offset;  	// ADC current calibration offset
    int32_t V_U, V_V, V_W;
    int32_t V_U_offset, V_V_offset, V_W_offset;
    float I_U_KF, I_V_KF, I_W_KF;
	float current_act;
	float position, position_f;
	float velocity, velocity_f;

	int16_t avelec_angle_U, avelec_angle_V, avelec_angle_W;
} MotorOut;

typedef struct _MotorIn {
    float V_U_input, V_V_input, V_W_input;

    /* From msg_hdlr_task */
	float auxiliary_input;

	/* From low_level_ctrl_task */
	float low_id_process_input;
	float friction_input;

	/* From mid_level_ctrl_task */
	float f_vector_input; // no feedback

	float mid_id_process_input;
	float irc_input;
	float analysis_input;
	float mid_ctrl_input;		// (imp_ctrl_input + mid_pid_ctrl_input + f_vector_input + ff_input + dob_input)

	/* Sum of all of inputs */
	float total_current_input;

	float ls_ctrl_input;
} MotorIn;

typedef struct _MotorElecSystemID {

	uint8_t flag;
	uint32_t cnt;
	uint16_t input;
	uint16_t signal;
	int32_t f_signal;
	int32_t output;
	float level;

//	int32_t input_arr[CHIRP_ARRAY_SIZE];
//	int32_t output_arr[CHIRP_ARRAY_SIZE];

} MotorElecSystemID;

typedef struct _Advanced_Friction_ID {
	float friction_cnt;
	uint8_t friction_index;

	float friction_ref;

	float max_vel1;
	float max_vel2;
	float vel_num1;
	float vel_num2;
	float duration;  // id time per single velocity reference (unit: s)
	float section_time[8];
	uint32_t time_per_vel;
	uint32_t cut_time;

	float gap1;
	float gap2;

    float P_gain;
    float I_gain;

    float e;
    float e_sum;

	uint8_t state;

	float scaling_factor; //for compensation

	float lut_mdl;
	float lut_p1;
	float lut_p2;
	float lut_p3;
	float lut_p4;
	float lut_p5;
	float lut_d_mu_v;
	float lut_epsilon;
	float gain;

	float adv_friction_compensator_LUT[FRICTION_LUT_SIZE];

	float overall_gain_curr;
    float overall_gain_des;
    float overall_gain_gap;
    uint8_t overall_gain_trigger;
    uint16_t overall_gain_time_stamp;

    float overall_gain_transition_time;

} Advanced_Friction_ID;

typedef enum _FrictionModel {
	BASIC_MODEL = (uint8_t)1,
	STRIBECK_MODEL,
	KARNO_MODEL
} FrictionModel;

typedef struct _TempSense {

	float stator_raw;
	float stator_raw_pre;
	float stator_filtered;
	float stator_initial;
	float voltage_initial;

	float max_temp;

} TempSense;



/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

extern TaskStruct 			low_level_ctrl_task;
extern MotorProperties 		motor_properties;
extern MotorSetting			motor_setting;
extern MotorIn 				motor_in;
extern MotorOut				motor_out;
extern Advanced_Friction_ID advanced_friction_id;
extern IOIF_IncEnc_t 		inc25KhzObj;
extern IOIF_HallSensor_t 	hallObj;
extern KFObject				kf_current_object;
extern TempSense            temp_sense;

extern uint8_t              low_level_triggered_msg_stop;

/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void Cal_Elec_System_ID_Batch(void);
void Send_Elec_BEMF_Value(void);
void Send_Elec_Bandwidth_Data(void);
void Send_Elec_Tracking_Data(void);
void Send_Advanced_Friction_ID_Data(void);
void Cal_Friction_LUT(void);

void InitLowLvCtrl();
void RunLowLvCtrl(void* params);


#endif /* APPS_LOW_LEVEL_CTRL_TASK_INC_LOW_LEVEL_CTRL_TASK_H_ */

