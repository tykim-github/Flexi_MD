/**
 * @file mid_level_ctrl_task.h
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics FW Team
 * @brief 
 * @ref 
 */

#ifndef APPS_MID_LEVEL_CTRL_TASK_INC_MID_LEVEL_CTRL_TASK_H_
#define APPS_MID_LEVEL_CTRL_TASK_INC_MID_LEVEL_CTRL_TASK_H_

#include "task_mngr.h"

#include "signal_generator.h"


#include "system_ctrl_task.h"


#include "msg_common.h"
#include "motor_controller.h"

#include "ioif_rmb20ic.h"
#include "ioif_rmb20sc.h"
#include "../../Low_Level_Ctrl_Task/Inc/low_level_ctrl_task.h"
//#include "../../Gait_Ctrl/Inc/gait_ctrl_task.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define MID_LEVEL_CONTROL_FREQUENCY     1000
#define MID_LEVEL_CONTROL_PERIOD      	0.001
#define PRBS_ARR_SIZE					8191
#define SBS_FREQUENCY_MAX				50
#define SBS_FREQUENCY_MIN				0.5
#define SBS_FREQUENCY_STEP_NUM			20
#define SBS_REPEAT_NUM					20

#define P_VECTOR_BUFF_SIZE			    20
#define F_VECTOR_BUFF_SIZE		        10
#define I_VECTOR_BUFF_SIZE			    10

#define F_MODE_NUM                 10

#define CONTROLLER_TRANSITION_DURATION  500 // unit: ms

/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef struct _PeriodicSignal {
	float amp;
	float freq;
	float offset;
} PeriodicSignal;

typedef struct _VirtualSpringDamper {
	float lower_stiffness, upper_stiffness;
	float lower_damper, upper_damper;

	float lower_limit, upper_limit;
	float lower_damped_range, upper_damped_range;
	float lower_stiff_range, upper_stiff_range;

	float lower_damper_origin, upper_damper_origin;
	float lower_spring_origin, upper_spring_origin;

	float control_input;
	float saturation;
} VirtualSpringDamper;

typedef struct _MidLevelState {
	float initial_pos;
	float position;    // unit: rad
	float position_f;
	float position_ff;

	float position_offset;
	float init_position;

	float velocity_raw;
	float velocity_final;

	uint8_t enable_freeze;
} MidLevelState;

typedef struct _System_ID_SBS {
	float fmin;
	float fmax;
	float N_iter;
	float N_samples;

	float amp;
	float offset;

	float *f_samples;

	float current_f;
	uint32_t f_cnt;
	uint32_t sys_id_cnt;

	float filtered_input;
	float filtered_output;
	uint8_t done;

	float verify_mag;
}System_ID_SBS;

typedef struct _Trapezoidal_ID_Obj {

	float Amp;
	float Delta;
	float Offset;
	float V_min;
	float V_max;
	float N_Samples;
	float N_Iter;

	float Current_Velocity;

	float *Vel_Samples;
	uint16_t *Durations;
	uint16_t *SineDurations;

	float PosRef;
	float PosRef_f;

	uint32_t Cnt;   // Time Tick
	uint16_t Iter;  // Current Iteration Number

	uint16_t isInit;
	uint16_t InitCnt;
	float    StartGain;

	uint16_t VelIdx; // Current Velocity Sample IDX

	uint8_t State;
	uint8_t ON;
	uint8_t Done;

}Trapezoidal_ID_Obj;

typedef struct _TVCF_Ver {
	float fmin;
	float fmax;
	float N_iter;
	float N_samples;

	float amp;
	float offset;

	float frequency;

	float delta_freq;
	float current_f;

	float *f_samples;

	uint32_t ver_cnt;
	float done;
	uint32_t f_cnt;
	float step;


}TVCF_Ver;

typedef struct _Backlash_Test{

	float range;
	float amplitude;

}Backlash_Test;

/************************ P Vector ************************/
typedef struct _P_Vector{

	int16_t yd;  // desired position  (deg)
	uint16_t L;  // trajectory length (ms)
	uint8_t s0;  // acceleration      (deg/s^2)
	uint8_t sd;  // deceleration      (deg/s^2)

}P_Vector;

typedef struct _P_Vector_Decoder{

	P_Vector p_buffer[P_VECTOR_BUFF_SIZE];
	uint8_t N;   // (=BufferCount) Number of Trajectory in Buffer
	uint8_t ON;  // (=TrajectoryON)

	float y0;    // initial position

	double a0, a2, a3, a4 ,a5; // coefficient of k'th step trajectory
	double b0, b2, b3, b4 ,b5; // coefficient of k+1'th step trajectory

	double t1;
	double t2;
	double t3;
	double t4;
	double t5;

	double L;
	double L_inv;

	uint16_t count;

	float yd_f; // yd(k-1)


}P_Vector_Decoder;


typedef struct _Cos3_Vector_Decoder{

	float Amplitude;  // Amplitude of Cos^3(x) function (unit: rad)
	float Period;     // Period of Cos^3(x) function    (unit: s)

	uint8_t  ON;      // 1: Trajectory is generating now

	uint8_t  Trigger;
	uint16_t cnt;

	float y;  // yd(k)
	float y1; // yd(k+1)
}Cos3_Vector_Decoder;

/************************ F Vector ************************/
typedef struct _F_Vector{
	uint8_t  mode_idx;     // mode
	int16_t  tau_max;      // 100*Nm
	uint16_t delay;        // delay (ms)

	float tau, tau_old1, tau_old2, u, u_old1, u_old2;
	uint32_t t_end;
	uint32_t time_stamp;
	uint8_t is_full;       // if full 1, else 0
}F_Vector;


typedef struct _F_Mode_Param{

	double wn, b0, b1, b2, a0, a1, a2;
	float  tp;

} F_Mode_Param;

typedef struct _F_Vector_Decoder {

	F_Vector f_buffer[F_VECTOR_BUFF_SIZE];
	F_Mode_Param mode_param[F_MODE_NUM];

	uint8_t temp_idx;
	float   tp[F_MODE_NUM]; // Tp to Mode Table

	float input;
} F_Vector_Decoder;

/************************ I Vector ************************/
typedef struct _I_Vector{

	uint8_t epsilon_target;     // Half width of the Corridor      (x10)
	uint8_t Kp_target;          // Magnitude of Virtual Spring
	uint8_t Kd_target;          // Magnitude of Virtual Damper
	uint8_t lambda_target;      // Impedance Ratio in the Corridor (x100)
	uint16_t duration;          // Duration for translation

}I_Vector;

typedef struct _ImpedanceCtrl {

	I_Vector i_buffer[I_VECTOR_BUFF_SIZE];
	I_Vector opt1_i_buffer;

	/* Parameters */
	float epsilon, Kp, Kd, lambda;

	float Kp_max; // Kp @ 100%
	float Kd_max; // Kd @ 100%

	float gap_epsilon;
	float gap_Kp;
	float gap_Kd;
	float gap_lambda;

	float e;  // error
	float ef; // output of error function

	float ef_f;
	float ef_diff;

	float control_input;  // control input

	float L;

	uint8_t option;
	uint16_t i; // 1ms counter
	uint8_t ON; // flag
	uint8_t N;  // buffer count

	float overall_gain_curr;
    float overall_gain_des;
    float overall_gain_gap;
    uint8_t overall_gain_trigger;
    uint16_t overall_gain_time_stamp;

    float overall_gain_transition_time;
    float on;
    float ref;
    float init_pos;


} ImpedanceCtrl;

typedef struct _WIDM_GaitData {
	float gaitPhase;			// Current Gait Phase 0 ~ 100%
	float gaitPhasePre;			// Previous Gait Phase
	uint16_t gaitPeriod;		// Gait Period (ms) < 2000ms
	uint8_t	walkingState;
} WIDM_GaitData;

typedef struct LoadCell {
	float loadcell_raw;
	float loadcell_raw_pre;
	float loadcell_filtered;
	float cutoff;
	float scale;
	float offset;
	float alpha;
	float torque;

} LoadCell;
typedef struct FlexiAnkle {
	float xa;
	float ya;
	float xs;
	float ys;
	float la;
	float ls;
	float alpha;
	float alpha_deg;
	float angle_bias;
	float ratio;
	float ratio_inv;
	float ratio_torque;
} FlexiAnkle;

typedef struct AnkleComp {
	float Jm;
	float Bm;
	float Kp;
	float theta;
	float theta_prev;
	float alpha;
	float beta;
	float theta_dot;
	float theta_dot_prev;
	float theta_ddot;
	float control_input;
	float gain;
	float cutoff;
	float a;
	float theta_dot_final;
	float theta_ddot_final;
} AnkleComp;

typedef struct pMMGSense{
	float pMMG1;
	float pMMG2;
	float pMMG3;
	float pMMG4;
} pMMGSense;

/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

extern TaskStruct mid_level_ctrl_task;
extern float mid_ctrl_saturation;
extern IOIF_AbsEnc_t AbsObj1;
extern IOIF_AbsEnc_t AbsObj2;
extern PIDObject velCtrl;
extern VelocityEstObject veObj;
extern PIDObject posCtrl;
extern DOBObject posDOB;
extern FFObject	posFF;
extern VirtualSpringDamper VSD;
extern ImpedanceCtrl impedanceCtrl;
extern IOIF_IncEnc_t inc1KhzObj;
extern ImpedanceReductionCtrl IRC;
extern uint32_t mid_level_loop_cnt;
extern MidLevelState 	mid_level_state;

extern uint8_t          mid_level_triggered_msg_stop;

extern uint8_t           MD_node_id;
/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

/* GUI */
void Tune_Gain(void);

void InitMidLvCtrl(void);
void RunMidLvCtrl(void* params);


#endif /* APPS_MID_LEVEL_CTRL_TASK_INC_MID_LEVEL_CTRL_TASK_H_ */
