

#include "object_dictionaries.h"

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

SDOinfo SDO_Table[TASK_NUM][SDO_MAX_NUM] = {0};
PDOinfo PDO_Table[TASK_NUM][PDO_MAX_NUM] = {0};


/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */




/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */




/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

/* ------------------- ASSEMBLE ------------------- */
uint8_t Convert_DataSize(int t_data_type)
{
	switch(t_data_type) {
		case UINT8_T: 		return 1;		break;
		case UINT16_T:		return 2;		break;
		case UINT32_T:		return 4;		break;
		case INT8_T:		return 1;		break;
		case INT16_T:		return 2;		break;
		case INT32_T:		return 4;		break;
		case FLOAT32_T:		return 4;		break;
		case FLOAT64_T:		return 8;		break;
		case STRING10:		return 32;		break;
		default:			return 0;		break;
	}
}

static void Assemble_SDO(SDOinfo* t_do_addr, uint8_t t_data_type)
{
	SDOinfo temp = t_data_type;
	memcpy(t_do_addr, &temp, sizeof(SDOinfo));
}

static void Assemble_PDO(PDOinfo* t_do_addr, uint8_t t_data_type, uint8_t t_num_of_data)
{
	PDOinfo temp = {t_data_type, t_num_of_data};
	memcpy(t_do_addr, &temp, sizeof(PDOinfo));
}

/* ------------------- SDO TABLE ------------------- */
void Create_SDOTable_ObjDictionary(void)
{
	//********************************************************************************//
	//						|	  Task_ID	  |			SDO_ID			| DATA_TYPE | //
	//********************************************************************************//
	/* Low Level Ctrl Task*/
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_GET_STATE],  	    			UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_SET_STATE],  	    			UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_GET_ROUTINE],  				UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_SET_ROUTINE],  				UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_NAME],  						STRING10);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_POLE_PAIR],  					UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_ENCODER_RESOLUTION],  			UINT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_GEAR_RATIO],  					FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_TORQUE_CONSTANT],  			FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_VELOCITY_CONSTANT],  			FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_PEAK_CURRENT_LIMIT],  			FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_CONTINUOUS_CURRENT_LIMIT],  	FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_MAX_VELOCITY],		  			FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_COMMUTATION_DUTY],  			UINT16_T);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_USER_DIRECTION],  				FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_ELEC_SYSTEM_ID_MAG],  			FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_TERMINAL_RESISTANCE],  		FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_TERMINAL_INDUCTANCE],  		FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_BEMF_ID_VELOCITY],  			FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_BEMF_ID_GAIN_PCTG],  			UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_CURRENT_CTRL_BW_RAD],  		FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_INERTIA],  					FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_DAMPING_COEF],  				FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_MECH_MODEL_A],  				FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_MECH_MODEL_B],  				FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_FRICTION_ID_INFO],  			FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_FRICTION_LUT_INFO],  			FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_SET_COMMUTATION_SENSOR],    	UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_SET_POS_FEEDBACK_SENSOR],  	UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_SET_E_ANGLE_HOMING_SENSOR],  	UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_SET_M_ANGLE_HOMING_SENSOR],  	UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_SET_SENSOR_USAGE],  	        UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_SET_AUX_INPUT],				FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_MD_VERSION],  	                UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_INC_ENCODER_PRESCALER],  	    UINT16_T);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_BEMF_COMP_GAIN],  	            FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_FRICTION_COMP_GAIN_SS],        FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_CURRENT_KF_ONOFF],             UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_CURRENT_KF_A],                 FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_CURRENT_KF_B],                 FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_CURRENT_KF_C],                 FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_CURRENT_KF_Q],                 FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_LOWLEVEL] [SDO_ID_LOWLEVEL_CURRENT_KF_R],                 FLOAT32_T);


	/* Mid Level Ctrl Task*/
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_GET_STATE],  					UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_SET_STATE],  					UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_GET_ROUTINE], 					UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_SET_ROUTINE], 					UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_IRC_NUMERATOR_LENGTH],	 		UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_IRC_DENOMINATOR_LENGTH],		UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_IRC_NUMERATOR],	  			FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_IRC_DENOMINATOR],	  			FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_IRC_SATURATION],	  			FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_IMP_VIRTUAL_STIFFNESS],	  	FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_IMP_VIRTUAL_DAMPER],	  		FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_VELOCITY_PERIODIC_SIG_INFO],	FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_POSITION_PERIODIC_SIG_INFO],	FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_DOB_Q_BW],						FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_DOB_GQ_NUM],					FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_DOB_GQ_DEN],					FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_DOB_Q_NUM],					FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_DOB_Q_DEN],					FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_DOB_SATURATION],				FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_VELOCITY_CTRL_BW],				FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_VELOCITY_CTRL_P_GAIN],			FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_VELOCITY_CTRL_I_GAIN],			FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_POSITION_CTRL_INPUT_PENALTY],	FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_POSITION_CTRL_P_GAIN],			FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_POSITION_CTRL_D_GAIN],			FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_MID_CTRL_SATURATION],			FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_INCENCODER_SET_OFFSET],		UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_ABSENCODER1_SET_OFFSET],		UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_ABSENCODER1_CHANGE_DIRECTION],	UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_ABSENCODER2_SET_OFFSET],		UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_ABSENCODER2_CHANGE_DIRECTION],	UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_VSD_STIFFNESS],				FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_VSD_DAMPER],					FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_VSD_DAMPED_RANGE],				FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_VSD_STIFF_RANGE],				FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_SET_VSD_UPPER_LIMIT],			UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_SET_VSD_LOWER_LIMIT],			UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_VSD_SATURATION],				FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_FEEDFORWARD_NUM],				FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_FEEDFORWARD_DEN],				FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_VELOCITY_ESTIMATOR],			UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_VELOCITY_ESTIMATOR_LEAD_LAG],	FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_ENCODER_RESOLUTION],			UINT16_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_SYSTEM_ID_SBS_INFO],			FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_SYSTEM_ID_VERIFICATION_MAG],	FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_C_VECTOR_FF_GAIN],             UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_C_VECTOR_PD_GAIN],             UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_C_VECTOR_IC_GAIN],             UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_C_VECTOR_DOB_GAIN],            UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_C_VECTOR_IRC_GAIN],            UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_C_VECTOR_FC_GAIN],             UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_P_VECTOR_YD],	                INT16_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_P_VECTOR_L],			        UINT16_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_P_VECTOR_S0],			        UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_P_VECTOR_SD],	                UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_F_VECTOR_MODE_IDX],	        UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_F_VECTOR_TMAX],			    INT16_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_F_VECTOR_DELAY],			    UINT16_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_I_VECTOR_EPSILON],			    UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_I_VECTOR_KP],	                UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_I_VECTOR_KD],	                UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_I_VECTOR_LAMBDA],			    UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_I_VECTOR_DURATION],			UINT16_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_I_VECTOR_KP_MAX],			    FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_I_VECTOR_KD_MAX],			    FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_DESIRED_MECH_ANGLE],	        FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_ABSENCODER1_LOCATION],			UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_ABSENCODER2_LOCATION],	        UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_ABSENCODER1_MODULE_ID],		UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_ABSENCODER2_MODULE_ID],	    UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_GAIN_TRANSITION_TIME],	        FLOAT32_T);

	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_FREEZE],                       UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_POSITION_REFERENCE_OFFSET],    FLOAT32_T);

	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_TRAPE_ID_INFO],                FLOAT32_T);

	Assemble_SDO( &SDO_Table [TASK_ID_MIDLEVEL] [SDO_ID_MIDLEVEL_FRICTION_COMP_PARAM],          FLOAT32_T);


	/* MSG Handler Task */
	Assemble_SDO( &SDO_Table [TASK_ID_MSG] [SDO_ID_MSG_GET_STATE],  	    UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MSG] [SDO_ID_MSG_SET_STATE],  	    UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MSG] [SDO_ID_MSG_GET_ROUTINE],  		UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MSG] [SDO_ID_MSG_SET_ROUTINE],  		UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MSG] [SDO_ID_MSG_PDO_LIST], 			UINT16_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MSG] [SDO_ID_MSG_MS_ENUM],  			UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MSG] [SDO_ID_MSG_GUI_COMM_ONOFF],  	UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MSG] [SDO_ID_MSG_GUI_COMM_COMMAND],  	UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MSG] [PDO_ID_WS5_MD_TRANSMIT_16BIT],  INT16_T);
	Assemble_SDO( &SDO_Table [TASK_ID_MSG] [PDO_ID_WS5_MD_TRANSMIT_32BIT],  UINT32_T);

	/* WIDM Task*/
	Assemble_SDO( &SDO_Table [TASK_ID_WIDM] [SDO_ID_WIDM_GET_STATE],  		UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_WIDM] [SDO_ID_WIDM_SET_STATE],  		UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_WIDM] [SDO_ID_WIDM_GET_ROUTINE],		UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_WIDM] [SDO_ID_WIDM_SET_ROUTINE],		UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_WIDM] [SDO_ID_WIDM_FOR_TEST],			UINT16_T);
	Assemble_SDO( &SDO_Table [TASK_ID_WIDM] [SDO_ID_WIDM_MAG_INVA],			FLOAT32_T);
	Assemble_SDO( &SDO_Table [TASK_ID_WIDM] [SDO_ID_WIDM_MAG_IRON_ERROR],  	FLOAT32_T);

	/*SYSMNGT Task*/
	Assemble_SDO( &SDO_Table [TASK_ID_SYSMNGT] [SDO_ID_SYSMNGT_GET_STATE],  	UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_SYSMNGT] [SDO_ID_SYSMNGT_SET_STATE],  	UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_SYSMNGT] [SDO_ID_SYSMNGT_GET_ROUTINE],  	UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_SYSMNGT] [SDO_ID_SYSMNGT_SET_ROUTINE],  	UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_SYSMNGT] [SDO_ID_SYSMNGT_FOR_TEST],  		UINT16_T);

	/*EXTDEV Task*/
	Assemble_SDO( &SDO_Table [TASK_ID_EXTDEV] [SDO_ID_EXTDEV_GET_STATE],  		UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_EXTDEV] [SDO_ID_EXTDEV_SET_STATE],  		UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_EXTDEV] [SDO_ID_EXTDEV_GET_ROUTINE],  	UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_EXTDEV] [SDO_ID_EXTDEV_SET_ROUTINE],  	UINT8_T);

	/*RISK Task*/
	Assemble_SDO( &SDO_Table [TASK_ID_RISK] [SDO_ID_RISK_GET_STATE],  		UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_RISK] [SDO_ID_RISK_SET_STATE],  		UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_RISK] [SDO_ID_RISK_GET_ROUTINE],  	UINT8_T);
	Assemble_SDO( &SDO_Table [TASK_ID_RISK] [SDO_ID_RISK_SET_ROUTINE],  	UINT8_T);
}

/* ------------------- PDO TABLE ------------------- */
void Create_PDOTable_ObjDictionary(void)
{
	//**************************************************************************************************//
	//							   |	  Task_ID	  |				PDO_ID				| DATA_TYPE | #_of_DATA //
	//**************************************************************************************************//
	/* LOWLEVEL Task */
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_LOWLEVEL] [PDO_ID_LOWLEVEL_3PHASES_CURRENT_RAW],  		INT32_T, 	 	3);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_LOWLEVEL] [PDO_ID_LOWLEVEL_3PHASES_CURRENT_KF], 			FLOAT32_T,  	3);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_LOWLEVEL] [PDO_ID_LOWLEVEL_3PHASES_VOLTAGE_RAW],  		INT32_T, 	 	3);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_LOWLEVEL] [PDO_ID_LOWLEVEL_3PHASES_VOLTAGE_KF], 			FLOAT32_T,  	3);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_LOWLEVEL] [PDO_ID_LOWLEVEL_POSITION],  					FLOAT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_LOWLEVEL] [PDO_ID_LOWLEVEL_VELOCITY],  					INT32_T,  		2);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_LOWLEVEL] [PDO_ID_LOWLEVEL_CLARKE_OUT],  				INT32_T,  		2);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_LOWLEVEL] [PDO_ID_LOWLEVEL_PARK_OUT],  					FLOAT32_T,  	3);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_LOWLEVEL] [PDO_ID_LOWLEVEL_VOLTAGE_IN],	    			FLOAT32_T,  	3);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_LOWLEVEL] [PDO_ID_LOWLEVEL_ELEC_ANGLE],	    			UINT16_T,  		1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_LOWLEVEL] [PDO_ID_LOWLEVEL_PRBS_DATA],	    			FLOAT32_T, 		2);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_LOWLEVEL] [PDO_ID_LOWLEVEL_TOTAL_CURRENT_INPUT],			FLOAT32_T, 		1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_LOWLEVEL] [PDO_ID_LOWLEVEL_CURRENT_OUTPUT],  			FLOAT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_LOWLEVEL] [PDO_ID_LOWLEVEL_AUXILIARY_INPUT],  			FLOAT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_LOWLEVEL] [PDO_ID_LOWLEVEL_LOW_ID_PROCESS_INPUT],		FLOAT32_T, 		1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_LOWLEVEL] [PDO_ID_LOWLEVEL_FRICTION_COMPENSATOR_INPUT],  FLOAT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_LOWLEVEL] [PDO_ID_LOWLEVEL_MID_ID_PROCESS_INPUT],		FLOAT32_T, 		1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_LOWLEVEL] [PDO_ID_LOWLEVEL_IRC_INPUT],					FLOAT32_T, 		1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_LOWLEVEL] [PDO_ID_LOWLEVEL_MID_CTRL_INPUT],  			FLOAT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_LOWLEVEL] [PDO_ID_LOWLEVEL_ANALYZER_INPUT],				FLOAT32_T, 		1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_LOWLEVEL] [PDO_ID_LOWLEVEL_COMMUTATION_STEP],			UINT8_T, 		1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_LOWLEVEL] [PDO_ID_LOWLEVEL_STATOR_TEMP],			        FLOAT32_T, 		1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_LOWLEVEL] [PDO_ID_LOWLEVEL_ADV_ELEC_ANGLE],			    INT16_T, 		1);


	/* Mid Level Ctrl Task */
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_MIDLEVEL] [PDO_ID_MIDLEVEL_LOOP_CNT],			UINT32_T, 		1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_MIDLEVEL] [PDO_ID_MIDLEVEL_REF_POSITION],  		FLOAT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_MIDLEVEL] [PDO_ID_MIDLEVEL_REF_VELOCITY],  		FLOAT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_MIDLEVEL] [PDO_ID_MIDLEVEL_ACTUAL_POSITION], 	FLOAT32_T, 		1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_MIDLEVEL] [PDO_ID_MIDLEVEL_ACTUAL_VELOCITY_RAW], FLOAT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_MIDLEVEL] [PDO_ID_MIDLEVEL_SYSTEM_ID_SBS_FREQ],	FLOAT32_T, 		1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_MIDLEVEL] [PDO_ID_MIDLEVEL_IMP_CTRL_INPUT], 				FLOAT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_MIDLEVEL] [PDO_ID_MIDLEVEL_POS_PID_CTRL_INPUT], 			FLOAT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_MIDLEVEL] [PDO_ID_MIDLEVEL_VEL_PID_CTRL_INPUT],			FLOAT32_T, 		1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_MIDLEVEL] [PDO_ID_MIDLEVEL_VSD_INPUT],					FLOAT32_T, 		1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_MIDLEVEL] [PDO_ID_MIDLEVEL_UNIT_TRAJECTORY_BUFF_COUNT],	FLOAT32_T, 		1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_MIDLEVEL] [PDO_ID_MIDLEVEL_F_VECTOR_INPUT],		        FLOAT32_T, 		1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_MIDLEVEL] [PDO_ID_MIDLEVEL_ABSENCODER1_POSITION],		FLOAT32_T, 		1);
//	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_MIDLEVEL] [PDO_ID_MIDLEVEL_ABSENCODER2_POSITION],		FLOAT32_T, 		1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_MIDLEVEL] [PDO_ID_MIDLEVEL_DOB_DISTURABNCE],				FLOAT32_T, 		1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_MIDLEVEL] [PDO_ID_MIDLEVEL_DOB_INPUT],					FLOAT32_T, 		1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_MIDLEVEL] [PDO_ID_MIDLEVEL_FF_INPUT],					FLOAT32_T, 		1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_MIDLEVEL] [PDO_ID_MIDLEVEL_VELOCITY_ESTIMATED],			FLOAT32_T, 		1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_MIDLEVEL] [PDO_ID_MIDLEVEL_RISK],			            UINT32_T, 		1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_MIDLEVEL] [PDO_ID_MIDLEVEL_TRAPE_ID_DONE],			    UINT8_T, 		1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_MIDLEVEL] [PDO_ID_MIDLEVEL_SBS_ID_DONE],  			    UINT8_T, 		1);

	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_MIDLEVEL] [PDO_ID_MIDLEVEL_PMMG1],  			            FLOAT32_T, 		1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_MIDLEVEL] [PDO_ID_MIDLEVEL_PMMG2],  			            FLOAT32_T, 		1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_MIDLEVEL] [PDO_ID_MIDLEVEL_PMMG3],  			            FLOAT32_T, 		1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_MIDLEVEL] [PDO_ID_MIDLEVEL_PMMG4],  			            FLOAT32_T, 		1);

	/* MSG Handler Task */
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_MSG] [PDO_ID_MSG_TEST1],  			INT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_MSG] [PDO_ID_MSG_TEST2],  			INT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_MSG] [PDO_ID_MSG_TEST3],  			INT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_MSG] [PDO_ID_MSG_TEST4],  			INT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_MSG] [PDO_ID_MSG_TEST5],  			INT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_MSG] [PDO_ID_MSG_TEST6],  			INT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_MSG] [PDO_ID_MSG_TEST7],  			INT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_MSG] [PDO_ID_MSG_TEST8],  			INT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_MSG] [PDO_ID_MSG_TEST9],  			INT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_MSG] [PDO_ID_MSG_TEST10],  			INT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_MSG] [PDO_ID_WS5_MD_TRANSMIT_16BIT], UINT16_T,  	7);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_MSG] [PDO_ID_WS5_MD_TRANSMIT_32BIT], UINT32_T,  	1);

	/* WIDM Task */
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_WIDM] [PDO_ID_WIDM_ACC_X],  			FLOAT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_WIDM] [PDO_ID_WIDM_ACC_Y],  			FLOAT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_WIDM] [PDO_ID_WIDM_ACC_Z],  			FLOAT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_WIDM] [PDO_ID_WIDM_GYR_X],  			FLOAT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_WIDM] [PDO_ID_WIDM_GYR_Y],  			FLOAT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_WIDM] [PDO_ID_WIDM_GYR_Z],  			FLOAT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_WIDM] [PDO_ID_WIDM_MAG_X],  			FLOAT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_WIDM] [PDO_ID_WIDM_MAG_Y],  			FLOAT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_WIDM] [PDO_ID_WIDM_MAG_Z],  			FLOAT32_T,  	1);

	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_WIDM] [PDO_ID_WIDM_ACC_XYZ],  		FLOAT32_T,  	3);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_WIDM] [PDO_ID_WIDM_GYR_XYZ],  		FLOAT32_T,  	3);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_WIDM] [PDO_ID_WIDM_MAG_XYZ],  		FLOAT32_T,  	3);

	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_WIDM] [PDO_ID_WIDM_ACC_GYR_XYZ],  	FLOAT32_T,  	6);
//	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_WIDM] [PDO_ID_WIDM_QUATERNION],		INT16_T,  	    4);

	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_WIDM] [PDO_ID_WIDM_WALKING_STATE],  	FLOAT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_WIDM] [PDO_ID_WIDM_GAIT_PHASE],		FLOAT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_WIDM] [PDO_ID_WIDM_GAIT_PERIOD],  	FLOAT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_WIDM] [PDO_ID_WIDM_TENSION_D],  		FLOAT32_T,  	1);

	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_WIDM] [PDO_ID_WIDM_DEG_ACC],  		FLOAT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_WIDM] [PDO_ID_WIDM_DEG_GYR],  		FLOAT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_WIDM] [PDO_ID_WIDM_DEG_ACC_FILTERED], FLOAT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_WIDM] [PDO_ID_WIDM_DEG_GYR_FILTERED], FLOAT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_WIDM] [PDO_ID_WIDM_DEG_TVCF],		FLOAT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_WIDM] [PDO_ID_WIDM_TVCF_ROLL],		FLOAT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_WIDM] [PDO_ID_WIDM_TVCF_PITCH],		FLOAT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_WIDM] [PDO_ID_WIDM_TVCF_YAW],		FLOAT32_T,  	1);

	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_WIDM] [PDO_ID_WIDM_SO3_GYR_FILTERED_X], FLOAT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_WIDM] [PDO_ID_WIDM_SO3_GYR_FILTERED_Y], FLOAT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_WIDM] [PDO_ID_WIDM_SO3_GYR_FILTERED_Z], FLOAT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_WIDM] [PDO_ID_WIDM_SO3_ACC_FILTERED_X], FLOAT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_WIDM] [PDO_ID_WIDM_SO3_ACC_FILTERED_Y], FLOAT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_WIDM] [PDO_ID_WIDM_SO3_ACC_FILTERED_Z], FLOAT32_T,  	1);

	/* System Ctrl Task */
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_SYSMNGT] [PDO_ID_SYSTEM_VOLT],		FLOAT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_SYSMNGT] [PDO_ID_SYSTEM_CURR],		FLOAT32_T,  	1);
//	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_SYSMNGT] [PDO_ID_SYSTEM_CURR],		INT16_T,  	    1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_SYSMNGT] [PDO_ID_SYSTEM_TEMP],		FLOAT32_T,  	1);
	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_SYSMNGT] [PDO_ID_SYSTEM_PCTG],		FLOAT32_T,  	1);

	/* Ext Dev Ctrl Task */
//	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_EXTDEV] [PDO_ID_EXTDEV_FSR],  				FLOAT32_T,  	1);
//	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_EXTDEV] [PDO_ID_EXTDEV_LP],  				FLOAT32_T,  	1);
//	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_EXTDEV] [PDO_ID_EXTDEV_DC_LENGTH_REF],		FLOAT32_T,  	1);
//	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_EXTDEV] [PDO_ID_EXTDEV_DC_DIRECTION_CMD],  	UINT8_T,  		1);
//	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_EXTDEV] [PDO_ID_EXTDEV_DC_LENGTH_ACT],  		FLOAT32_T,  	1);
//	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_EXTDEV] [PDO_ID_EXTDEV_DC_DIRECTION_ACT],  	UINT8_T,  		1);
//	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_EXTDEV] [PDO_ID_EXTDEV_DC_BUTTON_STATE],  	UINT8_T,  		1);
//	Assemble_PDO( (PDOinfo*)PDO_Table [TASK_ID_EXTDEV] [PDO_ID_EXTDEV_NTC_MOTOR_TEMP],  	FLOAT32_T,		1);
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

