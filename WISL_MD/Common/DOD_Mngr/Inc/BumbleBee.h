

#ifndef BUMBLEBEE_H_
#define BUMBLEBEE_H_

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "object_dictionaries.h"
#include "error_dictionary.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

/* SDO STATUS */
#define BUMBLEBEE_SDO_IDLE  2
#define BUMBLEBEE_SDO_REQU  1
#define BUMBLEBEE_SDO_SUCC  0
#define BUMBLEBEE_SDO_FAIL -1

#define BUMBLEBEE_STATUS_SUCCESS		 	0
#define BUMBLEBEE_STATUS_SDO_SET_FAIL		-1
#define BUMBLEBEE_STATUS_PDO_SET_FAIL		-2
#define BUMBLEBEE_STATUS_RECV_FAIL			-3
#define BUMBLEBEE_STATUS_RECV_INVALID_MSG	-4
#define BUMBLEBEE_STATUS_RECV_SDO_FAIL		-5
#define BUMBLEBEE_STATUS_RECV_PDO_FAIL		-6

#define D10_BUFFER_ARRAY_MAX_SIZE		10000
#define D10_BUFFER_ARRAY_SUB_SIZE		2000

#define D10_TRAJECTORY_TOTAL_LENGTH		10000
#define D10_TRAJECTORY_ELEMENT_NUMBER	10


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef int32_t BumbleBee_status;
typedef int32_t BumbleBee_SDO_info;

typedef int (*BumbleBee_recv_msghdlr)(uint16_t, uint8_t*);		// wasp id, rx Buffer

/*MD Version*/
typedef enum _MDVersion {
	e_None,
	e_R06,
	e_R07,
	e_R08,
	e_R08H,
} MDVersion;

/* Mode of Operation*/
typedef enum _ModeOfOperation {
	e_Current_Mode,
	e_Velocity_Mode,
	e_Position_Mode
} ModeOfOperation;

/* Input Method */
typedef enum _InputMethod {
	e_Communication,
	e_PWM,
	e_Analog,
	e_GUI
} InputMethod;

typedef struct _CurrentCtrlVerify {
   float mag;
   float freq;
   uint8_t type;
} CurrentCtrlVerify;

typedef struct _InputInfo {
	InputMethod	input_method;
	float	input_min;
	float	input_max;
	float 	output_min;
	float 	output_max;
} InputInfo;

typedef struct _SharedArrayBuffer {
	float buf_1st[D10_BUFFER_ARRAY_MAX_SIZE];
	float buf_2nd[D10_BUFFER_ARRAY_MAX_SIZE];
	float buf_3rd[D10_BUFFER_ARRAY_MAX_SIZE];
	float buf_4th[D10_BUFFER_ARRAY_MAX_SIZE];
} SharedArrayBuffer;

typedef struct _TrajectoryBuffer {
	float buff[D10_TRAJECTORY_TOTAL_LENGTH];
	uint16_t frame_idx;
} TrajectoryBuffer;

typedef struct _SDOParam {
	SDOStatus sdo_status;
	void* data;
	uint8_t num_of_data;
} SDOParam;

typedef void (*BumbelBee_SDOcallback)(SDOParam*, SDOParam*);

typedef struct _SDOUnit {
	SDOParam param;
	BumbelBee_SDOcallback callback;
	uint8_t	task_id;
	uint8_t	sdo_id;
} SDOUnit;

typedef struct _PDOUnit {
	void*	addr;
	uint8_t	task_id;
	uint8_t	pdo_id;
} PDOUnit;

typedef struct _SDOMsg {
	uint8_t txBuf[64];
	uint8_t n_sdo;
	uint8_t msg_length;
} SDOMsg;

typedef struct _PDOMsg {
	uint8_t txBuf[64];
	uint8_t n_pdo;
	uint8_t msg_length;
} PDOMsg;

typedef struct _TaskObject {
	void* sdos_addrs[SDO_MAX_NUM];
	void* pdos_addrs[PDO_MAX_NUM];

	ObjectDictionaryRoutineOnoff routines[ROUTINE_MAX_NUM];

	uint8_t task_id;
	uint8_t task_state;
} TaskObject;

typedef struct _BumbleBeeObject {
	TaskObject tasks[TASK_NUM];
	uint16_t error_code;
	uint8_t  node_id;
	uint8_t  n_task;
} BumbleBeeObject;


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */




/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void Init_BumbleBee(BumbleBeeObject* t_obj, uint8_t t_nodeId);

void Set_BumbleBee_SDOAddr(BumbleBeeObject* t_obj, uint8_t t_taskID, uint8_t t_sdoID, void* t_addr);
void* Get_BumbleBee_SDOAddr(BumbleBeeObject* t_obj, uint8_t t_taskID, uint8_t t_sdoID);

void Set_BumbleBee_PDOAddr(BumbleBeeObject* t_obj, uint8_t t_taskID, uint8_t t_pdoID, void* t_addr);
void* Get_BumbleBee_PDOAddr(BumbleBeeObject* t_obj, uint8_t t_taskID, uint8_t t_pdoID);

void Set_BumbleBee_RoutineOnOff(BumbleBeeObject* t_obj, ObjectDictionaryRoutineOnoff t_OnOff, uint8_t t_taskID, uint8_t t_rtID );
ObjectDictionaryRoutineOnoff Get_BumbleBee_RoutineOnOff(BumbleBeeObject* t_obj, uint8_t t_taskID, uint8_t t_rtID);

SDOUnit Create_BumbleBee_SDOUnit(BumbleBeeObject *t_obj,  SDOStatus t_sdoStatus, uint8_t t_taskID, uint8_t t_sdoID, uint8_t t_numOfData);
BumbleBee_status Append_BumbleBee_SDO(SDOUnit* t_sdoUnit, SDOMsg* t_sdoMsg);
void Clear_BumbleBee_SDO(SDOMsg* t_sdoMsg);

PDOUnit Create_BumbleBee_PDOUnit(uint8_t t_taskID, uint8_t t_pdoID, void* t_addr);
BumbleBee_status Append_BumbleBee_PDO(PDOMsg* t_pdoMsg, PDOUnit* t_pdoUnit);
void Clear_BumbleBee_PDO(PDOMsg* t_pdoMsg);

int Unpack_BumbleBee_SDO(BumbleBeeObject* t_obj, uint8_t* t_byte_arr);
int Unpack_BumbleBee_PDO(BumbleBeeObject* t_obj, uint8_t* t_byte_arr);


#endif /* BUMBLEBEE_H_ */
