/*
 * Angel_Kit_D10.h
 *
 *  Created on: Jul 27, 2023
 *      Author: AngelRobotics HW
 */

#ifndef HW_PERIPHERALS_ANGEL_KIT_D10_INC_ANGEL_KIT_D10_H_
#define HW_PERIPHERALS_ANGEL_KIT_D10_INC_ANGEL_KIT_D10_H_

#include "module.h"

#if defined(WALKON5_CM_ENABLED) || defined(L30_CM_ENABLED) || defined(SUIT_MINICM_ENABLED)

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "data_object_dictionaries.h"

#include "error_dictionary.h"

/* SDO STATUS */
#define D10_SDO_IDLE  2
#define D10_SDO_REQU  1
#define D10_SDO_SUCC  0
#define D10_SDO_FAIL -1

#define D10_STATUS_SUCCESS		 	0
#define D10_STATUS_SDO_SET_FAIL		-1
#define D10_STATUS_PDO_SET_FAIL		-2
#define D10_STATUS_RECV_FAIL			-3
#define D10_STATUS_RECV_INVALID_MSG	-4
#define D10_STATUS_RECV_SDO_FAIL		-5
#define D10_STATUS_RECV_PDO_FAIL		-6

typedef int32_t D10_STATUS;
typedef int32_t D10_SDO_Info;

typedef int (*D10_RecvMsgHdlr) (uint16_t, uint8_t*);		//wasp id, rx Buffer

/* Mode of Operation*/
typedef enum _ModeOfOperation{
	e_Current_Mode,
	e_Velocity_Mode,
	e_Position_Mode
}ModeOfOperation;

/* Input Method */
typedef enum _InputMethod{
	e_Communication,
	e_PWM,
	e_Analog,
	e_GUI
}InputMethod;

typedef struct _CurrentCtrlVerify {

   float mag;
   float freq;
   uint8_t type;

}CurrentCtrlVerify;


typedef struct _InputInfo{

	InputMethod	input_method;
	float	input_min;
	float	input_max;
	float 	output_min;
	float 	output_max;

} InputInfo;

/*
  ___ ___   ___    __       ___ ___   ___
 / __|   \ / _ \  / _|___  | _ \   \ / _ \
 \__ \ |) | (_) | > _|_ _| |  _/ |) | (_) |
 |___/___/ \___/  \_____|  |_| |___/ \___/

*/

typedef struct SDOparam
{
	SDOStatus sdo_status;
	uint8_t num_of_data;
	void* data;
}SDOparam;

typedef void (*D10_SDOcallback) (SDOparam*, SDOparam*);

typedef struct SDOUnit
{
	uint8_t	task_id;
	uint8_t	sdo_id;
	SDOparam param;

	D10_SDOcallback callback;
}SDOUnit;

typedef struct PDOUnit
{
	uint8_t	task_id;
	uint8_t	pdo_id;
	void*	addr;
}PDOUnit;


typedef struct SDOmsg
{
	uint8_t n_sdo;
	uint8_t msg_length;
	uint8_t txBuf[64];
}SDOmsg;

typedef struct PDOmsg
{
	uint8_t n_pdo;
	uint8_t msg_length;
	uint8_t txBuf[64];
}PDOmsg;

typedef struct SYNCmsg
{
	uint8_t n_node;
	uint8_t node_msg_length[3];
	uint8_t node_ids[3];
	PDOmsg pdo_msg[3];
}SYNCmsg;

/*
   ___  _     _        _
  / _ \| |__ (_)___ __| |_
 | (_) | '_ \| / -_) _|  _|
  \___/|_.__// \___\__|\__|
           |__/
*/


typedef struct Task_Object
{
	uint8_t task_id;
	uint8_t task_state;
	ObjectDictionaryRoutineOnoff routines[ROUTINE_MAX_NUM];

	void* sdos_addrs[SDO_MAX_NUM];
	void* pdos_addrs[PDO_MAX_NUM];

}Task_Object;


typedef struct _D10Object
{
	uint8_t  node_id;
	uint16_t error_code;
	uint8_t  n_task;

	Task_Object tasks[TASK_NUM];

}D10Object;



/*
  ___      _    _ _      ___             _   _
 | _ \_  _| |__| (_)__  | __|  _ _ _  __| |_(_)___ _ _
 |  _/ || | '_ \ | / _| | _| || | ' \/ _|  _| / _ \ ' \
 |_|  \_,_|_.__/_|_\__| |_| \_,_|_||_\__|\__|_\___/_||_|

*/

void Init_D10(D10Object* obj, uint8_t nodeId);

void Set_D10_SDO_Addr(D10Object* obj, uint8_t taskID, uint8_t sdoID, void* addr);
void* Get_D10_SDO_Addr(D10Object* obj, uint8_t taskID, uint8_t sdoID);

void Set_D10_PDO_Addr(D10Object* obj, uint8_t taskID, uint8_t pdoID, void* addr);
void* Get_D10_PDO_Addr(D10Object* obj, uint8_t taskID, uint8_t pdoID);

void Set_D10_RoutineOnOff(D10Object* obj, uint8_t taskID, uint8_t rtID, ObjectDictionaryRoutineOnoff OnOff);
ObjectDictionaryRoutineOnoff Get_D10_RoutineOnOff(D10Object* obj, uint8_t taskID, uint8_t rtID);

SDOUnit Create_D10_SDOUnit(D10Object *obj, uint8_t taskId, uint8_t sdoId, SDOStatus sdoStatus, uint8_t numOfData);
D10_STATUS Append_D10_SDO(SDOUnit* sdoUnit, SDOmsg* sdoMsg);
void Clear_D10_SDO(SDOmsg* sdoMsg);

PDOUnit Create_D10_PDOUnit(uint8_t taskId, uint8_t pdoId, void* addr);
D10_STATUS Append_D10_PDO(PDOUnit* pdoUnit, PDOmsg* pdoMsg);
void Clear_D10_PDO(PDOmsg* pdoMsg);

int Unpack_D10_SDO(D10Object* obj, uint8_t* byte_arr);
int Unpack_D10_PDO(D10Object* obj, uint8_t* byte_arr);
void D10_Recv_EMCY(uint32_t *t_err_code, uint8_t *t_byte_arr);

#endif /* CM Modules */

#endif /* HW_PERIPHERALS_ANGEL_KIT_D10_INC_ANGEL_KIT_D10_H_ */
