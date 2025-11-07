

#include "Angel_Kit_D10.h"

#if defined(WALKON5_CM_ENABLED) || defined(L30_CM_ENABLED) || defined(SUIT_MINICM_ENABLED)

/*
  ___     _     _      _         ___             _   _
 | _ \_ _(_)_ _(_)__ _| |_ ___  | __|  _ _ _  __| |_(_)___ _ _
 |  _/ '_| \ V / / _` |  _/ -_) | _| || | ' \/ _|  _| / _ \ ' \
 |_| |_| |_|\_/|_\__,_|\__\___| |_| \_,_|_||_\__|\__|_\___/_||_|

*/

static PDOinfo* Get_D10_PDOInfo(uint8_t taskID, uint8_t pdoID)
{
	return &PDO_Table[taskID][pdoID];
}

static SDOinfo Get_D10_SDOInfo(uint8_t taskID, uint8_t sdoID)
{
	return Convert_DataSize(SDO_Table[taskID][sdoID]);
}

static void D10_TaskAllocation(Task_Object* task, uint8_t taskID)
{
	task->task_id = taskID;
	task->task_state = STATE_IDLE;
}

static int D10_CheckSDO(uint8_t taskID, uint8_t sdoID)
{
	/* Check Error */
	switch(taskID){
	case TASK_ID_LOWLEVEL:
		if(sdoID >= SDO_ID_LOWLEVEL_NUM)	return D10_STATUS_SDO_SET_FAIL;
		break;
	case TASK_ID_MIDLEVEL:
		if(sdoID >= SDO_ID_MIDLEVEL_NUM)	return D10_STATUS_SDO_SET_FAIL;
		break;
	case TASK_ID_MSG:
		if(sdoID >= SDO_ID_MSG_NUM)			return D10_STATUS_SDO_SET_FAIL;
		break;
	case TASK_ID_WIDM:
		if(sdoID >= SDO_ID_WIDM_NUM)		return D10_STATUS_SDO_SET_FAIL;
		break;
	case TASK_ID_SYSMNGT:
		if(sdoID >= SDO_ID_SYSMNGT_NUM)		return D10_STATUS_SDO_SET_FAIL;
		break;
	case TASK_ID_EXTDEV:
		if(sdoID >= SDO_ID_EXTDEV_NUM)		return D10_STATUS_SDO_SET_FAIL;
		break;
	default:
		return D10_STATUS_SDO_SET_FAIL;
		break;
	}

	return D10_STATUS_SUCCESS;
}

static int D10_CheckPDO(uint8_t taskID, uint8_t pdoID)
{
	switch(taskID){
	case TASK_ID_LOWLEVEL:
		if(pdoID >= PDO_ID_LOWLEVEL_NUM)	return D10_STATUS_PDO_SET_FAIL;
		break;
	case TASK_ID_MIDLEVEL:
		if(pdoID >= PDO_ID_MIDLEVEL_NUM)	return D10_STATUS_PDO_SET_FAIL;
		break;
	case TASK_ID_MSG:
		if(pdoID >= PDO_ID_MSG_NUM)			return D10_STATUS_PDO_SET_FAIL;
		break;
	case TASK_ID_WIDM:
		if(pdoID >= PDO_ID_WIDM_NUM)		return D10_STATUS_PDO_SET_FAIL;
		break;
	case TASK_ID_SYSMNGT:
		if(pdoID >= PDO_ID_SYSMNGT_NUM)		return D10_STATUS_PDO_SET_FAIL;
		break;
	case TASK_ID_EXTDEV:
		if(pdoID >= PDO_ID_EXTDEV_NUM)		return D10_STATUS_PDO_SET_FAIL;
		break;
	default:
		return D10_STATUS_PDO_SET_FAIL;
		break;
	}

	return D10_STATUS_SUCCESS;
}

/*
  ___ ___   ___     ___  _     _        _
 / __|   \ / _ \   / _ \| |__ (_)___ __| |_
 \__ \ |) | (_) | | (_) | '_ \| / -_) _|  _|
 |___/___/ \___/   \___/|_.__// \___\__|\__|
                            |__/
*/

SDOUnit Create_D10_SDOUnit(D10Object *obj, uint8_t taskId, uint8_t sdoId, SDOStatus sdoStatus, uint8_t numOfData)
{
	SDOUnit sdo_unit;

	sdo_unit.task_id = taskId;
	sdo_unit.sdo_id = sdoId;
	sdo_unit.param.sdo_status = sdoStatus;
	sdo_unit.param.num_of_data = numOfData;
	sdo_unit.param.data = obj->tasks[taskId].sdos_addrs[sdoId];

	return sdo_unit;
}

D10_STATUS Append_D10_SDO(SDOUnit* sdoUnit, SDOmsg* sdoMsg)
{
	uint8_t cursor;

	/* Validation Check */
	if(D10_CheckSDO(sdoUnit->task_id, sdoUnit->sdo_id) != 0)	return D10_STATUS_SDO_SET_FAIL;

	/* Appending SDO and Packing */
	/* 0th txbuf is for a number of SDO */
	if(sdoMsg->msg_length == 0) {cursor = 1;}
	else						{cursor = sdoMsg->msg_length;}

	sdoMsg->txBuf[cursor++] = sdoUnit->task_id;
	sdoMsg->txBuf[cursor++] = sdoUnit->sdo_id;
	sdoMsg->txBuf[cursor++] = sdoUnit->param.sdo_status;
	sdoMsg->txBuf[cursor++] = sdoUnit->param.num_of_data;

	SDOinfo dataSize = Get_D10_SDOInfo(sdoUnit->task_id, sdoUnit->sdo_id);
	uint8_t t_size = sdoUnit->param.num_of_data*dataSize;

	memcpy(&sdoMsg->txBuf[cursor], sdoUnit->param.data, t_size);
	cursor += t_size;

	sdoMsg->n_sdo++;
	sdoMsg->txBuf[0] = sdoMsg->n_sdo;
	sdoMsg->msg_length = cursor;

	return D10_STATUS_SUCCESS;
}

void Clear_D10_SDO(SDOmsg* sdoMsg)
{
	memset(sdoMsg, 0, sizeof(SDOmsg));
}


/*
  ___ ___   ___     ___  _     _        _
 | _ \   \ / _ \   / _ \| |__ (_)___ __| |_
 |  _/ |) | (_) | | (_) | '_ \| / -_) _|  _|
 |_| |___/ \___/   \___/|_.__// \___\__|\__|
                            |__/
*/

PDOUnit Create_D10_PDOUnit(uint8_t taskId, uint8_t pdoId, void* addr)
{
	PDOUnit pdo_unit;

	pdo_unit.task_id = taskId;
	pdo_unit.pdo_id = pdoId;
	pdo_unit.addr = addr;

	return pdo_unit;
}

D10_STATUS Append_D10_PDO(PDOUnit* pdoUnit, PDOmsg* pdoMsg)
{
	uint8_t cursor;

	/* Validation Check */
	if(D10_CheckPDO(pdoUnit->task_id, pdoUnit->pdo_id) != 0)	return D10_STATUS_PDO_SET_FAIL;


	/* Appending PDO and Packing */
	/* 0th txbuf is for a number of PDO */
	if(pdoMsg->msg_length == 0) {cursor = 1;}
	else						{cursor = pdoMsg->msg_length;}

	pdoMsg->txBuf[cursor++] = pdoUnit->task_id;
	pdoMsg->txBuf[cursor++] = pdoUnit->pdo_id;

	PDOinfo* PDOinfo = Get_D10_PDOInfo(pdoUnit->task_id, pdoUnit->pdo_id);
	uint8_t t_size = ((*PDOinfo)[0])*((*PDOinfo)[1]);

	memcpy(&pdoMsg->txBuf[cursor], pdoUnit->addr, t_size);
	cursor += t_size;

	pdoMsg->n_pdo++;
	pdoMsg->txBuf[0] = pdoMsg->n_pdo;
	pdoMsg->msg_length = cursor;

	return D10_STATUS_SUCCESS;
}

void Clear_D10_PDO(PDOmsg* pdoMsg)
{
	memset(pdoMsg, 0, sizeof(PDOmsg));
}

/*
   ___     _        ___      _
  / __|___| |_     / __| ___| |_
 | (_ / -_)  _|    \__ \/ -_)  _|
  \___\___|\__|    |___/\___|\__|

*/

void Set_D10_SDO_Addr(D10Object* obj, uint8_t taskID, uint8_t sdoID, void* addr)
{
	obj->tasks[taskID].sdos_addrs[sdoID] = addr;
}

void* Get_D10_SDO_Addr(D10Object* obj, uint8_t taskID, uint8_t sdoID)
{
	return obj->tasks[taskID].sdos_addrs[sdoID];
}

void Set_D10_PDO_Addr(D10Object* obj, uint8_t taskID, uint8_t pdoID, void* addr)
{
	obj->tasks[taskID].pdos_addrs[pdoID] = addr;
}

void* Get_D10_PDO_Addr(D10Object* obj, uint8_t taskID, uint8_t pdoID)
{
	return obj->tasks[taskID].pdos_addrs[pdoID];
}

void Set_D10_RoutineOnOff(D10Object* obj, uint8_t taskID, uint8_t rtID, ObjectDictionaryRoutineOnoff OnOff)
{
	obj->tasks[taskID].routines[rtID] = OnOff;
}

ObjectDictionaryRoutineOnoff Get_D10_RoutineOnOff(D10Object* obj, uint8_t taskID, uint8_t rtID)
{
	return obj->tasks[taskID].routines[rtID];
}

/*
  ___            _           ___ ___   ___
 | _ \___ __ ___(_)_ _____  | _ \   \ / _ \
 |   / -_) _/ -_) \ V / -_) |  _/ |) | (_) |
 |_|_\___\__\___|_|\_/\___| |_| |___/ \___/

*/


static int Read_D10_PDO(D10Object* obj, uint8_t* byte_arr)
{
    int cursor = 0;

    uint8_t t_task_id, t_pdo_id;

    memcpy(&t_task_id, &byte_arr[cursor++], 1);
    memcpy(&t_pdo_id, &byte_arr[cursor++], 1);

    if(D10_CheckPDO(t_task_id, t_pdo_id) != 0 )	return D10_STATUS_RECV_FAIL;

    uint8_t t_size = Convert_DataSize(PDO_Table[t_task_id][t_pdo_id][0]) * PDO_Table[t_task_id][t_pdo_id][1];
    memcpy(obj->tasks[t_task_id].pdos_addrs[t_pdo_id], &byte_arr[cursor], t_size);
    cursor += t_size;

    return cursor;
}

int Unpack_D10_PDO(D10Object* obj, uint8_t* byte_arr)
{
    int cursor = 0;

    // Get # of PDOs
    uint8_t n_pdo = 0;
    memcpy(&n_pdo, &byte_arr[cursor++], 1);

    if (n_pdo > 0) {
        for (int i = 0; i < n_pdo; ++i) {
            int temp_cursor = Read_D10_PDO(obj, &byte_arr[cursor]);
            if (temp_cursor > 0) {
                cursor += temp_cursor;
            } else if (temp_cursor < 0) {
                //TODO: Unpack PDO Error
                return D10_STATUS_RECV_PDO_FAIL;
            }
        }
    }

    return D10_STATUS_SUCCESS;
}

/*
  ___            _           ___ ___   ___
 | _ \___ __ ___(_)_ _____  / __|   \ / _ \
 |   / -_) _/ -_) \ V / -_) \__ \ |) | (_) |
 |_|_\___\__\___|_|\_/\___| |___/___/ \___/

*/

static int Read_D10_SDO(D10Object* obj, uint8_t* byte_arr)
{
    uint8_t cursor = 0;
    uint8_t t_task_id, t_sdo_id, t_num_of_data;
    int8_t t_sdo_status;

    memcpy(&t_task_id, &byte_arr[cursor++], 1);
    memcpy(&t_sdo_id, &byte_arr[cursor++], 1);
    memcpy(&t_sdo_status, &byte_arr[cursor++], 1);
    memcpy(&t_num_of_data, &byte_arr[cursor++], 1);

    if(D10_CheckSDO(t_task_id, t_sdo_id) != 0 )	return D10_STATUS_RECV_FAIL;

    uint8_t t_size = t_num_of_data*SDO_Table[t_task_id][t_sdo_id];
    cursor += t_size;
    switch(t_sdo_status){

    case D10_SDO_IDLE:
    	break;

    case D10_SDO_SUCC:
    	break;

    case D10_SDO_FAIL:
    	return D10_STATUS_RECV_SDO_FAIL;
    	break;

    default:
    	return D10_STATUS_RECV_SDO_FAIL;
    	break;
    }

    return cursor;
}

int Unpack_D10_SDO(D10Object* obj, uint8_t* byte_arr)
{
    int cursor = 0;

    // Get # of SDOs
    uint8_t n_sdo = 0;
    memcpy(&n_sdo, &byte_arr[cursor++], 1);

    // Call & Respond SDOs
    if (n_sdo > 0) {
        for (int i = 0; i < n_sdo; ++i) {
            int temp_cursor = Read_D10_SDO(obj, &byte_arr[cursor]);
            if (temp_cursor > 0) {
                cursor += temp_cursor;
            } else if (temp_cursor < 0) {
                //TODO: Unpack SDO ERROR
                return D10_STATUS_RECV_SDO_FAIL;
            }
        }
    }

    return D10_STATUS_SUCCESS;
}

void D10_Recv_EMCY(uint32_t *t_err_code, uint8_t *t_byte_arr)
{
    memcpy(t_err_code, t_byte_arr, ERR_CODE_SIZE);
}

/*
  ___      _ _
 |_ _|_ _ (_) |_
  | || ' \| |  _|
 |___|_||_|_|\__|

*/

void Init_D10(D10Object* obj, uint8_t nodeId)
{
	static uint8_t t_temp = 0;

	if(t_temp == 0) {
		Create_SDOTable_ObjDictionary();
		Create_PDOTable_ObjDictionary();
		t_temp++;
	}

	/* D10 Object Initialization */
	obj->node_id = nodeId;
	obj->error_code = NO_ERROR;
	obj->n_task = TASK_NUM;

	/* Task Allocation */
	for(int i=0; i < TASK_NUM; ++i){
		D10_TaskAllocation(&obj->tasks[i], i);
	}
}

#endif /* CM Modules */