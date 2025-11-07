

#include "BumbleBee.h"

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

static PDOinfo* Get_BumbleBee_PDO_Info(uint8_t t_taskID, uint8_t t_pdoID);
static SDOinfo Get_BumbleBee_SDO_Info(uint8_t t_taskID, uint8_t t_sdoID);
static void Allocate_BumbleBee_Task(TaskObject* t_task, uint8_t t_taskID);
static int Check_BumbleBee_SDO(uint8_t t_taskID, uint8_t t_sdoID);
static int Check_BumbleBee_PDO(uint8_t t_taskID, uint8_t t_pdoID);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

/* ------------------- SDO Object ------------------- */
SDOUnit Create_BumbleBee_SDOUnit(BumbleBeeObject *t_obj,  SDOStatus t_sdoStatus, uint8_t t_taskID, uint8_t t_sdoID, uint8_t t_numOfData)
{
	SDOUnit sdo_unit;

	sdo_unit.task_id = t_taskID;
	sdo_unit.sdo_id = t_sdoID;
	sdo_unit.param.sdo_status = t_sdoStatus;
	sdo_unit.param.num_of_data = t_numOfData;
	sdo_unit.param.data = t_obj->tasks[t_taskID].sdos_addrs[t_sdoID];

	return sdo_unit;
}

BumbleBee_status Append_BumbleBee_SDO(SDOUnit* t_sdoUnit, SDOMsg* t_sdoMsg)
{
	uint8_t cursor;


	/* Validation Check */
	if(Check_BumbleBee_SDO(t_sdoUnit->task_id, t_sdoUnit->sdo_id) != 0)	return BUMBLEBEE_STATUS_SDO_SET_FAIL;


	/* Appending SDO and Packing */
	/* 0th txbuf is for a number of SDO */
	if(t_sdoMsg->msg_length == 0) {cursor = 1;}
	else						{cursor = t_sdoMsg->msg_length;}

	t_sdoMsg->txBuf[cursor++] = t_sdoUnit->task_id;
	t_sdoMsg->txBuf[cursor++] = t_sdoUnit->sdo_id;
	t_sdoMsg->txBuf[cursor++] = t_sdoUnit->param.sdo_status;
	t_sdoMsg->txBuf[cursor++] = t_sdoUnit->param.num_of_data;

	SDOinfo dataSize = Get_BumbleBee_SDO_Info(t_sdoUnit->task_id, t_sdoUnit->sdo_id);
	uint8_t t_size = t_sdoUnit->param.num_of_data*dataSize;

	memcpy(&t_sdoMsg->txBuf[cursor], t_sdoUnit->param.data, t_size);
	cursor += t_size;

	t_sdoMsg->n_sdo++;
	t_sdoMsg->txBuf[0] = t_sdoMsg->n_sdo;
	t_sdoMsg->msg_length = cursor;

	return BUMBLEBEE_STATUS_SUCCESS;
}

void Clear_BumbleBee_SDO(SDOMsg* t_sdoMsg)
{
	memset(t_sdoMsg, 0, sizeof(SDOMsg));
}

/* ------------------- PDO OBJECT ------------------- */
PDOUnit Create_BumbleBee_PDOUnit(uint8_t t_taskID, uint8_t t_pdoID, void* t_addr)
{
	PDOUnit pdo_unit;

	pdo_unit.task_id = t_taskID;
	pdo_unit.pdo_id = t_pdoID;
	pdo_unit.addr = t_addr;

	return pdo_unit;
}

BumbleBee_status Append_BumbleBee_PDO(PDOMsg* t_pdoMsg, PDOUnit* t_pdoUnit)
{
	uint8_t cursor;

	/* Validation Check */
	if(Check_BumbleBee_PDO(t_pdoUnit->task_id, t_pdoUnit->pdo_id) != 0)	return BUMBLEBEE_STATUS_PDO_SET_FAIL;


	/* Appending PDO and Packing */
	/* 0th txbuf is for a number of PDO */
	if(t_pdoMsg->msg_length == 0) {cursor = 1;}
	else						{cursor = t_pdoMsg->msg_length;}

	t_pdoMsg->txBuf[cursor++] = t_pdoUnit->task_id;
	t_pdoMsg->txBuf[cursor++] = t_pdoUnit->pdo_id;

	PDOinfo* t_pdoInfo = Get_BumbleBee_PDO_Info(t_pdoUnit->task_id, t_pdoUnit->pdo_id);
	uint8_t t_size = (*t_pdoInfo[0])*(*t_pdoInfo[1]);

	memcpy(&t_pdoMsg->txBuf[cursor], t_pdoUnit->addr, t_size);
	cursor += t_size;

	t_pdoMsg->n_pdo++;
	t_pdoMsg->txBuf[0] = t_pdoMsg->n_pdo;
	t_pdoMsg->msg_length = cursor;

	return BUMBLEBEE_STATUS_SUCCESS;
}

void Clear_BumbleBee_PDO(PDOMsg* t_pdoMsg)
{
	memset(t_pdoMsg, 0, sizeof(PDOMsg));
}

/* ------------------- GET & SET ------------------- */
void Set_BumbleBee_SDOAddr(BumbleBeeObject* t_obj, uint8_t t_taskID, uint8_t t_sdoID, void* t_addr)
{
	t_obj->tasks[t_taskID].sdos_addrs[t_sdoID] = t_addr;
}

void* Get_BumbleBee_SDOAddr(BumbleBeeObject* t_obj, uint8_t t_taskID, uint8_t t_sdoID)
{
	return t_obj->tasks[t_taskID].sdos_addrs[t_sdoID];
}


void Set_BumbleBee_PDOAddr(BumbleBeeObject* t_obj, uint8_t t_taskID, uint8_t t_pdoID, void* t_addr)
{
	t_obj->tasks[t_taskID].pdos_addrs[t_pdoID] = t_addr;
}

void* Get_BumbleBee_PDOAddr(BumbleBeeObject* t_obj, uint8_t t_taskID, uint8_t t_pdoID)
{
	return t_obj->tasks[t_taskID].pdos_addrs[t_pdoID];
}

void Set_BumbleBee_RoutineOnOff(BumbleBeeObject* t_obj, ObjectDictionaryRoutineOnoff t_OnOff, uint8_t t_taskID, uint8_t t_rtID )
{
	t_obj->tasks[t_taskID].routines[t_rtID] = t_OnOff;
}

ObjectDictionaryRoutineOnoff Get_BumbleBee_RoutineOnOff(BumbleBeeObject* t_obj, uint8_t t_taskID, uint8_t t_rtID){
	return t_obj->tasks[t_taskID].routines[t_rtID];
}

/* ------------------- RECEIVE SDO ------------------- */
static int Read_BumbleBee_SDO(BumbleBeeObject* t_obj, uint8_t* t_byte_arr)
{
    uint8_t cursor = 0;
    uint8_t t_task_id, t_sdo_id, t_num_of_data;
    int8_t t_sdo_status;

    memcpy(&t_task_id, &t_byte_arr[cursor++], 1);
    memcpy(&t_sdo_id, &t_byte_arr[cursor++], 1);
    memcpy(&t_sdo_status, &t_byte_arr[cursor++], 1);
    memcpy(&t_num_of_data, &t_byte_arr[cursor++], 1);

    if(Check_BumbleBee_SDO(t_task_id, t_sdo_id) != 0 )	return BUMBLEBEE_STATUS_RECV_FAIL;

    uint8_t t_size = t_num_of_data*SDO_Table[t_task_id][t_sdo_id];
    cursor += t_size;

    switch(t_sdo_status){

    case BUMBLEBEE_SDO_IDLE:
    	break;

    case BUMBLEBEE_SDO_SUCC:
    	break;

    case BUMBLEBEE_SDO_FAIL:
    	return BUMBLEBEE_STATUS_RECV_SDO_FAIL;
    	break;

    default:
    	return BUMBLEBEE_STATUS_RECV_SDO_FAIL;
    	break;
    }

    return cursor;
}


int Unpack_BumbleBee_SDO(BumbleBeeObject* t_obj, uint8_t* t_byte_arr)
{
    int cursor = 0;

    // Get # of SDOs
    uint8_t n_sdo = 0;
    memcpy(&n_sdo, &t_byte_arr[cursor++], 1);

    // Call & Respond SDOs
    if (n_sdo > 0) {
        for (int i = 0; i < n_sdo; ++i) {
            int temp_cursor = Read_BumbleBee_SDO(t_obj, &t_byte_arr[cursor]);
            if (temp_cursor > 0) {
                cursor += temp_cursor;
            } else if (temp_cursor < 0) {
                //TODO: Unpack SDO ERROR
                return BUMBLEBEE_STATUS_RECV_SDO_FAIL;
            }
        }
    }

    return BUMBLEBEE_STATUS_SUCCESS;
}


/* ------------------- RECEIVE PDO ------------------- */
static int Read_BumbleBee_PDO(BumbleBeeObject* t_obj, uint8_t* t_byte_arr)
{
    int cursor = 0;

    uint8_t t_task_id, t_pdo_id;

    memcpy(&t_task_id, &t_byte_arr[cursor++], 1);
    memcpy(&t_pdo_id, &t_byte_arr[cursor++], 1);

    if(Check_BumbleBee_PDO(t_task_id, t_pdo_id) != 0 )	return BUMBLEBEE_STATUS_RECV_FAIL;

    uint8_t t_size = Convert_DataSize(PDO_Table[t_task_id][t_pdo_id][0]) * PDO_Table[t_task_id][t_pdo_id][1];
    memcpy(t_obj->tasks[t_task_id].pdos_addrs[t_pdo_id], &t_byte_arr[cursor], t_size);
    cursor += t_size;

    return cursor;
}


int Unpack_BumbleBee_PDO(BumbleBeeObject* t_obj, uint8_t* t_byte_arr)
{
    int cursor = 0;

    // Get # of PDOs
    uint8_t n_pdo = 0;
    memcpy(&n_pdo, &t_byte_arr[cursor++], 1);

    if (n_pdo > 0) {
        for (int i = 0; i < n_pdo; ++i) {
            int temp_cursor = Read_BumbleBee_PDO(t_obj, &t_byte_arr[cursor]);
            if (temp_cursor > 0) {
                cursor += temp_cursor;
            } else if (temp_cursor < 0) {
                //TODO: Unpack PDO Error
                return BUMBLEBEE_STATUS_RECV_PDO_FAIL;
            }
        }
    }

    return BUMBLEBEE_STATUS_SUCCESS;
}

/* ------------------- INIT ------------------- */
void Init_BumbleBee(BumbleBeeObject* t_obj, uint8_t t_nodeId)
{
	static uint8_t t_temp = 0;

	if(t_temp == 0){
		Create_SDOTable_ObjDictionary();
		Create_PDOTable_ObjDictionary();
		t_temp++;
	}

	/* BumbleBee Object Initialization */
	t_obj->node_id = t_nodeId;
	t_obj->error_code = NO_ERROR;
	t_obj->n_task = TASK_NUM;

	/* Task Allocation */
	for(int i=0; i < TASK_NUM; ++i){
		Allocate_BumbleBee_Task(&t_obj->tasks[i], i);
	}

}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

static PDOinfo* Get_BumbleBee_PDO_Info(uint8_t t_taskID, uint8_t t_pdoID)
{
	return &PDO_Table[t_taskID][t_pdoID];
}

static SDOinfo Get_BumbleBee_SDO_Info(uint8_t t_taskID, uint8_t t_sdoID)
{
	return Convert_DataSize(SDO_Table[t_taskID][t_sdoID]);
}

static void Allocate_BumbleBee_Task(TaskObject* t_task, uint8_t t_taskID)
{
	t_task->task_id = t_taskID;
	t_task->task_state = STATE_IDLE;
}

static int Check_BumbleBee_SDO(uint8_t t_taskID, uint8_t t_sdoID)
{
	/* Check Error */
	switch(t_taskID){
	case TASK_ID_LOWLEVEL:
		if(t_sdoID >= SDO_ID_LOWLEVEL_NUM)	return BUMBLEBEE_STATUS_SDO_SET_FAIL;
		break;
	case TASK_ID_MIDLEVEL:
		if(t_sdoID >= SDO_ID_MIDLEVEL_NUM)	return BUMBLEBEE_STATUS_SDO_SET_FAIL;
		break;
	case TASK_ID_MSG:
		if(t_sdoID >= SDO_ID_MSG_NUM)	return BUMBLEBEE_STATUS_SDO_SET_FAIL;
		break;
	case TASK_ID_WIDM:
		if(t_sdoID >= SDO_ID_WIDM_NUM)	return BUMBLEBEE_STATUS_SDO_SET_FAIL;
		break;
	case TASK_ID_SYSMNGT:
		if(t_sdoID >= SDO_ID_SYSMNGT_NUM)	return BUMBLEBEE_STATUS_SDO_SET_FAIL;
		break;
	case TASK_ID_EXTDEV:
		if(t_sdoID >= SDO_ID_EXTDEV_NUM)	return BUMBLEBEE_STATUS_SDO_SET_FAIL;
		break;
	case TASK_ID_RISK:
		if(t_sdoID >= SDO_ID_RISK_NUM)	return BUMBLEBEE_STATUS_SDO_SET_FAIL;
		break;
	default:
		return BUMBLEBEE_STATUS_SDO_SET_FAIL;
		break;
	}

	return BUMBLEBEE_STATUS_SUCCESS;
}

static int Check_BumbleBee_PDO(uint8_t t_taskID, uint8_t t_pdoID)
{
	switch(t_taskID){
	case TASK_ID_LOWLEVEL:
		if(t_pdoID >= PDO_ID_LOWLEVEL_NUM)	return BUMBLEBEE_STATUS_PDO_SET_FAIL;
		break;
	case TASK_ID_MIDLEVEL:
		if(t_pdoID >= PDO_ID_MIDLEVEL_NUM)	return BUMBLEBEE_STATUS_PDO_SET_FAIL;
		break;
	case TASK_ID_MSG:
		if(t_pdoID >= PDO_ID_MSG_NUM)	return BUMBLEBEE_STATUS_PDO_SET_FAIL;
		break;
	case TASK_ID_WIDM:
		if(t_pdoID >= PDO_ID_WIDM_NUM)	return BUMBLEBEE_STATUS_PDO_SET_FAIL;
		break;
	case TASK_ID_SYSMNGT:
		if(t_pdoID >= PDO_ID_SYSMNGT_NUM)	return BUMBLEBEE_STATUS_PDO_SET_FAIL;
		break;
	case TASK_ID_EXTDEV:
		if(t_pdoID >= PDO_ID_EXTDEV_NUM)   return BUMBLEBEE_STATUS_PDO_SET_FAIL;
		break;
	case TASK_ID_RISK:
		if(t_pdoID >= PDO_ID_RISK_NUM)	return BUMBLEBEE_STATUS_PDO_SET_FAIL;
		break;
	default:
		return BUMBLEBEE_STATUS_PDO_SET_FAIL;
		break;
	}

	return BUMBLEBEE_STATUS_SUCCESS;
}


