

#include "msg_data_object.h"

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

static MsgDataObjectDictionary msg_dods[TASK_NUM] = {0};


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

MsgDataTypeInfoStruct Get_Msg_Data_Type_Info(MsgDataTypeEnum t_type)
{
	MsgDataTypeInfoStruct t_res;

    switch (t_type) {
    case e_Char   : t_res.name = "char"   ; t_res.size = sizeof(char);     break;
    case e_UInt8  : t_res.name = "uint8"  ; t_res.size = sizeof(uint8_t);  break;
    case e_UInt16 : t_res.name = "uint16" ; t_res.size = sizeof(uint16_t); break;
    case e_UInt32 : t_res.name = "uint32" ; t_res.size = sizeof(uint32_t); break;
    case e_Int8   : t_res.name = "int8"   ; t_res.size = sizeof(int8_t);   break;
    case e_Int16  : t_res.name = "int16"  ; t_res.size = sizeof(int16_t);  break;
    case e_Int32  : t_res.name = "int32"  ; t_res.size = sizeof(int32_t);  break;
    case e_Float32: t_res.name = "float32"; t_res.size = sizeof(float);    break;
    case e_Float64: t_res.name = "float64"; t_res.size = sizeof(double);   break;
    case e_String10: t_res.name = "string10"; t_res.size = 32*sizeof(char);   break;
    default: break;
    }
    return t_res;
}

/* ------------------- DATA OBJECT DICTIONARY ------------------- */
// Create Data Object & Dictionary
void Create_DOD(uint8_t t_dod_id)
{
	msg_dods[t_dod_id].id = t_dod_id;
}

void Create_PDO(uint8_t t_dod_id, uint8_t t_obj_id, MsgDataTypeEnum t_type, uint8_t t_size, void* t_addr)
{
    msg_dods[t_dod_id].pdos[t_obj_id].id   = t_obj_id;
    msg_dods[t_dod_id].pdos[t_obj_id].type = t_type;
    msg_dods[t_dod_id].pdos[t_obj_id].size  = t_size;
    msg_dods[t_dod_id].pdos[t_obj_id].addr = t_addr;

    msg_dods[t_dod_id].pdos[t_obj_id].data_size  = Get_Msg_Data_Type_Info(t_type).size * t_size;
    msg_dods[t_dod_id].pdos[t_obj_id].last_pub = malloc(msg_dods[t_dod_id].pdos[t_obj_id].data_size);

    memset(msg_dods[t_dod_id].pdos[t_obj_id].last_pub, 0xFF, msg_dods[t_dod_id].pdos[t_obj_id].data_size);
}

void Create_SDO(uint8_t t_dod_id, uint8_t t_obj_id, MsgDataTypeEnum t_type, MsgSDOCallback t_callback)
{
	msg_dods[t_dod_id].sdos[t_obj_id].id        = t_obj_id;
	msg_dods[t_dod_id].sdos[t_obj_id].type      = t_type;
	msg_dods[t_dod_id].sdos[t_obj_id].callback  = t_callback;

	msg_dods[t_dod_id].sdos[t_obj_id].args.status = DATA_OBJECT_SDO_IDLE;
	msg_dods[t_dod_id].sdos[t_obj_id].args.size   = 0;
	msg_dods[t_dod_id].sdos[t_obj_id].args.data   = NULL;
	msg_dods[t_dod_id].sdos[t_obj_id].args.data_size = Get_Msg_Data_Type_Info(t_type).size;
}

MsgPDOStruct* Find_PDO(uint8_t t_dod_id, uint8_t t_id)
{
	return &msg_dods[t_dod_id].pdos[t_id];
}

MsgSDOStruct* Find_SDO(uint8_t t_dod_id, uint8_t t_id)
{
	return &msg_dods[t_dod_id].sdos[t_id];
}

uint16_t Set_PDO(MsgPDOStruct* t_pdo, void* t_data)
{    
    memcpy(t_data, t_pdo->addr, t_pdo->data_size);
    memcpy(t_pdo->last_pub, t_pdo->addr, t_pdo->data_size);
    return t_pdo->data_size;
}

uint16_t Get_PDO(MsgPDOStruct* t_pdo, void* t_data)
{
    memcpy(t_pdo->addr, t_data, t_pdo->data_size);
    return t_pdo->data_size;
}

uint16_t Call_SDO(MsgSDOStruct* t_sdo, MsgSDOargs* t_req)
{
    if (t_sdo->args.data != NULL) {
        free(t_sdo->args.data);
        t_sdo->args.data = NULL;
    }

    t_sdo->args.status = DATA_OBJECT_SDO_IDLE;
    t_sdo->callback(t_req, &t_sdo->args);
    return t_req->size * t_sdo->args.data_size;
}

void Set_SDO_req(uint8_t t_dod_id, uint8_t t_id, void* t_data, uint8_t t_size)
{
    MsgSDOargs t_req;

    MsgSDOStruct* t_sdo = Find_SDO(t_dod_id, t_id);
    if (t_sdo == NULL) {
        return;
    }

    t_req.status = DATA_OBJECT_SDO_REQU;
    t_req.data = t_data;
    t_req.size = t_size;
    Set_SDO_args(t_sdo, &t_req);
}

MsgSDOargs* Get_SDO_res(uint8_t t_dod_id, uint8_t t_id)
{
    MsgSDOStruct* t_sdo = Find_SDO(t_dod_id, t_id);
    if (t_sdo == NULL) {
        return NULL;
    }

    return &t_sdo->args;
}

uint16_t Set_SDO_args(MsgSDOStruct* t_sdo, MsgSDOargs* t_args)
{
    // Copy status
	t_sdo->args.status = t_args->status;
    
    // Copy size
    int total_size = t_sdo->args.data_size * t_args->size;
    t_sdo->args.size = t_args->size;
    if (total_size <= 0) {
        return 0;
    }

    // Copy data
    if (t_sdo->args.data != NULL) {
        free(t_sdo->args.data);
        t_sdo->args.data = NULL;
    }
    t_sdo->args.data = malloc(total_size);
    memcpy(t_sdo->args.data, t_args->data, total_size);

    return total_size;
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

