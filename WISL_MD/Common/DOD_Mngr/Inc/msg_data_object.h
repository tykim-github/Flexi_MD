
#ifndef MSG_DATA_OBJECT_H_
#define MSG_DATA_OBJECT_H_

#if __cplusplus
extern "C"{
#endif

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "object_dictionaries.h"
#include "cvector.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define MSG_SDO_NOTHING 1

#define MSG_DO_SUCCESS 0
#define MSG_PDO_FAULT  -1
#define MSG_SDO_FAULT -2

#define DOP_CHAR_PDO 0x50
#define DOP_CHAR_SDO 0x53

#define CALLER_ID_SIZE 1
#define OBJ_CHAR_SIZE 1

#define DEVICE_ID_LEN       1
#define NODE_ID_LEN         1
#define CMD_LEN             1
#define MSG_1BYTE           1

#define DATA_OBJECT_SDO_IDLE  	2
#define DATA_OBJECT_SDO_REQU  	1
#define DATA_OBJECT_SDO_SUCC  	0
#define DATA_OBJECT_SDO_FAIL 	-1

#define DATA_OBJECT_SDO_GET_DOD_LIST 0
#define DATA_OBJECT_SDO_GET_PDO_LIST 1
#define DATA_OBJECT_SDO_GET_SDO_LIST 2

#define D10_TASK_NUM 4


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef enum _MsgDataTypeEnum {
    e_Char,
	e_UInt8,
	e_UInt16,
	e_UInt32,
	e_Int8,
	e_Int16,
	e_Int32,
	e_Float32,
	e_Float64,
	e_String10,
} MsgDataTypeEnum;

typedef struct _MsgDataTypeInfoStruct {
    char* name;
    uint8_t size;
} MsgDataTypeInfoStruct;

typedef struct _MsgSDOargs {
    void* data;
    uint8_t size;
    uint16_t data_size;
    int8_t status;
} MsgSDOargs;

typedef void (*MsgSDOCallback) (MsgSDOargs*, MsgSDOargs*);

typedef struct _MsgDataObjectHeader {
	uint8_t dod_id;
    uint8_t obj_id;
} MsgDataObjectHeader;

typedef struct _MsgPDOStruct {
    uint8_t id;
    char* name;

    MsgDataTypeEnum type;
    uint8_t size;

    void* addr;
    void* last_pub;
    uint8_t data_size;
} MsgPDOStruct;

typedef struct _MsgSDOStruct {
    uint8_t id;

    MsgDataTypeEnum type;
    
    MsgSDOCallback callback;
    MsgSDOargs args;
} MsgSDOStruct;

typedef struct _MsgDataObjectDictionary {
    char* name;
    uint8_t id;

    MsgPDOStruct pdos[PDO_MAX_NUM];
    MsgSDOStruct sdos[SDO_MAX_NUM];
} MsgDataObjectDictionary;


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

MsgDataTypeInfoStruct Get_Msg_Data_Type_Info(MsgDataTypeEnum type);

void Create_DOD(uint8_t t_dod_id);
void Create_PDO(uint8_t t_dod_id, uint8_t t_obj_id, MsgDataTypeEnum t_type, uint8_t t_size, void* t_addr);
void Create_SDO(uint8_t t_dod_id, uint8_t t_obj_id, MsgDataTypeEnum t_type, MsgSDOCallback t_callback);

MsgPDOStruct* Find_PDO(uint8_t t_dod_id, uint8_t t_id);
MsgSDOStruct* Find_SDO(uint8_t t_dod_id, uint8_t t_id);

uint16_t Set_PDO(MsgPDOStruct* t_pdo, void* t_data);
uint16_t Get_PDO(MsgPDOStruct* t_pdo, void* t_data);

void Set_SDO_req(uint8_t t_dod_id, uint8_t t_id, void* t_data, uint8_t t_size);
MsgSDOargs* Get_SDO_res(uint8_t t_dod_id, uint8_t t_id);

uint16_t Call_SDO(MsgSDOStruct* t_sdo, MsgSDOargs* t_req);
uint16_t Set_SDO_args(MsgSDOStruct* t_sdo, MsgSDOargs* t_args);


#if __cplusplus
}
#endif


#endif /* MSG_DATA_OBJECT_H_ */
