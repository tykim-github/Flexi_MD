

#ifndef MSG_COMMON_H_
#define MSG_COMMON_H_

#include <stdio.h>
#include <stdlib.h>

#include "task_mngr.h"
#include "msg_data_object.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

/* Common SDO */
#define MSG_COMMON_SDO_CALLBACK(t_task)                                                       \
    static void Get_State_Callback(MsgSDOargs* req, MsgSDOargs* res)                            \
    {                                                                                         \
        res->size = 1;                                                                        \
        res->data = malloc(1);                                                                \
        ((uint8_t*)res->data)[0] = (uint8_t)((t_task).state_machine.curr_state);              \
        res->status = DATA_OBJECT_SDO_SUCC;                                                   \
    }                                                                                         \
                                                                                              \
    static void Set_State_Callback(MsgSDOargs* req, MsgSDOargs* res)                \
    {                                                                                         \
        StateEnum state_cmd = (StateEnum)(((uint8_t*)req->data)[0]);                          \
        Transition_State(&(t_task).state_machine, state_cmd);                                  \
        res->data = NULL;                                                                     \
        res->size = 0;                                                                        \
        res->status = DATA_OBJECT_SDO_SUCC;                                                   \
    }                                                                                         \
                                                                                            \
    static void Get_Routine_Callback(MsgSDOargs* req, MsgSDOargs* res)               \
    {                                                                                         \
        res->status = DATA_OBJECT_SDO_SUCC;                                                   \
        res->size = ((t_task).routine.n_id);                                            \
        if (res->size == 0) {                                                                 \
            return;                                                                           \
        }                                                                                     \
        res->data = malloc(req->data_size * res->size);                                       \
        for (int i = 0; i < res->size; ++i){                                                  \
            ((uint8_t*)res->data)[i] = (uint8_t)((t_task).routine.id[i]);               \
        }                                                                                     \
    }                                                                                         \
                                                                                              \
    static void Set_Routine_Callback(MsgSDOargs* req, MsgSDOargs* res)  		   \
    {                                                                                         \
        res->size = 0;                                                                        \
        Clear_Routines(&(t_task).routine);                                          \
        if (req->size == 0) {                                                                 \
            res->status = DATA_OBJECT_SDO_SUCC;                                               \
            return;                                                                           \
        }                                                                                     \
        res->data = malloc(req->data_size * req->size);                                       \
        for (int i = 0; i < req->size; ++i){                                                  \
            uint8_t id = ((uint8_t*)req->data)[i];                                            \
            if (Push_Routine(&(t_task).routine, id) == 0) {                        \
                ((uint8_t*)res->data)[i] = ((uint8_t*)req->data)[i];                          \
                ++res->size;                                                                  \
            }                                                                                 \
        }                                                                                     \
        res->status = (res->size != req->size) ? DATA_OBJECT_SDO_FAIL : DATA_OBJECT_SDO_SUCC; \
    }

// Create all common SDOs

#define MSG_COMMON_SDO_CREATE(t_task_id)                                   \
	Create_SDO(t_task_id, 0, e_UInt8, Get_State_Callback);   \
    Create_SDO(t_task_id, 1, e_UInt8, Set_State_Callback);		\
    Create_SDO(t_task_id, 2, e_UInt8, Get_Routine_Callback);  \
    Create_SDO(t_task_id, 3, e_UInt8, Set_Routine_Callback);


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */




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




#endif /* MSG_COMMON_H_ */
