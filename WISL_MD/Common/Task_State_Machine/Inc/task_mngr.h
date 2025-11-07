

#ifndef TASK_MNGR_H_
#define TASK_MNGR_H_

#include <stdbool.h>

#include "module.h"

#include "routine_mngr.h"
#include "task_state_machine.h"

#ifdef _USE_CMSISV1
#include "cmsis_os.h"
#endif /* _USE_CMSISV1 */

#ifdef _USE_CMSISV2
#include "cmsis_os2.h"
#endif /* _USE_CMSISV2 */

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define TASK_CREATE_STATE(t_task_ptr, t_state, t_ent_fncptr, t_run_fncptr, t_ext_fncptr, t_is_default)  \
	Set_Task_State_Entity((t_task_ptr), (t_state), Create_State_Entity((t_ent_fncptr), (t_run_fncptr), (t_ext_fncptr)), t_is_default)

#define TASK_CREATE_ROUTINE(t_task_ptr, t_id, t_ent_fncptr, t_run_fncptr, t_ext_fncptr) \
    Set_Task_Routine_Entity((t_task_ptr), (t_id), Create_Routine_Entity((t_ent_fncptr), (t_run_fncptr), (t_ext_fncptr)))


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef struct TaskStruct {
    int id;
    int period;
    char* name;

    uint32_t err_code;

    StateMachineStruct state_machine;
    RoutineStruct routine;
} TaskStruct;


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

void Init_Task(TaskStruct* t_task);
void Run_Task(TaskStruct* t_task);

void Set_Task_State_Entity(TaskStruct* t_task, StateEnum t_state, StateEntityStruct t_entity, bool t_is_default);
void Set_Task_Routine_Entity(TaskStruct* t_task, int t_id, RoutineEntityStruct t_entity);

#ifdef _USE_OS_RTOS
void TaskDelay(TaskStruct* t_task);
#endif /* _USE_OS_RTOS */
void SetTaskErrCode(TaskStruct* t_task, uint32_t eCode);
uint32_t GetTaskErrCode(TaskStruct* t_task);

#endif /* TASK_MNGR_H_ */
