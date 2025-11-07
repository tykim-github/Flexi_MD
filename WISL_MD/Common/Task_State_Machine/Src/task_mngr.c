

#include "task_mngr.h"

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




/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

// Device Interface
void Init_Task(TaskStruct* t_task)
{
    Init_State_Machine(&t_task->state_machine);
    Init_Routine(&t_task->routine);
    t_task->err_code = 0;
}

void Run_Task(TaskStruct* t_task)
{
    Run_State_Machine(&t_task->state_machine);
}

void Set_Task_State_Entity(TaskStruct* t_task, StateEnum t_state, StateEntityStruct t_entity, bool t_is_default)
{
	t_task->state_machine.entity[t_state] = t_entity;

    if (t_is_default) {
    	t_task->state_machine.curr_state = t_state;
    	t_task->state_machine.prev_state = t_state;
    }
}

//TODO: id, order range/duplication check
void Set_Task_Routine_Entity(TaskStruct* t_task, int t_id, RoutineEntityStruct t_entity)
{
	t_task->routine.entities[t_id] = t_entity;
}

#ifdef _USE_OS_RTOS
void TaskDelay(TaskStruct* t_task)
{
    osDelay(t_task->period);
}
#endif /* _USE_OS_RTOS */

void SetTaskErrCode(TaskStruct* t_task, uint32_t eCode)
{
    t_task->err_code = eCode;
}

uint32_t GetTaskErrCode(TaskStruct* t_task)
{
    return t_task->err_code;
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

