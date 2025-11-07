

#ifndef TASK_STATE_MACHINE_H_
#define TASK_STATE_MACHINE_H_

#include <unistd.h>

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

// State & Transition Map
#define STATE_MACHINE_N_STATES 4


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

// State Functions
typedef void (*StateFuncPtr)(void);

typedef enum _StateEnum {
    e_State_Off = 0,
	e_State_Standby = 1,
	e_State_Enable = 2,
	e_State_Error = 3,
} StateEnum;

typedef struct _StateEntityStruct {
    StateFuncPtr on_enter;
    StateFuncPtr on_run;
    StateFuncPtr on_exit;
} StateEntityStruct;

// State Machine
typedef enum StateEntityLifeCycle {
    e_StateEntity_Ent,
	e_StateEntity_Run,
	e_StateEntity_Ext,
} StateEntityLifeCycle;

typedef struct _StateMachineStruct {
    StateEntityStruct entity[STATE_MACHINE_N_STATES];

    StateEnum curr_state;
    StateEnum prev_state;
    StateEntityLifeCycle entity_life_cycle;
} StateMachineStruct;

StateEntityStruct Create_State_Entity(StateFuncPtr t_ent, StateFuncPtr t_run, StateFuncPtr t_ext);


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

// State Machine Interfaces
void Init_State_Machine(StateMachineStruct* t_sm);
void Run_State_Machine(StateMachineStruct* t_sm);

void Transition_State(StateMachineStruct* t_sm, StateEnum t_state_cmd);


#endif /* TASK_STATE_MACHINE_H_ */