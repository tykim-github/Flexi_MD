

#include "task_state_machine.h"

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

static StateEnum Transition_Map(StateEnum t_curr, StateEnum t_cmd);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

// State Functions
StateEntityStruct Create_State_Entity(StateFuncPtr t_ent, StateFuncPtr t_run, StateFuncPtr t_ext)
{
    StateEntityStruct t_res;
    t_res.on_enter = t_ent;
    t_res.on_run =   t_run;
    t_res.on_exit =  t_ext;
    return t_res;
}


// State Machine Interfaces
void Init_State_Machine(StateMachineStruct* t_sm)
{
    for (int i = 0; i < STATE_MACHINE_N_STATES; i++) {
    	t_sm->entity[i].on_enter = NULL;
    	t_sm->entity[i].on_run   = NULL;
    	t_sm->entity[i].on_exit  = NULL;
    }
    t_sm->entity_life_cycle = e_StateEntity_Ent;
}

void Run_State_Machine(StateMachineStruct* t_sm)
{
    switch (t_sm->entity_life_cycle) {
        case e_StateEntity_Ent:
            if (t_sm->entity[t_sm->curr_state].on_enter) {
                t_sm->entity[t_sm->curr_state].on_enter();
            }
            if (t_sm->entity_life_cycle == e_StateEntity_Ent){
                t_sm->entity_life_cycle = e_StateEntity_Run;
            }
            break;

        case e_StateEntity_Run:
            if (t_sm->entity[t_sm->curr_state].on_run) {
                t_sm->entity[t_sm->curr_state].on_run();
            }
            break;
        
        case e_StateEntity_Ext:
            if (t_sm->entity[t_sm->prev_state].on_exit) {
                t_sm->entity[t_sm->prev_state].on_exit();
            }
            t_sm->entity_life_cycle = e_StateEntity_Ent;
            break;

        default: // Invalid Lifecycle
            break;
    }
}

void Transition_State(StateMachineStruct* t_sm, StateEnum t_state_cmd)
{
    StateEnum new_state = Transition_Map(t_sm->curr_state, t_state_cmd);
    if (t_sm->curr_state != new_state) {
    	t_sm->prev_state = t_sm->curr_state;
    	t_sm->curr_state = new_state;
    	t_sm->entity_life_cycle = e_StateEntity_Ext;
    }
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

// State & Transition Map
static StateEnum Transition_Map(StateEnum t_curr, StateEnum t_cmd)
{
    if (t_curr == t_cmd) {
        return t_curr;
    }

    switch (t_curr) {
        /*  |- From State -|  |---------------------------------- To State ----------------------------------| |- If valid -| |else| */
        case e_State_Off:     if (0              	 || t_cmd==e_State_Standby || 0      	            || 0               ) 	 {return t_cmd; } break;
        case e_State_Standby: if (t_cmd==e_State_Off || 0                  	   || t_cmd==e_State_Enable || 0               ) 	 {return t_cmd; } break;
        case e_State_Enable:  if (t_cmd==e_State_Off || t_cmd==e_State_Standby || 0         	        || t_cmd==e_State_Error) {return t_cmd; } break;
        case e_State_Error:   if (t_cmd==e_State_Off || t_cmd==e_State_Standby || 0             	    || 0               ) 	 {return t_cmd; } break;
        default: break;
    }

    return t_curr;
}


