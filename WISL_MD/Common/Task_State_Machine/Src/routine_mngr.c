

#include "routine_mngr.h"

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

// Routine Entity
RoutineEntityStruct Create_Routine_Entity(RoutineFncPtr t_ent, RoutineFncPtr t_run, RoutineFncPtr t_ext)
{
    RoutineEntityStruct t_res;
    t_res.on_enter = t_ent;
    t_res.on_run = t_run;
    t_res.on_exit = t_ext;
    return t_res;
}


// DriveRoutine Interface
void Init_Routine(RoutineStruct* t_routine)
{
    for (int i = 0; i < ROUTINE_MAX_ENTITIES; i++) {
    	t_routine->id[i] = 0;
    	t_routine->entities[i].on_enter = NULL;
    	t_routine->entities[i].on_run = NULL;
    	t_routine->entities[i].on_exit = NULL;
    }
    t_routine->n_id = 0;
}

int Ent_Routines(RoutineStruct* t_routine)
{
	static int t_res = 0;
	static int t_id;

    for (int i = 0; i < t_routine->n_id; i++){
        //TODO: routine func exception handling

    	t_id = t_routine->id[i];
		if (t_routine->entities[t_id].on_enter) {
			t_res = t_routine->entities[t_id].on_enter();
		}
        if (t_res < 0) {
            return t_res;
        }
    }
    return 0;
}

int Run_Routines(RoutineStruct* t_routine)
{
	int t_res = 0;
	int t_id;

    for (int i = 0; i < t_routine->n_id; i++){
        //TODO: routine func exception handling

    	t_id = t_routine->id[i];
		if (t_routine->entities[t_id].on_run) {
			t_res = t_routine->entities[t_id].on_run();
		}
        if (t_res < 0) {
            return t_res;
        }
    }
    return 0;
}

int Ext_Routines(RoutineStruct* t_routine)
{
	static int t_res = 0;
	static int t_id;

    for (int i = 0; i < t_routine->n_id; i++){
        //TODO: routine func exception handling

    	t_id = t_routine->id[i];
		if (t_routine->entities[t_id].on_exit) {
			t_res = t_routine->entities[t_id].on_exit();
		}
        if (t_res < 0) {
            return t_res;
        }
    }
    return 0;
}

void Clear_Routines(RoutineStruct* t_routine)
{
    for (int i = 0; i < ROUTINE_MAX_ENTITIES; i++) {
    	t_routine->id[i] = ROUTINE_DEFAULT_ID;
    }
    t_routine->n_id = 0;
}

int Push_Routine(RoutineStruct* t_routine, uint8_t t_id)
{
    if (t_routine->n_id >= ROUTINE_MAX_ENTITIES) {
        return -1;
    }

    for(int i = 0; i < t_routine->n_id; i++){
    	if(t_routine->id[i] == t_id){return 0;}
    }
    
    t_routine->id[t_routine->n_id++] = t_id;
    return 0;
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

