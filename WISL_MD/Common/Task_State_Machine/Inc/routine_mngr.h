

#ifndef ROUTINE_MNGR_H_
#define ROUTINE_MNGR_H_

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define ROUTINE_MAX_ENTITIES 32
#define ROUTINE_DEFAULT_ID -1


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

// Routine Entity
typedef int (*RoutineFncPtr) (void);

typedef enum _RoutineEntityEnum{
    e_RoutineEntity_Ent,
	e_RoutineEntity_Run,
	e_RoutineEntity_Ext,
} RoutineEntityEnum;

typedef struct _RoutineEntityStruct {
    RoutineFncPtr on_enter;
    RoutineFncPtr on_run;
    RoutineFncPtr on_exit;
} RoutineEntityStruct;

typedef struct RoutineStruct {
    int id[ROUTINE_MAX_ENTITIES];
    size_t n_id;
    RoutineEntityStruct entities[ROUTINE_MAX_ENTITIES];
} RoutineStruct;

RoutineEntityStruct Create_Routine_Entity(RoutineFncPtr t_ent, RoutineFncPtr t_run, RoutineFncPtr t_ext);


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

// DriveRoutine Interface
void Init_Routine(RoutineStruct* t_routine);

int Ent_Routines(RoutineStruct* t_routine);
int Run_Routines(RoutineStruct* t_routine);
int Ext_Routines(RoutineStruct* t_routine);

void Clear_Routines(RoutineStruct* t_routine);

int Push_Routine(RoutineStruct* t_routine, uint8_t t_id);


#endif /* ROUTINE_MNGR_H_ */
