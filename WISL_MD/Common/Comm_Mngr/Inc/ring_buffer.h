

#ifndef COMMON_COMM_MNGR_INC_RING_BUFFER_H_
#define COMMON_COMM_MNGR_INC_RING_BUFFER_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */




/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef struct {
	uint32_t head;
	uint32_t tail;
	uint32_t length;

	uint8_t* Buffer;
} RingBufferStruct;



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

bool RingBufferCreate(RingBufferStruct* RingBuf, uint8_t* buff, uint32_t length);
bool RingBufferPush(RingBufferStruct* RingBuf, uint8_t* data, uint32_t length);
bool RingBufferPop(RingBufferStruct* RingBuf, uint8_t* data, uint32_t length);
bool RingBufferFlush(RingBufferStruct* RingBuf);
bool RingBufferIsEmpty(RingBufferStruct* RingBuf);
bool RingBufferIsFull(RingBufferStruct* RingBuf);
bool RingBufferIsAvailable(RingBufferStruct* RingBuf);


#endif /* COMMON_COMM_MNGR_INC_RING_BUFFER_H_ */
