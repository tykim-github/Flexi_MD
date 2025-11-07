/**
 * @file bsp_i2c.h
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics HW Team
 * @brief Board Support Package for I2C functionalities.
 * @ref um2217-description-of-stm32h7-hal-and-lowlayer-drivers-stmicroelectronics.pdf
 */

#ifndef BSP_I2C_INC_BSP_I2C_H_
#define BSP_I2C_INC_BSP_I2C_H_

#include "main.h"
#include "module.h"

/** @defgroup I2C I2C
  * @brief I2C HAL BSP module driver
  * @
  */
#ifdef HAL_I2C_MODULE_ENABLED
#define BSP_I2C_MODULE_ENABLED

#include "i2c.h"
#include "../../../../../BSP/BSP_COMMON/Inc/bsp_common.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

/** @defgroup BSP I2C_MEMORY_ADDRESS_SIZE BSP I2C Memory Address size
  * @{
  */
#define BSP_I2C_MEMADD_SIZE_8BIT  I2C_MEMADD_SIZE_8BIT   ///< (0x00000001U)
#define BSP_I2C_MEMADD_SIZE_16BIT I2C_MEMADD_SIZE_16BIT  ///< (0x00000002U)

/** @defgroup BSP I2C_XFEROPTIONS BSP I2C Sequential Transfer Options
  * @{
  */
#define BSP_I2C_FIRST_FRAME          I2C_FIRST_FRAME          ///< ((uint32_t)I2C_SOFTEND_MODE)
#define BSP_I2C_FIRST_AND_NEXT_FRAME I2C_FIRST_AND_NEXT_FRAME ///< ((uint32_t)(I2C_RELOAD_MODE | I2C_SOFTEND_MODE))
#define BSP_I2C_NEXT_FRAME           I2C_NEXT_FRAME           ///< ((uint32_t)(I2C_RELOAD_MODE | I2C_SOFTEND_MODE))
#define BSP_I2C_FIRST_AND_LAST_FRAME I2C_FIRST_AND_LAST_FRAME ///< ((uint32_t)I2C_AUTOEND_MODE)
#define BSP_I2C_LAST_FRAME           I2C_LAST_FRAME           ///< ((uint32_t)I2C_AUTOEND_MODE)
#define BSP_I2C_LAST_FRAME_NO_STOP   I2C_LAST_FRAME_NO_STOP   ///< ((uint32_t)I2C_SOFTEND_MODE)


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

/**
 * @brief Function pointer type for I2C callback functions.
 */
typedef void (*BSP_I2CCBPtr_t)(void* params);

/**
 * @brief Enumeration for BSP I2C operations.
 *
 * Represents the various i2c-related operations available
 * within the BSP layer, abstracting HAL functions.
 */
typedef enum _BSP_I2COP_t {
    // I2C Blocking Mode
    BSP_I2C_MASTER_TRANSMIT,
    BSP_I2C_MASTER_RECEIVE,
    BSP_I2C_SLAVE_TRANSMIT,
    BSP_I2C_SLAVE_RECEIVE,
    BSP_I2C_MEM_WRITE,
    BSP_I2C_MEM_READ,

    // I2C Non-Blocking Mode with IT
    BSP_I2C_MASTER_TRANSMIT_IT,
    BSP_I2C_MASTER_RECEIVE_IT,
    BSP_I2C_MASTER_SEQ_TRANSMIT_IT,
    BSP_I2C_MASTER_SEQ_RECEIVE_IT,
    BSP_I2C_SLAVE_TRANSMIT_IT,
    BSP_I2C_SLAVE_RECEIVE_IT,
    BSP_I2C_SLAVE_SEQ_TRANSMIT_IT,
    BSP_I2C_SLAVE_SEQ_RECEIVE_IT,
    BSP_I2C_MEM_WRITE_IT,
    BSP_I2C_MEM_READ_IT,

    // I2C Non-Blocking Mode with DMA
    BSP_I2C_MASTER_TRANSMIT_DMA,
    BSP_I2C_MASTER_RECEIVE_DMA,
    BSP_I2C_MASTER_SEQ_TRANSMIT_DMA,
    BSP_I2C_MASTER_SEQ_RECEIVE_DMA,
      BSP_I2C_SLAVE_TRANSMIT_DMA,
    BSP_I2C_SLAVE_RECEIVE_DMA,
    BSP_I2C_SLAVE_SEQ_TRANSMIT_DMA,
    BSP_I2C_SLAVE_SEQ_RECEIVE_DMA,
    BSP_I2C_MEM_WRITE_DMA,
    BSP_I2C_MEM_READ_DMA,
} BSP_I2COP_t;

/**
 * @brief Enumeration for BSP I2C identifiers.
 * Starts from 1 to align with common STM32 naming (I2C1, I2C2, ...)
 */
typedef enum _BSP_I2C_t {
    BSP_I2C1 = 1,  ///< I2C 1 Identifier
    BSP_I2C2,      ///< I2C 2 Identifier
    BSP_I2C3,      ///< I2C 3 Identifier
    BSP_I2C4,      ///< I2C 4 Identifier
    BSP_I2C_COUNT
} BSP_I2C_t;

/**
 * @enum BSP_I2CCBType_t
 * @brief I2C callback types for various i2c events.
 */
typedef enum _BSP_I2CCBType_t {
    BSP_I2C_MASTER_TX_CPLT_CALLBACK,
    BSP_I2C_MASTER_RX_CPLT_CALLBACK,
    BSP_I2C_SLAVE_TX_CPLT_CALLBACK,
    BSP_I2C_SLAVE_RX_CPLT_CALLBACK,
    BSP_I2C_MEM_TX_CPLT_CALLBACK,
    BSP_I2C_MEM_RX_CPLT_CALLBACK,
    BSP_I2C_LISTEN_CPLT_CALLBACK,
    BSP_I2C_ERROR_CALLBACK,
    BSP_I2C_ABORT_CPLT_CALLBACK,
    BSP_I2C_CALLBACK_TYPE_COUNT,
} BSP_I2CCBType_t;

/**
 * @struct BSP_I2CMap_t
 * @brief Maps BSP I2C enumerations to their corresponding HAL I2C handles.
 */
typedef struct _BSP_I2CMap_t {
    BSP_I2C_t i2c;       ///< Enumeration of the i2c
    I2C_HandleTypeDef* handle;  ///< Pointer to the HAL I2C handle
} BSP_I2CMap_t;

#ifdef _USE_SEMAPHORE
typedef struct _BSP_I2CSemMap_t {
	BSP_I2C_t i2c;
	osSemaphoreId_t* BinSemHdlr;
	uint32_t timeout;
} BSP_I2CSemMap_t;
#endif /* _USE_SEMAPHORE */

/**
 * @struct BSP_I2CCB_t
 * @brief Manager BSP I2C Custom Callbacks and Parameters.
 */
typedef struct _BSP_I2CCB_t {
    BSP_I2CCBPtr_t callbacks[BSP_I2C_CALLBACK_TYPE_COUNT];
    void* params[BSP_I2C_CALLBACK_TYPE_COUNT];
} BSP_I2CCB_t;


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

/* ------------------- INITIALIZATION & DE-INITIALIZATION ------------------- */
BSP_StatusTypeDef_t BSP_InitI2C(BSP_I2C_t i2c);
BSP_StatusTypeDef_t BSP_DeInitI2C(BSP_I2C_t i2c);

/* ------------------- I2C DEVICE CHECK ------------------- */
BSP_StatusTypeDef_t BSP_IsDevReadyI2C(BSP_I2C_t i2c, uint16_t devAddress, uint32_t trials, uint32_t timeout);
BSP_StatusTypeDef_t BSP_GetStateI2C(BSP_I2C_t i2c);
BSP_StatusTypeDef_t BSP_GetModeI2C(BSP_I2C_t i2c);
BSP_StatusTypeDef_t BSP_GetErrorI2C(BSP_I2C_t i2c);

/* ------------------- BLOCKING MODE ------------------- */
// Master
BSP_StatusTypeDef_t BSP_I2CMasterBlock(BSP_I2C_t i2c, uint16_t devAddress, uint8_t* pData, uint16_t size, uint32_t timeout, BSP_I2COP_t operation);
// Slave
BSP_StatusTypeDef_t BSP_I2CSlaveBlock(BSP_I2C_t i2c, uint8_t* pData, uint16_t size, uint32_t timeout, BSP_I2COP_t operation);
// Mem
BSP_StatusTypeDef_t BSP_I2CMemBlock(BSP_I2C_t i2c, uint16_t devAddress, uint16_t memAddress, uint16_t memAddSize, uint8_t* pData, uint16_t size, uint32_t timeout, BSP_I2COP_t operation);

/* ------------------- NON-BLOCKING MODE WITH IT ------------------- */
// Master
BSP_StatusTypeDef_t BSP_I2CMasterIT(BSP_I2C_t i2c, uint16_t devAddress, uint8_t* pData, uint16_t size, BSP_I2COP_t operation);
BSP_StatusTypeDef_t BSP_I2CMasterSeqIT(BSP_I2C_t i2c, uint16_t devAddress, uint8_t* pData, uint16_t size, uint32_t xferOptions, BSP_I2COP_t operation);
BSP_StatusTypeDef_t BSP_AbortI2CMasterIT(BSP_I2C_t i2c, uint16_t devAddress);

// Slave
BSP_StatusTypeDef_t BSP_I2CSlaveIT(BSP_I2C_t i2c, uint8_t* pData, uint16_t size, BSP_I2COP_t operation);
BSP_StatusTypeDef_t BSP_I2CSlaveSeqIT(BSP_I2C_t i2c, uint8_t* pData, uint16_t size, uint32_t xferOptions, BSP_I2COP_t operation);

// Mem
BSP_StatusTypeDef_t BSP_I2CMemIT(BSP_I2C_t i2c, uint16_t devAddress, uint16_t memAddress, uint16_t memAddSize, uint8_t* pData, uint16_t size, BSP_I2COP_t operation);

/* ------------------- NON-BLOCKING MODE WITH DMA ------------------- */
// Master
BSP_StatusTypeDef_t BSP_I2CMasterDMA(BSP_I2C_t i2c, uint16_t devAddress, uint8_t* pData, uint16_t size, BSP_I2COP_t operation);
BSP_StatusTypeDef_t BSP_I2CMasterSeqDMA(BSP_I2C_t i2c, uint16_t devAddress, uint8_t* pData, uint16_t size, uint32_t xferOptions, BSP_I2COP_t operation);

// Slave
BSP_StatusTypeDef_t BSP_I2CSlaveDMA(BSP_I2C_t i2c, uint8_t* pData, uint16_t size, BSP_I2COP_t operation);
BSP_StatusTypeDef_t BSP_I2CSlaveSeqDMA(BSP_I2C_t i2c, uint8_t* pData, uint16_t size, uint32_t xferOptions, BSP_I2COP_t operation);

// Mem
BSP_StatusTypeDef_t BSP_I2CMemDMA(BSP_I2C_t i2c, uint16_t devAddress, uint16_t memAddress, uint16_t memAddSize, uint8_t* pData, uint16_t size, BSP_I2COP_t operation);

/* ------------------- I2C CALLBACKS ------------------- */
void BSP_SetI2CCB(BSP_I2C_t i2c, BSP_I2CCBType_t callbackType, BSP_I2CCBPtr_t callback, void* params);


#endif /* HAL_I2C_MODULE_ENABLED */

#endif /* BSP_I2C_INC_BSP_I2C_H_ */
