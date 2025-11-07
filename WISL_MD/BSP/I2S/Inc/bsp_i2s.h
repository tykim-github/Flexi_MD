/**
 * @file bsp_spi.h
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics HW Team
 * @brief Board Support Package for SPI functionalities.
 * @ref um2217-description-of-stm32h7-hal-and-lowlayer-drivers-stmicroelectronics.pdf
 */

#ifndef BSP_I2S_INC_123_H_
#define BSP_I2S_INC_123_H_

#include "main.h"
#include "module.h"

/** @defgroup I2S I2S
  * @brief I2S HAL BSP module driver
  * @
  */
#ifdef HAL_I2S_MODULE_ENABLED
#define BSP_I2S_MODULE_ENABLED

#include "i2s.h"
#include "bsp_common.h"

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

/**
 * @brief Function pointer type for I2S callback functions.
 */
typedef void (*BSP_I2SCBPtr_t)(void* params);

/**
 * @brief Enumeration for BSP I2S operations.
 *
 * Represents the various i2s-related operations available
 * within the BSP layer, abstracting HAL functions.
 */
typedef enum _BSP_I2SOP_t {
    // I2S Blocking Mode
    BSP_I2S_TRANSMIT,
    BSP_I2S_RECEIVE,
    BSP_I2SEX_TRANSMITRECEIVE,

    // I2S Non-Blocking Mode with IT
    BSP_I2S_TRANSMIT_IT,
    BSP_I2S_RECEIVE_IT,
    BSP_I2SEX_TRANSMITRECEIVE_IT,

    // I2S Non-Blocking Mode with DMA
    BSP_I2S_TRANSMIT_DMA,
    BSP_I2S_RECEIVE_DMA,
    BSP_I2SEX_TRANSMITRECEIVE_DMA,
    BSP_I2S_DMAPAUSE,
    BSP_I2S_DMARESUME,
    BSP_I2S_DMASTOP,
} BSP_I2SOP_t;

/**
 * @brief Enumeration for BSP I2S identifiers.
 * Starts from 1 to align with common STM32 naming (I2S1, I2S2, ...)
 */
typedef enum _BSP_I2S_t {
    BSP_I2S1 = 1,  ///< I2S 1 Identifier
    BSP_I2S2,      ///< I2S 2 Identifier
    BSP_I2S3,      ///< I2S 3 Identifier
    BSP_I2S_COUNT
} BSP_I2S_t;

/**
 * @enum BSP_I2SCBType_t
 * @brief I2S callback types for various i2s events.
 */
typedef enum _BSP_I2SCBType_t {
    BSP_I2S_TX_HALF_CPLT_CALLBACK,
    BSP_I2S_TX_CPLT_CALLBACK,
    BSP_I2S_RX_HALF_CPLT_CALLBACK,
    BSP_I2S_RX_CPLT_CALLBACK,
    BSP_I2S_TXRX_HALF_CPLT_CALLBACK,
    BSP_I2S_TXRX_CPLT_CALLBACK,
    BSP_I2S_ERROR_CALLBACK,
    BSP_I2S_CALLBACK_TYPE_COUNT,
} BSP_I2SCBType_t;

/**
 * @struct BSP_I2SMap_t
 * @brief Maps BSP I2S enumerations to their corresponding HAL I2S handles.
 */
typedef struct _BSP_I2SMap_t {
    BSP_I2S_t i2s;       ///< Enumeration of the i2s
    I2S_HandleTypeDef* handle;  ///< Pointer to the HAL I2S handle
} BSP_I2SMap_t;

/**
 * @struct BSP_I2SCB_t
 * @brief Manager BSP I2S Custom Callbacks and Parameters.
 */
typedef struct _BSP_I2SCB_t {
    BSP_I2SCBPtr_t callbacks[BSP_I2S_CALLBACK_TYPE_COUNT];
    void* params[BSP_I2S_CALLBACK_TYPE_COUNT];
} BSP_I2SCB_t;


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
BSP_StatusTypeDef_t BSP_InitI2S(BSP_I2S_t i2s);
BSP_StatusTypeDef_t BSP_DeInitI2S(BSP_I2S_t i2s);

/* ------------------- I2S DEVICE CHECK ------------------- */
BSP_StatusTypeDef_t BSP_GetStateI2S(BSP_I2S_t i2s);
BSP_StatusTypeDef_t BSP_GetErrorI2S(BSP_I2S_t i2s);

/* ------------------- BLOCKING MODE ------------------- */
BSP_StatusTypeDef_t BSP_RunI2SBlock(BSP_I2S_t i2s, const uint16_t* pTxData, uint16_t* pRxData, uint16_t size, uint32_t timeout, BSP_I2SOP_t operation);

/* ------------------- NON-BLOCKING MODE WITH IT ------------------- */
BSP_StatusTypeDef_t BSP_RunI2SIT(BSP_I2S_t i2s, const uint16_t* pTxData, uint16_t* pRxData, uint16_t size, BSP_I2SOP_t operation);

/* ------------------- NON-BLOCKING MODE WITH DMA ------------------- */
BSP_StatusTypeDef_t BSP_RunI2SDMA(BSP_I2S_t i2s, const uint16_t* pTxData, uint16_t* pRxData, uint16_t size, BSP_I2SOP_t operation);
BSP_StatusTypeDef_t BSP_PauseI2SDMA(BSP_I2S_t i2s);
BSP_StatusTypeDef_t BSP_ResumeI2SDMA(BSP_I2S_t i2s);
BSP_StatusTypeDef_t BSP_StopI2SDMA(BSP_I2S_t i2s);

/* ------------------- I2S CALLBACKS ------------------- */
void BSP_SetI2SCB(BSP_I2S_t i2s, BSP_I2SCBType_t callbackType, BSP_I2SCBPtr_t callback, void* params);


#endif /* HAL_I2S_MODULE_ENABLED */

#endif /* BSP_I2S_INC_123_H_ */
