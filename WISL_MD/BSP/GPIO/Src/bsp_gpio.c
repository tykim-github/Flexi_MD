/**
 * @file bsp_gpio.c
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics HW Team
 * @brief Board Support Package for GPIO functionalities.
 * @ref um2217-description-of-stm32h7-hal-and-lowlayer-drivers-stmicroelectronics.pdf
 */

#include "../../../../../BSP/GPIO/Inc/bsp_gpio.h"

/** @defgroup GPIO 
  * @brief GPIO HAL BSP module driver
  * @{
  */
#ifdef HAL_GPIO_MODULE_ENABLED

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

static bool bt_connection_status = false;

static BSP_GPIOPortMap_t bsp_gpioPortMap[BSP_GPIO_PORT_COUNT] = {
    {BSP_GPIO_PORT_A, GPIOA},       // GPIO A
    {BSP_GPIO_PORT_B, GPIOB},       // GPIO B
    {BSP_GPIO_PORT_C, GPIOC},       // GPIO C
    {BSP_GPIO_PORT_D, GPIOD},       // GPIO D
    {BSP_GPIO_PORT_E, GPIOE},       // GPIO E
    {BSP_GPIO_PORT_F, GPIOF},       // GPIO F
    {BSP_GPIO_PORT_G, GPIOG},       // GPIO G
    {BSP_GPIO_PORT_H, GPIOH},       // GPIO H
    {BSP_GPIO_PORT_I, GPIOI},       // GPIO I
#ifndef WIDM_ENABLED
    {BSP_GPIO_PORT_J, GPIOJ},       // GPIO J
    {BSP_GPIO_PORT_K, GPIOK},       // GPIO K
#endif
};

static BSP_GPIOCB_t bsp_gpioCB[BSP_GPIO_PIN_COUNT] = {0};


/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

static GPIO_TypeDef* FindGPIOHandle(BSP_GPIOPort_t gpioPort);
static void ExecuteGPIOCB(uint16_t gpioPin, BSP_GPIOCBType_t callbackType);

static uint8_t PinToIndex(uint16_t gpioPin);
static bool IsValidGPIOPin(uint16_t gpioPin);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

/* ------------------- INITIALIZATION & DE-INITIALIZATION ------------------- */
/**
 * @brief Initialize the GPIO peripheral.
 * @param gpioPort Identifier for the GPIO peripheral.
 * @param pGpioInit GPIO initialization settings.
 * @return HAL status.
 */
void BSP_InitGPIO(BSP_GPIOPort_t gpioPort, GPIO_InitTypeDef* pGpioInit)
{
    // TODO : Error Handling
    GPIO_TypeDef* GPIOx = FindGPIOHandle(gpioPort);

    if (!GPIOx) {
        return; // The specified GPIO handle is not valid
    }

    if (!pGpioInit) {
        return; // NULL pointer for pGpioInit
    }

    HAL_GPIO_Init(GPIOx, pGpioInit);
}

/**
 * @brief Deinitialize the GPIO peripheral.
 * @param gpioPort Identifier for the GPIO peripheral.
 * @param gpioPin Specifies the GPIO pins to be configured.
 * @return HAL status.
 */
void BSP_DeInitGPIO(BSP_GPIOPort_t gpioPort, uint16_t gpioPin)
{
    // TODO : Error Handling
    GPIO_TypeDef* GPIOx = FindGPIOHandle(gpioPort);

    if (!GPIOx) {
        return; // The specified GPIO handle is not valid
    }

    if (!IsValidGPIOPin(gpioPin)) {
        return;
    }

    HAL_GPIO_DeInit(GPIOx, gpioPin);
}

/* ------------------- GPIO OPERATION ------------------- */
/**
 * @brief Read the specified GPIO pin.
 * @param gpioPort Identifier for the GPIO peripheral.
 * @param gpioPin Specifies the GPIO pin to be read.
 * @param pinState State of the GPIO pin.
 * @param operation GPIO operation type.
 * @return Pin state.
 */
BSP_GPIOPinState_t BSP_ReadGPIOPin(BSP_GPIOPort_t gpioPort, uint16_t gpioPin)
{
	GPIO_TypeDef* GPIOx = FindGPIOHandle(gpioPort);

    if (!GPIOx) {
        return BSP_ERROR; // The specified GPIO handle is not valid
    }

    if (!IsValidGPIOPin(gpioPin)) {
        return BSP_ERROR;
    }

    return (BSP_GPIOPinState_t)HAL_GPIO_ReadPin(GPIOx, gpioPin);
}

/**
 * @brief Write to the specified GPIO pin.
 * @param gpioPort Identifier for the GPIO peripheral.
 * @param gpioPin Specifies the GPIO pin to be written to.
 * @param pinState State to be written to the GPIO pin.
 * @return HAL status.
 */
void BSP_WriteGPIOPin(BSP_GPIOPort_t gpioPort, uint16_t gpioPin, BSP_GPIOPinState_t pinState)
{
    // TODO : Error Handling
	GPIO_TypeDef* GPIOx = FindGPIOHandle(gpioPort);

    if (!GPIOx) {
        return; // The specified GPIO handle is not valid
    }

    if (!IsValidGPIOPin(gpioPin)) {
        return;
    }

    // Check if the specified pinState is valid
    if (pinState != BSP_GPIO_PIN_RESET && pinState != BSP_GPIO_PIN_SET) {
        return; // The specified pinState is not valid
    }

    HAL_GPIO_WritePin(GPIOx, gpioPin, pinState);
}

/**
 * @brief Toggle the specified GPIO pin.
 * @param gpioPort Identifier for the GPIO peripheral.
 * @param gpioPin Specifies the GPIO pin to be toggled.
 * @return HAL status.
 */
void BSP_ToggleGPIOPin(BSP_GPIOPort_t gpioPort, uint16_t gpioPin)
{
    // TODO : Error Handling
	GPIO_TypeDef* GPIOx = FindGPIOHandle(gpioPort);

    if (!GPIOx) {
        return; // The specified GPIO handle is not valid
    }

    if (!IsValidGPIOPin(gpioPin)) {
        return;
    }

    HAL_GPIO_TogglePin(GPIOx, gpioPin);
}

/**
 * @brief Lock the specified GPIO pin.
 * @param gpioPort Identifier for the GPIO peripheral.
 * @param gpioPin Specifies the GPIO pin to be locked.
 * @return HAL status.
 */
BSP_StatusTypeDef_t BSP_LockGPIOPin(BSP_GPIOPort_t gpioPort, uint16_t gpioPin)
{
	GPIO_TypeDef* GPIOx = FindGPIOHandle(gpioPort);

    if (!GPIOx) {
        return BSP_ERROR; // The specified GPIO handle is not valid
    }

    if (!IsValidGPIOPin(gpioPin)) {
        return BSP_ERROR;
    }

    BSP_StatusTypeDef_t status = BSP_OK;
    status = HAL_GPIO_LockPin(GPIOx, gpioPin);

    return status;
}

/* ------------------- GPIO CALLBACKS ------------------- */
/**
 * @brief Registers a custom callback for a specified GPIO port and callback type.
 *
 * @param gpioPort Enum value representing the GPIO port.
 * @param callbackType Enum value representing the type of callback (e.g., transmit complete, receive complete).
 * @param callback Function pointer to the custom callback function.
 * @param params Pointer to any parameters that the custom callback function may need.
 */
void BSP_SetGPIOCB(uint16_t gpioPin, BSP_GPIOCBType_t callbackType, BSP_GPIOCBPtr_t callback)
{
    if (!IsValidGPIOPin(gpioPin)) {
        return;
    }

    uint8_t index = PinToIndex(gpioPin);

    if (callbackType < BSP_GPIO_CALLBACK_TYPE_COUNT && callback) {
        bsp_gpioCB[index].callbacks[callbackType] = callback;
        bsp_gpioCB[index].gpioPins[callbackType] = gpioPin;
    }
}

/**
 * @brief Handle GPIO EXTI callback.
 * @param gpioPin Specifies the GPIO pin that generated the callback.
 */
void HAL_GPIO_EXTI_Callback(uint16_t gpioPin)
{
    UNUSED(gpioPin);

//    static uint8_t pin_state;

    // switch(gpioPin) {
    // 	case USB_OTG_VBUS_Pin: /* USB VBUS sensing */
    // 		HAL_GPIO_WritePin(USB_OTG_GPIO_GPIO_Port, USB_OTG_GPIO_Pin, HIGH);
    // 		break;
    // 	case BLE_BT_CONNECT_Pin: /* BLE check */
//    		pin_state = BSP_ReadGPIOPin(BSP_GPIO_PORT_E,BSP_GPIO_PIN_11);
//    		if (pin_state == HIGH) { //When BT is disconnected,
//    			bt_connection_status = false;
//    			BSP_WriteGPIOPin(BSP_GPIO_PORT_E, BSP_GPIO_PIN_11, BSP_GPIO_PIN_SET); //UART PD is high
//    //			Set_GPIOE_State(BLE_UART_EN_Pin, HIGH);
//    		} else if (pin_state == LOW) { //When BT is connected,
//    			bt_connection_status = true;
//    			BSP_WriteGPIOPin(BSP_GPIO_PORT_E, BSP_GPIO_PIN_11, BSP_GPIO_PIN_SET); //UART PD is low
//    //			Set_GPIOE_State(BLE_UART_EN_Pin, LOW);
//    		}
    // 		break;
    // 	case PB_SW_INT__Pin: /* Power reset */
    // 		HAL_GPIO_WritePin(PB_SW_CLR__GPIO_Port, PB_SW_CLR__Pin, LOW);
    // 		break;
    // 	default:
    // 		break;
    // }

    ExecuteGPIOCB(gpioPin, BSP_GPIO_EXTI_CALLBACK);
    
    // BSP_WriteGPIOPin(BSP_GPIO_PORT_A, BSP_GPIO_PIN_8, BSP_ReadGPIOPin(BSP_GPIO_PORT_A, gpioPin));
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/**
 * @brief Executes a custom callback for a specified GPIO port and callback type.
 *
 * @param hgpio HAL GPIO handle, used to find the corresponding GPIO port enum value.
 * @param callbackType Enum value representing the type of callback to execute.
 */
static void ExecuteGPIOCB(uint16_t gpioPin, BSP_GPIOCBType_t callbackType)
{
    if (!IsValidGPIOPin(gpioPin)) {
        return;
    }

    uint8_t index = PinToIndex(gpioPin);

    if (callbackType < BSP_GPIO_CALLBACK_TYPE_COUNT) {
        BSP_GPIOCBPtr_t callback = bsp_gpioCB[index].callbacks[callbackType];
        uint16_t params = bsp_gpioCB[index].gpioPins[callbackType];
        if (callback) {
            callback(params); // Executes the custom callback with the specified parameters
        }
    }
}

/**
 * @brief Find the corresponding GPIO handle for the provided GPIO enumeration.
 * @param gpioPort GPIO enumeration value.
 * @return A pointer to the corresponding GPIO type definition, or NULL if not found.
 */
static GPIO_TypeDef* FindGPIOHandle(BSP_GPIOPort_t gpioPort)
{
    for (int i = 0; i < ARRAY_SIZE(bsp_gpioPortMap); i++) {
        if (bsp_gpioPortMap[i].gpioPort == gpioPort) {
            return bsp_gpioPortMap[i].bsp_port;
        }
    }

    return NULL;
}

/**
 * @brief Convert a gpioPin value to its corresponding array index.
 * 
 * @param gpioPin The gpioPin value to be converted.
 * @return uint8_t The converted array index.
 */
static uint8_t PinToIndex(uint16_t gpioPin)
{
    return (uint8_t)log2(gpioPin);
}

/**
 * @brief Check if a given gpioPin value is valid.
 * 
 * A valid gpioPin is one where only a single bit is set 
 * (i.e., it's a power of 2) and it does not exceed the 
 * maximum defined gpioPin value (GPIO_PIN_15).
 * 
 * @param gpioPin The gpioPin value to be checked.
 * @return bool Returns true if the gpioPin is valid, false otherwise.
 */
static bool IsValidGPIOPin(uint16_t gpioPin)
{
    return gpioPin != 0 && (gpioPin & (gpioPin - 1)) == 0 && gpioPin <= GPIO_PIN_15;
}

bool Is_BT_connected(void)
{
//to do

	return bt_connection_status;
}

#endif /* HAL_GPIO_MODULE_ENABLED */
