

#include "ioif_usb_common.h"

/** @defgroup USB_OTG_FS USB_OTG_FS
  * @brief USB_OTG_FS BSP module driver
  * @{
  */
#ifdef BSP_USB_OTG_FS_MODULE_ENABLED

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

static BSP_Tim_t usb_timer_source;


/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

static void DetectUSBConnection(uint16_t gpioPin);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

void IOIF_SetUSBCB(void)
{
    BSP_SetGPIOCB(BSP_GPIO_PIN_9, BSP_GPIO_EXTI_CALLBACK, DetectUSBConnection);
}

IOIF_UsbState_t IOIF_InitUSB(BSP_Tim_t timer)
{
	usb_timer_source = timer;		//USB timer source init.

#ifdef L30_MD_REV06_ENABLED
    BSP_GPIOPinState_t vBusPin = BSP_ReadGPIOPin(BSP_GPIO_PORT_A, BSP_GPIO_PIN_9); // USB_OTG_FS_VBUS
    BSP_WriteGPIOPin(BSP_GPIO_PORT_A, BSP_GPIO_PIN_8, vBusPin);	// USB_OTG_FS_GPIO
#endif /* L30_MD_REV06_ENABLED */

#ifdef L30_MD_REV07_ENABLED
    BSP_GPIOPinState_t vBusPin = BSP_ReadGPIOPin(BSP_GPIO_PORT_A, BSP_GPIO_PIN_9); // USB_OTG_FS_VBUS
    BSP_WriteGPIOPin(BSP_GPIO_PORT_A, BSP_GPIO_PIN_15, vBusPin);	// USB_OTG_FS_GPIO
#endif /* L30_MD_REV07_ENABLED */

#ifdef L30_MD_REV08_ENABLED
    BSP_GPIOPinState_t vBusPin = BSP_ReadGPIOPin(BSP_GPIO_PORT_A, BSP_GPIO_PIN_9); // USB_OTG_FS_VBUS
    BSP_WriteGPIOPin(BSP_GPIO_PORT_A, BSP_GPIO_PIN_15, vBusPin);	// USB_OTG_FS_GPIO
#endif /* L30_MD_REV08_ENABLED */

#ifdef SUIT_MD_ENABLED
    BSP_GPIOPinState_t vBusPin = BSP_ReadGPIOPin(BSP_GPIO_PORT_A, BSP_GPIO_PIN_9); // USB_OTG_FS_VBUS
    BSP_WriteGPIOPin(BSP_GPIO_PORT_A, BSP_GPIO_PIN_8, vBusPin);	// USB_OTG_FS_GPIO
#endif /* SUIT_MD_ENABLED */


#ifdef WALKON5_CM_ENABLED
    USB_SetRXCB(IOIF_USBReadCB);	// Register USB CDC Read Callback
#endif
	
    BSP_USBCDCInit();				// BSP USB init.

	return vBusPin;
}

uint32_t IOIF_USBBufferIsAvailable(void)
{
	uint32_t length = 0;

	return length = BSP_USBCDCBufferIsAvailable();
}


bool IOIF_USBBufferByteRead(uint8_t *pData, uint32_t length)
{
	bool ret = false;

	return ret = BSP_USBCDCBufferRead(pData, length);
}

#if defined(WALKON5_CM_ENABLED)
USBRXDataStruct_t* IOIF_USBReadCB(USBRXDataStruct_t* USBRXData_t)
{
	return USBRXData_t;
}
#endif 

uint8_t IOIF_USBWrite(uint8_t* pData, uint32_t length, uint32_t timeout_ms)
{
	USBD_StatusTypeDef usb_tx_ret = USBD_BUSY;

	uint8_t ret = 0;
	uint8_t pre_time = 0;
	uint32_t current_time = 0;

	pre_time = BSP_GetCnt(usb_timer_source);		// get current timer counter

	while(usb_tx_ret)								// infinite loop to complete transmit
	{
		if(BSP_USBCDCWrite(pData, length) == USBD_OK)
			break;

		current_time = BSP_GetCnt(usb_timer_source);
		if(current_time - pre_time > timeout_ms)
		{
			ret = USBD_FAIL;
			break;
		}
	}
	return ret;

}

uint8_t IOIF_USBPrintf(uint8_t* data, ...)			// Variable Arguments
{

	uint8_t msg_buf[512];

	uint8_t ret;
	int32_t length;
	uint32_t timeout_ms = 100;

	va_list args;

	va_start(args, data);

	length = vsnprintf((char*)msg_buf, 512, (char*)data, args);
	ret = IOIF_USBWrite((uint8_t*)msg_buf, (int32_t)length, timeout_ms);

	va_end(args);

	return ret;
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

static void DetectUSBConnection(uint16_t gpioPin)
{
    BSP_WriteGPIOPin(BSP_GPIO_PORT_A, BSP_GPIO_PIN_8, BSP_ReadGPIOPin(BSP_GPIO_PORT_A, gpioPin));
}


#endif /* BSP_USB_OTG_FS_MODULE_ENABLED */
