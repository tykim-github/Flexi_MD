

#ifndef INTERFACES_IOIF_IOIF_COMMON_INC_IOIF_USB_COMMON_H_
#define INTERFACES_IOIF_IOIF_COMMON_INC_IOIF_USB_COMMON_H_

#include "../../../../BSP/GPIO/Inc/bsp_gpio.h"
#include "../../../../BSP/TIMER/Inc/bsp_tim.h"
#include "../../../../BSP/USB_OTG_FS/Inc/bsp_usb_otg_fs.h"
#include "../../../../BSP/USB_OTG_FS/ST_USB_Middleware_Library/USB_DEVICE/Inc/usbd_cdc_if.h"

/** @defgroup USB_OTG_FS USB_OTG_FS
  * @brief USB_OTG_FS BSP module driver
  * @{
  */
#ifdef BSP_USB_OTG_FS_MODULE_ENABLED

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

/*
typedef int (*IOIF_USBCBPtr_t)	(uint8_t* buf, uint32_t* len);

extern IOIF_USBCBPtr_t ioif_usbRxCBPtr;
*/


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef enum _IOIF_UsbState_t {
    IOIF_USB_PIN_RESET = BSP_GPIO_PIN_RESET,
    IOIF_USB_PIN_SET = BSP_GPIO_PIN_SET
} IOIF_UsbState_t;


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

//extern IOIF_USBCBPtr_t ioif_usbRxCBPtr;
//IOIF_USBCBPtr_t ioif_usbReadCBPrt = NULL;


/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void 				IOIF_SetUSBCB(void);
IOIF_UsbState_t 	IOIF_InitUSB(BSP_Tim_t timer);

uint32_t 			IOIF_USBBufferIsAvailable(void);
bool 	 			IOIF_USBBufferByteRead(uint8_t *pData, uint32_t length);
USBRXDataStruct_t*  IOIF_USBReadCB(USBRXDataStruct_t* USBRXData_t);
uint8_t  			IOIF_USBWrite(uint8_t* pData, uint32_t length, uint32_t timeout_ms);
uint8_t  			IOIF_USBPrintf(uint8_t* data, ...);


#endif /* BSP_USB_OTG_FS_MODULE_ENABLED */

#endif /* INTERFACES_IOIF_IOIF_COMMON_INC_IOIF_USB_COMMON_H_ */
