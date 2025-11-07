/**
 * @file bsp_usb_otg_fs.h
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics HW Team
 * @brief Board Support Package for USB OTG FS functionalities.
 * @ref um2217-description-of-stm32h7-hal-and-lowlayer-drivers-stmicroelectronics.pdf
 */

#ifndef BSP_USB_OTG_FS_INC_BSP_USB_OTG_FS_H_
#define BSP_USB_OTG_FS_INC_BSP_USB_OTG_FS_H_

#include "main.h"
#include "module.h"

/** @defgroup USB_OTG_FS USB_OTG_FS
  * @brief USB_OTG_FS HAL BSP module driver
  * @
  */
#ifdef USB_OTG_FS
#define BSP_USB_OTG_FS_MODULE_ENABLED

#include "../../../../../BSP/BSP_COMMON/Inc/bsp_common.h"

#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"
#include "stdarg.h"

#include "ring_buffer.h"				//ring-buffer used for received packet

#include "../../../../../BSP/USB_OTG_FS/ST_USB_Middleware_Library/USB_DEVICE/Inc/usb_device.h"
#include "../../../../../BSP/USB_OTG_FS/ST_USB_Middleware_Library/USB_DEVICE/Inc/usbd_core.h"
#include "../../../../../BSP/USB_OTG_FS/ST_USB_Middleware_Library/USB_DEVICE/Inc/usbd_desc.h"
#include "../../../../../BSP/USB_OTG_FS/ST_USB_Middleware_Library/USB_DEVICE/Inc/usbd_cdc.h"				//define USB cdc feature
#include "../../../../../BSP/USB_OTG_FS/ST_USB_Middleware_Library/USB_DEVICE/Inc/usbd_cdc_if.h"			//USB CDC functions, provided by ST middle-ware library

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
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

extern USBD_HandleTypeDef hUsbDeviceFS;
extern RingBufferStruct*  usbCDCringbuff;

extern RingBufferStruct usb_rx_buff[1];
extern uint8_t 		 	usb_rx_packet[1024];



/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

bool	 BSP_USBCDCInit(void);
bool	 BSP_USBCDCDeinit(void);

bool 	 BSP_USBCDCBufferRead(uint8_t* data, uint32_t length);						// USB data read from usb rx ring-buffer
uint8_t  BSP_USBCDCWrite(uint8_t* pData, uint32_t length);
uint32_t BSP_USBCDCBufferIsAvailable(void);

//uint8_t  BSP_USBCDCWrite(TIM_HandleTypeDef *ptimerSource, uint8_t* data, uint32_t length, uint32_t timeout_ms);		// USB data send using HAL_USB CDC Transmit
//uint8_t  BSP_USBCDCPrintf(uint8_t* data, ...);									// 'Printf' using the variable arguments, implemented on IOIF layer


#endif /* USB_OTG_FS */

#endif /* BSP_USB_OTG_FS_INC_BSP_USB_OTG_FS_H_ */
