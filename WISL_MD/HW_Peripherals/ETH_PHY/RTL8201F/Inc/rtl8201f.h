/**
 *-----------------------------------------------------------
 *            RTL8201F ETHERNET PHY DRIVER HEADER
 *-----------------------------------------------------------
 * @file rtl8201f.h
 * @date Created on: Aug 8, 2023
 * @author AngelRobotics HW Team
 * @brief RTL8201F Ethernet PHY Driver Interface
 * 
 * This file defines the application programming interface (API) 
 * for the RTL8201F Ethernet PHY. It includes function prototypes, 
 * macros, and type definitions required for configuring and 
 * accessing the RTL8201F device.
 * 
 * @ref RTL8201F Datasheet
 */

#ifndef RTL8201F_H_
#define RTL8201F_H_

#include <stdio.h>
#include <stdint.h>

#include "rtl8201f_regmap.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

/* Status codes for RTL8201F */
#define RTL8201F_STATUS_READ_ERROR            ((uint8_t)-5) // Read error status
#define RTL8201F_STATUS_WRITE_ERROR           ((uint8_t)-4) // Write error status
#define RTL8201F_STATUS_ADDRESS_ERROR         ((uint8_t)-3) // Address error status
#define RTL8201F_STATUS_RESET_TIMEOUT         ((uint8_t)-2) // Reset timeout status
#define RTL8201F_STATUS_ERROR                 ((uint8_t)-1) // Generic error status
#define RTL8201F_STATUS_OK                    ((uint8_t) 0) // Success status
#define RTL8201F_STATUS_LINK_DOWN             ((uint8_t) 1) // Link down status
#define RTL8201F_STATUS_100MBITS_FULLDUPLEX   ((uint8_t) 2) // 100 Mbps Full Duplex status
#define RTL8201F_STATUS_100MBITS_HALFDUPLEX   ((uint8_t) 3) // 100 Mbps Half Duplex status
#define RTL8201F_STATUS_10MBITS_FULLDUPLEX    ((uint8_t) 4) // 10 Mbps Full Duplex status
#define RTL8201F_STATUS_10MBITS_HALFDUPLEX    ((uint8_t) 5) // 10 Mbps Half Duplex status
#define RTL8201F_STATUS_AUTONEGO_NOTDONE      ((uint8_t) 6) // Auto-Negotiation not done status

/* Timeouts and delays for RTL8201F */
#define RTL8201F_SW_RESET_TIMEOUT  ((uint32_t)500U)  // Software Reset timeout
#define RTL8201F_REG_SETTING_DELAY ((uint32_t)200U)  // Register setting delay
#define RTL8201F_INIT_TIMEOUT      ((uint32_t)2000U) // Initialization timeout
#define RTL8201F_PHY_ADDR          ((uint32_t)0b00U) // PHY Address


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

/* Type definition for RTL8201F */
typedef uint8_t RTL8201F;

/* Function pointer typedefs */
typedef uint8_t (*RTL8201F_InitFunc)     (void);
typedef uint8_t (*RTL8201F_DeInitFunc)   (void);
typedef uint8_t (*RTL8201F_ReadRegFunc)  (uint32_t, uint32_t, uint32_t *);
typedef uint8_t (*RTL8201F_WriteRegFunc) (uint32_t, uint32_t, uint32_t);
typedef uint8_t (*RTL8201F_GetTickFunc)  (void);

/* IO context structure for RTL8201F */
typedef struct _RTL8201FIOctx_t {
    RTL8201F_InitFunc      Init;       // Initialization function
    RTL8201F_DeInitFunc    DeInit;     // De-Initialization function
    RTL8201F_ReadRegFunc   ReadReg;    // Read register function
    RTL8201F_WriteRegFunc  WriteReg;   // Write register function
    RTL8201F_GetTickFunc   GetTick;    // Get tick function
} RTL8201FIOctx_t;

/* Object structure for RTL8201F */
typedef struct _RTL8201FObject_t {
    uint32_t devAddr;               // Device address
    uint32_t isInitialized;         // Initialization status
    void *pData;                    // User data
    RTL8201FIOctx_t io;             // IO context
} RTL8201FObject_t;


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

// Set the I/O context for the RTL8201F object.
RTL8201F RTL8201F_SetIoctx(RTL8201FObject_t *rtl8201f_obj, RTL8201FIOctx_t *rtl8201f_ioctx);

// Initialize the RTL8201F object.
RTL8201F RTL8201F_Init(RTL8201FObject_t *rtl8201f_obj);

// Get Link State the RTL8201F object.
RTL8201F RTL8201F_GetLinkState(RTL8201FObject_t *rtl8201f_obj);


#endif /* RTL8201F_H_ */
