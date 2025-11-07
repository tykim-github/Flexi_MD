/**
 *-----------------------------------------------------------
 *              RTL8201F ETHERNET PHY DRIVER
 *-----------------------------------------------------------
 * @file rtl8201f.c
 * @date Created on: Aug 8, 2023
 * @author AngelRobotics HW Team
 * @brief RTL8201F Ethernet PHY Driver Implementation
 * 
 * This file provides the implementation of the application programming 
 * interface (API) for the RTL8201F Ethernet PHY. It includes the definitions 
 * for the functions, macros, and types required for configuring and accessing 
 * the RTL8201F device.
 * 
 * The provided functions allow the user to initialize, configure, and 
 * retrieve data from the RTL8201F device, following the specifications and 
 * guidelines described in the RTL8201F Datasheet.
 * 
 * @ref RTL8201F Datasheet
 */

#include "rtl8201f.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */




/**
 *-----------------------------------------------------------
 *              TYPE DEFINITioNS AND ENUMERATioNS
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
 *                     FUNCTioN PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes for this module.
 */

static void RTL8201F_Delay(RTL8201FObject_t *rtl8201f_obj, uint32_t time);
static RTL8201F RTL8201F_ReadReg(RTL8201FObject_t *rtl8201f_obj, uint16_t addr, uint32_t *pReadVal);
static RTL8201F RTL8201F_WriteReg(RTL8201FObject_t *rtl8201f_obj, uint16_t addr, uint32_t writeVal);
static RTL8201F RTL8201F_SetReg(RTL8201FObject_t *rtl8201f_obj, uint16_t addr, uint16_t mask, uint16_t flag);
static RTL8201F RTL8201F_SoftReset(RTL8201FObject_t *rtl8201f_obj, uint32_t timeout);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTioNS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

/**
 * @brief Registers the IO context for the RTL8201F device.
 * @param rtl8201f_obj Pointer to the RTL8201F object.
 * @param rtl8201f_ioctx Pointer to the IO context.
 * @return Registration status.
 */
RTL8201F RTL8201F_SetIoctx(RTL8201FObject_t *rtl8201f_obj, RTL8201FIOctx_t *rtl8201f_ioctx)
{
    // Validate parameters
    if (!rtl8201f_obj || !rtl8201f_ioctx->ReadReg || !rtl8201f_ioctx->WriteReg || !rtl8201f_ioctx->GetTick) {
        return RTL8201F_STATUS_ERROR;
    }

    // Register IO functions
    rtl8201f_obj->io.Init       = rtl8201f_ioctx->Init;
    rtl8201f_obj->io.DeInit     = rtl8201f_ioctx->DeInit;
    rtl8201f_obj->io.ReadReg    = rtl8201f_ioctx->ReadReg;
    rtl8201f_obj->io.WriteReg   = rtl8201f_ioctx->WriteReg;
    rtl8201f_obj->io.GetTick    = rtl8201f_ioctx->GetTick;

    return RTL8201F_STATUS_OK;
}

/**
 * @brief Initializes the RTL8201F device.
 * @param rtl8201f_obj Pointer to the RTL8201F object.
 * @return Initialization status.
 */
RTL8201F RTL8201F_Init(RTL8201FObject_t *rtl8201f_obj) 
{
    // Check for valid pointers and functions
    if (!rtl8201f_obj) {
        return RTL8201F_STATUS_ERROR;
    }

    uint32_t regVal = 0, id = 0;
    RTL8201F status = RTL8201F_STATUS_OK;

    // Initialize GPIO and clocks if not already initialized
    if (rtl8201f_obj->isInitialized == 0 && !rtl8201f_obj->io.Init) {
        rtl8201f_obj->io.Init();
    }

    rtl8201f_obj->devAddr = RTL8201F_PHY_ADDR;

    // Check PHY address
    status = RTL8201F_ReadReg(rtl8201f_obj, RTL8201F_PHYI1R, &regVal);
    id |= regVal;
    if (id != RTL8201F_PHYI1R_OUI_HIGH) {
        return RTL8201F_STATUS_ADDRESS_ERROR;
    }

    // Soft reset
    if ((status = RTL8201F_SoftReset(rtl8201f_obj, RTL8201F_SW_RESET_TIMEOUT)) != RTL8201F_STATUS_OK) return status;

    // Set Page 7
    if ((status = RTL8201F_SetReg(rtl8201f_obj, RTL8201F_PSR, RTL8201F_PSR_MASK, RTL8201F_PSR_P7)) != RTL8201F_STATUS_OK) return status;
    RTL8201F_Delay(rtl8201f_obj, RTL8201F_REG_SETTING_DELAY);

    // Set LED mode
    if ((status = RTL8201F_SetReg(rtl8201f_obj, RTL8201F_P7_LED, RTL8201F_P7_LED_MASK, RTL8201F_P7_LED_ACK_ALL_LINK_100)) != RTL8201F_STATUS_OK) return status;
    RTL8201F_Delay(rtl8201f_obj, RTL8201F_REG_SETTING_DELAY);

    // Set RMII mode
    if ((status = RTL8201F_SetReg(rtl8201f_obj, RTL8201F_P7_RMSR,
            RTL8201F_P7_RMSR_RMII_MODE_MASK | RTL8201F_P7_RMSR_TX_TIMING_MASK | RTL8201F_P7_RMSR_RX_TIMING_MASK,
            RTL8201F_P7_RMSR_SET_RMII_MODE | RTL8201F_P7_RMSR_TX_TIMING | RTL8201F_P7_RMSR_RX_TIMING)) != RTL8201F_STATUS_OK) return status;
    RTL8201F_Delay(rtl8201f_obj, RTL8201F_REG_SETTING_DELAY);

    // Set Page 0
    if ((status = RTL8201F_SetReg(rtl8201f_obj, RTL8201F_PSR, RTL8201F_PSR_MASK, RTL8201F_PSR_P0)) != RTL8201F_STATUS_OK) return status;

    // Delay for initialization completion
    RTL8201F_Delay(rtl8201f_obj, RTL8201F_INIT_TIMEOUT);
    rtl8201f_obj->isInitialized = 1;

    return status;
}

/**
 * @brief Retrieves the link state of the RTL8201F device.
 * @param rtl8201f_obj Pointer to the RTL8201F object.
 * @return Link state status.
 */
RTL8201F RTL8201F_GetLinkState(RTL8201FObject_t *rtl8201f_obj) 
{
    // Check for valid pointers and functions
    if (!rtl8201f_obj) {
        return RTL8201F_STATUS_ERROR;
    }
    
    uint32_t bsrVal = 0, bcrVal = 0;
    RTL8201F speed100 = 0, duplexFull = 0;
    RTL8201F status = RTL8201F_STATUS_OK;

    // Read Basic Status Register (BSR) twice to get the current link status
    status = RTL8201F_ReadReg(rtl8201f_obj, RTL8201F_BSR, &bsrVal);
    status = RTL8201F_ReadReg(rtl8201f_obj, RTL8201F_BSR, &bsrVal);

    // Check if the link status is down
    if ((status == RTL8201F_STATUS_OK) && ((bsrVal & RTL8201F_BSR_LINK_STATUS) == 0)) {
        return RTL8201F_STATUS_LINK_DOWN;
    }

    // Read Basic Control Register (BCR) to check auto-negotiation status
    status = RTL8201F_ReadReg(rtl8201f_obj, RTL8201F_BCR, &bcrVal);
    if (status != RTL8201F_STATUS_OK) return status;

    if ((bcrVal & RTL8201F_BCR_AUTONEGO_EN) != RTL8201F_BCR_AUTONEGO_EN) { // Auto-negotiation NOT enabled
        speed100 = (bcrVal & RTL8201F_BCR_SPEED_SELECT) == RTL8201F_BCR_SPEED_SELECT;
        duplexFull = (bcrVal & RTL8201F_BCR_DUPLEX_MODE) == RTL8201F_BCR_DUPLEX_MODE;

        // Determine link state based on speed and duplex mode
        if (speed100 && duplexFull) return RTL8201F_STATUS_100MBITS_FULLDUPLEX;
        if (speed100) return RTL8201F_STATUS_100MBITS_HALFDUPLEX;
        if (duplexFull) return RTL8201F_STATUS_10MBITS_FULLDUPLEX;
        return RTL8201F_STATUS_10MBITS_HALFDUPLEX;
    } else { // Auto-negotiation enabled
        // Check if auto-negotiation is complete
        if ((bsrVal & RTL8201F_BSR_AUTONEGO_CPLT) == 0) return RTL8201F_STATUS_AUTONEGO_NOTDONE;
        return RTL8201F_STATUS_100MBITS_FULLDUPLEX;
    }
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTioNS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/**
 * @brief Delays for a specified amount of time.
 * @param rtl8201f_obj Pointer to the RTL8201F object.
 * @param time The delay time in milliseconds.
 */
static void RTL8201F_Delay(RTL8201FObject_t *rtl8201f_obj, uint32_t time)
{
    uint32_t tickStart =  rtl8201f_obj->io.GetTick();
    while((rtl8201f_obj->io.GetTick() - tickStart) <= time) {}
}

/**
 * @brief Reads a register value.
 * @param rtl8201f_obj Pointer to the RTL8201F object.
 * @param addr Register address.
 * @param pReadVal Pointer to store the read value.
 * @return Status of the read operation.
 */
static RTL8201F RTL8201F_ReadReg(RTL8201FObject_t *rtl8201f_obj, uint16_t addr, uint32_t *pReadVal) 
{
    return (rtl8201f_obj->io.ReadReg(rtl8201f_obj->devAddr, addr, pReadVal) < 0)
        ? RTL8201F_STATUS_READ_ERROR : RTL8201F_STATUS_OK;
}

/**
 * @brief Writes a value to a register.
 * @param rtl8201f_obj Pointer to the RTL8201F object.
 * @param addr Register address.
 * @param writeVal Value to write.
 * @return Status of the write operation.
 */
static RTL8201F RTL8201F_WriteReg(RTL8201FObject_t *rtl8201f_obj, uint16_t addr, uint32_t writeVal)
{
    return (rtl8201f_obj->io.WriteReg(rtl8201f_obj->devAddr, addr, writeVal) < 0)
        ? RTL8201F_STATUS_WRITE_ERROR : RTL8201F_STATUS_OK;
}

/**
 * @brief Sets specific bits in a register.
 * @param rtl8201f_obj Pointer to the RTL8201F object.
 * @param addr Register address.
 * @param mask Mask to apply.
 * @param flag Values to set.
 * @return Status of the operation.
 */
static RTL8201F RTL8201F_SetReg(RTL8201FObject_t *rtl8201f_obj, uint16_t addr, uint16_t mask, uint16_t flag)
{
    uint32_t regVal;
    RTL8201F status = RTL8201F_ReadReg(rtl8201f_obj, addr, &regVal);

    if (status == RTL8201F_STATUS_OK) {
        regVal = (regVal & ~mask) | flag;
        status = RTL8201F_WriteReg(rtl8201f_obj, addr, regVal);
    }

    return status;
}

/**
 * @brief Performs a software reset.
 * @param rtl8201f_obj Pointer to the RTL8201F object.
 * @param timeout Timeout for the reset operation.
 * @return Status of the reset operation.
 */
static RTL8201F RTL8201F_SoftReset(RTL8201FObject_t *rtl8201f_obj, uint32_t timeout)
{
    uint32_t tickStart = 0, regVal = 0;
    RTL8201F status = RTL8201F_WriteReg(rtl8201f_obj, RTL8201F_BCR, RTL8201F_BCR_SOFT_RST);
    status = RTL8201F_ReadReg(rtl8201f_obj, RTL8201F_BCR, &regVal);

    if (status == RTL8201F_STATUS_OK) {
        tickStart = rtl8201f_obj->io.GetTick();
        do {
            if ((rtl8201f_obj->io.GetTick() - tickStart) > timeout) return RTL8201F_STATUS_RESET_TIMEOUT; // Check timeout
            status = RTL8201F_ReadReg(rtl8201f_obj, RTL8201F_BCR, &regVal);
        } while ((regVal & RTL8201F_BCR_SOFT_RST) && status == RTL8201F_STATUS_OK); // Wait until software reset is done
    }

    return status;
}
