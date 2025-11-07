/**
 *-----------------------------------------------------------
 *                MDBT48Q-AT BLE Slave Device
 *-----------------------------------------------------------
 * @file mdbt42q-at.h
 * @date Created on: Sep 19, 2023
 * @author AngelRobotics HW Team
 * @brief Code for the MDBT48Q-AT BLE Slave Device.
 *
 *
 * Refer to the MDBT48Q-AT datasheet and related documents for more information.
 * @ref MDBT48Q-AT Datasheet
 */

#ifndef MDBT48Q_AT_H_
#define MDBT48Q_AT_H_

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */


/* Write Command Definitions*/
#define BLE_SLAVE_WRITE_NAME 			"AT+NAMEA10SBS_1"
#define BLE_SLAVE_WRITE_RESET			"AT+RESET"
#define BLE_SLAVE_WRITE_ADV_START		"AT+ADVSTART"
#define BLE_SLAVE_WRITE_ADV_STOP		"AT+ADVSTOP"
#define BLE_SLAVE_WRITE_SLEEP			"AT+SLEEP"

#define BLE_SLAVE_WRITE_BAUD9600		"AT+BAUDRATE9600"
#define BLE_SLAVE_WRITE_BAUD57600		"AT+BAUDRATE57600"
#define BLE_SLAVE_WRITE_BAUD115200		"AT+BAUDRATE115200"

#define BLE_SLAVE_WRITE_PHY1MBPS		"AT+PHYMODE1MBPS"
#define BLE_SLAVE_WRITE_PHY2MBPS		"AT+PHYMODE2MBPS"

#define BLE_SLAVE_WRITE_SERIALNO		"AT+SERIALNO"
#define BLE_SLAVE_WRITE_DISCONNECT		"AT+DISCONNECT"
#define BLE_SLAVE_WRITE_DEFAULT			"AT+DEFAULT"
#define BLE_SLAVE_WRITE_MACADDR			"AT+MACADDR"

#define BLE_SLAVE_WRITE_ADV_LEDOFF		"AT+ADVPATTERN00000000"
#define BLE_SLAVE_WRITE_ADV_LEDON		"AT+ADVPATTERN01F401F4"			// 0.5msec on, 0.5msec off


/* Read Command Definitions */
#define BLE_SLAVE_READ_NAME				"AT?NAME"
#define BLE_SLAVE_READ_FW_VERSION		"AT?VERSION"
#define BLE_SLAVE_READ_MACADDR			"AT?MACADDR"
#define BLE_SLAVE_READ_BAUDRATE			"AT?BAUDRATE"

#define BLE_SLAVE_READ_PHYMODE			"AT?PHYMODE"

#define BLE_SLAVE_READ_CONN_INTV_MODE	"AT?CONNECTINTERVALMODE"	// To retrieve status of connection interval mode
#define BLE_SLAVE_READ_CONN_INDI		"AT?CONNECTINDICATOR"		// To retrieve logic of pin for BT-connecting indicator
#define BLE_SLAVE_READ_CONN_INTV_TIME	"AT?CONNECTINTERVALTIME"	// To retrieve value of connection interval time under Mode 2

#define BLE_SLAVE_READ_SERIALNO			"AT?SERIALNO"
#define BLE_SLAVE_READ_ALL_PARAM		"AT?ALLPARAMETERS"			// To retrieve value of all parameters


/* BLE IO State Machine */
#define BLE_IO_INIT						80
#define BLE_IO_READ_UART_TRANS			81
#define BLE_IO_UART_WRITE_DONE			82
#define BLE_IO_WRITE_RESET_DONE			83

#define BLE_IO_WAITING					90
#define BLE_IO_WAIT_50MS_DONE			91
#define BLE_IO_WAIT_100MS_DONE			92
#define BLE_IO_WAIT_350MS_DONE			93
#define BLE_IO_WAIT_600MS_DONE			94
#define BLE_IO_WAIT_1000MS_DONE			95
#define BLE_IO_WAIT_DONE				99
#define BLE_IO_COMPLETE					100


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

/* Definition of IO callback */


typedef enum{
	STATUS_LOW = 0,
	STATUS_HIGH = 1
} _GPIO_PIN_STATE;


typedef bool 			(*Write_GPIO_State_fptr) 	(_GPIO_PIN_STATE GPIO_set);		// Write GPIO
typedef uint32_t 		(*UART_GetBaudrate_fptr)  	(void);
typedef bool 			(*UART_SetBaudrate_fptr)	(uint32_t baudrate);
typedef uint8_t			(*UART_Read_fptr)			(void);
typedef bool			(*UART_Write_fptr)			(uint8_t* data, uint32_t length);
typedef uint32_t		(*UART_IsReady_fptr)		(void);
typedef uint8_t			(*IO_wait)					(uint32_t ms_wait);
typedef bool			(*HW_Default_GPIO_fptr)		(_GPIO_PIN_STATE GPIO_STAT);


typedef struct
{
	Write_GPIO_State_fptr 	BLE_UART_PD_GPIOWrite;
	UART_GetBaudrate_fptr	BLE_UART_Get_Baudrate;
	UART_SetBaudrate_fptr	BLE_UART_Set_Baudrate;
	UART_Read_fptr			BLE_UART_Read;
	UART_Write_fptr			BLE_UART_Write;
	UART_IsReady_fptr		BLE_UART_Ready;
	IO_wait					BLE_IO_Wait;
	HW_Default_GPIO_fptr	BLE_HW_Default;
}BLE_IO_CallbackStruct;

bool BLE_IO_RegisterCallback(BLE_IO_CallbackStruct* BLE_callback);			// Register Callback


/* BLE IO State Machine Structure*/

typedef struct
{
	uint8_t _BLE_state;
	uint8_t _IO_state;

} IO_state;


/* BLE Basic Function Definitions */

uint8_t 	MDBT42Q_Init(void);		/* Not defined */
bool 		MDBT42Q_Deinit(void);	/* Not defined */

IO_state* 	MDBT42Q_AT_Read(uint8_t* AT_READ_CMD, uint32_t size);
IO_state* 	MDBT42Q_AT_Write(uint8_t * AT_WRITE_CMD, uint32_t size, bool reset_active);
uint32_t 	MDBT42Q_ReceivedData(uint8_t uart_ch, uint8_t* AT_data, uint32_t length);
bool 		MDBT42Q_TransmitData(uint8_t* data, uint32_t length);

void		MDBT42Q_SetNodeName(uint8_t* name, uint8_t* data);
void 		MDBT42Q_SetNodeSerial(uint8_t* serialno, uint8_t* data);
IO_state* 	MDBT42Q_HW_Default(void);




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

/* Definition of IO callback Functions*/
bool BLE_IO_RegisterCallback(BLE_IO_CallbackStruct* BLE_callback);			// Register Callback

/* BLE Basic Function Definitions */
uint8_t 	MDBT42Q_Init(void);		/* Not defined */
bool 		MDBT42Q_Deinit(void);	/* Not defined */

IO_state* 	MDBT42Q_AT_Read(uint8_t* AT_READ_CMD, uint32_t size);
IO_state* 	MDBT42Q_AT_Write(uint8_t * AT_WRITE_CMD, uint32_t size, bool reset_active);
uint32_t 	MDBT42Q_ReceivedData(uint8_t uart_ch, uint8_t* AT_data, uint32_t length);
bool 		MDBT42Q_TransmitData(uint8_t* data, uint32_t length);

void		MDBT42Q_SetNodeName(uint8_t* name, uint8_t* data);
void 		MDBT42Q_SetNodeSerial(uint8_t* serialno, uint8_t* data);
IO_state* 	MDBT42Q_HW_Default(void);


#endif /* MDBT48Q_AT_H_ */
