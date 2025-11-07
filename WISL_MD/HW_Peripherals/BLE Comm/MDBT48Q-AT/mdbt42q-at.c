/**
 *-----------------------------------------------------------
 *                MDBT48Q-AT BLE Slave Device
 *-----------------------------------------------------------
 * @file mdbt42q-at.c
 * @date Created on: Sep 19, 2023
 * @author AngelRobotics HW Team
 * @brief Code for the MDBT48Q-AT BLE Slave Device.
 *
 *
 * Refer to the MDBT48Q-AT datasheet and related documents for more information.
 * @ref MDBT48Q-AT Datasheet
 */

#include "mdbt42q-at.h"

/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */

IO_state BLE_IO_state = {
		BLE_IO_INIT,			//BLE state initialization
		BLE_IO_WAITING,			//BLE IO wait state init.
};

static BLE_IO_CallbackStruct BLE_Callback_Struct;

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
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

uint8_t MDBT42Q_Init(void)
{
	uint8_t state = 0;
	return state;
}

bool MDBT42Q_Deinit(void)
{
	bool ret = false;
	return ret;
}

IO_state* MDBT42Q_AT_Read(uint8_t* AT_READ_CMD, uint32_t size)
{
	switch (BLE_IO_state._BLE_state)
	{
		case BLE_IO_INIT:
			BLE_Callback_Struct.BLE_UART_PD_GPIOWrite(STATUS_LOW);				//UART PD is low
			BLE_IO_state._IO_state = BLE_Callback_Struct.BLE_IO_Wait(5);		//UART PD is low 이후 반드시 50ms delay 이후 전송할 것! (이하 시 BLE module 에서 ack 를 제대로 보내지 않을때가 있음!)
			if (BLE_IO_state._IO_state == BLE_IO_WAIT_DONE)
			{
				BLE_IO_state._BLE_state = BLE_IO_WAIT_50MS_DONE;
				BLE_IO_state._IO_state  = BLE_IO_WAITING;						//IO waiting init.
			}
			break;

		case BLE_IO_WAIT_50MS_DONE:												// After 50ms IO wait
			if(BLE_Callback_Struct.BLE_UART_Write(&AT_READ_CMD[0], size) == true)					//Write Command
				BLE_IO_state._BLE_state = BLE_IO_READ_UART_TRANS;
			break;

		case BLE_IO_READ_UART_TRANS:											// After sending UART trasmit
			BLE_IO_state._IO_state = BLE_Callback_Struct.BLE_IO_Wait(35);
			if (BLE_IO_state._IO_state == BLE_IO_WAIT_DONE)
			{
				BLE_IO_state._BLE_state = BLE_IO_WAIT_350MS_DONE;
				BLE_IO_state._IO_state  = BLE_IO_WAITING;
			}
			break;

		case BLE_IO_WAIT_350MS_DONE:										// After 250ms IO wait
			BLE_Callback_Struct.BLE_UART_PD_GPIOWrite(STATUS_HIGH);			// UART PD is high
			BLE_IO_state._BLE_state = BLE_IO_COMPLETE;
			break;

		default:break;
	}
	return &BLE_IO_state;
}

IO_state* MDBT42Q_AT_Write(uint8_t *AT_WRITE_CMD, uint32_t size, bool reset_active)
{
	switch (BLE_IO_state._BLE_state)
	{
		case BLE_IO_INIT:
			BLE_Callback_Struct.BLE_UART_PD_GPIOWrite(STATUS_LOW);									//UART PD is low
			BLE_IO_state._IO_state = BLE_Callback_Struct.BLE_IO_Wait(5);
			if (BLE_IO_state._IO_state == BLE_IO_WAIT_DONE)
				BLE_IO_state._BLE_state = BLE_IO_WAIT_50MS_DONE;
			break;

		case BLE_IO_WAIT_50MS_DONE:
			if(BLE_Callback_Struct.BLE_UART_Write(AT_WRITE_CMD, size) == true)						//Write Command
			{
				BLE_IO_state._BLE_state = BLE_IO_UART_WRITE_DONE;
			}
			break;

		case BLE_IO_UART_WRITE_DONE:
			BLE_IO_state._IO_state = BLE_Callback_Struct.BLE_IO_Wait(35);						//Waiting at least 250ms after write command
			if (BLE_IO_state._IO_state == BLE_IO_WAIT_DONE)
					BLE_IO_state._BLE_state = BLE_IO_WAIT_350MS_DONE;
			break;

		case BLE_IO_WAIT_350MS_DONE:
			if(reset_active == true)
			{
				if(BLE_Callback_Struct.BLE_UART_Write((uint8_t*)BLE_SLAVE_WRITE_RESET, size) == true)
					BLE_IO_state._BLE_state = BLE_IO_WRITE_RESET_DONE;
			}
			else BLE_IO_state._BLE_state = BLE_IO_WAIT_1000MS_DONE;

			break;

		case BLE_IO_WRITE_RESET_DONE:
			BLE_IO_state._IO_state = BLE_Callback_Struct.BLE_IO_Wait(100);
			if (BLE_IO_state._IO_state == BLE_IO_WAIT_DONE)
				BLE_IO_state._BLE_state = BLE_IO_WAIT_1000MS_DONE;
			break;

		case BLE_IO_WAIT_1000MS_DONE:
			BLE_Callback_Struct.BLE_UART_PD_GPIOWrite(STATUS_HIGH);									//UART PD is high
			BLE_IO_state._BLE_state = BLE_IO_COMPLETE;
			break;

		default:break;
	}
	return &BLE_IO_state;
}

uint32_t MDBT42Q_ReceivedData(uint8_t uart_ch, uint8_t* AT_data, uint32_t length)
{
	uint32_t ret = 0;

	static uint32_t index 		  = 0;
	static uint32_t buffer_length = 0;


	buffer_length = BLE_Callback_Struct.BLE_UART_Ready();			//UART RX ring buffer 에 읽어올 데이터가 있는지 확인
	if(buffer_length <= 0)
		return false;

	AT_data[index] = BLE_Callback_Struct.BLE_UART_Read();
	//CDC_Transmit_FS(&AT_data[index], 1);							//Debugging USB output, 삭제 예정
	index++;

	if (index > length)
		index = 0;

	return ret = buffer_length;										//현재 사용가능한 buffer index 를 반환
}

bool MDBT42Q_TransmitData(uint8_t* data, uint32_t length)
{
	bool ret = false;

	ret = BLE_Callback_Struct.BLE_UART_Write(data, length);

	return ret;
}


void MDBT42Q_SetNodeName(uint8_t* name, uint8_t* data)
{
	uint32_t length = sizeof(BLE_SLAVE_WRITE_NAME);

	strncpy((char*)data, &BLE_SLAVE_WRITE_NAME[0], length);			//NodeName : AT+NAME
	strcpy((char*)data+(length-1), (char*)name);					//NodeName : AT+NAME+BLE_SLAVE_NODENAME

}


void MDBT42Q_SetNodeSerial(uint8_t* serialno, uint8_t* data)
{
	uint32_t length = sizeof(BLE_SLAVE_WRITE_SERIALNO);

	strncpy((char*)data, &BLE_SLAVE_WRITE_SERIALNO[0], length);		//NodeName : AT+SERIALNO
	strcpy((char*)data+(length-1), (char*)serialno);				//NodeName : AT+NAME+BLE_SLAVE_SERIAL

}


IO_state* MDBT42Q_HW_Default(void)
{
	switch (BLE_IO_state._BLE_state)
	{
		case BLE_IO_INIT:
			BLE_Callback_Struct.BLE_HW_Default(STATUS_LOW);									//UART PD is low
			BLE_IO_state._IO_state = BLE_Callback_Struct.BLE_IO_Wait(60);
			if (BLE_IO_state._IO_state == BLE_IO_WAIT_DONE)
				BLE_IO_state._BLE_state = BLE_IO_WAIT_600MS_DONE;
			break;

		case BLE_IO_WAIT_600MS_DONE:														//pulse low keep min. 600ms
			BLE_Callback_Struct.BLE_HW_Default(STATUS_HIGH);
			BLE_IO_state._IO_state = BLE_Callback_Struct.BLE_IO_Wait(100);
			if (BLE_IO_state._IO_state == BLE_IO_WAIT_DONE)
				BLE_IO_state._BLE_state = BLE_IO_WAIT_1000MS_DONE;
			break;

		case BLE_IO_WAIT_1000MS_DONE:														//wait for 1s.
			BLE_IO_state._BLE_state = BLE_IO_COMPLETE;
			break;

		default: break;
	}

	return &BLE_IO_state;
}


bool BLE_IO_RegisterCallback(BLE_IO_CallbackStruct* BLE_callback)	//Callback 함수 등록
{
	bool ret = true;

	if(!BLE_callback->BLE_IO_Wait || !BLE_callback->BLE_UART_Get_Baudrate || !BLE_callback->BLE_UART_PD_GPIOWrite || !BLE_callback->BLE_HW_Default \
			 || !BLE_callback->BLE_UART_Read || !BLE_callback->BLE_UART_Ready || !BLE_callback->BLE_UART_Set_Baudrate || !BLE_callback->BLE_UART_Write)
		return false;

	BLE_Callback_Struct.BLE_IO_Wait 			= BLE_callback->BLE_IO_Wait;
	BLE_Callback_Struct.BLE_UART_Get_Baudrate	= BLE_callback->BLE_UART_Get_Baudrate;
	BLE_Callback_Struct.BLE_UART_PD_GPIOWrite	= BLE_callback->BLE_UART_PD_GPIOWrite;
	BLE_Callback_Struct.BLE_UART_Read			= BLE_callback->BLE_UART_Read;
	BLE_Callback_Struct.BLE_UART_Ready			= BLE_callback->BLE_UART_Ready;
	BLE_Callback_Struct.BLE_UART_Set_Baudrate	= BLE_callback->BLE_UART_Set_Baudrate;
	BLE_Callback_Struct.BLE_UART_Write			= BLE_callback->BLE_UART_Write;
	BLE_Callback_Struct.BLE_HW_Default			= BLE_callback->BLE_HW_Default;

	return ret;
}



/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

