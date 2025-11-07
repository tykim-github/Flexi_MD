/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "module.h"

/* ########################## WALKON5_CM_ENABLED ############################## */
#ifdef WALKON5_CM_ENABLED
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "am_comm_hdlr.h"
#include "dev_comm_hdlr.h"
// #include "data_ctrl.h"
#include "ext_dev_ctrl.h"
#include "gait_ctrl.h"
#include "system_ctrl.h"
#include "L30_whole_body_ctrl.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for AM_Comm_Hdlr */
osThreadId_t AM_Comm_HdlrHandle;
const osThreadAttr_t AM_Comm_Hdlr_attributes = {
  .name = "AM_Comm_Hdlr",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Whole_Body_Ctrl */
osThreadId_t Whole_Body_CtrlHandle;
const osThreadAttr_t Whole_Body_Ctrl_attributes = {
  .name = "Whole_Body_Ctrl",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Ext_Dev_Ctrl */
osThreadId_t Ext_Dev_CtrlHandle;
const osThreadAttr_t Ext_Dev_Ctrl_attributes = {
  .name = "Ext_Dev_Ctrl",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for System_Ctrl */
osThreadId_t System_CtrlHandle;
const osThreadAttr_t System_Ctrl_attributes = {
  .name = "System_Ctrl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Gait_Task */
osThreadId_t Gait_TaskHandle;
const osThreadAttr_t Gait_Task_attributes = {
  .name = "Gait_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for DataStore */
osThreadId_t DataStoreHandle;
const osThreadAttr_t DataStore_attributes = {
  .name = "DataStore",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BinSem_I2C1 */
osSemaphoreId_t BinSem_I2C1Handle;
const osSemaphoreAttr_t BinSem_I2C1_attributes = {
  .name = "BinSem_I2C1"
};
/* Definitions for BinSem_I2C2 */
osSemaphoreId_t BinSem_I2C2Handle;
const osSemaphoreAttr_t BinSem_I2C2_attributes = {
  .name = "BinSem_I2C2"
};
/* Definitions for BinSem_I2C3 */
osSemaphoreId_t BinSem_I2C3Handle;
const osSemaphoreAttr_t BinSem_I2C3_attributes = {
  .name = "BinSem_I2C3"
};
/* Definitions for BinSem_I2C4 */
osSemaphoreId_t BinSem_I2C4Handle;
const osSemaphoreAttr_t BinSem_I2C4_attributes = {
  .name = "BinSem_I2C4"
};
/* Definitions for BinSem_SPI1 */
osSemaphoreId_t BinSem_SPI1Handle;
const osSemaphoreAttr_t BinSem_SPI1_attributes = {
  .name = "BinSem_SPI1"
};
/* Definitions for BinSem_SPI2 */
osSemaphoreId_t BinSem_SPI2Handle;
const osSemaphoreAttr_t BinSem_SPI2_attributes = {
  .name = "BinSem_SPI2"
};
/* Definitions for BinSem_SPI3 */
osSemaphoreId_t BinSem_SPI3Handle;
const osSemaphoreAttr_t BinSem_SPI3_attributes = {
  .name = "BinSem_SPI3"
};
/* Definitions for BinSem_UART7 */
osSemaphoreId_t BinSem_UART7Handle;
const osSemaphoreAttr_t BinSem_UART7_attributes = {
  .name = "BinSem_UART7"
};
/* Definitions for BinSem_SDMMC1 */
osSemaphoreId_t BinSem_SDMMC1Handle;
const osSemaphoreAttr_t BinSem_SDMMC1_attributes = {
  .name = "BinSem_SDMMC1"
};
/* Definitions for BinSem_SAI1 */
osSemaphoreId_t BinSem_SAI1Handle;
const osSemaphoreAttr_t BinSem_SAI1_attributes = {
  .name = "BinSem_SAI1"
};
/* Definitions for BinSem_FDCAN1 */
osSemaphoreId_t BinSem_FDCAN1Handle;
const osSemaphoreAttr_t BinSem_FDCAN1_attributes = {
  .name = "BinSem_FDCAN1"
};
/* Definitions for BinSem_FDCAN2 */
osSemaphoreId_t BinSem_FDCAN2Handle;
const osSemaphoreAttr_t BinSem_FDCAN2_attributes = {
  .name = "BinSem_FDCAN2"
};
/* Definitions for BinSem_SPI4 */
osSemaphoreId_t BinSem_SPI4Handle;
const osSemaphoreAttr_t BinSem_SPI4_attributes = {
  .name = "BinSem_SPI4"
};
/* Definitions for BinSem_SPI5 */
osSemaphoreId_t BinSem_SPI5Handle;
const osSemaphoreAttr_t BinSem_SPI5_attributes = {
  .name = "BinSem_SPI5"
};
/* Definitions for BinSem_SPI6 */
osSemaphoreId_t BinSem_SPI6Handle;
const osSemaphoreAttr_t BinSem_SPI6_attributes = {
  .name = "BinSem_SPI6"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartAMCommHdlr(void *argument);
void StartWholeBodyCtrl(void *argument);
void StartExtDevCtrl(void *argument);
void StartSystemCtrl(void *argument);
void StartGaitTask(void *argument);
void Start_DataStore(void *argument);

extern void MX_LWIP_Init(void);
extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of BinSem_I2C1 */
  BinSem_I2C1Handle = osSemaphoreNew(1, 1, &BinSem_I2C1_attributes);

  /* creation of BinSem_I2C2 */
  BinSem_I2C2Handle = osSemaphoreNew(1, 1, &BinSem_I2C2_attributes);

  /* creation of BinSem_I2C3 */
  BinSem_I2C3Handle = osSemaphoreNew(1, 1, &BinSem_I2C3_attributes);

  /* creation of BinSem_I2C4 */
  BinSem_I2C4Handle = osSemaphoreNew(1, 1, &BinSem_I2C4_attributes);

  /* creation of BinSem_SPI1 */
  BinSem_SPI1Handle = osSemaphoreNew(1, 1, &BinSem_SPI1_attributes);

  /* creation of BinSem_SPI2 */
  BinSem_SPI2Handle = osSemaphoreNew(1, 1, &BinSem_SPI2_attributes);

  /* creation of BinSem_SPI3 */
  BinSem_SPI3Handle = osSemaphoreNew(1, 1, &BinSem_SPI3_attributes);

  /* creation of BinSem_UART7 */
  BinSem_UART7Handle = osSemaphoreNew(1, 1, &BinSem_UART7_attributes);

  /* creation of BinSem_SDMMC1 */
  BinSem_SDMMC1Handle = osSemaphoreNew(1, 1, &BinSem_SDMMC1_attributes);

  /* creation of BinSem_SAI1 */
  BinSem_SAI1Handle = osSemaphoreNew(1, 1, &BinSem_SAI1_attributes);

  /* creation of BinSem_FDCAN1 */
  BinSem_FDCAN1Handle = osSemaphoreNew(1, 1, &BinSem_FDCAN1_attributes);

  /* creation of BinSem_FDCAN2 */
  BinSem_FDCAN2Handle = osSemaphoreNew(1, 1, &BinSem_FDCAN2_attributes);

  /* creation of BinSem_SPI4 */
  BinSem_SPI4Handle = osSemaphoreNew(1, 1, &BinSem_SPI4_attributes);

  /* creation of BinSem_SPI5 */
  BinSem_SPI5Handle = osSemaphoreNew(1, 1, &BinSem_SPI5_attributes);

  /* creation of BinSem_SPI6 */
  BinSem_SPI6Handle = osSemaphoreNew(1, 1, &BinSem_SPI6_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of AM_Comm_Hdlr */
  AM_Comm_HdlrHandle = osThreadNew(StartAMCommHdlr, NULL, &AM_Comm_Hdlr_attributes);

  /* creation of Whole_Body_Ctrl */
  Whole_Body_CtrlHandle = osThreadNew(StartWholeBodyCtrl, NULL, &Whole_Body_Ctrl_attributes);

  /* creation of Ext_Dev_Ctrl */
  Ext_Dev_CtrlHandle = osThreadNew(StartExtDevCtrl, NULL, &Ext_Dev_Ctrl_attributes);

  /* creation of System_Ctrl */
  System_CtrlHandle = osThreadNew(StartSystemCtrl, NULL, &System_Ctrl_attributes);

  /* creation of Gait_Task */
  Gait_TaskHandle = osThreadNew(StartGaitTask, NULL, &Gait_Task_attributes);

  /* creation of DataStore */
  DataStoreHandle = osThreadNew(Start_DataStore, NULL, &DataStore_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartAMCommHdlr */
/**
  * @brief  Function implementing the AM_Comm_Hdlr thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartAMCommHdlr */
void StartAMCommHdlr(void *argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();

  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartAMCommHdlr */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartAMCommHdlr */
}

/* USER CODE BEGIN Header_StartWholeBodyCtrl */
/**
* @brief Function implementing the Whole_Body_Ctrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartWholeBodyCtrl */
void StartWholeBodyCtrl(void *argument)
{
  /* USER CODE BEGIN StartWholeBodyCtrl */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartWholeBodyCtrl */
}

/* USER CODE BEGIN Header_StartExtDevCtrl */
/**
* @brief Function implementing the Ext_Dev_Ctrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartExtDevCtrl */
void StartExtDevCtrl(void *argument)
{
  /* USER CODE BEGIN StartExtDevCtrl */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartExtDevCtrl */
}

/* USER CODE BEGIN Header_StartSystemCtrl */
/**
* @brief Function implementing the System_Ctrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSystemCtrl */
void StartSystemCtrl(void *argument)
{
  /* USER CODE BEGIN StartSystemCtrl */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartSystemCtrl */
}

/* USER CODE BEGIN Header_StartGaitTask */
/**
* @brief Function implementing the Gait_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGaitTask */
void StartGaitTask(void *argument)
{
  /* USER CODE BEGIN StartGaitTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartGaitTask */
}

/* USER CODE BEGIN Header_Start_DataStore */
/**
* @brief Function implementing the DataStore thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_DataStore */
void Start_DataStore(void *argument)
{
  /* USER CODE BEGIN Start_DataStore */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_DataStore */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

#endif /* WALKON5_CM_ENABLED */

/* ########################## L30_CM_ENABLED ############################## */
#ifdef L30_CM_ENABLED
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "am_comm_hdlr.h"
#include "dev_comm_hdlr.h"
#include "ext_dev_ctrl.h"
#include "gait_ctrl.h"
#include "system_ctrl.h"
#include "L30_whole_body_ctrl.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for System_Ctrl */
osThreadId_t System_CtrlHandle;
const osThreadAttr_t System_Ctrl_attributes = {
  .name = "System_Ctrl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Ext_Dev_Ctrl */
osThreadId_t Ext_Dev_CtrlHandle;
const osThreadAttr_t Ext_Dev_Ctrl_attributes = {
  .name = "Ext_Dev_Ctrl",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Gait_Ctrl */
osThreadId_t Gait_CtrlHandle;
const osThreadAttr_t Gait_Ctrl_attributes = {
  .name = "Gait_Ctrl",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Whole_Body_Ctrl */
osThreadId_t Whole_Body_CtrlHandle;
const osThreadAttr_t Whole_Body_Ctrl_attributes = {
  .name = "Whole_Body_Ctrl",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Dev_Comm_Hdlr */
osThreadId_t Dev_Comm_HdlrHandle;
const osThreadAttr_t Dev_Comm_Hdlr_attributes = {
  .name = "Dev_Comm_Hdlr",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for AM_Comm_Hdlr */
osThreadId_t AM_Comm_HdlrHandle;
const osThreadAttr_t AM_Comm_Hdlr_attributes = {
  .name = "AM_Comm_Hdlr",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BinSem_I2C1 */
osSemaphoreId_t BinSem_I2C1Handle;
const osSemaphoreAttr_t BinSem_I2C1_attributes = {
  .name = "BinSem_I2C1"
};
/* Definitions for BinSem_I2C2 */
osSemaphoreId_t BinSem_I2C2Handle;
const osSemaphoreAttr_t BinSem_I2C2_attributes = {
  .name = "BinSem_I2C2"
};
/* Definitions for BinSem_I2C3 */
osSemaphoreId_t BinSem_I2C3Handle;
const osSemaphoreAttr_t BinSem_I2C3_attributes = {
  .name = "BinSem_I2C3"
};
/* Definitions for BinSem_I2C4 */
osSemaphoreId_t BinSem_I2C4Handle;
const osSemaphoreAttr_t BinSem_I2C4_attributes = {
  .name = "BinSem_I2C4"
};
/* Definitions for BinSem_SPI1 */
osSemaphoreId_t BinSem_SPI1Handle;
const osSemaphoreAttr_t BinSem_SPI1_attributes = {
  .name = "BinSem_SPI1"
};
/* Definitions for BinSem_SPI2 */
osSemaphoreId_t BinSem_SPI2Handle;
const osSemaphoreAttr_t BinSem_SPI2_attributes = {
  .name = "BinSem_SPI2"
};
/* Definitions for BinSem_SPI3 */
osSemaphoreId_t BinSem_SPI3Handle;
const osSemaphoreAttr_t BinSem_SPI3_attributes = {
  .name = "BinSem_SPI3"
};
/* Definitions for BinSem_UART7 */
osSemaphoreId_t BinSem_UART7Handle;
const osSemaphoreAttr_t BinSem_UART7_attributes = {
  .name = "BinSem_UART7"
};
/* Definitions for BinSem_SDMMC1 */
osSemaphoreId_t BinSem_SDMMC1Handle;
const osSemaphoreAttr_t BinSem_SDMMC1_attributes = {
  .name = "BinSem_SDMMC1"
};
/* Definitions for BinSem_SAI1 */
osSemaphoreId_t BinSem_SAI1Handle;
const osSemaphoreAttr_t BinSem_SAI1_attributes = {
  .name = "BinSem_SAI1"
};
/* Definitions for BinSem_FDCAN1 */
osSemaphoreId_t BinSem_FDCAN1Handle;
const osSemaphoreAttr_t BinSem_FDCAN1_attributes = {
  .name = "BinSem_FDCAN1"
};
/* Definitions for BinSem_FDCAN2 */
osSemaphoreId_t BinSem_FDCAN2Handle;
const osSemaphoreAttr_t BinSem_FDCAN2_attributes = {
  .name = "BinSem_FDCAN2"
};
/* Definitions for BinSem_SPI4 */
osSemaphoreId_t BinSem_SPI4Handle;
const osSemaphoreAttr_t BinSem_SPI4_attributes = {
  .name = "BinSem_SPI4"
};
/* Definitions for BinSem_SPI5 */
osSemaphoreId_t BinSem_SPI5Handle;
const osSemaphoreAttr_t BinSem_SPI5_attributes = {
  .name = "BinSem_SPI5"
};
/* Definitions for BinSem_SPI6 */
osSemaphoreId_t BinSem_SPI6Handle;
const osSemaphoreAttr_t BinSem_SPI6_attributes = {
  .name = "BinSem_SPI6"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartSystemCtrl(void *argument);
void StartExtDevCtrl(void *argument);
void StartGaitCtrl(void *argument);
void StartWholeBodyCtrl(void *argument);
void StartDevCommHdlr(void *argument);
void StartAMCommHdlr(void *argument);

extern void MX_USB_DEVICE_Init(void);
extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of BinSem_I2C1 */
  BinSem_I2C1Handle = osSemaphoreNew(1, 1, &BinSem_I2C1_attributes);

  /* creation of BinSem_I2C2 */
  BinSem_I2C2Handle = osSemaphoreNew(1, 1, &BinSem_I2C2_attributes);

  /* creation of BinSem_I2C3 */
  BinSem_I2C3Handle = osSemaphoreNew(1, 1, &BinSem_I2C3_attributes);

  /* creation of BinSem_I2C4 */
  BinSem_I2C4Handle = osSemaphoreNew(1, 1, &BinSem_I2C4_attributes);

  /* creation of BinSem_SPI1 */
  BinSem_SPI1Handle = osSemaphoreNew(1, 1, &BinSem_SPI1_attributes);

  /* creation of BinSem_SPI2 */
  BinSem_SPI2Handle = osSemaphoreNew(1, 1, &BinSem_SPI2_attributes);

  /* creation of BinSem_SPI3 */
  BinSem_SPI3Handle = osSemaphoreNew(1, 1, &BinSem_SPI3_attributes);

  /* creation of BinSem_UART7 */
  BinSem_UART7Handle = osSemaphoreNew(1, 1, &BinSem_UART7_attributes);

  /* creation of BinSem_SDMMC1 */
  BinSem_SDMMC1Handle = osSemaphoreNew(1, 1, &BinSem_SDMMC1_attributes);

  /* creation of BinSem_SAI1 */
  BinSem_SAI1Handle = osSemaphoreNew(1, 1, &BinSem_SAI1_attributes);

  /* creation of BinSem_FDCAN1 */
  BinSem_FDCAN1Handle = osSemaphoreNew(1, 1, &BinSem_FDCAN1_attributes);

  /* creation of BinSem_FDCAN2 */
  BinSem_FDCAN2Handle = osSemaphoreNew(1, 1, &BinSem_FDCAN2_attributes);

  /* creation of BinSem_SPI4 */
  BinSem_SPI4Handle = osSemaphoreNew(1, 1, &BinSem_SPI4_attributes);

  /* creation of BinSem_SPI5 */
  BinSem_SPI5Handle = osSemaphoreNew(1, 1, &BinSem_SPI5_attributes);

  /* creation of BinSem_SPI6 */
  BinSem_SPI6Handle = osSemaphoreNew(1, 1, &BinSem_SPI6_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of System_Ctrl */
  System_CtrlHandle = osThreadNew(StartSystemCtrl, NULL, &System_Ctrl_attributes);

  /* creation of Ext_Dev_Ctrl */
  Ext_Dev_CtrlHandle = osThreadNew(StartExtDevCtrl, NULL, &Ext_Dev_Ctrl_attributes);

  /* creation of Gait_Ctrl */
  Gait_CtrlHandle = osThreadNew(StartGaitCtrl, NULL, &Gait_Ctrl_attributes);

  /* creation of Whole_Body_Ctrl */
  Whole_Body_CtrlHandle = osThreadNew(StartWholeBodyCtrl, NULL, &Whole_Body_Ctrl_attributes);

  /* creation of Dev_Comm_Hdlr */
  Dev_Comm_HdlrHandle = osThreadNew(StartDevCommHdlr, NULL, &Dev_Comm_Hdlr_attributes);

  /* creation of AM_Comm_Hdlr */
  AM_Comm_HdlrHandle = osThreadNew(StartAMCommHdlr, NULL, &AM_Comm_Hdlr_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartSystemCtrl */
/**
  * @brief  Function implementing the System_Ctrl thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartSystemCtrl */
void StartSystemCtrl(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN StartSystemCtrl */

  InitSysCtrl();
  /* Infinite loop */
  for(;;)
  {
    RunSysCtrl();
  }
  /* USER CODE END StartSystemCtrl */
}

/* USER CODE BEGIN Header_StartExtDevCtrl */
/**
* @brief Function implementing the Ext_Dev_Ctrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartExtDevCtrl */
void StartExtDevCtrl(void *argument)
{
  /* USER CODE BEGIN StartExtDevCtrl */
  InitExtDevCtrl();
  /* Infinite loop */
  for(;;)
  {
    RunExtDevCtrl();
  }
  /* USER CODE END StartExtDevCtrl */
}

/* USER CODE BEGIN Header_StartGaitCtrl */
/**
* @brief Function implementing the Gait_Ctrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGaitCtrl */
void StartGaitCtrl(void *argument)
{
  /* USER CODE BEGIN StartGaitCtrl */
  InitGaitCtrl();
  /* Infinite loop */
  for(;;)
  {
    RunGaitCtrl();
  }
  /* USER CODE END StartGaitCtrl */
}

/* USER CODE BEGIN Header_StartWholeBodyCtrl */
/**
* @brief Function implementing the Whole_Body_Ctrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartWholeBodyCtrl */
void StartWholeBodyCtrl(void *argument)
{
  /* USER CODE BEGIN StartWholeBodyCtrl */
  // L30_InitWholeBodyCtrl();
  /* Infinite loop */
  for(;;)
  {
    // L30_RunWholeBodyCtrl();
  }
  /* USER CODE END StartWholeBodyCtrl */
}

/* USER CODE BEGIN Header_StartDevCommHdlr */
/**
* @brief Function implementing the Dev_Comm_Hdlr thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDevCommHdlr */
void StartDevCommHdlr(void *argument)
{
  /* USER CODE BEGIN StartDevCommHdlr */
  // InitDevCommHdlr();
  /* Infinite loop */
  for(;;)
  {
    // RunDevCommHdlr();
  }
  /* USER CODE END StartDevCommHdlr */
}

/* USER CODE BEGIN Header_StartAMCommHdlr */
/**
* @brief Function implementing the AM_Comm_Hdlr thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAMCommHdlr */
void StartAMCommHdlr(void *argument)
{
  /* USER CODE BEGIN StartAMCommHdlr */
  // InitAMCommHdlr();
  /* Infinite loop */
  for(;;)
  {
    // RunAMCommHdlr();
  }
  /* USER CODE END StartAMCommHdlr */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

#endif /* L30_CM_ENABLED */

/* ########################## SUIT_MINICM_ENABLED ############################## */
#ifdef SUIT_MINICM_ENABLED
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ble_comm_hdlr.h"
#include "debug_ctrl.h"
#include "ext_dev_ctrl.h"
#include "gait_ctrl.h"
#include "system_ctrl.h"
#include "AS_whole_body_ctrl.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for System_Ctrl */
osThreadId_t System_CtrlHandle;
const osThreadAttr_t System_Ctrl_attributes = {
  .name = "System_Ctrl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Debug_Ctrl */
osThreadId_t Debug_CtrlHandle;
const osThreadAttr_t Debug_Ctrl_attributes = {
  .name = "Debug_Ctrl",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Ext_Dev_Ctrl */
osThreadId_t Ext_Dev_CtrlHandle;
const osThreadAttr_t Ext_Dev_Ctrl_attributes = {
  .name = "Ext_Dev_Ctrl",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Gait_Ctrl */
osThreadId_t Gait_CtrlHandle;
const osThreadAttr_t Gait_Ctrl_attributes = {
  .name = "Gait_Ctrl",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Whole_Body_Ctrl */
osThreadId_t Whole_Body_CtrlHandle;
const osThreadAttr_t Whole_Body_Ctrl_attributes = {
  .name = "Whole_Body_Ctrl",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for BLE_Comm_Hdlr */
osThreadId_t BLE_Comm_HdlrHandle;
const osThreadAttr_t BLE_Comm_Hdlr_attributes = {
  .name = "BLE_Comm_Hdlr",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BinSem_I2C1 */
osSemaphoreId_t BinSem_I2C1Handle;
const osSemaphoreAttr_t BinSem_I2C1_attributes = {
  .name = "BinSem_I2C1"
};
/* Definitions for BinSem_I2C2 */
osSemaphoreId_t BinSem_I2C2Handle;
const osSemaphoreAttr_t BinSem_I2C2_attributes = {
  .name = "BinSem_I2C2"
};
/* Definitions for BinSem_I2C3 */
osSemaphoreId_t BinSem_I2C3Handle;
const osSemaphoreAttr_t BinSem_I2C3_attributes = {
  .name = "BinSem_I2C3"
};
/* Definitions for BinSem_I2C4 */
osSemaphoreId_t BinSem_I2C4Handle;
const osSemaphoreAttr_t BinSem_I2C4_attributes = {
  .name = "BinSem_I2C4"
};
/* Definitions for BinSem_SPI1 */
osSemaphoreId_t BinSem_SPI1Handle;
const osSemaphoreAttr_t BinSem_SPI1_attributes = {
  .name = "BinSem_SPI1"
};
/* Definitions for BinSem_SPI2 */
osSemaphoreId_t BinSem_SPI2Handle;
const osSemaphoreAttr_t BinSem_SPI2_attributes = {
  .name = "BinSem_SPI2"
};
/* Definitions for BinSem_SPI3 */
osSemaphoreId_t BinSem_SPI3Handle;
const osSemaphoreAttr_t BinSem_SPI3_attributes = {
  .name = "BinSem_SPI3"
};
/* Definitions for BinSem_UART7 */
osSemaphoreId_t BinSem_UART7Handle;
const osSemaphoreAttr_t BinSem_UART7_attributes = {
  .name = "BinSem_UART7"
};
/* Definitions for BinSem_SDMMC1 */
osSemaphoreId_t BinSem_SDMMC1Handle;
const osSemaphoreAttr_t BinSem_SDMMC1_attributes = {
  .name = "BinSem_SDMMC1"
};
/* Definitions for BinSem_SAI1 */
osSemaphoreId_t BinSem_SAI1Handle;
const osSemaphoreAttr_t BinSem_SAI1_attributes = {
  .name = "BinSem_SAI1"
};
/* Definitions for BinSem_FDCAN1 */
osSemaphoreId_t BinSem_FDCAN1Handle;
const osSemaphoreAttr_t BinSem_FDCAN1_attributes = {
  .name = "BinSem_FDCAN1"
};
/* Definitions for BinSem_FDCAN2 */
osSemaphoreId_t BinSem_FDCAN2Handle;
const osSemaphoreAttr_t BinSem_FDCAN2_attributes = {
  .name = "BinSem_FDCAN2"
};
/* Definitions for BinSem_SPI4 */
osSemaphoreId_t BinSem_SPI4Handle;
const osSemaphoreAttr_t BinSem_SPI4_attributes = {
  .name = "BinSem_SPI4"
};
/* Definitions for BinSem_SPI5 */
osSemaphoreId_t BinSem_SPI5Handle;
const osSemaphoreAttr_t BinSem_SPI5_attributes = {
  .name = "BinSem_SPI5"
};
/* Definitions for BinSem_SPI6 */
osSemaphoreId_t BinSem_SPI6Handle;
const osSemaphoreAttr_t BinSem_SPI6_attributes = {
  .name = "BinSem_SPI6"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartSystemCtrl(void *argument);
void StartDebugCtrl(void *argument);
void StartExtDevCtrl(void *argument);
void StartGaitCtrl(void *argument);
void StartWholeBodyCtrl(void *argument);
void StartBLEComm(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of BinSem_I2C1 */
  BinSem_I2C1Handle = osSemaphoreNew(1, 1, &BinSem_I2C1_attributes);

  /* creation of BinSem_I2C2 */
  BinSem_I2C2Handle = osSemaphoreNew(1, 1, &BinSem_I2C2_attributes);

  /* creation of BinSem_I2C3 */
  BinSem_I2C3Handle = osSemaphoreNew(1, 1, &BinSem_I2C3_attributes);

  /* creation of BinSem_I2C4 */
  BinSem_I2C4Handle = osSemaphoreNew(1, 1, &BinSem_I2C4_attributes);

  /* creation of BinSem_SPI1 */
  BinSem_SPI1Handle = osSemaphoreNew(1, 1, &BinSem_SPI1_attributes);

  /* creation of BinSem_SPI2 */
  BinSem_SPI2Handle = osSemaphoreNew(1, 1, &BinSem_SPI2_attributes);

  /* creation of BinSem_SPI3 */
  BinSem_SPI3Handle = osSemaphoreNew(1, 1, &BinSem_SPI3_attributes);

  /* creation of BinSem_UART7 */
  BinSem_UART7Handle = osSemaphoreNew(1, 1, &BinSem_UART7_attributes);

  /* creation of BinSem_SDMMC1 */
  BinSem_SDMMC1Handle = osSemaphoreNew(1, 1, &BinSem_SDMMC1_attributes);

  /* creation of BinSem_SAI1 */
  BinSem_SAI1Handle = osSemaphoreNew(1, 1, &BinSem_SAI1_attributes);

  /* creation of BinSem_FDCAN1 */
  BinSem_FDCAN1Handle = osSemaphoreNew(1, 1, &BinSem_FDCAN1_attributes);

  /* creation of BinSem_FDCAN2 */
  BinSem_FDCAN2Handle = osSemaphoreNew(1, 1, &BinSem_FDCAN2_attributes);

  /* creation of BinSem_SPI4 */
  BinSem_SPI4Handle = osSemaphoreNew(1, 1, &BinSem_SPI4_attributes);

  /* creation of BinSem_SPI5 */
  BinSem_SPI5Handle = osSemaphoreNew(1, 1, &BinSem_SPI5_attributes);

  /* creation of BinSem_SPI6 */
  BinSem_SPI6Handle = osSemaphoreNew(1, 1, &BinSem_SPI6_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of System_Ctrl */
  System_CtrlHandle = osThreadNew(StartSystemCtrl, NULL, &System_Ctrl_attributes);

  /* creation of Debug_Ctrl */
  Debug_CtrlHandle = osThreadNew(StartDebugCtrl, NULL, &Debug_Ctrl_attributes);

  /* creation of Ext_Dev_Ctrl */
  Ext_Dev_CtrlHandle = osThreadNew(StartExtDevCtrl, NULL, &Ext_Dev_Ctrl_attributes);

  /* creation of Gait_Ctrl */
  Gait_CtrlHandle = osThreadNew(StartGaitCtrl, NULL, &Gait_Ctrl_attributes);

  /* creation of Whole_Body_Ctrl */
  Whole_Body_CtrlHandle = osThreadNew(StartWholeBodyCtrl, NULL, &Whole_Body_Ctrl_attributes);

  /* creation of BLE_Comm_Hdlr */
  BLE_Comm_HdlrHandle = osThreadNew(StartBLEComm, NULL, &BLE_Comm_Hdlr_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartSystemCtrl */
/**
  * @brief  Function implementing the System_Ctrl thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartSystemCtrl */
void StartSystemCtrl(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartSystemCtrl */
  InitSysCtrl();
  /* Infinite loop */
  for(;;)
  {
    RunSysCtrl();
  }
  /* USER CODE END StartSystemCtrl */
}

/* USER CODE BEGIN Header_StartDebugCtrl */
/**
* @brief Function implementing the Debug_Ctrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDebugCtrl */
void StartDebugCtrl(void *argument)
{
  /* USER CODE BEGIN StartDebugCtrl */
  InitDebugTask();
  /* Infinite loop */
  for(;;)
  {
    RunDebugTask();
  }
  /* USER CODE END StartDebugCtrl */
}

/* USER CODE BEGIN Header_StartExtDevCtrl */
/**
* @brief Function implementing the Ext_Dev_Ctrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartExtDevCtrl */
void StartExtDevCtrl(void *argument)
{
  /* USER CODE BEGIN StartExtDevCtrl */
  InitExtDevCtrl();
  /* Infinite loop */
  for(;;)
  {
    RunExtDevCtrl();
  }
  /* USER CODE END StartExtDevCtrl */
}

/* USER CODE BEGIN Header_StartGaitCtrl */
/**
* @brief Function implementing the Gait_Ctrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGaitCtrl */
void StartGaitCtrl(void *argument)
{
  /* USER CODE BEGIN StartGaitCtrl */
  InitGaitCtrl();
  /* Infinite loop */
  for(;;)
  {
    RunGaitCtrl();
  }
  /* USER CODE END StartGaitCtrl */
}

/* USER CODE BEGIN Header_StartWholeBodyCtrl */
/**
* @brief Function implementing the Whole_Body_Ctrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartWholeBodyCtrl */
void StartWholeBodyCtrl(void *argument)
{
  /* USER CODE BEGIN StartWholeBodyCtrl */
  AS_InitWholeBodyCtrl();
  /* Infinite loop */
  for(;;)
  {
    AS_RunWholeBodyCtrl();
  }
  /* USER CODE END StartWholeBodyCtrl */
}

/* USER CODE BEGIN Header_StartBLEComm */
/**
* @brief Function implementing the BLE_Comm_Hdlr thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBLEComm */
void StartBLEComm(void *argument)
{
  /* USER CODE BEGIN StartBLEComm */
  InitBLEComm();
  /* Infinite loop */
  for(;;)
  {
    RunBLEComm();
  }
  /* USER CODE END StartBLEComm */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

#endif /* SUIT_MINICM_ENABLED */

