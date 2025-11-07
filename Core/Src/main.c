/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

#include "low_level_ctrl_task.h"
#include "mid_level_ctrl_task.h"
#include "system_ctrl_task.h"
#include "gait_ctrl_task.h"
#include "risk_mngr.h"

#include "../../WISL_MD/Apps/Risk_Mngr_Task/Inc/risk_mngr.h"
//#include "../../WISL_MD/Msg_Hdlr_Task/Inc/msg_hdlr_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define READ_FLASH_ARRAY_SIZE 8 // 32 Byte Aligned
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
void UploadProperties();
void SaveProperties();
void DownloadProperties();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t stackUsage;
uint8_t _estack;
uint8_t _Min_Stack_Size;

MainSequence_Enum MS_enum;


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)


{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */
/* ########################## SUIT_MD_ENABLED ############################## */
#ifdef SUIT_MD_ENABLED
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_ADC3_Init();
  MX_TIM16_Init();
  MX_TIM7_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_USB_DEVICE_Init();
  MX_I2C2_Init();
  MX_TIM15_Init();
  MX_I2C1_Init();
  MX_SPI3_Init();
  MX_TIM2_Init();
#endif

/* ########################## L30_MD_REV06_ENABLED ############################## */
#ifdef L30_MD_REV06_ENABLED
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_ADC3_Init();
  MX_ADC2_Init();
  MX_TIM16_Init();
  MX_TIM7_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_USB_DEVICE_Init();
  MX_I2C2_Init();
  MX_TIM15_Init();
  MX_I2C1_Init();
  MX_SPI3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
#endif /* L30_MD_REV06_ENABLED */

/* ########################## L30_MD_REV07_ENABLED ############################## */
#ifdef L30_MD_REV07_ENABLED
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_ADC3_Init();
  MX_ADC2_Init();
  MX_TIM16_Init();
  MX_TIM7_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_USB_DEVICE_Init();
  MX_I2C2_Init();
  MX_TIM15_Init();
  MX_I2C1_Init();
  MX_SPI3_Init();
  MX_TIM4_Init();
  MX_I2C3_Init();
  MX_I2C4_Init();
  MX_TIM2_Init();
#endif /* L30_MD_REV07_ENABLED */

/* ########################## L30_MD_REV08_ENABLED ############################## */
#ifdef L30_MD_REV08_ENABLED
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_ADC3_Init();
  MX_ADC2_Init();
  MX_TIM16_Init();
  MX_TIM7_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_USB_DEVICE_Init();
  MX_I2C2_Init();
  MX_TIM15_Init();
  MX_I2C1_Init();
  MX_SPI3_Init();
  MX_TIM4_Init();
  MX_I2C3_Init();
  MX_I2C4_Init();
  MX_TIM2_Init();
  MX_SPI5_Init();
#endif /* L30_MD_REV08_ENABLED */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */

  /* USER CODE BEGIN 2 */
  DownloadProperties();
  //SaveProperties();   // updated

  Create_PDOTable_ObjDictionary();
  Create_SDOTable_ObjDictionary();

  MS_enum = IDLE;

  InitSysMngtTask();
  InitMsgHdlr();
  InitMidLvCtrl();
  InitLowLvCtrl();
  InitGaitCtrl();
  InitRiskMngrCtrl();

#if defined(L30_MD_REV06_ENABLED) || defined (L30_MD_REV07_ENABLED) || defined (L30_MD_REV08_ENABLED)
  IOIF_WriteGPIOPin(IOIF_GPIO_PORT_E, LED_TEST_Pin, IOIF_GPIO_PIN_SET);
#endif

#ifdef SUIT_MD_ENABLED
  IOIF_WriteGPIOPin(IOIF_GPIO_PORT_E, STATUS_LED_B_Pin, IOIF_GPIO_PIN_SET);
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
     if(MS_enum == IDLE)                         continue;
    else if(MS_enum == UPLOAD_PROPERTIES)       UploadProperties();
    else if(MS_enum == SAVE_PROPERTIES)         SaveProperties();
    else if(MS_enum == DOWNLOAD_PROPERTIES)     DownloadProperties();
    else if(MS_enum == ELECTRICAL_SYSTEM_ID)    Cal_Elec_System_ID_Batch();
    else if(MS_enum == BEMF_ID)                 Send_Elec_BEMF_Value();
    else if(MS_enum == CURRENT_TRACKING_CHECK)  Send_Elec_Tracking_Data();
    else if(MS_enum == CURRENT_BANDWIDTH_CHECK) Send_Elec_Bandwidth_Data();
    else if(MS_enum == AUTO_TUNING)             Tune_Gain();
    else if(MS_enum == ADV_FRICTION_ID)         Send_Advanced_Friction_ID_Data();
    else if(MS_enum == CAL_FRICTION_LUT)        Cal_Friction_LUT();
    else                                        continue;
    MS_enum = IDLE;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 12;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void UploadProperties()
{
	uint8_t memory_isUsed = 0;
	uint8_t buf = 0;
	uint16_t t_identifier = 0;

	IOIF_ReadFlash(IOIF_FLASH_START_USER_ADDR, &memory_isUsed, MEMORY_SECOND_HAND_CHECK);

	if(memory_isUsed == 0xFF){	// First Use
		for(int i = 0; i<25000; ++i) {}

		t_identifier = GUI_SYNC|FIRST_USE;
		Send_MSG(t_identifier, 0, &buf);

		return;
	} else {
		uint32_t len = memory_isUsed + 1;
		IOIF_ReadFlash(IOIF_FLASH_START_USER_ADDR, &motor_properties.name_length, len);

		for(int i = 0; i < 25000; ++i) {}

		t_identifier = GUI_SYNC|SECOND_USE;
		Send_MSG(t_identifier, len, &motor_properties.name_length);
	}

	return;
}

void SaveProperties()
{
	uint32_t writeAddr = 0;
	float memArr1[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr2[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr3[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr4[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr5[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr6[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr7[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr8[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr9[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr10[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr11[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr12[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr13[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr14[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr15[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr16[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr17[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr18[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr19[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr20[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr21[READ_FLASH_ARRAY_SIZE] = {0};
	float memArr22[READ_FLASH_ARRAY_SIZE] = {0};

	IOIF_EraseFlash(IOIF_FLASH_START_USER_ADDR, IOIF_ERASE_ONE_SECTOR);
	writeAddr = IOIF_FLASH_START_USER_ADDR;

	/* Save Name */
	IOIF_WriteFlash(writeAddr, &motor_properties.name_length);
	writeAddr += 32;

	/* Save Motor Setting */
	memcpy(&memArr1[0], &motor_properties.pole_pair,     	   sizeof(motor_properties.pole_pair));
	memcpy(&memArr1[1], &inc25KhzObj.resolution,         	   sizeof(inc25KhzObj.resolution));
	memcpy(&memArr1[2], &motor_properties.gear_ratio,    	   sizeof(motor_properties.gear_ratio));
	memcpy(&memArr1[3], &motor_properties.Kt,            	   sizeof(motor_properties.Kt));
	memcpy(&memArr1[4], &motor_properties.Ke,            	   sizeof(motor_properties.Ke));
	memcpy(&memArr1[5], &motor_setting.peakCurr_limit,   	   sizeof(motor_setting.peakCurr_limit));
	memcpy(&memArr1[6], &motor_setting.contCurr_limit,         sizeof(motor_setting.contCurr_limit));
	memcpy(&memArr1[7], &motor_setting.max_velocity_rpm,       sizeof(motor_setting.max_velocity_rpm));
	IOIF_WriteFlash(writeAddr, memArr1);
	writeAddr += 32;

//	kf_current_object.kf_C = 48.323;
//	motor_properties.R = 1.006;
//	motor_properties.L = 0.0001669;
//	motor_setting.low_level_kalman_on = 0;

	memcpy(&memArr2[0], &motor_setting.commutation_set.done,   sizeof(motor_setting.commutation_set.done));
	memcpy(&memArr2[1], &motor_setting.commutation_set.cc_dir, sizeof(motor_setting.commutation_set.cc_dir));
	memcpy(&memArr2[2], &motor_setting.commutation_set.ma_dir, sizeof(motor_setting.commutation_set.ma_dir));
	memcpy(&memArr2[3], &motor_setting.commutation_set.ea_dir, sizeof(motor_setting.commutation_set.ea_dir));
	memcpy(&memArr2[4], &motor_properties.R,                   sizeof(motor_properties.R));
	memcpy(&memArr2[5], &motor_properties.L,                   sizeof(motor_properties.L));
	memcpy(&memArr2[6], &kf_current_object.kf_A,               sizeof(kf_current_object.kf_A));
	memcpy(&memArr2[7], &kf_current_object.kf_B,               sizeof(kf_current_object.kf_B));
	IOIF_WriteFlash(writeAddr, memArr2);
	writeAddr += 32;

	memcpy(&memArr3[0], &kf_current_object.kf_C,			   sizeof(kf_current_object.kf_C));
	memcpy(&memArr3[1], &motor_setting.low_level_kalman_on,    sizeof(motor_setting.low_level_kalman_on));
	memcpy(&memArr3[2], &motor_setting.currCtrl_BW_radPsec,    sizeof(motor_setting.currCtrl_BW_radPsec));
	memcpy(&memArr3[3], &motor_properties.J, 				   sizeof(motor_properties.J));
	memcpy(&memArr3[4], &motor_properties.B,                   sizeof(motor_properties.B));
	memcpy(&memArr3[5], &motor_properties.a,                   sizeof(motor_properties.a));
	memcpy(&memArr3[6], &motor_properties.b,                   sizeof(motor_properties.b));
	memcpy(&memArr3[7], &motor_properties.c,                   sizeof(motor_properties.c));
	IOIF_WriteFlash(writeAddr, memArr3);
	writeAddr += 32;

	memcpy(&memArr4[0], &motor_properties.d,                   sizeof(motor_properties.d));
	memcpy(&memArr4[1], &mid_ctrl_saturation,                  sizeof(mid_ctrl_saturation));
	memcpy(&memArr4[2], &velCtrl.Kp,                           sizeof(velCtrl.Kp));
	memcpy(&memArr4[3], &velCtrl.Ki,                           sizeof(velCtrl.Ki));
	memcpy(&memArr4[4], &velCtrl.Ctrl_BW_Hz,                   sizeof(velCtrl.Ctrl_BW_Hz));
	memcpy(&memArr4[5], &IRC.numerator_length,                 sizeof(IRC.numerator_length));
	memcpy(&memArr4[6], &IRC.denominator_length,               sizeof(IRC.denominator_length));
	memcpy(&memArr4[7], &IRC.saturation,                       sizeof(IRC.saturation));	// 4*6
	IOIF_WriteFlash(writeAddr, memArr4);
	writeAddr += 32;

	memcpy(&memArr5[0], IRC.irc_num,                           sizeof(IRC.irc_num));	// 4*6
	IOIF_WriteFlash(writeAddr, memArr5);
	writeAddr += 32;

	memcpy(&memArr6[0], IRC.irc_den,                           sizeof(IRC.irc_den));	// 4*6
	memcpy(&memArr6[6], &veObj.type,                           sizeof(veObj.type));
	memcpy(&memArr6[7], &posCtrl.Kp,                           sizeof(posCtrl.Kp));
	IOIF_WriteFlash(writeAddr, memArr6);
	writeAddr += 32;

	memcpy(&memArr7[0], &posCtrl.Kd, 						   sizeof(posCtrl.Kd));
	memcpy(&memArr7[1], &posCtrl.R,							   sizeof(posCtrl.R));
	memcpy(&memArr7[2], &AbsObj1.offset, 				       sizeof(AbsObj1.offset));
	memcpy(&memArr7[3], &AbsObj1.sign, 					       sizeof(AbsObj1.sign));
	memcpy(&memArr7[4], &VSD.lower_limit, 					   sizeof(VSD.lower_limit));
	memcpy(&memArr7[5], &VSD.upper_limit, 					   sizeof(VSD.upper_limit));
	memcpy(&memArr7[6], &VSD.lower_damped_range, 			   sizeof(VSD.lower_damped_range));
	memcpy(&memArr7[7], &VSD.upper_damped_range, 			   sizeof(VSD.upper_damped_range));
	IOIF_WriteFlash(writeAddr, memArr7);
	writeAddr += 32;

	memcpy(&memArr8[0], &VSD.lower_stiff_range, 			   sizeof(VSD.lower_stiff_range));
	memcpy(&memArr8[1], &VSD.upper_stiff_range, 			   sizeof(VSD.upper_stiff_range));
	memcpy(&memArr8[2], &VSD.lower_damper, 					   sizeof(VSD.lower_damper));
	memcpy(&memArr8[3], &VSD.upper_damper, 				       sizeof(VSD.upper_damper));
	memcpy(&memArr8[4], &VSD.lower_stiffness, 				   sizeof(VSD.lower_stiffness));
	memcpy(&memArr8[5], &VSD.upper_stiffness, 				   sizeof(VSD.upper_stiffness));
	memcpy(&memArr8[6], &VSD.saturation, 					   sizeof(VSD.saturation));
	memcpy(&memArr8[7], &VSD.saturation, 					   sizeof(VSD.saturation));
	IOIF_WriteFlash(writeAddr, memArr8);
	writeAddr += 32;

	memcpy(&memArr9[0], &posDOB.wc_Q, 				           sizeof(posDOB.wc_Q));
	memcpy(&memArr9[1], &posDOB.saturation, 		           sizeof(posDOB.saturation));
	memcpy(&memArr9[2], &posDOB.gq_num,				           sizeof(posDOB.gq_num));
	IOIF_WriteFlash(writeAddr, memArr9);
	writeAddr += 32;

	memcpy(&memArr10[0], &posDOB.gq_den, 			           sizeof(posDOB.gq_den));
	memcpy(&memArr10[6], &posFF.num_length, 			       sizeof(posFF.num_length));
	memcpy(&memArr10[7], &posFF.den_length,	                   sizeof(posFF.den_length));
	IOIF_WriteFlash(writeAddr, memArr10);
	writeAddr += 32;

	memcpy(&memArr11[0], &posDOB.q_num, 				       sizeof(posDOB.q_num));
	memcpy(&memArr11[4], &posDOB.q_den,		                   sizeof(posDOB.q_den));
	IOIF_WriteFlash(writeAddr, memArr11);
	writeAddr += 32;

	memcpy(&memArr12[0], posFF.num, 			               sizeof(posFF.num));		//4*5
	memcpy(&memArr12[4], posFF.den, 			               sizeof(posFF.den));		//4*5
	IOIF_WriteFlash(writeAddr, memArr12);
	writeAddr += 32;

	memcpy(&memArr13[0], &veObj.lpf_a, sizeof(veObj.lpf_a));
	memcpy(&memArr13[1], &veObj.lpf_b, sizeof(veObj.lpf_b));
	memcpy(&memArr13[2], &veObj.masking_size, sizeof(veObj.masking_size));
	memcpy(&memArr13[3], &veObj.leadlag_a, sizeof(veObj.leadlag_a));
	memcpy(&memArr13[4], &veObj.leadlag_b, sizeof(veObj.leadlag_b));
	memcpy(&memArr13[5], &posDOB.gq_num_length,  sizeof(posDOB.gq_num_length));
	memcpy(&memArr13[6], &posDOB.gq_den_length,  sizeof(posDOB.gq_den_length));
	memcpy(&memArr13[7], &posDOB.q_num_length, sizeof(posDOB.q_num_length));
	IOIF_WriteFlash(writeAddr, memArr13);
	writeAddr += 32;

	memcpy(&memArr14[0], &posDOB.q_den_length, sizeof(posDOB.q_den_length));
	memcpy(&memArr14[1], &advanced_friction_id.scaling_factor, sizeof(advanced_friction_id.scaling_factor));
	IOIF_WriteFlash(writeAddr, memArr14);
	writeAddr += 32;

	/**/
	for(int i = 0; i < (uint16_t)(FRICTION_LUT_SIZE/8); ++i){
		memcpy(memArr15, &advanced_friction_id.adv_friction_compensator_LUT[i*8], 4*8);
		IOIF_WriteFlash(writeAddr, memArr15);
		writeAddr += 32;
	}

	for(int i = 0; i < (((uint16_t)FRICTION_LUT_SIZE)%8); ++i){
		memcpy(&memArr16[i], &advanced_friction_id.adv_friction_compensator_LUT[((uint16_t)FRICTION_LUT_SIZE/8)*8 + i], 4);
	}
	IOIF_WriteFlash(writeAddr, memArr16);
	writeAddr += 32;

	memcpy(&memArr17[0], &motor_setting.commutation_set.hall_sensor_table[0],	sizeof(motor_setting.commutation_set.hall_sensor_table[0]));
	memcpy(&memArr17[1], &motor_setting.commutation_set.hall_sensor_table[1],	sizeof(motor_setting.commutation_set.hall_sensor_table[1]));
	memcpy(&memArr17[2], &motor_setting.commutation_set.hall_sensor_table[2],	sizeof(motor_setting.commutation_set.hall_sensor_table[2]));
	memcpy(&memArr17[3], &motor_setting.commutation_set.hall_sensor_table[3],	sizeof(motor_setting.commutation_set.hall_sensor_table[3]));
	memcpy(&memArr17[4], &motor_setting.commutation_set.hall_sensor_table[4],	sizeof(motor_setting.commutation_set.hall_sensor_table[4]));
	memcpy(&memArr17[5], &motor_setting.commutation_set.hall_sensor_table[5],	sizeof(motor_setting.commutation_set.hall_sensor_table[5]));
	memcpy(&memArr17[6], &motor_setting.commutation_set.hall_sensor_dir,	    sizeof(motor_setting.commutation_set.hall_sensor_dir));
	memcpy(&memArr17[7], &motor_setting.commutation_set.abs_encoder_offset,	    sizeof(motor_setting.commutation_set.abs_encoder_offset));
	IOIF_WriteFlash(writeAddr, memArr17);
	writeAddr += 32;

	memcpy(&memArr18[0], &motor_setting.sensor_setting.e_angle_homing_sensor,  	 sizeof(motor_setting.sensor_setting.e_angle_homing_sensor));
	memcpy(&memArr18[1], &motor_setting.sensor_setting.m_angle_homing_sensor,    sizeof(motor_setting.sensor_setting.m_angle_homing_sensor));
	memcpy(&memArr18[2], &motor_setting.sensor_setting.commutation_sensor,	     sizeof(motor_setting.sensor_setting.commutation_sensor));
	memcpy(&memArr18[3], &motor_setting.sensor_setting.pos_feedback_sensor,	     sizeof(motor_setting.sensor_setting.pos_feedback_sensor));
	memcpy(&memArr18[4], &motor_setting.sensor_setting.temperature_sensor_usage, sizeof(motor_setting.sensor_setting.temperature_sensor_usage));
	memcpy(&memArr18[5], &motor_setting.sensor_setting.imu_6axis_usage,	         sizeof(motor_setting.sensor_setting.imu_6axis_usage));
	memcpy(&memArr18[6], &motor_setting.sensor_setting.imu_3axis_usage,	         sizeof(motor_setting.sensor_setting.imu_3axis_usage));
	memcpy(&memArr18[7], &AbsObj1.location,	                                     sizeof(AbsObj1.location));
	IOIF_WriteFlash(writeAddr, memArr18);
	writeAddr += 32;

	memcpy(&memArr19[0], &AbsObj2.location,	                                     sizeof(AbsObj2.location));
	memcpy(&memArr19[1], &AbsObj2.offset,	                                     sizeof(AbsObj2.offset));
	memcpy(&memArr19[2], &AbsObj2.sign,	                                         sizeof(AbsObj2.sign));
	memcpy(&memArr19[3], &impedanceCtrl.Kp_max,	                                 sizeof(impedanceCtrl.Kp_max));
	memcpy(&memArr19[4], &impedanceCtrl.Kd_max,                                  sizeof(impedanceCtrl.Kd_max));
	memcpy(&memArr19[5], &impedanceCtrl.option,                                  sizeof(impedanceCtrl.option));
	memcpy(&memArr19[6], &impedanceCtrl.opt1_i_buffer.epsilon_target,            sizeof(impedanceCtrl.opt1_i_buffer.epsilon_target));
	memcpy(&memArr19[7], &impedanceCtrl.opt1_i_buffer.Kp_target,                 sizeof(impedanceCtrl.opt1_i_buffer.Kp_target));
	IOIF_WriteFlash(writeAddr, memArr19);
	writeAddr += 32;

	memcpy(&memArr20[0], &impedanceCtrl.opt1_i_buffer.Kd_target,                 sizeof(impedanceCtrl.opt1_i_buffer.Kd_target));
	memcpy(&memArr20[1], &impedanceCtrl.opt1_i_buffer.lambda_target,             sizeof(impedanceCtrl.opt1_i_buffer.lambda_target));
	memcpy(&memArr20[2], &impedanceCtrl.opt1_i_buffer.duration,                  sizeof(impedanceCtrl.opt1_i_buffer.duration));
 	memcpy(&memArr20[3], &vqfMagCalibObj.a11,                                    sizeof(vqfMagCalibObj.a11));
	memcpy(&memArr20[4], &vqfMagCalibObj.a12,                                    sizeof(vqfMagCalibObj.a12));
	memcpy(&memArr20[5], &vqfMagCalibObj.a13,                                    sizeof(vqfMagCalibObj.a13));
	memcpy(&memArr20[6], &vqfMagCalibObj.a21,                                    sizeof(vqfMagCalibObj.a21));
	memcpy(&memArr20[7], &vqfMagCalibObj.a22,                                    sizeof(vqfMagCalibObj.a22));
	IOIF_WriteFlash(writeAddr, memArr20);
	writeAddr += 32;

	memcpy(&memArr21[0], &vqfMagCalibObj.a23,                                    sizeof(vqfMagCalibObj.a23));
	memcpy(&memArr21[1], &vqfMagCalibObj.a31,                                    sizeof(vqfMagCalibObj.a31));
	memcpy(&memArr21[2], &vqfMagCalibObj.a32,                                    sizeof(vqfMagCalibObj.a32));
	memcpy(&memArr21[3], &vqfMagCalibObj.a33,                                    sizeof(vqfMagCalibObj.a33));
	memcpy(&memArr21[4], &vqfMagCalibObj.b1,                                     sizeof(vqfMagCalibObj.b1));
	memcpy(&memArr21[5], &vqfMagCalibObj.b2,                                     sizeof(vqfMagCalibObj.b2));
	memcpy(&memArr21[6], &vqfMagCalibObj.b3,                                     sizeof(vqfMagCalibObj.b3));
	memcpy(&memArr21[7], &vqfMagCalibObj.done,                                   sizeof(vqfMagCalibObj.done));
	IOIF_WriteFlash(writeAddr, memArr21);
	writeAddr += 32;

	memcpy(&memArr22[0], &motor_properties.md_version,                           sizeof(motor_properties.md_version));
	memcpy(&memArr22[1], &inc25KhzObj.prescaler,                                 sizeof(inc25KhzObj.prescaler));
	memcpy(&memArr22[2], &AbsObj1.module_id,                                     sizeof(AbsObj1.module_id));
	memcpy(&memArr22[3], &AbsObj2.module_id,                                     sizeof(AbsObj2.module_id));
	memcpy(&memArr22[4], &kf_current_object.BEMF_Comp_Gain,                      sizeof(kf_current_object.BEMF_Comp_Gain));
	memcpy(&memArr22[5], &kf_current_object.kf_Q,                                sizeof(kf_current_object.kf_Q));
	memcpy(&memArr22[6], &kf_current_object.kf_R,                                sizeof(kf_current_object.kf_R));

	IOIF_WriteFlash(writeAddr, memArr22);
	writeAddr += 32;
	/********************************************************************************************************************/
	Send_MSG((uint16_t)(GUI_SYNC|SAVE_DONE), 1, (uint8_t*)0);

	return;
}

void DownloadProperties()
{
  uint32_t readAddr = IOIF_FLASH_START_USER_ADDR;
  // IOIF_ReadFlash(readAddr, nameTestRes, IOIF_FLASH_READ_SIZE_32B);
  readAddr += 32;

  /* Download Motor Setting */
  IOIF_ReadFlash(readAddr, &motor_properties.pole_pair, 	IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &inc25KhzObj.resolution,         IOIF_FLASH_READ_SIZE_4B);
  IOIF_ReadFlash(readAddr, &inc1KhzObj.resolution, 	        IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &motor_properties.gear_ratio, 	IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &motor_properties.Kt, 	        IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &motor_properties.Ke,            IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &motor_setting.peakCurr_limit, 	IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &motor_setting.contCurr_limit, 	IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &motor_setting.max_velocity_rpm, IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;

  /* rpm 2 rad/s */
  motor_setting.max_velocity_radPsec = motor_setting.max_velocity_rpm*M_PI/30;

  for(int i = 0; i<500; ++i) {}

  IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.done,    IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.cc_dir,  IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.ma_dir,  IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.ea_dir,  IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &motor_properties.R,                    IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &motor_properties.L,                    IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &kf_current_object.kf_A,                IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &kf_current_object.kf_B,                IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;

  for(int i = 0; i<500; ++i) {}

  IOIF_ReadFlash(readAddr, &kf_current_object.kf_C,   IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &motor_setting.low_level_kalman_on,  IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &motor_setting.currCtrl_BW_radPsec,  IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &motor_properties.J,       IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &motor_properties.B,       IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &motor_properties.a,       IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &motor_properties.b,       IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &motor_properties.c,       IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B; //garbage

  for(int i = 0; i<500; ++i) {}

  IOIF_ReadFlash(readAddr, &motor_properties.d,       IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &mid_ctrl_saturation,      IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &velCtrl.Kp,               IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &velCtrl.Ki,               IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &velCtrl.Ctrl_BW_Hz,       IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &IRC.numerator_length,     IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &IRC.denominator_length,   IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &IRC.saturation,           IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;

  for(int i = 0; i<500; ++i) {}

  IOIF_ReadFlash(readAddr, IRC.irc_num, sizeof(IRC.irc_num)); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B * 8; // float 4 * 6

  for(int i = 0; i<500; ++i) {}

  IOIF_ReadFlash(readAddr, IRC.irc_den, sizeof(IRC.irc_den));     readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B * 6; // float 4 * 6
  IOIF_ReadFlash(readAddr, &veObj.type, IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &posCtrl.Kp, IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;

  for(int i = 0; i<500; ++i) {}

  IOIF_ReadFlash(readAddr, &posCtrl.Kd,             IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &posCtrl.R,              IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &AbsObj1.offset,         IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  if (isnanf(AbsObj1.offset)) AbsObj1.offset = 0;
  IOIF_ReadFlash(readAddr, &AbsObj1.sign,           IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  if (isnanf(AbsObj1.sign)) AbsObj1.sign = 1;
  IOIF_ReadFlash(readAddr, &VSD.lower_limit,        IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &VSD.upper_limit,        IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &VSD.lower_damped_range, IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &VSD.upper_damped_range, IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;

  for(int i = 0; i<500; ++i) {}

  IOIF_ReadFlash(readAddr, &VSD.lower_stiff_range,  IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &VSD.lower_stiff_range,  IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &VSD.lower_damper,       IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &VSD.upper_damper,       IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &VSD.lower_stiffness,    IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &VSD.upper_stiffness,    IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &VSD.saturation,         IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B * 2; // !need to 32 byte align

  for(int i = 0; i<500; ++i) {}

  IOIF_ReadFlash(readAddr, &posDOB.wc_Q,        IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &posDOB.saturation,  IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, posDOB.gq_num,       sizeof(posDOB.gq_num));   readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B * 6; // float 4 * 6

  for(int i = 0; i<500; ++i) {}

  IOIF_ReadFlash(readAddr, posDOB.gq_den,     sizeof(posDOB.gq_den));   readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B * 6; // float 4 * 6
  IOIF_ReadFlash(readAddr, &posFF.num_length, IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &posFF.den_length, IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;

  for(int i = 0; i<500; ++i) {}

  IOIF_ReadFlash(readAddr, posDOB.q_num, sizeof(posDOB.q_num)); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B * 4; // float 4 * 4
  IOIF_ReadFlash(readAddr, posDOB.q_den, sizeof(posDOB.q_den)); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B * 4; // float 4 * 4

  for(int i = 0; i<500; ++i){}

  IOIF_ReadFlash(readAddr, posFF.num, sizeof(posFF.num)); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B * 4; // float 4 * 4
  IOIF_ReadFlash(readAddr, posFF.den, sizeof(posFF.den)); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B * 4; // float 4 * 4

  for(int i = 0; i<500; ++i){}

  IOIF_ReadFlash(readAddr, &veObj.lpf_a,          IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &veObj.lpf_b,          IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &veObj.masking_size,   IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &veObj.leadlag_a,      IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &veObj.leadlag_b,      IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &posDOB.gq_num_length, IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &posDOB.gq_den_length, IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &posDOB.q_num_length,  IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;

  for(int i = 0; i<500; ++i) {}

  IOIF_ReadFlash(readAddr, &posDOB.q_den_length,                IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &advanced_friction_id.scaling_factor,IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B*7;

  /**/
  for(int i = 0; i<500; ++i) {}

  for(int i = 0; i < (uint16_t)(FRICTION_LUT_SIZE/8); ++i){
    IOIF_ReadFlash(readAddr, &advanced_friction_id.adv_friction_compensator_LUT[i*8],	IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &advanced_friction_id.adv_friction_compensator_LUT[i*8+1],	IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &advanced_friction_id.adv_friction_compensator_LUT[i*8+2],	IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &advanced_friction_id.adv_friction_compensator_LUT[i*8+3],	IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &advanced_friction_id.adv_friction_compensator_LUT[i*8+4],	IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &advanced_friction_id.adv_friction_compensator_LUT[i*8+5],	IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &advanced_friction_id.adv_friction_compensator_LUT[i*8+6],	IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &advanced_friction_id.adv_friction_compensator_LUT[i*8+7],	IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;

    for(int j = 0; j<500; ++j) {}
  }

  for(int i = 0; i < (((uint16_t)FRICTION_LUT_SIZE)%8); ++i) {
    IOIF_ReadFlash(readAddr, &advanced_friction_id.adv_friction_compensator_LUT[((uint16_t)FRICTION_LUT_SIZE/8)*8 + i], IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  }
  readAddr += (32 - (((uint16_t)FRICTION_LUT_SIZE)%8)*4);

  for(int i = 0; i<500; ++i){}

  IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.hall_sensor_table[0], IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.hall_sensor_table[1], IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.hall_sensor_table[2], IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.hall_sensor_table[3], IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.hall_sensor_table[4], IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.hall_sensor_table[5], IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.hall_sensor_dir,	    IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &motor_setting.commutation_set.abs_encoder_offset,   IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;

  for(int i = 0; i<500; ++i){}

  IOIF_ReadFlash(readAddr, &motor_setting.sensor_setting.e_angle_homing_sensor,    IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &motor_setting.sensor_setting.m_angle_homing_sensor,    IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &motor_setting.sensor_setting.commutation_sensor,       IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &motor_setting.sensor_setting.pos_feedback_sensor,      IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &motor_setting.sensor_setting.temperature_sensor_usage, IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &motor_setting.sensor_setting.imu_6axis_usage,          IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &motor_setting.sensor_setting.imu_3axis_usage,          IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &AbsObj1.location,                                      IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;

  for(int i = 0; i<500; ++i){}

  IOIF_ReadFlash(readAddr, &AbsObj2.location,                                      IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &AbsObj2.offset,                                        IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &AbsObj2.sign,                                          IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &impedanceCtrl.Kp_max,                                  IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &impedanceCtrl.Kd_max,                                  IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &impedanceCtrl.option,                                  IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &impedanceCtrl.opt1_i_buffer.epsilon_target,            IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &impedanceCtrl.opt1_i_buffer.Kp_target,                 IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;

  for(int i = 0; i<500; ++i){}

  IOIF_ReadFlash(readAddr, &impedanceCtrl.opt1_i_buffer.Kd_target,                 IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &impedanceCtrl.opt1_i_buffer.lambda_target,           IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &impedanceCtrl.opt1_i_buffer.duration,                IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &vqfMagCalibObj.a11,                                  IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &vqfMagCalibObj.a12,                                  IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &vqfMagCalibObj.a13,                                  IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &vqfMagCalibObj.a21,                                  IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &vqfMagCalibObj.a22,                                  IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;

    for(int i = 0; i<500; ++i){}

    IOIF_ReadFlash(readAddr, &vqfMagCalibObj.a23,                                  IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &vqfMagCalibObj.a31,                                  IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &vqfMagCalibObj.a32,                                  IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &vqfMagCalibObj.a33,                                  IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &vqfMagCalibObj.b1,                                   IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &vqfMagCalibObj.b2,                                   IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &vqfMagCalibObj.b3,                                   IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &vqfMagCalibObj.done,                                 IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;

    for(int i = 0; i<500; ++i){}

    IOIF_ReadFlash(readAddr, &motor_properties.md_version,                         IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &inc25KhzObj.prescaler,                               IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    if (inc25KhzObj.prescaler < 0)     inc25KhzObj.prescaler = 1;
    if (inc25KhzObj.prescaler > 8192)  inc25KhzObj.prescaler = 8192;
    if (isnanf(inc25KhzObj.prescaler)) inc25KhzObj.prescaler = 1;
    inc1KhzObj.prescaler = inc25KhzObj.prescaler;
    IOIF_ReadFlash(readAddr, &AbsObj1.module_id,                                   IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &AbsObj2.module_id,                                   IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &kf_current_object.BEMF_Comp_Gain,                    IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    if (isnanf(kf_current_object.BEMF_Comp_Gain)) kf_current_object.BEMF_Comp_Gain = 1;

    IOIF_ReadFlash(readAddr, &kf_current_object.kf_Q,                              IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    IOIF_ReadFlash(readAddr, &kf_current_object.kf_R,                              IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B*2;

    if (isnanf(kf_current_object.kf_Q)) kf_current_object.kf_Q = 1;
    if (isnanf(kf_current_object.kf_R)) kf_current_object.kf_R = 10;
    if (kf_current_object.kf_R == 0)    kf_current_object.kf_R = 10;

  /********************** Send Data for Synchronization With UI ***************************/
    uint16_t t_identifier;
      float temp_arr[16];
      uint8_t temp_arr_index = 0;

      // UI_MOTOR_PROPERTIES
      memcpy(&temp_arr[0], &motor_properties.pole_pair,             sizeof(motor_properties.pole_pair));           temp_arr_index++;
      memcpy(&temp_arr[1], &inc25KhzObj.resolution,                 sizeof(inc25KhzObj.resolution));               temp_arr_index++;
      memcpy(&temp_arr[2], &motor_properties.gear_ratio,            sizeof(motor_properties.gear_ratio));          temp_arr_index++;
      memcpy(&temp_arr[3], &motor_properties.Kt,          	        sizeof(motor_properties.Kt));                  temp_arr_index++;
      memcpy(&temp_arr[4], &motor_properties.Ke,                    sizeof(motor_properties.Ke));                  temp_arr_index++;
      memcpy(&temp_arr[5], &motor_setting.peakCurr_limit,           sizeof(motor_setting.peakCurr_limit));         temp_arr_index++;
      memcpy(&temp_arr[6], &motor_setting.contCurr_limit,           sizeof(motor_setting.contCurr_limit));         temp_arr_index++;
      memcpy(&temp_arr[7], &motor_setting.max_velocity_rpm,         sizeof(motor_setting.max_velocity_rpm));       temp_arr_index++;
      memcpy(&temp_arr[8], &motor_properties.md_version,            sizeof(motor_properties.md_version));          temp_arr_index++;

      for(int i = 0; i<500000; ++i){}

      t_identifier = GUI_SYNC|MEMORY_UI_MOTOR_PROPERTIES;
      Send_MSG(t_identifier, 4*temp_arr_index, (uint8_t*)temp_arr);
      temp_arr_index = 0;
      /****************************************************************************************************************************/

      // UI_SENSOR_SETTING
      memcpy(&temp_arr[0],  &motor_setting.sensor_setting.e_angle_homing_sensor,          sizeof(motor_setting.sensor_setting.e_angle_homing_sensor));    temp_arr_index++;
      memcpy(&temp_arr[1],  &motor_setting.sensor_setting.m_angle_homing_sensor,          sizeof(motor_setting.sensor_setting.m_angle_homing_sensor));    temp_arr_index++;
      memcpy(&temp_arr[2],  &motor_setting.sensor_setting.commutation_sensor,             sizeof(motor_setting.sensor_setting.commutation_sensor));       temp_arr_index++;
      memcpy(&temp_arr[3],  &motor_setting.sensor_setting.pos_feedback_sensor,            sizeof(motor_setting.sensor_setting.pos_feedback_sensor));      temp_arr_index++;
      memcpy(&temp_arr[4],  &motor_setting.sensor_setting.temperature_sensor_usage,       sizeof(motor_setting.sensor_setting.temperature_sensor_usage)); temp_arr_index++;
      memcpy(&temp_arr[5],  &motor_setting.sensor_setting.imu_6axis_usage,                sizeof(motor_setting.sensor_setting.imu_6axis_usage));          temp_arr_index++;
      memcpy(&temp_arr[6],  &motor_setting.sensor_setting.imu_3axis_usage,                sizeof(motor_setting.sensor_setting.imu_3axis_usage));          temp_arr_index++;
      memcpy(&temp_arr[7],  &AbsObj1.location,                                            sizeof(AbsObj1.location));                                      temp_arr_index++;
      memcpy(&temp_arr[8],  &AbsObj2.location,                                            sizeof(AbsObj2.location));                                      temp_arr_index++;
      memcpy(&temp_arr[9],  &inc25KhzObj.prescaler,                                       sizeof(inc25KhzObj.prescaler));                                 temp_arr_index++;
      memcpy(&temp_arr[10], &AbsObj1.module_id,                                           sizeof(AbsObj1.module_id));                                     temp_arr_index++;
      memcpy(&temp_arr[11], &AbsObj2.module_id,                                           sizeof(AbsObj2.module_id));                                     temp_arr_index++;

      for(int i = 0; i<500000; ++i){}

      t_identifier = GUI_SYNC|MEMORY_UI_SENSOR_SETTING;
      Send_MSG(t_identifier, 4*temp_arr_index, (uint8_t*)temp_arr);
      temp_arr_index = 0;
      /****************************************************************************************************************************/

      // ELECTRICAL PROPERTIES
      memcpy(&temp_arr[0], &motor_properties.R,                sizeof(motor_properties.R));                        temp_arr_index++;
      memcpy(&temp_arr[1], &motor_properties.L,                sizeof(motor_properties.L));                        temp_arr_index++;
      memcpy(&temp_arr[2], &kf_current_object.kf_C, 		   sizeof(kf_current_object.kf_C));                    temp_arr_index++;
      memcpy(&temp_arr[3], &motor_setting.currCtrl_BW_radPsec, sizeof(motor_setting.currCtrl_BW_radPsec));         temp_arr_index++;
      memcpy(&temp_arr[4], &kf_current_object.BEMF_Comp_Gain,  sizeof(kf_current_object.BEMF_Comp_Gain));          temp_arr_index++;
      memcpy(&temp_arr[5], &motor_setting.low_level_kalman_on, sizeof(motor_setting.low_level_kalman_on));         temp_arr_index++;
      memcpy(&temp_arr[6], &kf_current_object.kf_A,            sizeof(kf_current_object.kf_A));                    temp_arr_index++;
      memcpy(&temp_arr[7], &kf_current_object.kf_B, 		   sizeof(kf_current_object.kf_B));                    temp_arr_index++;
      memcpy(&temp_arr[8], &kf_current_object.kf_Q,            sizeof(kf_current_object.kf_Q));                    temp_arr_index++;
      memcpy(&temp_arr[9], &kf_current_object.kf_R, 		   sizeof(kf_current_object.kf_R));                    temp_arr_index++;

      for(int i = 0; i<500000; ++i){}

      t_identifier = GUI_SYNC|MEMORY_UI_ELECTRICAL_PROPERTIES;
      Send_MSG(t_identifier, 4*temp_arr_index, (uint8_t*)temp_arr);
      temp_arr_index = 0;
      /****************************************************************************************************************************/

      memcpy(&temp_arr[0], &motor_properties.J,       sizeof(motor_properties.J));                                 temp_arr_index++;
      memcpy(&temp_arr[1], &motor_properties.B,       sizeof(motor_properties.B));                                 temp_arr_index++;
      memcpy(&temp_arr[2], &motor_properties.a,       sizeof(motor_properties.a));                                 temp_arr_index++;
      memcpy(&temp_arr[3], &motor_properties.b,       sizeof(motor_properties.b));                                 temp_arr_index++;
      memcpy(&temp_arr[4], &mid_ctrl_saturation,      sizeof(mid_ctrl_saturation));                                temp_arr_index++;

      for(int i = 0; i<500000; ++i){}

      t_identifier = GUI_SYNC|MEMORY_UI_MECHANICAL_PROPERTIES;
      Send_MSG(t_identifier, 4*temp_arr_index, (uint8_t*)temp_arr);
      temp_arr_index = 0;
      /****************************************************************************************************************************/

      memcpy(&temp_arr[0],  &velCtrl.Kp,         sizeof(velCtrl.Kp));                                              temp_arr_index++;
      memcpy(&temp_arr[1],  &velCtrl.Ki,         sizeof(velCtrl.Ki));                                              temp_arr_index++;
      memcpy(&temp_arr[2],  &velCtrl.Ctrl_BW_Hz, sizeof(velCtrl.Ctrl_BW_Hz));                                      temp_arr_index++;
      memcpy(&temp_arr[3],  &posCtrl.Kp,         sizeof(posCtrl.Kp));                                              temp_arr_index++;
      memcpy(&temp_arr[4],  &posCtrl.Kd,         sizeof(posCtrl.Kd));                                              temp_arr_index++;
      memcpy(&temp_arr[5],  &posCtrl.R,          sizeof(posCtrl.R));                                               temp_arr_index++;
      memcpy(&temp_arr[6],  &posDOB.wc_Q,        sizeof(posDOB.wc_Q));                                             temp_arr_index++;
      memcpy(&temp_arr[7],  &posDOB.saturation,  sizeof(posDOB.saturation));                                       temp_arr_index++;
      memcpy(&temp_arr[8],  &veObj.type,         sizeof(veObj.type));                                              temp_arr_index++;
      memcpy(&temp_arr[9],  &veObj.masking_size, sizeof(veObj.masking_size));                                      temp_arr_index++;
      memcpy(&temp_arr[10], &IRC.saturation,     sizeof(IRC.saturation));                                          temp_arr_index++;
      memcpy(&temp_arr[11], &impedanceCtrl.Kp_max,   sizeof(impedanceCtrl.Kp_max));                                temp_arr_index++;
      memcpy(&temp_arr[12], &impedanceCtrl.Kd_max,   sizeof(impedanceCtrl.Kd_max));                                temp_arr_index++;
      memcpy(&temp_arr[13], &impedanceCtrl.option,   sizeof(impedanceCtrl.option));                                temp_arr_index++;

      for(int i = 0; i<500000; ++i){}

      t_identifier = GUI_SYNC|MEMORY_UI_CONTROL_PARAMETERS1;
      Send_MSG(t_identifier, 4*temp_arr_index, (uint8_t*)temp_arr);
      temp_arr_index = 0;
      /****************************************************************************************************************************/

      memcpy(&temp_arr[0],  &impedanceCtrl.opt1_i_buffer.epsilon_target,       sizeof(impedanceCtrl.opt1_i_buffer.epsilon_target));    temp_arr_index++;
      memcpy(&temp_arr[1],  &impedanceCtrl.opt1_i_buffer.Kp_target,            sizeof(impedanceCtrl.opt1_i_buffer.Kp_target));         temp_arr_index++;
      memcpy(&temp_arr[2],  &impedanceCtrl.opt1_i_buffer.Kd_target,            sizeof(impedanceCtrl.opt1_i_buffer.Kd_target));         temp_arr_index++;
      memcpy(&temp_arr[3],  &impedanceCtrl.opt1_i_buffer.lambda_target,        sizeof(impedanceCtrl.opt1_i_buffer.lambda_target));     temp_arr_index++;
      memcpy(&temp_arr[4],  &impedanceCtrl.opt1_i_buffer.duration,             sizeof(impedanceCtrl.opt1_i_buffer.duration));          temp_arr_index++;

      for(int i = 0; i<500000; ++i){}

      t_identifier = GUI_SYNC|MEMORY_UI_CONTROL_PARAMETERS2;
      Send_MSG(t_identifier, 4*temp_arr_index, (uint8_t*)temp_arr);
      temp_arr_index = 0;

      /****************************************************************************************************************************/
      memcpy(&temp_arr[0],  &VSD.lower_limit,        sizeof(VSD.lower_limit));                                      temp_arr_index++;
      memcpy(&temp_arr[1],  &VSD.upper_limit,        sizeof(VSD.upper_limit));                                      temp_arr_index++;
      memcpy(&temp_arr[2],  &VSD.lower_damped_range, sizeof(VSD.lower_damped_range));                               temp_arr_index++;
      memcpy(&temp_arr[3],  &VSD.upper_damped_range, sizeof(VSD.upper_damped_range));                               temp_arr_index++;
      memcpy(&temp_arr[4],  &VSD.lower_stiff_range,  sizeof(VSD.lower_stiff_range));                                temp_arr_index++;
      memcpy(&temp_arr[5],  &VSD.upper_stiff_range,  sizeof(VSD.upper_stiff_range));                                temp_arr_index++;
      memcpy(&temp_arr[6],  &VSD.lower_damper,       sizeof(VSD.lower_damper));                                     temp_arr_index++;
      memcpy(&temp_arr[7],  &VSD.upper_damper,       sizeof(VSD.upper_damper));                                     temp_arr_index++;
      memcpy(&temp_arr[8],  &VSD.lower_stiffness,    sizeof(VSD.lower_stiffness));                                  temp_arr_index++;
      memcpy(&temp_arr[9],  &VSD.upper_stiffness,    sizeof(VSD.upper_stiffness));                                  temp_arr_index++;
      memcpy(&temp_arr[10], &VSD.saturation,         sizeof(VSD.saturation));                                       temp_arr_index++;

      for(int i = 0; i<500000; ++i){}

      t_identifier = GUI_SYNC|MEMORY_UI_ADDITIONAL_FUNCTION_PARAMETERS;
      Send_MSG(t_identifier, 4*temp_arr_index, (uint8_t*)temp_arr);
      temp_arr_index = 0;

      /****************************************************************************************************************************/
      memcpy(&temp_arr[0],  &vqfMagCalibObj.a11,     sizeof(vqfMagCalibObj.a11));                                  temp_arr_index++;
      memcpy(&temp_arr[1],  &vqfMagCalibObj.a12,     sizeof(vqfMagCalibObj.a12));                                  temp_arr_index++;
      memcpy(&temp_arr[2],  &vqfMagCalibObj.a13,     sizeof(vqfMagCalibObj.a13));                                  temp_arr_index++;
      memcpy(&temp_arr[3],  &vqfMagCalibObj.a21,     sizeof(vqfMagCalibObj.a21));                                  temp_arr_index++;
      memcpy(&temp_arr[4],  &vqfMagCalibObj.a22,     sizeof(vqfMagCalibObj.a22));                                  temp_arr_index++;
      memcpy(&temp_arr[5],  &vqfMagCalibObj.a23,     sizeof(vqfMagCalibObj.a23));                                  temp_arr_index++;
      memcpy(&temp_arr[6],  &vqfMagCalibObj.a31,     sizeof(vqfMagCalibObj.a31));                                  temp_arr_index++;
      memcpy(&temp_arr[7],  &vqfMagCalibObj.a32,     sizeof(vqfMagCalibObj.a32));                                  temp_arr_index++;
      memcpy(&temp_arr[8],  &vqfMagCalibObj.a33,     sizeof(vqfMagCalibObj.a33));                                  temp_arr_index++;
      memcpy(&temp_arr[9],  &vqfMagCalibObj.b1,      sizeof(vqfMagCalibObj.b1));                                   temp_arr_index++;
      memcpy(&temp_arr[10], &vqfMagCalibObj.b2,      sizeof(vqfMagCalibObj.b2));                                   temp_arr_index++;
      memcpy(&temp_arr[11], &vqfMagCalibObj.b3,      sizeof(vqfMagCalibObj.b3));                                   temp_arr_index++;
      memcpy(&temp_arr[12], &vqfMagCalibObj.done,    sizeof(vqfMagCalibObj.done));                                 temp_arr_index++;

      for(int i = 0; i<500000; ++i){}

      t_identifier = GUI_SYNC|MEMORY_UI_IMU_PARAMETERS;
      Send_MSG(t_identifier, 4*temp_arr_index, (uint8_t*)temp_arr);
      temp_arr_index = 0;

    }

    void ExportProperties()
    {
    	  uint16_t t_identifier;
    	  float temp_arr[8];
    	  uint8_t temp_arr_index = 0;
    	  // UI_MOTOR_PROPERTIES
    	  memcpy(&temp_arr[0], &motor_properties.pole_pair,             sizeof(motor_properties.pole_pair));           temp_arr_index++;
    	  memcpy(&temp_arr[1], &inc25KhzObj.resolution,                 sizeof(inc25KhzObj.resolution));               temp_arr_index++;
    	  memcpy(&temp_arr[2], &motor_properties.gear_ratio,            sizeof(motor_properties.gear_ratio));          temp_arr_index++;
    	  memcpy(&temp_arr[3], &motor_properties.Kt,          	        sizeof(motor_properties.Kt));                  temp_arr_index++;
    	  memcpy(&temp_arr[4], &motor_properties.Ke,                    sizeof(motor_properties.Ke));                  temp_arr_index++;
    	  memcpy(&temp_arr[5], &motor_setting.peakCurr_limit,           sizeof(motor_setting.peakCurr_limit));         temp_arr_index++;
    	  memcpy(&temp_arr[6], &motor_setting.contCurr_limit,           sizeof(motor_setting.contCurr_limit));         temp_arr_index++;
    	  memcpy(&temp_arr[7], &motor_setting.max_velocity_rpm,         sizeof(motor_setting.max_velocity_rpm));       temp_arr_index++;
    	  for(int i = 0; i<25000; ++i){}

    	  t_identifier = GUI_SYNC|EXPORT_SETTING_PROPERTIES;
    	  Send_MSG(t_identifier, 4*temp_arr_index, (uint8_t*)temp_arr);
    	  temp_arr_index = 0;
    	  /**************************************************************************************************************/
    	  memcpy(&temp_arr[0], &motor_setting.commutation_set.done,     sizeof(motor_setting.commutation_set.done));   temp_arr_index++;
    	  memcpy(&temp_arr[1], &motor_setting.commutation_set.cc_dir,   sizeof(motor_setting.commutation_set.cc_dir)); temp_arr_index++;
    	  memcpy(&temp_arr[2], &motor_setting.commutation_set.ma_dir,   sizeof(motor_setting.commutation_set.ma_dir)); temp_arr_index++;
    	  memcpy(&temp_arr[3], &motor_setting.commutation_set.ea_dir,   sizeof(motor_setting.commutation_set.ea_dir)); temp_arr_index++;
    	  memcpy(&temp_arr[4], &motor_properties.R,                     sizeof(motor_properties.R));                   temp_arr_index++;
    	  memcpy(&temp_arr[5], &motor_properties.L,                     sizeof(motor_properties.L));                   temp_arr_index++;
    	  memcpy(&temp_arr[6], &kf_current_object.kf_A,                 sizeof(kf_current_object.kf_A));               temp_arr_index++;
    	  memcpy(&temp_arr[7], &kf_current_object.kf_B,                 sizeof(kf_current_object.kf_B));               temp_arr_index++;
    	  for(int i = 0; i<25000; ++i){}

    	  t_identifier = GUI_SYNC|EXPORT_SETTING_PROPERTIES;
    	  Send_MSG(t_identifier, 4*temp_arr_index, (uint8_t*)temp_arr);
    	  temp_arr_index = 0;
    	  /**************************************************************************************************************/
    	  memcpy(&temp_arr[0], &kf_current_object.kf_C,                 sizeof(kf_current_object.kf_C));               temp_arr_index++;
    	  memcpy(&temp_arr[1], &motor_setting.low_level_kalman_on,      sizeof(motor_setting.low_level_kalman_on));    temp_arr_index++;
    	  memcpy(&temp_arr[2], &motor_setting.currCtrl_BW_radPsec,      sizeof(motor_setting.currCtrl_BW_radPsec));    temp_arr_index++;
    	  memcpy(&temp_arr[3], &motor_properties.J,                     sizeof(motor_properties.J));                   temp_arr_index++;
    	  memcpy(&temp_arr[4], &motor_properties.B,                     sizeof(motor_properties.B));                   temp_arr_index++;
    	  memcpy(&temp_arr[5], &motor_properties.a,                     sizeof(motor_properties.a));                   temp_arr_index++;
    	  memcpy(&temp_arr[6], &motor_properties.b,                     sizeof(motor_properties.b));                   temp_arr_index++;
    	  memcpy(&temp_arr[7], &motor_properties.c,                     sizeof(motor_properties.c));                   temp_arr_index++;
    	  for(int i = 0; i<25000; ++i){}

    	  t_identifier = GUI_SYNC|EXPORT_SETTING_PROPERTIES;
    	  Send_MSG(t_identifier, 4*temp_arr_index, (uint8_t*)temp_arr);
    	  temp_arr_index = 0;
    	  /**************************************************************************************************************/
    	  memcpy(&temp_arr[0], &motor_properties.d,                     sizeof(motor_properties.d));                   temp_arr_index++;
    	  memcpy(&temp_arr[1], &mid_ctrl_saturation,                    sizeof(mid_ctrl_saturation));                  temp_arr_index++;
    	  memcpy(&temp_arr[2], &velCtrl.Kp,                             sizeof(velCtrl.Kp));                           temp_arr_index++;
    	  memcpy(&temp_arr[3], &velCtrl.Ki,                             sizeof(velCtrl.Ki));                           temp_arr_index++;
    	  memcpy(&temp_arr[4], &velCtrl.Ctrl_BW_Hz,                     sizeof(velCtrl.Ctrl_BW_Hz));                   temp_arr_index++;
    	  memcpy(&temp_arr[5], &IRC.numerator_length,                   sizeof(IRC.numerator_length));                 temp_arr_index++;
    	  memcpy(&temp_arr[6], &IRC.denominator_length,                 sizeof(IRC.denominator_length));               temp_arr_index++;
    	  memcpy(&temp_arr[7], &IRC.saturation,                         sizeof(IRC.saturation));                       temp_arr_index++;
    	  for(int i = 0; i<25000; ++i){}

    	  t_identifier = GUI_SYNC|EXPORT_SETTING_PROPERTIES;
    	  Send_MSG(t_identifier, 4*temp_arr_index, (uint8_t*)temp_arr);
    	  temp_arr_index = 0;
    	  /**************************************************************************************************************/
    	  memcpy(&temp_arr[0], &IRC.irc_num,                            sizeof(IRC.irc_num));                          temp_arr_index = 6;
    	  for(int i = 0; i<25000; ++i){}

    	  t_identifier = GUI_SYNC|EXPORT_SETTING_PROPERTIES;
    	  Send_MSG(t_identifier, 4*temp_arr_index, (uint8_t*)temp_arr);
    	  temp_arr_index = 0;
    	  /**************************************************************************************************************/
    	  memcpy(&temp_arr[0], &IRC.irc_den,                            sizeof(IRC.irc_den));                          temp_arr_index = 6;
    	  memcpy(&temp_arr[1], &veObj.type,                             sizeof(veObj.type));                           temp_arr_index++;
    	  memcpy(&temp_arr[2], &posCtrl.Kp,                             sizeof(posCtrl.Kp));                           temp_arr_index++;
    	  for(int i = 0; i<25000; ++i){}

    	  t_identifier = GUI_SYNC|EXPORT_SETTING_PROPERTIES;
    	  Send_MSG(t_identifier, 4*temp_arr_index, (uint8_t*)temp_arr);
    	  temp_arr_index = 0;
    	  /**************************************************************************************************************/
    	  memcpy(&temp_arr[0], &posCtrl.Kd,                             sizeof(posCtrl.Kd));                           temp_arr_index++;
    	  memcpy(&temp_arr[1], &posCtrl.R,                              sizeof(posCtrl.R));                            temp_arr_index++;
    	  memcpy(&temp_arr[2], &AbsObj1.offset,                         sizeof(AbsObj1.offset));                       temp_arr_index++;
    	  memcpy(&temp_arr[3], &AbsObj1.sign,                           sizeof(AbsObj1.sign));                         temp_arr_index++;
    	  memcpy(&temp_arr[4], &VSD.lower_limit,                        sizeof(VSD.lower_limit));                      temp_arr_index++;
    	  memcpy(&temp_arr[5], &VSD.upper_limit,                        sizeof(VSD.upper_limit));                      temp_arr_index++;
    	  memcpy(&temp_arr[6], &VSD.lower_damped_range,                 sizeof(VSD.lower_damped_range));               temp_arr_index++;
    	  memcpy(&temp_arr[7], &VSD.upper_damped_range,                 sizeof(VSD.upper_damped_range));               temp_arr_index++;
    	  for(int i = 0; i<25000; ++i){}

    	  t_identifier = GUI_SYNC|EXPORT_SETTING_PROPERTIES;
    	  Send_MSG(t_identifier, 4*temp_arr_index, (uint8_t*)temp_arr);
    	  temp_arr_index = 0;
    	  /**************************************************************************************************************/
    	  memcpy(&temp_arr[0], &VSD.lower_stiff_range,                   sizeof(posCtrl.Kd));                           temp_arr_index++;
    	  memcpy(&temp_arr[1], &VSD.lower_stiff_range,                   sizeof(posCtrl.R));                            temp_arr_index++;
    	  memcpy(&temp_arr[2], &VSD.lower_damper,                        sizeof(AbsObj1.offset));                       temp_arr_index++;
    	  memcpy(&temp_arr[3], &VSD.upper_damper,                        sizeof(AbsObj1.sign));                         temp_arr_index++;
    	  memcpy(&temp_arr[4], &VSD.lower_stiffness,                     sizeof(VSD.lower_limit));                      temp_arr_index++;
    	  memcpy(&temp_arr[5], &VSD.upper_stiffness,                     sizeof(VSD.upper_limit));                      temp_arr_index++;
    	  memcpy(&temp_arr[6], &VSD.saturation,                          sizeof(VSD.lower_damped_range));               temp_arr_index++;
    	  memcpy(&temp_arr[7], &kf_current_object.BEMF_Comp_Gain,        sizeof(kf_current_object.BEMF_Comp_Gain));     temp_arr_index++;
    	  for(int i = 0; i<25000; ++i){}

    	  t_identifier = GUI_SYNC|EXPORT_SETTING_PROPERTIES;
    	  Send_MSG(t_identifier, 4*temp_arr_index, (uint8_t*)temp_arr);
    	  temp_arr_index = 0;
}
/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x30000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
