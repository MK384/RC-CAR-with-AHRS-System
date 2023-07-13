/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "mpu6050.h"
#include "GY271.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//#include "arm_math.h"
#include "motion_fx.h"

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define RAD_TO_DEG	((float) 57.2957795)

MPU6050 mpu1;
GY271   mag1;






float roll , pitch , yaw , heading;


void calculateGbias(float * bias)
{

	bias[0] = 0.0;
	bias[1] = 0.0;
	bias[2] = 0.0;


	for (int var = 0; var < 50; ++var) {


		MPU6050_ReadAll(&mpu1);

		bias[0] += mpu1.Gyro_Xyz[0];
		bias[1] += mpu1.Gyro_Xyz[1];
		bias[2] += mpu1.Gyro_Xyz[2];

		HAL_Delay(10);
	}

	bias[0] /= (float)50.0;
	bias[1] /= (float)50.0;
	bias[2] /= (float)50.0;
}

#define MFX_STR_LENG 35

void init_MotionFX(void){

	/*** Initialization ***/
	char lib_version[MFX_STR_LENG];

	/* Sensor Fusion API initialization function */
	MotionFX_initialize();
	/* Optional: Get version */
	MotionFX_GetLibVersion(lib_version);


	MFX_knobs_t iKnobs;
	MotionFX_getKnobs(&iKnobs);

	iKnobs.start_automatic_gbias_calculation = 1;
	iKnobs.output_type = MFX_ENGINE_OUTPUT_ENU;

	iKnobs.acc_orientation[0] = 'e';
	iKnobs.acc_orientation[0] = 'n';
	iKnobs.acc_orientation[0] = 'u';

	iKnobs.gyro_orientation[0] = 'e';
	iKnobs.gyro_orientation[0] = 'n';
	iKnobs.gyro_orientation[0] = 'u';


	iKnobs.mag_orientation[0] = 's';
	iKnobs.mag_orientation[0] = 'e';
	iKnobs.mag_orientation[0] = 'u';

	MotionFX_setKnobs(&iKnobs);


	MotionFX_enable_6X(MFX_ENGINE_DISABLE);
	MotionFX_enable_9X(MFX_ENGINE_ENABLE);


}

MFX_input_t data_in;
MFX_output_t data_out;
uint32_t LastTimePropagate = 0 , LastTimeUpdate = 0.0, dT;
float dT_sec;
uint8_t printFlag;


void 	SensorFusionAlgorithm(void){


	MPU6050_ReadAll(&mpu1);
	GY271_getData(&mag1);

	data_in.acc[0] = mpu1.Accel_Xyz[0];
	data_in.acc[1] = mpu1.Accel_Xyz[1];
	data_in.acc[2] = mpu1.Accel_Xyz[2];

	data_in.gyro[0] = mpu1.Gyro_Xyz[0];
	data_in.gyro[1] = mpu1.Gyro_Xyz[1];
	data_in.gyro[2] = mpu1.Gyro_Xyz[2];

	data_in.mag[0] = mag1.Compass_Xyz[0];
	data_in.mag[1] = mag1.Compass_Xyz[1];
	data_in.mag[2] = mag1.Compass_Xyz[2];


	dT = TIM2->CNT - LastTimePropagate;
	LastTimePropagate = TIM2->CNT;
	dT_sec = ( ((float) dT) / ((float)1000000.0) );

	MotionFX_propagate(&data_out, &data_in, &dT_sec);


	dT = TIM2->CNT - LastTimeUpdate;
	LastTimeUpdate = TIM2->CNT;
	dT_sec = ( ((float) dT) / ((float)1000000.0) );

	MotionFX_update(&data_out, &data_in, &dT_sec, NULL);


	yaw = data_out.rotation_9X[0];
	pitch = data_out.rotation_9X[1];
	roll = data_out.rotation_9X[2];

	heading = data_out.heading_9X;

	printFlag = 1;


}

void init_sensors(void){


	  mpu1.I2Cx = &hi2c1;
	  mpu1.Address = MPU6050_I2C_ADDR_AD0_LOW;
	  mpu1.LowPassFilter = MPU6050_DLPF_BW_98Hz;
	  mpu1.DataRate = MPU6050_DataRate_100Hz;
	  mpu1.AccelerometerRange = MPU6050_Accelerometer_2G;
	  mpu1.GyroscopeRange= MPU6050_Gyroscope_250s;
	  mpu1.interruptState = MPU6050_Interrupt_Enabled;
	  mpu1.AccOutputUnit = MPU6050_ACC_UNIT_G;
	  mpu1.GyrOutputUnit = MPU6050_GYR_UNIT_DPS;

	  if (MPU6050_Init(&mpu1))
	  {
		  while(1)
		  {
			  HAL_Delay(1000);
			  printf("Error in init MPU6050! \n\r");
		  }
	  }


	  mag1.I2Cx = &hi2c1;
	  mag1.dataRate = GY271_DataRate_30_Hz;
	  mag1.range = GY271_Range_4_7Ga;
	  mag1.mode = GY271_Mode_Continuous;
	  mag1.samplesRate = GY271_SampleAvg_4_S;
	  mag1.outputUnit = GY271_UNIT_MICRO_TESLA_PER_50;

	  if (GY271_Init(&mag1))
	  {

		  while(1)
		  {
			  HAL_Delay(1000);
			  printf("Error in init GY271! \n\r");
		  }
	  }


}




void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	SensorFusionAlgorithm();


}




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim2);
  init_sensors();
//
  init_MotionFX();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {



	  	if (printFlag){

	  		printf("%.3f,%.3f,%.3f\r\n",roll ,pitch ,heading);
	  		printFlag = 0;

	  	}







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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : MPUP6050_RDY_INT_Pin */
  GPIO_InitStruct.Pin = MPUP6050_RDY_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MPUP6050_RDY_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

int _write(int file, char *ptr, int len)
{
  (void)file;
  CDC_Transmit_FS( (uint8_t*) ptr, len);
  return len;
}


/* USER CODE END 4 */

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
