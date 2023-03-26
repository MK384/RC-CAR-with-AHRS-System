/*
 * JOYSTICK.c
 *
 *  Created on: Mar 24, 2023
 *      Author: moham
 */
#include "JOYSTICK.h"
#include "JOYSTICK_cfg.h"
#include "stm32f1xx_hal_dma.h"
#include "stm32f1xx_hal_adc.h"

void DMA_init(void);
void ADC_init(void);
static void Error_Handler(void);

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc;

/* Configure the DMA buffer */
uint32_t adc_buffer[JOYSTICK_UNITS<<1];

void JoyStick_Init(void){
	DMA_init();
	ADC_init();
	/* Start the ADC with DMA */
	HAL_ADC_Start_DMA(&hadc1, adc_buffer, JOYSTICK_UNITS<<1);
}

JoyStick_obj* JoyStick_getObj(uint8_t joystick_index){
	
	return &JoyStick_CfgParam[joystick_index];
}

void JoyStick_Read(uint8_t joystick_index){
	
	if(joystick_index >= JOYSTICK_UNITS) return;
	
	uint8_t i = (joystick_index<<1)-1;
	JoyStick_CfgParam[joystick_index].JS_xVal = adc_buffer[i];
	JoyStick_CfgParam[joystick_index].JS_yVal = adc_buffer[i+1];
}


void DMA_init(void){

	__HAL_RCC_DMA1_CLK_ENABLE();

	// Configure DMA
	hdma_adc.Instance = DMA1_Channel1;
	hdma_adc.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_adc.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_adc.Init.MemInc = DMA_MINC_ENABLE;
	hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	hdma_adc.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	hdma_adc.Init.Mode = DMA_CIRCULAR;
	hdma_adc.Init.Priority = DMA_PRIORITY_HIGH;
	HAL_DMA_Init(&hdma_adc);
	__HAL_LINKDMA(&hadc, DMA_Handle, hdma_adc);

	// Enable DMA interrupt
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

void ADC_init(void){
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
	Error_Handler();
  }


  ADC_ChannelConfTypeDef sConfig = {0};

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = JOYSTICK_UNITS << 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
	Error_Handler();
  }

  /** Configure Regular Channel
  */

  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;


  for (int i = 0; i < JOYSTICK_UNITS<<1;) {
	  sConfig.Channel = JoyStick_CfgParam[i].ADCx_CH;
	  sConfig.Rank = ++i;
	  HAL_ADC_ConfigChannel(&hadc, &sConfig);

	  sConfig.Channel = JoyStick_CfgParam[i].ADCy_CH;
	  sConfig.Rank = ++i;
	  HAL_ADC_ConfigChannel(&hadc, &sConfig);
	}

  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */


// Calibrate The ADC On Power-Up For Better Accuracy
	HAL_ADCEx_Calibration_Start(&hadc1);

}
static void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}


