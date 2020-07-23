/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "cordic.h"
#include "dma.h"
#include "fmac.h"
#include "rng.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>

#include "2ToneNoise.h"
#include "3ToneNoise.h"
#include "stm32g4xx_nucleo.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* CORDIC configuration structure */
CORDIC_ConfigTypeDef sCordicConfig;

__IO uint16_t g_ADCBuf[ADC_CHAN_NO];

#if defined(__ARMCC_VERSION)
int stdout_putchar (int ch) {
	uint8_t c = ch;
	HAL_UART_Transmit(&huart1, &c, 1, 1);
	return ch;
}

int stderr_putchar (int ch) {
	uint8_t c = ch;
	HAL_UART_Transmit(&huart1, &c, 1, 1);
	return ch;
}

void ttywrch (int ch) {
	uint8_t c = ch;
	HAL_UART_Transmit(&huart1, &c, 1, 1);	
}
#else
int _write (int fd, const void *buf, size_t count) {
	for(uint32_t i=0; i<count; ++i) {
		HAL_UART_Transmit(&huart2, buf+i, 1, 1);
	}
	return count;
}
#endif

/* FMAC configuration structure */
FMAC_FilterConfigTypeDef sFmacConfig;

/* Array of input values in Q1.15 format */
static int16_t aInputValues[2][ARRAY_SIZE];

static int16_t aFilterCoeffB[COEFF_VECTOR_B_SIZE];

/* Array of calculated filtered data in Q1.15 format (two parts) */
static int16_t aCalculatedFilteredData[ARRAY_SIZE];

/* Expected number of calculated samples for the used aCalculatedFilteredDataX */
uint16_t CurrentInputArraySize;

/* Expected number of calculated samples for the used aCalculatedFilteredDataX */
uint16_t ExpectedCalculatedFilteredDataSize;

/* Status of the calculation */
__IO uint32_t FilterPreloadCallbackCount   = 0;
__IO uint32_t HalfGetDataCallbackCount     = 0;
__IO uint32_t GetDataCallbackCount         = 0;
__IO uint32_t OutputDataReadyCallbackCount = 0;
__IO uint32_t ErrorCount                   = 0;
uint32_t OldValue;

/* Frame selelector */
uint8_t OldFrame = 0;
uint8_t Frame = 1;
/* Auxiliary counter */
uint32_t Index;

void SystemClock_Config(void);
static void MX_DMA_Init(void);
static void MX_FMAC_Init(void);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint32_t start_ticks;
	uint32_t end_ticks;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	printf("BOR: %u\n", __HAL_RCC_GET_FLAG(RCC_FLAG_BORRST));
	printf("OBLRST: %u\n", __HAL_RCC_GET_FLAG(RCC_FLAG_OBLRST));
	printf("Pin: %u\n", __HAL_RCC_GET_FLAG(RCC_FLAG_PINRST));
	printf("Software: %u\n", __HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST));
	printf("Independent Watchdog: %u\n", __HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST));
	printf("Window Watchdog: %u\n", __HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST));
	printf("Low Power: %u\n", __HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST));
	
	printf("%u\n", SystemCoreClock);

	__HAL_RCC_CLEAR_RESET_FLAGS();

  BSP_LED_Init(LED2);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_CORDIC_Init();
  MX_FMAC_Init();
  MX_RNG_Init();
  /* USER CODE BEGIN 2 */
/*## Configure the FMAC peripheral ###########################################*/
	/* Fill first frame with input signal and zero the other frame */
  for (Index=0; Index<ARRAY_SIZE; Index++) {
    aInputValues[0][Index] = TwoToneNoise[Index];
    aInputValues[1][Index] = 0;
  }
  /* generate filter coefficents (in this example they are simply read from flash */
  for (Index=0; Index<COEFF_VECTOR_B_SIZE; Index++) {
    aFilterCoeffB[Index] = TwoToneFilter[Index];
  }

  /*## Static FMAC configuration parameters #################################*/
  sFmacConfig.InputBaseAddress  = INPUT_BUFFER_BASE;
  sFmacConfig.InputBufferSize   = INPUT_BUFFER_SIZE;
  sFmacConfig.InputThreshold    = INPUT_THRESHOLD;
  sFmacConfig.CoeffBaseAddress  = COEFFICIENT_BUFFER_BASE;
  sFmacConfig.CoeffBufferSize   = COEFFICIENT_BUFFER_SIZE;
  sFmacConfig.OutputBaseAddress = OUTPUT_BUFFER_BASE;
  sFmacConfig.OutputBufferSize  = OUTPUT_BUFFER_SIZE;
  sFmacConfig.OutputThreshold   = OUTPUT_THRESHOLD;
  sFmacConfig.pCoeffA           = NULL;
  sFmacConfig.CoeffASize        = 0;
  sFmacConfig.CoeffBSize        = COEFF_VECTOR_B_SIZE;
  sFmacConfig.Filter            = FMAC_FUNC_CONVO_FIR;
  sFmacConfig.InputAccess       = FMAC_BUFFER_ACCESS_DMA;
  sFmacConfig.OutputAccess      = FMAC_BUFFER_ACCESS_DMA;
#if defined(CLIP_ENABLED)
  sFmacConfig.Clip              = FMAC_CLIP_ENABLED;
#else
  sFmacConfig.Clip              = FMAC_CLIP_DISABLED;
#endif
  sFmacConfig.P                 = COEFF_VECTOR_B_SIZE;
  sFmacConfig.Q                 = FILTER_PARAM_Q_NOT_USED;
  sFmacConfig.R                 = GAIN;
	
/*### Main loop repeats each frame ########################################*/
	start_ticks = HAL_GetTick();

  /* Repeat until required number of frames have been processed */
  do {
    /* Point to filter coefficients for current frame*/
    sFmacConfig.pCoeffB           = aFilterCoeffB;
    /* Configure filter and load coefficients by polling */
    if (HAL_FMAC_FilterConfig(&hfmac, &sFmacConfig) != HAL_OK) {
      /* Configuration Error */
      Error_Handler();
    }

    /* Preload the filter state at end of previous frame */
    if (HAL_FMAC_FilterPreload_DMA(
			&hfmac,
			&aInputValues[Frame][ARRAY_SIZE-COEFF_VECTOR_B_SIZE+1],
			COEFF_VECTOR_B_SIZE-1, NULL, 0)
			!= HAL_OK) {
      /* Configuration Error */
      Error_Handler();
    }
    /* Switch input frames */
    OldFrame = Frame;
    Frame = (Frame ? 0 : 1);

    /* Wait for preload to finish */
    while (HAL_FMAC_GetState(&hfmac) != HAL_FMAC_STATE_READY) {
			;
		}

    /* Start calculation of FIR filter in DMA mode */
    ExpectedCalculatedFilteredDataSize = ARRAY_SIZE;
    if (HAL_FMAC_FilterStart(&hfmac, aCalculatedFilteredData, &ExpectedCalculatedFilteredDataSize) != HAL_OK) {
      /* Processing Error */
      Error_Handler();
    }

    /*## Append data to start the DMA process after the preloaded data handling ##*/
    CurrentInputArraySize = ARRAY_SIZE;
    if (HAL_FMAC_AppendFilterData(&hfmac,
                                &aInputValues[Frame][0],
                                &CurrentInputArraySize) != HAL_OK) {
      ErrorCount++;
    }
																
    /* While processing is going on, fill next frame with input signal */
    for (Index=0; Index<ARRAY_SIZE; Index++) {
      aInputValues[OldFrame][Index] = ThreeToneNoise[Index];
    }
		
    /* Update filter coefficients for next frame */
    /* Normally this would be done by the adaptive algorithm, in this example the */
    /* coefficients are loaded from flash */
    for (Index=0; Index<COEFF_VECTOR_B_SIZE; Index++)
    {
      aFilterCoeffB[Index] = ThreeToneFilter[Index];
    }
    /* Wait for FMAC to finish processing frame */
    while (OutputDataReadyCallbackCount == OldValue) {
			;
		}
    OldValue = OutputDataReadyCallbackCount;

    /* Stop the calculation of FIR filter in polling/DMA mode */
    if (HAL_FMAC_FilterStop(&hfmac) != HAL_OK) {
      /* Processing Error */
      Error_Handler();
    }
		
		/* Reached the end of processing : Turn LED2 on */
		BSP_LED_On(LED2);

	#ifdef PRINT_OUTPUT
		printf("Filtered[%d]\n",Frame);
		for (Index=0; Index<ARRAY_SIZE; Index++) {
			printf("%d\n", aCalculatedFilteredData[Index]);
		}
	#endif		
  /* end of loop */
  } while(OutputDataReadyCallbackCount < DATA_RDY_CALLBACK_COUNT);

  /*## Check the final error status ############################################*/
	if(ErrorCount != 0) {
		printf("%d, %u\n", __LINE__, ErrorCount);
		/* Processing Error */
		Error_Handler();
	} else {
		end_ticks = HAL_GetTick();
		printf("No error in the process %u[%u-%u]\n",
				end_ticks-start_ticks, start_ticks, end_ticks);			
	}	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)g_ADCBuf, ADC_CHAN_NO);		
	printf("After Start ADC DMA, %u\n", SystemCoreClock);
  while (1) {
		printf("%u %u %u\n",
		g_ADCBuf[0], g_ADCBuf[1], g_ADCBuf[2]
		);
		
		BSP_LED_Toggle(LED2);	

		HAL_Delay(4000);		
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV8;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_8) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks 
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_RNG
                              |RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_PLL;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief FMAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_FMAC_Init(void)
{

  /* USER CODE BEGIN FMAC_Init 0 */

  /* USER CODE END FMAC_Init 0 */

  /* USER CODE BEGIN FMAC_Init 1 */

  /* USER CODE END FMAC_Init 1 */
  hfmac.Instance = FMAC;
  if (HAL_FMAC_Init(&hfmac) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FMAC_Init 2 */

  /* USER CODE END FMAC_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

//  /* DMA1_Channel4_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
//  /* DMA1_Channel5_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
//  /* DMA1_Channel6_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);	
}


/* USER CODE BEGIN 4 */
/**
  * @brief FMAC filter preload callback
  * @par hfmac: FMAC HAL handle
  * @retval None
  */
void HAL_FMAC_FilterPreloadCallback(FMAC_HandleTypeDef *hfmac)
{
  FilterPreloadCallbackCount++;;
}

/**
  * @brief FMAC half get data callback
  * @par hfmac: FMAC HAL handle
  * @retval None
  */
void HAL_FMAC_HalfGetDataCallback(FMAC_HandleTypeDef *hfmac)
{
  HalfGetDataCallbackCount++;
}

/**
  * @brief FMAC get data callback
  * @par hfmac: FMAC HAL handle
  * @retval None
  */
void HAL_FMAC_GetDataCallback(FMAC_HandleTypeDef *hfmac)
{
  GetDataCallbackCount++;
}

/**
  * @brief FMAC output data ready callback
  * @par hfmac: FMAC HAL handle
  * @retval None
  */
void HAL_FMAC_OutputDataReadyCallback(FMAC_HandleTypeDef *hfmac)
{
  OutputDataReadyCallbackCount++;
}

/**
  * @brief FMAC error callback
  * @par hfmac: FMAC HAL handle
  * @retval None
  */
void HAL_FMAC_ErrorCallback(FMAC_HandleTypeDef *hfmac)
{
  ErrorCount++;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	printf("%s\n", __func__);
  /* User can add his own implementation to report the HAL error return state */
  while (1) {
    /* LED2 is blinking */
    BSP_LED_Toggle(LED2);
    HAL_Delay(500);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
