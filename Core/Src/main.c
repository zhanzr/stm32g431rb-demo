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

void SystemClock_Config(void);
static void MX_DMA_Init(void);
static void MX_FMAC_Init(void);

struct msg_header {
	uint32_t type;
	uint32_t size;
	uint8_t content[0];
};

void ranges(char c) {
	switch(c) {
	case '0' ... '9':
	printf("[%c] is a number.\n", c);
	break;

	case 'a' ... 'z':
	printf("[%c] is a lowercase letter.\n", c);
	break;

	case 'A' ... 'Z':
	printf("[%c] is an uppercase letter.\n", c);
	break;

	default:
		printf("[%c] is not a valid character!\n", c);
		break;
	}
}

#define max(a,b) \
		({ typeof (a) _a = (a); \
		typeof (b) _b = (b); \
		_a > _b ? _a : _b; })

struct empty {
};

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
	printf("BOR: %u\n", __HAL_RCC_GET_FLAG(RCC_FLAG_BORRST));
	printf("OBLRST: %u\n", __HAL_RCC_GET_FLAG(RCC_FLAG_OBLRST));
	printf("Pin: %u\n", __HAL_RCC_GET_FLAG(RCC_FLAG_PINRST));
	printf("Software: %u\n", __HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST));
	printf("Independent Watchdog: %u\n", __HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST));
	printf("Window Watchdog: %u\n", __HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST));
	printf("Low Power: %u\n", __HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST));
	
	printf("Clock:%u, ARMCC Ver:%u\n", SystemCoreClock, __ARMCC_VERSION);

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
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
//		void nested_func(void) {
//		printf("Inside a nested function!\n");
//		printf("%s\n", __VERSION__);
//	}

//		printf("Test nested function\n");
//	nested_func();
//	printf("\n");
	
	// Test zero length of array
	printf("Test zero length of array\n");
	{
		struct msg_header *msg;
		int size = 128;

		msg = (struct msg_header *) malloc(sizeof (struct msg_header) + size);

		msg->type = 0x01;
		msg->size = size;

		printf("msg header  is at %p\n", (void *)msg);
		printf("msg content is at %p\n", (void *)msg->content);
		free(msg);
	}
	printf("\n");
	
	// Test case range
	printf("Test case range\n");
	{
		ranges('a');
		ranges('8');
	}
	printf("\n");
	
	// Test typeof
	printf("Test typeof\n");
	{
		int x = 1024, y = 4096;
		char a = 10, b = 20;
		float j = 1.0, k = 2.0;

		printf("char  max is %d\n", max(a,b));
		printf("int   max is %d\n", max(x,y));
		printf("float max is %f\n", max(j,k));
	}
	printf("\n");

	// Test empty structure
	printf("Test empty structure\n");
	printf("sizeof struct empty is %u\n", sizeof(struct empty));
	printf("\n");

	// Test binary immediate and ternary operator
	printf("binary immediate and ternary operator\n");
	{
		int a = 10;
		int x = 0;

		a = x ? : 0b0011;

		printf("a is %d.\n", a);
	}
	printf("\n");
	
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)g_ADCBuf, ADC_CHAN_NO);		
	printf("After Start ADC DMA, %u\n", SystemCoreClock);
  while (1) {
		printf("%u %u %u\n",
		g_ADCBuf[0], g_ADCBuf[1], g_ADCBuf[2]
		);
		
		BSP_LED_Toggle(LED2);	

		HAL_Delay(10000);		
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
}

/**
  * @brief FMAC half get data callback
  * @par hfmac: FMAC HAL handle
  * @retval None
  */
void HAL_FMAC_HalfGetDataCallback(FMAC_HandleTypeDef *hfmac)
{
}

/**
  * @brief FMAC get data callback
  * @par hfmac: FMAC HAL handle
  * @retval None
  */
void HAL_FMAC_GetDataCallback(FMAC_HandleTypeDef *hfmac)
{
}

/**
  * @brief FMAC output data ready callback
  * @par hfmac: FMAC HAL handle
  * @retval None
  */
void HAL_FMAC_OutputDataReadyCallback(FMAC_HandleTypeDef *hfmac)
{
}

/**
  * @brief FMAC error callback
  * @par hfmac: FMAC HAL handle
  * @retval None
  */
void HAL_FMAC_ErrorCallback(FMAC_HandleTypeDef *hfmac)
{
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
