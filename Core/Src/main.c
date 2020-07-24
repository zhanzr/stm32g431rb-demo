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
#include "stm32g4xx_nucleo.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>

#include <arm_math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/**
 * @defgroup CMSIS DSP_Lib example 1
 * \par CMSIS DSP Software Library Functions Used:
 * \par
 * - arm_mat_init_f32()
 * - arm_mat_mult_f32()
 * - arm_max_f32()
 * - arm_min_f32()
 * - arm_mean_f32()
 * - arm_std_f32()
 * - arm_var_f32()
 *
 * - arm_cos_f32()
 * - arm_sin_f32()
 * - arm_mult_f32()
 * - arm_add_f32()
  */
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

__IO uint16_t g_ADCBuf[ADC_CHAN_NO];
__IO uint32_t g_start_tick;
__IO uint32_t g_end_tick;

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

void matrix_basic_calculcation(void) {
	#define USE_STATIC_INIT

	const uint32_t NUMSTUDENTS = 20;
	const uint32_t NUMSUBJECTS = 4;
	const uint32_t TEST_LENGTH_SAMPLES = ((NUMSTUDENTS) * (NUMSUBJECTS));

	/* ----------------------------------------------------------------------
	** List of Marks scored by 20 students for 4 subjects
	** ------------------------------------------------------------------- */
	const float32_t testMarks_f32[TEST_LENGTH_SAMPLES] = {
		42.000000,	37.000000,	81.000000,	28.000000,	 
		83.000000,	72.000000,	36.000000,	38.000000,	 
		32.000000,	51.000000,	63.000000,	64.000000,	 
		97.000000,	82.000000,	95.000000,	90.000000,	 
		66.000000,	51.000000,	54.000000,	42.000000,	 
		67.000000,	56.000000,	45.000000,	57.000000,	 
		67.000000,	69.000000,	35.000000,	52.000000,	 
		29.000000,	81.000000,	58.000000,	47.000000,	 
		38.000000,	76.000000,	100.000000,	29.000000,	 
		33.000000,	47.000000,	29.000000,	50.000000,	 
		34.000000,	41.000000,	61.000000,	46.000000,	 
		52.000000,	50.000000,	48.000000,	36.000000,	 
		47.000000,	55.000000,	44.000000,	40.000000,	 
		100.000000,	94.000000,	84.000000,	37.000000,	 
		32.000000,	71.000000,	47.000000,	77.000000,	 
		31.000000,	50.000000,	49.000000,	35.000000,	 
		63.000000,	67.000000,	40.000000,	31.000000,	 
		29.000000,	68.000000,	61.000000,	38.000000,	 
		31.000000,	28.000000,	28.000000,	76.000000,	 
		55.000000,	33.000000,	29.000000,	39.000000 
	};


	/* ----------------------------------------------------------------------
	* Number of subjects X 1
	* ------------------------------------------------------------------- */
	const float32_t testUnity_f32[NUMSUBJECTS] = {
		1.000,  1.000, 	1.000,  1.000 
	};


	/* ----------------------------------------------------------------------
	** f32 Output buffer
	** ------------------------------------------------------------------- */
	static float32_t testOutput[TEST_LENGTH_SAMPLES];

	uint32_t    numStudents = NUMSTUDENTS;
	uint32_t    numSubjects = NUMSUBJECTS;
	float32_t    max_marks, min_marks, mean, std, var;
	uint32_t    student_num;

#ifndef  USE_STATIC_INIT

  arm_matrix_instance_f32 srcA;
  arm_matrix_instance_f32 srcB;
  arm_matrix_instance_f32 dstC;

  /* Input and output matrices initializations */
  arm_mat_init_f32(&srcA, numStudents, numSubjects, (float32_t *)testMarks_f32);
  arm_mat_init_f32(&srcB, numSubjects, 1, (float32_t *)testUnity_f32);
  arm_mat_init_f32(&dstC, numStudents, 1, testOutput);

#else

  /* Static Initializations of Input and output matrix sizes and array */
  arm_matrix_instance_f32 srcA = {NUMSTUDENTS, NUMSUBJECTS, (float32_t *)testMarks_f32};
  arm_matrix_instance_f32 srcB = {NUMSUBJECTS, 1, (float32_t *)testUnity_f32};
  arm_matrix_instance_f32 dstC = {NUMSTUDENTS, 1, testOutput};

#endif	
  /* ----------------------------------------------------------------------
  *Call the Matrix multiplication process function
  * ------------------------------------------------------------------- */
  arm_mat_mult_f32(&srcA, &srcB, &dstC);

  /* ----------------------------------------------------------------------
  ** Call the Max function to calculate max marks among numStudents
  ** ------------------------------------------------------------------- */
  arm_max_f32(testOutput, numStudents, &max_marks, &student_num);

  /* ----------------------------------------------------------------------
  ** Call the Min function to calculate min marks among numStudents
  ** ------------------------------------------------------------------- */
  arm_min_f32(testOutput, numStudents, &min_marks, &student_num);

  /* ----------------------------------------------------------------------
  ** Call the Mean function to calculate mean
  ** ------------------------------------------------------------------- */
  arm_mean_f32(testOutput, numStudents, &mean);

  /* ----------------------------------------------------------------------
  ** Call the std function to calculate standard deviation
  ** ------------------------------------------------------------------- */
  arm_std_f32(testOutput, numStudents, &std);

  /* ----------------------------------------------------------------------
  ** Call the var function to calculate variance
  ** ------------------------------------------------------------------- */
  arm_var_f32(testOutput, numStudents, &var);	
	
	return;
}

void test_sin_cos(void) {
	const uint32_t MAX_BLOCKSIZE = 50000;
	const float32_t DELTA = (0.0001f);

	const float32_t testRefOutput_f32 = 1.000000000;

	/* ----------------------------------------------------------------------
	* Declare Global variables
	* ------------------------------------------------------------------- */
	float32_t  testOutput;
	float32_t  cosOutput;
	float32_t  sinOutput;
	float32_t  cosSquareOutput;
	float32_t  sinSquareOutput;

  float32_t diff;

	g_start_tick = HAL_GetTick();
	// Calculation by CMSIS DSP
  for(uint32_t i=0; i< MAX_BLOCKSIZE; i++) {
		float32_t test_rad = ((float32_t)rand())/(RAND_MAX/2);
    cosOutput = arm_cos_f32(test_rad);
    sinOutput = arm_sin_f32(test_rad);

//    arm_mult_f32(&cosOutput, &cosOutput, &cosSquareOutput, 1);
//    arm_mult_f32(&sinOutput, &sinOutput, &sinSquareOutput, 1);

//    arm_add_f32(&cosSquareOutput, &sinSquareOutput, &testOutput, 1);
		cosSquareOutput = cosOutput * cosOutput;
		sinSquareOutput = sinOutput * sinOutput;

		testOutput = cosSquareOutput + sinSquareOutput;
		
    /* absolute value of difference between ref and test */
    diff = fabsf(testRefOutput_f32 - testOutput);

    /* Comparison of sin_cos value with reference */
    if(diff > DELTA) {
      printf("CMSIS DSP Failed, %d, %f > %f\n", i, diff, DELTA);
			printf("rad:%f, sine:%f, sine^2:%f, cosine:%f, cosine^2:%f, res:%f\n", 
			test_rad, sinOutput, sinSquareOutput, cosOutput, cosSquareOutput, testOutput);
			return;
		}
	}
	g_end_tick = HAL_GetTick();
	printf("\n\n");	
	printf("CMSIS DSP duration, %u - %u = [%u]\n", g_end_tick, g_start_tick, g_end_tick - g_start_tick);
	printf("\n\n");
	
	g_start_tick = HAL_GetTick();
	// Calculation by libc math 
  for(uint32_t i=0; i< MAX_BLOCKSIZE; i++) {
		float32_t test_rad = ((float32_t)rand())/(RAND_MAX/2);
    cosOutput = cosf(test_rad);
    sinOutput = sinf(test_rad);

		cosSquareOutput = cosOutput * cosOutput;
		sinSquareOutput = sinOutput * sinOutput;

		testOutput = cosSquareOutput + sinSquareOutput;

    /* absolute value of difference between ref and test */
    diff = fabsf(testRefOutput_f32 - testOutput);

    /* Comparison of sin_cos value with reference */
    if(diff > DELTA) {
      printf("LibC Failed, %d, %f > %f\n", i, diff, DELTA);
			printf("rad:%f, sine:%f, sine^2:%f, cosine:%f, cosine^2:%f, res:%f\n", 
			test_rad, sinOutput, sinSquareOutput, cosOutput, cosSquareOutput, testOutput);
			return;
		}
	}
	g_end_tick = HAL_GetTick();
	printf("\n\n");
	printf("LibC duration, %u - %u = [%u]\n", g_end_tick, g_start_tick, g_end_tick - g_start_tick);
	printf("\n\n");
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
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
#ifdef __MICROLIB
	printf("MicroLib\n");
#else
	printf("StdandardLib\n");
#endif
	
#if ((defined (__FPU_PRESENT) && (__FPU_PRESENT == 1U)) && \
     (defined (__FPU_USED   ) && (__FPU_USED    == 1U))     )
	printf("FPU Enabled\n");
#else
	printf("FPU Disabled\n");
#endif
	
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
	matrix_basic_calculcation();
	
	test_sin_cos();
	
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)g_ADCBuf, ADC_CHAN_NO);
	printf("\n\n");	
	printf("After Start ADC DMA, %u\n", SystemCoreClock);
	printf("%u %u %u\n",
	g_ADCBuf[0], g_ADCBuf[1], g_ADCBuf[2]
	);	
  while (1) {
//		printf("%u %u %u\n",
//		g_ADCBuf[0], g_ADCBuf[1], g_ADCBuf[2]
//		);
		
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
