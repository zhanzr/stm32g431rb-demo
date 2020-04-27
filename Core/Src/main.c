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

#include <arm_math.h>
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

/* Array of filter coefficients A (feedback coefficients) in Q1.15 format */
static int16_t aFilterCoeffA[] = {
   19448,-29793,  9645, -4884,   671,   -81
};

/* Array of filter coefficients B (feed-forward taps) in Q1.15 format */
static int16_t aFilterCoeffB[] = {
     590,  3540,  8851, 11801,  8851,  3540,   590
};

/* Array of input values in Q1.15 format */
static int16_t aInputValues[] = {
       0,  5276, -1548, 13844,     7, 17551,  5802, 16142, 14198, 12009,
   21624,  8678, 24576,  8672, 21611, 11990, 14172, 16111,  5765, 17510,
     -37, 13797, -1598,  5225,   -51, -5327,  1498,-13892,   -52,-17592,
   -5838,-16174,-14223,-12029,-21637, -8685,-24576, -8665,-21597,-11970,
  -14146,-16080, -5729,-17469,    82,-13749,  1647, -5174,   103,  5378,
   -1449, 13939,    96, 17632,  5874, 16205, 14249, 12048, 21650,  8691,
   24575,  8658, 21583, 11950, 14120, 16048,  5692, 17428,  -127, 13701,
   -1697,  5122,  -154, -5429,  1399,-13987,  -141,-17673, -5910,-16236,
  -14274,-12068,-21663, -8698,-24575, -8651,-21570,-11930,-14094,-16016,
   -5655,-17387,   171,-13654,  1747, -5071,   206,  5480, -1349, 14034,
     185, 17713,  5946, 16267, 14299, 12087, 21676,  8704, 24574,  8643,
   21556, 11909, 14067, 15984,  5618, 17346,  -216, 13606, -1797,  5020,
    -257, -5530,  1300,-14081,  -229,-17754, -5982,-16297,-14324,-12106,
  -21688, -8710,-24574, -8636,-21542,-11889,-14041,-15952, -5581,-17304,
     261,-13558,  1847, -4969,   309,  5581, -1250, 14128,   273, 17794,
    6018, 16328, 14349, 12124, 21701,  8715, 24573,  8628, 21527, 11868,
   14014, 15920,  5544, 17263,  -306, 13510, -1897,  4918,  -360, -5632,
    1201,-14176,  -317,-17834, -6053,-16358,-14374,-12143,-21713, -8721,
  -24571, -8620,-21513,-11847,-13988,-15888, -5507,-17221,   352,-13462,
    1947, -4867,   412,  5683, -1152, 14223,   361, 17874,  6089, 16389,
   14399, 12162, 21725,  8726, 24570,  8612, 21498, 11826, 13961, 15856,
    5470, 17180,  -397, 13414, -1997,  4816,  -463, -5734,  1102,-14270,
    -405,-17914, -6124,-16419,-14423,-12180,-21737, -8732,-24569, -8604,
  -21484,-11805,-13934,-15823, -5432,-17138,   442,-13366,  2047, -4764,
     515,  5785, -1053, 14317,   449, 17954,  6160, 16449, 14447, 12198,
   21749,  8737, 24567,  8596, 21469, 11784
};

/* Array of output data to preload in Q1.15 format */
static int16_t aOutputDataToPreload[] = {
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000
};

/* Array of calculated filtered data in Q1.15 format */
static int16_t aCalculatedFilteredData[ARRAY_SIZE];

/* Expected number of calculated samples */
uint16_t ExpectedCalculatedOutputSize = MEMORY_PARAMETER_D2;

/* Status of the FMAC callbacks */
__IO uint32_t HalfOutputDataReadyCallback = CALLBACK_NOT_CALLED;
__IO uint32_t OutputDataReadyCallback     = CALLBACK_NOT_CALLED;
__IO uint32_t ErrorCount                  = 0;

/* Array of reference filtered data for IIR "7 feed-forward taps, 6 feedback coefficients, gain = 1" in Q1.15 format */
static const int16_t aRefFilteredData[] = {
    7140, 13565, 12916, 10624, 12289, 15712, 16843, 15986, 15635, 16225,
   16376, 15489, 14241, 13174, 12045, 10498,  8621,  6684,  4744,  2697,
     532, -1645, -3752, -5782, -7735, -9568,-11228,-12685,-13927,-14937,
  -15692,-16174,-16376,-16301,-15947,-15319,-14427,-13288,-11923,-10352,
   -8603,- 6707, -4698, -2607,  -470,  1674,  3789,  5839,  7789,  9605,
   11257, 12716, 13957, 14959, 15704, 16179, 16377, 16295, 15934, 15299,
   14401, 13257, 11886, 10311,  8559,  6659,  4646,  2554,   418, -1726,
   -3840, -5888, -7836, -9649,-11296,-12749,-13985,-14981,-15719,-16188,
  -16380,-16292,-15923,-15281,-14378,-13229,-11852,-10271, -8516, -6615,
   -4599, -2504,  -367,  1776,  3889,  5935,  7880,  9689, 11332, 12781,
   14011, 15001, 15733, 16195, 16380, 16285, 15910, 15262, 14352, 13196,
   11815, 10231,  8471,  6566,  4549,  2453,   315, -1828, -3940, -5984,
   -7925, -9730,-11370,-12815,-14039,-15021,-15747,-16205,-16383,-16280,
  -15898,-15244,-14329,-13167,-11779,-10190, -8428, -6520, -4500, -2403,
    -265,  1878,  3989,  6031,  7970,  9771, 11406, 12845, 14064, 15042,
   15761, 16210, 16382, 16274, 15885, 15223, 14301, 13135, 11744, 10150,
    8382,  6471,  4450,  2351,   212, -1930, -4039, -6079, -8016, -9814,
  -11443,-12877,-14091,-15063,-15776,-16219,-16384,-16268,-15872,-15205,
  -14278,-13106,-11709,-10110, -8339, -6425, -4401, -2301,  -161,  1981,
    4088,  6126,  8060,  9854, 11479, 12908, 14117, 15083, 15789, 16225,
   16383, 16261, 15859, 15185, 14251, 13074, 11673, 10070,  8294,  6376,
    4350,  2249,   109, -2032, -4139, -6175, -8106, -9896,-11517,-12941,
  -14143,-15103,-15804,-16234,-16385,-16255,-15847,-15168,-14228,-13043,
  -11635,-10029, -8251, -6330, -4301, -2199,   -59,  2083,  4189,  6222,
    8149,  9936, 11553, 12972, 14169, 15122, 15815, 16239, 16385, 16249,
   15833, 15146, 14200, 13011, 11599,  9987
};

/**
  * @brief FMAC half output data ready callback
  * @par hfmac: FMAC HAL handle
  * @retval None
  */
void HAL_FMAC_HalfOutputDataReadyCallback(FMAC_HandleTypeDef *hfmac) {
  HalfOutputDataReadyCallback = CALLBACK_CALLED;
}

/**
  * @brief FMAC output data ready callback
  * @par hfmac: FMAC HAL handle
  * @retval None
  */
void HAL_FMAC_OutputDataReadyCallback(FMAC_HandleTypeDef *hfmac) {
  OutputDataReadyCallback = CALLBACK_CALLED;
}

/**
  * @brief FMAC error callback
  * @par hfmac: FMAC HAL handle
  * @retval None
  */
void HAL_FMAC_ErrorCallback(FMAC_HandleTypeDef *hfmac){
  ErrorCount++;
}
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
	printf("Configure the FMAC\n");
  sFmacConfig.InputBaseAddress  = INPUT_BUFFER_BASE;
  sFmacConfig.InputBufferSize   = INPUT_BUFFER_SIZE;
  sFmacConfig.InputThreshold    = INPUT_THRESHOLD;
  sFmacConfig.CoeffBaseAddress  = COEFFICIENT_BUFFER_BASE;
  sFmacConfig.CoeffBufferSize   = COEFFICIENT_BUFFER_SIZE;
  sFmacConfig.OutputBaseAddress = OUTPUT_BUFFER_BASE;
  sFmacConfig.OutputBufferSize  = OUTPUT_BUFFER_SIZE;
  sFmacConfig.OutputThreshold   = OUTPUT_THRESHOLD;
  sFmacConfig.pCoeffA           = aFilterCoeffA;
  sFmacConfig.CoeffASize        = COEFF_VECTOR_A_SIZE;
  sFmacConfig.pCoeffB           = aFilterCoeffB;
  sFmacConfig.CoeffBSize        = COEFF_VECTOR_B_SIZE;
  sFmacConfig.Filter            = FMAC_FUNC_IIR_DIRECT_FORM_1;
  sFmacConfig.InputAccess       = FMAC_BUFFER_ACCESS_POLLING;
  sFmacConfig.OutputAccess      = FMAC_BUFFER_ACCESS_DMA;
  sFmacConfig.Clip              = FMAC_CLIP_DISABLED;
  sFmacConfig.P                 = COEFF_VECTOR_B_SIZE;
  sFmacConfig.Q                 = COEFF_VECTOR_A_SIZE;
  sFmacConfig.R                 = GAIN;
	printf("InputBaseAddress:%02X\n", sFmacConfig.InputBaseAddress);
	printf("InputBufferSize:%02X\n", sFmacConfig.InputBufferSize);
	printf("InputThreshold:%08X\n", sFmacConfig.InputThreshold);
	printf("CoeffBaseAddress:%02X\n", sFmacConfig.CoeffBaseAddress);
	printf("CoeffBufferSize:%02X\n", sFmacConfig.CoeffBufferSize);
	printf("OutputBaseAddress:%02X\n", sFmacConfig.OutputBaseAddress);
	printf("OutputBufferSize:%02X\n", sFmacConfig.OutputBufferSize);
	printf("OutputThreshold:%08X\n", sFmacConfig.OutputThreshold);
	printf("pCoeffA:%p\n", sFmacConfig.pCoeffA);
	printf("CoeffASize:%02X\n", sFmacConfig.CoeffASize);
	printf("pCoeffB:%p\n", sFmacConfig.pCoeffB);
	printf("CoeffBSize:%02X\n", sFmacConfig.CoeffBSize);
	printf("InputAccess:%02X\n", sFmacConfig.InputAccess);
	printf("OutputAccess:%02X\n", sFmacConfig.OutputAccess);
	printf("Clip:%08X\n", sFmacConfig.Clip);
	printf("P (vector length, number of filter taps, etc.):%02X\n", sFmacConfig.P);
	printf("Q (vector length, etc.). Ignored if not needed:%02X\n", sFmacConfig.Q);
	printf("R (gain, etc.). Ignored if not needed.:%02X\n", sFmacConfig.R);

	start_ticks = HAL_GetTick();
	printf("HAL_FMAC_FilterConfig\n");
  if (HAL_FMAC_FilterConfig(&hfmac, &sFmacConfig) != HAL_OK) {
    Error_Handler();
  }

  /*## Preload the input and output buffers ####################################*/
	printf("Preload the input and output buffers\n");
  if (HAL_FMAC_FilterPreload(&hfmac, aInputValues,         INPUT_BUFFER_SIZE,
                                             aOutputDataToPreload, COEFF_VECTOR_A_SIZE) != HAL_OK) {
    Error_Handler();
  }

  /*## Start calculation of IIR filter in polling/DMA mode #####################*/
	printf("Start calculation of IIR filter in polling/DMA mode\n");
  if (HAL_FMAC_FilterStart(&hfmac, aCalculatedFilteredData, &ExpectedCalculatedOutputSize) != HAL_OK) {
    Error_Handler();
  }

  /*## Wait for the end of the handling (no new data written) #################*/
  /*  For simplicity reasons, this example is just waiting till the end of the
      calculation, but the application may perform other tasks while the operation
      is ongoing. */
	printf("Wait for the end of the handling (no new data written)\n");
  while(HalfOutputDataReadyCallback == CALLBACK_NOT_CALLED) {
  }
	printf("HalfOutputDataReadyCallback called\n");	
  while(OutputDataReadyCallback == CALLBACK_NOT_CALLED) {
  }

  /*## Stop the calculation of IIR filter in polling/DMA mode ##################*/
	printf("Stop the calculation of IIR filter in polling/DMA mode\n");
  if (HAL_FMAC_FilterStop(&hfmac) != HAL_OK) {
    Error_Handler();
  }

  /*## Check the final error status ############################################*/
	printf("Check the final error status\n");
  if(ErrorCount != 0) {
    Error_Handler();
  }

  /*## Compare FMAC results to the reference values #####################*/
	printf("Compare FMAC results to the reference values\n");
  for (uint32_t i = 0; i < ExpectedCalculatedOutputSize; i++) {
    if (aCalculatedFilteredData[i]  != aRefFilteredData[i]) {
      /* Processing Error */
      Error_Handler();
    }
  }
	end_ticks = HAL_GetTick();

	printf("There is no error in the output values %u[%u-%u]\n", end_ticks-start_ticks, start_ticks, end_ticks);
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
