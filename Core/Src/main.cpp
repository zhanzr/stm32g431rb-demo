/* USER CODE BEGIN Header */
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

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <ctime>

#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <ratio>
#include <chrono>
#include <utility>
//#define	NDEBUG
#include <cassert>

using namespace std;
using namespace std::chrono;
using std::chrono::system_clock;
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
volatile uint16_t g_ADCBuf[ADC_CHAN_NO];
volatile uint32_t g_start_tick;
volatile uint32_t g_end_tick;

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

static void MX_DMA_Init_local(void);
static void MX_FMAC_Init_local(void);

extern "C" {
	extern void Reset_Handler(void);
	extern uint32_t Stack_Mem_label;
	extern uint32_t __initial_sp_label;
	extern uint32_t __heap_base_label;
	extern uint32_t __heap_limit_label;
	
	extern uint32_t __Vectors;
	extern uint32_t __Vectors_End;
}
extern auto test_decltype(void) -> float;
extern void test_default_1(void);
extern void test_default_2(void);
	
extern auto test_extent(void) -> void;

extern auto test_ref0(void) -> void;
extern auto test_ref1(void) -> void;
extern auto test_ref2(void) -> void;
extern auto test_ref3(void) -> void;

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
	cout << "BOR:" << __HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) << endl;;
	cout << "OBLRST:" << __HAL_RCC_GET_FLAG(RCC_FLAG_OBLRST) << endl;;
	cout << "PinRST:" << __HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) << endl;;
	cout << "SoftwareRST:" << __HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) << endl;;
	cout << "Independent Watchdog:" << __HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) << endl;;
	cout << "Window Watchdog:" << __HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) << endl;;
	cout << "Low Power:" << __HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) << endl;;
	cout << "BSP Ver:" << std::hex << BSP_GetVersion() << std::dec << endl;
#ifdef __MICROLIB
	cout << "MicroLib\n");
#else
	cout << "StdandardLib" << endl;
#endif
	
	cout << "cpp std:" << __cplusplus << " compiler ver:" << __VERSION__ << "ARMCC Ver:" << __ARMCC_VERSION << endl;
	vector<uint8_t> v_U8;
	cout << sizeof(v_U8) << ' ' << v_U8.capacity() << endl;
	v_U8.push_back(1);
	cout << sizeof(v_U8) << ' ' << v_U8.capacity() << endl;
	cout << "v_U8:" << reinterpret_cast<void*>(v_U8.data()) << endl;
	
	std::unique_ptr<uint8_t[]> up_U8{new uint8_t[3]};
	cout << "up_U8:" << static_cast<void*>(up_U8.get()) << endl;
	
	cout << "Reset_Handler:" << reinterpret_cast<void*>(Reset_Handler) << endl;
	cout << "Stack_Mem_label:" << reinterpret_cast<void*>(&__heap_base_label) << endl;
	cout << "__initial_sp_label:" << reinterpret_cast<void*>(&__heap_limit_label) << endl;
	cout << "__heap_base_label:" << reinterpret_cast<void*>(&Stack_Mem_label) << endl;
	cout << "__heap_limit_label:" << reinterpret_cast<void*>(&__initial_sp_label) << endl;
	
	cout << "__Vectors:" << reinterpret_cast<void*>(&__Vectors) << endl;
	cout << "__Vectors_End:" << reinterpret_cast<void*>(&__Vectors_End) << endl;
	
	cout << "SystemCoreClock@:" << reinterpret_cast<void*>(&SystemCoreClock) << endl;

#if ((defined (__FPU_PRESENT) && (__FPU_PRESENT == 1U)) && \
     (defined (__FPU_USED   ) && (__FPU_USED    == 1U))     )
	cout << ("FPU Enabled") << endl;
#else
	cout << ("FPU Disabled") << endl;
#endif
	
	__HAL_RCC_CLEAR_RESET_FLAGS();

  BSP_LED_Init(LED2);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init_local();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_CORDIC_Init();
  MX_FMAC_Init_local();
  MX_RNG_Init();
  /* USER CODE BEGIN 2 */
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

//	assert(true);
//	assert_param(false);	
//	assert(false);
	static_assert(true, "this is a static assert of true");
//	static_assert(false, "this is a static assert of false");
	static_assert(sizeof(void *) == 4, "64-bit code generation is not supported.");

	cout << test_decltype() << endl;;
	test_default_1();
	test_default_2();
	test_extent();
	
//	test_ref0();
//	test_ref1();
//	test_ref2();
	test_ref3();
	
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)g_ADCBuf, ADC_CHAN_NO);
	cout << "After Start ADC DMA, SystemCoreClock:" << SystemCoreClock << endl;
	cout << g_ADCBuf[0] << ' ' << g_ADCBuf[1] << ' ' << g_ADCBuf[2] << endl;
  while (1) {
		cout << "DTS:" << g_ADCBuf[0] << ' '
		<< "VBAT:" << g_ADCBuf[1] << ' '
		<< "VREF:" << g_ADCBuf[2] << endl;
		BSP_LED_Toggle(LED2);	

		HAL_Delay(60000);		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_8) != HAL_OK) {
    Error_Handler();
  }
  /** Initializes the peripherals clocks 
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_RNG
                              |RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_PLL;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief FMAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_FMAC_Init_local(void) {
  /* USER CODE BEGIN FMAC_Init 0 */

  /* USER CODE END FMAC_Init 0 */

  /* USER CODE BEGIN FMAC_Init 1 */

  /* USER CODE END FMAC_Init 1 */
  hfmac.Instance = FMAC;
  if (HAL_FMAC_Init(&hfmac) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN FMAC_Init 2 */

  /* USER CODE END FMAC_Init 2 */
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init_local(void) {
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
void HAL_FMAC_FilterPreloadCallback(FMAC_HandleTypeDef *hfmac) {
}

/**
  * @brief FMAC half get data callback
  * @par hfmac: FMAC HAL handle
  * @retval None
  */
void HAL_FMAC_HalfGetDataCallback(FMAC_HandleTypeDef *hfmac) {
}

/**
  * @brief FMAC get data callback
  * @par hfmac: FMAC HAL handle
  * @retval None
  */
void HAL_FMAC_GetDataCallback(FMAC_HandleTypeDef *hfmac) {
}

/**
  * @brief FMAC output data ready callback
  * @par hfmac: FMAC HAL handle
  * @retval None
  */
void HAL_FMAC_OutputDataReadyCallback(FMAC_HandleTypeDef *hfmac) {
}

/**
  * @brief FMAC error callback
  * @par hfmac: FMAC HAL handle
  * @retval None
  */
void HAL_FMAC_ErrorCallback(FMAC_HandleTypeDef *hfmac) {
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
	cout << __func__ << ' ' << __LINE__ << endl;
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	cout << file << ' ' << line << endl;	
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
