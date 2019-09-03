/**
  ******************************************************************************
  * File Name          : CORDIC.c
  * Description        : This file provides code for the configuration
  *                      of the CORDIC instances.
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

/* Includes ------------------------------------------------------------------*/
#include "cordic.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

CORDIC_HandleTypeDef hcordic;
DMA_HandleTypeDef hdma_cordic_write;
DMA_HandleTypeDef hdma_cordic_read;

/* CORDIC init function */
void MX_CORDIC_Init(void)
{

  hcordic.Instance = CORDIC;
  if (HAL_CORDIC_Init(&hcordic) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_CORDIC_MspInit(CORDIC_HandleTypeDef* cordicHandle)
{

  if(cordicHandle->Instance==CORDIC)
  {
  /* USER CODE BEGIN CORDIC_MspInit 0 */

  /* USER CODE END CORDIC_MspInit 0 */
    /* CORDIC clock enable */
    __HAL_RCC_CORDIC_CLK_ENABLE();
  
    /* CORDIC DMA Init */
    /* CORDIC_WRITE Init */
    hdma_cordic_write.Instance = DMA1_Channel1;
    hdma_cordic_write.Init.Request = DMA_REQUEST_CORDIC_WRITE;
    hdma_cordic_write.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_cordic_write.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_cordic_write.Init.MemInc = DMA_MINC_ENABLE;
    hdma_cordic_write.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_cordic_write.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_cordic_write.Init.Mode = DMA_NORMAL;
    hdma_cordic_write.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_cordic_write) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(cordicHandle,hdmaIn,hdma_cordic_write);

    /* CORDIC_READ Init */
    hdma_cordic_read.Instance = DMA1_Channel2;
    hdma_cordic_read.Init.Request = DMA_REQUEST_CORDIC_READ;
    hdma_cordic_read.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_cordic_read.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_cordic_read.Init.MemInc = DMA_MINC_ENABLE;
    hdma_cordic_read.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_cordic_read.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_cordic_read.Init.Mode = DMA_NORMAL;
    hdma_cordic_read.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_cordic_read) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(cordicHandle,hdmaOut,hdma_cordic_read);

  /* USER CODE BEGIN CORDIC_MspInit 1 */

  /* USER CODE END CORDIC_MspInit 1 */
  }
}

void HAL_CORDIC_MspDeInit(CORDIC_HandleTypeDef* cordicHandle)
{

  if(cordicHandle->Instance==CORDIC)
  {
  /* USER CODE BEGIN CORDIC_MspDeInit 0 */

  /* USER CODE END CORDIC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CORDIC_CLK_DISABLE();

    /* CORDIC DMA DeInit */
    HAL_DMA_DeInit(cordicHandle->hdmaIn);
    HAL_DMA_DeInit(cordicHandle->hdmaOut);
  /* USER CODE BEGIN CORDIC_MspDeInit 1 */

  /* USER CODE END CORDIC_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
