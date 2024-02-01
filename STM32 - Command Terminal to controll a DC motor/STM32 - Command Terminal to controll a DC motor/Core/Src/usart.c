/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


volatile uint8_t UART3Rx_Buffer[128];
volatile uint8_t Rx_buffer[128];
volatile int receive_flag = 0;
volatile int flag_RX = 0;			 //flag necessaria para, no main verificar q recebeu algo
volatile uint8_t UART3Rx_index = 0;
/* USER CODE END 0 */

UART_HandleTypeDef huart3;

/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
    PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PD8     ------> USART3_TX
    PD9     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */
    //#define RCC_base 0x40023800
    //#define RCC_USART3EN 1
    //#define RCC_USART3EN_pos 18
    //#define UART3SEL 0b01
    //#define UART3SEL_pos 4
    //#define GPIOD_base 0x40020C00
    //#define MODER9 0b10
    //#define MODER9_pos 18
    //#define MODER8 0b10
    //#define MODER8_pos 16
    //#define OSPEEDR9 0b11
    //#define OSPEEDR9_pos 18
    //#define OSPEEDR8 0b11
    //#define OSPEEDR8_pos 16
    //#define AFR9 0b0111
    //#define AFR9_pos 4
    //#define AFR8 0b0111
    //#define AFR8_pos 0

//    uint32_t *ptr_RCC_RCC_APB1ENR = RCC_base + 0x40;
//    uint32_t *ptr_RCC_RCC_DCKCFGR2 = RCC_base + 0x90;
//    uint32_t *ptr_GPIOD_GPIOx_MODER = GPIOD_base + 0x0;
//    uint32_t *ptr_GPIOD_GPIOx_OSPEEDR = GPIOD_base + 0x08;
//    uint32_t *ptr_GPIOD_GPIOx_AFRH = GPIOD_base + 0x24;
//    // Configurar a fonte do clock da USART3
//    *ptr_RCC_RCC_DCKCFGR2 |= (UART3SEL) << UART3SEL_pos;
//
//    //Da enable ao clock da USART3
//    *ptr_RCC_RCC_APB1ENR |= (RCC_USART3EN) << RCC_USART3EN_pos;
//
//    __HAL_RCC_GPIOD_CLK_ENABLE();
//
//     // TX & RX
//     *ptr_GPIOD_GPIOx_MODER |= (MODER9) << MODER9_pos;
//     *ptr_GPIOD_GPIOx_MODER |= (MODER8) << MODER8_pos;
//
//     // Configurar speed
//     *ptr_GPIOD_GPIOx_OSPEEDR |= (OSPEEDR9) << OSPEEDR9_pos;
//     *ptr_GPIOD_GPIOx_OSPEEDR |= (OSPEEDR8) << OSPEEDR8_pos;
//
//     // Configurar o alternate
//     *ptr_GPIOD_GPIOx_AFRH |= (AFR9) << AFR9_pos;
//     *ptr_GPIOD_GPIOx_AFRH |= (AFR8) << AFR8_pos;
//
//     HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
//     HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PD8     ------> USART3_TX
    PD9     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_8|GPIO_PIN_9);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void init_UART3()
	{
		HAL_UART_Receive_IT(&huart3, &UART3Rx_Buffer[UART3Rx_index], 1);
	}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3){
		UART3Rx_index++;
		UART3Rx_index &= ~(1<<7);
		HAL_UART_Receive_IT(&huart3, &UART3Rx_Buffer[UART3Rx_index], 1);
	}
}
char* newMessage()
{
	static int local_index = 0;
	int out_index = 0;
	while (local_index !=UART3Rx_index)
	{
		Rx_buffer[out_index]= UART3Rx_Buffer[local_index];
		out_index++;
		local_index++;
		local_index &= ~(1<<7);
		receive_flag = 1;
	}
	Rx_buffer[out_index]= '\0';
	if(receive_flag)
	{
		receive_flag=0; //reset da flag
		flag_RX = 1;
		return Rx_buffer;
	}
	HAL_Delay(100);
}

/* USER CODE END 1 */
