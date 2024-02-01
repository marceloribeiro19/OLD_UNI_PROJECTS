/*
 * signals.c
 *
 *  Created on: 9 Mar 2021
 *      Author: mrgomes
 */

/* Includes ------------------------------------------------------------------*/
#include <math.h>

/* Private includes ----------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "dac.h"
#include "tim.h"
#include "dma.h"

#include "signals.h"

/* Private variables ---------------------------------------------------------*/
uint32_t signal_buff[100] = {0};

/* Private Defines ---------------------------------------------------------*/
#define PI 3.1415926

/* Private Functions ---------------------------------------------------------*/

/**
  * @brief  Inits Timer and DAC Peripherals
  * @param	none
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef wavegen_init(void){

	MX_DAC_Init();
	MX_TIM2_Init();
	HAL_TIM_Base_Start(&htim2);

	return HAL_OK;
}

/**
  * @brief  Starts DAC CHANNEL 2 with DMA
  * The DAC update is triggered by Timer 2 overflow
  * @param	none
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef wavegen_start(void){

	return HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_2, signal_buff, 100, DAC_ALIGN_12B_R);

}

/**
  * @brief  Stops DAC CHANNEL 2
  * Timer 2 is still running...
  * @param	none
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef wavegen_stop(void){

	return HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_2);

}

/**
  * @brief  Updates the signal frequency by just changing the
  * Timer 2 period, used to update one sample of the DAC output
  * Don't forget that we have n = 100 samples
  * @param	frequency in MHz (suggested values are between 1 and 100 MHz)
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef wavegen_freq_update(uint32_t freq){

	__HAL_TIM_DISABLE(&htim2);

		htim2.Init.Period = 1080000/freq;

		if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
			Error_Handler();
		}
	__HAL_TIM_ENABLE(&htim2);
	return HAL_OK;
}
HAL_StatusTypeDef Sampling_freq_update(double freq){
	static double Clock_freq = 108000000; //108Mhz
	//double desired_period = 1/(freq);
	uint32_t prescaler;
	uint32_t preload;

	//----cálculo do prescaler e do preload-------
	prescaler = ((Clock_freq/ (freq * (65536)))-1)+4;
	preload = (Clock_freq / ((prescaler + 1) * freq)) - 1;

	// Configurar o timer3 com os valores calculados
	TIM_OC_InitTypeDef sConfigOC = {0};

	htim3.Init.Prescaler = prescaler;
	htim3.Init.Period = preload;

	HAL_TIM_Base_Init(&htim3);

}
/**
  * @brief  Calculates and generates the sample values for a sin wave
  * with A between 0 and VRef 0000 - FFF (0V to 3.3V)
  * @param	none
  * @retval none
  */
void wavegen_sin(void){

	for (int i = 0; i < 100; i++)
		signal_buff[i] = ((sin(i*2*PI/100) + 1) * ((0xFFF)/2));
}

/**
  * @brief  Calculates and generates the sample values for a triangle wave
  * with A between 0 and VRef 0000 - FFF (0V to 3.3V)
  * @param	none
  * @retval none
  */
void wavegen_tri(void){

	for (int i = 0; i < 50; i++)
		signal_buff[i] = i * ((0xFFF)/50);

	for (int i = 50; i > 0; i--)
		signal_buff[100 - i] = i * ((0xFFF)/50);
}

/**
  * @brief  Calculates and generates the sample values for a square wave
  * with A between 0 and VRef 0000 - FFF (0V to 3.3V)
  * @param	none
  * @retval none
  */
void wavegen_sqr(void){

	for (int i = 0; i < 50; i++){
		signal_buff[i] = (0xFFF);
		signal_buff[i + 50] = 0;
	}
}

/**
  * @brief  Calculates and generates the sample values for a saw tooth wave
  * with A between 0 and VRef 0000 - FFF (0V to 3.3V)
  * @param	none
  * @retval none
  */
void wavegen_stw(void){

	for (int i = 0; i < 100; i++)
		signal_buff[i] = i * ((0xFFF)/100);
}
