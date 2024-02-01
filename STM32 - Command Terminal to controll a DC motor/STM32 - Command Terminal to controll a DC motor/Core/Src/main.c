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
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>
#include "signal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RCC_base 0x40023800
#define GPIOA_base 0x40020000
#define GPIOAEN_Pos 0
#define GPIOxEN 1
#define nMODER_INPUT 0b11
#define OSPEEDRx 0b10
#define MODER_OUTPUT 0b01
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t *ptr_RCC_RCC_AHB1ENR = RCC_base + 0x30; //os gpio's estao todos pendurados no AHB1 que tem offset de 0x30 em relação ao endereço base do RCC
uint32_t *ptr_GPIOx_GPIOx_MODER;
uint32_t *ptr_GPIOx_GPIOx_OSPEEDR;
uint32_t *ptr_GPIOx_GPIOx_IDR;
uint32_t *ptr_GPIOx_GPIOx_ODR;
uint32_t GPIOx_base;
uint32_t Samples;
uint32_t ADC_counter = 0;
uint8_t  can_print = 0;
volatile double ADC_Buffer[500];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void missing_arg(int x);
int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}
int __io_getchar(void)
{
	uint8_t ch = 0;
	__HAL_UART_CLEAR_OREFLAG(&huart3);
	HAL_UART_Receive(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static int Hexa_to_bin(int hexa,char setting)
	{
		itoa(hexa,setting,2);						//passa de hexadecimal para string em binario
		return strtoul(setting, NULL, 2);			//passa de string em binario para int binario
	}
void comandos(char* INPUT)
{
if(flag_RX == 1){
	const char* time_units[5]={"s","ms","us","ns"};
	const char* KEYS[18] = {"empty","MR","MW","MI","MO","RD","WD","RA","WG","SP","AC","S","ST","\b","ESC","$","?","VER"};
	const char* WAVES[6] = {"empty","SIN","SQR","TRI","STW","OFF"};
	const char DIVISON[1]= ".";
	char *arg_char[5];
	char *token;
	char SettingChar[17];
	char ValueChar;
	int arg_Hexa[4];
	int arg_Dec;
	static int index=0;
    int counter = 0;
	static int state;
	int Wave_state;
	int bit;
	int Port_num;
	unsigned long int SettingINT;
	unsigned long int ValueINT;
	static double Samp_Per;


	uint8_t *Mem_ptr;

	flag_RX=0;

int missing_arg(int args)
	{
		if(counter != args)
		{
			state = 0;
			printf("[ERRO]\r\n");
			return 1;
		}
	}

	token = strtok(INPUT,DIVISON);							 //separa as letras dos numeros
	arg_char[0] = token;
	for(index = 0;index< strlen(arg_char[0]);index++)
	{
		arg_char[0][index] = toupper(arg_char[0][index]);	 // garante que todas as letras sejam maiusculas
	}
	while (token != NULL) 									 // guarda as variaveis arg[1],[2]e[3]
		{

			token = strtok(NULL,DIVISON);
			counter ++;
			arg_char[counter] = token;
		}
	if(counter == 1){                   					// Quando so recebe uma palavra (VER,?,$..) coloca o ultimo caracter a null para a string ficar apenas com os digitos que tambem tem no array
		arg_char[0][strlen(arg_char[0])-1] = '\0';
	}

	for(index=1;index<=17;index++) 							//decide o estado se o valor introduzido for igual a algum do array
		{

			if( strcmp(arg_char[0],KEYS[index]) == 0)
				{
					state = index;
				}
		}

	for (index=0;index<3;index++) //passa as variaveis de char* para um array de inteiros hexadecimais
		{

			arg_Hexa[index] = strtol(arg_char[index+1], NULL, 16); //converte a string num numero de base 16(hexadecimal)

		}
	switch(state)
	{
		case 1:	// MEMORY READ (MR) MR.1.2
			 if (missing_arg(3)==1) {
			            break; 							   // Sai do switch case se ocorrer um erro na função
			        }

			Mem_ptr = arg_Hexa[0];
			for (index =0; index < arg_Hexa[1];index++)
				{
					printf("O endereco %p contem [%x]\r\n",Mem_ptr++,*Mem_ptr);
				}
		//	printf("O programa vai ler %s posicoes de memoria, a partir do endereco %s\r\n", arg[1], arg[2]);
			state = 0;
			break;
		case 2:	// MEMORY WRITE(MW)
			 if (missing_arg(4)==1) {
			            break; 							   // Sai do switch case se ocorrer um erro na função
			        }

			Mem_ptr = arg_Hexa[0]+1;
			(arg_Hexa[2]>0xFF)?arg_Hexa[2]= 0xFF:arg_Hexa[2];
			for(index = 0; index<arg_Hexa[1];index++)
				{
					*Mem_ptr = arg_Hexa[2];
					Mem_ptr++;
				}
			printf("A palavra de 8 bits %d,foi escrita a partir da posicao de memoria %x durante %d posicoes\r\n",arg_Hexa[2] , arg_Hexa[0],arg_Hexa[1]);
			state = 0;
			break;

		case 3:	// MAKE PIN INPUT(MI)
			 if (missing_arg(3)==1) {
			            break; 							   // Sai do switch case se ocorrer um erro na função
			        }
			(arg_Hexa[0]>0x0A)?arg_Hexa[0]= 0x0A:arg_Hexa[0]; 		// garante q o numero seja no maximo 0x0A que corresponde a porta K.

			Port_num = arg_Hexa[0];

			*ptr_RCC_RCC_AHB1ENR |= GPIOxEN << Port_num;
			ptr_GPIOx_GPIOx_MODER = GPIOA_base +(0x400*arg_Hexa[0]) ; 				//Moder tem offset = 0x0

			SettingINT = Hexa_to_bin(arg_Hexa[1],SettingChar);
			for(index=0;index<9;index++)
				{
					if( SettingINT & (1 << (index)))
					{
						bit = 2*index;
						*ptr_GPIOx_GPIOx_MODER &= ~(nMODER_INPUT << bit);
						printf("O pino P%c%d foi definido como entrada.\r\n",Port_num+65, index);
					}
			}
			state = 0;
			break;

		case 4:	// MAKE PIN OUTPUT(MO)
			 if (missing_arg(3)==1) {
			            break; 							   // Sai do switch case se nao receber o nº de argumentos necessarios
			        }
			(arg_Hexa[0]>0x0A)?arg_Hexa[0]= 0x0A:arg_Hexa[0]; 		// garante q o numero seja no maximo 0x0A que corresponde a porta K.

			Port_num = arg_Hexa[0];
			*ptr_RCC_RCC_AHB1ENR |= GPIOxEN << Port_num;

			ptr_GPIOx_GPIOx_MODER = GPIOA_base + (0x400*arg_Hexa[0]); 				//Moder tem offset = 0x0
			ptr_GPIOx_GPIOx_OSPEEDR = GPIOA_base + (0x400*arg_Hexa[0])+ 0x08;

			SettingINT = Hexa_to_bin(arg_Hexa[1],SettingChar);
			for(index=0;index<9;index++)
				{

					if(SettingINT & (1<<(index)))
					{

						bit = 2*index;
						*ptr_GPIOx_GPIOx_MODER |= ((MODER_OUTPUT) << bit);
						*ptr_GPIOx_GPIOx_OSPEEDR |= ((OSPEEDRx) << bit);
						printf("O pino P%c%d foi definido como saida.\r\n",Port_num+65, index);

					}
				}

			state = 0;
			break;

		case 5:	// READ DIGITAL INPUT	(RD)
			 if (missing_arg(3)==1) {
			            break; 							   			// Sai do switch case se ocorrer um erro na função
			        }
			 Port_num =arg_Hexa[0];
			(arg_Hexa[0]>0x0A)? arg_Hexa[0]= 0x0A : arg_Hexa[0]; 	// Garante q o numero do GPIO seja no maximo 0x0A que corresponde a porta K.
			SettingINT = Hexa_to_bin(arg_Hexa[1],SettingChar);		//

			for(index=0;index<9;index++)
				{
					if(SettingINT & (1<<(index)))
					{
						GPIOx_base = GPIOA_base+(0x400*arg_Hexa[0]);
						GPIO_InitTypeDef GPIO_InitStruct = {0};
						GPIO_InitStruct.Pin = 0x01<<index;
						GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
						GPIO_InitStruct.Pull = GPIO_NOPULL;
						HAL_GPIO_Init(GPIOx_base, &GPIO_InitStruct);
						printf("P%c%d = %d\r\n",Port_num+65, index, HAL_GPIO_ReadPin(GPIOx_base, 0x01<<index)); //
					}
				}

			state = 0;
			break;

		case 6:	// WRITE DIGITAL INPUT	(WD)
			 if (missing_arg(4)==1) {
			            break; 							   // Sai do switch case se ocorrer um erro na função
			        }
			 Port_num =arg_Hexa[0];
			(arg_Hexa[0]>0x0A)?arg_Hexa[0]= 0x0A:arg_Hexa[0]; 		// garante q o numero seja no maximo 0x0A que corresponde a porta K.

			SettingINT = Hexa_to_bin(arg_Hexa[1],SettingChar);
			ValueINT = Hexa_to_bin(arg_Hexa[2],ValueChar);

			for(index=0;index<9;index++)		// 001010
										// 000010
				{
					if(SettingINT & (1<<(index)))
					{
						GPIOx_base = GPIOA_base+(0x400*arg_Hexa[0]);
						GPIO_InitTypeDef GPIO_InitStruct = {0};

						GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
						GPIO_InitStruct.Pull = GPIO_NOPULL;
						GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

						if(ValueINT & (1<<(index)))
						{
							GPIO_InitStruct.Pin = (0x01<<index);

							HAL_GPIO_Init(GPIOx_base, &GPIO_InitStruct);
							HAL_GPIO_WritePin(GPIOx_base, 0x01<<index,SET);
							printf("P%c%d = 1\r\n",Port_num+65,index);
						}
						else{
							GPIO_InitStruct.Pin = (0x01<<index);
							HAL_GPIO_Init(GPIOx_base, &GPIO_InitStruct);
							HAL_GPIO_WritePin(GPIOx_base, 0x01<<index,RESET);
							printf("P%c%d = 0\r\n",Port_num+65,index);
						}
					}
				}

			state = 0;
			break;

		case 7:	// ANALOG READ	(RA)
			 if (missing_arg(2)==1 || arg_Hexa[0] > 0XF || arg_Hexa[0]< 0) {
			            break; 							   // Sai do switch case se ocorrer um erro na função
			        }

			ptr_GPIOx_GPIOx_MODER = GPIOA_base;
			*ptr_RCC_RCC_AHB1ENR |= ((GPIOxEN) << 0);//configuração do RCC (clock do adc)
			*ptr_GPIOx_GPIOx_MODER |=(0b11 << arg_Hexa[0]*2);

			ADC_ChannelConfTypeDef sConfig = {0};
			sConfig.Channel = arg_Hexa[0];
			sConfig.Rank = ADC_REGULAR_RANK_1;
			sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

			if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
			{
				Error_Handler();
			}
			can_print = 1;
			Samples = 1;
			HAL_ADC_Start_IT(&hadc1);
			ADC_counter = 0;
			state = 0;
			break;
		case 8:		// WAVE GENERATOR
				for(index = 0;index< strlen(arg_char[1]);index++) 		// garante que todas as letras sejam maiusculas
	 				{
						arg_char[1][index] = toupper(arg_char[1][index]);
	 				}

				if(counter == 2){                   					// Quando so recebe uma palavra (VER,?,$,OFF..) coloca o ultimo caracter a null para a string ficar apenas com os digitos que tambem tem no array
						arg_char[1][strlen(arg_char[1])-1] = '\0';
					}
				for(index=1;index<=5;index++) 							//decide o estado seguinte, se o valor introduzido for igual a algum do array
					{
						if( strcmp(arg_char[1],WAVES[index]) == 0){
								Wave_state = index;
							}
					}
				if(((arg_Hexa[1]<0X01) || (arg_Hexa[1]>0X64)) && Wave_state !=5 ){
						Wave_state= 0;
					}
				switch(Wave_state)
					{
						case 1:
							wavegen_init();
							wavegen_freq_update(arg_Hexa[1]);
							wavegen_sin();
							wavegen_start();
							printf("--------Onda Sinosoidal com frequencia de %dHz--------\r\n",arg_Hexa[1]);
							Wave_state = 0;
							break;
						case 2:
							wavegen_init();
							wavegen_freq_update(arg_Hexa[1]);
							wavegen_sqr();
							wavegen_start();
							printf("--------Onda Quadrada com frequencia de %dHz--------\r\n",arg_Hexa[1]);
							Wave_state = 0;
							break;
						case 3:
							wavegen_init();
							wavegen_freq_update(arg_Hexa[1]);
							wavegen_tri();
							wavegen_start();
							printf("--------Onda Triangular com frequencia de %dHz--------\r\n",arg_Hexa[1]);
							Wave_state = 0;
							break;
						case 4:
							wavegen_init();
							wavegen_freq_update(arg_Hexa[1]);
							wavegen_stw();
							wavegen_start();
							printf("--------Onda Dente-de-Serra com frequencia de %dHz--------\r\n",arg_Hexa[1]);
							Wave_state = 0;
							break;
						case 5:
							wavegen_stop();
							printf("--------Onda OFF--------\r\n");
							Wave_state = 0;
							break;
						default:    // ERROR
							printf("[ERRO]\r\n");
							break;
					}
				state = 0;
				break;
		case 9:		// PERIODO DE AMOSTRAGEM (SP)
			for(index = 0;index< strlen(arg_char[1]);index++)
			{
				arg_char[1][index] = tolower(arg_char[1][index]);	 // garante que todas as letras sejam maiusculas
			}
			for(index = 0;index < 4;index++)
			{
				if(strcmp(arg_char[1],time_units[index])==0)
				{
					arg_Dec = atoi(arg_char[2]);
					Samp_Per = arg_Dec/pow(1000,index);
					printf("--------Frequencia de %.1fHz (periodo de %d%s)--------\r\n",(1/Samp_Per),arg_Dec,arg_char[1]);
					Sampling_freq_update(1/Samp_Per);
				}
			}
			state = 0;
			break;
	    case 10:		// CANAL ADC (AC)

			 if (missing_arg(2)==1 || arg_Hexa[0] > 0XF || arg_Hexa[0]< 0) {
			            break; 							   // Sai do switch case se ocorrer um erro na função
			        }
			ptr_GPIOx_GPIOx_MODER = GPIOA_base;
			*ptr_RCC_RCC_AHB1ENR |= ((GPIOxEN) << 0);//configuração do RCC (clock do adc)
			*ptr_GPIOx_GPIOx_MODER |=(0b11 << arg_Hexa[0]*2);

			//ADC_ChannelConfTypeDef sConfig = {0};
			sConfig.Channel = arg_Hexa[0];
			sConfig.Rank = ADC_REGULAR_RANK_1;
			sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

			if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
				{
					Error_Handler();
				}
			printf("--------Selecionou o pino PA%d--------\r\n",arg_Hexa[0]);

			state = 0;
			break;
		case 11:		// INICIAR AQUISIÇÃO DE DADOS COM OU SEM LIMITE	(S OU S.x)

			if(counter == 1)
				{
					Samples = 0;
				 	HAL_TIM_Base_Start_IT(&htim3);
				}
			else if(counter == 2)
				{
					Samples = atoi(arg_char[1]);
					HAL_TIM_Base_Start_IT(&htim3);
				}
			state =0;
			break;
		case 12:		// PARA A AQUISIÇÃO DE DADOS (ST)
			printf("--------Sampling Stopped--------\r\n");
			HAL_TIM_Base_Stop_IT(&htim3);
			can_print = 1;
			ADC_counter = 0;
			state = 0;
			break;
		case 13:		// CLEAN LAST CHAR 	(<BCKSP>)
			printf("");
			state = 0;
			break;
		case 14:	// CLEAN STRINGs 	(<ESC>)
			printf("");
			state = 0;
			break;
		case 15:	// CLEAN STRINGs and repeat last one ($)
			printf("");
			state = 0;
			break;
		case 16:	// HELP 	(?)
			printf("Help Menu:\r\n");
			printf("\"MR\" - Memory Read (ex:MR.<addr>.<length>)\r\n");
			printf("\"MW\" - Memory Write (ex:MW.<addr>.<length>.<NUM>)\r\n");
			printf("\"MI\" - Make pin Input (ex:MI.<port(0..0A)>.<pin setting>)\r\n");
			printf("\"MO\" - Make pin Output (ex:MO.<port(0..0A)>.<pin setting>)\r\n");
			printf("\"RD\" - Read Digital Input (ex:RD.<port(0..0A)>.<pin setting>)\r\n");
			printf("\"WD\" - Write Digital Output (ex:WD.<port(0..0A)>.<pin setting>.<pin values>)\r\n");
			printf("\"RA\" - Read Analog (ADC) (ex:RA.<ADC_CH(0..F)>\r\n");
			printf("\"WG\" - Wave Generator (ex:wg.<wave_type>.<freq>)\r\n");
			printf("\"SP\" - Sampling Period (ex:SP.<timeunit>.<units>)\r\n");
			printf("\"AC\" - Analog Channel (ex:AC.<add3>)\r\n");
			printf("\"WG\" - Sample (ex:S)\r\n");
			printf("\"WG\" - Sample Only K Values(ex:S.<dig>)\r\n");
			printf("\"ST\" - Stop Sampling (ex:ST)\r\n");
			printf("\"<BCKSP>\" - Backspace\r\n");
			printf("\"<ESC>\" - Clean all characters received\r\n");
			printf("\"$\" - Clean all characters received and repeat the last one\r\n");
			printf("\"?\" - Display all valid commands\r\n");
			printf("\"VER\"- Firmware Version, our class number and work group \r\n");
			state = 0;
			break;
		case 17:
			printf("v3.0 [PL2 group9] - PI2 LEEIC2023\r\n");
			state = 0;
			break;
		default:    // ERROR
			printf("[ERRO]\r\n");
			break;
	}
	printf(">\r\n");
  }

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
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  init_UART3();
  HAL_TIM_Base_Start_IT(&htim4);
  setvbuf(stdin,NULL,_IONBF,0);

  printf(">\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  comandos(newMessage());
	  if(can_print == 1)
		  {
		      can_print = 0;
		  	  for(int x = 0;x<Samples ;x++)
		  	  {
		  		 printf("%.2f\r\n",ADC_Buffer[x]);
		  	  }
		  }

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
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
