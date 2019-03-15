
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"
#include "adc.h"
#include "dac.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

#include "string.h"
#include "stdlib.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

char delim = 0x20;
char *ms = "ms";
char *micro = "micro";
char *ptr;
char str[128];
struct func{
	int n_args;
	char* args[3];
}typedef func_t;

enum comando{
	MR,
	MW,
	MI,
	MO,
	RD,
	WD,
	RA,
	VR,
	SP,
	AC,
	S,
	NF,
	ST,
	FF,
	FN
}command;

int memory_read(func_t func);
int memory_write(func_t func);
int pin_input(func_t func);
int pin_output(func_t func);
int digital_read(func_t func);
int digital_write(func_t func);
int analog_read(func_t func);
int version(func_t func);
int sample_config(func_t func);
int adc_config(func_t func);
int sample(func_t func);

typedef int(*func_exec)(func_t );

static func_exec func_execute[] = {
    &memory_read,
    &memory_write,
    &pin_input,
    &pin_output,
    &digital_read,
    &digital_write,
    &analog_read,
		&version,
		&sample_config,
		&adc_config,
		&sample,
    NULL};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void get_arg(func_t* f, char* str);
enum comando get_command(char* s);
void code_exec(char* str);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void get_arg(func_t* f, char* str){
	int i=0;
	char *pt;
	pt = strtok(str, &delim);
	while(i < 3){
		pt = strtok(NULL, &delim);
		f->args[i] = pt;
		i++;
	}
	return; 
}
enum comando get_command(char* s){
	enum comando command;
	if(!strcmp(s, "MR"))
		command = MR;
	if(!strcmp(s, "MW"))
		command = MW;
	if(!strcmp(s, "MI"))
		command = MI;
	if(!strcmp(s, "MO"))
		command = MO;
	if(!strcmp(s, "RD"))
		command = RD;
	if(!strcmp(s, "WD"))
		command = WD;
	if(!strcmp(s, "RA"))
		command = RA;
	if(!strcmp(s, "VER"))
		command = VR;
	if(!strcmp(s, "SP"))
		command = SP;
	if(!strcmp(s, "AC"))
		command = AC;
	if(!strcmp(s, "FN"))
		command = FN;
	if(!strcmp(s, "FF"))
		command = FF;
	if(!strcmp(s, "S"))
		command = S;
	if(!strcmp(s, "ST"))
		command = ST;
	return command;
}
void code_exec(char* str){
	char aux[2];
	enum comando command;
	command = NF;
	func_t function;
	strcpy(aux, str);
	strtok(aux, &delim);
	command = get_command(aux);
	get_arg(&function, str);
	if(command == NF){
		printf("Invalid command\r\n");
		return;
	}
	else if(command > NF){
		switch((int)command){
			//ST
			case 12:
				HAL_TIM_Base_Stop_IT(&htim3);
				HAL_ADC_Stop(&hadc1);
				break;
			//FF
			case 13:
				printf("Filter off\r\n");
				FILTER_FLAG = 0;
				break;
			//FN
			case 14:
				printf("Filter on\r\n");
				FILTER_FLAG = 1;
				break;
		}
	}
	else 
		func_execute[command](function);
	
	memset(function.args, NULL, sizeof(function.args));
	return;
}

int memory_read(func_t func){
	int len;
	int i = 0;
	char* pt;
	int mem;
	char* ptr;
	printf("Memory Read\r\n");
	printf("addr: %s\r\n", func.args[0]);
	printf("len: %s\r\n", func.args[1]);
	mem = strtol(func.args[0], &ptr, 16);
	
	len = strtol(func.args[1], &ptr, 10);
	pt = (char*)mem;
	while(len){
		printf("Byte %d ===>", i);
		printf("ascii: %c  ", pt[i]);
		printf("hex: %x || ", pt[i]);
		printf("\r\n");
		len--;
		i++;
	}
	printf("\r\n");
	return 1;
}
	
int memory_write(func_t func){
	int len;
	char* pt;
	int i = 0;
	int mem;
	int byte;
	printf("Memory Write\r\n");
	printf("addr: %s\r\n", func.args[0]);
	printf("len: %s\r\n", func.args[1]);
	printf("byte: %s\r\n", func.args[2]);
	mem = strtol(func.args[0], &ptr, 16);
	
	len = strtol(func.args[1], &ptr, 10);
	
	byte = strtol(func.args[2], &ptr, 16);
	//strcpy(&byte, func.args[2]);
	if (mem < 0x20000000 || mem > 0x3FFFFFFF){
		printf("Invalid memory addr\r\n");
		printf("Try again\r\n");
		return -1;
	}
	
	pt = (char*)mem;
	while(len){
		*(pt + i) = (char)byte;
		len--;
		i++;
	}
	printf("Memory writen\r\n");
	return 1;
}
	
int pin_input(func_t func){
	GPIO_InitTypeDef GPIO_InitStruct;
	int pinSetting;
	char port;
	//getting port and pins from arguments
	strcpy(&port, func.args[0]);
	pinSetting = strtol(func.args[1], &ptr, 16);
 //-------------------------------------//

		switch ( port )
	{
		case 'A': 
		{
				GPIO_InitStruct.Pin = pinSetting;
				GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
				GPIO_InitStruct.Pull = GPIO_NOPULL;
				HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
				printf("Pin Set\r\n");
			//break;
			return 1;
		}
		case 'B':
		{
				GPIO_InitStruct.Pin = pinSetting;
				GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
				GPIO_InitStruct.Pull = GPIO_NOPULL;
				HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
				printf("Pin Set\r\n");
			//break;
			return 1;
		}
		case 'C':
		{
				GPIO_InitStruct.Pin = pinSetting;
				GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
				GPIO_InitStruct.Pull = GPIO_NOPULL;
				HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
				printf("Pin Set\r\n");
			//break;
			return 1;
		}
		case 'D':
		{
				GPIO_InitStruct.Pin = pinSetting;
				GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
				GPIO_InitStruct.Pull = GPIO_NOPULL;
				HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
				printf("Pin Set\r\n");
		//	break;
			return 1;
		}
		case 'E':
		{
				GPIO_InitStruct.Pin = pinSetting;
				GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
				GPIO_InitStruct.Pull = GPIO_NOPULL;
				HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
				printf("Pin Set\r\n");
		//	break;
			return 1;
		}
		case 'F':
		{
				GPIO_InitStruct.Pin = pinSetting;
				GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
				GPIO_InitStruct.Pull = GPIO_NOPULL;
				HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
				printf("Pin Set\r\n");
		//	break;
			return 1;
		}
		case 'G':
		{
				GPIO_InitStruct.Pin = pinSetting;
				GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
				GPIO_InitStruct.Pull = GPIO_NOPULL;
				HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
				printf("Pin Set\r\n");
			return 1;
		}		
		default:
			printf("Pin not Set\r\n");
			printf("Retry\r\n");
			return -1;
	}	
}
	
int pin_output(func_t func){
	GPIO_InitTypeDef GPIO_InitStruct;
	uint16_t pinSetting;
	char port;
	//getting port and pins from arguments
	strcpy(&port, func.args[0]);
	pinSetting = strtol(func.args[1], &ptr, 16);
 //-------------------------------------//

	switch(port)
		{
			case 'A': 
			{
				GPIO_InitStruct.Pin = pinSetting;
				GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
				GPIO_InitStruct.Pull = GPIO_NOPULL;
				HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
				printf("Pin Set\r\n");
			//break;
			return 1;
			}
			case 'B':
			{
				GPIO_InitStruct.Pin = pinSetting;
				GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
				GPIO_InitStruct.Pull = GPIO_NOPULL;
				HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
				printf("Pin Set\r\n");
			//break;
			return 1;
			}
			case 'C':
			{
				GPIO_InitStruct.Pin = pinSetting;
				GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
				GPIO_InitStruct.Pull = GPIO_NOPULL;
				HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
				printf("Pin Set\r\n");
			//break;
			return 1;
			}
			case 'D':
			{
				GPIO_InitStruct.Pin = pinSetting;
				GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
				GPIO_InitStruct.Pull = GPIO_NOPULL;
				HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
				printf("Pin Set\r\n");
		//	break;
			return 1;
			}
			case 'E':
			{
				GPIO_InitStruct.Pin = pinSetting;
				GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
				GPIO_InitStruct.Pull = GPIO_NOPULL;
				HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
				printf("Pin Set\r\n");
		//	break;
			return 1;
			}
			case 'F':
			{
				GPIO_InitStruct.Pin = pinSetting;
				GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
				GPIO_InitStruct.Pull = GPIO_NOPULL;
				HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
				printf("Pin Set\r\n");
		//	break;
			return 1;
			}
			case 'G':
			{
				GPIO_InitStruct.Pin = pinSetting;
				GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
				GPIO_InitStruct.Pull = GPIO_NOPULL;
				HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
				printf("Pin Set\r\n");
			return 1;
			}		
			default:
			{
			printf("Pin not Set\r\n");
			printf("Retry\r\n");
			return -1;
			}
		}
}

int digital_read(func_t func){
	uint16_t data;
	char* pt;
	char port;
	int pinSetting;
	pinSetting = strtol(func.args[1], &pt, 16);
	strcpy(&port, func.args[0]);
	
switch(port){
		//A
		case 'A':	{
			data = (GPIOA->IDR & pinSetting);
			break;
			}
		//B
		case 'B':{
			data = (GPIOB->IDR & pinSetting);
			break;
			}
		//C
		case 'C':	{
			data = (GPIOC->IDR & pinSetting);
			break;
			}
		//D
		case 'D':	{
			data = (GPIOD->IDR & pinSetting);
			break;
			}
		//E
		case 'E':	{
			data = (GPIOE->IDR & pinSetting);
			break;
			}
		//F
		case 'F':	{
			data = (GPIOF->IDR & pinSetting);
			break;
			}
		//G
		case 'G':	{
			data = (GPIOG->IDR & pinSetting);
			break;
			}
		//H
		case 'H':	{
			data = (GPIOH->IDR & pinSetting);
			break;
			}
		}
	printf("Pin information: %d\r\n", data);
	return 1;
}

int digital_write(func_t func){
	char byte;
	char* pt;
	char port;
	int pinSetting;
	pinSetting = strtol(func.args[1], &pt, 16);
	strcpy(&port, func.args[0]);
	strcpy(&byte, func.args[2]);
	int piN2Write = 255 - pinSetting;
	piN2Write = pinSetting & byte;
	
switch ( port )
	{
		case 'A': 
		{
			GPIOD->ODR &=~pinSetting;
			GPIOD->ODR |= piN2Write;
			return ( GPIOD->ODR & pinSetting);
		}
		case 'B':
		{
			GPIOD->ODR &=~pinSetting;
			GPIOD->ODR |= piN2Write;
			return ( GPIOD->ODR & pinSetting);
		}
		case 'C':
		{
			GPIOD->ODR &=~pinSetting;
			GPIOD->ODR |= piN2Write;
			return ( GPIOD->ODR & pinSetting);
		}
		case 'D':
		{
			GPIOD->ODR &=~pinSetting;
			GPIOD->ODR |= piN2Write;
			return ( GPIOD->ODR & pinSetting);
		}
		case 'E':
		{
			GPIOD->ODR &=~pinSetting;
			GPIOD->ODR |= piN2Write;
			return ( GPIOD->ODR & pinSetting);
		}
		case 'F':
		{
			GPIOD->ODR &=~pinSetting;
			GPIOD->ODR |= piN2Write;
			return ( GPIOD->ODR & pinSetting);
		}
		case 'G':
		{
			GPIOD->ODR &=~pinSetting;
			GPIOD->ODR |= piN2Write;
			return ( GPIOD->ODR & pinSetting);
		}		
		default:
			return -1;
	}	
}

int analog_read(func_t func){
	printf("Not implemented, sowy UwU\r\n");
return 1;}

int version(func_t func){
	printf("v1.0, grupo 12 ==> Eduardo Sousa e Rita Rodrigues\r\n");
	return 1;
}

int sample_config(func_t func){
	float period;
	int val_reload;
	
	printf("_______Sample Config_______\r\n");
	
	period = strtol(func.args[1],&ptr, 10);
	
	if(!strcmp(func.args[0], ms)){
		period = period/1000;
	}
	else if(!strcmp(func.args[0], micro)){
		period = period/1000000;
	}
	
	val_reload = (period*108000000)/864;
	
	MX_TIM3_Init(val_reload);
	
	return 1;
}
	
int adc_config(func_t func){
	uint8_t channel;
	
	printf("_______ADC Config_______\r\n");
	
	channel = strtol(func.args[0], &ptr, 10);
	
	MX_ADC1_Init(channel);
	return 1;
}
	
int sample(func_t func){
	
	printf("_______Sampling_______\r\n");
	if (func.args[0] != NULL)
	k_values = strtol(func.args[0], &ptr, 10);
	else k_values = 0;
	HAL_ADC_Start(&hadc1);
	HAL_TIM_Base_Start_IT(&htim3);
	return 1;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART3_UART_Init();
  MX_ADC1_Init(1);
  MX_DAC_Init();
  MX_TIM2_Init();
  MX_TIM3_Init(62499);

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	
	init_UART3();
	printf("Hello STM32\r\n");
	printf(">");
//	printf("This is a echo program made with HAL API\r\n");
//	printf("Type a message and press enter\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(messageReceived()){
			printf("\r\n");
			//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
			strcpy(str, (char*)Rx_Buffer);
			code_exec(str);
			printf("Type another command\r\n");
			printf(">");
		}
		HAL_Delay(50);
	}
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
