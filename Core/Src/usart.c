/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
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
#include "usart.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "string.h"

#define	BSPACE 0x08
#define	DOLLAR 0x24
#define ESC	0x1B
#define CR	0x0D

//UART3 RX data structures
uint8_t UART3Rx_Buffer[128];
uint8_t Rx_Buffer[128];
volatile uint8_t UART3Rx_index;
int ISR_DONE = 0;
static int bckspc_flag;
static int dollar_flag;
static int esc_flag;
static int end_flag;

/* USER CODE END 0 */

UART_HandleTypeDef huart3;

/* USART3 init function */

void MX_USART3_UART_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();
  
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

  /* USER CODE BEGIN USART3_MspInit 1 */

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

//redifine the stdout
int fputc(int ch, FILE *f){
	HAL_UART_Transmit(&huart3, (uint8_t*)&ch, 1, 100);
	return ch;
}

//implemantation of UART ISR
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart){
	if (huart->Instance == USART3){		//current UART?
		if(UART3Rx_Buffer[UART3Rx_index] != 0 && UART3Rx_Buffer[UART3Rx_index] != BSPACE){
			//printf("%c", UART3Rx_Buffer[UART3Rx_index]);
			HAL_UART_Transmit_IT(&huart3, &UART3Rx_Buffer[UART3Rx_index], 1);
		}		
		if(UART3Rx_Buffer[UART3Rx_index] == BSPACE){
			UART3Rx_Buffer[UART3Rx_index] = NULL;
			UART3Rx_Buffer[UART3Rx_index - 1] = NULL;
			UART3Rx_index --;
			UART3Rx_index &= ~(1<<7);
			bckspc_flag = 1;
			HAL_UART_Receive_IT(&huart3, &UART3Rx_Buffer[UART3Rx_index], 1);
			return;
		}
		else if(UART3Rx_Buffer[UART3Rx_index] == DOLLAR){
			dollar_flag = 1;
			HAL_UART_Receive_IT(&huart3, &UART3Rx_Buffer[UART3Rx_index], 1);
			return;
		}
		else if(UART3Rx_Buffer[UART3Rx_index] == ESC){
			esc_flag = 1;
			HAL_UART_Receive_IT(&huart3, &UART3Rx_Buffer[UART3Rx_index], 1);
			return;
		}
		else if(UART3Rx_Buffer[UART3Rx_index] == CR){
			UART3Rx_Buffer[UART3Rx_index] = 0x20;
			UART3Rx_index = 0;
			end_flag = 1;
			HAL_UART_Receive_IT(&huart3, &UART3Rx_Buffer[UART3Rx_index], 1);
			return;
		}
		//HAL_UART_Transmit_IT(&huart3, &UART3Rx_Buffer[UART3Rx_index], 1);
		UART3Rx_index++;
		UART3Rx_index &= ~(1<<7); //keep index inside the limits
		// set the interrupt for UART3 Rx again
		HAL_UART_Receive_IT(&huart3, &UART3Rx_Buffer[UART3Rx_index], 1);
	}
	return;
}

int messageReceived(){
	if (dollar_flag){
		dollar_flag = 0;
		while(UART3Rx_index > 0){
			printf("%c", BSPACE);
			UART3Rx_index--;
		}
		printf("%c", BSPACE);
		UART3Rx_index = 0;
		return 1;
	}
	else if (esc_flag){
		esc_flag = 0;
		while(UART3Rx_index > 0){
			printf("%c", BSPACE);
			UART3Rx_index--;
		}
		printf("%c", BSPACE);
		UART3Rx_index = 0;
		return 0;
	}
	else if (bckspc_flag){
		bckspc_flag = 0;
		printf("%c", BSPACE);
		return 0;
	}
	else if (end_flag){
		end_flag = 0;
		memset((char*)Rx_Buffer, NULL, sizeof(Rx_Buffer));
		strcpy((char*)Rx_Buffer, (char*)UART3Rx_Buffer);
		memset((char*)UART3Rx_Buffer, NULL, sizeof(UART3Rx_Buffer));
		return 1;
	}
return 0;
}

void init_UART3(){
	// set the interrupt for UART3 Rx
	HAL_UART_Receive_IT(&huart3, &UART3Rx_Buffer[UART3Rx_index], 1);
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
