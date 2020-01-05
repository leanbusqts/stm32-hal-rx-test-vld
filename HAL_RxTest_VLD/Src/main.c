
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
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
char CRxBuff[2];
char RxBuff[4];
char RxAxis[1];
char RxMult[1];
char RxEnc[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/


/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	/** 
	#define UART_IT_MASK  0x0000FFFFU
	#define __HAL_UART_ENABLE_IT(__HANDLE__, __INTERRUPT__)   ((((__INTERRUPT__) >> 28U) == 1U)? ((__HANDLE__)->Instance->CR1 |= ((__INTERRUPT__) & UART_IT_MASK)): \
                                                           (((__INTERRUPT__) >> 28U) == 2U)? ((__HANDLE__)->Instance->CR2 |=  ((__INTERRUPT__) & UART_IT_MASK)): \
                                                        ((__HANDLE__)->Instance->CR3 |= ((__INTERRUPT__) & UART_IT_MASK)))
	@brief  Enable the specified UART interrupt.
  * @param  __HANDLE__ specifies the UART Handle.
  *         This parameter can be UARTx where x: 1, 2, 3, 4, 5, 6, 7 or 8 to select the USART or 
  *         UART peripheral.
  * @param  __INTERRUPT__ specifies the UART interrupt source to enable.
  *          This parameter can be one of the following values:
  *            @arg UART_IT_CTS:  CTS change interrupt
  *            @arg UART_IT_LBD:  LIN Break detection interrupt
  *            @arg UART_IT_TXE:  Transmit Data Register empty interrupt
  *            @arg UART_IT_TC:   Transmission complete interrupt
  *            @arg UART_IT_RXNE: Receive Data register not empty interrupt
  *            @arg UART_IT_IDLE: Idle line detection interrupt
  *            @arg UART_IT_PE:   Parity Error interrupt
  *            @arg UART_IT_ERR:  Error interrupt(Frame error, noise error, overrun error)
  * @retval None
  */
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_TC|UART_IT_RXNE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		/**
		HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
		* @brief  Receives an amount of data in non blocking mode. 
		* @param  huart: pointer to a UART_HandleTypeDef structure that contains
		*                the configuration information for the specified UART module.
		* @param  pData: Pointer to data buffer
		* @param  Size: Amount of data to be received
		* @retval HAL status
		*/
		//HAL_UART_Receive_IT(&huart1, (uint8_t *)RxBuff, strlen(RxBuff)); // receive the data and seve in rxBuff		
		/**
		extern _ARMABI void *memcpy(void * __restrict s1,
                  const void * __restrict s2, size_t n) __attribute__((__nonnull__(1,2)));
		* copies n characters from the object pointed to by s2 into the object
		* pointed to by s1. If copying takes place between objects that overlap,
		* the behaviour is undefined.
		* Returns: the value of s1.
		*/
		memcpy(RxAxis, &RxBuff[0], 1);	
		/**	
		int strncmp(const char * s1, const char * s2, size_t n) __attribute__((__nonnull__(1,2)));
    * compares not more than n characters (characters that follow a null
    * character are not compared) from the array pointed to by s1 to the array
    * pointed to by s2.
    * Returns: an integer greater than, equal to, or less than zero, according
    *          as the string pointed to by s1 is greater than, equal to, or
    *          less than the string pointed to by s2.
    */
		if(strncmp(RxAxis, "1", 1) == 0)	//Compara rxBuff con "_"
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
			//HAL_Delay(100);
		} else if(strncmp(RxAxis, "2", 1) == 0)
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
				//HAL_Delay(100);
			} else if(strncmp(RxAxis, "3", 1) == 0)
				{
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
					//HAL_Delay(100);
				}
		memcpy(RxMult, &RxBuff[1], 1);
		if(strncmp(RxMult, "1", 1) == 0)	
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			//HAL_Delay(100);
		} else if(strncmp(RxMult, "2", 1) == 0)
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
				//HAL_Delay(100);
			} else if(strncmp(RxMult, "3", 1) == 0)
				{
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
					//HAL_Delay(100);
				}
		memcpy(RxEnc, &RxBuff[2], 2);
		if(strncmp(RxEnc, "00", 2) == 0)	
		{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
		} else if(strncmp(RxEnc, "01", 2) == 0)	
			{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
			} else if(strncmp(RxEnc, "10", 2) == 0)
				{
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
				} else if(strncmp(RxEnc, "11", 2) == 0)
					{
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_2;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 
                           PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
		if(strncmp(CRxBuff, "$", 1) == 0)	//Compara RxBuff con "$"
			HAL_NVIC_SystemReset();
		//AXIS X
		else if(strncmp(CRxBuff, "(", 1) == 0)
		{
			RxBuff[0] = '1';
			RxBuff[1] = '1';
			RxBuff[2] = '0';
			RxBuff[3] = '0';
		}
		else if(strncmp(CRxBuff, "*", 1) == 0)	
		{
			RxBuff[0] = '1';
			RxBuff[1] = '1';
			RxBuff[2] = '0';
			RxBuff[3] = '1';
		}
		else if(strncmp(CRxBuff, ",", 1) == 0)	
		{
			RxBuff[0] = '1';
			RxBuff[1] = '1';
			RxBuff[2] = '1';
			RxBuff[3] = '0';
		}
		else if(strncmp(CRxBuff, ".", 1) == 0)	
		{
			RxBuff[0] = '1';
			RxBuff[1] = '1';
			RxBuff[2] = '1';
			RxBuff[3] = '1';
		}
		else if(strncmp(CRxBuff, "0", 1) == 0)	
		{
			RxBuff[0] = '1';
			RxBuff[1] = '2';
			RxBuff[2] = '0';
			RxBuff[3] = '0';
		}
		else if(strncmp(CRxBuff, "2", 1) == 0)	
		{
			RxBuff[0] = '1';
			RxBuff[1] = '2';
			RxBuff[2] = '0';
			RxBuff[3] = '1';
		}
		else if(strncmp(CRxBuff, "4", 1) == 0)	
		{
			RxBuff[0] = '1';
			RxBuff[1] = '2';
			RxBuff[2] = '1';
			RxBuff[3] = '0';
		}
		else if(strncmp(CRxBuff, "6", 1) == 0)	
		{
			RxBuff[0] = '1';
			RxBuff[1] = '2';
			RxBuff[2] = '1';
			RxBuff[3] = '1';
		}
		else if(strncmp(CRxBuff, "8", 1) == 0)	
		{
			RxBuff[0] = '1';
			RxBuff[1] = '3';
			RxBuff[2] = '0';
			RxBuff[3] = '0';
		}
		else if(strncmp(CRxBuff, ":", 1) == 0)	
		{
			RxBuff[0] = '1';
			RxBuff[1] = '3';
			RxBuff[2] = '0';
			RxBuff[3] = '1';
		}
		else if(strncmp(CRxBuff, "<", 1) == 0)	
		{
			RxBuff[0] = '1';
			RxBuff[1] = '3';
			RxBuff[2] = '1';
			RxBuff[3] = '0';
		}
		else if(strncmp(CRxBuff, ">", 1) == 0)	
		{
			RxBuff[0] = '1';
			RxBuff[1] = '3';
			RxBuff[2] = '1';
			RxBuff[3] = '1';
		}
		//AXIS Y
		else if(strncmp(CRxBuff, "H", 1) == 0)	
		{
			RxBuff[0] = '2';
			RxBuff[1] = '1';
			RxBuff[2] = '0';
			RxBuff[3] = '0';
		}
		else if(strncmp(CRxBuff, "J", 1) == 0)	
		{
			RxBuff[0] = '2';
			RxBuff[1] = '1';
			RxBuff[2] = '0';
			RxBuff[3] = '1';
		}
		else if(strncmp(CRxBuff, "L", 1) == 0)	
		{
			RxBuff[0] = '2';
			RxBuff[1] = '1';
			RxBuff[2] = '1';
			RxBuff[3] = '0';
		}
		else if(strncmp(CRxBuff, "N", 1) == 0)	
		{
			RxBuff[0] = '2';
			RxBuff[1] = '1';
			RxBuff[2] = '1';
			RxBuff[3] = '1';
		}
		else if(strncmp(CRxBuff, "P", 1) == 0)	
		{
			RxBuff[0] = '2';
			RxBuff[1] = '2';
			RxBuff[2] = '0';
			RxBuff[3] = '0';
		}
		else if(strncmp(CRxBuff, "R", 1) == 0)	
		{
			RxBuff[0] = '2';
			RxBuff[1] = '2';
			RxBuff[2] = '0';
			RxBuff[3] = '1';
		}
		else if(strncmp(CRxBuff, "T", 1) == 0)	
		{
			RxBuff[0] = '2';
			RxBuff[1] = '2';
			RxBuff[2] = '1';
			RxBuff[3] = '0';
		}
		else if(strncmp(CRxBuff, "V", 1) == 0)	
		{
			RxBuff[0] = '2';
			RxBuff[1] = '2';
			RxBuff[2] = '1';
			RxBuff[3] = '1';
		}
		else if(strncmp(CRxBuff, "X", 1) == 0)	
		{
			RxBuff[0] = '2';
			RxBuff[1] = '3';
			RxBuff[2] = '0';
			RxBuff[3] = '0';
		}
		else if(strncmp(CRxBuff, "Z", 1) == 0)	
		{
			RxBuff[0] = '2';
			RxBuff[1] = '3';
			RxBuff[2] = '0';
			RxBuff[3] = '1';
		}
		else if(strncmp(CRxBuff, "&", 1) == 0)	
		{
			RxBuff[0] = '2';
			RxBuff[1] = '3';
			RxBuff[2] = '1';
			RxBuff[3] = '0';
		}
		else if(strncmp(CRxBuff, "^", 1) == 0)	
		{
			RxBuff[0] = '2';
			RxBuff[1] = '3';
			RxBuff[2] = '1';
			RxBuff[3] = '1';
		}
		//AXIS Z
		else if(strncmp(CRxBuff, "h", 1) == 0)	
		{
			RxBuff[0] = '3';
			RxBuff[1] = '1';
			RxBuff[2] = '0';
			RxBuff[3] = '0';
		}
		else if(strncmp(CRxBuff, "j", 1) == 0)	
		{
			RxBuff[0] = '3';
			RxBuff[1] = '1';
			RxBuff[2] = '0';
			RxBuff[3] = '1';
		}
		else if(strncmp(CRxBuff, "l", 1) == 0)	
		{
			RxBuff[0] = '3';
			RxBuff[1] = '1';
			RxBuff[2] = '1';
			RxBuff[3] = '0';
		}
		else if(strncmp(CRxBuff, "n", 1) == 0)	
		{
			RxBuff[0] = '3';
			RxBuff[1] = '1';
			RxBuff[2] = '1';
			RxBuff[3] = '1';
		}
		else if(strncmp(CRxBuff, "p", 1) == 0)	
		{
			RxBuff[0] = '3';
			RxBuff[1] = '2';
			RxBuff[2] = '0';
			RxBuff[3] = '0';
		}
		else if(strncmp(CRxBuff, "r", 1) == 0)	
		{
			RxBuff[0] = '3';
			RxBuff[1] = '2';
			RxBuff[2] = '0';
			RxBuff[3] = '1';
		}
		else if(strncmp(CRxBuff, "t", 1) == 0)	
		{
			RxBuff[0] = '3';
			RxBuff[1] = '2';
			RxBuff[2] = '1';
			RxBuff[3] = '0';
		}
		else if(strncmp(CRxBuff, "v", 1) == 0)	
		{
			RxBuff[0] = '3';
			RxBuff[1] = '2';
			RxBuff[2] = '1';
			RxBuff[3] = '1';
		}
		else if(strncmp(CRxBuff, "x", 1) == 0)	
		{
			RxBuff[0] = '3';
			RxBuff[1] = '3';
			RxBuff[2] = '0';
			RxBuff[3] = '0';
		}
		else if(strncmp(CRxBuff, "z", 1) == 0)	
		{
			RxBuff[0] = '3';
			RxBuff[1] = '3';
			RxBuff[2] = '0';
			RxBuff[3] = '1';
		}
		else if(strncmp(CRxBuff, "|", 1) == 0)	
		{
			RxBuff[0] = '3';
			RxBuff[1] = '3';
			RxBuff[2] = '1';
			RxBuff[3] = '0';
		}
		else if(strncmp(CRxBuff, "~", 1) == 0)	
		{
			RxBuff[0] = '3';
			RxBuff[1] = '3';
			RxBuff[2] = '1';
			RxBuff[3] = '1';
		}
}
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
