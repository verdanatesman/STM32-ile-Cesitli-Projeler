/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define WRCFG 0x01 //Write Configuration Registers
#define RDCFG 0x02 // Read config
#define RDCV 0x04 // Read cells
#define STCVAD 0x10 // Start all A/D's - poll status
#define RDFLG 0x06 //Read Flags
#define RDTMP 0x08 //Read Temperatures
#define STCDC 0x60 //A/D converter and poll Status
#define STOWAD 0x20 //Start Test - poll status
#define STTMPAD 0x30// Temperature Reading - ALL
#define address 0x80

#include "string.h"
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
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define BUFFERSIZE                       (COUNTOF(aTxBuffer) - 1)
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))



uint8_t  uartTxBuffer[] = "\r\n SLAVE SPI - INTERRUPT MODDA  IKI KART HABERLESMESI \r\n ";
uint8_t  aTxBuffer[] = "SPI SLAVE'DEN CEVAP: SLAVE' DEN MESAJ ALINDI";
uint8_t  aRxBuffer[43]= {0};

uint8_t data1[2];
uint8_t data2[2];
uint8_t data3[1];
uint8_t data4[2];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  for(volatile uint8_t i=0;i<16;i++)
    {
  	  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
  	  HAL_Delay(100);
    }
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

      if(HAL_UART_Transmit(&huart2, (uint8_t*)uartTxBuffer, 55, 5000)!= HAL_OK)
      {
        Error_Handler();
      }
    /* CLK sinyalinin d??zg??n ??ekilde a??a???? ??ekilmesi i??in SPI g??nderme / alma i??levlerini ??a????rmadan ??nce SPI blo??u etkinle??tirilir.
          Aksi takdirde, SPI CLK sinyali bu kartta temiz de??ildir ve aktar??m s??ras??nda hatalara neden olur */

    __HAL_SPI_ENABLE(&hspi2);

//    switch(HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, 43, 5000))
//    {
//      case HAL_OK:
//    	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
//   	      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
//        break;
//
//      case HAL_TIMEOUT:
//    	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
//   	      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
//   	      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
//      case HAL_ERROR:
//    	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
//   	      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
//   	      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
//        break;
//      default:
//        break;
//    }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  // YAZ ??????N TRANSM??T
	  //while(HAL_SPI_GetState(&hspi2)!= HAL_SPI_STATE_READY)
	  HAL_SPI_Receive(&hspi2, data1, 2, 2000);
	  HAL_Delay(1000);
	  HAL_UART_Transmit(&huart2,data1, 2, 2000);
	/*  //OKUNANLAR ??????N GELEN TRANSM??T
	  HAL_SPI_Receive(&hspi2, data2, 2, 2000);
	  HAL_Delay(1000);
	  HAL_UART_Transmit(&huart2, data2, 2, 2000);
	  // VOLTAJ DEGERLER?? KOMUT G??TT??
	  HAL_SPI_Receive(&hspi2, data3, 1, 2000);
	  HAL_Delay(1000);
	  HAL_UART_Transmit(&huart2, data3, 1, 2000);
	  // VOLTAJ DEGERLER?? GEL??YOR
	  HAL_SPI_Receive(&hspi2, data4, 2, 2000);
	  HAL_Delay(1000);
	  HAL_UART_Transmit(&huart2, data4, 2, 2000);*/

	  	  /*while(HAL_SPI_GetState(&hspi2)!= HAL_SPI_STATE_READY);
	 	  switch(HAL_SPI_Receive(&hspi2, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, 43, 5000))
	 	  {
	 	    case HAL_OK:
	 	    	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
	 	   	      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
	 	   	      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
	 	   	      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);

	 	      break;

	 	    case HAL_TIMEOUT:
	 	    	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
	 	   	      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
	 	   	      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
	 	    	break;
	 	    case HAL_ERROR:
	 	    	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
	 	   	      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
	 	   	      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
	 	      break;
	 	    default:
	 	      break;
	 	  }
  	      while(HAL_UART_GetState(&huart2)!= HAL_UART_STATE_READY);
  	      if(HAL_UART_Transmit(&huart2, (uint8_t*)aRxBuffer, 43, 5000)!= HAL_OK)
  		  {
  		    Error_Handler();
  		  }*/


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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
//{
//	HAL_SPI_TransmitReceive_IT(&hspi2, data4, data1, 2);
//	  HAL_UART_Transmit_IT(&huart2, data1, 2);
//

	  /*if(HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t *)aTxBuffer, (uint8_t *)aRxBuffer, 43) != HAL_OK)
	  {
		  Error_Handler();
	  }
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, SET);
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, SET);
	  while(HAL_UART_GetState(&huart2)!= HAL_UART_STATE_READY);
	  if(HAL_UART_Transmit(&huart2, (uint8_t*)aRxBuffer, 43, 5000)!= HAL_OK)
	    {
		  Error_Handler();
		}*/
//}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
