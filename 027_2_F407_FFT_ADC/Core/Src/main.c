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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#define ARM_MATH_CM4
#include "arm_math.h"

uint16_t adc_value[2], adc_value2, adc_value1;


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
ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */


#define         FFT_SIZE       4096 //fft boyutunuz 2 nin katları şeklinde olmalı 16 - 4096 arasında olmalı
float32_t       fft_in[FFT_SIZE];//time domaindeki sinyal arayimizi içerir
float32_t       fft_out[FFT_SIZE];//fftsi alınmış frekans domaindeki çıktımız olacak
float32_t       fft_out_abs[FFT_SIZE / 2];//fft si alınmış değerin absolute (mutlak) değerin yani sinyalin bize power density ( güç yoğunluuğu)
float32_t       fft_out_abs_norm[FFT_SIZE / 2];//absolute değerin daha alt değerlere normalize etmek için
arm_rfft_fast_instance_f32  rfft_f32;//cmsis kütüphanesi kullanmak için tanımladığımız bir değişken
uint32_t i;
float32_t f_step, max_value;
uint32_t max_index;
float32_t frequency;

float32_t test = 0, test1 = 0, test2 = 0;



uint32_t i;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Read_ADC()
{
		  HAL_ADC_Start(&hadc1);
		  if(HAL_ADC_PollForConversion(&hadc1,100000) == HAL_OK)
		  fft_in[i] = HAL_ADC_GetValue(&hadc1);
		  HAL_ADC_Stop(&hadc1);
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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */



	  for(i=0; i<4096; i++)
	  {
		  Read_ADC();
		  HAL_Delay(1);
	  }


//	  			arm_cfft_radix4_instance_f32 S;
//	  		  	float32_t maxValue;


	  		  	/* Initialize the CFFT/CIFFT module */
//	  		  	arm_cfft_radix4_init_f32(&S, fftSize, ifftFlag, doBitReverse);

	  		  	/* Process the data through the CFFT/CIFFT module */
//	  		  	arm_cfft_radix4_f32(&S, testInput_f32_10khz);


	  		  	/* Process the data through the Complex Magnitude Module for
	  		  	calculating the magnitude at each bin */
//	  		  	arm_cmplx_mag_f32(testInput_f32_10khz, testOutput,fftSize);

	  		  	/* Calculates maxValue and returns corresponding BIN value */
//	  		  	arm_max_f32(testOutput, fftSize, &maxValue, &testIndex);

/*
	  		  	for(int j=0; j<1024; j++)
	  		  	{
	  		  		adc_value1=testOutput[j];
	  		  		HAL_Delay(1);
	  		  	}
*/


	  	/* ----------------------------------------------------------------------
	  	** Loop here if the signals fail the PASS check.
	  	** This denotes a test failure
	  	** ------------------------------------------------------------------- */


	  /*// sinüs oluşturmak için gerekli kısım
	  	  	  unsigned int fs=1000;
	  	 	  unsigned int amp=10;
	  	 	  unsigned int freq=100;

	  	 	  for (i = 0; i < FFT_SIZE; i++)
	  		{
	  			fft_in[i] = (amp * ((float32_t)sin(2.0f * PI * freq * (float32_t)i / (float32_t)fs)));
	  			//test=fft_in[i];
	  		}
	  		HAL_Delay(200);
	  */


//	   unsigned int fs=1000;

	  // f_step = (float32_t)3.38/1000000 / (float32_t)FFT_SIZE; //100 hz ile örneklenen bir sinyal için fft_in dizisine girdi olarak sağlıyoruz bizim 4096 karakterimiz varsa
	  	  	  	  	  	  	  	  	  	  	  	  	//stepimiz 100/4095 den 0.0244 olur buda absolute değerlerimizi yani mag değerlerimizi her bir frekans aralığı
	  	  	  	  	  	  	  	  	  	  	  	  	//bu aralık ile artıyor demektir yani arayin ilk elemanı 0.024 olur ikincisi 0.048 gibi


	  arm_rfft_fast_init_f32(&rfft_f32, FFT_SIZE);  //bu fonksiyonda fft init ayarlarımızı yapıyoruz
	  	  	  	  	  	  	  	  	  	  	  	    //burada cmsis kütüphanemizin için tanımladığımız değişkenimizi ve dizimizin boyutunu tanımlıyoruz

	  arm_rfft_fast_f32(&rfft_f32, fft_in, fft_out, 0);	//Bu fonksiyonda ise fft alıyor time domaindeki değerleri frekans domaine dönüştürüyor.
	   	   	   	   	   	   	   	   	   	   	   	   	   	//sondaki sıfır tanımlaması tersi demek time domaindeki bir
	  												    // frekansı domeninde bir sinyale çevirmek istiyorsanız sıfır kullanmalıyız.

	  arm_cmplx_mag_f32(fft_out, fft_out_abs, FFT_SIZE / 2);  //absolute değerini yani karelerinin karekökünü alıyor
	  												  //sinyalimiz power density yani güç yoğunluğunu alıyor


	  fft_out_abs_norm[0] = fft_out_abs[0] / FFT_SIZE;
	  for (i = 1; i < FFT_SIZE; i++)
	  		{
	  			fft_out_abs_norm[i] = (2.0f * fft_out_abs[i]) / (float32_t)FFT_SIZE; //Verimizi normalize etme işlemi buda fft değerimizi 2 ile çarpıp fft_size a bölmek
	  		}

	  fft_out_abs_norm[0]=0;//0. indisi sıfıra çekiyor

	  arm_max_f32(fft_out_abs_norm, FFT_SIZE, &max_value, &max_index);	//aray içerisinden max değerini veriyor.Bunun önemi bize en yüksek genlikli maximum değeri veriyor.
	    	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	//Bu değerin hangi indis olduğunu veriyor bu indiside f_step ile çarparsak bize hangi sinyalin daha ağırlıklı	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	 //olduğunu veriyor
	  frequency = max_index * f_step;	//max index ile step'i çarptığımız zamanda bize frekans değerinini verir.

	  for(int j=0; j<FFT_SIZE; j++)
	  {
		  adc_value1=fft_out_abs_norm[j];
		 // HAL_Delay(5);
	  }

	  HAL_Delay(50);
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
