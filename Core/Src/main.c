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
#include "senser_FFT.h"
#include <string.h>
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	H_IDLE, H_ERROR, H_BUSY, H_END
} H_simple_states;
typedef enum {
	H_P_ERROR, H_P_FIND_F, H_P_FIXED_V
} H_porgram_states;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */
void SET_EBN(uint32_t F, uint32_t V);
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float32_t amplitude = 20;
uint32_t output = 20;

uint16_t Data_adc[256];
H_simple_states DMA_states = H_IDLE;
H_porgram_states Now_states = H_P_FIND_F;

uint32_t now_frequency = 60;
uint32_t max_value_frequency = 0;

uint16_t Fdata[512] = { 3019.000, 2036.000, 1084.000, 822.000, 730.000,
		1846.000, 2864.000, 2753.000, 2472.000, 1516.000, 897.000, 794.000,
		1150.000, 2384.000, 2633.000, 3222.000, 2031.000, 1309.000, 455.000,
		945.000, 1370.000, 2268.000, 3108.000, 2948.000, 1563.000, 1145.000,
		383.000, 781.000, 1997.000, 2935.000, 2704.000, 2506.000, 1545.000,
		862.000, 945.000, 1370.000, 2508.000, 3203.000, 2840.000, 2236.000,
		1265.000, 468.000, 1097.000, 1586.000, 2581.000, 2762.000, 2422.000,
		1794.000, 987.000, 404.000, 1560.000, 2038.000, 3221.000, 3108.000,
		1897.000, 978.000, 805.000, 664.000, 1989.000, 2930.000, 2883.000,
		2348.000, 1417.000, 618.000, 861.000, 1168.000, 2082.000, 3160.000,
		2985.000, 2344.000, 1006.000, 775.000, 493.000, 1670.000, 2645.000,
		2816.000, 2942.000, 1938.000, 1107.000, 757.000, 1278.000, 2132.000,
		2905.000, 2802.000, 2477.000, 1143.000, 532.000, 481.000, 1240.000,
		2572.000, 3256.000, 2637.000, 1664.000, 1130.000, 408.000, 1076.000,
		2128.000, 3021.000, 2752.000, 2482.000, 1472.000, 475.000, 401.000,
		1260.000, 2088.000, 3228.000, 2778.000, 2091.000, 889.000, 810.000,
		798.000, 1549.000, 2424.000, 2803.000, 2522.000, 1562.000, 530.000,
		765.000, 1260.000, 2368.000, 3071.000, 2788.000, 2090.000, 1464.000,
		350.000, 941.000, 1344.000, 2624.000, 2922.000, 2464.000, 1579.000,
		1010.000, 781.000, 1116.000, 2309.000, 2673.000, 2832.000, 2280.000,
		1102.000, 504.000, 889.000, 1528.000, 2701.000, 2912.000, 2608.000,
		2142.000, 1200.000, 374.000, 839.000, 2170.000, 2532.000, 2834.000,
		2690.000, 1247.000, 462.000, 896.000, 1392.000, 2596.000, 2706.000,
		3097.000, 2216.000, 1275.000, 486.000, 780.000, 1619.000, 2548.000,
		3124.000, 2388.000, 1836.000, 742.000, 876.000, 1002.000, 2457.000,
		2683.000, 2684.000, 1903.000, 1374.000, 373.000, 754.000, 1569.000,
		2628.000, 3016.000, 2721.000, 1964.000, 589.000, 847.000, 888.000,
		2348.000, 2709.000, 3035.000, 2254.000, 1091.000, 494.000, 994.000,
		1324.000, 2744.000, 3144.000, 2932.000, 1892.000, 957.000, 753.000,
		955.000, 2096.000, 3072.000, 2992.000, 2273.000, 1528.000, 502.000,
		936.000, 1309.000, 2194.000, 2786.000, 2544.000, 2166.000, 1187.000,
		800.000, 1192.000, 1762.000, 2996.000, 3296.000, 2313.000, 1438.000,
		895.000, 844.000, 1124.000, 2513.000, 3088.000, 3125.000, 2004.000,
		1398.000, 527.000, 997.000, 1508.000, 2842.000, 3224.000, 2336.000,
		1692.000, 587.000, 804.000, 1372.000, 2344.000, 2818.000, 2742.000,
		1951.000, 1030.000, 590.000, 489.000, 1833.000, 2561.000, 3276.000,
		2981.000, 1958.000, 651.000, 828.000, 1270.000, 1777.000, 3096.000,
		3036.000, 2100.000, 1168.000, 466.000, 864.000, 1623.000, 2173.000,
		2804.000, 3005.000, 1701.000, 801.000, 426.000, 753.000, 1988.000,
		2856.000, 2760.000, 2240.000, 1264.000, 477.000, 418.000, 1397.000,
		2073.000, 2898.000, 2931.000, 2304.000, 1172.000, 440.000, 935.000,
		1747.000, 2829.000, 2897.000, 2500.000, 1676.000, 622.000, 766.000,
		1063.000, 2440.000, 3008.000, 2681.000, 1902.000, 1015.000, 374.000,
		665.000, 1416.000, 2306.000, 3232.000, 2684.000, 1729.000, 1070.000,
		641.000, 977.000, 2248.000, 2582.000, 3202.000, 2272.000, 1061.000,
		707.000, 585.000, 1692.000, 2526.000, 2778.000, 2489.000, 1986.000,
		693.000, 490.000, 757.000, 2050.000, 2657.000, 3156.000, 2701.000,
		1465.000, 763.000, 434.000, 1461.000, 2612.000, 2876.000, 2663.000,
		1791.000, 1264.000, 701.000, 1156.000, 1946.000, 2436.000, 3010.000,
		2294.000, 1329.000, 593.000, 728.000, 1282.000, 2122.000, 2740.000,
		2648.000, 2280.000, 1070.000, 614.000, 1036.000, 1422.000, 2880.000,
		3170.000, 2424.000, 1458.000, 957.000, 700.000, 1243.000, 2296.000,
		3164.000, 2826.000, 2496.000, 1536.000, 659.000, 763.000, 1280.000,
		2292.000, 3101.000, 2998.000, 1741.000, 1105.000, 705.000, 938.000,
		1723.000, 2870.000, 3020.000, 2416.000, 1318.000, 928.000, 427.000,
		1181.000, 2582.000, 2748.000, 2576.000, 2008.000, 1232.000, 578.000,
		816.000, 1793.000, 2857.000, 2890.000, 2388.000, 1826.000, 890.000,
		807.000, 1550.000, 2528.000, 2802.000, 2696.000, 1920.000, 882.000,
		634.000, 736.000, 1739.000, 2811.000, 2906.000, 2629.000, 1537.000,
		944.000, 884.000, 1412.000, 2363.000, 3109.000, 3194.000, 2205.000,
		1238.000, 881.000, 970.000, 1844.000, 2408.000, 3112.000, 2616.000,
		2066.000, 844.000, 432.000, 1344.000, 2034.000, 3076.000, 2856.000,
		2364.000, 1146.000, 517.000, 882.000, 1152.000, 2702.000, 2732.000,
		2538.000, 2029.000, 997.000, 795.000, 971.000, 1724.000, 2873.000,
		2800.000, 2198.000, 1540.000, 464.000, 901.000, 1220.000, 2144.000,
		2707.000, 2774.000, 2326.000, 1263.000, 793.000, 613.000, 1494.000,
		2903.000, 3302.000, 2421.000, 1922.000, 947.000, 728.000, 928.000,
		2180.000, 2921.000, 3136.000, 1941.000, 1024.000, 429.000, 1010.000,
		1885.000, 2420.000, 2994.000, 2620.000, 1590.000, 1149.000, 584.000,
		1329.000, 1807.000, 2818.000, 3106.000, 2090.000, 1120.000, 694.000,
		936.000, 1574.000, 2398.000, 3284.000, 2752.000, 1952.000, 929.000,
		335.000, 923.000, 2194.000, 2742.000, 3152.000, 2538.000, 1287.000,
		916.000, 829.000, 1103.000, 2389.000, 3077.000, 3108.000, 1761.000,
		1060.000, 824.000, 757.000 };
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
	float32_t max_Value;
	uint32_t max_Index;
	float32_t value_Buffer;
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART2_UART_Init();
	MX_ADC1_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */
	printf("START\r\n");
	//HAL_ADC_Start_DMA(&hadc1, (uint32_t*) Data_adc, 256);
	//HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);


	//Fdata
	DMA_states = H_IDLE;
	FFT_(Fdata, &value_Buffer, &max_Index);
	printf("%f, %d\r\n", value_Buffer, (int) (max_Index));
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		switch (Now_states) {
		case H_P_FIND_F:
			if (DMA_states == H_END) {
				DMA_states = H_IDLE;
				FFT_(Data_adc, &value_Buffer, &max_Index);
				printf("%f, %d\r\n", value_Buffer, (int) (max_Index));

				if (value_Buffer > max_Value) { //比較最大的電壓值
					max_Value = value_Buffer;
					max_value_frequency = now_frequency;
				}

				if (now_frequency == 400) {		//掃頻完成，匯入理想頻率
					Now_states = H_P_FIXED_V;
					now_frequency = max_value_frequency;
					SET_EBN(now_frequency, 14);
					continue;
				}

				now_frequency = now_frequency + 5;		//
				SET_EBN(now_frequency, 14);
				while (!HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin)) {
					printf("START\r\n");
					HAL_Delay(100);
				}
				HAL_ADC_Start_DMA(&hadc1, (uint32_t*) Data_adc, 256);
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

			}
			break;
		case H_P_FIXED_V:
			if (DMA_states == H_END) {
				FFT_(Data_adc, &value_Buffer, &max_Index);
				if (value_Buffer > amplitude)
					output++;
				if (value_Buffer < amplitude)
					output--;
				if (output < 20)
					output = 20;
				if (output > 80)
					output = 80;
				SET_EBN(now_frequency, output);		//設定震幅
			}
			break;

		default:
			break;
		}

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
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	RCC_OscInitStruct.PLL.PLLN = 180;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_CC1;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 90;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1000;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void SET_EBN(uint32_t F, uint32_t V) {
	printf("set V: %d F: %d\r\n", (int) F, (int) V);
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	printf("DMA END\r\n");
	DMA_states = H_END;
}
PUTCHAR_PROTOTYPE //for printf used
{
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF);
	return ch;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
