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
#include "app_subghz_phy.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "radio_conf.h"
#include "radio_driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PAYLOAD_LENGTH          (40)

#define FREQ_433_MHZ            (433000000)

#define PA_DUTY_CYCLE           (0x04)
#define HP_MAX                  (0x07)
#define PA_SEL                  (0x00)

#define POWER                   (0x0)
#define RAMP_TIME               (0x06)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// temperature sensor LL
#define ADC_CALIBRATION_TIMEOUT_MS       (   1U)
#define ADC_ENABLE_TIMEOUT_MS            (   1U)
#define ADC_DISABLE_TIMEOUT_MS           (   1U)
#define ADC_STOP_CONVERSION_TIMEOUT_MS   (   1U)
#define ADC_CONVERSION_TIMEOUT_MS        (4000U)

/* Delay between ADC end of calibration and ADC enable.                     */
/* Delay estimation in CPU cycles: Case of ADC enable done                  */
/* immediately after ADC calibration, ADC clock setting slow                */
/* (LL_ADC_CLOCK_ASYNC_DIV32). Use a higher delay if ratio                  */
/* (CPU clock / ADC clock) is above 32.                                     */
#define ADC_DELAY_CALIB_ENABLE_CPU_CYCLES  (LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES * 32)

/* Definitions of environment analog values */
/* Value of analog reference voltage (Vref+), connected to analog voltage   */
/* supply Vdda (unit: mV).                                                  */
#define VDDA_APPLI                       (3300UL)

/* Definitions of data related to this example */
/* ADC unitary conversion timeout */
/* Considering ADC settings, duration of 1 ADC conversion should always    */
/* be lower than 1ms.                                                      */
#define ADC_UNITARY_CONVERSION_TIMEOUT_MS (   1UL)

/* Init variable out of expected ADC conversion data range */
#define VAR_CONVERTED_DATA_INIT_VALUE    (__LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B) + 1)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

SUBGHZ_HandleTypeDef hsubghz;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
HAL_StatusTypeDef UART_STATE;
uint8_t data_pkt[2]; //[20] ;
int count = 0;

//timer counter
uint16_t time_uSec = 0;
uint16_t time_mSec = 0;
uint8_t time_Sec = 0;
uint8_t time_Min = 0;
uint8_t time_Hour = 0;

//uint8_t *lora_pkt[4];
uint8_t tx_buf[PAYLOAD_LENGTH];
uint8_t rx_data[PAYLOAD_LENGTH];
uint8_t TEMP_SENS[20];
uint8_t Temp_Count = 0;

uint8_t Temp_Ready = 0;		//Indicats that Temperature data is ready for use
uint8_t TEMP_FLAG = 0;	//FLAG for starting the collection of Temperature data

uint8_t TX_FLAG = 1; 		//FLAG used to control when to Send Data
uint8_t RX_FLAG = 0;			//FLAG used to identify when data is received

/* Variables for ADC conversion data */
__IO uint16_t uhADCxConvertedData = VAR_CONVERTED_DATA_INIT_VALUE; /* ADC group regular conversion data */

/* Variables for ADC conversion data computation to physical values */
__IO int16_t hADCxConvertedData_Temperature_DegreeCelsius = 0UL; /* Value of temperature calculated from ADC conversion data (unit: degree Celsius) */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

void DioIrqHndlr(RadioIrqMasks_t radioIrq);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void Activate_ADC(void);
void ConversionStartPoll_ADC_GrpRegular(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim1) {
		time_mSec++;
		if (time_mSec % 10 == 0 && Temp_Ready == 0) {
			TEMP_FLAG = 1;
		}
		if (time_mSec > 999) {
			time_Sec++;
			time_mSec = 0;
		}
		if (time_Sec > 59) {
			time_Min++;
			time_Sec = 0;
		}
		if (time_Min > 59) {
			time_Hour++;
			time_Min = 0;
		}

	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart1) {
		RX_FLAG = 1;			// Setting RX reception flag
//		if (rx_data[0] == tx_buf[0] && rx_data[1] == tx_buf[1]) {
//			UART_STATE = HAL_UART_Transmit(&huart2, rx_data, PAYLOAD_LENGTH,
//					500);
//			count++;
//		} else {
//			count = 0;
//		}
	}
}
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

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_SUBGHZ_Init();
	MX_USART2_UART_Init();
	MX_SubGHz_Phy_Init();
	MX_ADC_Init();
	MX_USART1_UART_Init();
	MX_TIM1_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim1);

	PacketParams_t pkt_params;
	pkt_params.PacketType = PACKET_TYPE_LORA;
	pkt_params.Params.LoRa.PayloadLength = PAYLOAD_LENGTH;
	pkt_params.Params.LoRa.PreambleLength = 8;
	pkt_params.Params.LoRa.HeaderType = RADIO_PREAMBLE_DETECTOR_08_BITS;
	pkt_params.Params.LoRa.CrcMode = LORA_CRC_ON;
	pkt_params.Params.LoRa.InvertIQ = RADIO_ADDRESSCOMP_FILT_OFF;

	ModulationParams_t mod_params;
	mod_params.PacketType = PACKET_TYPE_LORA;
	mod_params.Params.LoRa.Bandwidth = LORA_BW_125;
	mod_params.Params.LoRa.SpreadingFactor = LORA_SF12;
	mod_params.Params.LoRa.CodingRate = LORA_CR_4_8;
	mod_params.Params.LoRa.LowDatarateOptimize = 1;

	SUBGRF_Init(DioIrqHndlr);
	SUBGRF_SetBufferBaseAddress(0x00, 0x00);
	SUBGRF_SetPayload(tx_buf, PAYLOAD_LENGTH);
	SUBGRF_SetPacketParams(&pkt_params);
	SUBGRF_SetSyncWord(( uint8_t[] ) { 0xC1, 0x94, 0xC1, 0x00, 0x00, 0x00, 0x00,
					0x00 });
	SUBGRF_SetWhiteningSeed(0x01FF);
	SUBGRF_SetRfFrequency(FREQ_433_MHZ);
	SUBGRF_SetPaConfig(PA_DUTY_CYCLE, HP_MAX, PA_SEL, 0x01);
	SUBGRF_SetTxParams(RFO_LP, POWER, RAMP_TIME);
	SUBGRF_SetModulationParams(&mod_params);
	SUBGRF_SetDioIrqParams(
			IRQ_TX_DONE | IRQ_PREAMBLE_DETECTED | IRQ_RX_DONE
					| IRQ_RX_TX_TIMEOUT | IRQ_SYNCWORD_VALID,
			IRQ_TX_DONE | IRQ_PREAMBLE_DETECTED | IRQ_RX_DONE
					| IRQ_RX_TX_TIMEOUT | IRQ_SYNCWORD_VALID, IRQ_RADIO_NONE,
			IRQ_RADIO_NONE);

	SUBGRF_SetSwitch(RFO_LP, RFSWITCH_TX); /*Set RF switch*/
//	SUBGRF_SendPayload(tx_buf, PAYLOAD_LENGTH, 0);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		if (TEMP_FLAG) {

			if (Temp_Count < 20) {

				//initializing temperature sensor channel
				uhADCxConvertedData = 0;
				hADCxConvertedData_Temperature_DegreeCelsius = 0;
				HAL_Delay(3);
				Activate_ADC();
				/* Init variable containing ADC conversion data */
				uhADCxConvertedData = VAR_CONVERTED_DATA_INIT_VALUE;
				/* Perform ADC group regular conversion start, poll for conversion        */
				/* completion.                                                            */
				ConversionStartPoll_ADC_GrpRegular();
				LL_ADC_REG_StopConversion(ADC);
				/* Retrieve ADC conversion data */
				/* (data scale corresponds to ADC resolution: 12 bits) */
				uhADCxConvertedData = LL_ADC_REG_ReadConversionData12(ADC);
				hADCxConvertedData_Temperature_DegreeCelsius =
						__LL_ADC_CALC_TEMPERATURE(VDDA_APPLI,
								uhADCxConvertedData, LL_ADC_RESOLUTION_12B);
				TEMP_SENS[Temp_Count] =
						hADCxConvertedData_Temperature_DegreeCelsius;
				Temp_Count++;
				HAL_Delay(10);
				TEMP_FLAG = 0;
			} else {
				Temp_Ready = 1;
				TEMP_FLAG = 0;
			}
		}

		if (Temp_Ready && TX_FLAG) {
			TX_FLAG = 0;
			for (int i = 0; i < 20; i++) {
				tx_buf[2 * i] = 0xDA;
				tx_buf[2 * i + 1] = TEMP_SENS[i];
			}
			memset(TEMP_SENS, '\0', sizeof(TEMP_SENS));
			SUBGRF_SetSwitch(RFO_LP, RFSWITCH_TX); /*Set RF switch*/
			SUBGRF_SendPayload(tx_buf, PAYLOAD_LENGTH, 0);
			HAL_UART_Receive_IT(&huart1, rx_data, PAYLOAD_LENGTH);
		}

		if (RX_FLAG) {
			for (int i = 0; i < PAYLOAD_LENGTH; i++) {
				if (tx_buf[i] == rx_data[i]) {
					if (tx_buf[i + 1] == rx_data[i + 1]) {

					} else {
						rx_data[i] = 0xAD;
					}
				} else {
					rx_data[i] = 0xAD;
				}
				i++;
			}
			HAL_UART_Transmit(&huart2, rx_data, PAYLOAD_LENGTH, 2000);
			RX_FLAG = 0;
			Temp_Ready = 0;
			Temp_Count = 0;
			TX_FLAG = 1;
			memset(rx_data, '\0', PAYLOAD_LENGTH);
			memset(tx_buf, '\0', PAYLOAD_LENGTH);
		}

		HAL_Delay(10);

		/* USER CODE END WHILE */
		MX_SubGHz_Phy_Process();

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
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS_PWR;
	RCC_OscInitStruct.HSEDiv = RCC_HSE_DIV1;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
	RCC_OscInitStruct.PLL.PLLN = 6;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3 | RCC_CLOCKTYPE_HCLK
			| RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC_Init(void) {

	/* USER CODE BEGIN ADC_Init 0 */

	/* USER CODE END ADC_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC_Init 1 */

	/* USER CODE END ADC_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc.Instance = ADC;
	hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
	hadc.Init.Resolution = ADC_RESOLUTION_12B;
	hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc.Init.LowPowerAutoWait = DISABLE;
	hadc.Init.LowPowerAutoPowerOff = DISABLE;
	hadc.Init.ContinuousConvMode = DISABLE;
	hadc.Init.NbrOfConversion = 1;
	hadc.Init.DiscontinuousConvMode = DISABLE;
	hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc.Init.DMAContinuousRequests = DISABLE;
	hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
	hadc.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
	hadc.Init.OversamplingMode = DISABLE;
	hadc.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
	if (HAL_ADC_Init(&hadc) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC_Init 2 */

	/* USER CODE END ADC_Init 2 */

}

/**
 * @brief SUBGHZ Initialization Function
 * @param None
 * @retval None
 */
void MX_SUBGHZ_Init(void) {

	/* USER CODE BEGIN SUBGHZ_Init 0 */

	/* USER CODE END SUBGHZ_Init 0 */

	/* USER CODE BEGIN SUBGHZ_Init 1 */

	/* USER CODE END SUBGHZ_Init 1 */
	hsubghz.Init.BaudratePrescaler = SUBGHZSPI_BAUDRATEPRESCALER_8;
	if (HAL_SUBGHZ_Init(&hsubghz) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SUBGHZ_Init 2 */

	/* USER CODE END SUBGHZ_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 23;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 999;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

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
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, FE_CTRL3_Pin | FE_CTRL2_Pin | FE_CTRL1_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pins : FE_CTRL3_Pin FE_CTRL2_Pin FE_CTRL1_Pin */
	GPIO_InitStruct.Pin = FE_CTRL3_Pin | FE_CTRL2_Pin | FE_CTRL1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Activate_ADC(void) {
	__IO uint32_t wait_loop_index = 0U;
	__IO uint32_t backup_setting_adc_dma_transfer = 0U;
#if (USE_TIMEOUT == 1)
  uint32_t Timeout = 0U; /* Variable used for timeout management */
  #endif /* USE_TIMEOUT */

	/*## Operation on ADC hierarchical scope: ADC instance #####################*/

	/* Note: Hardware constraint (refer to description of the functions         */
	/*       below):                                                            */
	/*       On this STM32 series, setting of these features is conditioned to  */
	/*       ADC state:                                                         */
	/*       ADC must be disabled.                                              */
	/* Note: In this example, all these checks are not necessary but are        */
	/*       implemented anyway to show the best practice usages                */
	/*       corresponding to reference manual procedure.                       */
	/*       Software can be optimized by removing some of these checks, if     */
	/*       they are not relevant considering previous settings and actions    */
	/*       in user application.                                               */
	if (LL_ADC_IsEnabled(ADC) == 0) {
		/* Enable ADC internal voltage regulator */
		LL_ADC_EnableInternalRegulator(ADC);

		/* Delay for ADC internal voltage regulator stabilization.                */
		/* Compute number of CPU cycles to wait for, from delay in us.            */
		/* Note: Variable divided by 2 to compensate partially                    */
		/*       CPU processing cycles (depends on compilation optimization).     */
		/* Note: If system core clock frequency is below 200kHz, wait time        */
		/*       is only a few CPU processing cycles.                             */
		wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US
				* (SystemCoreClock / (100000 * 2))) / 10);
		while (wait_loop_index != 0) {
			wait_loop_index--;
		}

		/* Disable ADC DMA transfer request during calibration */
		/* Note: Specificity of this STM32 series: Calibration factor is          */
		/*       available in data register and also transferred by DMA.          */
		/*       To not insert ADC calibration factor among ADC conversion data   */
		/*       in DMA destination address, DMA transfer must be disabled during */
		/*       calibration.                                                     */
		backup_setting_adc_dma_transfer = LL_ADC_REG_GetDMATransfer(ADC);
		LL_ADC_REG_SetDMATransfer(ADC, LL_ADC_REG_DMA_TRANSFER_NONE);

		/* Run ADC self calibration */
		LL_ADC_StartCalibration(ADC);

		/* Poll for ADC effectively calibrated */
#if (USE_TIMEOUT == 1)
    Timeout = ADC_CALIBRATION_TIMEOUT_MS;
    #endif /* USE_TIMEOUT */

		while (LL_ADC_IsCalibrationOnGoing(ADC) != 0) {
#if (USE_TIMEOUT == 1)
      /* Check Systick counter flag to decrement the time-out value */
      if (LL_SYSTICK_IsActiveCounterFlag())
      {
        if(Timeout-- == 0)
        {
          /* Error: Time-out */
          Error_Handler();
        }
      }
    #endif /* USE_TIMEOUT */
		}

		/* Restore ADC DMA transfer request after calibration */
		LL_ADC_REG_SetDMATransfer(ADC, backup_setting_adc_dma_transfer);

		/* Delay between ADC end of calibration and ADC enable.                   */
		/* Note: Variable divided by 2 to compensate partially                    */
		/*       CPU processing cycles (depends on compilation optimization).     */
		wait_loop_index = (ADC_DELAY_CALIB_ENABLE_CPU_CYCLES >> 1);
		while (wait_loop_index != 0) {
			wait_loop_index--;
		}

		/* Enable ADC */
		LL_ADC_Enable(ADC);

		/* Poll for ADC ready to convert */
#if (USE_TIMEOUT == 1)
    Timeout = ADC_ENABLE_TIMEOUT_MS;
    #endif /* USE_TIMEOUT */

		while (LL_ADC_IsActiveFlag_ADRDY(ADC) == 0) {
#if (USE_TIMEOUT == 1)
      /* Check Systick counter flag to decrement the time-out value */
      if (LL_SYSTICK_IsActiveCounterFlag())
      {
        if(Timeout-- == 0)
        {
          /* Error: Time-out */
          Error_Handler();
        }
      }
    #endif /* USE_TIMEOUT */
		}

		/* Note: ADC flag ADRDY is not cleared here to be able to check ADC       */
		/*       status afterwards.                                               */
		/*       This flag should be cleared at ADC Deactivation, before a new    */
		/*       ADC activation, using function "LL_ADC_ClearFlag_ADRDY()".       */
	}

}

/**
 * @brief  Perform ADC group regular conversion start, poll for conversion
 *         completion.*/

void ConversionStartPoll_ADC_GrpRegular(void) {
#if (USE_TIMEOUT == 1)
  uint32_t Timeout = 0U; /* Variable used for timeout management */
  #endif /* USE_TIMEOUT */

	/* Start ADC group regular conversion */

	if ((LL_ADC_IsEnabled(ADC) == 1) && (LL_ADC_IsDisableOngoing(ADC) == 0)
			&& (LL_ADC_REG_IsConversionOngoing(ADC) == 0)) {
		LL_ADC_REG_StartConversion(ADC);
	} else {
		/* Error: ADC conversion start could not be performed */
		Error_Handler();
	}

#if (USE_TIMEOUT == 1)
  Timeout = ADC_UNITARY_CONVERSION_TIMEOUT_MS;
  #endif /* USE_TIMEOUT */

	while (LL_ADC_IsActiveFlag_EOC(ADC) == 0) {
#if (USE_TIMEOUT == 1)
    /* Check Systick counter flag to decrement the time-out value */
    if (LL_SYSTICK_IsActiveCounterFlag())
    {
      if(Timeout-- == 0)
      {
        /* Error: Time-out */
        Error_Handler();
      }
    }
  #endif /* USE_TIMEOUT */
	}

	/* Clear flag ADC group regular end of unitary conversion */
	LL_ADC_ClearFlag_EOC(ADC);

}
void DioIrqHndlr(RadioIrqMasks_t radioIrq) {
	switch (radioIrq) {
	case IRQ_RADIO_NONE:

		break;
	case IRQ_TX_DONE:
//		for (int i = 0; i < 10; ++i) {
//			for (int i = 0; i < 100000; ++i) {
//			}
//		}

		break;
	case IRQ_PREAMBLE_DETECTED:

		break;
	case IRQ_SYNCWORD_VALID:

		break;
	case IRQ_HEADER_VALID:

		break;
	case IRQ_HEADER_ERROR:

		break;
	case IRQ_CRC_ERROR:

		break;
	case IRQ_CAD_CLEAR:

		break;
	case IRQ_CAD_DETECTED:

		break;
	case IRQ_RX_TX_TIMEOUT:

		break;
	}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
