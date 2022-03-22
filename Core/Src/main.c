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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_subghz_phy.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "radio_driver.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// packet type
#define PAYLOAD_LENGTH          (40)

#define FREQ_433_MHZ            (433000000)

#define PA_DUTY_CYCLE           (0x04)
#define HP_MAX                  (0x07)
#define PA_SEL                  (0x00)

#define POWER                   (0x00)
#define RAMP_TIME               (0x06)


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SUBGHZ_HandleTypeDef hsubghz;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t rx_buf[PAYLOAD_LENGTH]={0};
uint8_t p_len = PAYLOAD_LENGTH;
uint8_t temp_tx[2];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void DioIrqHndlr(RadioIrqMasks_t radioIrq);
void Activate_ADC(void);
void ConversionStartPoll_ADC_GrpRegular(void);
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
  MX_SUBGHZ_Init();
  MX_SubGHz_Phy_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */

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
    SUBGRF_SetPayload(rx_buf, PAYLOAD_LENGTH );
    SUBGRF_SetPacketParams(&pkt_params);
    SUBGRF_SetSyncWord( ( uint8_t[] ){ 0xC1, 0x94, 0xC1, 0x00, 0x00, 0x00, 0x00, 0x00 } );
    SUBGRF_SetWhiteningSeed( 0x01FF );
    SUBGRF_SetRfFrequency(FREQ_433_MHZ);
    SUBGRF_SetPaConfig(PA_DUTY_CYCLE, HP_MAX, PA_SEL, 0x01);
    SUBGRF_SetTxParams(RFO_LP, POWER, RAMP_TIME);
    SUBGRF_SetModulationParams(&mod_params);
    SUBGRF_SetDioIrqParams(IRQ_TX_DONE | IRQ_PREAMBLE_DETECTED | IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_SYNCWORD_VALID,
            IRQ_TX_DONE | IRQ_PREAMBLE_DETECTED | IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_SYNCWORD_VALID,
            IRQ_RADIO_NONE,
            IRQ_RADIO_NONE);


    SUBGRF_SetSwitch(0, RFSWITCH_RX); /*Set RF switch*/
     	       //SUBGRF_WriteRegister( REG_RX_GAIN, 0x7F );
     	    SUBGRF_SetRxBoosted(0xFFFFFF);
    //SUBGRF_SetRx(0xFFFFFF);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){

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
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SUBGHZ Initialization Function
  * @param None
  * @retval None
  */
void MX_SUBGHZ_Init(void)
{

  /* USER CODE BEGIN SUBGHZ_Init 0 */

  /* USER CODE END SUBGHZ_Init 0 */

  /* USER CODE BEGIN SUBGHZ_Init 1 */

  /* USER CODE END SUBGHZ_Init 1 */
  hsubghz.Init.BaudratePrescaler = SUBGHZSPI_BAUDRATEPRESCALER_8;
  if (HAL_SUBGHZ_Init(&hsubghz) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SUBGHZ_Init 2 */

  /* USER CODE END SUBGHZ_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, FE_CTRL3_Pin|FE_CTRL2_Pin|FE_CTRL1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : FE_CTRL3_Pin FE_CTRL2_Pin FE_CTRL1_Pin */
  GPIO_InitStruct.Pin = FE_CTRL3_Pin|FE_CTRL2_Pin|FE_CTRL1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void DioIrqHndlr( RadioIrqMasks_t radioIrq )
{
    switch (radioIrq) {
        case IRQ_RX_DONE:
//            for (int i = 0; i < 10; ++i) {
////                HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
//                for (int i = 0; i < 100000; ++i) {}
//            }
            SUBGRF_GetPayload(rx_buf, &p_len, PAYLOAD_LENGTH);
            if (HAL_UART_Transmit(&huart1, rx_buf, p_len, 2000) == HAL_OK){
            	memset(rx_buf, '\0', PAYLOAD_LENGTH);
            }
            break;
        case IRQ_PREAMBLE_DETECTED:
//            for (int i = 0; i < 10; ++i) {
////                HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
//                for (int i = 0; i < 100000; ++i) {}
////                SUBGRF_GetPayload(rx_buf, &p_len, PAYLOAD_LENGTH);
//
//            }
            break;
        case IRQ_SYNCWORD_VALID:
//            for (int i = 0; i < 10; ++i) {
////                HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
//                for (int i = 0; i < 100000; ++i) {}
//            }
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
