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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MODE 1
#define MAX_LED 8
#define USE_BRIGHTNESS 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim2_ch2_ch4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
//UART
uint8_t Tx_data[58];
uint8_t Rx_data[58];
uint8_t DMX_data[512];
uint8_t our_data[8]; //[r1][g1][b1][fade1][r2][g2][b2][fade2]

//Addressable LED
uint8_t LED_Data[MAX_LED][4];
uint8_t LED_Mod[MAX_LED][4];  // for brightness

//ID
uint8_t DIP_ID;

//Panic button flag
uint8_t panic = 0;

uint8_t datasentflag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void initialise_e32()
{
	  HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(M0_GPIO_Port, M0_Pin, GPIO_PIN_SET);

	  memset(Tx_data, 0xC1, 3);

	  HAL_UART_Transmit(&huart1, Tx_data, 3, 1000);

	  HAL_UART_Receive(&huart1, Rx_data, 6, 1000);

	  HAL_Delay(100);

	  Tx_data[0] = 0xC0;//head
	  Tx_data[1] = 0x00;//addrl
	  Tx_data[2] = 0x00;//addrh
	  Tx_data[3] = 0x3D;
	  Tx_data[4] = 0x17;
	  Tx_data[5] = 0x80;

	  HAL_UART_Transmit(&huart1, Tx_data, 6, 1000);

	  HAL_UART_Receive(&huart1, Rx_data, 6, 1000);

	  HAL_Delay(100);

	  memset(Tx_data, 0xC1, 3);

	  memset(Rx_data, 0, sizeof(Rx_data));

	  HAL_UART_Transmit(&huart1, Tx_data, 3, 1000);

	  HAL_UART_Receive(&huart1, Rx_data, 6, 1000);

	  HAL_Delay(100);

	  HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(M0_GPIO_Port, M0_Pin, GPIO_PIN_RESET);

	  HAL_Delay(100);

	  USART1 -> CR1 &= ~(USART_CR1_UE);
	  USART1 -> BRR = 0x8b;
	  USART1 -> CR1 |= USART_CR1_UE;
}

uint8_t get_id(){

	uint8_t id = 0;

	if (HAL_GPIO_ReadPin(DIP1_GPIO_Port, DIP1_Pin)){
		id |= 0b00000001;
	}

	if (HAL_GPIO_ReadPin(DIP2_GPIO_Port, DIP2_Pin)){
		id |= 0b00000010;
	}

	if (HAL_GPIO_ReadPin(DIP3_GPIO_Port, DIP3_Pin)){
		id |= 0b00000100;
	}

	if (HAL_GPIO_ReadPin(DIP4_GPIO_Port, DIP4_Pin)){
		id |= 0b00001000;
	}

	if (HAL_GPIO_ReadPin(DIP5_GPIO_Port, DIP5_Pin)){
		id |= 0b00010000;
	}

	if (HAL_GPIO_ReadPin(DIP6_GPIO_Port, DIP6_Pin)){
		id |= 0b00100000;
	}

	if (HAL_GPIO_ReadPin(DIP7_GPIO_Port, DIP7_Pin)){
		id |= 0b01000000;
	}

	if (HAL_GPIO_ReadPin(DIP8_GPIO_Port, DIP8_Pin)){
		id |= 0b10000000;
	}

	return id;
}

//https://controllerstech.com/interface-ws2812-with-stm32/
void Set_LED (int LEDnum, int Red, int Green, int Blue)
{
	LED_Data[LEDnum][0] = LEDnum;
	LED_Data[LEDnum][1] = Green;
	LED_Data[LEDnum][2] = Red;
	LED_Data[LEDnum][3] = Blue;
}

#define PI 3.14159265

void Set_Brightness (int brightness)  // 0-45
{
#if USE_BRIGHTNESS

	if (brightness > 45) brightness = 45;
	for (int i=0; i<MAX_LED; i++)
	{
		LED_Mod[i][0] = LED_Data[i][0];
		for (int j=1; j<4; j++)
		{
			float angle = 90-brightness;  // in degrees
			angle = angle*PI / 180;  // in rad
			LED_Mod[i][j] = (LED_Data[i][j])/(tan(angle));
		}
	}

#endif

}

uint16_t pwmData[(24*MAX_LED)+50];

void WS2812_Send (void)
{
	uint32_t indx=0;
	uint32_t color;


	for (int i= 0; i<MAX_LED; i++)
	{
#if USE_BRIGHTNESS
		color = ((LED_Mod[i][1]<<16) | (LED_Mod[i][2]<<8) | (LED_Mod[i][3]));
#else
		color = ((LED_Data[i][1]<<16) | (LED_Data[i][2]<<8) | (LED_Data[i][3]));
#endif

		for (int i=23; i>=0; i--)
		{
			if (color&(1<<i))
			{
				pwmData[indx] = 13;  // 2/3 of 90
			}

			else pwmData[indx] = 6;  // 1/3 of 90

			indx++;
		}

	}

	for (int i=0; i<50; i++)
	{
		pwmData[indx] = 0;
		indx++;
	}

	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_4, (uint32_t *)pwmData, indx);
	while (!datasentflag){};
	datasentflag = 0;
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  DIP_ID = get_id();

  initialise_e32();

  // enable LINE IDLE detect
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *) Rx_data, sizeof(Rx_data));
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

  //Dumb RGB strip mode
  if (MODE == 1){
	  //Start timers
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  } else if(MODE == 2){
	  TIM2 -> ARR = 20;
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  while (panic == 1){
		  TIM2->CCR1 = 255;
		  TIM2->CCR2 = 255;
		  TIM2->CCR3 = 255;
		  TIM3->CCR1 = 255;
		  TIM3->CCR2 = 255;
		  TIM3->CCR3 = 255;
	  }

	  //Test by using RealTerm. Open the COM port of the E32 device (and optionally select Hex in Display As) and go to the Send tab. Using the Send Numbers button it's possible to send integers instead of ASCII characters.

	  //Copy data and put it in our array
	  memcpy(&our_data, &DMX_data[DIP_ID*8], 8);

	  switch (MODE) {
		case 1:
			TIM2->CCR1 = our_data[0];
			TIM2->CCR2 = our_data[1];
		    TIM2->CCR3= our_data[2];
		    TIM3->CCR1 = our_data[4];
		    TIM3->CCR2 = our_data[5];
		    TIM3->CCR3 = our_data[6];
			break;

		case 2:
			for (uint8_t i = 1; i < MAX_LED; ++i)
			  {
				Set_LED(i, our_data[0], our_data[1], our_data[2]);
				Set_Brightness(our_data[4]); //Can use channel 5 as fade for example :)
				WS2812_Send();
			  }
			break;

		default:
			break;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 255;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 255;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, M0_Pin|M1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DIP5_Pin DIP6_Pin DIP7_Pin DIP8_Pin
                           DIP4_Pin DIP3_Pin DIP2_Pin DIP1_Pin */
  GPIO_InitStruct.Pin = DIP5_Pin|DIP6_Pin|DIP7_Pin|DIP8_Pin
                          |DIP4_Pin|DIP3_Pin|DIP2_Pin|DIP1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PANIC_Pin */
  GPIO_InitStruct.Pin = PANIC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PANIC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : M0_Pin M1_Pin */
  GPIO_InitStruct.Pin = M0_Pin|M1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : AUX_Pin */
  GPIO_InitStruct.Pin = AUX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(AUX_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	HAL_UART_Receive_IT(&huart1, Rx_data, 40);
}
*/

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART1)
	{

		// https://controllerstech.com/uart-dma-with-idle-line-detection/


		//check if total data receiving is not exceeding 512 bytes (check for whether data belongs to us)
		if (Rx_data[0] <= Rx_data[1] && Rx_data[1] <= 10){
			//calculate offset
			int offset = (Rx_data[0] - 1) * 56;
			//Copy to DMX_data + offset of the receiving data, copy from Rx_data minus the message info, with the size of 56
			memcpy(&DMX_data[offset], &Rx_data[2], 56);
		}

		// empty the RX array
		memset(Rx_data, 0, sizeof(Rx_data));

		//toggle LED
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

		/* start the DMA again */
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *) Rx_data, sizeof(Rx_data));
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

	}
}


void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_4);
	datasentflag=1;
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	switch(GPIO_Pin){
		case AUX_Pin:
			break;

		case PANIC_Pin:
			panic = !panic;
			break;
		default:
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
