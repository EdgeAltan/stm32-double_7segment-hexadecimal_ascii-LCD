/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t first;
uint8_t second;
uint8_t line;

const uint16_t SEGMENT_PINS_1[9] = { GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_4, GPIO_PIN_5,
                                     GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10, GPIO_PIN_12, GPIO_PIN_14};

const uint16_t SEGMENT_PINS_2[9] = { GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_4, GPIO_PIN_5,
                                     GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10, GPIO_PIN_12, GPIO_PIN_15};

#define SEGMENT_PORT GPIOB


const uint16_t LCD_PINS[9] = { GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15};

#define LCD_PORT GPIOE


const uint16_t HEX_TO_SEGMENT[16] = {
	0b111000000, // 0
	0b111111001, // 1
	0b110100100, // 2
	0b110110000, // 3
	0b110011001, // 4
	0b110010010, // 5
	0b110000010, // 6
	0b111111000, // 7
	0b110000000, // 8
	0b110010000, // 9
	0b110001000, // A
	0b110000011, // B
	0b111000110, // C
	0b110100001, // D
	0b110000110, // E
	0b110001110  // F
};

const uint8_t NO_SEGMENT[1] = {
	0b000000000
};

const uint16_t HEX_TO_CHAR[16] = {
	0b0000,  //0
	0b0001,  //1
	0b0010,  //2
	0b0011,  //3
	0b0100,  //4
	0b0101,  //5
	0b0110,  //6
	0b0111,  //7
	0b1000,  //8
	0b1001,  //9
	0b1010,  //A
	0b1011,  //B
	0b1100,  //C
	0b1101,  //D
	0b1110,  //E
	0b1111   //F

};

const uint16_t START_SEQ[9] = {
	0b0010,
	0b0010,
	0b1100,
	0b0000,
	0b0001,
	0b0000,
	0b0010,
	0b0000,
	0b1100
	//0b1100,
	//0b0000
};

const uint16_t BOTTOM_ROW[4] = {
	0b0000,
	0b0000,
	0b1100,
	0b0000
};

const uint16_t OUR_NAMES[] ={


};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x2007c000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x2007c0a0
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x2007c000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x2007c0a0))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void display_digit_1() {

		// Updates each segment by looking at the i'th binary digit to turn on or off the corresponding led of the 7 segment led. we used bitsifting by i to get the correct digit value.
		for (int i = 0; i < 9; i++) {
			HAL_GPIO_WritePin(SEGMENT_PORT, SEGMENT_PINS_1[i], (HEX_TO_SEGMENT[first] >> i) & 0x01);
		}
}
void display_digit_2() {

		// Updates each segment by looking at the i'th binary digit to turn on or off the corresponding led of the 7 segment led. we used bitsifting by i to get the correct digit value.
		for (int i = 0; i < 9; i++) {
			HAL_GPIO_WritePin(SEGMENT_PORT, SEGMENT_PINS_2[i], (HEX_TO_SEGMENT[second] >> i) & 0x01);
		}
}

void close_disp_1(){

	HAL_GPIO_WritePin(SEGMENT_PORT, SEGMENT_PINS_1[8], (NO_SEGMENT[0] >> 8) & 0x01);

}

void close_disp_2(){

	HAL_GPIO_WritePin(SEGMENT_PORT, SEGMENT_PINS_2[8], (NO_SEGMENT[0] >> 8) & 0x01);

}

void start_up(){

	HAL_Delay(10);

	for(int j = 0; j < 9; j++){

		for (int i = 0; i < 4; i++) {
			HAL_GPIO_WritePin(LCD_PORT, LCD_PINS[i], (START_SEQ[j] >> i) & 0x01);
		}

		HAL_Delay(5);

		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);

		HAL_Delay(5);

		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);

		HAL_Delay(5);

	}

	line = 0;

}


void bottom_row(){

	//writing our names to display and setting cursor to the second line
	get_digit_and_write_to_disp(4,5);
	get_digit_and_write_to_disp(6,7);
	get_digit_and_write_to_disp(6,5);
	get_digit_and_write_to_disp(2,0);
	get_digit_and_write_to_disp(2,13);
	get_digit_and_write_to_disp(2,0);
	get_digit_and_write_to_disp(4,5);
	get_digit_and_write_to_disp(6,13);
	get_digit_and_write_to_disp(7,2);
	get_digit_and_write_to_disp(6,5);


	for(int j = 0; j < 2; j++){

		if (line == 0){
			for (int i = 0; i < 4; i++) {
				HAL_GPIO_WritePin(LCD_PORT, LCD_PINS[i], (BOTTOM_ROW[j] >> i) & 0x01);
			}

			line = 1;

		}

		if (line == 1){
			for (int i = 0; i < 4; i++) {
				HAL_GPIO_WritePin(LCD_PORT, LCD_PINS[i], (BOTTOM_ROW[j+2] >> i) & 0x01);
			}

			line = 0;

		}

		HAL_Delay(5);

		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);

		HAL_Delay(5);

		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);

		HAL_Delay(5);

	}

}


void get_digit_and_write_to_disp(uint8_t a, uint8_t b){

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);

	for (int i = 0; i < 4; i++) {
		HAL_GPIO_WritePin(LCD_PORT, LCD_PINS[i], (HEX_TO_CHAR[a] >> i) & 0x01);
	}

	HAL_Delay(5);

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);

	HAL_Delay(5);

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);

	HAL_Delay(5);

	for (int i = 0; i < 4; i++) {
		HAL_GPIO_WritePin(LCD_PORT, LCD_PINS[i], (HEX_TO_CHAR[b] >> i) & 0x01);
	}

	HAL_Delay(5);

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);

	HAL_Delay(5);

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);

	HAL_Delay(5);

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

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
  GPIO_InitTypeDef GPIO_InitStruct_1 = {0};
  GPIO_InitStruct_1.Pin = SEGMENT_PINS_1[0] | SEGMENT_PINS_1[1] | SEGMENT_PINS_1[2] |
		  	  	  	  	  SEGMENT_PINS_1[3] | SEGMENT_PINS_1[4] | SEGMENT_PINS_1[5] |
						  SEGMENT_PINS_1[6] | SEGMENT_PINS_1[7] | SEGMENT_PINS_1[8];
  GPIO_InitStruct_1.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct_1.Pull  = GPIO_NOPULL;
  GPIO_InitStruct_1.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SEGMENT_PORT, &GPIO_InitStruct_1);

  GPIO_InitTypeDef GPIO_InitStruct_2 = {0};
  GPIO_InitStruct_2.Pin = SEGMENT_PINS_2[0] | SEGMENT_PINS_2[1] | SEGMENT_PINS_2[2] |
		   	   	   	   	  SEGMENT_PINS_2[3] | SEGMENT_PINS_2[4] | SEGMENT_PINS_2[5] |
						  SEGMENT_PINS_2[6] | SEGMENT_PINS_2[7] | SEGMENT_PINS_2[8];
  GPIO_InitStruct_2.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct_2.Pull  = GPIO_NOPULL;
  GPIO_InitStruct_2.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SEGMENT_PORT, &GPIO_InitStruct_2);

  GPIO_InitTypeDef GPIO_InitStruct_3 = {0};
  GPIO_InitStruct_3.Pin   = LCD_PINS[0] | LCD_PINS[1] | LCD_PINS[2] | LCD_PINS[3];
  GPIO_InitStruct_3.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct_3.Pull  = GPIO_NOPULL;
  GPIO_InitStruct_3.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_PORT, &GPIO_InitStruct_3);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */

  first = 0;
  second = 0;
  line = 0;

  start_up();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  // Count 1
	  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET) {
		  if(first <= 15){
			  first = (first + 1);
		  }
	  }

	  // Decount 1
	  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_RESET) {
		  if(first >= 1){
			  first = (first - 1);
		  }
	  }

	  // Count 2
	  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_RESET) {
		  if(second <= 15){
			second = (second + 1);
		  }
	  }

	  // Decount 2
	  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_RESET) {
		  if(second >= 1){
			  second = (second - 1);
		  }
	  }

	  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_SET) {
		  get_digit_and_write_to_disp(first, second);
	  }

	  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_SET) {
		  start_up();
	  }


	  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == GPIO_PIN_SET) {
		  bottom_row();
	  }

	  // Display
	  display_digit_1(first);
	  HAL_Delay(25);
	  close_disp_1();
	  display_digit_2(second);
	  HAL_Delay(25);
	  close_disp_2();



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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_4|GPIO_PIN_5|LD2_Pin|GPIO_PIN_8
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin PB1 PB2 PB10
                           PB11 PB12 PB14 PB15
                           PB4 PB5 LD2_Pin PB8
                           PB9 */
  GPIO_InitStruct.Pin = LD1_Pin|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_4|GPIO_PIN_5|LD2_Pin|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PE12 PE13 PE14 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD11 PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
