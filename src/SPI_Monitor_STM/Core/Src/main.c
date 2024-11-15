/* USER CODE BEGIN Header */
/**
 * READ ME
 *
 * In this code, we can select MASTER or SLAVE mode of SPI by define MASTER_BOARD or not
 * using ifdef macro, some functions and initialization will differ
 *
 * GPIO set (at CS PIN)
   GPIO_InitStruct.Pin = SPI2_CS_Pin;
   #ifdef MASTER_BOARD
     GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
     GPIO_InitStruct.Pull = GPIO_PULLUP;
     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   #else
     GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
     GPIO_InitStruct.Pull = GPIO_PULLUP;
     HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
     HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
   #endif
 *
 *
 * SPI - 기본 4bit 기준임 -> 8bit이나 경우 따라서 바꿀 것
 * SPI Clock -> Clock은 50MHz (AFB1) -> Prescaler 4에서 12.5Mbps>7Mbps 달성함
 * 해당 사항들 ioc (MX) 에서 미리 다 설정할 수 있음. 설정 확인하고 코드 제네레이션 하기
 * Pin - Low로 유지 (바꿔도 성능 증가 거의 없이 노이지해짐)
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
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*for slave remove this line, and for master write this line.*/
//#define MASTER_BOARD

#define true 1
#define false 0

//#define BUFFER_SIZE 128
#define BUFFER_SIZE 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi2;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

// buffer declare
typedef struct {
    __IO int read_index;
    __IO int write_index;
    uint8_t buffer[BUFFER_SIZE];
} uint8_queue_t; // FIFO circular queue

char Queue_Is_Empty(uint8_queue_t *cb) {
	return cb->write_index == cb->read_index;
}

char Queue_Is_Full(uint8_queue_t *cb) {
	return ((cb->write_index + 1) % BUFFER_SIZE) == cb->read_index;
}

int Queue_Num_Read(uint8_queue_t *cb) {
	return (BUFFER_SIZE + cb->write_index - cb->read_index) % BUFFER_SIZE;
}

uint8_t* Queue_Pos_Write(uint8_queue_t *cb) {
	return cb->buffer + cb->write_index;
}

uint8_t* Queue_Pos_Read(uint8_queue_t *cb) {
	return cb->buffer + cb->read_index;
}

char Queue_Append(uint8_queue_t *cb, uint8_t data) { // return success of appending
    if (Queue_Is_Full(cb)) {
        return false;
    }
    cb->buffer[cb->write_index] = data;
    cb->write_index = (cb->write_index + 1) % BUFFER_SIZE;
    return true;
}

char Queue_Pop(uint8_queue_t *cb) { // return pop'ed data
    if (Queue_Is_Empty(cb)) {
        return 0;
    }
    char data = cb->buffer[cb->read_index];
    cb->read_index = (cb->read_index + 1) % BUFFER_SIZE;
    return data;
}

uint8_queue_t usart2spi_buf = {0, 0};
uint8_queue_t echo_buf = {0, 0}; // Tx buffers (when send spi, data attached echo)
uint8_queue_t spi2usart_buf = {0,0};  // Rx buffer

volatile uint8_t spi_ready = 0;


/*Print buffer for debugging purpose*/
void Print_Buffer(uint8_queue_t* buf, char* label) {
    static char temp_buf[1024] = {0};  // 버퍼의 내용을 저장할 임시 배열
    int idx = 0;

    // 라벨 출력: "Label: " 형식으로 버퍼 내용을 시작
    idx += sprintf(temp_buf + idx, "[%s(%d, %d): ", label, buf->read_index, buf->write_index);

    // 버퍼 내 데이터를 읽고 임시 배열에 추가
//    volatile int i = buf->read_index;
    for (int i = buf->read_index; i != buf->write_index; i = (i + 1) % BUFFER_SIZE) {
        idx += sprintf(temp_buf + idx, "%02X ", buf->buffer[i]);
    }

    // 줄바꿈 추가하여 가독성을 높임
    sprintf(temp_buf + idx, "]\r\n");

    // UART로 전송 (temp_buf 내용 전체를 시리얼 모니터로 출력)
//    TSART_Tx((uint8_t*)temp_buf, strlen(temp_buf));
	TSART_Tx(temp_buf, strlen(temp_buf));
}


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */

// Serial parts
void TSART_Tx(uint8_t* buffer, uint8_t size)
{
    HAL_UART_Transmit(&huart1, buffer, size, HAL_MAX_DELAY);
}
void TSART_Rx_Start(void)
{
	uint8_queue_t* rx_buf = &usart2spi_buf;
	HAL_UART_Receive_IT(&huart1, Queue_Pos_Write(rx_buf), 1);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_queue_t* rx_buf = &usart2spi_buf;
    if (huart->Instance == USART1)
    {
    	if (!Queue_Is_Full(rx_buf) && *Queue_Pos_Write(rx_buf) != 0)
    	{
    		rx_buf->write_index = (rx_buf->write_index+1) % BUFFER_SIZE; // if full, overwrite last bit
    	}
		HAL_UART_Receive_IT(&huart1, Queue_Pos_Write(rx_buf), 1);
    }
}

// SPI parts

void SPI2_TxRx_Start(void)
{
#ifdef MASTER_BOARD
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
#else
	if (!spi_ready) {return;}
#endif
	uint8_queue_t* rx_buf = &spi2usart_buf;
	uint8_queue_t* tx_buf = &usart2spi_buf;

	if (!Queue_Is_Empty(tx_buf))
	{
		Queue_Append(&echo_buf, *Queue_Pos_Read(tx_buf));
		int prev_read_index = tx_buf->read_index;
		tx_buf->read_index = (tx_buf->read_index+1) % BUFFER_SIZE;

//		HAL_Delay(1);

		HAL_SPI_TransmitReceive_IT(&hspi2, tx_buf->buffer + prev_read_index, Queue_Pos_Write(rx_buf), 1);
	}
	else
	{
		HAL_SPI_TransmitReceive_IT(&hspi2, "\0", Queue_Pos_Write(rx_buf), 1);
	}

}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	uint8_queue_t* rx_buf = &spi2usart_buf;
    if (hspi->Instance == SPI2)
    {
    	if (!Queue_Is_Full(rx_buf) && *Queue_Pos_Write(rx_buf) != 0)
		{
    		rx_buf->write_index = (rx_buf->write_index+1) % BUFFER_SIZE; /*circular buffer*/
		}
#ifdef MASTER_BOARD
    	else
    	{
    		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET); // transmission end
    		return;
    	}
#endif
    	SPI2_TxRx_Start();
    }
}


#ifndef MASTER_BOARD // for slave. automatically SPI is ready when CS low

void EXTI9_5_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(SPI2_CS_Pin);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == SPI2_CS_Pin) {
    	if (HAL_GPIO_ReadPin(SPI2_CS_GPIO_Port, SPI2_CS_Pin) == GPIO_PIN_RESET) // SPI 통신 준비 상
		{
    		spi_ready = 1;
    		SPI2_TxRx_Start();
		}
    	else{
    		spi_ready = 0;
    	}
//    	else if (HAL_GPIO_ReadPin(SPI2_CS_GPIO_Port, SPI2_CS_Pin) == GPIO_PIN_SET)
//    	{
//    		HAL_SPI_Abort_IT(&hspi2);
//    	}
    }
}

#endif

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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  uint8_queue_t* tx_buf = &usart2spi_buf;
  uint8_queue_t* rx_buf = &spi2usart_buf;

  TSART_Rx_Start(); // USB serial on
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//#ifndef MASTER_BOARD
//  uint32_t previous_tick = HAL_GetTick(); // 이전 tick 시간 저장
//#endif
  char pop_buf;
  while (1)
  {
#ifdef MASTER_BOARD
	if (!Queue_Is_Empty(tx_buf)) // && *(Queue_Pos_Write(tx_buf)-1)=='\n')
	{
		SPI2_TxRx_Start();
	}
#endif
	// echo_buf (UART 로 부터 받은 msg 저장) 에 msg가 차있으면,
	if (Queue_Is_Full(&echo_buf) || (!Queue_Is_Empty(&echo_buf) && *(Queue_Pos_Write(&echo_buf)-1)=='\n'))
	{
		TSART_Tx("Condition1\n", strlen("Condition1\n"));
#ifdef MASTER_BOARD /* for debugging purpose*/
		TSART_Tx("M<:", strlen("M<:"));
		Print_Buffer(&echo_buf, "Echo Buffer (Master)");
		Print_Buffer(tx_buf, "TX Buffer (Master)");
		Print_Buffer(rx_buf, "RX Buffer (Master)");
#else
		TSART_Tx("S<:", strlen("S<:"));
		Print_Buffer(&echo_buf, "Echo Buffer (Slave)");
		Print_Buffer(tx_buf, "TX Buffer (Slave)");
		Print_Buffer(rx_buf, "RX Buffer (Slave)");
#endif
		for (int read_num = Queue_Num_Read(&echo_buf); read_num > 0; read_num--)
		{
			pop_buf = Queue_Pop(&echo_buf);
			TSART_Tx(&pop_buf, 1);
		}
	}
//	Print_Buffer(rx_buf, "RX Buffer");
	if (Queue_Is_Full(rx_buf) || (!Queue_Is_Empty(rx_buf) && *(Queue_Pos_Write(rx_buf)-1)=='\n'))
	{
	TSART_Tx("Condition2\n", strlen("Condition2\n"));
#ifdef MASTER_BOARD
		TSART_Tx("M>:", strlen("M>:"));

#else
		TSART_Tx("S>:", strlen("S>:"));

#endif
		for (int read_num = Queue_Num_Read(rx_buf); read_num > 0; read_num--)
		{
			pop_buf = Queue_Pop(rx_buf);
			TSART_Tx(&pop_buf, 1);
		}
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//#ifndef MASTER_BOARD
//
//	        uint32_t current_tick = HAL_GetTick(); // 현재 tick 시간 가져오기
//
//	        // 지정된 주기(OUTPUT_INTERVAL)가 경과했는지 확인
//	        if ((current_tick - previous_tick) >= 1000)
//	        {
//	        	previous_tick=current_tick;
//	        	Print_Buffer(&echo_buf, "Echo Buffer (Slave)");
//	        	Print_Buffer(rx_buf, "RX Buffer (Slave)");
//				Print_Buffer(rx_buf, "RX Buffer (Slave)");
//	        }
//
//#endif
  } // while loop
  /* USER CODE END 3 */
} // main loop

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_SAI2
                              |RCC_PERIPHCLK_SDMMC1|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 5;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_8;
  PeriphClkInitStruct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLSAI;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
  PeriphClkInitStruct.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  #ifdef MASTER_BOARD
	hspi2.Init.Mode = SPI_MODE_MASTER;
  #else
    hspi2.Init.Mode = SPI_MODE_SLAVE;
  #endif
  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/

  hspi2.Instance = SPI2;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, ARDUINO_D7_Pin|ARDUINO_D8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_DISP_GPIO_Port, LCD_DISP_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DCMI_PWR_EN_GPIO_Port, DCMI_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, ARDUINO_D4_Pin|ARDUINO_D2_Pin|EXT_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : OTG_HS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_HS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_HS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_D7_Pin ULPI_D6_Pin ULPI_D5_Pin ULPI_D3_Pin
                           ULPI_D2_Pin ULPI_D1_Pin ULPI_D4_Pin */
  GPIO_InitStruct.Pin = ULPI_D7_Pin|ULPI_D6_Pin|ULPI_D5_Pin|ULPI_D3_Pin
                          |ULPI_D2_Pin|ULPI_D1_Pin|ULPI_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_VBUS_Pin */
  GPIO_InitStruct.Pin = OTG_FS_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Audio_INT_Pin */
  GPIO_InitStruct.Pin = Audio_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Audio_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_D7_Pin ARDUINO_D8_Pin LCD_DISP_Pin */
  GPIO_InitStruct.Pin = ARDUINO_D7_Pin|ARDUINO_D8_Pin|LCD_DISP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : uSD_Detect_Pin */
  GPIO_InitStruct.Pin = uSD_Detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(uSD_Detect_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_BL_CTRL_Pin */
  GPIO_InitStruct.Pin = LCD_BL_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_BL_CTRL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TP3_Pin NC2_Pin */
  GPIO_InitStruct.Pin = TP3_Pin|NC2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : DCMI_PWR_EN_Pin */
  GPIO_InitStruct.Pin = DCMI_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DCMI_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  #ifdef MASTER_BOARD
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  #else
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  #endif
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_INT_Pin */
  GPIO_InitStruct.Pin = LCD_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LCD_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ULPI_NXT_Pin */
  GPIO_InitStruct.Pin = ULPI_NXT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(ULPI_NXT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_D4_Pin ARDUINO_D2_Pin EXT_RST_Pin */
  GPIO_InitStruct.Pin = ARDUINO_D4_Pin|ARDUINO_D2_Pin|EXT_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_STP_Pin ULPI_DIR_Pin */
  GPIO_InitStruct.Pin = ULPI_STP_Pin|ULPI_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_RXER_Pin */
  GPIO_InitStruct.Pin = RMII_RXER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RMII_RXER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_CLK_Pin ULPI_D0_Pin */
  GPIO_InitStruct.Pin = ULPI_CLK_Pin|ULPI_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
