/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/empty.h>

#include"FreeRTOS.h"
#include"task.h"
#include "semphr.h"
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
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__          // GCC / arm-none-eabi-gcc
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
#else
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
#endif

volatile bool lcd_need_refresh = false;
BaseType_t xReturned;
TaskHandle_t xcoinHandle=NULL;
TaskHandle_t xpulseHandle=NULL;
UART_HandleTypeDef huart2;

#define LCD_ADDRESS (0x27 << 1)

int i=0;
int impulse=0;
int total=0;
int day=1;
int now=0;
char MonitorTset[100];
SemaphoreHandle_t i2cMutex;

void lcd_send_cmd(char cmd)
{
//	xSemaphoreTake(i2cMutex, portMAX_DELAY);

	char data_h,data_l;
	uint8_t frame_data[4];
	data_h = (cmd&0xf0);
	data_l = ((cmd <<4)&0xf0);
	frame_data[0] = data_h | 0x0C;
	frame_data[1] = data_h | 0x08;
	frame_data[2] = data_l | 0x0C;
	frame_data[3] = data_l | 0x08;

	HAL_I2C_Master_Transmit(&hi2c1,LCD_ADDRESS,(uint8_t *)frame_data,4,0x100);

	//HAL_Delay(1);
//	xSemaphoreGive(i2cMutex);
}

void lcd_send_data(char data)
{
	char data_h,data_l;
	uint8_t frame_data[4];
	data_h = (data&0xf0);
	data_l = ((data <<4)&0xf0);
	frame_data[0] = data_h | 0x0D;
	frame_data[1] = data_h | 0x09;
	frame_data[2] = data_l | 0x0D;
	frame_data[3] = data_l | 0x09;

	HAL_I2C_Master_Transmit(&hi2c1,LCD_ADDRESS,(uint8_t *)frame_data,4,0x100);

	//HAL_Delay(1);
}

void lcd_clear()
{
	lcd_send_cmd(0x01); // clear display
	HAL_Delay(1);
}


void lcd_Init()
{
	HAL_Delay(50); // 8-bit initialization start
	lcd_send_cmd(0x30);
	HAL_Delay(5);
	lcd_send_cmd(0x30);
	HAL_Delay(1);
	lcd_send_cmd(0x30);
	HAL_Delay(10);
	lcd_send_cmd(0x20);
	HAL_Delay(10); // 8-bit initialization finish

	lcd_send_cmd(0x28);		//function set
	HAL_Delay(1);
	lcd_send_cmd(0x08);		//Display on/off
	HAL_Delay(1);
	lcd_send_cmd(0x01);		//clear display
	HAL_Delay(1);
	lcd_send_cmd(0x06);		//Enter mode set
	HAL_Delay(1);
	lcd_send_cmd(0x0C);		//Display on/off
	HAL_Delay(1);

}

void lcd_send_string (char *str)
{
	while(*str)
	{
		lcd_send_data(*str++);
	}
	HAL_Delay(1);
}

void lcd_put_cur(uint8_t row,uint8_t col) // the location on the lcd screen
{
	lcd_send_cmd(0x80 | (col + (0x40 * row)));
}

void I2C_Scan(I2C_HandleTypeDef *hi2c) {
    char msg[64];
    for (uint8_t addr = 1; addr < 127; addr++) {
        if (HAL_I2C_IsDeviceReady(hi2c, addr << 1, 1, 10) == HAL_OK) {
            sprintf(msg, "Found device at 0x%02X\r\n", addr << 1);
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xffff);
        }
    }
}

void incomingImpulse() {
	for (;;) {
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 0) {
			while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 0){}
			impulse = impulse + 1;
			i = 0;
		}
	}
}

void COIN_Task() {
  // put your main code here, to run repeatedly:

	memset(MonitorTset,'\0',sizeof(MonitorTset));
	I2C_Scan(&hi2c1);
	for (;;) {

	 if (lcd_need_refresh) {
			lcd_need_refresh = false;
			lcd_clear();
			lcd_put_cur(0, 0);
			lcd_send_string("Money reset!");
			lcd_put_cur(1, 0);
			lcd_send_string("Total = 0");
		}

	  i = i + 1;

	  if (i >= 5 && impulse == 1){ // 10 dollar
		total = total + 10;
		now = now + 10;

		while (day <= now){
		  now = now - day;
		  day = day + 1;
		}

		// uart
		sprintf(MonitorTset,"The day is %d. Today already saved %d. The total value is %d.\n\r",day, now, total);
//		HAL_UART_Transmit(&huart2,(uint8_t *)MonitorTset,strlen(MonitorTset),0xffff);

		lcd_clear();
		lcd_put_cur(0,0);
		lcd_send_string("Insert 10 dollar.");
		lcd_put_cur(1,0);
		sprintf(MonitorTset,"Total value is %d", total);
		lcd_send_string(MonitorTset);

		impulse = 0;
	  }
	  else if (i >= 5 && impulse == 2){ // 5 dollar
		total = total + 5;
		now = now + 5;

		while(day <= now){
		  now = now - day;
		  day = day + 1;
		}

		sprintf(MonitorTset,"The day is %d. Today already saved %d. The total value is %d.\n\r",day, now, total);
//		HAL_UART_Transmit(&huart2,(uint8_t *)MonitorTset,strlen(MonitorTset),0xffff);

		lcd_clear();
		lcd_put_cur(0,0);
		lcd_send_string("Insert 5 dollar.");
		lcd_put_cur(1,0);
		sprintf(MonitorTset,"Total value is %d", total);
		lcd_send_string(MonitorTset);

		impulse = 0;
	  }
	  else if (i >= 5 && impulse == 3){ // 1 dollar
		total = total + 1;
		now = now + 1;

		while(day <= now){
		  now = now - day;
		  day = day + 1;
		}

		sprintf(MonitorTset,"The day is %d. Today already saved %d. The total value is %d.\n\r",day, now, total);
//		HAL_UART_Transmit(&huart2,(uint8_t *)MonitorTset,strlen(MonitorTset),0xffff);

		lcd_clear();
		lcd_put_cur(0,0);
		lcd_send_string("Insert 1 dollar.");
		lcd_put_cur(1,0);
		sprintf(MonitorTset,"Total value is %d", total);
		lcd_send_string(MonitorTset);

		impulse=0;
	  }

	  if (day > 365){
		day = 1;
	  }

	  vTaskDelay(100);
	}
}

//void USART_Test(void *pvParameters){
//	uint32_t Monitortimer = 400;
//	char MonitorTset[30];
//	char num[15];
//	int i = 0;
//	while(1){
//		memset(MonitorTset,'\0',sizeof(MonitorTset));
//		memset(num,'\0',sizeof(num));
//		itoa(i,num,10);
//		strcat(num," ");
//		sprintf(MonitorTset,"The point is %s\n\r",num);
//		HAL_UART_Transmit(&huart2,(uint8_t *)MonitorTset,strlen(MonitorTset),0xffff);
//		vTaskDelay(Monitortimer);
//		Monitortimer += 1;
//		i += 1;
//	}
//}

void reset_callback(const void * msgin)
{
  (void) msgin;
  total = 0;
  now   = 0;
  day   = 1;
//  i = 0;
  lcd_need_refresh = true;
  printf("Received /money_reset, total reset to 0.\n");
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
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
//  for (int i = 0; i < 10; i++) {
//  	    char msg[32];
//  	    sprintf(msg, "Count: %d\r\n", i);
//  	    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//  	  HAL_Delay(1000);
//  	}

  HAL_Delay(30);
  lcd_Init();

  const osThreadAttr_t coinTask_attributes = {
    .name = "COIN_Task",
    .priority = osPriorityNormal,
    .stack_size = 128 * 4
  };

  const osThreadAttr_t impulseTask_attributes = {
    .name = "Impulse",
    .priority = osPriorityNormal,
    .stack_size = 128 * 4
  };

//  vTaskStartScheduler();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  osThreadNew(COIN_Task, NULL, &coinTask_attributes);
  osThreadNew(incomingImpulse, NULL, &impulseTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  sprintf(MonitorTset,"The day is %d. Today already saved %d. The total value is %d.\n\r",day, now, total);
//  HAL_UART_Transmit(&huart2,(uint8_t *)MonitorTset,strlen(MonitorTset),0xffff);
  while (1)
  {
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart2.Init.BaudRate = 115200;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */


void StartDefaultTask(void *argument)
{
    /*---------------- 0. 讓 USB Host 與系統先穩定 ----------------*/
    MX_USB_HOST_Init();      // 若無用到可註解
    HAL_Delay(3000);

    /*---------------- 1. 設定 UART custom transport --------------*/
    rmw_uros_set_custom_transport(
        true,
        (void*)&huart2,
        cubemx_transport_open,
        cubemx_transport_close,
        cubemx_transport_write,
        cubemx_transport_read);

    /*---------------- 2. (可選) 換成 FreeRTOS 記憶體配置 ----------*/
    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate      = microros_allocate;
    freeRTOS_allocator.deallocate    = microros_deallocate;
    freeRTOS_allocator.reallocate    = microros_reallocate;
    freeRTOS_allocator.zero_allocate = microros_zero_allocate;
    rcutils_set_default_allocator(&freeRTOS_allocator);

    /*---------------- 3. 建立 support (內含 DDS session) ----------*/
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t  support;
    rclc_support_init(&support, 0, NULL, &allocator);

    /*---------------- 4. 建立節點 / 發布者 / 訂閱者 ---------------*/
    rcl_node_t node;
    rclc_node_init_default(&node, "cubemx_node", "", &support);

    /* publisher : /money_count */
    rcl_publisher_t publisher;
    rclc_publisher_init_default(
        &publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "money_count");

    /* subscriber : /money_reset */
    rcl_subscription_t subscriber;
    std_msgs__msg__Empty reset_msg;   // 資料緩衝區
    std_msgs__msg__Empty__init(&reset_msg);
    rclc_subscription_init_default(
        &subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
        "money_reset");

    /*---------------- 5. 建立 executor 並加入訂閱 -----------------*/
    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(
        &executor, &subscriber,
        &reset_msg, &reset_callback, ON_NEW_DATA);

    /*---------------- 6. 主迴圈：送累積金額 + spin ----------------*/
    std_msgs__msg__Int32 msg;
    for (;;)
    {
        msg.data = total;                        // 發布目前總額
        rcl_publish(&publisher, &msg, NULL);

        rclc_executor_spin_some(&executor,
                                RCL_MS_TO_NS(5)); // 處理 /money_reset

        osDelay(10);                             // 10 ms 週期
    }
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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

