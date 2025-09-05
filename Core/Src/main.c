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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
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

/* USER CODE BEGIN PV */
uint8_t segments[10] =
{
		0b00111111,
		0b00000110,
		0b01011011,
		0b01001111,
		0b01100110,
		0b01101101,
		0b01111101,
		0b00000111,
		0b01111111,
		0b01100111
};

int Dpins[2] = {DIO1_Pin, DIO2_Pin};
int Cpins[2] = {Clk1_Pin, Clk2_Pin};

void delay_us (int time)
{
	for (int i=0; i<time; i++)
	{
		for (int j=0; j<1; j++)
		{
			__asm__("nop");
		}
	}
}

void data(int set, int pin)
{
	HAL_GPIO_WritePin(GPIOB, Dpins[pin], set);
}

void clock(int set, int pin)
{
	HAL_GPIO_WritePin(GPIOB, Cpins[pin], set);
}

void start (int pin)
{
	clock(1, pin);
	data(1, pin);
	delay_us (2);
	data(0, pin);
}

void stop (int pin)
{
	clock(0, pin);
	delay_us (2);
	data(0, pin);
	delay_us (2);
	clock(1, pin);
	delay_us (2);
	data(1, pin);
}

void ack(int pin)
{
	clock(0, pin);
	delay_us(5);
	clock(1, pin);
	delay_us(2);
	clock(0, pin);
}

void sendByte(uint8_t d, int pin)
{
	for(int i = 0; i < 8; i++)
	{
		clock(0, pin);
		data(d%2, pin);
		delay_us(3);
		d = d >> 1;
		clock(1, pin);
		delay_us(3);
	}
	ack(pin);
}

void brightness(int pin)
{
	start(pin);
	sendByte(0x8A, pin);
	stop(pin);
}

void sendData(int index, uint8_t* d, int size, int pin)
{
	start(pin);
	sendByte(0x40, pin);
	stop(pin);

	start(pin);
	sendByte(192 + index, pin);
	for(int i = 0; i < size; i++) sendByte(d[i], pin);
	stop(pin);
	brightness(pin);
}

void sendNumber(int index, int rIndex, int n, int pin, int size, int colon)
{
	if(size == 0) for(int i = n; i > 0; i/=10) size++;

	uint8_t data[size];
	for(int i = size - 1; i >= 0; i--)
	{
		data[i] = segments[n%10];
		n/=10;
		if(colon != 0 && data[i] < 128) data[i] += 128;
	}

	if(rIndex != 0) index -= size - 1;

	sendData(index, data, size, pin);
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
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
  delay_us (2);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  //All code copied mostly from https://controllerstech.com/interface-7-segment-display-with-stm32-tm1637/
  uint8_t num = 0;
  uint8_t data[4] = {0, 0, 0, 0};
  int dir = 1;
  int i = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  sendNumber(2, 0, (i/1000)%60, 0, 2, 0);
	  sendNumber(0, 0, i/60000, 0, 2, 1);
	  i += 150;
	  HAL_Delay(75);
	  sendData(0, data, 4, 1);
	  HAL_Delay(75);
	  data[0] = data[1];
	  data[1] = data[2];
	  data[2] = data[3];
	  data[3] = num;

	  if(dir > 0)
	  {
		  num = num << 1;
		  num++;
		  if(num == 0xFF) dir = -1;
	  }
	  else
	  {
		  num = num << 1;
		  if(num == 0) dir = 1;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Clk2_Pin|DIO2_Pin|Clk1_Pin|DIO1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Clk2_Pin DIO2_Pin Clk1_Pin DIO1_Pin */
  GPIO_InitStruct.Pin = Clk2_Pin|DIO2_Pin|Clk1_Pin|DIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
#ifdef USE_FULL_ASSERT
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
