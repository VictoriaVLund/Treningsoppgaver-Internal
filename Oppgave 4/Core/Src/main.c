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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
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

  // Del 1 - Test SPI communication

//	// WHO_AM_I address + Read bit set
//	const uint8_t r_WHO_AM_I = 0x0F| 0x80;
//	// Expected WHO_AM_I value
//	uint8_t WHO_AM_I_value = 0xD3;
//	// Received data from SPI communication
//	uint8_t received_Data;
//	// LED port
//	GPIO_TypeDef* LED_Port = GPIOD;
//	// Red LED pin
//	uint16_t rLED_Pin = GPIO_PIN_14;
//	// Green LED pin
//	uint16_t gLED_Pin = GPIO_PIN_12;
//	// CS port
//	GPIO_TypeDef* CS_Port = GPIOE;
//	// CS pin
//	uint16_t CS_Pin = GPIO_PIN_3;


  // Del 2 - Light up LEDs based on the direction the board is tipped

	/* CTRL Register variables */
	uint8_t CTRL_REG1_addr = 0x20; // CTRL_REG1 address
	uint8_t CTRL_REG1_val = 0x0F; // Value to write to CTRL_REG1 to configure gyroscope: set ODR to lowest and enable X, Y and Z-axes

	/* SPI-CS variables */
	GPIO_TypeDef* CS_Port = GPIOE; // CS port
	uint16_t CS_Pin = GPIO_PIN_3; // CS pin

	/* LED variables */
	GPIO_TypeDef* LED_Port = GPIOD; // LED port
	uint16_t gLED_Pin = GPIO_PIN_12; // Green LED pin
	uint16_t rLED_Pin = GPIO_PIN_14; // Red LED pin
	uint16_t bLED_Pin = GPIO_PIN_15; // Blue LED pin
	uint16_t oLED_Pin = GPIO_PIN_13; // Orange LED pin

	/* SPI delay */
	uint32_t SPI_Delay = HAL_MAX_DELAY;

	/* Threshold variable */
	uint16_t THRESHOLD = 5000;

/*Read data method 1: Messy*/
	/* X-axis variables */
	uint8_t rOUT_X_L_addr = 0xA8; // Address for the lower 8 bits of the X-axis data + read operation
	uint8_t rOUT_X_H_addr = 0xA9; // Address for the upper 8 bits of the X-axis data + read operation
	uint8_t x_lsb; // Variable to read the low byte into
	uint8_t x_msb; // Variable to read the high byte into
	int16_t x_val; // Variable to combine high and low bytes into 16-bit value

	/* Y-axis variables */
	uint8_t rOUT_Y_L_addr = 0xAA; // Address for the lower 8 bits of the Y-axis data + read operation
	uint8_t rOUT_Y_H_addr = 0xAB; // Address for the upper 8 bits of the Y-axis data + read operation
	uint8_t y_lsb; // Variable to read the low byte into
	uint8_t y_msb; // Variable to read the high byte into
	int16_t y_val; // Variable to combine high and low bytes into 16-bit value

	//uint8_t xy_reg_val[4] = {0};

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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

// Del 1 - Test SPI communication

//  // Set CS low
//  HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_RESET);
//
//  // Transmit and Receive data from the WHO_AM_I register
//  HAL_SPI_TransmitReceive(&hspi1, &r_WHO_AM_I, &received_Data, 1, SPI_Delay);
//
//  // Set CS high
//  HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_SET);
//
//	  // Not task relevant, just testing USB
//		  // Convert received data to string for USB transmission
//		  //char buffer[50];
//		  //int length = sprintf(buffer, "WHO_AM_I value: 0x%02X\r\n", received_Data);
// 	  	  //CDC_Transmit_FS((uint8_t*)buffer, length);
//
//  // Check if received data is equal to the expected value from the WHO_AM_I register
//  	  // If they are equal, turn on Green LED
//  	  // If they are not equal, turn on the Red LED
//  if (received_Data == WHO_AM_I_value){
//
//	  // Turn on Green LED
//	  HAL_GPIO_WritePin(LED_Port, gLED_Pin, GPIO_PIN_SET);
//
//  } else {
//
//	  // Turn on Red LED
//	  HAL_GPIO_WritePin(LED_Port, rLED_Pin, GPIO_PIN_SET);
//  }

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

// DEL 2 - Light up LEDs based on the direction the board is tipped

  /* Configure built-in gyroscope */

  HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_RESET); // Set CS low
  HAL_SPI_Transmit(&hspi1, &CTRL_REG1_addr, 1, SPI_Delay); // Send CTRL_REG1 address 0x20
  HAL_SPI_Transmit(&hspi1, &CTRL_REG1_val, 1, SPI_Delay); // Send configure value 0x0F
  HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_SET); // Set CS high

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  /* Not task relevant, just testing USB */
	  // Send data over USB CDC
	  //CDC_Transmit_FS((uint8_t*)buffer, length);

// DEL 2 - Light up LEDs based on the direction the board is tipped

   /* Read data method 1: Messy */

	  // Set CS low
	  HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_RESET);

	  // Send read OUT_X_L
	  HAL_SPI_Transmit(&hspi1, &rOUT_X_L_addr, 1, SPI_Delay);
	  // Receive OUT_X_L data
	  HAL_SPI_Receive(&hspi1, &x_lsb, 1, SPI_Delay);

	  // Set CS high
	  HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_SET);

	  // For OUT_X_H
	  HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi1, &rOUT_X_H_addr, 1, SPI_Delay);
	  HAL_SPI_Receive(&hspi1, &x_msb, 1, SPI_Delay);
	  HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_SET);

	  // For OUT_Y_L
	  HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi1, &rOUT_Y_L_addr, 1, SPI_Delay);
	  HAL_SPI_Receive(&hspi1, &y_lsb, 1, SPI_Delay);
	  HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_SET);

	  // For OUT_Y_H
	  HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi1, &rOUT_Y_H_addr, 1, SPI_Delay);
	  HAL_SPI_Receive(&hspi1, &y_msb, 1, SPI_Delay);
	  HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_SET);

	  // Combine x_msb and x_lsb
	  //x_val = ((uint16_t) xy_reg_val[1]) << 8 | ((uint16_t) xy_reg_val[0]);
	  x_val = ((uint16_t) x_msb << 8) | ((uint16_t) x_lsb);

	  // Combine y_msb and y_lsb
	  //y_val = ((uint16_t) xy_reg_val[3]) << 8 | ((uint16_t) xy_reg_val[2]);
	  y_val = ((uint16_t) y_msb << 8) | ((uint16_t) y_lsb);

//	  char tx_string[25];
//	  sprintf(tx_string, "X: %d, Y: %d\r\n", x_val, y_val);
//	  CDC_Transmit_FS((uint8_t*)tx_string, strlen(tx_string));

   /* Update LEDs based on X- and Y-axis data */

	  // Update LEDs based on X-axis data
	  if (x_val > THRESHOLD){
		  HAL_GPIO_WritePin(LED_Port, gLED_Pin, GPIO_PIN_SET); // Turn on green LED if X is negative
		  HAL_GPIO_WritePin(LED_Port, rLED_Pin, GPIO_PIN_RESET); // Turn off red LED if X is negative
		  HAL_GPIO_WritePin(LED_Port, bLED_Pin, GPIO_PIN_RESET); // Turn off blue LED if X is negative
		  HAL_GPIO_WritePin(LED_Port, oLED_Pin, GPIO_PIN_RESET); // Turn off orange LED if X is negative
	  }

	  if (x_val < -THRESHOLD){
		  HAL_GPIO_WritePin(LED_Port, gLED_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LED_Port, rLED_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(LED_Port, bLED_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LED_Port, oLED_Pin, GPIO_PIN_RESET);
	  }

	  if (y_val > THRESHOLD){
		  HAL_GPIO_WritePin(LED_Port, gLED_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LED_Port, rLED_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LED_Port, bLED_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(LED_Port, oLED_Pin, GPIO_PIN_RESET);
	  }

	  if (y_val < -THRESHOLD){
		  HAL_GPIO_WritePin(LED_Port, gLED_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LED_Port, rLED_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LED_Port, bLED_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LED_Port, oLED_Pin, GPIO_PIN_SET);
	  }


	  HAL_Delay(100); // Small delay for stability

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
