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
#include "usb_device.h"

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
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

//// TMP102 sensor address, shifted 1 bit to left because slave address is 7 bits (DevAddress)
//static const uint8_t TMP102_addr = 0x48 << 1;
//
//// Temperature register address, 'buffer[0]' is set to this value (*pData)
//static const uint8_t TMP102_TEMP_REG = 0x00;
//
//// Timeout delay
//uint32_t delay = HAL_MAX_DELAY;
//
//// LED port
//GPIO_TypeDef* LED_Port = GPIOD;
//
//// LED Pin
//uint16_t LED_Pin = GPIO_PIN_14;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
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

	// TMP102 sensor address, shifted 1 bit to left because slave address is 7 bits (DevAddress)
	static const uint8_t TMP102_addr = 0x48 << 1;

	// Temperature register address, 'buffer[0]' is set to this value (*pData)
	static const uint8_t TMP102_TEMP_REG = 0x00;

	// Timeout delay
	uint32_t delay = HAL_MAX_DELAY;

	// LED port
	GPIO_TypeDef* LED_Port = GPIOD;

	// LED Pin
	uint16_t LED_Pin = GPIO_PIN_14;

	// Array to store the parameter sent and data received, 2 bytes big (*pData)
	uint8_t buffer[2];

	// Variable to store status of communication
	HAL_StatusTypeDef status;

	// Variable to store raw 12-bit temperature
	int16_t temp_raw;

	// Variable to store converted temperature
	float temperature;

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
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  // Telling TMP102 that we want to read from the temperature register (*pData)
	  buffer[0] = TMP102_TEMP_REG;

	  // Transmit register pointer to the sensor
	  status = HAL_I2C_Master_Transmit(&hi2c1, TMP102_addr, buffer, 1, HAL_MAX_DELAY);

	  // Check status of transmission, if OK keep going, if not OK do nothing
	  if (status == HAL_OK){

		  // Receive 2 bytes of data from TMP102
		  status = HAL_I2C_Master_Receive(&hi2c1, TMP102_addr, buffer, 2, delay);

		  // Check status of receive, if OK keep going, if not OK do nothing
		  if (status == HAL_OK){

			  // Combine the MSB and LSB from the buffer, and isolate the first 12 bits to get the raw temperature value
			  	  /** Casting to int16_t ensures that any negative temperature, where the sign bit (the 12th bit) is set,
			  	   * will be properly treated as a negative number. Without the cast, the value would be treated as unsigned,
			  	   * and you'd have to manually adjust it for negative values (using 2's complement), shown underneath */
			  temp_raw = ((int16_t)buffer[0] << 4) | (buffer[1] >> 4);

			  /***** Example without casting *****/
			  // Combine the MSB and LSB from the buffer, and isolate the first 12 bits to get the raw temperature value
			  // temp_raw = (buffer[0] << 4) | (buffer[1] >> 4);

			  // Check if the temperature is negative (sign bit is set),
			  // if (temp_raw & 0x800) { // 0x800 is the sign bit for a 12-bit number
			  //     // Convert to 2's complement (for negative temperatures)
			  //     temp_raw = temp_raw - 4096;
			  // }

			  // Convert raw value to a temperature in Celsius
			  temperature = temp_raw * 0.0625f;

			  // Format the temperature as a string
			  char temp_str[50];
			  sprintf(temp_str, "Temperature: %.2f°C\r\n", temperature);

			  // Send the temperature data over USB
			  CDC_Transmit_FS((uint8_t*)temp_str, strlen(temp_str));

			  /** Check temperature
			   	   * - if it's over 30 degrees, turn on LED
			   	   * - if it's under 30 degrees, turn off LED */
			  if (temperature >= 27.0f){
				  // LED on
				  HAL_GPIO_WritePin(LED_Port, LED_Pin, GPIO_PIN_SET);
			  } else {
				  // LED off
				  HAL_GPIO_WritePin(LED_Port, LED_Pin, GPIO_PIN_RESET);
			  }

		  } else {

			  char status_str[50];
			  sprintf(status_str, "Receive error: %d\r\n", status);
			  CDC_Transmit_FS((uint8_t*)status_str, strlen(status_str));
		  }

	  } else {

		  char status_str[50];
		  sprintf(status_str, "Transmit error: %d\r\n", status);
		  CDC_Transmit_FS((uint8_t*)status_str, strlen(status_str));
	  }

	  // Delay
	  HAL_Delay(100);
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
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
