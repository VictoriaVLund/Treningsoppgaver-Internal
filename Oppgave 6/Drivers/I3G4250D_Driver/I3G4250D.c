/*
 * I3G4250D.c
 *
 *  Created on: Sep 19, 2024
 *      Author: vvl
 */

/**
  ******************************************************************************
  * Source file for a simple self-created driver for the I3G4250D gyroscope
  * based on instructions from Oppgave 6 under Internal Competency Training
  ******************************************************************************
  * It includes:
  * 	-
  *
  ******************************************************************************
  * The
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "I3G4250D.h"

/* Global Static Variable -----------------------------------------------------*/
extern SPI_HandleTypeDef hspi1;

/* Macros ---------------------------------------------------------------------*/
/**
  * @brief  Enables the Chip Select (CS) line for the I3G4250D gyroscope.
  *         This macro sets the specified GPIO pin low to select the gyroscope
  *         for SPI communication.
  * @note   Adjust the GPIO port and pin values to match the actual hardware
  * 		configuration.
  * @retval None.
  */
#define I3G4250D_CS_ENABLE		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

/**
  * @brief  Disables the Chip Select (CS) line for the I3G4250D gyroscope.
  *         This macro sets the specified GPIO pin high to deselect the gyroscope
  *         and end SPI communication.
  * @note   Adjust the GPIO port and pin values to match the actual hardware
  * 		configuration.
  * @retval None.
  */
#define I3G4250D_CS_DISABLE		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

/**
  *
@verbatim
 ===============================================================================
                      ##### Initialization function #####
 ===============================================================================
  [..]
    This section provides a function allowing to initialize the I3G4250D
    gyroscope to be ready for use.

@endverbatim
  *
  */


/**
  * @brief  Initializes the I3G4250D gyroscope with the given configuration.
  *         This function configures the control register 1 (CTRL_REG1) of the
  *         I3G4250D gyroscope using the values from the provided configuration.
  * @param  config: Pointer to a structure of type I3G4250D_InitTypeDef that
  *                 contains the configuration settings for the data rate,
  *                 power mode, and axis enablement.
  *                 - dataRate: Specifies the output data rate (ODR).
  *                 - powerDown: Specifies the power mode (active or power down).
  *                 - axesEnable: Specifies which axes (X, Y, Z) are enabled.
  * @retval None
  */
void I3G4250D_CTRL_REG1_Init(I3G4250D_InitTypeDef *config)
{
	uint8_t ctrlReg1 = 0;

	ctrlReg1 |= config->dataRate;
	ctrlReg1 |= config->powerDown;
	ctrlReg1 |= config->axesEnable;

	I3G4250D_WriteReg(I3G4250D_CTRL_REG1, ctrlReg1);
}

/**
  *
@verbatim
 ===============================================================================
                       ##### IO operation functions #####
 ===============================================================================

@endverbatim
  *
  */

/**
  * @brief  Reads a register from the I3G4250D gyroscope via SPI.
  *         This function sends the register address (with read bit set) over SPI
  *         and receives the value stored in the specified register.
  * @param  reg_addr: Address of the register to be read.
  *                   The most significant bit (MSB) will automatically be set to 1
  *                   to indicate a read operation.
  * @retval The value of the register located at the specified address.
  */
uint8_t I3G4250D_ReadReg(uint8_t reg_addr)
{
	uint8_t rx_data = 0;
	uint8_t tx_data = reg_addr | 0x80;

	I3G4250D_CS_ENABLE;
	HAL_SPI_Transmit(&hspi1, &tx_data, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi1, &rx_data, 1, HAL_MAX_DELAY);
	I3G4250D_CS_DISABLE;

	return rx_data;

}

/**
  * @brief  Writes a value to a register of the I3G4250D gyroscope via SPI.
  *         This function sends the register address and the data to be written
  *         over SPI to the I3G4250D.
  * @param  reg_addr: Address of the register to which the data will be written.
  *                   The most significant bit (MSB) is cleared for a write operation.
  * @param  reg_val: The value to be written to the specified register.
  * @retval None
  */
void I3G4250D_WriteReg(uint8_t reg_addr, uint8_t reg_val)
{
	I3G4250D_CS_ENABLE;
	HAL_SPI_Transmit(&hspi1, &reg_addr, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, &reg_val, 1, HAL_MAX_DELAY);
	I3G4250D_CS_DISABLE;
}

