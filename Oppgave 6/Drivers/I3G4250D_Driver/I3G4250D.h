/*
 * I3G4250D.h
 *
 *  Created on: Sep 19, 2024
 *      Author: vvl
 */

/**
  ******************************************************************************
  * Simple self-created driver for the I3G4250D gyroscope based on instructions
  * from Oppgave 6 under Internal Competency Training
  ******************************************************************************
  * It includes:
  * 	- a list of the gyroscope registers and their addresses
  * 	- register values for the different data rate options
  * 	- register values for XYZ axes activation
  *
  ******************************************************************************
  * The I3G4250D Datasheet is the source of registers and register values
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef I3G4250D_DRIVER_I3G4250D_H_
#define I3G4250D_DRIVER_I3G4250D_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/**
  * @brief I3G4250D gyroscope Init structure definition
  */
typedef struct
{
	uint8_t dataRate;		/*!< Specifies the Output Data Rate selection.
                           	   	 This parameter can be any value of @ref I3G4250D_ODR_define */

	uint8_t powerDown;		/*!< Specifies the power status of the gyroscope.
                           	   	 This parameter can be any value of @ref I3G4250D_powerDown_define */

	uint8_t axesEnable;		/*!< Specifies the axes' enabled/disabled.
                           	   	 This parameter can be any value of @ref I3G4250D_axes_define */

} I3G4250D_InitTypeDef;

/* Registers (p. 28) ---------------------------------------------------------*/

/** @defgroup I3G4250D_address_define I3G4250D address define
  * @{
  */
/* Relevant to task */
#define I3G4250D_WHO_AM_I					0x0F
#define I3G4250D_CTRL_REG1					0x20
#define I3G4250D_OUT_X_L					0x28
#define I3G4250D_OUT_X_H					0x29
#define I3G4250D_OUT_Y_L					0x2A
#define I3G4250D_OUT_Y_H					0x2B
#define I3G4250D_OUT_Z_L					0x2C
#define I3G4250D_OUT_Z_H					0x2D

/* Not relevant to task */
#define I3G4250D_CTRL_REG2					0x21
#define I3G4250D_CTRL_REG3					0x22
#define I3G4250D_CTRL_REG4					0x23
#define I3G4250D_CTRL_REG5					0x24
#define I3G4250D_REFERENCE_DATACAPTURE		0x25
#define I3G4250D_OUT_TEMP					0x26
#define I3G4250D_STATUS_REG					0x27
#define I3G4250D_FIFO_CTRL_REG				0x2E
#define I3G4250D_FIFO_SRC_REG				0x2F
#define I3G4250D_INT1_CFG					0x30
#define I3G4250D_INT1_SRC					0x31
#define I3G4250D_INT1_THS_XH				0x32
#define I3G4250D_INT1_THS_XL				0x33
#define I3G4250D_INT1_THS_YH				0x34
#define I3G4250D_INT1_THS_YL				0x35
#define I3G4250D_INT1_THS_ZH				0x36
#define I3G4250D_INT1_THS_ZL				0x37
#define I3G4250D_INT1_DURATION				0x38
/**
  * @}
  */

/* CTRL_REG1 register values (p. 30-31) --------------------------------------*/

/** @defgroup I3G4250D_ODR_define I3G4250D ODR define
  * @{
  */
#define I3G4250D_ODR_100HZ_FC_12_5			0x00
#define I3G4250D_ODR_100HZ_FC_25			0x10
#define I3G4250D_ODR_100HZ_FC_25			0x20
#define I3G4250D_ODR_100HZ_FC_25			0x30
#define I3G4250D_ODR_200HZ_FC_12_5			0x40
#define I3G4250D_ODR_200HZ_FC_25			0x50
#define I3G4250D_ODR_200HZ_FC_50			0x60
#define I3G4250D_ODR_200HZ_FC_70			0x70
#define I3G4250D_ODR_400HZ_FC_20			0x80
#define I3G4250D_ODR_400HZ_FC_25			0x90
#define I3G4250D_ODR_400HZ_FC_50			0xA0
#define I3G4250D_ODR_400HZ_FC_110			0xB0
#define I3G4250D_ODR_800HZ_FC_30			0xC0
#define I3G4250D_ODR_800HZ_FC_35			0xD0
#define I3G4250D_ODR_800HZ_FC_50			0xE0
#define I3G4250D_ODR_800HZ_FC_110			0xF0
/**
  * @}
  */

/** @defgroup I3G4250D_powerDown_define I3G4250D powerDown define
  * @{
  */
#define I3G4250D_PD_ENABLE					0x00 // Power-down
#define I3G4250D_PD_DISABLE					0x08 // Normal/sleep mode depending on value of X, Y, and Z
/**
  * @}
  */

/** @defgroup I3G4250D_axes_define I3G4250D axes define
  * @{
  */
#define I3G4250D_ENABLE_X					0x01
#define I3G4250D_ENABLE_Y					0x02
#define I3G4250D_ENABLE_Z					0x03
#define I3G4250D_DISABLE_X					0x00
#define I3G4250D_DISABLE_Y					0x00
#define I3G4250D_DISABLE_Z					0x00

#define I3G4250D_ENABLE_XYZ					(I3G4250D_ENABLE_X | I3G4250D_ENABLE_Y | I3G4250D_ENABLE_Z)
#define I3G4250D_DISABLE_XYZ				(I3G4250D_DISABLE_X | I3G4250D_DISABLE_Y | I3G4250D_DISABLE_Z)
/**
  * @}
  */

/* Initialization function ---------------------------------------------------*/
void I3G4250D_Init(I3G4250D_InitTypeDef *config);

/* IO operation functions ----------------------------------------------------*/
uint8_t I3G4250D_ReadReg(uint8_t reg);
void I3G4250D_WriteReg(uint8_t reg, uint8_t data);

#endif /* I3G4250D_DRIVER_I3G4250D_H_ */
