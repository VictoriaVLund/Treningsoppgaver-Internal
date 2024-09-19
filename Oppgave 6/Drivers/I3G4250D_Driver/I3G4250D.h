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

/* Registers (p. 28) ---------------------------------------------------------*/
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

/* CTRL_REG1 register values (p. 30-31) --------------------------------------*/

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

#define I3G4250D_PD_ENABLE					0x00
#define I3G4250D_PD_DISABLE					0x08

#define I3G4250D_ENABLE_X
#define I3G4250D_ENABLE_Y
#define I3G4250D_ENABLE_Z
#define I3G4250D_DISABLE_X
#define I3G4250D_DISABLE_Y
#define I3G4250D_DISABLE_Z

#endif /* I3G4250D_DRIVER_I3G4250D_H_ */
