/*******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSoucery / The Firebrand Forge Inc

	This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
/*******************************************************************************/
/*******************************************************************************/
/*!
    @file
    @author FireSoucery
    @brief
    @version V0
*/
/*******************************************************************************/
#ifndef MOTOR_FLASH_H
#define MOTOR_FLASH_H

#include "Motor/Motor.h"
#include "Peripheral/Flash/Flash.h"

//typedef const struct
//{
//
////	Flash_T EepromController;
//
//	//	uint32_t *p_FlashStart;
//	//	uint32_t FlashSize;
//
//	uint32_t *p_ParametersStart;
//	uint32_t ParametersSize;
//
//
//}
//MotorFlash_T;
//
//typedef struct
//{
//
////__aligned__
//	qfrac16_t FocOpenLoopVq;
//	qfrac16_t FocAlignVd;
//
//    uint8_t PolePairs;
//	uint32_t EncoderCountsPerRevolution;
//	uint32_t EncoderDistancePerCount;
//}
//Motor_Parameters_T; //flash map

//#define MOTOR_FLASH_PARAMETERS_BASE                             FLASH_EEPROM_START
//#define MOTOR_FLASH_PARAMETERS                                  ((Motor_Parameters_T *)FLASH_EEPROM_START)
//#define MOTOR_FLASH_PARAMETERS_DATA                             ((uint32_t *)FLASH_EEPROM_START)

static Flash_T MotorFlashController;

static inline uint32_t MotorFlash_LoadParameterAll(Motor_T * p_motor)
{
	Flash_EEPROM_ReadBytes(&MotorFlashController, &p_motor->Parameters, p_motor->p_MotorFlashRegion->p_Address, sizeof(Motor_Parameters_T));
}

static inline uint32_t MotorFlash_SaveParametersAll(Motor_T * p_motor)
{
 	Flash_EEPROM_WriteAlignedBytes(&MotorFlashController, p_motor->p_MotorFlashRegion->p_Address, &p_motor->Parameter, sizeof(Motor_Parameters_T));
}


//static inline void MotorFlash_Init //per app
//(
//	Motor_T* p_motor,
//	HAL_Flash * p_hal_flash,
//	void (* flashCallback)(void *),
//	void * p_callbackData
//)
//{
//
//	Flash_Init(&p_motor->Flash, hal);
//}

static inline void MotorFlashRegion_Init //per motor
(
	Motor_T* p_motor,
	const Flash_Region_T * p_flashRegion
)
{
	p_motor->p_MotorFlashRegion = p_flashRegion;
}
//static inline uint32_t MotorFlash_LoadParameter_FocOpenLoopVq(Motor * p_motor)
//{
//	return ((Motor_Parameters_T *)(p_flash->p_ParametersStart))->FocOpenLoopVq;
//
//}
//
//static inline void MotorFlash_WriteParameter_FocOpenLoopVq(Motor * p_motor,  qfrac16_t value)
//{
//
//
//	Flash_WriteBuffer(&((Motor_Parameters_T *)(p_flash->p_ParametersStart))->FocOpenLoopVq,  value);
//
//	Flash_WriteBuffer16(&(MOTOR_FLASH_PARAMETERS->FocOpenLoopVq)),  value);
//
//}

#endif
