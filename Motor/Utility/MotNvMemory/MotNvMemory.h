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
#ifndef MOT_FLASH_H
#define MOT_FLASH_H

#include "Motor/Motor/Motor.h"
//#include "Peripheral/EEPROM/EEPROM.h"
//#include "Peripheral/Flash/Flash.h"



//typedef struct
//{
//	Flash_T Flash;
//	EEPROM_T Eeprom;
//}
//MotFlash_T;
//static inline uint32_t MotorFlash_Proc(MotFlash_T * p_motor)
//{
// if context
// saveall

//}
//#ifdef CONFIG_MOTOR_PARAM_IN_RAM
//static inline uint32_t MotorFlash_LoadParameterAll(Motor_T * p_motor)
//{
//	EEPROM_ReadBytes(&MotorFlashMain, &p_motor->Parameters, p_motor->p_Constants->P_EEPROM, sizeof(Motor_Parameters_T));
//}
//
//static inline uint32_t MotorFlash_SaveParametersAll(Motor_T * p_motor)
//{
// 	EEPROM_WriteAlignedBytes(&MotorFlashMain, p_motor->p_Constants->P_EEPROM, &p_motor->Parameters, sizeof(Motor_Parameters_T));
//}
//
////static inline void MotorFlash_OnBlock(void * p_void)
////{
////
////}
//
////static inline uint32_t MotorFlash_Load_PolePairs(Motor_T * p_motor)
////{
////	return p_motor->CONFIG.p_Parameters->PolePairs;
////}
//
//#endif




//static inline void MotorFlash_Save_PolePairs(Motor_T * p_motor,  qfrac16_t value)
//{
//	EEPROM_WriteBuffer16(&p_motor->CONFIG.P_PARAMETERS->PolePairs,  value);
//
//#ifdef CONFIG_MOT_FLASH_PARAMS_EEPROM
//	EEPROM_WriteBuffer16(&p_motor->CONFIG.p_Parameters->PolePairs,  value);
//#elif defined(CONFIG_MOT_FLASH_PARAMS_FLASH)
//	Flash_WriteBuffer(&p_motor->CONFIG.p_Parameters->PolePairs,  value);
//#endif
//}

#endif



