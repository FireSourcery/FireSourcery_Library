/******************************************************************************/
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
/******************************************************************************/
/******************************************************************************/
/*!
	@file 	HAL_Hall.h
	@author FireSoucery
	@brief
	@version V0
*/
/******************************************************************************/
#ifndef HAL_HALL_PLATFORM_H
#define HAL_HALL_PLATFORM_H

#include "External/S32K142/include/S32K142.h"

#include <stdint.h>
#include <stdbool.h>

typedef const struct
{
// 	GPIO_Type * p_GpioBase;
// 	GPIO_Type * p_GpioBase;
// 	GPIO_Type * p_GpioBase;
//	uint8_t PinSensorA;
//	uint8_t PinSensorB;
//	uint8_t PinSensorC;
} HAL_Hall_T;


static inline uint8_t HAL_Hall_ReadSensors(const HAL_Hall_T * p_hall)
{
//	return (((p_hall->p_GpioBase->PDIR >> (p_hall->PinIndexSensorA)) | (p_hall->p_GpioBase->PDIR >>  (p_hall->PinIndexSensorB - 1)) | (p_hall->p_GpioBase->PDIR >>  (p_hall->PinIndexSensorC - 2))) & 0x07);
}

#endif
