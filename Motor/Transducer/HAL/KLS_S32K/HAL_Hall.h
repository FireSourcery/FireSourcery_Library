/******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSoucery / The Firebrand Forge Inc

	This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).

	This program is free software: you can redistribute it and/or modify
	it under the terupdateInterval of the GNU General Public License as published by
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
#ifndef HAL_HALL_HAL_H
#define HAL_HALL_HAL_H

#include "External/S32K142/include/S32K142.h"

#include <stdint.h>
#include <stdbool.h>

typedef const struct
{
 	GPIO_Type * p_GpioBase;
	uint8_t PinIndexSensorA;
	uint8_t PinIndexSensorB;
	uint8_t PinIndexSensorC;
} HAL_Hall_T;

/*
 * Only 1 set of hall sensor on KLS_S32K. Can use hard coded value
 */
static inline uint8_t HAL_Hall_ReadSensors(const HAL_Hall_T * p_hall)
{
	(void)p_hall;

	/*
	 * 	PTE4 -> SA
	 * 	PTE5 -> SB
	 * 	PTE10 -> SC
	 */
	return (((PTE->PDIR >> 4) | (PTE->PDIR >> 4) | (PTE->PDIR >> 8)) & 0x07);
}

#endif
