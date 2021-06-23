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
    @file 	Linear_Ramp.c
    @author FireSoucery
    @version V0
    @brief  dynamic look up table
			linear_f(index) = user units
*/
/*******************************************************************************/

#include "Linear.h"

#include <stdint.h>

/******************************************************************************/
/*!
	@brief
 */
/******************************************************************************/
void Linear_Ramp_InitAcceleration(Linear_T * p_linear, uint32_t updateFreq_Hz, int32_t initial, int32_t final, int32_t acceleration_UnitPerSecond)
{
	Linear_Init(p_linear, acceleration_UnitPerSecond, updateFreq_Hz, initial, final);
}

void Linear_Ramp_InitMillis(Linear_T * p_linear, uint32_t updateFreq_Hz, int32_t initial, int32_t final, uint32_t peroid_Ms)
{
	Linear_Init(p_linear, (final - initial), peroid_Ms * updateFreq_Hz / 1000U, initial, final);
}



