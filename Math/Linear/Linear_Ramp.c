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
    @file 	Linear_Ramp.c
    @author FireSoucery
    @version V0
    @brief  dynamic look up table
			linear_f(index) = user units
*/
/******************************************************************************/
#include "Linear.h"

#include <stdint.h>


#define RAMP_SHIFT 14U
/*
 * slope must always be positive for directional index version
 */

/******************************************************************************/
/*!
	@brief
 */
/******************************************************************************/
void Linear_Ramp_InitSlope(Linear_T * p_linear, int32_t slope_UnitPerTick, int32_t initial, int32_t final)
{
	Linear_Init(p_linear, slope_UnitPerTick, 1U, 0, initial, final);
}

/*
 * user must account for acceleration sign,
 * if initial > final, acceleration is positive, ramp returns final value
 *
 * Overflow: slope_UnitPerSecond 32,767 max
 */
void Linear_Ramp_InitAcceleration(Linear_T * p_linear, int32_t slope_UnitPerSecond, uint32_t updateFreq_Hz, int32_t initial, int32_t final)
{
	Linear_Init(p_linear, slope_UnitPerSecond, updateFreq_Hz, 0, initial, final);
}

/*
 *  Overflow: (peroid_Ms * updateFreq_Hz ) max 32,767,000
 */
void Linear_Ramp_InitMillis(Linear_T * p_linear, uint16_t peroid_Ms, uint16_t updateFreq_Hz, int32_t initial, int32_t final)
{
//	Linear_Init(p_linear, (final - initial)*10U, (uint32_t)peroid_Ms * (uint32_t)updateFreq_Hz / 10000U, initial, final);
	Linear_Init_Shift(p_linear, (final - initial), (uint32_t)peroid_Ms * (uint32_t)updateFreq_Hz / 1000U, 0, initial, final, RAMP_SHIFT);
}


void Linear_Ramp_SetSlope(Linear_T * p_linear, uint32_t slope_UnitPerTick, int32_t initial)
{
	p_linear->SlopeFactor 	= ((int32_t)slope_UnitPerTick << RAMP_SHIFT);
	p_linear->YOffset 		= initial;
}


void Linear_Ramp_SetSlopeAcceleration(Linear_T * p_linear, int32_t slope_UnitPerSecond, uint32_t updateFreq_Hz, int32_t initial)
{
	p_linear->SlopeFactor 	= ((int32_t)slope_UnitPerSecond << RAMP_SHIFT) / updateFreq_Hz;
	p_linear->YOffset 		= initial;
	p_linear->SlopeDivisor 	= ((int32_t)updateFreq_Hz << RAMP_SHIFT) / slope_UnitPerSecond;
}

/*
 *  Overflow: (peroid_Ms * updateFreq_Hz ) max 32,767,000
 */
void Linear_Ramp_SetSlopeMillis(Linear_T * p_linear, uint16_t peroid_Ms, uint16_t updateFreq_Hz, int32_t initial, int32_t final)
{
	int32_t factor 		= (final - initial);
	int32_t divisor 	= (uint32_t)peroid_Ms * (uint32_t)updateFreq_Hz / 1000U;

	p_linear->SlopeFactor 	= ((int32_t)factor << RAMP_SHIFT) / divisor;
	p_linear->YOffset 		= initial;
	p_linear->YReference 	= final;
	p_linear->SlopeDivisor 	= ((int32_t)divisor << RAMP_SHIFT) / factor;

}

