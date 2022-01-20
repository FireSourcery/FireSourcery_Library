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
    @file 	Linear_Ramp.h
    @author FireSoucery
    @brief 	dynamic look up table
			linear_f(index) = user units

    @version V0
*/
/******************************************************************************/
#ifndef LINEAR_RAMP_H
#define LINEAR_RAMP_H

#include "Linear.h"

#include <stdint.h>


static inline void Linear_Ramp_InitSlope(Linear_T * p_linear, int32_t initial, int32_t final, int32_t slope_UnitPerTick)
{
	Linear_Init(p_linear, slope_UnitPerTick, 1U, initial, final);
}

/*
 * user must account for acceleration sign,
 * if initial > final, acceleration is positive, ramp returns final value
 *
 * slope_UnitPerSecond 32,767 max
 */
//static inline void Linear_Ramp_InitAcceleration(Linear_T * p_linear, uint32_t updateFreq_Hz, int32_t initial, int32_t final, int32_t slope_UnitPerSecond)
//{
//	Linear_Init(p_linear, slope_UnitPerSecond, updateFreq_Hz, initial, final);
//}

/*
 *  Overflow: (peroid_Ms * updateFreq_Hz ) max 32,767,000
 *  (peroid_Ms * updateFreq_Hz) max 327,670,000
 *
 *  (final - initial)  max 3276
 */
static inline void Linear_Ramp_InitMillis(Linear_T * p_linear, uint16_t updateFreq_Hz, int32_t initial, int32_t final, uint16_t peroid_Ms)
{
//	Linear_Init(p_linear, (final - initial)*10U, (uint32_t)peroid_Ms * (uint32_t)updateFreq_Hz / 10000U, initial, final);
	Linear_Init_Shift(p_linear, (final - initial) , (uint32_t)peroid_Ms * (uint32_t)updateFreq_Hz / 1000U, initial, final, 14U);
}

static inline int32_t Linear_Ramp_CalcUp(Linear_T * p_linear, uint32_t index, int32_t final)
{
	int32_t rampValue = Linear_Function(p_linear, index);
	if(rampValue > final)	{rampValue = final;}
	return rampValue;
}

static inline int32_t Linear_Ramp_CalcDown(Linear_T * p_linear, uint32_t index, int32_t final)
{
	int32_t rampValue = Linear_Function(p_linear, index);
	if(rampValue < final)	{rampValue = final;}
	return rampValue;
}

static inline int32_t Linear_Ramp_CalcTarget(Linear_T * p_linear, uint32_t index, int32_t final)
{
	int32_t rampValue = Linear_Function(p_linear, index);

	if(p_linear->SlopeFactor < (int32_t)0) /* slope is negative, inc ramp if greater than final */
	{
		if(rampValue < final)	{rampValue = final;}
	}
	else if(p_linear->SlopeFactor > (int32_t)0) /* slope is positive, inc ramp if less than final */
	{
		if(rampValue > final)	{rampValue = final;}
	}
	/* else slope is 0, continue to return the same value */

	return rampValue;
}

static inline int32_t Linear_Ramp_ProcTarget(Linear_T * p_linear, uint32_t * p_index, int32_t final)
{
	int32_t rampValue = Linear_Ramp_CalcTarget(p_linear, *p_index, final);

	if(rampValue != final)
	{
		*p_index += 1U;
	}

	return rampValue;
}

static inline int32_t Linear_Ramp_ProcTargetIncIndex(Linear_T * p_linear, uint32_t * p_index, uint32_t indexIncreament, int32_t final)
{
	int32_t rampValue = Linear_Ramp_CalcTarget(p_linear, *p_index, final);

	if(rampValue != final)
	{
		*p_index += indexIncreament;
	}

	return rampValue;
}


static inline int32_t Linear_Ramp_GetFinal(Linear_T * p_linear)
{
	return p_linear->RangeReference;
}

static inline void Linear_Ramp_SetFinal(Linear_T * p_linear,  int32_t final)
{
	p_linear->RangeReference = final;
}

static inline int32_t Linear_Ramp_Calc(Linear_T * p_linear, uint32_t index)
{
	return Linear_Ramp_CalcTarget(p_linear, index, p_linear->RangeReference);
}

static inline int32_t Linear_Ramp_Proc(Linear_T * p_linear, uint32_t * p_index)
{
	int32_t rampValue = Linear_Ramp_Calc(p_linear, *p_index);

	if(rampValue != p_linear->RangeReference)
	{
		*p_index += 1U;
	}

	return rampValue;
}

static inline int32_t Linear_Ramp_ProcIncIndex(Linear_T * p_linear, uint32_t * p_index, uint32_t indexIncreament)
{
	int32_t rampValue = Linear_Ramp_Calc(p_linear, *p_index);

	if(rampValue != p_linear->RangeReference)
	{
		*p_index += indexIncreament;
	}

	return rampValue;
}

//static inline int32_t Linear_Ramp_Proc(Linear_T * p_linear, uint32_t value)
//{
//	int32_t rampValue = value << 16U + Slope >>16
//	return rampValue;
//}

//static inline uint32_t Linear_Ramp_CalcIndex(Linear_T * p_linear, int32_t rampValue)
//{
//
//}
//
//static inline void Linear_Ramp_SetIndex(Linear_T * p_linear, uint32_t * p_index, int32_t rampValue)
//{
//	*p_index = Linear_Ramp_CalcIndex(p_linear, rampValue);
//}

#endif
