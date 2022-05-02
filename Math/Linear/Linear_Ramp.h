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

static inline void Linear_Ramp_SetTarget(Linear_T * p_linear, int32_t target)
{
	p_linear->YReference = target;
}

static inline int32_t Linear_Ramp_GetTarget(const Linear_T * p_linear)
{
	return p_linear->YReference;
}

/*
	Index version - allows non sequential calculation, shallow slope use must use index
	Init using Signed Slope. Index is unsigned
*/
static inline int32_t Linear_Ramp_ProcIndexOutputInc(const Linear_T * p_linear, uint32_t * p_index, uint32_t indexIncreament, int32_t currentRampValue)
{
	int32_t newRampValue;

	if(currentRampValue != p_linear->YReference)
	{
		newRampValue = Linear_Function(p_linear, *p_index);

		if(p_linear->Slope > 0) /* slope is positive, inc ramp if less than final */
		{
			if(newRampValue > p_linear->YReference) { newRampValue = p_linear->YReference; }
		}
		else if(p_linear->Slope < 0) /* slope is negative, inc ramp if greater than final */
		{
			if(newRampValue < p_linear->YReference) { newRampValue = p_linear->YReference; }
		}
		/* else slope is 0, continue to return the same value */

		if(newRampValue != p_linear->YReference) { *p_index += indexIncreament; }
	}
	else
	{
		newRampValue = currentRampValue;
	}

	return newRampValue;
}

static inline int32_t Linear_Ramp_ProcIndexOutput(const Linear_T * p_linear, uint32_t * p_index, int32_t currentRampValue)
{
	return Linear_Ramp_ProcIndexOutputInc(p_linear, p_index, 1U, currentRampValue);
}

static inline int32_t Linear_Ramp_ResetIndex(const Linear_T * p_linear, uint32_t * p_index)
{
	*p_index = 0U;
	return Linear_Function(p_linear, 0);
}

static inline void Linear_Ramp_SetIndex(const Linear_T * p_linear, uint32_t * p_index, int32_t rampValue)
{
	*p_index = Linear_InvFunction(p_linear, rampValue);
}

/*
	Accumulative version - sequential calculation only
	Init with positive slope
*/
static inline int32_t Linear_Ramp_CalcNextOutput(const Linear_T * p_linear, int32_t currentRampValue)
{
	int32_t newRampValue;

	if(currentRampValue < p_linear->YReference)
	{
		newRampValue = ((currentRampValue << p_linear->SlopeShift) + p_linear->Slope) >> p_linear->SlopeShift;

		if(newRampValue > p_linear->YReference) { newRampValue = p_linear->YReference; }
	}
	else if(currentRampValue > p_linear->YReference)
	{
		newRampValue = ((currentRampValue << p_linear->SlopeShift) - p_linear->Slope) >> p_linear->SlopeShift;

		if(newRampValue < p_linear->YReference) { newRampValue = p_linear->YReference; }
	}
	else
	{
		newRampValue = currentRampValue;
	}

	return newRampValue;
}


/*
	Sets slope and initial for dynamically generated ramp
*/
#define RAMP_SHIFT 14U

static inline void Linear_Ramp_SetSlope(Linear_T * p_linear, int32_t slope_UnitPerTick, int32_t initial)
{
	p_linear->Slope = slope_UnitPerTick << RAMP_SHIFT;
	p_linear->YOffset = initial;
}

static inline void Linear_Ramp_SetSlopeAcceleration(Linear_T * p_linear, int32_t slope_UnitPerSecond, uint32_t updateFreq_Hz, int32_t initial)
{
	p_linear->Slope = (slope_UnitPerSecond << RAMP_SHIFT) / updateFreq_Hz;
	p_linear->InvSlope = (updateFreq_Hz << RAMP_SHIFT) / slope_UnitPerSecond;
	p_linear->YOffset = initial;
}

static inline void Linear_Ramp_SetSlopeMillis(Linear_T * p_linear, uint16_t peroid_Ms, uint32_t updateFreq_Hz, int32_t initial, int32_t final)
{
	const int32_t factor = (final - initial);
	const int32_t divisor = (uint32_t)peroid_Ms * updateFreq_Hz / 1000U;

	p_linear->Slope = (factor << RAMP_SHIFT) / divisor;
	p_linear->InvSlope = (divisor << RAMP_SHIFT) / factor;
	p_linear->YOffset = initial;
	p_linear->YReference = final;
} 

extern void Linear_Ramp_Init(Linear_T * p_linear, int32_t slope_UnitPerTick, int32_t initial, int32_t final);
extern void Linear_Ramp_InitAcceleration(Linear_T * p_linear, int32_t slope_UnitPerSecond, uint32_t updateFreq_Hz, int32_t initial, int32_t final);
extern void Linear_Ramp_InitMillis(Linear_T * p_linear, uint16_t peroid_Ms, uint32_t updateFreq_Hz, int32_t initial, int32_t final);

#endif
