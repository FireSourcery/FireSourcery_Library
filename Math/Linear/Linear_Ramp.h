/******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSourcery / The Firebrand Forge Inc

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
	@author FireSourcery
	@brief 	dynamic look up table
			linear_f(index) = user units

	@version V0
*/
/******************************************************************************/
#ifndef LINEAR_RAMP_H
#define LINEAR_RAMP_H

#include "Linear.h"
#include <stdint.h>

/*
	Common
*/
static inline void Linear_Ramp_SetTarget(Linear_T * p_linear, int32_t target) { p_linear->YReference = target; }
static inline int32_t Linear_Ramp_GetTarget(const Linear_T * p_linear) { return p_linear->YReference; }

/*
	Index version - allows non sequential calculation, shallow slope use must use index
	Init using Signed Slope. Index is unsigned
*/

static inline int32_t Linear_Ramp_CalcIndexOutput(const Linear_T * p_linear, uint32_t index, int32_t currentRampValue)
{
	int32_t newRampValue;

	if(currentRampValue != p_linear->YReference)
	{
		newRampValue = Linear_Function(p_linear, index);
		/* bound and set YRef as indicator of end  */
		if		((p_linear->Slope > 0) && (newRampValue > p_linear->YReference)) 	{ newRampValue = p_linear->YReference; } /* slope is positive, inc ramp if less than final */
		else if	((p_linear->Slope < 0) && (newRampValue < p_linear->YReference)) 	{ newRampValue = p_linear->YReference; } /* slope is negative, inc ramp if greater than final */
		else 																		{ newRampValue = currentRampValue; } /* else slope is 0, continue to return the same value */
	}
	else { newRampValue = currentRampValue; }

	return newRampValue;
}

static inline int32_t Linear_Ramp_ProcIndexOutputIncN(const Linear_T * p_linear, uint32_t * p_index, uint32_t indexIncreament, int32_t currentRampValue)
{
	int32_t newRampValue = Linear_Ramp_CalcIndexOutput(p_linear, *p_index, currentRampValue);
	if(newRampValue != p_linear->YReference) { *p_index += indexIncreament; }
	return newRampValue;
}

static inline int32_t Linear_Ramp_ProcIndexOutput(const Linear_T * p_linear, uint32_t * p_index, int32_t currentRampValue)
{
	return Linear_Ramp_ProcIndexOutputIncN(p_linear, p_index, 1U, currentRampValue);
}

static inline void Linear_Ramp_SetIndex(const Linear_T * p_linear, uint32_t * p_index, int32_t rampValue) { *p_index = Linear_InvFunction(p_linear, rampValue); }
static inline int32_t Linear_Ramp_ResetIndex(const Linear_T * p_linear, uint32_t * p_index) { *p_index = 0U; return Linear_Function(p_linear, 0); }

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

static inline void Linear_Ramp_ProcCmdOutput(Linear_T * p_linear, int32_t * p_currentRampValue)
{
	*p_currentRampValue = Linear_Ramp_CalcNextOutput(p_linear, *p_currentRampValue);
}

static inline void Linear_Ramp_SetCmdOutput(Linear_T * p_linear, int32_t * p_currentRampValue, int32_t matchOutput)
{
	*p_currentRampValue = matchOutput;
	Linear_Ramp_SetTarget(p_linear, matchOutput);
}

static inline void Linear_Ramp_ResetCmdOutput(Linear_T * p_linear, int32_t * p_currentRampValue)
{
	*p_currentRampValue = 0;
	Linear_Ramp_SetTarget(p_linear, 0);
}

/*
	Sets slope and initial for dynamically generated ramp
*/
static inline void Linear_Ramp_SetSlope(Linear_T * p_linear, int32_t slope_UnitPerTick, int32_t initial)
{
	_Linear_SetSlope_Y0(p_linear, slope_UnitPerTick, 1U, initial);
}

static inline void Linear_Ramp_SetSlope_Acceleration(Linear_T * p_linear, uint32_t updateFreq_Hz, int32_t slope_UnitPerSecond, int32_t initial)
{
	_Linear_SetSlope_Y0(p_linear, slope_UnitPerSecond, updateFreq_Hz, initial);
}

static inline void Linear_Ramp_SetSlope_Millis(Linear_T * p_linear, uint32_t updateFreq_Hz, uint16_t period_Ms, int32_t initial, int32_t final)
{
	_Linear_SetSlope_Y0(p_linear, final - initial, period_Ms * updateFreq_Hz / 1000U, initial);
	p_linear->YReference = final;
}

extern void Linear_Ramp_Init(Linear_T * p_linear, int32_t slope_UnitPerTick, int32_t initial, int32_t final);
extern void Linear_Ramp_Init_Acceleration(Linear_T * p_linear, uint32_t updateFreq_Hz, int32_t slope_UnitPerSecond, int32_t initial, int32_t final);
extern void Linear_Ramp_Init_Millis(Linear_T * p_linear, uint32_t updateFreq_Hz, uint16_t period_Ms, int32_t initial, int32_t final);

#endif
