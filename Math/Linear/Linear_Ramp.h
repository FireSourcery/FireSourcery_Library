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
	@brief
	@version V0
*/
/******************************************************************************/
#ifndef LINEAR_RAMP_H
#define LINEAR_RAMP_H

#include "Linear.h"
#include <stdint.h>

/******************************************************************************/
/*
	Accumulative version
	Init with positive slope
*/
/******************************************************************************/
static inline void Linear_Ramp_SetTarget(Linear_T * p_linear, int32_t target) { p_linear->YReference = target; }
static inline int32_t Linear_Ramp_GetTarget(const Linear_T * p_linear) { return p_linear->YReference; }

static inline int32_t _Linear_Ramp_CalcCmdOutput(const Linear_T * p_linear, int32_t currentRampValue, int32_t steps)
{
	return ((currentRampValue << p_linear->SlopeShift) + (p_linear->Slope * steps)) >> p_linear->SlopeShift;
}

static inline int32_t Linear_Ramp_CalcCmdNextOutputN(const Linear_T * p_linear, int32_t currentRampValue, int32_t steps)
{
	int32_t newRampValue;

	if(currentRampValue < p_linear->YReference)
	{
		newRampValue = _Linear_Ramp_CalcCmdOutput(p_linear, currentRampValue, steps);
		if		(newRampValue > p_linear->YReference) 	{ newRampValue = p_linear->YReference; }
		else if	(newRampValue == currentRampValue) 		{ newRampValue++; } /* Quick fix for |Slope| < 1 */
	}
	else if(currentRampValue > p_linear->YReference)
	{
		newRampValue = _Linear_Ramp_CalcCmdOutput(p_linear, currentRampValue, 0 - steps);
		if		(newRampValue < p_linear->YReference) 	{ newRampValue = p_linear->YReference; }
		else if	(newRampValue == currentRampValue) 		{ newRampValue--; }
	}
	else { newRampValue = currentRampValue; }

	return newRampValue;
}

static inline int32_t Linear_Ramp_CalcCmdNextOutput(const Linear_T * p_linear, int32_t currentRampValue)
{
	return Linear_Ramp_CalcCmdNextOutputN(p_linear, currentRampValue, 1U);
}

static inline void Linear_Ramp_ProcCmdOutputN(Linear_T * p_linear, int32_t * p_currentRampValue, int32_t steps)
{
	*p_currentRampValue = Linear_Ramp_CalcCmdNextOutputN(p_linear, *p_currentRampValue, steps);
}

static inline void Linear_Ramp_ProcCmdOutput(Linear_T * p_linear, int32_t * p_currentRampValue)
{
	*p_currentRampValue = Linear_Ramp_CalcCmdNextOutputN(p_linear, *p_currentRampValue, 1U);
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

/******************************************************************************/
/*
	Index version
	Accumulates |Slope| < 1
*/
/******************************************************************************/
/* Set XRef and Slope on SetTarget, Unsigned Index Signed Slope */
static inline int32_t Linear_Ramp_CalcIndexOutput(const Linear_T * p_linear, uint32_t index)
{
	return ((int32_t)index < p_linear->XReference) ? Linear_Function(p_linear, index) : p_linear->YReference;
}

static inline int32_t Linear_Ramp_ProcIndexOutputN(const Linear_T * p_linear, uint32_t * p_index, uint32_t indexIncrement)
{
	int32_t newRampValue = Linear_Ramp_CalcIndexOutput(p_linear, *p_index);
	if((int32_t)*p_index < p_linear->XReference) { *p_index += indexIncrement; }
	return newRampValue;
}

/* Set Slope only on SetTarget */
// static inline int32_t Linear_Ramp_CalcIndexOutput(const Linear_T * p_linear, uint32_t index)
// {
// 	int32_t newRampValue;

// 	// if(currentRampValue != p_linear->YReference)
// 	// {
// 		newRampValue = Linear_Function(p_linear, index);
// 		/* bound and set YRef as indicator of end  */
// 		if		((p_linear->Slope > 0) && (newRampValue > p_linear->YReference)) 	{ newRampValue = p_linear->YReference; } /* slope is positive, inc ramp if less than final */
// 		else if	((p_linear->Slope < 0) && (newRampValue < p_linear->YReference)) 	{ newRampValue = p_linear->YReference; } /* slope is negative, inc ramp if greater than final */
// 		// else 																		{ newRampValue = currentRampValue; } /* else slope is 0, continue to return the same value */
// 	// }
// 	return newRampValue;
// }

// static inline int32_t Linear_Ramp_ProcIndexOutputN(const Linear_T * p_linear, uint32_t * p_index, uint32_t indexIncrement)
// {
// 	int32_t newRampValue = Linear_Ramp_CalcIndexOutput(p_linear, *p_index);

// 	if 		((p_linear->Slope > 0) && (newRampValue < p_linear->YReference)) { *p_index += indexIncrement; }
// 	else if	((p_linear->Slope < 0) && (newRampValue > p_linear->YReference)) { *p_index -= indexIncrement; }

// 	return newRampValue;
// }

static inline int32_t Linear_Ramp_ProcIndexOutput(const Linear_T * p_linear, uint32_t * p_index)
{
	return Linear_Ramp_ProcIndexOutputN(p_linear, p_index, 1U);
}

static inline void Linear_Ramp_SetIndex(const Linear_T * p_linear, uint32_t * p_index, int32_t rampValue) { *p_index = Linear_InvFunction(p_linear, rampValue); }
static inline int32_t Linear_Ramp_ResetIndex(const Linear_T * p_linear, uint32_t * p_index) { *p_index = 0; return Linear_Function(p_linear, 0); }

/******************************************************************************/
/*
	Sets slope and initial for dynamically generated ramp
	need xref for index
*/
/******************************************************************************/
static inline void Linear_Ramp_SetSlope(Linear_T * p_linear, int32_t slope_UnitPerTick)
{
	_Linear_SetSlope(p_linear, slope_UnitPerTick, 1U);
}

static inline void Linear_Ramp_SetSlope_Ticks(Linear_T * p_linear, uint32_t updatePeriod_Ticks, int32_t initial, int32_t final)
{
	uint32_t divider = (updatePeriod_Ticks != 0U) ? (updatePeriod_Ticks) : 1U;
	_Linear_SetSlope(p_linear, final - initial, divider);
}

static inline void Linear_Ramp_SetSlope_Millis(Linear_T * p_linear, uint32_t updateFreq_Hz, uint16_t period_Ms, int32_t initial, int32_t final)
{
	uint32_t divider = (period_Ms != 0U) ? (period_Ms * updateFreq_Hz / 1000U) : 1U;
	_Linear_SetSlope(p_linear, final - initial, divider);
}

/* interpolate */
// static inline void Linear_Ramp_SetSlopeTarget(Linear_T * p_linear, int32_t slope_UnitPerTick, int32_t final)
// {
// 	Linear_Ramp_SetSlope(p_linear, slope_UnitPerTick);
// 	p_linear->YReference = final;
// }

// static inline void Linear_Ramp_SetSlopeTarget_Ticks(Linear_T * p_linear, uint32_t updatePeriod_Ticks, int32_t initial, int32_t final)
// {
// 	Linear_Ramp_SetSlope_Ticks(p_linear, updatePeriod_Ticks, initial, final);
// 	p_linear->YReference = final;
// }

// static inline void Linear_Ramp_SetSlopeTarget_Millis(Linear_T * p_linear, uint32_t updateFreq_Hz, uint16_t period_Ms, int32_t initial, int32_t final)
// {
// 	Linear_Ramp_SetSlope_Millis(p_linear, updateFreq_Hz, period_Ms, initial, final);
// 	p_linear->YReference = final;
// }

/******************************************************************************/
/*
	Extern
*/
/******************************************************************************/
extern void Linear_Ramp_Init(Linear_T * p_linear, int32_t slope_UnitPerTick, int32_t initial, int32_t final);
extern void Linear_Ramp_Init_Ticks(Linear_T * p_linear, uint32_t updatePeriod_Ticks, int32_t initial, int32_t final);
extern void Linear_Ramp_Init_Millis(Linear_T * p_linear, uint32_t updateFreq_Hz, uint16_t period_Ms, int32_t initial, int32_t final);

#endif

