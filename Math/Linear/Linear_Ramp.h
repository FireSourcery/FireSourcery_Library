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

*/
/******************************************************************************/
static inline void Linear_Ramp_SetTarget(Linear_T * p_linear, int32_t target) { p_linear->YReference = target << p_linear->SlopeShift; }
static inline int32_t Linear_Ramp_GetTarget(const Linear_T * p_linear) { return p_linear->YReference >> p_linear->SlopeShift; }

/*
	store as y shifted
	y0_shifted = y0_shifted + m_shifted

	alternatively, y0 = ((m_shifted * x0) >> shift)
*/
static inline int32_t _Linear_Ramp_CalcOutput(const Linear_T * p_linear, int32_t currentRampValue, int32_t steps)
{
	return ((currentRampValue) + (p_linear->Slope * steps));
}

static inline int32_t _Linear_Ramp_CalcOutput_Bounded(const Linear_T * p_linear, int32_t currentRampValue, int32_t steps)
{
	int32_t newRampValue;

	if(currentRampValue < p_linear->YReference)
	{
		newRampValue = _Linear_Ramp_CalcOutput(p_linear, currentRampValue, steps);
		if(newRampValue > p_linear->YReference) { newRampValue = p_linear->YReference; }
	}
	else if(currentRampValue > p_linear->YReference)
	{
		newRampValue = _Linear_Ramp_CalcOutput(p_linear, currentRampValue, 0 - steps);
		if(newRampValue < p_linear->YReference) { newRampValue = p_linear->YReference; }
	}
	else { newRampValue = currentRampValue; }

	return newRampValue;
}


static inline int32_t Linear_Ramp_CalcOutputN(const Linear_T * p_linear, int32_t currentRampValue, int32_t steps)
{
	return _Linear_Ramp_CalcOutput_Bounded(p_linear, currentRampValue, steps) >> 16U;
}

static inline int32_t Linear_Ramp_CalcOutput(const Linear_T * p_linear, int32_t currentRampValue)
{
	return Linear_Ramp_CalcOutputN(p_linear, currentRampValue, 1U);
}

static inline void Linear_Ramp_ProcOutputN(Linear_T * p_linear, int32_t steps)
{
	if(p_linear->YOffset != p_linear->YReference)
	{
		p_linear->YOffset = _Linear_Ramp_CalcOutput_Bounded(p_linear, p_linear->YOffset, steps);
	}
}

static inline void Linear_Ramp_ProcOutput(Linear_T * p_linear)
{
	Linear_Ramp_ProcOutputN(p_linear, 1U);
}

static inline int32_t Linear_Ramp_GetOutput(const Linear_T * p_linear)
{
	return p_linear->YOffset >> p_linear->SlopeShift;
}

static inline void Linear_Ramp_SetOutputState(Linear_T * p_linear, int32_t matchOutput)
{
	p_linear->YOffset = matchOutput << p_linear->SlopeShift;
	Linear_Ramp_SetTarget(p_linear, matchOutput);
}

static inline void Linear_Ramp_ZeroOutputState(Linear_T * p_linear)
{
	p_linear->YOffset = 0;
	Linear_Ramp_SetTarget(p_linear, 0);
}

/******************************************************************************/
/*
	Sets slope and initial for dynamically generated ramp
	need xref for index
*/
/******************************************************************************/
/* period_Ticks != 0  */
static inline void Linear_Ramp_SetSlope(Linear_T * p_linear, uint32_t period_Ticks, int32_t initial, int32_t final)
{
	_Linear_SetSlope(p_linear, final - initial, period_Ticks);
}

static inline void Linear_Ramp_SetSlope_Millis(Linear_T * p_linear, uint32_t updateFreq_Hz, uint16_t period_Ms, int32_t initial, int32_t final)
{
	uint32_t ticks = (period_Ms != 0U) ? (period_Ms * updateFreq_Hz / 1000U) : 1U;
	Linear_Ramp_SetSlope(p_linear, ticks, initial, final);
}

static inline void Linear_Ramp_SetSlopeTarget(Linear_T * p_linear, uint32_t updatePeriod_Ticks, int32_t initial, int32_t final)
{
	Linear_Ramp_SetSlope(p_linear, updatePeriod_Ticks, initial, final);
	Linear_Ramp_SetTarget(p_linear, final);
}

static inline void Linear_Ramp_SetSlopeTarget_Millis(Linear_T * p_linear, uint32_t updateFreq_Hz, uint16_t period_Ms, int32_t initial, int32_t final)
{
	Linear_Ramp_SetSlope_Millis(p_linear, updateFreq_Hz, period_Ms, initial, final);
	Linear_Ramp_SetTarget(p_linear, final);
}

/******************************************************************************/
/*
	Extern
*/
/******************************************************************************/
extern void Linear_Ramp_Init(Linear_T * p_linear, uint32_t updatePeriod_Ticks, int32_t initial, int32_t final);
extern void Linear_Ramp_Init_Millis(Linear_T * p_linear, uint32_t updateFreq_Hz, uint16_t period_Ms, int32_t initial, int32_t final);

#endif

