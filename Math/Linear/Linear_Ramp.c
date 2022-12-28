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
	@file 	Linear_Ramp.c
	@author FireSourcery
	@version V0
	@brief
*/
/******************************************************************************/
#include "Linear_Ramp.h"

#define LINEAR_RAMP_SHIFT 15U

/******************************************************************************/
/*
	Ramp using Linear

	Slope 		=> RampInc << Shift
	YReference 	=> Target << Shift
	YOffset 	=> Current Value << Shift
	XReference 	=>
	XOffset 	=>
	DeltaX => Slope_Divisor
	DeltaY => Slope_Factor
*/
/******************************************************************************/
/*
	if initial > final AND acceleration is positive, ramp returns final value
	Overflow: final > 65535
*/
void Linear_Ramp_Init(Linear_T * p_linear, uint32_t period_Ticks, int32_t initial, int32_t final)
{
	p_linear->SlopeShift 	= LINEAR_RAMP_SHIFT;
	p_linear->InvSlopeShift = LINEAR_RAMP_SHIFT;
	Linear_Ramp_SetSlope(p_linear, period_Ticks, initial, final);
	Linear_Ramp_SetTarget(p_linear, final);
	p_linear->YOffset = initial << p_linear->SlopeShift;

}

/*
	Overflow: 	(period_Ms * updateFreq_Hz) > 131,071,000
*/
void Linear_Ramp_Init_Millis(Linear_T * p_linear, uint32_t updateFreq_Hz, uint16_t period_Ms, int32_t initial, int32_t final)
{
	p_linear->SlopeShift 	= LINEAR_RAMP_SHIFT;
	p_linear->InvSlopeShift = LINEAR_RAMP_SHIFT;
	Linear_Ramp_SetSlope_Millis(p_linear, updateFreq_Hz, period_Ms, initial, final);
	Linear_Ramp_SetTarget(p_linear, final);
	p_linear->YOffset = initial << p_linear->SlopeShift;
}
