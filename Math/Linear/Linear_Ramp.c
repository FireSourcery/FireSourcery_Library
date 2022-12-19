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

/******************************************************************************/
/*

*/
/******************************************************************************/
/*
	Other functions provide higher granularity
	Slope is signed, if initial > final AND acceleration is positive, ramp returns final value
	Overflow: slope_UnitPerSecond > 131,071 [17-bits]
*/
void Linear_Ramp_Init(Linear_T * p_linear, int32_t slope_UnitPerTick, int32_t initial, int32_t final)
{
	Linear_Init(p_linear, slope_UnitPerTick, 1U, initial, final);
}

void Linear_Ramp_Init_Ticks(Linear_T * p_linear, uint32_t updatePeriod_Ticks,  int32_t initial, int32_t final)
{
	uint32_t divider = (updatePeriod_Ticks != 0U) ? (updatePeriod_Ticks) : 1U;
	Linear_Init(p_linear, final - initial, divider, initial, final);
}

/*
	Overflow: 	(period_Ms * updateFreq_Hz) > 131,071,000
				(final - initial) > 131,071
*/
void Linear_Ramp_Init_Millis(Linear_T * p_linear, uint32_t updateFreq_Hz, uint16_t period_Ms, int32_t initial, int32_t final)
{
	uint32_t divider = (period_Ms != 0U) ? (period_Ms * updateFreq_Hz / 1000U) : 1U;
	Linear_Init(p_linear, final - initial, divider, initial, final);
}
