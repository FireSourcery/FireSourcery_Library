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
	@file  	Encoder_DeltaD.c
	@author FireSourcery
	@brief
	@version V0
*/
/******************************************************************************/
#include "Encoder_DeltaT.h"
#include <string.h>

void _Encoder_ResetTimerFreq(Encoder_T * p_encoder)
{
	/*
		RPM * CPR / 60[Seconds] = CPS
		CPS = T_FREQ [Hz] / deltaT_ticks [timerticks/Count]
		RPM * CPR / 60[Seconds] = T_FREQ [Hz] / deltaT_ticks
		Error ~ 1, deltaT_ticks = 100
		=> T_FREQ/CPS >= 100, (CPS/T_FREQ <= .01)
			T_FREQ /(RPM * CPR / 60) >= 100
		eg. RPM = 10000
			T_FREQ >= 100*(10000RPM * CPR / 60)
			T_FREQ >= 16666 * CPR

		Min: deltaT_ticks = 65535
			RPM = (T_FREQ / CPR) * (60 / 65535)
			=> 15 ~= 16666 * (60 / 65535)
			T_FREQ = DeltaT_Ticks * (CountsPerRevolution * rpm) / 60
			T_FREQ = 65535 * (24 * 100) / 60 ~=2.6Mhz

		TIMER_FREQ ~= 10000 * CPR
			=> 10000RPM error ~1%
			=> RPM Min ~= 10RPM
	*/
	p_encoder->UnitT_Freq = HAL_Encoder_ConfigTimerFreq(p_encoder->CONFIG.P_HAL_ENCODER_TIMER, p_encoder->Params.CountsPerRevolution * 16666U);

	p_encoder->ExtendedTimerConversion = p_encoder->UnitT_Freq / p_encoder->CONFIG.EXTENDED_TIMER_FREQ;
}

void _Encoder_DeltaT_Init(Encoder_T * p_encoder)
{
	HAL_Encoder_InitTimer(p_encoder->CONFIG.P_HAL_ENCODER_TIMER);

#ifdef CONFIG_ENCODER_DYNAMIC_TIMER
	_Encoder_ResetTimerFreq(p_encoder);
#else
	p_encoder->ExtendedTimerConversion = p_encoder->CONFIG.TIMER_FREQ / p_encoder->CONFIG.EXTENDED_TIMER_FREQ;
#endif
}

/*!
	Uses pin ISR, or polling
*/
void Encoder_DeltaT_Init(Encoder_T * p_encoder)
{
	if(p_encoder->CONFIG.P_PARAMS != 0U) { memcpy(&p_encoder->Params, p_encoder->CONFIG.P_PARAMS, sizeof(Encoder_Params_T)); }
	_Encoder_DeltaT_Init(p_encoder);

	p_encoder->UnitT_Freq = p_encoder->CONFIG.TIMER_FREQ;
	_Encoder_ResetUnitsAngular(p_encoder);
	_Encoder_ResetUnitsLinear(p_encoder);
	_Encoder_ResetUnitsScalarSpeed(p_encoder);

	p_encoder->DeltaD = 1U; /* Unused if using Encoder_DeltaT Functions, effective for shared functions only */
	Encoder_DeltaT_SetInitial(p_encoder);
}

/*
	Set initial capture to lowest speed, DeltaT = ENCODER_TIMER_MAX
*/
void Encoder_DeltaT_SetInitial(Encoder_T * p_encoder)
{
	HAL_Encoder_WriteTimer(p_encoder->CONFIG.P_HAL_ENCODER_TIMER, 0U);
	HAL_Encoder_ClearTimerOverflow(p_encoder->CONFIG.P_HAL_ENCODER_TIMER);
	p_encoder->ExtendedTimerPrev = *p_encoder->CONFIG.P_EXTENDED_TIMER;
	p_encoder->DeltaT = 0xFFFFFFFFUL;
	p_encoder->InterpolationIndex = 0U;
	_Encoder_ZeroAngle(p_encoder);
}

/*
	Extended timer ticks to determine capture stopped
	EXTENDED_TIMER_FREQ should be small, 1000, < 65536
*/
void Encoder_DeltaT_SetExtendedTimerWatchStop_Millis(Encoder_T * p_encoder, uint16_t effectiveStopTime_Millis)
{
	p_encoder->Params.ExtendedTimerDeltaTStop = effectiveStopTime_Millis * p_encoder->CONFIG.EXTENDED_TIMER_FREQ / 1000U;
}

// void Encoder_DeltaT_SetExtendedTimerWatchStop_RPM(Encoder_T * p_encoder)
// {
// 	p_encoder->Params.ExtendedTimerDeltaTStop = Encoder_DeltaT_ConvertFromRotationalSpeed_RPM(p_encoder, 1U) * p_encoder->CONFIG.EXTENDED_TIMER_FREQ / p_encoder->UnitT_Freq;
// }

void Encoder_DeltaT_SetInterpolateAngleScalar(Encoder_T * p_encoder, uint16_t scalar)
{
	p_encoder->Params.InterpolateAngleScalar = scalar;
	_Encoder_ResetUnitsInterpolateAngle(p_encoder);
}