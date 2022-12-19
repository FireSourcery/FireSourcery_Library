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

		TIMER_FREQ ~= 10000 * CPR
			=> 10000RPM error ~1%
			=> RPM Min ~= 10RPM
	*/
	p_encoder->UnitT_Freq = HAL_Encoder_ConfigTimerFreq(p_encoder->CONFIG.P_HAL_ENCODER_TIMER, p_encoder->Params.CountsPerRevolution * 16666U);

	p_encoder->ExtendedTimerConversion = p_encoder->UnitT_Freq / p_encoder->CONFIG.EXTENDED_TIMER_FREQ;
}

/*!
	Uses pin ISR, or polling
*/
void Encoder_DeltaT_Init(Encoder_T * p_encoder)
{
	HAL_Encoder_InitTimer(p_encoder->CONFIG.P_HAL_ENCODER_TIMER);

	if(p_encoder->CONFIG.P_PARAMS != 0U) { memcpy(&p_encoder->Params, p_encoder->CONFIG.P_PARAMS, sizeof(Encoder_Params_T)); }

#ifdef CONFIG_ENCODER_DYNAMIC_TIMER
	_Encoder_ResetTimerFreq(p_encoder);
#else
	/* Freq * 60 < UINT32_MAX for RPM calc */
	/* Freq * 60 * PolePairs < UINT32_MAX for RPM calc */
	p_encoder->UnitT_Freq = p_encoder->CONFIG.TIMER_FREQ;
	p_encoder->ExtendedTimerConversion = p_encoder->UnitT_Freq / p_encoder->CONFIG.EXTENDED_TIMER_FREQ;
#endif

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
	p_encoder->DeltaT = ENCODER_TIMER_MAX;
	p_encoder->ExtendedTimerSaved = *p_encoder->CONFIG.P_EXTENDED_TIMER;
	HAL_Encoder_WriteTimer(p_encoder->CONFIG.P_HAL_ENCODER_TIMER, 0U);
	HAL_Encoder_ClearTimerOverflow(p_encoder->CONFIG.P_HAL_ENCODER_TIMER);
	_Encoder_ZeroAngle(p_encoder);
}

/*
	Extended timer ticks to determine capture stopped
		short timer overflow/conversion determined by TimerFreq and CountsPerRotation
	p_encoder->CONFIG.EXTENDED_TIMER_FREQ should be small, 1000, < 65536
*/
void Encoder_DeltaT_SetExtendedTimerWatchStop_Millis(Encoder_T * p_encoder, uint16_t effectiveStopTime_Millis)
{
	p_encoder->Params.ExtendedTimerDeltaTStop = effectiveStopTime_Millis * p_encoder->CONFIG.EXTENDED_TIMER_FREQ / 1000U;
}

void Encoder_DeltaT_SetExtendedTimerWatchStop_RPM(Encoder_T * p_encoder)
{
	p_encoder->Params.ExtendedTimerDeltaTStop = Encoder_DeltaT_ConvertFromRotationalSpeed_RPM(p_encoder, 1U) * p_encoder->CONFIG.EXTENDED_TIMER_FREQ / p_encoder->UnitT_Freq;
}

void Encoder_DeltaT_SetInterpolateAngleLimit(Encoder_T * p_encoder, uint16_t limit)
{
	p_encoder->InterpolateAngleLimit = limit;
}