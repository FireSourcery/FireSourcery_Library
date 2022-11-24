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

/*!
	Uses pin ISR, or polling
*/
void Encoder_DeltaT_Init(Encoder_T * p_encoder)
{
	HAL_Encoder_InitCaptureTime
	(
		p_encoder->CONFIG.P_HAL_ENCODER,
		p_encoder->PhaseA.CONFIG.P_HAL_PIN, p_encoder->PhaseA.CONFIG.ID,
		p_encoder->PhaseB.CONFIG.P_HAL_PIN, p_encoder->PhaseB.CONFIG.ID
	);

	// Pin_Input_Init(&p_encoder->PhaseA);
	// Pin_Input_Init(&p_encoder->PhaseB);

	if (p_encoder->CONFIG.P_PARAMS != 0U)
	{
		memcpy(&p_encoder->Params, p_encoder->CONFIG.P_PARAMS, sizeof(Encoder_Params_T));
	}

	p_encoder->UnitT_Freq = p_encoder->CONFIG.DELTA_T_TIMER_FREQ;
	_Encoder_ResetUnitsAngular(p_encoder);
	_Encoder_ResetUnitsLinear(p_encoder);
	_Encoder_ResetUnitsScalarSpeed(p_encoder);

	Encoder_Zero(p_encoder);
	Encoder_DeltaT_SetInitial(p_encoder, 0U);
}

/*
	Long timer ticks to determine short timer overflowed or stopped capture
	p_encoder->CONFIG.EXTENDED_TIMER_FREQ should be small, < 65536
*/
void Encoder_DeltaT_SetExtendedTimer(Encoder_T * p_encoder, uint16_t effectiveStopTime_Millis)
{
	p_encoder->Params.ExtendedTimerDeltaTStop = effectiveStopTime_Millis * p_encoder->CONFIG.EXTENDED_TIMER_FREQ / 1000U;
}

/*
	Set default as 1s or 1rpm
*/
void Encoder_DeltaT_SetExtendedTimerDefault(Encoder_T * p_encoder)
{
	uint32_t time1Second = p_encoder->CONFIG.EXTENDED_TIMER_FREQ;
	uint32_t time1Rpm = Encoder_DeltaT_ConvertFromRotationalSpeed_RPM(p_encoder, 1U) * p_encoder->CONFIG.EXTENDED_TIMER_FREQ / p_encoder->CONFIG.DELTA_T_TIMER_FREQ;
	p_encoder->Params.ExtendedTimerDeltaTStop = (time1Second < time1Rpm) ? time1Second : time1Rpm;
}

/*
	Set initial capture to lowest speed, DeltaT = CONFIG_ENCODER_HW_TIMER_COUNTER_MAX
*/
void Encoder_DeltaT_SetInitial(Encoder_T * p_encoder, uint16_t initialRpm)
{
	(void)initialRpm;
	p_encoder->DeltaT = CONFIG_ENCODER_HW_TIMER_COUNTER_MAX;
//	p_encoder->DeltaT = Encoder_DeltaT_ConvertFromRotationalSpeed_RPM(p_encoder, initialRpm);
//	CONFIG_ENCODER_HW_TIMER_COUNTER_MAX - p_encoder->CONFIG.DELTA_T_TIMER_FREQ / p_encoder->CONFIG.POLLING_FREQ + 1U;
	p_encoder->TimerCounterSaved = HAL_Encoder_ReadTimerCounter(p_encoder->CONFIG.P_HAL_ENCODER) - (CONFIG_ENCODER_HW_TIMER_COUNTER_MAX - p_encoder->CONFIG.DELTA_T_TIMER_FREQ / p_encoder->CONFIG.POLLING_FREQ + 1U);
	p_encoder->ExtendedTimerSaved = *p_encoder->CONFIG.P_EXTENDED_TIMER;
}

/*
	Run on calibration routine
*/
// void Encoder_DeltaT_CalibrateQuadratureReference(Encoder_T * p_encoder)
// {

// }

// //call after having moved in the positive direction
// void Encoder_DeltaT_CalibrateQuadraturePositive(Encoder_T * p_encoder)
// {
// 	//deltaT check if phaseB is negative on an edge
// //	if ((HAL_Encoder_ReadPhaseA(p_encoder->CONFIG.P_HAL_ENCODER) == true) && (HAL_Encoder_ReadPhaseB(p_encoder->CONFIG.P_HAL_ENCODER) == false))
// //	{
// ////		 p_encoder->IsALeadBPositive = true;
// //	}
// //	else
// //	{
// //		//	p_encoder->IsALeadBPositive = false;
// //	}
// }
