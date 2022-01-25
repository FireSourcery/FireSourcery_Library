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
	@file  	Encoder_DeltaD.c
	@author FireSourcery
	@brief
	@version V0
 */
/******************************************************************************/
#include "Encoder_DeltaT.h"
#include "Encoder.h"
#include "HAL_Encoder.h"

#include <stdint.h>
#include <stdbool.h>

/*!
	Uses pin ISR, or polling
 */
void Encoder_DeltaT_Init(Encoder_T * p_encoder)
{
	//	Pin_Init(&p_encoder->CONFIG.PIN_PHASE_A);
	//	Pin_Init(&p_encoder->CONFIG.PIN_PHASE_B);

	HAL_Encoder_InitCaptureTime
	(
		p_encoder->CONFIG.P_HAL_ENCODER,
		p_encoder->CONFIG.PIN_PHASE_A.P_HAL_PIN,
		p_encoder->CONFIG.PIN_PHASE_A.ID,
		p_encoder->CONFIG.PIN_PHASE_B.P_HAL_PIN,
		p_encoder->CONFIG.PIN_PHASE_B.ID
	);

	if (p_encoder->CONFIG.P_PARAMS != 0U)
	{
		memcpy(&p_encoder->Params, p_encoder->CONFIG.P_PARAMS, sizeof(Encoder_Params_T));
	}

	Encoder_DeltaT_SetUnitConversion(p_encoder, p_encoder->Params.CountsPerRevolution, p_encoder->Params.DistancePerCount);

	p_encoder->DeltaT = UINT32_MAX;
}

//void Encoder_DeltaT_InitParams(Encoder_T * p_encoder, uint32_t encoderCountsPerRevolution, uint32_t encoderDistancePerCount)
//{
//	//	Pin_Init(&p_encoder->CONFIG.PIN_PHASE_A);
//	//	Pin_Init(&p_encoder->CONFIG.PIN_PHASE_B);
//
//	HAL_Encoder_InitCaptureTime
//	(
//		p_encoder->CONFIG.P_HAL_ENCODER,
//		p_encoder->CONFIG.PIN_PHASE_A.P_HAL_PIN,
//		p_encoder->CONFIG.PIN_PHASE_A.ID,
//		p_encoder->CONFIG.PIN_PHASE_B.P_HAL_PIN,
//		p_encoder->CONFIG.PIN_PHASE_B.ID
//	);
//
//	_Encoder_Init(p_encoder, encoderCountsPerRevolution, encoderDistancePerCount, p_encoder->CONFIG.DELTA_T_TIMER_FREQ);
//
//	p_encoder->DeltaT = UINT32_MAX;
//}

void Encoder_DeltaT_SetUnitConversion(Encoder_T * p_encoder, uint32_t encoderCountsPerRevolution, uint32_t encoderDistancePerCount)
{
	_Encoder_SetUnitConversion(p_encoder, encoderCountsPerRevolution, encoderDistancePerCount, p_encoder->CONFIG.DELTA_T_TIMER_FREQ);
}

/*
 * p_encoder->CONFIG.EXTENDED_TIMER_FREQ should be small, < 65536
 * //long timer ticks per short timer overflow
 */
void Encoder_DeltaT_SetExtendedTimer(Encoder_T * p_encoder, uint16_t effectiveStopTime_Millis)
{
//	p_encoder->ExtendedTimerThreshold = (CONFIG_ENCODER_HW_TIMER_COUNTER_MAX + 1UL) * p_encoder->CONFIG.EXTENDED_TIMER_FREQ / p_encoder->UnitT_Freq;
	p_encoder->Params.ExtendedTimerDeltaTStop = effectiveStopTime_Millis * p_encoder->CONFIG.EXTENDED_TIMER_FREQ / 1000U ;
	p_encoder->ExtendedTimerSaved = *p_encoder->CONFIG.P_EXTENDED_TIMER;
}

/*
 * set to lowest speed
 */
void Encoder_DeltaT_SetInitial(Encoder_T * p_encoder, uint16_t initialRpm)
{
	p_encoder->DeltaT = 0xFF00U; //minus some delay time before first capture //Encoder_DeltaT_ConvertFromRotationalSpeed_RPM(p_encoder, initialRpm);
	p_encoder->TimerCounterSaved = HAL_Encoder_ReadTimerCounter(p_encoder->CONFIG.P_HAL_ENCODER) - p_encoder->DeltaT;
	p_encoder->ExtendedTimerSaved = *p_encoder->CONFIG.P_EXTENDED_TIMER;
}

//void Encoder_DeltaT_CalibrateAngularD(Encoder_T * p_encoder)
//{
//	p_encoder->AngularD = 0U;
//}

void Encoder_DeltaT_CalibrateQuadratureReference(Encoder_T * p_encoder)
{

}

//call after having moved in the positive direction
void Encoder_DeltaT_CalibrateQuadratureDirectionPositive(Encoder_T * p_encoder)
{
	//deltaT check if phaseB is negative on an edge
//	if ((HAL_Encoder_ReadPhaseA(p_encoder->CONFIG.P_HAL_ENCODER) == true) && (HAL_Encoder_ReadPhaseB(p_encoder->CONFIG.P_HAL_ENCODER) == false))
//	{
////		 p_encoder->IsALeadBPositive = true;
//	}
//	else
//	{
//		//		 p_encoder->IsALeadBPositive = false;
//	}
}
