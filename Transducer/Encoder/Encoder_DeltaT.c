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
#include "Encoder.h"
#include "HAL_Encoder.h"

#include <stdint.h>
#include <stdbool.h>

/*!
	Uses pin ISR, or polling
 */
void Encoder_DeltaT_Init
(
	Encoder_T * p_encoder,
	HAL_Encoder_T * p_hal_encoder,
	uint32_t timerCounterMax,
	uint32_t timerFreq,
	uint32_t pollingFreq,	/* Polling and InterpolateD */
	uint32_t encoderDistancePerCount,
	uint32_t encoderCountsPerRevolution
//	uint8_t angleDataBits
)
{
	Encoder_Init
	(
		p_encoder,
		p_hal_encoder,
		timerCounterMax,
		timerFreq,
		pollingFreq,
		encoderDistancePerCount,
		encoderCountsPerRevolution
//		angleDataBits
	);

	HAL_Encoder_InitCaptureTime(p_hal_encoder);
//	HAL_Encoder_WriteTimerCounterMax(p_hal_encoder, timerCounterMax);
//	HAL_Encoder_WriteTimerCounterFreq(p_hal_encoder, timerFreq);
}

/*
 * extendedTimerFreq should be small, < 65536
 */
void Encoder_DeltaT_InitExtendedTimer(Encoder_T * p_encoder, const volatile uint32_t * p_extendedTimer, uint16_t extendedTimerFreq, uint16_t effectiveStopTime_Millis)
{
	p_encoder->p_ExtendedDeltaTimer = p_extendedTimer;
	p_encoder->ExtendedDeltaTimerFreq = extendedTimerFreq;
	p_encoder->ExtendedDeltaTimerThreshold = (p_encoder->TimerCounterMax + 1U) * extendedTimerFreq / p_encoder->UnitT_Freq; //long timer ticks per short timer overflow
	p_encoder->ExtendedDeltaTimerEffectiveStopTime = effectiveStopTime_Millis * extendedTimerFreq / 1000U ;
	p_encoder->ExtendedDeltaTimerSaved = *p_encoder->p_ExtendedDeltaTimer;
}

/*
 * set to lowest speed or 1rpm
 */
void Encoder_DeltaT_SetInitial(Encoder_T * p_encoder, uint16_t initialRpm)
{
	p_encoder->DeltaT = Encoder_ConvertRotationalSpeedToDeltaT_RPM(p_encoder, initialRpm);
//	if (p_encoder->TimerCounterSaved > )
	p_encoder->TimerCounterSaved = HAL_Encoder_ReadTimerCounter(p_encoder->p_HAL_Encoder);
			//- p_encoder->DeltaT; or loop around
	p_encoder->ExtendedDeltaTimerSaved = *p_encoder->p_ExtendedDeltaTimer;
//	Encoder_Zero( p_encoder);
}


void Encoder_DeltaT_CalibrateAngularD(Encoder_T * p_encoder)
{
	p_encoder->AngularD = 0U;
}

void Encoder_DeltaT_CalibrateQuadratureReference(Encoder_T * p_encoder)
{

}

//call after having moved in the positive direction
void Encoder_DeltaT_CalibrateQuadratureDirectionPositive(Encoder_T * p_encoder)
{
	//deltaT check if phaseB is negative on an edge
//	if ((HAL_Encoder_ReadPhaseA(p_encoder->p_HAL_Encoder) == true) && (HAL_Encoder_ReadPhaseB(p_encoder->p_HAL_Encoder) == false))
//	{
////		 p_encoder->IsALeadBPositive = true;
//	}
//	else
//	{
//		//		 p_encoder->IsALeadBPositive = false;
//	}
}
