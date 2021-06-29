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
	@file  	Encoder_DeltaD.h
	@author FireSourcery
	@brief 	Capture Delta D Mode, Polling time is fixed
	@version V0
 */
/******************************************************************************/
#ifndef ENCODER_DELTA_D_H
#define ENCODER_DELTA_D_H

#include "Encoder.h"
#include "HAL_Encoder.h"
#include "Private.h"

#include <stdint.h>
#include <stdbool.h>

/*!
	@brief 	Capture DeltaD, per fixed changed in time, via timer periodic interrupt
			Looping Angle Capture

 	 	 	TimerCounter should loop for correct angular position
 	 	 	TimerCounterMax == EncoderRes

 */
static inline void Encoder_DeltaD_CaptureSinglePhase_IO(Encoder_T * p_encoder)
{
//	CaptureEncoderDeltaIncreasing(p_encoder, &p_encoder->DeltaD);

	uint32_t timerCounterValue = HAL_Encoder_ReadTimerCounter(p_encoder->p_HAL_Encoder);

	if (timerCounterValue < p_encoder->TimerCounterSaved) /* TimerCounter overflow */
	{
		p_encoder->DeltaD = p_encoder->EncoderResolution - p_encoder->TimerCounterSaved + timerCounterValue + 1U;
	}
	else /* normal case */
	{
		p_encoder->DeltaD = timerCounterValue - p_encoder->TimerCounterSaved;
	}

	p_encoder->TimerCounterSaved = timerCounterValue;

	p_encoder->AngularD = p_encoder->TimerCounterSaved;
	p_encoder->TotalD += p_encoder->DeltaD;
	p_encoder->TotalT += 1;
}




//		HW Quadrature
// must use if counter ticks downs
// todo overflow both directions, flag in isr
static inline void Encoder_DeltaD_CaptureQuadrature_IO(Encoder_T * p_encoder)
{
	uint32_t counterValue = HAL_Encoder_ReadTimerCounter(p_encoder->p_HAL_Encoder);
	bool isIncrement;
	bool isCounterIncrementDirectionPositive;

	if (HAL_Encoder_ReadTimerCounterOverflow(p_encoder->p_HAL_Encoder))
	{
		if(HAL_Encoder_ReadQuadratureCounterOverflowIncrement(p_encoder->p_HAL_Encoder))
		{
			p_encoder->DeltaD = p_encoder->EncoderResolution - p_encoder->TimerCounterSaved + counterValue + 1U;
			isIncrement = true;
		}
		else if (HAL_Encoder_ReadQuadratureCounterOverflowDecrement(p_encoder->p_HAL_Encoder)) //counter counts down, deltaD is negative
		{
			p_encoder->DeltaD = p_encoder->EncoderResolution - counterValue + p_encoder->TimerCounterSaved + 1U;
			isIncrement = false;
		}

		HAL_Encoder_ClearTimerCounterOverflow(p_encoder->p_HAL_Encoder);
	}
	else
	{
		if (counterValue > p_encoder->TimerCounterSaved)
		{
			p_encoder->DeltaD = counterValue - p_encoder->TimerCounterSaved; //only this side is needed if using signed DeltaD, check sign for following
			isIncrement = true;
		}
		else //counter counts down, deltaD is negative
		{
			p_encoder->DeltaD = p_encoder->TimerCounterSaved - counterValue;
			isIncrement = false;
		}
	}

	p_encoder->TimerCounterSaved = counterValue;

#ifdef CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_INCREMENT
	isCounterIncrementDirectionPositive = p_encoder->IsALeadBDirectionPositive;
#elif defined(CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_DECREMENT)
	isCounterIncrementDirectionPositive = !p_encoder->IsALeadBDirectionPositive;
#endif

	if (isCounterIncrementDirectionPositive) //Positive DeltaD is positive direction
	{
		p_encoder->AngularD = counterValue;
//		p_encoder->TotalD += p_encoder->DeltaD;
		//direction = or use deltaD sign?
	}
	else
	{
		p_encoder->AngularD = p_encoder->EncoderResolution - counterValue;	//allow AngularD == EncoderResolution?
//		p_encoder->TotalD -= p_encoder->DeltaD;
	}

	p_encoder->TotalT += 1;
}


//static inline void Encoder_DeltaD_GetQuadratureDirectionInstant(Encoder_T * p_encoder)
//{
//	return HAL_Encoder_ReadQuadratureCounterDirection(p_encoder->p_HAL_Encoder);
//}
//
////get averaged/Captured direction
//static inline void Encoder_DeltaD_GetQuadratureDirectionAverage(Encoder_T * p_encoder)
//{
//
//}
//
//
//static inline void Encoder_DeltaD_GetQuadratureDirection(Encoder_T * p_encoder)
//{
//
//}

static inline void Encoder_DeltaD_Capture_IO(Encoder_T * p_encoder)
{
#ifdef CONFIG_ENCODER_HW_QUADRATURE_CAPABLE //hardware capture ticks up and down
	if(p_encoder->IsQuadratureCounterEnabled)
	{
		Encoder_DeltaD_CaptureQuadrature_IO(p_encoder);
	}
	else
#endif
	{
		Encoder_DeltaD_CaptureSinglePhase_IO(p_encoder);
	}
}



#endif
