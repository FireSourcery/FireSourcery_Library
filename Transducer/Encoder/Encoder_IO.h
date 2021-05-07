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
	@file  	Encoder_IO.h
	@author FireSourcery
	@brief 	Encoder module functions must be placed into corresponding user app threads
	@version V0
 */
/******************************************************************************/
#ifndef ENCODER_IO_H
#define ENCODER_IO_H

#include "Encoder.h"
#include "HAL.h"

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*!
	@addtogroup	CaptureDelta
	@brief	Capture DeltaT or DeltaD between 2 samples.
			Either DeltaT or DeltaD is selected to be fixed.
			Used for all speed calculations.

			Filtering Delta is responsibility of caller.
 */
/*! @{ */
/******************************************************************************/

/*!
	@brief Private CaptureDelta Helper
 */
//static inline void CaptureDelta(Encoder_T * p_encoder, volatile uint32_t * p_delta)
//{
//	uint32_t timerCounterValue = HAL_Encoder_ReadTimerCounter(p_encoder->p_HAL_Encoder);
//
//	if (timerCounterValue < p_encoder->TimerCounterSaved) /* TimerCounter overflow */
//	{
//		*p_delta = p_encoder->TimerCounterMax - p_encoder->TimerCounterSaved + timerCounterValue;
//	}
//	else /* normal case */
//	{
//		*p_delta = timerCounterValue - p_encoder->TimerCounterSaved;
//	}
//
//	p_encoder->TimerCounterSaved = timerCounterValue;
//}

/*!
	@brief Capture DeltaT, per fixed changed in distance, via pin edge interrupt

 	Call each hall cycle / electric rotation inside hall edge interrupt
	Interval cannot be greater than TimerCounterMax [ticks] i.e. (TimerCounterMax / TimerFreq) [seconds]
 */
static inline void Encoder_CaptureDeltaT_IO(Encoder_T * p_encoder)
{
//	CaptureDelta(p_encoder,  &p_encoder->DeltaT);
	uint32_t timerCounterValue = HAL_Encoder_ReadTimerCounter(p_encoder->p_HAL_Encoder);

	if (timerCounterValue < p_encoder->TimerCounterSaved) /* TimerCounter overflow */
	{
		p_encoder->DeltaT = p_encoder->TimerCounterMax - p_encoder->TimerCounterSaved + timerCounterValue;
	}
	else /* normal case */
	{
		p_encoder->DeltaT = timerCounterValue - p_encoder->TimerCounterSaved;
	}

	p_encoder->TimerCounterSaved = timerCounterValue;

	p_encoder->TotalT += p_encoder->DeltaT;

	if (!HAL_Encoder_ReadPhaseB(p_encoder->p_HAL_Encoder)) //only when capturing rising edge A only
	{
		p_encoder->TotalD += 1U;

		if (p_encoder->AngularD < p_encoder->EncoderResolution - 1U)
		{
			p_encoder->AngularD++;
		}
		else
		{
			p_encoder->AngularD = 0U;
		}
	}
	else
	{
		p_encoder->TotalD -= 1;

		if(p_encoder->AngularD > 0U)
		{
			p_encoder->AngularD--;
		}
		else //==0
		{
			p_encoder->AngularD = p_encoder->EncoderResolution - 1U;
		}
	}
}

/*!
	@brief Capture DeltaT, via polling from main loop or control isr, when pin interrupt is not available.

	e.g. use 1 hall edge for reference
	polling frequency must be > signal freq, at least 2x to satisfy Nyquist theorem
	2000ppr encoder, 20000hz sample => 300rpm max
 */
static inline void Encoder_PollDeltaT_IO(Encoder_T * p_encoder)
{
	bool reference = HAL_Encoder_ReadPhaseA(p_encoder->p_HAL_Encoder);

	/* rising edge detect */
	/* if (reference - p_encoder->ReferenceSignalSaved > 0) */
	if ((reference == true) && (p_encoder->PulseReferenceSaved == false))
	{
		Encoder_CaptureDeltaT_IO(p_encoder);
	}

	/* both edge detect*/
//	if ((reference ^ p_encoder->PulseReferenceSaved) == true)
//	{
//		Encoder_CaptureDeltaT_IO(p_encoder);
//	}

	p_encoder->PulseReferenceSaved = reference;
}


//static inline void Encoder_PollDeltaT_Ref_IO(Encoder_T * p_encoder, bool ref)
//{
//	bool reference = ref;
//
//	/* rising edge detect */
//	/* if (reference - p_encoder->ReferenceSignalSaved > 0) */
//	if ((reference == true) && (p_encoder->PulseReferenceSaved == false))
//	{
//		Encoder_CaptureDeltaT_IO(p_encoder);
//	}
//
//	/* both edge detect*/
////	if ((reference ^ p_encoder->PulseReferenceSaved) == true)
////	{
////		Encoder_CaptureDeltaT_IO(p_encoder);
////	}
//
//	p_encoder->PulseReferenceSaved = reference;
//}

/*!
	@brief 	Capture DeltaD, per fixed changed in time, via timer periodic interrupt
			Looping Angle Capture

 	 	 	TimerCounter should loop for correct angular position
 	 	 	TimerCounterMax == EncoderRes
 */
static inline void Encoder_CaptureDeltaD_IO(Encoder_T * p_encoder)
{
//	CaptureDelta(p_encoder,  &p_encoder->DeltaD);
	uint32_t timerCounterValue = HAL_Encoder_ReadTimerCounter(p_encoder->p_HAL_Encoder);

	if (HAL_Encoder_ReadDirection(p_encoder->p_HAL_Encoder))
	{
		if (timerCounterValue < p_encoder->TimerCounterSaved) /* TimerCounter overflow */
		{
			p_encoder->DeltaD = p_encoder->TimerCounterMax - p_encoder->TimerCounterSaved + timerCounterValue;
		}
		else /* normal case */
		{
			p_encoder->DeltaD = timerCounterValue - p_encoder->TimerCounterSaved;
		}

		p_encoder->TotalD += p_encoder->DeltaD;
	}
	else
	{
		//set direction
		if (timerCounterValue < p_encoder->TimerCounterSaved)
		{
			p_encoder->DeltaD = p_encoder->TimerCounterSaved - timerCounterValue;
		}
		else
		{
			p_encoder->DeltaD = timerCounterValue - p_encoder->TimerCounterSaved;
		}

		p_encoder->TotalD -= p_encoder->DeltaD;
	}

	p_encoder->TimerCounterSaved = timerCounterValue;
	p_encoder->TotalT += 1;
//	p_encoder->AngularD = HAL_Encoder_ReadTimerCounter(p_encoder->p_HAL_Encoder);
	p_encoder->AngularD = timerCounterValue;
}

/******************************************************************************/
/*! @} */
/******************************************************************************/

#endif
