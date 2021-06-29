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
	@file  	Encoder_DeltaT.h
	@author FireSourcery
	@brief 	Capture Delta T Mode, displacement is fixed
	@version V0
 */
/******************************************************************************/
#ifndef ENCODER_DELTA_T_H
#define ENCODER_DELTA_T_H

#include "Encoder.h"
#include "HAL_Encoder.h"
#include "Private.h"

#include <stdint.h>
#include <stdbool.h>

static inline void CaptureEncoderDeltaIncreasing(Encoder_T * p_encoder, volatile uint32_t * p_delta)
{
	uint32_t timerCounterValue = HAL_Encoder_ReadTimerCounter(p_encoder->p_HAL_Encoder);

	if (timerCounterValue < p_encoder->TimerCounterSaved) /* TimerCounter overflow */
	{
		*p_delta = p_encoder->TimerCounterMax - p_encoder->TimerCounterSaved + timerCounterValue + 1U;
	}
	else /* normal case */
	{
		*p_delta = timerCounterValue - p_encoder->TimerCounterSaved;
	}

	p_encoder->TimerCounterSaved = timerCounterValue;
}


//static inline void CaptureSumDIncreasing(Encoder_T * p_encoder)
//{
//	if (p_encoder->AngularD < p_encoder->EncoderResolution - 1U)
//	{
//		p_encoder->AngularD++;
//	}
//	else
//	{
//		p_encoder->AngularD = 0U;
//	}
//
//	p_encoder->TotalD += 1U;
//}


/*!
	@brief Capture DeltaT, per fixed changed in distance, via pin edge interrupt

 	Call each hall cycle / electric rotation inside hall edge interrupt
	Interval cannot be greater than TimerCounterMax [ticks] i.e. (TimerCounterMax / TimerFreq) [seconds]
 */
static inline void Encoder_DeltaT_Capture_IO(Encoder_T * p_encoder)
{
	CaptureEncoderDeltaIncreasing(p_encoder,  &p_encoder->DeltaT);

	if (p_encoder->AngularD < p_encoder->EncoderResolution - 1U)
	{
		p_encoder->AngularD++;
	}
	else
	{
		p_encoder->AngularD = 0U;
	}

	//capture integral
//	p_encoder->TotalD += 1U;
//	p_encoder->TotalT += p_encoder->DeltaT;
}

//static inline void Encoder_DeltaT_CaptureDeltaT_IO(Encoder_T * p_encoder)
//{
//	CaptureEncoderDeltaIncreasing(p_encoder,  &p_encoder->DeltaT);
//}

//
//static inline void Encoder_DeltaT_CaptureTotal_IO(Encoder_T * p_encoder)
//{
// 	//capture integral
//	p_encoder->TotalT += p_encoder->DeltaT;
//	p_encoder->TotalD += 1U;
//}
//
//static inline void Encoder_DeltaT_CaptureAll_IO(Encoder_T * p_encoder)
//{
//	Encoder_DeltaT_Capture_IO(p_encoder);
//	Encoder_DeltaT_CaptureTotal_IO(p_encoder);
//
//	//capture user
////	p_encoder->UserT += p_encoder->DeltaT;
////	p_encoder->UserD += 1U;
//}

/*
 * SW Quadrature
 *
 * only when capturing single phase (of 2 phase quadrature mode), not for 3 phase hall commutation capture
 */
static inline void Encoder_DeltaT_CaptureQuadrature_IO(Encoder_T * p_encoder)
{
	bool phaseB = HAL_Encoder_ReadPhaseB(p_encoder->p_HAL_Encoder);
	CaptureEncoderDeltaIncreasing(p_encoder, &p_encoder->DeltaT);
	p_encoder->TotalT += p_encoder->DeltaT;

	if (!phaseB ^ p_encoder->IsALeadBDirectionPositive)
//	if (HAL_Encoder_ReadPhaseB(p_encoder->p_HAL_Encoder) == false)
	{
		//		p_encoder->DeltaD = 1; or set quadrature read direction

		if (p_encoder->AngularD < p_encoder->EncoderResolution - 1U)
		{
			p_encoder->AngularD++;
		}
		else
		{
			p_encoder->AngularD = 0U;
		}

//		if(p_encoder->TotalD < INT32_MAX)
//		{
//			p_encoder->TotalD += 1U;
//		}
	}
	else
	{
		  //		p_encoder->DeltaD = -1; or set quadrature read direction

		if(p_encoder->AngularD > 0U)
		{
			p_encoder->AngularD--;
		}
		else //==0
		{
			p_encoder->AngularD = p_encoder->EncoderResolution - 1U;
		}

//		if(p_encoder->TotalD > INT32_MIN)
//		{
//			p_encoder->TotalD -= 1;
//		}
	}
}

static inline bool Encoder_DeltaT_PollReferenceEdgeRising(Encoder_T * p_encoder, bool reference)
{
	bool status;

	/* rising edge detect *//* if (reference - p_encoder->ReferenceSignalSaved > 0) */
	if ((reference == true) && (p_encoder->PulseReferenceSaved == false))
	{
		Encoder_DeltaT_Capture_IO(p_encoder);
		status = true;
	}
	else
	{
		status = false;
	}

	p_encoder->PulseReferenceSaved = reference;

	return status;
}

static inline bool Encoder_DeltaT_PollReferenceEdgeDual(Encoder_T * p_encoder, bool reference)
{
	bool status;

	/* both edge detect*/
	if ((reference ^ p_encoder->PulseReferenceSaved) == true)
	{
		Encoder_CaptureDeltaT_IO(p_encoder);
		status = true;
	}
	else
	{
		status = false;
	}

	p_encoder->PulseReferenceSaved = reference;

	return status;
}

/*!
	@brief Capture DeltaT, via polling from main loop or control isr, when pin interrupt is not available.

	e.g. use 1 hall edge for reference
	polling frequency must be > signal freq, at least 2x to satisfy Nyquist theorem
	2000ppr encoder, 20000hz sample => 300rpm max
 */
static inline bool Encoder_DeltaT_PollPhaseAEdgeRising(Encoder_T * p_encoder)
{
	bool reference = HAL_Encoder_ReadPhaseA(p_encoder->p_HAL_Encoder);
	return Encoder_DeltaT_PollReferenceEdgeRising(p_encoder, reference);
}

static inline void Encoder_DeltaT_PollPhaseAEdgeDual(Encoder_T * p_encoder)
{
	bool reference = HAL_Encoder_ReadPhaseA(p_encoder->p_HAL_Encoder);
	return Encoder_DeltaT_PollReferenceEdgeDual(p_encoder, reference);
}

/******************************************************************************/
/*!
 * @brief	Extend base timer using millis timer.
 * 			32-bit timer for 3.2us intervals is 3.8 hours
 */
/******************************************************************************/
static inline uint32_t GetExtendedTimerDelta(Encoder_T * p_encoder)
{
//	CaptureEncoderDeltaIncreasing(p_encoder, &capturedest, &p_encoder->p_ExtendedDeltaTimer, p_encoder->p_ExtendedDeltaTimerMax, p_encoder->ExtendedDeltaTimerSaved)
	uint32_t time = *(p_encoder->p_ExtendedDeltaTimer);
	uint32_t deltaTime;

	//program must run for 40+ days is 32bit timer is millis to overflow
	if (time < p_encoder->ExtendedDeltaTimerSaved)
	{
		deltaTime = UINT32_MAX - p_encoder->ExtendedDeltaTimerSaved + time + 1U;
	}
	else
	{
		deltaTime = time - p_encoder->ExtendedDeltaTimerSaved;
	}

	return deltaTime;
}

//static inline uint32_t Encoder_DeltaT_CaptureExtendedTimer(Encoder_T * p_encoder)
//static inline uint32_t CaptureExtendedTimerDelta(Encoder_T * p_encoder)
//{
//	uint32_t deltaTime = GetEncoderExtendedTimerDelta(p_encoder);
//
////	p_encoder->ExtendedTimerDelta = deltaTime;
//
//	p_encoder->ExtendedDeltaTimerSaved = time;
//
//	return deltaTime;
//}

//extend 16bit timer cases to 32bit
//
//209ms to 13743ms for TimerFreq = 312500
//104ms to 6871ms for TimerFreq = 625000
static inline void Encoder_DeltaT_CaptureExtension_IO(Encoder_T * p_encoder)
{
//	uint32_t extendedTimerDelta = CaptureExtendedTimerDelta(p_encoder);

	uint32_t extendedTimerDelta = GetExtendedTimerDelta(p_encoder);
	p_encoder->ExtendedDeltaTimerSaved = *(p_encoder->p_ExtendedDeltaTimer);

	if (extendedTimerDelta > p_encoder->ExtendedDeltaTimerThreshold) //time exceed short timer max value
	{
//		if (extendedTimerDelta > (UINT32_MAX) / p_encoder->UnitT_Freq)
//		{
//			// if (extendedTimerDelta * UnitT_Freq) / ExtendedDeltaTimerFreq > UINT32_MAX;
//			//ExtendedDeltaTimerFreq always < UnitT_Freq
//			if (extendedTimerDelta > (UINT32_MAX / p_encoder->UnitT_Freq) *  p_encoder->ExtendedDeltaTimerFreq)
//			{
//				p_encoder->DeltaT = UINT32_MAX;
//			}
//			else
//			{
				p_encoder->DeltaT = extendedTimerDelta * (p_encoder->UnitT_Freq / p_encoder->ExtendedDeltaTimerFreq);
//			}
//		}
//		else
//		{
//			p_encoder->DeltaT = (extendedTimerDelta * p_encoder->UnitT_Freq) / p_encoder->ExtendedDeltaTimerFreq;
//			extendedTimerDelta = extendedTimerDelta - (extendedTimerDelta % p_encoder->ExtendedDeltaTimerThreshold)*;
//			p_encoder->DeltaT += (extendedTimerDelta * p_encoder->UnitT_Freq) / p_encoder->ExtendedDeltaTimerFreq;
//		}
	}
}

// feed stop without extended capture
static inline void Encoder_DeltaT_FeedWatchStop(Encoder_T * p_encoder)
{
	p_encoder->ExtendedDeltaTimerSaved = *(p_encoder->p_ExtendedDeltaTimer);
}

//Poll if capture has stoped
static inline bool Encoder_DeltaT_GetWatchStop(Encoder_T * p_encoder)
{
	uint32_t deltaTime = GetExtendedTimerDelta(p_encoder);
	bool isStop;

	if (deltaTime > p_encoder->ExtendedDeltaTimerEffectiveStopTime)
	{
		isStop = true;
	}
	else
	{
		isStop = false;
	}

	return isStop;
}

static inline bool Encoder_DeltaT_PollWatchStop(Encoder_T * p_encoder)
{
	bool isStop;

	if (Encoder_DeltaT_GetWatchStop(p_encoder))
	{
		isStop = true;
		p_encoder->DeltaT = UINT32_MAX;
		p_encoder->SpeedSaved = 0;
	}
	else
	{
		isStop = false;
	}

	return isStop;
}


//is smaller delta T overflow
static inline uint32_t Encoder_DeltaT_GetOverTime(Encoder_T * p_encoder)
{
	uint32_t deltaTime = GetEncoderExtendedTimerDelta(p_encoder);

	return (deltaTime - p_encoder->ExtendedDeltaTimerThreshold);
}


//static inline uint32_t Encoder_GetDeltaTOverflow(Encoder_T * p_encoder)
//{
//	uint32_t deltaTime = Encoder_GetExtendedTimerDelta(p_encoder);
//
//	return (deltaTime - p_encoder->ExtendedDeltaTimerConversion) * p_encoder->UnitT_Freq / p_encoder->ExtendedDeltaTimerFreq;
//}

#endif
