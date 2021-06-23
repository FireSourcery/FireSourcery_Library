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

	capture increasing
 */
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

/*!
	@brief Capture DeltaT, per fixed changed in distance, via pin edge interrupt

 	Call each hall cycle / electric rotation inside hall edge interrupt
	Interval cannot be greater than TimerCounterMax [ticks] i.e. (TimerCounterMax / TimerFreq) [seconds]
 */
static inline void Encoder_CaptureDeltaT_IO(Encoder_T * p_encoder)
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
	p_encoder->TotalT += p_encoder->DeltaT;
	p_encoder->TotalD += 1U;
}

//static inline void Encoder_DeltaT_CaptureTotal_IO(Encoder_T * p_encoder)
//{
// 	//capture integral
//	p_encoder->TotalT += p_encoder->DeltaT;
//	p_encoder->TotalD += 1U;
//
//}
//
//static inline void Encoder_DeltaT_CaptureAll_IO(Encoder_T * p_encoder)
//{
//	Encoder_CaptureDeltaT_IO(p_encoder);
//	Encoder_DeltaT_CaptureTotal_IO(p_encoder);
//
//	//capture user
////	p_encoder->UserT += p_encoder->DeltaT;
////	p_encoder->UserD += 1U;
//}

//static inline void Encoder_PollDeltaTReference_IO(Encoder_T * p_encoder, bool ref)
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
	@brief Capture DeltaT, via polling from main loop or control isr, when pin interrupt is not available.

	e.g. use 1 hall edge for reference
	polling frequency must be > signal freq, at least 2x to satisfy Nyquist theorem
	2000ppr encoder, 20000hz sample => 300rpm max
 */
static inline bool Encoder_PollDeltaTRisingEdge_IO(Encoder_T * p_encoder)
{
	bool reference = HAL_Encoder_ReadPhaseA(p_encoder->p_HAL_Encoder);
	bool status;

	/* rising edge detect */
	/* if (reference - p_encoder->ReferenceSignalSaved > 0) */
	if ((reference == true) && (p_encoder->PulseReferenceSaved == false))
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

static inline void Encoder_PollDeltaTDualEdge_IO(Encoder_T * p_encoder)
{
	bool reference = HAL_Encoder_ReadPhaseA(p_encoder->p_HAL_Encoder);
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

/*
 * SW Quadrature
 */
static inline void Encoder_CaptureDeltaT_Quadrature_IO(Encoder_T * p_encoder)
{
	CaptureEncoderDeltaIncreasing(p_encoder, &p_encoder->DeltaT);

	 //only when capturing single phase, not for 3 phase hall commutation capture
	if (HAL_Encoder_ReadPhaseB(p_encoder->p_HAL_Encoder) == false) //^positive calibration
	{
		if (p_encoder->AngularD < p_encoder->EncoderResolution - 1U)
		{
			p_encoder->AngularD++;
		}
		else
		{
			p_encoder->AngularD = 0U;
		}

		p_encoder->TotalD += 1U;
	}
	else
	{
		if(p_encoder->AngularD > 0U)
		{
			p_encoder->AngularD--;
		}
		else //==0
		{
			p_encoder->AngularD = p_encoder->EncoderResolution - 1U;
		}

		p_encoder->TotalD -= 1;

		// 	p_encoder->DeltaD = 0 - p_encoder->DeltaD;
	}

	p_encoder->TotalT += p_encoder->DeltaT;
}




/******************************************************************************/
/*!
 * @brief	Extend base timer using millis timer.
 * 			32-bit timer for 3.2us intervals is 3.8 hours
 */
/******************************************************************************/
static inline uint32_t CaptureEncoderExtendedDeltaT(Encoder_T * p_encoder)
{
	uint32_t time = *(p_encoder->p_ExtendedDeltaTimer);
	uint32_t deltaTime;

	if (time < p_encoder->ExtendedDeltaTimerSaved)
	{
		deltaTime = UINT32_MAX - p_encoder->ExtendedDeltaTimerSaved + time;
	}
	else
	{
		deltaTime = time - p_encoder->ExtendedDeltaTimerSaved;
	}

	p_encoder->ExtendedDeltaTimerSaved = time;

	return deltaTime;
}

static inline bool Encoder_IsDeltaTOverflow(Encoder_T * p_encoder)
{
	bool isDeltaTOverflow;
	uint32_t time = *(p_encoder->p_ExtendedDeltaTimer);
	uint32_t deltaTime;

	if (time < p_encoder->ExtendedDeltaTimerSaved)
	{
		deltaTime = UINT32_MAX - p_encoder->ExtendedDeltaTimerSaved + time;
	}
	else
	{
		deltaTime = time - p_encoder->ExtendedDeltaTimerSaved;
	}

	if (deltaTime > p_encoder->ExtendedDeltaTimerConversion)
	{
		isDeltaTOverflow = true;
	}
	else
	{
		isDeltaTOverflow = false;
	}

	return isDeltaTOverflow;
}

// use this after captureDeltaT or extended capture
static inline void Encoder_FeedDeltaTStop_IO(Encoder_T * p_encoder)
{
	p_encoder->ExtendedDeltaTimerSaved = *(p_encoder->p_ExtendedDeltaTimer);
}

//extend using 16bit ot 32 bit only
static inline void Encoder_CaptureExtendedDeltaT_IO(Encoder_T * p_encoder)
{
	uint32_t time = *(p_encoder->p_ExtendedDeltaTimer);
	uint32_t deltaTime;

//	Encoder_CaptureDeltaT_IO(p_encoder);
//	Encoder_FeedDeltaTStop_IO(p_encoder);

	//program must run for 40+ days is 32bit timer is millis to overflow
	if (time < p_encoder->ExtendedDeltaTimerSaved)
	{
		deltaTime = UINT32_MAX - p_encoder->ExtendedDeltaTimerSaved + time;
	}
	else
	{
		deltaTime = time - p_encoder->ExtendedDeltaTimerSaved;
	}

	p_encoder->ExtendedDeltaTimerSaved = time;

	if (deltaTime > p_encoder->ExtendedDeltaTimerConversion) //time exceed short timer max value
	{
		// change in millis must be < 13,743 for TimerFreq = 312,500

		if (deltaTime > (UINT32_MAX - p_encoder->DeltaT) / p_encoder->UnitT_Freq * p_encoder->ExtendedDeltaTimerFreq)
		{
			if (deltaTime > (UINT32_MAX - p_encoder->DeltaT) / p_encoder->UnitT_Freq)
			{
				p_encoder->DeltaT += deltaTime * (p_encoder->UnitT_Freq / p_encoder->ExtendedDeltaTimerFreq);
			}
			else
			{
				p_encoder->DeltaT = UINT32_MAX;
			}
		}
		else
		{
			p_encoder->DeltaT += (deltaTime * p_encoder->UnitT_Freq) / p_encoder->ExtendedDeltaTimerFreq;
		}
	}
}

static inline bool Encoder_PollDeltaTStop_IO(Encoder_T * p_encoder)
{
	uint32_t time = *(p_encoder->p_ExtendedDeltaTimer);
	uint32_t deltaTime;
	bool isStop;

	if (time < p_encoder->ExtendedDeltaTimerSaved)
	{
		deltaTime = UINT32_MAX - p_encoder->ExtendedDeltaTimerSaved + time;
	}
	else
	{
		deltaTime = time - p_encoder->ExtendedDeltaTimerSaved;
	}

	if (deltaTime > p_encoder->ExtendedDeltaTimerEffectiveStopTime)
	{
		p_encoder->DeltaT = UINT32_MAX;
		p_encoder->SpeedSaved = 0;
		isStop = true;
	}
	else
	{
		isStop = false;
	}

	return isStop;
}



/*!
	@brief 	Capture DeltaD, per fixed changed in time, via timer periodic interrupt
			Looping Angle Capture

 	 	 	TimerCounter should loop for correct angular position
 	 	 	TimerCounterMax == EncoderRes

 */
static inline void Encoder_CaptureDeltaD_IO(Encoder_T * p_encoder)
{
	CaptureEncoderDeltaIncreasing(p_encoder,  &p_encoder->DeltaD);

	p_encoder->AngularD = p_encoder->TimerCounterSaved;
	p_encoder->TotalD += p_encoder->DeltaD;
	p_encoder->TotalT += 1;
}

//		HW Quadrature
// must use if counter ticks downs
static inline void Encoder_CaptureDeltaD_Quadrature_IO(Encoder_T * p_encoder)
{
 	uint32_t counterValue = HAL_Encoder_ReadTimerCounter(p_encoder->p_HAL_Encoder);

	if (HAL_Encoder_ReadDirection(p_encoder->p_HAL_Encoder)) //^quadrature mode
	{
		CaptureEncoderDeltaIncreasing(p_encoder, &p_encoder->DeltaD);
		p_encoder->TotalD += p_encoder->DeltaD;
	}
	else
	{
		//counter counts down, deltaD is negative
		if (counterValue > p_encoder->TimerCounterSaved)  /* TimerCounter underflow */
		{
			p_encoder->DeltaD = p_encoder->TimerCounterMax - counterValue + p_encoder->TimerCounterSaved + 1U;
		}
		else /* normal case */
		{
			p_encoder->DeltaD = p_encoder->TimerCounterSaved - counterValue;
		}

		p_encoder->TotalD -= p_encoder->DeltaD;
	}

	p_encoder->TimerCounterSaved = counterValue;
	p_encoder->AngularD = counterValue;
	p_encoder->TotalT += 1;
}

//full revolution counting deltaD
static inline void Encoder_ProcCounterDOverflow_IO(Encoder_T * p_encoder)
{
	HAL_Encoder_ClearTimerCounterOverflow(p_encoder->p_HAL_Encoder);
}

static inline void Encoder_ProcIndex_IO(Encoder_T * p_encoder)
{
	HAL_Encoder_WriteTimerCounter(p_encoder->p_HAL_Encoder, 0U);
	p_encoder->AngularD = 0U;
}

/******************************************************************************/
/*! @} */
/******************************************************************************/

#endif
