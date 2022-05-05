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

#include "Peripheral/Pin/Pin.h"

#include <stdint.h>
#include <stdbool.h>

static inline void CaptureAngularDIncreasing(Encoder_T * p_encoder)
{
	p_encoder->AngularD = (p_encoder->AngularD < p_encoder->Params.CountsPerRevolution - 1U) ? p_encoder->AngularD + 1U : 0U;
}


/*!
	@brief Capture DeltaT, per fixed changed in distance, via pin edge interrupt

 	Call each hall cycle / electric rotation inside hall edge interrupt
	Interval cannot be greater than TimerCounterMax [ticks] i.e. (TimerCounterMax / TimerFreq) [seconds]
 */
static inline void Encoder_DeltaT_Capture(Encoder_T * p_encoder)
{
	uint32_t deltaT = 0U;
	_Encoder_CaptureDelta(p_encoder, &deltaT, CONFIG_ENCODER_HW_TIMER_COUNTER_MAX);
	p_encoder->DeltaT = (deltaT + p_encoder->DeltaT) / 2U;
	CaptureAngularDIncreasing(p_encoder);

	//capture integral
	p_encoder->TotalD += 1U;
	p_encoder->TotalT += p_encoder->DeltaT;
}



/*
 * SW Quadrature
 *
 * only when capturing single phase (of 2 phase quadrature mode) 180 degree offset, not for 3 phase 120 degree offset
 */
static inline void Encoder_DeltaT_CaptureQuadrature(Encoder_T * p_encoder)
{
//	bool phaseB = Pin_Input_Read(&p_encoder->PhaseB);
	//#ifdef enocder hal pin read
//	bool phaseB = HAL_Encoder_ReadPhaseB(&p_encoder->PhaseB);

//	_Encoder_CaptureDelta(p_encoder, &p_encoder->DeltaT, CONFIG_ENCODER_HW_TIMER_COUNTER_MAX);
//	p_encoder->TotalT += p_encoder->DeltaT;
//
//	if (!phaseB ^ p_encoder->Params.IsALeadBPositive)
//	{
//		//		p_encoder->DeltaD = 1; or set quadrature read direction
//		CaptureAngularDIncreasing(p_encoder);
//		if(p_encoder->TotalD < INT32_MAX) {p_encoder->TotalD += 1U;}
//	}
//	else
//	{
//		//		p_encoder->DeltaD = -1; or set quadrature read direction
//		p_encoder->AngularD = (p_encoder->AngularD > 0U) ? p_encoder->AngularD - 1U : p_encoder->Params.CountsPerRevolution - 1U;
//		if(p_encoder->TotalD > INT32_MIN) {p_encoder->TotalD -= 1;}
//	}
}

//static inline uint32_t Encoder_DeltaT_CaptureAngularSpeedAverage(Encoder_T * p_encoder)
//{
//	uint32_t speed = Encoder_CalcAngularSpeed(p_encoder, p_encoder->TotalD / 8 , p_encoder->TotalT / 8);
//	p_encoder->TotalD = 0U;
//	p_encoder->TotalT = 0U;
//	return speed;
//}
//
static inline uint32_t Encoder_DeltaT_PollRotationalSpeedAverage_RPM(Encoder_T * p_encoder)
{
//	uint32_t speed;
//

//	speed = Encoder_CalcRotationalSpeed_RPM(p_encoder, p_encoder->TotalD + 1, p_encoder->TotalT + p_encoder->DeltaT);

//
//	p_encoder->TotalD = 0U;
//	p_encoder->TotalT = 1U;
//	return speed;
}

/* rising edge detect */
static inline bool Encoder_DeltaT_PollReferenceEdgeRising(Encoder_T * p_encoder, bool reference)
{
	bool status = (reference == true) && (p_encoder->PulseReferenceSaved == false);
	if (status == true) {Encoder_DeltaT_Capture(p_encoder);}
	p_encoder->PulseReferenceSaved = reference;
	return status;
}

/* both edge detect*/
static inline bool Encoder_DeltaT_PollReferenceEdgeDual(Encoder_T * p_encoder, bool reference)
{
	bool status = (reference ^ p_encoder->PulseReferenceSaved);
	if (status == true)	{Encoder_DeltaT_Capture(p_encoder);}
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
	bool reference = Pin_Input_Read(&p_encoder->PhaseA);
	return Encoder_DeltaT_PollReferenceEdgeRising(p_encoder, reference);
}

static inline bool Encoder_DeltaT_PollPhaseAEdgeDual(Encoder_T * p_encoder)
{
	bool reference = Pin_Input_Read(&p_encoder->PhaseA);
	return Encoder_DeltaT_PollReferenceEdgeDual(p_encoder, reference);
}

/******************************************************************************/
/*!
 * @brief	Extend base timer using millis timer.
 * 			32-bit timer for 3.2us intervals is 3.8 hours
 */
/******************************************************************************/
static inline uint32_t GetEncoderExtendedTimerDelta(Encoder_T * p_encoder)
{
	uint32_t time = *(p_encoder->CONFIG.P_EXTENDED_TIMER);
	uint32_t deltaTime;

#ifdef CONFIG_ENCODER_EXTENDED_TIMER_CHECK_OVERFLOW
	/* Millis overflow 40+ days */
	if (time < p_encoder->ExtendedDeltaTimerSaved)
	{
		deltaTime = UINT32_MAX - p_encoder->ExtendedDeltaTimerSaved + time + 1U;
	}
	else
#endif
	{
		deltaTime = time - p_encoder->ExtendedTimerSaved;
	}

	return deltaTime;
}

/*
 * Short timer overflow time in long timer counts
 * e.g. TimerFreq = 625000, TimerCounterMax = 0xFFFF, ExtTimerFreq = 1000Hz => 104ms
 *
 * Should optimize to compile time const
 */
static inline uint32_t GetEncoderExtendedTimerConversion(Encoder_T * p_encoder)
{
	return ((uint32_t)CONFIG_ENCODER_HW_TIMER_COUNTER_MAX + 1UL) * p_encoder->CONFIG.EXTENDED_TIMER_FREQ / p_encoder->CONFIG.DELTA_T_TIMER_FREQ;
}


//extend 16bit timer cases to 32bit
//
//209ms to 13743ms for TimerFreq = 312500
//104ms to 6871ms for TimerFreq = 625000
static inline void Encoder_DeltaT_CaptureExtendedTimer(Encoder_T * p_encoder)
{
	uint32_t extendedTimerDelta = GetEncoderExtendedTimerDelta(p_encoder);
	p_encoder->ExtendedTimerSaved = *(p_encoder->CONFIG.P_EXTENDED_TIMER);

	if (extendedTimerDelta > GetEncoderExtendedTimerConversion(p_encoder)) //time exceed short timer max value
	{
		//should be caught by poll watch stop
//	if (extendedTimerDelta > (UINT32_MAX / p_encoder->CONFIG.DELTA_T_TIMER_FREQ) *  p_encoder->CONFIG.EXTENDED_TIMER_FREQ)
//	{
//		p_encoder->DeltaT = UINT32_MAX;
//	}
//	else
//	{
		p_encoder->DeltaT = extendedTimerDelta * (p_encoder->CONFIG.DELTA_T_TIMER_FREQ / p_encoder->CONFIG.EXTENDED_TIMER_FREQ);
//	}
	}
}

//Poll if capture has stoped
static inline bool Encoder_DeltaT_GetWatchStop(Encoder_T * p_encoder)
{
 	return (GetEncoderExtendedTimerDelta(p_encoder) > p_encoder->Params.ExtendedTimerDeltaTStop);
}

static inline bool Encoder_DeltaT_PollWatchStop(Encoder_T * p_encoder)
{
	bool isStop = Encoder_DeltaT_GetWatchStop(p_encoder);

	if (isStop == true)
	{
		p_encoder->DeltaT = UINT32_MAX;
		p_encoder->TimerCounterSaved = UINT32_MAX; /* Best chance at returning largest deltaT on next capture , TimerCounter < 32bits */
		p_encoder->SpeedSaved = 0;
	}

	return isStop;
}

// feed stop only without extended capture, cannot use in combination with capture
//static inline void Encoder_DeltaT_FeedWatchStop(Encoder_T * p_encoder)
//{
//	p_encoder->ExtendedTimerSaved = *(p_encoder->CONFIG.P_EXTENDED_TIMER);
//}

// smaller delta T overflow time
//static inline uint32_t Encoder_DeltaT_GetOverflowTime(Encoder_T * p_encoder)
//{
//	return (GetEncoderExtendedTimerDelta(p_encoder) - GetEncoderExtendedTimerConversion(p_encoder) );
//}

//static inline uint32_t Encoder_DeltaT_GetOverflowTime_Units2(Encoder_T * p_encoder)
//{
//	uint32_t deltaTime = Encoder_GetExtendedTimerDelta(p_encoder);
//
//	return (deltaTime - p_encoder->ExtendedDeltaTimerConversion) * p_encoder->UnitT_Freq / p_encoder->CONFIG.EXTENDED_DELTA_TIMER_FREQ;
//}


/******************************************************************************/
/*!
	Interpolation Functions
 */
/******************************************************************************/
/*!
	@brief 	  CaptureDeltaT mode

	Estimate angle each control period in between encoder pulse

	(AngleControlIndex / POLLING_FREQ) * (1(DeltaD) * UnitT_Freq / DeltaT) * (AngleSize / EncoderResolution)
	AngleControlIndex * 1(DeltaD) * AngleSize * UnitT_Freq / DeltaT / POLLING_FREQ / EncoderResolution

	AngleControlIndex ranges from 0 to ControlResolution [InterpolationFreq]

	UnitInterpolateAngle = UnitAngularSpeed / POLLING_FREQ
	UnitInterpolateAngle = [AngleSize[65536] * UnitDeltaT_Freq / POLLING_FREQ / CountsPerRevolution]
	return pollingIndex * Encoder_GetAngularSpeed(p_encoder) / POLLING_FREQ;
	return pollingIndex * [UnitAngularSpeed / POLLING_FREQ] / DeltaT;
 */
static inline uint32_t Encoder_DeltaT_InterpolateAngle(Encoder_T * p_encoder, uint32_t pollingIndex)
{
	return pollingIndex * p_encoder->UnitInterpolateAngle / p_encoder->DeltaT;
}

static inline uint32_t Encoder_DeltaT_InterpolateAngleIncIndex(Encoder_T * p_encoder, uint32_t * p_pollingIndex)
{
	uint32_t angle = Encoder_DeltaT_InterpolateAngle(p_encoder, *p_pollingIndex);
	(*p_pollingIndex)++;
	return angle;
}

/*
	Samples per DeltaT Capture
	CaptureDeltaT only (DeltaD ==1) (DeltaT Mode UnitT_Freq > POLLING_FREQ) -  cannot capture fractional DeltaD
	POLLING_FREQ/DeltaTCaptureFreq =  POLLING_FREQ / (UnitT_Freq / DeltaT);
 */
static inline uint32_t Encoder_DeltaT_GetInterpolationFreq(Encoder_T *p_encoder)
{
	return p_encoder->CONFIG.POLLING_FREQ * p_encoder->DeltaT / p_encoder->UnitT_Freq;
}

/******************************************************************************/
/*!
	Unit Conversions - Variable DeltaT (DeltaD is fixed, == 1). 
 */
/******************************************************************************/
//static inline uint32_t Encoder_ConvertToTime_Millis(Encoder_T * p_encoder, uint32_t deltaT_Ticks)		{return deltaT_Ticks * 1000U / p_encoder->UnitT_Freq;}
//static inline uint32_t Encoder_ConvertToTime_Seconds(Encoder_T * p_encoder, uint32_t deltaT_Ticks)	{return deltaT_Ticks / p_encoder->UnitT_Freq;}
//static inline uint32_t Encoder_ConvertToFreq(Encoder_T * p_encoder, uint32_t deltaT_Ticks)			{return (deltaT_Ticks == 0U) ? 0U : p_encoder->UnitT_Freq / deltaT_Ticks;}
//static inline uint32_t Encoder_ConvertToFreq_CPM(Encoder_T * p_encoder, uint32_t deltaT_Ticks)		{return (deltaT_Ticks == 0U) ? 0U : p_encoder->UnitT_Freq * 60U / deltaT_Ticks;}
////static inline uint32_t Encoder_ConvertFreqTo(Encoder_T * p_encoder, uint32_t deltaT_FreqHz)			{return (deltaT_FreqHz == 0U) ? 0U : p_encoder->UnitT_Freq / deltaT_FreqHz;}
//
///*!
//	@brief DeltaT period. unit in raw timer ticks.
// */
//static inline uint32_t Encoder_DeltaT_Get(Encoder_T * p_encoder)			{return p_encoder->DeltaT;}
//
///*!
//	@brief DeltaT period. unit in milliseconds
// */
//static inline uint32_t Encoder_DeltaT_Get_Millis(Encoder_T * p_encoder)	{return Encoder_ConvertToTime_Millis(p_encoder, p_encoder->DeltaT);}
//
///*!
//	@brief DeltaT period. unit in microseconds
// */
//static inline uint32_t Encoder_DeltaT_Get_Micros(Encoder_T * p_encoder)	{return _Encoder_MicrosHelper(p_encoder->DeltaT, p_encoder->UnitT_Freq);}
//
///*!
//	@brief DeltaT freq.	unit in Hz
// */
//static inline uint32_t Encoder_DeltaT_GetFreq(Encoder_T * p_encoder)		{return Encoder_ConvertToFreq(p_encoder, p_encoder->DeltaT);}
//
///*!
//	@brief DeltaT freq.	unit in cycles per minute
// */
//static inline uint32_t Encoder_DeltaT_GetFreq_CPM(Encoder_T * p_encoder)	{return Encoder_ConvertToFreq_CPM(p_encoder, p_encoder->DeltaT);}
 

/*!
	 Capture DeltaT Only -
 	CaptureDeltaT, DeltaD == 1: DeltaT timer ticks for given speed
	CaptureDeltaD, DeltaT == 1: <= 1 (Number of fixed DeltaT samples, before a deltaD increment)

	@param speed
	@param unitsPerSecond
	@return
 */


static inline uint32_t Encoder_DeltaT_ConvertToSpeed(Encoder_T * p_encoder, uint32_t deltaT_ticks)
{
	return Encoder_DeltaT_ConvertFromSpeed(p_encoder, deltaT_ticks); /* Same division */
}

static inline uint32_t Encoder_DeltaT_ConvertToSpeed_UnitsPerMinute(Encoder_T * p_encoder, uint32_t deltaT_Ticks)
{
	return Encoder_DeltaT_ConvertFromSpeed_UnitsPerMinute(p_encoder, deltaT_Ticks); /* Same division */
}

static inline uint32_t Encoder_DeltaT_ConvertFromSpeed(Encoder_T * p_encoder, uint32_t speed_UnitsPerSecond)
{
	return (speed_UnitsPerSecond == 0U) ? 0U : p_encoder->UnitLinearSpeed / speed_UnitsPerSecond;
}

static inline uint32_t Encoder_DeltaT_ConvertFromSpeed_UnitsPerMinute(Encoder_T * p_encoder, uint32_t speed_UnitsPerMinute)
{
	return (speed_UnitsPerMinute == 0U) ? 0U : p_encoder->UnitLinearSpeed * 60U / speed_UnitsPerMinute;
}

static inline uint32_t Encoder_DeltaT_GetSpeed(Encoder_T * p_encoder)
{
	// return Encoder_CalcSpeed(p_encoder, 1U, p_encoder->DeltaT);
	return Encoder_DeltaT_ConvertToSpeed(p_encoder, p_encoder->DeltaT);
}

/*!
	@return DeltaT
	CaptureDeltaT, DeltaD == 1: DeltaT timer ticks for given speed
	CaptureDeltaD, DeltaT == 1: <= 1 (Number of fixed DeltaT samples, before a deltaD increment)
 */
static inline uint32_t Encoder_DeltaT_ConvertFromRotationalSpeed_RPM(Encoder_T * p_encoder, uint32_t rpm)
{
	return  p_encoder->UnitT_Freq * 60U / (p_encoder->Params.CountsPerRevolution * rpm);
}

static inline uint32_t Encoder_DeltaT_ConvertToRotationalSpeed_RPM(Encoder_T * p_encoder, uint32_t deltaT_ticks)
{
	return p_encoder->UnitT_Freq * 60U / (p_encoder->Params.CountsPerRevolution * deltaT_ticks);
}


/******************************************************************************/
/*!
	@brief 	Angular Interpolation Estimated Functions
 */
/*! @{ */
/******************************************************************************/
/*
	Polls per encoder count, when polls per encoder count > 1
	only when pollingFreq > Encoder Counts Freq, i.e. 0 encoder counts per poll

	DeltaD Mode, applicable to low speeds
	8192 CPR Encoder, 20Khz Polling Freq => 146 RPM
	encoder count per Poll > 1, use Encoder_ConvertRotationalSpeedToDeltaAngle_RPM
 */
static inline uint32_t Encoder_DeltaT_ConvertRotationalSpeedToInterpolationFreq_RPM(Encoder_T * p_encoder, uint16_t mechRpm)
{
	return p_encoder->CONFIG.POLLING_FREQ * 60U / (p_encoder->Params.CountsPerRevolution * mechRpm);
}

static inline uint32_t Encoder_DeltaT_ConvertInterpolationFreqToRotationalSpeed_RPM(Encoder_T * p_encoder, uint16_t controlPeriods)
{
	return p_encoder->CONFIG.POLLING_FREQ * 60U / (p_encoder->Params.CountsPerRevolution * controlPeriods);
}
/******************************************************************************/
/*! @} */
/******************************************************************************/

/******************************************************************************/
/*!
	@brief  LinearInterpolation Estimated Functions
 */
/*! @{ */
/******************************************************************************/
///*!
//	@brief CaptureDeltaD Mode: Estimate D using captured DeltaD sample. Assuming constant speed.
//	@param domain unitless
// */
//static inline uint32_t Encoder_InterpolateDistance_Slope(Encoder_T * p_encoder, uint32_t index, uint32_t domain)
//{
//	return index * Encoder_GetDeltaD_Units(p_encoder) / domain;
//}
//
///*!
//	@brief CaptureDeltaT Mode: Estimate DeltaD using captured DeltaT sample. Assuming constant speed.
//
//	Delta Peroid > Control Peroid
//	time domain - if domain is with respect to time =>
//	@param index range [0:interpolationFreq/DeltaTFreq]. interpolationFreq/DeltaT_Freq == [interpolationFreq/(UnitT_Freq/DeltaT)] == [interpolationFreq * DeltaT / UnitT_Freq]
// */
//static inline uint32_t Encoder_InterpolateDistance(Encoder_T * p_encoder, uint32_t index)
//{
////	return index * p_encoder->UnitInterpolateD / p_encoder->DeltaT; /* index * [UnitD * UnitT_Freq / interpolationFreq] / DeltaT */
//	return index * Encoder_GetSpeed(p_encoder) / p_encoder->CONFIG.POLLING_FREQ; 	//index * 1 * [UnitD * UnitT_Freq] / p_encoder->DeltaT / interpolationFreq;
//}

/*!
	Interpolate Delta Angle
 */
//static inline uint32_t Encoder_InterpolateAngle_IndexDomain(Encoder_T * p_encoder, uint32_t index, uint32_t domain)
//{
//	return index * Encoder_GetDeltaAngle(p_encoder) / domain;
//}

//static inline uint32_t Encoder_InterpolateSpeed_Slope(Encoder_T * p_encoder, uint32_t index, uint32_t domain)
//{
//	return index * Encoder_GetSpeed(p_encoder) / domain;
//}

//static inline uint32_t Encoder_InterpolateSpeed(Encoder_T * p_encoder, uint32_t index, uint32_t interpolationFreq)
//{
////	return index * Encoder_GetSpeed(p_encoder) * p_encoder->UnitInterpolatedD / p_encoder->DeltaT;
//
////	index * Encoder_GetAcceleration(p_encoder) / interpolationFreq;
//}
/******************************************************************************/
/*! @} */
/******************************************************************************/

#endif
