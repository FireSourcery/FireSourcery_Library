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
	@file  	Encoder_DeltaT.h
	@author FireSourcery
	@version V0
	@brief 	Capture Delta T Mode, displacement is fixed
*/
/******************************************************************************/
#ifndef ENCODER_DELTA_T_H
#define ENCODER_DELTA_T_H

#include "Encoder.h"

/******************************************************************************/
/*!
	@brief 	Capture Functions - Poll Pulse, ISR
 			Capture DeltaT, per fixed changed in distance, via pin edge interrupt
*/
/******************************************************************************/

/*
	Separate Capture Angle at High Freq
*/
static inline void Encoder_DeltaT_CaptureD(Encoder_T * p_encoder)
{
	_Encoder_CaptureAngularDIncreasing(p_encoder);
	p_encoder->TotalD += 1U; /* Capture integral */
}

/*
	Separate Capture Speed at Low Freq
*/
static inline void Encoder_DeltaT_CaptureT(Encoder_T * p_encoder)
{
// // #if defined(CONFIG_ENCODER_HW_CAPTURE_TIME)
// 	if(HAL_Encoder_ReadTimerCounterOverflow(p_encoder->CONFIG.P_HAL_ENCODER) == true)
// 	{
// 		HAL_Encoder_ClearTimerCounterOverflow(p_encoder->CONFIG.P_HAL_ENCODER);
// 		p_encoder->DeltaT = CONFIG_ENCODER_HW_TIMER_COUNTER_MAX;
// 	}
// 	else
// 	{
// 		p_encoder->DeltaT = HAL_Encoder_ReadTimerCounter(p_encoder->CONFIG.P_HAL_ENCODER);
// 	}
// 	HAL_Encoder_WriteTimerCounter(p_encoder->CONFIG.P_HAL_ENCODER, 0U);
// // #else
// // 	_Encoder_CaptureDelta(p_encoder, &p_encoder->DeltaT, CONFIG_ENCODER_HW_TIMER_COUNTER_MAX);
// // #endif
// 	p_encoder->TotalT += p_encoder->DeltaT;
}

/*!
	Captures AngularD and DeltaT each pulse - ideal for low freq < POLLING_FREQ, use interpolation
 		e.g. Call each hall cycle / electric rotation inside hall edge interrupt
	Interval cannot be greater than 0xFFFF [ticks] => (0xFFFF / TimerFreq) [seconds]
*/
static inline void Encoder_DeltaT_Capture(Encoder_T * p_encoder)
{
	Encoder_DeltaT_CaptureD(p_encoder);
	Encoder_DeltaT_CaptureT(p_encoder);
}

/*
	SW Quadrature
	only when capturing a single phase of 2 phase quadrature mode, 180 degree offset. (not for 3 phase 120 degree offset)
*/
// static inline void Encoder_DeltaT_CaptureQuadrature(Encoder_T * p_encoder)
// {
// 	bool phaseB = Pin_Input_Read(&p_encoder->PhaseB);
// 	// #ifdef enocder hal pin read
// 	// bool phaseB = HAL_Encoder_ReadPhaseB(&p_encoder->PhaseB);

// 	Encoder_DeltaT_CaptureT(p_encoder);

// 	if (phaseB ^ p_encoder->Params.IsALeadBPositive) //check
// 	{
// 		_Encoder_CaptureAngularDIncreasing(p_encoder);
// 		if(p_encoder->TotalD < INT32_MAX) {p_encoder->TotalD += 1U;}
// 		//		p_encoder->DeltaD = 1; or set quadrature read direction
// 	}
// 	else
// 	{
// 		p_encoder->AngularD = (p_encoder->AngularD > 0U) ? p_encoder->AngularD - 1U : p_encoder->Params.CountsPerRevolution - 1U;
// 		if(p_encoder->TotalD > INT32_MIN) {p_encoder->TotalD -= 1;}
// 		//		p_encoder->DeltaD = -1; or set quadrature read direction
// 	}
// }

/*
	Polling Capture
*/
/* rising edge detect */
static inline bool Encoder_DeltaT_PollReferenceEdgeRising(Encoder_T * p_encoder, bool reference)
{
	bool status = (reference == true) && (p_encoder->PulseReferenceSaved == false);
	if (status == true) {Encoder_DeltaT_Capture(p_encoder);}
	p_encoder->PulseReferenceSaved = reference;
	return status;
}

/* both edge detect */
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
// static inline bool Encoder_DeltaT_PollPhaseAEdgeRising(Encoder_T * p_encoder)
// {
// 	return Encoder_DeltaT_PollReferenceEdgeRising(p_encoder, Pin_Input_Read(&p_encoder->PhaseA));
// }

// static inline bool Encoder_DeltaT_PollPhaseAEdgeDual(Encoder_T * p_encoder)
// {
// 	return Encoder_DeltaT_PollReferenceEdgeDual(p_encoder, Pin_Input_Read(&p_encoder->PhaseA));
// }

/******************************************************************************/
/*!
	@brief 	Extend base timer, 16bit timer cases to 32bit.

	ExtTimerFreq = 1000Hz, TimerCounterMax = 0xFFFF
		209ms to 13743S for TimerFreq = 312500Hz, 3.2us intervals
		104ms to 6871S for TimerFreq = 625000Hz
		1.6ms to 107S for TimerFreq = 40Mhz
*/
/******************************************************************************/
static inline uint32_t _Encoder_GetExtendedTimerDelta(Encoder_T * p_encoder)
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

// /*
// 	Extended timer counts equal to short timer overflow time
// 	This should optimize to compile time const
// */
// static inline uint32_t _Encoder_GetExtendedTimerThreshold(Encoder_T * p_encoder)
// {
// 	// return ((uint32_t)CONFIG_ENCODER_HW_TIMER_COUNTER_MAX + 1UL) * p_encoder->CONFIG.EXTENDED_TIMER_FREQ / p_encoder->CONFIG.DELTA_T_TIMER_FREQ;
// 	return ((uint32_t)CONFIG_ENCODER_HW_TIMER_COUNTER_MAX + 1UL) * p_encoder->CONFIG.EXTENDED_TIMER_FREQ / p_encoder->UnitT_Freq;
// }

/*
	Capture Extended Timer on Encoder Pulse
*/
static inline void Encoder_DeltaT_CaptureExtendedTimer(Encoder_T * p_encoder)
{
// #if defined(CONFIG_ENCODER_HW_CAPTURE_TIME)
	/* check time exceed short timer max value */
	// if(HAL_Encoder_ReadTimerCounterOverflow(p_encoder->CONFIG.P_HAL_ENCODER) == true)
	// {
	// 	p_encoder->DeltaT = _Encoder_GetExtendedTimerDelta(p_encoder) * (p_encoder->ExtendedTimerConversion);

	// 	/* If checking 32bit overflow is needed. should be caught by poll watch stop, which occurs prior */
	// 	//	if (extendedTimerDelta > (UINT32_MAX / p_encoder->CONFIG.DeltaT) * p_encoder->CONFIG.EXTENDED_TIMER_FREQ)
	// 	//	{
	// 	//		p_encoder->DeltaT = UINT32_MAX;
	// 	//	}
	// }
	// p_encoder->ExtendedTimerSaved = *(p_encoder->CONFIG.P_EXTENDED_TIMER);
// #else
	// uint32_t extendedTimerDelta = _Encoder_GetExtendedTimerDelta(p_encoder);
	// p_encoder->ExtendedTimerSaved = *(p_encoder->CONFIG.P_EXTENDED_TIMER);

	// if (extendedTimerDelta > _Encoder_GetExtendedTimerThreshold(p_encoder))
	// {
	// 	p_encoder->DeltaT = extendedTimerDelta * (p_encoder->ExtendedTimerConversion);
	// }
// #endif
}

/* WatchStop Must use ExtendedTimer */
/* Check Only */
static inline bool Encoder_DeltaT_GetWatchStop(Encoder_T * p_encoder)
{
 	return (_Encoder_GetExtendedTimerDelta(p_encoder) > p_encoder->Params.ExtendedTimerDeltaTStop);
}

/* Check and update saved state to 0 */
static inline bool Encoder_DeltaT_PollWatchStop(Encoder_T * p_encoder)
{
	bool isStop = Encoder_DeltaT_GetWatchStop(p_encoder);

	if (isStop == true)
	{
		p_encoder->DeltaT = UINT32_MAX;
		p_encoder->TimerCounterSaved = UINT32_MAX; /* Best chance at returning largest deltaT on next capture, TimerCounter < 32bits */
		p_encoder->SpeedSaved = 0U;
	}

	return isStop;
}

/******************************************************************************/
/*!
	T Unit Conversions - Variable DeltaT (DeltaD is fixed, == 1).
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

/******************************************************************************/
/*!
	@brief Linear Speed
*/
/******************************************************************************/
/*!
	CaptureDeltaT, DeltaD == 1: DeltaT timer ticks for given speed
	CaptureDeltaD, DeltaT == 1: (Number of fixed DeltaT samples, before a deltaD increment) (results in less than 1)
*/
static inline uint32_t Encoder_DeltaT_ConvertFromSpeed(Encoder_T * p_encoder, uint32_t speed_UnitsPerSecond)
{
	return (speed_UnitsPerSecond == 0U) ? 0U : p_encoder->UnitLinearSpeed / speed_UnitsPerSecond;
}

static inline uint32_t Encoder_DeltaT_ConvertFromSpeed_UnitsPerMinute(Encoder_T * p_encoder, uint32_t speed_UnitsPerMinute)
{
	return (speed_UnitsPerMinute == 0U) ? 0U : p_encoder->UnitLinearSpeed * 60U / speed_UnitsPerMinute;
}

static inline uint32_t Encoder_DeltaT_ConvertToSpeed(Encoder_T * p_encoder, uint32_t deltaT_ticks)
{
	return Encoder_DeltaT_ConvertFromSpeed(p_encoder, deltaT_ticks); /* Same division */
}

static inline uint32_t Encoder_DeltaT_ConvertToSpeed_UnitsPerMinute(Encoder_T * p_encoder, uint32_t deltaT_Ticks)
{
	return Encoder_DeltaT_ConvertFromSpeed_UnitsPerMinute(p_encoder, deltaT_Ticks); /* Same division */
}

/* In UnitsPerSecond */
static inline uint32_t Encoder_DeltaT_GetSpeed(Encoder_T * p_encoder)
{
	// return Encoder_CalcSpeed(p_encoder, 1U, p_encoder->DeltaT);
	return Encoder_DeltaT_ConvertToSpeed(p_encoder, p_encoder->DeltaT);
}

/******************************************************************************/
/*!
	@brief Rotational Speed
*/
/******************************************************************************/
static inline uint32_t Encoder_DeltaT_ConvertFromRotationalSpeed_RPM(Encoder_T * p_encoder, uint32_t rpm)
{
	return  p_encoder->UnitT_Freq * 60U / (p_encoder->Params.CountsPerRevolution * rpm);
}

static inline uint32_t Encoder_DeltaT_ConvertToRotationalSpeed_RPM(Encoder_T * p_encoder, uint32_t deltaT_ticks)
{
	return p_encoder->UnitT_Freq * 60U / (p_encoder->Params.CountsPerRevolution * deltaT_ticks);
}

static inline uint32_t Encoder_DeltaT_GetRotationalSpeed_RPM(Encoder_T * p_encoder)
{
	Encoder_DeltaT_ConvertToRotationalSpeed_RPM(p_encoder, p_encoder->DeltaT);
	// return Encoder_CalcRotationalSpeed_RPM(p_encoder, 1U, p_encoder->DeltaT);
	// return Encoder_GetAngularSpeed(p_encoder) * 60U >> CONFIG_ENCODER_ANGLE_DEGREES_BITS;
}

/******************************************************************************/
/*!
	@brief Frac16 Speed
*/
/******************************************************************************/
static inline uint32_t Encoder_DeltaT_GetFrac16Speed(Encoder_T * p_encoder)
{
	return Encoder_CalcFrac16Speed(p_encoder, 1U, p_encoder->DeltaT);
}

/******************************************************************************/
/*!
	@brief 	Interpolation Functions
			Estimated in between Encoder counts
			Only when POLLING_FREQ > EncoderFreq, i.e. polls per encoder count > 1

			//todo auto enable if capture freq < PollingFreq
*/
/*! @{ */
/******************************************************************************/
/*!
	@brief 	Angular Interpolation

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
	CaptureDeltaT only (DeltaD ==1) (DeltaT Mode UnitT_Freq > POLLING_FREQ) - cannot capture fractional DeltaD
	POLLING_FREQ/DeltaTCaptureFreq =  POLLING_FREQ / (UnitT_Freq / DeltaT);
 */
static inline uint32_t Encoder_DeltaT_GetInterpolationCount(Encoder_T *p_encoder)
{
	return p_encoder->CONFIG.POLLING_FREQ * p_encoder->DeltaT / p_encoder->UnitT_Freq;
}

/*
	InterpolationCycles - numbers of Polls per encoder count.
	only when pollingFreq > Encoder Counts Freq, i.e. 0 encoder counts per poll

	DeltaD Mode, applicable to low speeds
	8192 CPR Encoder, 20Khz Polling Freq => 146 RPM
	encoder count per Poll > 1, use Encoder_ConvertRotationalSpeedToDeltaAngle_RPM
*/
static inline uint32_t Encoder_DeltaT_ConvertRotationalSpeedToInterpolationCount_RPM(Encoder_T * p_encoder, uint16_t mechRpm)
{
	return p_encoder->CONFIG.POLLING_FREQ * 60U / (p_encoder->Params.CountsPerRevolution * mechRpm);
}

static inline uint32_t Encoder_DeltaT_ConvertInterpolationCountToRotationalSpeed_RPM(Encoder_T * p_encoder, uint16_t controlPeriods)
{
	return p_encoder->CONFIG.POLLING_FREQ * 60U / (p_encoder->Params.CountsPerRevolution * controlPeriods);
}

/*!
	@brief  LinearInterpolation Functions
*/
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

// /* Avg speed functions. reset during polling, use capture instead */
// static inline uint32_t Encoder_DeltaT_PollRotationalSpeedAvg_RPM(Encoder_T * p_encoder)
// {
// 	uint32_t avgSpeed = Encoder_CalcRotationalSpeed_RPM(p_encoder, p_encoder->TotalD, p_encoder->DeltaT);
// 	// speed = Encoder_CalcRotationalSpeed_RPM(p_encoder, p_encoder->TotalD + 1, p_encoder->TotalT + p_encoder->DeltaT);
// 	p_encoder->TotalD = 0U;
// 	p_encoder->TotalT = 0U;
// 	return avgSpeed;
// }

// static inline uint32_t Encoder_DeltaT_PollUnitSpeedAvg(Encoder_T * p_encoder)
// {
// 	uint32_t avgSpeed = Encoder_CalcFrac16Speed(p_encoder, p_encoder->TotalD, p_encoder->TotalT);
// 	p_encoder->TotalD = 0U;
// 	p_encoder->TotalT = 0U;
// 	return avgSpeed;
// }


/******************************************************************************/
/*!
	@brief 	Extern Declarations
*/
/*! @{ */
/******************************************************************************/
extern void Encoder_DeltaT_Init(Encoder_T * p_encoder);
extern void Encoder_DeltaT_SetExtendedTimerWatchStop(Encoder_T * p_encoder, uint16_t effectiveStopTime_Millis);
extern void Encoder_DeltaT_SetExtendedTimerWatchStop_Default(Encoder_T * p_encoder);
extern void Encoder_DeltaT_SetInitial(Encoder_T * p_encoder);
// extern void Encoder_DeltaT_CalibrateQuadratureReference(Encoder_T * p_encoder);
// extern void Encoder_DeltaT_CalibrateQuadraturePositive(Encoder_T * p_encoder);
/******************************************************************************/
/*! @} */
/******************************************************************************/

#endif
