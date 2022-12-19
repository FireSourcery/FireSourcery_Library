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
	@brief 	Capture DeltaT - Poll Pulse, ISR
 			Capture DeltaT, per count, fixed changed in distance
			Interval cannot be greater than 0xFFFF [ticks] => (0xFFFF / TimerFreq) [seconds]
			Low freq, < POLLING_FREQ, use interpolation
				e.g. Call each hall cycle / electric rotation inside hall edge interrupt
*/
/******************************************************************************/
static inline void Encoder_DeltaT_Capture(Encoder_T * p_encoder)
{
	if(HAL_Encoder_ReadTimerOverflow(p_encoder->CONFIG.P_HAL_ENCODER_TIMER) == false)
	{
		p_encoder->DeltaT = HAL_Encoder_ReadTimer(p_encoder->CONFIG.P_HAL_ENCODER_TIMER);
	}
	else
	{
		HAL_Encoder_ClearTimerOverflow(p_encoder->CONFIG.P_HAL_ENCODER_TIMER);
		p_encoder->DeltaT = ENCODER_TIMER_MAX;
	}

	HAL_Encoder_WriteTimer(p_encoder->CONFIG.P_HAL_ENCODER_TIMER, 0U);
}

/******************************************************************************/
/*!
	@brief Extend base timer, 16bit timer cases to 32bit.

	ExtTimerFreq = 1000Hz, TimerMax = 0xFFFF
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
	if(time < p_encoder->ExtendedDeltaTimerSaved) { deltaTime = UINT32_MAX - p_encoder->ExtendedDeltaTimerSaved + time + 1U; }
	else
#endif
	{ deltaTime = time - p_encoder->ExtendedTimerSaved;	}

	return deltaTime;
}

/*
	Use Extended Timer to extend Low RPM range.
	Call on Encoder Edge.
		32 bit DeltaT overflow should be caught by poll watch stop, which is a shorter period
*/
static inline void Encoder_DeltaT_CaptureExtended(Encoder_T * p_encoder)
{
	if(HAL_Encoder_ReadTimerOverflow(p_encoder->CONFIG.P_HAL_ENCODER_TIMER) == false)
	{
		p_encoder->DeltaT = HAL_Encoder_ReadTimer(p_encoder->CONFIG.P_HAL_ENCODER_TIMER);
	}
	else
	{
		HAL_Encoder_ClearTimerOverflow(p_encoder->CONFIG.P_HAL_ENCODER_TIMER);
		p_encoder->DeltaT = _Encoder_GetExtendedTimerDelta(p_encoder) * p_encoder->ExtendedTimerConversion;
	}

	HAL_Encoder_WriteTimer(p_encoder->CONFIG.P_HAL_ENCODER_TIMER, 0U);
	p_encoder->ExtendedTimerSaved = *(p_encoder->CONFIG.P_EXTENDED_TIMER);
}

/*
	Use Extended Timer to check 0 speed.
	Call periodically.
	Check Only, use with CaptureExtended
*/
static inline bool Encoder_DeltaT_CheckWatchStop(Encoder_T * p_encoder)
{
 	return (_Encoder_GetExtendedTimerDelta(p_encoder) > p_encoder->Params.ExtendedTimerDeltaTStop);
}

/******************************************************************************/
/*!
	Polling Capture
*/
/******************************************************************************/
/*
	Polling Capture, Single Phase
*/
/* rising edge detect */
static inline bool Encoder_DeltaT_PollReferenceEdgeRising(Encoder_T * p_encoder, bool reference)
{
	bool status = (reference == true) && (p_encoder->Phases.A == false);
	if(status == true) { Encoder_DeltaT_Capture(p_encoder); }
	p_encoder->Phases.A = reference;
	return status;
}

/* both edge detect */
static inline bool Encoder_DeltaT_PollReferenceEdgeDual(Encoder_T * p_encoder, bool reference)
{
	bool status = (reference ^ p_encoder->Phases.A);
	if(status == true) { Encoder_DeltaT_Capture(p_encoder); }
	p_encoder->Phases.A = reference;
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
	return Encoder_DeltaT_PollReferenceEdgeRising(p_encoder, Pin_Input_Read(&p_encoder->PinA));
}

static inline bool Encoder_DeltaT_PollPhaseAEdgeDual(Encoder_T * p_encoder)
{
	return Encoder_DeltaT_PollReferenceEdgeDual(p_encoder, Pin_Input_Read(&p_encoder->PinA));
}

/******************************************************************************/
/*!
	T Unit Conversions
*/
/******************************************************************************/
static inline uint32_t Encoder_DeltaT_ConvertToFreq(Encoder_T * p_encoder, uint32_t deltaT_Ticks) { return (deltaT_Ticks == 0U) ? 0U : p_encoder->UnitT_Freq / deltaT_Ticks; }

/******************************************************************************/
/*!
	@brief	Angle Interpolation Functions

	Estimate Angle each control cycle in between encoder counts
	AngleControlIndex * AngularSpeed / POLLING_FREQ;
		(AngleControlIndex / POLLING_FREQ) * [1(DeltaD) * UnitT_Freq / DeltaT) * (AngleSize / EncoderResolution)]
		AngleControlIndex * [1(DeltaD) * AngleSize * UnitT_Freq / DeltaT / EncoderResolution] / POLLING_FREQ
		AngleControlIndex * [1(DeltaD) * AngleSize * UnitT_Freq / EncoderResolution / POLLING_FREQ] / DeltaT
	AngleControlIndex ranges from 0 to InterpolationCount

	Only when POLLING_FREQ > EncoderFreq, polls per encoder count > 1
		todo auto enable if EncoderFreq < PollingFreq
*/
/*! @{ */
/******************************************************************************/
/*!
	@brief InterpolateAngle
	pollingIndex * 1(DeltaD) * [UnitInterpolateAngle] / DeltaT
	UnitInterpolateAngle = [AngleSize[65536] * UnitT_Freq / POLLING_FREQ / CountsPerRevolution]
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
	Samples per DeltaT Capture, index max
	CaptureDeltaT only (DeltaD ==1) (DeltaT Mode UnitT_Freq > POLLING_FREQ) - cannot capture fractional DeltaD
	POLLING_FREQ/DeltaTCaptureFreq =  POLLING_FREQ / (UnitT_Freq / DeltaT);
*/
static inline uint32_t Encoder_DeltaT_GetInterpolationCount(Encoder_T * p_encoder)
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

static inline uint32_t Encoder_DeltaT_ConvertInterpolationCountToRotationalSpeed_RPM(Encoder_T * p_encoder, uint16_t interpolationCount)
{
	return p_encoder->CONFIG.POLLING_FREQ * 60U / (p_encoder->Params.CountsPerRevolution * interpolationCount);
}

/******************************************************************************/
/*!
	@brief Angular/Rotational Speed
*/
/******************************************************************************/
static inline uint32_t Encoder_DeltaT_GetAngularSpeed(Encoder_T * p_encoder)
{
	return Encoder_CalcAngularSpeed(p_encoder, 1U, p_encoder->DeltaT);
}

static inline uint32_t Encoder_DeltaT_GetRotationalSpeed(Encoder_T * p_encoder)
{
	return _Encoder_CalcRotationalSpeed(p_encoder, 1U, p_encoder->DeltaT);
}

static inline uint32_t Encoder_DeltaT_GetRotationalSpeed_RPM(Encoder_T * p_encoder)
{
	return _Encoder_CalcRotationalSpeed(p_encoder, 60U, p_encoder->DeltaT);
}

static inline uint32_t Encoder_DeltaT_ConvertFromRotationalSpeed_RPM(Encoder_T * p_encoder, uint32_t rpm)
{
	return (rpm == 0U) ? 0U : p_encoder->UnitT_Freq * 60U / (p_encoder->Params.CountsPerRevolution * rpm);
}

static inline uint32_t Encoder_DeltaT_ConvertToRotationalSpeed_RPM(Encoder_T * p_encoder, uint32_t deltaT_ticks)
{
	return Encoder_DeltaT_ConvertFromRotationalSpeed_RPM(p_encoder, deltaT_ticks);
}

/******************************************************************************/
/*!
	@brief Scalar Speed
*/
/******************************************************************************/
static inline uint32_t Encoder_DeltaT_GetScalarSpeed(Encoder_T * p_encoder)
{
	return Encoder_CalcScalarSpeed(p_encoder, 1U, p_encoder->DeltaT);
}

static inline uint32_t Encoder_DeltaT_GetScalarSpeed_WatchStop(Encoder_T * p_encoder)
{
	return (Encoder_DeltaT_CheckWatchStop(p_encoder) == true) ? 0U : Encoder_DeltaT_GetScalarSpeed(p_encoder);
}

/******************************************************************************/
/*!
	@brief Linear Speed
*/
/******************************************************************************/
/* In UnitsPerSecond */
static inline uint32_t Encoder_DeltaT_GetLinearSpeed(Encoder_T * p_encoder)
{
	return Encoder_CalcLinearSpeed(p_encoder, 1U, p_encoder->DeltaT);
}

static inline uint32_t Encoder_DeltaT_ConvertFromLinearSpeed(Encoder_T * p_encoder, uint32_t speed_UnitsPerSecond)
{
	return (speed_UnitsPerSecond == 0U) ? 0U : p_encoder->UnitLinearSpeed / speed_UnitsPerSecond;
}

static inline uint32_t Encoder_DeltaT_ConvertFromLinearSpeed_UnitsPerMinute(Encoder_T * p_encoder, uint32_t speed_UnitsPerMinute)
{
	return (speed_UnitsPerMinute == 0U) ? 0U : p_encoder->UnitLinearSpeed * 60U / speed_UnitsPerMinute;
}

static inline uint32_t Encoder_DeltaT_ConvertToLinearSpeed(Encoder_T * p_encoder, uint32_t deltaT_ticks)
{
	return Encoder_DeltaT_ConvertFromLinearSpeed(p_encoder, deltaT_ticks); /* Same division */
}

static inline uint32_t Encoder_DeltaT_ConvertToLinearSpeed_UnitsPerMinute(Encoder_T * p_encoder, uint32_t deltaT_Ticks)
{
	return Encoder_DeltaT_ConvertFromLinearSpeed_UnitsPerMinute(p_encoder, deltaT_Ticks); /* Same division */
}

static inline uint32_t Encoder_DeltaT_GetGroundSpeed_Mph(Encoder_T * p_encoder)
{
	return Encoder_CalcGroundSpeed_Mph(p_encoder, 1U, p_encoder->DeltaT);
}

static inline uint32_t Encoder_DeltaT_GetGroundSpeed_Kmh(Encoder_T * p_encoder)
{
	return Encoder_CalcGroundSpeed_Kmh(p_encoder, 1U, p_encoder->DeltaT);
}

/******************************************************************************/
/*!
	@brief 	Extern Declarations
*/
/*! @{ */
/******************************************************************************/
extern void Encoder_DeltaT_Init(Encoder_T * p_encoder);
extern void Encoder_DeltaT_SetInitial(Encoder_T * p_encoder);
extern void Encoder_DeltaT_SetExtendedTimerWatchStop_Millis(Encoder_T * p_encoder, uint16_t effectiveStopTime_Millis);
extern void Encoder_DeltaT_SetExtendedTimerWatchStop_RPM(Encoder_T * p_encoder);
/******************************************************************************/
/*! @} */
/******************************************************************************/

#endif

/*
	e.g. CaptureDeltaT, Fixed DeltaD:
	UnitT_Freq = 312500							=> Period = 3.2 uS
	TimerMax = 0xFFFF							=> Overflow 209,712 us, 209 ms
	DeltaDUnits(CountsPerRevolution) = 1/8 (8) 	=> AngleRes 0.125 (of 1), 8,169.5 Angle16

	DeltaT = 0xFFFF => Speed = 312500*(1/8)/0xFFFF	= 0.5976 rps, 35.76 RPM
	DeltaT = 1 		=> Speed = 312500*(1/8) 		= 39062.5 rps, 2343750 RPM
	DeltaT = 100	=> Speed = 312500*(1/8)/100		= 390.625 rps, 23437.50 RPM // accurate within 1% below

	1RPM: DeltaT = (312500/8)/(1/60) = 2,343,750
	RPM = 100, 		RPS = 1.6667, 	Angle16Real = 109225 	=> ISR = 13.3333 Hz, 75ms,	 => DeltaT = 23437, 	Angle16 = 109227, error 2,
	RPM = 1000, 	RPS = 16.667, 	Angle16Real = 1092250 	=> ISR = 133.333 Hz, 7.5ms,	=> DeltaT = 2343, 		Angle16 = 1092599, error 349, 0.0053 rps
	RPM = 10000, 	RPS = 166.67,	Angle16Real = 10922500 	=> ISR = 1333.33 Hz, .75ms,	=> DeltaT = 234, 		Angle16 = 10940004, error 17504, .2670 rps

	e.g. 2:
	UnitT_Freq = 2,000,000						=> Period = .5 uS
	TimerMax = 0xFFFF								=> Overflow 32677.5us, 32.6775 ms
	TimerMax = 0xFFFFFFFF							=> Overflow 35.7913941 minutes
	DeltaDUnits(CountsPerRevolution) = 1/2048(2048) 	=> AngleRes 0.000488 (of 1), 32 Angle16

	DeltaT = 0xFFFF => Speed = 2,000,000*(1/2048)/0xFFFF	= 0.01490 rps, 0.89408331426 RPM
	DeltaT = 1 		=> Speed = 2,000,000*(1/2048) 			= 976.5625 rps, 58593.75 RPM
	DeltaT = 10		=> Speed = 2,000,000*(1/2048)/10 		= 97.65625 rps, 5859.375 RPM => accurate within 10% below

	1RPM: DeltaT = (2,000,000/2048)/(1/60) = 58593.75, angle 1092.26666667
	RPM = 100, 		RPS = 1.6667, Angle16Real = 109226.66667 => ISR = 3413.3333 Hz, 292.96875us, 	=> DeltaT = 585, 	Angle16 = 109400, error 175
	RPM = 1000, 	RPS = 16.667, Angle16Real = 1092266.6667 => ISR = 34133.333 Hz, 29.29687us, 	=> DeltaT = 58, 	Angle16 = 1103431, error 11181,	0.1706 rps
	RPM = 10000, 	RPS = 166.67, Angle16Real = 10922666.667 => ISR = 341333.33 Hz, 2.92968us,		=> DeltaT = 5, 		Angle16 = 12799804, error 1877304, 28.645 rps

	PPR = 8192 => 136.5333 ISR/s/RPM => 10000 rpm => 1,365,333 ISR/s
	2MHz Period = .5 uS => accurate within 1% when speed < 146.484375 RPM, within 10% when 1464.84375
	20MHz Period = .05 uS => accurate within 1% when speed < 1464.84375 RPM, within 10% when 14648.4375

	20000Hz/(10000RPM/60) => 120 PPR. Less than 120 PPR, use Fixed DeltaD
*/