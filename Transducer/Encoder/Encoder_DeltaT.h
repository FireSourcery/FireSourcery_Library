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
	@brief 	Capture DeltaT Mode, displacement is fixed
	@version V0
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
		p_encoder->DeltaT = (HAL_Encoder_ReadTimer(p_encoder->CONFIG.P_HAL_ENCODER_TIMER) + p_encoder->DeltaT) / 2U;
	}
	else
	{
		HAL_Encoder_ClearTimerOverflow(p_encoder->CONFIG.P_HAL_ENCODER_TIMER);
		p_encoder->DeltaT = ENCODER_TIMER_MAX;
	}

	HAL_Encoder_WriteTimer(p_encoder->CONFIG.P_HAL_ENCODER_TIMER, 0U);
}

static inline bool Encoder_DeltaT_CheckStop(Encoder_T * p_encoder)
{
 	return (p_encoder->DeltaT == ENCODER_TIMER_MAX);
}

/******************************************************************************/
/*!
	@brief Extend base timer, 16bit timer cases to 32bit.
	Use Extended Timer to extend Low RPM range.
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
	{ deltaTime = time - p_encoder->ExtendedTimerPrev;	}

	return deltaTime;
}

/*
	Call on Encoder Edge.
	32 bit DeltaT overflow should be caught by poll watch stop, which is a shorter period
*/
static inline void Encoder_DeltaT_CaptureExtended(Encoder_T * p_encoder)
{
	if(HAL_Encoder_ReadTimerOverflow(p_encoder->CONFIG.P_HAL_ENCODER_TIMER) == false)
	{
		p_encoder->DeltaT = (HAL_Encoder_ReadTimer(p_encoder->CONFIG.P_HAL_ENCODER_TIMER) + p_encoder->DeltaT) / 2U;
		// p_encoder->DeltaT = HAL_Encoder_ReadTimer(p_encoder->CONFIG.P_HAL_ENCODER_TIMER);
	}
	else
	{
		HAL_Encoder_ClearTimerOverflow(p_encoder->CONFIG.P_HAL_ENCODER_TIMER);
		p_encoder->DeltaT = _Encoder_GetExtendedTimerDelta(p_encoder) * p_encoder->ExtendedTimerConversion;
	}

	HAL_Encoder_WriteTimer(p_encoder->CONFIG.P_HAL_ENCODER_TIMER, 0U);
	p_encoder->ExtendedTimerPrev = *(p_encoder->CONFIG.P_EXTENDED_TIMER);
}

/*
	Use Extended Timer to check 0 speed.
*/
static inline bool Encoder_DeltaT_CheckExtendedStop(Encoder_T * p_encoder)
{
 	return (_Encoder_GetExtendedTimerDelta(p_encoder) > p_encoder->Params.ExtendedTimerDeltaTStop);
}

/******************************************************************************/
/*!
	@brief	Angle Interpolation Functions

	Estimate Angle each control cycle in between encoder counts
	AngularSpeed * AngleControlIndex / POLLING_FREQ ;
		(AngleControlIndex / POLLING_FREQ) * [1(DeltaD) * UnitT_Freq / DeltaT) * (AngleSize / EncoderResolution)]
		AngleControlIndex * [1(DeltaD) * AngleSize * UnitT_Freq / EncoderResolution / POLLING_FREQ] / DeltaT
	AngleControlIndex [0:InterpolationCount]

	Only when POLLING_FREQ > EncoderFreq, i.e. 0 encoder counts per poll, polls per encoder count > 1
	e.g. High res break even point
		8192 CountsPerRevolution, 20Khz POLLING_FREQ => 146 RPM
*/
/******************************************************************************/
/*!
	@brief InterpolateAngle
	pollingIndex * 1(DeltaD) * [UnitInterpolateAngle] / DeltaT
	UnitInterpolateAngle = [AngleSize[65536] * UnitT_Freq / POLLING_FREQ / CountsPerRevolution]
*/
static inline uint32_t Encoder_DeltaT_InterpolateAngleIndex(Encoder_T * p_encoder, uint32_t pollingIndex)
{
	uint32_t angle = pollingIndex * p_encoder->UnitInterpolateAngle / p_encoder->DeltaT;
	return (angle > p_encoder->InterpolateAngleLimit) ? p_encoder->InterpolateAngleLimit : angle;
}

static inline uint32_t Encoder_DeltaT_ProcInterpolateAngle(Encoder_T * p_encoder)
{
	p_encoder->InterpolationIndex++;
	return Encoder_DeltaT_InterpolateAngleIndex(p_encoder, p_encoder->InterpolationIndex);
}

static inline void Encoder_DeltaT_ZeroInterpolateAngle(Encoder_T * p_encoder)
{
	p_encoder->InterpolationIndex = 0U;
}

/*
	InterpolationCycles - numbers of Polls per encoder count.
	Samples per DeltaT Capture, index max
	POLLING_FREQ/PulseFreq == POLLING_FREQ / (UnitT_Freq / DeltaT);
*/
static inline uint32_t Encoder_DeltaT_GetInterpolationCount(Encoder_T * p_encoder)
{
	return p_encoder->CONFIG.POLLING_FREQ * p_encoder->DeltaT / p_encoder->UnitT_Freq;
}

static inline uint32_t Encoder_DeltaT_ConvertRotationalSpeedToInterpolationCount_RPM(Encoder_T * p_encoder, uint16_t mechRpm)
{
	return p_encoder->CONFIG.POLLING_FREQ * 60U / (p_encoder->Params.CountsPerRevolution * mechRpm);
}

static inline uint32_t Encoder_DeltaT_ConvertInterpolationCountToRotationalSpeed_RPM(Encoder_T * p_encoder, uint16_t interpolationCount)
{
	return Encoder_DeltaT_ConvertRotationalSpeedToInterpolationCount_RPM(p_encoder, interpolationCount);
}

/******************************************************************************/
/*!
	@brief DeltaT only Functions
*/
/******************************************************************************/
/******************************************************************************/
/*!
	@brief Angular/Rotational Speed
*/
/******************************************************************************/
static inline uint32_t Encoder_DeltaT_GetAngularSpeed(Encoder_T * p_encoder)
{
	/* p_encoder->UnitAngularSpeed set to 0U if overflow */
	return (p_encoder->UnitAngularSpeed == 0U) ?
		((p_encoder->UnitT_Freq / p_encoder->DeltaT) << ENCODER_ANGLE16) / p_encoder->Params.CountsPerRevolution :
		_Encoder_CalcAngularSpeed(p_encoder, 1U, p_encoder->DeltaT);
}

static inline uint32_t Encoder_DeltaT_GetRotationalSpeed(Encoder_T * p_encoder)
{
	return _Encoder_CalcRotationalSpeed(p_encoder, 1U, p_encoder->DeltaT);
}

static inline uint32_t Encoder_DeltaT_GetRotationalSpeed_RPM(Encoder_T * p_encoder)
{
	// check (p_encoder->UnitRotationalSpeed60 == 0U) || *60 > INTMAX
	return _Encoder_CalcRotationalSpeed(p_encoder, 60U, p_encoder->DeltaT);
}

// static inline uint32_t Encoder_DeltaT_ConvertFromRotationalSpeed_RPM(Encoder_T * p_encoder, uint32_t rpm)
// {
// 	check (p_encoder->UnitAngularSpeed == 0U) || *60 > INTMAX
// 	return (rpm == 0U) ? 0U : p_encoder->UnitT_Freq * 60U / (p_encoder->Params.CountsPerRevolution * rpm);
// }

// static inline uint32_t Encoder_DeltaT_ConvertToRotationalSpeed_RPM(Encoder_T * p_encoder, uint32_t deltaT_ticks)
// {
// 	return Encoder_DeltaT_ConvertFromRotationalSpeed_RPM(p_encoder, deltaT_ticks);
// }

/******************************************************************************/
/*!
	@brief Scalar Speed
*/
/******************************************************************************/
static inline uint32_t Encoder_DeltaT_GetScalarSpeed(Encoder_T * p_encoder)
{
	return p_encoder->UnitScalarSpeed / p_encoder->DeltaT;
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
extern void _Encoder_DeltaT_Init(Encoder_T * p_encoder);
extern void Encoder_DeltaT_Init(Encoder_T * p_encoder);
extern void Encoder_DeltaT_SetInitial(Encoder_T * p_encoder);
extern void Encoder_DeltaT_SetExtendedTimerWatchStop_Millis(Encoder_T * p_encoder, uint16_t effectiveStopTime_Millis);
extern void Encoder_DeltaT_SetExtendedTimerWatchStop_RPM(Encoder_T * p_encoder);
extern void Encoder_DeltaT_SetInterpolateAngleScalar(Encoder_T * p_encoder, uint16_t scalar);
/******************************************************************************/
/*! @} */
/******************************************************************************/

#endif

