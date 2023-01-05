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
	@file  	Encoder_ModeDT.h
	@author FireSourcery
	@brief 	Mixed Frequency Sampling
	@version V0
*/
/******************************************************************************/
#ifndef ENCODER_MODE_DT_H
#define ENCODER_MODE_DT_H

#include "Encoder_DeltaD.h"
#include "Encoder_DeltaT.h"

/******************************************************************************/
/*
*/
/******************************************************************************/
/* In SAMPLE_FREQ time */
static inline void Encoder_ModeDT_CaptureFreqD(Encoder_T * p_encoder)
{
	const uint32_t sampleFreq = p_encoder->CONFIG.SAMPLE_FREQ; /* periodTs = 1 / SAMPLE_FREQ */
	const uint32_t timerFreq = p_encoder->CONFIG.TIMER_FREQ;
	uint32_t deltaTh;
	uint32_t freqTk;

	Encoder_DeltaD_Capture(p_encoder);

	if(p_encoder->DeltaD == 0)
	{
		/* Assume its user direction, low speed opposite direction will be seen as aligned direction */
		// p_encoder->FreqD = Encoder_GetDirection_SinglePhase(p_encoder) * (timerFreq / p_encoder->DeltaT);
		p_encoder->FreqD = p_encoder->DirectionD * (timerFreq / p_encoder->DeltaT);
		/* Set next periodTk as ~DeltaT on overflow */
		p_encoder->DeltaTh = (HAL_Encoder_ReadTimerOverflow(p_encoder->CONFIG.P_HAL_ENCODER_TIMER) == true) ?
			p_encoder->DeltaT - timerFreq / sampleFreq : HAL_Encoder_ReadTimer(p_encoder->CONFIG.P_HAL_ENCODER_TIMER);
	}
	else
	{
		p_encoder->DirectionD = (p_encoder->DeltaD > 0) - (p_encoder->DeltaD < 0);
		deltaTh = HAL_Encoder_ReadTimer(p_encoder->CONFIG.P_HAL_ENCODER_TIMER); /* Overflow is > SampleTime. DeltaD == 0 occurs prior. */
		/* periodTk = periodTs + DeltaThPrev/timerFreq - deltaTh/timerFreq */
		freqTk = (timerFreq * sampleFreq) / (timerFreq + (sampleFreq * (p_encoder->DeltaTh - deltaTh)));
		/* if(periodTk > periodTs / 2) { freqD = (deltaD / periodTk) } */
		if((sampleFreq * 2U) > freqTk) { p_encoder->FreqD = (p_encoder->DeltaD * freqTk); }
		// p_encoder->FreqD = (p_encoder->DeltaD * freqTk);
		p_encoder->DeltaTh = deltaTh;
	}
}

static inline void Encoder_ModeDT_CaptureVelocity(Encoder_T * p_encoder)
{
	if(Encoder_DeltaT_CheckExtendedStop(p_encoder) == false) 	{ Encoder_ModeDT_CaptureFreqD(p_encoder); }
	else 														{ p_encoder->FreqD = 0U; }
}

static inline uint32_t Encoder_ModeDT_InterpolateAngle(Encoder_T * p_encoder)
{
	uint32_t freqD = (p_encoder->FreqD < 0) ? 0 - p_encoder->FreqD : p_encoder->FreqD; /* |DeltaD| <= 1 */
	return (freqD < p_encoder->CONFIG.POLLING_FREQ / 2U) ? Encoder_DeltaT_ProcInterpolateAngle(p_encoder) : 0U;
}

/*
	DirectionD to the last DeltaD capture
*/
static inline int32_t Encoder_ModeDT_InterpolateAngularDisplacement(Encoder_T * p_encoder)
{
	// return Encoder_GetDirection_SinglePhase(p_encoder) * Encoder_ModeDT_InterpolateAngle(p_encoder); assume aligned to user select
	return Encoder_GetDirection(p_encoder) * p_encoder->DirectionD * Encoder_ModeDT_InterpolateAngle(p_encoder);
}

/******************************************************************************/
/*
*/
/******************************************************************************/
/* Signed with capture reference */
static inline int32_t _Encoder_ModeDT_GetScalarSpeed(Encoder_T * p_encoder)
{
	return p_encoder->FreqD * p_encoder->UnitScalarSpeed >> p_encoder->UnitScalarSpeedShift;
}

static inline int32_t Encoder_ModeDT_GetScalarVelocity(Encoder_T * p_encoder)
{
	return Encoder_GetDirection(p_encoder) * _Encoder_ModeDT_GetScalarSpeed(p_encoder);
}

static inline int32_t Encoder_ModeDT_PollScalarVelocity(Encoder_T * p_encoder)
{
	Encoder_ModeDT_CaptureVelocity(p_encoder);
	return Encoder_ModeDT_GetScalarVelocity(p_encoder);
}

/******************************************************************************/
/*
*/
/******************************************************************************/
static inline int32_t Encoder_ModeDT_GetAngularSpeed(Encoder_T * p_encoder)
{
	return p_encoder->FreqD * p_encoder->UnitAngularSpeed >> p_encoder->UnitAngularSpeedShift;
}

static inline int32_t Encoder_ModeDT_GetRotationalSpeed(Encoder_T * p_encoder)
{
	return p_encoder->FreqD / p_encoder->Params.CountsPerRevolution;
}

static inline int32_t Encoder_ModeDT_GetRotationalSpeed_RPM(Encoder_T * p_encoder)
{
	return p_encoder->FreqD * 60U / p_encoder->Params.CountsPerRevolution;
}

static inline int32_t Encoder_ModeDT_GetAngularVelocity(Encoder_T * p_encoder)
{
}

static inline int32_t Encoder_ModeDT_GetRotationalVelocity(Encoder_T * p_encoder)
{
}

static inline int32_t Encoder_ModeDT_GetRotationalVelocity_RPM(Encoder_T * p_encoder)
{
}

static inline int32_t Encoder_ModeDT_GetSurfaceSpeed(Encoder_T * p_encoder)
{
	return p_encoder->FreqD * p_encoder->UnitSurfaceSpeed >> p_encoder->UnitSurfaceSpeedShift;
}

static inline int32_t Encoder_ModeDT_GetSurfaceVelocity(Encoder_T * p_encoder)
{

}

static inline int32_t Encoder_ModeDT_GetGroundVelocity_Mph(Encoder_T * p_encoder)
{

}

static inline int32_t Encoder_ModeDT_GetGroundVelocity_Kmh(Encoder_T * p_encoder)
{

}

// static inline int32_t Encoder_GetDeltaAngularDisplacement(Encoder_T * p_encoder)
// {

// }

// static inline int32_t Encoder_GetDeltaRotationalDisplacement(Encoder_T * p_encoder)
// {

// }

// static inline int32_t Encoder_GetDeltaLinearDisplacement(Encoder_T * p_encoder)
// {

// }

// static inline int32_t Encoder_GetTotalAngularDisplacement(Encoder_T * p_encoder)
// {
// }

// static inline int32_t Encoder_GetTotalLinearDisplacement(Encoder_T * p_encoder)
// {
// }

/******************************************************************************/
/*
*/
/******************************************************************************/
extern void Encoder_ModeDT_Init(Encoder_T * p_encoder);
extern void Encoder_ModeDT_SetInitial(Encoder_T * p_encoder);

#endif