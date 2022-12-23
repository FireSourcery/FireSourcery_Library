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
	const uint32_t periodTs_Freq = p_encoder->CONFIG.SAMPLE_FREQ; /* periodTs = 1 / SAMPLE_FREQ */
	const uint32_t timerFreq = p_encoder->CONFIG.TIMER_FREQ;
	uint32_t deltaTh; /* clear on Capture DeltaT */
	uint32_t periodTk_Freq;

	Encoder_DeltaD_Capture(p_encoder);

	if (HAL_Encoder_ReadTimerOverflow(p_encoder->CONFIG.P_HAL_ENCODER_TIMER) == true)
	{
		p_encoder->FreqD = (timerFreq / p_encoder->DeltaT);
	}
	else
	{
		deltaTh = HAL_Encoder_ReadTimer(p_encoder->CONFIG.P_HAL_ENCODER_TIMER);

		if(p_encoder->DeltaD != 0)
		{
			/* 	periodTk = periodTs + DeltaThPrev/timerFreq - deltaTh/timerFreq */
			periodTk_Freq = (timerFreq * periodTs_Freq) / (timerFreq + (periodTs_Freq * (p_encoder->DeltaTh - deltaTh)));
			/* 	if(periodTk > periodTs / 2) { freqD = (deltaD / periodTk) } */
			// if((periodTs_Freq * 2U) > periodTk_Freq) { p_encoder->FreqD = (p_encoder->DeltaD * periodTk_Freq); }
			p_encoder->FreqD = (p_encoder->DeltaD * periodTk_Freq);
		}

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
	uint32_t freqD = (p_encoder->FreqD < 0) ? 0 - p_encoder->FreqD : p_encoder->FreqD; /* DeltaD < 2  */
	return (freqD < p_encoder->CONFIG.POLLING_FREQ / 2U) ? Encoder_DeltaT_ProcInterpolateAngle(p_encoder) : 0U;
}


static inline int32_t Encoder_ModeDT_GetAngularVelocity(Encoder_T * p_encoder)
{
	return ((p_encoder->FreqD << 8U) / p_encoder->Params.CountsPerRevolution) << 8U;
}

static inline int32_t Encoder_ModeDT_GetRotationalVelocity(Encoder_T * p_encoder)
{
	return p_encoder->FreqD / p_encoder->Params.CountsPerRevolution;
	// return _Encoder_CalcRotationalVelocity_Shift(p_encoder, p_encoder->FreqD, 1U);
}

static inline int32_t Encoder_ModeDT_GetRotationalVelocity_RPM(Encoder_T * p_encoder)
{
	return p_encoder->FreqD * 60U / p_encoder->Params.CountsPerRevolution;
	// return _Encoder_CalcRotationalVelocity_Shift(p_encoder, p_encoder->FreqD * 60U, 1U);
}

static inline int32_t Encoder_ModeDT_GetScalarVelocity(Encoder_T * p_encoder)
{
	// p_encoder->FreqD * [60U * 65536U  / CountsPerRevolution / ScalarSpeedRef_Rpm]
	// return p_encoder->FreqD * ((uint32_t)60U * 65536UL / p_encoder->Params.CountsPerRevolution) / p_encoder->Params.ScalarSpeedRef_Rpm;
	return p_encoder->FreqD * (p_encoder->UnitScalarSpeed) / p_encoder->Params.ScalarSpeedRef_Rpm;
	// return Encoder_CalcScalarVelocity(p_encoder, p_encoder->FreqD, 1U);
}

static inline int32_t Encoder_ModeDT_GetLinearVelocity(Encoder_T * p_encoder)
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

extern void Encoder_ModeDT_Init(Encoder_T * p_encoder);
extern void Encoder_ModeDT_SetInitial(Encoder_T * p_encoder);
#endif