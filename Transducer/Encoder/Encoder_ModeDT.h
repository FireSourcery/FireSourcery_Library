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
	@file  	Encoder_DeltaD.h
	@author FireSourcery
	@brief 	Capture Delta D Mode, Polling time is fixed
	@version V0
*/
/******************************************************************************/
#ifndef ENCODER_MODE_DT_H
#define ENCODER_MODE_DT_H

#include "Encoder.h"
#include "Encoder_DeltaD.h"
#include "Encoder_DeltaT.h"
#include "Encoder_Motor.h"

/******************************************************************************/
/*
*/
/******************************************************************************/
static inline uint32_t Encoder_ModeDT_CaptureFreqD(Encoder_T * p_encoder)
{
	/* periodTs = 1 / SPEED_SAMPLE_FREQ */
	static const uint32_t periodTs_Freq = p_encoder->CONFIG.SPEED_SAMPLE_FREQ;
	static const uint32_t timerFreq = p_encoder->CONFIG.TIMER_FREQ;
	uint32_t deltaTh = HAL_Encoder_ReadTimer(p_encoder->CONFIG.P_HAL_ENCODER_TIMER);
	int32_t deltaD = p_encoder->DeltaD;

	/* 	periodTk = periodTs + DeltaThPrev/timerFreq - deltaTh/timerFreq */
	uint32_t periodTk_Freq = (periodTs_Freq * timerFreq) / (timerFreq + (periodTs_Freq * (p_encoder->DeltaTh - deltaTh)));

	/* 	if(periodTk > periodTs / 2) { freqD = (deltaD / periodTk) } */
	if((deltaD != 0) && ((periodTs_Freq * 2UL) > periodTk_Freq) { p_encoder->FreqD = (deltaD * periodTk_Freq) }

	p_encoder->DeltaTh = deltaTh;
}

static inline uint32_t Encoder_ModeDT_GetAngularVelocity(Encoder_T * p_encoder)
{
	return ((p_encoder->FreqD << 8) / p_encoder->Params.CountsPerRevolution) << 8;
}

static inline uint32_t Encoder_ModeDT_GetAngularSpeed_RadS(Encoder_T * p_encoder)
{

}

static inline uint32_t Encoder_ModeDT_GetRotationalSpeed_RPS(Encoder_T * p_encoder)
{

}

static inline uint32_t Encoder_ModeDT_GetRotationalSpeed_RPM(Encoder_T * p_encoder)
{
	return p_encoder->FreqD * 60U / p_encoder->Params.CountsPerRevolution;
}

static inline uint32_t Encoder_ModeDT_GetScalarSpeed(Encoder_T * p_encoder)
{
	// p_encoder->FreqD * [60U * 65535U  / CountsPerRevolution / ScalarSpeedRef_Rpm]
	Encoder_CalcScalarSpeed(p_encoder, p_encoder->FreqD / p_encoder->CONFIG.SPEED_SAMPLE_FREQ, 1U);
}

static inline uint32_t Encoder_ModeDT_GetLinearSpeed(Encoder_T * p_encoder)
{

}

static inline uint32_t Encoder_ModeDT_GetGroundSpeed_Mph(Encoder_T * p_encoder)
{

}

static inline uint32_t Encoder_ModeDT_GetGroundSpeed_Kmh(Encoder_T * p_encoder)
{

}

#endif