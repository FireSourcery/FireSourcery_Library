
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
	@file  	Encoder_DeltaDT.c
	@author FireSourcery
	@brief
	@version V0
*/
/******************************************************************************/
#include "Encoder_DeltaD.h"
#include "Encoder_DeltaT.h"
#include <string.h>


/* Iterative log2 */
uint8_t Log2(uint32_t num)
{
	uint8_t shift = 0U;
	while((num >> shift) > 1U) { shift++; }
	return shift;
}

uint8_t GetMaxShift_Signed(int32_t num)
{
	return Log2(INT32_MAX / num);
}

/* 1 << Shift <= INT32_MAX / ((x_max - x0) * Slope) */
uint8_t GetMaxSlopeShift_Signed(int32_t factor, int32_t divisor, int32_t maxDelta)
{
	return GetMaxShift_Signed(maxDelta * factor / divisor); /* divide first rounds up log output */
}


static void ResetUnitsScalarSpeed(Encoder_T * p_encoder);

void Encoder_ModeDT_Init(Encoder_T * p_encoder)
{
	if(p_encoder->CONFIG.P_PARAMS != 0U) { memcpy(&p_encoder->Params, p_encoder->CONFIG.P_PARAMS, sizeof(Encoder_Params_T)); }
	_Encoder_DeltaT_Init(p_encoder);
	_Encoder_DeltaD_Init(p_encoder);

	p_encoder->UnitT_Freq = 1U;
	_Encoder_ResetUnitsAngular(p_encoder);
	_Encoder_ResetUnitsLinear(p_encoder);
	ResetUnitsScalarSpeed(p_encoder);

	Encoder_DeltaD_SetInitial(p_encoder);
	Encoder_DeltaT_SetInitial(p_encoder);
}

void Encoder_ModeDT_SetInitial(Encoder_T * p_encoder)
{
	Encoder_DeltaD_SetInitial(p_encoder);
	Encoder_DeltaT_SetInitial(p_encoder);
	p_encoder->DeltaTh = 0U;
	p_encoder->FreqD = 0;
}

// ((uint32_t)60U * 65536UL / (p_encoder->Params.CountsPerRevolution * p_encoder->Params.ScalarSpeedRef_Rpm);
static void ResetUnitsScalarSpeed(Encoder_T * p_encoder)
{
	p_encoder->UnitScalarSpeed = (uint32_t)60U * 65536U / p_encoder->Params.CountsPerRevolution; // p_encoder->Params.ScalarSpeedRef_Rpm;
	// uint32_t freqDMax = p_encoder->Params.ScalarSpeedRef_Rpm * 2U * p_encoder->Params.CountsPerRevolution / 60U;
	// // freq max = 204,800
 	// p_encoder->UnitScalarSpeed_Shift = GetMaxShift_Signed((uint64_t)60U * 65536U / p_encoder->Params.CountsPerRevolution * freqDMax / p_encoder->Params.ScalarSpeedRef_Rpm);
	// // 131,072
	// p_encoder->UnitScalarSpeed = ((uint64_t)60U * (uint64_t)65536U) << (uint64_t)p_encoder->UnitScalarSpeed_Shift / ((uint64_t)p_encoder->Params.CountsPerRevolution * (uint64_t)p_encoder->Params.ScalarSpeedRef_Rpm);
}

void Encoder_ModeDT_SetCountsPerRevolution(Encoder_T * p_encoder, uint16_t countsPerRevolution)
{
	p_encoder->Params.CountsPerRevolution = countsPerRevolution;
	_Encoder_ResetUnitsAngular(p_encoder);
	_Encoder_ResetUnitsInterpolateAngle(p_encoder);
	ResetUnitsScalarSpeed(p_encoder);
}

void Encoder_ModeDT_SetScalarSpeedRef(Encoder_T * p_encoder, uint16_t speedRef)
{
	p_encoder->Params.ScalarSpeedRef_Rpm = speedRef;
	ResetUnitsScalarSpeed(p_encoder);
}
