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
	@file 	SinCos.h
	@author FireSoucery
	@brief
	@version V0
*/
/******************************************************************************/
#ifndef SIN_COS_H
#define SIN_COS_H

#include "Math/Linear/Linear_ADC.h"
#include "Math/Q/QFrac16.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct __attribute__((aligned(4U)))
{
	uint16_t Zero_Adcu;
	uint16_t Max_Adcu;
	uint16_t Max_MilliV;
	uint16_t ElectricalRotationsPerCycle; /* =  PolePairs / CyclesPerRotation */
//	uint16_t CyclePerMechRotation;
	qangle16_t AngleOffet;
	bool IsBPositive;		/* CCW is fixed to positive */
}
SinCos_Params_T;

/*
	Activate Adc outside module
*/
typedef const struct
{
	const SinCos_Params_T * const P_PARAMS;
}
SinCos_Config_T;

typedef struct
{
	SinCos_Config_T CONFIG;
	SinCos_Params_T Params;
	Linear_T UnitsAngle;
	bool IsDirectionPositive;
	//	qangle16_t AngleOffet;

	qangle16_t ElectricalAngle;
	qangle16_t MechanicalAngle;
}
SinCos_T;

#define SIN_COS_CONFIG(p_Params)				\
{												\
	.CONFIG = 									\
	{											\
		.P_PARAMS = p_Params, 					\
	}											\
}

static inline qangle16_t SinCos_CalcAngle(SinCos_T * p_sincos, uint16_t sin_Adcu, uint16_t cos_Adcu)
{
	qfrac16_t sin = Linear_ADC_CalcFractionSigned16(&p_sincos->UnitsAngle, sin_Adcu);
	qfrac16_t cos = Linear_ADC_CalcFractionSigned16(&p_sincos->UnitsAngle, cos_Adcu);
	qangle16_t angle = qfrac16_atan2(sin, cos);

	angle = (int32_t)angle - p_sincos->Params.AngleOffet;
	//	angle = (qangle16_t)((int32_t)angle * p_sincos->Params.ElectricalRotationsPerCycle); /* effectively modulus angle max */

	//	if (p_sincos->IsDirectionPositive == false) {angle = 0 - angle;};

	return angle;
}

static inline qangle16_t SinCos_CaptureAngle(SinCos_T * p_sincos, uint16_t sin_Adcu, uint16_t cos_Adcu)
{
	qangle16_t angle = SinCos_CalcAngle(p_sincos, sin_Adcu, cos_Adcu);
	p_sincos->MechanicalAngle = angle; //need counter to add offset if multiple cycles per rotation
	return angle;
}

static inline qangle16_t SinCos_GetMechanicalAngle(SinCos_T * p_sincos) { return p_sincos->MechanicalAngle; }
static inline qangle16_t SinCos_GetElectricalAngle(SinCos_T * p_sincos) { return (qangle16_t)((int32_t)p_sincos->MechanicalAngle * p_sincos->Params.ElectricalRotationsPerCycle); }

/*
	CCW is positive
*/
static inline void SinCos_SetDirectionCcw(SinCos_T * p_sincos) 	{ p_sincos->IsDirectionPositive = p_sincos->Params.IsBPositive; }
static inline void SinCos_SetDirectionCw(SinCos_T * p_sincos) 	{ p_sincos->IsDirectionPositive = !p_sincos->Params.IsBPositive; }

/*
	Extern declarations
*/
extern void SinCos_Init(SinCos_T * p_sincos);
extern void SinCos_SetParamsAdc(SinCos_T * p_sincos, uint16_t zero_Adcu, uint16_t max_Adcu, uint16_t max_mV);
extern void SinCos_SetParamsAdc_mV(SinCos_T * p_sincos, uint16_t adcVref_mV, uint16_t min_mV, uint16_t max_mV);
extern void SinCos_CalibrateAngleOffset(SinCos_T * p_sincos, uint16_t sin_Adcu, uint16_t cos_Adcu);
extern void SinCos_CalibrateCcwPositive(SinCos_T * p_sincos, uint16_t sin_Adcu, uint16_t cos_Adcu);
extern void SinCos_CalibrateA(SinCos_T * p_sincos, uint16_t sin_Adcu, uint16_t cos_Adcu);
extern void SinCos_CalibrateB(SinCos_T * p_sincos, uint16_t sin_Adcu, uint16_t cos_Adcu);

#endif

