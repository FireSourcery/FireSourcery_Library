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
	@file 	SinCos.h
	@author FireSourcery
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

typedef struct __attribute__((aligned(2U)))
{
	uint16_t Zero_Adcu;
	uint16_t Max_Adcu;
	uint16_t Max_MilliV;
	uint16_t ElectricalRotationsRatio; /* = PolePairs / CyclesPerRotation */
//	uint16_t CyclePerMechRotation;
	qangle16_t AngleOffet;
	bool IsCcwPositive;		/* Calibrates Ccw as positive */
}
SinCos_Params_T;

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
	qangle16_t Angle; /* Sensor Output Angle, Mechanical Angle or proportional */
	// bool IsDirectionPositive;

	qangle16_t DebugAPre;
	qangle16_t DebugBPre;
	qangle16_t DebugAPostMech;
	qangle16_t DebugBPostMech;
	qangle16_t DebugAPostElec;
	qangle16_t DebugBPostElec;
}
SinCos_T;

#define SIN_COS_INIT(p_Params) { .CONFIG = { .P_PARAMS = p_Params, } }

/*
	Activate Adc outside module
*/
static inline qangle16_t _SinCos_CalcAngle(SinCos_T * p_sincos, uint16_t sin_Adcu, uint16_t cos_Adcu)
{
	qfrac16_t sin = Linear_ADC_CalcFractionSigned16(&p_sincos->UnitsAngle, sin_Adcu);
	qfrac16_t cos = Linear_ADC_CalcFractionSigned16(&p_sincos->UnitsAngle, cos_Adcu);
	qangle16_t angle = qfrac16_atan2(sin, cos);
	return angle;
}

/*
	Calibration set to return CCW as positive
*/
static inline qangle16_t SinCos_CaptureAngle(SinCos_T * p_sincos, uint16_t sin_Adcu, uint16_t cos_Adcu)
{
	qangle16_t angle = _SinCos_CalcAngle(p_sincos, sin_Adcu, cos_Adcu);
	angle = angle - p_sincos->Params.AngleOffet; /* move to sinCos calc */
	if(p_sincos->Params.IsCcwPositive == false) { angle = 0 - angle; };
	p_sincos->Angle = angle; //need counter to add offset if multiple cycles per rotation
	return angle;
}

static inline qangle16_t SinCos_GetMechanicalAngle(SinCos_T * p_sincos) { return p_sincos->Angle; }
/* effectively modulus angle max */
static inline qangle16_t SinCos_GetElectricalAngle(SinCos_T * p_sincos) { return (qangle16_t)((int32_t)p_sincos->Angle * p_sincos->Params.ElectricalRotationsRatio); }

/*
	CCW is positive
*/
// static inline void SinCos_SetDirectionCcw(SinCos_T * p_sincos) 	{ p_sincos->IsDirectionPositive = p_sincos->Params.IsCcwPositive; }
// static inline void SinCos_SetDirectionCw(SinCos_T * p_sincos) 	{ p_sincos->IsDirectionPositive = !p_sincos->Params.IsCcwPositive; }

/*
	Extern declarations
*/
extern void SinCos_Init(SinCos_T * p_sincos);
extern void SinCos_SetParamsAdc(SinCos_T * p_sincos, uint16_t zero_Adcu, uint16_t max_Adcu, uint16_t max_mV);
extern void SinCos_SetParamsAdc_mV(SinCos_T * p_sincos, uint16_t adcVref_mV, uint16_t min_mV, uint16_t max_mV);
extern void SinCos_SetAngleRatio(SinCos_T * p_sincos, uint16_t polePairs);
// extern void SinCos_SetERotationsPerCycle_(SinCos_T * p_sincos, uint16_t polePairs, uint16_t cyclesPerMRotation);
extern void SinCos_CalibrateAngleOffset(SinCos_T * p_sincos, uint16_t sin_Adcu, uint16_t cos_Adcu);
extern void SinCos_CalibrateCcwPositive(SinCos_T * p_sincos, uint16_t sin_Adcu, uint16_t cos_Adcu);
extern void SinCos_CalibrateA(SinCos_T * p_sincos, uint16_t sin_Adcu, uint16_t cos_Adcu);
extern void SinCos_CalibrateB(SinCos_T * p_sincos, uint16_t sin_Adcu, uint16_t cos_Adcu);

#endif

