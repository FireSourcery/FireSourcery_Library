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
#include "SinCos.h"
#include "Config.h"

#include "Peripheral/Analog/Analog/Global_Analog.h"
#include <string.h>

void SinCos_Init(SinCos_T * p_sincos)
{
	if(p_sincos->CONFIG.P_PARAMS != 0U)
	{
		memcpy(&p_sincos->Params, p_sincos->CONFIG.P_PARAMS, sizeof(SinCos_Params_T));
		SinCos_ResetUnitsAngle(p_sincos);
	}
}

void SinCos_ResetUnitsAngle(SinCos_T * p_sincos)
{
	Linear_ADC_Init(&p_sincos->UnitsAngle, p_sincos->Params.Min_Adcu, p_sincos->Params.Max_Adcu, p_sincos->Params.Min_MilliV, p_sincos->Params.Max_MilliV);
}

void SinCos_SetParamsAdc(SinCos_T * p_sincos, uint16_t min_Adcu, uint16_t max_Adcu, uint16_t min_mV, uint16_t max_mV)
{
	p_sincos->Params.Min_Adcu = min_Adcu;
	p_sincos->Params.Max_Adcu = max_Adcu;
	p_sincos->Params.Min_MilliV = min_mV;
	p_sincos->Params.Max_MilliV = max_mV;
	SinCos_ResetUnitsAngle(p_sincos);
}

void SinCos_SetParamsAdc_mV(SinCos_T * p_sincos, uint16_t adcVref_mV, uint16_t min_mV, uint16_t max_mV)
{
	uint16_t min_Adcu = (uint32_t)min_mV * GLOBAL_ANALOG.ADC_MAX / adcVref_mV;
	uint16_t max_Adcu = (uint32_t)max_mV * GLOBAL_ANALOG.ADC_MAX / adcVref_mV;
	SinCos_SetParamsAdc(p_sincos, min_Adcu, max_Adcu, min_mV, max_mV);
}

void SinCos_SetAngleRatio(SinCos_T * p_sincos, uint16_t polePairs)
{
	p_sincos->Params.ElectricalRotationsRatio = polePairs;
}

void SinCos_CalibrateAngleOffset(SinCos_T * p_sincos, uint16_t sin_Adcu, uint16_t cos_Adcu)
{
	// p_sincos->DebugAPre = _SinCos_CalcAngle(p_sincos, sin_Adcu, cos_Adcu);	//debug //check == 0

	p_sincos->Params.AngleOffet = _SinCos_CalcAngle(p_sincos, sin_Adcu, cos_Adcu);
}

/*
	Call after angle offset
*/
void SinCos_CalibrateCcwPositive(SinCos_T * p_sincos, uint16_t sin_Adcu, uint16_t cos_Adcu)
{
	// p_sincos->DebugBPre = _SinCos_CalcAngle(p_sincos, sin_Adcu, cos_Adcu);	//debug //check == 120

	p_sincos->Params.IsCcwPositive = (_SinCos_CalcAngle(p_sincos, sin_Adcu, cos_Adcu) > 0);
}

void SinCos_CalibrateA(SinCos_T * p_sincos, uint16_t sin_Adcu, uint16_t cos_Adcu)
{
	SinCos_CalibrateAngleOffset(p_sincos, sin_Adcu, cos_Adcu);
}

void SinCos_CalibrateB(SinCos_T * p_sincos, uint16_t sin_Adcu, uint16_t cos_Adcu)
{
	SinCos_CalibrateCcwPositive(p_sincos, sin_Adcu, cos_Adcu);
}
