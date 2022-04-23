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
#include "SinCos.h"
#include "Config.h"

#include <string.h>

void SinCos_Init(SinCos_T * p_sincos)
{
	if (p_sincos->CONFIG.P_PARAMS != 0U)
	{
		memcpy(&p_sincos->Params, p_sincos->CONFIG.P_PARAMS, sizeof(SinCos_Params_T));
		Linear_ADC_Init(&p_sincos->UnitsAngle, p_sincos->Params.Zero_ADCU, p_sincos->Params.Max_ADCU, p_sincos->Params.Max_MilliV);
	}
	else
	{
		Linear_ADC_Init(&p_sincos->UnitsAngle, 2048U, 4095U, 5000U);
	}
}

void SinCos_SetParamsAdc(SinCos_T * p_sincos, uint16_t zero_ADCU, uint16_t max_ADCU, uint16_t max_MilliV)
{
	p_sincos->Params.Zero_ADCU = zero_ADCU;
	p_sincos->Params.Max_ADCU = max_ADCU;
	p_sincos->Params.Max_MilliV = max_MilliV;
	Linear_ADC_Init(&p_sincos->UnitsAngle, zero_ADCU, max_ADCU, max_MilliV);
}

void SinCos_SetParamsAdc_MilliV(SinCos_T * p_sincos, uint16_t min_MilliV, uint16_t max_MilliV) //todo adcrefmv
{
	SinCos_SetParamsAdc(p_sincos, (uint32_t)(max_MilliV + min_MilliV) * ADC_MAX / 2U / ADC_VREF_MILLIV, (uint32_t)max_MilliV * ADC_MAX / ADC_VREF_MILLIV, max_MilliV);
}

/*
 *
 */
void SinCos_SetERotationsPerCycle(SinCos_T * p_sincos, uint16_t polePairs, uint16_t cyclesPerMRotation)
{
	p_sincos->Params.ElectricalRotationsPerCycle = polePairs / cyclesPerMRotation;
}

/*
 * run on measure 0
 */
void SinCos_CalibrateAngleOffset(SinCos_T * p_sincos, uint16_t sin_ADCU, uint16_t cos_ADCU)
{
	p_sincos->Params.AngleOffet = 0;
	p_sincos->Params.AngleOffet = SinCos_CalcAngle(p_sincos, sin_ADCU, cos_ADCU);
}

/*
 * Call after angle offset
 */
void SinCos_CalibrateCcwPositive(SinCos_T * p_sincos, uint16_t sin_ADCU, uint16_t cos_ADCU)
{
	p_sincos->Params.IsBPositive = (SinCos_CalcAngle(p_sincos, sin_ADCU, cos_ADCU) > 0U) ? true : false;
}

void SinCos_CalibrateA(SinCos_T * p_sincos, uint16_t sin_ADCU, uint16_t cos_ADCU)
{
	SinCos_CalibrateAngleOffset(p_sincos, sin_ADCU, cos_ADCU);
}

void SinCos_CalibrateB(SinCos_T * p_sincos, uint16_t sin_ADCU, uint16_t cos_ADCU)
{
	SinCos_CalibrateCcwPositive(p_sincos, sin_ADCU, cos_ADCU);
}
