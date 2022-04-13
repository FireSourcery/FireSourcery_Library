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

//#include <stdbool.h>
//#include <stdint.h>
#include <string.h>

void SinCos_Init(SinCos_T * p_sincos)
{
	if (p_sincos->CONFIG.P_PARAMS != 0U)
	{
		memcpy(&p_sincos->Params, p_sincos->CONFIG.P_PARAMS, sizeof(SinCos_Params_T));
//		Linear_ADC_Init(&p_sincos->UnitSin, p_sincos->Params.SinZero_ADCU, p_sincos->Params.SinMax_ADCU, p_sincos->Params.SinMax_MilliV);
//		Linear_ADC_Init(&p_sincos->UnitCos, p_sincos->Params.CosZero_ADCU, p_sincos->Params.CosMax_ADCU, p_sincos->Params.CosMax_MilliV);
		Linear_ADC_Init(&p_sincos->UnitConversion, p_sincos->Params.Zero_ADCU, p_sincos->Params.Max_ADCU, p_sincos->Params.Max_MilliV);
	}
	else
	{
//		Linear_ADC_Init(&p_sincos->UnitSin, 2048U, 4095U, 4500U);
//		Linear_ADC_Init(&p_sincos->UnitCos, 2048U, 4095U, 4500U);
		Linear_ADC_Init(&p_sincos->UnitConversion, 2048U, 4095U, 5000U);
	}
}

//void SinCos_CalibrateSin(SinCos_T * p_sincos, uint16_t zero_ADCU, uint16_t max_ADCU, uint16_t max_MilliV)
//{
//	p_sincos->Params.SinZero_ADCU = zero_ADCU;
//	p_sincos->Params.SinMax_ADCU = max_ADCU;
//	p_sincos->Params.SinMax_MilliV = max_MilliV;
//	Linear_ADC_Init(&p_sincos->UnitSin, p_sincos->Params.SinZero_ADCU, p_sincos->Params.SinMax_ADCU, p_sincos->Params.SinMax_MilliV);
//}
//
//
//void SinCos_CalibrateCos(SinCos_T * p_sincos, uint16_t zero_ADCU, uint16_t max_ADCU, uint16_t max_MilliV)
//{
//	p_sincos->Params.CosZero_ADCU = zero_ADCU;
//	p_sincos->Params.CosMax_ADCU = max_ADCU;
//	p_sincos->Params.CosMax_MilliV = max_MilliV;
//	Linear_ADC_Init(&p_sincos->UnitCos, p_sincos->Params.CosZero_ADCU, p_sincos->Params.CosMax_ADCU, p_sincos->Params.CosMax_MilliV);
//}

void SinCos_SetParams(SinCos_T * p_sincos, uint16_t zero_ADCU, uint16_t max_ADCU, uint16_t max_MilliV)
{
	p_sincos->Params.Zero_ADCU = zero_ADCU;
	p_sincos->Params.Max_ADCU = max_ADCU;
	p_sincos->Params.Max_MilliV = max_MilliV;
	Linear_ADC_Init(&p_sincos->UnitConversion, zero_ADCU, max_ADCU, max_MilliV);
}

void SinCos_Calibrate_MilliV(SinCos_T * p_sincos, uint16_t min_MilliV, uint16_t max_MilliV)
{
//	SinCos_SetParams(p_sincos, (uint32_t)(max_MilliV + min_MilliV) * ADC_MAX / 2U / ADC_VREF_MILLIV, (uint32_t)max_MilliV * ADC_MAX / ADC_VREF_MILLIV, max_MilliV);
}

