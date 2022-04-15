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

typedef struct __attribute__((aligned (4U)))
{
//	uint16_t SinZero_ADCU;
//	uint16_t SinMax_ADCU;
//	uint16_t SinMax_MilliV;
//	uint16_t CosZero_ADCU;
//	uint16_t CosMax_ADCU;
//	uint16_t CosMax_MilliV;

	uint16_t Zero_ADCU;
	uint16_t Max_ADCU;
	uint16_t Max_MilliV;
	qangle16_t AngleOffet;
}
SinCos_Params_T;

/*
 * Activate Adc outside module
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
	Linear_T UnitConversion;
//	Linear_T UnitSin;
//	Linear_T UnitCos;
}
SinCos_T;

#define SIN_COS_CONFIG(p_Params)								\
{																\
	.CONFIG = 													\
	{															\
		.P_PARAMS 		= p_Params, 							\
	}															\
}

static inline qangle16_t SinCos_CalcAngle(SinCos_T * p_sincos, uint16_t sin_ADCU, uint16_t cos_ADCU)
{
	qfrac16_t sin = Linear_ADC_CalcFractionSigned16(&p_sincos->UnitConversion, sin_ADCU);
	qfrac16_t cos = Linear_ADC_CalcFractionSigned16(&p_sincos->UnitConversion, cos_ADCU);

	return qfrac16_atan2(sin, cos);
}

#endif

