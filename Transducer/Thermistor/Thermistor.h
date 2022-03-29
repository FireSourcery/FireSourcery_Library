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
	@file  	Blinky.h
	@author FireSourcery
	@brief 	Pin Indicator

	@version V0
 */
/******************************************************************************/
#ifndef THERMISTOR_H
#define THERMISTOR_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

typedef enum
{
	THERMISTOR_THRESHOLD_OK,
	THERMISTOR_ERROR_LIMIT,
	THERMISTOR_ERROR_THRESHOLD,
}
Thermistor_ThesholdStatus_T;

typedef struct __attribute__((aligned (4U)))
{
	uint32_t RNominal;
	uint32_t TNominal;
	uint32_t BConstant;

	uint16_t Limit_ADCU;
	uint16_t Threshold_ADCU;

	uint16_t IntScalar;
	bool IsEnable;
}
Thermistor_Params_T;

/*
 * 1/T = 1/T0 + (1/B)ln(R/R0))
 */
typedef struct
{
//#ifdef CONFIG_THERMISTOR_ADC_LUT
//	uint8_t * ADC_LUT; //look up table using adc base, fixed r0 value
//	uint8_t * R_LUT;

//#ifdef config pointer
//	uint16_t * P_ADC_VALUE;

	const Thermistor_Params_T * P_PARAMS;

	uint8_t ADC_VREF;
	uint16_t ADC_MAX;

	uint8_t V_IN;
	uint32_t R_SERIES; 	//pull up
	uint32_t R_PARALLEL; // parallel if applicable
}
Thermistor_Config_T;

typedef struct
{
	const Thermistor_Config_T CONFIG;
	Thermistor_Params_T Params;

	Thermistor_ThesholdStatus_T Status;
	int32_t Heat_DegCInt;
}
Thermistor_T;


#define THERMISTOR_CONFIG(AdcVref, AdcMax, Vin, RSeries, RParallel, p_Params) 	\
{												\
	.CONFIG =									\
	{											\
		.ADC_VREF 		= AdcVref,				\
		.ADC_MAX 		= AdcMax,				\
		.V_IN 			= Vin,					\
		.R_SERIES 		= RSeries,				\
		.R_PARALLEL		= RParallel,			\
		.P_PARAMS		= p_Params,				\
	}											\
}

static inline Thermistor_ThesholdStatus_T Thermistor_GetStatus(Thermistor_T * p_thermistor)
{
	return p_thermistor->Status;
}

static inline bool Thermistor_GetIsEnable(Thermistor_T * p_thermistor)
{
	return p_thermistor->Params.IsEnable;
}

static inline int32_t Thermistor_GetHeat_DegCInt(Thermistor_T * p_thermistor)
{
	return p_thermistor->Heat_DegCInt;
}

#endif
