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

//#include "Peripheral/Pin/Pin.h"
//#include "Utility/Timer/Timer.h"

/*
 * 1/T = 1/T0 + (1/B)ln(R/R0))
 */
typedef struct
{
//#ifdef CONFIG_THERMISTOR_ADC_LUT
//	uint8_t * ADC_LUT; //look up table using adc base, fixed r0 value
//	uint8_t * R_LUT;
	uint16_t * P_ADC_VALUE;
	uint16_t ADC_MAX;
	uint32_t R_SERIES;

	uint8_t V_IN;
	uint8_t ADC_VREF;

	uint32_t R0;
	uint32_t T0;
	uint16_t B;
}
Thermistor_Config_T;

typedef struct
{
	const Thermistor_Config_T CONFIG;

	//todo runtime config
	uint32_t RNominal;
	uint32_t TNominal;
	uint16_t BConstant;

	uint32_t DegreesC;
}
Thermistor_T;

#define THERMISTOR_CONFIG(p_Adcu, AdcRes, RSeries, Vin, AdcVref, RNominal, TNominal, BConst) 	\
{												\
	.CONFIG =									\
	{											\
		.P_ADC_VALUE 	= p_Adcu,				\
		.ADC_MAX 		= AdcRes,				\
		.R_SERIES 		= RSeries,				\
		.V_IN 			= Vin,					\
		.ADC_VREF 		= AdcVref,				\
		.R0 			= RNominal,				\
		.T0 			= TNominal,				\
		.B 				= BConst,				\
	}											\
}

#endif
