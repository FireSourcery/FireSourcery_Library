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

typedef struct
{
	uint32_t RNominal;
	uint32_t TNominal;
	uint32_t BConstant;

	uint16_t Limit_ADCU;
	uint16_t Threshold_ADCU;
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

	Thermistor_Params_T * P_PARAMS; //nv memory

	uint8_t V_IN;
	uint8_t V_AREF;
	uint16_t ADC_MAX;
	uint32_t R_SERIES; 	//pull up
//	uint32_t R_PARALLEL; //pull down
}
Thermistor_Config_T;

typedef struct
{
	const Thermistor_Config_T CONFIG;
	Thermistor_Params_T Params;

	Thermistor_ThesholdStatus_T Status;
}
Thermistor_T;

//#define THERMISTOR_CONFIG(AdcMax, RSeries, Vin, Varef, RNominal, TNominal, BConst) 	\
//{												\
//	.CONFIG =									\
//	{											\
//		.ADC_MAX 		= AdcRes,				\
//		.R_SERIES 		= RSeries,				\
//		.V_IN 			= Vin,					\
//		.V_AREF 		= Varef,				\
//		.R0 			= RNominal,				\
//		.T0 			= TNominal,				\
//		.B 				= BConst,				\
//	}											\
//}

extern float Thermistor_ConvertToDegreesC_Float(Thermistor_T * p_thermistor, uint16_t adcu);
extern uint32_t Thermistor_SetHeatLimit_DegreesC(Thermistor_T * p_thermistor, uint8_t limit_degreesC);
extern uint32_t Thermistor_SetHeatThreshold_DegreesC(Thermistor_T * p_thermistor, uint8_t threshold_degreesC);
#endif
