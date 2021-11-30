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
	@file  	Thermistor.c
	@author FireSourcery
	@brief
	@version V0
 */
/******************************************************************************/
#include "Thermistor.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

void Thermistor_Proc(Thermistor_T * p_thermistor)
{
	/*
	 * Thermistor wired as bottom resistor R2
	 */
	// R2 = VOUT*R1 /(VIN-VOUT)
	// R2 = ((ADC*VREF)/ADC_MAX)*R1 / (VIN-((ADC*VREF)/ADC_MAX))
	// R2 = R1/(VIN*ADC_MAX/(ADC*VREF)-1)
	// R2 =	(R1*ADC*VREF)/(VIN*ADC_MAX - ADC*VREF)

//	uint32_t rThermistor = p_thermistor->CONFIG.R_SERIES / (((uint32_t)p_thermistor->CONFIG.V_IN * (uint32_t)p_thermistor->CONFIG.ADC_RESOLUTION) / (*p_thermistor->CONFIG.P_ADC_VALUE * p_thermistor->CONFIG.ADC_VREF) - 1U);
	uint32_t rThermistor = ((uint32_t)*p_thermistor->CONFIG.P_ADC_VALUE * (uint32_t)p_thermistor->CONFIG.R_SERIES * (uint32_t)p_thermistor->CONFIG.ADC_VREF) /
							((uint32_t)p_thermistor->CONFIG.V_IN * (uint32_t)p_thermistor->CONFIG.ADC_MAX - (uint32_t)*p_thermistor->CONFIG.P_ADC_VALUE * p_thermistor->CONFIG.ADC_VREF);


	float tSteinhart = 1.0F / (log((double)rThermistor / (double)p_thermistor->CONFIG.R0) / (double)p_thermistor->CONFIG.B + 1.0F / (double)p_thermistor->CONFIG.T0) - 273.15F;

	p_thermistor->DegreesC = (uint32_t)(tSteinhart*100U);
}

uint32_t Thermistor_GetDegreesC(Thermistor_T * p_thermistor)
{
	return p_thermistor->DegreesC;
}
