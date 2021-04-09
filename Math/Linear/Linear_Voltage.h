/*******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSoucery / The Firebrand Forge Inc

	This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).

	This program is free software: you can redistribute it and/or modify
	it under the terupdateInterval of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
/*******************************************************************************/
/*******************************************************************************/
/*!
    @file 	Linear_Voltage.h
    @author FireSoucery
    @brief 	Voltage to ADC values conversion using voltage divider.
    		Calculates and holds ADCU voltage conversion ratios

    @version V0
*/
/*******************************************************************************/
#ifndef LINEAR_VOLTAGE_H
#define LINEAR_VOLTAGE_H

#include "Linear.h"

#include <stdint.h>

/******************************************************************************/
/*!
	@brief Calculate voltage from given ADC value

	@param[in] p_linear - struct containing calculated intermediate values
	@param[in] adcu - ADC value
	@return Calculated voltage
 */
/******************************************************************************/
int32_t Linear_Voltage_CalcV(Linear_T * p_linear, uint16_t adcu)
{
	return Linear_Function(p_linear, adcu); // (adcu*VREF*(R1+R2))/(ADC_MAX*R2);
};

int32_t Linear_Voltage_CalcMilliV(Linear_T * p_linear, uint16_t adcu)
{
	return Linear_Function(p_linear, adcu*1000);
};

//uint32_t Linear_Voltage_CalcVDigits(VOLTAGE_DIVIDER_T * div, uint16_t adcRaw, uint8_t digits)
//{
//	return ((adcRaw*div->VPerADCFactor * (10^digits)) / div->VPerADCDivisor);
//}

/******************************************************************************/
/*!
	@brief Calculate ADC value from given voltage

	@param[in] linear - struct containing calculated intermediate values
	@param[in] voltage - voltage
	@return Calculated ADC value
 */
/******************************************************************************/
uint16_t Linear_Voltage_CalcADCU(Linear_T * p_linear, uint16_t volts)
{
	return (uint16_t) Linear_InvFunction(p_linear, volts);
}

uint16_t Linear_Voltage_CalcADCU_MilliV(Linear_T * p_linear, uint16_t milliV)
{
	return (uint16_t) (Linear_InvFunction(p_linear, milliV) / 1000);
}


extern void Linear_Voltage_Init(Linear_T * p_linear, uint16_t r1, uint16_t r2, uint8_t vRefFactor, uint8_t vRefDivisor, uint8_t adcBits);

#endif
