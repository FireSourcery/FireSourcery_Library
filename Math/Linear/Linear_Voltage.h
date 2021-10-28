/*******************************************************************************/
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


void Linear_Voltage_Init(Linear_T * p_linear, uint16_t r1, uint16_t r2, uint8_t adcVRef10, uint8_t adcBits, uint16_t vInMax)
{
	/*
	 * 	Init as Linear_Function(adcu) == voltage
	 *  VPerADCFactor = vRef * (r1 + r2);
	 *	VPerADCDivisor = (2^adcBits - 1) * r2;
	 */
#ifdef CONFIG_LINEAR_SHIFT_DIVIDE
	p_linear->SlopeFactor = ((adcVRef10 * (r1 + r2)) << (16U - adcBits)) / r2 / 10; 	// (VREF*(R1 + R2) << 16)/(ADC_MAX*R2)
	p_linear->SlopeDivisor_Shift = 16U;
	p_linear->SlopeDivisor = ((r2 << 16U) / (adcVRef10 * (r1 + r2)) / 10);				// ((R2) << 16)/(VREF*(R1 + R2))
	p_linear->SlopeFactor_Shift = 16U - adcBits;
#elif defined (CONFIG_LINEAR_NUMIRICAL_DIVIDE)
	p_linear->SlopeFactor = adcVRef10 * (r1 + r2);									// (VREF*(R1+R2))
	p_linear->SlopeDivisor = (((uint32_t)1U << adcBits) - 1U) * r2 * 10; 			// (ADC_MAX*R2)
#endif

	p_linear->Offset = 0U;
 	p_linear->RangeReference = vInMax - 0U;
}

#define LINEAR_VOLTAGE_CONFIG(r1, r2, adcVRef10, adcBits, vInMax) 			\
{																			\
	.SlopeFactor = ((adcVRef10 * (r1 + r2)) << (16U - adcBits)) / r2 / 10;  \
}


/******************************************************************************/
/*!
	@brief Calculate voltage from given ADC value

	@param[in] p_linear - struct containing calculated intermediate values
	@param[in] adcu - ADC value
	@return Calculated voltage
 */
/******************************************************************************/
static inline uint32_t Linear_Voltage_CalcV(Linear_T * p_linear, uint16_t adcu)
{
	return Linear_Function(p_linear, adcu); // (adcu*VREF*(R1+R2))/(ADC_MAX*R2);
}

static inline uint32_t Linear_Voltage_CalcMilliV(Linear_T * p_linear, uint16_t adcu)
{
	return Linear_Function(p_linear, adcu*1000);
}

static inline uint32_t Linear_Voltage_CalcV_NDecimal(Linear_T * p_linear, uint16_t adcu, uint8_t nDigits)
{
	return Linear_Function(p_linear, adcu*(10^nDigits));
}

/******************************************************************************/
/*!
	@brief 	results expressed in Q1.15 where 32,767 ~= 100%
 */
/******************************************************************************/
/******************************************************************************/
/*!
	@brief 	results expressed where 65356 = 100%,
			unbounded where > 65536 is > 100%
 */
/******************************************************************************/
static inline uint32_t Linear_Voltage_CalcFraction16(Linear_T * p_linear, uint16_t adcu)
{
	return Linear_Function_UnsignedFraction16(p_linear, adcu);
}
//	Linear_Voltage_ConvertAdcuToFractionUnsigned16()
static inline uint16_t Linear_Voltage_CalcUnsignedFraction16(Linear_T * p_linear, uint16_t adcu)
{
	return Linear_Function_UnsignedFraction16(p_linear, adcu);
}

/******************************************************************************/
/*!
	@brief Calculate ADC value from given voltage

	@param[in] linear - struct containing calculated intermediate values
	@param[in] voltage - voltage
	@return Calculated ADC value
 */
/******************************************************************************/
static inline uint16_t Linear_Voltage_CalcAdcu(Linear_T * p_linear, uint16_t volts)
{
	return (uint16_t) Linear_InvFunction(p_linear, volts);
}

static inline uint16_t Linear_Voltage_CalcAdcu_MilliV(Linear_T * p_linear, uint16_t milliV)
{
	return (uint16_t) (Linear_InvFunction(p_linear, milliV) / 1000); //only when 0 offset
}

//extern void Linear_Voltage_Init(Linear_T * p_linear, uint16_t r1, uint16_t r2, uint8_t vRefFactor, uint8_t vRefDivisor, uint8_t adcBits);

#endif
