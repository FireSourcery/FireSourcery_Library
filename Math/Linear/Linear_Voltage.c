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
    @file 	Linear_Voltage.c
    @author FireSoucery
    @brief  Linear_Voltage module conventional function definitions
    @version V0
*/
/*******************************************************************************/
#include "Linear_Voltage.h"

#include "Linear.h"

#include <stdint.h>

/******************************************************************************/
/*!
	@brief Initialize Linear struct using Voltage Divider parameters.
	Struct contains numerator and denominator of ADC to Vin conversion factor.

	VDIV = VIN*(R2/(R1+R2))
	DIV = (R2/(R1+R2))
	VADC_RES = VREF/ADC_MAX

	ADC = VIN*DIV/VADC_RES = VIN*(R2*ADC_MAX)/((R1+R2)*VREF)
	VIN = ADC*VADC_RES/DIV = ADC*(VREF*(R1+R2))/(ADC_MAX*R2)
	VIN/ADC = VADC_RES/DIV = VREF*(R1 + R2)/(ADC_MAX*R2)

	@param[in] line - Struct containing calculated intermediate values
	@param[in] r1 - R1 value expressed as a whole number
	@param[in] r2 - R2 value expressed as a whole number
	@param[in] vRef - reference voltage
	@param[in] adcBit - Number of ADC bits
 */
/******************************************************************************/
void Linear_Voltage_Init(Linear_T * p_linear, uint16_t r1, uint16_t r2, uint8_t vRefFactor, uint8_t vRefDivisor, uint8_t adcBits)
{
	/*
	 * 	Init as Linear_Function(adcu) == voltage
	 *  VPerADCFactor = vRef * (r1 + r2);
	 *	VPerADCDivisor = (2^adcBits - 1) * r2;
	 */
#ifdef CONFIG_LINEAR_SHIFT_DIVIDE
	p_linear->SlopeFactor = ((vRefFactor * (r1 + r2) ) << (16U - adcBits)) / r2 / vRefDivisor; 	// (VREF*(R1 + R2) << 16)/(ADC_MAX*R2)
	p_linear->SlopeFactorShift = 16U;
	p_linear->SlopeDivisor = ((r2 << 16U) / (vRefFactor * (r1 + r2)) / vRefDivisor);			// ((R2) << 16)/(VREF*(R1 + R2))
	p_linear->SlopeDivisorShift = 16U - adcBits;
 	p_linear->Offset = 0;
#elif defined (CONFIG_LINEAR_NUMIRICAL_DIVIDE)
	p_linear->SlopeFactor = vRefFactor * (r1 + r2);									// (VREF*(R1+R2))
	p_linear->SlopeDivisor = (((uint32_t)1U << adcBits) - 1U) * r2 * vRefDivisor; 	// (ADC_MAX*R2)
	p_linear->Offset = 0;
#endif
}

//void Linear_Voltage_Init_Frac16(Linear_T * p_linear, uint32_t r1, uint32_t r2, uint8_t vRef, uint16_t adcBits, )
//{
////	linear->V100Percent;
////	linear->V0Percent;
////	linear->VPercentPerADCU;
////	linear->V0PercentADCU;
//}
