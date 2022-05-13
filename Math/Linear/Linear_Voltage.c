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
    @brief
    @version V0
*/
/*******************************************************************************/
#include "Linear_Voltage.h"

/******************************************************************************/
/*!
	@brief Initialize Linear struct using Voltage Divider parameters.

	f(adcu) = voltage
	f16(adcu) = frac16
	invf(voltage) = adcu
	invf16(frac16)  = adcu

	VDIV = VIN*(R2/(R1+R2))
	DIV = (R2/(R1+R2))
	VADC_RES = VREF/ADC_MAX

	ADC = VIN*DIV/VADC_RES = VIN*(R2*ADC_MAX)/((R1+R2)*VREF)
	VIN = ADC*VADC_RES/DIV = ADC*(VREF*(R1+R2))/(ADC_MAX*R2)
	VIN/ADC = VADC_RES/DIV = VREF*(R1 + R2)/(ADC_MAX*R2)

	VPerADCFactor = vRef * (r1 + r2) / r2;
	VPerADCDivisor = ((int32_t)1 << adcBits);

	Overflow: R2 > 65536
	(uint32_t)adcVRef_MilliV * (r1 + r2)) << (15U - adcBits):
	 ~ r1, r1 = 10000, adcVRef = 5000, shift = 3

	Shift 15. Divider should not return oversaturated value

	@param[in] line - Struct containing calculated intermediate values
	@param[in] r1 - R1 value expressed as a whole number, < 65536
	@param[in] r2 - R2 value expressed as a whole number, < 65536
	@param[in] adcVRef - reference voltage
	@param[in] adcBit - Number of ADC bits
*/
/******************************************************************************/
void Linear_Voltage_Init(Linear_T * p_linear, uint16_t r1, uint16_t r2, uint8_t adcBits, uint16_t adcVRef_MilliV, uint16_t vInMax)
{
#ifdef CONFIG_LINEAR_DIVIDE_SHIFT
	//alternatively call maxleftshift divide
	p_linear->Slope 			= (((uint32_t)adcVRef_MilliV * (r1 + r2)) << (15U - adcBits)) / r2 / 1000U; 	/* (VREF*(R1 + R2) << 16)/(ADC_MAX*R2) */
	p_linear->SlopeShift 		= 15U;
	p_linear->InvSlope 			= ((uint32_t)r2 << 15U) / adcVRef_MilliV * 1000U / (r1 + r2);				/* ((R2) << 16)/(VREF*(R1 + R2)) */
	p_linear->InvSlopeShift 	= 15U - adcBits;
#elif defined (CONFIG_LINEAR_DIVIDE_NUMERICAL)
	p_linear->SlopeFactor 	= adcVRef_MilliV * (r1 + r2) / 1000U;			/* (VREF*(R1+R2)) */
	p_linear->SlopeDivisor 	= (((uint32_t)1UL << adcBits) - 1U) * r2; 		/* (ADC_MAX*R2) */
#endif
	p_linear->XOffset = 0;
	p_linear->YOffset = 0;
 	p_linear->YReference = vInMax; /* Frac16 refernce only */
}

uint16_t Linear_Voltage_CalcAdcu_UserV(const Linear_T * p_linear, uint16_t volts)
{
	uint16_t adcu = Linear_Voltage_CalcAdcu_V(p_linear, volts);
	while((uint16_t)Linear_Voltage_CalcV(p_linear, adcu) < volts) { adcu += 1U; }
	return adcu;
}

uint16_t Linear_Voltage_CalcAdcu_UserMilliV(const Linear_T * p_linear, uint32_t milliV)
{
	uint16_t adcu = Linear_Voltage_CalcAdcu_MilliV(p_linear, milliV);

	while(((uint32_t)Linear_Voltage_CalcMilliV(p_linear, adcu) < milliV) && (adcu < ADC_MAX))
	{
		 adcu += 1U;
	}

	return adcu;
}