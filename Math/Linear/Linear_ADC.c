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
    @file 	Linear_ADC.c
    @author FireSoucery
    @brief  Scale ADCU to provided reference value
    @version V0
*/
/******************************************************************************/
#include "Linear_ADC.h"
#include "Linear.h"

#include <stdint.h>

/******************************************************************************/
/*!
	f(adcu) = physical
  	f16(adcu) = adc_frac16

	f(adcuZero) = 0
	f(adcuRef) = physicalRef
	f16(adcuRef) = 65535

	frac16 conversion returns without division, as frac16 calc is performed more frequently
	division in frac16 to physical units
	shift 14 to allow frac16 oversaturation [-2:~2] instead of [-1:~1]
 */
/******************************************************************************/
void Linear_ADC_Init(Linear_T * p_linear, uint16_t adcuZero, uint16_t adcuRef, int16_t physicalRef)
{
	p_linear->SlopeFactor 			= (65536 << 14U) / (adcuRef - adcuZero);
	p_linear->SlopeDivisor_Shift 	= 14U;
	p_linear->SlopeDivisor 			= ((adcuRef - adcuZero) << 14U) / 65536;
	p_linear->SlopeFactor_Shift 	= 14U;
	p_linear->XOffset 				= adcuZero;
	p_linear->YOffset 				= 0;
	p_linear->XReference 			= adcuRef;
	p_linear->YReference 			= physicalRef;
}

void Linear_ADC_Init_Inverted(Linear_T * p_linear, uint16_t adcuZero, uint16_t adcuRef, int16_t physicalRef)
{
	Linear_ADC_Init(p_linear, adcuZero, adcuRef, physicalRef);
	p_linear->SlopeFactor 	= 0 - p_linear->SlopeFactor;
	p_linear->SlopeDivisor 	= 0 - p_linear->SlopeDivisor;
}

