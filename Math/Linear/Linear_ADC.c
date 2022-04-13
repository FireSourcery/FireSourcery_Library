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


//@input - physicalRef < 32768, 65536, resolution of initial y offset
//			set > adcuResolution for max res
//
//e.g
//adcuZero 		= 2000
//adcuRef 		= 4095
//physicalRef 	= 100
//factor 		= 100
//divisor 	= 2095
//
//x0 with shifted b
//	m_shiftedFactor 		= 3128
//	m_shiftedDivisor 		= 1372979
//	Intercept	 			= -95 =>
//	shift back 		-6,225,920 (error 300) (res 32)
//	save shifted 	-6,256,420	=> (error - 4) (res 27)
//
//e.g
//adcuZero 		= 2000
//adcuRef 		= 4095
//physicalRef 	= 1000
//factor 		= 1000
//divisor 	= 2095
//
//x0 with shifted b
//	m_shiftedFactor 		= 31282
//	m_shiftedDivisor 		= 137297
//	Intercept	 		= -954
//	shift back 	-62,521,344 (error 42)
//	preshift  	-62,564,200 (error 0)
//
//
//adcuZero 		= 2000
//adcuRef 		= 4095
//physicalRef 	= 10000
//factor 			= 10000
//divisor 		= 2095
//
//x0 with shifted b
//	m_shiftedFactor 		= 312821
//	m_shiftedDivisor 		= 13729
//	Intercept	 		= -9546
//	shift back 		-625,606,656 (3) (res 34)
//	save shifted  	-625,642,004 (error0)
