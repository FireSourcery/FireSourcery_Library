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
    @file 	Linear_ADC.c
    @author FireSoucery
    @brief  linear function using known measured value adc limits
    @version V0
*/
/*******************************************************************************/
#include "Linear_ADC.h"
#include "Linear.h"

#include <stdint.h>

/******************************************************************************/
/*!
	@brief  return 0-65535 using max reading as 100 percent

	linear_invf(adcu) = frac16
	linear_f(frac16) = adcu

 */
/******************************************************************************/
void Linear_ADC_Init(Linear_T * p_linear, uint16_t adcUnitZero, uint16_t adcUnitRef, int16_t physicalUnitRef)
{
	int32_t factor 	= physicalUnitRef;
	int32_t divisor = (adcUnitRef - adcUnitZero);

#ifdef CONFIG_LINEAR_SHIFT_DIVIDE
	Linear_Init_X0(p_linear, factor, divisor, adcUnitZero, physicalUnitRef);

#elif defined (CONFIG_LINEAR_NUMIRICAL_DIVIDE)
//	p_linear->SlopeFactor = factor;
//	p_linear->SlopeDivisor = divisor;
//	p_linear->OffsetY = (0 -  factor * adcUnitZero  / divisor);

	//use InvFunction to calc using offset as X0
	p_linear->SlopeFactor =  factor;
	p_linear->SlopeDivisor = divisor;
	p_linear->Offset = adcuZero; //offset x, linear_invf(adcu) = frac16
 	p_linear->RangeReference = physicalUnitRef;
#endif
}

