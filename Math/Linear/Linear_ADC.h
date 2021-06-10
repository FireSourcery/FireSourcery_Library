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
    @file 	Linear_ADC.h
    @author FireSoucery
    @brief 	linear function using known measured value adc limits

    @version V0
*/
/*******************************************************************************/
#ifndef LINEAR_ADC_H
#define LINEAR_ADC_H

#include "Linear.h"

#include <stdint.h>

//static inline int16_t Linear_ADC_ConvertAdcuToFraction_Signed16(Linear_T * p_linear, uint16_t adcu)

static inline int16_t Linear_ADC_CalcSignedFraction16(Linear_T * p_linear, uint16_t adcu)
{
#ifdef CONFIG_LINEAR_SHIFT_DIVIDE
	return Linear_Function_SignedFraction16(p_linear, adcu);
#elif defined (CONFIG_LINEAR_NUMIRICAL_DIVIDE)
//	return  p_linear->SlopeDivisor * (adcu - p_linear->Offset) / p_linear->SlopeFactor / 2U;
	return (Linear_InvFunction(p_linear, adcu) / 2U);
#endif
}

static inline uint16_t Linear_ADC_CalcUnsignedFraction16(Linear_T * p_linear, uint16_t adcu)
{
#ifdef CONFIG_LINEAR_SHIFT_DIVIDE
	return Linear_Function_UnsignedFraction16(p_linear, adcu);
#elif defined (CONFIG_LINEAR_NUMIRICAL_DIVIDE)
	Linear_InvFunction(p_linear, adcu);

//	if (adcu > p_linear->Offset)
//	{
//		return  p_linear->SlopeDivisor * (adcu - p_linear->Offset) / p_linear->SlopeFactor;
//		//	return (uint16_t) Linear_InvFunction(p_linear, adcu);
//	}
//	else
//	{
//		return  0 - (p_linear->SlopeDivisor * (adcu - p_linear->Offset) / p_linear->SlopeFactor);
//		//	return (uint16_t) (0-Linear_InvFunction(p_linear, adcu));
//	}
#endif

}

static inline int32_t Linear_ADC_CalcPhysical(Linear_T * p_linear, uint16_t adcu)
{
#ifdef CONFIG_LINEAR_SHIFT_DIVIDE
	return Linear_Function(p_linear, adcu);
#elif defined (CONFIG_LINEAR_NUMIRICAL_DIVIDE)
//	return Linear_InvFunction(p_linear, adcu);
//	p_linear->SlopeDivisor * (adcu - p_linear->Offset) * p_linear->RangeReference / p_linear->SlopeFactor / 65536;
#endif
}

static inline int32_t Linear_ADC_CalcAdcu_SignedFraction16(Linear_T * p_linear, int16_t signedFrac16)
{
#ifdef CONFIG_LINEAR_SHIFT_DIVIDE
	return Linear_InvFunction_SignedFraction16(p_linear, signedFrac16);
#elif defined (CONFIG_LINEAR_NUMIRICAL_DIVIDE)
	Linear_Function_SignedFraction16(p_linear, signedFrac16); //todo check
	//	return signedFrac16 * 2U * p_linear->SlopeFactor / p_linear->SlopeDivisor + p_linear->Offset;
#endif
}

static inline uint32_t Linear_ADC_CalcAdcu_UnsignedFraction16(Linear_T * p_linear, uint16_t unsignedFrac16)
{
#ifdef CONFIG_LINEAR_SHIFT_DIVIDE
	return Linear_InvFunction_UnsignedFraction16(p_linear, unsignedFrac16);
#elif defined (CONFIG_LINEAR_NUMIRICAL_DIVIDE)
	Linear_Function_SignedFraction16(p_linear, unsignedFrac16)  //todo check
//	return unsignedFrac16* p_linear->SlopeFactor / p_linear->SlopeDivisor + p_linear->Offset;
#endif
}

static inline uint32_t Linear_ADC_CalcAdcu_Physical(Linear_T * p_linear, int16_t units)
{
#ifdef CONFIG_LINEAR_SHIFT_DIVIDE
	return Linear_InvFunction(p_linear, units);
#elif defined (CONFIG_LINEAR_NUMIRICAL_DIVIDE)
	Linear_Function(p_linear, unsignedFrac16)  //todo check
//	return units * 65536 * p_linear->SlopeFactor / p_linear->RangeReference / p_linear->SlopeDivisor + p_linear->Offset;
#endif
}

#endif
