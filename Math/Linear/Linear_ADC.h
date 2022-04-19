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
    @file 	Linear_ADC.h
    @author FireSoucery
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef LINEAR_ADC_H
#define LINEAR_ADC_H

#include "Linear_Frac16.h"
#include <stdint.h>

static inline int32_t Linear_ADC_CalcPhysical(const Linear_T * p_linear, uint16_t adcu)
{
	return Linear_Frac16_CalcUnits(p_linear, adcu);
}

static inline int32_t Linear_ADC_CalcFraction16(const Linear_T * p_linear, uint16_t adcu)
{
	return Linear_Frac16(p_linear, adcu);
}

static inline int32_t Linear_ADC_CalcFractionSigned16(const Linear_T * p_linear, uint16_t adcu)
{
	return Linear_Frac16_Signed(p_linear, adcu);
}

static inline uint32_t Linear_ADC_CalcFractionUnsigned16(const Linear_T * p_linear, uint16_t adcu)
{
	return Linear_Frac16_Unsigned(p_linear, adcu);
}

static inline uint32_t Linear_ADC_CalcFractionUnsigned16_Abs(const Linear_T * p_linear, uint16_t adcu)
{
	return Linear_Frac16_Unsigned_Abs(p_linear, adcu);
}

static inline uint16_t Linear_ADC_CalcAdcu_Physical(const Linear_T * p_linear, int16_t units)
{
	return Linear_Frac16_CalcInvUnits(p_linear, units);
}

static inline uint16_t Linear_ADC_CalcAdcu_FractionSigned16(const Linear_T * p_linear, int32_t signedFrac16)
{
	return Linear_Frac16_InvSigned(p_linear, signedFrac16);
}

static inline uint16_t Linear_ADC_CalcAdcu_FractionUnsigned16(const Linear_T * p_linear, uint32_t unsignedFrac16)
{
	return Linear_Frac16_InvUnsigned(p_linear, unsignedFrac16);
}

extern void Linear_ADC_Init(Linear_T * p_linear, uint16_t adcuZero, uint16_t adcuRef, int16_t physicalRef);
extern void Linear_ADC_Init_Inverted(Linear_T * p_linear, uint16_t adcuZero, uint16_t adcuRef, int16_t physicalRef);

#endif
