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
    @file 	Linear_Frac16.h
    @author FireSoucery
    @brief	Linear Frac16 calc without division
    @version V0
*/
/******************************************************************************/
#ifndef LINEAR_FRAC16_H
#define LINEAR_FRAC16_H

#include "Linear.h"

static inline int32_t Linear_Frac16(const Linear_T * p_linear, int32_t x)
{
	return linear_m16_f16(p_linear->SlopeFactor, p_linear->SlopeDivisor_Shift, p_linear->XOffset, p_linear->YOffset, x);
}

static inline int32_t Linear_Frac16_Inv(const Linear_T * p_linear, int32_t y_frac16)
{
	return linear_m16_invf16(p_linear->SlopeDivisor, p_linear->SlopeFactor_Shift, p_linear->XOffset, p_linear->YOffset, y_frac16);
}

static inline int32_t Linear_Frac16_CalcUnits(const Linear_T * p_linear, int32_t x)
{
	return linear_m16_f(p_linear->SlopeFactor, p_linear->SlopeDivisor_Shift, p_linear->XOffset, p_linear->YOffset, p_linear->YReference, x);
}

static inline int32_t Linear_Frac16_CalcInvUnits(const Linear_T * p_linear, int32_t y)
{
	return linear_m16_invf(p_linear->SlopeDivisor, p_linear->SlopeFactor_Shift, p_linear->XOffset, p_linear->YOffset, p_linear->YReference, y);
}

//todo common bound function
static inline uint16_t Linear_Frac16_Unsigned(const Linear_T * p_linear, int32_t x)
{
	int32_t frac16 = Linear_Frac16(p_linear, x);

	if 		(frac16 > 65535)	{frac16 = 65535;}
	else if (frac16 < 0) 		{frac16 = 0;}

	return (uint16_t)frac16;
}

static inline uint16_t Linear_Frac16_Unsigned_Abs(const Linear_T * p_linear, int32_t x)
{
	int32_t frac16 = Linear_Frac16(p_linear, x);

	if 		(frac16 < 0)		{frac16 = 0 - frac16;}
	else if (frac16 > 65535)	{frac16 = 65535;}

	return (uint16_t)frac16;
}

static inline int32_t Linear_Frac16_InvUnsigned(const Linear_T * p_linear, uint16_t y_frac16)
{
	return Linear_Frac16_Inv(p_linear, y_frac16);
}

static inline int16_t Linear_Frac16_Signed(const Linear_T * p_linear, int32_t x)
{
	int32_t frac16 = Linear_Frac16(p_linear, x) / 2;

	if 		(frac16 > 32767)	{frac16 = 32767;}
	else if (frac16 < -32768) 	{frac16 = -32768;}

	return (int16_t)frac16;
}

static inline int32_t Linear_Frac16_InvSigned(const Linear_T * p_linear, int16_t y_fracSigned16)
{
	return Linear_Frac16_Inv(p_linear, y_fracSigned16 * 2);
}

#endif