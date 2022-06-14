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

/*! @return q1.16 */
static inline int32_t Linear_Frac16(const Linear_T * p_linear, int32_t x)
{
	return linear_m16_f16(p_linear->Slope, p_linear->SlopeShift, p_linear->XOffset, p_linear->YOffset, x);
}

/*! @param[in] y_frac16 in q1.16 */
static inline int32_t Linear_Frac16_Inv(const Linear_T * p_linear, int32_t y_frac16)
{
	return linear_m16_invf16(p_linear->InvSlope, p_linear->InvSlopeShift, p_linear->XOffset, p_linear->YOffset, y_frac16);
}

/* User Units using YRef */
static inline int32_t Linear_Frac16_Units(const Linear_T * p_linear, int32_t x)
{
	return linear_m16_f(p_linear->Slope, p_linear->SlopeShift, p_linear->XOffset, p_linear->YOffset, p_linear->YReference, x);
}

static inline int32_t Linear_Frac16_InvUnits(const Linear_T * p_linear, int32_t y)
{
	return linear_m16_invf(p_linear->InvSlope, p_linear->InvSlopeShift, p_linear->XOffset, p_linear->YOffset, p_linear->YReference, y);
}

//may be iterative shift convert
static inline int32_t Linear_Frac16_Units_Scalar(const Linear_T * p_linear, int32_t x, uint16_t scalar)
{
	(void)p_linear;
	(void)x;
	(void)scalar;
	return 0U; //todo
}

static inline uint16_t Linear_Frac16_Unsigned(const Linear_T * p_linear, int32_t x)
{
	return _Linear_SatUnsigned16(Linear_Frac16(p_linear, x));
}

static inline uint16_t Linear_Frac16_Unsigned_Abs(const Linear_T * p_linear, int32_t x)
{
	return _Linear_SatUnsigned16_Abs(Linear_Frac16(p_linear, x));
}

static inline int32_t Linear_Frac16_InvUnsigned(const Linear_T * p_linear, uint16_t y_fracU16)
{
	return Linear_Frac16_Inv(p_linear, y_fracU16);
}

static inline int16_t Linear_Frac16_Signed(const Linear_T * p_linear, int32_t x)
{
	return _Linear_SatSigned16(Linear_Frac16(p_linear, x) / 2);
}

static inline int32_t Linear_Frac16_InvSigned(const Linear_T * p_linear, int16_t y_fracS16)
{
	return Linear_Frac16_Inv(p_linear, (int32_t)y_fracS16 * 2);
}

extern void Linear_Frac16_Init(Linear_T * p_linear, int32_t factor, int32_t divisor, int32_t y0, int32_t yRef);
extern void Linear_Frac16_Init_Map(Linear_T * p_linear, int32_t x0, int32_t xRef, int32_t y0, int32_t yRef);

#endif
