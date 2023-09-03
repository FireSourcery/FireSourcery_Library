/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

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
    @file   Linear_Frac16.h
    @author FireSourcery
    @brief  Linear Frac16 calc without division
    @version V0
*/
/******************************************************************************/
#ifndef LINEAR_FRAC16_H
#define LINEAR_FRAC16_H

#include "Linear.h"

/******************************************************************************/
/*!
todo ufrac16 as 2

    Frac16 => [-32768:32767], [-1:1) <=> [-32768:32767] in Q1.15
    UFrac16 <=> Fixed32 => [0:65535], [0:2) <=> [0:65535] in Q1.15,
    Fixed32 => [INT32_MIN:INT32_MAX], [-1:1] <=> [-65536:65536] in Q16.16

    Fixed32
    Unsaturated Fraction in q16.16 [-2,147,483,648:2,147,483,647]
    f([-XRef:XRef]) => [-65536:65536], Range bounded to [-65536*2:65535*2] via shift 14 at init.
    Pre saturation to FracU16 [0:65535], FracS16 [-32768:32767]
*/
/******************************************************************************/
static inline int32_t Linear_Frac16(const Linear_T * p_linear, int32_t x)
{
    return linear_m16_f16(p_linear->Slope, p_linear->SlopeShift, p_linear->XOffset, x);
}

/*! @param[in] y_frac16 in q16.16. range bounded to [-65536*2:65535*2] */
static inline int32_t Linear_Frac16_Inv(const Linear_T * p_linear, int32_t y_frac16)
{
    return linear_m16_invf16(p_linear->InvSlope, p_linear->InvSlopeShift, p_linear->XOffset, y_frac16);
}

static inline int32_t Linear_Frac16_Units16(const Linear_T * p_linear, int32_t y_frac16)
{
    return linear_m16_g(p_linear->YOffset, p_linear->DeltaY, y_frac16);
}

static inline int32_t Linear_Frac16_InvUnits16(const Linear_T * p_linear, int32_t y_units)
{
    return linear_m16_invg(p_linear->YOffset, p_linear->DeltaY, y_units);
}

/* User Units using YRef */
static inline int32_t Linear_Frac16_Units(const Linear_T * p_linear, int32_t x)
{
    return linear_m16_f(p_linear->Slope, p_linear->SlopeShift, p_linear->XOffset, p_linear->YOffset, p_linear->DeltaY, x);
}

/* Division limited to this function only */
static inline int32_t Linear_Frac16_InvUnits(const Linear_T * p_linear, int32_t y)
{
    return linear_m16_invf(p_linear->InvSlope, p_linear->InvSlopeShift, p_linear->XOffset, p_linear->YOffset, p_linear->DeltaY, y);
}

/******************************************************************************/
/*!
    Saturate to uint16_t, q0.16 [0:65535]
    65535 => 1.0f
*/
/******************************************************************************/
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

/******************************************************************************/
/*!
    Convert and Saturate to int16_t, q1.15 [-32768:32767]
    32767 => 1.0f
*/
/******************************************************************************/
static inline int16_t Linear_Frac16_Signed(const Linear_T * p_linear, int32_t x)
{
    return _Linear_SatSigned16(Linear_Frac16(p_linear, x) / 2);
}

static inline int32_t Linear_Frac16_InvSigned(const Linear_T * p_linear, int16_t y_fracS16)
{
    return Linear_Frac16_Inv(p_linear, (int32_t)y_fracS16 * 2);
}

/******************************************************************************/
/*!
    Extern
*/
/******************************************************************************/
extern void Linear_Frac16_Init(Linear_T * p_linear, int32_t x0, int32_t xRef, int32_t y0_Units, int32_t yRef_Units);
extern void Linear_Frac16_Init_Slope(Linear_T * p_linear, int32_t factor, int32_t divisor, int32_t x0, int32_t xRef);

#endif
