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
    @file   Linear.h
    @author FireSourcery
    @brief  Mathematical linear function.
            e.g. Dynamic look up table, unit/ADC conversion.
    @version V0
*/
/******************************************************************************/
#ifndef LINEAR_H
#define LINEAR_H

#include "math_linear.h"
#include "Config.h"
#include "Math/math_general.h"
#include <stdint.h>

typedef struct Linear_Tag
{
#if defined(CONFIG_LINEAR_DIVIDE_SHIFT)
    int32_t Slope;            /* y = (x - XOffset) * Slope >> SlopeShift + YOffset */
    uint8_t SlopeShift;
    int32_t InvSlope;        /* x = (y - YOffset) * InvSlope >> InvSlopeShift + XOffset */
    uint8_t InvSlopeShift;
#elif defined(CONFIG_LINEAR_DIVIDE_NUMERICAL)
    int32_t SlopeFactor;
    int32_t SlopeDivisor;
#endif
    int32_t XOffset;
    int32_t YOffset;
    int32_t XReference;        /* f([0:XRef]) => frac16(x)[0:65536] *//* User Info only for now */
    int32_t YReference;        /* f(x)[0:YRef] => frac16(x)[0:65536] */
    int32_t DeltaX;            /* (XRef - X0), f([X0-DeltaX:X0+DeltaX]) == [-YRef:YRef] */
    int32_t DeltaY;
}
Linear_T;

#if defined(CONFIG_LINEAR_DIVIDE_SHIFT)
#define LINEAR_INIT(factor, divisor, y0, yRef)  \
{                                               \
    .Slope              = ,                     \
    .SlopeShift         = ,                     \
    .InvSlope           = ,                     \
    .InvSlopeShift      = ,                     \
    .XOffset            = ,                     \
    .YOffset            = ,                     \
    .XReference         = ,                     \
    .YReference         = ,                     \
}
#endif

/******************************************************************************/
/*!
    Protected
*/
/******************************************************************************/
static inline int32_t _Linear_Sat(int32_t min, int32_t max, int32_t value)
{
    return math_clamp(value, min, max);
}

static inline int16_t _Linear_SatSigned16(int32_t frac16) { return (int16_t)_Linear_Sat(INT16_MIN, INT16_MAX, frac16); }
static inline uint16_t _Linear_SatUnsigned16(int32_t frac16) { return (uint16_t)_Linear_Sat(0, UINT16_MAX, frac16); }
static inline uint16_t _Linear_SatUnsigned16_Abs(int32_t frac16)
{
    int32_t sat = _Linear_Sat(0 - (int32_t)UINT16_MAX, UINT16_MAX, frac16);
    return (sat < 0) ? (uint16_t)(0 - sat): (uint16_t)sat;
}

/* NonError Checked */
static inline void _Linear_SetSlope(Linear_T * p_linear, int32_t slopeFactor, int32_t slopeDivisor)
{
    p_linear->Slope = (slopeFactor << p_linear->SlopeShift) / slopeDivisor;
    p_linear->InvSlope = (slopeDivisor << p_linear->InvSlopeShift) / slopeFactor;
}

/******************************************************************************/
/*!
    @brief Linear Essential Functions - User Units
*/
/******************************************************************************/
/*
    ((x - x0) * factor / divisor) + y0
    Overflow: Slope * (x - x0) must be < INT32_MAX [2,147,483,647]
*/
static inline int32_t Linear_Function(const Linear_T * p_linear, int32_t x)
{
#if defined(CONFIG_LINEAR_DIVIDE_SHIFT)
    return linear_f_shift(p_linear->Slope, p_linear->SlopeShift, p_linear->XOffset, p_linear->YOffset, x);
#elif defined(CONFIG_LINEAR_DIVIDE_NUMERICAL)
    return linear_f(p_linear->SlopeFactor, p_linear->SlopeDivisor, p_linear->XOffset, p_linear->YOffset, x);
#endif
}

/*
    ((y - y0) * divisor / factor) + x0;
*/
static inline int32_t Linear_InvFunction(const Linear_T * p_linear, int32_t y)
{
#if defined(CONFIG_LINEAR_DIVIDE_SHIFT)
    return linear_invf_shift(p_linear->InvSlope, p_linear->InvSlopeShift, p_linear->XOffset, p_linear->YOffset, y);
#elif defined(CONFIG_LINEAR_DIVIDE_NUMERICAL)
    return linear_invf(p_linear->SlopeFactor, p_linear->SlopeDivisor, p_linear->XOffset, p_linear->YOffset, y);
#endif
}

/******************************************************************************/
/*!
    @brief Frac16 with division
    Fraction in q16.16 [-2,147,483,648:2,147,483,647]
    f([-XRef:XRef]) => [-65536:65536]
    @{
*/
/******************************************************************************/
/*  */
static inline int32_t Linear_Function_Fixed32(const Linear_T * p_linear, int32_t x)
{
    return linear_f16(p_linear->XOffset, p_linear->DeltaX, x);
}

/*!
    y_fixed32 in q16.16 format
    @param[in] y_fixed32 overflow limit ~q1.16
*/
static inline int32_t Linear_InvFunction_Fixed32(const Linear_T * p_linear, int32_t y_fixed32)
{
    return linear_invf16(p_linear->XOffset, p_linear->DeltaX, y_fixed32);
}

/******************************************************************************/
/*!
    Frac16 Saturated Output
    Saturate to uint16_t, q0.16 [0:65535]
    f([-XRef:XRef]) => [0:65536]
*/
/******************************************************************************/
/* negative returns zero */
static inline uint16_t Linear_Function_FracU16(const Linear_T * p_linear, int32_t x)
{
    return _Linear_SatUnsigned16(Linear_Function_Fixed32(p_linear, x));
}

/* negative returns abs */
static inline uint16_t Linear_Function_FracU16_Abs(const Linear_T * p_linear, int32_t x)
{
    return _Linear_SatUnsigned16_Abs(Linear_Function_Fixed32(p_linear, x));
}

/* y_frac16 in q0.16 format is handled by q16.16 case */
static inline int32_t Linear_InvFunction_FracU16(const Linear_T * p_linear, uint16_t y_fracU16)
{
    return Linear_InvFunction_Fixed32(p_linear, y_fracU16);
}

/******************************************************************************/
/*!
    Saturate to int16_t, q1.15 [-32768:32767]
    f([-XRef:XRef]) => [-32768:32767]
*/
/******************************************************************************/
/* */
static inline int16_t Linear_Function_FracS16(const Linear_T * p_linear, int32_t x)
{
    return _Linear_SatSigned16(Linear_Function_Fixed32(p_linear, x) / 2);
}

/* y_frac16 use q1.15 */
static inline int32_t Linear_InvFunction_FracS16(const Linear_T * p_linear, int16_t y_fracS16)
{
    return Linear_InvFunction_Fixed32(p_linear, (int32_t)y_fracS16 * 2);
}


/******************************************************************************/
/*!
    Saturated on Input - indirectly saturates output and avoids overflow
*/
/******************************************************************************/
static inline int32_t Linear_Function_Sat(const Linear_T * p_linear, int32_t x)
{
    return Linear_Function(p_linear, _Linear_Sat(0 - p_linear->XReference, p_linear->XReference, x));
}

static inline int32_t Linear_InvFunction_Sat(const Linear_T * p_linear, int32_t y)
{
    return Linear_InvFunction(p_linear, _Linear_Sat(0 - p_linear->YReference, p_linear->YReference, y));
}

/******************************************************************************/
/*!
    Round
*/
/******************************************************************************/
static inline int32_t Linear_Function_Round(const Linear_T * p_linear, int32_t x)
{
#if defined(CONFIG_LINEAR_DIVIDE_SHIFT)
    return Linear_Function(p_linear, x);
#elif defined(CONFIG_LINEAR_DIVIDE_NUMERICAL)
    return linear_f_rounded(p_linear->SlopeFactor, p_linear->SlopeDivisor, p_linear->XOffset, p_linear->YOffset, x);
#endif
}

static inline int32_t Linear_InvFunction_Round(const Linear_T * p_linear, int32_t y)
{
#if defined(CONFIG_LINEAR_DIVIDE_SHIFT)
    return Linear_InvFunction(p_linear, y);
#elif defined(CONFIG_LINEAR_DIVIDE_NUMERICAL)
    return linear_invf_rounded(p_linear->SlopeFactor, p_linear->SlopeDivisor, p_linear->XOffset, p_linear->YOffset, y);
#endif
}

/******************************************************************************/
/*!
    Extern
*/
/******************************************************************************/
extern void Linear_Init(Linear_T * p_linear, int32_t factor, int32_t divisor, int32_t y0, int32_t yRef);
extern void Linear_Init_Map(Linear_T * p_linear, int32_t x0, int32_t xRef, int32_t y0, int32_t yRef);

extern int32_t Linear_Function_Scalar(const Linear_T * p_linear, int32_t x, uint16_t scalar);
extern int32_t Linear_InvFunction_Scalar(const Linear_T * p_linear, int32_t y, uint16_t scalar);

extern uint16_t Linear_Function_FracU16(const Linear_T * p_linear, int32_t x);
extern uint16_t Linear_Function_FracU16_Abs(const Linear_T * p_linear, int32_t x);
extern int32_t Linear_InvFunction_FracU16(const Linear_T * p_linear, uint16_t y_fracU16);
extern int16_t Linear_Function_FracS16(const Linear_T * p_linear, int32_t x);
extern int32_t Linear_InvFunction_FracS16(const Linear_T * p_linear, int16_t y_fracS16);

#endif