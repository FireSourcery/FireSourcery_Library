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

#include "Config.h"
#include "math_linear.h"
#include "Math/math_general.h"
#include <stdint.h>


typedef struct Linear
{
#if defined(CONFIG_LINEAR_DIVIDE_SHIFT)
    int32_t Slope;              /* y = (x - XOffset) * Slope >> SlopeShift + YOffset */
    uint8_t SlopeShift;
    int32_t InvSlope;           /* x = (y - YOffset) * InvSlope >> InvSlopeShift + XOffset */
    uint8_t InvSlopeShift;
#elif defined(CONFIG_LINEAR_DIVIDE_NUMERICAL)
    int32_t SlopeFactor;
    int32_t SlopeDivisor;
#endif
    int32_t XOffset;
    int32_t YOffset;
    /* Zero-to-peak */
    int32_t XDeltaRef;      /* (XRef - X0), f([X0-XDeltaRef:X0+XDeltaRef]) => [-YRef:YRef] */
    int32_t YDeltaRef;
    /* Intermediate values, optionally store for saturation check */
    int32_t XReference;     /* f([X0:XRef]) <=> fixed32(x)[0:65536] */
    int32_t YReference;     /* f(x)[Y0:YRef] <=> fixed32(x)[0:65536] */
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
static inline int16_t _Linear_SatSigned16(int32_t frac16)           { return ((int16_t)math_clamp(frac16, INT16_MIN, INT16_MAX)); }
static inline uint16_t _Linear_SatUnsigned16(int32_t frac16)        { return ((uint16_t)math_clamp(frac16, 0, UINT16_MAX)); }
static inline uint16_t _Linear_SatUnsigned16_Abs(int32_t frac16)    { return _Linear_SatUnsigned16(math_abs(frac16)); }

/* NonError Checked */
static inline void _Linear_SetSlope(Linear_T * p_linear, int32_t slopeFactor, int32_t slopeDivisor)
{
    p_linear->Slope = (slopeFactor << p_linear->SlopeShift) / slopeDivisor;
    p_linear->InvSlope = (slopeDivisor << p_linear->InvSlopeShift) / slopeFactor;
}

/******************************************************************************/
/*!
    @brief Linear Main Functions - Configured by Linear_Init
*/
/******************************************************************************/
/*
    ((x - x0) * factor / divisor) + y0
    Overflow: f([-XRef*2:XRef*2])
*/
static inline int32_t Linear_Of(const Linear_T * p_linear, int32_t x)
{
#if defined(CONFIG_LINEAR_DIVIDE_SHIFT)
    return linear_shift_f(p_linear->Slope, p_linear->SlopeShift, p_linear->XOffset, p_linear->YOffset, x);
#elif defined(CONFIG_LINEAR_DIVIDE_NUMERICAL)
    return linear_f(p_linear->SlopeFactor, p_linear->SlopeDivisor, p_linear->XOffset, p_linear->YOffset, x);
#endif
}

/*
    ((y - y0) * divisor / factor) + x0;
*/
static inline int32_t Linear_InvOf(const Linear_T * p_linear, int32_t y)
{
#if defined(CONFIG_LINEAR_DIVIDE_SHIFT)
    return linear_shift_invf(p_linear->InvSlope, p_linear->InvSlopeShift, p_linear->XOffset, p_linear->YOffset, y);
#elif defined(CONFIG_LINEAR_DIVIDE_NUMERICAL)
    return linear_invf(p_linear->SlopeFactor, p_linear->SlopeDivisor, p_linear->XOffset, p_linear->YOffset, y);
#endif
}

/******************************************************************************/
/*!
    Saturated on Input - indirectly saturates output to prevent overflow
*/
/******************************************************************************/
static inline int32_t Linear_Of_Sat(const Linear_T * p_linear, int32_t x)
{
    return Linear_Of(p_linear, math_clamp(x, p_linear->XOffset - p_linear->XDeltaRef, p_linear->XReference));
}

static inline int32_t Linear_InvOf_Sat(const Linear_T * p_linear, int32_t y)
{
    return Linear_InvOf(p_linear, math_clamp(y, p_linear->YOffset - p_linear->YDeltaRef, p_linear->YReference));
}

/******************************************************************************/
/*!
    Round
*/
/******************************************************************************/
static inline int32_t Linear_Of_Round(const Linear_T * p_linear, int32_t x)
{
#if defined(CONFIG_LINEAR_DIVIDE_SHIFT)
    return Linear_Of(p_linear, x);
#elif defined(CONFIG_LINEAR_DIVIDE_NUMERICAL)
    return linear_f_rounded(p_linear->SlopeFactor, p_linear->SlopeDivisor, p_linear->XOffset, p_linear->YOffset, x);
#endif
}

static inline int32_t Linear_InvOf_Round(const Linear_T * p_linear, int32_t y)
{
#if defined(CONFIG_LINEAR_DIVIDE_SHIFT)
    return Linear_InvOf(p_linear, y);
#elif defined(CONFIG_LINEAR_DIVIDE_NUMERICAL)
    return linear_invf_rounded(p_linear->SlopeFactor, p_linear->SlopeDivisor, p_linear->XOffset, p_linear->YOffset, y);
#endif
}

/******************************************************************************/
/*!
    @brief Auxiliary extension to primary configuration
*/
/******************************************************************************/

/******************************************************************************/
/*!
    @brief Fixed32 with division
        aux to primary configuration. Use Linear_Fixed for fixed as primary.
    Format q16.16
    f([-XRef:XRef]) => [-65536:65536]
*/
/******************************************************************************/
/*  */
static inline int32_t _Linear_Fixed32(const Linear_T * p_linear, int32_t x)
{
    return linear_fixed32(p_linear->XOffset, p_linear->XDeltaRef, x);
    // return linear_f_x0(65536, deltax, x0, x);
}

/*!
    y_fixed32 in q16.16 format
    @param[in] y_fixed32 overflow limit ~q1.16
*/
static inline int32_t _Linear_InvFixed32(const Linear_T * p_linear, int32_t y_fixed32)
{
    return linear_invfixed32(p_linear->XOffset, p_linear->XDeltaRef, y_fixed32);
    // return linear_invf_x0(65536, deltax, x0, y_fixed32);
}

/******************************************************************************/
/*!
    Saturate to uint16_t, q0.16 [0:65535]
    f([-XRef:XRef]) => [0:65536]
*/
/******************************************************************************/
/* negative returns zero */
static inline uint16_t _Linear_Percent16(const Linear_T * p_linear, int32_t x)
{
    return _Linear_SatUnsigned16(_Linear_Fixed32(p_linear, x));
}

/* negative returns abs */
static inline uint16_t _Linear_Percent16_Abs(const Linear_T * p_linear, int32_t x)
{
    return _Linear_SatUnsigned16_Abs(_Linear_Fixed32(p_linear, x));
}

/* y_frac16 in q0.16 format is handled by q16.16 case */
static inline int32_t _Linear_InvPercent16(const Linear_T * p_linear, uint16_t y_percent16)
{
    return _Linear_InvFixed32(p_linear, y_percent16);
}

/******************************************************************************/
/*!
    Saturate to int16_t, q1.15 [-32768:32767]
    f([-XRef:XRef]) => [-32768:32767]
*/
/******************************************************************************/
/* */
static inline int16_t _Linear_Frac16(const Linear_T * p_linear, int32_t x)
{
    return _Linear_SatSigned16(_Linear_Fixed32(p_linear, x) / 2);
    // return  linear_f_x0(32768, deltax, x0, x);
}

/* y_frac16 use q1.15 */
static inline int32_t _Linear_InvFrac16(const Linear_T * p_linear, int16_t y_frac16)
{
    return _Linear_InvFixed32(p_linear, (int32_t)y_frac16 * 2);
}

/******************************************************************************/
/*!
    Extern
*/
/******************************************************************************/
extern void Linear_Init(Linear_T * p_linear, int32_t factor, int32_t divisor, int32_t y0, int32_t yRef);
extern void Linear_Init_Map(Linear_T * p_linear, int32_t x0, int32_t xRef, int32_t y0, int32_t yRef);

extern int32_t Linear_Of_Scalar(const Linear_T * p_linear, int32_t x, uint16_t scalar);
extern int32_t Linear_InvOf_Scalar(const Linear_T * p_linear, int32_t y, uint16_t scalar);

// extern uint16_t _Linear_Percent16(const Linear_T * p_linear, int32_t x);
// extern uint16_t _Linear_Percent16_Abs(const Linear_T * p_linear, int32_t x);
// extern int32_t _Linear_InvPercent16(const Linear_T * p_linear, uint16_t y_fracU16);
// extern int16_t _Linear_Frac16(const Linear_T * p_linear, int32_t x);
// extern int32_t _Linear_InvFrac16(const Linear_T * p_linear, int16_t y_frac16);

#endif