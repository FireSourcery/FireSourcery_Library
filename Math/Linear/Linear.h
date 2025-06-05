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

*/
/******************************************************************************/
#ifndef LINEAR_H
#define LINEAR_H

#include "Config.h"
#include "math_linear.h"
#include "Math/math_general.h"
#include "Math/Fixed/fixed.h"

#include <stdint.h>


typedef struct Linear
{
#if defined(CONFIG_LINEAR_DIVIDE_SHIFT)
    int32_t Slope;              /* y = (x - X0) * Slope >> SlopeShift + Y0 */
    uint8_t SlopeShift;
    int32_t InvSlope;           /* x = (y - Y0) * InvSlope >> InvSlopeShift + X0 */
    uint8_t InvSlopeShift;
#elif defined(CONFIG_LINEAR_DIVIDE_NUMERICAL)
    int32_t SlopeFactor;
    int32_t SlopeDivisor;
#endif
    int32_t X0;
    int32_t Y0;
    /* Zero-to-peak */
    int32_t XDelta;      /* (XRef - X0), f([X0-XDelta:X0+XDelta]) => [-YRef:YRef] */
    int32_t YDelta;
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
    .X0                 = ,                     \
    .Y0                 = ,                     \
    .XReference         = ,                     \
    .YReference         = ,                     \
}
#endif

/******************************************************************************/
/*!
    Protected
*/
/******************************************************************************/
static inline int16_t _Linear_SatSigned16(int32_t value16)           { return ((int16_t)math_clamp(value16, INT16_MIN, INT16_MAX)); }
static inline uint16_t _Linear_SatUnsigned16(int32_t value16)        { return ((uint16_t)math_clamp(value16, 0, UINT16_MAX)); }
static inline uint16_t _Linear_SatUnsigned16_Abs(int32_t value16)    { return _Linear_SatUnsigned16(math_abs(value16)); }

/* Getters In case implementation changes */
static inline int32_t Linear_GetXRef(const Linear_T * p_linear) { return p_linear->X0 + p_linear->XDelta; }
static inline int32_t Linear_GetYRef(const Linear_T * p_linear) { return p_linear->Y0 + p_linear->YDelta; }
static inline int32_t Linear_GetXDelta(const Linear_T * p_linear) { return p_linear->XDelta; }
static inline int32_t Linear_GetYDelta(const Linear_T * p_linear) { return p_linear->YDelta; }
static inline int32_t Linear_GetYRefOverflow(const Linear_T * p_linear) { return p_linear->Y0 + p_linear->YDelta * 2; }
static inline int32_t Linear_GetXRefOverflow(const Linear_T * p_linear) { return p_linear->X0 + p_linear->XDelta * 2; }

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
    return linear_shift_f(p_linear->Slope, p_linear->SlopeShift, p_linear->X0, p_linear->Y0, x);
#elif defined(CONFIG_LINEAR_DIVIDE_NUMERICAL)
    return linear_f(p_linear->SlopeFactor, p_linear->SlopeDivisor, p_linear->X0, p_linear->Y0, x);
#endif
}

/*
    ((y - y0) * divisor / factor) + x0;
*/
static inline int32_t Linear_InvOf(const Linear_T * p_linear, int32_t y)
{
#if defined(CONFIG_LINEAR_DIVIDE_SHIFT)
    return linear_shift_invf(p_linear->InvSlope, p_linear->InvSlopeShift, p_linear->X0, p_linear->Y0, y);
#elif defined(CONFIG_LINEAR_DIVIDE_NUMERICAL)
    return linear_invf(p_linear->SlopeFactor, p_linear->SlopeDivisor, p_linear->X0, p_linear->Y0, y);
#endif
}

/******************************************************************************/
/*!
    Saturated on Input - indirectly saturates output to prevent overflow
*/
/******************************************************************************/
static inline int32_t Linear_Of_Sat(const Linear_T * p_linear, int32_t x)
{
    return Linear_Of(p_linear, math_clamp(x, p_linear->X0 - p_linear->XDelta, p_linear->XReference));
}

static inline int32_t Linear_InvOf_Sat(const Linear_T * p_linear, int32_t y)
{
    // return Linear_InvOf(p_linear, math_clamp(y, p_linear->Y0 - p_linear->YDelta, p_linear->YReference));
    return Linear_InvOf(p_linear, math_clamp(y, p_linear->Y0 - p_linear->YDelta * 2, p_linear->Y0 + p_linear->YDelta * 2));
}

/******************************************************************************/
/*!
    Round
*/
/******************************************************************************/
static inline int32_t Linear_Of_Round(const Linear_T * p_linear, int32_t x)
{
#if defined(CONFIG_LINEAR_DIVIDE_SHIFT)
    return linear_shift_f_round(p_linear->Slope, p_linear->SlopeShift, p_linear->X0, p_linear->Y0, x);
#elif defined(CONFIG_LINEAR_DIVIDE_NUMERICAL)
    return linear_f_round(p_linear->SlopeFactor, p_linear->SlopeDivisor, p_linear->X0, p_linear->Y0, x);
#endif
}

static inline int32_t Linear_InvOf_Round(const Linear_T * p_linear, int32_t y)
{
#if defined(CONFIG_LINEAR_DIVIDE_SHIFT)
    return linear_shift_invf_round(p_linear->InvSlope, p_linear->InvSlopeShift, p_linear->X0, p_linear->Y0, y);
#elif defined(CONFIG_LINEAR_DIVIDE_NUMERICAL)
    return linear_invf_round(p_linear->SlopeFactor, p_linear->SlopeDivisor, p_linear->X0, p_linear->Y0, y);
#endif
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

#endif

/******************************************************************************/
/*!
    @brief Numerical Divide auxiliary extension to primary configuration
*/
/******************************************************************************/
/******************************************************************************/
/*!
    @brief Fixed32 with division
        aux configuration. Use Linear_Fixed for fixed as primary.
    Format q16.16
    f([-XRef:XRef]) => [-65536:65536]
*/
/******************************************************************************/
// /*  */
// static inline int32_t _Linear_Fixed32(const Linear_T * p_linear, int32_t x)
// {
//     return linear_f_x0(65536, p_linear->XDelta, p_linear->X0, x);
// }

// /*!
//     y_fixed32 in q16.16 format
//     @param[in] y_fixed32 overflow limit ~q1.16
// */
// static inline int32_t _Linear_InvFixed32(const Linear_T * p_linear, int32_t y_fixed32)
// {
//     return linear_invf_x0(65536, p_linear->XDelta, p_linear->X0, y_fixed32);
// }

// /******************************************************************************/
// /*!
//     Saturate to uint16_t, q0.16 [0:65535]
//     f([-XRef:XRef]) => [0:65536]
// */
// /******************************************************************************/
// /* negative returns zero */
// static inline uint16_t _Linear_Percent16_Clamp(const Linear_T * p_linear, int32_t x)
// {
//     return _Linear_SatUnsigned16(_Linear_Fixed32(p_linear, x));
// }

// /* negative returns abs */
// static inline uint16_t _Linear_Percent16_Abs(const Linear_T * p_linear, int32_t x)
// {
//     return _Linear_SatUnsigned16_Abs(_Linear_Fixed32(p_linear, x));
// }

// /* y_fract16 in q0.16 format is handled by q16.16 case */
// static inline int32_t _Linear_InvPercent16(const Linear_T * p_linear, uint16_t y_percent16)
// {
//     return _Linear_InvFixed32(p_linear, y_percent16);
// }

// /******************************************************************************/
// /*!
//     Saturate to int16_t, q1.15 [-32768:32767]
//     f([-XRef:XRef]) => [-32768:32767]
// */
// /******************************************************************************/
// /* */
// static inline int16_t _Linear_Fract16(const Linear_T * p_linear, int32_t x)
// {
//     return _Linear_SatSigned16(_Linear_Fixed32(p_linear, x) / 2);
//     // return  linear_f_x0(32768, deltax, x0, x);
// }

// /* y_fract16 in q1.15 */
// static inline int32_t _Linear_InvFract16(const Linear_T * p_linear, int16_t y_fract16)
// {
//     return _Linear_InvFixed32(p_linear, (int32_t)y_fract16 * 2);
// }

/******************************************************************************/
/*!
    Scalar
*/
/******************************************************************************/
// /* scalar may be compile time constant, can compiler unroll loop to optimize? */
// static inline int32_t Linear_Of_Scalar(const Linear_T * p_linear, int32_t x, uint16_t scalar)
// {
//     int32_t factor = x * p_linear->Slope;
//     int32_t result = 0;

//     /*
//         Loop N = Log_[DivisorN](scalar)
//         scalar 1000
//             [DivisorN == 10] => 4
//             [DivisorN == 2]  => 10
//     */
//     for (uint16_t iDivisor = 1U; scalar >= iDivisor; iDivisor *= 4U) /* scalar / iDivisor > 0U */
//     {
//         if (factor < INT32_MAX / scalar * iDivisor) /* (factor * (scalar / iDivisor) < INT32_MAX) */
//         {
//             result = Linear_Of(p_linear, x * scalar / iDivisor) * iDivisor;
//             break;
//         }
//     }

//     if (result == 0) { result = Linear_Of(p_linear, x) * scalar; }

//     return result;
// }


