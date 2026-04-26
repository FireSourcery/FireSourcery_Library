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

#include "linear_math.h"
#include "Math/math_general.h"
#include "Math/Fixed/fixed.h"

#include <stdint.h>
#include <assert.h>

#if     defined(LINEAR_DIVIDE_SHIFT)
#elif   defined(LINEAR_DIVIDE_NUMERICAL)
#else
    #define LINEAR_DIVIDE_SHIFT
#endif


/******************************************************************************/
/*!

*/
/******************************************************************************/
typedef struct Linear
{
#if defined(LINEAR_DIVIDE_SHIFT)
    int32_t Slope;              /* y = (x - X0) * Slope >> SlopeShift + Y0 */
    uint8_t SlopeShift;
    int32_t InvSlope;           /* x = (y - Y0) * InvSlope >> InvSlopeShift + X0 *///split caller handle
    uint8_t InvSlopeShift;
#elif defined(LINEAR_DIVIDE_NUMERICAL)
    int32_t SlopeFactor;
    int32_t SlopeDivisor;
#endif
    int32_t X0;
    int32_t Y0;
    /* Zero-to-peak */
    int32_t XDelta;         /* (XRef - X0), f([X0-XDelta:X0+XDelta]) => [-YRef:YRef] */
    int32_t YDelta;
}
Linear_T;

#if defined(LINEAR_DIVIDE_SHIFT)
#define LINEAR_INIT(factor, divisor, y0, yRef)  \
{                                               \
    .Slope              = ,                     \
    .SlopeShift         = ,                     \
    .InvSlope           = ,                     \
    .InvSlopeShift      = ,                     \
    .X0                 = ,                     \
    .Y0                 = ,                     \
}
#endif

// static inline fixed_factor_t Linear_Coefficient(const Linear_T * p_linear) { return (fixed_factor_t) { p_linear->Slope, p_linear->SlopeShift }; }


/******************************************************************************/
/*!
    Protected
*/
/******************************************************************************/
/* Getters In case implementation changes */
static inline int32_t Linear_GetXRef(const Linear_T * p_linear) { return p_linear->X0 + p_linear->XDelta; }
static inline int32_t Linear_GetYRef(const Linear_T * p_linear) { return p_linear->Y0 + p_linear->YDelta; }
static inline int32_t Linear_GetXDelta(const Linear_T * p_linear) { return p_linear->XDelta; }
static inline int32_t Linear_GetYDelta(const Linear_T * p_linear) { return p_linear->YDelta; }
static inline int32_t Linear_GetYRefOverflow(const Linear_T * p_linear) { return p_linear->Y0 + math_abs(p_linear->YDelta) * 2; }
static inline int32_t Linear_GetXRefOverflow(const Linear_T * p_linear) { return p_linear->X0 + math_abs(p_linear->XDelta) * 2; }

static inline int32_t Linear_GetXMin(const Linear_T * p_linear) { return p_linear->X0 - math_abs(p_linear->XDelta); }
static inline int32_t Linear_GetXMax(const Linear_T * p_linear) { return p_linear->X0 + math_abs(p_linear->XDelta); }
static inline int32_t Linear_GetYMin(const Linear_T * p_linear) { return p_linear->Y0 - math_abs(p_linear->YDelta); }
static inline int32_t Linear_GetYMax(const Linear_T * p_linear) { return p_linear->Y0 + math_abs(p_linear->YDelta); }

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
#if defined(LINEAR_DIVIDE_SHIFT)
    return linear_shift_f(p_linear->Slope, p_linear->SlopeShift, p_linear->X0, p_linear->Y0, x);
#elif defined(LINEAR_DIVIDE_NUMERICAL)
    return linear_f(p_linear->SlopeFactor, p_linear->SlopeDivisor, p_linear->X0, p_linear->Y0, x);
#endif
}

/*
    ((y - y0) * divisor / factor) + x0;
*/
static inline int32_t Linear_InvOf(const Linear_T * p_linear, int32_t y)
{
#if defined(LINEAR_DIVIDE_SHIFT)
    return linear_shift_invf(p_linear->InvSlope, p_linear->InvSlopeShift, p_linear->X0, p_linear->Y0, y);
#elif defined(LINEAR_DIVIDE_NUMERICAL)
    return linear_invf(p_linear->SlopeFactor, p_linear->SlopeDivisor, p_linear->X0, p_linear->Y0, y);
#endif
}

/******************************************************************************/
/*!
    Saturated on input - indirectly saturate output
*/
/******************************************************************************/
static inline int32_t Linear_Of_Sat(const Linear_T * p_linear, int32_t x)
{
    return Linear_Of(p_linear, math_clamp(x, Linear_GetXMin(p_linear), Linear_GetXMax(p_linear)));
}

static inline int32_t Linear_InvOf_Sat(const Linear_T * p_linear, int32_t y)
{
    return Linear_InvOf(p_linear, math_clamp(y, Linear_GetYMin(p_linear), Linear_GetYMax(p_linear)));
}

/******************************************************************************/
/*!
    Round
*/
/******************************************************************************/
static inline int32_t Linear_Of_Round(const Linear_T * p_linear, int32_t x)
{
#if defined(LINEAR_DIVIDE_SHIFT)
    return linear_shift_f_round(p_linear->Slope, p_linear->SlopeShift, p_linear->X0, p_linear->Y0, x);
#elif defined(LINEAR_DIVIDE_NUMERICAL)
    return linear_f_round(p_linear->SlopeFactor, p_linear->SlopeDivisor, p_linear->X0, p_linear->Y0, x);
#endif
}

static inline int32_t Linear_InvOf_Round(const Linear_T * p_linear, int32_t y)
{
#if defined(LINEAR_DIVIDE_SHIFT)
    return linear_shift_invf_round(p_linear->InvSlope, p_linear->InvSlopeShift, p_linear->X0, p_linear->Y0, y);
#elif defined(LINEAR_DIVIDE_NUMERICAL)
    return linear_invf_round(p_linear->SlopeFactor, p_linear->SlopeDivisor, p_linear->X0, p_linear->Y0, y);
#endif
}


/******************************************************************************/
/*!
    Extern
*/
/******************************************************************************/
extern void Linear_Init(Linear_T * p_linear, int32_t factor, int32_t divisor, int32_t y0, int32_t yRef);
extern void Linear_Map_Init(Linear_T * p_linear, int32_t x0, int32_t xRef, int32_t y0, int32_t yRef);
extern void Linear_Fixed_Init(Linear_T * p_linear, uint8_t nFractionalBits, int32_t x0, int32_t xRef);

// extern int32_t Linear_Of_Scalar(const Linear_T * p_linear, int32_t x, uint16_t scalar);
// extern int32_t Linear_InvOf_Scalar(const Linear_T * p_linear, int32_t y, uint16_t scalar);

#endif


