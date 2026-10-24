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
#include "Math/Fixed/fract16.h"

#include <stdint.h>
#include <assert.h>


/******************************************************************************/
/*!

*/
/******************************************************************************/
/* Forward coefficients only, the inverse divides */
typedef struct Linear
{
    int32_t Slope;          /* y = (x - X0) * Slope >> SlopeShift */
    uint8_t SlopeShift;
    int32_t X0;             /* Zero crossing, f(X0) = 0 */
    int32_t XDelta;         /* (XRef - X0), f([X0-XDelta:X0+XDelta]) => [-YRef:YRef] */
}
Linear_T;


/******************************************************************************/
/*!
    Protected
*/
/******************************************************************************/
/* Getters In case implementation changes */
static inline int32_t Linear_GetXRef(const Linear_T * p_linear) { return p_linear->X0 + p_linear->XDelta; }
static inline int32_t Linear_GetXDelta(const Linear_T * p_linear) { return p_linear->XDelta; }
static inline int32_t Linear_GetXRefOverflow(const Linear_T * p_linear) { return p_linear->X0 + math_abs(p_linear->XDelta) * 2; }

static inline int32_t Linear_GetXMin(const Linear_T * p_linear) { return p_linear->X0 - math_abs(p_linear->XDelta); }
static inline int32_t Linear_GetXMax(const Linear_T * p_linear) { return p_linear->X0 + math_abs(p_linear->XDelta); }

/* [X0:XRef] either order, f() => [0:YRef] */
static inline interval_t Linear_GetXRefInterval(const Linear_T * p_linear) { return interval_of_bounds(p_linear->X0, Linear_GetXRef(p_linear)); }

/******************************************************************************/
/*!
    @brief Linear Main Functions - Configured by Linear_Init
*/
/******************************************************************************/
/*
    ((x - X0) * Slope) >> SlopeShift
    Overflow: f([X0-XDelta*2:X0+XDelta*2])
*/
static inline int32_t Linear_Of(const Linear_T * p_linear, int32_t x)
{
    return linear_shift_f_x0(p_linear->Slope, p_linear->SlopeShift, p_linear->X0, x);
}

/*
    ((y << SlopeShift) / Slope) + X0
    Overflow: |y| >= 2^(31 - SlopeShift)
*/
static inline int32_t Linear_InvOf(const Linear_T * p_linear, int32_t y)
{
    return linear_invf_x0(p_linear->Slope, (int32_t)1 << p_linear->SlopeShift, p_linear->X0, y);
}

/******************************************************************************/
/*!
    Y side, derived
*/
/******************************************************************************/
static inline int32_t Linear_GetYRef(const Linear_T * p_linear) { return Linear_Of(p_linear, Linear_GetXRef(p_linear)); }
static inline int32_t Linear_GetYRefOverflow(const Linear_T * p_linear) { return math_abs(Linear_GetYRef(p_linear)) * 2; }

static inline int32_t Linear_GetYMax(const Linear_T * p_linear) { return math_abs(Linear_GetYRef(p_linear)); }
static inline int32_t Linear_GetYMin(const Linear_T * p_linear) { return 0 - Linear_GetYMax(p_linear); }

/******************************************************************************/
/*!
    Saturated on input - indirectly saturate output
    Bounds the multiply for slopes sized to 2x the interval, Map_Init.
*/
/******************************************************************************/
/* [X0 - XDelta:X0 + XDelta] => [-YRef:YRef] */
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
    return linear_shift_f_round(p_linear->Slope, p_linear->SlopeShift, p_linear->X0, 0, x);
}

static inline int32_t Linear_InvOf_Round(const Linear_T * p_linear, int32_t y)
{
    return linear_invf_round(p_linear->Slope, (int32_t)1 << p_linear->SlopeShift, 0, y) + p_linear->X0;
}

/******************************************************************************/
/*!
    Extern
*/
/******************************************************************************/
extern void Linear_Init(Linear_T * p_linear, int32_t factor, int32_t divisor, int32_t y0, int32_t yRef);
extern void Linear_Map_Init(Linear_T * p_linear, int32_t x0, int32_t xRef, int32_t y0, int32_t yRef);
extern void Linear_Fixed_Init(Linear_T * p_linear, uint8_t nFractionalBits, int32_t x0, int32_t xRef);


/******************************************************************************/
/*!
    Format inits, with the readings of each. The init sets the unit, the reading picks the side.
    The XRef side maps to [0:1.0) for either sign of XDelta, the mirrored side is negative.
    Fixed slope does not overflow for |x - X0| < 2^16, saturate on output. Monotonic, equivalent to saturate on input.
*/
/******************************************************************************/
/* fract16, ufract16 */
static inline void Linear_Fract16_Init(Linear_T * p_linear, int32_t x0, int32_t xRef) { Linear_Fixed_Init(p_linear, 15U, x0, xRef); }
/* [X0 - XDelta:X0 + XDelta] => [-32768:32767] */
static inline fract16_t Linear_Fract16_Sat(const Linear_T * p_linear, int32_t x) { return math_clamp(Linear_Of(p_linear, x), INT16_MIN, INT16_MAX); }
/* [X0:XRef] => [0:32767] */
static inline ufract16_t Linear_Fract16_SatPos(const Linear_T * p_linear, int32_t x) { return math_clamp(Linear_Of(p_linear, x), 0, INT16_MAX); }

/* percent16 */
static inline void Linear_Percent16_Init(Linear_T * p_linear, int32_t x0, int32_t xRef) { Linear_Fixed_Init(p_linear, 16U, x0, xRef); }
/* [X0:XRef] => [0:65535] */
static inline uint16_t Linear_Percent16_Sat(const Linear_T * p_linear, int32_t x) { return math_clamp(Linear_Of(p_linear, x), 0, UINT16_MAX); }


#endif


