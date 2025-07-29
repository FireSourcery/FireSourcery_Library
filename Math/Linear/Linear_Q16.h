#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery

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
    @file   Linear_Q16.h
    @author FireSourcery
    @brief  Linear Fract16 without division
*/
/******************************************************************************/
#include "Linear.h"

#include "Math/Fixed/fixed.h"

/******************************************************************************/
/*!
    Fract16     => int16_t      as [-1:1)  in Q1.15
    UFract16    => uint16_t     as [0:2)   in Q1.15
    Percent16   => uint16_t     as [0:1)   in Q0.16
    Accum32     => int32_t      as [-65536:65536) in Q17.15
    (Storage)   => int32_t      as [-2:2)         in Q2.30
    Fract32     => int32_t      as [-1:1)         in Q1.31
    Fixed32     => int32_t      as [-32768:32768) in Q16.16
*/
/******************************************************************************/

/******************************************************************************/
/*!
    Helper
*/
/******************************************************************************/
static inline int16_t _Linear_SatSigned16(int32_t value16) { return ((int16_t)math_clamp(value16, INT16_MIN, INT16_MAX)); }
static inline int16_t _Linear_SatSigned16_Abs(int32_t value16) { return _Linear_SatSigned16(math_abs(value16)); }
static inline uint16_t _Linear_SatUnsigned16(int32_t value16) { return ((uint16_t)math_clamp(value16, 0, UINT16_MAX)); }
static inline uint16_t _Linear_SatUnsigned16_Abs(int32_t value16) { return _Linear_SatUnsigned16(math_abs(value16)); }


/******************************************************************************/
/*!

*/
/******************************************************************************/
/*!
    f([-XRef:XRef]) => [-65536:65536]
        with over saturation of [-65535*2:65535*2] (via INT32_MAX/2)

    @param[in]  x [-XRef*2:XRef*2]
    @return     y_fixed32 Q1.16 format. [-65535*2:65535*2]
*/
static inline int32_t Linear_Q16_Of(const Linear_T * p_linear, int32_t x)
{
    return linear_shift_f_x0(p_linear->Slope, p_linear->SlopeShift, p_linear->X0, x);
}

/*!
    @param[in] y_fixed32 in Q1.16 format.
*/
static inline int32_t Linear_Q16_InvOf(const Linear_T * p_linear, int32_t y_fixed32)
{
    return linear_shift_invf_x0(p_linear->InvSlope, p_linear->InvSlopeShift, p_linear->X0, y_fixed32);
}

/******************************************************************************/
/*!
    Convert Qx.15
*/
/******************************************************************************/
static inline int32_t Linear_Q16_Accum32(const Linear_T * p_linear, int32_t x) { return Linear_Q16_Of(p_linear, x) / 2; } // { return linear_shift_f_x0(p_linear->Slope, p_linear->SlopeShift + 1, p_linear->X0, x); }
static inline int16_t Linear_Q16_Fract(const Linear_T * p_linear, int32_t x) { return _Linear_SatSigned16(Linear_Q16_Accum32(p_linear, x)); }
static inline uint16_t Linear_Q16_UFract(const Linear_T * p_linear, int32_t x) { return _Linear_SatUnsigned16(Linear_Q16_Accum32(p_linear, x)); }

/* Input interval units */
static inline int32_t Linear_Q16_InvFract(const Linear_T * p_linear, int16_t y_fract16) { return Linear_Q16_InvOf(p_linear, (int32_t)y_fract16 * 2); }

/******************************************************************************/
/*!
    Saturate to Q0.16 [0:65535]
*/
/******************************************************************************/
static inline int32_t _Linear_Q16_Percent(const Linear_T * p_linear, int32_t x) { return Linear_Q16_Of(p_linear, x); }
static inline uint16_t Linear_Q16_Percent(const Linear_T * p_linear, int32_t x) { return _Linear_SatUnsigned16(Linear_Q16_Of(p_linear, x)); }
static inline uint16_t Linear_Q16_Percent_Abs(const Linear_T * p_linear, int32_t x) { return _Linear_SatUnsigned16_Abs(Linear_Q16_Of(p_linear, x)); }

/* Input interval units */
static inline int32_t Linear_Q16_InvPercent(const Linear_T * p_linear, uint16_t y_percent16) { return Linear_Q16_InvOf(p_linear, y_percent16); }


/******************************************************************************/
/*!
    Q8.8
*/
/******************************************************************************/
static inline int16_t Linear_Q16_General(const Linear_T * p_linear, int32_t x)
{
    return linear_shift_f_x0(p_linear->Slope, p_linear->SlopeShift + 8U, p_linear->X0, x);
}


/******************************************************************************/
/*!
    Extern
*/
/******************************************************************************/

/******************************************************************************/
/*!
    Q0.16 and Q1.15
    signed/unsigned 16-bit precision, using 32-bits for intermediate calculations

    [x0:xRef] => [0:65536] => [y0_Units:yRef_Units]
        direct shift back is Q16.16
    f(x0) = y0_Fixed == 0
    f(xRef) = 65535
*/
/******************************************************************************/
/*!
    Map [x0, xRef] to [0, 65535]
*/
static void Linear_Q16_Init(Linear_T * p_linear, int32_t x0, int32_t xRef)
{
    Linear_Fixed_Init(p_linear, 16U, x0, xRef);
}

// void Linear_Fixed_InitAsFract16(Linear_T * p_linear, int32_t x0, int32_t xRef)
// {
//     Linear_Fixed_Init(p_linear, 15U, x0, xRef);
// }

// void Linear_Fixed_InitAsPercent16(Linear_T * p_linear, int32_t x0, int32_t xRef)
// {
//     Linear_Fixed_Init(p_linear, 16U, x0, xRef);
// }


