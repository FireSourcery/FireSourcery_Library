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
    @file   Linear_Q16_Of.h
    @author FireSourcery
    @brief  Linear Fract16 calc without division

*/
/******************************************************************************/
#ifndef LINEAR_Q16_H
#define LINEAR_Q16_H

#include "Linear.h"

#include "Math/Fixed/fixed.h"

/******************************************************************************/
/*!
    Fract16     => [-32768:32767],  [-1:1)  in Q1.15
    UFract16    => [0:65535],       [0:2)   in Q1.15
    Percent16   => [0:65535],       [0:1)   in Q0.16
    Accum32     => [INT32_MIN:INT32_MAX], [2*-32768:2*32768) in Q17.15
    Fixed32     => [INT32_MIN:INT32_MAX], [-32768:32768) in Q16.16
*/
/******************************************************************************/
/*!
    f([-XRef:XRef]) => [-65536:65536]
        with over saturation of [-65535*2:65535*2] (via INT32_MAX/2)

    @param[in]  x [-XRef*2:XRef*2]
    @return     y_fixed32 q16.16 format. [-65535*2:65535*2]
*/
static inline int32_t Linear_Q16_Of(const Linear_T * p_linear, int32_t x)
{
    return linear_shift_f_x0(p_linear->Slope, p_linear->SlopeShift, p_linear->X0, x);
}

/*!
    @param[in] y_fixed32 in q16.16 format. range bounded to 17-bits
*/
static inline int32_t Linear_Q16_InvOf(const Linear_T * p_linear, int32_t y_fixed32)
{
    return linear_shift_invf_x0(p_linear->InvSlope, p_linear->InvSlopeShift, p_linear->X0, y_fixed32);
}

/******************************************************************************/
/*!
    Convert and Saturate to int16_t, Q1.15 [-32768:32767]
*/
/******************************************************************************/
static inline int16_t Linear_Q16_Fract(const Linear_T * p_linear, int32_t x)
{
    return _Linear_SatSigned16(Linear_Q16_Of(p_linear, x) / 2);
    // return linear_shift_f_x0(p_linear->Slope, p_linear->SlopeShift, p_linear->X0, x);
}

/* [0:2) */
static inline uint16_t Linear_Q16_UFract(const Linear_T * p_linear, int32_t x)
{
    return _Linear_SatUnsigned16(Linear_Q16_Of(p_linear, x) / 2);
    // return linear_shift_f_x0(p_linear->Slope, p_linear->SlopeShift, p_linear->X0, x);
}

/* Input interval units */
static inline int32_t Linear_Q16_InvFract(const Linear_T * p_linear, int16_t y_fract16)
{
    return Linear_Q16_InvOf(p_linear, (int32_t)y_fract16 * 2);
}

/******************************************************************************/
/*!
    Saturate to uint16_t, Q0.16 [0:65535]
*/
/******************************************************************************/
static inline uint16_t Linear_Q16_Percent(const Linear_T * p_linear, int32_t x)
{
    return _Linear_SatUnsigned16(Linear_Q16_Of(p_linear, x));
    // return linear_shift_f_x0(p_linear->Slope, p_linear->SlopeShift - 1, p_linear->X0, x);
}

static inline uint16_t Linear_Q16_Percent_Abs(const Linear_T * p_linear, int32_t x)
{
    return _Linear_SatUnsigned16_Abs(Linear_Q16_Of(p_linear, x));
}

/* Input interval units */
static inline int32_t Linear_Q16_InvPercent(const Linear_T * p_linear, uint16_t y_percent16)
{
    return Linear_Q16_InvOf(p_linear, y_percent16);
}


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
extern void Linear_Q16_Init(Linear_T * p_linear, int32_t x0, int32_t xRef);


#endif
