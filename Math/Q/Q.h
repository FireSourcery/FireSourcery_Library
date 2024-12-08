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
    @file   Q.h
    @author FireSourcery
    @brief     Q fixed point math operations
    @version V0
*/
/******************************************************************************/
#ifndef Q_MATH_H
#define Q_MATH_H

#include <stdint.h>

extern uint16_t q_sqrt(int32_t x);
extern uint8_t q_log2(uint32_t x);
extern uint8_t q_log2_ceiling(uint32_t x);
extern uint8_t q_log2_round(uint32_t x);
extern uint32_t q_pow2_round(uint32_t x);
extern uint8_t q_lshift_max_signed(int32_t x);
extern uint8_t q_lshift_max_unsigned(uint32_t x);

#endif

/******************************************************************************/
/*
    Aliases, move to q

    fixed32 with division
    f16(x) = 65536 * (x - x0) / (xref - x0)
    invf16(y_fixed32) = y_fixed32 * (xref - x0) / 65536 - x0;
*/
/******************************************************************************/
// static inline int32_t linear_q(int32_t x0, int32_t deltax, int32_t q_delta, int32_t x)
// {
//     return linear_f_x0(q_delta, deltax, x0, x);
// }

// static inline int32_t linear_inv_q(int32_t x0, int32_t deltax, int32_t q_delta, int32_t value)
// {
//     return linear_invf_x0(q_delta, deltax, x0, value);
// }

// static inline int32_t linear_fixed32(int32_t x0, int32_t deltax, int32_t x)
// {
//     return linear_f_x0(65536, deltax, x0, x);
// }

// static inline int32_t linear_invfixed32(int32_t x0, int32_t deltax, int32_t y_fixed32)
// {
//     return linear_invf_x0(65536, deltax, x0, y_fixed32);
// }

/******************************************************************************/
/*
    with offset and shit
    Aliases
*/
/******************************************************************************/
// linear_shift_fixed32
// linear_shift_invfixed32
// static inline int32_t linear_units_of_fixed(uint8_t nBits, int32_t y0_units, int32_t deltay_units, int32_t y_fixed32)
// {
//     return ((y_fixed32 * deltay_units) >> nBits) + y0_units;
// }

// static inline int32_t linear_fixed_of_units(uint8_t nBits, int32_t y0_units, int32_t deltay_units, int32_t y_units)
// {
//     return (((y_units - y0_units) << nBits) / deltay_units);
// }

/* y_fixed32 * (yref_units - y0_units) >> 16U + y0_units */
/* fixed32 to y_units */
// static inline int32_t linear_units_of_fixed(int32_t y0_units, int32_t deltay_units, int32_t y_fixed32)
// {
//     return ((y_fixed32 * deltay_units) / 65536) + y0_units;
// }

// /* (y_units - y0_units) << 16U / (yref_units - y0_units) */
// /* y_units to fixed32 */
// static inline int32_t linear_fixed_of_units(int32_t y0_units, int32_t deltay_units, int32_t y_units)
// {
//     return ((y_units - y0_units) * 65536 / deltay_units);
// }