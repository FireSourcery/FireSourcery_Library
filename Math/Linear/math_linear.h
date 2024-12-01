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
    @file   math_linear.h
    @author FireSourcery
    @brief  Linear pure functions.
    @version V0
*/
/******************************************************************************/
#ifndef MATH_LINEAR_H
#define MATH_LINEAR_H

#include <stdint.h>

/******************************************************************************/
/*
    f(x) = (x - x0) * m + y0
    invf(y) = (y - y0) * (1 / m) + x0, m in f(x) direction
*/
/******************************************************************************/
static inline int32_t linear_f(int32_t m_factor, int32_t m_divisor, int32_t b, int32_t x)
{
    return (x * m_factor / m_divisor + b);
}

static inline int32_t linear_f_x0(int32_t m_factor, int32_t m_divisor, int32_t x0, int32_t x)
{
    return ((x - x0) * m_factor / m_divisor);
}

static inline int32_t linear_invf(int32_t m_factor, int32_t m_divisor, int32_t b, int32_t y)
{
    return linear_f_x0(m_divisor, m_factor, b, y); /* ((y - b) * m_divisor / m_factor + x0) */
}

static inline int32_t linear_invf_x0(int32_t m_factor, int32_t m_divisor, int32_t x0, int32_t y)
{
    return linear_f(m_divisor, m_factor, x0, y); /* ((y - b) * m_divisor / m_factor + x0) */
}

/******************************************************************************/
/* */
/******************************************************************************/
static inline int32_t linear_f_round(int32_t m_factor, int32_t m_divisor, int32_t b, int32_t x)
{
    return ((m_factor * x + (m_divisor / 2)) / m_divisor + b);
}

static inline int32_t linear_invf_round(int32_t m_factor, int32_t m_divisor, int32_t b, int32_t y)
{
    return (((y - b) * m_divisor + (m_factor / 2)) / m_factor);
}

/******************************************************************************/
/*
    Shift implementation
    Naming pattern function_[implementation_variant]_[parameter_variant]
*/
/******************************************************************************/
/* Overflow: m_shifted * (x - x0) > INT32_MAX */
static inline int32_t linear_shift_f(int32_t m_shifted, uint8_t shift, int32_t x0, int32_t y0, int32_t x)
{
    return (((m_shifted * (x - x0)) >> shift) + y0);
}

/* f(x0) = 0 */
static inline int32_t linear_shift_f_x0(int32_t m_shifted, uint8_t shift, int32_t x0, int32_t x)
{
    return ((m_shifted * (x - x0)) >> shift);
}

static inline int32_t linear_shift_f_y0(int32_t m_shifted, uint8_t shift, int32_t y0, int32_t x)
{
    return (((m_shifted * x) >> shift) + y0);
}

static inline int32_t linear_shift_invf(int32_t invm_shifted, uint8_t shift, int32_t x0, int32_t y0, int32_t y)
{
    return linear_shift_f(invm_shifted, shift, y0, x0, y); /* (((invm_shifted * (y - y0)) >> shift) + x0) */
}

static inline int32_t linear_shift_invf_x0(int32_t invm_shifted, uint8_t shift, int32_t x0, int32_t y)
{
    return linear_shift_f_y0(invm_shifted, shift, x0, y); /* (((invm_shifted * (y)) >> shift) + x0) */
}

static inline int32_t linear_shift_invf_y0(int32_t invm_shifted, uint8_t shift, int32_t y0, int32_t y)
{
    return linear_shift_f_x0(invm_shifted, shift, y0, y); /* ((invm_shifted * (y - y0)) >> shift) */
}

/******************************************************************************/
/*
    Aliases

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

static inline int32_t linear_fixed32(int32_t x0, int32_t deltax, int32_t x)
{
    return linear_f_x0(65536, deltax, x0, x);
}

static inline int32_t linear_invfixed32(int32_t x0, int32_t deltax, int32_t y_fixed32)
{
    return linear_invf_x0(65536, deltax, x0, y_fixed32);
}

/******************************************************************************/
/*
    Aliases
*/
/******************************************************************************/
// linear_shift_fixed32
// linear_shift_invfixed32
// static inline int32_t linear_units_of_fixed(int32_t nBits, int32_t y0_units, int32_t deltay_units, int32_t y_fixed32)
// {
//     return ((y_fixed32 * deltay_units) / nBits) + y0_units;
// }

// static inline int32_t linear_fixed_of_units(int32_t nBits, int32_t y0_units, int32_t deltay_units, int32_t y_units)
// {
//     return ((y_units - y0_units) * nBits / deltay_units);
// }

/* y_fixed32 * (yref_units - y0_units) >> 16U + y0_units */
/* fixed32 to y_units */
static inline int32_t linear_units_of_fixed(int32_t y0_units, int32_t deltay_units, int32_t y_fixed32)
{
    return ((y_fixed32 * deltay_units) / 65536) + y0_units;
}

/* (y_units - y0_units) << 16U / (yref_units - y0_units) */
/* y_units to fixed32 */
static inline int32_t linear_fixed_of_units(int32_t y0_units, int32_t deltay_units, int32_t y_units)
{
    return ((y_units - y0_units) * 65536 / deltay_units);
}


#endif