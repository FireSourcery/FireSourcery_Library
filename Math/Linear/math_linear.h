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
/*  */
static inline int32_t linear_f(int32_t m_factor, int32_t m_divisor, int32_t b, int32_t x) { return (x * m_factor / m_divisor + b); }
static inline int32_t linear_f_x0(int32_t m_factor, int32_t m_divisor, int32_t x0, int32_t x) { return ((x - x0) * m_factor / m_divisor); }
static inline int32_t linear_f_full(int32_t m_factor, int32_t m_divisor, int32_t x0, int32_t y0, int32_t x) { return ((x - x0) * m_factor / m_divisor + y0); }

/* ((y - b) * m_divisor / m_factor + 0) */
static inline int32_t linear_invf(int32_t m_factor, int32_t m_divisor, int32_t b, int32_t y) { return linear_f_x0(m_divisor, m_factor, b, y); }

/* ((y - 0) * m_divisor / m_factor + x0) */
static inline int32_t linear_invf_x0(int32_t m_factor, int32_t m_divisor, int32_t x0, int32_t y) { return linear_f(m_divisor, m_factor, x0, y); }

/* ((y - y0) * m_divisor / m_factor + x0) */
static inline int32_t linear_invf_full(int32_t m_factor, int32_t m_divisor, int32_t x0, int32_t y0, int32_t y) { return linear_f_full(m_divisor, m_factor, y0, x0, y); }

/******************************************************************************/
/* */
/******************************************************************************/
static inline int32_t linear_f_round(int32_t m_factor, int32_t m_divisor, int32_t b, int32_t x) { return ((m_factor * x + (m_divisor / 2)) / m_divisor + b); }
static inline int32_t linear_invf_round(int32_t m_factor, int32_t m_divisor, int32_t b, int32_t y) { return (((y - b) * m_divisor + (m_factor / 2)) / m_factor); }

/******************************************************************************/
/*
    Shift implementation
    Naming pattern function_[implementation_variant]_[parameter_variant]
*/
/******************************************************************************/
/* With both parameters as default */
/* Overflow: m_shifted * (x - x0) > INT32_MAX */
static inline int32_t linear_shift_f(int32_t m_shifted, uint8_t shift, int32_t x0, int32_t y0, int32_t x) { return (((m_shifted * (x - x0)) >> shift) + y0); }

/* f(x0) = 0 */
static inline int32_t linear_shift_f_x0(int32_t m_shifted, uint8_t shift, int32_t x0, int32_t x) { return ((m_shifted * (x - x0)) >> shift); }
static inline int32_t linear_shift_f_y0(int32_t m_shifted, uint8_t shift, int32_t y0, int32_t x) { return (((m_shifted * x) >> shift) + y0); }
static inline int32_t linear_shift_f_round(int32_t m_shifted, uint8_t shift, int32_t x0, int32_t y0, int32_t x) { return (((m_shifted * (x - x0) + (1 << (shift - 1))) >> shift) + y0); }

/* (((invm_shifted * (y - y0)) >> shift) + x0) */
static inline int32_t linear_shift_invf(int32_t invm_shifted, uint8_t shift, int32_t x0, int32_t y0, int32_t y) { return linear_shift_f(invm_shifted, shift, y0, x0, y); }

/* (((invm_shifted * (y)) >> shift) + x0) */
static inline int32_t linear_shift_invf_x0(int32_t invm_shifted, uint8_t shift, int32_t x0, int32_t y) { return linear_shift_f_y0(invm_shifted, shift, x0, y); }

/* ((invm_shifted * (y - y0)) >> shift) */
static inline int32_t linear_shift_invf_y0(int32_t invm_shifted, uint8_t shift, int32_t y0, int32_t y) { return linear_shift_f_x0(invm_shifted, shift, y0, y); }

/* (((invm_shifted * (y - y0) + (1 << (shift-1))) >> shift) + x0) */
static inline int32_t linear_shift_invf_round(int32_t invm_shifted, uint8_t shift, int32_t x0, int32_t y0, int32_t y) { return linear_shift_f_round(invm_shifted, shift, y0, x0, y); }

#endif