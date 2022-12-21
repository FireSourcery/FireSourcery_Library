/******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSourcery / The Firebrand Forge Inc

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
	@file 	Linear.c
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

/* Overflow: m_shifted * (x - x0) > INT32_MAX[2,147,483,647] */
static inline int32_t linear_f_shift(int32_t m_shifted, uint8_t shift, int32_t x0, int32_t y0, int32_t x)
{
	return (((m_shifted * (x - x0)) >> shift) + y0);
}

/* f(x0) = 0 */
static inline int32_t linear_f_x0_shift(int32_t m_shifted, uint8_t shift, int32_t x0, int32_t x)
{
	return ((m_shifted * (x - x0)) >> shift);
}

static inline int32_t linear_f_y0_shift(int32_t m_shifted, uint8_t shift, int32_t y0, int32_t x)
{
	return (((m_shifted * x) >> shift) + y0);
}

/******************************************************************************/
/*
	invf(y) = (y - y0) * (1 / m) + x0, m in f(x) direction
*/
/******************************************************************************/
static inline int32_t linear_invf(int32_t m_factor, int32_t m_divisor, int32_t b, int32_t y)
{
	return linear_f_x0(m_divisor, m_factor, b, y);
}

static inline int32_t linear_invf_x0(int32_t m_factor, int32_t m_divisor, int32_t x0, int32_t y)
{
	return linear_f(m_divisor, m_factor, x0, y);
}

/* (((invm_shifted * (y - y0)) >> shift) + x0) */
static inline int32_t linear_invf_shift(int32_t invm_shifted, uint8_t shift, int32_t x0, int32_t y0, int32_t y)
{
	return linear_f_shift(invm_shifted, shift, y0, x0, y);
}

/* (((invm_shifted * (y)) >> shift) + x0) */
static inline int32_t linear_invf_x0_shift(int32_t invm_shifted, uint8_t shift, int32_t x0, int32_t y)
{
	return linear_f_y0_shift(invm_shifted, shift, x0, y);
}

static inline int32_t linear_invf_y0_shift(int32_t invm_shifted, uint8_t shift, int32_t y0, int32_t y)
{
	return linear_f_x0_shift(invm_shifted, shift, y0, y);
}

/******************************************************************************/
/*
*/
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
	f16(x) = 65536 * (x - x0) / (xref - x0)
	invf16(y_frac16) = y_frac16 * (xref - x0) / 65536 - x0;
*/
/******************************************************************************/
static inline int32_t linear_f16(int32_t x0, int32_t deltax, int32_t x)
{
	return ((x - x0) * 65536 / (deltax));
}

static inline int32_t linear_invf16(int32_t x0, int32_t deltax, int32_t y_frac16)
{
	return y_frac16 * (deltax) / 65536 - x0;
}

/******************************************************************************/
/*
	when slope is set to frac16 conversion, m16_f returns user units secondarily
	m16_shifted == [65536 / (xref - x0) << shift]
 	f16([x0:xRef]) = [0:65536]
	g([0:65536]) = [y0_Units:yRef_Units]
	f([x0:xRef]) = g(f16([x0:xRef])) = [y0_Units:yRef_Units]
*/
/******************************************************************************/
/* ((m16_shifted * (x - x0)) >> shift) */
/* x to y_frac16 */
static inline int32_t linear_m16_f16(int32_t m16_shifted, uint8_t shift, int32_t x0, int32_t x)
{
	return linear_f_x0_shift(m16_shifted, shift, x0, x);
}

/* (((invm16_shifted * (y_frac16 - 0)) >> shift) + x0) */
/* y_frac16 to x */
static inline int32_t linear_m16_invf16(int32_t invm16_shifted, uint8_t shift, int32_t x0, int32_t y_frac16)
{
	return linear_invf_x0_shift(invm16_shifted, shift, x0, y_frac16);
}

/* y_frac16 * (yref_units - y0_units) >> 16U + y0_units */
/* frac16 to y_units */
static inline int32_t linear_m16_g(int32_t y0_units, int32_t deltay_units, int32_t y_frac16)
{
	return ((y_frac16 * deltay_units) / 65536) + y0_units;
}

/* (y_units - y0_units) << 16U / (yref_units - y0_units) */
/* y_units to frac16 */
static inline int32_t linear_m16_invg(int32_t y0_units, int32_t deltay_units, int32_t y_units)
{
	return ((y_units - y0_units) * 65536 / deltay_units);
}

/* f(x) = f16(x) * (yref - y0) >> 16 + y0 */
/* x to y_units */
static inline int32_t linear_m16_f(int32_t m16_shifted, uint8_t shift, int32_t x0, int32_t y0_units, int32_t deltay_units, int32_t x)
{
	return linear_m16_g(deltay_units, y0_units, linear_m16_f16(m16_shifted, shift, x0, x));
}

/* y_units to x */
static inline int32_t linear_m16_invf(int32_t invm16_shifted, uint8_t shift, int32_t x0, int32_t y0_units, int32_t deltay_units, int32_t y_units)
{
	return  linear_m16_invf16(invm16_shifted, shift, x0, linear_m16_invg(deltay_units, y0_units, y_units));
}

#endif