/*******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSoucery / The Firebrand Forge Inc

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
/*******************************************************************************/
/*******************************************************************************/
/*!
    @file 	Linear.c
    @author FireSoucery
    @brief  Mathematical linear function.
    @version V0
*/
/*******************************************************************************/
#ifndef MATH_LINEAR_H
#define MATH_LINEAR_H

#include <stdint.h>



static inline int32_t linear_f(int32_t m_factor, int32_t m_divisor, int32_t b, int32_t x)
{
	return (x * m_factor / m_divisor + b);
}

/*
 * y = m*x + b, m in f(x) direction
 */
static inline int32_t linear_invf(int32_t m_factor, int32_t m_divisor, int32_t b, int32_t y)
{
	return ((y - b) * m_divisor / m_factor);
}

static inline int32_t linear_f_round(int32_t m_factor, int32_t m_divisor, int32_t b, int32_t x)
{
	return ((m_factor * x + (m_divisor / 2)) / m_divisor + b); // add (line->SlopeDivisor/2) to round up .5
}

static inline int32_t linear_invf_round(int32_t m_factor, int32_t m_divisor, int32_t b, int32_t y)
{
	return (((y - b) * m_divisor + (m_factor / 2)) / m_factor); // add (line->SlopeFactor/2) to round up .5
}

static inline int32_t linear_f_frac16(int32_t m_factor, int32_t m_divisor, int32_t b, int32_t yref, int32_t x)
{
	//	return ((m_factor * x * 65536) / (m_divisor * yref) + (b * 65536 / yref));
	//	return ((m_factor * x * 65536 / m_divisor) + (b * 65536)) / yref;
	return ((m_factor * x + b * m_divisor) * 65536 / (m_divisor * yref));
}

static inline int32_t linear_invf_frac16(int32_t m_factor, int32_t m_divisor, int32_t b, int32_t yref, int32_t y_frac16)
{
	return m_divisor * (y_frac16 * yref - (b * 65536)) / (m_factor * 65536); //todo check
}



static inline int32_t linear_f_shift(int32_t m_shifted, uint8_t shift, int32_t b_shifted, int32_t x)
{
//	return (((m_shifted * x) >> (shift)) + b);
//	return ((m_shifted * x + b_shifted) >> shift);
	return (linear_f(m_shifted, 1U, b_shifted, x) >> shift);
}

static inline int32_t linear_invf_shift(int32_t invm_shifted, uint8_t shift, int32_t b_shifted, int32_t y)
{
//	return (((y - b) * invm_shifted) >> shift);
//	return (((y <<shift) - b_shifted) * invm_shifted) >> shift ;
	return (linear_invf(1U, invm_shifted, b_shifted, (y << shift)) >> shift);
}

static inline int32_t linear_invf_yfrac16_shift(int32_t invm_shifted, uint8_t shift, int32_t b_shifted, int32_t y_frac16)
{
	return (linear_invf(1U, invm_shifted, b_shifted, y_frac16) >> shift); //todo check
}

static inline int32_t linear_f_frac16_shift(int32_t m_shift16, int32_t b_shift16, int32_t yref, int32_t x)
{
//	return (m_shift16 * x + b_shift16) / yref;
	return (linear_f(m_shift16, 1U, b_shift16, x) / yref);
}

static inline int32_t linear_invf_frac16_shift(int32_t m_shift16, int32_t b_shift16, int32_t yref, int32_t y)
{

}

/*
 * y = m*(x - x0), m in f(x) direction
 */
static inline int32_t linear_f_x0(int32_t m_factor, int32_t m_divisor, int32_t x0, int32_t x)
{
	return linear_invf(m_divisor, m_factor, x, x0);
}

#endif
