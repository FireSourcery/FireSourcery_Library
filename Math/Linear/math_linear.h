/******************************************************************************/
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
/******************************************************************************/
/******************************************************************************/
/*!
	@file 	Linear.c
	@author FireSoucery
	@brief  Mathematical linear function.
	@version V0
*/
/******************************************************************************/
#ifndef MATH_LINEAR_H
#define MATH_LINEAR_H

#include <stdint.h>

/******************************************************************************/
/*
	f(x) 	= (x - x0) * m + y0
		Using Shift:
			= (((m_shifted * (x - x0)) >> shift) + y0)
		Using f16,yref:
			= f16(x) * yref >> 16

	f16(x)	=
		Using xref:
			= 65536 * (x - x0) / (xref - x0)
		Using xref, shift:
			= ((m16_shifted[65536 / (xref - x0) << shift] * (x - x0)) >> shift) + y0_frac16
		Using f, yref:
			= (65536 / yref) * f(x)
			= ((m_factor * x * 65536 / m_divisor) + (y0 * 65536)) / yref;
			= ((m_factor * x + y0 * m_divisor) * 65536) / (m_divisor * yref); Overflow
		Using f, yref, shift:
			= ((m_shifted * x >> shift) + y0) * 65536 / yref;
			= ((m_shifted * x >> (shift - 16)) + y0 * 65536) / yref;
*/
/******************************************************************************/
static inline int32_t linear_f(int32_t m_factor, int32_t m_divisor, int32_t b, int32_t x)
{
	return (x * m_factor / m_divisor + b);
}

static inline int32_t linear_f_round(int32_t m_factor, int32_t m_divisor, int32_t b, int32_t x)
{
	return ((m_factor * x + (m_divisor / 2)) / m_divisor + b);
}

static inline int32_t linear_f_x0(int32_t m_factor, int32_t m_divisor, int32_t x0, int32_t x)
{
	return ((x - x0) * m_factor / m_divisor);
}

/* Overflow: m_shifted * (x - x0) must be < INT32_MAX [2,147,483,647] */
static inline int32_t linear_f_shift(int32_t m_shifted, uint8_t shift, int32_t x0, int32_t y0, int32_t x)
{
	return (((m_shifted * (x - x0)) >> shift) + y0);
}

// static inline int32_t linear_f_x0_shift(int32_t m_shifted, uint8_t shift, int32_t x0, int32_t x)
// {
// 	return ((m_shifted * (x - x0)) >> shift); /* linear_invf(m_divisor, m_factor, x0, x); */
// }

static inline int32_t linear_f16(int32_t x0, int32_t xref, int32_t x)
{
	return ((x - x0) * 65536 / (xref - x0));
}

/* Overflow: 65536 * m_factor * x must be < INT32_MAX [2,147,483,647] */
static inline int32_t linear_f16_yref(int32_t m_factor, int32_t m_divisor, int32_t b, int32_t yref, int32_t x)
{
	return ((m_factor * x * 65536 / m_divisor) + (b * 65536)) / yref;
}

static inline int32_t linear_f16_yref_shift(int32_t m_shifted, uint8_t shift, int32_t x0, int32_t y0, int32_t yref, int32_t x)
{
	int32_t yfrac16;

	if(shift >= 16U)
	{
		yfrac16 = linear_f_shift(m_shifted, (shift - 16U), x0, y0 << 16U, x) / yref;
	}
	else
	{
		yfrac16 = (((m_shifted * (x - x0)) << (16U - shift)) + (y0 << 16U)) / yref;
	}

	return yfrac16;
}

static inline int32_t linear_f_b_shift(int32_t m_shifted, uint8_t shift, int32_t b_shifted, int32_t x)
{
	return ((m_shifted * x + b_shifted) >> shift);
}

static inline int32_t linear_f16_yref_b_shift(int32_t m_shifted, uint8_t shift, int32_t b_shifted, int32_t yref, int32_t x)
{
	int32_t yfrac16;

	if(shift >= 16U)
	{
		yfrac16 = linear_f_b_shift(m_shifted, (shift - 16U), b_shifted, x) / yref;
	}
	else
	{
		yfrac16 = ((m_shifted * x + b_shifted) << (16U - shift)) / yref;
	}

	return yfrac16;
}

/*
	invf(y) 	= (y - y0) * (1 / m) + x0, m in f(x) direction
		Using shift:
				= (((invm_shifted * (y - y0)) >> shift) + x0);

	invf16(y_frac16) = x
		Using yref:
				= invf(yref * y_frac16 / 65536)
				= ((yref * y_frac16 / 65536) - b) * m_divisor / m_factor);
				= (m_divisor * (y_frac16 * yref - b * 65536)) / (m_factor * 65536); Overflow
		Using xref:
*/

static inline int32_t linear_invf(int32_t m_factor, int32_t m_divisor, int32_t b, int32_t y)
{
	return linear_f_x0(m_divisor, m_factor, b, y);
}

static inline int32_t linear_invf_round(int32_t m_factor, int32_t m_divisor, int32_t b, int32_t y)
{
	return (((y - b) * m_divisor + (m_factor / 2)) / m_factor);
}

static inline int32_t linear_invf_x0(int32_t m_factor, int32_t m_divisor, int32_t x0, int32_t y)
{
	return linear_f(m_divisor, m_factor, x0, y);
}

static inline int32_t linear_invf_shift(int32_t invm_shifted, uint8_t shift, int32_t x0, int32_t y0, int32_t y)
{
	return linear_f_shift(invm_shifted, shift, y0, x0, y);
}

static inline int32_t linear_invf16(int32_t x0, int32_t xref, int32_t y_frac16)
{
	return y_frac16 * (xref - x0) / 65536 + x0;
}

static inline int32_t linear_invf16_yref(int32_t m_factor, int32_t m_divisor, int32_t b, int32_t yref, int32_t y_frac16)
{
	return linear_invf(m_factor, m_divisor, b, y_frac16 * yref / 65536);
}

static inline int32_t linear_invf16_yref_shift(int32_t invm_shifted, uint8_t shift, int32_t x0, int32_t y0, int32_t yref, int32_t y_frac16)
{
	return linear_invf_shift(invm_shifted, shift, x0, y0, y_frac16 * yref >> 16U);
}

//static inline int32_t linear_invf_b_shift(int32_t invm_shifted, uint8_t shift, int32_t b_shifted, int32_t y)
//{
//	return ((((y << shift) - b_shifted) >> shift) * invm_shifted >> shift); //b is shifted same as m but may not be shifted same as invm
//}

//static inline int32_t linear_invf16_b_shift(int32_t invm_shifted, uint8_t shift, int32_t b_shifted, int32_t yref, int32_t y_frac16)
//{
//	return (((y_frac16 * yref - b_shifted) >> 16U)) * invm_shifted >> shift;
//}

/******************************************************************************/
/*
	when slope is set to frac16 conversion, f_m16 returns user units secondarily
*/
/******************************************************************************/
/* e.g. x to y_frac16 */
static inline int32_t linear_m16_f16(int32_t m16_shifted, uint8_t shift, int32_t x0, int32_t y0_frac16, int32_t x)
{
	return linear_f_shift(m16_shifted, shift, x0, y0_frac16, x);
}

/* e.g. x to y_units */
static inline int32_t linear_m16_f(int32_t m16_shifted, uint8_t shift, int32_t x0, int32_t y0_frac16, int32_t yref_units, int32_t x)
{
	return linear_f_shift(m16_shifted, shift, x0, y0_frac16, x) * yref_units >> 16U;
}

/* e.g. y_frac16 to x */
static inline int32_t linear_m16_invf16(int32_t invm16_shifted, uint8_t shift, int32_t x0, int32_t y0_frac16, int32_t y_frac16)
{
	return linear_invf_shift(invm16_shifted, shift, x0, y0_frac16, y_frac16);
}

/* e.g. y_units to x */
static inline int32_t linear_m16_invf(int32_t invm16_shifted, uint8_t shift, int32_t x0, int32_t y0_frac16, int32_t yref_units, int32_t y)
{
	return linear_invf_shift(invm16_shifted, shift, x0, y0_frac16, y << 16U / yref_units);
}

/* e.g. y_units to frac16 */
// y<<16U/yref_units

/* e.g. frac16 to y_units */
// y_frac16*yref_units>>16U
#endif
