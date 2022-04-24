/**************************************************************************/
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
/**************************************************************************/
/**************************************************************************/
/*!
	@file 	Q.h
	@author FireSoucery
	@brief 	Math with 16 bit fractions in Q1.15 format
	@version V0
*/
/**************************************************************************/
#ifndef QFRAC16_H
#define QFRAC16_H

#include "Q.h"

#include <stdint.h>

#define QFRAC16_N_FRAC_BITS (15U) /*!< Q1.15, 15 fractional bits, shift mul/div by 32768 */

typedef int16_t qfrac16_t; 		/*!< Q1.15 [-1.0, 0.999969482421875], res 1/(2^15) == .000030517578125 */
typedef int16_t qangle16_t; 	/*!< [-pi, pi) signed or [0, 2pi) unsigned, angle loops. */

static const qfrac16_t QFRAC16_MAX = 0x7FFF; /*!< (32767) */
static const qfrac16_t QFRAC16_MIN = (int16_t) 0x8000; /*!< (-32768) */

static const qfrac16_t QFRAC16_1_DIV_2 = 0x4000; /*!< 16384 */
static const qfrac16_t QFRAC16_1_DIV_4 = 0x2000;
static const qfrac16_t QFRAC16_3_DIV_4 = 0x6000;
static const qfrac16_t QFRAC16_1_DIV_3 = 0x2AAA;
static const qfrac16_t QFRAC16_2_DIV_3 = 0x5555;
static const qfrac16_t QFRAC16_1_DIV_SQRT3 = 0x49E7; /*!< 0.57735026919f */
static const qfrac16_t QFRAC16_SQRT3_DIV_2 = 0x6EDA;
static const qfrac16_t QFRAC16_SQRT3_DIV_4 = 0x376D;
static const qfrac16_t QFRAC16_SQRT2_DIV_2 = 0x5A82;

static const qfrac16_t QFRAC16_PI_DIV_4 = 0x6487;
static const int32_t QFRAC16_PI = 0x0001921F;			/* Oversaturated */
static const int32_t QFRAC16_3PI_DIV_4 = 0x00012D97; 	/* Oversaturated */

static const int32_t QFRAC16_1_OVERSAT = (int32_t)0x00008000; /*!< (32768) */

static const qangle16_t QANGLE16_90 = 0x4000; /*!< 16384 */
static const qangle16_t QANGLE16_180 = 0x8000; /*!< 32768, -32768, 180 == -180 */
static const qangle16_t QANGLE16_270 = 0xC000; /*!< 49152, -16384, 270 == -90 */

static const qangle16_t QANGLE16_0 = 0; 		/*!  */
static const qangle16_t QANGLE16_120 = 0x5555; 	/*!  */
static const qangle16_t QANGLE16_240 = 0xAAAA; 	/*!  */

#define SINE_90_TABLE_ENTRIES 	256
#define SINE_90_TABLE_LSB 		6	/*!< Insignificant bits, shifted away*/

/*! Resolution: 1024 steps per revolution */
extern const qfrac16_t QFRAC16_SINE_90_TABLE[SINE_90_TABLE_ENTRIES];

#define QFRAC16(x) ((qfrac16_t) (((x) < 1.0) ? (((x) >= -1.0) ? (x*32768.0) : INT16_MIN) : INT16_MAX))

static inline qfrac16_t qfrac16(int16_t num, int32_t max)
{
	return (qfrac16_t) (((int32_t)num << QFRAC16_N_FRAC_BITS) / max);
}

static inline qfrac16_t qfrac16_convert(int16_t num, int32_t max)
{
	return qfrac16(num, max);
}

static inline qfrac16_t qfrac16_sat(int32_t qfrac)
{
	qfrac16_t sat;

	if 		(qfrac > (int32_t)QFRAC16_MAX) 	{sat = QFRAC16_MAX;}
	else if (qfrac < (int32_t)QFRAC16_MIN) 	{sat = QFRAC16_MIN;}
	else 									{sat = (qfrac16_t)qfrac;}

	return sat;
}

/*!
	@brief Unsaturated multiply
	@return [-32767, 32768] as int32_t
	qfrac16_mul(frac, frac) 	returns frac value, 0x8000 -> over saturated 1
	qfrac16_mul(integer, frac) 	returns integer value, 0x8000 -> positive 32768

	0x8000 casts from -32768 int16_t to -32768 int32_t correctly
	0x8000 [-32768] * 0x8000 [-32768] returns as positive int32_t 0x00008000
	0x8000 casts from 32768 int32_t to -32768 int16_t incorrectly
	must call sat to convert to correct int16_t value
 */
static inline int32_t qfrac16_mul(int32_t factor, int32_t frac)
{
	return (((int32_t)factor * (int32_t)frac) >> QFRAC16_N_FRAC_BITS);
}

/*!
	@return [-32767, 32767] as int16_t
 */
static inline qfrac16_t qfrac16_mul_sat(int32_t factor, int32_t frac)
{
	int32_t product = qfrac16_mul(factor, frac);

	if (product == 32768) product = 32767;

	return product;
}

/*!
	@return [-1073741824, 1073709056] as int32_t
	@return [0XC0000000, 0X3FFF8000]
	(frac, frac) 		returns frac value
		when dividend >= divisor, over saturated qfrac16_t. 0x8000 -> over saturated 1
		when dividend < divisor, within qfrac16_t range
	(integer, frac) 	returns integer value
*/
static inline int32_t qfrac16_div(int16_t dividend, int32_t divisor)
{
	return (((int32_t)dividend << QFRAC16_N_FRAC_BITS) / (int32_t)divisor);
}

/*!
	@return [-32768, 32767] as int16_t
 */
static inline qfrac16_t qfrac16_div_sat(int16_t dividend, int32_t divisor)
{
	int32_t quotient = qfrac16_div(dividend, divisor);
	return qfrac16_sat(quotient);
}

static inline qfrac16_t qfrac16_abs(qfrac16_t x)
{
	qfrac16_t val = x;

	if (val < 0)
	{
		if (val == -32768)
		{
			val = 32767;
		}
		else
		{
			val = -val;
		}
	}

	return val;
}

static inline qfrac16_t qfrac16_sqrt(int32_t x)
{
	return q_sqrt((int32_t)x << QFRAC16_N_FRAC_BITS);
}

/*
 * 0b so11 1111 11xx xxxx
 * use 8 most significant digits of 90 degree bound.
 * remove sign / 180 degree place bit, 90 degree place bit, and 6 lsb.
 */
static inline qfrac16_t sin90(qangle16_t theta)
{
    return QFRAC16_SINE_90_TABLE[(uint8_t)(theta >> SINE_90_TABLE_LSB)];
}

static inline qfrac16_t cos90(qangle16_t theta)
{
    return QFRAC16_SINE_90_TABLE[((uint8_t)0xFF - (uint8_t)(theta >> SINE_90_TABLE_LSB))];
}

/*
 * [0, 90)		=> [0x0000, 0x3FFF]	=> [0, 0xFF] == [0, 1)
 * [90, 180)	=> [0x4000, 0x7FFF] => [0xFF, 0] == (1, 0]
 * [180, 270)	=> [0x8000, 0xBFFF] => [0, 0xFF] == [0, -1)
 * [270, 360)	=> [0xC000, 0xFFFF] => [0xFF, 0] == (-1, 0]
 *
 *  can also check first 2 bits for branch
 *  #define QFRAC16_SIN_MASK 	0xC000
 *	#define QANGLE16_0_90    	0x0000
 *	#define QANGLE16_90_180   	0x4000
 *	#define QANGLE16_180_270  	0x8000
 *	#define QANGLE16_270_360  	0xC000
 */
static inline qfrac16_t qfrac16_sin(qangle16_t theta)
{
	if ((uint16_t)theta < (uint16_t)QANGLE16_180)
	{
		if ((uint16_t)theta < (uint16_t)QANGLE16_90)	return sin90(theta);
		else											return sin90(QANGLE16_180 - 1 - theta);
	}
	else
	{
		if ((uint16_t)theta < (uint16_t)QANGLE16_270) 	return (0 - sin90(theta));
		else											return (0 - sin90(QANGLE16_180 - 1 - theta));
	}
}

static inline qfrac16_t qfrac16_cos(qangle16_t theta)
{
	if ((uint16_t)theta < (uint16_t)QANGLE16_180)
	{
		if ((uint16_t)theta < (uint16_t)QANGLE16_90)	return sin90(QANGLE16_180 - 1 - theta);
		else											return (0 - sin90(theta));
	}
	else
	{
		if ((uint16_t)theta < (uint16_t)QANGLE16_270) 	return (0 - sin90(QANGLE16_180 - 1 - theta));
		else											return sin90(theta);
	}
}

static inline void qfrac16_vector(qfrac16_t * p_cos, qfrac16_t * p_sin, qangle16_t theta)
{
	if ((uint16_t)theta < (uint16_t)QANGLE16_180)
	{
		if ((uint16_t)theta < (uint16_t)QANGLE16_90)
		{
			*p_sin = sin90(theta);
			*p_cos = sin90(QANGLE16_180 - 1 - theta);
		}
		else
		{
			*p_sin = sin90(QANGLE16_180 - 1 - theta);
			*p_cos = 0 - sin90(theta);
		}
	}
	else
	{
		if ((uint16_t)theta < (uint16_t)QANGLE16_270)
		{
			*p_sin = 0 - sin90(theta);
			*p_cos = 0 - sin90(QANGLE16_180 - 1 - theta);;
		}
		else
		{
			*p_sin = 0 - sin90(QANGLE16_180 - 1 - theta);
			*p_cos = sin90(theta);
		}
	}

	return;
}

/*
 * from libfixmath https://github.com/PetteriAimonen/libfixmath/blob/master/libfixmath/qfrac16_trig.c
 */
static inline qangle16_t qfrac16_atan2(qfrac16_t y, qfrac16_t x)
{
	int32_t mask = (y >> QFRAC16_N_FRAC_BITS);
	int32_t yAbs = (y + mask) ^ mask;
	int32_t r, r_3, angle;

	if(x >= 0)
	{
		r = qfrac16_div((x - yAbs), (x + yAbs));
		r_3 = qfrac16_mul(qfrac16_mul(r, r), r);
		angle = qfrac16_mul(0x07FF, r_3) - qfrac16_mul(0x27FF, r) + QFRAC16_1_DIV_4;
	}
	else
	{
		r = qfrac16_div((x + yAbs), (yAbs - x));
		r_3 = qfrac16_mul(qfrac16_mul(r, r), r);
		angle = qfrac16_mul(0x07FF, r_3) - qfrac16_mul(0x27FF, r) + QFRAC16_3_DIV_4;
	}

	if (y < 0) {angle = 0 - angle;}

	return angle; /* angle loops, no need to saturate */
}

#endif
