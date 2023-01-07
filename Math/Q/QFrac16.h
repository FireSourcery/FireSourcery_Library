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
	@file 	Q.h
	@author FireSourcery
	@brief 	Math with 16 bit fractions in Q1.15 format
	@version V0
*/
/******************************************************************************/
#ifndef QFRAC16_H
#define QFRAC16_H

#include "Q.h"
#include "Math/math_general.h"
#include <stdint.h>
#include <stdbool.h>

#define QFRAC16_N_FRAC_BITS (15U) /*!< Q1.15, 15 fractional bits, shift mul/div by 32768 */

typedef int16_t qfrac16_t; 		/*!< Q1.15 [-1.0, 0.999969482421875], res 1/(2^15) == .000030517578125 */
// typedef int32_t qfrac16_t; 		/* Allow calculation with over saturation */

static const qfrac16_t QFRAC16_MAX = INT16_MAX; /*!< (32767) */
static const qfrac16_t QFRAC16_MIN = INT16_MIN; /*!< (-32768) */

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

static const int32_t QFRAC16_1_OVERSAT 	= 0x00008000; /*!< (32768) */
static const int32_t QFRAC16_PI 		= 0x0001921F; /* Over saturated */
static const int32_t QFRAC16_3PI_DIV_4 	= 0x00012D97; /* Over saturated */

#define QFRAC16_FLOAT_MAX (0.999969482421875F)
#define QFRAC16_FLOAT_MIN (-1.0F)
#define QFRAC16(x) ((qfrac16_t)(((x) < QFRAC16_FLOAT_MAX) ? (((x) >= QFRAC16_FLOAT_MIN) ? ((x)*32768.0F) : INT16_MIN) : INT16_MAX))

static inline qfrac16_t qfrac16(int16_t num, int32_t max) { return (qfrac16_t)(((int32_t)num << QFRAC16_N_FRAC_BITS) / max); }
static inline qfrac16_t qfrac16_convert(int16_t num, int32_t max) { return qfrac16(num, max); }
static inline qfrac16_t qfrac16_sat(int32_t qfrac) { return math_clamp(qfrac, QFRAC16_MIN, QFRAC16_MAX); }

/*!
	@brief Unsaturated Multiply

	overflow input max factor1 * factor2 < INT32_MAX (2,147,483,647)
		e.g. (65,536, 32,767), (131,072, 16,383)

	qfrac16_mul(+/-32768, +/-32768) returns 32768 [0x8000]
		(int32_t)32768 -> positive 32768, over saturated 1
		(int16_t)32768 -> -32768, -1

	@return int32_t[-65536:65535] <=> [-2:2)
*/
static inline int32_t qfrac16_mul(int32_t factor, int32_t frac)
{
	return (((int32_t)factor * (int32_t)frac) >> QFRAC16_N_FRAC_BITS);
}

/*!
	Saturate to QFRAC16_MIN, QFRAC16_MAX

	qfrac16_mul(int16_t factor, int16_t frac) must still check for 32768 case
		(product == +32768) ? 32767 : product;

	@return int16_t[-32768, 32767] <=> [-1:1)
*/
static inline qfrac16_t qfrac16_mul_sat(int32_t factor, int32_t frac)
{
	return qfrac16_sat(qfrac16_mul(factor, frac));
}

/*!
	@brief Unsaturated Divide

	dividend >= divisor returns [-1073741824:1073709056] <=> (-32768, 32767)
		over saturated qfrac16_t, 32768 [0x8000] -> over saturated 1
	dividend < divisor returns [-32767:32767] <=> (-1:1)
		within qfrac16_t range

	@return int32_t[-1073741824:1073709056], [0XC0000000, 0X3FFF8000]
*/
static inline int32_t qfrac16_div(int16_t dividend, int32_t divisor)
{
	return (((int32_t)dividend << QFRAC16_N_FRAC_BITS) / (int32_t)divisor);
}

/*!
	@return int16_t[-32768, 32767]
*/
static inline qfrac16_t qfrac16_div_sat(int16_t dividend, int32_t divisor)
{
	return qfrac16_sat(qfrac16_div(dividend, divisor));
}

static inline qfrac16_t qfrac16_abs(qfrac16_t x)
{
	qfrac16_t val;
	if(x < 0) 	{ val = (x == -32768) ? 32767 : 0 - x; }
	else 		{ val = x; }
	return val;
}

static inline qfrac16_t qfrac16_sqrt(qfrac16_t x)
{
	return q_sqrt((int32_t)x << QFRAC16_N_FRAC_BITS);
}

/******************************************************************************/
/*!
	qangle16
*/
/******************************************************************************/
typedef int16_t qangle16_t; 	/*!< [-pi, pi) signed or [0, 2pi) unsigned, angle loops. */

#define QFRAC16_SINE_90_TABLE_LENGTH 	(256U)
#define QFRAC16_SINE_90_TABLE_LSB 		(6U)	/*!< Insignificant bits, shifted away*/
extern const qfrac16_t QFRAC16_SINE_90_TABLE[QFRAC16_SINE_90_TABLE_LENGTH];	/*! Resolution: 1024 steps per revolution */

static const qangle16_t QANGLE16_0 = 0; 		/*! 0 */
static const qangle16_t QANGLE16_30 = 0x1555; 	/*! 5461 */
static const qangle16_t QANGLE16_60 = 0x2AAA; 	/*! 10922 */
static const qangle16_t QANGLE16_90 = 0x4000; 	/*! 16384 */
static const qangle16_t QANGLE16_120 = 0x5555;	/*! 21845 */
static const qangle16_t QANGLE16_150 = 0x6AAA;	/*! 27306 */
static const qangle16_t QANGLE16_180 = 0x8000;	/*! 32768, -32768, 180 == -180 */
static const qangle16_t QANGLE16_210 = 0x9555;	/*! 38229 */
static const qangle16_t QANGLE16_240 = 0xAAAA;	/*! 43690, -21845 */
static const qangle16_t QANGLE16_270 = 0xC000;	/*! 49152, -16384, 270 == -90 */

#define QANGLE16_QUADRANT_MASK (0xC000U)

typedef enum qangle16_quadrant_tag
{
	QANGLE16_QUADRANT_I, 	/* 0_90 */
	QANGLE16_QUADRANT_II, 	/* 90_180 */
	QANGLE16_QUADRANT_III, 	/* 180_270 */
	QANGLE16_QUADRANT_IV, 	/* 270_360 */
}
qangle16_quadrant_t;

static inline qangle16_quadrant_t qangle16_quadrant(qangle16_t theta)
{
	qangle16_quadrant_t quadrant;
	switch((uint16_t)theta & QANGLE16_QUADRANT_MASK)
	{
		case (uint16_t)QANGLE16_0: 		quadrant = QANGLE16_QUADRANT_I; break;
		case (uint16_t)QANGLE16_90: 	quadrant = QANGLE16_QUADRANT_II; break;
		case (uint16_t)QANGLE16_180: 	quadrant = QANGLE16_QUADRANT_III; break;
		case (uint16_t)QANGLE16_270: 	quadrant = QANGLE16_QUADRANT_IV; break;
		default: quadrant = 0U; break; /* Should not occur */
	}
	return quadrant;
}


static inline bool qangle16_cycle(qangle16_t theta0, qangle16_t theta1)
{
	return ((theta0 < 0) && (theta1 > 0));
}

static inline bool qangle16_cycle2(qangle16_t theta0, qangle16_t theta1)
{
	// return (((theta0 ^ theta1) & 0x8000U) != (uint16_t)0U);
	return ((theta0 ^ theta1) < 0);
}

static inline bool qangle16_cycle4(qangle16_t theta0, qangle16_t theta1)
{
	return (((theta0 ^ theta1) & QANGLE16_QUADRANT_MASK) != (uint16_t)0U);
}

/*
	0b xx11 1111 11xx xxxx
	Use 8 most significant digits of 90 degree bound.
	Remove sign / 180 degree bit, 90 degree bit, and 6 lsb.
*/
static inline qfrac16_t sin90(qangle16_t theta)
{
	return QFRAC16_SINE_90_TABLE[(uint8_t)(theta >> QFRAC16_SINE_90_TABLE_LSB)];
}

static inline qfrac16_t cos90(qangle16_t theta)
{
	return QFRAC16_SINE_90_TABLE[(0xFFU - (uint8_t)(theta >> QFRAC16_SINE_90_TABLE_LSB))];
}

/*
	[0, 90)		=> [0x0000, 0x3FFF]	=> [0, 0xFF] == [0, 1)
	[90, 180)	=> [0x4000, 0x7FFF] => [0xFF, 0] == (1, 0]
	[180, 270)	=> [0x8000, 0xBFFF] => [0, 0xFF] == [0, -1)
	[270, 360)	=> [0xC000, 0xFFFF] => [0xFF, 0] == (-1, 0]
*/
static inline qfrac16_t qfrac16_sin(qangle16_t theta)
{
	qfrac16_t sine;
	switch(qangle16_quadrant(theta))
	{
		case QANGLE16_QUADRANT_I: 	sine = sin90(theta); 							break;
		case QANGLE16_QUADRANT_II: 	sine = sin90(QANGLE16_180 - 1 - theta); 		break;
		case QANGLE16_QUADRANT_III: sine = 0 - sin90(theta); 						break;
		case QANGLE16_QUADRANT_IV: 	sine = 0 - sin90(QANGLE16_180 - 1 - theta); 	break;
		default: sine = 0; break;
	}
	return sine;
}

static inline qfrac16_t qfrac16_cos(qangle16_t theta)
{
	qfrac16_t cosine;
	switch(qangle16_quadrant(theta))
	{
		case QANGLE16_QUADRANT_I: 	cosine = sin90(QANGLE16_180 - 1 - theta); 		break;
		case QANGLE16_QUADRANT_II: 	cosine = 0 - sin90(theta); 						break;
		case QANGLE16_QUADRANT_III: cosine = 0 - sin90(QANGLE16_180 - 1 - theta); 	break;
		case QANGLE16_QUADRANT_IV: 	cosine = sin90(theta); 							break;
		default: cosine = 0; break;
	}
	return cosine;
}

/* compiler optimize into single switch? */
static inline void qfrac16_vector(qfrac16_t * p_cos, qfrac16_t * p_sin, qangle16_t theta)
{
	*p_sin = qfrac16_sin(theta);
	*p_cos = qfrac16_cos(theta);
	return; /* (*p_cos, *p_sin) */
}

static inline uint16_t qfrac16_vectormagnitude(qfrac16_t x, qfrac16_t y)
{
	return q_sqrt((int32_t)x * x + (int32_t)y * y);
}

/* circle limit */
static inline void qfrac16_vectorlimit(qfrac16_t * p_x, qfrac16_t * p_y, qfrac16_t magnitudeMax)
{
	uint32_t magnitudeMaxSquared = (int32_t)magnitudeMax * magnitudeMax;
	uint32_t vectorMagnitudeSquared = ((int32_t)(*p_x) * (*p_x)) + ((int32_t)(*p_y) * (*p_y));
	uint16_t vectorMagnitude;
	qfrac16_t ratio; /* where 32767 q1.15 ~= 1 */

	if(vectorMagnitudeSquared > magnitudeMaxSquared)
	{
		vectorMagnitude = q_sqrt(vectorMagnitudeSquared);
		ratio = qfrac16_div(magnitudeMax, vectorMagnitude);  /* no saturation needed, vectorMagnitude < magnitudeMax */
		*p_x = (qfrac16_t)qfrac16_mul(*p_x, ratio); /* no saturation needed, ratio < 1 */
		*p_y = (qfrac16_t)qfrac16_mul(*p_y, ratio);
	}
	return;
}

/*
	Adapted from libfixmath https://github.com/PetteriAimonen/libfixmath/blob/master/libfixmath/qfrac16_trig.c
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

	if(y < 0) { angle = 0 - angle; }

	return angle; /* angle loops, no need to saturate */
}

#endif
