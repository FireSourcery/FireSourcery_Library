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
	@file 	math_foc.h
	@author FireSourcery
	@brief	FOC pure math functions.
			Aligned in CCW order: a, b, c; alpha, beta; d, q
	@version V0
*/
/******************************************************************************/
#ifndef MATH_FOC_H
#define MATH_FOC_H

#include "Math/Q/QFrac16.h"

/******************************************************************************/
/*!
	@brief	Clarke
			Transform 3-phase (120 degree) stationary reference frame quantities: Ia, Ib, Ic
			into 2-axis orthogonal stationary reference frame quantities: Ialpha and Ibeta

	Ialpha = (2*Ia - Ib - Ic)/3
	Ibeta = sqrt3/3*(Ib - Ic) = (Ib - Ic)/sqrt3

	Alternatively, Simplified:
	Ialpha = Ia
	Ibeta = (Ib - Ic)/sqrt(3)
	alpha = a;
	beta = (int32_t)qfrac16_mul(b, QFRAC16_1_DIV_SQRT3) - (int32_t)qfrac16_mul(c, QFRAC16_1_DIV_SQRT3);

	@param[out] p_alpha
	@param[out] p_beta
	@param[in] a
	@param[in] b
	@param[in] c
	@return  void
 */
 /******************************************************************************/
static inline void foc_clarke(qfrac16_t * p_alpha, qfrac16_t * p_beta, qfrac16_t a, qfrac16_t b, qfrac16_t c)
{
	int32_t alpha = qfrac16_mul((int32_t)a * 2 - (int32_t)b - (int32_t)c, QFRAC16_1_DIV_3);
	int32_t beta = qfrac16_mul((int32_t)b - (int32_t)c, QFRAC16_1_DIV_SQRT3);

	*p_alpha = qfrac16_sat(alpha);  /* Can 2*a - b - c > 113,509 ? */
	*p_beta = qfrac16_sat(beta);
}

/******************************************************************************/
/*!
	@brief	2-Phase Version

	Ialpha = Ia
	Ibeta = (Ia + 2*Ib)/sqrt3

	@param[out] p_alpha
	@param[out] p_beta
	@param[in] Ia
	@param[in] Ib
	@return  void
*/
/******************************************************************************/
static inline void foc_clarke_ab(qfrac16_t * p_alpha, qfrac16_t * p_beta, qfrac16_t a, qfrac16_t b)
{
	int32_t beta = qfrac16_mul((int32_t)a + (int32_t)b * 2, QFRAC16_1_DIV_SQRT3);

	*p_alpha = a;
	*p_beta = qfrac16_sat(beta);
}

/******************************************************************************/
/*!
	@brief	Inverse Clarke

	A = alpha;
	B = (-alpha + sqrt3*beta)/2
	C = (-alpha - sqrt3*beta)/2
*/
/******************************************************************************/
static inline void foc_invclarke(qfrac16_t * p_a, qfrac16_t * p_b, qfrac16_t * p_c, qfrac16_t alpha, qfrac16_t beta)
{
	int32_t alphaDiv2 = 0 - qfrac16_mul(alpha, QFRAC16_1_DIV_2);
	int32_t betaSqrt3Div2 = qfrac16_mul(beta, QFRAC16_SQRT3_DIV_2);

	int32_t b = alphaDiv2 + betaSqrt3Div2;
	int32_t c = alphaDiv2 - betaSqrt3Div2;

	*p_a = alpha;
	*p_b = qfrac16_sat(b);
	*p_c = qfrac16_sat(c);
}

/******************************************************************************/
/*!
	@brief  Park
			Transforms 2-axis orthogonal stationary reference frame quantities: Ialpha and Ibeta
			into 2-axis orthogonal rotor synchronous reference frame quantities: Id and Iq.

	Id = alpha*cos(theta) + beta*sin(theta)
	Iq = -alpha*sin(theta) + beta*cos(theta)

	@param[out] p_d
	@param[out] p_q
	@param[in] Ialpha
	@param[in] Ibeta
	@param[in] theta - rotating frame angle in q1.15 format
	@return void
*/
/******************************************************************************/
/* shared sin cos calc */
static inline void foc_park_vector(qfrac16_t * p_d, qfrac16_t * p_q, qfrac16_t alpha, qfrac16_t beta, qfrac16_t sin, qfrac16_t cos)
{
	int32_t d = (int32_t)qfrac16_mul(alpha, cos) + (int32_t)qfrac16_mul(beta, sin);
	int32_t q = (int32_t)qfrac16_mul(beta, cos) - (int32_t)qfrac16_mul(alpha, sin);

	*p_d = qfrac16_sat(d);
	*p_q = qfrac16_sat(q);
}

static inline void foc_park(qfrac16_t * p_d, qfrac16_t * p_q, qfrac16_t alpha, qfrac16_t beta, qangle16_t theta)
{
	qfrac16_t cos, sin;

	qfrac16_vector(&cos, &sin, theta);
	foc_park_vector(p_d, p_q, alpha, beta, sin, cos);
}

/******************************************************************************/
/*!
	@brief  Inverse Park

	alpha = -q*sin(theta) + d*cos(theta)
	beta = q*cos(theta) + d*sin(theta)
*/
/******************************************************************************/
static inline void foc_invpark_vector(qfrac16_t * p_alpha, qfrac16_t * p_beta, qfrac16_t d, qfrac16_t q, qfrac16_t sin, qfrac16_t cos)
{
	int32_t alpha = (int32_t)qfrac16_mul(d, cos) - (int32_t)qfrac16_mul(q, sin);
	int32_t beta = (int32_t)qfrac16_mul(d, sin) + (int32_t)qfrac16_mul(q, cos);

	*p_alpha = qfrac16_sat(alpha);
	*p_beta = qfrac16_sat(beta);
}

static inline void foc_invpark(qfrac16_t * p_alpha, qfrac16_t * p_beta, qfrac16_t d, qfrac16_t q, qangle16_t theta)
{
	qfrac16_t cos, sin;

	qfrac16_vector(&cos, &sin, theta);
	foc_invpark_vector(p_alpha, p_beta, d, q, sin, cos);
}

static inline uint16_t foc_magnitude(qfrac16_t d, qfrac16_t q)
{
	return q_sqrt((int32_t)d * (int32_t)d + (int32_t)q * (int32_t)q);
}

/* unitize, q d proportional */
/* saves sqrt operation if magnitude is not needed every cycle */
static inline void foc_circlelimit(qfrac16_t * p_d, qfrac16_t * p_q, qfrac16_t vectorMax)
{
	uint32_t vectorMaxSquared = (int32_t)vectorMax * (int32_t)vectorMax;
	uint32_t dqSquared = ((int32_t)(*p_d) * (int32_t)(*p_d)) + ((int32_t)(*p_q) * (int32_t)(*p_q));
	uint16_t vectorMagnitude;
	qfrac16_t ratio; /* where 32767 q1.15 = 1 */

	if(dqSquared > vectorMaxSquared)
	{
		vectorMagnitude = q_sqrt(dqSquared);
		ratio = qfrac16_div(vectorMax, vectorMagnitude);
		*p_d = (qfrac16_t)qfrac16_mul(*p_d, ratio); /* no saturation needed, ratio < 1 */
		*p_q = (qfrac16_t)qfrac16_mul(*p_q, ratio);
	}
}

/* limit, prioritize maintaining d */
static inline void foc_circlelimit_dmax(qfrac16_t * p_d, qfrac16_t * p_q, qfrac16_t vectorMax, qfrac16_t dMax)
{
	uint32_t vectorMaxSquared = (int32_t)vectorMax * (int32_t)vectorMax;
	uint32_t dSquared = (int32_t)(*p_d) * (int32_t)(*p_d);
	uint32_t qSquared = (int32_t)(*p_q) * (int32_t)(*p_q);
	uint32_t dqSquared = dSquared + qSquared;
	uint32_t qMaxSquared;
	uint16_t qMax;

	if(dqSquared > vectorMaxSquared)
	{
		if(qfrac16_abs(*p_d) > dMax)
		{
			*p_d = (*p_d < 0) ? 0 - dMax : dMax;
			dSquared = dMax * dMax;
		}
		qMaxSquared = vectorMaxSquared - dSquared;
		qMax = q_sqrt(qMaxSquared);
		*p_q = (*p_q < 0) ? 0 - qMax : qMax;
	}
}

#endif
