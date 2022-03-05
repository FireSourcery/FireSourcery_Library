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
	@file 	math_foc.h
	@author FireSoucery
	@brief	FOC pure math functions.
			Aligned in CW order: a, b, c; alpha, beta; d, q
	@version V0
*/
/******************************************************************************/
#ifndef MATH_FOC_H
#define MATH_FOC_H

#include "Math/Q/QFrac16.h"

/******************************************************************************/
/*!
	@brief	Transform 3-phase (120 degree) stationary reference frame quantities: Ia, Ib, Ic
			into 2-axis orthogonal stationary reference frame quantities: Ialpha and Ibeta

	@param[out] p_alpha
	@param[out] p_beta
	@param[in] a
	@param[in] b
	@param[in] c
	@return  void

	Ialpha = (2*Ia - Ib - Ic)/3
	Ibeta = sqrt3/3*(Ib - Ic) = (Ib - Ic)/sqrt3

	Simplified:
	Ialpha = Ia
	Ibeta = (Ib - Ic)/sqrt(3)

	//	alpha = a;
	//	beta = (int32_t)qfrac16_mul(b, QFRAC16_1_DIV_SQRT3) - (int32_t)qfrac16_mul(c, QFRAC16_1_DIV_SQRT3);

 */
/******************************************************************************/
static inline void foc_clarke(qfrac16_t * p_alpha, qfrac16_t * p_beta, qfrac16_t a, qfrac16_t b, qfrac16_t c)
{
	int32_t alpha, beta;

	alpha = qfrac16_mul((int32_t)a*2 - (int32_t)b - (int32_t)c, QFRAC16_1_DIV_3);
	beta = qfrac16_mul((int32_t)b - (int32_t)c, QFRAC16_1_DIV_SQRT3);

	*p_alpha = alpha;	// qfrac16_sat(alpha);
	*p_beta = beta; 	//qfrac16_sat(beta);
}

///******************************************************************************/
///*!
//	@brief	2-Phase Version
//
//	Ialpha = Ia
//	Ibeta = (Ia + 2*Ib)/sqrt3
//
//	@param[out] p_alpha
//	@param[out] p_beta
//	@param[in] Ia
//	@param[in] Ib
//	@return  void
//  */
///******************************************************************************/
//static inline void foc_clarke_ab(qfrac16_t * p_alpha, qfrac16_t * p_beta, qfrac16_t a, qfrac16_t b)
//{
//	int32_t beta;
//
//	beta = (int32_t)qfrac16_mul(a, QFRAC16_1_DIV_SQRT3) + (int32_t)qfrac16_mul(b, QFRAC16_1_DIV_SQRT3) * (int32_t)2;
//
//	*p_alpha = a;
//	*p_beta = qfrac16_sat(beta);
//
//	return;
//}

/******************************************************************************/
/*!
	@brief	Inverse Clark

	A = alpha;
	B = (-alpha + sqrt3*beta)/2
	C = (-alpha - sqrt3*beta)/2

 */
/******************************************************************************/
static inline void foc_invclarke(qfrac16_t * p_A, qfrac16_t * p_B, qfrac16_t * p_C, qfrac16_t alpha, qfrac16_t beta)
{
	int32_t b, c;
	int32_t alphaDiv2 		= 0 - qfrac16_mul(alpha, QFRAC16_1_DIV_2);
	int32_t betaSqrt3Div2 	= qfrac16_mul(beta, QFRAC16_SQRT3_DIV_2);

	b = alphaDiv2 + betaSqrt3Div2;
	c = alphaDiv2 - betaSqrt3Div2;

	*p_A = alpha;
	*p_B = b; //qfrac16_sat(b);
	*p_C = c; //qfrac16_sat(c);
}

/******************************************************************************/
/*!
	@brief  Transforms 2-axis orthogonal stationary reference frame quantities: Ialpha and Ibeta
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
static inline void foc_park(qfrac16_t * p_d, qfrac16_t * p_q, qfrac16_t alpha, qfrac16_t beta, qangle16_t theta)
{
	qfrac16_t cos, sin;
	int32_t d, q;

	qfrac16_vector(&cos, &sin, theta);

	d = (int32_t)qfrac16_mul(alpha, cos) + (int32_t)qfrac16_mul(beta, sin);
	q = (int32_t)qfrac16_mul(beta, cos) - (int32_t)qfrac16_mul(alpha, sin);

	*p_d = qfrac16_sat(d);
	*p_q = qfrac16_sat(q);
}

/*
 * shared sin cos calc
 */
static inline void foc_park_vector(qfrac16_t * p_d, qfrac16_t * p_q, qfrac16_t alpha, qfrac16_t beta, qfrac16_t sin, qfrac16_t cos)
{
	int32_t d, q;

	d = (int32_t)qfrac16_mul(alpha, cos) + (int32_t)qfrac16_mul(beta, sin);
	q = (int32_t)qfrac16_mul(beta, cos) - (int32_t)qfrac16_mul(alpha, sin);

	*p_d = qfrac16_sat(d);
	*p_q = qfrac16_sat(q);
}

/******************************************************************************/
/*!
	@brief  Inverse Park

	alpha = d*cos(theta) - q*sin(theta)
	beta = d*sin(theta) + q*cos(theta)
  */
/******************************************************************************/
static inline void foc_invpark(qfrac16_t * p_alpha, qfrac16_t * p_beta, qfrac16_t d, qfrac16_t q, qangle16_t theta)
{
	qfrac16_t cos, sin;
	int32_t alpha, beta;

	qfrac16_vector(&cos, &sin, theta);

	alpha = (int32_t)qfrac16_mul(d, cos) - (int32_t)qfrac16_mul(q, sin);
	beta = (int32_t)qfrac16_mul(d, sin) + (int32_t)qfrac16_mul(q, cos);

	*p_alpha = qfrac16_sat(alpha);
	*p_beta = qfrac16_sat(beta);
}

static inline void foc_invpark_vector(qfrac16_t * p_alpha, qfrac16_t * p_beta, qfrac16_t d, qfrac16_t q, qfrac16_t sin, qfrac16_t cos)
{
	int32_t alpha, beta;

	alpha = (int32_t)qfrac16_mul(d, cos) - (int32_t)qfrac16_mul(q, sin);
	beta = (int32_t)qfrac16_mul(d, sin) + (int32_t)qfrac16_mul(q, cos);

	*p_alpha = qfrac16_sat(alpha);
	*p_beta = qfrac16_sat(beta);
}


// bound q d proportionally
static inline void foc_limitvector(qfrac16_t * p_d, qfrac16_t * p_q, qfrac16_t vectorMax)
{
	int32_t vectorMaxSquared =  (int32_t)vectorMax * (int32_t)vectorMax; //32767*32767 for 1;
	int32_t dqSquared = ((int32_t)(*p_d)*(int32_t)(*p_d)) + ((int32_t)(*p_q)*(int32_t)(*p_q));
	qfrac16_t vectorMagnitutde;

	if (dqSquared > vectorMaxSquared)
	{
		dqSquared = (dqSquared>>15U); // normalize to frac after multiplying
		vectorMagnitutde = qfrac16_sqrt(dqSquared);
		*p_d = (qfrac16_t)((int32_t)(*p_d) * (int32_t)vectorMax / (int32_t)vectorMagnitutde);
		*p_q = (qfrac16_t)((int32_t)(*p_q) * (int32_t)vectorMax / (int32_t)vectorMagnitutde);
	}
}

// prioritize maintaining d
static inline void foc_limitvector_dmax(qfrac16_t * p_d, qfrac16_t * p_q, qfrac16_t vectorMax, qfrac16_t dMax)
{
	int32_t vectorMaxSquared = (int32_t)vectorMax * (int32_t)vectorMax; //max 32767*32767;
	int32_t dSquared = (int32_t)(*p_d) * (int32_t)(*p_d);
	int32_t qSquared = (int32_t)(*p_q) * (int32_t)(*p_q);
	int32_t dqSquared = dSquared + qSquared;
	int32_t qMaxSquared;

	if (dqSquared > vectorMaxSquared)
	{
		if (qfrac16_abs(*p_d) > dMax)
		{
			if ((*p_d) < 0)
			{
				*p_d = -dMax;
			}
			else
			{
				*p_d = dMax;
			}
			qMaxSquared = vectorMaxSquared - (dMax*dMax);
		}
		else
		{
			qMaxSquared = vectorMaxSquared - dSquared;
		}

		if ((*p_q) < 0)
		{
			*p_q = 0 - qfrac16_sqrt(qMaxSquared);
		}
		else
		{
			*p_q = qfrac16_sqrt(qMaxSquared);
		}
	}
}

#endif
