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
	@file 	math_svpwm.h
	@author FireSoucery
	@brief	SVPWM pure math functions.
	@version V0
*/
/*******************************************************************************/
#ifndef MATH_SVPWM_H
#define MATH_SVPWM_H

#include "Math/Q/QFrac16.h"


#define QFRAC16_N_FRAC_BITS_MINUS_1 14U
/*
 * Standard SVM calculation method. Inclusive of equivalent reverse Clarke transform.
 * Mid clamp, determining sector first. SVPWM determined by shifting magnitudes such that the midpoint is 50% PWM
 *
 * dutyA, dutyB, dutyC -> 16 bits, q0.16, always positive
 */
static inline void svpwm_midclamp(uint16_t * p_dutyA, uint16_t * p_dutyB, uint16_t * p_dutyC, qfrac16_t alpha, qfrac16_t beta)
{
	int32_t magX, magY, magZ, z0;

	/*
	 * Derives 3 magnitudes (duty cycles) belonging to basic unit vectors.
	 * The other 3 of 6 are inverse of the 3 derived
	 * Magnitudes are normalized by a factor of √(3)/2, i.e alpha = 1 => A = .866
	 *
	 * X = beta;
	 * Y = (beta + sqrt3 * alpha) / 2;
	 * Z = (beta - sqrt3 * alpha) / 2;
	 */
	magX = beta * QFRAC16_1_OVERSAT;
	magY = ((beta * QFRAC16_1_OVERSAT) + (QFRAC16_SQRT3_MOD_1 * alpha) + (alpha * QFRAC16_1_OVERSAT)) / 2;
	magZ = ((beta * QFRAC16_1_OVERSAT) - (QFRAC16_SQRT3_MOD_1 * alpha) - (alpha * QFRAC16_1_OVERSAT)) / 2;

	if (magX >= 0)
	{
		if (magZ < 0)
		{
			/*
			 * Sector 1: X >= 0 and Z < 0
			 *
			 * Duty Cycle:
			 * A:		|v100| = (sqrt3 * alpha - beta) / 2 = -Z;
			 * invC:	|v110| = beta = X;
			 *
			 * SVPWM:
			 * A -> Max, B -> Mid, C -> Min
			 * z0 = (1/2) - (max + min)/2 = (1 - (-Z - X))/2
			 * A = z0 - Z;
			 * B = z0;
			 * C = z0 - X;
			 *
			 * A = (1 + X - Z) / 2;
			 * B = (1 + X + Z) / 2;
			 * C = (1 - X + Z) / 2;
			 */
			z0 = (QFRAC16_1_OVERSAT*QFRAC16_1_OVERSAT + magX + magZ) / 2;
			*p_dutyA = (z0 - magZ) 	>> QFRAC16_N_FRAC_BITS_MINUS_1;
			*p_dutyB = z0 			>> QFRAC16_N_FRAC_BITS_MINUS_1;
			*p_dutyC = (z0 - magX) 	>> QFRAC16_N_FRAC_BITS_MINUS_1;
		}
		else if (magY >= 0)
		{
			/*
			 * Sector 2: Y >= 0 and Z >= 0
			 *
			 *
			 * Duty Cycle:
			 * invC: 	|v110| = 1 * (beta + sqrt3 * alpha) / 2 = Y;
			 * B:	 	|v010| = 1 * (beta - sqrt3 * alpha) / 2 = Z;
			 *
			 * SVPWM:
			 * A -> Mid, B -> Max, C -> Min
			 * z0 = (1/2) - (max + min)/2 = (1 - (Z - Y))/2
			 * A = z0;
			 * B = z0 + Z;
			 * C = z0 - Y;
			 */
			z0 = (QFRAC16_1_OVERSAT*QFRAC16_1_OVERSAT +  magY -  magZ) / 2;
			*p_dutyA = z0 			>> QFRAC16_N_FRAC_BITS_MINUS_1;
			*p_dutyB = (z0 + magZ) 	>> QFRAC16_N_FRAC_BITS_MINUS_1;
			*p_dutyC = (z0 - magY) 	>> QFRAC16_N_FRAC_BITS_MINUS_1;
		}
		else
		{
			/*
			 * Sector 3: X >= 0 and Y < 0
			 *
			 * Duty Cycle:
			 * B: 		|v010| = X;
			 * invA:	|v011| = -Y;
			 *
			 * SVPWM:
			 * A -> Min, B -> Max, C -> Mid
			 * z0 = (1 - (X + Y))/2
			 * A = z0 + Y;
			 * B = z0 + X;
			 * C = z0;
			 */
			z0 = (QFRAC16_1_OVERSAT*QFRAC16_1_OVERSAT -  magX -  magY) / 2;
			*p_dutyA = (z0 + magY) 	>> QFRAC16_N_FRAC_BITS_MINUS_1;
			*p_dutyB = (z0 + magX) 	>> QFRAC16_N_FRAC_BITS_MINUS_1;
			*p_dutyC = z0 			>> QFRAC16_N_FRAC_BITS_MINUS_1;
		}
	}
	else
	{
		if (magZ >= 0)
		{
			/*
			 * Sector 4: X < 0 and Z >= 0
			 *
			 * Duty Cycle:
			 * invA: 	|v011| = Z;
			 * C:		|v001| = -X;
			 *
			 * SVPWM:
			 * A -> Min, B -> Mid, C -> Max
			 * z0 = (1 - (-X - Z))/2
			 * A = z0 - Z;
			 * B = z0;
			 * C = z0 - X;
			 */
			z0 = (QFRAC16_1_OVERSAT*QFRAC16_1_OVERSAT +  magX +  magZ) / 2;
			*p_dutyA = (z0 - magZ) 	>> QFRAC16_N_FRAC_BITS_MINUS_1;
			*p_dutyB = z0 			>> QFRAC16_N_FRAC_BITS_MINUS_1;
			*p_dutyC = (z0 - magX) 	>> QFRAC16_N_FRAC_BITS_MINUS_1;
		}
		else if (magY < 0)
		{
			/*
			 * Sector 5:  Y < 0 and Z < 0
			 *
			 * Duty Cycle:
			 * C: 		|v001| = -Y;
			 * invB:	|v101| = -Z;
			 *
			 * SVPWM:
			 * A -> Mid, B -> Min, C -> Max
			 * z0 = (1 - (-Y + Z))/2
			 * A = z0;
			 * B = z0 + Z;
			 * C = z0 - Y;
			 */
			z0 = (QFRAC16_1_OVERSAT*QFRAC16_1_OVERSAT +  magY -  magZ) / 2;
			*p_dutyA = z0 			>> QFRAC16_N_FRAC_BITS_MINUS_1;
			*p_dutyB = (z0 + magZ) 	>> QFRAC16_N_FRAC_BITS_MINUS_1;
			*p_dutyC = (z0 - magY) 	>> QFRAC16_N_FRAC_BITS_MINUS_1;
		}
		else
		{
			/*
			 * Sector 6: X < 0 and Y >= 0
			 *
			 * Duty Cycle:
			 * invB: 	|v101| = -X;
			 * A:		|v100| = Y;
			 *
			 * SVPWM:
			 * A -> Max, B -> Min, C -> Mid
			 * z0 = (1 - (X + Y))/2
			 * A = z0 + Y;
			 * B = z0 + X;
			 * C = z0;
			 */
			z0 = (QFRAC16_1_OVERSAT*QFRAC16_1_OVERSAT - magX -  magY) / 2;
			*p_dutyA = (z0 + magY) 	>> QFRAC16_N_FRAC_BITS_MINUS_1;
			*p_dutyB = (z0 + magX) 	>> QFRAC16_N_FRAC_BITS_MINUS_1;
			*p_dutyC = z0			>> QFRAC16_N_FRAC_BITS_MINUS_1;
		}
	}
}

//static inline void svpwm_calc2(uint16_t * p_pwmA, uint16_t * p_pwmB, uint16_t * p_pwmC, uint16_t pwmPeriod, qfrac16_t vA, qfrac16_t vB, qfrac16_t vC)
//{
//
//}
//
//static inline void svpwm_calc3(uint16_t * p_pwmA, uint16_t * p_pwmB, uint16_t * p_pwmC, uint16_t pwmPeriod, qfrac16_t d, qfrac16_t q, qfrac16_t theta)
//{
//
//}
//
//static inline void svpwm_calc4(uint16_t * p_pwmA, uint16_t * p_pwmB, uint16_t * p_pwmC, uint16_t pwmPeriod, qfrac16_t magnitude, qfrac16_t theta)
//{
//
//}

extern const uint16_t MATH_SVPWM_SADDLE_120[170];

static inline uint16_t svpwm_saddle120(qangle16_t theta)
{
	return MATH_SVPWM_SADDLE_120[(((uint16_t)theta) >> 7U)];
}
static inline uint16_t svpwm_saddle(qangle16_t theta)
{
	uint16_t saddle;

	if ((uint16_t)theta < (uint16_t)QANGLE16_120)
	{
		saddle = svpwm_saddle120(theta);
	}
	else if ((uint16_t)theta < (uint16_t)QANGLE16_240)
	{
		saddle = svpwm_saddle120((QANGLE16_240 - 1U - theta));
	}
	else
	{
		saddle = 0U;
	}

	return saddle;
}

/*
 *
 */
static inline void svpwm_unipolar1(uint16_t * p_dutyA, uint16_t * p_dutyB, uint16_t * p_dutyC, uint16_t dutyScalar, qangle16_t rotorAngle, qangle16_t voltageLeadAngle)
{
	qangle16_t angleA;
	qangle16_t angleB;
	qangle16_t angleC;

	uint16_t saddleA;
	uint16_t saddleB;
	uint16_t saddleC;

	#define PHASE_SHIFT_A  QANGLE16_120
	#define PHASE_SHIFT_B  QANGLE16_240
	#define PHASE_SHIFT_C  0

	angleA = PHASE_SHIFT_A + rotorAngle + voltageLeadAngle; // angle loops
	angleB = PHASE_SHIFT_B + rotorAngle + voltageLeadAngle;
	angleC = PHASE_SHIFT_C + rotorAngle + voltageLeadAngle;

	saddleA = svpwm_saddle(angleA);
	saddleB = svpwm_saddle(angleB);
	saddleC = svpwm_saddle(angleC);

	//saddle and duty are NOT qfrac16
	*p_dutyA = ((uint32_t)saddleA * (uint32_t)dutyScalar) / 65536U;
	*p_dutyB = ((uint32_t)saddleB * (uint32_t)dutyScalar) / 65536U;
	*p_dutyC = ((uint32_t)saddleC * (uint32_t)dutyScalar) / 65536U;
}

#endif
