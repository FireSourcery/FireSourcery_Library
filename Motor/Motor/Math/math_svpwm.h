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
    @file   math_svpwm.h
    @author FireSourcery
    @brief  SVPWM pure math functions.
    @version V0
*/
/******************************************************************************/
#ifndef MATH_SVPWM_H
#define MATH_SVPWM_H

#include "Math/Q/QFrac16.h"

#include <assert.h>

/*
    Standard SVM calculation method. Inclusive of equivalent reverse Clarke transform.
    Mid clamp, determining sector first. SVPWM determined by shifting magnitudes such that the midpoint is 50% PWM

    dutyA, dutyB, dutyC -> 16 bits, q1.15, always positive
*/
static inline void svpwm_midclamp(uint16_t * p_dutyA, uint16_t * p_dutyB, uint16_t * p_dutyC, qfrac16_t alpha, qfrac16_t beta)
{
    /*
        The other 3 of 6 are inverse of the 3 derived
        Magnitudes are normalized by a factor of sqrt(3)/2, i.e alpha = 1 => A = .866
        Derives 3 magnitudes (duty cycles) belonging to basic unit vectors.

        X = beta;
        Y = (beta + sqrt3 * alpha) / 2;
        Z = (beta - sqrt3 * alpha) / 2;
    */

    int32_t betaDiv2 = qfrac16_mul(beta, QFRAC16_1_DIV_2);
    int32_t alphaSqrt3Div2 = qfrac16_mul(alpha, QFRAC16_SQRT3_DIV_2);

    int32_t magX = beta;
    int32_t magY = betaDiv2 + alphaSqrt3Div2;
    int32_t magZ = betaDiv2 - alphaSqrt3Div2;

    int32_t z0;

    if(magX >= 0)
    {
        if(magZ < 0)
        {
            /*
                Sector 1: X >= 0 and Z < 0

                Duty Cycle:
                A:       |v100| = (sqrt3 * alpha - beta) / 2 = -Z;
                invC:    |v110| = beta = X;

                SVPWM:
                A -> Max, B -> Mid, C -> Min
                z0 = (1/2) - (max + min)/2 = (1 - (-Z - X))/2
                A = z0 - Z;
                B = z0;
                C = z0 - X;

                A = (1 + X - Z) / 2;
                B = (1 + X + Z) / 2;
                C = (1 - X + Z) / 2;
            */
            z0 = (QFRAC16_MAX + magX + magZ) / 2;
            *p_dutyA = (z0 - magZ);
            *p_dutyB = z0;
            *p_dutyC = (z0 - magX);
        }
        else if(magY >= 0)
        {
            /*
                Sector 2: Y >= 0 and Z >= 0


                Duty Cycle:
                invC:     |v110| = 1 * (beta + sqrt3 * alpha) / 2 = Y;
                B:         |v010| = 1 * (beta - sqrt3 * alpha) / 2 = Z;

                SVPWM:
                A -> Mid, B -> Max, C -> Min
                z0 = (1/2) - (max + min)/2 = (1 - (Z - Y))/2
                A = z0;
                B = z0 + Z;
                C = z0 - Y;
            */
            z0 = (QFRAC16_MAX + magY - magZ) / 2;
            *p_dutyA = z0;
            *p_dutyB = (z0 + magZ);
            *p_dutyC = (z0 - magY);
        }
        else
        {
            /*
                Sector 3: X >= 0 and Y < 0

                Duty Cycle:
                B:         |v010| = X;
                invA:    |v011| = -Y;

                SVPWM:
                A -> Min, B -> Max, C -> Mid
                z0 = (1 - (X + Y))/2
                A = z0 + Y;
                B = z0 + X;
                C = z0;
            */
            z0 = (QFRAC16_MAX - magX - magY) / 2;
            *p_dutyA = (z0 + magY);
            *p_dutyB = (z0 + magX);
            *p_dutyC = z0;
        }
    }
    else
    {
        if(magZ >= 0)
        {
            /*
                Sector 4: X < 0 and Z >= 0

                Duty Cycle:
                invA:     |v011| = Z;
                C:        |v001| = -X;

                SVPWM:
                A -> Min, B -> Mid, C -> Max
                z0 = (1 - (-X - Z))/2
                A = z0 - Z;
                B = z0;
                C = z0 - X;
            */
            z0 = (QFRAC16_MAX + magX + magZ) / 2;
            *p_dutyA = (z0 - magZ);
            *p_dutyB = z0;
            *p_dutyC = (z0 - magX);
        }
        else if(magY < 0)
        {
            /*
                Sector 5:  Y < 0 and Z < 0

                Duty Cycle:
                C:         |v001| = -Y;
                invB:    |v101| = -Z;

                SVPWM:
                A -> Mid, B -> Min, C -> Max
                z0 = (1 - (-Y + Z))/2
                A = z0;
                B = z0 + Z;
                C = z0 - Y;
            */
            z0 = (QFRAC16_MAX + magY - magZ) / 2;
            *p_dutyA = z0;
            *p_dutyB = (z0 + magZ);
            *p_dutyC = (z0 - magY);
        }
        else
        {
            /*
                Sector 6: X < 0 and Y >= 0

                Duty Cycle:
                invB:     |v101| = -X;
                A:        |v100| = Y;

                SVPWM:
                A -> Max, B -> Min, C -> Mid
                z0 = (1 - (X + Y))/2
                A = z0 + Y;
                B = z0 + X;
                C = z0;
            */
            z0 = (QFRAC16_MAX - magX - magY) / 2;
            *p_dutyA = (z0 + magY);
            *p_dutyB = (z0 + magX);
            *p_dutyC = z0;
        }
    }

    *p_dutyA = qfrac16_sat_abs(*p_dutyA);
    *p_dutyB = qfrac16_sat_abs(*p_dutyB);
    *p_dutyC = qfrac16_sat_abs(*p_dutyC);

    // assert((int16_t)*p_dutyA > 0);
    // assert((int16_t)*p_dutyB > 0);
    // assert((int16_t)*p_dutyC > 0);
}

#endif
