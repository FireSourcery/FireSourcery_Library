#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2026 FireSourcery

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
    @file   _svpwm_sector.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Math/Fixed/fract16.h"

/*!
    SVM by sector, the null vectors shared equally: midclamp, the duty cycles of [svpwm_midclamp] of the same phase voltages.
    Determines the sector first, then the duty cycles of its 2 basic vectors. Inclusive of the equivalent inverse Clarke transform.
    SVPWM determined by shifting magnitudes such that the midpoint is 50% PWM.

    @param[in] alpha, beta - duty scaling, 1.0F <=> VBus/sqrt3: the linear limit, the hexagon's inscribed circle, at sqrt(alpha^2 + beta^2) = 1.
                            From a phase voltage normalized to VBus as 1.0F [svpwm_norm_vbus_inv]: sqrt3 * v
    @return [0:32767]
*/
static inline struct svpwm_abc svpwm_midclamp_sector(fract16_t alpha, fract16_t beta)
{
    /*
        Derive 3 magnitudes (duty cycles) belonging to basic unit vectors.
        The other 3 of 6 are inverse of the 3 derived.
        In duty scaling they are also the line-to-line duties:
            X = B - C,
            Y = A - C,
            Z = B - A.

        X = beta;
        Y = (beta + sqrt3 * alpha) / 2;
        Z = (beta - sqrt3 * alpha) / 2;
    */
    int32_t betaDiv2 = fract16_mul(beta, FRACT16_1_DIV_2);
    int32_t alphaSqrt3Div2 = fract16_mul(alpha, FRACT16_SQRT3_DIV_2);

    int32_t magX = beta;
    int32_t magY = betaDiv2 + alphaSqrt3Div2;
    int32_t magZ = betaDiv2 - alphaSqrt3Div2;

    int32_t z0; /* The middle duty. Max and min, relative to it, center on 1/2: z0 = (1/2) - (max + min)/2 = (1 - (max + min))/2 */

    int32_t dutyA;
    int32_t dutyB;
    int32_t dutyC;

    uint8_t sector;

    if (magX >= 0)
    {
        if      (magZ < 0)  { sector = 1; }
        else if (magY >= 0) { sector = 2; }
        else                { sector = 3; }
    }
    else
    {
        if      (magZ >= 0) { sector = 4; }
        else if (magY < 0)  { sector = 5; }
        else                { sector = 6; }
    }

    switch (sector)
    {
        case 1:
            /*
                Sector 1: X >= 0 and Z < 0
                Duty Cycle:
                A:       |v100| = (sqrt3 * alpha - beta) / 2 = -Z;
                invC:    |v110| = beta = X;

                SVPWM:
                A -> Max, B -> Mid, C -> Min
                z0 = (1 - (-Z - X))/2
                A = z0 - Z;
                B = z0;
                C = z0 - X;

                A = (1 + X - Z) / 2;
                B = (1 + X + Z) / 2;
                C = (1 - X + Z) / 2;
            */
            z0 = FRACT16_1_DIV_2 + (magX + magZ) / 2;
            dutyA = (z0 - magZ);
            dutyB = z0;
            dutyC = (z0 - magX);
            break;

        case 2:
            /*
                Sector 2: Y >= 0 and Z >= 0
                Duty Cycle:
                invC:     |v110| = (beta + sqrt3 * alpha) / 2 = Y;
                B:        |v010| = (beta - sqrt3 * alpha) / 2 = Z;

                SVPWM:
                A -> Mid, B -> Max, C -> Min
                z0 = (1 - (Z - Y))/2
                A = z0;
                B = z0 + Z;
                C = z0 - Y;
            */
            z0 = FRACT16_1_DIV_2 + (magY - magZ) / 2;
            dutyA = z0;
            dutyB = (z0 + magZ);
            dutyC = (z0 - magY);
            break;

        case 3:
            /*
                Sector 3: X >= 0 and Y < 0

                Duty Cycle:
                B:       |v010| = X;
                invA:    |v011| = -Y;

                SVPWM:
                A -> Min, B -> Max, C -> Mid
                z0 = (1 - (X + Y))/2
                A = z0 + Y;
                B = z0 + X;
                C = z0;
            */
            z0 = FRACT16_1_DIV_2 - (magX + magY) / 2;
            dutyA = (z0 + magY);
            dutyB = (z0 + magX);
            dutyC = z0;
            break;

        case 4:
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
            z0 = FRACT16_1_DIV_2 + (magX + magZ) / 2;
            dutyA = (z0 - magZ);
            dutyB = z0;
            dutyC = (z0 - magX);
            break;

        case 5:
            /*
                Sector 5:  Y < 0 and Z < 0

                Duty Cycle:
                C:       |v001| = -Y;
                invB:    |v101| = -Z;

                SVPWM:
                A -> Mid, B -> Min, C -> Max
                z0 = (1 - (-Y + Z))/2
                A = z0;
                B = z0 + Z;
                C = z0 - Y;
            */
            z0 = FRACT16_1_DIV_2 + (magY - magZ) / 2;
            dutyA = z0;
            dutyB = (z0 + magZ);
            dutyC = (z0 - magY);
            break;

        case 6:
        default: /* The tree assigns 1 to 6 */
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
            z0 = FRACT16_1_DIV_2 - (magX + magY) / 2;
            dutyA = (z0 + magY);
            dutyB = (z0 + magX);
            dutyC = z0;
            break;
    }

    return (struct svpwm_abc) { .a = fract16_sat_positive(dutyA), .b = fract16_sat_positive(dutyB), .c = fract16_sat_positive(dutyC) };
}

