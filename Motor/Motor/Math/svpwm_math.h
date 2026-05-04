#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery

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
*/
/******************************************************************************/
#include "Math/Fixed/fract16.h"


/*!
    VBusNorm
    Normalize V to VBus as 1.0F
    [VBus/sqrt3] => [1/sqrt3]
    Svpwm input as VPhase/VBus
    @return [-1/sqrt3:1/sqrt3]
*/
/*!
    @note overflow vPhase_fract16 < vBus_fract16, assuming consistent VBus
    vBusInv_fract32 * vBus_fract16 = INT32_MAX
*/
/* phase_norm_of_v */
static inline fract16_t svpwm_norm_vbus_inv(uint32_t vBusInv_fract32, fract16_t v_fract16) { return (int32_t)v_fract16 * vBusInv_fract32 / 65536; }
static inline fract16_t svpwm_norm_vbus(ufract16_t vBus_fract16, fract16_t v_fract16) { return fract16_div(v_fract16, vBus_fract16); }
/*  */
static inline fract16_t svpwm_vphase_vbus(ufract16_t vBus_fract16, fract16_t vNorm) { return fract16_mul(vNorm, vBus_fract16); }


struct svpwm_abc { ufract16_t a, b, c; };

/*!
    @param[in] vA, vB, vC - scalars normalized to VBus as 1.0F. range [-1/sqrt3:1/sqrt3].
                            vA = 1/sqrt3 <=> Phase A voltage output VBus/sqrt3
    @param[out] p_dutyA, p_dutyB, p_dutyC - [0:32767]
*/
static inline struct svpwm_abc svpwm_midclamp(fract16_t vA, fract16_t vB, fract16_t vC)
{
    // Find the maximum and minimum of the three phase voltages
    int32_t vMax = math_max(math_max(vA, vB), vC);
    int32_t vMin = math_min(math_min(vA, vB), vC);

    // Calculate the zero - sequence voltage (midclamp adjustment)
    int32_t vZero = (vMax + vMin) / 2 - FRACT16_1_DIV_2;

    // Adjust the phase voltages to ensure midclamp
    int32_t dutyA = vA - vZero;
    int32_t dutyB = vB - vZero;
    int32_t dutyC = vC - vZero;

    // Saturate the duty cycles to ensure they are within the range of 0 to 1
    return (struct svpwm_abc) { .a = fract16_sat_positive(dutyA), .b = fract16_sat_positive(dutyB), .c = fract16_sat_positive(dutyC) };
}


static inline struct svpwm_abc svpwm_midclamp_vbus(uint32_t vBusInv_fract32, fract16_t vA, fract16_t vB, fract16_t vC)
{
    return svpwm_midclamp(svpwm_norm_vbus_inv(vBusInv_fract32, vA), svpwm_norm_vbus_inv(vBusInv_fract32, vB), svpwm_norm_vbus_inv(vBusInv_fract32, vC));
}


/*
    Transform V with midclamp 3rd harmonic adjustment
    without shifting to half Duty
*/
static inline struct svpwm_abc svpwm_midclamp_transform(fract16_t vA, fract16_t vB, fract16_t vC)
{
    // # Find the maximum and minimum of the three phase voltages
    int32_t vMax = math_max(math_max(vA, vB), vC);
    int32_t vMin = math_min(math_min(vA, vB), vC);

    // # Calculate the zero - sequence voltage(midclamp adjustment)
    int32_t vZero = (vMax + vMin) / 2;

    return (struct svpwm_abc) { .a = fract16_sat_positive(vA - vZero), .b = fract16_sat_positive(vB - vZero), .c = fract16_sat_positive(vC - vZero) };
}


/*!
    SVM calculation with 3rd harmonic. Inclusive of equivalent reverse Clarke transform.
    Mid clamp, determining sector first. SVPWM determined by shifting magnitudes such that the midpoint is 50% PWM

    @param[in] alpha scaled to VBus
*/
// static inline void svpwm_midclamp(fract16_t * p_dutyA, fract16_t * p_dutyB, fract16_t * p_dutyC, fract16_t alpha, fract16_t beta)
// {
//     /*
//         Derive 3 magnitudes (duty cycles) belonging to basic unit vectors.
//         The other 3 of 6 are inverse of the 3 derived

//         X = beta;
//         Y = (beta + sqrt3 * alpha) / 2;
//         Z = (beta - sqrt3 * alpha) / 2;
//     */
//     int32_t betaDiv2 = fract16_mul(betaDuty, FRACT16_1_DIV_2);
//     int32_t alphaSqrt3Div2 = fract16_mul(alphaDuty, FRACT16_SQRT3_DIV_2);

//     int32_t magX = betaDuty;
//     int32_t magY = betaDiv2 + alphaSqrt3Div2;
//     int32_t magZ = betaDiv2 - alphaSqrt3Div2;

//     int32_t z0; /* z0 = (1/2) - (max + min)/2 = (1 - (max + min))/2 */

//     int32_t dutyA;
//     int32_t dutyB;
//     int32_t dutyC;

//     uint8_t sector;

//     if (magX >= 0)
//     {
//         if      (magZ < 0)  { sector = 1; }
//         else if (magY >= 0) { sector = 2; }
//         else                { sector = 3; }
//     }
//     else
//     {
//         if      (magZ >= 0) { sector = 4; }
//         else if (magY < 0)  { sector = 5; }
//         else                { sector = 6; }
//     }

//     switch (sector)
//     {
//         case 1:
//             /*
//                 Sector 1: X >= 0 and Z < 0
//                 Duty Cycle:
//                 A:       |v100| = (sqrt3 * alpha - beta) / 2 = -Z;
//                 invC:    |v110| = beta = X;

//                 SVPWM:
//                 A -> Max, B -> Mid, C -> Min
//                 z0 = (1 - (-Z - X))/2
//                 A = z0 - Z;
//                 B = z0;
//                 C = z0 - X;

//                 A = (1 + X - Z) / 2;
//                 B = (1 + X + Z) / 2;
//                 C = (1 - X + Z) / 2;
//             */
//             z0 = (FRACT16_MAX + magX + magZ) / 2;
//             dutyA = (z0 - magZ);
//             dutyB = z0;
//             dutyC = (z0 - magX);
//             break;

//         case 2:
//             /*
//                 Sector 2: Y >= 0 and Z >= 0
//                 Duty Cycle:
//                 invC:     |v110| = 1 * (beta + sqrt3 * alpha) / 2 = Y;
//                 B:        |v010| = 1 * (beta - sqrt3 * alpha) / 2 = Z;

//                 SVPWM:
//                 A -> Mid, B -> Max, C -> Min
//                 z0 = (1 - (Z - Y))/2
//                 A = z0;
//                 B = z0 + Z;
//                 C = z0 - Y;
//             */
//             z0 = (FRACT16_MAX + magY - magZ) / 2;
//             dutyA = z0;
//             dutyB = (z0 + magZ);
//             dutyC = (z0 - magY);
//             break;

//         case 3:
//             /*
//                 Sector 3: X >= 0 and Y < 0

//                 Duty Cycle:
//                 B:       |v010| = X;
//                 invA:    |v011| = -Y;

//                 SVPWM:
//                 A -> Min, B -> Max, C -> Mid
//                 z0 = (1 - (X + Y))/2
//                 A = z0 + Y;
//                 B = z0 + X;
//                 C = z0;
//             */
//             z0 = (FRACT16_MAX - magX - magY) / 2;
//             dutyA = (z0 + magY);
//             dutyB = (z0 + magX);
//             dutyC = z0;
//             break;

//         case 4:
//             /*
//                 Sector 4: X < 0 and Z >= 0

//                 Duty Cycle:
//                 invA:     |v011| = Z;
//                 C:        |v001| = -X;

//                 SVPWM:
//                 A -> Min, B -> Mid, C -> Max
//                 z0 = (1 - (-X - Z))/2
//                 A = z0 - Z;
//                 B = z0;
//                 C = z0 - X;
//             */
//             z0 = (FRACT16_MAX + magX + magZ) / 2;
//             dutyA = (z0 - magZ);
//             dutyB = z0;
//             dutyC = (z0 - magX);
//             break;

//         case 5:
//             /*
//                 Sector 5:  Y < 0 and Z < 0

//                 Duty Cycle:
//                 C:       |v001| = -Y;
//                 invB:    |v101| = -Z;

//                 SVPWM:
//                 A -> Mid, B -> Min, C -> Max
//                 z0 = (1 - (-Y + Z))/2
//                 A = z0;
//                 B = z0 + Z;
//                 C = z0 - Y;
//             */
//             z0 = (FRACT16_MAX + magY - magZ) / 2;
//             dutyA = z0;
//             dutyB = (z0 + magZ);
//             dutyC = (z0 - magY);
//             break;

//         case 6:
//             /*
//                 Sector 6: X < 0 and Y >= 0

//                 Duty Cycle:
//                 invB:     |v101| = -X;
//                 A:        |v100| = Y;

//                 SVPWM:
//                 A -> Max, B -> Min, C -> Mid
//                 z0 = (1 - (X + Y))/2
//                 A = z0 + Y;
//                 B = z0 + X;
//                 C = z0;
//             */
//             z0 = (FRACT16_MAX - magX - magY) / 2;
//             dutyA = (z0 + magY);
//             dutyB = (z0 + magX);
//             dutyC = z0;
//             break;

//         default:
//             break;
//     }

//     *p_dutyA = fract16_sat_positive(dutyA);
//     *p_dutyB = fract16_sat_positive(dutyB);
//     *p_dutyC = fract16_sat_positive(dutyC);
// }

