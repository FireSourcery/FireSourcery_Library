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

    @note overflow
    vPhase_pu < vBus_pu * .57
    vBusInv_pu * vBus_pu = INT32_MAX
    overflow when VBus drops more than 42% before v_pu updates
    VBus fault detection should trigger before this point
*/
static inline fract16_t svpwm_norm_vbus_inv(uint32_t vBusInv_pu, fract16_t vphase_pu) { return (int32_t)vphase_pu * (int32_t)vBusInv_pu / FRACT16_SCALE; }
static inline fract16_t svpwm_norm_vbus(ufract16_t vBus_pu, fract16_t vphase_pu) { return fract16_div(vphase_pu, vBus_pu); }
/*  */
static inline fract16_t svpwm_vphase_vbus(ufract16_t vBus_pu, fract16_t vNorm) { return fract16_mul(vNorm, vBus_pu); }


struct svpwm_abc { ufract16_t a, b, c; };

/*!
    @param[in] vA, vB, vC - normalized to VBus as 1.0F, [-1/sqrt3:1/sqrt3]
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

// static inline struct svpwm_abc svpwm_midclamp_vbus(uint32_t vBusInv_pu, fract16_t vA, fract16_t vB, fract16_t vC)
// {
//     return svpwm_midclamp(svpwm_norm_vbus_inv(vBusInv_pu, vA), svpwm_norm_vbus_inv(vBusInv_pu, vB), svpwm_norm_vbus_inv(vBusInv_pu, vC));
// }

/*!
    General clamp. The zero sequence vZero = k·(vMax − 1) + (1 − k)·vMin places k of the null time at O111, all phases high,
    and 1 − k at O000, all phases low. The line-to-line voltages are the same for any k.
        lowest duty     k·(1 − spread)
        highest duty    spread + k·(1 − spread), the shortest low-side window (1 − k)·(1 − spread)
    k = 0, the bottom clamp, vZero = vMin. k = 1/2, midclamp [svpwm_midclamp], to rounding. k = 1, the top clamp, vZero = vMax − 1.

    @param[in] vA, vB, vC - normalized to VBus as 1.0F, [-1/sqrt3:1/sqrt3]
    @param[in] k - [0:32768] <=> [0:1]
    @return [0:32767]
*/
static inline struct svpwm_abc svpwm_clamp(fract16_t vA, fract16_t vB, fract16_t vC, ufract16_t k)
{
    int32_t vMax = math_max(math_max(vA, vB), vC);
    int32_t vMin = math_min(math_min(vA, vB), vC);
    int32_t vZero = fract16_mul(k, vMax - FRACT16_SCALE) + fract16_mul(FRACT16_SCALE - k, vMin);

    return (struct svpwm_abc) { .a = fract16_sat_positive(vA - vZero), .b = fract16_sat_positive(vB - vZero), .c = fract16_sat_positive(vC - vZero) };
}

/*!
    [svpwm_clamp] with k solved per vector for the highest duty at dutyCeiling, k = (dutyCeiling − spread) / (1 − spread),
    held to [0, 1/2]: midclamp until the highest duty reaches the ceiling, then toward the bottom clamp.
    vZero is linear in k, so the clamp holds vZero between the two, without the division.
    A spread past the ceiling takes the bottom clamp, the closest it can come.
        √3/2:           midclamp below 0.42·VBus, the bottom clamp at the linear limit VBus/√3
        FRACT16_MAX:    midclamp, inside the linear range

    @param[in] vA, vB, vC - normalized to VBus as 1.0F, [-1/sqrt3:1/sqrt3]
    @param[in] dutyCeiling - [0:32767]
    @return [0:32767]
*/
static inline struct svpwm_abc svpwm_midclamp_ceiling(fract16_t vA, fract16_t vB, fract16_t vC, ufract16_t dutyCeiling)
{
    int32_t vMax = math_max(math_max(vA, vB), vC);
    int32_t vMin = math_min(math_min(vA, vB), vC);
    int32_t vZero = math_clamp(vMax - dutyCeiling, (vMax + vMin) / 2 - FRACT16_1_DIV_2, vMin);

    return (struct svpwm_abc) { .a = fract16_sat_positive(vA - vZero), .b = fract16_sat_positive(vB - vZero), .c = fract16_sat_positive(vC - vZero) };
}

/******************************************************************************/
/*!
    Modulation limit of a duty limit: the radius |v|/VBus at which the highest duty over a cycle reaches a peak.
    A current read in the low-side window keeps (1 − dutyPeak)·T of the period there.

    The macros are the constant-expression forms, for configuration: FOC_V_CIRCLE_PER_V_BUS, initializers, static_assert.
    The functions take and return the same fract16 values.
    FRACT16_SQRT3_DIV_2 and FRACT16_1_DIV_SQRT3 are objects, not constant expressions, so the macros round their own to the same values.
*/
/******************************************************************************/
#define _SVPWM_FRACT16_OF(x)    ((int32_t)((x) * FRACT16_SCALE + 0.5))
#define _SVPWM_SQRT3_DIV_2      _SVPWM_FRACT16_OF(0.86602540378443865)
#define _SVPWM_1_DIV_SQRT3      _SVPWM_FRACT16_OF(0.57735026918962576)

#define SVPWM_MIDCLAMP_DUTY_PEAK(Radius)                        (FRACT16_SCALE / 2 + (((int32_t)(Radius) * _SVPWM_SQRT3_DIV_2) >> 15))
#define SVPWM_MIDCLAMP_RADIUS(DutyPeak)                         ((((int32_t)(DutyPeak) * 2 - FRACT16_SCALE) * _SVPWM_1_DIV_SQRT3) >> 15)
#define SVPWM_MIDCLAMP_CEILING_RADIUS(DutyPeak, DutyCeiling)    (((DutyPeak) < (DutyCeiling)) ? SVPWM_MIDCLAMP_RADIUS(DutyPeak) : (((int32_t)(DutyPeak) * _SVPWM_1_DIV_SQRT3) >> 15))

/*
    The highest duty that leaves the low-side window Window in each Period: 1 − Window/Period, rounded down to keep the window.
    Window and Period in one unit, ns or timer ticks: 3350 ns of 50000 ns (20 kHz), the window VBus/2 leaves under midclamp, is 93.3 %.
*/
#define SVPWM_DUTY_PEAK_OF_WINDOW(Window, Period)                               ((int32_t)((int64_t)FRACT16_SCALE * ((Period) - (Window)) / (Period)))
#define SVPWM_MIDCLAMP_RADIUS_OF_WINDOW(Window, Period)                         SVPWM_MIDCLAMP_RADIUS(SVPWM_DUTY_PEAK_OF_WINDOW(Window, Period))
#define SVPWM_MIDCLAMP_CEILING_RADIUS_OF_WINDOW(Window, Period, DutyCeiling)    SVPWM_MIDCLAMP_CEILING_RADIUS(SVPWM_DUTY_PEAK_OF_WINDOW(Window, Period), DutyCeiling)

/*
    The window forms above as one expression, in floating point, for precompile values. With w = Window/Period,
    the highest duty over a cycle under [svpwm_clamp] is k + (1 − k)·√3·r, so the largest radius that leaves the window is
        r = (1 − w/(1 − k)) / √3
    k = 1/2, midclamp: (1 − 2w)/√3.
    k = 0, the bottom clamp: (1 − w)/√3. Also [svpwm_midclamp_ceiling]'s, for a ceiling at or under 1 − w; over it, midclamp's.
    Window and Period in one unit, k in [0:1). FRACT16 truncates, keeping the window.
*/
#define SVPWM_CLAMP_RADIUS_OF_WINDOW(Window, Period, K) FRACT16((1.0F - (float)(Window) / (float)(Period) / (1.0F - (float)(K))) / 1.73205080756887729F)

/*!
    The highest duty [svpwm_midclamp] writes over a cycle at radius r, at each line-to-line peak: 1/2 + (√3/2)·r.

    @param[in] radius - |v| normalized to VBus as 1.0F, [0:1/sqrt3]
    @return [1/2:1]
*/
static inline ufract16_t svpwm_midclamp_duty_peak(ufract16_t radius) { return SVPWM_MIDCLAMP_DUTY_PEAK(radius); }

/*!
    The largest radius at which midclamp's highest duty stays at or under dutyPeak, the inverse of [svpwm_midclamp_duty_peak]: (2·dutyPeak − 1) / √3.

    @param[in] dutyPeak - [1/2:1]
    @return [0:1/sqrt3]
*/
static inline ufract16_t svpwm_midclamp_radius(ufract16_t dutyPeak) { return SVPWM_MIDCLAMP_RADIUS(dutyPeak); }

/*!
    The largest radius at which the highest duty [svpwm_midclamp_ceiling] writes over a cycle stays at or under dutyPeak.
    That duty is midclamp's [svpwm_midclamp_duty_peak], held to the ceiling, then the bottom clamp's spread √3·r once it passes the ceiling:
        dutyPeak at or over the ceiling     dutyPeak / √3
        under                               [svpwm_midclamp_radius]
    Midclamp's at VBus/2, 93.3 %, keeps the same low-side window under the ceiling √3/2 to 0.539·VBus.

    @param[in] dutyPeak - [1/2:1]
    @param[in] dutyCeiling - [0:32767]
    @return [0:1/sqrt3]
*/
static inline ufract16_t svpwm_midclamp_ceiling_radius(ufract16_t dutyPeak, ufract16_t dutyCeiling) { return SVPWM_MIDCLAMP_CEILING_RADIUS(dutyPeak, dutyCeiling); }




/*
    Transform V with midclamp 3rd harmonic adjustment
    without shifting to half Duty
*/
static inline struct svpwm_abc svpwm_midclamp_transform(fract16_t vA, fract16_t vB, fract16_t vC)
{
    // Find the maximum and minimum of the three phase voltages
    int32_t vMax = math_max(math_max(vA, vB), vC);
    int32_t vMin = math_min(math_min(vA, vB), vC);

    // Calculate the zero - sequence voltage(midclamp adjustment)
    int32_t vZero = (vMax + vMin) / 2;

    return (struct svpwm_abc) { .a = fract16_sat_positive(vA - vZero), .b = fract16_sat_positive(vB - vZero), .c = fract16_sat_positive(vC - vZero) };
}

