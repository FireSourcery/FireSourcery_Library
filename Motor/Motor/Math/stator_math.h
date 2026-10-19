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
    @file   stator_math.h
    @author FireSourcery
    @brief  Stator voltage equation of one axis across one control period.
*/
/******************************************************************************/
#include "Math/Fixed/fract16.h"


/******************************************************************************/
/*!
    @brief  Voltage-Model Back-EMF Estimator

        v = R·i + L·di/dt + e
        e = ω·ψ = v - Rs·i - Ls·di/dt

    Discrete one-step form, caller supplies the precomputed Δi = i[k] - i[k-1]:
        ê[k] = v[k] - Rs_pu·i[k] - Ls_pu_rad·Δi[k]

    L at the rad base, Fs / ω_base = 1. A turn-base L carries 1 / π: L_rad = L_turn / π.

    Stationary axes don't couple: the equation holds per axis in abc, and in αβ with L = Lq, where ē is the extended EMF.
    Solved for v, the drive. Solved for e, the voltage model. Solved for i, the current model.
*/
/******************************************************************************/
/* The drop across the winding: R·ī across the resistance, L·Δi across the inductance */
static inline accum32_t stator_v_drop(accum32_t r, accum32_t l, fract16_t i_prev, fract16_t i) { return accum32_mul(r, i) + accum32_mul(l, i - i_prev); }

/* The v held across the period that moves i_prev to i against e. Across the next period, the deadbeat command */
static inline accum32_t stator_v(accum32_t r, accum32_t l, fract16_t i_prev, fract16_t i, accum32_t e) { return stator_v_drop(r, l, i_prev, i) + e; }

/*
    Voltage model. ē is the period average.
    R on the end sample instead offsets ē by R·Δi / 2, in proportion to speed.
*/
static inline accum32_t stator_emf(accum32_t r, accum32_t l, fract16_t i_prev, fract16_t i, accum32_t v) { return v - stator_v_drop(r, l, i_prev, i); }


/******************************************************************************/
/*!
    Current model. Forward Euler from i at the start of the period, e given:
        i[k+1] = i[k] + G·(v − R·i[k] − e),     G = 1 / L_rad = Ts · V_base / (L · I_base)
*/
/******************************************************************************/
static inline accum32_t stator_i_next(accum32_t r, accum32_t g, fract16_t i, accum32_t v, accum32_t e) { return (accum32_t)i + accum32_mul(g, v - fract16_mul(r, i) - e); }


/******************************************************************************/
/*!
    Power across the period, a voltage times ī. Each term of the equation splits the terminal power:
        v·ī = R·ī² + L·Δi·ī + ē·ī       winding loss, stored-energy rate L·Δ(i²)/2, air gap
    v·i units as [foc_active_power], the 3/2 belongs to the quantity.
*/
/******************************************************************************/
static inline accum32_t stator_power(accum32_t v, fract16_t i_prev, fract16_t i) { return fract16_mul(v, (i_prev + i) / 2); }

/******************************************************************************/
/*!
    Rotating frame. The reactance X = ω·L is the arc of the frame's turn per period at radius L:
    ω·Ts = Δθ in radians, Δθ · π / 2^15, and L at the rad base carries the Fs. One truncation.
*/
/******************************************************************************/
static inline accum32_t stator_reactance(accum32_t l, angle16_t delta) { return ((int64_t)l * delta * FRACT16_PI) >> (2 * FRACT16_N_BITS); }
