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
    @file   Stator.h
    @author FireSourcery
    @brief  Per-axis image of the stator.
*/
/******************************************************************************/
#include "Motor/Motor/Math/stator_math.h"
#include "Motor/Motor/Math/foc_math.h"
#include "Math/Fixed/fract16.h"


/******************************************************************************/
/*
    One axis across one control period: i at both ends, v held across it.
    Stationary axes don't couple, so the αβ and abc images are this axis two and three times.
*/
/******************************************************************************/
typedef struct Stator_Axis
{
    fract16_t I;        /* sampled this cycle, the end of the period */
    fract16_t IPrev;    /* sampled last cycle, the start */
    fract16_t V;        /* applied across the period */
    // fract16_t VOut;     /* commanded this cycle, applied across the next */
    // fract16_t IEst;
}
Stator_Axis_T;

/* The sample closes the period: the VOut commanded at its start is the V held across it */
// static inline void Stator_Axis_CaptureI(Stator_Axis_T * p_axis, fract16_t i) { p_axis->IPrev = p_axis->I; p_axis->I = i; p_axis->V = p_axis->VOut; }
// static inline void Stator_Axis_CaptureVOut(Stator_Axis_T * p_axis, fract16_t v) { p_axis->VOut = v; }


static inline fract16_t Stator_Axis_IAvg(const Stator_Axis_T * p_axis) { return ((int32_t)p_axis->IPrev + p_axis->I) / 2; }
static inline fract16_t Stator_Axis_IDelta(const Stator_Axis_T * p_axis) { return p_axis->I - p_axis->IPrev; }


/*
    Per phase. L = Lq resolves the extended EMF E = ω·(ψ + (Ld − Lq)·i_d) of a salient rotor. Ls = (Ld + Lq) / 2 leaves a 2θ error.
*/
typedef struct Stator_Winding
{
    accum32_t R;    /* phase resistance */
    accum32_t L;    /* phase inductance at the rad base */
    accum32_t G;    /* 1 / L at the rad base */
    // union { accum32_t L; accum32_t G; }; /* determined by axis type */
}
Stator_Winding_T;

/* From the model at the turn base. L = L_turn / π at the rad base, G = 1 / L, the current model's step [g_l_pu_turn] */
static inline void Stator_Winding_Init(Stator_Winding_T * p_winding, accum32_t r, uint32_t l_turn)
{
    *p_winding = (Stator_Winding_T) { .R = r, .L = accum32_mul(l_turn, FRACT16_1_DIV_PI), .G = g_l_pu_turn(l_turn) };
}

/*
    The period through the winding, V = R·I + L·ΔI + ē
*/
static inline accum32_t Stator_Axis_VDrop(const Stator_Winding_T * p_winding, const Stator_Axis_T * p_axis) { return stator_v_drop(p_winding->R, p_winding->L, p_axis->IPrev, p_axis->I); }

/* ē over the period, [stator_emf] */
static inline accum32_t Stator_Axis_Emf(const Stator_Winding_T * p_winding, const Stator_Axis_T * p_axis) { return stator_emf(p_winding->R, p_winding->L, p_axis->IPrev, p_axis->I, p_axis->V); }

/* The next period: the VOut that moves I to i against e, [stator_v] */
static inline accum32_t Stator_Axis_VOutOf(const Stator_Winding_T * p_winding, const Stator_Axis_T * p_axis, fract16_t i, accum32_t e) { return stator_v(p_winding->R, p_winding->L, p_axis->I, i, e); }

/* Power over the period, [stator_power]: at the terminals, and through ē, the air gap */
static inline accum32_t Stator_Axis_Power(const Stator_Axis_T * p_axis) { return stator_power(p_axis->V, p_axis->IPrev, p_axis->I); }
static inline accum32_t Stator_Axis_PowerEmf(const Stator_Winding_T * p_winding, const Stator_Axis_T * p_axis) { return stator_power(Stator_Axis_Emf(p_winding, p_axis), p_axis->IPrev, p_axis->I); }

static inline accum32_t Stator_Axis_I(const Stator_Winding_T * p_winding, const Stator_Axis_T * p_axis) { return stator_i_next(p_winding->R, p_winding->G, p_axis->I, p_axis->V, Stator_Axis_Emf(p_winding, p_axis)); }

// static inline Stator_Axis_T Stator_Axis_Step(Stator_Axis_T * p_axis, fract16_t i, fract16_t v) {  return (Stator_Axis_T){ .IPrev = p_axis->I, .I = i, .V = p_axis->VOut, .VOut = v }; }
static inline fract16_t Stator_Winding_INext(const Stator_Winding_T * p_winding, fract16_t i, accum32_t v, accum32_t e) { return fract16_sat(stator_i_next(p_winding->R, p_winding->G, i, v, e)); }


/*
    The sample closes the period, with the v held across it: the last command, when captured before the next is made.
    The record then holds the whole period between calls.
*/
static inline void Stator_Axis_Capture(Stator_Axis_T * p_axis, fract16_t i, fract16_t v)
{
    p_axis->IPrev = p_axis->I;
    p_axis->I = i;
    p_axis->V = v;
}



/*
    The stator equation closes the period in place of a sample: I from v held across it, e in place of the EMF [stator_i_next].
    A model's image, run beside the sampled one.
*/
static inline fract16_t Stator_Axis_Integrate(const Stator_Winding_T * p_winding, Stator_Axis_T * p_axis, accum32_t v, accum32_t e)
{
    p_axis->IPrev = p_axis->I;
    p_axis->I = fract16_sat(stator_i_next(p_winding->R, p_winding->G, p_axis->I, v, e));
    p_axis->V = fract16_sat(v);
    return p_axis->I;
}

/* The equation forward across the period: V held, e in place of the EMF [stator_i_next] */
// static inline void Stator_Axis_Integrate(const Stator_Winding_T * p_winding, Stator_Axis_T * p_axis, accum32_t e)
// {
//     p_axis->I = fract16_sat(stator_i_next(p_winding->R, p_winding->G, p_axis->I, p_axis->V, e));
// }
// static inline void Stator_Axis_Integrate(const Stator_Winding_T * p_winding, Stator_Axis_T * p_axis, accum32_t v, accum32_t e)
// {
//     p_axis->I = fract16_sat(stator_i_next(p_winding->R, p_winding->G, p_axis->I, v, e));
//     p_axis->V = fract16_sat(v);
// }

// typedef struct Stator_Orthogonal
// {
//     Stator_Winding_T Winding;
//     Stator_Axis_T X;
//     Stator_Axis_T Y;
// }
// Stator_Orthogonal_T;


