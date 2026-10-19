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
    fract16_t VOut;     /* commanded this cycle, applied across the next */
}
Stator_Axis_T;

/* The sample closes the period: the VOut commanded at its start is the V held across it */
static inline void Stator_Axis_CaptureI(Stator_Axis_T * p_axis, fract16_t i) { p_axis->IPrev = p_axis->I; p_axis->I = i; p_axis->V = p_axis->VOut; }
static inline void Stator_Axis_CaptureVOut(Stator_Axis_T * p_axis, fract16_t v) { p_axis->VOut = v; }

static inline fract16_t Stator_Axis_IMean(const Stator_Axis_T * p_axis) { return stator_i_mean(p_axis->IPrev, p_axis->I); }
static inline accum32_t Stator_Axis_IDelta(const Stator_Axis_T * p_axis) { return stator_i_delta(p_axis->IPrev, p_axis->I); }

/*
    Per phase. L = Lq resolves the extended EMF E = ω·(ψ + (Ld − Lq)·i_d) of a salient rotor. Ls = (Ld + Lq) / 2 leaves a 2θ error.
*/
typedef struct Stator_RL
{
    fract16_t R;    /* phase resistance */
    accum32_t L;    /* phase inductance at the rad base */
}
Stator_RL_T;

/*
    The period through the winding, V = R·ī + L·ΔI + ē
*/
static inline accum32_t Stator_Axis_VR(const Stator_Axis_T * p_axis, const Stator_RL_T * p_rl) { return stator_v_r(p_rl->R, p_axis->IPrev, p_axis->I); }
static inline accum32_t Stator_Axis_VL(const Stator_Axis_T * p_axis, const Stator_RL_T * p_rl) { return stator_v_l(p_rl->L, p_axis->IPrev, p_axis->I); }
static inline accum32_t Stator_Axis_VDrop(const Stator_Axis_T * p_axis, const Stator_RL_T * p_rl) { return stator_v_drop(p_rl->R, p_rl->L, p_axis->IPrev, p_axis->I); }

/* ē over the period, [stator_emf] */
static inline accum32_t Stator_Axis_Emf(const Stator_Axis_T * p_axis, const Stator_RL_T * p_rl) { return stator_emf(p_rl->R, p_rl->L, p_axis->IPrev, p_axis->I, p_axis->V); }

/* The next period: the VOut that moves I to i against e, [stator_v] */
static inline accum32_t Stator_Axis_VOutOf(const Stator_Axis_T * p_axis, const Stator_RL_T * p_rl, fract16_t i, accum32_t e) { return stator_v(p_rl->R, p_rl->L, p_axis->I, i, e); }

/*
    Power over the period, each term of the period equation times ī, [stator_power]:
        V·ī = R·ī² + L·ΔI·ī + ē·ī
*/
static inline accum32_t _Stator_Axis_PowerOf(const Stator_Axis_T * p_axis, accum32_t v) { return stator_power(v, p_axis->IPrev, p_axis->I); }

static inline accum32_t Stator_Axis_Power(const Stator_Axis_T * p_axis) { return _Stator_Axis_PowerOf(p_axis, p_axis->V); }
static inline accum32_t Stator_Axis_PowerR(const Stator_Axis_T * p_axis, const Stator_RL_T * p_rl) { return _Stator_Axis_PowerOf(p_axis, Stator_Axis_VR(p_axis, p_rl)); }
static inline accum32_t Stator_Axis_PowerL(const Stator_Axis_T * p_axis, const Stator_RL_T * p_rl) { return _Stator_Axis_PowerOf(p_axis, Stator_Axis_VL(p_axis, p_rl)); }
static inline accum32_t Stator_Axis_PowerEmf(const Stator_Axis_T * p_axis, const Stator_RL_T * p_rl) { return _Stator_Axis_PowerOf(p_axis, Stator_Axis_Emf(p_axis, p_rl)); }

/*
    A frame struct, [foc_abc] [foc_alphabeta] [foc_dq], holds one quantity across the axes: the operand of the transforms.
    An axis holds the quantities of one axis: the operand of the stator equation, stator_math.h.
*/
/******************************************************************************/
/*
    abc image, per phase.
*/
/******************************************************************************/
// typedef struct Stator_Abc
// {
//     Stator_RL_T RL;
//     Stator_Axis_T A;
//     Stator_Axis_T B;
//     Stator_Axis_T C;
// }
// Stator_Abc_T;


/******************************************************************************/
/*
    αβ image. No zero sequence: the neutral offset of the voltages and the common error of the current sensors drop out.
*/
/******************************************************************************/
typedef struct Stator_AlphaBeta
{
    Stator_RL_T RL;
    Stator_Axis_T Alpha;
    Stator_Axis_T Beta;
}
Stator_AlphaBeta_T;

static inline void Stator_AlphaBeta_CaptureI(Stator_AlphaBeta_T * p_stator, fract16_t i_alpha, fract16_t i_beta)
{
    Stator_Axis_CaptureI(&p_stator->Alpha, i_alpha);
    Stator_Axis_CaptureI(&p_stator->Beta, i_beta);
}

static inline void Stator_AlphaBeta_CaptureVOut(Stator_AlphaBeta_T * p_stator, fract16_t v_alpha, fract16_t v_beta)
{
    Stator_Axis_CaptureVOut(&p_stator->Alpha, v_alpha);
    Stator_Axis_CaptureVOut(&p_stator->Beta, v_beta);
}

/* From the phase sample, through Clarke */
static inline void Stator_AlphaBeta_CaptureIabc(Stator_AlphaBeta_T * p_stator, fract16_t i_a, fract16_t i_b, fract16_t i_c)
{
    struct foc_alphabeta i = foc_clarke(i_a, i_b, i_c);
    Stator_AlphaBeta_CaptureI(p_stator, fract16_sat(i.alpha), fract16_sat(i.beta));
}

/* Back across the axes, as the operand of the transforms */
static inline struct foc_alphabeta Stator_AlphaBeta_I(const Stator_AlphaBeta_T * p_stator) { return (struct foc_alphabeta) { .alpha = p_stator->Alpha.I, .beta = p_stator->Beta.I }; }
static inline struct foc_alphabeta Stator_AlphaBeta_V(const Stator_AlphaBeta_T * p_stator) { return (struct foc_alphabeta) { .alpha = p_stator->Alpha.V, .beta = p_stator->Beta.V }; }
static inline struct foc_alphabeta Stator_AlphaBeta_Emf(const Stator_AlphaBeta_T * p_stator) { return (struct foc_alphabeta) { .alpha = Stator_Axis_Emf(&p_stator->Alpha, &p_stator->RL), .beta = Stator_Axis_Emf(&p_stator->Beta, &p_stator->RL) }; }

static inline ufract16_t Stator_AlphaBeta_IMagnitude(const Stator_AlphaBeta_T * p_stator) { return fract16_vector_magnitude(p_stator->Alpha.I, p_stator->Beta.I); }
static inline ufract16_t Stator_AlphaBeta_VMagnitude(const Stator_AlphaBeta_T * p_stator) { return fract16_vector_magnitude(p_stator->Alpha.V, p_stator->Beta.V); }

/* Power over the period, the sum over the axes */
static inline accum32_t Stator_AlphaBeta_Power(const Stator_AlphaBeta_T * p_stator) { return Stator_Axis_Power(&p_stator->Alpha) + Stator_Axis_Power(&p_stator->Beta); }
static inline accum32_t Stator_AlphaBeta_PowerR(const Stator_AlphaBeta_T * p_stator) { return Stator_Axis_PowerR(&p_stator->Alpha, &p_stator->RL) + Stator_Axis_PowerR(&p_stator->Beta, &p_stator->RL); }
static inline accum32_t Stator_AlphaBeta_PowerL(const Stator_AlphaBeta_T * p_stator) { return Stator_Axis_PowerL(&p_stator->Alpha, &p_stator->RL) + Stator_Axis_PowerL(&p_stator->Beta, &p_stator->RL); }
static inline accum32_t Stator_AlphaBeta_PowerEmf(const Stator_AlphaBeta_T * p_stator) { return Stator_Axis_PowerEmf(&p_stator->Alpha, &p_stator->RL) + Stator_Axis_PowerEmf(&p_stator->Beta, &p_stator->RL); }

/* V × ī, rotation invariant: the dq form [foc_reactive_power] */
static inline accum32_t Stator_AlphaBeta_ReactivePower(const Stator_AlphaBeta_T * p_stator)
{
    return foc_reactive_power(p_stator->Alpha.V, Stator_Axis_IMean(&p_stator->Alpha), p_stator->Beta.V, Stator_Axis_IMean(&p_stator->Beta));
}

/*
    In the frame at θ, turning Δθ per period. I is the sample at θ.
    V, held across the period, parks at the period's mean angle θ − Δθ/2.
*/
static inline struct foc_dq Stator_AlphaBeta_IDq(const Stator_AlphaBeta_T * p_stator, angle16_t theta)
{
    struct fract16_xy unit = fract16_vector(theta);
    return foc_park(p_stator->Alpha.I, p_stator->Beta.I, unit.y, unit.x);
}

static inline struct foc_dq Stator_AlphaBeta_VDq(const Stator_AlphaBeta_T * p_stator, angle16_t theta, angle16_t delta)
{
    struct fract16_xy unit = fract16_vector(theta - delta / 2);
    return foc_park(p_stator->Alpha.V, p_stator->Beta.V, unit.y, unit.x);
}

/*
    ē in the frame at the operating point, i_dq DC: L·ΔI of the axes becomes X·J·i, X = ω·L [stator_reactance], with no difference of samples.
    With L = Lq, ê_d is the phase detector and ê_q = E. [foc_vemf_d] [foc_vemf_q]
*/
static inline struct foc_dq Stator_AlphaBeta_EmfDq(const Stator_AlphaBeta_T * p_stator, angle16_t theta, angle16_t delta)
{
    struct foc_dq i = Stator_AlphaBeta_IDq(p_stator, theta);
    struct foc_dq v = Stator_AlphaBeta_VDq(p_stator, theta, delta);
    accum32_t x = stator_reactance(p_stator->RL.L, delta);
    return (struct foc_dq) { .d = foc_vemf_d(p_stator->RL.R, x, v.d, i.d, i.q), .q = foc_vemf_q(p_stator->RL.R, x, v.q, i.d, i.q) };
}


/******************************************************************************/
/*
    dq image at the operating point, signals DC: no di/dt term, no IPrev.
*/
/******************************************************************************/
// typedef struct Stator_DqAxis
// {
//     fract16_t I;
//     fract16_t V;
// }
// Stator_DqAxis_T;

// typedef struct Stator_Dq
// {
//     Stator_RL_T RL;
//     Stator_DqAxis_T D;
//     Stator_DqAxis_T Q;
// }
// Stator_Dq_T;
