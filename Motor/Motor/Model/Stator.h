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

static inline fract16_t Stator_Axis_IAvg(const Stator_Axis_T * p_axis) { return ((int32_t)p_axis->IPrev + p_axis->I) / 2; }
static inline fract16_t Stator_Axis_IDelta(const Stator_Axis_T * p_axis) { return p_axis->I - p_axis->IPrev; }


/*
    Per phase. L = Lq resolves the extended EMF E = ω·(ψ + (Ld − Lq)·i_d) of a salient rotor. Ls = (Ld + Lq) / 2 leaves a 2θ error.
*/
typedef struct Stator_RL
{
    accum32_t R;    /* phase resistance */
    accum32_t L;    /* phase inductance at the rad base */
    accum32_t G;    /* 1 / L at the rad base */
    // union { accum32_t L; accum32_t G; }; /* determined by axis type */
}
Stator_RL_T;

/*
    The period through the winding, V = R·I + L·ΔI + ē
*/
static inline accum32_t Stator_Axis_VDrop(const Stator_Axis_T * p_axis, const Stator_RL_T * p_rl) { return stator_v_drop(p_rl->R, p_rl->L, p_axis->IPrev, p_axis->I); }

/* ē over the period, [stator_emf] */
static inline accum32_t Stator_Axis_Emf(const Stator_Axis_T * p_axis, const Stator_RL_T * p_rl) { return stator_emf(p_rl->R, p_rl->L, p_axis->IPrev, p_axis->I, p_axis->V); }

/* The next period: the VOut that moves I to i against e, [stator_v] */
static inline accum32_t Stator_Axis_VOutOf(const Stator_Axis_T * p_axis, const Stator_RL_T * p_rl, fract16_t i, accum32_t e) { return stator_v(p_rl->R, p_rl->L, p_axis->I, i, e); }

/* Power over the period, [stator_power]: at the terminals, and through ē, the air gap */
static inline accum32_t Stator_Axis_Power(const Stator_Axis_T * p_axis) { return stator_power(p_axis->V, p_axis->IPrev, p_axis->I); }
static inline accum32_t Stator_Axis_PowerEmf(const Stator_Axis_T * p_axis, const Stator_RL_T * p_rl) { return stator_power(Stator_Axis_Emf(p_axis, p_rl), p_axis->IPrev, p_axis->I); }

static inline accum32_t Stator_Axis_I(const Stator_Axis_T * p_axis, const Stator_RL_T * p_rl) { return stator_i_next(p_rl->R, p_rl->G, p_axis->I, p_axis->V, Stator_Axis_Emf(p_axis, p_rl)); }

// duplicates params
// typedef struct
// {
//     Stator_RL_T RL;
//     Stator_Axis_T Axis;
// }
// Stator_AxisModel_T;

// calling function can handle
// typedef struct
// {
//     Stator_RL_T * p_RL;
//     Stator_Axis_T Axis;
// }
// Stator_AxisModel_T;


// static inline Stator_Axis_T Stator_Axis_Step(Stator_Axis_T * p_axis, fract16_t i, fract16_t v) {  return (Stator_Axis_T){ .IPrev = p_axis->I, .I = i, .V = p_axis->VOut, .VOut = v }; }
/*
    The stator equation closes the period in place of a sample: I from v held across it, e in place of the EMF [stator_i_next].
    A model's image, run beside the sampled one.
*/
static inline void Stator_Axis_Integrate(Stator_Axis_T * p_axis, const Stator_RL_T * p_rl, accum32_t v, accum32_t e)
{
    p_axis->IPrev = p_axis->I;
    p_axis->I = fract16_sat(stator_i_next(p_rl->R, p_rl->G, p_axis->I, v, e));
    p_axis->V = fract16_sat(v);
}


/*
    An axis holds the quantities of one axis: the operand of the stator equation, stator_math.h. The image stores axes.
    A frame struct, [foc_abc] [foc_alphabeta] [foc_dq], holds one quantity across the axes: the return of a transform, unpacked at the call.
*/
/******************************************************************************/
/*
    abc image, per phase.
*/
/******************************************************************************/
// typedef struct Stator_Physical
// {
//     Stator_RL_T RL;
//     Stator_Axis_T A;
//     Stator_Axis_T B;
//     Stator_Axis_T C;
// }
// Stator_Physical_T;


/******************************************************************************/
/*
    αβ image. No zero sequence: the neutral offset of the voltages and the common error of the current sensors drop out.
*/
/******************************************************************************/
/* Stator_Stationary */
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

static inline void Stator_AlphaBeta_StepAlpha(Stator_AlphaBeta_T * p_stator, accum32_t v_alpha, accum32_t emf_alpha)
{
    Stator_Axis_Integrate(&p_stator->Alpha, &p_stator->RL, v_alpha, emf_alpha);
}

static inline void Stator_AlphaBeta_StepBeta(Stator_AlphaBeta_T * p_stator, accum32_t v_beta, accum32_t emf_beta)
{
    Stator_Axis_Integrate(&p_stator->Beta, &p_stator->RL, v_beta, emf_beta);
}

// caller handle
// static inline void Stator_AlphaBeta_Step(Stator_AlphaBeta_T * p_stator, accum32_t v_alpha, accum32_t emf_alpha, accum32_t v_beta, accum32_t emf_beta)
// {
//     Stator_AlphaBeta_StepAlpha(p_stator, v_alpha, emf_alpha);
//     Stator_AlphaBeta_StepBeta(p_stator, v_beta, emf_beta);
// }

// static inline void Stator_AlphaBeta_Step(Stator_AlphaBeta_T * p_stator)
// {
//     // Stator_Axis_Integrate
//     // Stator_Axis_Step(&p_stator->Alpha, &p_stator->RL);
//     // Stator_Axis_Step(&p_stator->Beta, &p_stator->RL);
// }

static inline ufract16_t Stator_AlphaBeta_IMagnitude(const Stator_AlphaBeta_T * p_stator) { return fract16_vector_magnitude(p_stator->Alpha.I, p_stator->Beta.I); }
static inline ufract16_t Stator_AlphaBeta_VMagnitude(const Stator_AlphaBeta_T * p_stator) { return fract16_vector_magnitude(p_stator->Alpha.V, p_stator->Beta.V); }

/* Power over the period, the sum over the axes */
static inline accum32_t Stator_AlphaBeta_Power(const Stator_AlphaBeta_T * p_stator) { return Stator_Axis_Power(&p_stator->Alpha) + Stator_Axis_Power(&p_stator->Beta); }
static inline accum32_t Stator_AlphaBeta_PowerEmf(const Stator_AlphaBeta_T * p_stator) { return Stator_Axis_PowerEmf(&p_stator->Alpha, &p_stator->RL) + Stator_Axis_PowerEmf(&p_stator->Beta, &p_stator->RL); }

/* V × ī, rotation invariant: the dq form [foc_reactive_power] */
static inline accum32_t Stator_AlphaBeta_ReactivePower(const Stator_AlphaBeta_T * p_stator)
{
    return foc_reactive_power(p_stator->Alpha.V, Stator_Axis_IAvg(&p_stator->Alpha), p_stator->Beta.V, Stator_Axis_IAvg(&p_stator->Beta));
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
//     struct {  fract16_t Sine, Cosine; } Theta;
//     Stator_DqAxis_T D;
//     Stator_DqAxis_T Q;
// }
// Stator_Dq_T;

// static inline void Stator_StepD(Stator_Dq_T * p_stator, accum32_t v, accum32_t v_ff, accum32_t emf)
// {
// }