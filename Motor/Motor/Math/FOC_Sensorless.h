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
    @file   FOC_Sensorless.h
    @author FireSourcery
    @brief  Stateful sensorless rotor-position observer.

    Composes foc_sensorless_math primitives into a per-control-tick step, over the observer's αβ image of the stator [Stator_AlphaBeta_T].
    Four EMF front ends, one tail:
        Step            SMO, stationary frame                   i_αβ, v_αβ → ê_αβ → LPF → PLL → θ̂, ω̂
        StepVModel      voltage model, estimated dq             i_dq, v_dq → ê_dq → LPF → PLL → θ̂, ω̂
        StepEemf        SMO, estimated dq                       i_dq, v_dq → ê_dq → LPF → PLL → θ̂, ω̂
        StepEemfSta     super-twisting SMO, estimated dq        i_dq, v_dq → ê_dq → PLL → θ̂, ω̂
    The tail detects the rotor vector of ê [foc_rotor_of_emf] in the shared [AnglePll_T].

    Caller owns the FOC chain. Each control tick, after the FOC proc, the integration layer:
        1. Captures the period into Stator: the i_abc sampled this tick [Stator_AlphaBeta_CaptureIabc],
           then the v_αβ commanded this tick [Stator_AlphaBeta_CaptureVOut].
        2. Steps one front end.
        3. Reads θ̂ via FOC_Sensorless_GetAngle().

    Per-unit, the plant at the rad base [stator_math.h]
        v_pu   = V / V_base
        i_pu   = I / I_base
        Rs_pu  = Rs · I_base / V_base
        L_rad  = L · Fs · I_base / V_base = L_turn / π
*/
/******************************************************************************/
#include "foc_sensorless_math.h"
#include "smo_math.h"
#include "../Model/Stator.h"
#include "Math/Fixed/fract16.h"
#include "Math/Angle/Angle.h"
#include "Math/Angle/AnglePll.h"
#include "Math/PID/PID.h"

#include <stdint.h>
#include <stdbool.h>


/*
    Observer tuning. Holds no live state, and nothing derived from the motor model.
*/
typedef struct FOC_SensorlessConfig
{
    /* SMO tuning, Step and StepEemf. */
    fract16_t K_smo;        /* sliding gain — > peak EMF in pu */
    fract16_t SmoSat;       /* boundary-layer width for smo_sat */

    /* Super-twisting tuning, StepEemfSta. */
    fract16_t StaLambda;    /* √|σ| gain */
    fract16_t StaAlphaDt;   /* integrator gain × dt */

    /* ê smoothing — the equivalent control of z (Step, StepEemf), measurement noise on the dq residual (StepVModel). */
    fract16_t LpfCoef;      /* dt / (τ + dt) */

    /* Lock detector. */
    ufract16_t LockEmfMin;  /* |ê| floor below which lock is indeterminate */
    ufract16_t LockErrTol;  /* |err| tolerance (normalized PLL error) */
    uint16_t   LockHoldCount; /* consecutive cycles within tol+floor for lock */

    /* PLL loop filter — output is ω̂ in angle16/dt units. */
    PID_Config_T PllPid;
}
FOC_SensorlessConfig_T;

/*
    Conservative starting point; K_smo and PllPid will need tuning per motor.
    LpfCoef: ~100 Hz at 20 kHz (k_lp = dt/(τ+dt), τ = 1/(2π·100) = 1592 µs, dt = 50 µs).
    PLL gains: normalized phase error (fract16 ±1) → ω̂ (angle16/dt). Crossover ≈ Kp·π rad/dt, at the LPF cutoff,
        the PI zero Ki / (Fs·Kp) a quarter below it. Far above the LPF, the loop loses its phase margin.
*/
#define FOC_SENSORLESS_CONFIG_DEFAULT(VBus, v_max, I, i_max) (FOC_SensorlessConfig_T)     \
{                                                                                   \
    .K_smo         = (fract16_t)FRACT16(.60F * VBus / v_max),                       \
    .SmoSat        = (fract16_t)FRACT16(.15F * I / i_max),          /* 15 % of IMax boundary layer */                       \
    .StaLambda     = (fract16_t)FRACT16(.50F),                      /* STA √|σ| gain — re-tune per motor */                 \
    .StaAlphaDt    = (fract16_t)FRACT16(.001F),                     /* STA integrator gain × dt */                          \
    .LpfCoef       = 998,                                           /* ~100 Hz LPF at 20 kHz */                             \
    .LockEmfMin    = (ufract16_t)FRACT16(.05F * VBus / v_max),      /* 5 % of VMax */                                        \
    .LockErrTol    = (ufract16_t)FRACT16(.05F),                     /* 5 % normalized PLL error */                           \
    .LockHoldCount = 200U,                                          /* 10 ms at 20 kHz */                                   \
    .PllPid =                                                                       \
    {                                                                               \
        .Mode       = PID_MODE_PI,                                                  \
        .SampleFreq = 20000U,                                                       \
        .Kp         = 328,       /* ≈ 0.01 Q17.15, crossover ≈ 100 Hz */            \
        .Ki         = 51472,     /* ≈ 1.57 /s Q17.15, zero ≈ 25 Hz */               \
        .Kd         = 0,                                                            \
    },                                                                              \
}

typedef struct FOC_Sensorless
{
    /* The plant beside its period, derived from the caller's model: R, L and 1 / L, and the αβ image the caller captures. */
    Stator_AlphaBeta_T Stator;
    // accum32_t G;                        /* 1 / L, the current model's step [stator_i_next] */

    /* The image in the frame θ̂, for the dq front ends. */
    fract16_t Id, Iq;
    fract16_t Vd, Vq;

    /* The model's image beside the sampled one: the stator equation closes each period, z in place of e [Stator_Axis_Integrate]. */
    Stator_Axis_T SmoAlpha, SmoBeta;
    Stator_Axis_T SmoD, SmoQ;           /* in the frame θ̂, its turn in the axis voltage */

    /* SMO internal state. Step in αβ, StepEemf and StepEemfSta in the estimated dq. */
    // fract16_t SmoIAlpha, SmoIBeta;   /* current-estimator state î */
    fract16_t SmoZAlpha, SmoZBeta;      /* raw switching variable z */

    // fract16_t SmoId, SmoIq;
    fract16_t SmoZd, SmoZq;

    /* Smoothed back-EMF estimate — observer output. αβ from Step, dq in the estimated frame from the dq front ends. */
    fract16_t EmfAlpha, EmfBeta;
    fract16_t EmfD, EmfQ;               /* StepEemfSta: the super-twisting integrators */
    ufract16_t EmfMag;

    AnglePll_T Pll;                     /* θ̂, ω̂ */

    /* Lock detector. */
    uint16_t LockCount;

    FOC_SensorlessConfig_T Config;
}
FOC_Sensorless_T;


/******************************************************************************/
/*!
    Per-tick step, over the αβ image [Stator_AlphaBeta_T]: i sampled this tick, v applied across the period it ends.
*/
/******************************************************************************/
/*
    The tail, shared by the front ends: |ê|, the PLL on the rotor vector of ê, and the lock detector.
*/
/* Rotation direction of θ̂. ω̂ = 0 reads forward, so the phase detector never stalls at zero */
static inline sign_t _FOC_Sensorless_Direction(const FOC_Sensorless_T * p_obs) { return (Angle_Delta(&p_obs->Pll.Base) < 0) ? SIGN_NEGATIVE : SIGN_POSITIVE; }

/* Lock detector — strong EMF AND tight PLL error, sustained. */
static void _FOC_Sensorless_ProcLock(FOC_Sensorless_T * p_obs)
{
    bool stable = (p_obs->EmfMag > p_obs->Config.LockEmfMin) && (fract16_abs(p_obs->Pll.PhaseError) < p_obs->Config.LockErrTol);
    p_obs->LockCount = stable ? (uint16_t)math_min(p_obs->LockCount + 1, p_obs->Config.LockHoldCount) : 0;
}

/******************************************************************************/
/*!
    @brief  Rotor angle from the back-EMF.

    For CCW rotation (ω > 0), with d aligned to rotor flux, e = jω·ψ·e^{jθ}:
        e_α = −ω·ψ·sin(θ),
        e_β =  ω·ψ·cos(θ)
    therefore:
        θ = atan2(−e_α,  e_β)               (ω > 0)
        θ = atan2( e_α, −e_β)               (ω < 0)

    The direction comes from outside the EMF, e.g. sign(ω̂) of the PLL or the open-loop ramp.
*/
/******************************************************************************/
/* ê_αβ: the rotor vector, detected against θ̂ */
static void _FOC_Sensorless_ProcPllAlphaBeta(FOC_Sensorless_T * p_obs)
{
    struct fract16_xy rotor = foc_rotor_of_emf(_FOC_Sensorless_Direction(p_obs), p_obs->EmfAlpha, p_obs->EmfBeta);
    p_obs->EmfMag = fract16_vector_magnitude(rotor.x, rotor.y);
    AnglePll_ProcVector(&p_obs->Pll, p_obs->Config.LockEmfMin, rotor.x, rotor.y);
    _FOC_Sensorless_ProcLock(p_obs);
}

/*
    SMO, stationary frame. The model's image across the period [Stator_Axis_Integrate], then z against the sample that ends it [smo_z]; z is the raw EMF.
*/
// Stator_AlphaBeta_CaptureIabc(p_stator, p_context->PhaseInput.I.Values.A, p_context->PhaseInput.I.Values.B, p_context->PhaseInput.I.Values.C);
// Stator_AlphaBeta_CaptureVOut(p_stator, fract16_sat(v.alpha), fract16_sat(v.beta));
static void FOC_Sensorless_Step(FOC_Sensorless_T * p_obs)
// static void FOC_Sensorless_Step(FOC_Sensorless_T * p_obs, fract16_t i_alpha, fract16_t i_beta)
{
    /* 1. The model across the period, then z against the sample. */
    Stator_Axis_Integrate(&p_obs->Stator.Winding, &p_obs->SmoAlpha, p_obs->Stator.Alpha.V, p_obs->SmoZAlpha);
    Stator_Axis_Integrate(&p_obs->Stator.Winding, &p_obs->SmoBeta, p_obs->Stator.Beta.V, p_obs->SmoZBeta);
    p_obs->SmoZAlpha = smo_z(p_obs->Config.K_smo, p_obs->Config.SmoSat, smo_sigma(p_obs->SmoAlpha.I, p_obs->Stator.Alpha.I));
    p_obs->SmoZBeta = smo_z(p_obs->Config.K_smo, p_obs->Config.SmoSat, smo_sigma(p_obs->SmoBeta.I, p_obs->Stator.Beta.I));

    // Stator_Axis_ISmo(&p_obs->Stator.Alpha, &p_obs->Stator.Winding, p_obs->SmoZAlpha);
    // Stator_Axis_ISmo(&p_obs->Stator.Beta, &p_obs->Stator.Winding, p_obs->SmoZBeta);

    // p_obs->Stator.Alpha.I = Stator_Axis_ISmo(&p_obs->Stator.Alpha, &p_obs->Stator.Winding, p_obs->SmoZAlpha);
    // p_obs->Stator.Beta.I = Stator_Axis_ISmo(&p_obs->Stator.Beta, &p_obs->Stator.Winding, p_obs->SmoZBeta);

    // p_obs->SmoIAlpha = smo_axis_i(p_obs->G, p_obs->Stator.Winding.R, p_obs->Stator.Alpha.V, p_obs->SmoIAlpha, p_obs->SmoZAlpha);
    // p_obs->SmoIBeta = smo_axis_i(p_obs->G, p_obs->Stator.Winding.R, p_obs->Stator.Beta.V, p_obs->SmoIBeta, p_obs->SmoZBeta);
    // p_obs->SmoZAlpha = smo_axis_z(p_obs->Config.K_smo, p_obs->Config.SmoSat, p_obs->SmoIAlpha, p_obs->Stator.Alpha.I);
    // p_obs->SmoZBeta = smo_axis_z(p_obs->Config.K_smo, p_obs->Config.SmoSat, p_obs->SmoIBeta, p_obs->Stator.Beta.I);

    /* 2. LPF to extract equivalent control / smooth measurement noise. */
    p_obs->EmfAlpha = lpf_step(p_obs->Config.LpfCoef, p_obs->EmfAlpha, p_obs->SmoZAlpha);
    p_obs->EmfBeta = lpf_step(p_obs->Config.LpfCoef, p_obs->EmfBeta, p_obs->SmoZBeta);

    _FOC_Sensorless_ProcPllAlphaBeta(p_obs);
}

/******************************************************************************/
/*!
    @brief  PLL Phase Detector — d-axis projection of estimated EMF.

    With d aligned to rotor flux ψ_f, back-EMF lies on the q-axis, e = jω·ψ·e^{jθ}. In the frame of θ̂:
        e_d  =   e_α·cos(θ̂) + e_β·sin(θ̂)
        e_q  =  -e_α·sin(θ̂) + e_β·cos(θ̂)
    e_d → 0 at lock; near lock e_d ≈ −E·sin(θ − θ̂) ≈ −E·Δθ, E = ω·ψ.

    Sign of e_d flips with rotation direction (E ∝ ω). The rotor vector [foc_rotor_of_emf]
    removes it, so the detector [angle_pll_error] sees a vector along θ in either direction.
*/
/******************************************************************************/
/* ê_dq, already in the frame of θ̂: the rotor vector relative to it */
static void _FOC_Sensorless_ProcPllDq(FOC_Sensorless_T * p_obs)
{
    struct fract16_xy rotor = foc_rotor_of_emf(_FOC_Sensorless_Direction(p_obs), p_obs->EmfD, p_obs->EmfQ);
    p_obs->EmfMag = fract16_vector_magnitude(rotor.x, rotor.y);
    AnglePll_Proc(&p_obs->Pll, angle_pll_error_normalized_relative(p_obs->Config.LockEmfMin, rotor.x, rotor.y));
    _FOC_Sensorless_ProcLock(p_obs);
}

/*
    The image in the frame θ̂ [foc_park]: I sampled at θ̂, V held across the period at its mean angle θ̂ − Δθ̂/2.
*/
static void _FOC_Sensorless_ProcPark(FOC_Sensorless_T * p_obs)
{
    struct fract16_xy sample = fract16_vector(Angle_Value(&p_obs->Pll.Base));
    struct fract16_xy hold = fract16_vector(Angle_Value(&p_obs->Pll.Base) - Angle_Delta(&p_obs->Pll.Base) / 2);
    struct foc_dq i = foc_park(p_obs->Stator.Alpha.I, p_obs->Stator.Beta.I, sample.y, sample.x);
    struct foc_dq v = foc_park(p_obs->Stator.Alpha.V, p_obs->Stator.Beta.V, hold.y, hold.x);
    p_obs->Id = fract16_sat(i.d);  p_obs->Iq = fract16_sat(i.q);
    p_obs->Vd = fract16_sat(v.d);  p_obs->Vq = fract16_sat(v.q);
}

/*
    Voltage model, estimated dq frame θ̂ [foc_vemf_d] [foc_vemf_q]. Steady state: i_dq is DC at lock, so L·ΔI of the axes
    becomes X·J·i, X = ω̂·L [stator_reactance], with no difference of samples, and no LPF phase lag.
*/
static void FOC_Sensorless_StepVModel(FOC_Sensorless_T * p_obs)
{
    _FOC_Sensorless_ProcPark(p_obs);
    accum32_t x = stator_reactance(p_obs->Stator.Winding.L, Angle_Delta(&p_obs->Pll.Base));

    p_obs->EmfD = lpf_step(p_obs->Config.LpfCoef, p_obs->EmfD, fract16_sat(foc_vemf_d(p_obs->Stator.Winding.R, x, p_obs->Vd, p_obs->Id, p_obs->Iq)));
    p_obs->EmfQ = lpf_step(p_obs->Config.LpfCoef, p_obs->EmfQ, fract16_sat(foc_vemf_q(p_obs->Stator.Winding.R, x, p_obs->Vq, p_obs->Id, p_obs->Iq)));

    _FOC_Sensorless_ProcPllDq(p_obs);
}

/*
    The model's image in the frame θ̂ across the period [Stator_Axis_Integrate]: z of the last tick in place of e,
    and the frame's turn X·J·i on the measured current in the axis voltage, X = ω̂·L, [foc_vd_ff] [foc_vq_ff].
    Each axis is then a scalar SMO. On î, the boundary layer's î − i ≈ E·thr/K would cross through X into the other axis,
    θ̂ lagging by atan(X·thr / (K + R·thr)).
*/
static void _FOC_Sensorless_ProcIEstDq(FOC_Sensorless_T * p_obs)
{
    accum32_t x = stator_reactance(p_obs->Stator.Winding.L, Angle_Delta(&p_obs->Pll.Base));
    Stator_Axis_Integrate(&p_obs->Stator.Winding, &p_obs->SmoD, p_obs->Vd - foc_vd_ff(x, p_obs->Iq), p_obs->SmoZd);
    Stator_Axis_Integrate(&p_obs->Stator.Winding, &p_obs->SmoQ, p_obs->Vq - foc_vq_ff(x, 0, p_obs->Id), p_obs->SmoZq);
    // p_obs->SmoId = smo_eemf_i_decouple(p_obs->G, p_obs->Stator.Winding.R, p_obs->Vd, foc_vd_ff(x, p_obs->Iq), p_obs->SmoId, p_obs->SmoZd);
    // p_obs->SmoIq = smo_eemf_i_decouple(p_obs->G, p_obs->Stator.Winding.R, p_obs->Vq, foc_vq_ff(x, 0, p_obs->Id), p_obs->SmoIq, p_obs->SmoZq);
}

/*
    EEMF SMO, estimated dq frame θ̂. î across the period, then z against the sample that ends it.
    z is DC at lock, so the LPF takes the chatter without the lag it costs in the stationary frame.
*/
static void FOC_Sensorless_StepEemf(FOC_Sensorless_T * p_obs)
{
    _FOC_Sensorless_ProcPark(p_obs);
    _FOC_Sensorless_ProcIEstDq(p_obs);
    p_obs->SmoZd = smo_z(p_obs->Config.K_smo, p_obs->Config.SmoSat, smo_sigma(p_obs->SmoD.I, p_obs->Id));
    p_obs->SmoZq = smo_z(p_obs->Config.K_smo, p_obs->Config.SmoSat, smo_sigma(p_obs->SmoQ.I, p_obs->Iq));
    // p_obs->SmoZd = smo_axis_z(p_obs->Config.K_smo, p_obs->Config.SmoSat, p_obs->SmoId, p_obs->Id);
    // p_obs->SmoZq = smo_axis_z(p_obs->Config.K_smo, p_obs->Config.SmoSat, p_obs->SmoIq, p_obs->Iq);

    p_obs->EmfD = lpf_step(p_obs->Config.LpfCoef, p_obs->EmfD, p_obs->SmoZd);
    p_obs->EmfQ = lpf_step(p_obs->Config.LpfCoef, p_obs->EmfQ, p_obs->SmoZq);

    _FOC_Sensorless_ProcPllDq(p_obs);
}

/*
    Super-twisting EEMF SMO, estimated dq frame θ̂: î as StepEemf, z by [sta_axis_step]. z is continuous;

    its integrator w converges to the EMF, so ê is w itself, no LPF.
*/
// static void FOC_Sensorless_StepEemfSta(FOC_Sensorless_T * p_obs)
// {
//     _FOC_Sensorless_ProcPark(p_obs);
//     _FOC_Sensorless_ProcIEstDq(p_obs);
//     struct sta_axis d = sta_axis_step(p_obs->Config.StaLambda, p_obs->Config.StaAlphaDt, p_obs->EmfD, p_obs->SmoId, p_obs->Id);
//     struct sta_axis q = sta_axis_step(p_obs->Config.StaLambda, p_obs->Config.StaAlphaDt, p_obs->EmfQ, p_obs->SmoIq, p_obs->Iq);
//     p_obs->SmoZd = d.z;  p_obs->EmfD = d.w;
//     p_obs->SmoZq = q.z;  p_obs->EmfQ = q.w;

//     _FOC_Sensorless_ProcPllDq(p_obs);
// }

/******************************************************************************/
/*!
    Per-control-tick step.

    Call once per PWM cycle, after Stator captured this tick's period.
    Updates ê, θ̂, ω̂, and the lock state internally.
*/
/******************************************************************************/
/* Bumpless seed: align θ̂ and ω̂ to a known reference (e.g. open-loop ramp). */
static inline void FOC_Sensorless_SeedAngle(FOC_Sensorless_T * p_obs, angle16_t theta, angle16_t delta) { AnglePll_Seed(&p_obs->Pll, theta, delta); }

/******************************************************************************/
/*!
    Lifecycle
*/
/******************************************************************************/
/* Reset all per-cycle state. Calibration in CONFIG and the plant are untouched. */
static void FOC_Sensorless_ResetState(FOC_Sensorless_T * p_obs)
{
    p_obs->Stator.Alpha = (Stator_Axis_T) { 0 };  p_obs->Stator.Beta = (Stator_Axis_T) { 0 };
    p_obs->Id = 0;  p_obs->Iq = 0;  p_obs->Vd = 0;  p_obs->Vq = 0;
    p_obs->SmoAlpha = (Stator_Axis_T) { 0 };  p_obs->SmoBeta = (Stator_Axis_T) { 0 };  p_obs->SmoZAlpha = 0;  p_obs->SmoZBeta = 0;
    p_obs->SmoD = (Stator_Axis_T) { 0 };  p_obs->SmoQ = (Stator_Axis_T) { 0 };  p_obs->SmoZd = 0;  p_obs->SmoZq = 0;
    p_obs->EmfAlpha = 0;  p_obs->EmfBeta = 0;
    p_obs->EmfD = 0;  p_obs->EmfQ = 0;
    p_obs->EmfMag = 0;
    p_obs->LockCount = 0;
    AnglePll_Reset(&p_obs->Pll);
}


static void FOC_Sensorless_Init(FOC_Sensorless_T * p_obs, const FOC_SensorlessConfig_T * p_config)
{
    if (p_config != NULL) { p_obs->Config = *p_config; }
    AnglePll_InitFrom(&p_obs->Pll, &p_obs->Config.PllPid);
    FOC_Sensorless_ResetState(p_obs);
}

/*
    The plant from the caller's model, turn base. Lq on both axes, the extended EMF [Stator_Winding_T], one plant for every front end.
    L at the rad base, L_turn / π. G = 1 / L = Ts · V_base / (Lq · I_base) is the current model's step, derived here.

    Validate checks (lq_turn != 0U)
*/
static void FOC_Sensorless_InitElectrical(FOC_Sensorless_T * p_obs, uint32_t lq_turn, accum32_t rs)
{
    p_obs->Stator.Winding = (Stator_Winding_T) { .R = rs, .L = accum32_mul(lq_turn, FRACT16_1_DIV_PI), .G = g_l_pu_turn(lq_turn) };
}


/******************************************************************************/
/*!
    Observer outputs
*/
/******************************************************************************/
static inline angle16_t  FOC_Sensorless_GetAngle(const FOC_Sensorless_T * p_obs) { return Angle_Value(&p_obs->Pll.Base); }
static inline angle16_t  FOC_Sensorless_GetDelta(const FOC_Sensorless_T * p_obs) { return Angle_Delta(&p_obs->Pll.Base); }
static inline fract16_t  FOC_Sensorless_GetEmfAlpha(const FOC_Sensorless_T * p_obs) { return p_obs->EmfAlpha; }
static inline fract16_t  FOC_Sensorless_GetEmfBeta(const FOC_Sensorless_T * p_obs) { return p_obs->EmfBeta; }
static inline ufract16_t FOC_Sensorless_GetEmfMag(const FOC_Sensorless_T * p_obs) { return p_obs->EmfMag; }
static inline bool       FOC_Sensorless_IsLocked(const FOC_Sensorless_T * p_obs) { return  (p_obs->LockCount >= p_obs->Config.LockHoldCount); }
static inline fract16_t  FOC_Sensorless_GetPllErr(const FOC_Sensorless_T * p_obs) { return p_obs->Pll.PhaseError; }

/******************************************************************************/
/*!  */
/******************************************************************************/
typedef enum FOC_SensorlessVar
{
    FOC_SENSORLESS_VAR_ANGLE,
    FOC_SENSORLESS_VAR_DELTA,
    FOC_SENSORLESS_VAR_EMF_ALPHA,
    FOC_SENSORLESS_VAR_EMF_BETA,
    FOC_SENSORLESS_VAR_EMF_MAG,
    FOC_SENSORLESS_VAR_PLL_ERR,
}
FOC_SensorlessVar_T;

static inline int32_t FOC_Sensorless_GetVar(const FOC_Sensorless_T * p_obs, FOC_SensorlessVar_T var)
{
    switch (var)
    {
        case FOC_SENSORLESS_VAR_ANGLE: return FOC_Sensorless_GetAngle(p_obs);
        case FOC_SENSORLESS_VAR_DELTA: return FOC_Sensorless_GetDelta(p_obs);
        case FOC_SENSORLESS_VAR_EMF_ALPHA: return FOC_Sensorless_GetEmfAlpha(p_obs);
        case FOC_SENSORLESS_VAR_EMF_BETA: return FOC_Sensorless_GetEmfBeta(p_obs);
        case FOC_SENSORLESS_VAR_EMF_MAG: return FOC_Sensorless_GetEmfMag(p_obs);
        case FOC_SENSORLESS_VAR_PLL_ERR: return FOC_Sensorless_GetPllErr(p_obs);
        default: return 0;
    }
}

typedef enum FOC_SensorlessConfigVar
{
    FOC_SENSORLESS_CONFIG_VAR_K_SMO,
    FOC_SENSORLESS_CONFIG_VAR_SMO_SAT,
    FOC_SENSORLESS_CONFIG_VAR_LPF_COEF,
    FOC_SENSORLESS_CONFIG_VAR_LOCK_EMF_MIN,
    FOC_SENSORLESS_CONFIG_VAR_LOCK_ERR_TOL,
    FOC_SENSORLESS_CONFIG_VAR_LOCK_HOLD_COUNT,
}
FOC_SensorlessConfigVar_T;

static inline int32_t FOC_SensorlessConfig_Get(const FOC_SensorlessConfig_T * p_obs, FOC_SensorlessConfigVar_T var)
{
    switch (var)
    {
        case FOC_SENSORLESS_CONFIG_VAR_K_SMO: return p_obs->K_smo;
        case FOC_SENSORLESS_CONFIG_VAR_SMO_SAT: return p_obs->SmoSat;
        case FOC_SENSORLESS_CONFIG_VAR_LPF_COEF: return p_obs->LpfCoef;
        case FOC_SENSORLESS_CONFIG_VAR_LOCK_EMF_MIN: return p_obs->LockEmfMin;
        case FOC_SENSORLESS_CONFIG_VAR_LOCK_ERR_TOL: return p_obs->LockErrTol;
        case FOC_SENSORLESS_CONFIG_VAR_LOCK_HOLD_COUNT: return p_obs->LockHoldCount;
        default: return 0;
    }
}
