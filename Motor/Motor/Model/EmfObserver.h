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
    @file   EmfObserver.h
    @author FireSourcery
    @brief  Rotor angle from the EMF of the stator.
*/
/******************************************************************************/
#include "../Math/foc_sensorless_math.h"
#include "Stator.h"
#include "Math/Angle/Angle.h"
#include "Math/Angle/AnglePll.h"
#include "Math/PID/PID.h"
#include "Math/Fixed/fract16.h"

#include <stdint.h>
#include <stdbool.h>

/*
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

/*
    Tuning. The plant, the stator's winding, derives from the motor's model [EmfObserver_InitWinding], and is not stored.
*/
typedef struct EmfObserver_Config
{
    /* SMO, StepSmo and StepEemf */
    fract16_t K_smo;            /* sliding gain — > peak EMF in pu */
    fract16_t SmoSat;           /* boundary-layer width for smo_sat */

    /* ê smoothing — the equivalent control of z (StepSmo, StepEemf), measurement noise on the dq residual (StepVModel) */
    fract16_t LpfCoef;          /* dt / (τ + dt) */

    /* The tracker */
    AnglePll_FilterTuning_T PllFilterTuning;    /* MagFloor is the |ê| floor */
    PID_Config_T Pll;                           /* loop filter, phase error → ω̂ [angle16/dt] */
}
EmfObserver_Config_T;

/*
    K_smo will need tuning per motor.
    LpfCoef: ~100 Hz at 20 kHz (k_lp = dt/(τ+dt), τ = 1/(2π·100) = 1592 µs, dt = 50 µs).
    PLL gains: normalized phase error (fract16 ±1) → ω̂ (angle16/dt). Crossover ≈ Kp·π rad/dt, at the LPF cutoff,
        the PI zero Ki / (Fs·Kp) a quarter below it. Far above the LPF, the loop loses its phase margin.
*/
#define EMF_OBSERVER_CONFIG_DEFAULT(VBus, v_max, I, i_max) (EmfObserver_Config_T)                                  \
{                                                                                                                   \
    .K_smo          = (fract16_t)FRACT16(.60F * VBus / v_max),                                                      \
    .SmoSat         = (fract16_t)FRACT16(.15F * I / i_max),         /* 15 % of IMax boundary layer */               \
    .LpfCoef        = 998,                                          /* ~100 Hz LPF at 20 kHz */                     \
    .PllFilterTuning =                                                                                              \
    {                                                                                                               \
        .MagFloor       = (ufract16_t)FRACT16(.05F * VBus / v_max), /* 5 % of VMax */                               \
        .LockErrTol     = (ufract16_t)FRACT16(.05F),                /* 5 % normalized error */                      \
        .LockHoldCount  = 200U,                                     /* 10 ms at 20 kHz */                           \
        .LossHoldCount  = 200U,                                     /* 10 ms at 20 kHz */                           \
    },                                                                                                              \
    .Pll =                                                                                                          \
    {                                                                                                               \
        .Mode       = PID_MODE_PI,                                                                                  \
        .SampleFreq = 20000U,                                                                                       \
        .Kp         = 328,                                          /* ≈ 0.01 Q17.15, crossover ≈ 100 Hz */         \
        .Ki         = 51472,                                        /* ≈ 1.57 /s Q17.15, zero ≈ 25 Hz */            \
        .Kd         = 0,                                                                                            \
    },                                                                                                              \
}


typedef struct EmfObserver_Axis
{
    Stator_Axis_T Stator;   /* the stator equation in place of a sample [Stator_Axis_Integrate] */
                            /* Stator.I the model's I: the next sample, predicted [Stator_Winding_INext] */
    fract16_t Z;            /* the correction in place of e, the raw EMF */
    fract16_t Emf;          /* ê */
}
EmfObserver_Axis_T;

typedef struct EmfObserver
{
    Stator_Winding_T Winding;
    EmfObserver_Axis_T Alpha, Beta;
    /* union optionally */
    /* VModel_Proc and Eemf_Proc, in the frame θ̂ */
    EmfObserver_Axis_T D, Q;
    AnglePll_T Pll;                     /* θ̂, ω̂, |ê|, lock */
    EmfObserver_Config_T Config;
}
EmfObserver_T;


/******************************************************************************/
/*!
    The tail, shared by the front ends: the rotor vector of ê, to the tracker.
*/
/******************************************************************************/
/* Rotation direction of the frame. ω̂ = 0 reads forward, so the detector never stalls at zero */
static inline sign_t _EmfObserver_Direction(const Angle_T * p_frame) { return (Angle_Delta(p_frame) < 0) ? SIGN_NEGATIVE : SIGN_POSITIVE; }

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
/*
    After the FOC loop, Stator.I is previous sample
    z[k]    = K_smo · sat( î[k] - i_meas[k], thr )
    î[k+1]  = î[k] + G_pu · ( v[k] - Rs_pu·î[k] - z[k] )      z in place of e
*/
/*
    One SMO axis, z first: z against the sample [smo_z], then the model across the next period, v the command just made, z in place of e [Stator_Axis_Integrate].
    The command is consumed in the call; nothing waits a tick.
*/
static inline void _EmfObserver_ProcSmoNext(EmfObserver_T * p_obs, EmfObserver_Axis_T * p_axis, fract16_t i, accum32_t v)
{
    p_axis->Z = smo_z(p_obs->Config.K_smo, p_obs->Config.SmoSat, smo_sigma(p_axis->Stator.I, i));
    p_axis->Emf = lpf_step(p_obs->Config.LpfCoef, p_axis->Emf, p_axis->Z);
    Stator_Axis_Integrate(&p_obs->Winding, &p_axis->Stator, v, p_axis->Z);
}

/*
    SMO, stationary frame. Once per control tick, after the FOC proc: the αβ sample it took and the αβ command it made.
*/
static void EmfObserver_AlphaBeta_Proc(EmfObserver_T * p_obs, fract16_t i_alpha, fract16_t i_beta, fract16_t v_alpha, fract16_t v_beta)
{
    _EmfObserver_ProcSmoNext(p_obs, &p_obs->Alpha, i_alpha, v_alpha);
    _EmfObserver_ProcSmoNext(p_obs, &p_obs->Beta, i_beta, v_beta);

    /* ê_αβ: the rotor vector, detected against θ̂ [AnglePll_ProcVector] */
    struct fract16_xy rotor = foc_rotor_of_emf(_EmfObserver_Direction(&p_obs->Pll.Base), p_obs->Alpha.Emf, p_obs->Beta.Emf);
    AnglePll_ProcVector(&p_obs->Pll, rotor.x, rotor.y);
}

/*
    Before the next FOC loop,
    î[k]  = î[k-1] + G_pu · ( v - Rs_pu·î[k-1] - z[k-1] )     z in place of e
    z[k]  = K_smo · sat( î[k] - i_meas[k], thr )
*/
/*
    One SMO axis, the model first: the model across the period the sample i closes, v held across it, z of the last tick in place of e [Stator_Axis_Integrate],
    then z against the sample [smo_z]; z is the raw EMF, ê its equivalent control.
    For a v that depends on the sample, as the frame's turn on the measured current does [EmfObserver_Eemf_Proc].
*/
static inline void _EmfObserver_ProcSmo(EmfObserver_T * p_obs, EmfObserver_Axis_T * p_axis, fract16_t i, accum32_t v)
{
    Stator_Axis_Integrate(&p_obs->Winding, &p_axis->Stator, v, p_axis->Z);
    p_axis->Z = smo_z(p_obs->Config.K_smo, p_obs->Config.SmoSat, smo_sigma(p_axis->Stator.I, i));
    p_axis->Emf = lpf_step(p_obs->Config.LpfCoef, p_axis->Emf, p_axis->Z);
}

/*
    SMO, stationary frame, the model first [_EmfObserver_ProcSmo]. Once per control tick, before the FOC proc:
    the αβ sample, and the αβ voltage held across the period it closes.
    The steps of [EmfObserver_AlphaBeta_Proc], with its integration moved to the start of the next call.
*/
static void EmfObserver_AlphaBeta_ProcHeld(EmfObserver_T * p_obs, fract16_t i_alpha, fract16_t i_beta, fract16_t v_alpha, fract16_t v_beta)
{
    _EmfObserver_ProcSmo(p_obs, &p_obs->Alpha, i_alpha, v_alpha);
    _EmfObserver_ProcSmo(p_obs, &p_obs->Beta, i_beta, v_beta);

    struct fract16_xy rotor = foc_rotor_of_emf(_EmfObserver_Direction(&p_obs->Pll.Base), p_obs->Alpha.Emf, p_obs->Beta.Emf);
    AnglePll_ProcVector(&p_obs->Pll, rotor.x, rotor.y);
}

/******************************************************************************/
/*!
    Front ends in the frame θ̂ [Angle_Value] of [EmfObserver_T.Pll] at the call, parked by the caller [foc_park]:
    i the sample at θ̂, v the voltage held across the period the sample closes, at its mean angle θ̂ − Δθ̂/2.
*/
/******************************************************************************/
/* ê_dq, already in the frame of θ̂: the rotor vector relative to it [AnglePll_ProcRelative] */
/*
    Voltage model, frame θ̂ [foc_vemf_d] [foc_vemf_q]. Steady state: i_dq is DC at lock, so L·ΔI of the axes
    becomes X·J·i, X = ω̂·L [stator_reactance], with no difference of samples, and no LPF phase lag.
*/
static void EmfObserver_VModel_Proc(EmfObserver_T * p_obs, fract16_t i_d, fract16_t i_q, fract16_t v_d, fract16_t v_q)
{
    accum32_t x = stator_reactance(p_obs->Winding.L, Angle_Delta(&p_obs->Pll.Base));

    p_obs->D.Emf = lpf_step(p_obs->Config.LpfCoef, p_obs->D.Emf, fract16_sat(foc_vemf_d(p_obs->Winding.R, x, v_d, i_d, i_q)));
    p_obs->Q.Emf = lpf_step(p_obs->Config.LpfCoef, p_obs->Q.Emf, fract16_sat(foc_vemf_q(p_obs->Winding.R, x, v_q, i_d, i_q)));

    struct fract16_xy rotor = foc_rotor_of_emf(_EmfObserver_Direction(&p_obs->Pll.Base), p_obs->D.Emf, p_obs->Q.Emf);
    AnglePll_ProcRelative(&p_obs->Pll, rotor.x, rotor.y);
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
/*
    EEMF SMO, frame θ̂. The model across the period [_EmfObserver_ProcSmo]: z of the last tick in place of e,
    and the frame's turn X·J·i on the measured current in the axis voltage, X = ω̂·L, [foc_vd_ff] [foc_vq_ff].
    Each axis is then a scalar SMO. On î, the boundary layer's î − i ≈ E·thr/K would cross through X into the other axis,
    θ̂ lagging by atan(X·thr / (K + R·thr)).
    z is DC at lock, so the LPF takes the chatter without the lag it costs in the stationary frame.
    The model goes first: its axis voltage takes the turn on the sample that closes the period.
*/
/*
    FOC_SetTheta            θ
    FOC_ProcClarkePark      Ialpha, Ibeta, Id, Iq ← i[k]
    EmfObserver_Eemf_Proc   i[k], and v[k−1] still in Vd/Vq and Valpha/Vbeta
    FOC_ProcIFeedback       Vd, Vq ← v[k]
    FOC_ProcInvClarkePark   Valpha, Vbeta, Va..Vc ← v[k]
*/
static angle16_t EmfObserver_Eemf_Proc(EmfObserver_T * p_obs, fract16_t i_d, fract16_t i_q, fract16_t v_d, fract16_t v_q)
{
    accum32_t x = stator_reactance(p_obs->Winding.L, Angle_Delta(&p_obs->Pll.Base));

    _EmfObserver_ProcSmo(p_obs, &p_obs->D, i_d, v_d - foc_vd_ff(x, i_q));
    _EmfObserver_ProcSmo(p_obs, &p_obs->Q, i_q, v_q - foc_vq_ff(x, 0, i_d));

    struct fract16_xy rotor = foc_rotor_of_emf(_EmfObserver_Direction(&p_obs->Pll.Base), p_obs->D.Emf, p_obs->Q.Emf);
    AnglePll_ProcRelative(&p_obs->Pll, rotor.x, rotor.y);
    return Angle_Value(&p_obs->Pll.Base);
}

/* Bumpless seed: θ̂ and ω̂ from a known reference, e.g. the open-loop ramp */
static inline void EmfObserver_SeedAngle(EmfObserver_T * p_obs, angle16_t theta, angle16_t delta) { AnglePll_Seed(&p_obs->Pll, theta, delta); }

static inline bool EmfObserver_IsLocked(const EmfObserver_T * p_obs) { return AnglePll_IsLocked(&p_obs->Pll); }


/******************************************************************************/
/*!
    Lifecycle
*/
/******************************************************************************/
/* Per-tick state cleared; Config and the winding held */
static void EmfObserver_Reset(EmfObserver_T * p_obs)
{
    p_obs->Alpha = (EmfObserver_Axis_T){ 0 };
    p_obs->Beta = (EmfObserver_Axis_T){ 0 };
    p_obs->D = (EmfObserver_Axis_T){ 0 };
    p_obs->Q = (EmfObserver_Axis_T){ 0 };
    AnglePll_Reset(&p_obs->Pll);
}

static void EmfObserver_Init(EmfObserver_T * p_obs, const EmfObserver_Config_T * p_config)
{
    if (p_config != NULL) { p_obs->Config = *p_config; }
    AnglePll_InitPid(&p_obs->Pll, &p_obs->Config.Pll);
    AnglePll_InitFilterTuning(&p_obs->Pll, &p_obs->Config.PllFilterTuning);
    EmfObserver_Reset(p_obs);
}

/* The plant from the motor's model at the turn base. Lq on both axes, the extended EMF: one plant for every front end. Validate checks (l_turn != 0U) */
static inline void EmfObserver_InitWinding(EmfObserver_T * p_obs, accum32_t r, uint32_t l_turn) { Stator_Winding_Init(&p_obs->Winding, r, l_turn); }


/******************************************************************************/
/*!
    Var
*/
/******************************************************************************/
typedef enum EmfObserver_Var
{
    EMF_OBSERVER_VAR_ANGLE,
    EMF_OBSERVER_VAR_DELTA,
    EMF_OBSERVER_VAR_EMF_ALPHA,
    EMF_OBSERVER_VAR_EMF_BETA,
    EMF_OBSERVER_VAR_EMF_MAG,
    EMF_OBSERVER_VAR_PLL_ERR,
}
EmfObserver_Var_T;

static inline int32_t EmfObserver_GetVar(const EmfObserver_T * p_obs, EmfObserver_Var_T var)
{
    switch (var)
    {
        case EMF_OBSERVER_VAR_ANGLE:        return Angle_Value(&p_obs->Pll.Base);
        case EMF_OBSERVER_VAR_DELTA:        return Angle_Delta(&p_obs->Pll.Base);
        case EMF_OBSERVER_VAR_EMF_ALPHA:    return p_obs->Alpha.Emf;
        case EMF_OBSERVER_VAR_EMF_BETA:     return p_obs->Beta.Emf;
        case EMF_OBSERVER_VAR_EMF_MAG:      return p_obs->Pll.Magnitude;
        case EMF_OBSERVER_VAR_PLL_ERR:      return p_obs->Pll.PhaseError;
        default: return 0;
    }
}

typedef enum EmfObserver_ConfigVar
{
    EMF_OBSERVER_CONFIG_VAR_K_SMO,
    EMF_OBSERVER_CONFIG_VAR_SMO_SAT,
    EMF_OBSERVER_CONFIG_VAR_LPF_COEF,
    EMF_OBSERVER_CONFIG_VAR_LOCK_EMF_MIN,
    EMF_OBSERVER_CONFIG_VAR_LOCK_ERR_TOL,
    EMF_OBSERVER_CONFIG_VAR_LOCK_HOLD_COUNT,
}
EmfObserver_ConfigVar_T;

static inline int32_t EmfObserver_Config_Get(const EmfObserver_Config_T * p_config, EmfObserver_ConfigVar_T var)
{
    switch (var)
    {
        case EMF_OBSERVER_CONFIG_VAR_K_SMO:             return p_config->K_smo;
        case EMF_OBSERVER_CONFIG_VAR_SMO_SAT:           return p_config->SmoSat;
        case EMF_OBSERVER_CONFIG_VAR_LPF_COEF:          return p_config->LpfCoef;
        case EMF_OBSERVER_CONFIG_VAR_LOCK_EMF_MIN:      return p_config->PllFilterTuning.MagFloor;
        case EMF_OBSERVER_CONFIG_VAR_LOCK_ERR_TOL:      return p_config->PllFilterTuning.LockErrTol;
        case EMF_OBSERVER_CONFIG_VAR_LOCK_HOLD_COUNT:   return p_config->PllFilterTuning.LockHoldCount;
        default: return 0;
    }
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