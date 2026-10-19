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

    Composes foc_sensorless_math primitives into a per-control-tick step. Two EMF front ends, one tail:
        Step         SMO, stationary frame          i_αβ, v_αβ → ê_αβ → LPF → PLL → θ̂, ω̂
        StepVModel   voltage model, estimated dq    i_dq, v_dq → ê_dq → LPF → PLL → θ̂, ω̂
    Each front end's phase detector feeds the shared [AnglePll_T].

    Caller owns the FOC chain. Each control tick the integration layer must:
        1. Run the FOC proc: i_abc → i_dq → v_dq at θ.
        2. FOC_CaptureStator — the αβ image [Stator_AlphaBeta_T]: i sampled this tick, v applied across the period it ends.
        3. FOC_Sensorless_Step or FOC_Sensorless_StepVModel(p, &p_foc->Stator).
        4. Read θ̂ via FOC_Sensorless_GetAngle().

    Per-unit at the turn base, ω_base = π · Fs
        v_pu   = V / V_base
        i_pu   = I / I_base
        Rs_pu  = Rs · I_base / V_base
        L_turn = L · ω_base · I_base / V_base
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
    /* SMO tuning, Step only. */
    fract16_t K_smo;        /* sliding gain — > peak EMF in pu */
    fract16_t SmoSat;       /* boundary-layer width for smo_sat */

    /* ê smoothing — both front ends: the equivalent control of z (Step), measurement noise on the dq residual (StepVModel). */
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
    PLL gains: normalized phase error (fract16 ±1) → ω̂ (angle16/dt). Kp ≈ 0.01, Ki ≈ 0.001.
*/
#define FOC_SENSORLESS_CONFIG_DEFAULT(VBus, v_max, I, i_max) (FOC_SensorlessConfig_T)     \
{                                                                                   \
    .K_smo         = (fract16_t)FRACT16(.60F * VBus / v_max),                       \
    .SmoSat        = (fract16_t)FRACT16(.15F * I / i_max),          /* 15 % of IMax boundary layer */                       \
    .LpfCoef       = 998,                                           /* ~100 Hz LPF at 20 kHz */                             \
    .LockEmfMin    = (ufract16_t)FRACT16(.05F * VBus / v_max),      /* 5 % of VMax */                                        \
    .LockErrTol    = (ufract16_t)FRACT16(.05F),                     /* 5 % normalized PLL error */                           \
    .LockHoldCount = 200U,                                          /* 10 ms at 20 kHz */                                   \
    .PllPid =                                                                       \
    {                                                                               \
        .Mode       = PID_MODE_PI,                                                  \
        .SampleFreq = 20000U,                                                       \
        .Kp         = 3280,      /* ≈ 0.1 Q17.15 */                                 \
        .Ki         = 330,       /* ≈ 0.01 Q17.15 */                                \
        .Kd         = 0,                                                            \
    },                                                                              \
}

// .StaLambda     = (fract16_t)FRACT16(.50F),                      /* STA √|σ| gain — re-tune per motor */
// .StaAlphaDt    = (fract16_t)FRACT16(.01F),                      /* STA integrator gain × dt */

/*
    [FOC_SensorlessElectrical_T] View of the machine model at the turn base. Derived from the caller's model, FOC_Sensorless_InitElectrical.
        G = π / Ls_turn = Ts · V_base / (Ls · I_base),  Ls = (Ld + Lq) / 2     SMO, Step
        Lq_turn                                                             voltage model, StepVModel
*/
typedef struct FOC_SensorlessElectrical
{
    accum32_t G;
    uint32_t Lq;        /* [V_base / I_base · Ts / π] */
    fract16_t Rs;       /* [V_base / I_base] */
}
FOC_SensorlessElectrical_T;

typedef struct FOC_Sensorless
{
    /* SMO internal state, Step. */
    struct smo_axis SmoAlpha, SmoBeta;  /* current-estimator state î, raw switching variable z */

    /* Smoothed back-EMF estimate — observer output. αβ from Step, dq in the estimated frame from StepVModel. */
    fract16_t EmfAlpha, EmfBeta;
    fract16_t EmfD, EmfQ;
    ufract16_t EmfMag;

    AnglePll_T Pll;                     /* θ̂, ω̂ */

    /* Lock detector. */
    uint16_t LockCount;

    FOC_SensorlessElectrical_T Electrical;  /* Derived from the caller's model */
    FOC_SensorlessConfig_T Config;
}
FOC_Sensorless_T;


/******************************************************************************/
/*!
    Per-tick step, over the αβ image [Stator_AlphaBeta_T]: i sampled this tick, v applied across the period it ends.
*/
/******************************************************************************/
/*
    PLL and lock detector, shared by the front ends. The caller sets EmfMag.
*/
static void _FOC_Sensorless_ProcPll(FOC_Sensorless_T * p_obs, fract16_t pllErr)
{
    AnglePll_Proc(&p_obs->Pll, pllErr);

    /* Lock detector — strong EMF AND tight PLL error, sustained. */
    bool stable = (p_obs->EmfMag > p_obs->Config.LockEmfMin) && (fract16_abs(pllErr) < p_obs->Config.LockErrTol);
    p_obs->LockCount = stable ? (uint16_t)math_min(p_obs->LockCount + 1, p_obs->Config.LockHoldCount) : 0;
}

/* î across the period, then z against the sample that ends it, [smo_axis_step] */
static inline struct smo_axis _FOC_Sensorless_SmoStep(const FOC_Sensorless_T * p_obs, struct smo_axis smo, const Stator_Axis_T * p_axis)
{
    return smo_axis_step(p_obs->Config.K_smo, p_obs->Config.SmoSat, p_obs->Electrical.G, p_obs->Electrical.Rs, p_axis->V, smo.i_est, smo.z, p_axis->I);
}

/*
    SMO, stationary frame. Updates î and z in place; z is the raw EMF.
*/
static void FOC_Sensorless_Step(FOC_Sensorless_T * p_obs, const Stator_AlphaBeta_T * p_stator)
{
    /* 1. SMO î and z per axis. */
    p_obs->SmoAlpha = _FOC_Sensorless_SmoStep(p_obs, p_obs->SmoAlpha, &p_stator->Alpha);
    p_obs->SmoBeta = _FOC_Sensorless_SmoStep(p_obs, p_obs->SmoBeta, &p_stator->Beta);

    /* 2. LPF to extract equivalent control / smooth measurement noise. */
    p_obs->EmfAlpha = lpf_step(p_obs->Config.LpfCoef, p_obs->EmfAlpha, p_obs->SmoAlpha.z);
    p_obs->EmfBeta = lpf_step(p_obs->Config.LpfCoef, p_obs->EmfBeta, p_obs->SmoBeta.z);
    p_obs->EmfMag = fract16_vector_magnitude(p_obs->EmfAlpha, p_obs->EmfBeta);

    /* 3. PLL phase detector — speed-normalized so loop gain is |e|-invariant. */
    struct fract16_xy vec = Angle_UnitVector(&p_obs->Pll.Base);  /* {x=cos, y=sin} */
    _FOC_Sensorless_ProcPll(p_obs, foc_pll_error_normalized(p_obs->Config.LockEmfMin, p_obs->EmfAlpha, p_obs->EmfBeta, vec.y, vec.x));
}

/* Rotation direction of θ̂. ω̂ = 0 reads forward, so the phase detector never stalls at zero */
static inline sign_t _FOC_Sensorless_Direction(const FOC_Sensorless_T * p_obs) { return (Angle_Delta(&p_obs->Pll.Base) < 0) ? SIGN_NEGATIVE : SIGN_POSITIVE; }

/*
    Voltage model, estimated dq frame θ̂. Steady state: i_dq is DC at lock, so no di/dt and no LPF phase lag.
    i_αβ is the sample at θ̂. v_αβ was held over the last period, its dq image at the midpoint θ̂ − Δθ̂/2.
*/
static void FOC_Sensorless_StepVModel(FOC_Sensorless_T * p_obs, const Stator_AlphaBeta_T * p_stator)
{
    struct fract16_xy unit_i = Angle_UnitVector(&p_obs->Pll.Base);
    struct fract16_xy unit_v = fract16_vector(Angle_Value(&p_obs->Pll.Base) - Angle_Delta(&p_obs->Pll.Base) / 2);
    struct foc_dq i = foc_park(p_stator->Alpha.I, p_stator->Beta.I, unit_i.y, unit_i.x);
    struct foc_dq v = foc_park(p_stator->Alpha.V, p_stator->Beta.V, unit_v.y, unit_v.x);
    accum32_t omega_Lq = accum32_mul(p_obs->Electrical.Lq, Angle_Delta(&p_obs->Pll.Base));

    p_obs->EmfD = lpf_step(p_obs->Config.LpfCoef, p_obs->EmfD, fract16_sat(foc_vemf_d(p_obs->Electrical.Rs, omega_Lq, v.d, i.d, i.q)));
    p_obs->EmfQ = lpf_step(p_obs->Config.LpfCoef, p_obs->EmfQ, fract16_sat(foc_vemf_q(p_obs->Electrical.Rs, omega_Lq, v.q, i.d, i.q)));
    p_obs->EmfMag = fract16_vector_magnitude(p_obs->EmfD, p_obs->EmfQ);

    /* Already in the estimated frame: ê_d is the phase detector, no rotation */
    _FOC_Sensorless_ProcPll(p_obs, foc_eemf_dq_error_normalized(_FOC_Sensorless_Direction(p_obs), p_obs->Config.LockEmfMin, p_obs->EmfD, p_obs->EmfQ));
}

/******************************************************************************/
/*!
    Per-control-tick step.

    Call once per PWM cycle, after the αβ image captured this tick's sample.
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
/* Reset all per-cycle state. Calibration in CONFIG is untouched. */
static void FOC_Sensorless_ResetState(FOC_Sensorless_T * p_obs)
{
    p_obs->SmoAlpha = (struct smo_axis) { 0 };  p_obs->SmoBeta = (struct smo_axis) { 0 };
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
    The plant from the caller's model, turn base. Ls = (Ld + Lq) / 2.
    G = Ts · V_base / (Ls · I_base) is the SMO discretization, derived here. Ls = 0 leaves the SMO inert, G = 0.

    Validate checks (ls_turn != 0U)
*/
static void FOC_Sensorless_InitElectrical(FOC_Sensorless_T * p_obs, uint32_t ls_turn, uint32_t lq_turn, fract16_t rs)
{
    p_obs->Electrical = (FOC_SensorlessElectrical_T) { .G = g_l_pu_turn(ls_turn), .Lq = lq_turn, .Rs = rs, };
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

/******************************************************************************/
/*!  Per-tick step  */
/******************************************************************************/
/* (id, iq) come from Park applied with previous θ̂. */
// static void FOC_Sensorless_Step( FOC_Sensorless_T * p_obs)
// {
//     /* 0. Cross-coupling: ω̂·Lq evaluated with the current ω̂ estimate. */
//     int32_t omega_Lq = fract16_mul(Angle_ResolveSpeed_Fract16(&p_obs->AngleSpeed, &p_obs->SpeedFractRef), p_foc->Electrical.Lq);
//     // int32_t omega_Lq = p_foc->ElectricalSpeed.OmegaLq;

//     /* 1. EEMF SMO step — predicts (îd, îq) and produces switching variables (zd, zq). */
//     int32_t vd_ff = foc_vd_ff(omega_Lq, p_obs->SmoIq);     /* −ω·Lq·îq */
//     int32_t vq_ff = foc_vq_ff(omega_Lq, 0, p_obs->SmoId);  /* +ω·Lq·îd, ψ absorbed into EEMF */

//     p_obs->SmoZd = foc_eemf_zd(p_obs->Config.K_smo, p_obs->Config.SmoSat, p_obs->SmoId, p_foc->Id);
//     p_obs->SmoZq = foc_eemf_zq(p_obs->Config.K_smo, p_obs->Config.SmoSat, p_obs->SmoIq, p_foc->Iq);
//     p_obs->SmoId = foc_eemf_id(p_obs->G_pu, p_foc->Electrical.Rs, p_foc->Vd, vd_ff, p_obs->SmoId, p_obs->SmoZd);
//     p_obs->SmoIq = foc_eemf_iq(p_obs->G_pu, p_foc->Electrical.Rs, p_foc->Vq, vq_ff, p_obs->SmoIq, p_obs->SmoZq);

//     /* 2. LPF to extract equivalent-control EMF (DC at lock). */
//     p_obs->EmfD = foc_lpf_step(p_obs->Config.LpfCoef, p_obs->EmfD, p_obs->SmoZd);
//     p_obs->EmfQ = foc_lpf_step(p_obs->Config.LpfCoef, p_obs->EmfQ, p_obs->SmoZq);
//     p_obs->EmfMag = fract16_vector_magnitude(p_obs->EmfD, p_obs->EmfQ);

//     /* 3. Phase error from EEMF — d-axis at lock = 0, normalised by |E|. */
//     p_obs->PllErr = foc_eemf_dq_error_normalized(p_obs->Config.LockEmfMin, p_obs->EmfD, p_obs->EmfQ);

//     /* 4. PI loop filter → ω̂ in angle16/poll. */
//     int16_t omega = PID_ProcPI(&p_obs->PllPid, p_obs->PllErr, 0);

//     /* 5. Integrate ω̂ → θ̂. */
//     Angle_IntegrateSpeed_Fract16(&p_obs->AngleSpeed, &p_obs->SpeedFractRef, omega);

//     /* 6. Lock detector — strong EMF AND tight PLL error, sustained. */
//     bool stable = (p_obs->EmfMag > p_obs->Config.LockEmfMin) && (fract16_abs(p_obs->PllErr) < p_obs->Config.LockErrTol);
//     p_obs->LockCount = stable ? (uint16_t)math_min(p_obs->LockCount + 1, p_obs->Config.LockHoldCount) : 0;
// }



/******************************************************************************/
/*!
    Per-tick step — Super-Twisting variant.

    Same plant model and cross-coupling decoupling as FOC_Sensorless_Step, but
    z is produced by foc_sta_axis_step. The integrator state (Wd, Wq) is the
    continuous EMF estimate and is consumed directly as (EmfD, EmfQ) — no LPF.

    Pre-conditions identical to the classical Step: (Id, Iq) from Park with
    previous θ̂; (Vd, Vq) commanded last cycle.
*/
/******************************************************************************/
// static void FOC_Sensorless_StaStep(const FOC_T * p_foc, FOC_Sensorless_T * p_obs)
// {
//     /* 0. Cross-coupling: ω̂·Lq evaluated with the current ω̂ estimate. */
//     int32_t omega_Lq = fract16_mul(Angle_ResolveSpeed_Fract16(&p_obs->AngleSpeed, &p_obs->SpeedFractRef), p_foc->Electrical.Lq);

//     /* 1. EEMF feedforward — same EEMF form as classical SMO. */
//     int32_t vd_ff = foc_vd_ff(omega_Lq, p_obs->SmoIq);     /* −ω·Lq·îq */
//     int32_t vq_ff = foc_vq_ff(omega_Lq, 0, p_obs->SmoId);  /* +ω·Lq·îd, ψ absorbed into EEMF */

//     /* 2. STA per axis — continuous z and integrator state w. */
//     struct foc_sta_axis d = foc_sta_axis_step(p_obs->Config.StaLambda, p_obs->Config.StaAlphaDt, p_obs->Wd, p_obs->SmoId, p_foc->Id);
//     struct foc_sta_axis q = foc_sta_axis_step(p_obs->Config.StaLambda, p_obs->Config.StaAlphaDt, p_obs->Wq, p_obs->SmoIq, p_foc->Iq);
//     p_obs->SmoZd = d.z;  p_obs->Wd = d.w;
//     p_obs->SmoZq = q.z;  p_obs->Wq = q.w;

//     /* 3. Current observer update — same primitive as classical, only z source differs. */
//     p_obs->SmoId = foc_eemf_id(p_obs->G_pu, p_foc->Electrical.Rs, p_foc->Vd, vd_ff, p_obs->SmoId, p_obs->SmoZd);
//     p_obs->SmoIq = foc_eemf_iq(p_obs->G_pu, p_foc->Electrical.Rs, p_foc->Vq, vq_ff, p_obs->SmoIq, p_obs->SmoZq);

//     /* 4. EMF estimate = STA integrator state (continuous; no LPF). */
//     p_obs->EmfD = p_obs->Wd;
//     p_obs->EmfQ = p_obs->Wq;
//     p_obs->EmfMag = fract16_vector_magnitude(p_obs->EmfD, p_obs->EmfQ);

//     /* 5. Phase error → PLL → θ̂. */
//     p_obs->PllErr = foc_eemf_dq_error_normalized(p_obs->Config.LockEmfMin, p_obs->EmfD, p_obs->EmfQ);
//     int16_t omega = PID_ProcPI(&p_obs->PllPid, p_obs->PllErr, 0);
//     Angle_IntegrateSpeed_Fract16(&p_obs->AngleSpeed, &p_obs->SpeedFractRef, omega);

//     /* 6. Lock detector. */
//     bool stable = (p_obs->EmfMag > p_obs->Config.LockEmfMin) && (fract16_abs(p_obs->PllErr) < p_obs->Config.LockErrTol);
//     p_obs->LockCount = stable ? (uint16_t)math_min(p_obs->LockCount + 1, p_obs->Config.LockHoldCount) : 0;
// }

