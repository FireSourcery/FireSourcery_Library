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
        1. Sample i_abc → foc_clarke → i_αβ.
        2. FOC_Sensorless_Step or FOC_Sensorless_StepVModel(p, i_α, i_β) — observer reads v_αβ stored last cycle.
        3. Read θ̂ via FOC_Sensorless_GetAngle(), compute sin/cos, run Park / PID / inv-Park as usual.
        4. After SVPWM, FOC_Sensorless_CaptureVoltage(p, v_α, v_β) — stores v applied this cycle for next cycle's observer.

    Per-unit at the turn base, ω_base = π · Fs
        v_pu   = V / V_base
        i_pu   = I / I_base
        Rs_pu  = Rs · I_base / V_base
        L_turn = L · ω_base · I_base / V_base
*/
/******************************************************************************/
#include "foc_sensorless_math.h"
#include "smo_math.h"
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
    fract16_t SmoSat;       /* boundary-layer width for foc_smo_sat */

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
    /* Input from previous cycle. */
    fract16_t VAlpha, VBeta;            /* voltage applied last cycle */

    /* SMO internal state, Step. */
    fract16_t SmoIAlpha, SmoIBeta;      /* current-estimator state î */
    fract16_t SmoZAlpha, SmoZBeta;      /* raw switching variable z */

    /* Smoothed back-EMF estimate — observer output. αβ from Step, dq in the estimated frame from StepVModel. */
    fract16_t EmfAlpha, EmfBeta;
    fract16_t EmfD, EmfQ;
    ufract16_t EmfMag;

    AnglePll_T Pll;                     /* θ̂, ω̂ */
    // Angle_SpeedPuRef_T SpeedPuRef;

    /* Lock detector. */
    uint16_t LockCount;

    FOC_SensorlessElectrical_T Electrical;  /* Derived from the caller's model */
    FOC_SensorlessConfig_T Config;
}
FOC_Sensorless_T;


/******************************************************************************/
/*!
    Per-tick step.
    i_αβ sampled this tick, v_αβ from the last CaptureVoltage.
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

/*
    SMO, stationary frame. Updates î and z in place; z is the raw EMF.
*/
static void FOC_Sensorless_Step(FOC_Sensorless_T * p_obs, fract16_t i_alpha, fract16_t i_beta)
{
    /* 1. SMO z and î from v_αβ applied last cycle and i_αβ sampled now. */
    p_obs->SmoZAlpha = smo_z(p_obs->Config.K_smo, p_obs->Config.SmoSat, p_obs->SmoIAlpha, i_alpha);
    p_obs->SmoZBeta = smo_z(p_obs->Config.K_smo, p_obs->Config.SmoSat, p_obs->SmoIBeta, i_beta);
    p_obs->SmoIAlpha = smo_i(p_obs->Electrical.G, p_obs->Electrical.Rs, p_obs->VAlpha, p_obs->SmoIAlpha, p_obs->SmoZAlpha);
    p_obs->SmoIBeta = smo_i(p_obs->Electrical.G, p_obs->Electrical.Rs, p_obs->VBeta, p_obs->SmoIBeta, p_obs->SmoZBeta);

    /* 2. LPF to extract equivalent control / smooth measurement noise. */
    p_obs->EmfAlpha = lpf_step(p_obs->Config.LpfCoef, p_obs->EmfAlpha, p_obs->SmoZAlpha);
    p_obs->EmfBeta = lpf_step(p_obs->Config.LpfCoef, p_obs->EmfBeta, p_obs->SmoZBeta);
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
static void FOC_Sensorless_StepVModel(FOC_Sensorless_T * p_obs, fract16_t i_alpha, fract16_t i_beta)
{
    struct fract16_xy unit_i = Angle_UnitVector(&p_obs->Pll.Base);
    struct fract16_xy unit_v = fract16_vector(Angle_Value(&p_obs->Pll.Base) - Angle_Delta(&p_obs->Pll.Base) / 2);
    struct foc_dq i = foc_park_vector(i_alpha, i_beta, unit_i.y, unit_i.x);
    struct foc_dq v = foc_park_vector(p_obs->VAlpha, p_obs->VBeta, unit_v.y, unit_v.x);
    accum32_t omega_Lq = accum32_mul(p_obs->Electrical.Lq, Angle_Delta(&p_obs->Pll.Base));

    p_obs->EmfD = lpf_step(p_obs->Config.LpfCoef, p_obs->EmfD, fract16_sat(foc_dq_emf_d(p_obs->Electrical.Rs, omega_Lq, v.d, i.d, i.q)));
    p_obs->EmfQ = lpf_step(p_obs->Config.LpfCoef, p_obs->EmfQ, fract16_sat(foc_dq_emf_q(p_obs->Electrical.Rs, omega_Lq, v.q, i.d, i.q)));
    p_obs->EmfMag = fract16_vector_magnitude(p_obs->EmfD, p_obs->EmfQ);

    /* Already in the estimated frame: ê_d is the phase detector, no rotation */
    _FOC_Sensorless_ProcPll(p_obs, foc_eemf_dq_error_normalized(_FOC_Sensorless_Direction(p_obs), p_obs->Config.LockEmfMin, p_obs->EmfD, p_obs->EmfQ));
}

/******************************************************************************/
/*!
    Per-control-tick step.

    Call once per PWM cycle, after Clarke on the just-sampled i_abc.
    Reads v_αβ stored by the previous CaptureVoltage call.
    Updates ê, θ̂, ω̂, and the lock state internally.
*/
/******************************************************************************/
/* Store v_αβ that was applied this cycle (post-SVPWM). Read by next Step. */
static inline void FOC_Sensorless_CaptureVoltage(FOC_Sensorless_T * p_obs, fract16_t valpha, fract16_t vbeta)
{
    p_obs->VAlpha = valpha;
    p_obs->VBeta = vbeta;
}

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
    p_obs->VAlpha = 0;  p_obs->VBeta = 0;
    p_obs->SmoIAlpha = 0;  p_obs->SmoIBeta = 0;
    p_obs->SmoZAlpha = 0;  p_obs->SmoZBeta = 0;
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
    p_obs->Electrical = (FOC_SensorlessElectrical_T) { .G = foc_g_pu_of_l_turn(ls_turn), .Lq = lq_turn, .Rs = rs, };
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