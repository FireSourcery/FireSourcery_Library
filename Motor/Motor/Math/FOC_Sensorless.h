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

    Composes foc_sensorless_math primitives into a per-control-tick step:
        i_αβ_prev → Δi → ê_αβ (voltage-model OR SMO) → LPF → PLL → θ̂, ω̂

    Caller owns the FOC chain. Each control tick the integration layer must:
        1. Sample i_abc → foc_clarke → i_αβ.
        2. FOC_Sensorless_Step(p, i_α, i_β) — observer reads v_αβ stored last cycle.
        3. Read θ̂ via FOC_Sensorless_GetAngle(), compute sin/cos, run Park / PID /
           inv-Park as usual.
        4. After SVPWM, FOC_Sensorless_CaptureVoltage(p, v_α, v_β) — stores v
           applied this cycle for next cycle's observer.

    All gains are in the per-unit basis defined by motor_params_math.h
    (Rs_pu, Ls_pu, Psi_pu).
*/
/******************************************************************************/
#include "foc_sensorless_math.h"
#include "Math/Fixed/fract16.h"
#include "Math/Angle/Angle.h"
#include "Math/PID/PID.h"
#include "FOC.h"

#include <stdint.h>
#include <stdbool.h>


// typedef enum FOC_Sensorless_Mode
// {
//     FOC_SENSORLESS_MODE_VOLTAGE_MODEL,  /* e = v - Rs·i - Ls·di/dt */
//     FOC_SENSORLESS_MODE_SMO,            /* sliding-mode current observer */
// }
// FOC_Sensorless_Mode_T;

#ifdef FOC_SENSORLESS_ALPHA_BETA
/*
    Calibrated motor params + observer tuning. Holds no live state.
    Compute the *_pu fields with motor_params_math.h.
*/
typedef struct FOC_SensorlessConfig
{
    // FOC_Sensorless_Mode_T Mode;

    /* Motor electrical params, per-unit basis (foc_sensorless_math.h). */
    // FOC_Electrical_T ElectricalParams;
    fract16_t Rs_pu;        /* Rs · I_max / V_max */
    fract16_t Ls_pu;        /* Ls · I_max · Fs / V_max  (multiplies Δi_pu → v_pu) */
    fract16_t G_pu;     /* 1 / Ls_pu — SMO integrator gain (precomputed) */
    fract16_t Psi_pu;       /* π · Fs · ψ_f · 32768 / V_max  (psi_vfract16_per_angle16) */

    /* SMO tuning (unused in voltage-model mode). */
    fract16_t K_smo;        /* sliding gain — > peak EMF in pu */
    fract16_t SmoSat;       /* boundary-layer width for foc_smo_sat */

    /* ê smoothing — used in both modes (extracts equivalent control of z in SMO,
       and reduces measurement noise in voltage-model). */
    fract16_t LpfCoef;      /* dt / (τ + dt) */

    /* Lock detector. */
    ufract16_t LockEmfMin;  /* |ê| floor below which lock is indeterminate */
    ufract16_t LockErrTol;  /* |err| tolerance (normalised PLL error) */
    uint16_t   LockHoldCount; /* consecutive cycles within tol+floor for lock */

    /* PLL loop filter — output is ω̂ in angle16/poll units. */
    PID_Config_T PllPid;
}
FOC_SensorlessConfig_T;


typedef struct FOC_Sensorless
{
    /* Inputs from previous cycle. */
    fract16_t IAlphaPrev, IBetaPrev;
    fract16_t VAlpha, VBeta;            /* voltage applied last cycle */

    /* SMO internal state (mode == SMO). */
    fract16_t SmoIAlpha, SmoIBeta;      /* current-estimator state î */
    fract16_t SmoZAlpha, SmoZBeta;      /* raw switching variable z */

    /* Smoothed back-EMF estimate (αβ) — observer output. */
    fract16_t EmfAlpha, EmfBeta;
    ufract16_t EmfMag;
    fract16_t  PllErr;                  /* last normalised PLL phase error */

    /* Angle tracker (PLL). AngleSpeed.Angle is θ̂; AngleSpeed.Delta is ω̂ (angle16/poll). */
    Angle_T AngleSpeed;
    PID_T   PllPid;

    /* Lock detector. */
    uint16_t LockCount;
    bool     IsLocked;

    FOC_SensorlessConfig_T Config;
}
FOC_Sensorless_T;


/******************************************************************************/
/*!
    Lifecycle
*/
/******************************************************************************/
/* Reset all per-cycle state. Calibration in CONFIG is untouched. */
static void FOC_Sensorless_ResetState(FOC_Sensorless_T * p_obs)
{
    p_obs->IAlphaPrev = 0;  p_obs->IBetaPrev = 0;
    p_obs->VAlpha = 0;  p_obs->VBeta = 0;
    p_obs->SmoIAlpha = 0;  p_obs->SmoIBeta = 0;
    p_obs->SmoZAlpha = 0;  p_obs->SmoZBeta = 0;
    p_obs->EmfAlpha = 0;  p_obs->EmfBeta = 0;
    p_obs->EmfMag = 0;
    p_obs->PllErr = 0;

    Angle_ZeroCaptureState(&p_obs->AngleSpeed);
    PID_Reset(&p_obs->PllPid);

    p_obs->LockCount = 0;
    p_obs->IsLocked = false;
}

static void FOC_Sensorless_Init(FOC_Sensorless_T * p_obs)
{
    PID_InitFrom(&p_obs->PllPid, &p_obs->Config.PllPid);
    FOC_Sensorless_ResetState(p_obs);
}

/* Bumpless seed: align θ̂ and ω̂ to a known reference (e.g. open-loop ramp).
   Resets the PLL integrator so it doesn't fight the pre-seeded value. */
static void FOC_Sensorless_SeedAngle(FOC_Sensorless_T * p_obs, angle16_t theta, angle16_t delta)
{
    Angle_CaptureAngle(&p_obs->AngleSpeed, theta);
    Angle_CaptureDelta(&p_obs->AngleSpeed, delta);
    PID_Reset(&p_obs->PllPid);
    _PID_SetOutputState(&p_obs->PllPid, delta);
}


/******************************************************************************/
/*!
    Per-tick step.
*/
/******************************************************************************/
/*
    Voltage-model EMF — branch shared with SMO via the LPF stage below.
    Returns raw (pre-LPF) ê for the axis.
*/
static void FOC_Sensorless_Step(FOC_Sensorless_T * p_obs, fract16_t i_alpha, fract16_t i_beta)
{
    /* 1. Raw ê (or SMO z) from previous v_αβ and current i_αβ + Δi. */
    struct foc_alphabeta e_raw = foc_emf_alphabeta
    (
        p_obs->Config.Rs_pu, p_obs->Config.Ls_pu,
        p_obs->VAlpha, p_obs->VBeta, i_alpha, i_beta, i_alpha - p_obs->IAlphaPrev, i_beta - p_obs->IBetaPrev
    );

    //  int16_t alpha = foc_emf_axis( p_obs->Config.Rs_pu, p_obs->Config.Ls_pu, p_obs->VAlpha, p_obs->IAlphaPrev, i_alpha);
    //  int16_t beta = foc_emf_axis( p_obs->Config.Rs_pu, p_obs->Config.Ls_pu, p_obs->VBeta,  p_obs->IBetaPrev, i_beta);

    /* 2. LPF to extract equivalent control / smooth measurement noise. */
    p_obs->EmfAlpha = foc_lpf_step(p_obs->EmfAlpha, e_raw.alpha, p_obs->Config.LpfCoef);
    p_obs->EmfBeta = foc_lpf_step(p_obs->EmfBeta, e_raw.beta, p_obs->Config.LpfCoef);
    p_obs->EmfMag = fract16_vector_magnitude(p_obs->EmfAlpha, p_obs->EmfBeta);

    /* 3. PLL phase detector — speed-normalised so loop gain is |e|-invariant. */
    struct fract16_xy uv = Angle_UnitVector(&p_obs->AngleSpeed);  /* {x=cos, y=sin} */
    p_obs->PllErr = foc_pll_error_normalized(p_obs->Config.LockEmfMin, p_obs->EmfAlpha, p_obs->EmfBeta, uv.y, uv.x);

    /* 4. PID loop filter → ω̂ in angle16/poll. setpoint=err, feedback=0: PID error = err, output = Kp·err + Ki·∫err. Positive err (θ̂ lags) → +ω̂. */
    int16_t omega = PID_ProcPI(&p_obs->PllPid, 0, p_obs->PllErr);

    /* 5. Integrate ω̂ → θ̂ (free-wrap). */
    Angle_Integrate(&p_obs->AngleSpeed, (angle16_t)omega);

    /* 6. Lock detector — strong EMF AND tight PLL error, sustained. */
    bool stable = (p_obs->EmfMag > p_obs->Config.LockEmfMin) && (fract16_abs(p_obs->PllErr) < p_obs->Config.LockErrTol);

    p_obs->LockCount = stable ? (uint16_t)math_min(p_obs->LockCount + 1, p_obs->Config.LockHoldCount) : 0;
    p_obs->IsLocked = (p_obs->LockCount >= p_obs->Config.LockHoldCount);

    /* 7. Save i_αβ for next-cycle Δi. */
    p_obs->IAlphaPrev = i_alpha;
    p_obs->IBetaPrev = i_beta;
}


/*
    SMO — also updates î and z state in-place. Returns z (raw EMF candidate).
*/
// static void FOC_Sensorless_Step(const FOC_Sensorless_T * p_obs, fract16_t i_alpha, fract16_t i_beta)
// {

//     /* 1. Raw ê (or SMO z) from previous v_αβ and current i_αβ + Δi. */
//     struct foc_alphabeta e_raw = (p_obs->Config.Mode == FOC_SENSORLESS_MODE_SMO)
//         ? emf_smo(p, p_obs->Config, i_alpha, i_beta)
//         : emf_voltage_model(p, p_obs->Config, i_alpha, i_beta);

//     /* 2. LPF to extract equivalent control / smooth measurement noise. */
//     p_obs->EmfAlpha = foc_lpf_step(p_obs->EmfAlpha, e_raw.alpha, p_obs->Config.LpfCoef);
//     p_obs->EmfBeta = foc_lpf_step(p_obs->EmfBeta, e_raw.beta, p_obs->Config.LpfCoef);
//     p_obs->EmfMag = fract16_vector_magnitude(p_obs->EmfAlpha, p_obs->EmfBeta);

//     /* 3. PLL phase detector — speed-normalised so loop gain is |e|-invariant. */
//     struct fract16_xy uv = Angle_UnitVector(&p_obs->AngleSpeed);  /* {x=cos, y=sin} */
//     p_obs->PllErr = foc_pll_error_normalized(p_obs->EmfAlpha, p_obs->EmfBeta, uv.y, uv.x, p_obs->Config.LockEmfMin);

//     /* 4. PID loop filter → ω̂ in angle16/poll. setpoint=err, feedback=0:
//           PID error = err, output = Kp·err + Ki·∫err. Positive err (θ̂ lags) → +ω̂. */
//     int16_t omega = PID_ProcPI(&p_obs->PllPid, 0, p_obs->PllErr);

//     /* 5. Integrate ω̂ → θ̂ (free-wrap). */
//     Angle_Integrate(&p_obs->AngleSpeed, (angle16_t)omega);

//     /* 6. Lock detector — strong EMF AND tight PLL error, sustained. */
//     bool stable = (p_obs->EmfMag > c->LockEmfMin) && (fract16_abs(p_obs->PllErr) < c->LockErrTol);
//     p_obs->LockCount = stable ? (uint16_t)math_min(p_obs->LockCount + 1, c->LockHoldCount) : 0;
//     p_obs->IsLocked = (p_obs->LockCount >= c->LockHoldCount);

//     /* 7. Save i_αβ for next-cycle Δi. */
//     p_obs->IAlphaPrev = i_alpha;
//     p_obs->IBetaPrev = i_beta;
// }


/******************************************************************************/
/*!
    Per-control-tick step.

    Call once per PWM cycle, after Clarke on the just-sampled i_abc.
    Reads v_αβ stored by the previous CaptureVoltage call.
    Updates ê, θ̂, ω̂, and the lock state internally.
*/
/******************************************************************************/
/* Store v_αβ that was applied this cycle (post-SVPWM). Read by next Step. */
static inline void FOC_Sensorless_CaptureVoltage(FOC_Sensorless_T * p_obs, fract16_t v_alpha, fract16_t v_beta)
{
    p_obs->VAlpha = v_alpha;
    p_obs->VBeta = v_beta;
}


/******************************************************************************/
/*!
    Observer outputs
*/
/******************************************************************************/
static inline angle16_t  FOC_Sensorless_GetAngle(const FOC_Sensorless_T * p_obs)    { return Angle_Value(&p_obs->AngleSpeed); }
static inline angle16_t  FOC_Sensorless_GetDelta(const FOC_Sensorless_T * p_obs)    { return Angle_Delta(&p_obs->AngleSpeed); }
static inline fract16_t  FOC_Sensorless_GetEmfAlpha(const FOC_Sensorless_T * p_obs) { return p_obs->EmfAlpha; }
static inline fract16_t  FOC_Sensorless_GetEmfBeta(const FOC_Sensorless_T * p_obs)  { return p_obs->EmfBeta; }
static inline ufract16_t FOC_Sensorless_GetEmfMag(const FOC_Sensorless_T * p_obs)   { return p_obs->EmfMag; }
static inline bool       FOC_Sensorless_IsLocked(const FOC_Sensorless_T * p_obs)    { return p_obs->IsLocked; }

#endif


/******************************************************************************/
/*!
    DQ-frame variant — EEMF sliding-mode observer in the estimated dq frame.

    Composes foc_sensorless_math primitives:
        prev (vd, vq) + (id, iq) → EEMF SMO step → LPF → PLL → θ̂, ω̂

    Integration with the FOC chain:
        1. Sample i_abc; Park with previous θ̂ → (id, iq).
        2. FOC_Sensorless_dq_Step(p, id, iq) — observer reads (vd, vq) saved
           by previous cycle's CaptureVoltage. Updates θ̂, ω̂.
        3. Read θ̂ via FOC_Sensorless_dq_GetAngle(); (optional re-Park).
        4. PI current loop → (vd, vq).
        5. FOC_Sensorless_dq_CaptureVoltage(p, vd, vq) — store for next cycle.

    Signals are DC at lock — the LPF cleans noise without the bandwidth tradeoff
    of the sinusoidal αβ case. Native IPM handling (Lq on both axes, saliency
    absorbed into the EEMF disturbance). The observer is closed-loop in θ̂:
    requires reasonable startup alignment (open-loop ramp or HFI handoff).
*/
/******************************************************************************/
typedef struct FOC_SensorlessConfig
{
    /* EEMF SMO tuning. */
    fract16_t K_smo;
    fract16_t SmoSat;
    fract16_t LpfCoef;      /* dt / (τ + dt) — applied to (zd, zq) → (EmfD, EmfQ) */

    /* Lock detector — uses |Emf_dq|, which is DC at lock and ≈ ω·(ψ + (Ld−Lq)·id). */
    ufract16_t LockEmfMin;
    ufract16_t LockErrTol;
    uint16_t   LockHoldCount;

    /* PLL loop filter — output is ω̂ in angle16/poll units. */
    PID_Config_T PllPid;
    // Angle_SpeedFractCalib_T
}
FOC_SensorlessConfig_T;

/*
    Conservative starting point; K_smo and PllPid will need tuning per motor.
    G_pu is not in config — computed at runtime by FOC_Sensorless_InitG from Lq.
    LpfCoef: ~100 Hz at 20 kHz (k_lp = dt/(τ+dt), τ = 1/(2π·100) = 1592 µs, dt = 50 µs).
    PLL gains: normalised PllErr (fract16 ±1) → ω̂ (angle16/poll). Kp ≈ 0.01, Ki ≈ 0.001.
*/
#define FOC_SENSORLESS_CONFIG_DEFAULT(VBus, v_max, I, i_max) (FOC_SensorlessConfig_T)     \
{                                                                                   \
    .K_smo         = (fract16_t)FRACT16(.75F * VBus / v_max),                       \
    .SmoSat        = (fract16_t)FRACT16(.15F * I / i_max),          /* 15 % of IMax boundary layer */                       \
    .LpfCoef       = 998,                                           /* ~100 Hz LPF at 20 kHz */                             \
    .LockEmfMin    = (ufract16_t)FRACT16(.05F * VBus / v_max),      /* 5 % of VMax */                                        \
    .LockErrTol    = (ufract16_t)FRACT16(.05F),                     /* 5 % normalised PLL error */                           \
    .LockHoldCount = 200U,                                          /* 10 ms at 20 kHz */                                   \
    .PllPid =                                                                       \
    {                                                                               \
        .Mode       = PID_MODE_PI,                                                  \
        .SampleFreq = 20000U,                                                       \
        .Kp_Fixed32 = 328,      /* ≈ 0.01 Q17.15 */                                 \
        .Ki_Fixed32 = 33,       /* ≈ 0.001 Q17.15 */                                \
        .Kd_Fixed32 = 0,                                                            \
    },                                                                              \
}



typedef struct FOC_Sensorless
{
    accum32_t G_pu;     /* 1 / (Lq · I_max  / Psi_max) — EEMF integrator gain */

    /* Voltage captured at end of previous control tick (commanded vd, vq). */
    // fract16_t VdPrev, VqPrev;

    /* EEMF SMO state — current estimator î and switching variable z. */
    fract16_t SmoId, SmoIq;
    fract16_t SmoZd, SmoZq;

    /* Smoothed EEMF in estimated dq — observer output (DC at lock). */
    fract16_t EmfD, EmfQ;
    ufract16_t EmfMag;
    fract16_t  PllErr;

    /* Angle tracker. AngleSpeed.Angle = θ̂; AngleSpeed.Delta = ω̂ (angle16/poll). */
    Angle_T AngleSpeed;
    Angle_SpeedFractRef_T SpeedFractRef;
    // accum32_t Speed_Fract16;
    PID_T   PllPid;

    /* Lock detector. */
    uint16_t LockCount;
    // bool IsLocked;

    FOC_SensorlessConfig_T Config;
}
FOC_Sensorless_T;


/******************************************************************************/
/*!  Lifecycle  */
/******************************************************************************/
static void FOC_Sensorless_ResetState(FOC_Sensorless_T * p_obs)
{
    // p_obs->VdPrev = 0;  p_obs->VqPrev = 0;
    p_obs->SmoId = 0;   p_obs->SmoIq = 0;
    p_obs->SmoZd = 0;   p_obs->SmoZq = 0;
    p_obs->EmfD = 0;    p_obs->EmfQ = 0;
    p_obs->EmfMag = 0;
    p_obs->PllErr = 0;

    Angle_ZeroCaptureState(&p_obs->AngleSpeed);
    PID_Reset(&p_obs->PllPid);

    p_obs->LockCount = 0;
    // p_obs->IsLocked = false;
}

static void FOC_Sensorless_Init(FOC_Sensorless_T * p_obs, FOC_SensorlessConfig_T * p_config)
{
    if (p_config != NULL) { p_obs->Config = *p_config; }
    // p_obs->G_pu = math_min((int32_t)((uint64_t)FRACT16_PI * FRACT16_SCALE / p_foc->Electrical.Lq), FRACT16_MAX);
    PID_InitFrom(&p_obs->PllPid, &p_obs->Config.PllPid);
    FOC_Sensorless_ResetState(p_obs);
}

static void FOC_Sensorless_InitG(FOC_T * p_foc, FOC_Sensorless_T * p_obs)
{
    p_obs->G_pu = math_min(((uint64_t)FRACT16_PI * FRACT16_SCALE / p_foc->Electrical.Lq), FRACT16_MAX);
    // p_obs->G_pu = ((uint64_t)FRACT16_PI * FRACT16_SCALE / p_foc->Electrical.Lq);
}

static void FOC_Sensorless_InitAngleUnits(FOC_T * p_foc, FOC_Sensorless_T * p_obs, Angle_SpeedFractCalib_T * p_speed_calib)
{
    Angle_SpeedRef_Init_Rpm(&p_obs->SpeedFractRef, p_speed_calib->PollingFreq, p_speed_calib->SpeedMax_Rpm);
}

/* Bumpless seed: align θ̂ and ω̂ to a known reference (open-loop ramp / handoff).
   Resets PLL integrator to delta so it doesn't fight the seeded value. */
static void FOC_Sensorless_SeedAngle(FOC_Sensorless_T * p_obs, angle16_t theta, angle16_t delta)
{
    Angle_CaptureAngle(&p_obs->AngleSpeed, theta);
    Angle_CaptureDelta(&p_obs->AngleSpeed, delta);
    PID_Reset(&p_obs->PllPid);
    _PID_SetOutputState(&p_obs->PllPid, delta);
}


/******************************************************************************/
/*!  Per-tick step  */
/******************************************************************************/
/* (id, iq) come from Park applied with previous θ̂. */
static void FOC_Sensorless_Step(const FOC_T * p_foc, FOC_Sensorless_T * p_obs)
{
    /* 0. Cross-coupling: ω̂·Lq evaluated with the current ω̂ estimate. */
    int32_t omega_Lq = fract16_mul(Angle_ResolveSpeed_Fract16(&p_obs->AngleSpeed, &p_obs->SpeedFractRef), p_foc->Electrical.Lq);
    // int32_t omega_Lq = p_foc->ElectricalSpeed.OmegaLq;

    /* 1. EEMF SMO step — predicts (îd, îq) and produces switching variables (zd, zq). */
    int32_t vd_ff = foc_vd_ff(omega_Lq, p_obs->SmoIq);     /* −ω·Lq·îq */
    int32_t vq_ff = foc_vq_ff(omega_Lq, 0, p_obs->SmoId);  /* +ω·Lq·îd, ψ absorbed into EEMF */

    p_obs->SmoZd = foc_eemf_zd(p_obs->Config.K_smo, p_obs->Config.SmoSat, p_obs->SmoId, p_foc->Id);
    p_obs->SmoZq = foc_eemf_zq(p_obs->Config.K_smo, p_obs->Config.SmoSat, p_obs->SmoIq, p_foc->Iq);
    p_obs->SmoId = foc_eemf_id(p_obs->G_pu, p_foc->Electrical.Rs, p_foc->Vd, vd_ff, p_obs->SmoId, p_obs->SmoZd);
    p_obs->SmoIq = foc_eemf_iq(p_obs->G_pu, p_foc->Electrical.Rs, p_foc->Vq, vq_ff, p_obs->SmoIq, p_obs->SmoZq);

    /* 2. LPF to extract equivalent-control EMF (DC at lock). */
    p_obs->EmfD = foc_lpf_step(p_obs->Config.LpfCoef, p_obs->EmfD, p_obs->SmoZd);
    p_obs->EmfQ = foc_lpf_step(p_obs->Config.LpfCoef, p_obs->EmfQ, p_obs->SmoZq);
    p_obs->EmfMag = fract16_vector_magnitude(p_obs->EmfD, p_obs->EmfQ);

    /* 3. Phase error from EEMF — d-axis at lock = 0, normalised by |E|. */
    p_obs->PllErr = foc_eemf_dq_error_normalized(p_obs->Config.LockEmfMin, p_obs->EmfD, p_obs->EmfQ);

    /* 4. PI loop filter → ω̂ in angle16/poll. */
    int16_t omega = PID_ProcPI(&p_obs->PllPid, 0, p_obs->PllErr);

    /* 5. Integrate ω̂ → θ̂. */
    Angle_IntegrateSpeed_Fract16(&p_obs->AngleSpeed, &p_obs->SpeedFractRef, omega);

    /* 6. Lock detector — strong EMF AND tight PLL error, sustained. */
    bool stable = (p_obs->EmfMag > p_obs->Config.LockEmfMin) && (fract16_abs(p_obs->PllErr) < p_obs->Config.LockErrTol);
    p_obs->LockCount = stable ? (uint16_t)math_min(p_obs->LockCount + 1, p_obs->Config.LockHoldCount) : 0;
}

/* Store (vd, vq) commanded this cycle — read by next Step. */
// static inline void FOC_Sensorless_CaptureVoltage(FOC_Sensorless_T * p_obs, fract16_t vd, fract16_t vq)
// {
//     // p_obs->VdPrev = vd;
//     // p_obs->VqPrev = vq;
// }



/******************************************************************************/
/*!  Observer outputs  */
/******************************************************************************/
static inline angle16_t  FOC_Sensorless_GetAngle(const FOC_Sensorless_T * p_obs)  { return Angle_Value(&p_obs->AngleSpeed); }
static inline angle16_t  FOC_Sensorless_GetDelta(const FOC_Sensorless_T * p_obs)  { return Angle_Delta(&p_obs->AngleSpeed); }
static inline fract16_t  FOC_Sensorless_GetSpeed(const FOC_Sensorless_T * p_obs) { return Angle_ResolveSpeed_Fract16(&p_obs->AngleSpeed, &p_obs->SpeedFractRef); }
static inline fract16_t  FOC_Sensorless_GetEmfD(const FOC_Sensorless_T * p_obs)   { return p_obs->EmfD; }
static inline fract16_t  FOC_Sensorless_GetEmfQ(const FOC_Sensorless_T * p_obs)   { return p_obs->EmfQ; }
static inline ufract16_t FOC_Sensorless_GetEmfMag(const FOC_Sensorless_T * p_obs) { return p_obs->EmfMag; }
static inline fract16_t  FOC_Sensorless_GetPllErr(const FOC_Sensorless_T * p_obs) { return p_obs->PllErr; }
// static inline bool       FOC_Sensorless_IsLocked(const FOC_Sensorless_T * p_obs)  { return p_obs->IsLocked; }
static inline bool       FOC_Sensorless_IsLocked(const FOC_Sensorless_T * p_obs) { return  (p_obs->LockCount >= p_obs->Config.LockHoldCount); }




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
        case FOC_SENSORLESS_VAR_DELTA: return FOC_Sensorless_GetSpeed(p_obs);
        // case FOC_SENSORLESS_VAR_EMF_ALPHA: return FOC_Sensorless_GetEmfAlpha(p_obs);
        // case FOC_SENSORLESS_VAR_EMF_BETA: return FOC_Sensorless_GetEmfBeta(p_obs);
        case FOC_SENSORLESS_VAR_EMF_ALPHA: return FOC_Sensorless_GetEmfD(p_obs); /* temp */
        case FOC_SENSORLESS_VAR_EMF_BETA: return FOC_Sensorless_GetEmfQ(p_obs);
        case FOC_SENSORLESS_VAR_EMF_MAG: return FOC_Sensorless_GetEmfMag(p_obs);
        case FOC_SENSORLESS_VAR_PLL_ERR: return FOC_Sensorless_GetPllErr(p_obs);
        default: return 0;
    }
}



typedef enum FOC_SensorlessConfigVar
{
    FOC_SENSORLESS_CONFIG_VAR_RS_PU,
    FOC_SENSORLESS_CONFIG_VAR_LS_PU,
    FOC_SENSORLESS_CONFIG_VAR_G_INT_PU,
    FOC_SENSORLESS_CONFIG_VAR_PSI_PU,
    FOC_SENSORLESS_CONFIG_VAR_K_SMO,
    FOC_SENSORLESS_CONFIG_VAR_SMO_SAT,
    FOC_SENSORLESS_CONFIG_VAR_LPF_COEF,
}
FOC_SensorlessConfigVar_T;

static inline int32_t FOC_SensorlessConfig_Get(const FOC_SensorlessConfig_T * p_obs, FOC_SensorlessVar_T var)
{
    switch (var)
    {
        case FOC_SENSORLESS_CONFIG_VAR_K_SMO: return p_obs->K_smo;
        case FOC_SENSORLESS_CONFIG_VAR_SMO_SAT: return p_obs->SmoSat;
        case FOC_SENSORLESS_CONFIG_VAR_LPF_COEF: return p_obs->LpfCoef;
        default: return 0;
    }
}
