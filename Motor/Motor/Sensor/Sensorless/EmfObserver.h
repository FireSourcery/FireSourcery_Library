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

    Composes the stator image [Stator_AlphaBeta_T], the EMF estimate, and the tracker [AnglePll_T].
    Each front end estimates ê across the period, and the tracker detects the rotor vector of ê [foc_rotor_of_emf]
    against θ̂, sin(θ̂ − θ), and locks on it:
        StepSmo         SMO, stationary frame       ê_αβ, LPF
        StepVModel      voltage model, frame θ̂      ê_dq, LPF
        StepEemf        SMO, frame θ̂                ê_dq, LPF
    The dq front ends estimate in the frame of θ̂: closed-loop in θ̂ through the tracker.

    Each control tick, after the FOC proc, the caller captures the period [EmfObserver_Capture], then steps one front end.
*/
/******************************************************************************/
#include "../../Math/foc_sensorless_math.h"
#include "../../Model/Stator.h"
#include "Math/Angle/Angle.h"
#include "Math/Angle/AnglePll.h"
#include "Math/PID/PID.h"
#include "Math/Fixed/fract16.h"

#include <stdint.h>
#include <stdbool.h>


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

typedef struct EmfObserver
{
    /* The winding, from the motor's model, and the period the caller captures */
    Stator_AlphaBeta_T Stator;

    /* The model's axes beside the measured ones, z in place of e [Stator_Axis_Integrate]. Stationary for StepSmo; in the frame θ̂ for StepEemf, its turn in the axis voltage */
    Stator_Axis_T ModelAlpha, ModelBeta;
    Stator_Axis_T ModelD, ModelQ;
    fract16_t ZAlpha, ZBeta;            /* the correction, the raw EMF */
    fract16_t Zd, Zq;

    /* ê, smoothed. αβ from StepSmo, in the frame θ̂ from the dq front ends */
    fract16_t EmfAlpha, EmfBeta;
    fract16_t EmfD, EmfQ;

    AnglePll_T Pll;                     /* θ̂, ω̂, |ê|, lock */

    EmfObserver_Config_T Config;
}
EmfObserver_T;


/******************************************************************************/
/*!
    The period, once per control tick: the sample that closes it, and the command made on it, held across the next.
*/
/******************************************************************************/
static inline void EmfObserver_Capture(EmfObserver_T * p_obs, fract16_t i_alpha, fract16_t i_beta, fract16_t v_alpha, fract16_t v_beta)
{
    Stator_AlphaBeta_Capture(&p_obs->Stator, i_alpha, i_beta, v_alpha, v_beta);
}


/******************************************************************************/
/*!
    The tail, shared by the front ends: the rotor vector of ê, to the tracker.
*/
/******************************************************************************/
/* Rotation direction of the frame. ω̂ = 0 reads forward, so the detector never stalls at zero */
static inline sign_t _EmfObserver_Direction(const Angle_T * p_frame) { return (Angle_Delta(p_frame) < 0) ? SIGN_NEGATIVE : SIGN_POSITIVE; }

/* ê_αβ: the rotor vector, detected against θ̂ [AnglePll_ProcVector] */
static void _EmfObserver_ProcPllAlphaBeta(EmfObserver_T * p_obs)
{
    struct fract16_xy rotor = foc_rotor_of_emf(_EmfObserver_Direction(&p_obs->Pll.Base), p_obs->EmfAlpha, p_obs->EmfBeta);
    AnglePll_ProcVector(&p_obs->Pll, rotor.x, rotor.y);
}

/* ê_dq, already in the frame of θ̂: the rotor vector relative to it [AnglePll_ProcRelative] */
static void _EmfObserver_ProcPllDq(EmfObserver_T * p_obs)
{
    struct fract16_xy rotor = foc_rotor_of_emf(_EmfObserver_Direction(&p_obs->Pll.Base), p_obs->EmfD, p_obs->EmfQ);
    AnglePll_ProcRelative(&p_obs->Pll, rotor.x, rotor.y);
}

/******************************************************************************/
/*!
    Front ends. Once per control tick, after the capture; one front end per observer.
    ê across the captured period, then θ̂ tracks its own error against the rotor vector of ê.
*/
/******************************************************************************/
/*
    SMO, stationary frame. The model across the period [Stator_Axis_Integrate], then z against the sample that ends it [smo_z]; z is the raw EMF.
*/
static void EmfObserver_StepSmo(EmfObserver_T * p_obs)
{
    Stator_Axis_Integrate(&p_obs->Stator.Winding, &p_obs->ModelAlpha, p_obs->Stator.Alpha.V, p_obs->ZAlpha);
    Stator_Axis_Integrate(&p_obs->Stator.Winding, &p_obs->ModelBeta, p_obs->Stator.Beta.V, p_obs->ZBeta);
    p_obs->ZAlpha = smo_z(p_obs->Config.K_smo, p_obs->Config.SmoSat, smo_sigma(p_obs->ModelAlpha.I, p_obs->Stator.Alpha.I));
    p_obs->ZBeta = smo_z(p_obs->Config.K_smo, p_obs->Config.SmoSat, smo_sigma(p_obs->ModelBeta.I, p_obs->Stator.Beta.I));

    p_obs->EmfAlpha = lpf_step(p_obs->Config.LpfCoef, p_obs->EmfAlpha, p_obs->ZAlpha);
    p_obs->EmfBeta = lpf_step(p_obs->Config.LpfCoef, p_obs->EmfBeta, p_obs->ZBeta);

    _EmfObserver_ProcPllAlphaBeta(p_obs);
}

/*
    Voltage model, frame θ̂ [foc_vemf_d] [foc_vemf_q]. Steady state: i_dq is DC at lock, so L·ΔI of the axes
    becomes X·J·i, X = ω̂·L [stator_reactance], with no difference of samples, and no LPF phase lag.
*/
static void EmfObserver_StepVModel(EmfObserver_T * p_obs)
{
    Stator_AlphaBeta_ProcPark(&p_obs->Stator, Angle_Value(&p_obs->Pll.Base), Angle_Delta(&p_obs->Pll.Base));
    accum32_t x = stator_reactance(p_obs->Stator.Winding.L, Angle_Delta(&p_obs->Pll.Base));

    p_obs->EmfD = lpf_step(p_obs->Config.LpfCoef, p_obs->EmfD, fract16_sat(foc_vemf_d(p_obs->Stator.Winding.R, x, p_obs->Stator.Vd, p_obs->Stator.Id, p_obs->Stator.Iq)));
    p_obs->EmfQ = lpf_step(p_obs->Config.LpfCoef, p_obs->EmfQ, fract16_sat(foc_vemf_q(p_obs->Stator.Winding.R, x, p_obs->Stator.Vq, p_obs->Stator.Id, p_obs->Stator.Iq)));

    _EmfObserver_ProcPllDq(p_obs);
}

/*
    EEMF SMO, frame θ̂. The model across the period [Stator_Axis_Integrate]: z of the last tick in place of e,
    and the frame's turn X·J·i on the measured current in the axis voltage, X = ω̂·L, [foc_vd_ff] [foc_vq_ff].
    Each axis is then a scalar SMO. On î, the boundary layer's î − i ≈ E·thr/K would cross through X into the other axis,
    θ̂ lagging by atan(X·thr / (K + R·thr)).
    z is DC at lock, so the LPF takes the chatter without the lag it costs in the stationary frame.
*/
static void EmfObserver_StepEemf(EmfObserver_T * p_obs)
{
    Stator_AlphaBeta_ProcPark(&p_obs->Stator, Angle_Value(&p_obs->Pll.Base), Angle_Delta(&p_obs->Pll.Base));
    accum32_t x = stator_reactance(p_obs->Stator.Winding.L, Angle_Delta(&p_obs->Pll.Base));

    Stator_Axis_Integrate(&p_obs->Stator.Winding, &p_obs->ModelD, p_obs->Stator.Vd - foc_vd_ff(x, p_obs->Stator.Iq), p_obs->Zd);
    Stator_Axis_Integrate(&p_obs->Stator.Winding, &p_obs->ModelQ, p_obs->Stator.Vq - foc_vq_ff(x, 0, p_obs->Stator.Id), p_obs->Zq);
    p_obs->Zd = smo_z(p_obs->Config.K_smo, p_obs->Config.SmoSat, smo_sigma(p_obs->ModelD.I, p_obs->Stator.Id));
    p_obs->Zq = smo_z(p_obs->Config.K_smo, p_obs->Config.SmoSat, smo_sigma(p_obs->ModelQ.I, p_obs->Stator.Iq));

    p_obs->EmfD = lpf_step(p_obs->Config.LpfCoef, p_obs->EmfD, p_obs->Zd);
    p_obs->EmfQ = lpf_step(p_obs->Config.LpfCoef, p_obs->EmfQ, p_obs->Zq);

    _EmfObserver_ProcPllDq(p_obs);
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
    Stator_AlphaBeta_ZeroCaptureState(&p_obs->Stator);
    p_obs->ModelAlpha = (Stator_Axis_T) { 0 };  p_obs->ModelBeta = (Stator_Axis_T) { 0 };
    p_obs->ModelD = (Stator_Axis_T) { 0 };  p_obs->ModelQ = (Stator_Axis_T) { 0 };
    p_obs->ZAlpha = 0;  p_obs->ZBeta = 0;  p_obs->Zd = 0;  p_obs->Zq = 0;
    p_obs->EmfAlpha = 0;  p_obs->EmfBeta = 0;  p_obs->EmfD = 0;  p_obs->EmfQ = 0;
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
static inline void EmfObserver_InitWinding(EmfObserver_T * p_obs, accum32_t r, uint32_t l_turn) { Stator_Winding_Init(&p_obs->Stator.Winding, r, l_turn); }


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
        case EMF_OBSERVER_VAR_EMF_ALPHA:    return p_obs->EmfAlpha;
        case EMF_OBSERVER_VAR_EMF_BETA:     return p_obs->EmfBeta;
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
