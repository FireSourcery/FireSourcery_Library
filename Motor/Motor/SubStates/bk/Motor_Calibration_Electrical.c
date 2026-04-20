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
    @file   Motor_Calibration_Electrical.c
    @author FireSourcery
    @brief  Self-identification of Rs, Ld, Lq at standstill.

    Three individually callable substates of MOTOR_STATE_CALIBRATION:
        CALIBRATION_STATE_RS : ALIGN -> RS_MEASURE -> RAMPDOWN.                     Writes Rs_MilliOhms, Rs_Fract16.
        CALIBRATION_STATE_LD : ALIGN -> RS_MEASURE (Vd_bias capture) -> LD_INJECT -> RAMPDOWN.
                                Writes Ld_MicroHenries (and KLd_Fract16 under MOTOR_DECOUPLE_ENABLE).
        CALIBRATION_STATE_LQ : ALIGN -> RS_MEASURE (Vd_bias capture) -> LQ_HFI -> RAMPDOWN.
                                Writes Lq_MicroHenries (and KLq_Fract16 under MOTOR_DECOUPLE_ENABLE).

    Each substate is self-contained: it captures Vd_bias internally during the brief
    steady-state averaging phase that precedes the main measurement, so LD/LQ do not
    require a prior RS run.
*/
/******************************************************************************/
#include "Motor_Calibration.h"
#include "../Motor_FOC.h"
#include "../Motor_ControlFreq.h"
#include "../Phase_Input/Phase_Calibration.h"
#include "../Phase_Input/Phase_VBus.h"

#include "Math/Fixed/fract16.h"

#include <stdint.h>


/******************************************************************************/
/*!
    @brief  Tunable constants
*/
/******************************************************************************/
/* Durations in control cycles (20 kHz => 1 ms = 20 cycles) */
#define PARAMID_ALIGN_SETTLE_MS     (200U)  /* rotor pull-in + damping */
#define PARAMID_RS_WINDOW_MS        (100U)  /* Vd/Id averaging (also used by LD/LQ to capture Vd_bias) */
#define PARAMID_LD_WINDOW_MS        (2U)    /* Vd step measurement */
#define PARAMID_LQ_WINDOW_MS        (32U)   /* HFI integration window */
#define PARAMID_RAMPDOWN_MS         (100U)

#define PARAMID_HF_FREQ_HZ          (1000U) /* HFI frequency */
#define PARAMID_VHF_FRACT16         (3277)  /* ~10% of V_MAX injection amplitude */
#define PARAMID_LD_VSTEP_FRACT16    (1638)  /* ~5% of V_MAX step amplitude */

/* Phase advance per PWM cycle for HFI sinusoid = fh * 2^16 / Fs (angle16) */
#define PARAMID_HFI_PHASE_DELTA     (angle16_t)((uint32_t)PARAMID_HF_FREQ_HZ * 65536U / MOTOR_CONTROL_FREQ)


typedef enum Motor_ParamId_Step
{
    PARAMID_STEP_ALIGN = 0,
    PARAMID_STEP_RS_MEASURE,
    PARAMID_STEP_LD_INJECT,
    PARAMID_STEP_LQ_HFI,
    PARAMID_STEP_RAMPDOWN,
    PARAMID_STEP_DONE,
}
Motor_ParamId_Step_T;


/******************************************************************************/
/*!
    @brief  Math helpers
*/
/******************************************************************************/
/* Rs_Fract16 = Vd_Fract16 / Id_Fract16 (normalized by R_REF = V_MAX/I_MAX). */
static fract16_t compute_rs_fract16(int32_t vd_fract16, int32_t id_fract16)
{
    if (id_fract16 <= 0) { return 0; }
    return (fract16_t)math_clamp(fract16_div(vd_fract16, id_fract16), 0, FRACT16_MAX);
}

/* Ld/Lq [uH] from first-order tau measured in PWM cycles: L = Rs * tau.
   uH = (mOhm/1000 * cycles/Fs) * 1e6 = mOhm * cycles * 1000 / Fs. */
static uint16_t compute_l_uH_from_tau(uint16_t rs_mOhm, uint32_t tau_cycles)
{
    uint32_t uH = ((uint32_t)rs_mOhm * tau_cycles * 1000U) / MOTOR_CONTROL_FREQ;
    if (uH > UINT16_MAX) { uH = UINT16_MAX; }
    return (uint16_t)uH;
}

/* Lq [uH] = V_hf_Volts / (2*pi*fh * I_amp_Amps) * 1e6
   = (V_hf_fract16 * V_MAX * 1e6) / (I_amp_fract16 * I_MAX * 2*pi*fh). */
static uint16_t compute_lq_uH_from_hfi(uint16_t vhf_fract16, uint32_t iq_amp_fract16, uint32_t hf_freq)
{
    if (iq_amp_fract16 == 0U || hf_freq == 0U) { return 0U; }
    uint64_t num = (uint64_t)vhf_fract16 * Phase_Calibration_GetVMaxVolts() * 1000000ULL;
    uint64_t den = (uint64_t)iq_amp_fract16 * Phase_Calibration_GetIMaxAmps() * 628318ULL * hf_freq / 100000ULL;
    if (den == 0U) { return 0U; }
    uint64_t uH = num / den;
    if (uH > UINT16_MAX) { uH = UINT16_MAX; }
    return (uint16_t)uH;
}

/*
    Decouple coefficient from inductance [uH]:
        KL_Fract16 = pi * L * I_MAX / (V_MAX * Ts) = pi * uH * I_MAX * Fs / (V_MAX * 1e6)
*/
static fract16_t compute_kl_fract16(uint16_t l_uH)
{
    uint32_t v_max = Phase_Calibration_GetVMaxVolts();
    uint32_t i_max = Phase_Calibration_GetIMaxAmps();
    if (v_max == 0U) { return 0; }
    uint64_t num = (uint64_t)3141593ULL * l_uH * i_max * MOTOR_CONTROL_FREQ;
    uint64_t den = (uint64_t)v_max * 1000000ULL * 1000000ULL;
    uint64_t k = num / den;
    if (k > FRACT16_MAX) { k = FRACT16_MAX; }
    return (fract16_t)k;
}

/* Integer sqrt for the HFI demod magnitude. */
static uint32_t integer_sqrt(uint64_t x)
{
    uint64_t r = 0U;
    uint64_t bit = 1ULL << 62;
    while (bit > x) { bit >>= 2; }
    while (bit != 0U)
    {
        if (x >= r + bit) { x -= r + bit; r = (r >> 1) + bit; }
        else              { r >>= 1; }
        bit >>= 2;
    }
    return (uint32_t)r;
}


/******************************************************************************/
/*!
    @brief  Shared per-step processors
    Each returns true when the step is complete and the caller should transition.
*/
/******************************************************************************/
static void paramid_reset_accumulators(Motor_State_T * p_motor)
{
    p_motor->ParamId.VdAccum = 0;
    p_motor->ParamId.IdAccum = 0;
    p_motor->ParamId.AccumN  = 0U;
    p_motor->ParamId.IqSumI  = 0;
    p_motor->ParamId.IqSumQ  = 0;
}

static void paramid_advance(Motor_State_T * p_motor, Motor_ParamId_Step_T next)
{
    p_motor->ParamId.Step = (uint8_t)next;
    p_motor->ParamId.CycleCount = 0U;
    paramid_reset_accumulators(p_motor);
}

/* ALIGN: ramp Id to bias at electrical angle 0, hold until rotor settles. */
static bool paramid_step_align(Motor_State_T * p_motor)
{
    fract16_t idReq = Motor_OpenLoopTorqueRampOf(p_motor, p_motor->ParamId.IdBias);
    Motor_FOC_AngleControl(p_motor, 0, idReq, 0);

    if (p_motor->ParamId.CycleCount >= MOTOR_CONTROL_CYCLES(PARAMID_ALIGN_SETTLE_MS)) { return true; }
    p_motor->ParamId.CycleCount++;
    return false;
}

/* RS_MEASURE: hold current-loop at Id_bias, accumulate Vd/Id; capture Rs interim + Vd_bias. */
static bool paramid_step_rs_measure(Motor_State_T * p_motor)
{
    Motor_FOC_AngleControl(p_motor, 0, p_motor->ParamId.IdBias, 0);

    p_motor->ParamId.VdAccum += FOC_Vd(&p_motor->Foc);
    p_motor->ParamId.IdAccum += FOC_Id(&p_motor->Foc);
    p_motor->ParamId.AccumN++;

    if (p_motor->ParamId.CycleCount >= MOTOR_CONTROL_CYCLES(PARAMID_RS_WINDOW_MS))
    {
        uint32_t n = (p_motor->ParamId.AccumN > 0U) ? p_motor->ParamId.AccumN : 1U;
        int32_t vd_avg = (int32_t)(p_motor->ParamId.VdAccum / (int64_t)n);
        int32_t id_avg = (int32_t)(p_motor->ParamId.IdAccum / (int64_t)n);

        fract16_t rs_f = compute_rs_fract16(vd_avg, id_avg);
        p_motor->ParamId.Rs_Fract16   = rs_f;
        p_motor->ParamId.Rs_MilliOhms = Phase_R_MilliOhmsOfFract16(rs_f);
        p_motor->ParamId.VdBias       = (fract16_t)vd_avg;
        p_motor->ParamId.IdSteady     = (int16_t)id_avg;
        p_motor->ParamId.VdStep       = PARAMID_LD_VSTEP_FRACT16;
        p_motor->ParamId.IdTauCycles  = 0U;
        return true;
    }
    p_motor->ParamId.CycleCount++;
    return false;
}

/*
    LD_INJECT: voltage-mode Vd = Vbias + step, measure time for Id to cross 63% of (Id_steady + step/Rs).
*/
static bool paramid_step_ld_inject(Motor_State_T * p_motor)
{
    fract16_t vd = p_motor->ParamId.VdBias + p_motor->ParamId.VdStep;
    Motor_FOC_ProcAngleFeedforwardV(p_motor, 0, vd, 0);

    int16_t id_now = FOC_Id(&p_motor->Foc);
    if (p_motor->ParamId.CycleCount == 0U)
    {
        int32_t delta_id = (p_motor->ParamId.Rs_Fract16 > 0)
            ? fract16_div(p_motor->ParamId.VdStep, p_motor->ParamId.Rs_Fract16)
            : 0;
        /* 0.632 ~= 20724/32768 */
        int32_t mid = p_motor->ParamId.IdSteady + (delta_id * 20724) / 32768;
        p_motor->ParamId.IdAccum = mid; /* reuse as threshold holder */
    }

    if ((p_motor->ParamId.IdTauCycles == 0U) && (id_now >= (int32_t)p_motor->ParamId.IdAccum))
    {
        p_motor->ParamId.IdTauCycles = (p_motor->ParamId.CycleCount == 0U) ? 1U : p_motor->ParamId.CycleCount;
    }

    p_motor->ParamId.CycleCount++;

    if (p_motor->ParamId.CycleCount >= MOTOR_CONTROL_CYCLES(PARAMID_LD_WINDOW_MS))
    {
        if (p_motor->ParamId.IdTauCycles == 0U) { p_motor->ParamId.IdTauCycles = p_motor->ParamId.CycleCount; }
        p_motor->ParamId.Ld_MicroHenries = compute_l_uH_from_tau(p_motor->ParamId.Rs_MilliOhms, p_motor->ParamId.IdTauCycles);
        return true;
    }
    return false;
}

/*
    LQ_HFI: voltage-mode Vd = Vbias, Vq = V_hf*sin(phase). Demodulate Iq synchronously.
*/
static bool paramid_step_lq_hfi(Motor_State_T * p_motor)
{
    fract16_t s = fract16_sin(p_motor->ParamId.HfiPhase);
    fract16_t c = fract16_cos(p_motor->ParamId.HfiPhase);
    fract16_t vq_inject = fract16_mul(PARAMID_VHF_FRACT16, s);
    Motor_FOC_ProcAngleFeedforwardV(p_motor, 0, p_motor->ParamId.VdBias, vq_inject);

    uint32_t warmup = (MOTOR_CONTROL_FREQ / PARAMID_HF_FREQ_HZ) * 2U;
    if (p_motor->ParamId.CycleCount >= warmup)
    {
        int32_t iq = FOC_Iq(&p_motor->Foc);
        p_motor->ParamId.IqSumI += (int64_t)iq * s;
        p_motor->ParamId.IqSumQ += (int64_t)iq * c;
        p_motor->ParamId.AccumN++;
    }

    p_motor->ParamId.HfiPhase += PARAMID_HFI_PHASE_DELTA;
    p_motor->ParamId.CycleCount++;

    if (p_motor->ParamId.CycleCount >= MOTOR_CONTROL_CYCLES(PARAMID_LQ_WINDOW_MS))
    {
        uint32_t n = (p_motor->ParamId.AccumN > 0U) ? p_motor->ParamId.AccumN : 1U;
        /* iq*sin scaled as fract16*fract16; normalize by 32768 then by N, then multiply by 2 for amplitude. */
        int64_t i_norm = (p_motor->ParamId.IqSumI * 2) / (int64_t)n / 32768;
        int64_t q_norm = (p_motor->ParamId.IqSumQ * 2) / (int64_t)n / 32768;
        uint64_t mag_sq = (uint64_t)(i_norm * i_norm + q_norm * q_norm);
        uint32_t mag = integer_sqrt(mag_sq);
        p_motor->ParamId.Lq_MicroHenries = compute_lq_uH_from_hfi(PARAMID_VHF_FRACT16, mag, PARAMID_HF_FREQ_HZ);
        return true;
    }
    return false;
}

/* RAMPDOWN: ramp Id back to zero. */
static bool paramid_step_rampdown(Motor_State_T * p_motor)
{
    fract16_t idReq = Motor_OpenLoopTorqueRampOf(p_motor, 0);
    Motor_FOC_AngleControl(p_motor, 0, idReq, 0);

    if (p_motor->ParamId.CycleCount >= MOTOR_CONTROL_CYCLES(PARAMID_RAMPDOWN_MS)) { return true; }
    p_motor->ParamId.CycleCount++;
    return false;
}

static void paramid_step_idle(Motor_State_T * p_motor)
{
    Motor_FOC_AngleControl(p_motor, 0, 0, 0);
}


/******************************************************************************/
/*!
    @brief  Shared substate entry
*/
/******************************************************************************/
extern const State_T MOTOR_STATE_CALIBRATION;

static void paramid_common_entry(const Motor_T * p_motor, Motor_Calibration_StateId_T stateIdx)
{
    Motor_State_T * p_state = p_motor->P_MOTOR;

    Phase_ActivateT0(&p_motor->PHASE);
    p_state->FeedbackMode.Current = 1U;
    Motor_FOC_ClearFeedbackState(p_state);
    Angle_ZeroCaptureState(&p_state->OpenLoopAngle);

    p_state->ControlTimerBase = 0U;
    p_state->CalibrationStateIndex = stateIdx;

    p_state->ParamId = (struct Motor_ParamId){ 0 };
    p_state->ParamId.Step   = (uint8_t)PARAMID_STEP_ALIGN;
    p_state->ParamId.IdBias = Motor_GetIAlign(p_state);
    p_state->ParamId.HfFreqHz = PARAMID_HF_FREQ_HZ;
    p_state->ParamId.VhfAmpFract16 = PARAMID_VHF_FRACT16;
}


/******************************************************************************/
/*!
    @brief  CALIBRATION_STATE_RS
    ALIGN -> RS_MEASURE (commit inline) -> RAMPDOWN -> DONE
*/
/******************************************************************************/
extern const State_T CALIBRATION_STATE_RS;

static void Rs_Entry(const Motor_T * p_motor) { paramid_common_entry(p_motor, MOTOR_CALIBRATION_STATE_RS); }

static void Rs_Proc(const Motor_T * p_motor)
{
    Motor_State_T * p_state = p_motor->P_MOTOR;

    switch ((Motor_ParamId_Step_T)p_state->ParamId.Step)
    {
        case PARAMID_STEP_ALIGN:
            if (paramid_step_align(p_state)) { paramid_advance(p_state, PARAMID_STEP_RS_MEASURE); }
            break;
        case PARAMID_STEP_RS_MEASURE:
            if (paramid_step_rs_measure(p_state))
            {
                p_state->Config.Rs_MilliOhms = p_state->ParamId.Rs_MilliOhms;
                p_state->Config.Rs_Fract16   = p_state->ParamId.Rs_Fract16;
                paramid_advance(p_state, PARAMID_STEP_RAMPDOWN);
            }
            break;
        case PARAMID_STEP_RAMPDOWN:
            if (paramid_step_rampdown(p_state)) { paramid_advance(p_state, PARAMID_STEP_DONE); }
            break;
        default:
            paramid_step_idle(p_state);
            break;
    }
    Motor_FOC_WriteDuty(p_motor);
}

const State_T CALIBRATION_STATE_RS =
{
    .P_PARENT = &MOTOR_STATE_CALIBRATION,
    .P_TOP    = &MOTOR_STATE_CALIBRATION,
    .DEPTH    = 1U,
    .ENTRY    = (State_Action_T)Rs_Entry,
    .LOOP     = (State_Action_T)Rs_Proc,
};

static State_T * Rs_Start(const Motor_T * p_motor, state_value_t v) { (void)p_motor; (void)v; return (State_T *)&CALIBRATION_STATE_RS; }

void Motor_Calibration_StartRs(const Motor_T * p_motor)
{
    static const StateMachine_TransitionCmd_T CMD = { .P_START = &MOTOR_STATE_CALIBRATION, .NEXT = (State_Input_T)Rs_Start };
    StateMachine_Tree_InvokeTransition(&p_motor->STATE_MACHINE, &CMD, 0U);
}

bool Motor_Calibration_IsRs(const Motor_T * p_motor)
{
    return StateMachine_IsLeafState(p_motor->STATE_MACHINE.P_ACTIVE, &CALIBRATION_STATE_RS);
}


/******************************************************************************/
/*!
    @brief  CALIBRATION_STATE_LD
    ALIGN -> RS_MEASURE (Vd_bias capture, interim Rs) -> LD_INJECT (commit inline) -> RAMPDOWN -> DONE
*/
/******************************************************************************/
extern const State_T CALIBRATION_STATE_LD;

static void Ld_Entry(const Motor_T * p_motor) { paramid_common_entry(p_motor, MOTOR_CALIBRATION_STATE_LD); }

static void Ld_Proc(const Motor_T * p_motor)
{
    Motor_State_T * p_state = p_motor->P_MOTOR;

    switch ((Motor_ParamId_Step_T)p_state->ParamId.Step)
    {
        case PARAMID_STEP_ALIGN:
            if (paramid_step_align(p_state)) { paramid_advance(p_state, PARAMID_STEP_RS_MEASURE); }
            break;
        case PARAMID_STEP_RS_MEASURE:
            if (paramid_step_rs_measure(p_state)) { paramid_advance(p_state, PARAMID_STEP_LD_INJECT); }
            break;
        case PARAMID_STEP_LD_INJECT:
            if (paramid_step_ld_inject(p_state))
            {
                p_state->Config.Ld_MicroHenries = p_state->ParamId.Ld_MicroHenries;
#if defined(MOTOR_DECOUPLE_ENABLE)
                p_state->Config.KLd_Fract16 = compute_kl_fract16(p_state->ParamId.Ld_MicroHenries);
#endif
                paramid_advance(p_state, PARAMID_STEP_RAMPDOWN);
            }
            break;
        case PARAMID_STEP_RAMPDOWN:
            if (paramid_step_rampdown(p_state)) { paramid_advance(p_state, PARAMID_STEP_DONE); }
            break;
        default:
            paramid_step_idle(p_state);
            break;
    }
    Motor_FOC_WriteDuty(p_motor);
}

const State_T CALIBRATION_STATE_LD =
{
    .P_PARENT = &MOTOR_STATE_CALIBRATION,
    .P_TOP    = &MOTOR_STATE_CALIBRATION,
    .DEPTH    = 1U,
    .ENTRY    = (State_Action_T)Ld_Entry,
    .LOOP     = (State_Action_T)Ld_Proc,
};

static State_T * Ld_Start(const Motor_T * p_motor, state_value_t v) { (void)p_motor; (void)v; return (State_T *)&CALIBRATION_STATE_LD; }

void Motor_Calibration_StartLd(const Motor_T * p_motor)
{
    static const StateMachine_TransitionCmd_T CMD = { .P_START = &MOTOR_STATE_CALIBRATION, .NEXT = (State_Input_T)Ld_Start };
    StateMachine_Tree_InvokeTransition(&p_motor->STATE_MACHINE, &CMD, 0U);
}

bool Motor_Calibration_IsLd(const Motor_T * p_motor)
{
    return StateMachine_IsLeafState(p_motor->STATE_MACHINE.P_ACTIVE, &CALIBRATION_STATE_LD);
}


/******************************************************************************/
/*!
    @brief  CALIBRATION_STATE_LQ
    ALIGN -> RS_MEASURE (Vd_bias capture) -> LQ_HFI (commit inline) -> RAMPDOWN -> DONE
*/
/******************************************************************************/
extern const State_T CALIBRATION_STATE_LQ;

static void Lq_Entry(const Motor_T * p_motor) { paramid_common_entry(p_motor, MOTOR_CALIBRATION_STATE_LQ); }

static void Lq_Proc(const Motor_T * p_motor)
{
    Motor_State_T * p_state = p_motor->P_MOTOR;

    switch ((Motor_ParamId_Step_T)p_state->ParamId.Step)
    {
        case PARAMID_STEP_ALIGN:
            if (paramid_step_align(p_state)) { paramid_advance(p_state, PARAMID_STEP_RS_MEASURE); }
            break;
        case PARAMID_STEP_RS_MEASURE:
            if (paramid_step_rs_measure(p_state)) { paramid_advance(p_state, PARAMID_STEP_LQ_HFI); }
            break;
        case PARAMID_STEP_LQ_HFI:
            if (paramid_step_lq_hfi(p_state))
            {
                p_state->Config.Lq_MicroHenries = p_state->ParamId.Lq_MicroHenries;
#if defined(MOTOR_DECOUPLE_ENABLE)
                p_state->Config.KLq_Fract16 = compute_kl_fract16(p_state->ParamId.Lq_MicroHenries);
#endif
                paramid_advance(p_state, PARAMID_STEP_RAMPDOWN);
            }
            break;
        case PARAMID_STEP_RAMPDOWN:
            if (paramid_step_rampdown(p_state)) { paramid_advance(p_state, PARAMID_STEP_DONE); }
            break;
        default:
            paramid_step_idle(p_state);
            break;
    }
    Motor_FOC_WriteDuty(p_motor);
}

const State_T CALIBRATION_STATE_LQ =
{
    .P_PARENT = &MOTOR_STATE_CALIBRATION,
    .P_TOP    = &MOTOR_STATE_CALIBRATION,
    .DEPTH    = 1U,
    .ENTRY    = (State_Action_T)Lq_Entry,
    .LOOP     = (State_Action_T)Lq_Proc,
};

static State_T * Lq_Start(const Motor_T * p_motor, state_value_t v) { (void)p_motor; (void)v; return (State_T *)&CALIBRATION_STATE_LQ; }

void Motor_Calibration_StartLq(const Motor_T * p_motor)
{
    static const StateMachine_TransitionCmd_T CMD = { .P_START = &MOTOR_STATE_CALIBRATION, .NEXT = (State_Input_T)Lq_Start };
    StateMachine_Tree_InvokeTransition(&p_motor->STATE_MACHINE, &CMD, 0U);
}

bool Motor_Calibration_IsLq(const Motor_T * p_motor)
{
    return StateMachine_IsLeafState(p_motor->STATE_MACHINE.P_ACTIVE, &CALIBRATION_STATE_LQ);
}
