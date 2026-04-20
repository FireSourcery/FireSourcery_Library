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

    Sequence:
        ALIGN         : ramp Id to bias, rotor settles on d-axis (angle = 0)
        RS_MEASURE    : average Vd / Id at steady state, Rs = Vd/Id
        LD_INJECT     : voltage-mode square wave on Vd, fit first-order tau, Ld = Rs * tau
        LQ_HFI        : HFI sinusoid on Vq with DC Id bias, demodulate Iq, Lq = V_hf / (2*pi*fh*|Iq|)
        COMMIT        : write SI + fract16 forms to Config, recompute KLd/KLq/KPsi
        RAMPDOWN      : ramp Id back to zero, exit to parent
*/
/******************************************************************************/
#include "Motor_Calibration.h"
#include "../Motor_FOC.h"
#include "../Motor_ControlFreq.h"
#include "../Phase_Input/Phase_Calibration.h"
#include "../Phase_Input/Phase_VBus.h"

#include "Math/Fixed/fract16.h"
#include "Math/math_general.h"

#include <stdint.h>


/******************************************************************************/
/*!
    @brief  Tunable constants
*/
/******************************************************************************/
/* Durations in control cycles (20 kHz => 1 ms = 20 cycles) */
#define PARAMID_ALIGN_SETTLE_MS     (200U)  /* rotor pull-in + damping */
#define PARAMID_RS_WINDOW_MS        (100U)  /* averaging window */
#define PARAMID_LD_HALFPERIOD_MS    (2U)    /* Vd square-wave half period */
#define PARAMID_LD_SETTLE_CYCLES    (8U)    /* samples skipped at start of each half */
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
    PARAMID_STEP_COMMIT,
    PARAMID_STEP_RAMPDOWN,
    PARAMID_STEP_DONE,
}
Motor_ParamId_Step_T;


/******************************************************************************/
/*!
    @brief  Math helpers (private)
*/
/******************************************************************************/
/* Rs_Fract16 = Vd_Fract16 / Id_Fract16 (normalized by R_REF = V_MAX/I_MAX). */
static fract16_t compute_rs_fract16(int32_t vd_fract16, int32_t id_fract16)
{
    if (id_fract16 <= 0) { return 0; }
    return (fract16_t)math_clamp(fract16_div(vd_fract16, id_fract16), 0, FRACT16_MAX);
}

/* Ld [uH] from first-order tau measured in PWM cycles: Ld = Rs * tau.
   uH = (mOhm/1000 * cycles/Fs) * 1e6 = mOhm * cycles * 1000 / Fs. */
static uint16_t compute_l_uH(uint16_t rs_mOhm, uint32_t tau_cycles)
{
    uint32_t uH = ((uint32_t)rs_mOhm * tau_cycles * 1000U) / MOTOR_CONTROL_FREQ;
    if (uH > UINT16_MAX) { uH = UINT16_MAX; }
    return (uint16_t)uH;
}

/* Lq [uH] = V_hf_Volts / (2*pi*fh * I_amp_Amps) * 1e6.
   Computed as (V_hf_fract16 * V_MAX * 1e6) / (I_amp_fract16 * I_MAX * 2*pi*fh). */
static uint16_t compute_lq_uH_from_hfi(uint16_t vhf_fract16, uint32_t iq_amp_fract16, uint32_t hf_freq)
{
    if (iq_amp_fract16 == 0U || hf_freq == 0U) { return 0U; }
    uint64_t num = (uint64_t)vhf_fract16 * Phase_Calibration_GetVMaxVolts() * 1000000ULL;
    /* 2*pi approx 6283185 / 1e6. Use 628318/1e5 to keep integer math: 2*pi*fh = 628318 * fh / 1e5 */
    uint64_t den = (uint64_t)iq_amp_fract16 * Phase_Calibration_GetIMaxAmps() * 628318ULL * hf_freq / 100000ULL;
    if (den == 0U) { return 0U; }
    uint64_t uH = num / den;
    if (uH > UINT16_MAX) { uH = UINT16_MAX; }
    return (uint16_t)uH;
}

/*
    Decouple coefficients from Ld/Lq [H]:
        KL_Fract16 = pi * L * I_MAX / (V_MAX * Ts)
    where fract16_mul(delta_angle16, KL) yields omega_e * L * I_MAX / V_MAX in fract16 voltage basis.
    With L in uH, V_MAX in volts, Ts = 1/MOTOR_CONTROL_FREQ s:
        KL = pi * (uH/1e6) * I_MAX * MOTOR_CONTROL_FREQ / V_MAX
           = pi * uH * I_MAX * MOTOR_CONTROL_FREQ / (V_MAX * 1e6)
*/
static fract16_t compute_kl_fract16(uint16_t l_uH)
{
    uint32_t v_max = Phase_Calibration_GetVMaxVolts();
    uint32_t i_max = Phase_Calibration_GetIMaxAmps();
    if (v_max == 0U) { return 0; }
    /* pi approx 3141593/1e6 */
    uint64_t num = (uint64_t)3141593ULL * l_uH * i_max * MOTOR_CONTROL_FREQ;
    uint64_t den = (uint64_t)v_max * 1000000ULL * 1000000ULL;
    uint64_t k = num / den;
    if (k > FRACT16_MAX) { k = FRACT16_MAX; }
    return (fract16_t)k;
}

/*
    Psi_f [Wb] = 60 / (2*pi * Kv_RpmPerVolt * PolePairs)
    KPsi_Fract16 = pi * Psi_f / (V_MAX * Ts) = pi * Psi_f * Fs / V_MAX
*/
static fract16_t compute_kpsi_fract16(uint16_t kv_rpmPerVolt, uint8_t polePairs)
{
    uint32_t v_max = Phase_Calibration_GetVMaxVolts();
    if (kv_rpmPerVolt == 0U || polePairs == 0U || v_max == 0U) { return 0; }
    /* KPsi = pi * 60 * Fs / (2*pi * Kv * PolePairs * V_MAX) = 30 * Fs / (Kv * PolePairs * V_MAX) */
    uint64_t num = (uint64_t)30U * MOTOR_CONTROL_FREQ * 32768ULL;
    uint64_t den = (uint64_t)kv_rpmPerVolt * polePairs * v_max;
    uint64_t k = num / den;
    if (k > FRACT16_MAX) { k = FRACT16_MAX; }
    return (fract16_t)k;
}


/******************************************************************************/
/*!
    @brief  Per-step processing (runs inside the substate LOOP on every PWM tick)
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
static void paramid_proc_align(Motor_State_T * p_motor)
{
    fract16_t idReq = Motor_OpenLoopTorqueRampOf(p_motor, p_motor->ParamId.IdBias);
    Motor_FOC_AngleControl(p_motor, 0, idReq, 0);

    if (p_motor->ParamId.CycleCount >= MOTOR_CONTROL_CYCLES(PARAMID_ALIGN_SETTLE_MS))
    {
        paramid_advance(p_motor, PARAMID_STEP_RS_MEASURE);
    }
    else
    {
        p_motor->ParamId.CycleCount++;
    }
}

/* RS_MEASURE: hold current-loop at Id_bias, accumulate Vd and Id samples; Rs = Vd_avg / Id_avg. */
static void paramid_proc_rs(Motor_State_T * p_motor)
{
    Motor_FOC_AngleControl(p_motor, 0, p_motor->ParamId.IdBias, 0);

    p_motor->ParamId.VdAccum += FOC_Vd(&p_motor->Foc);
    p_motor->ParamId.IdAccum += FOC_Id(&p_motor->Foc);
    p_motor->ParamId.AccumN++;

    if (p_motor->ParamId.CycleCount >= MOTOR_CONTROL_CYCLES(PARAMID_RS_WINDOW_MS))
    {
        int32_t vd_avg = (int32_t)(p_motor->ParamId.VdAccum / (int64_t)p_motor->ParamId.AccumN);
        int32_t id_avg = (int32_t)(p_motor->ParamId.IdAccum / (int64_t)p_motor->ParamId.AccumN);

        fract16_t rs_f = compute_rs_fract16(vd_avg, id_avg);
        p_motor->ParamId.Rs_Fract16  = rs_f;
        p_motor->ParamId.Rs_MilliOhms = Phase_R_MilliOhmsOfFract16(rs_f);

        /* Precompute steady-state Vd bias so Ld step averages to Id_bias. */
        p_motor->ParamId.VdBias = (fract16_t)vd_avg;
        p_motor->ParamId.VdStep = PARAMID_LD_VSTEP_FRACT16;
        p_motor->ParamId.IdSteady = (int16_t)id_avg;
        p_motor->ParamId.IdTauCycles = 0U;

        paramid_advance(p_motor, PARAMID_STEP_LD_INJECT);
    }
    else
    {
        p_motor->ParamId.CycleCount++;
    }
}

/*
    LD_INJECT: drive voltage-mode Vd = Vbias + step, sample Id rise time from Vbias steady.
    Simple single-positive-step method:
        At t = 0: Vd := Vbias + VdStep; Id rises from IdSteady toward (Vbias+VdStep)/Rs.
        Time constant tau when Id reaches IdSteady + 0.632 * (IdTarget - IdSteady).
*/
static void paramid_proc_ld(Motor_State_T * p_motor)
{
    fract16_t vd = p_motor->ParamId.VdBias + p_motor->ParamId.VdStep;
    Motor_FOC_ProcAngleFeedforwardV(p_motor, 0, vd, 0);

    int16_t id_now = FOC_Id(&p_motor->Foc);
    if (p_motor->ParamId.CycleCount == 0U)
    {
        /* Target current = (Vbias + step) / Rs; since Vbias = Rs * IdSteady, target = IdSteady + step / Rs.
           Using normalized: delta_id = step_fract16 * R_REF / Rs_fract16_ohm = fract16_div(step, Rs_Fract16). */
        int32_t delta_id = (p_motor->ParamId.Rs_Fract16 > 0)
            ? fract16_div(p_motor->ParamId.VdStep, p_motor->ParamId.Rs_Fract16)
            : 0;
        /* 63.2% crossing threshold, scaled: 2/pi approx 0.632 ~= 20724/32768 */
        int32_t mid = p_motor->ParamId.IdSteady + (delta_id * 20724) / 32768;
        p_motor->ParamId.IdAccum = mid; /* reuse: stash target as threshold */
    }

    /* Mark first crossing of the 63% threshold */
    if ((p_motor->ParamId.IdTauCycles == 0U) && (id_now >= (int32_t)p_motor->ParamId.IdAccum))
    {
        p_motor->ParamId.IdTauCycles = (p_motor->ParamId.CycleCount == 0U) ? 1U : p_motor->ParamId.CycleCount;
    }

    p_motor->ParamId.CycleCount++;

    /* End of step window (use one half-period of the planned square wave) */
    if (p_motor->ParamId.CycleCount >= MOTOR_CONTROL_CYCLES(PARAMID_LD_HALFPERIOD_MS))
    {
        if (p_motor->ParamId.IdTauCycles == 0U)
        {
            /* Never crossed -> step too small or Rs estimate off. Fall back to window end. */
            p_motor->ParamId.IdTauCycles = p_motor->ParamId.CycleCount;
        }
        p_motor->ParamId.Ld_MicroHenries = compute_l_uH(p_motor->ParamId.Rs_MilliOhms, p_motor->ParamId.IdTauCycles);
        paramid_advance(p_motor, PARAMID_STEP_LQ_HFI);
    }
}

/*
    LQ_HFI: voltage-mode Vd = Vbias (holds Id ~ IdBias), Vq = V_hf * sin(phase).
    No current-loop interference on Vq. Sample Iq, demodulate synchronously with sin/cos.
    Lq = V_hf / (2*pi*fh * |Iq_amp|) where |Iq_amp| = 2*sqrt(I^2+Q^2)/N.
*/
static void paramid_proc_lq(Motor_State_T * p_motor)
{
    fract16_t vq_inject = fract16_mul(PARAMID_VHF_FRACT16, fract16_sin(p_motor->ParamId.HfiPhase));
    Motor_FOC_ProcAngleFeedforwardV(p_motor, 0, p_motor->ParamId.VdBias, vq_inject);

    /* Skip a couple HFI cycles at start so Iq steady-state demod window is clean */
    uint32_t warmup = MOTOR_CONTROL_FREQ / PARAMID_HF_FREQ_HZ * 2U;
    if (p_motor->ParamId.CycleCount >= warmup)
    {
        int32_t iq = FOC_Iq(&p_motor->Foc);
        p_motor->ParamId.IqSumI += (int64_t)iq * fract16_sin(p_motor->ParamId.HfiPhase);
        p_motor->ParamId.IqSumQ += (int64_t)iq * fract16_cos(p_motor->ParamId.HfiPhase);
        p_motor->ParamId.AccumN++;
    }

    p_motor->ParamId.HfiPhase += PARAMID_HFI_PHASE_DELTA;
    p_motor->ParamId.CycleCount++;

    if (p_motor->ParamId.CycleCount >= MOTOR_CONTROL_CYCLES(PARAMID_LQ_WINDOW_MS))
    {
        /* Demod magnitude in fract16: sqrt((2/N * Σ iq*sin)^2 + (2/N * Σ iq*cos)^2).
           Scale-down by 2^15 since iq and sin are both fract16. */
        uint32_t n = (p_motor->ParamId.AccumN > 0U) ? p_motor->ParamId.AccumN : 1U;
        int64_t i_norm = (p_motor->ParamId.IqSumI * 2) / (int64_t)n / 32768;
        int64_t q_norm = (p_motor->ParamId.IqSumQ * 2) / (int64_t)n / 32768;
        uint64_t mag_sq = (uint64_t)(i_norm * i_norm + q_norm * q_norm);
        uint32_t mag = 0U;
        /* integer sqrt */
        uint64_t r = 0U;
        uint64_t bit = 1ULL << 62;
        while (bit > mag_sq) { bit >>= 2; }
        while (bit != 0U)
        {
            if (mag_sq >= r + bit) { mag_sq -= r + bit; r = (r >> 1) + bit; }
            else                   { r >>= 1; }
            bit >>= 2;
        }
        mag = (uint32_t)r;

        p_motor->ParamId.Lq_MicroHenries = compute_lq_uH_from_hfi(PARAMID_VHF_FRACT16, mag, PARAMID_HF_FREQ_HZ);

        paramid_advance(p_motor, PARAMID_STEP_COMMIT);
    }
}

/* COMMIT: copy results into Motor_Config_T, recompute decoupling K's. */
static void paramid_proc_commit(Motor_State_T * p_motor)
{
    Motor_FOC_AngleControl(p_motor, 0, p_motor->ParamId.IdBias, 0); /* keep holding until rampdown */

    p_motor->Config.Rs_MilliOhms    = p_motor->ParamId.Rs_MilliOhms;
    p_motor->Config.Rs_Fract16      = p_motor->ParamId.Rs_Fract16;
    p_motor->Config.Ld_MicroHenries = p_motor->ParamId.Ld_MicroHenries;
    p_motor->Config.Lq_MicroHenries = p_motor->ParamId.Lq_MicroHenries;

#if defined(MOTOR_DECOUPLE_ENABLE)
    p_motor->Config.KLd_Fract16  = compute_kl_fract16(p_motor->ParamId.Ld_MicroHenries);
    p_motor->Config.KLq_Fract16  = compute_kl_fract16(p_motor->ParamId.Lq_MicroHenries);
    p_motor->Config.KPsi_Fract16 = compute_kpsi_fract16(p_motor->Config.Kv, p_motor->Config.PolePairs);
#endif

    paramid_advance(p_motor, PARAMID_STEP_RAMPDOWN);
}

/* RAMPDOWN: ramp Id back to zero via TorqueRamp. */
static void paramid_proc_rampdown(Motor_State_T * p_motor)
{
    fract16_t idReq = Motor_OpenLoopTorqueRampOf(p_motor, 0);
    Motor_FOC_AngleControl(p_motor, 0, idReq, 0);

    if (p_motor->ParamId.CycleCount >= MOTOR_CONTROL_CYCLES(PARAMID_RAMPDOWN_MS))
    {
        paramid_advance(p_motor, PARAMID_STEP_DONE);
    }
    else
    {
        p_motor->ParamId.CycleCount++;
    }
}


/******************************************************************************/
/*!
    @brief  Substate plumbing
*/
/******************************************************************************/
extern const State_T MOTOR_STATE_CALIBRATION;
extern const State_T CALIBRATION_STATE_ELECTRICAL;

static void Electrical_Entry(const Motor_T * p_motor)
{
    Motor_State_T * p_state = p_motor->P_MOTOR;

    Phase_ActivateT0(&p_motor->PHASE);
    p_state->FeedbackMode.Current = 1U;
    Motor_FOC_ClearFeedbackState(p_state);
    Angle_ZeroCaptureState(&p_state->OpenLoopAngle);

    p_state->ControlTimerBase = 0U;
    p_state->CalibrationStateIndex = MOTOR_CALIBRATION_STATE_ELECTRICAL;

    /* Clear full scratch, then set bias */
    p_state->ParamId = (struct Motor_ParamId){ 0 };
    p_state->ParamId.Step   = (uint8_t)PARAMID_STEP_ALIGN;
    p_state->ParamId.IdBias = Motor_GetIAlign(p_state);
    p_state->ParamId.HfFreqHz = PARAMID_HF_FREQ_HZ;
    p_state->ParamId.VhfAmpFract16 = PARAMID_VHF_FRACT16;
}

static void Electrical_Proc(const Motor_T * p_motor)
{
    Motor_State_T * p_state = p_motor->P_MOTOR;

    switch ((Motor_ParamId_Step_T)p_state->ParamId.Step)
    {
        case PARAMID_STEP_ALIGN:      paramid_proc_align(p_state);    break;
        case PARAMID_STEP_RS_MEASURE: paramid_proc_rs(p_state);       break;
        case PARAMID_STEP_LD_INJECT:  paramid_proc_ld(p_state);       break;
        case PARAMID_STEP_LQ_HFI:     paramid_proc_lq(p_state);       break;
        case PARAMID_STEP_COMMIT:     paramid_proc_commit(p_state);   break;
        case PARAMID_STEP_RAMPDOWN:   paramid_proc_rampdown(p_state); break;
        case PARAMID_STEP_DONE:
        default:
            /* Hold zero Vd/Vq until parent promotes back to CALIBRATION root */
            Motor_FOC_AngleControl(p_state, 0, 0, 0);
            break;
    }

    Motor_FOC_WriteDuty(p_motor);
}

const State_T CALIBRATION_STATE_ELECTRICAL =
{
    .P_PARENT = &MOTOR_STATE_CALIBRATION,
    .P_TOP    = &MOTOR_STATE_CALIBRATION,
    .DEPTH    = 1U,
    .ENTRY    = (State_Action_T)Electrical_Entry,
    .LOOP     = (State_Action_T)Electrical_Proc,
};


/* Transition entry */
static State_T * Electrical_Start(const Motor_T * p_motor, state_value_t value)
{
    (void)p_motor; (void)value;
    return (State_T *)&CALIBRATION_STATE_ELECTRICAL;
}

void Motor_Calibration_StartElectrical(const Motor_T * p_motor)
{
    static const StateMachine_TransitionCmd_T CMD =
    {
        .P_START = &MOTOR_STATE_CALIBRATION,
        .NEXT    = (State_Input_T)Electrical_Start,
    };
    StateMachine_Tree_InvokeTransition(&p_motor->STATE_MACHINE, &CMD, 0U);
}

bool Motor_Calibration_IsElectrical(const Motor_T * p_motor)
{
    return StateMachine_IsLeafState(p_motor->STATE_MACHINE.P_ACTIVE, &CALIBRATION_STATE_ELECTRICAL);
}
