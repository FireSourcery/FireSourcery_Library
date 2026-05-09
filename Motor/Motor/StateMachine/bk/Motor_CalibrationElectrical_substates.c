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
#include "Motor/Math/motor_params_math.h"

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
// #define PARAMID_LD_WINDOW_MS        (20U)   /* step-response observation window */
#define PARAMID_LQ_WINDOW_MS        (32U)   /* HFI integration window */
#define PARAMID_RAMPDOWN_MS         (100U)

#define PARAMID_HF_FREQ_HZ          (1000U) /* HFI frequency */
#define PARAMID_VHF_FRACT16         (3277)  /* ~10% of V_MAX injection amplitude */
#define PARAMID_LD_VSTEP_FRACT16    (1638)  /* ~5% of V_MAX step amplitude */

/* Phase advance per PWM cycle for HFI sinusoid = fh * 2^16 / Fs (angle16) */
#define PARAMID_HFI_PHASE_DELTA     (angle16_t)((uint32_t)PARAMID_HF_FREQ_HZ * ANGLE16_PER_REVOLUTION / MOTOR_CONTROL_FREQ)

/*
    Electrical parameter identification scratch. Used by CALIBRATION_STATE_ELECTRICAL.
*/
typedef struct Motor_ParamsBuffer
{
    // uint8_t  Step;              /* Motor_ParamId_Step_T */
    // uint32_t CycleCount;        /* cycles within current step */
    // fract16_t IdBias;           /* aligned Id setpoint */
    /* accumulators */
    // int64_t  VdAccum;
    // int64_t  IdAccum;
    // uint32_t AccumN;

    /* Ld step */
    fract16_t VdBias;
    fract16_t VdStep;
    int16_t   IdSteady;
    uint32_t  IdTau; /* threshold */
    uint32_t  IdTauCycles;
    /* Lq HFI */
    angle16_t HfiPhase;
    angle16_t HfiDelta;
    // int64_t   IqSumI;
    // int64_t   IqSumQ;
    uint16_t  Vhf_Fract16;
    uint16_t  HfFreqHz;
    // Motor_ElectricalParams_T Results;
}
Motor_ParamsBuffer_T;

// typedef enum Motor_ParamId_Step
// {
//     PARAMID_STEP_ALIGN = 0,
//     PARAMID_STEP_RS_MEASURE,
//     PARAMID_STEP_LD_INJECT,
//     PARAMID_STEP_LQ_HFI,
//     PARAMID_STEP_COMMIT,
//     PARAMID_STEP_RAMPDOWN,
//     PARAMID_STEP_DONE,
// }
// Motor_ParamId_Step_T;

static void Electrical_Common(Motor_T * p_motor)
{
    Motor_State_T * p_runtime = p_motor->P_MOTOR;
    Motor_ParamsBuffer_T * p_params; // temp

    p_runtime->FeedbackMode.Current = 1U;
    Motor_FOC_ClearFeedbackState(p_runtime);
    Angle_ZeroCaptureState(&p_runtime->OpenLoopAngle);
    p_runtime->ControlTimerBase = 0U;
    p_runtime->CalibrationTimer = 0U;

    // p_runtime->CalibrationStateIndex = MOTOR_CALIBRATION_STATE_ELECTRICAL;

    /* Clear full scratch, then set bias */
    *p_params = (Motor_ParamsBuffer_T){ 0 };
    p_params->HfFreqHz = PARAMID_HF_FREQ_HZ;
    p_params->Vhf_Fract16 = PARAMID_VHF_FRACT16;
}

/******************************************************************************/
/*!
    @brief  CALIBRATION_RS_STATE
    RS_MEASURE: hold current-loop at Id_bias, accumulate Vd and Id samples; Rs = Vd_avg / Id_avg.
*/
/******************************************************************************/
static void Rs_Entry(Motor_T * p_motor)
{
    Motor_State_T * p_runtime = p_motor->P_MOTOR;
    Phase_ActivateV0(&p_motor->PHASE);
    TimerT_Periodic_Init(&p_motor->CONTROL_TIMER, MOTOR_CONTROL_CYCLES(PARAMID_RS_WINDOW_MS));
    Accumulator_Init(&p_runtime->Accumulators[0]);
    Accumulator_Init(&p_runtime->Accumulators[1]);
    Electrical_Common(p_motor);
}


static void Rs_Proc(Motor_T * p_motor)
{
    Motor_State_T * p_runtime = p_motor->P_MOTOR;
    Motor_ElectricalParams_T * p_results;
    Motor_ParamsBuffer_T * p_buffer; /* temp */

    Motor_FOC_AngleAlign(p_runtime, 0); /* align to d-axis, angle = 0 */

    Accumulator_Add(&p_runtime->Accumulators[0], FOC_Vd(&p_runtime->Foc));
    Accumulator_Add(&p_runtime->Accumulators[1], FOC_Id(&p_runtime->Foc));
    // p_runtime->ParamId.AccumN++;
    p_runtime->CalibrationTimer++;
    Motor_FOC_WriteDuty(p_motor);

    // inherit by substates
    if (p_runtime->CalibrationTimer >= MOTOR_CONTROL_CYCLES(PARAMID_RS_WINDOW_MS))
    {
        int32_t vd_avg = Accumulator_Output(&p_runtime->Accumulators[0]) / (p_runtime->CalibrationTimer);
        int32_t id_avg = Accumulator_Output(&p_runtime->Accumulators[1]) / (p_runtime->CalibrationTimer);
        fract16_t rs_f = rs_fract16(vd_avg, id_avg);

        p_results->Rs_Fract16 = rs_f;
        p_results->Rs_MilliOhms = rs_mohms_of_fract16(Phase_Calibration_GetVMaxVolts(), Phase_Calibration_GetIMaxAmps(), rs_f);

        /* Store for L  */
        p_buffer->VdBias = (fract16_t)vd_avg;
        p_buffer->IdSteady = (int16_t)id_avg;
        p_buffer->VdStep = PARAMID_LD_VSTEP_FRACT16;
        p_buffer->IdTauCycles = 0U;
    }
}


static State_T * Rs_Next(Motor_T * p_motor)
{
    Motor_State_T * p_runtime = p_motor->P_MOTOR;
    Motor_ElectricalParams_T * p_results;
    Motor_ParamsBuffer_T * p_buffer; /* temp */

    if (TimerT_Poll(&p_motor->CONTROL_TIMER))
    {
        return &MOTOR_STATE_CALIBRATION;
    }

    return NULL;
}

const State_T CALIBRATION_RS_STATE =
{
    .P_PARENT = &MOTOR_STATE_CALIBRATION,
    .P_TOP    = &MOTOR_STATE_CALIBRATION,
    .DEPTH    = 1U,
    .ENTRY    = (State_Action_T)Rs_Entry,
    .LOOP     = (State_Action_T)Rs_Proc,
    .NEXT     = (State_Input0_T)Rs_Next,
};

// static State_T * RsChain_Next(Motor_T * p_motor)
// {
//     if (TimerT_Poll(&p_motor->CONTROL_TIMER)) { return &CALIBRATION_STATE_LD_CHAIN; }
//     return NULL;
// }

// const State_T CALIBRATION_RS_STATE_CHAIN =
// {
//     .P_PARENT = &CALIBRATION_RS_STATE,
//     .P_TOP    = &MOTOR_STATE_CALIBRATION,
//     .DEPTH    = 2U,
//     .NEXT     = (State_Input0_T)RsChain_Next,
// };

/*
    Mapped auto transition under ALIGN
*/
static void RsAlign_Entry(Motor_T * p_motor)
{
    TimerT_Periodic_Init(&p_motor->CONTROL_TIMER, MOTOR_CONTROL_CYCLES(PARAMID_ALIGN_SETTLE_MS));
}

static State_T * RsAlign_Next(Motor_T * p_motor)
{
    if (TimerT_Poll(&p_motor->CONTROL_TIMER)) { return &CALIBRATION_RS_STATE; }
    return NULL;
}

/* Parent State Loop handles Align */
static const State_T RS_ALIGN_START_STATE =
{
    .P_PARENT   = &CALIBRATION_STATE_ANGLE_ALIGN,
    // .P_PARENT = &CALIBRATION_RS_STATE, alternatively restart disregarding results. substate timer overrides
    .P_TOP      = &MOTOR_STATE_CALIBRATION,
    .DEPTH      = 2U,
    .ENTRY      = (State_Action_T)RsAlign_Entry,
    .NEXT       = (State_Input0_T)RsAlign_Next,
};

static State_T * Rs_Start(Motor_T * p_motor, state_value_t v) { (void)p_motor; (void)v; return (State_T *)&RS_ALIGN_START_STATE; }

void Motor_Calibration_StartRs(Motor_T * p_motor)
{
    static const StateMachine_TransitionCmd_T CMD = { .P_START = &MOTOR_STATE_CALIBRATION, .NEXT = (State_Input_T)Rs_Start };
    StateMachine_Tree_InvokeTransition(&p_motor->STATE_MACHINE, &CMD, 0U);
}

bool Motor_Calibration_IsRs(Motor_T * p_motor)
{
    return StateMachine_IsLeafState(p_motor->STATE_MACHINE.P_ACTIVE, &CALIBRATION_RS_STATE);
}

// /* Transition entry */
// static State_T * RsAlign_Cmd(Motor_T * p_motor, state_value_t value)
// {
//     (void)p_motor; (void)value;  return (State_T *)&RS_ALIGN_START_STATE;
// }

// void Motor_Calibration_StartRsAlign(Motor_T * p_motor)
// {
//     static const StateMachine_TransitionCmd_T CMD = { .P_START = &MOTOR_STATE_CALIBRATION, .NEXT = (State_Input_T)RsAlign_Cmd, };
//     StateMachine_Tree_InvokeTransition(&p_motor->STATE_MACHINE, &CMD, 0U);
// }


/******************************************************************************/
/*!
    @brief  CALIBRATION_STATE_LD
    LD_INJECT: voltage-mode Vd = Vbias + step, measure time for Id to cross 63% of (Id_steady + step/Rs).
*/
/*
    LD_INJECT: drive voltage-mode Vd = Vbias + step, sample Id rise time from Vbias steady.
    Simple single-positive-step method:
        At t = 0: Vd := Vbias + VdStep; Id rises from IdSteady toward (Vbias+VdStep)/Rs.
        Time constant tau when Id reaches IdSteady + 0.632 * (IdTarget - IdSteady).
*/
/******************************************************************************/
// uint32_t GetIdTau(Motor_ParamsBuffer_T * p_params) { return p_params->IdSteady + ((uint64_t)fract16_div(p_params->VdStep, p_params->Results.Rs_Fract16) * 20724) / 32768; }
static void Ld_Entry(Motor_T * p_motor)
{
    Motor_State_T * p_runtime = p_motor->P_MOTOR;
    Motor_ParamsBuffer_T * p_params; /* temp */
    Motor_ElectricalParams_T * p_results;
    Phase_ActivateV0(&p_motor->PHASE);
    // TimerT_Periodic_Init(&p_motor->CONTROL_TIMER, MOTOR_CONTROL_CYCLES(PARAMID_LD_WINDOW_MS));
    TimerT_Periodic_Init(&p_motor->CONTROL_TIMER, MOTOR_CONTROL_CYCLES(PARAMID_LD_HALFPERIOD_MS));

    // p_params->VdBias     = FOC_Vd(&p_runtime->Foc);
    // p_params->VdStep     = PARAMID_LD_VSTEP_FRACT16;
    // p_params->IdSteady   = FOC_Id(&p_runtime->Foc);

    p_runtime->CalibrationTimer = 0U;
    p_params->IdTauCycles = 0U;

    /*
        Target current = (Vbias + step) / Rs; since Vbias = Rs * IdSteady, target = IdSteady + step / Rs.
        Using normalized: delta_id = step_fract16 * R_REF / Rs_fract16_ohm = fract16_div(step, Rs_Fract16).
    */
    /* 63.2% crossing threshold, scaled: 2/pi approx 0.632 ~= 20724/32768 */
    p_params->IdTau = p_params->IdSteady + ((uint64_t)fract16_div(p_params->VdStep, p_results->Rs_Fract16) * 20724) / 32768;
}

static void Ld_Proc(Motor_T * p_motor)
{
    Motor_State_T * p_runtime = p_motor->P_MOTOR;
    Motor_ParamsBuffer_T * p_params; /* temp */
    Motor_ElectricalParams_T * p_results;

    FOC_CaptureIabc(&p_runtime->Foc, &p_runtime->PhaseInput.I);
    Motor_FOC_ProcAngleFeedforwardV(p_motor, 0, p_params->VdBias + p_params->VdStep, 0);

    /* Mark first crossing of the 63% threshold */
    if ((p_params->IdTauCycles == 0U) && (FOC_Id(&p_runtime->Foc) >= (int32_t)p_params->IdTau))
    {
        p_params->IdTauCycles = p_runtime->CalibrationTimer;
    }

    p_runtime->CalibrationTimer++;
    Motor_FOC_WriteDuty(p_motor);
}

static State_T * Ld_Next(Motor_T * p_motor)
{
    Motor_State_T * p_runtime = p_motor->P_MOTOR;
    Motor_ElectricalParams_T * p_results;/*  = &p_motor->ParamId.Results; */
    Motor_ParamsBuffer_T * p_buffer;

    /* End of step window (use one half-period of the planned square wave) */
    if (TimerT_Poll(&p_motor->CONTROL_TIMER))
    {
        /* Never crossed -> step too small or Rs estimate off. Fall back to window end. */
        if (p_buffer->IdTauCycles == 0U) { p_buffer->IdTauCycles = p_runtime->CalibrationTimer; }
        p_results->Ld_MicroHenries = l_uh_of_rs_tau(MOTOR_CONTROL_FREQ, p_results->Rs_MilliOhms, p_buffer->IdTauCycles);

        return &MOTOR_STATE_CALIBRATION;
    }
    return NULL;
}


const State_T CALIBRATION_STATE_LD =
{
    .P_PARENT = &MOTOR_STATE_CALIBRATION,
    .P_TOP    = &MOTOR_STATE_CALIBRATION,
    .DEPTH    = 1U,
    .ENTRY    = (State_Action_T)Ld_Entry,
    .LOOP     = (State_Action_T)Ld_Proc,
    .NEXT     = (State_Input0_T)Ld_Next,
};


// const State_T LD_RS_STATE =
// {
//     .P_PARENT = &CALIBRATION_RS_STATE,
//     .P_TOP    = &MOTOR_STATE_CALIBRATION,
//     .DEPTH    = 2U,
//     .NEXT     = (State_Input0_T)Rs_Next, /* transition to CALIBRATION_STATE_LD after Rs */
// };

/* settle then transition to LD. assuming Rs already stored */
static State_T * LdAlign_Next(Motor_T * p_motor, state_value_t v) { (void)p_motor; (void)v; return &CALIBRATION_STATE_LD; }

const State_T LD_ALIGN_STATE =
{
    .P_PARENT = &RS_ALIGN_START_STATE,
    .P_TOP    = &MOTOR_STATE_CALIBRATION,
    .DEPTH    = 3U,
    .NEXT     = (State_Input0_T)LdAlign_Next,
};

static State_T * Ld_Start(Motor_T * p_motor, state_value_t v) { (void)p_motor; (void)v; return (State_T *)&LD_ALIGN_STATE; }

void Motor_Calibration_StartLd(Motor_T * p_motor)
{
    static const StateMachine_TransitionCmd_T CMD = { .P_START = &MOTOR_STATE_CALIBRATION, .NEXT = (State_Input_T)Ld_Start };
    StateMachine_Tree_InvokeTransition(&p_motor->STATE_MACHINE, &CMD, 0U);
}

bool Motor_Calibration_IsLd(Motor_T * p_motor)
{
    return StateMachine_IsLeafState(p_motor->STATE_MACHINE.P_ACTIVE, &CALIBRATION_STATE_LD);
}


/******************************************************************************/
/*!
    @brief  CALIBRATION_STATE_LQ

    LQ_HFI: voltage-mode Vd = Vbias, Vq = V_hf*sin(phase). Demodulate Iq synchronously.
*/
/*
    LQ_HFI: voltage-mode Vd = Vbias (holds Id ~ IdBias), Vq = V_hf * sin(phase).
    No current-loop interference on Vq. Sample Iq, demodulate synchronously with sin/cos.
    Lq = V_hf / (2*pi*fh * |Iq_amp|) where |Iq_amp| = 2*sqrt(I^2+Q^2)/N.
*/
/******************************************************************************/
static void Lq_Entry(Motor_T * p_motor)
{
    Motor_State_T * p_runtime = p_motor->P_MOTOR;
    Motor_ParamsBuffer_T * p_buffer;
    Phase_ActivateV0(&p_motor->PHASE);
    TimerT_Periodic_Init(&p_motor->CONTROL_TIMER, MOTOR_CONTROL_CYCLES(PARAMID_LQ_WINDOW_MS));
    // p_runtime->ParamId.VdBias    = FOC_Vd(&p_runtime->Foc);
    p_runtime->CalibrationTimer = 0U;

    p_buffer->HfiPhase = 0U;
    // p_runtime->ParamId.IqSumI    = 0;
    // p_runtime->ParamId.IqSumQ    = 0;
    // p_runtime->ParamId.AccumN    = 0U;
    // p_runtime->ParamId.CycleCount = 0U;
}

static void Lq_Proc(Motor_T * p_motor)
{
    Motor_State_T * p_runtime = p_motor->P_MOTOR;
    Motor_ElectricalParams_T * p_results;/*  = &p_motor->ParamId.Results; */
    Motor_ParamsBuffer_T * p_buffer;

    fract16_t vq_inject = fract16_mul(PARAMID_VHF_FRACT16, fract16_sin(p_buffer->HfiPhase));
    Motor_FOC_ProcAngleFeedforwardV(p_motor, 0, p_buffer->VdBias, vq_inject);

    /* Skip a couple HFI cycles at start so Iq steady-state demod window is clean */
    if (TimerT_GetElapsed(&p_motor->CONTROL_TIMER) >= MOTOR_CONTROL_FREQ / PARAMID_HF_FREQ_HZ * 2U)
    {
        // p_buffer->IqSumI += (int64_t)FOC_Iq(&p_runtime->Foc) * fract16_sin(p_buffer->HfiPhase);
        // p_buffer->IqSumQ += (int64_t)FOC_Iq(&p_runtime->Foc) * fract16_cos(p_buffer->HfiPhase);
        Accumulator_Add(&p_runtime->Accumulators[0], fract16_mul(FOC_Iq(&p_runtime->Foc), fract16_sin(p_buffer->HfiPhase)));
        Accumulator_Add(&p_runtime->Accumulators[1], fract16_mul(FOC_Iq(&p_runtime->Foc), fract16_cos(p_buffer->HfiPhase)));
    }
    p_runtime->CalibrationTimer++;

    p_buffer->HfiPhase += PARAMID_HFI_PHASE_DELTA;

    Motor_FOC_WriteDuty(p_motor);
}

static State_T * Lq_Next(Motor_T * p_motor)
{
    Motor_State_T * p_runtime = p_motor->P_MOTOR;
    Motor_ElectricalParams_T * p_results;/*  = &p_buffer->Results; */
    Motor_ParamsBuffer_T * p_buffer;

    if (TimerT_Poll(&p_motor->CONTROL_TIMER))
    {
        /* iq*sin scaled as fract16*fract16; normalize by 32768 then by N, then multiply by 2 for amplitude. */
        /* Demod magnitude in fract16: sqrt((2/N * Σ iq*sin)^2 + (2/N * Σ iq*cos)^2).  Scale-down by 2^15 since iq and sin are both fract16. */
        // int64_t i_norm = (p_buffer->IqSumI * 2) / p_runtime->CalibrationTimer / 32768;
        // int64_t q_norm = (p_buffer->IqSumQ * 2) / p_runtime->CalibrationTimer / 32768;
        // uint64_t mag_sq = (uint64_t)(i_norm * i_norm + q_norm * q_norm);
        int64_t i_norm = (Accumulator_Output(&p_runtime->Accumulators[0]) * 2) / p_runtime->CalibrationTimer;
        int64_t q_norm = (Accumulator_Output(&p_runtime->Accumulators[1]) * 2) / p_runtime->CalibrationTimer;
        uint32_t mag = fixed_sqrt(i_norm * i_norm + q_norm * q_norm);
        p_results->Lq_MicroHenries = l_uh_of_hfi(Phase_Calibration_GetVMaxVolts(), Phase_Calibration_GetIMaxAmps(), PARAMID_HF_FREQ_HZ, PARAMID_VHF_FRACT16, mag);
        return &MOTOR_STATE_CALIBRATION;
    }
    return NULL;
}


const State_T CALIBRATION_STATE_LQ =
{
    .P_PARENT = &MOTOR_STATE_CALIBRATION,
    .P_TOP    = &MOTOR_STATE_CALIBRATION,
    .DEPTH    = 1U,
    .ENTRY    = (State_Action_T)Lq_Entry,
    .LOOP     = (State_Action_T)Lq_Proc,
    .NEXT     = (State_Input0_T)Lq_Next,
};

// const State_T LQ_RS_STATE =
// {
//     .P_PARENT = &CALIBRATION_RS_STATE,
//     .P_TOP    = &MOTOR_STATE_CALIBRATION,
//     .DEPTH    = 2U,
//     .NEXT     = (State_Input0_T)Rs_Next, /* transition to CALIBRATION_STATE_LQ after Rs */
// };

static State_T * LqAlign_Next(Motor_T * p_motor, state_value_t v) { (void)p_motor; (void)v; return &CALIBRATION_STATE_LQ; }

const State_T LQ_ALIGN_STATE =
{
    .P_PARENT = &RS_ALIGN_START_STATE,
    .P_TOP    = &MOTOR_STATE_CALIBRATION,
    .DEPTH    = 3U,
    .NEXT     = (State_Input0_T)LqAlign_Next, /* transition to CALIBRATION_STATE_LQ */
};

static State_T * Lq_Start(Motor_T * p_motor, state_value_t v) { (void)p_motor; (void)v; return (State_T *)&LQ_ALIGN_STATE; }

void Motor_Calibration_StartLq(Motor_T * p_motor)
{
    static const StateMachine_TransitionCmd_T CMD = { .P_START = &MOTOR_STATE_CALIBRATION, .NEXT = (State_Input_T)Lq_Start };
    StateMachine_Tree_InvokeTransition(&p_motor->STATE_MACHINE, &CMD, 0U);
}

bool Motor_Calibration_IsLq(Motor_T * p_motor)
{
    return StateMachine_IsLeafState(p_motor->STATE_MACHINE.P_ACTIVE, &CALIBRATION_STATE_LQ);
}
