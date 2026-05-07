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
*/
/******************************************************************************/
#include "Motor_Calibration.h"
#include "../Motor_FOC.h"
#include "../Motor_ControlFreq.h"
#include "../Phase_Input/Phase_Calibration.h"
#include "../Phase_Input/Phase_VBus.h"

#include "Math/Fixed/fract16.h"
#include "Math/math_general.h"
#include "../Math/motor_params_math.h"

#include <stdint.h>

/******************************************************************************/
/*!
    @brief  Tunable constants
*/
/******************************************************************************/

/* Durations in control cycles (20 kHz => 1 ms = 20 cycles) */
#define PARAMID_ALIGN_SETTLE_MS     (500U)  /* rotor pull-in + damping */
#define PARAMID_RS_WINDOW_MS        (100U)  /* averaging window */
// #define PARAMID_LD_HALFPERIOD_MS    (2U)    /* Vd square-wave half period */
// #define PARAMID_LD_SETTLE_CYCLES    (8U)    /* samples skipped at start of each half */
#define PARAMID_LD_WINDOW_MS        (20U)   /* step-response observation window */
#define PARAMID_LQ_WINDOW_MS        (32U)   /* HFI integration window */
#define PARAMID_RAMPDOWN_MS         (100U)
#define PARAMID_HFI_FREQ_HZ          (1000U) /* HFI frequency */
#define PARAMID_VHFI_FRACT16         (3277)  /* ~10% of V_MAX injection amplitude */
#define PARAMID_LD_VSTEP_FRACT16     (1638)  /* ~5% of V_MAX step amplitude */

/* Phase advance per PWM cycle for HFI sinusoid = fh * 2^16 / Fs (angle16) */
#define PARAMID_HFI_PHASE_DELTA     (angle16_t)((uint32_t)PARAMID_HFI_FREQ_HZ * ANGLE16_PER_REVOLUTION / MOTOR_CONTROL_FREQ)


typedef enum ElectricalCalibraton_Stage
{
    PARAMID_STEP_ALIGN = 0,
    PARAMID_STEP_RS_MEASURE,
    PARAMID_STEP_LD_INJECT,
    PARAMID_STEP_LQ_HFI,
    PARAMID_STEP_COMMIT,
    PARAMID_STEP_RAMPDOWN,
    PARAMID_STEP_DONE,
}
ElectricalCalibraton_Stage_T;

/*
    Electrical parameter identification scratch. Used by CALIBRATION_STATE_ELECTRICAL.
*/
typedef struct ElectricalCalibration
{
    ElectricalCalibraton_Stage_T  Step;              /* ElectricalCalibraton_Stage_T */
    uint32_t CycleCount;        /* cycles within current step */

    fract16_t IdBias;           /* aligned Id setpoint */

    int64_t  VdAccum;
    int64_t  IdAccum;
    uint32_t AccumN;

    /* Ld step */
    fract16_t VdStep;

    fract16_t VdBias;
    int16_t   IdSteady;
    uint32_t  IdTau; /* threshold */
    uint32_t  IdTauCycles;

    /* Lq HFI */
    angle16_t HfiPhase;
    angle16_t HfiDelta;
    uint16_t  Vhfi;
    uint16_t  HfiFreqHz;

    int64_t   IqSumI;
    int64_t   IqSumQ;
    Motor_ElectricalParams_T Results;
}
ElectricalCalibration_T;

static_assert(sizeof(ElectricalCalibration_T) <= MOTOR_CALIBRATION_BUFFER_SIZE, "ElectricalCalibration_T must fit within MOTOR_CALIBRATION_BUFFER_SIZE");

static ElectricalCalibration_T * ElectricalCalibrationBuffer(Motor_State_T * p_motor) { return (ElectricalCalibration_T *)(p_motor->CalibrationBuffer); }


/******************************************************************************/
/*!
    Sequence:
    ALIGN         : ramp Id to bias, rotor settles on d-axis (angle = 0)
    RS_MEASURE    : average Vd / Id at steady state, Rs = Vd/Id
    LD_INJECT     : voltage-mode square wave on Vd, fit first-order tau, Ld = Rs * tau
    LQ_HFI        : HFI sinusoid on Vq with DC Id bias, demodulate Iq, Lq = V_hf / (2*pi*fh*|Iq|)
    COMMIT        : write SI + fract16 forms to Config, recompute KLd/KLq/KPsi
    RAMPDOWN      : ramp Id back to zero, exit to parent
*/
/******************************************************************************/

// static void paramid_reset_accumulators(ElectricalCalibration_T * p_params)
// {
//     p_params->VdAccum = 0;
//     p_params->IdAccum = 0;
//     p_params->AccumN  = 0U;
//     p_params->IqSumI  = 0;
//     p_params->IqSumQ  = 0;
// }

static void SetNext(ElectricalCalibration_T * p_params, ElectricalCalibraton_Stage_T next)
{
    p_params->Step = next;
    p_params->CycleCount = 0U;
    p_params->AccumN = 0U;
}


/*
    LD_INJECT: voltage-mode Vd = Vbias + step, measure time for Id to cross 63% of (Id_steady + step/Rs).
*/
static int32_t GetIdTau(ElectricalCalibration_T * p_params) { return p_params->IdSteady + ((uint64_t)fract16_div(p_params->VdStep, p_params->Results.Rs_Fract16) * 20724) / 32768; }

/*
    Motor_FOC_AngleControl(p_motor, 0, p_params->IdBias, 0);
*/
static void ProcRs(ElectricalCalibration_T * p_params, fract16_t vd, fract16_t id)
{
    p_params->VdAccum += vd;
    p_params->IdAccum += id;
    p_params->AccumN++;

    if (p_params->CycleCount >= MOTOR_CONTROL_CYCLES(PARAMID_RS_WINDOW_MS))
    {
        int32_t vd_avg = (int32_t)(p_params->VdAccum / p_params->AccumN);
        int32_t id_avg = (int32_t)(p_params->IdAccum / p_params->AccumN);

        p_params->Results.Rs_Fract16 = rs_fract16(vd_avg, id_avg);
        p_params->Results.Rs_MilliOhms = rs_mohms_of_fract16(Phase_Calibration_GetVMaxVolts(), Phase_Calibration_GetIMaxAmps(), p_params->Results.Rs_Fract16);

        /* Precompute steady-state Vd bias so Ld step averages to Id_bias. */
        p_params->VdStep               = PARAMID_LD_VSTEP_FRACT16;
        p_params->VdBias               = (fract16_t)vd_avg;
        p_params->IdSteady             = (int16_t)id_avg;
        p_params->IdTauCycles          = 0U;
        p_params->IdTau                = GetIdTau(p_params);
        SetNext(p_params, PARAMID_STEP_LD_INJECT);
    }
    // else
    {
        p_params->CycleCount++;
    }
}

/*
    LD_INJECT: drive voltage-mode Vd = Vbias + step, sample Id rise time from Vbias steady.
    Simple single-positive-step method:
        At t = 0: Vd := Vbias + VdStep; Id rises from IdSteady toward (Vbias+VdStep)/Rs.
        Time constant tau when Id reaches IdSteady + 0.632 * (IdTarget - IdSteady).
*/
static void ProcLd(ElectricalCalibration_T * p_params, fract16_t id)
{
    assert(p_params->IdTau > 0);

    if ((p_params->IdTauCycles == 0U) && ((int32_t)id >= (int32_t)p_params->IdTau)) { p_params->IdTauCycles = p_params->CycleCount; }

    p_params->CycleCount++;

    // if (p_params->CycleCount >= MOTOR_CONTROL_CYCLES(PARAMID_LD_HALFPERIOD_MS))
    if (p_params->CycleCount >= MOTOR_CONTROL_CYCLES(PARAMID_LD_WINDOW_MS))
    {
        if (p_params->IdTauCycles == 0U) { p_params->IdTauCycles = p_params->CycleCount; }
        p_params->Results.Ld_MicroHenries = l_uh_of_rs_tau(MOTOR_CONTROL_FREQ, p_params->Results.Rs_MilliOhms, p_params->IdTauCycles);
        SetNext(p_params, PARAMID_STEP_LQ_HFI);
    }
}

/*
    LQ_HFI: voltage-mode Vd = Vbias (holds Id ~ IdBias), Vq = V_hf * sin(phase).
    No current-loop interference on Vq. Sample Iq, demodulate synchronously with sin/cos.
    Lq = V_hf / (2*pi*fh * |Iq_amp|) where |Iq_amp| = 2*sqrt(I^2+Q^2)/N.
*/
static void ProcLq(ElectricalCalibration_T * p_params, fract16_t iq)
{
    if (p_params->CycleCount >= MOTOR_CONTROL_FREQ / p_params->HfiFreqHz * 2U)
    {
        p_params->IqSumI += (int64_t)iq * fract16_sin(p_params->HfiPhase);
        p_params->IqSumQ += (int64_t)iq * fract16_cos(p_params->HfiPhase);
        p_params->AccumN++;
    }

    p_params->HfiPhase += p_params->HfiDelta; /* PARAMID_HFI_PHASE_DELTA */
    p_params->CycleCount++;

    if (p_params->CycleCount >= MOTOR_CONTROL_CYCLES(PARAMID_LQ_WINDOW_MS))
    {
        int32_t i_norm = (int32_t)((p_params->IqSumI * 2) / p_params->AccumN / 32768);
        int32_t q_norm = (int32_t)((p_params->IqSumQ * 2) / p_params->AccumN / 32768);
        uint32_t mag = fixed_sqrt(i_norm * i_norm + q_norm * q_norm);
        p_params->Results.Lq_MicroHenries = l_uh_of_hfi(Phase_Calibration_GetVMaxVolts(), Phase_Calibration_GetIMaxAmps(), p_params->HfiFreqHz, p_params->Vhfi, mag);
        SetNext(p_params, PARAMID_STEP_COMMIT);
    }
}


/* ALIGN: ramp Id to bias at electrical angle 0, hold until rotor settles. */
static void ProcAlign(ElectricalCalibration_T * p_params)
{
    if (p_params->CycleCount >= MOTOR_CONTROL_CYCLES(PARAMID_ALIGN_SETTLE_MS)) { SetNext(p_params, PARAMID_STEP_RS_MEASURE); }
    p_params->CycleCount++;
}

/* RAMPDOWN: ramp Id back to zero. */
static void ProcRampDown(ElectricalCalibration_T * p_params)
{
    if (p_params->CycleCount >= MOTOR_CONTROL_CYCLES(PARAMID_RAMPDOWN_MS)) { SetNext(p_params, PARAMID_STEP_DONE); }
    p_params->CycleCount++;
}

/* COMMIT: copy results into Motor_Config_T, recompute decoupling K's. */
static void CommitResults(ElectricalCalibration_T * p_params, Motor_ElectricalParams_T * p_storage)
{
    *p_storage = p_params->Results;
    SetNext(p_params, PARAMID_STEP_RAMPDOWN);
}


/******************************************************************************/
/*!
    @brief  Substate plumbing
*/
/******************************************************************************/
extern const State_T MOTOR_STATE_CALIBRATION;

static void Electrical_Entry(Motor_T * p_motor)
{
    Motor_State_T * p_context = p_motor->P_MOTOR;
    ElectricalCalibration_T * p_params = ElectricalCalibrationBuffer(p_context);

    Phase_ActivateV0(&p_motor->PHASE);
    p_context->FeedbackMode.Current = 1U;
    Motor_FOC_ClearFeedbackState(p_context);
    Angle_ZeroCaptureState(&p_context->OpenLoopAngle);

    p_context->ControlTimerBase = 0U;

    /* Clear full scratch, then set bias */
    *p_params = (ElectricalCalibration_T){ 0 };
    p_params->Step = PARAMID_STEP_ALIGN;
    p_params->IdBias = Motor_GetIAlign(&p_context->Config);
    p_params->VdStep = PARAMID_LD_VSTEP_FRACT16;
    p_params->HfiFreqHz = PARAMID_HFI_FREQ_HZ;
    p_params->HfiDelta = PARAMID_HFI_PHASE_DELTA;
    p_params->Vhfi = PARAMID_VHFI_FRACT16;
}

static void Electrical_Proc(Motor_T * p_motor)
{
    Motor_State_T * p_context = p_motor->P_MOTOR;
    ElectricalCalibration_T * p_params = ElectricalCalibrationBuffer(p_context);

    switch (p_params->Step)
    {
        case PARAMID_STEP_ALIGN:
            Motor_FOC_ProcAngleAlignOf(p_context, 0, p_params->IdBias);
            ProcAlign(p_params);
            break;
        case PARAMID_STEP_RS_MEASURE:
            Motor_FOC_ProcAngleAlignOf(p_context, 0, p_params->IdBias);
            ProcRs(p_params, FOC_Vd(&p_context->Foc), FOC_Id(&p_context->Foc));
            break;
        case PARAMID_STEP_LD_INJECT:
            fract16_t vd = p_params->VdBias + p_params->VdStep;
            Motor_FOC_ProcAngleFeedforwardV(p_context, 0, vd, 0);
            ProcLd(p_params, FOC_Id(&p_context->Foc));
            break;
        case PARAMID_STEP_LQ_HFI:
            fract16_t vq_inject = fract16_mul(p_params->Vhfi, fract16_sin(p_params->HfiPhase));
            Motor_FOC_ProcAngleFeedforwardV(p_context, 0, p_params->VdBias, vq_inject);
            ProcLq(p_params, FOC_Iq(&p_context->Foc));
            break;
        case PARAMID_STEP_COMMIT:
            CommitResults(p_params, &p_context->Config.ElectricalParams);
            Motor_ResolveDecouplingCoeffs(p_context);
            break;
        case PARAMID_STEP_RAMPDOWN:
            Motor_FOC_ProcAngleAlignOf(p_context, 0, 0);
            ProcRampDown(p_params);
            break;
        case PARAMID_STEP_DONE:
        default:
            /* Hold zero Vd/Vq until parent promotes back to CALIBRATION root */
            Motor_FOC_ProcAngleAlignOf(p_context, 0, 0);
            break;
    }

    Motor_FOC_WriteDuty(p_motor);
}

State_T * Electrical_Next(Motor_T * p_motor)
{
    if (ElectricalCalibrationBuffer(p_motor->P_MOTOR)->Step == PARAMID_STEP_DONE) { return &MOTOR_STATE_CALIBRATION; }
    return NULL; /* No external input transitions. */
}

const State_T CALIBRATION_STATE_ELECTRICAL =
{
    .P_PARENT = &MOTOR_STATE_CALIBRATION,
    .P_TOP    = &MOTOR_STATE_CALIBRATION,
    .DEPTH    = 1U,
    .ENTRY    = (State_Action_T)Electrical_Entry,
    .LOOP     = (State_Action_T)Electrical_Proc,
    .NEXT     = (State_Input0_T)Electrical_Next,
};


/* Transition entry */
static State_T * Electrical_Start(Motor_T * p_motor, state_value_t value) { (void)p_motor; (void)value; return (State_T *)&CALIBRATION_STATE_ELECTRICAL; }

void Motor_Calibration_StartElectrical(Motor_T * p_motor)
{
    static const StateMachine_TransitionCmd_T CMD = { .P_START = &MOTOR_STATE_CALIBRATION, .NEXT = (State_Input_T)Electrical_Start, };
    StateMachine_Tree_InvokeTransition(&p_motor->STATE_MACHINE, &CMD, 0U);
}

bool Motor_Calibration_IsElectrical(Motor_T * p_motor)
{
    return StateMachine_IsLeafState(p_motor->STATE_MACHINE.P_ACTIVE, &CALIBRATION_STATE_ELECTRICAL);
}

