/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

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
    @file   Motor_FOC.c
    @author FireSourcery
    @brief

*/
/******************************************************************************/
#include "Motor_FOC.h"


/******************************************************************************/
/*!

*/
/******************************************************************************/
static bool CaptureIabc(Motor_State_T * p_motor)
{
    bool isCaptureI = (p_motor->PhaseInput.IFlags.Bits == PHASE_ID_ABC);
    if (isCaptureI == true)  /* alternatively use batch callback */
    {
        FOC_ProcClarkePark(&p_motor->Foc, p_motor->PhaseInput.Iabc.A, p_motor->PhaseInput.Iabc.B, p_motor->PhaseInput.Iabc.C);
        p_motor->PhaseInput.IFlags.Bits = PHASE_ID_0;
    }
    return isCaptureI;
}

// static inline interval_t Motor_GetVqLimits(const Motor_State_T * p_motor)
// {
//     int32_t window = Phase_VBus_GetVNominal() / 8; /* plugging clamped beyond 1/2 VPhaseRef. e.g 20VBus -> [-2.5, 2.5] or [0, 5] */
//     int32_t vSpeed = _Motor_GetVSpeed_Fract16(p_motor);
//     return (interval_t) { .low = vSpeed - window, .high = vSpeed + window, };
// }

/*
    Current Feedback Loop
*/
/*
    Limit-first: compute Vq budget from voltage circle after Vd, set PidIq limits before proc
*/
static void ProcIFeedback(Motor_State_T * p_motor, int16_t idReq, int16_t iqReq)
{
    FOC_SetVd(&p_motor->Foc, PID_ProcPI(&p_motor->PidId, FOC_Id(&p_motor->Foc), idReq));
    int16_t vqLimit = _FOC_VqCircleLimit(&p_motor->Foc, fract16_mul(Phase_VBus_Fract16(), FRACT16_1_DIV_2));
    interval_t band = interval_of_sign((sign_t)p_motor->Direction, vqLimit);
    PID_CaptureOutputLimits(&p_motor->PidIq, band.low, band.high);
    FOC_SetVq(&p_motor->Foc, PID_ProcPI(&p_motor->PidIq, FOC_Iq(&p_motor->Foc), iqReq));
}

static void ProcIFeedback_BackLimit(Motor_State_T * p_motor, int16_t idReq, int16_t iqReq)
{
    FOC_SetVd(&p_motor->Foc, PID_ProcPI(&p_motor->PidId, FOC_Id(&p_motor->Foc), idReq));
    FOC_SetVq(&p_motor->Foc, PID_ProcPI(&p_motor->PidIq, FOC_Iq(&p_motor->Foc), iqReq));
    /* the combine output state can still grow outside of circle limit. limit after proc may still have windup. propagate if limited. */
    if (FOC_ProcVectorLimit(&p_motor->Foc, Phase_VBus_Fract16()) == true)
    {
        _PID_SetOutputState(&p_motor->PidIq, FOC_Vq(&p_motor->Foc)); // immediately saturates on input anamoly
        // if (math_abs(FOC_Vq(&p_motor->Foc)) > PID_GetIntegral(&p_motor->PidIq)) { PID_SetOutputState(&p_motor->PidIq, FOC_Vq(&p_motor->Foc)); }
    }
}


/* Set Vdq */
static void ProcInnerFeedback(Motor_State_T * p_motor, int16_t dReq, int16_t qReq)
{
    if (CaptureIabc(p_motor) == true)    /* else update angle only for voutput, until next cycle */
    {
        if (p_motor->FeedbackMode.Current == 1U)  /* Current Control mode */
        {
            ProcIFeedback(p_motor, dReq, qReq);
        }
        else /* if (p_motor->FeedbackMode.Current == 0U)  Voltage Control mode - Apply limits only */
        {
            FOC_SetVq(&p_motor->Foc, qReq);
            FOC_SetVd(&p_motor->Foc, dReq);
            FOC_ProcVectorLimit(&p_motor->Foc, Phase_VBus_Fract16());
            /* Still check CaptureIabc for overcurrent */
        }
    }
}


/* From Vdq to Vabc Duty */
static void ProcAngleOutput(Motor_State_T * p_motor)
{
    FOC_ProcInvClarkePark(&p_motor->Foc);
    FOC_ProcSvpwm(&p_motor->Foc, Phase_VBus_Inv_Fract32());
}


/******************************************************************************/
/*!

*/
/******************************************************************************/
/*
    Activate angle with or without current feedback
    Req dq to Vabc to Duty abc
*/
void Motor_FOC_AngleControl(Motor_State_T * p_motor, angle16_t theta, fract16_t dReq, fract16_t qReq)
{
    FOC_SetTheta(&p_motor->Foc, theta);
    // split capture feedback
    ProcInnerFeedback(p_motor, dReq, qReq);
    ProcAngleOutput(p_motor);
}

/*
    Feed forward voltage angle
    set once, no feedback
    SetVAngle
*/
void Motor_FOC_ProcAngleFeedforwardV(Motor_State_T * p_motor, angle16_t theta, fract16_t vd, fract16_t vq)
{
    FOC_SetTheta(&p_motor->Foc, theta);
    FOC_SetVd(&p_motor->Foc, vd);
    FOC_SetVq(&p_motor->Foc, vq);
    // circle limit here if needed
    ProcAngleOutput(p_motor);
}

void Motor_FOC_ProcTorqueReq(Motor_State_T * p_motor, fract16_t dReq, fract16_t qReq)
{
    /* Call bounds input */
    // Motor_FOC_AngleControl(p_motor, Angle_Value(&p_motor->SensorState.AngleSpeed), dReq, Ramp_ProcNextOf(&p_motor->TorqueRamp, qReq));
    Motor_FOC_AngleControl(p_motor, Angle_Value(&p_motor->SensorState.AngleSpeed), dReq, Motor_IRampOf(p_motor, qReq));

    // as i loop only
    // FOC_SetTheta(&p_motor->Foc, theta);

    // if (CaptureIabc(p_motor) == true)
    // {
    //     ProcIFeedback(p_motor, dReq, qReq);
    // }
}




/*
    Feedback Control Loop
    StateMachine calls each PWM, ~20kHz

    State:
        Angle,
        FeebackMode,
        ReqQ/PidSpeed/TorqueRampOutput,
        Iabc,
        PidIq,
        PidId,
        VSourceInvScalar,
*/
void Motor_FOC_ProcAngleControl(Motor_State_T * p_motor)
{
#ifdef MOTOR_EXTERN_CONTROL_ENABLE
    Motor_ExternControl(p_motor);
#endif
    int16_t qReq = (p_motor->FeedbackMode.Current == 1U) ? Ramp_ProcNext(&p_motor->TorqueRamp) : Motor_VRamp(p_motor);
    Motor_FOC_AngleControl(p_motor, Angle_Value(&p_motor->SensorState.AngleSpeed), 0, qReq);
}

/*
    ProcAngleObserve
    Angle Observe VBemf FreeWheel and Stop State
    Updates Vabc, Valpha, Vbeta, Vd, Vq
*/
void Motor_FOC_ProcCaptureAngleVBemf(Motor_State_T * p_motor)
{
    FOC_SetTheta(&p_motor->Foc, Angle_Value(&p_motor->SensorState.AngleSpeed));

    if (p_motor->PhaseInput.VFlags.Bits == PHASE_ID_ABC)
    {
        FOC_ProcVBemfClarkePark(&p_motor->Foc, p_motor->PhaseInput.Vabc.A, p_motor->PhaseInput.Vabc.B, p_motor->PhaseInput.Vabc.C);
        p_motor->PhaseInput.VFlags.Bits = PHASE_ID_0;
    }
}


/******************************************************************************/
/*!

*/
/******************************************************************************/
/*
    also clears user view during vbemf, Ifeedback not updated
*/
void Motor_FOC_ClearFeedbackState(Motor_State_T * p_motor)
{
    FOC_ClearCaptureState(&p_motor->Foc); /* Clear for view, updated again on enter control */
    FOC_ClearOutputState(&p_motor->Foc); /* Emergency stop, capture bemf subsitute */
    PID_Reset(&p_motor->PidIq);
    PID_Reset(&p_motor->PidId);
    PID_Reset(&p_motor->PidSpeed);
    Ramp_SetOutputState(&p_motor->SpeedRamp, 0);
    Ramp_SetOutputState(&p_motor->TorqueRamp, 0);
    Phase_Input_ClearI(&p_motor->PhaseInput);
    Phase_Input_ClearV(&p_motor->PhaseInput);
    Ramp_SetTarget(&p_motor->TorqueRamp, 0);
    Ramp_SetTarget(&p_motor->SpeedRamp, 0);
}

void _Motor_FOC_MatchIVState(Motor_State_T * p_motor, int16_t vd, int16_t vq)
{
    PID_SetOutputState(&p_motor->PidId, vd);
    PID_SetOutputState(&p_motor->PidIq, vq);
}

/* torque only states */
void Motor_FOC_MatchIVState(Motor_State_T * p_motor)
{
    if (FOC_Vq(&p_motor->Foc) == 0) { _Motor_FOC_MatchIVState(p_motor, 0, Motor_GetVSpeed_Fract16(p_motor)); }
    else { _Motor_FOC_MatchIVState(p_motor, 0, FOC_Vq(&p_motor->Foc)); }
}

/*!
    Match Feedback State/Output to Output Voltage
    On FeedbackMode update and Freewheel to Run
    StateMachine blocks feedback proc
*/
void Motor_FOC_MatchFeedbackState(Motor_State_T * p_motor)
{
    if (p_motor->FeedbackMode.Current == 1U)
    {
        Motor_FOC_MatchIVState(p_motor);
        Ramp_SetOutputState(&p_motor->TorqueRamp, FOC_Iq(&p_motor->Foc)); /* case transitioning without release into freewheel */
    }
    else
    {
        Ramp_SetOutputState(&p_motor->TorqueRamp, FOC_Vq(&p_motor->Foc)); //different units for now
    }

    Ramp_SetTarget(&p_motor->TorqueRamp, Ramp_GetOutput(&p_motor->TorqueRamp));
    Motor_MatchSpeedTorqueState(p_motor, Ramp_GetOutput(&p_motor->TorqueRamp)); // or let state machine handle
}

/*
    Set on Direction change
    Iq/Id PID always Vq/Vd. Clip opposite user direction range, no plugging.

    p_motor->Direction is set by caller
*/
void _Motor_FOC_ApplyVLimits(Motor_State_T * p_motor, int16_t vRef)
{
    interval_t v = VBus_AntiPluggingLimits(p_motor->p_VBus, (sign_t)p_motor->Direction);
    PID_SetOutputLimits(&p_motor->PidIq, v.low, v.high);
    PID_SetOutputLimits(&p_motor->PidId, 0 - vRef, vRef);
}

void Motor_FOC_ApplyVLimits(Motor_State_T * p_motor)
{
    _Motor_FOC_ApplyVLimits(p_motor, Phase_VBus_GetVRefSvpwm());
}

void Motor_FOC_SetDirection(Motor_T * p_dev, Motor_Direction_T direction)
{
    Motor_SetDirection(p_dev, direction); /* alternatively caller handle */
    Motor_FOC_ApplyVLimits(p_dev->P_MOTOR);
}




/******************************************************************************/
/*!
    Open Loop
    StateMachine mapping
*/
/******************************************************************************/
/*
    Align using TorqueRamp.Target
*/
/* ElectricalAngle set by caller. Does not Angle_ZeroCaptureState */
void Motor_FOC_StartAlignCmd(Motor_State_T * p_motor)
{
    p_motor->FeedbackMode.Current = 1U; /* config alternatively */
    Motor_FOC_ClearFeedbackState(p_motor); /* reset TorqueRamp i/v to start at 0 */
}

void Motor_FOC_ProcAlignCmd(Motor_State_T * p_motor)
{
    Motor_FOC_AngleControl(p_motor, Angle_Value(&p_motor->OpenLoopAngle), Motor_OpenLoopTorqueRampOf(p_motor, Ramp_GetTarget(&p_motor->TorqueRamp)), 0);
}


/*
    Ramp towards Preset AlignScalar_Fract16 * IRatedPeak
    Caller sets time
*/
void Motor_FOC_StartStartUpAlign(Motor_State_T * p_motor)
{
    p_motor->FeedbackMode.Current = 1U; /* StartUp always select current feedback */
    Motor_FOC_ClearFeedbackState(p_motor); /* Resets Ramp */
    Angle_ZeroCaptureState(&p_motor->OpenLoopAngle); /* always align A */
}

void Motor_FOC_ProcStartUpAlign(Motor_State_T * p_motor)
{
    Motor_FOC_AngleControl(p_motor, Angle_Value(&p_motor->OpenLoopAngle), Motor_OpenLoopTorqueRampOf(p_motor, Motor_GetIAlign(&p_motor->Config)), 0); /*  */
}

/*
    OpenLoop Spin - Feed forward input angle, enable/disable current feedback
    ElectricalAngle => integrate speed to angle
*/
void Motor_FOC_StartOpenLoop(Motor_State_T * p_motor)
{
    // Ramp_SetOutputState(&p_motor->OpenLoopIRamp, Motor_GetIAlign(&p_motor->Config) * p_motor->Direction);  /* continue from align current */
    Ramp_SetOutputState(&p_motor->OpenLoopIRamp, 0);
    Ramp_SetOutputState(&p_motor->OpenLoopSpeedRamp, 0);
    /* continue from align angle State, in case it is not 0/A */
}

/*
    ElectricalAngle += (RPM * PolePairs * ANGLE16_PER_REV) / (60 * CONTROL_FREQ)
*/
void Motor_FOC_ProcOpenLoop(Motor_State_T * p_motor)
{
    fract16_t speed = Ramp_ProcNextOf(&p_motor->OpenLoopSpeedRamp, (int32_t)p_motor->Config.OpenLoopRampSpeedFinal_Fract16 * p_motor->Direction);
    angle16_t angle = Angle_IntegrateSpeed_Fract16(&p_motor->OpenLoopAngle, &p_motor->OpenLoopSpeedRef, speed);
    Motor_FOC_AngleControl(p_motor, angle, 0, Ramp_ProcNextOf(&p_motor->OpenLoopIRamp, (int32_t)p_motor->Config.OpenLoopRampIFinal_Fract16 * p_motor->Direction));
}



// static inline void ProcClarkePark(Motor_State_T * p_motor)
// {
// #if     defined(MOTOR_I_SENSORS_AB)
//     FOC_ProcClarkePark_AB(&p_motor->Foc);
// #elif   defined(MOTOR_I_SENSORS_ABC)
//     FOC_ProcClarkePark(&p_motor->Foc);
// #endif
// }
// #if defined(MOTOR_DECOUPLE_ENABLE)
// /*
//     Cross-coupling decoupling feedforward, limit-first ordering.
//         Vd_ff = -omega_e * Lq * Iq
//         Vq_ff = +omega_e * Ld * Id + omega_e * psi_f

//     omega_e source: RotorSensor_GetElectricalDelta (electrical angle16 per control cycle).
//     K coefficients (fract16) are tuned such that fract16_mul(delta, K_*) yields the decoupling
//     product in the same fract16 voltage basis as the PI output.
// */
// static void ProcIFeedback(Motor_State_T * p_motor, int16_t idReq, int16_t iqReq)
// {
//     fract16_t id = FOC_Id(&p_motor->Foc);
//     fract16_t iq = FOC_Iq(&p_motor->Foc);
//     angle16_t delta = RotorSensor_GetElectricalDelta(p_motor->p_ActiveSensor);

//     fract16_t omega_Lq = fract16_mul(delta, p_motor->Config.Decouple_KLq_Fract16);
//     fract16_t omega_Ld = fract16_mul(delta, p_motor->Config.Decouple_KLd_Fract16);
//     fract16_t omega_psi = fract16_mul(delta, p_motor->Config.Decouple_KPsi_Fract16);

//     /* Vd = PI_Id + Vd_ff, clipped to phase limit so Vq budget is well-defined */
//     int16_t vPhaseLimit = fract16_mul(Phase_VBus_Fract16(), FRACT16_1_DIV_2);
//     fract16_t vd = foc_decouple_vd(PID_ProcPI(&p_motor->PidId, id, idReq), iq, omega_Lq);
//     vd = math_clamp(vd, -vPhaseLimit, vPhaseLimit);
//     FOC_SetVd(&p_motor->Foc, vd);

//     /* Vq PI budget = remaining circle after Vd, with room reserved for Vq_ff */
//     int16_t vqBudget = _FOC_VqCircleLimit(&p_motor->Foc, vPhaseLimit);
//     fract16_t vq_ff = foc_vq_ff(id, omega_Ld, omega_psi);
//     int16_t vqPiLimit = vqBudget - math_abs(vq_ff);
//     if (vqPiLimit < 0) { vqPiLimit = 0; }

//     PID_CaptureOutputLimits(&p_motor->PidIq, (p_motor->Direction == MOTOR_DIRECTION_CW) * -vqPiLimit, (p_motor->Direction == MOTOR_DIRECTION_CCW) * vqPiLimit);
//     FOC_SetVq(&p_motor->Foc, fract16_sat((accum32_t)PID_ProcPI(&p_motor->PidIq, iq, iqReq) + vq_ff));
// }
// #else