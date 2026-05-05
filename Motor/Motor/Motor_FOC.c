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

void Motor_FOC_WriteDuty(Motor_T * p_motor)
{
    Phase_WriteSvpwm(&p_motor->PHASE, VBus_Inv_Fract32(p_motor->P_VBUS), FOC_Va(&p_motor->P_MOTOR->Foc), FOC_Vb(&p_motor->P_MOTOR->Foc), FOC_Vc(&p_motor->P_MOTOR->Foc));
}

// static inline interval_t Motor_GetVqLimits(const Motor_State_T * p_motor)
// {
//     int32_t window = Phase_VBus_GetVNominal() / 8; /* plugging clamped beyond 1/2 VPhaseRef. e.g 20VBus -> [-2.5, 2.5] or [0, 5] */
//     int32_t vSpeed = _Motor_GetVSpeed_Fract16(p_motor);
//     return (interval_t) { .low = vSpeed - window, .high = vSpeed + window, };
// }

/* Set Vdq */
static inline void ProcInnerFeedback(Motor_State_T * p_motor, int16_t vBus, int16_t dReq, int16_t qReq)
{
    if (FOC_CaptureIabc(&p_motor->Foc, &p_motor->PhaseInput.I) == true)    /* else update angle only for voutput, until next cycle */
    {
        if (p_motor->FeedbackMode.Current == 1U)  /* Current Control mode */
        {
            FOC_ProcIFeedback(&p_motor->Foc, vBus, (sign_t)p_motor->Direction, dReq, qReq);
        }
        else /* if (p_motor->FeedbackMode.Current == 0U)  Voltage Control mode - Apply limits only */
        {
            FOC_SetVq(&p_motor->Foc, qReq);
            FOC_SetVd(&p_motor->Foc, dReq);
            FOC_ProcVectorLimit(&p_motor->Foc, vBus);
            /* Still check CaptureIabc for overcurrent */
        }
    }
    FOC_ProcInvClarkePark(&p_motor->Foc);
}


// common call
/*
    Activate angle with or without current feedback
    Req dq to Vabc to Duty abc
*/
// void Motor_FOC_AngleControl(Motor_State_T * p_motor, vBus, angle16_t theta, fract16_t dReq, fract16_t qReq)
void Motor_FOC_AngleControl(Motor_State_T * p_motor, angle16_t theta, fract16_t dReq, fract16_t qReq)
{
    FOC_SetTheta(&p_motor->Foc, theta);
    ProcInnerFeedback(p_motor, Phase_VBus_Fract16(), dReq, qReq);
}

/*
    Feed forward voltage angle
    set once, no feedback
    SetVAngle
*/
void Motor_FOC_ProcAngleFeedforwardV(Motor_State_T * p_motor, angle16_t theta, fract16_t vd, fract16_t vq)
{
    FOC_FeedforwardAngleV(&p_motor->Foc, theta, vd, vq);
}



/******************************************************************************/
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
/* applicable on batch callback */
// void Motor_FOC_ProcAngleControl(Motor_T * p_motor)
// void Motor_FOC_ProcAngleControl(Motor_State_T * p_motor, VBus_T * p_vBus)

void Motor_FOC_ProcAngleControl(Motor_State_T * p_motor)
{
    int16_t qReq = (p_motor->FeedbackMode.Current == 1U) ? Ramp_ProcNext(&p_motor->TorqueRamp) : Motor_VRamp(p_motor);
    // int16_t dReq = (p_motor->FeedbackMode.Current == 1U) ? FOC_ProcIdFieldWeakening(&p_motor->Foc, Phase_VBus_Fract16()) : 0;
    Motor_FOC_AngleControl(p_motor, Angle_Value(&p_motor->SensorState.AngleSpeed), 0, qReq);
}


// void Motor_FOC_ProcAngleControl_Extern(Motor_State_T * p_motor)
// {
// #ifdef MOTOR_EXTERN_CONTROL_ENABLE
//     Motor_ExternControl(p_motor);
// #endif
//     int16_t qReq = (p_motor->FeedbackMode.Current == 1U) ? Ramp_ProcNext(&p_motor->TorqueRamp) : Motor_VRamp(p_motor);
//     Motor_FOC_AngleControl(p_motor, Angle_Value(&p_motor->SensorState.AngleSpeed), 0, qReq);
// }

/* alternate source analogous to Motor_ProcSpeedControlSource */
void Motor_FOC_ProcTorqueReq(Motor_State_T * p_motor, fract16_t qReq)
{
    int16_t dReq = (p_motor->FeedbackMode.Current == 1U) ? FOC_ProcIdFieldWeakening(&p_motor->Foc, Phase_VBus_Fract16()) : 0;
    Motor_FOC_AngleControl(p_motor, Angle_Value(&p_motor->SensorState.AngleSpeed), dReq, Ramp_ProcNextOnInputOf(&p_motor->TorqueRamp, qReq));

    // as i loop only
    // FOC_SetTheta(&p_motor->Foc, theta);

    // if (CaptureIabc(p_motor) == true)
    // {
    //     ProcIFeedback(p_motor, dReq, qReq);
    // }
}

/*
    ProcAngleObserve
    Angle Observe VBemf FreeWheel and Stop State
    Updates Vabc, Valpha, Vbeta, Vd, Vq
*/
void Motor_FOC_ProcCaptureAngleVBemf(Motor_State_T * p_motor)
{
    FOC_SetTheta(&p_motor->Foc, Angle_Value(&p_motor->SensorState.AngleSpeed));
    FOC_CaptureVBemf(&p_motor->Foc, &p_motor->PhaseInput.V);
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
    FOC_ResetFeedbackLoop(&p_motor->Foc);
    PID_Reset(&p_motor->PidSpeed);
    Ramp_SetOutputState(&p_motor->SpeedRamp, 0);
    Ramp_SetOutputState(&p_motor->TorqueRamp, 0);
    Phase_Input_ClearI(&p_motor->PhaseInput);
    Phase_Input_ClearV(&p_motor->PhaseInput);
    Ramp_SetTarget(&p_motor->TorqueRamp, 0);
    Ramp_SetTarget(&p_motor->SpeedRamp, 0);
}


/* torque only states */
void Motor_FOC_MatchIVState(Motor_State_T * p_motor)
{
    int16_t vqMatch = (FOC_Vq(&p_motor->Foc) == 0) ? Motor_GetVSpeed_Fract16(p_motor) : FOC_Vq(&p_motor->Foc);
    FOC_MatchIVState(&p_motor->Foc, 0, vqMatch);
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






// static void ProcIFeedback(Motor_State_T * p_motor, int16_t idReq, int16_t iqReq)
// {
//     FOC_SetVd(&p_motor->Foc, PID_ProcPI(&p_motor->Foc.PidId, FOC_Id(&p_motor->Foc), idReq));
//     int16_t vqLimit = _FOC_VqCircleLimit(&p_motor->Foc, fract16_mul(Phase_VBus_Fract16(), FRACT16_1_DIV_2));
//     interval_t band = interval_of_sign((sign_t)p_motor->Direction, vqLimit);
//     PID_CaptureOutputLimits(&p_motor->Foc.PidIq, band.low, band.high);
//     FOC_SetVq(&p_motor->Foc, PID_ProcPI(&p_motor->Foc.PidIq, FOC_Iq(&p_motor->Foc), iqReq));
// }

// static void ProcIFeedback_BackLimit(Motor_State_T * p_motor, int16_t idReq, int16_t iqReq)
// {
//     FOC_SetVd(&p_motor->Foc, PID_ProcPI(&p_motor->Foc.PidId, FOC_Id(&p_motor->Foc), idReq));
//     FOC_SetVq(&p_motor->Foc, PID_ProcPI(&p_motor->Foc.PidIq, FOC_Iq(&p_motor->Foc), iqReq));
//     /* the combine output state can still grow outside of circle limit. limit after proc may still have windup. propagate if limited. */
//     if (FOC_ProcVectorLimit(&p_motor->Foc, Phase_VBus_Fract16()) == true)
//     {
//         _PID_SetOutputState(&p_motor->Foc.PidIq, FOC_Vq(&p_motor->Foc)); // immediately saturates on input anamoly
//         // if (math_abs(FOC_Vq(&p_motor->Foc)) > PID_GetIntegral(&p_motor->Foc.PidIq)) { PID_SetOutputState(&p_motor->Foc.PidIq, FOC_Vq(&p_motor->Foc)); }
//     }
// }
