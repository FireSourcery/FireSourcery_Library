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
/*
    Current-mode angle control. Req Id/Iq to Vabc.
*/
void Motor_FOC_AngleControl(Motor_Context_T * p_motor, fract16_t vBus, angle16_t theta, fract16_t dReq, fract16_t qReq)
{
    FOC_SetTheta(&p_motor->Foc, theta);
#if (MOTOR_CONTROL_FREQ != MOTOR_I_LOOP_FREQ)  /* update angle only for voutput, until divider from adc is set */
    if (FOC_CaptureIabc(&p_motor->Foc, &p_motor->PhaseInput.I)) { FOC_ProcIFeedback(&p_motor->Foc, vBus, dReq, qReq); }
#else
    _FOC_CaptureIabc(&p_motor->Foc, &p_motor->PhaseInput.I);
    FOC_ProcIFeedback(&p_motor->Foc, vBus, dReq, qReq);
#endif
    FOC_ProcInvClarkePark(&p_motor->Foc);
}

/*
    Feed forward voltage angle
    set once, no feedback
    SetVAngle
*/
void Motor_FOC_ProcAngleFeedforwardV(Motor_Context_T * p_motor, angle16_t theta, fract16_t vd, fract16_t vq)
{
    FOC_CaptureIabc(&p_motor->Foc, &p_motor->PhaseInput.I); /* Update for measurements */
    FOC_FeedforwardAngleV(&p_motor->Foc, theta, vd, vq);
}

/* Common state machine call for torque */
void _Motor_FOC_ProcTorqueReq(Motor_Context_T * p_context, ufract16_t vbus, fract16_t req)
{
    Motor_FOC_AngleControl(p_context, vbus, Angle_Value(&p_context->SensorState.AngleSpeed), FOC_ProcIdFieldWeakening(&p_context->Foc, vbus), Ramp_ProcNextOf(&p_context->TorqueRamp, req));
    // FOC_ProcIFeedback_FieldWeakening(&p_context->Foc, VBus_Fract16(p_motor->P_VBUS), Ramp_ProcNext(&p_context->TorqueRamp));
}

void Motor_FOC_ProcTorqueReq(Motor_T * p_motor, fract16_t req)
{
    _Motor_FOC_ProcTorqueReq(p_motor->P_MOTOR, VBus_Fract16(p_motor->P_VBUS), req);
}

/* Common state machine call for align */
void _Motor_FOC_ProcAngleAlign(Motor_Context_T * p_motor, fract16_t vBus, angle16_t angle, fract16_t idReq)
{
    Motor_FOC_AngleControl(p_motor, vBus, angle, Ramp_ProcNextOf(&p_motor->TorqueRamp, math_clamp(idReq, 0, Motor_OpenLoopILimit(&p_motor->Config))), 0);
    // Motor_FOC_AngleControl(p_motor, vBus, angle, Ramp_ProcNextOf(&p_motor->TorqueRamp, math_clamp(idReq, 0, Motor_GetIAlign(&p_motor->Config))), 0);
}

void Motor_FOC_ProcAngleAlign(Motor_T * p_motor, angle16_t angle, fract16_t idReq)
{
    _Motor_FOC_ProcAngleAlign(p_motor->P_MOTOR, VBus_Fract16(p_motor->P_VBUS), angle, idReq);
}

/******************************************************************************/
/*!
    Run State
    Per-PWM-tick FOC pipelines.
    StateMachine picks one based on FeedbackMode.Current; no inner-loop dispatch.
*/
/******************************************************************************/
/*
    Voltage-mode on a seperate path
*/
void Motor_FOC_ProcVControl(Motor_T * p_motor)
{
    Motor_Context_T * p_context = p_motor->P_MOTOR;
    fract16_t vReq = _Ramp_ProcNextOf(&p_context->TorqueRamp, Ramp_GetTarget(&p_context->TorqueRamp)); /* inner loop clips with VLimit */
    FOC_SetTheta(&p_context->Foc, Angle_Value(&p_context->SensorState.AngleSpeed));
    FOC_CaptureIabc(&p_context->Foc, &p_context->PhaseInput.I); /* Still capture I for overcurrent */
    FOC_ProcVControl(&p_context->Foc, fract16_mul(VBus_Fract16(p_motor->P_VBUS), FRACT16_1_DIV_SQRT3), 0, vReq);
    FOC_ProcInvClarkePark(&p_context->Foc);
}

/*
    Current-mode pipeline. Sensor theta, TorqueRamp on Iq, Id from field weakening.
*/
void Motor_FOC_ProcAngleControl(Motor_T * p_motor)
{
    Motor_FOC_ProcTorqueReq(p_motor, Ramp_GetTarget(&p_motor->P_MOTOR->TorqueRamp));
}

/******************************************************************************/
/*
    Passive State
    ProcAngleObserve
    Angle Observe VBemf FreeWheel and Stop State
    Updates Vabc, Valpha, Vbeta, Vd, Vq
*/
/******************************************************************************/
void Motor_FOC_ProcCaptureAngleVBemf(Motor_Context_T * p_motor)
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
void Motor_FOC_ClearFeedbackState(Motor_Context_T * p_motor)
{
    FOC_ClearCaptureState(&p_motor->Foc); /* Clear for view, updated again on enter control */
    FOC_ClearOutputState(&p_motor->Foc); /* Emergency stop, capture bemf subsitute */
    FOC_ResetFeedbackState(&p_motor->Foc);
    PID_Reset(&p_motor->PidSpeed);
    Ramp_SetOutputState(&p_motor->SpeedRamp, 0);
    Ramp_SetOutputState(&p_motor->TorqueRamp, 0);
    Phase_Input_ClearI(&p_motor->PhaseInput);
    Phase_Input_ClearV(&p_motor->PhaseInput);
    Ramp_SetTarget(&p_motor->TorqueRamp, 0);
    Ramp_SetTarget(&p_motor->SpeedRamp, 0);
}

static inline int32_t Motor_FOC_VSpeed_Fract16(const Motor_Context_T * p_context) { return fract16_mul(Motor_GetSpeedFeedback(p_context), p_context->Foc.Electrical.Psi); }

/*!
    Match Feedback State/Output to Output Voltage
    On FeedbackMode update and Freewheel to Run
    StateMachine blocks feedback proc
*/
/* vq == 0 from passive before bemf sample completes */
/* Vabc is either 0 from clear on entry to PASSIVE or set by CaptureAngleVBemf */
void Motor_FOC_MatchTorqueIState(Motor_Context_T * p_context)
{
    Ramp_SetOutputState(&p_context->TorqueRamp, FOC_Iq(&p_context->Foc)); /* transitioning without release into freewheel, math iq */
    Ramp_SetTarget(&p_context->TorqueRamp, FOC_Iq(&p_context->Foc)); /*  may be ~1-50ms before next user input */
    // int16_t vqMatch = (FOC_Vq(&p_context->Foc) == 0) ? Motor_FOC_VSpeed_Fract16(p_context) : FOC_Vq(&p_context->Foc);
    int16_t vqMatch = Motor_FOC_VSpeed_Fract16(p_context);
    FOC_MatchIVState(&p_context->Foc, FOC_Vd(&p_context->Foc), vqMatch); /* Keep Vd for resume while Field Weakening */
    FOC_CaptureSpeed(&p_context->Foc, Motor_GetDecouplingOmega(p_context));
}

void Motor_FOC_MatchTorqueVState(Motor_Context_T * p_context)
{
    // int16_t vqMatch = (FOC_Vq(&p_context->Foc) == 0) ? Motor_FOC_VSpeed_Fract16(p_context) : FOC_Vq(&p_context->Foc);
    int16_t vqMatch = Motor_FOC_VSpeed_Fract16(p_context);
    Ramp_SetOutputState(&p_context->TorqueRamp, vqMatch); //different units for now
    Ramp_SetTarget(&p_context->TorqueRamp, vqMatch);
    FOC_SetVq(&p_context->Foc, vqMatch);/* repeat set is ok */
    FOC_CaptureSpeed(&p_context->Foc, Motor_GetDecouplingOmega(p_context));
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
void Motor_FOC_SetAlignCmdAngle(Motor_Context_T * p_motor, angle16_t angle)
{
    Angle_CaptureAngle(&p_motor->OpenLoopAngle, angle);
}

void Motor_FOC_StartAlignCmd(Motor_Context_T * p_motor)
{
    p_motor->FeedbackMode.Current = 1U;
    Motor_FOC_ClearFeedbackState(p_motor); /* reset TorqueRamp i/v to start at 0 */
}

void Motor_FOC_ProcAlignCmd(Motor_T * p_motor)
{
    Motor_Context_T * p_context = p_motor->P_MOTOR;
    _Motor_FOC_ProcAngleAlign(p_context, VBus_Fract16(p_motor->P_VBUS), Angle_Value(&p_context->OpenLoopAngle), Ramp_GetTarget(&p_context->TorqueRamp));
}


/******************************************************************************/
/*  */
/******************************************************************************/
/*
    Ramp towards Preset AlignScalar_Fract16 * IRatedPeak
    Caller sets time
*/
void Motor_FOC_StartStartUpAlign(Motor_Context_T * p_motor)
{
    p_motor->FeedbackMode.Current = 1U; /* StartUp always select current feedback */
    Motor_FOC_ClearFeedbackState(p_motor); /* Resets Ramp */
    Angle_ZeroCaptureState(&p_motor->OpenLoopAngle); /* always align A */
}

void Motor_FOC_ProcStartUpAlign(Motor_T * p_motor)
{
    Motor_Context_T * p_context = p_motor->P_MOTOR;
    _Motor_FOC_ProcAngleAlign(p_context, VBus_Fract16(p_motor->P_VBUS), Angle_Value(&p_context->OpenLoopAngle), Motor_GetIAlign(&p_context->Config));
}


/******************************************************************************/
/*  */
/******************************************************************************/
/*
    OpenLoop Spin - Feed forward input angle, enable/disable current feedback
    ElectricalAngle => integrate speed to angle
*/
void Motor_FOC_StartOpenLoop(Motor_Context_T * p_motor)
{
    Ramp_SetOutputState(&p_motor->OpenLoopIRamp, Motor_GetIAlign(&p_motor->Config) * p_motor->Direction);  /* continue from align current */
    // Ramp_SetOutputState(&p_motor->OpenLoopIRamp, 0);
    Ramp_SetOutputState(&p_motor->OpenLoopSpeedRamp, 0);
    /* continue from OpenLoopAngle */
}

/*
    ElectricalAngle += (RPM * PolePairs * ANGLE16_PER_REV) / (60 * CONTROL_FREQ)
*/
void Motor_FOC_ProcOpenLoop(Motor_T * p_motor)
{
    Motor_Context_T * p_context = p_motor->P_MOTOR;
    fract16_t speed = Ramp_ProcNextOf(&p_context->OpenLoopSpeedRamp, (int32_t)p_context->Config.OpenLoopRampSpeedFinal_Fract16 * p_context->Direction);
    angle16_t angle = Angle_IntegrateSpeed_Fract16(&p_context->OpenLoopAngle, &p_context->OpenLoopSpeedRef, speed);
    fract16_t iq = Ramp_ProcNextOf(&p_context->OpenLoopIRamp, (int32_t)p_context->Config.OpenLoopRampIFinal_Fract16 * p_context->Direction);
    Motor_FOC_AngleControl(p_context, VBus_Fract16(p_motor->P_VBUS), angle, 0, iq);

//     FOC_SetTheta(&p_context->Foc, angle);
//     if (FOC_CaptureIabc(&p_context->Foc, &p_context->PhaseInput.I) == true)
//     {
//         FOC_ProcIFeedback(&p_context->Foc, VBus_Fract16(p_motor->P_VBUS), 0, iq);
//     }
//     FOC_ProcInvClarkePark(&p_context->Foc);
}

// void Motor_FOC_ProcSensorless(Motor_T * p_motor)
// {
//     Motor_Context_T * p_context = p_motor->P_MOTOR;
//     FOC_SetTheta(&p_context->Foc, angle);
//     if (FOC_CaptureIabc(&p_context->Foc, &p_context->PhaseInput.I) == true)
//     {
//         FOC_ProcIFeedback(&p_context->Foc, VBus_Fract16(p_motor->P_VBUS), 0, iq);
//     }
//     // Step
//     FOC_ProcInvClarkePark(&p_context->Foc);
// }



// static inline void ProcClarkePark(Motor_Context_T * p_motor)
// {
// #if     defined(MOTOR_I_SENSORS_AB)
//     FOC_ProcClarkePark_AB(&p_motor->Foc);
// #elif   defined(MOTOR_I_SENSORS_ABC)
//     FOC_ProcClarkePark(&p_motor->Foc);
// #endif
// }



