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

// static inline void  _FOC_ProcIFeedbackLoop(FOC_T * p_foc, Phase_Data_T * p_phaseData, angle16_t angle, ufract16_t vBus, sign_t direction, int16_t dReq, int16_t qReq)
// {
//     FOC_SetTheta(p_foc, angle);
//     FOC_CaptureIabc(p_foc, p_phaseData);
//     FOC_ProcIFeedback(p_foc, vBus, direction, dReq, qReq);
//     FOC_ProcOutputDuty(p_foc, Phase_VBus_Inv_Fract32());
// }

// static inline interval_t Motor_GetVqLimits(const Motor_State_T * p_motor)
// {
//     int32_t window = Phase_VBus_GetVNominal() / 8; /* plugging clamped beyond 1/2 VPhaseRef. e.g 20VBus -> [-2.5, 2.5] or [0, 5] */
//     int32_t vSpeed = _Motor_GetVSpeed_Fract16(p_motor);
//     return (interval_t) { .low = vSpeed - window, .high = vSpeed + window, };
// }



// common call
/*
    Activate angle with or without current feedback
    Req dq to Vabc to Duty abc
*/
void Motor_FOC_AngleControl(Motor_State_T * p_motor, fract16_t vBus, angle16_t theta, fract16_t dReq, fract16_t qReq)
{
    FOC_SetTheta(&p_motor->Foc, theta);
    ProcInnerFeedback(p_motor, vBus, dReq, qReq);
}

/*
    Feed forward voltage angle
    set once, no feedback
    SetVAngle
*/
void Motor_FOC_ProcAngleFeedforwardV(Motor_State_T * p_motor, angle16_t theta, fract16_t vd, fract16_t vq)
{
    // FOC_CaptureIabc(&p_motor->Foc, &p_motor->PhaseInput.I); /* Update angle for feedforward, but ignore I feedback */
    FOC_FeedforwardAngleV(&p_motor->Foc, theta, vd, vq);
}


void Motor_FOC_ProcAngleAlignOf(Motor_State_T * p_motor, fract16_t vBus, angle16_t angle, fract16_t idReq)
{
    Motor_FOC_AngleControl(p_motor, vBus, angle, Ramp_ProcNextOf(&p_motor->TorqueRamp, math_clamp(idReq, 0, Motor_OpenLoopILimit(&p_motor->Config))), 0);
    // Motor_FOC_AngleControl(p_motor, angle, Ramp_ProcNextOf(&p_motor->TorqueRamp, Motor_OpenLoopILimitOf(&p_motor->Config, idReq)), 0);
}

// void Motor_FOC_ProcAngleAlign(Motor_State_T * p_motor, fract16_t vBus, angle16_t angle)
// {
//     Motor_FOC_ProcAngleAlignOf(p_motor, vBus, angle, Motor_GetIAlign(&p_motor->Config));
// }

// one compilation unit foc loop
// void ProcIControl(Motor_State_T * p_motor, int16_t vBus, angle16_t theta, int16_t dReq, int16_t qReq)
// {
//     FOC_SetTheta(&p_motor->Foc, theta);
//     if (FOC_CaptureIabc(&p_motor->Foc, &p_motor->PhaseInput.I) == true) /* else update angle only for voutput, until next cycle */
//     {
//         FOC_ProcIFeedback(&p_motor->Foc, vBus, (sign_t)p_motor->Direction, dReq, qReq);
//     }
//     FOC_ProcInvClarkePark(&p_motor->Foc);
// }

void _Motor_FOC_ProcTorqueReq(Motor_State_T * p_state, ufract16_t vbus, fract16_t req)
{
    FOC_SetTheta(&p_state->Foc, Angle_Value(&p_state->SensorState.AngleSpeed));
    if (FOC_CaptureIabc(&p_state->Foc, &p_state->PhaseInput.I) == true)
    {
        FOC_ProcIFeedback(&p_state->Foc, vbus, (sign_t)p_state->Direction, FOC_ProcIdFieldWeakening(&p_state->Foc, vbus), Ramp_ProcNextOnInputOf(&p_state->TorqueRamp, req));
    }
    FOC_ProcInvClarkePark(&p_state->Foc);
}

void Motor_FOC_ProcTorqueReq(Motor_T * p_motor, fract16_t req)
{
    _Motor_FOC_ProcTorqueReq(p_motor->P_MOTOR, VBus_Fract16(p_motor->P_VBUS), req);
}



/*
    TorqueRampV
    TorqueRampOpenLoop
*/
/* I/V rate mismatch, okay if voltage mode is for testing only */
/* Reads stored target; anti-plug applied per tick without persisting on Target. */
static inline fract16_t Motor_VRamp(Motor_State_T * p_state, ufract16_t vRefSvpwm)
{
    interval_t v = interval_of_sign((sign_t)p_state->Direction, vRefSvpwm);
    return _Ramp_ProcNextOnInputOf(&p_state->TorqueRamp, math_clamp(Ramp_GetTarget(&p_state->TorqueRamp), v.low, v.high));
    // return Ramp_ProcNext(&p_state->VRamp);
}

void Motor_FOC_ProcVControl(Motor_T * p_motor)
{
    Motor_State_T * p_state = p_motor->P_MOTOR;
    ufract16_t vbus = VBus_GetVPhaseRefSvpwm(p_motor->P_VBUS);
    interval_t v = interval_of_sign((sign_t)p_state->Direction, vbus);
    FOC_CaptureIabc(&p_state->Foc, &p_state->PhaseInput.I);
    FOC_SetVq(&p_state->Foc, _Ramp_ProcNextOnInputOf(&p_state->TorqueRamp, math_clamp(Ramp_GetTarget(&p_state->TorqueRamp), v.low, v.high)));
    FOC_SetVd(&p_state->Foc, 0);
    FOC_ProcVectorLimit(&p_state->Foc, vbus);
    FOC_ProcInvClarkePark(&p_state->Foc);
}

/******************************************************************************/

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

// void Motor_FOC_ProcAngleControl(Motor_State_T * p_motor, fract16_t vBus)
void Motor_FOC_ProcAngleControl(Motor_T * p_motor)
{
    Motor_State_T * p_state = p_motor->P_MOTOR;
    ufract16_t vbus = VBus_Fract16(p_motor->P_VBUS);
    int16_t qReq = (p_state->FeedbackMode.Current == 1U) ? Ramp_ProcNext(&p_state->TorqueRamp) : Motor_VRamp(p_state, VBus_GetVPhaseRefSvpwm(p_motor->P_VBUS));
    int16_t dReq = (p_state->FeedbackMode.Current == 1U) ? FOC_ProcIdFieldWeakening(&p_state->Foc, vbus) : 0;
    Motor_FOC_AngleControl(p_state, vbus, Angle_Value(&p_state->SensorState.AngleSpeed), dReq, qReq);

    // seperate, current only
}

/* applicable on batch callback */
// void Motor_FOC_ProcAngleControl(Motor_T * p_motor) {   Motor_FOC_ProcAngleControl(p_motor, VBus_Fract16(p_motor->P_VBUS)); }


// void Motor_FOC_ProcAngleTorque(Motor_T * p_motor)
// {
//     Motor_FOC_ProcTorqueReq(p_motor, Ramp_GetTarget(&p_motor->P_MOTOR->TorqueRamp));
// }

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
// void Motor_FOC_MatchIVState(Motor_State_T * p_motor)
// {
//     int16_t vqMatch = (FOC_Vq(&p_motor->Foc) == 0) ? Motor_GetVSpeed_Fract16(p_motor) : FOC_Vq(&p_motor->Foc);
//     FOC_MatchIVState(&p_motor->Foc, 0, vqMatch);
// }

/*!
    Match Feedback State/Output to Output Voltage
    On FeedbackMode update and Freewheel to Run
    StateMachine blocks feedback proc
*/
void Motor_FOC_MatchFeedbackState(Motor_T * p_motor)
{
    Motor_State_T * p_state = p_motor->P_MOTOR;
    if (p_state->FeedbackMode.Current == 1U)
    {
        // Motor_FOC_MatchIVState(p_motor);
        if (FOC_Vq(&p_state->Foc) == 0) /* from passive before bemf sample completes */
        {
            FOC_MatchIVState(&p_state->Foc, 0, Motor_GetVSpeed_Fract16(p_motor));
        }
        else
        {
            FOC_MatchIVState(&p_state->Foc, FOC_Vd(&p_state->Foc), FOC_Vq(&p_state->Foc));
        }

        Ramp_SetOutputState(&p_state->TorqueRamp, FOC_Iq(&p_state->Foc)); /*   transitioning without release into freewheel, math iq */
        Ramp_SetTarget(&p_state->TorqueRamp, FOC_Iq(&p_state->Foc));
    }
    else
    {
        Ramp_SetOutputState(&p_state->TorqueRamp, FOC_Vq(&p_state->Foc)); //different units for now
        Ramp_SetTarget(&p_state->TorqueRamp, FOC_Vq(&p_state->Foc));
    }

    Motor_MatchSpeedTorqueState(p_state, Ramp_GetOutput(&p_state->TorqueRamp)); // or let state machine handle
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
void Motor_FOC_SetAlignCmdAngle(Motor_State_T * p_motor, angle16_t angle)
{
    Angle_CaptureAngle(&p_motor->OpenLoopAngle, angle);
}

void Motor_FOC_StartAlignCmd(Motor_State_T * p_motor)
{
    p_motor->FeedbackMode.Current = 1U;
    Motor_FOC_ClearFeedbackState(p_motor); /* reset TorqueRamp i/v to start at 0 */
}

void Motor_FOC_ProcAlignCmd(Motor_T * p_motor)
{
    Motor_State_T * p_state = p_motor->P_MOTOR;
    // Motor_FOC_AngleControl(p_state, Angle_Value(&p_state->OpenLoopAngle), Motor_OpenLoopTorqueRampOf(p_state, Ramp_GetTarget(&p_state->TorqueRamp)), 0);
    Motor_FOC_ProcAngleAlignOf(p_state, VBus_Fract16(p_motor->P_VBUS), Angle_Value(&p_state->OpenLoopAngle), Ramp_GetTarget(&p_state->TorqueRamp));
    // Motor_FOC_ProcAngleAlign(p_state, VBus_Fract16(p_motor->P_VBUS), Angle_Value(&p_state->OpenLoopAngle));
}

// void Motor_FOC_ProcAlignCmdOf(Motor_State_T * p_motor, fract16_t idReq)
// {
//     // Motor_FOC_AngleControl(p_motor, Angle_Value(&p_motor->OpenLoopAngle), Motor_OpenLoopTorqueRampOf(p_motor, idReq), 0);
//     Motor_FOC_ProcAngleAlignOf(p_motor, Phase_VBus_Fract16(), Angle_Value(&p_motor->OpenLoopAngle), idReq);
// }


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

void Motor_FOC_ProcStartUpAlign(Motor_T * p_motor)
{
    Motor_State_T * p_state = p_motor->P_MOTOR;
    Motor_FOC_ProcAngleAlignOf(p_state, VBus_Fract16(p_motor->P_VBUS), Angle_Value(&p_state->OpenLoopAngle), Motor_GetIAlign(&p_state->Config));
}

/*
    OpenLoop Spin - Feed forward input angle, enable/disable current feedback
    ElectricalAngle => integrate speed to angle
*/
void Motor_FOC_StartOpenLoop(Motor_State_T * p_motor)
{
    Ramp_SetOutputState(&p_motor->OpenLoopIRamp, Motor_GetIAlign(&p_motor->Config) * p_motor->Direction);  /* continue from align current */
    // Ramp_SetOutputState(&p_motor->OpenLoopIRamp, 0);
    Ramp_SetOutputState(&p_motor->OpenLoopSpeedRamp, 0);
    /* continue from align angle State, in case it is not 0/A */
}

/*
    ElectricalAngle += (RPM * PolePairs * ANGLE16_PER_REV) / (60 * CONTROL_FREQ)
*/
void Motor_FOC_ProcOpenLoop(Motor_T * p_motor)
{
    Motor_State_T * p_state = p_motor->P_MOTOR;
    fract16_t speed = Ramp_ProcNextOf(&p_state->OpenLoopSpeedRamp, (int32_t)p_state->Config.OpenLoopRampSpeedFinal_Fract16 * p_state->Direction);
    angle16_t angle = Angle_IntegrateSpeed_Fract16(&p_state->OpenLoopAngle, &p_state->OpenLoopSpeedRef, speed);
    fract16_t iq = Ramp_ProcNextOf(&p_state->OpenLoopIRamp, (int32_t)p_state->Config.OpenLoopRampIFinal_Fract16 * p_state->Direction);
    Motor_FOC_AngleControl(p_state, VBus_Fract16(p_motor->P_VBUS), angle, 0, iq);
}



// static inline void ProcClarkePark(Motor_State_T * p_motor)
// {
// #if     defined(MOTOR_I_SENSORS_AB)
//     FOC_ProcClarkePark_AB(&p_motor->Foc);
// #elif   defined(MOTOR_I_SENSORS_ABC)
//     FOC_ProcClarkePark(&p_motor->Foc);
// #endif
// }



