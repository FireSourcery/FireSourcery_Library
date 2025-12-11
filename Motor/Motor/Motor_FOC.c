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
    Common
*/
/******************************************************************************/
/* From Iabc to Idq */
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

/******************************************************************************/
/*!
    Feedback Loops
*/
/******************************************************************************/
/*
    Current Feedback Loop
*/
static void ProcIFeedback(Motor_State_T * p_motor, int16_t idReq, int16_t iqReq)
{
    FOC_SetVd(&p_motor->Foc, PID_ProcPI(&p_motor->PidId, FOC_GetId(&p_motor->Foc), idReq));
    FOC_SetVq(&p_motor->Foc, PID_ProcPI(&p_motor->PidIq, FOC_GetIq(&p_motor->Foc), iqReq)); /* PidIq configured with VLimits */
    // FOC_ProcVectorLimit(&p_motor->Foc, Phase_VBus_Fract16());
    /* the combine output state can still grow outside of circle limit. limit after proc may still have windup */
    /* clamp if limited. sqrt operation on clamp only */
    if (FOC_ProcVectorLimit(&p_motor->Foc, Phase_VBus_Fract16()) == true)
    {
        PID_SetOutputLimits(&p_motor->PidIq, _Motor_VClampCwOf(p_motor, FOC_GetVq(&p_motor->Foc)), _Motor_VClampCcwOf(p_motor, FOC_GetVq(&p_motor->Foc)));
    }
}

/* apply limit first */
// static void ProcIFeedback(Motor_State_T * p_motor, int16_t idReq, int16_t iqReq)
// {
//     FOC_SetVd(&p_motor->Foc, PID_ProcPI(&p_motor->PidId, FOC_GetId(&p_motor->Foc), idReq));
//     if (FOC_ProcVectorLimit(&p_motor->Foc, Phase_VBus_Fract16()))
//         { _PID_SetOutputLimits(&p_motor->PidIq, -FOC_GetVq(&p_motor->Foc), FOC_GetVq(&p_motor->Foc)); }
//     FOC_SetVq(&p_motor->Foc, PID_ProcPI(&p_motor->PidIq, FOC_GetIq(&p_motor->Foc), iqReq)); /* PidIq configured with VLimits */
// }

/* From Iabc to Idq/IdqReq to Vdq */
static void ProcInnerFeedback(Motor_State_T * p_motor, angle16_t theta, int16_t dReq, int16_t qReq)
{
    FOC_SetTheta(&p_motor->Foc, theta);

    if (CaptureIabc(p_motor) == true)    /* else update angle only, until next cycle */
    {
        if (p_motor->FeedbackMode.Current == 1U)  /* Current Control mode */
        {
            // ProcIFeedback(p_motor, p_motor->Foc.ReqD, p_motor->Foc.ReqQ);
            ProcIFeedback(p_motor, dReq, qReq);
        }
        else /* if (p_motor->FeedbackMode.Current == 0U)  Voltage Control mode - Apply limits only */
        {
            FOC_SetVq(&p_motor->Foc, qReq);
            FOC_SetVd(&p_motor->Foc, dReq);
            FOC_ProcVectorLimit(&p_motor->Foc, Phase_VBus_Fract16());
            /* Still check CaptureIabc for overcurrent */
            /* todo on decrement */
            // ProcIFeedback(p_motor, 0, Motor_FOC_GetILimit(p_motor) * p_motor->Direction);
            // if (Motor_FOC_IsILimitReached(p_motor) == true)
            // {
            //     FOC_SetVq(&p_motor->Foc, PID_GetOutput(&p_motor->PidIq));
            //     FOC_SetVd(&p_motor->Foc, PID_GetOutput(&p_motor->PidId));
            // }
            // else
            // {
                // FOC_SetVq(&p_motor->Foc, p_motor->Foc.ReqQ);
                // FOC_SetVd(&p_motor->Foc, p_motor->Foc.ReqD);
            // }
        }
    }
}

/******************************************************************************/
/*!

*/
/******************************************************************************/
/* From Vdq to Vabc Duty */
static void ProcAngleOutput(Motor_State_T * p_motor)
{
    FOC_ProcOutputV_VBusInv(&p_motor->Foc, Phase_VBus_Inv_Fract32());  /* Set Vabc, VDuty */
}


/******************************************************************************/
/*!

*/
/******************************************************************************/
/*
    Activate angle with or without current feedback
    for align and openloop
*/
/* ProcInnerFeedback */
void Motor_FOC_ProcAngleFeedforward(Motor_State_T * p_motor, angle16_t angle, fract16_t dReq, fract16_t qReq)
{
    ProcInnerFeedback(p_motor, angle, dReq, qReq);
    ProcAngleOutput(p_motor);
}

/*
    Feed forward voltage angle
    set once, no feedback
    SetVAngle
*/
void Motor_FOC_ProcAngleFeedforwardV(Motor_State_T * p_motor, angle16_t angle, fract16_t vd, fract16_t vq)
{
    FOC_SetTheta(&p_motor->Foc, angle);
    FOC_SetVd(&p_motor->Foc, vd);
    FOC_SetVq(&p_motor->Foc, vq);
    // circle limit here if needed
    ProcAngleOutput(p_motor);
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
#ifdef CONFIG_MOTOR_EXTERN_CONTROL_ENABLE
    Motor_ExternControl(p_motor);
#endif
    Motor_FOC_ProcAngleFeedforward(p_motor, p_motor->SensorState.AngleSpeed.Angle, 0, Motor_ProcTorqueRamp(p_motor));
    // ProcInnerFeedback(p_motor, p_motor->SensorState.AngleSpeed.Angle, 0, Motor_ProcTorqueRamp(p_motor));
    // ProcAngleOutput(p_motor);
}

/*
    ProcAngleObserve
    Angle Observe VBemf FreeWheel and Stop State
    Updates Vabc, Valpha, Vbeta, Vd, Vq
*/
void Motor_FOC_ProcCaptureAngleVBemf(Motor_State_T * p_motor)
{
    FOC_SetTheta(&p_motor->Foc, p_motor->SensorState.AngleSpeed.Angle);

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
    PID_Reset(&p_motor->PidIq);
    PID_Reset(&p_motor->PidId);
    PID_Reset(&p_motor->PidSpeed);
    Ramp_SetOutput(&p_motor->SpeedRamp, 0);
    Ramp_SetOutput(&p_motor->TorqueRamp, 0);
    Phase_Input_ClearI(&p_motor->PhaseInput);
    Phase_Input_ClearV(&p_motor->PhaseInput);
}

void Motor_FOC_MatchVoltageState(Motor_State_T * p_motor, int16_t vd, int16_t vq)
{
    int16_t qReq;

    if (p_motor->FeedbackMode.Current == 1U)
    {
        PID_SetOutputState(&p_motor->PidId, vd);
        PID_SetOutputState(&p_motor->PidIq, vq);
        qReq = FOC_GetIq(&p_motor->Foc); /* if transitioning without release into freewheel */
    }
    else
    {
        qReq = vq;
    }

    Motor_MatchSpeedTorqueState(p_motor, qReq);
}

/*!
    Match Feedback State/Ouput to Output Voltage
    On FeedbackMode update and Freewheel to Run
    StateMachine blocks feedback proc
*/
void Motor_FOC_MatchFeedbackState(Motor_State_T * p_motor)
{
    // Motor_FOC_MatchVoltageState(p_motor, 0, Motor_GetVSpeed_Fract16(p_motor)); /* match without ad sampling */
    Motor_FOC_MatchVoltageState(p_motor, FOC_GetVd(&p_motor->Foc), FOC_GetVq(&p_motor->Foc)); /* Using Bemf capture */
    // Motor_FOC_MatchVoltageState(p_motor, 0, FOC_GetVPhase(&p_motor->Foc)); /* Using Bemf capture */
}


/******************************************************************************/
/*!
    FOC Direction
*/
/******************************************************************************/
/*
    Set on Direction change
    Iq/Id PID always Vq/Vd. Clip opposite user direction range, no plugging.

    p_motor->Direction is set by caller
*/
void _Motor_FOC_ApplyVLimits(Motor_State_T * p_motor, int16_t vRef)
{
    PID_SetOutputLimits(&p_motor->PidIq, 0 - _Motor_VClampCwOf(p_motor, vRef), _Motor_VClampCcwOf(p_motor, vRef));
    PID_SetOutputLimits(&p_motor->PidId, 0 - vRef, vRef);
}

void Motor_FOC_ApplyVLimits(Motor_State_T * p_motor)
{
    _Motor_FOC_ApplyVLimits(p_motor, Phase_VBus_GetVRefSvpwm());
}

void Motor_FOC_SetDirection(Motor_State_T * p_motor, Motor_Direction_T direction)
{
    Motor_SetDirection(p_motor, direction); /* alternatively caller handle */
    Motor_FOC_ApplyVLimits(p_motor);
}

// void Motor_FOC_SetDirectionForward(Motor_State_T * p_motor) { Motor_FOC_SetDirection(p_motor, p_motor->Config.DirectionForward); }


/******************************************************************************/
/*!
    Open Loop
    StateMachine mapping
*/
/******************************************************************************/
/* Align using user cmd value */
/* ElectricalAngle set by caller */
void Motor_FOC_StartAlignCmd(Motor_State_T * p_motor)
{
    p_motor->FeedbackMode.Current = 1U; /* config alternatively */
    Motor_FOC_ClearFeedbackState(p_motor); /* reset TorqueRamp i/v to start at 0 */
    Angle_ZeroCaptureState(&p_motor->OpenLoopAngle);
}

/* User Input ramp */
void Motor_FOC_ProcAlignCmd(Motor_State_T * p_motor)
{
    Motor_FOC_ProcAngleFeedforward(p_motor, p_motor->OpenLoopAngle.Angle, Motor_ProcTorqueRampOpenLoop(p_motor), 0);
}

/******************************************************************************/
/*!
    Open Loop Run
    alternatively use seperate angle
*/
/******************************************************************************/
/*
    preset align [OpenLoopIRamp]
    Ramp towards Preset AlignScalar_Fract16 * IRatedPeak
    Caller sets time
*/
void Motor_FOC_StartStartUpAlign(Motor_State_T * p_motor)
{
    Ramp_Set(&p_motor->OpenLoopIRamp, p_motor->Config.AlignTime_Cycles, 0, Motor_GetIAlign(p_motor)); /* Ramp d always positive */
    p_motor->FeedbackMode.Current = 1U; /* StartUp always select current feedback */

    Motor_FOC_ClearFeedbackState(p_motor);
    Angle_ZeroCaptureState(&p_motor->OpenLoopAngle);
}

void Motor_FOC_ProcStartUpAlign(Motor_State_T * p_motor)
{
    Motor_FOC_ProcAngleFeedforward(p_motor, p_motor->OpenLoopAngle.Angle, Ramp_ProcOutput(&p_motor->OpenLoopIRamp), 0);
}

/*
    OpenLoop Spin - Feed forward input angle, enable/disable current feedback
    ElectricalAngle => integrate speed to angle
*/
void Motor_FOC_StartOpenLoop(Motor_State_T * p_motor)
{
    Ramp_Set(&p_motor->OpenLoopIRamp, p_motor->Config.OpenLoopRampITime_Cycles, 0, (int32_t)p_motor->Config.OpenLoopRampIFinal_Fract16 * p_motor->Direction); /* reset slope set by align */
    // Ramp_SetTarget(&p_motor->OpenLoopIRamp, p_motor->Config.OpenLoopRampIFinal_Fract16 * p_motor->Direction); /* continue from align current */

    Ramp_SetOutput(&p_motor->OpenLoopSpeedRamp, 0);
    Ramp_SetTarget(&p_motor->OpenLoopSpeedRamp, (int32_t)p_motor->Config.OpenLoopRampSpeedFinal_Fract16 * p_motor->Direction);

    Angle_ZeroCaptureState(&p_motor->OpenLoopAngle);
}

/*
    ElectricalAngle += (RPM * PolePairs * ANGLE16_PER_REV) / (60 * CONTROL_FREQ)
*/
void Motor_FOC_ProcOpenLoop(Motor_State_T * p_motor)
{
    Angle_SetFeedforwardSpeed_Fract16(&p_motor->OpenLoopAngle, Ramp_ProcOutput(&p_motor->OpenLoopSpeedRamp));
    // p_angle->MechanicalAngle += (delta_degPerCycle / p_angle->Config.PolePairs);
    Motor_FOC_ProcAngleFeedforward(p_motor, p_motor->OpenLoopAngle.Angle, 0, Ramp_ProcOutput(&p_motor->OpenLoopIRamp));
}


// static inline void Motor_FOC_CaptureIa(Motor_State_T * p_motor) { FOC_SetIa(&p_motor->Foc, ((int32_t)Phase_Input_GetIa_Fract16(&p_motor->PhaseInput) + FOC_GetIa(&p_motor->Foc)) / 2); }
// static inline void Motor_FOC_CaptureIb(Motor_State_T * p_motor) { FOC_SetIb(&p_motor->Foc, ((int32_t)Phase_Input_GetIb_Fract16(&p_motor->PhaseInput) + FOC_GetIb(&p_motor->Foc)) / 2); }
// static inline void Motor_FOC_CaptureIc(Motor_State_T * p_motor) { FOC_SetIc(&p_motor->Foc, ((int32_t)Phase_Input_GetIc_Fract16(&p_motor->PhaseInput) + FOC_GetIc(&p_motor->Foc)) / 2); }
// static inline void Motor_FOC_CaptureVa(Motor_State_T * p_motor) { FOC_SetVBemfA(&p_motor->Foc, ((int32_t)Phase_Input_GetVa_Fract16(&p_motor->PhaseInput) + FOC_GetVBemfA(&p_motor->Foc)) / 2); }
// static inline void Motor_FOC_CaptureVb(Motor_State_T * p_motor) { FOC_SetVBemfB(&p_motor->Foc, ((int32_t)Phase_Input_GetVb_Fract16(&p_motor->PhaseInput) + FOC_GetVBemfB(&p_motor->Foc)) / 2); }
// static inline void Motor_FOC_CaptureVc(Motor_State_T * p_motor) { FOC_SetVBemfC(&p_motor->Foc, ((int32_t)Phase_Input_GetVc_Fract16(&p_motor->PhaseInput) + FOC_GetVBemfC(&p_motor->Foc)) / 2); }

// static inline void ProcClarkePark(Motor_State_T * p_motor)
// {
// #if     defined(CONFIG_MOTOR_I_SENSORS_AB)
//     FOC_ProcClarkePark_AB(&p_motor->Foc);
// #elif   defined(CONFIG_MOTOR_I_SENSORS_ABC)
//     FOC_ProcClarkePark(&p_motor->Foc);
// #endif
// }
