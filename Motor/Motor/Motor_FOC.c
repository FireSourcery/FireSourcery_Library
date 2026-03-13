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
// static inlinebool _FOC_CaptureIabc(FOC_T * p_foc, Phase_Data_T * p_phaseData)
// {
//     if (p_phaseData->Flags.Bits == PHASE_ID_ABC)  /* alternatively use batch callback */
//     {
//         FOC_ProcClarkePark(p_foc, p_phaseData->Values.A, p_phaseData->Values.B, p_phaseData->Values.C);
//         p_phaseData->Flags.Bits = PHASE_ID_0; /* Clear capture flag after processing */
//         return true;
//     }
//     else
//     {
//         return false; /* No new data captured */
//     }
// }
// /* FeedbackState */
// static inline void  _FOC_CaptureFeedback(FOC_T * p_foc, Phase_Data_T * p_phaseData, angle16_t angle)
// {
//     FOC_SetTheta(p_foc, angle);
//     FOC_ProcClarkePark(p_foc, p_phaseData->Values.A, p_phaseData->Values.B, p_phaseData->Values.C);
// }

// /* Angle and Phase data set */
// /* limit req or Torque Ramp hold output limtis */
// static inline void  _FOC_TorqueControl(FOC_T * p_foc, PID_T * p_pidId, PID_T * p_pidIq, uint32_t vbus, Ramp_T * p_torqueRamp, fract16_t torqueReq)
// {
//     FOC_SetVd(p_foc, PID_ProcPI(p_pidId, FOC_GetId(p_foc), 0));
//     FOC_SetVq(p_foc, PID_ProcPI(p_pidIq, FOC_GetIq(p_foc), Ramp_ProcNextOf(p_torqueRamp, torqueReq)));
//     /* the combine output state can still grow outside of circle limit. limit after proc may still have windup */ /* propagate if limited.  */
//     if (FOC_ProcVectorLimit(p_foc, vbus) == true)
//     {
//         PID_SetOutputLimits(p_pidIq, VClampCwOf(FOC_GetVq(p_foc)), VClampCcwOf(FOC_GetVq(p_foc)));
//     }
// }

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
    /* the combine output state can still grow outside of circle limit. limit after proc may still have windup */ /* propagate if limited.  */
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
/* AngleControl */
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

void Motor_FOC_ProcTorqueReq(Motor_State_T * p_motor, fract16_t dReq, fract16_t qReq)
{
    Motor_FOC_ProcAngleFeedforward(p_motor, p_motor->SensorState.AngleSpeed.Angle, dReq, Motor_TorqueRampOf(p_motor, qReq));
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

    Motor_FOC_ProcAngleFeedforward(p_motor, p_motor->SensorState.AngleSpeed.Angle, 0, Motor_ProcTorqueRamp(p_motor));
    // _FOC_CaptureAngleState(&p_motor->Foc, &p_motor->PhaseInput, p_motor->SensorState.AngleSpeed.Angle);
    // ProcInnerFeedback(p_motor, p_motor->SensorState.AngleSpeed.Angle, 0, Motor_TorqueRampOf(p_motor, p_motor->userReq));
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
    // Ramp_SetOutputState(&p_motor->TorqueRamp, qReq);

    Motor_MatchSpeedTorqueState(p_motor, qReq);
}

/*!
    Match Feedback State/Ouput to Output Voltage
    On FeedbackMode update and Freewheel to Run
    StateMachine blocks feedback proc
*/
void Motor_FOC_MatchFeedbackState(Motor_State_T * p_motor)
{
    Motor_FOC_MatchVoltageState(p_motor, FOC_GetVd(&p_motor->Foc), FOC_GetVq(&p_motor->Foc)); /* Using Bemf capture */
    // Motor_FOC_MatchVoltageState(p_motor, 0, Motor_GetVSpeed_Fract16(p_motor)); /* match without ad sampling */
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
    // Angle_ZeroCaptureState(&p_motor->OpenLoopAngle);
}

/* User Input ramp */
void Motor_FOC_ProcAlignCmd(Motor_State_T * p_motor)
{
    int32_t userCmd = Ramp_GetTarget(&p_motor->TorqueRamp); // temp
    Motor_FOC_ProcAngleFeedforward(p_motor, p_motor->OpenLoopAngle.Angle, Motor_OpenLoopTorqueRampOf(p_motor, userCmd), 0);
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
    Motor_FOC_ProcAngleFeedforward(p_motor, p_motor->OpenLoopAngle.Angle, Motor_OpenLoopTorqueRampOf(p_motor, Motor_GetIAlign(p_motor)), 0); /*  */
}

/*
    OpenLoop Spin - Feed forward input angle, enable/disable current feedback
    ElectricalAngle => integrate speed to angle
*/
void Motor_FOC_StartOpenLoop(Motor_State_T * p_motor)
{
    // Ramp_SetOutput(&p_motor->OpenLoopIRamp, Motor_GetIAlign(p_motor) * p_motor->Direction);  /* continue from align current */
    Ramp_SetOutput(&p_motor->OpenLoopIRamp, 0);
    Ramp_SetOutput(&p_motor->OpenLoopSpeedRamp, 0);
    /* continue from align angle State, in case it is not 0/A */
}

/*
    ElectricalAngle += (RPM * PolePairs * ANGLE16_PER_REV) / (60 * CONTROL_FREQ)
*/
void Motor_FOC_ProcOpenLoop(Motor_State_T * p_motor)
{
    Angle_SetFeedforwardSpeed_Fract16(&p_motor->OpenLoopAngle, Ramp_ProcNextOf(&p_motor->OpenLoopSpeedRamp, (int32_t)p_motor->Config.OpenLoopRampSpeedFinal_Fract16 * p_motor->Direction));
    Motor_FOC_ProcAngleFeedforward(p_motor, p_motor->OpenLoopAngle.Angle, 0, Ramp_ProcNextOf(&p_motor->OpenLoopIRamp, (int32_t)p_motor->Config.OpenLoopRampIFinal_Fract16 * p_motor->Direction));
    // p_angle->MechanicalAngle += (delta_degPerCycle / p_angle->Config.PolePairs); sync sensor angle state
}



// static inline void ProcClarkePark(Motor_State_T * p_motor)
// {
// #if     defined(MOTOR_I_SENSORS_AB)
//     FOC_ProcClarkePark_AB(&p_motor->Foc);
// #elif   defined(MOTOR_I_SENSORS_ABC)
//     FOC_ProcClarkePark(&p_motor->Foc);
// #endif
// }
