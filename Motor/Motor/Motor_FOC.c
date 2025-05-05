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
    @version V0
*/
/******************************************************************************/
#include "Motor_FOC.h"


/******************************************************************************/
/*!
    Common
*/
/******************************************************************************/
/* From Iabc to Idq */
static inline void ProcClarkePark(Motor_T * p_motor)
{
#if     defined(CONFIG_MOTOR_I_SENSORS_AB)
    FOC_ProcClarkePark_AB(&p_motor->Foc);
#elif   defined(CONFIG_MOTOR_I_SENSORS_ABC)
    FOC_ProcClarkePark(&p_motor->Foc);
#endif
}

static bool CaptureIabc(Motor_T * p_motor)
{
    bool isCaptureI = (p_motor->IBatch.Value == PHASE_ID_ABC);
    if (isCaptureI == true)  /* alternatively use batch callback */
    {
        Motor_FOC_CaptureIa(p_motor);
        Motor_FOC_CaptureIb(p_motor);
        Motor_FOC_CaptureIc(p_motor);
        ProcClarkePark(p_motor);
        p_motor->IBatch.Value = 0U;
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

    input ReqQ

    Speed Mode, PID clamps IReq
    Current Mode, IReq clamped on set
*/
static void ProcIFeedback(Motor_T * p_motor, int16_t idReq, int16_t iqReq)
{
    FOC_SetVd(&p_motor->Foc, PID_ProcPI(&p_motor->PidId, FOC_GetId(&p_motor->Foc), idReq));
    FOC_SetVq(&p_motor->Foc, PID_ProcPI(&p_motor->PidIq, FOC_GetIq(&p_motor->Foc), iqReq)); /* PidIq configured with VLimits */
    FOC_ProcVectorLimit(&p_motor->Foc);
}

/* From Iabc to Idq to Vdq */
static void ProcInnerFeedback(Motor_T * p_motor)
{
    if (CaptureIabc(p_motor) == true)
    {
        if (p_motor->FeedbackMode.Current == 1U)  /* Current Control mode */
        {
            ProcIFeedback(p_motor, p_motor->Foc.ReqD, p_motor->Foc.ReqQ);
        }
        else if (p_motor->FeedbackMode.Current == 0U) /* Voltage Control mode - Apply limits only */
        {
            /* todo on decrement */
            // ProcIFeedback(p_motor, 0, Motor_FOC_GetILimit(p_motor) * p_motor->Direction); /* sign(p_motor->Foc.ReqQ)-sign(p_motor->Foc.Iq) */

            // if (Motor_FOC_IsILimitReached(p_motor) == true)
            // {
            //     FOC_SetVq(&p_motor->Foc, PID_GetOutput(&p_motor->PidIq));
            //     FOC_SetVd(&p_motor->Foc, PID_GetOutput(&p_motor->PidId));
            // }
            // else
            // {
            //     FOC_SetVq(&p_motor->Foc, p_motor->Foc.ReqQ);
            //     FOC_SetVd(&p_motor->Foc, p_motor->Foc.ReqD);
            // }
        }
    }
}

/*
    Speed Feedback

    input   RampCmd - Speed_Fract16. [-32767:32767]x2
    output  IqReq or VqReq
*/
/*
    Ramp input ~100Hz,
    Ramp proc output 1000Hz
    SpeedFeedback update 1000Hz,
    input   User SpeedRamp or TorqueRamp
    output  IqReq or VqReq
*/
/* From Input Ramp to ReqQ */
static void ProcOuterFeedback(Motor_T * p_motor)
{
    Motor_ProcOuterFeedback(p_motor);

    /* 20Khz */
    if (p_motor->FeedbackMode.Speed == 1U)
    {
        FOC_SetReqQ(&p_motor->Foc, PID_GetOutput(&p_motor->PidSpeed));
    }
    else
    {
        FOC_SetReqQ(&p_motor->Foc, Motor_ProcTorqueRamp(p_motor));
    }

    FOC_SetReqD(&p_motor->Foc, 0);
}


/******************************************************************************/
/*!

*/
/******************************************************************************/
/* From Vdq to Vabc */
static void ProcAngleOutput(Motor_T * p_motor)
{
    FOC_ProcOutputV(&p_motor->Foc, MotorAnalogRef_GetVSourceInvScalar());  /* Set Vabc */
    Phase_WriteDuty_Fract16(&p_motor->Phase, FOC_GetDutyA(&p_motor->Foc), FOC_GetDutyB(&p_motor->Foc), FOC_GetDutyC(&p_motor->Foc));
}


/******************************************************************************/
/*!

*/
/******************************************************************************/
/*
    Enables PWM Output - From Stop and Freewheel
*/
void Motor_FOC_ActivateOutput(Motor_T * p_motor)
{
    Phase_WriteDuty_Fract16(&p_motor->Phase, FOC_GetDutyA(&p_motor->Foc), FOC_GetDutyB(&p_motor->Foc), FOC_GetDutyC(&p_motor->Foc));
    Phase_ActivateOutput(&p_motor->Phase);
}

void Motor_FOC_ActivateOutputZero(Motor_T * p_motor)
{
    FOC_ZeroSvpwm(&p_motor->Foc);
    Motor_FOC_ActivateOutput(p_motor);
}

/* Update Vbus on start Angle Control */

/*
    Feedback Control Loop
    StateMachine calls each PWM, ~20kHz
*/
void Motor_FOC_ProcAngleControl(Motor_T * p_motor)
{
    p_motor->ElectricalAngle = Motor_PollSensorAngle(p_motor);
    FOC_SetTheta(&p_motor->Foc, p_motor->ElectricalAngle);

#ifdef CONFIG_MOTOR_EXTERN_CONTROL_ENABLE
    Motor_ExternControl(p_motor);
#endif
    ProcOuterFeedback(p_motor);
    ProcInnerFeedback(p_motor);
    ProcAngleOutput(p_motor);
}

/*
    ProcAngleObserve
    Angle Observe VBemf FreeWheel and Stop State
    Updates Vabc, Valpha, Vbeta, Vd, Vq
*/
void Motor_FOC_ProcCaptureAngleVBemf(Motor_T * p_motor)
{
    p_motor->ElectricalAngle = Motor_PollSensorAngle(p_motor);
    FOC_SetTheta(&p_motor->Foc, p_motor->ElectricalAngle);
    Motor_PollCaptureSpeed(p_motor);

    if (p_motor->VBatch.Value == PHASE_ID_ABC)
    {
        Motor_FOC_CaptureVa(p_motor);
        Motor_FOC_CaptureVb(p_motor);
        Motor_FOC_CaptureVc(p_motor);
        FOC_ProcVBemfClarkePark(&p_motor->Foc);
        p_motor->VBatch.Value = 0U;
    }
}

/*
    Activate angle with or without current feedback
    for align and openloop
*/
void Motor_FOC_ProcAngleFeedforward(Motor_T * p_motor, angle16_t angle, fract16_t dReq, fract16_t qReq)
{
    FOC_SetTheta(&p_motor->Foc, angle);
    FOC_SetReqD(&p_motor->Foc, dReq);
    FOC_SetReqQ(&p_motor->Foc, qReq);
    ProcInnerFeedback(p_motor);
    ProcAngleOutput(p_motor);
}

// void Motor_FOC_SetAngleFeedforward(Motor_T * p_motor, angle16_t angle, fract16_t dReq, fract16_t qReq)
// {
//     Motor_SetElecAngleFeedforward(p_motor, angle);
//     FOC_SetTheta(&p_motor->Foc, angle);
//     FOC_SetReqD(&p_motor->Foc, dReq);
//     FOC_SetReqQ(&p_motor->Foc, qReq);
// }


/*
    Feed forward voltage angle without feedback on current
*/
void Motor_FOC_ActivateAngle(Motor_T * p_motor, angle16_t angle, fract16_t vd, fract16_t vq)
{
    p_motor->ElectricalAngle = angle;
    FOC_SetTheta(&p_motor->Foc, angle);
    FOC_SetVd(&p_motor->Foc, vd);
    FOC_SetVq(&p_motor->Foc, vq);
    ProcAngleOutput(p_motor);
}

/* Begin Observe, Ifeedback not updated */
void Motor_FOC_ClearFeedbackState(Motor_T * p_motor)
{
    FOC_ClearControlState(&p_motor->Foc); /* Clear for view, updated again on enter control */
    PID_Reset(&p_motor->PidIq);
    PID_Reset(&p_motor->PidId);
    PID_Reset(&p_motor->PidSpeed);
    Ramp_ZeroOutputState(&p_motor->SpeedRamp);
    Ramp_ZeroOutputState(&p_motor->TorqueRamp);
    p_motor->IBatch.Value = 0U;
    p_motor->VBatch.Value = 0U;
}

/*!
    On FeedbackMode change and Freewheel to Run
    Match Feedback State/Ouput to Output Voltage
        Ramp, PID to Vd, Vq
    StateMachine blocks feedback proc
*/
void Motor_FOC_MatchFeedbackState(Motor_T * p_motor)
{
    // int32_t vq = Motor_GetVSpeed_Fract16(p_motor) / 2; // match without ad sampling
    int16_t vq = FOC_GetVMagnitude(&p_motor->Foc) * p_motor->Direction;
    int16_t qReq;

    if (p_motor->FeedbackMode.Current == 1U)
    {
        PID_SetOutputState(&p_motor->PidIq, vq);
        PID_SetOutputState(&p_motor->PidId, 0);
        qReq = FOC_GetIq(&p_motor->Foc); /* if transitioning without release into freewheel */
    }
    else
    {
        qReq = vq;
    }

    Motor_MatchSpeedTorqueState(p_motor, qReq);
}


/******************************************************************************/
/*!
    FOC Direction
*/
/******************************************************************************/
static inline int32_t Motor_FOC_GetVPhaseRef(void) { (fract16_mul(MotorAnalogRef_GetVSource_Fract16(), FRACT16_1_DIV_SQRT3)); }

static inline ufract16_t VLimitOf(const Motor_T * p_motor, Motor_Direction_T select) { return (p_motor->Direction == select) ? Motor_FOC_GetVPhaseRef() : 0; }

/*
    Set on Direction change
    Iq/Id PID always Vq/Vd. Clip opposite user direction range, no plugging.
    Voltage FeedbackMode active during over current only.
*/
void Motor_FOC_SetDirection(Motor_T * p_motor, Motor_Direction_T direction)
{
    Motor_SetDirection(p_motor, direction);
    PID_SetOutputLimits(&p_motor->PidIq, 0 - VLimitOf(p_motor, MOTOR_DIRECTION_CW), VLimitOf(p_motor, MOTOR_DIRECTION_CCW));
    PID_SetOutputLimits(&p_motor->PidId, 0 - Motor_FOC_GetVPhaseRef(), Motor_FOC_GetVPhaseRef());
    /* the combine output state can still grow outside of circle limit. limit after proc may still have windup */
}

void Motor_FOC_SetDirection_Cast(Motor_T * p_motor, int direction) { Motor_FOC_SetDirection(p_motor, (Motor_Direction_T)direction); }
void Motor_FOC_SetDirectionForward(Motor_T * p_motor) { Motor_FOC_SetDirection(p_motor, p_motor->Config.DirectionForward); }


/******************************************************************************/
/*!
    Open Loop
    StateMachine mapping
*/
/******************************************************************************/
/* Align with user cmd value */
void Motor_FOC_StartAlignCmd(Motor_T * p_motor)
{
    /* ElectricalAngle set by caller  */
    Phase_ActivateOutput(&p_motor->Phase);  /* reset the voltage to start at 0 */
    Ramp_Set(&p_motor->TorqueRamp, p_motor->Config.AlignTime_Cycles, 0, p_motor->Config.AlignPower_Fract16);
}

/* User ramp */
void Motor_FOC_ProcAlignCmd(Motor_T * p_motor)
{
    // Motor_IReqLimitOf
    Motor_FOC_ProcAngleFeedforward(p_motor, p_motor->ElectricalAngle, Ramp_ProcOutput(&p_motor->TorqueRamp), 0);
}

/*

*/
void Motor_FOC_StartStartUpAlign(Motor_T * p_motor)
{
    Timer_StartPeriod(&p_motor->ControlTimer, p_motor->Config.AlignTime_Cycles);
    Ramp_Set(&p_motor->OpenLoopIRamp, p_motor->Config.OpenLoopIRamp_Cycles, 0, p_motor->Config.OpenLoopIFinal_Fract16);
    p_motor->FeedbackMode.Current = 1U; /* config alternatively */
    p_motor->ElectricalAngle = 0U;
}

void Motor_FOC_ProcStartUpAlign(Motor_T * p_motor)
{
    Motor_FOC_ProcAngleFeedforward(p_motor, p_motor->ElectricalAngle, Ramp_ProcOutput(&p_motor->OpenLoopIRamp), 0);
}


/*
    OpenLoop Spin - Feed forward input angle, enable/disable current feedback
    ElectricalAngle => integrate speed to angle
*/
void Motor_FOC_StartOpenLoop(Motor_T * p_motor)
{
    p_motor->Speed_Fract16 = 0;
    Ramp_Set(&p_motor->OpenLoopIRamp, p_motor->Config.OpenLoopIRamp_Cycles, 0, Motor_DirectionalValueOf(p_motor, p_motor->Config.OpenLoopIFinal_Fract16));
    Ramp_Set(&p_motor->OpenLoopSpeedRamp, p_motor->Config.OpenLoopSpeedRamp_Cycles, 0, Motor_DirectionalValueOf(p_motor, p_motor->Config.OpenLoopSpeedFinal_Fract16));
}


/*
    RPM = Speed_Fract16 * SpeedFeedbackRef_Rpm / FRAC16_MAX
    ElectricalAngle += (RPM * FRAC16_MAX * PolePairs) / (60 * CONTROL_FREQ)
*/
void Motor_FOC_ProcOpenLoop(Motor_T * p_motor)
{
    p_motor->Speed_Fract16 = Ramp_ProcOutput(&p_motor->OpenLoopSpeedRamp);
    p_motor->MechanicalAngle += Motor_MechAngleOfSpeed_Fract16(p_motor, p_motor->Speed_Fract16);
    p_motor->ElectricalAngle = p_motor->MechanicalAngle * p_motor->Config.PolePairs;

    Motor_FOC_ProcAngleFeedforward(p_motor, p_motor->ElectricalAngle, 0, Ramp_ProcOutput(&p_motor->OpenLoopIRamp));
}

/*

*/
// void Motor_FOC_ProcOpenLoopAngle(Motor_T * p_motor)
// {
//     uint16_t electricalAngle = p_motor->ElectricalAngle;

//     p_motor->ElectricalAngle += angle;
//     Motor_FOC_ProcAngleFeedforward(p_motor, p_motor->ElectricalAngle, 0, Ramp_ProcOutput(&p_motor->AuxRamp));

//     p_motor->MechanicalAngle += (p_motor->ElectricalAngle - electricalAngle) / p_motor->Config.PolePairs;
// }


