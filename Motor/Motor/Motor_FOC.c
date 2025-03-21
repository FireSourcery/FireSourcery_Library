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
    Feedback Loops
*/
/******************************************************************************/
/*
    Current Feedback Loop

    input ReqQ

    Speed Mode, PID clamps IReq
    Current Mode, IReq clamped on set
*/

// static inline void ProcIFeedback(FOC_T * p_foc, PID_T * p_pidIq, PID_T * p_pidId)
// {
//     FOC_SetVq(p_foc, PID_ProcPI(p_pidIq, FOC_GetIq(p_foc), FOC_GetReqQ(p_foc))); /* PidIq configured with VLimits */
//     FOC_SetVd(p_foc, PID_ProcPI(p_pidId, FOC_GetId(p_foc), FOC_GetReqD(p_foc)));
// }

static inline void ProcIFeedback(Motor_T * p_motor)
{
    FOC_SetVq(&p_motor->Foc, PID_ProcPI(&p_motor->PidIq, FOC_GetIq(&p_motor->Foc), FOC_GetReqQ(&p_motor->Foc))); /* PidIq configured with VLimits */
    FOC_SetVd(&p_motor->Foc, PID_ProcPI(&p_motor->PidId, FOC_GetId(&p_motor->Foc), FOC_GetReqD(&p_motor->Foc)));
    // FOC_ProcVectorLimit(&p_motor->Foc); /* This does not have to apply when updating the angle only */
}

static inline void ProcILimitV(Motor_T * p_motor)
{
    FOC_SetVq(&p_motor->Foc, math_limit_magnitude(FOC_GetReqQ(&p_motor->Foc), Motor_FOC_GetILimit(p_motor), math_abs(FOC_GetIq(&p_motor->Foc))));
    FOC_SetVd(&p_motor->Foc, FOC_GetReqD(&p_motor->Foc));

    p_motor->StateFlags.ILimited = (FOC_GetVq(&p_motor->Foc) != FOC_GetReqQ(&p_motor->Foc)) ? 1U : 0U;
}

static inline void ProcVReq(Motor_T * p_motor)
{
    FOC_SetVq(&p_motor->Foc, FOC_GetReqQ(&p_motor->Foc));
    FOC_SetVd(&p_motor->Foc, FOC_GetReqD(&p_motor->Foc));
}

/*
    Speed Feedback Loop
    SpeedControl_Fract16 update ~1000Hz,
    Ramp input ~100Hz, Ramp proc output 20000Hz

    input   RampCmd[-32767:32767] - Speed_Fract16[-32767:32767] accepts over saturated inputs
    output  SpeedControl_Fract16[-32767:32767] => IqReq or VqReq
*/
static inline void ProcSpeedFeedback(Motor_T * p_motor)
{
    // Ramp_GetOutput(&p_motor->SpeedRamp); todo as speed freq
    FOC_SetReqQ(&p_motor->Foc, PID_ProcPI(&p_motor->PidSpeed, p_motor->Speed_Fract16, Ramp_GetOutput(&p_motor->Ramp)));
}

static inline void ProcSpeedLimitReqQ(Motor_T * p_motor)
{
    FOC_SetReqQ(&p_motor->Foc, math_limit_magnitude(Ramp_GetOutput(&p_motor->Ramp), Motor_GetSpeedLimit(p_motor), math_abs(p_motor->Speed_Fract16))); /* todo i input ramp limits */

    p_motor->StateFlags.SpeedLimited = (FOC_GetReqQ(&p_motor->Foc) != Ramp_GetOutput(&p_motor->Ramp)) ? 1U : 0U;
}

static inline void ProcReqQ(Motor_T * p_motor)
{
    FOC_SetReqQ(&p_motor->Foc, Ramp_GetOutput(&p_motor->Ramp));
}


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

/* From Ramp to ReqDQ */
static void ProcOuterFeedback(Motor_T * p_motor)
{
    bool hasSpeedFeedback = Motor_PollCaptureSpeed(p_motor);

    if (p_motor->FeedbackMode.Speed == 1U)
    {
        if (hasSpeedFeedback == true) { ProcSpeedFeedback(p_motor); }
    }
    else if (p_motor->FeedbackMode.Speed == 0U)
    {
        if (hasSpeedFeedback == true) { ProcSpeedLimitReqQ(p_motor); }
        else if (p_motor->StateFlags.SpeedLimited == 0U) { ProcReqQ(p_motor); }
    }

    FOC_SetReqD(&p_motor->Foc, 0);
}

static bool CaptureIabc(Motor_T * p_motor)
{
    bool isCaptureI = (p_motor->IFlags.Value == PHASE_ID_ABC);
    if (isCaptureI == true)  /* alternatively use batch callback */
    {
        Motor_FOC_CaptureIa(p_motor, p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_IA.P_STATE->Result);
        Motor_FOC_CaptureIb(p_motor, p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_IB.P_STATE->Result);
        Motor_FOC_CaptureIc(p_motor, p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_IC.P_STATE->Result);
        ProcClarkePark(p_motor);
        p_motor->IFlags.Value = 0U;
    }
    return isCaptureI;
}

/* From Iabc to Idq to Vdq */
static void ProcInnerFeedback(Motor_T * p_motor)
{
    bool hasIFeedback = CaptureIabc(p_motor);

    if (p_motor->FeedbackMode.Current == 1U)  /* Current Control mode - Proc FeedbackLoop, Iq Id set by ADC routine */
    {
        /* when speedMode == 0, clamp on req and limit update */
        // FOC_SetReqQ(&p_motor->Foc, Motor_IReqLimitOf(p_motor, FOC_GetReqQ(&p_motor->Foc))); /* alternatively, share common clamp outside of speed loop */
        if (hasIFeedback == true) { ProcIFeedback(p_motor); }
    }
    else if (p_motor->FeedbackMode.Current == 0U) /* Voltage Control mode - Apply limits without FeedbackLoop */
    {
        if (hasIFeedback == true) { ProcILimitV(p_motor); }
        else if (p_motor->StateFlags.ILimited == 0U) { ProcVReq(p_motor); }
    }
}

/* From Vdq to Vabc */
static void ProcAngleOutput(Motor_T * p_motor)
{
    FOC_ProcInvParkInvClarkeSvpwm(&p_motor->Foc);  /* Set Vabc */
    Phase_WriteDuty_Fract16(&p_motor->Phase, FOC_GetDutyA(&p_motor->Foc), FOC_GetDutyB(&p_motor->Foc), FOC_GetDutyC(&p_motor->Foc));
}

// void Motor_FOC_CaptureIabc(Motor_T * p_motor)
// {
//     CaptureIabc(p_motor);
// }

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


/*
    Feedback Control Loop
    StateMachine calls each PWM, ~20kHz
*/
void Motor_FOC_ProcAngleControl(Motor_T * p_motor)
{
    p_motor->ElectricalAngle = Motor_PollSensorAngle(p_motor);
    FOC_SetTheta(&p_motor->Foc, p_motor->ElectricalAngle);
    Ramp_ProcOutput(&p_motor->Ramp); /* alternatively split ramp proc freq */

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

    if (p_motor->VFlags.Value == PHASE_ID_ABC)
    {
        Motor_FOC_CaptureVa(p_motor, p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_VA.P_STATE->Result);
        Motor_FOC_CaptureVb(p_motor, p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_VB.P_STATE->Result);
        Motor_FOC_CaptureVc(p_motor, p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_VC.P_STATE->Result);
        FOC_ProcVBemfClarkePark(&p_motor->Foc);
        p_motor->VFlags.Value = 0U;
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
    Ramp_ZeroOutputState(&p_motor->Ramp);
    p_motor->IFlags.Value = 0U;
    p_motor->VFlags.Value = 0U;
}

/*!
    On FeedbackMode change and Freewheel to Run
    Match Feedback State/Ouput to Output Voltage
        Ramp, PID to Vd, Vq
    StateMachine blocks feedback proc
*/
void Motor_FOC_MatchFeedbackState(Motor_T * p_motor)
{
    // int16_t vq = FOC_GetVq(&p_motor->Foc) * 3/2;
    int32_t vq = Motor_GetVSpeed_Fract16(p_motor); // match without ad sampling
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

    if (p_motor->FeedbackMode.Speed == 1U)
    {
        PID_SetOutputState(&p_motor->PidSpeed, qReq);
        Ramp_SetOutputState(&p_motor->Ramp, p_motor->Speed_Fract16);
    }
    else
    {
        Ramp_SetOutputState(&p_motor->Ramp, qReq);
    }
}

// void Motor_UpdateIControlLimits(Motor_T * p_motor)
// {
//     // update on direction only for now
//     // if (p_motor->FeedbackMode.Current == 1U)
//     // {
//     //     if (p_motor->Direction == MOTOR_DIRECTION_CCW) { PID_SetOutputLimits(&p_motor->PidIq, 0, INT16_MAX); }
//     //     else { PID_SetOutputLimits(&p_motor->PidIq, INT16_MIN, 0); }
//     // }
//     // PID_SetOutputLimits(&p_motor->PidId, INT16_MIN / 2, INT16_MAX / 2);
// }


/******************************************************************************/
/*!
    FOC Direction
*/
/******************************************************************************/
/*
    Set on Direction change
    Iq/Id PID always Vq/Vd. Clip opposite user direction range, no plugging.
    Voltage FeedbackMode active during over current only.
*/
// void Motor_FOC_SetDirectionCcw(Motor_T * p_motor)
// {
//     Motor_SetDirectionCcw(p_motor);
//     PID_SetOutputLimits(&p_motor->PidIq, 0, INT16_MAX);
//     PID_SetOutputLimits(&p_motor->PidId, -FRACT16_1_DIV_SQRT3, FRACT16_1_DIV_SQRT3);
// }

// void Motor_FOC_SetDirectionCw(Motor_T * p_motor)
// {
//     Motor_SetDirectionCw(p_motor);
//     PID_SetOutputLimits(&p_motor->PidIq, INT16_MIN, 0);
//     PID_SetOutputLimits(&p_motor->PidId, -FRACT16_1_DIV_SQRT3, FRACT16_1_DIV_SQRT3);
// }

void Motor_FOC_SetDirection(Motor_T * p_motor, Motor_Direction_T direction)
{
    // if (direction == MOTOR_DIRECTION_CCW) { Motor_FOC_SetDirectionCcw(p_motor); } else { Motor_FOC_SetDirectionCw(p_motor); }

    // switch (direction)
    // {
    //     case MOTOR_DIRECTION_CCW: Motor_FOC_SetDirectionCcw(p_motor);   break;
    //     case MOTOR_DIRECTION_CW:  Motor_FOC_SetDirectionCw(p_motor);    break;
    //     default:            break;
    // }
    Motor_SetDirection(p_motor, direction);
    PID_SetOutputLimits(&p_motor->PidIq, _Motor_GetVLimitCw(p_motor), _Motor_GetVLimitCcw(p_motor));
    PID_SetOutputLimits(&p_motor->PidId, -FRACT16_1_DIV_SQRT3, FRACT16_1_DIV_SQRT3);
}

void Motor_FOC_SetDirection_Cast(Motor_T * p_motor, uint8_t direction)
{
    Motor_FOC_SetDirection(p_motor, (Motor_Direction_T)direction);
}

void Motor_FOC_SetDirectionForward(Motor_T * p_motor) { Motor_FOC_SetDirection(p_motor, p_motor->Config.DirectionForward); }


/******************************************************************************/
/*!
    StateMachine mapping
    defined as inline for StateMachine wrapper functions tdo
*/
/******************************************************************************/
/* Align with user cmd value */
void Motor_FOC_StartAlignCmd(Motor_T * p_motor)
{
    Phase_ActivateOutput(&p_motor->Phase);
    Ramp_SetOutput(&p_motor->Ramp, 0); /* reset the voltage to start at 0 */
    // p_motor->ElectricalAngle = angle;
}

/* User ramp */
void Motor_FOC_ProcAlignCmd(Motor_T * p_motor)
{
    Ramp_SetOutputState(&p_motor->Ramp, Ramp_GetTarget(&p_motor->Ramp)); /* disable ramp */
    Motor_FOC_ProcAngleFeedforward(p_motor, p_motor->ElectricalAngle, Ramp_GetTarget(&p_motor->Ramp), 0);
}

void Motor_FOC_StartStartUpAlign(Motor_T * p_motor)
{
    Timer_StartPeriod(&p_motor->ControlTimer, p_motor->Config.AlignTime_Cycles);
    Ramp_Set(&p_motor->AuxRamp, p_motor->Config.AlignTime_Cycles, 0, p_motor->Config.AlignPower_UFract16);
    p_motor->FeedbackMode.Current = 1U; /* config alternatively */
    p_motor->ElectricalAngle = 0U;
}

void Motor_FOC_ProcStartUpAlign(Motor_T * p_motor)
{
    Motor_FOC_ProcAngleFeedforward(p_motor, p_motor->ElectricalAngle, Ramp_ProcOutput(&p_motor->AuxRamp), 0);
}


/*
    OpenLoop Spin - Feed forward input angle, enable/disable current feedback
    ElectricalAngle => integrate speed to angle
*/
void Motor_FOC_StartOpenLoop(Motor_T * p_motor)
{
    p_motor->Speed_Fract16 = 0;
    Ramp_Set(&p_motor->AuxRamp, p_motor->Config.RampAccel_Cycles, 0, Motor_DirectionalValueOf(p_motor, p_motor->Config.OpenLoopPower_UFract16));    // alternatively, clamp user input ramp
    Ramp_Set(&p_motor->OpenLoopSpeedRamp, p_motor->Config.OpenLoopAccel_Cycles, 0, Motor_DirectionalValueOf(p_motor, p_motor->Config.OpenLoopSpeed_UFract16)); // use get target to
}

// void Motor_FOC_StartOpenLoopSpeed(Motor_T * p_motor, uint16_t speed_ufract16)
// {
//     p_motor->Speed_Fract16 = 0;
//     Ramp_Set(&p_motor->AuxRamp, p_motor->Config.RampAccel_Cycles, 0, Motor_DirectionalValueOf(p_motor, p_motor->Config.OpenLoopPower_UFract16));
//     Ramp_Set(&p_motor->OpenLoopSpeedRamp, p_motor->Config.OpenLoopAccel_Cycles, 0, Motor_DirectionalValueOf(p_motor, speed_ufract16));
// }


/*
    RPM = Speed_Fract16 * SpeedFeedbackRef_Rpm / FRAC16_MAX
    ElectricalAngle += (RPM * FRAC16_MAX * PolePairs) / (60 * CONTROL_FREQ)
*/
void Motor_FOC_ProcOpenLoop(Motor_T * p_motor)
{
    uint16_t electricalAngle = p_motor->ElectricalAngle;

    p_motor->Speed_Fract16 = Ramp_ProcOutput(&p_motor->OpenLoopSpeedRamp);
    p_motor->ElectricalAngle += Motor_ElectricalAngleOfSpeed(p_motor, p_motor->Speed_Fract16);
    Motor_FOC_ProcAngleFeedforward(p_motor, p_motor->ElectricalAngle, 0, Ramp_ProcOutput(&p_motor->AuxRamp));

    p_motor->MechanicalAngle += (p_motor->ElectricalAngle - electricalAngle) / p_motor->Config.PolePairs;
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


