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

*/
/******************************************************************************/
void Motor_FOC_EnqueueVabc(Motor_T * p_motor)
{
    // todo move to threads
#if defined(CONFIG_MOTOR_V_SENSORS_ANALOG)
    if((p_motor->ControlTimerBase & MOTOR_STATIC.CONTROL_ANALOG_DIVIDER) == 0UL)
    {
        AnalogN_Group_PauseQueue(p_motor->CONST.P_ANALOG_N, p_motor->CONST.ANALOG_CONVERSIONS.ADCS_GROUP_V);
        AnalogN_Group_EnqueueConversion(p_motor->CONST.P_ANALOG_N, &p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_VA);
        AnalogN_Group_EnqueueConversion(p_motor->CONST.P_ANALOG_N, &p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_VB);
        AnalogN_Group_EnqueueConversion(p_motor->CONST.P_ANALOG_N, &p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_VC);
        AnalogN_Group_ResumeQueue(p_motor->CONST.P_ANALOG_N, p_motor->CONST.ANALOG_CONVERSIONS.ADCS_GROUP_V);
        // AnalogN_SetChannelConversion(p_motor->CONST.P_ANALOG_N, MOTOR_ANALOG_CHANNEL_VA);
    }
#else
    (void)p_motor;
#endif
}

void Motor_FOC_EnqueueIabc(Motor_T * p_motor)
{
    if((p_motor->ControlTimerBase & MOTOR_STATIC.CONTROL_ANALOG_DIVIDER) == 0UL)
    {
        AnalogN_Group_PauseQueue(p_motor->CONST.P_ANALOG_N, p_motor->CONST.ANALOG_CONVERSIONS.ADCS_GROUP_I);
        AnalogN_Group_EnqueueConversion(p_motor->CONST.P_ANALOG_N, &p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_IA);
        AnalogN_Group_EnqueueConversion(p_motor->CONST.P_ANALOG_N, &p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_IB);
#if defined(CONFIG_MOTOR_I_SENSORS_ABC)
        AnalogN_Group_EnqueueConversion(p_motor->CONST.P_ANALOG_N, &p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_IC);
#endif
        AnalogN_Group_ResumeQueue(p_motor->CONST.P_ANALOG_N, p_motor->CONST.ANALOG_CONVERSIONS.ADCS_GROUP_I);

    }
}


/******************************************************************************/
/*!
    Feedback Loops
*/
/******************************************************************************/
static inline void ProcInnerFeedback(Motor_T * p_motor)
{
    int32_t req = Motor_GetILimitReq(p_motor, FOC_GetQReq(&p_motor->Foc), FOC_GetIq(&p_motor->Foc));

    if(p_motor->FeedbackMode.Current == 1U) /* Current Control mode - proc using last adc measure */
    {
        FOC_SetVq(&p_motor->Foc, PID_ProcPI(&p_motor->PidIq, req, FOC_GetIq(&p_motor->Foc)));
        FOC_SetVd(&p_motor->Foc, PID_ProcPI(&p_motor->PidId, FOC_GetDReq(&p_motor->Foc), FOC_GetId(&p_motor->Foc)));
    }
    else /* Voltage Control mode - use current feedback for over current only */
    {
        FOC_SetVq(&p_motor->Foc, req);
        FOC_SetVd(&p_motor->Foc, FOC_GetDReq(&p_motor->Foc));
    }
}

/*
    Speed Feedback Loop
    SpeedControl_Frac16 update ~1000Hz, Ramp input 1000Hz, Ramp proc output 20000Hz
    input   RampCmd[-32767:32767] - Speed_Frac16[-32767:32767]
            accepts over saturated inputs
    output  SpeedControl_Frac16[-32767:32767] => IqReq or VqReq
*/
static inline void ProcOuterFeedback(Motor_T * p_motor)
{
    int32_t req = Motor_GetSpeedLimitReq(p_motor);

    if((Motor_ProcSensorSpeed(p_motor) == true) && (p_motor->FeedbackMode.Speed == 1U))
    {
        FOC_SetQReq(&p_motor->Foc, PID_ProcPI(&p_motor->PidSpeed, req, p_motor->Speed_Frac16));
        FOC_SetDReq(&p_motor->Foc, 0);
    }
    else if(p_motor->FeedbackMode.Speed == 0U) /* Current or Voltage Control mode */
    {
        FOC_SetQReq(&p_motor->Foc, req);
        FOC_SetDReq(&p_motor->Foc, 0);
    }
}

/*!
    Match Feedback Ouput to VOutput (Vd, Vq)
    Update PID state when changing FeedbackMode
*/
void Motor_FOC_ProcFeedbackMatch(Motor_T * p_motor)
{
    int32_t qReq;

    if(p_motor->FeedbackMode.Current == 1U)
    {
        PID_SetOutputState(&p_motor->PidIq, FOC_GetVq(&p_motor->Foc));
        PID_SetOutputState(&p_motor->PidId, FOC_GetVd(&p_motor->Foc));
        qReq = FOC_GetIq(&p_motor->Foc);
    }
    else
    {
        qReq = FOC_GetVq(&p_motor->Foc); /* alternatively use phase q_sqrt(vd vq) */
    }

    if(p_motor->FeedbackMode.Speed == 1U)
    {
        Linear_Ramp_SetOutputState(&p_motor->Ramp, p_motor->Speed_Frac16);
        PID_SetOutputState(&p_motor->PidSpeed, qReq);
    }
    else /* CONSTANT_CURRENT, or CONSTANT_VOLTAGE */
    {
        Linear_Ramp_SetOutputState(&p_motor->Ramp, qReq);
    }
}

/******************************************************************************/
/*!

*/
/******************************************************************************/
static inline void ProcClarkePark(Motor_T * p_motor)
{
#if     defined(CONFIG_MOTOR_I_SENSORS_AB)
    FOC_ProcClarkePark_AB(&p_motor->Foc);
#elif   defined(CONFIG_MOTOR_I_SENSORS_ABC)
    FOC_ProcClarkePark(&p_motor->Foc);
#endif
}

/* Vd Vq to DutyABC */
static void ActivateAngle(Motor_T * p_motor)
{
    FOC_ProcInvParkInvClarkeSvpwm(&p_motor->Foc);
    Phase_ActivateDuty(&p_motor->Phase, FOC_GetDutyA(&p_motor->Foc), FOC_GetDutyB(&p_motor->Foc), FOC_GetDutyC(&p_motor->Foc));
}

static void ProcInnerFeedbackOutput(Motor_T * p_motor)
{
    if((p_motor->ControlTimerBase & MOTOR_STATIC.CONTROL_ANALOG_DIVIDER) == 0UL)
    {
        ProcClarkePark(p_motor);
        ProcInnerFeedback(p_motor); /* Set Vd Vq */
    }
    ActivateAngle(p_motor);
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
    FOC_ZeroSvpwm(&p_motor->Foc);
    Phase_ActivateDuty(&p_motor->Phase, FOC_GetDutyA(&p_motor->Foc), FOC_GetDutyB(&p_motor->Foc), FOC_GetDutyC(&p_motor->Foc));
    Phase_ActivateOutputABC(&p_motor->Phase);
}

#ifdef CONFIG_MOTOR_EXTERN_CONTROL_ENABLE
extern void Motor_ExternControl(Motor_T * p_motor);
#endif

/*
    Feedback Control Loop
    StateMachine calls each PWM, ~20kHz
*/
void Motor_FOC_ProcAngleControl(Motor_T * p_motor)
{
    Motor_FOC_EnqueueIabc(p_motor);     /* Samples chain completes sometime after queue resumes. if ADC ISR priority higher than PWM. */

#ifdef CONFIG_MOTOR_EXTERN_CONTROL_ENABLE
    Motor_ExternControl(p_motor);
#endif
    // /* ~10us */ Motor_Debug_CaptureTime(p_motor, 1U);
    p_motor->ElectricalAngle = Motor_PollSensorAngle(p_motor);
    FOC_SetTheta(&p_motor->Foc, p_motor->ElectricalAngle);
    Linear_Ramp_ProcOutput(&p_motor->Ramp);
    ProcOuterFeedback(p_motor);
    // /* ~29 us */ Motor_Debug_CaptureTime(p_motor, 2U);

    ProcInnerFeedbackOutput(p_motor);
    // /* ~37us */ Motor_Debug_CaptureTime(p_motor, 4U);
}

/*
    activate angle with current feedback for align and openloop
    Super function antipattern, but meaningful
*/
void Motor_FOC_ProcAngleFeedforward(Motor_T * p_motor, qangle16_t angle, qfrac16_t dReq, qfrac16_t qReq)
{
    Motor_FOC_EnqueueIabc(p_motor);
    FOC_SetTheta(&p_motor->Foc, angle);
    FOC_SetDReq(&p_motor->Foc, dReq);
    FOC_SetQReq(&p_motor->Foc, qReq);
    ProcInnerFeedbackOutput(p_motor);
}

/*
    Angle Observe VBemf FreeWheel and Stop State
    Updates Vabc, Valphabeta, Vd, Vq
*/
void Motor_FOC_ProcAngleVBemf(Motor_T * p_motor)
{
    Motor_FOC_EnqueueVabc(p_motor);
    p_motor->ElectricalAngle = Motor_PollSensorAngle(p_motor);
    FOC_SetTheta(&p_motor->Foc, p_motor->ElectricalAngle);
    Motor_ProcSensorSpeed(p_motor);
    FOC_ProcVBemfClarkePark(&p_motor->Foc);
}

/* use speed as V match */
static inline void Motor_FOC_ProcVSpeed(Motor_T * p_motor)
{
    FOC_SetVq(&p_motor->Foc, Motor_GetVSpeed_Frac16(p_motor));
    FOC_SetVd(&p_motor->Foc, 0);
}


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
void Motor_FOC_SetDirectionCcw(Motor_T * p_motor)
{
    Motor_SetDirectionCcw(p_motor);
    PID_SetOutputLimits(&p_motor->PidIq, 0, INT16_MAX);
    PID_SetOutputLimits(&p_motor->PidId, INT16_MIN / 2, INT16_MAX / 2); /* Symmetrical for now */
}

void Motor_FOC_SetDirectionCw(Motor_T * p_motor)
{
    Motor_SetDirectionCw(p_motor);
    PID_SetOutputLimits(&p_motor->PidIq, INT16_MIN, 0);
    PID_SetOutputLimits(&p_motor->PidId, INT16_MIN / 2, INT16_MAX / 2);
}

void Motor_FOC_SetDirection(Motor_T * p_motor, Motor_Direction_T direction)
{
    if(direction == MOTOR_DIRECTION_CCW) { Motor_FOC_SetDirectionCcw(p_motor); } else { Motor_FOC_SetDirectionCw(p_motor); }
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
void Motor_FOC_StartAlign(Motor_T * p_motor)
{
    p_motor->FeedbackMode.Current = 1U;
    Linear_Ramp_Set(&p_motor->AuxRamp, p_motor->Config.AlignTime_Cycles, 0, p_motor->Config.AlignPower_Scalar16 / 2U);
    Motor_FOC_ProcFeedbackMatch(p_motor);
}

void Motor_FOC_ProcAlign(Motor_T * p_motor)
{
    Motor_FOC_ProcAngleFeedforward(p_motor, 0, Linear_Ramp_ProcOutput(&p_motor->AuxRamp), 0);
}

void Motor_FOC_StartAlignValidate(Motor_T * p_motor)
{
    // Motor_CalibrateSensorZero(p_motor);
    // Motor_ZeroSensor(p_motor);
    // FOC_SetDReq(&p_motor->Foc, 0);
    // // Linear_Ramp_Set(&p_motor->Ramp, p_motor->Config.RampAccel_Cycles, 0, Motor_DirectionalCmd(p_motor, INT16_MAX / 2U)); //clmap user input?
    // Motor_FOC_ProcFeedbackMatch(p_motor);
    // // p_motor->FeedbackMode.OpenLoop = 0U;
}

/*
    OpenLoop - Feed forward input angle, enable/disable current feedback
    ElectricalAngle => integrate speed to angle
*/
void Motor_FOC_StartOpenLoop(Motor_T * p_motor)
{
    p_motor->Speed_Frac16 = 0;
    Linear_Ramp_Set(&p_motor->AuxRamp, p_motor->Config.RampAccel_Cycles, 0, Motor_DirectionalValueOf(p_motor, p_motor->Config.OpenLoopPower_Scalar16 / 2U));    // alternatively, clamp user input ramp
    Linear_Ramp_Set(&p_motor->OpenLoopSpeedRamp, p_motor->Config.OpenLoopAccel_Cycles, 0, Motor_DirectionalValueOf(p_motor, p_motor->Config.OpenLoopSpeed_Scalar16 / 2U));
    // FOC_SetDReq(&p_motor->Foc, 0); //Motor_FOC_ProcAngleFeedforward
}

/*
    RPM = Speed_Frac16 * 2 * SpeedFeedbackRef_Rpm / 65535
    ElectricalAngle += (RPM * 65536 * PolePairs) / (60 * CONTROL_FREQ)
*/
static void _Motor_FOC_ProcOpenLoop(Motor_T * p_motor)
{
    p_motor->Speed_Frac16 = Linear_Ramp_ProcOutput(&p_motor->OpenLoopSpeedRamp);
    p_motor->ElectricalAngle += ((p_motor->Speed_Frac16 * p_motor->Config.SpeedFeedbackRef_Rpm * p_motor->Config.PolePairs * 2) / ((int32_t)60 * MOTOR_STATIC.CONTROL_FREQ));
}

void Motor_FOC_ProcOpenLoop(Motor_T * p_motor)
{
    _Motor_FOC_ProcOpenLoop(p_motor);
    Motor_FOC_ProcAngleFeedforward(p_motor, p_motor->ElectricalAngle, 0, Linear_Ramp_ProcOutput(&p_motor->AuxRamp));
}

/*
    Feed Forward Angle without ClarkPark on Current
*/
void Motor_FOC_ActivateAngle(Motor_T * p_motor, qangle16_t angle, qfrac16_t vq, qfrac16_t vd)
{
    FOC_SetVq(&p_motor->Foc, vq);
    FOC_SetVd(&p_motor->Foc, vd);
    FOC_SetTheta(&p_motor->Foc, angle);
    ActivateAngle(p_motor);
}
