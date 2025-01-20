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
    @file   Motor_User.c
    @author FireSourcery
    @version V0

    @brief
*/
/******************************************************************************/
#include "Motor_User.h"
#include "System/Critical/Critical.h"

/******************************************************************************/
/*!
    StateMachine handled inputs

    Motor State Machine Thread Safety
    State Proc in PWM thread.
        Includes _StateMachine_ProcStateTransition.
    User Input [Motor_User_StartControlMode] in Main thread.
        Inputs do not directly proc transition, set for sync proc

    Sync Mode -
        Must check input flags every control cycle.
        Multiple calls to [StateMachine_SetInput] overwrites within 1 control cycle may be lost (e.g within 1 packet)
            handle with input image buffer, input queue, spin-lock delegates to Xcvr buffer, or client side constraints

    Async Mode - Need critical section during input.
        No Critical during transistion -> Prev State Proc may overwrite new State Entry.

    CmdValue (Ramp Target) and CmdMode selectively sync inputs for StateMachine
*/
/******************************************************************************/


/******************************************************************************/
/*!
    On/Off Control
*/
/******************************************************************************/
/*
    Set [FeedbackMode] and Transition to Run State
*/
inline void Motor_User_StartControlMode(Motor_T * p_motor, Motor_FeedbackMode_T mode)
{
    //  (FeedbackMode_IsValid(mode));
    // StateMachine_ProcInput(&p_motor->StateMachine, MSM_INPUT_CONTROL, mode.Value);
    // StateMachine_SetInput(&p_motor->StateMachine, MSM_INPUT_CONTROL, mode.Value);
    StateMachine_ProcInput(&p_motor->StateMachine, MSM_INPUT_FEEDBACK_MODE, mode.Value); // disables interrupts twice
    StateMachine_ProcInput(&p_motor->StateMachine, MSM_INPUT_CONTROL_MODE, PHASE_STATE_ACTIVE);
}

/* Generic array functions use */
// inline void Motor_User_StartControl_Cast(Motor_T * p_motor, uint8_t modeValue)
// {
//     Motor_User_StartControlMode(p_motor, Motor_FeedbackMode_Cast(modeValue));
// }

/* combine process by statemachine */
// typedef union Motor_Cmd
// {
//     struct
//     {
//         uint32_t CmdValue       : 16U;
//         uint32_t FeedbackMode   : 8U;
//         uint32_t Activate       : 2U;
//     };
//     uint32_t Value;
// }
// Motor_Cmd_T;

inline void Motor_User_ActivateControl(Motor_T * p_motor)
{
    StateMachine_ProcInput(&p_motor->StateMachine, MSM_INPUT_CONTROL_MODE, PHASE_STATE_ACTIVE);
}

/*
    always State machine checked Disable
*/
inline void Motor_User_Release(Motor_T * p_motor)
{
    // StateMachine_SetInput(&p_motor->StateMachine, MSM_INPUT_RELEASE, STATE_MACHINE_INPUT_VALUE_NULL);
    StateMachine_SetInput(&p_motor->StateMachine, MSM_INPUT_CONTROL_MODE, PHASE_STATE_FLOAT);
}

inline void Motor_User_Hold(Motor_T * p_motor)
{
    // StateMachine_SetInput(&p_motor->StateMachine, MSM_INPUT_HOLD, STATE_MACHINE_INPUT_VALUE_NULL);
    StateMachine_SetInput(&p_motor->StateMachine, MSM_INPUT_CONTROL_MODE, PHASE_STATE_GROUND);
}

/*
    Force Disable control Non StateMachine checked
*/
void Motor_User_ForceDisableControl(Motor_T * p_motor)
{
    Phase_Float(&p_motor->Phase);
    // _Motor_User_SetCmd(p_motor, 0);
    Motor_User_Release(p_motor);
}

/*   User set [FeedbackMode] without starting Run */
inline void Motor_User_SetFeedbackMode(Motor_T * p_motor, Motor_FeedbackMode_T mode)
{
    StateMachine_SetInput(&p_motor->StateMachine, MSM_INPUT_FEEDBACK_MODE, mode.Value);
}

inline void Motor_User_SetFeedbackMode_Cast(Motor_T * p_motor, uint8_t modeValue)
{
    StateMachine_SetInput(&p_motor->StateMachine, MSM_INPUT_FEEDBACK_MODE, modeValue);
}

/*
    Bypasses User Mode restrictions, bounds
    Concurrency note: only 1 thread updates RampTarget. StateMachine_Proc thread only updates OutputState
    Ramp input allows over saturated input
*/
static inline void _Motor_User_SetCmd(Motor_T * p_motor, int32_t userCmd)
{
    if (p_motor->StateFlags.RampDisable == 0U) { Linear_Ramp_SetTarget(&p_motor->Ramp, Motor_DirectionalValueOf(p_motor, userCmd)); }
    else { Linear_Ramp_SetOutputState(&p_motor->Ramp, userCmd); }
}

/******************************************************************************/
/*!
    Feedback Control Modes: torque/speed/position, above Commutation layer: foc/six-step

    UserCmdValue functions
    UserCmdValue sign +/- indicates along or against Direction selected. NOT virtual CW/CCW.
        Handle direction and limits check on input.
        Limits check again in feedback loop, as input may have stopped
    Call regularly to update cmd value, lower frequency compared to control loop, ~100-1000Hz.

    SetMode       - Invokes StateMachine - Sets control mode only
    SetCmd[Value] - Without invoking StateMachine - Sets buffered cmd value, sets on all states even when inactive
*/
/******************************************************************************/
/******************************************************************************/
/*!
    Voltage Mode
*/
/******************************************************************************/
// Motor_User_SetFeedbackMode
void Motor_User_StartVoltageMode(Motor_T * p_motor) { Motor_User_StartControlMode(p_motor, MOTOR_FEEDBACK_MODE_VOLTAGE); }

/*!
    @param[in] voltage [-32768:32767]
*/
void Motor_User_SetVoltageCmd(Motor_T * p_motor, int16_t volts_fract16)
{
    _Motor_User_SetCmd(p_motor, math_max(volts_fract16, 0));  /* Reverse voltage use change direction, no plugging */
}

void Motor_User_SetVoltageCmd_Scalar(Motor_T * p_motor, int16_t scalar_fract16)
{
    int32_t limitedCmd = (scalar_fract16 > 0) ? fract16_mul(INT16_MAX, scalar_fract16) : 0;
    _Motor_User_SetCmd(p_motor, limitedCmd);
}

// [v/2 +/- scalar * ]
void Motor_User_SetVSpeedScalarCmd(Motor_T * p_motor, int16_t scalar_fract16)
{
    Motor_User_SetVoltageCmd(p_motor, Motor_GetVSpeed_Fract16(p_motor) / 2);
}


/******************************************************************************/
/*!
    Current Mode
*/
/******************************************************************************/
void Motor_User_StartIMode(Motor_T * p_motor) { Motor_User_StartControlMode(p_motor, MOTOR_FEEDBACK_MODE_CURRENT); }

/*!
    @param[in] i [-32768:32767]
*/
void Motor_User_SetICmd(Motor_T * p_motor, int16_t i_Fract16)
{
    _Motor_User_SetCmd(p_motor, math_clamp(i_Fract16, (int32_t)0 - Motor_User_GetILimitGenerating(p_motor), Motor_User_GetILimitMotoring(p_motor)));
}

void Motor_User_SetICmd_Scalar(Motor_T * p_motor, int16_t scalar_Fract16)
{
    int32_t limit = (scalar_Fract16 > 0) ? Motor_User_GetILimitMotoring(p_motor) : Motor_User_GetILimitGenerating(p_motor);
    _Motor_User_SetCmd(p_motor, fract16_mul(limit, scalar_Fract16));
}

/******************************************************************************/
/*!
    Torque Mode
    I with release when feedback is near 0
*/
/******************************************************************************/
void Motor_User_StartTorqueMode(Motor_T * p_motor) { Motor_User_StartIMode(p_motor); }

/*!
    @param[in] torque [-32768:32767]
*/
void Motor_User_SetTorqueCmd_Scalar(Motor_T * p_motor, int16_t scalar_Fract16)
{
    if (scalar_Fract16 < 0)
    {
        Motor_User_SetICmd_Scalar(p_motor, scalar_Fract16);
        // if (Motor_User_GetIPhase_UFract16(p_motor) > (INT16_MAX / 20)) // 5%
        // {
        //     Motor_User_SetICmd_Scalar(p_motor, scalar_Fract16);
        // }
        // else
        // {
        //     Motor_User_TryRelease(p_motor);
        // }
    }
    else
    {
        Motor_User_SetICmd_Scalar(p_motor, scalar_Fract16);
    }
}

// void Motor_User_SetIFollow(Motor_T * p_motor)
// {
//     Motor_User_SetTorqueModeCmd(p_motor, 0U);
// }


/******************************************************************************/
/*!
    Speed Mode
*/
/******************************************************************************/
/*!
    Default speed mode is speed torque mode
*/
void Motor_User_StartSpeedMode(Motor_T * p_motor) { Motor_User_StartControlMode(p_motor, MOTOR_FEEDBACK_MODE_SPEED_CURRENT); }

/*!
    Only allow forward direction, reverse direction use MOTOR_DIRECTION,
        Hall sensor directional read, ILimit directional set
    @param[in] speed [-32768:32767]
*/
void Motor_User_SetSpeedCmd(Motor_T * p_motor, int16_t speed_fract16)
{
    _Motor_User_SetCmd(p_motor, math_clamp(speed_fract16, 0, Motor_User_GetSpeedLimit(p_motor)));
}

void Motor_User_SetSpeedCmd_Scalar(Motor_T * p_motor, int16_t scalar_fract16)
{
    int32_t limitedCmd = (scalar_fract16 > 0) ? fract16_mul(Motor_User_GetSpeedLimit(p_motor), scalar_fract16) : 0;
    _Motor_User_SetCmd(p_motor, limitedCmd);
}

/******************************************************************************/
/*!
    Position Mode
*/
/******************************************************************************/
/*!
    @param[in] angle [0:65535]
*/
void Motor_User_SetPositionCmd(Motor_T * p_motor, uint16_t angle)
{
    // _Motor_User_ActivateControlMode(p_motor, MOTOR_FEEDBACK_MODE_POSITION_SPEED_CURRENT);
    // _Motor_User_SetCmd(p_motor, angle);
}

/******************************************************************************/
/*!
    Open Loop
*/
/******************************************************************************/
#if defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE) || defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)
/*!

*/
void Motor_User_StartOpenLoopMode(Motor_T * p_motor) { Motor_User_StartControlMode(p_motor, MOTOR_FEEDBACK_MODE_OPEN_LOOP_SCALAR); }

void Motor_User_SetOpenLoopSpeed(Motor_T * p_motor, int32_t speed_fract16)
{
    Linear_Ramp_SetTarget(&p_motor->OpenLoopSpeedRamp, speed_fract16);
}

/*!

*/
void Motor_User_SetOpenLoopCmd(Motor_T * p_motor, int16_t ivCmd)
{
    int32_t ivCmdIn = math_clamp(ivCmd, 0, p_motor->Config.OpenLoopPower_UFract16);
    _Motor_User_SetCmd(p_motor, ivCmdIn);
}

#endif


/******************************************************************************/
/*!
    Generic Mode
*/
/******************************************************************************/
/*!
    The untyped command must be the outer super function, as the boundaries of each type are different.
    @param[in] userCmd[-32768:32767] A union value, bounds to be determined by active mode
*/
void Motor_User_SetActiveCmdValue(Motor_T * p_motor, int16_t userCmd)
{
    Motor_FeedbackMode_T flags = p_motor->FeedbackMode;

    if      (flags.OpenLoop == 1U)  { Motor_User_SetOpenLoopCmd(p_motor, userCmd); }
    // else if (flags.Position == 1U)  { Motor_User_SetPositionCmd(p_motor, userCmd); }
    else if (flags.Speed == 1U)     { Motor_User_SetSpeedCmd(p_motor, userCmd); }
    else if (flags.Current == 1U)   { Motor_User_SetTorqueCmd_Scalar(p_motor, userCmd); }
    else                            { Motor_User_SetVoltageCmd(p_motor, userCmd); }
}


/******************************************************************************/
/*!
    Direction
*/
/******************************************************************************/
/*
    ProcInput use Critical, as States may transition during input
*/
void Motor_User_SetDirection(Motor_T * p_motor, Motor_Direction_T direction)
{
    if (p_motor->Direction != direction) { StateMachine_SetInput(&p_motor->StateMachine, MSM_INPUT_DIRECTION, direction); }
}

void Motor_User_SetDirectionForward(Motor_T * p_motor) { Motor_User_SetDirection(p_motor, p_motor->Config.DirectionForward); }
void Motor_User_SetDirectionReverse(Motor_T * p_motor) { Motor_User_SetDirection(p_motor, Motor_GetDirectionReverse(p_motor)); }

// bool Motor_User_TryDirection(Motor_T * p_motor, Motor_Direction_T direction)
// {
//     if (p_motor->Direction != direction) { StateMachine_ProcInput(&p_motor->StateMachine, MSM_INPUT_DIRECTION, direction); }
//     return (direction == p_motor->Direction);
// }

// bool Motor_User_TryDirectionForward(Motor_T * p_motor) { return Motor_User_TryDirection(p_motor, p_motor->Config.DirectionForward); }
// bool Motor_User_TryDirectionReverse(Motor_T * p_motor) { return Motor_User_TryDirection(p_motor, Motor_GetDirectionReverse(p_motor)); }

/******************************************************************************/
/*
    Calibration State - Blocking Run Calibration functions

    Check SensorMode on input to determine start, or proc may run before checking during entry
    alternatively, implement critical, move check into state machine, share 1 active function this way.
*/
/******************************************************************************/
void Motor_User_CalibrateAdc(Motor_T * p_motor)
{
    StateMachine_ProcInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, MOTOR_CALIBRATION_STATE_ADC);
}

void Motor_User_CalibrateSensor(Motor_T * p_motor)
{
    switch (p_motor->Config.SensorMode)
    {
        case MOTOR_SENSOR_MODE_HALL:        StateMachine_ProcInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, MOTOR_CALIBRATION_STATE_HALL);       break;
        case MOTOR_SENSOR_MODE_ENCODER:     StateMachine_ProcInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, MOTOR_CALIBRATION_STATE_ENCODER);    break;
        #if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
        case MOTOR_SENSOR_MODE_SIN_COS:     StateMachine_ProcInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, MOTOR_CALIBRATION_STATE_SIN_COS);    break;
        #endif
        default: break;
    }
}


/******************************************************************************/
/*
    Ground Speed
*/
/******************************************************************************/
#if defined(CONFIG_MOTOR_UNIT_CONVERSION_LOCAL) && defined(CONFIG_MOTOR_SURFACE_SPEED_ENABLE)
int16_t Motor_User_GetGroundSpeed_Kmh(Motor_T * p_motor)
{
    int16_t speed;

    switch(p_motor->Config.SensorMode)
    {
        case MOTOR_SENSOR_MODE_HALL:        speed = Encoder_DeltaD_GetGroundSpeed_Kmh(&p_motor->Encoder);    break;
        case MOTOR_SENSOR_MODE_ENCODER:     speed = Encoder_DeltaD_GetGroundSpeed_Kmh(&p_motor->Encoder);    break;
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
        case MOTOR_SENSOR_MODE_SIN_COS:     speed = Linear_Speed_CalcGroundSpeed(&p_motor->Units, p_motor->Speed_Fixed32); break;
#endif
#if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
        case MOTOR_SENSOR_MODE_SENSORLESS:     speed = 0;     break;
#endif
        default:                             speed = 0;     break;
    }

    return (p_motor->Direction == MOTOR_DIRECTION_CCW) ? speed : 0 - speed;
}

int16_t Motor_User_GetGroundSpeed_Mph(Motor_T * p_motor)
{
    int16_t speed;

    switch(p_motor->Config.SensorMode)
    {
        case MOTOR_SENSOR_MODE_HALL:         speed = Encoder_DeltaD_GetGroundSpeed_Mph(&p_motor->Encoder);    break;
        case MOTOR_SENSOR_MODE_ENCODER:     speed = Encoder_DeltaD_GetGroundSpeed_Mph(&p_motor->Encoder);    break;
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
        case MOTOR_SENSOR_MODE_SIN_COS:     speed =  Linear_Speed_CalcGroundSpeed(&p_motor->Units, p_motor->Speed_Fixed32); break;
#endif
#if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
        case MOTOR_SENSOR_MODE_SENSORLESS:     speed = 0;     break;
#endif
        default:                             speed = 0;     break;
    }

    return (p_motor->Direction == MOTOR_DIRECTION_CCW) ? speed : 0 - speed;
}

void Motor_User_SetGroundSpeed_Kmh(Motor_T * p_motor, uint32_t wheelDiameter_Mm, uint32_t wheelToMotorRatio_Factor, uint32_t wheelToMotorRatio_Divisor)
{
    switch(p_motor->Config.SensorMode)
    {
        case MOTOR_SENSOR_MODE_HALL:         Encoder_SetGroundRatio_Metric(&p_motor->Encoder, wheelDiameter_Mm, wheelToMotorRatio_Factor, wheelToMotorRatio_Divisor);    break;
        case MOTOR_SENSOR_MODE_ENCODER:     Encoder_SetGroundRatio_Metric(&p_motor->Encoder, wheelDiameter_Mm, wheelToMotorRatio_Factor, wheelToMotorRatio_Divisor);    break;
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
        case MOTOR_SENSOR_MODE_SIN_COS:     Linear_Speed_CalcGroundSpeed(&p_motor->Units, p_motor->Speed_Fixed32); break;
#endif
#if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
        case MOTOR_SENSOR_MODE_SENSORLESS:     speed = 0;     break;
#endif
        default:     break;
    }
}

void Motor_User_SetGroundSpeed_Mph(Motor_T * p_motor, uint32_t wheelDiameter_Inch10, uint32_t wheelToMotorRatio_Factor, uint32_t wheelToMotorRatio_Divisor)
{
    switch(p_motor->Config.SensorMode)
    {
        case MOTOR_SENSOR_MODE_HALL:         Encoder_SetGroundRatio_US(&p_motor->Encoder, wheelDiameter_Inch10, wheelToMotorRatio_Factor, wheelToMotorRatio_Divisor);    break;
        case MOTOR_SENSOR_MODE_ENCODER:     Encoder_SetGroundRatio_US(&p_motor->Encoder, wheelDiameter_Inch10, wheelToMotorRatio_Factor, wheelToMotorRatio_Divisor);    break;
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
        case MOTOR_SENSOR_MODE_SIN_COS:      Linear_Speed_CalcGroundSpeed(&p_motor->Units, p_motor->Speed_Fixed32); break;
#endif
#if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
        case MOTOR_SENSOR_MODE_SENSORLESS:     speed = 0;     break;
#endif
        default:                             break;
    }
}
#endif


