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
    @brief
    @version V0
*/
/******************************************************************************/
#include "Motor_User.h"
#include "System/Critical/Critical.h"

static int32_t Scale16(uint16_t scalar16, int32_t value) { return scalar16 * value / 65536; }

/******************************************************************************/
/*!
    State Machine Error checked inputs
*/
/******************************************************************************/
/*
    Motor State Machine Thread Safety

    SemiSync Mode -
    State Proc in PWM thread. User Input in Main Thread. may Need critical section during input.
    No Critical during transistion -> Prev State Proc may run before Entry.

    Sync Mode
    Must check input flags every pwm cycle

    Set async to proc, sync issue can recover?
    control proc may overwrite pid state set, but last to complete is always user input?

    CmdValue (Ramp Target) and CmdMode selectively sync inputs for StateMachine
*/

/******************************************************************************/
/*!
    UserCmd functions
    User Feedback Control modes, torque/speed/position, above foc/sixstep commutation layer
    Call regularly to update cmd value, lower frequency compared to control loop, ~100-1000Hz.

    UserCmd sign +/- indicates along or against Direction selected. NOT virtual CW/CCW.
    Handle direction and limit check on input.

    SetMode     - Invokes StateMachine - Sets control mode only
    SetCmdValue - Without invoking StateMachine - Sets buffered cmd value, sets on all states even when inactive
    SetModeCmd  - Check/sets control mode and cmd value
*/
/******************************************************************************/
/*
    ControlFeedbackMode update: Set mode flags => match Ramp and PID state to output on change
*/
void Motor_User_ActivateFeedbackMode(Motor_T * p_motor, Motor_FeedbackMode_T mode)
{
    // Motor_FeedbackMode_T mode = Motor_FeedbackModeFlags(mode);

    if(p_motor->ControlFeedbackMode.Word != mode.Word)
    {
        // //does not need critical if run state transition through freewheel first?
        // Critical_Enter(); /* Block PWM Thread, do not proc new flags before matching output with StateMachine */
        // p_motor->ControlFeedbackMode.State = mode.State;
        // Motor_SetFeedbackMode(p_motor, feedbackMode);
        // StateMachine_ProcAsyncInput(&p_motor->StateMachine, MSM_INPUT_CONTROL, mode);
        // Critical_Exit();
        // Critical_Enter();
        StateMachine_SetSyncInput(&p_motor->StateMachine, MSM_INPUT_CONTROL, (statemachine_inputvalue_t)mode.Word);
        // Critical_Exit();
    }
}

/*
    Sets Ramp Target
    Ramp Proc interrupt only update OutputState, RampTarget is safe from sync?
*/
void Motor_User_SetCmd(Motor_T * p_motor, int16_t userCmd)  { Linear_Ramp_SetTarget(&p_motor->Ramp, Motor_ConvertUserDirection(p_motor, userCmd)); }
int32_t Motor_User_GetCmd(Motor_T * p_motor)                { return Motor_ConvertUserDirection(p_motor, Linear_Ramp_GetTarget(&p_motor->Ramp)); }

/******************************************************************************/
/*!
    Voltage Mode
*/
/******************************************************************************/
void Motor_User_SetVoltageMode(Motor_T * p_motor)
{
    Motor_User_ActivateFeedbackMode(p_motor, MOTOR_FEEDBACK_MODE_VOLTAGE);
}

/*!
    @param[in] voltage [-32768:32767]
*/
void Motor_User_SetVoltageCmdValue(Motor_T * p_motor, int16_t voltageCmd)
{
    int32_t voltageCmdIn = (voltageCmd > 0) ? voltageCmd : 0; /* Reverse voltage use change direction */
    Motor_User_SetCmd(p_motor, voltageCmdIn);

}

void Motor_User_SetVoltageModeCmd(Motor_T * p_motor, int16_t voltageCmd)
{
    Motor_User_SetVoltageMode(p_motor);
    Motor_User_SetVoltageCmdValue(p_motor, voltageCmd);
}

/******************************************************************************/
/*!
    Voltage Freq Mode
*/
/******************************************************************************/
// void Motor_User_SetScalarMode(Motor_T * p_motor)
// {
//     _Motor_User_ActivateControlMode(p_motor, MOTOR_FEEDBACK_MODE_SCALAR_VOLTAGE_FREQ);
// }

// /*!
//     @param[in] voltage [0:65535] temp scalar 65535 => 1
// */
// void Motor_User_SetScalarCmdValue(Motor_T * p_motor, uint32_t scalar)
// {
//     Linear_Ramp_SetTarget(&p_motor->Ramp, scalar);
// }

// void Motor_User_SetScalarModeCmd(Motor_T * p_motor, uint32_t scalar)
// {
//     Motor_User_SetScalarMode(p_motor);
//     Motor_User_SetScalarCmdValue(p_motor, scalar);
// }

/******************************************************************************/
/*!
    Torque Mode
*/
/******************************************************************************/
void Motor_User_SetTorqueMode(Motor_T * p_motor)
{
    Motor_User_ActivateFeedbackMode(p_motor, MOTOR_FEEDBACK_MODE_CURRENT);
}

/*!
    @param[in] torque [-32768:32767]
*/
void Motor_User_SetTorqueCmdValue(Motor_T * p_motor, int16_t torqueCmd)
{
    int32_t torqueCmdIn = (torqueCmd > 0) ? Scale16(p_motor->ILimitMotoring_Scalar16, torqueCmd) : Scale16(p_motor->ILimitGenerating_Scalar16, torqueCmd);
    Motor_User_SetCmd(p_motor, torqueCmdIn);
}

void Motor_User_SetTorqueModeCmd(Motor_T * p_motor, int16_t torqueCmd)
{
    Motor_User_SetTorqueMode(p_motor);
    Motor_User_SetTorqueCmdValue(p_motor, torqueCmd);
}

/******************************************************************************/
/*!
    Speed Mode
*/
/******************************************************************************/
/*!
    Default speed mode is speed torque mode
*/
void Motor_User_SetSpeedMode(Motor_T * p_motor)
{
    Motor_User_ActivateFeedbackMode(p_motor, MOTOR_FEEDBACK_MODE_SPEED_CURRENT);
}

/*!
    Only allow forward direction, reverse direction use MOTOR_DIRECTION,
        Hall sensor directional read, ILimit directional set
    @param[in] speed [-32768:32767]
*/
void Motor_User_SetSpeedCmdValue(Motor_T * p_motor, int16_t speedCmd)
{
    int32_t speedCmdIn = (speedCmd > 0) ? Scale16(p_motor->SpeedLimitDirect_Scalar16, speedCmd) : 0;
    Motor_User_SetCmd(p_motor, speedCmdIn);
}

void Motor_User_SetSpeedModeCmd(Motor_T * p_motor, int16_t speedCmd)
{
    Motor_User_SetSpeedMode(p_motor);
    Motor_User_SetSpeedCmdValue(p_motor, speedCmd);
}

/******************************************************************************/
/*!
    Position Mode
*/
/******************************************************************************/
/*!
    @param[in] angle [0:65535]
    todo
*/
void Motor_User_SetPositionCmdValue(Motor_T * p_motor, uint16_t angle)
{
    // _Motor_User_ActivateControlMode(p_motor, MOTOR_FEEDBACK_MODE_POSITION_SPEED_CURRENT);
    // Motor_User_SetCmd(p_motor, angle);
}

/******************************************************************************/
/*!
    Open Loop
*/
/******************************************************************************/
/*!

*/
void Motor_User_SetOpenLoopMode(Motor_T * p_motor)
{
    Motor_User_ActivateFeedbackMode(p_motor, MOTOR_FEEDBACK_MODE_OPEN_LOOP);
}

void Motor_SetOpenLoopSpeed(Motor_T * p_motor, int32_t speed_Frac16)
{
    // Linear_Ramp_SetTarget(&p_motor->OpenLoopSpeedRamp, speed_Frac16);
}

/*!

*/
void Motor_User_SetOpenLoopCmdValue(Motor_T * p_motor, int16_t ivCmd)
{
    int32_t ivCmdIn = math_clamp(ivCmd, 0, p_motor->Parameters.OpenLoopPower_Scalar16 / 2U);
    Motor_User_SetCmd(p_motor, ivCmdIn);
}

void Motor_User_SetOpenLoopModeCmd(Motor_T * p_motor, int16_t ivMagnitude)
{
    Motor_User_SetOpenLoopMode(p_motor);
    Motor_User_SetOpenLoopCmdValue(p_motor, ivMagnitude);
}

/******************************************************************************/
/*!
    User Mode
*/
/******************************************************************************/
void Motor_User_ActivateDefaultFeedbackMode(Motor_T * p_motor)
{
    Motor_User_ActivateFeedbackMode(p_motor, p_motor->Parameters.DefaultFeedbackMode);
}

void Motor_User_SetDefaultFeedbackCmdValue(Motor_T * p_motor, int16_t userCmd)
{
    // Motor_FeedbackMode_T flags = Motor_FeedbackModeFlags(p_motor->Parameters.DefaultFeedbackMode);
    Motor_FeedbackMode_T flags = p_motor->Parameters.DefaultFeedbackMode;

    if      (flags.OpenLoop == 1U)  { Motor_User_SetOpenLoopCmdValue(p_motor, userCmd); }
    else if (flags.Position == 1U)  { Motor_User_SetPositionCmdValue(p_motor, userCmd); }
    else if (flags.Speed == 1U)     { Motor_User_SetSpeedCmdValue(p_motor, userCmd); }
    else if (flags.Current == 1U)   { Motor_User_SetTorqueCmdValue(p_motor, userCmd); }
    else                            { Motor_User_SetVoltageCmdValue(p_motor, userCmd); }
}

/*!
    @param[in] userCmd [-32768:32767]
*/
void Motor_User_SetDefaultModeCmd(Motor_T * p_motor, int16_t userCmd)
{
    Motor_User_ActivateDefaultFeedbackMode(p_motor);
    Motor_User_SetDefaultFeedbackCmdValue(p_motor, userCmd);
}

/******************************************************************************/
/*!
    Vehicle Wrapper
    Throttle and Brake accept uint16_t, wrapped functions use int16_t
    - UserCmd value in configured DefaultFeedbackMode
*/
/******************************************************************************/
/*!
    @param[in] throttle [0:65535] throttle percentage, 65535 => speed limit
*/
void Motor_User_SetThrottleCmd(Motor_T * p_motor, uint16_t throttle)
{
    Motor_User_SetDefaultModeCmd(p_motor, throttle / 2U); // change to speed?
}

/*!
    Always request opposite direction current
    req opposite iq, bound vq to 0 for no plugging brake

    transition from accelerating to decelerating,
    use signed ramp to transition through 0 without discontinuity
    ramp from in-direction torque to 0 to counter-direction torque

    @param[in] brake [0:65535]
*/
void Motor_User_SetBrakeCmd(Motor_T * p_motor, uint16_t brake)
{

    // if(p_motor->ControlFeedbackMode.Hold == 0U)
    // {
    if(Motor_User_GetSpeed_UFrac16(p_motor) > INT16_MAX / 100U)
    {
        Motor_User_SetTorqueModeCmd(p_motor, (int32_t)0 - (brake / 2U));
    }
    else
    {
        // p_motor->ControlFeedbackMode.Hold = 1U;  //clears on throttle
        // Phase_Ground(&p_motor->Phase);
        // Phase_Float(&p_motor->Phase);
        Motor_User_ReleaseControl(p_motor);
    }
    // }
}

void Motor_User_SetVBrakeCmd(Motor_T * p_motor, uint16_t brake)
{
    // Motor_User_SetScalarModeCmd(p_motor, (65535U - brake)); /* Higher brake => lower voltage */
}

void Motor_User_SetCruise(Motor_T * p_motor)
{
    Motor_User_SetTorqueModeCmd(p_motor, 0U);
}

/******************************************************************************/
/*!

*/
/******************************************************************************/
/*
    always State machine checked version
*/
void Motor_User_ReleaseControl(Motor_T * p_motor)
{
    StateMachine_ProcAsyncInput(&p_motor->StateMachine, MSM_INPUT_RELEASE, STATE_MACHINE_INPUT_VALUE_NULL);
    /* no critical for transition, only 1 transistion in run state? cannot conflict? */
}

/*
    Disable control
*/
void Motor_User_DisableControl(Motor_T * p_motor)
{
    Phase_Float(&p_motor->Phase);
    StateMachine_ProcAsyncInput(&p_motor->StateMachine, MSM_INPUT_RELEASE, STATE_MACHINE_INPUT_VALUE_NULL);
}

void Motor_User_Ground(Motor_T * p_motor)
{
    Phase_Ground(&p_motor->Phase);
    //    StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_GROUND);    //todo only ground in stop state
    // Motor_User_SetVoltageModeCmd(p_motor, 0U);
}

/*
    Does not need critical section if PWM interrupt does not read/write direction
*/
bool Motor_User_SetDirection(Motor_T * p_motor, Motor_Direction_T direction)
{
    if(p_motor->Direction != direction) { StateMachine_ProcAsyncInput(&p_motor->StateMachine, MSM_INPUT_DIRECTION, direction); }
    return (direction == p_motor->Direction);
}

bool Motor_User_SetDirectionForward(Motor_T * p_motor)
{
    Motor_Direction_T direction = (p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW) ? MOTOR_DIRECTION_CCW : MOTOR_DIRECTION_CW;
    return Motor_User_SetDirection(p_motor, direction);
}

bool Motor_User_SetDirectionReverse(Motor_T * p_motor)
{
    Motor_Direction_T direction = (p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW) ? MOTOR_DIRECTION_CW : MOTOR_DIRECTION_CCW;
    return Motor_User_SetDirection(p_motor, direction);
}

/*
    Fault - Shared StateMachine InputId
    Fault State - checks exit
    Other States - user initiated transistion to fault state
*/
bool Motor_User_ClearFault(Motor_T * p_motor)
{
    if(StateMachine_GetActiveStateId(&p_motor->StateMachine) == MSM_STATE_ID_FAULT)
        { StateMachine_ProcAsyncInput(&p_motor->StateMachine, MSM_INPUT_FAULT, STATE_MACHINE_INPUT_VALUE_NULL); }

    return (StateMachine_GetActiveStateId(&p_motor->StateMachine) != MSM_STATE_ID_FAULT);
}

void Motor_User_SetFault(Motor_T * p_motor)
{
    if(StateMachine_GetActiveStateId(&p_motor->StateMachine) != MSM_STATE_ID_FAULT)
        { StateMachine_ProcAsyncInput(&p_motor->StateMachine, MSM_INPUT_FAULT, STATE_MACHINE_INPUT_VALUE_NULL); }
}

bool Motor_User_CheckFault(Motor_T * p_motor)
{
    return (StateMachine_GetActiveStateId(&p_motor->StateMachine) == MSM_STATE_ID_FAULT);
}

void Motor_User_ToggleFault(Motor_T * p_motor)
{
    StateMachine_ProcAsyncInput(&p_motor->StateMachine, MSM_INPUT_FAULT, STATE_MACHINE_INPUT_VALUE_NULL);
}

/******************************************************************************/
/*
    Calibration State - Blocking Run Calibration functions

    Check SensorMode on input to determine start, or proc may run before checking during entry
    alternatively, implement critical, move check into state machine, share 1 active function this way.
*/
/******************************************************************************/
void Motor_User_CalibrateAdc(Motor_T * p_motor)
{
    StateMachine_ProcAsyncInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, MOTOR_CALIBRATION_STATE_ADC);
}

void Motor_User_CalibrateHall(Motor_T * p_motor)
{
    if(p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_HALL)
        { StateMachine_ProcAsyncInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, MOTOR_CALIBRATION_STATE_HALL); }
}

void Motor_User_CalibrateEncoder(Motor_T * p_motor)
{
    if(p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_ENCODER)
        { StateMachine_ProcAsyncInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, MOTOR_CALIBRATION_STATE_ENCODER); }
}

#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
void Motor_User_CalibrateSinCos(Motor_T * p_motor)
{
    if(p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_SIN_COS)
        { StateMachine_ProcAsyncInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, MOTOR_CALIBRATION_STATE_SIN_COS); }
}
#endif

void Motor_User_CalibrateSensor(Motor_T * p_motor)
{
    switch(p_motor->Parameters.SensorMode)
    {
        case MOTOR_SENSOR_MODE_HALL:        StateMachine_ProcAsyncInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, MOTOR_CALIBRATION_STATE_HALL);       break;
        case MOTOR_SENSOR_MODE_ENCODER:     StateMachine_ProcAsyncInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, MOTOR_CALIBRATION_STATE_ENCODER);    break;
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
        case MOTOR_SENSOR_MODE_SIN_COS:     StateMachine_ProcAsyncInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, MOTOR_CALIBRATION_STATE_SIN_COS);    break;
#endif
        default: break;
    }
}
/******************************************************************************/
/*   */
/******************************************************************************/

/******************************************************************************/
/*!
    User Error Checked Get Set Functions
*/
/******************************************************************************/
/******************************************************************************/
/*!
    Active Limits

    RefMax -> Param -> [Direction] -> Scalar

    RefMax => 5000 RPM
    Parameters.Limit_Frac16 == 32767 => 2500 RPM
    scalar16 == 52428 => Limit[Active]_Frac16 == 26213 => 2000 RPM

    Limit[Active]_Frac16 is applied every SetUserCmd
*/
/******************************************************************************/
/*
    Speed limit always sets most recent input
*/
void Motor_User_SetSpeedLimitActive(Motor_T * p_motor, uint16_t scalar16)
{
    p_motor->SpeedLimitForward_Scalar16 = Scale16(scalar16, p_motor->Parameters.SpeedLimitForward_Scalar16);
    p_motor->SpeedLimitReverse_Scalar16 = Scale16(scalar16, p_motor->Parameters.SpeedLimitReverse_Scalar16);
    Motor_SetFeedbackSpeedLimits(p_motor);
}

void Motor_User_ClearSpeedLimitActive(Motor_T * p_motor)
{
    Motor_ResetActiveSpeedLimits(p_motor);
}

// bool Motor_User_SetSpeedLimitActive_Id(Motor_T * p_motor, uint16_t scalar16, Motor_SpeedLimitActiveId_T id)
// {
//     bool isSet = (scalar16 < p_motor->SpeedLimitActiveScalar);
//     if(isSet == true)
//     {
//         Motor_User_SetSpeedLimitActive(p_motor, scalar16);
//         p_motor->SpeedLimitActiveId = id;
//     }
//     return isSet;
// }

// bool Motor_User_ClearSpeedLimitActive_Id(Motor_T * p_motor, Motor_SpeedLimitActiveId_T id)
// {
//     if(p_motor->SpeedLimitActiveId == id)
//     {
//         Motor_User_ClearSpeedLimitActive(p_motor);
//         p_motor->SpeedLimitActiveId = MOTOR_SPEED_LIMIT_ACTIVE_DISABLE;
//     }
// }

/*
    ILimit
*/
/*
    Unchecked
*/
void _Motor_User_SetILimitActive(Motor_T * p_motor, uint16_t scalar16)
{
    p_motor->ILimitActiveSentinel_Scalar16 = scalar16;
    p_motor->ILimitMotoring_Scalar16 = Scale16(scalar16, p_motor->Parameters.ILimitMotoring_Scalar16);
    p_motor->ILimitGenerating_Scalar16 = Scale16(scalar16, p_motor->Parameters.ILimitGenerating_Scalar16);
    Motor_SetFeedbackILimits(p_motor);
}

void _Motor_User_ClearILimitActive(Motor_T * p_motor)
{
    Motor_ResetActiveILimits(p_motor);
    Motor_SetFeedbackILimits(p_motor);
}

/*
    Check Lower
    E.g.
    LimitParam = 32768 => LimitActive = 32768
    Scalar1 = 32768 => LimitActive = 16384
    Scalar2 = 6553 => LimitActive = 3276
    Scalar3 = 32768 => LimitActive = 3276

    need list to restore previous limit
*/
/*! @return true if set */
bool Motor_User_SetILimitActive(Motor_T * p_motor, uint16_t scalar16, Motor_ILimitActiveId_T id)
{
    bool isSet = (scalar16 < p_motor->ILimitActiveSentinel_Scalar16);
    if(isSet == true)
    {
        _Motor_User_SetILimitActive(p_motor, scalar16);
        p_motor->ILimitActiveId = id;
    }
    return isSet;
}

/*!
    Restores 100%, need list to restore previous limit
    @return true if cleared. ILimit of input id
*/
bool Motor_User_ClearILimitActive(Motor_T * p_motor, Motor_ILimitActiveId_T id)
{
    bool isClear = (p_motor->ILimitActiveId == id);
    if(isClear == true)
    {
        _Motor_User_ClearILimitActive(p_motor);
        p_motor->ILimitActiveId = MOTOR_I_LIMIT_ACTIVE_DISABLE;
    }
    return isClear;
}

Motor_ILimitActiveId_T Motor_User_GetILimitActiveId(Motor_T * p_motor) { return p_motor->ILimitActiveId; }



/******************************************************************************/
/*
    Ground Speed
*/
/******************************************************************************/
#if defined(CONFIG_MOTOR_UNIT_CONVERSION_LOCAL) && defined(CONFIG_MOTOR_SURFACE_SPEED_ENABLE)
int16_t Motor_User_GetGroundSpeed_Kmh(Motor_T * p_motor)
{
    int16_t speed;

    switch(p_motor->Parameters.SensorMode)
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

    switch(p_motor->Parameters.SensorMode)
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
    switch(p_motor->Parameters.SensorMode)
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
    switch(p_motor->Parameters.SensorMode)
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