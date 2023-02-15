/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2021 FireSourcery / The Firebrand Forge Inc

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
    @file     Motor_User.c
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#include "Motor_User.h"
#include "System/Critical/Critical.h"

static int32_t Scale16(uint16_t scalarU16, int32_t value) { return scalarU16 * value / 65536; }

/******************************************************************************/
/*!
    State Machine Error checked inputs
*/
/******************************************************************************/
/*
    Motor State Machine Thread Safety

    SemiSync Mode -
    State Proc in PWM thread. User Input in Main Thread. Need critical section during input.
    No Critical during transistion -> Prev State Proc may run before Entry.

    Sync Mode
    Must check input flags every pwm cycle

    Set async to proc, sync issue can recover?
    control proc may overwrite pid state set, but last to complete is always user input?

    CmdValue (Ramp Target) and CmdMode selectively sync inputs for StateMachine
*/
/*
    ControlFeedbackMode update: match Ramp and PID state to output
    Transition to Run State (Active Control)

    proc if (not run state) or (run state and change feedbackmode)
    Motor_User_Set[Control]ModeCmd check feedback flags,
    check control active flag
    Store control active flag as ControlFeedbackMode.IsDisable
*/

// static inline void _CheckSetCurrentLimitFeedback(Motor_T * p_motor)
// {
//     if((modeControl.Current == 0U) && Motor_FOC_CheckIOverThreshold(p_motor) == true) { modeControl.Current = 1U; }
// }
// static inline void _CheckSetSpeedLimitFeedback(Motor_T * p_motor)
// {
//     if((modeControl.Speed == 0U) && Motor_CheckSpeedOverThreshold(p_motor) == true) { modeControl.Speed = 1U; }
// }

//todo split super function
static inline Motor_FeedbackMode_T Motor_GetValidFeedbackMode(Motor_T * p_motor, Motor_FeedbackModeId_T modeCmd)
{
    Motor_FeedbackMode_T modeControl = Motor_ConvertFeedbackModeId(modeCmd);
    if((modeControl.Speed == 0U) && Motor_CheckSpeedOverThreshold(p_motor) == true)     { modeControl.Speed = 1U; }
    if((modeControl.Current == 0U) && Motor_FOC_CheckIOverThreshold(p_motor) == true)     { modeControl.Current = 1U; }
    return modeControl;
}

void _Motor_User_ActivateControlMode(Motor_T * p_motor, Motor_FeedbackModeId_T modeCmd)
{
    Motor_FeedbackMode_T modeControl;

    if(p_motor->ControlFeedbackMode.State != Motor_ConvertFeedbackModeId(modeCmd).State)
    {
        modeControl = Motor_GetValidFeedbackMode(p_motor, modeCmd);

        if(p_motor->ControlFeedbackMode.State != modeControl.State)
        {
            Critical_Enter(); /* Block PWM Thread, do not proc new flags before matching output with StateMachine */
            p_motor->ControlFeedbackMode.State = modeControl.State;
            StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_CONTROL, modeCmd);
            Critical_Exit();
        }
    }
}

/* Ramp Proc interrupt may overwrite set */
static void Motor_User_SetCmd(Motor_T * p_motor, int16_t userCmd)
{
    Linear_Ramp_SetTarget(&p_motor->Ramp, Motor_ConvertUserDirection(p_motor, userCmd));
}

static void Motor_User_GetCmd(Motor_T * p_motor)
{
    Motor_ConvertUserDirection(p_motor, Linear_Ramp_GetTarget(&p_motor->Ramp));
}

/******************************************************************************/
/*!
    UserCmd functions
    User Feedback Control modes, torque/speed/position, above foc/sixstep commutation layer
    Call regularly to update cmd value

    User input sign +/- indicates along or against Direction selected. NOT virtual CW/CCW.
    Handle direction and limit check on input.  Called less frequently than control loop, 1/Millis.

    SetMode     - Invokes StateMachine - Sets control mode only
    SetCmdValue - Without invoking StateMachine - Sets cmd value
        Although cmd/ramp value need not set on every state, handle outside
    SetModeCmd     - Check/sets control mode, and sets cmd value
*/
/******************************************************************************/
/******************************************************************************/
/*!
    Voltage Mode
*/
/******************************************************************************/
void Motor_User_SetVoltageMode(Motor_T * p_motor)
{
    // Motor_FeedbackMode_T modeControl = Motor_ConvertFeedbackModeId(modeCmd);
    // if((modeControl.Speed == 0U) && Motor_CheckSpeedOverThreshold(p_motor) == true)         { modeControl.Speed = 1U; }
    // if((modeControl.Current == 0U) && Motor_FOC_CheckIOverThreshold(p_motor) == true)     { modeControl.Current = 1U; }
    // return modeControl;

    _Motor_User_ActivateControlMode(p_motor, MOTOR_FEEDBACK_MODE_CONSTANT_VOLTAGE);
}

/*!
    @param[in] voltage [-32768:32767]
*/
void Motor_User_SetVoltageCmdValue(Motor_T * p_motor, int16_t voltageCmd)
{
    int32_t voltageCmdIn = (voltageCmd > 0) ? voltageCmd : 0; /* Reverse voltage use change direction */
    if((p_motor->ControlFeedbackMode.Speed == 0U) && (p_motor->ControlFeedbackMode.Current == 0U))
    {
        Motor_User_SetCmd(p_motor, voltageCmdIn);
    }
    else
    {
        // if(voltageCmdIn < Motor_User_GetCmd(p_motor))
        // {
        //     if((p_motor->ControlFeedbackMode.Speed == 1U) && (Motor_CheckSpeedOverThreshold(p_motor) == false))     { p_motor->ControlFeedbackMode.Speed = 0U; }
        //     if((p_motor->ControlFeedbackMode.Current == 1U) && (Motor_FOC_CheckIOverThreshold(p_motor) == false))     { p_motor->ControlFeedbackMode.Current = 0U; }

        //     if(p_motor->ControlFeedbackMode.Speed == 1U)
        //     {
        //         if((Motor_CheckSpeedOverThreshold(p_motor) == false) || (math_abs(vReq) < math_abs(p_motor->PidSpeed.Output)))
        //         {
        //             p_motor->ControlFeedbackMode.Speed = 0U;
        //         }
        //     }
        // }
    }
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
    _Motor_User_ActivateControlMode(p_motor, MOTOR_FEEDBACK_MODE_CONSTANT_CURRENT);
}

/*!
    @param[in] torque [-32768:32767]
*/
void Motor_User_SetTorqueCmdValue(Motor_T * p_motor, int16_t torqueCmd)
{

    int32_t torqueCmdIn = (torqueCmd > 0) ? Scale16(p_motor->ILimitMotoring_ScalarU16, torqueCmd) : Scale16(p_motor->ILimitGenerating_ScalarU16, torqueCmd);
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
    _Motor_User_ActivateControlMode(p_motor, MOTOR_FEEDBACK_MODE_CONSTANT_SPEED_CURRENT);
}

/*!
    Only allow forward direction, reverse direction use MOTOR_DIRECTION,
        Hall sensor directional read, ILimit directional set
    @param[in] speed [-32768:32767]
*/
void Motor_User_SetSpeedCmdValue(Motor_T * p_motor, int16_t speedCmd)
{
    int32_t speedCmdIn = (speedCmd > 0) ? Scale16(p_motor->SpeedLimitDirect_ScalarU16, speedCmd) : 0;
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
    _Motor_User_ActivateControlMode(p_motor, MOTOR_FEEDBACK_MODE_OPEN_LOOP);
}

void Motor_SetOpenLoopSpeed(Motor_T * p_motor, int32_t speed_Frac16)
{
    // Linear_Ramp_SetTarget(&p_motor->OpenLoopSpeedRamp, speed_Frac16);
}

/*!

*/
void Motor_User_SetOpenLoopCmdValue(Motor_T * p_motor, int16_t ivCmd)
{
    int32_t ivCmdIn = math_clamp(ivCmd, 0, p_motor->Parameters.OpenLoopPower_FracU16 / 2U);
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
void Motor_User_SetDefaultFeedbackMode(Motor_T * p_motor)
{
    _Motor_User_ActivateControlMode(p_motor, p_motor->Parameters.DefaultFeedbackMode);
}

void Motor_User_SetDefaultFeedbackCmdValue(Motor_T * p_motor, int16_t userCmd)
{
    Motor_FeedbackMode_T flags = Motor_ConvertFeedbackModeId(p_motor->Parameters.DefaultFeedbackMode);

    if        (flags.OpenLoop == 1U)     { Motor_User_SetOpenLoopCmdValue(p_motor, userCmd); }
    else if    (flags.Position == 1U)     { Motor_User_SetPositionCmdValue(p_motor, userCmd); }
    else if    (flags.Speed == 1U)     { Motor_User_SetSpeedCmdValue(p_motor, userCmd); }
    else if    (flags.Current == 1U)     { Motor_User_SetTorqueCmdValue(p_motor, userCmd); }
    else                             { Motor_User_SetVoltageCmdValue(p_motor, userCmd); }
}

/*!
    @param[in] userCmd [-32768:32767]
*/
void Motor_User_SetDefaultCmd(Motor_T * p_motor, int16_t userCmd)
{
    Motor_User_SetDefaultFeedbackMode(p_motor);
    Motor_User_SetDefaultFeedbackCmdValue(p_motor, userCmd);
}

/******************************************************************************/
/*!
    Throttle and Brake accept uint16_t, wrapped functions use int16_t
    - UserCmd value in configured DefaultFeedbackMode
*/
/******************************************************************************/
/*!
    @param[in] throttle [0:65535] throttle percentage, 65535 => speed limit
*/
void Motor_User_SetThrottleCmd(Motor_T * p_motor, uint16_t throttle)
{
    Motor_User_SetDefaultCmd(p_motor, throttle / 2U);
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
    //     if(Motor_GetSpeed_RPM(p_motor) > 10U)
    //     {
    Motor_User_SetTorqueModeCmd(p_motor, (int32_t)0 - (brake / 2U));
    //     }
    //     else
    //     {
    //         p_motor->ControlFeedbackMode.Hold = 1U;  //clears on throttle
    //         Phase_Ground(&p_motor->Phase);
    //     }
    // }
}

void Motor_User_SetVBrakeCmd(Motor_T * p_motor, uint16_t brake)
{
    Motor_User_SetScalarModeCmd(p_motor, (65535U - brake)); /* Higher brake => lower voltage */
}

void Motor_User_SetCruise(Motor_T * p_motor)
{
    Motor_User_SetTorqueModeCmd(p_motor, 0U);
}

/******************************************************************************/
/*!
    StateMachine Functions
*/
/******************************************************************************/
/*
    set buffered direction, check on state machine
*/
bool Motor_User_SetDirection(Motor_T * p_motor, Motor_Direction_T direction)
{
    if(p_motor->Direction != direction) { StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_DIRECTION, direction); }
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
    Calibration State - Run Calibration functions

    Check SensorMode on input to determine start, or proc may run before checking during entry
    alternatively, implement critical, move check into state machine, share 1 active function this way.
*/
void Motor_User_ActivateCalibrationAdc(Motor_T * p_motor)
{
    StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, MOTOR_CALIBRATION_STATE_ADC);
}

void Motor_User_ActivateCalibrationHall(Motor_T * p_motor)
{
    if(p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_HALL)
        { StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, MOTOR_CALIBRATION_STATE_HALL); }
}

void Motor_User_ActivateCalibrationEncoder(Motor_T * p_motor)
{
    if(p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_ENCODER)
        { StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, MOTOR_CALIBRATION_STATE_ENCODER); }
}

#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
void Motor_User_ActivateCalibrationSinCos(Motor_T * p_motor)
{
    if(p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_SIN_COS)
        { StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, MOTOR_CALIBRATION_STATE_SIN_COS); }
}
#endif

void Motor_User_ActivateCalibrationSensor(Motor_T * p_motor)
{
    switch(p_motor->Parameters.SensorMode)
    {
        case MOTOR_SENSOR_MODE_HALL:        StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, MOTOR_CALIBRATION_STATE_HALL);         break;
        case MOTOR_SENSOR_MODE_ENCODER:        StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, MOTOR_CALIBRATION_STATE_ENCODER);    break;
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
        case MOTOR_SENSOR_MODE_SIN_COS:        StateMachine_Semi_ProcInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, MOTOR_CALIBRATION_STATE_SIN_COS);    break;
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
    p_motor->SpeedLimitForward_ScalarU16     = Scale16(scalar16, p_motor->Parameters.SpeedLimitForward_ScalarU16);
    p_motor->SpeedLimitReverse_ScalarU16     = Scale16(scalar16, p_motor->Parameters.SpeedLimitReverse_ScalarU16);
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
    p_motor->ILimitActiveSentinel = scalar16;
    p_motor->ILimitMotoring_ScalarU16 = Scale16(scalar16, p_motor->Parameters.ILimitMotoring_ScalarU16);
    p_motor->ILimitGenerating_ScalarU16 = Scale16(scalar16, p_motor->Parameters.ILimitGenerating_ScalarU16);
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
    bool isSet = (scalar16 < p_motor->ILimitActiveSentinel);
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
int16_t Motor_User_GetGroundSpeed_Kmh(Motor_T * p_motor)
{
    int16_t speed;

    switch(p_motor->Parameters.SensorMode)
    {
        case MOTOR_SENSOR_MODE_HALL:         speed = Encoder_DeltaD_GetGroundSpeed_Kmh(&p_motor->Encoder);    break;
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


/******************************************************************************/
/*
    Nvm Params
*/
/******************************************************************************/
typedef void(*Motor_PropagateSet_T)(Motor_T * p_motor);

static inline void PropagateSet(Motor_T * p_motor, Motor_PropagateSet_T reset)
{
#ifdef CONFIG_MOTOR_PROPAGATE_SET_PARAM_ENABLE
    reset(p_motor);
#else
    (void)p_motor; (void)reset;
#endif
}

/******************************************************************************/
/*
    Nvm Param Persistent Limits
*/
/******************************************************************************/
/*
    Persistent SpeedLimit - effective Speed Feedback Mode only
*/
void Motor_User_SetSpeedLimitParam_Frac16(Motor_T * p_motor, uint16_t forwardScalar16, uint16_t reverseScalar16)
{
    p_motor->Parameters.SpeedLimitForward_ScalarU16 = forwardScalar16;
    p_motor->Parameters.SpeedLimitReverse_ScalarU16 = reverseScalar16;
    // if(p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW)
    // {
    //     p_motor->Parameters.SpeedLimitCcw_FracS16 = forward_Frac16;
    //     p_motor->Parameters.SpeedLimitCw_FracS16 = reverse_Frac16;
    // }
    // else
    // {
    //     p_motor->Parameters.SpeedLimitCcw_FracS16 = reverse_Frac16;
    //     p_motor->Parameters.SpeedLimitCw_FracS16 = forward_Frac16;
    // }
    PropagateSet(p_motor, Motor_User_ClearSpeedLimitActive);
}

void Motor_User_SetSpeedLimitForwardParam_Frac16(Motor_T * p_motor, uint16_t forward_Frac16)
{
    // if(p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW)     { p_motor->Parameters.SpeedLimitCcw_FracS16 = forward_Frac16; }
    // else                                                                     { p_motor->Parameters.SpeedLimitCw_FracS16 = forward_Frac16; }
    p_motor->Parameters.SpeedLimitForward_ScalarU16 = forward_Frac16;
    PropagateSet(p_motor, Motor_User_ClearSpeedLimitActive);
}

void Motor_User_SetSpeedLimitReverseParam_Frac16(Motor_T * p_motor, uint16_t reverse_Frac16)
{
    // if(p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW)     { p_motor->Parameters.SpeedLimitCw_FracS16 = reverse_Frac16; }
    // else                                                                     { p_motor->Parameters.SpeedLimitCcw_FracS16 = reverse_Frac16; }
    p_motor->Parameters.SpeedLimitReverse_ScalarU16 = reverse_Frac16;
    PropagateSet(p_motor, Motor_User_ClearSpeedLimitActive);
}

#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
static uint16_t ConvertToSpeedLimitFrac16(Motor_T * p_motor, uint16_t speed_rpm)
{
    int32_t speed_frac16 = _Motor_ConvertSpeed_RpmToScalar16(p_motor, speed_rpm);
    return (speed_frac16 > UINT16_MAX) ? UINT16_MAX : speed_frac16;
}

void Motor_User_SetSpeedLimitParam_Rpm(Motor_T * p_motor, uint16_t forward_Rpm, uint16_t reverse_Rpm)
{
    Motor_User_SetSpeedLimitParam_Frac16(p_motor, ConvertToSpeedLimitFrac16(p_motor, forward_Rpm), ConvertToSpeedLimitFrac16(p_motor, reverse_Rpm));
}

void Motor_User_SetSpeedLimitForwardParam_Rpm(Motor_T * p_motor, uint16_t forward_Rpm)
{
    Motor_User_SetSpeedLimitForwardParam_Frac16(p_motor, ConvertToSpeedLimitFrac16(p_motor, forward_Rpm));
}

void Motor_User_SetSpeedLimitReverseParam_Rpm(Motor_T * p_motor, uint16_t reverse_Rpm)
{
    Motor_User_SetSpeedLimitReverseParam_Frac16(p_motor, ConvertToSpeedLimitFrac16(p_motor, reverse_Rpm));
}

uint16_t Motor_User_GetSpeedLimitForwardParam_Rpm(Motor_T * p_motor)
{
    // uint16_t speedLimit = (p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW) ?
    //     p_motor->Parameters.SpeedLimitCcw_FracS16 : p_motor->Parameters.SpeedLimitCw_FracS16;
    // return _Motor_ConvertSpeed_Scalar16ToRpm(p_motor, speedLimit);
    return _Motor_ConvertSpeed_Scalar16ToRpm(p_motor, p_motor->Parameters.SpeedLimitForward_ScalarU16);
}

uint16_t Motor_User_GetSpeedLimitReverseParam_Rpm(Motor_T * p_motor)
{
    // uint16_t speedLimit = (p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW) ?
    //     p_motor->Parameters.SpeedLimitCw_FracS16 : p_motor->Parameters.SpeedLimitCcw_FracS16;
    return _Motor_ConvertSpeed_Scalar16ToRpm(p_motor, p_motor->Parameters.SpeedLimitReverse_ScalarU16);
}
#endif

/*
    Persistent ILimit
*/
void Motor_User_SetILimitParam_Frac16(Motor_T * p_motor, uint16_t motoring_Frac16, uint16_t generating_Frac16)
{
    p_motor->Parameters.ILimitMotoring_ScalarU16 = motoring_Frac16;
    p_motor->Parameters.ILimitGenerating_ScalarU16 = generating_Frac16;
    PropagateSet(p_motor, _Motor_User_ClearILimitActive);
}

void Motor_User_SetILimitMotoringParam_Frac16(Motor_T * p_motor, uint16_t motoring_Frac16)
{
    p_motor->Parameters.ILimitMotoring_ScalarU16 = motoring_Frac16;
    PropagateSet(p_motor, _Motor_User_ClearILimitActive);
}

void Motor_User_SetILimitGeneratingParam_Frac16(Motor_T * p_motor, uint16_t generating_Frac16)
{
    p_motor->Parameters.ILimitGenerating_ScalarU16 = generating_Frac16;
    PropagateSet(p_motor, _Motor_User_ClearILimitActive);
}

#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
static uint16_t ConvertToILimitFrac16(Motor_T * p_motor, uint16_t i_amp)
{
    int32_t i_frac16 = _Motor_ConvertI_AmpToFrac16(i_amp);
    return (i_frac16 > UINT16_MAX) ? UINT16_MAX : i_frac16;
}

void Motor_User_SetILimitParam_Amp(Motor_T * p_motor, uint16_t motoring_Amp, uint16_t generating_Amp)
{
    Motor_User_SetILimitParam_Frac16(p_motor, ConvertToILimitFrac16(p_motor, motoring_Amp), ConvertToILimitFrac16(p_motor, generating_Amp));
}

void Motor_User_SetILimitMotoringParam_Amp(Motor_T * p_motor, uint16_t motoring_Amp)
{
    Motor_User_SetILimitMotoringParam_Frac16(p_motor, ConvertToILimitFrac16(p_motor, motoring_Amp));
}

void Motor_User_SetILimitGeneratingParam_Amp(Motor_T * p_motor, uint16_t generating_Amp)
{
    Motor_User_SetILimitGeneratingParam_Frac16(p_motor, ConvertToILimitFrac16(p_motor, generating_Amp));
}

uint16_t Motor_User_GetILimitMotoringParam_Amp(Motor_T * p_motor)         { return _Motor_ConvertI_Frac16ToAmp(p_motor->Parameters.ILimitMotoring_ScalarU16); }
uint16_t Motor_User_GetILimitGeneratingParam_Amp(Motor_T * p_motor)     { return _Motor_ConvertI_Frac16ToAmp(p_motor->Parameters.ILimitGenerating_ScalarU16); }
#endif

/******************************************************************************/
/*
    Nvm Reference/Calibration
    Optionally propagate values during set, or wait for reboot
*/
/******************************************************************************/
/* SpeedFeedbackRef_Rpm => 100% speed for PID feedback. */
void Motor_User_SetSpeedFeedbackRef_Rpm(Motor_T * p_motor, uint16_t rpm)
{
    p_motor->Parameters.SpeedFeedbackRef_Rpm = rpm;
    if(rpm < p_motor->Parameters.VSpeedRef_Rpm) { p_motor->Parameters.VSpeedRef_Rpm = rpm; }
    PropagateSet(p_motor, Motor_ResetUnitsVSpeed);
    PropagateSet(p_motor, Motor_ResetUnitsSensor);
}

/* SpeedVRef =< SetSpeedFeedbackRef to ensure not match to higher speed */
void Motor_User_SetVSpeedRef_Rpm(Motor_T * p_motor, uint16_t rpm)
{
    p_motor->Parameters.VSpeedRef_Rpm = (rpm <= p_motor->Parameters.SpeedFeedbackRef_Rpm) ? rpm : p_motor->Parameters.SpeedFeedbackRef_Rpm;
    PropagateSet(p_motor, Motor_ResetUnitsVSpeed);
    PropagateSet(p_motor, Motor_ResetUnitsSensor);
}

void Motor_User_SetSpeedFeedbackRef_Kv(Motor_T * p_motor, uint16_t kv)
{
    uint16_t kvRpm = kv * Global_Motor_GetVSource_V();
    if((p_motor->Parameters.SpeedFeedbackRef_Rpm == 0U) || kvRpm < p_motor->Parameters.SpeedFeedbackRef_Rpm)
    {
        Motor_User_SetSpeedFeedbackRef_Rpm(p_motor, kvRpm);
    }
}

void Motor_User_SetVSpeedRef_Kv(Motor_T * p_motor, uint16_t kv)
{
    Motor_User_SetVSpeedRef_Rpm(p_motor, kv * Global_Motor_GetVSource_V());
}

/* Setting Kv overwrites higher SpeedFeedbackRef. SpeedFeedbackRef can be set independently from Kv */
void Motor_User_SetKv(Motor_T * p_motor, uint16_t kv)
{
    p_motor->Parameters.Kv = kv;
    Motor_User_SetSpeedFeedbackRef_Kv(p_motor, kv);
    Motor_User_SetVSpeedRef_Kv(p_motor, kv);
}

// void Motor_User_SetIaZero_Adcu(Motor_T * p_motor, uint16_t adcu)
// {
//     p_motor->Parameters.IaZeroRef_Adcu = adcu;
//     // PropagateSet(p_motor, Motor_ResetUnitsIa);
// }
// void Motor_User_SetIbZero_Adcu(Motor_T * p_motor, uint16_t adcu)
// {
//     p_motor->Parameters.IbZeroRef_Adcu = adcu;
//     // PropagateSet(p_motor, Motor_ResetUnitsIb);
// }
// void Motor_User_SetIcZero_Adcu(Motor_T * p_motor, uint16_t adcu)
// {
//     p_motor->Parameters.IcZeroRef_Adcu = adcu;
//     // PropagateSet(p_motor, Motor_ResetUnitsIc);
// }

void Motor_User_SetIaIbIcZero_Adcu(Motor_T * p_motor, uint16_t ia_adcu, uint16_t ib_adcu, uint16_t ic_adcu)
{
    p_motor->Parameters.IaZeroRef_Adcu = ia_adcu;
    p_motor->Parameters.IbZeroRef_Adcu = ib_adcu;
    p_motor->Parameters.IcZeroRef_Adcu = ic_adcu;
    PropagateSet(p_motor, Motor_ResetUnitsIabc);
}

void Motor_User_SetDirectionCalibration(Motor_T * p_motor, Motor_DirectionCalibration_T mode)
{
// #ifdef CONFIG_MOTOR_PROPAGATE_SET_PARAM_ENABLE
//     uint32_t ccw, cw;
//     if(p_motor->Parameters.DirectionCalibration != mode)
//     {
//         ccw = p_motor->Parameters.SpeedLimitCcw_FracS16;
//         cw = p_motor->Parameters.SpeedLimitCw_FracS16;
//         p_motor->Parameters.SpeedLimitCcw_FracS16 = cw;
//         p_motor->Parameters.SpeedLimitCw_FracS16 = ccw;
//     }
// #endif
    p_motor->Parameters.DirectionCalibration = mode;
    PropagateSet(p_motor, Motor_SetDirectionForward);
}

void Motor_User_SetPolePairs(Motor_T * p_motor, uint8_t polePairs) { p_motor->Parameters.PolePairs = polePairs; PropagateSet(p_motor, Motor_ResetUnitsSensor); }

/* Reboot unless deinit is implemented in HAL */
void Motor_User_SetSensorMode(Motor_T * p_motor, Motor_SensorMode_T mode) { p_motor->Parameters.SensorMode = mode; PropagateSet(p_motor, Motor_InitSensor); }

/******************************************************************************/
/*   */
/******************************************************************************/
#if defined(CONFIG_MOTOR_DEBUG_ENABLE)
void Motor_User_SetIPeakRef_Adcu_Debug(Motor_T * p_motor, uint16_t adcu)
{
    p_motor->Parameters.IPeakRef_Adcu = adcu;
    Motor_ResetUnitsIabc(p_motor);
}

void Motor_User_SetIPeakRef_Adcu(Motor_T * p_motor, uint16_t adcu)
{
    p_motor->Parameters.IPeakRef_Adcu = (adcu >  GLOBAL_MOTOR.I_MAX_ZTP_ADCU) ?  GLOBAL_MOTOR.I_MAX_ZTP_ADCU : adcu;
    PropagateSet(p_motor, Motor_ResetUnitsIabc);
}

void Motor_User_SetIPeakRef_MilliV(Motor_T * p_motor, uint16_t min_MilliV, uint16_t max_MilliV)
{
    uint16_t adcuZero = (uint32_t)(max_MilliV + min_MilliV) * GLOBAL_ANALOG.ADC_MAX / 2U / GLOBAL_ANALOG.ADC_VREF_MILLIV;
    uint16_t adcuMax = (uint32_t)max_MilliV * GLOBAL_ANALOG.ADC_MAX / GLOBAL_ANALOG.ADC_VREF_MILLIV;
    Motor_User_SetIPeakRef_Adcu(p_motor, adcuMax - adcuZero);
}
#endif