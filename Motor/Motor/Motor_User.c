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

static int32_t Scale16(uint16_t percent16, int32_t value) { return (int32_t)percent16 * value / 65536; }

/******************************************************************************/
/*!
    StateMachine handled inputs
*/
/******************************************************************************/
/*
    Motor State Machine Thread Safety
    State Proc in PWM thread. User Input [Motor_ActivateControl] in Main thread.

    Async Mode - Need critical section during input.
    No Critical during transistion ->
        Prev State Proc may run before Entry.
        control proc may overwrite pid state set

    _StateMachine_ProcStateTransition inside Proc will run on PWM thread

    Inputs do not directly proc transition

    Sync Mode - Input value before Input Id, sufficient?
    Must check input flags every pwm cycle.

    CmdValue (Ramp Target) and CmdMode selectively sync inputs for StateMachine
*/
static void Motor_ActivateControl(Motor_T * p_motor, Motor_FeedbackMode_T mode)
{
    // _Critical_DisableIrq(); /* is this needed for setting buffers? */
    // if(p_motor->FeedbackMode.Word != mode.Word)
    { StateMachine_SetInput(&p_motor->StateMachine, MSM_INPUT_CONTROL, mode.Word); }
    // _Critical_EnableIrq();
}

/* Generic array functions use */
static void Motor_ActivateControl_Cast(Motor_T * p_motor, uint8_t modeWord) { Motor_ActivateControl(p_motor, Motor_FeedbackMode_Cast(modeWord)); }

/******************************************************************************/
/*!
    Feedback Control Modes: torque/speed/position, above Commutation layer: foc/six-step

    UserCmdValue functions
    UserCmdValue sign +/- indicates along or against Direction selected. NOT virtual CW/CCW.
        Handle direction and limits check on input.
        Limits check again in feedback loop, as input may have stopped
    Call regularly to update cmd value, lower frequency compared to control loop, ~100-1000Hz.

    SetMode     - Invokes StateMachine - Sets control mode only
    SetCmdValue - Without invoking StateMachine - Sets buffered cmd value, sets on all states even when inactive
*/
/******************************************************************************/

/******************************************************************************/
/*!
    Voltage Mode
*/
/******************************************************************************/
void Motor_User_StartVoltageMode(Motor_T * p_motor) { Motor_ActivateControl(p_motor, MOTOR_FEEDBACK_MODE_VOLTAGE); }

/*!
    @param[in] voltage [-32768:32767]
*/
void Motor_User_SetVoltageCmdValue(Motor_T * p_motor, int16_t voltageCmd)
{
    int32_t limitedCmd = (voltageCmd > 0) ? voltageCmd : 0; /* Reverse voltage use change direction, no plugging */
    Motor_SetCmd(p_motor, limitedCmd);
}

// [v/2 +/- scalar * ]
void Motor_User_SetVSpeedScalarCmd(Motor_T * p_motor, int16_t scalar)
{
    Motor_User_SetVoltageCmdValue(p_motor, Motor_GetVSpeed_Fract16(p_motor) / 2);
}


/******************************************************************************/
/*!
    Current Mode
*/
/******************************************************************************/
void Motor_User_StartIMode(Motor_T * p_motor) { Motor_ActivateControl(p_motor, MOTOR_FEEDBACK_MODE_CURRENT); }

/*!
    @param[in] i [-32768:32767]
*/
void Motor_User_SetICmdValue(Motor_T * p_motor, int16_t iCmd)
{
    int32_t limitedCmd = (iCmd > 0) ? Scale16(p_motor->ILimitMotoring_Percent16, iCmd) : Scale16(p_motor->ILimitGenerating_Percent16, iCmd);
    Motor_SetCmd(p_motor, limitedCmd);
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
void Motor_User_SetTorqueCmdValue(Motor_T * p_motor, int16_t torqueCmd)
{
    // int32_t limitedCmd = (torqueCmd > 0) ? Scale16(p_motor->ILimitMotoring_Percent16, torqueCmd) : Scale16(p_motor->ILimitGenerating_Percent16, torqueCmd);

    if (torqueCmd < 0)
    {
        if (Motor_FOC_GetIPhase_UFract16(p_motor) > (INT16_MAX / 20)) // 5%
        {
            Motor_User_SetICmdValue(p_motor, torqueCmd);
        }
        else
        {
            // Motor_User_SetICmdValue(p_motor, 0U);
            Motor_User_TryRelease(p_motor);
        }
    }
    else
    {
        Motor_User_SetICmdValue(p_motor, torqueCmd);
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
void Motor_User_StartSpeedMode(Motor_T * p_motor) { Motor_ActivateControl(p_motor, MOTOR_FEEDBACK_MODE_SPEED_CURRENT); }

/*!
    Only allow forward direction, reverse direction use MOTOR_DIRECTION,
        Hall sensor directional read, ILimit directional set
    @param[in] speed [-32768:32767]
*/
void Motor_User_SetSpeedCmdValue(Motor_T * p_motor, int16_t speedCmd)
{
    // int32_t limitedCmd = (p_motor->Config.DirectionForward == MOTOR_DIRECTION_CCW) ? p_motor->SpeedLimitForward_Percent16 : p_motor->SpeedLimitReverse_Percent16;
    int16_t limitedCmd = (speedCmd > 0) ? Scale16(Limit_GetUpper(&p_motor->SpeedLimit), speedCmd) : 0;
    Motor_SetCmd(p_motor, limitedCmd);
}

/******************************************************************************/
/*!
    Position Mode
*/
/******************************************************************************/
/*!
    @param[in] angle [0:65535]
*/
void Motor_User_SetPositionCmdValue(Motor_T * p_motor, uint16_t angle)
{
    // _Motor_User_ActivateControlMode(p_motor, MOTOR_FEEDBACK_MODE_POSITION_SPEED_CURRENT);
    // Motor_SetCmd(p_motor, angle);
}

/******************************************************************************/
/*!
    Open Loop
*/
/******************************************************************************/
#if defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE) || defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)
/*!

*/
void Motor_User_StartOpenLoopMode(Motor_T * p_motor) { Motor_ActivateControl(p_motor, MOTOR_FEEDBACK_MODE_OPEN_LOOP_SCALAR); }

void Motor_User_SetOpenLoopSpeed(Motor_T * p_motor, int32_t speed_Fract16)
{
    Linear_Ramp_SetTarget(&p_motor->OpenLoopSpeedRamp, speed_Fract16);
}

/*!

*/
void Motor_User_SetOpenLoopCmdValue(Motor_T * p_motor, int16_t ivCmd)
{
    int32_t ivCmdIn = math_clamp(ivCmd, 0, p_motor->Config.OpenLoopPower_Percent16 / 2U);
    Motor_SetCmd(p_motor, ivCmdIn);
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

    if      (flags.OpenLoop == 1U)  { Motor_User_SetOpenLoopCmdValue(p_motor, userCmd); }
    // else if (flags.Position == 1U)  { Motor_User_SetPositionCmdValue(p_motor, userCmd); }
    else if (flags.Speed == 1U)     { Motor_User_SetSpeedCmdValue(p_motor, userCmd); }
    else if (flags.Current == 1U)   { Motor_User_SetTorqueCmdValue(p_motor, userCmd); }
    else                            { Motor_User_SetVoltageCmdValue(p_motor, userCmd); }
}

// void Motor_User_ActivateDefaultFeedbackMode(Motor_T * p_motor)
// {
//     Motor_ActivateControl(p_motor, p_motor->Config.FeedbackModeDefault);
// }



/******************************************************************************/
/*!
    On/Off Control
*/
/******************************************************************************/
/*
    Force Disable control Non StateMachine checked
*/
void Motor_User_ForceDisableControl(Motor_T * p_motor)
{
    Phase_Float(&p_motor->Phase);
    Motor_SetCmd(p_motor, 0);
    StateMachine_SetInput(&p_motor->StateMachine, MSM_INPUT_RELEASE, STATE_MACHINE_INPUT_VALUE_NULL);
}

// void Motor_ActivateControl(Motor_T * p_motor) { StateMachine_SetInput(&p_motor->StateMachine, MSM_INPUT_CONTROL, STATE_MACHINE_INPUT_VALUE_NULL); }

/*
    always State machine checked Disable
*/
/* no critical for transition. no sync issue if states do no contain transition during proc, or may result in running input function on a different state */
// TryRelease transition to FREEWHEEL
bool Motor_User_TryRelease(Motor_T * p_motor)
{
    // Motor_SetCmd(p_motor, 0);
    StateMachine_SetInput(&p_motor->StateMachine, MSM_INPUT_RELEASE, STATE_MACHINE_INPUT_VALUE_NULL);
    return (StateMachine_GetActiveStateId(&p_motor->StateMachine) == MSM_STATE_ID_FREEWHEEL ||
        StateMachine_GetActiveStateId(&p_motor->StateMachine) == MSM_STATE_ID_STOP);
}

bool Motor_User_TryHold(Motor_T * p_motor)
{
    StateMachine_SetInput(&p_motor->StateMachine, MSM_INPUT_HOLD, STATE_MACHINE_INPUT_VALUE_NULL);
    return (StateMachine_GetActiveStateId(&p_motor->StateMachine) == MSM_STATE_ID_STOP); // && PwmA+B+C == 0
}

/******************************************************************************/
/*!
    Direction
*/
/******************************************************************************/
/*
    Does not need critical section if PWM interrupt does not read/write direction
*/
bool Motor_User_TryDirection(Motor_T * p_motor, Motor_Direction_T direction)
{
    if(p_motor->Direction != direction) { StateMachine_SetInput(&p_motor->StateMachine, MSM_INPUT_DIRECTION, direction); }
    return (direction == p_motor->Direction);
}

bool Motor_User_TryDirectionForward(Motor_T * p_motor) { return Motor_User_TryDirection(p_motor, p_motor->Config.DirectionForward); }
bool Motor_User_TryDirectionReverse(Motor_T * p_motor) { return Motor_User_TryDirection(p_motor, Motor_GetDirectionReverse(p_motor)); }


/******************************************************************************/
/*!
    Getters/Setters with interface function
*/
/******************************************************************************/
/*
   User Conditional - set compare with array
*/
bool Motor_User_TrySpeedLimit(Motor_T * p_motor, uint16_t percent16)    { return Motor_SetSpeedLimitEntry(p_motor, MOTOR_SPEED_LIMIT_ACTIVE_USER, percent16); }
bool Motor_User_ClearSpeedLimit(Motor_T * p_motor)                      { return Motor_ClearSpeedLimitEntry(p_motor, MOTOR_SPEED_LIMIT_ACTIVE_USER); }
bool Motor_User_TryILimit(Motor_T * p_motor, uint16_t percent16)        { return Motor_SetILimitEntry(p_motor, MOTOR_I_LIMIT_ACTIVE_USER, percent16); }
bool Motor_User_ClearILimit(Motor_T * p_motor)                          { return Motor_ClearILimitEntry(p_motor, MOTOR_I_LIMIT_ACTIVE_USER); }


/******************************************************************************/
/*
    Calibration State - Blocking Run Calibration functions

    Check SensorMode on input to determine start, or proc may run before checking during entry
    alternatively, implement critical, move check into state machine, share 1 active function this way.
*/
/******************************************************************************/
void Motor_User_CalibrateAdc(Motor_T * p_motor)
{
    StateMachine_SetInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, MOTOR_CALIBRATION_STATE_ADC);
}

void Motor_User_CalibrateSensor(Motor_T * p_motor)
{
    switch(p_motor->Config.SensorMode)
    {
        case MOTOR_SENSOR_MODE_HALL:        StateMachine_SetInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, MOTOR_CALIBRATION_STATE_HALL);       break;
        case MOTOR_SENSOR_MODE_ENCODER:     StateMachine_SetInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, MOTOR_CALIBRATION_STATE_ENCODER);    break;
        #if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
        case MOTOR_SENSOR_MODE_SIN_COS:     StateMachine_SetInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, MOTOR_CALIBRATION_STATE_SIN_COS);    break;
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


