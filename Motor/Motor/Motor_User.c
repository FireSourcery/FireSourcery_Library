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
    @brief  User Input/Output interface function, including error checking
    @version V0
*/
/******************************************************************************/
#include "Motor_User.h"
#include "System/Critical/Critical.h"

static int32_t Scale16(uint16_t scalar16, int32_t value) { return (int32_t)scalar16 * value / 65536; }

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
        SetModeCmd  - Check/sets control mode and cmd value
*/
/******************************************************************************/

/******************************************************************************/
/*!
    Getters satisfy generic use. Setters are specific to control mode.
*/
/******************************************************************************/
int32_t Motor_User_GetCmd(const MotorPtr_T p_motor) { return Motor_DirectionalValueOf(p_motor, Linear_Ramp_GetTarget(&p_motor->Ramp)); }
int32_t Motor_User_GetSetPoint(const MotorPtr_T p_motor) { return Motor_DirectionalValueOf(p_motor, Linear_Ramp_GetOutput(&p_motor->Ramp)); }
// void Motor_User_ClearState(MotorPtr_T p_motor) { Linear_Ramp_SetOutputState(&p_motor->Ramp, 0); }

/******************************************************************************/
/*!
    Voltage Mode
*/
/******************************************************************************/
void Motor_User_SetVoltageMode(MotorPtr_T p_motor)
{
    Motor_ActivateControl(p_motor, MOTOR_FEEDBACK_MODE_VOLTAGE);
}

/*!
    @param[in] voltage [-32768:32767]
*/
void Motor_User_SetVoltageCmdValue(MotorPtr_T p_motor, int16_t voltageCmd)
{
    int32_t voltageCmdIn = (voltageCmd > 0) ? voltageCmd : 0; /* Reverse voltage use change direction, no plugging */
    Motor_SetCmd(p_motor, voltageCmdIn);
}

// void Motor_User_SetVoltageModeCmd(MotorPtr_T p_motor, int16_t voltageCmd)
// {
//     Motor_User_SetVoltageMode(p_motor);
//     Motor_User_SetVoltageCmdValue(p_motor, voltageCmd);
// }

/******************************************************************************/
/*!
    Torque Mode
*/
/******************************************************************************/
void Motor_User_SetTorqueMode(MotorPtr_T p_motor)
{
    Motor_ActivateControl(p_motor, MOTOR_FEEDBACK_MODE_CURRENT);
}

/*!
    @param[in] torque [-32768:32767]
*/
void Motor_User_SetTorqueCmdValue(MotorPtr_T p_motor, int16_t torqueCmd)
{
    int32_t torqueCmdIn = (torqueCmd > 0) ? Scale16(p_motor->ILimitMotoring_Scalar16, torqueCmd) : Scale16(p_motor->ILimitGenerating_Scalar16, torqueCmd);
    Motor_SetCmd(p_motor, torqueCmdIn);
}

// void Motor_User_SetTorqueModeCmd(MotorPtr_T p_motor, int16_t torqueCmd)
// {
//     Motor_User_SetTorqueMode(p_motor);
//     Motor_User_SetTorqueCmdValue(p_motor, torqueCmd);
// }

/******************************************************************************/
/*!
    Speed Mode
*/
/******************************************************************************/
/*!
    Default speed mode is speed torque mode
*/
void Motor_User_SetSpeedMode(MotorPtr_T p_motor)
{
    Motor_ActivateControl(p_motor, MOTOR_FEEDBACK_MODE_SPEED_CURRENT);
}

/*!
    Only allow forward direction, reverse direction use MOTOR_DIRECTION,
        Hall sensor directional read, ILimit directional set
    @param[in] speed [-32768:32767]
*/
void Motor_User_SetSpeedCmdValue(MotorPtr_T p_motor, int16_t speedCmd)
{
    int16_t speedCmdIn = (speedCmd > 0) ? Scale16(p_motor->SpeedLimitDirect_Scalar16, speedCmd) : 0;
    Motor_SetCmd(p_motor, speedCmdIn);
}

// void Motor_User_SetSpeedModeCmd(MotorPtr_T p_motor, int16_t speedCmd)
// {
//     Motor_User_SetSpeedMode(p_motor);
//     Motor_User_SetSpeedCmdValue(p_motor, speedCmd);
// }

/******************************************************************************/
/*!
    Position Mode
*/
/******************************************************************************/
/*!
    @param[in] angle [0:65535]
    todo
*/
void Motor_User_SetPositionCmdValue(MotorPtr_T p_motor, uint16_t angle)
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
void Motor_User_SetOpenLoopMode(MotorPtr_T p_motor)
{
    Motor_ActivateControl(p_motor, MOTOR_FEEDBACK_MODE_OPEN_LOOP_SCALAR);
}

void Motor_User_SetOpenLoopSpeed(MotorPtr_T p_motor, int32_t speed_Frac16)
{
    Linear_Ramp_SetTarget(&p_motor->OpenLoopSpeedRamp, speed_Frac16);
}

/*!

*/
void Motor_User_SetOpenLoopCmdValue(MotorPtr_T p_motor, int16_t ivCmd)
{
    int32_t ivCmdIn = math_clamp(ivCmd, 0, p_motor->Config.OpenLoopPower_Scalar16 / 2U);
    Motor_SetCmd(p_motor, ivCmdIn);
}

// void Motor_User_SetOpenLoopModeCmd(MotorPtr_T p_motor, int16_t ivMagnitude)
// {
//     Motor_User_SetOpenLoopMode(p_motor);
//     Motor_User_SetOpenLoopCmdValue(p_motor, ivMagnitude);
// }
#endif


/******************************************************************************/
/*!
    Generic Mode
*/
/******************************************************************************/
/*!
    @param[in] userCmd[-32768:32767] A union value, bounds to be determined by active mode
*/
void Motor_User_SetActiveCmdValue(MotorPtr_T p_motor, int16_t userCmd)
{
    Motor_FeedbackMode_T flags = p_motor->FeedbackMode;

    if      (flags.OpenLoop == 1U)  { Motor_User_SetOpenLoopCmdValue(p_motor, userCmd); }
    // else if (flags.Position == 1U)  { Motor_User_SetPositionCmdValue(p_motor, userCmd); }
    else if (flags.Speed == 1U)     { Motor_User_SetSpeedCmdValue(p_motor, userCmd); }
    else if (flags.Current == 1U)   { Motor_User_SetTorqueCmdValue(p_motor, userCmd); }
    else                            { Motor_User_SetVoltageCmdValue(p_motor, userCmd); }
}

// void Motor_User_ActivateDefaultFeedbackMode(MotorPtr_T p_motor)
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
void Motor_User_ForceDisableControl(MotorPtr_T p_motor)
{
    Phase_Float(&p_motor->Phase);
    Motor_SetCmd(p_motor, 0);
    StateMachine_SetInput(&p_motor->StateMachine, MSM_INPUT_RELEASE, STATE_MACHINE_INPUT_VALUE_NULL);
}

/*
    always State machine checked Disable
*/
/* no critical for transition. no sync issue if states do no contain transition during proc, or may result in running input function on a different state */
// TryRelease transition to FREEWHEEL
bool Motor_User_TryRelease(MotorPtr_T p_motor)
{
    // Motor_SetCmd(p_motor, 0);
    StateMachine_SetInput(&p_motor->StateMachine, MSM_INPUT_RELEASE, STATE_MACHINE_INPUT_VALUE_NULL);
}

// void Motor_ActivateControl(MotorPtr_T p_motor) { StateMachine_SetInput(&p_motor->StateMachine, MSM_INPUT_CONTROL, STATE_MACHINE_INPUT_VALUE_NULL); }

// return if stop
bool Motor_User_TryHold(MotorPtr_T p_motor) { StateMachine_SetInput(&p_motor->StateMachine, MSM_INPUT_HOLD, STATE_MACHINE_INPUT_VALUE_NULL); }

/******************************************************************************/
/*!
    Direction
*/
/******************************************************************************/
/*
    Does not need critical section if PWM interrupt does not read/write direction
*/
bool Motor_User_TryDirection(MotorPtr_T p_motor, Motor_Direction_T direction)
{
    if(p_motor->Direction != direction) { StateMachine_SetInput(&p_motor->StateMachine, MSM_INPUT_DIRECTION, direction); }
    return (direction == p_motor->Direction);
}

bool Motor_User_TryDirectionForward(MotorPtr_T p_motor) { return Motor_User_TryDirection(p_motor, p_motor->Config.DirectionForward); }
bool Motor_User_TryDirectionReverse(MotorPtr_T p_motor) { return Motor_User_TryDirection(p_motor, Motor_DirectionReverse(p_motor)); }


/******************************************************************************/
/*!
    Getters/Setters with interface function
*/
/******************************************************************************/


/*
   User Conditional - set compare with array
*/
bool Motor_User_TrySpeedLimit(MotorPtr_T p_motor, uint16_t scalar16)
{
    Motor_SetSpeedLimitEntry(p_motor, scalar16, MOTOR_SPEED_LIMIT_ACTIVE_USER);
}

bool Motor_User_TryILimit(MotorPtr_T p_motor, uint16_t scalar16)
{
    Motor_SetILimitEntry(p_motor, scalar16, MOTOR_I_LIMIT_ACTIVE_USER);
}

bool Motor_User_ClearSpeedLimit(MotorPtr_T p_motor)
{
}

bool Motor_User_ClearILimit(MotorPtr_T p_motor)
{
}


/******************************************************************************/
/*
    Calibration State - Blocking Run Calibration functions

    Check SensorMode on input to determine start, or proc may run before checking during entry
    alternatively, implement critical, move check into state machine, share 1 active function this way.
*/
/******************************************************************************/
void Motor_User_CalibrateAdc(MotorPtr_T p_motor)
{
    StateMachine_SetInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, MOTOR_CALIBRATION_STATE_ADC);
}

void Motor_User_CalibrateSensor(MotorPtr_T p_motor)
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
int16_t Motor_User_GetGroundSpeed_Kmh(MotorPtr_T p_motor)
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

int16_t Motor_User_GetGroundSpeed_Mph(MotorPtr_T p_motor)
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

void Motor_User_SetGroundSpeed_Kmh(MotorPtr_T p_motor, uint32_t wheelDiameter_Mm, uint32_t wheelToMotorRatio_Factor, uint32_t wheelToMotorRatio_Divisor)
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

void Motor_User_SetGroundSpeed_Mph(MotorPtr_T p_motor, uint32_t wheelDiameter_Inch10, uint32_t wheelToMotorRatio_Factor, uint32_t wheelToMotorRatio_Divisor)
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



// /******************************************************************************/
// /*!
//     Vehicle Wrapper
//     Throttle and Brake accept uint16_t, wrapped functions use int16_t
//     - UserCmdValue value in configured FeedbackModeDefault
// */
// /******************************************************************************/
// /*!
//     @param[in] throttle [0:65535] throttle percentage, 65535 => speed limit
// */
// void Motor_User_SetThrottleCmd(MotorPtr_T p_motor, uint16_t throttle)
// {
//     Motor_User_SetDefaultModeCmd(p_motor, throttle / 2U); // change to speed?
// }

// /*!
//     Always request opposite direction current
//     req opposite iq, bound vq to 0 for no plugging brake

//     transition from accelerating to decelerating,
//     use signed ramp to transition through 0 without discontinuity
//     ramp from in-direction torque to 0 to counter-direction torque

//     @param[in] brake [0:65535]
// */
// void Motor_User_SetBrakeCmd(MotorPtr_T p_motor, uint16_t brake)
// {

//     // if(p_motor->FeedbackMode.Hold == 0U)
//     // {
//     if(Motor_User_GetSpeed_UFrac16(p_motor) > INT16_MAX / 100U)
//     {
//         Motor_User_SetTorqueModeCmd(p_motor, (int32_t)0 - (brake / 2U));
//     }
//     else
//     {
//         // p_motor->FeedbackMode.Hold = 1U;  //clears on throttle
//         // Phase_Ground(&p_motor->Phase);
//         // Phase_Float(&p_motor->Phase);
//         Motor_User_TryRelease(p_motor);
//     }
//     // }
// }

// void Motor_User_SetVBrakeCmd(MotorPtr_T p_motor, uint16_t brake)
// {
//     // Motor_User_SetScalarModeCmd(p_motor, (65535U - brake)); /* Higher brake => lower voltage */
// }

// void Motor_User_SetCruise(MotorPtr_T p_motor)
// {
//     Motor_User_SetTorqueModeCmd(p_motor, 0U);
// }