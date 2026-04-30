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
    @file   Traction.c
    @author FireSourcery

    @brief
*/
/******************************************************************************/
#include "Traction.h"


void _Traction_InitFrom(Traction_State_T * p_fields, const Traction_Config_T * p_config)
{
    if (p_config != NULL) { p_fields->Config = *p_config; }
}

void Traction_Init(const Traction_T * p_handle)
{
    _Traction_InitFrom(p_handle->P_TRACTION_STATE, p_handle->P_NVM_CONFIG);
    // StateMachine_Init(&p_handle->STATE_MACHINE);
}

// typedef const struct MotorDrive
// {
//     Motor_FeedbackMode_T FEEDBACK_MODE;
//     void (*APPLY_CMD)(const Motor_T * p , motor_value_t value);
// }
// MotorDrive_T;

/******************************************************************************/
/*
    Called from StateMachine
    State Mode use full context
*/
/******************************************************************************/
/* alternatively convert input only */
void Traction_StartThrottleMode(const Traction_T * p_vehicle)
{
    switch (p_vehicle->P_TRACTION_STATE->Config.ThrottleMode)
    {
        case TRACTION_THROTTLE_MODE_SPEED:  Motor_Table_ApplyFeedbackMode(&p_vehicle->MOTORS, MOTOR_FEEDBACK_MODE_SPEED_CURRENT);     break;
        case TRACTION_THROTTLE_MODE_TORQUE: Motor_Table_ApplyFeedbackMode(&p_vehicle->MOTORS, MOTOR_FEEDBACK_MODE_CURRENT);           break;
        default: break;
    }

    /* alternatively from Release only */
    Motor_Table_ApplyControl(&p_vehicle->MOTORS, PHASE_VOUT_PWM);
}

/* if handle each value update with statemachine on input */
void Traction_ApplyThrottleValue(const Traction_T * p_vehicle, uint16_t userCmdThrottle)
{
    int16_t cmdValue = (int32_t)userCmdThrottle / 2;

    switch (p_vehicle->P_TRACTION_STATE->Config.ThrottleMode)
    {
        case TRACTION_THROTTLE_MODE_SPEED:  Motor_Table_SetCmdWith(&p_vehicle->MOTORS, Motor_SetSpeedMotoringCmdScalar, (int32_t)cmdValue);     break;
        case TRACTION_THROTTLE_MODE_TORQUE: Motor_Table_SetCmdWith(&p_vehicle->MOTORS, Motor_SetICmdScalar, (int32_t)cmdValue);         break;
        default: break;
    }
}

// apply hold on low speed
void Traction_StartBrakeMode(const Traction_T * p_vehicle)
{
    switch (p_vehicle->P_TRACTION_STATE->Config.BrakeMode)
    {
        case TRACTION_BRAKE_MODE_TORQUE:  Motor_Table_ApplyFeedbackMode(&p_vehicle->MOTORS, MOTOR_FEEDBACK_MODE_CURRENT);  break;
            // case TRACTION_BRAKE_MODE_VOLTAGE: Motor_Table_ApplyFeedbackMode(&p_vehicle->MOTORS, MOTOR_FEEDBACK_MODE_VOLTAGE);  break;
        default: break;
    }
    Motor_Table_ApplyControl(&p_vehicle->MOTORS, PHASE_VOUT_PWM);     /* alternatively from Release only */
}

/*!
    Always request opposite direction current
    req opposite iq, bound vq to 0 for no plugging brake

    transition from accelerating to decelerating,
    use signed ramp to transition through 0 without discontinuity
    ramp from in-direction torque to 0 to counter-direction torque

    @param[in] brake [0:32767]
*/
void Traction_ApplyBrakeValue(const Traction_T * p_vehicle, uint16_t userCmdBrake)
{
    int16_t cmdValue = 0 - ((int32_t)userCmdBrake / 2); // 32767 max

    switch (p_vehicle->P_TRACTION_STATE->Config.BrakeMode)
    {
        case TRACTION_BRAKE_MODE_TORQUE: Motor_Table_SetCmdWith(&p_vehicle->MOTORS, Motor_SetICmdScalar, cmdValue); break; /* note: torqueRamp also written in voltage mode */
            // case TRACTION_BRAKE_MODE_VOLTAGE: Motor_Table_SetCmdWith(&p_vehicle->MOTORS, Motor_SetRegenCmd, 0); break;
        default: break;
    }
}


/* an alternate cmd for float is required */
void Traction_StartDriveZero(const Traction_T * p_vehicle)
{
    switch (p_vehicle->P_TRACTION_STATE->Config.ZeroMode)
    {
        case TRACTION_ZERO_MODE_FLOAT:   Motor_Table_ApplyControl(&p_vehicle->MOTORS, PHASE_VOUT_Z);         break;
        case TRACTION_ZERO_MODE_IZERO:   Motor_Table_ForEachApply(&p_vehicle->MOTORS, Motor_ApplyTorque0);   break;
        case TRACTION_ZERO_MODE_REGEN:       /* Traction_SetRegenMotorAll(p_this); */ break;
        default: break;
    }
}

// optionally select speed ramp down
void Traction_ProcRelease(const Traction_T * p_vehicle)
{
    // Motor_Table_ForEachApply(&p_vehicle->MOTORS, Motor_Hypervisor);
}




/******************************************************************************/
/*
    Var Id
*/
/******************************************************************************/
int Traction_ConfigId_Get(const Traction_Config_T * p_config, Traction_ConfigId_T id)
{
    int value = 0;
    switch (id)
    {
        case TRACTION_CONFIG_THROTTLE_MODE:    value = p_config->ThrottleMode;                 break;
        case TRACTION_CONFIG_BRAKE_MODE:       value = p_config->BrakeMode;                    break;
        case TRACTION_CONFIG_ZERO_MODE:        value = p_config->ZeroMode;                     break;
    }
    return value;
}

void Traction_ConfigId_Set(Traction_Config_T * p_config, Traction_ConfigId_T id, int value)
{
    switch (id)
    {
        case TRACTION_CONFIG_THROTTLE_MODE:    p_config->ThrottleMode = (Traction_ThrottleMode_T)value;     break;
        case TRACTION_CONFIG_BRAKE_MODE:       p_config->BrakeMode = (Traction_BrakeMode_T)value;           break;
        case TRACTION_CONFIG_ZERO_MODE:        p_config->ZeroMode = (Traction_ZeroMode_T)value;             break;
        default: break;
    }
}

// void Traction_VarId_Set(const Traction_Input_T * p_traction, Traction_VarId_T id, int value)
// {
//     switch (id)
//     {
//         case TRACTION_VAR_DIRECTION:   Traction_Input_PollDirectionEdge(p_traction, (sign_t)value);       break; // call outer passthrough in most cases
//         case TRACTION_VAR_THROTTLE:    Traction_Input_PollThrottle(p_traction, (uint16_t)value);      break;
//         case TRACTION_VAR_BRAKE:       Traction_Input_PollBrake(p_traction, (uint16_t)value);         break;
//     }
// }

// int Traction_VarId_Get(const Traction_Input_T * p_traction, Traction_VarId_T id)
// {
//     int value = 0;
//     switch (id)
//     {
//         case TRACTION_VAR_DIRECTION:   value = p_traction->Direction;      break;
//         case TRACTION_VAR_THROTTLE:    value = p_traction->ThrottleValue;  break;
//         case TRACTION_VAR_BRAKE:       value = p_traction->BrakeValue;     break;
//     }
//     return value;
// }


// optionally handle by motor layer
// static inline int16_t Traction_Input_GetMotorCmd(const Traction_Input_T * p_input)
// {
//     switch (p_input->DriveCmd)
//     {
//         case TRACTION_CMD_BRAKE:     return -(int16_t)p_input->BrakeValue; // as motoring
//         case TRACTION_CMD_BRAKE:     return -(int16_t)p_input->BrakeValue * p_input->Direction; // as signed torque
//         case TRACTION_CMD_THROTTLE:  return (int16_t)p_input->ThrottleValue * p_input->Direction;
//         case TRACTION_CMD_RELEASE:   return 0;
//         default: return 0;
//     }
// }

// alternatively as input conversion,

// interface for   data common

// static inline void Traction_Input_FromAnalogUser(Traction_Input_T * p_user, MotAnalogUser_T * P_analog)
// {
//     p_user->ThrottleValue = MotAnalogUser_GetAInValue(P_analog, MOT_ANALOG_USER_AIN_THROTTLE);
//     p_user->BrakeValue = MotAnalogUser_GetAInValue(P_analog, MOT_ANALOG_USER_AIN_BRAKE);
// }

// void Traction_Input_ToMotorInput (const Traction_Input_T * p_user, Motor_Input_T * P_input )
// {
    //     switch (dir)
    //     {
    //         case MOTOR_CONTROLLER_DIRECTION_PARK:
    //             P_input->Direction = MOTOR_DIRECTION_NULL;
    //             P_input->PhaseOutput = PHASE_VOUT_Z;
    //             break;
    //         case MOTOR_CONTROLLER_DIRECTION_REVERSE:
    //             P_input->Direction = MOTOR_DIRECTION_CW;
    //             P_input->PhaseOutput = PHASE_VOUT_PWM;
    //             break;
    //         case MOTOR_CONTROLLER_DIRECTION_FORWARD:
    //             P_input->Direction = MOTOR_DIRECTION_CCW;
    //             P_input->PhaseOutput = PHASE_VOUT_PWM;
    //             break;
    //         case MOTOR_CONTROLLER_DIRECTION_NEUTRAL:
    //             P_input->PhaseOutput = PHASE_VOUT_Z;
    //             break;
    //         default:
    //             break;
    //     }
//
// }

// static inline void Input_FromThrottle(Motor_Input_T * p_user, Traction_ThrottleMode_T mode, uint16_t throttle)
// {
//     p_user->CmdValue = (int32_t)throttle / 2; // [0:32767]

//     switch (mode)
//     {
//         case TRACTION_THROTTLE_MODE_SPEED:   p_user->FeedbackMode = MOTOR_FEEDBACK_MODE_SPEED_CURRENT;      break;
//         case TRACTION_THROTTLE_MODE_TORQUE:  p_user->FeedbackMode = MOTOR_FEEDBACK_MODE_CURRENT;            break;
//         default: break;
//     }
// }

// static inline void Traction_Input_FromProtocol(Traction_T * vehicle, Motor_Input_T * p_user, id, value)

// static inline void Traction_Input_FromAnalogUser(Traction_T * vehicle, Motor_Input_T * p_user, MotAnalogUser_T * P_analog)
// {
//     p_user->CmdValue = (int32_t)MotAnalogUser_GetThrottle(P_analog) / 2; // [0:32767]

//     switch (vehicle->P_TRACTION_STATE->Config.ThrottleMode)
//     {
//         case TRACTION_THROTTLE_MODE_SPEED:   p_user->FeedbackMode = MOTOR_FEEDBACK_MODE_SPEED_CURRENT;      break;
//         case TRACTION_THROTTLE_MODE_TORQUE:  p_user->FeedbackMode = MOTOR_FEEDBACK_MODE_CURRENT;            break;
//         default: break;
//     }
// }
