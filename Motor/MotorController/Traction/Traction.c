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
    // p_fields->p_ThrottleDrive = &THROTTLE_MAPS[p_fields->Config.ThrottleMode];
    // p_fields->p_BrakeDrive = &BRAKE_MAPS[p_fields->Config.BrakeMode];
}

void Traction_Init(const Traction_T * p_handle)
{
    _Traction_InitFrom(p_handle->P_TRACTION_STATE, p_handle->P_NVM_CONFIG);
    // StateMachine_Init(&p_handle->STATE_MACHINE);
}

// void _Traction_ApplyThrottleValue(Motor_T * p_motor, Traction_Input_T * p_vehicle)
// {
//     Motor_SetSpeedMotoringCmdScalar(p_motor->P_MOTOR, p_vehicle->ThrottleValue / 2);
// }

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

// handle select direction align
// static const Motor_Drive_T THROTTLE_MAPS[] =
// {
//     [TRACTION_THROTTLE_MODE_SPEED]  = { MOTOR_FEEDBACK_MODE_SPEED_CURRENT, Motor_SetSpeedMotoringCmdScalar  },
//     [TRACTION_THROTTLE_MODE_TORQUE] = { MOTOR_FEEDBACK_MODE_CURRENT, Motor_SetICmdScalar },
// };

// static const Motor_Drive_T BRAKE_MAPS[] =
// {
//     [TRACTION_BRAKE_MODE_TORQUE] = { MOTOR_FEEDBACK_MODE_CURRENT, Motor_SetICmdScalar },
// };


// Motor_Input_T Traction_ToMotorInput(const Traction_State_T * p_traction)
// {
//     const Traction_Input_T * p_in = &p_traction->Input;
//     const Traction_Config_T * p_config = &p_traction->Config;

//     switch (p_in->DriveCmd)
//     {
//         case TRACTION_CMD_THROTTLE: return (Motor_Input_T)
//         {
//             .FeedbackMode = (p_config->ThrottleMode == TRACTION_THROTTLE_MODE_SPEED) ? MOTOR_FEEDBACK_MODE_SPEED_CURRENT : MOTOR_FEEDBACK_MODE_CURRENT,
//             .CmdValue = (int16_t)(p_in->ThrottleValue / 2),
//             .PhaseOutput = PHASE_VOUT_PWM,
//             .Direction = p_in->Direction,
//         };
//         case TRACTION_CMD_BRAKE: return (Motor_Input_T)
//         {
//             .FeedbackMode = MOTOR_FEEDBACK_MODE_CURRENT,
//             .PhaseOutput = PHASE_VOUT_PWM,
//             .CmdValue = -(int16_t)(p_in->BrakeValue / 2),
//             .Direction = p_in->Direction,
//         };
//         case TRACTION_CMD_RELEASE: return (Motor_Input_T)
//         {
//             .PhaseOutput = (p_config->ZeroMode == TRACTION_ZERO_MODE_FLOAT) ? PHASE_VOUT_Z : PHASE_VOUT_PWM,
//             .CmdValue = 0,
//             .Direction = p_in->Direction,
//         };
//         default: return (Motor_Input_T) { .PhaseOutput = PHASE_VOUT_Z };
//     }
// }

// Motor_Input_T Traction_ToMotorInput(const Traction_State_T * p_traction)
// {
//     const Traction_Input_T * p_in = &p_traction->Input;
//     const Traction_Config_T * p_config = &p_traction->Config;

//     switch (p_in->DriveCmd)
//     {
//         case TRACTION_CMD_THROTTLE: return (Motor_Input_T)
//         {
//             .FeedbackMode = (p_config->ThrottleMode == TRACTION_THROTTLE_MODE_SPEED) ? MOTOR_FEEDBACK_MODE_SPEED_CURRENT : MOTOR_FEEDBACK_MODE_CURRENT,
//             .CmdValue = (int16_t)(p_in->ThrottleValue / 2),
//             .PhaseOutput = PHASE_VOUT_PWM,
//             .Direction = p_in->Direction,
//         };
//         case TRACTION_CMD_BRAKE: return (Motor_Input_T)
//         {
//             .FeedbackMode = MOTOR_FEEDBACK_MODE_CURRENT,
//             .PhaseOutput = PHASE_VOUT_PWM,
//             .CmdValue = -(int16_t)(p_in->BrakeValue / 2),
//             .Direction = p_in->Direction,
//         };
//         case TRACTION_CMD_RELEASE: return (Motor_Input_T)
//         {
//             .PhaseOutput = (p_config->ZeroMode == TRACTION_ZERO_MODE_FLOAT) ? PHASE_VOUT_Z : PHASE_VOUT_PWM,
//             .CmdValue = 0,
//             .Direction = p_in->Direction,
//         };
//         default: return (Motor_Input_T) { .PhaseOutput = PHASE_VOUT_Z };
//     }
// }