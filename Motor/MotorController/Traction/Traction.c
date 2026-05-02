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


/*

*/
void Traction_InitFrom(Traction_T * p_traction, const Traction_Config_T * p_config)
{
    if (p_config != NULL) { p_traction->Config = *p_config; }
    // p_traction->p_ThrottleDrive = &THROTTLE_MAPS[p_traction->Config.ThrottleMode];
    // p_traction->p_BrakeDrive = &BRAKE_MAPS[p_traction->Config.BrakeMode];
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

// void _Traction_VarId_Set(const Traction_Input_T * p_traction, Traction_VarId_T id, int value)
// {
//     switch (id)
//     {
//         case TRACTION_VAR_DIRECTION:   Traction_Input_PollDirectionEdge(p_traction, (sign_t)value);       break; // call outer passthrough in most cases
//         case TRACTION_VAR_THROTTLE:    Traction_Input_PollThrottle(p_traction, (uint16_t)value);      break;
//         case TRACTION_VAR_BRAKE:       Traction_Input_PollBrake(p_traction, (uint16_t)value);         break;
//     }
// }

// int _Traction_VarId_Get(const Traction_Input_T * p_traction, Traction_VarId_T id)
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

