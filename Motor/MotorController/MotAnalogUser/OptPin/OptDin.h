#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2026 FireSourcery

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
    @file   MotorController_OptPin.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Transducer/UserIn/UserDIn_Cmd.h"


typedef enum MotorController_OptDinMode
{
    MOTOR_CONTROLLER_OPT_DIN_DISABLE,
    MOTOR_CONTROLLER_OPT_DIN_SPEED_LIMIT,
    MOTOR_CONTROLLER_OPT_DIN_I_LIMIT,
    MOTOR_CONTROLLER_OPT_DIN_PARK,
    MOTOR_CONTROLLER_OPT_DIN_FORWARD,
    MOTOR_CONTROLLER_OPT_DIN_REVERSE,
    MOTOR_CONTROLLER_OPT_DIN_SWITCH_BRAKE,
    MOTOR_CONTROLLER_OPT_DIN_MODE_COUNT,
}
MotorController_OptDinMode_T;

typedef struct OptDin_Config
{
    // bool           ActiveHigh; /* polarity */
    // uint16_t       DebounceMs;
    uint16_t       SpeedPreset_Fract16;
    uint16_t       SpeedLimit_Fract16;
    uint16_t       TorqueLimit_Fract16;
    uint16_t       SwitchBrakeFloor_Percent16;
    bool           AutoParkOnStop;
}
OptDin_Config_T;

typedef struct OptDin_State
{
    UserDIn_T * p_SwitchBrake;
    // UserDIn_T * p_Forward;
    // UserDIn_T * p_Reverse;
    // OptDin_Config_T Config;
}
OptDin_State_T;


/******************************************************************************/
/*!
    Handlers
*/
/******************************************************************************/
static inline uint16_t OptDin_FuseSwitchBrake(UserDIn_T * p_din, uint16_t switchBrakeFloor, uint16_t brake_Percent16)
{
    if (p_din == NULL) { return brake_Percent16; }
    return (uint16_t)math_max(brake_Percent16, UserDIn_ApplyGate(p_din, switchBrakeFloor));
}

#if defined(OPT_DIN_SWITCH_BRAKE_ID)
static inline UserDIn_T * OptDin_SwitchBrake(UserDIn_T * p_pins, OptDin_State_T * p_state) { (void)p_state; return &p_pins[OPT_DIN_SWITCH_BRAKE_ID]; }
#else
static inline UserDIn_T * OptDin_SwitchBrake(UserDIn_T * p_pins, OptDin_State_T * p_state) { (void)p_pins; return p_state->p_SwitchBrake; }
#endif



/******************************************************************************/
/*!
    Helpers
*/
/******************************************************************************/
/* OptDin_PollingMap_T polling ids include descriptor. move this to generic */
static inline void OptDin_ResolveBindings(OptDin_State_T * p_state, UserDIn_T * p_dins, const UserDIn_Config_T * p_configs, uint8_t count)
{
    for (uint8_t i = 0; i < count; i++)
    {
        UserDIn_T ** target = NULL;
        switch (p_configs[i].CmdId)
        {
            case MOTOR_CONTROLLER_OPT_DIN_SWITCH_BRAKE: target = &p_state->p_SwitchBrake; break;
            // case MOTOR_CONTROLLER_OPT_DIN_FORWARD:      target = &p_state->p_Forward;     break;
            // case MOTOR_CONTROLLER_OPT_DIN_REVERSE:      target = &p_state->p_Reverse;     break;
            default: break;
        }
        if (target != NULL) { *target = (*target == NULL) ? &p_dins[i] : NULL; }   /* fail-closed on duplicate */
    }
}

static inline void OptDin_ResolveCallbacks(UserDIn_T * p_dins, const UserDIn_Config_T * p_configs, uint8_t count, const UserDIn_Fn_T * p_cmdTable)
{
    for (uint8_t i = 0; i < count; i++) { p_dins[i].P_STATE->OptCmd = p_cmdTable[p_configs[i].CmdId]; }
}

static inline void OptDin_ResolveConfig(OptDin_State_T * p_state, UserDIn_T * p_dins, const UserDIn_Config_T * p_configs, uint8_t count, const UserDIn_Fn_T * p_cmdTable)
{
    OptDin_ResolveBindings(p_state, p_dins, p_configs, count);
    OptDin_ResolveCallbacks(p_dins, p_configs, count, p_cmdTable);
}

/******************************************************************************/
/*!
    [VarId] Interface
*/
/******************************************************************************/
typedef enum OptDin_ConfigId
{
    OPT_DIN_CONFIG_ID_SPEED_PRESET_FRACT16,
    OPT_DIN_CONFIG_ID_SPEED_LIMIT_FRACT16,
    OPT_DIN_CONFIG_ID_TORQUE_LIMIT_FRACT16,
    OPT_DIN_CONFIG_ID_SWITCH_BRAKE_SCALAR,
    OPT_DIN_CONFIG_ID_AUTO_PARK_ON_STOP,
}
OptDin_ConfigId_T;

static inline int OptDin_ConfigId_Get(const OptDin_Config_T * p_config, OptDin_ConfigId_T id)
{
    int value = 0;
    switch (id)
    {
        case OPT_DIN_CONFIG_ID_SPEED_PRESET_FRACT16:    value = p_config->SpeedPreset_Fract16; break;
        case OPT_DIN_CONFIG_ID_SPEED_LIMIT_FRACT16:     value = p_config->SpeedLimit_Fract16;  break;
        case OPT_DIN_CONFIG_ID_TORQUE_LIMIT_FRACT16:    value = p_config->TorqueLimit_Fract16; break;
        case OPT_DIN_CONFIG_ID_SWITCH_BRAKE_SCALAR:     value = p_config->SwitchBrakeFloor_Percent16;   break;
        case OPT_DIN_CONFIG_ID_AUTO_PARK_ON_STOP:       value = p_config->AutoParkOnStop;      break;
        default: break;
    }
    return value;
}

static inline void OptDin_ConfigId_Set(OptDin_Config_T * p_config, OptDin_ConfigId_T id, int value)
{
    switch (id)
    {
        case OPT_DIN_CONFIG_ID_SPEED_PRESET_FRACT16:    p_config->SpeedPreset_Fract16 = value; break;
        case OPT_DIN_CONFIG_ID_SPEED_LIMIT_FRACT16:     p_config->SpeedLimit_Fract16  = value; break;
        case OPT_DIN_CONFIG_ID_TORQUE_LIMIT_FRACT16:    p_config->TorqueLimit_Fract16 = value; break;
        case OPT_DIN_CONFIG_ID_SWITCH_BRAKE_SCALAR:     p_config->SwitchBrakeFloor_Percent16   = value; break;
        case OPT_DIN_CONFIG_ID_AUTO_PARK_ON_STOP:       p_config->AutoParkOnStop      = value; break;
        default: break;
    }
}


// static inline UserDIn_T * _OptDin_FindByMode(UserDIn_T * p_dins, const UserDIn_Config_T * p_configs, uint8_t count, MotorController_OptDinMode_T mode)
// {
//     UserDIn_T * found = NULL;
//     for (uint8_t i = 0U; i < count; i++)
//     {
//         if (p_configs[i].CmdId == mode)
//         {
//             if (found != NULL) { return NULL; }
//             found = &p_dins[i];
//         }
//     }
//     return found;
// }

// static inline void OptDin_ResolveBindings(OptDin_State_T * p_state, UserDIn_T * p_dins, const UserDIn_Config_T * p_configs, uint8_t count)
// {
//     // p_state->p_SwitchBrake = _OptDin_FindByMode(p_dins, p_configs, count, MOTOR_CONTROLLER_OPT_DIN_SWITCH_BRAKE);
//     // p_state->p_Forward = _OptDin_FindByMode(p_dins, p_configs, count, MOTOR_CONTROLLER_OPT_DIN_FORWARD);
//     // p_state->p_Reverse = _OptDin_FindByMode(p_dins, p_configs, count, MOTOR_CONTROLLER_OPT_DIN_REVERSE);
// }