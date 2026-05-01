#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery

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
    @file   MotorController_AnalogUser.h
    @author FireSourcery
    @brief  Facade — VarId / ConfigId surface kept on the MotAnalogUser_*
            namespace for protocol-layer interface stability. Implementations
            now resolve against MotorController_T directly:
              - pedals via AINS[MOT_AIN_THROTTLE / MOT_AIN_BRAKE]
              - direction pins via SHIFTER
              - handbrake via OptDin_SwitchBrake() and OptDinConfig
*/
/******************************************************************************/
#include "Transducer/UserIn/UserAIn.h"
#include "Transducer/UserIn/UserDIn.h"
#include "MotAnalogUser.h"
#include "../MotorController.h"
#include "../MotorController_User.h"
#include "OptPin/MotorController_SwitchBrake.h"
#include <stdint.h>
#include <stdbool.h>



/******************************************************************************/
/*

*/
/******************************************************************************/
typedef enum MotAnalogUser_VarId
{
    MOT_ANALOG_USER_THROTTLE,
    MOT_ANALOG_USER_THROTTLE_DIN,
    MOT_ANALOG_USER_BRAKE,
    MOT_ANALOG_USER_BRAKE_DIN,
    MOT_ANALOG_USER_SWITCH_BRAKE_DIN,
    MOT_ANALOG_USER_FORWARD_DIN,
    MOT_ANALOG_USER_NEUTRAL_DIN,
    MOT_ANALOG_USER_REVERSE_DIN,
    //
    // MOT_ANALOG_USER_OPT_DIN,
}
MotAnalogUser_VarId_T;

typedef enum MotAnalogUser_ConfigId
{
    MOT_ANALOG_USER_THROTTLE_ZERO_ADCU,
    MOT_ANALOG_USER_THROTTLE_MAX_ADCU,
    MOT_ANALOG_USER_THROTTLE_EDGE_PIN_IS_ENABLE,
    MOT_ANALOG_USER_BRAKE_ZERO_ADCU,
    MOT_ANALOG_USER_BRAKE_MAX_ADCU,
    MOT_ANALOG_USER_BRAKE_EDGE_PIN_IS_ENABLE,
    MOT_ANALOG_USER_SWITCH_BRAKE_VALUE,
    MOT_ANALOG_USER_SWITCH_BRAKE_IS_ENABLE,
    MOT_ANALOG_USER_DIRECTION_PINS,
    //
    // MOT_ANALOG_USER_OPT_DIN_FN,
    // MOT_ANALOG_USER_OPT_DIN_IS_ENABLED,
}
MotAnalogUser_ConfigId_T;

#include "Transducer/UserIn/UserDIn.h"




/******************************************************************************/
/*
    Var (live readings)
    Resolves AINS[] via MOT_AIN_THROTTLE / MOT_AIN_BRAKE; SwitchBrake via OptDin_SwitchBrake
*/
/******************************************************************************/
static inline int32_t MotAnalogUser_VarId_Get(const MotorController_T * p_mc, MotAnalogUser_VarId_T id)
{
    int32_t value = 0;
    switch (id)
    {
        case MOT_ANALOG_USER_THROTTLE:           value = UserAIn_GetValue(&p_mc->AINS[MOT_AIN_THROTTLE].PIN);                            break;
        case MOT_ANALOG_USER_BRAKE:              value = UserAIn_GetValue(&p_mc->AINS[MOT_AIN_BRAKE].PIN);                               break;
        case MOT_ANALOG_USER_THROTTLE_DIN:       value = UserAIn_IsOn(&p_mc->AINS[MOT_AIN_THROTTLE].PIN);                                break;
        case MOT_ANALOG_USER_BRAKE_DIN:          value = UserAIn_IsOn(&p_mc->AINS[MOT_AIN_BRAKE].PIN);                                   break;
        case MOT_ANALOG_USER_SWITCH_BRAKE_DIN:
        {
            UserDIn_T * p_pin = OptDin_SwitchBrake((UserDIn_T *)&p_mc->DINS[0U], &p_mc->P_MC->OptDinState);
            value = (p_pin != NULL) ? UserDIn_GetState(p_pin) : 0;
            break;
        }
        case MOT_ANALOG_USER_FORWARD_DIN:        value = UserDIn_GetState(&p_mc->SHIFTER.FORWARD_DIN);                                   break;
        case MOT_ANALOG_USER_REVERSE_DIN:        value = UserDIn_GetState(&p_mc->SHIFTER.REVERSE_DIN);                                   break;
        case MOT_ANALOG_USER_NEUTRAL_DIN:        value = UserDIn_GetState(&p_mc->SHIFTER.NEUTRAL_DIN);                                   break;
        default: break;
    }
    return value;
}

static inline int32_t MotAnalogUser_VarId_GetAsInput(const MotorController_T * p_mc, MotAnalogUser_VarId_T id)
{
    int32_t value = 0;
    switch (id)
    {
        case MOT_ANALOG_USER_THROTTLE:           value = p_mc->AINS[MOT_AIN_THROTTLE].PIN.P_STATE->RawValue_Adcu;                        break;
        case MOT_ANALOG_USER_BRAKE:              value = p_mc->AINS[MOT_AIN_BRAKE].PIN.P_STATE->RawValue_Adcu;                           break;
        case MOT_ANALOG_USER_THROTTLE_DIN:       value = _UserAIn_IsEdgePinOn(p_mc->AINS[MOT_AIN_THROTTLE].PIN.P_EDGE_PIN);              break;
        case MOT_ANALOG_USER_BRAKE_DIN:          value = _UserAIn_IsEdgePinOn(p_mc->AINS[MOT_AIN_BRAKE].PIN.P_EDGE_PIN);                 break;
        case MOT_ANALOG_USER_SWITCH_BRAKE_DIN:
        {
            UserDIn_T * p_pin = OptDin_SwitchBrake((UserDIn_T *)&p_mc->DINS[0U], &p_mc->P_MC->OptDinState);
            value = (p_pin != NULL) ? Pin_Input_ReadPhysical(&p_pin->PIN) : 0;
            break;
        }
        case MOT_ANALOG_USER_FORWARD_DIN:        value = Pin_Input_ReadPhysical(&p_mc->SHIFTER.FORWARD_DIN.PIN);                         break;
        case MOT_ANALOG_USER_REVERSE_DIN:        value = Pin_Input_ReadPhysical(&p_mc->SHIFTER.REVERSE_DIN.PIN);                         break;
        case MOT_ANALOG_USER_NEUTRAL_DIN:        value = Pin_Input_ReadPhysical(&p_mc->SHIFTER.NEUTRAL_DIN.PIN);                         break;
        default: break;
    }
    return value;
}

/******************************************************************************/
/*
    Config (NVM-backed runtime values)
*/
/******************************************************************************/
static inline int32_t MotAnalogUser_ConfigId_Get(const MotorController_T * p_mc, MotAnalogUser_ConfigId_T id)
{
    const MotorController_Config_T * p_config = &p_mc->P_MC->Config;
    int32_t value = 0;
    switch (id)
    {
        case MOT_ANALOG_USER_THROTTLE_ZERO_ADCU:           value = p_config->AInConfigs[MOT_AIN_THROTTLE].AdcZero;                break;
        case MOT_ANALOG_USER_THROTTLE_MAX_ADCU:            value = p_config->AInConfigs[MOT_AIN_THROTTLE].AdcMax;                 break;
        case MOT_ANALOG_USER_THROTTLE_EDGE_PIN_IS_ENABLE:  value = p_config->AInConfigs[MOT_AIN_THROTTLE].UseEdgePin;             break;
        case MOT_ANALOG_USER_BRAKE_ZERO_ADCU:              value = p_config->AInConfigs[MOT_AIN_BRAKE].AdcZero;                   break;
        case MOT_ANALOG_USER_BRAKE_MAX_ADCU:               value = p_config->AInConfigs[MOT_AIN_BRAKE].AdcMax;                    break;
        case MOT_ANALOG_USER_BRAKE_EDGE_PIN_IS_ENABLE:     value = p_config->AInConfigs[MOT_AIN_BRAKE].UseEdgePin;                break;
        case MOT_ANALOG_USER_SWITCH_BRAKE_VALUE:           value = p_config->OptDinConfig.SwitchBrakeFloor_Percent16;            break;
        default: break;
    }
    return value;
}

//todo mc reset replace UserAIn_Init(&p_mc->AINS[MOT_AIN_THROTTLE].PIN);
static inline void MotAnalogUser_ConfigId_Set(const MotorController_T * p_mc, MotAnalogUser_ConfigId_T id, int32_t value)
{
    MotorController_Config_T * p_config = &p_mc->P_MC->Config;
    switch (id)
    {
        case MOT_ANALOG_USER_THROTTLE_ZERO_ADCU:           p_config->AInConfigs[MOT_AIN_THROTTLE].AdcZero    = (uint16_t)value;
                                                           UserAIn_Init(&p_mc->AINS[MOT_AIN_THROTTLE].PIN);                       break;
        case MOT_ANALOG_USER_THROTTLE_MAX_ADCU:            p_config->AInConfigs[MOT_AIN_THROTTLE].AdcMax     = (uint16_t)value;
                                                           UserAIn_Init(&p_mc->AINS[MOT_AIN_THROTTLE].PIN);                       break;
        case MOT_ANALOG_USER_THROTTLE_EDGE_PIN_IS_ENABLE:  p_config->AInConfigs[MOT_AIN_THROTTLE].UseEdgePin = (bool)value;        break;
        case MOT_ANALOG_USER_BRAKE_ZERO_ADCU:              p_config->AInConfigs[MOT_AIN_BRAKE].AdcZero       = (uint16_t)value;
                                                           UserAIn_Init(&p_mc->AINS[MOT_AIN_BRAKE].PIN);                          break;
        case MOT_ANALOG_USER_BRAKE_MAX_ADCU:               p_config->AInConfigs[MOT_AIN_BRAKE].AdcMax        = (uint16_t)value;
                                                           UserAIn_Init(&p_mc->AINS[MOT_AIN_BRAKE].PIN);                          break;
        case MOT_ANALOG_USER_BRAKE_EDGE_PIN_IS_ENABLE:     p_config->AInConfigs[MOT_AIN_BRAKE].UseEdgePin    = (bool)value;        break;
        case MOT_ANALOG_USER_SWITCH_BRAKE_VALUE:           p_config->OptDinConfig.SwitchBrakeFloor_Percent16 = (uint16_t)value;   break;
        default: break;
    }
}
