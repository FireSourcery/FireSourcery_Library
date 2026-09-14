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



static inline bool MotorController_AIn_IsEveryZero(MotorController_T * p_dev)
{
    bool isZero = true;
    for (uint8_t i = 0U; i < MOT_USER_AIN_COUNT; ++i) { isZero &= (UserAIn_GetValue(&p_dev->AINS[i].PIN) == 0); }
    return isZero;
}




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


/******************************************************************************/
/*
    Var (live readings)
    Resolves AINS[] via MOT_AIN_THROTTLE / MOT_AIN_BRAKE; SwitchBrake via OptDin_SwitchBrake
*/
/******************************************************************************/
static inline int32_t MotAnalogUser_VarId_Get(MotorController_T * p_mc, MotAnalogUser_VarId_T id)
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
                UserDIn_T * p_pin = OptDin_SwitchBrake(&p_mc->DINS[0U], &p_mc->P_MC->OptDinState);
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

static inline int32_t MotAnalogUser_VarId_GetAsInput(MotorController_T * p_mc, MotAnalogUser_VarId_T id)
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
static inline int32_t MotAnalogUser_ConfigId_Get(MotorController_T * p_mc, MotAnalogUser_ConfigId_T id)
{
    switch (id)
    {
        case MOT_ANALOG_USER_THROTTLE_ZERO_ADCU:           return UserAIn_Config_Get(&p_mc->AINS[MOT_AIN_THROTTLE].PIN, USER_AIN_ZERO_ADCU);
        case MOT_ANALOG_USER_THROTTLE_MAX_ADCU:            return UserAIn_Config_Get(&p_mc->AINS[MOT_AIN_THROTTLE].PIN, USER_AIN_MAX_ADCU);
        case MOT_ANALOG_USER_THROTTLE_EDGE_PIN_IS_ENABLE:  return UserAIn_Config_Get(&p_mc->AINS[MOT_AIN_THROTTLE].PIN, USER_AIN_EDGE_PIN_IS_ENABLE);
        case MOT_ANALOG_USER_BRAKE_ZERO_ADCU:              return UserAIn_Config_Get(&p_mc->AINS[MOT_AIN_BRAKE].PIN, USER_AIN_ZERO_ADCU);
        case MOT_ANALOG_USER_BRAKE_MAX_ADCU:               return UserAIn_Config_Get(&p_mc->AINS[MOT_AIN_BRAKE].PIN, USER_AIN_MAX_ADCU);
        case MOT_ANALOG_USER_BRAKE_EDGE_PIN_IS_ENABLE:     return UserAIn_Config_Get(&p_mc->AINS[MOT_AIN_BRAKE].PIN, USER_AIN_EDGE_PIN_IS_ENABLE);
        case MOT_ANALOG_USER_SWITCH_BRAKE_VALUE:           return p_mc->P_MC->Config.OptDinConfig.SwitchBrakeFloor_Percent16;
        default: return 0;
    }
}

/* UserAIn_Config_Set propagates — rescales the linear units and re-gates the edge pin */
static inline void MotAnalogUser_ConfigId_Set(MotorController_T * p_mc, MotAnalogUser_ConfigId_T id, int32_t value)
{
    switch (id)
    {
        case MOT_ANALOG_USER_THROTTLE_ZERO_ADCU:           UserAIn_Config_Set(&p_mc->AINS[MOT_AIN_THROTTLE].PIN, USER_AIN_ZERO_ADCU, value);              break;
        case MOT_ANALOG_USER_THROTTLE_MAX_ADCU:            UserAIn_Config_Set(&p_mc->AINS[MOT_AIN_THROTTLE].PIN, USER_AIN_MAX_ADCU, value);               break;
        case MOT_ANALOG_USER_THROTTLE_EDGE_PIN_IS_ENABLE:  UserAIn_Config_Set(&p_mc->AINS[MOT_AIN_THROTTLE].PIN, USER_AIN_EDGE_PIN_IS_ENABLE, value);     break;
        case MOT_ANALOG_USER_BRAKE_ZERO_ADCU:              UserAIn_Config_Set(&p_mc->AINS[MOT_AIN_BRAKE].PIN, USER_AIN_ZERO_ADCU, value);                 break;
        case MOT_ANALOG_USER_BRAKE_MAX_ADCU:               UserAIn_Config_Set(&p_mc->AINS[MOT_AIN_BRAKE].PIN, USER_AIN_MAX_ADCU, value);                  break;
        case MOT_ANALOG_USER_BRAKE_EDGE_PIN_IS_ENABLE:     UserAIn_Config_Set(&p_mc->AINS[MOT_AIN_BRAKE].PIN, USER_AIN_EDGE_PIN_IS_ENABLE, value);        break;
        case MOT_ANALOG_USER_SWITCH_BRAKE_VALUE:           p_mc->P_MC->Config.OptDinConfig.SwitchBrakeFloor_Percent16 = (uint16_t)value;                  break;
        default: break;
    }
}
