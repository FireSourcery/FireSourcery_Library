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
    @file   MotAnalogUser.h
    @author FireSourcery
    @brief  Analog user controls. 1 instance for all motor, input
*/
/******************************************************************************/
#include "Transducer/UserIn/UserAIn.h"
#include "Transducer/UserIn/UserDIn.h"
#include <stdint.h>
#include <stdbool.h>


typedef enum MotAnalogUser_Direction
{
    MOT_ANALOG_USER_DIRECTION_NEUTRAL,
    MOT_ANALOG_USER_DIRECTION_FORWARD,
    MOT_ANALOG_USER_DIRECTION_REVERSE,
}
MotAnalogUser_Direction_T;

typedef enum MotAnalogUser_DirectionPins
{
    MOT_ANALOG_USER_DIRECTION_PINS_FNR,
    MOT_ANALOG_USER_DIRECTION_PINS_FR,
    MOT_ANALOG_USER_DIRECTION_PINS_R,
}
MotAnalogUser_DirectionPins_T;

typedef struct MotAnalogUser_Config
{
    UserAIn_Config_T ThrottleAInConfig; /* Analog input config */
    UserAIn_Config_T BrakeAInConfig;    /* Analog input config */

    /* alternatively load from Enum */
    struct
    {
        uint8_t UseNeutralPin       : 1U; /*  alternatively as compile time enable */
        uint8_t UseForwardPin       : 1U; /*  alternatively as compile time enable */
        uint8_t UseThrottleEdgePin  : 1U;
        uint8_t UseBrakeEdgePin     : 1U;
        uint8_t UseSwitchBrakePin   : 1U;
    };

    uint16_t SwitchBrakeValue_Percent16;
}
MotAnalogUser_Config_T;

/******************************************************************************/
/*
    Runtime State - Using UserAIn modules
*/
/******************************************************************************/
/*
    Place SubModule States in contiguous memory
*/
typedef struct MotAnalogUser_State
{
    /* Analog input states */
    UserAIn_State_T ThrottleAInState;
    UserAIn_State_T BrakeAInState;

    /* Digital input states */
    UserDIn_State_T ThrottleEdgePinState;
    UserDIn_State_T BrakeEdgePinState;
    UserDIn_State_T SwitchBrakeState;
    UserDIn_State_T ReverseState;
    UserDIn_State_T ForwardState;
    UserDIn_State_T NeutralState;

    // MotAnalogUser_Cmd_T Cmd; /* Store last active Cmd */

    MotAnalogUser_Config_T Config;
}
MotAnalogUser_State_T;

typedef const struct MotAnalogUser
{
    MotAnalogUser_State_T * P_STATE;

    /* UserAIn instances */
    UserAIn_T THROTTLE_AIN;
    UserAIn_T BRAKE_AIN;

    /* UserDIn instances */
    UserDIn_T REVERSE_DIN;
    UserDIn_T FORWARD_DIN; /* alternatively as compile time enable */
    UserDIn_T NEUTRAL_DIN; /* alternatively as compile time enable */

    UserDIn_T SWITCH_BRAKE_DIN;

    const MotAnalogUser_Config_T * P_NVM_CONFIG;
}
MotAnalogUser_T;


/* Pins can pass by value */
#define MOT_ANALOG_USER_INIT(p_State, BrakePin, ThrottlePin, ForwardPin, ReversePin, NeutralPin, SwitchBrakePin, p_Timer, p_Config) (MotAnalogUser_T) \
{                                                                                                                       \
    .P_STATE = p_State,                                                                                                 \
    .THROTTLE_AIN = USER_AIN_INIT(ThrottlePin, &(p_State)->ThrottleEdgePinState, &(p_Config)->ThrottleAInConfig, 0U),   \
    .BRAKE_AIN = USER_AIN_INIT(BrakePin, &(p_State)->BrakeEdgePinState, &(p_Config)->BrakeAInConfig, 0U),               \
    .FORWARD_DIN = USER_DIN_INIT(ForwardPin, &(p_State)->ForwardState, p_Timer, 10U), \
    .REVERSE_DIN = USER_DIN_INIT(ReversePin, &(p_State)->ReverseState, p_Timer, 10U), \
    .NEUTRAL_DIN = USER_DIN_INIT(NeutralPin, &(p_State)->NeutralState, p_Timer, 10U), \
    .SWITCH_BRAKE_DIN = USER_DIN_INIT(SwitchBrakePin, &(p_State)->SwitchBrakeState, p_Timer, 10U), \
    .P_NVM_CONFIG = p_Config,                                                                                           \
}

/*  NeutralPinHal, NeutralPinId,  */
#define MOT_ANALOG_USER_INIT_FROM(BrakePinHal, BrakePinId, ThrottlePinHal, ThrottlePinId, ForwardPinHal, ForwardPinId, ReversePinHal, ReversePinId, SwitchBrakePinHal, SwitchBrakePinId, IsInvert, p_Timer, p_State, p_Config) (MotAnalogUser_T)\
{ \
    .P_STATE = (p_State),   \
    .THROTTLE_AIN =         \
    {                       \
        .P_EDGE_PIN = & USER_DIN_INIT_FROM(ThrottlePinHal, ThrottlePinId, IsInvert, &(p_State)->ThrottleEdgePinState, p_Timer, 10U), \
        .P_STATE = &(p_State)->ThrottleAInState,            \
        .P_NVM_CONFIG = &((p_Config)->ThrottleAInConfig),   \
        .FILTER_SHIFT = 0U,                                 \
    },                                                      \
    .BRAKE_AIN =                                            \
    {                                                       \
        .P_EDGE_PIN = & USER_DIN_INIT_FROM(BrakePinHal, BrakePinId, IsInvert, &(p_State)->BrakeEdgePinState, p_Timer, 10U), \
        .P_STATE = &(p_State)->BrakeAInState,             \
        .P_NVM_CONFIG = &((p_Config)->BrakeAInConfig),    \
        .FILTER_SHIFT = 0U,                               \
    },                                                    \
    .FORWARD_DIN        = USER_DIN_INIT_FROM(ForwardPinHal, ForwardPinId, IsInvert, &(p_State)->ForwardState, p_Timer, 10U), \
    .REVERSE_DIN        = USER_DIN_INIT_FROM(ReversePinHal, ReversePinId, IsInvert, &(p_State)->ReverseState, p_Timer, 10U), \
    .SWITCH_BRAKE_DIN   = USER_DIN_INIT_FROM(SwitchBrakePinHal, SwitchBrakePinId, IsInvert, &(p_State)->SwitchBrakeState, p_Timer, 10U), \
    .P_NVM_CONFIG = p_Config,                                                                                          \
}

/******************************************************************************/
/*
    Inline Functions
*/
/******************************************************************************/
/*
    Capture Inputs
*/
static inline void MotAnalogUser_CaptureThrottleValue(const MotAnalogUser_T * p_user, uint16_t throttle_Adcu) { UserAIn_CaptureValue(&p_user->THROTTLE_AIN, throttle_Adcu); }
static inline void MotAnalogUser_CaptureBrakeValue(const MotAnalogUser_T * p_user, uint16_t brake_Adcu) { UserAIn_CaptureValue(&p_user->BRAKE_AIN, brake_Adcu); }

static inline void MotAnalogUser_CaptureShifterPins(const MotAnalogUser_T * p_user)
{
    UserDIn_PollEdge(&p_user->REVERSE_DIN);
    if (p_user->P_STATE->Config.UseForwardPin == true) { UserDIn_PollEdge(&p_user->FORWARD_DIN); }
    if (p_user->P_STATE->Config.UseNeutralPin == true) { UserDIn_PollEdge(&p_user->NEUTRAL_DIN); }
}

static inline void MotAnalogUser_CaptureSwitchBrakePin(const MotAnalogUser_T * p_user) { if (p_user->P_STATE->Config.UseSwitchBrakePin == true) { UserDIn_PollEdge(&p_user->SWITCH_BRAKE_DIN); } }

static inline void MotAnalogUser_CaptureInput(const MotAnalogUser_T * p_user, uint16_t throttle_Adcu, uint16_t brake_Adcu)
{
    MotAnalogUser_CaptureShifterPins(p_user);
    MotAnalogUser_CaptureSwitchBrakePin(p_user);
    MotAnalogUser_CaptureThrottleValue(p_user, throttle_Adcu);
    MotAnalogUser_CaptureBrakeValue(p_user, brake_Adcu);
}



/******************************************************************************/
/*
    Shifter Direction
*/
/******************************************************************************/
static inline bool _MotAnalogUser_IsNeutralOn(const MotAnalogUser_T * p_user) { return (p_user->P_STATE->Config.UseNeutralPin == true) && UserDIn_GetState(&p_user->NEUTRAL_DIN); }
static inline bool MotAnalogUser_IsReverseOn(const MotAnalogUser_T * p_user) { return UserDIn_GetState(&p_user->REVERSE_DIN); }
static inline bool MotAnalogUser_IsForwardOn(const MotAnalogUser_T * p_user) { return (p_user->P_STATE->Config.UseForwardPin == true) ? UserDIn_GetState(&p_user->FORWARD_DIN) : !UserDIn_GetState(&p_user->REVERSE_DIN); }

static inline MotAnalogUser_Direction_T MotAnalogUser_GetDirection(const MotAnalogUser_T * p_user)
{
    MotAnalogUser_Direction_T direction;

    if (_MotAnalogUser_IsNeutralOn(p_user) == true) { direction = MOT_ANALOG_USER_DIRECTION_NEUTRAL; }
    else if (MotAnalogUser_IsReverseOn(p_user) == true) { direction = MOT_ANALOG_USER_DIRECTION_REVERSE; }
    else if (MotAnalogUser_IsForwardOn(p_user) == true) { direction = MOT_ANALOG_USER_DIRECTION_FORWARD; }
    else { direction = MOT_ANALOG_USER_DIRECTION_NEUTRAL; }

    return direction;
}


// static inline bool MotAnalogUser_GetDirectionEdge(const MotAnalogUser_T * p_user)
// {
//     switch (MotAnalogUser_GetDirection(p_user))
//     {
//         case MOT_ANALOG_USER_DIRECTION_NEUTRAL: edge = UserDIn_IsEdge(&p_user->NEUTRAL_DIN) ?  break;
//         case MOT_ANALOG_USER_DIRECTION_FORWARD: edge = UserDIn_IsEdge(&p_user->FORWARD_DIN) ?  break;
//         case MOT_ANALOG_USER_DIRECTION_REVERSE: edge = UserDIn_IsEdge(&p_user->REVERSE_DIN) ?  break;
//         default: break;
//     }
// }


/******************************************************************************/
/*
    DinBrake/Handbrake
*/
/******************************************************************************/
static inline bool MotAnalogUser_IsSwitchBrakeOn(const MotAnalogUser_T * p_user) { return (p_user->P_STATE->Config.UseSwitchBrakePin == true) && UserDIn_GetState(&p_user->SWITCH_BRAKE_DIN); }
static inline bool MotAnalogUser_IsSwitchBrakeFallingEdge(const MotAnalogUser_T * p_user) { return (p_user->P_STATE->Config.UseSwitchBrakePin == true) && UserDIn_IsFallingEdge(&p_user->SWITCH_BRAKE_DIN); }

/* Edge pins are polled automatically in UserAIn_CaptureValue, just check states */
static inline bool MotAnalogUser_IsAnyBrakeOn(const MotAnalogUser_T * p_user) { return (UserAIn_IsOn(&p_user->BRAKE_AIN) == true) || (MotAnalogUser_IsSwitchBrakeOn(p_user) == true); }

/******************************************************************************/
/*
    Throttle/Brake
*/
/******************************************************************************/
static inline uint16_t MotAnalogUser_GetThrottle(const MotAnalogUser_T * p_user) { return UserAIn_GetValue(&p_user->THROTTLE_AIN); }

static inline uint16_t MotAnalogUser_GetBrake(const MotAnalogUser_T * p_user)
{
    uint16_t brakeValue = UserAIn_GetValue(&p_user->BRAKE_AIN);
    if ((MotAnalogUser_IsSwitchBrakeOn(p_user) == true) && (p_user->P_STATE->Config.SwitchBrakeValue_Percent16 > brakeValue))
    {
        brakeValue = p_user->P_STATE->Config.SwitchBrakeValue_Percent16;
    }
    return brakeValue;
}

/* Diagnostic functions */
static inline uint16_t MotAnalogUser_GetThrottleInput(const MotAnalogUser_T * p_user) { return _UserAIn_GetValue(p_user->THROTTLE_AIN.P_STATE); }
static inline uint16_t MotAnalogUser_GetBrakeInput(const MotAnalogUser_T * p_user) { return _UserAIn_GetValue(p_user->BRAKE_AIN.P_STATE); }


/******************************************************************************/
/*
    Extern Functions
*/
/******************************************************************************/
extern void MotAnalogUser_Init(const MotAnalogUser_T * p_user);
extern void MotAnalogUser_SetBrakeZero(const MotAnalogUser_T * p_user, uint16_t zero_Adcu);
extern void MotAnalogUser_SetThrottleZero(const MotAnalogUser_T * p_user, uint16_t zero_Adcu);
extern void MotAnalogUser_SetBrakeRange(const MotAnalogUser_T * p_user, uint16_t zero_Adcu, uint16_t max_Adcu);
extern void MotAnalogUser_SetThrottleRange(const MotAnalogUser_T * p_user, uint16_t zero_Adcu, uint16_t max_Adcu);
extern void MotAnalogUser_SetBrakeAIn(const MotAnalogUser_T * p_user, uint16_t zero_Adcu, uint16_t max_Adcu, bool useBrakeEdgePin);
extern void MotAnalogUser_SetThrottleAIn(const MotAnalogUser_T * p_user, uint16_t zero_Adcu, uint16_t max_Adcu, bool useThrottleEdgePin);
extern void MotAnalogUser_SetSwitchBrake(const MotAnalogUser_T * p_user, bool useSwitchBrake, uint16_t bistateBrakeIntensity_Fract16);
extern void MotAnalogUser_SetDirectionPins(const MotAnalogUser_T * p_user, MotAnalogUser_DirectionPins_T pins);

/******************************************************************************/
/*
    VarId / ConfigId Access Functions
*/
/******************************************************************************/
/* Configuration access */
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
    MOT_ANALOG_USER_OPT_DIN,
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
}
MotAnalogUser_ConfigId_T;

extern int32_t MotAnalogUser_VarId_Get(const MotAnalogUser_T * p_user, MotAnalogUser_VarId_T id);
extern int32_t MotAnalogUser_VarId_GetAsInput(const MotAnalogUser_T * p_user, MotAnalogUser_VarId_T id);
extern int32_t MotAnalogUser_ConfigId_Get(const MotAnalogUser_T * p_user, MotAnalogUser_ConfigId_T id);
extern void MotAnalogUser_ConfigId_Set(const MotAnalogUser_T * p_user, MotAnalogUser_ConfigId_T id, int32_t value);