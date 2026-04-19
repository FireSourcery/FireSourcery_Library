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
*/
/******************************************************************************/
/******************************************************************************/
/*!
    @file   Pedals.h
    @author FireSourcery
    @brief  Throttle + Brake analog pedal pair. No direction, no auxiliary brake.
            Each pedal owns a UserAIn_T (optionally with a UserDIn edge/enable pin).
            Per-pedal enable comes from the UserDIn mode; no *Use*Pin* flags here.
*/
/******************************************************************************/
#include "Transducer/UserIn/UserAIn.h"
#include "Transducer/UserIn/UserDIn.h"
#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*
    Config — per-pedal NVM calibration is in each UserAIn's config.
    Nothing module-scoped yet; keep the struct for future policy bits.
*/
/******************************************************************************/
typedef struct Pedals_Config
{
    UserAIn_Config_T ThrottleAInConfig;
    UserAIn_Config_T BrakeAInConfig;
}
Pedals_Config_T;

/******************************************************************************/
/*
    Runtime state
*/
/******************************************************************************/
typedef struct Pedals_State
{
    UserAIn_State_T ThrottleAInState;
    UserAIn_State_T BrakeAInState;
    UserDIn_State_T ThrottleEdgePinState; /* UserDIn Mode=DISABLED when pin absent */
    UserDIn_State_T BrakeEdgePinState;
    Pedals_Config_T Config;
}
Pedals_State_T;

/******************************************************************************/
/*
    Compile-time context
*/
/******************************************************************************/
typedef const struct Pedals
{
    Pedals_State_T * P_STATE;
    UserAIn_T THROTTLE_AIN;
    UserAIn_T BRAKE_AIN;
    const Pedals_Config_T * P_NVM_CONFIG;
}
Pedals_T;

#define PEDALS_STATE_ALLOC() (&(Pedals_State_T){0})

#define PEDALS_INIT_FROM(p_State, ThrottlePinHal, ThrottlePinId, BrakePinHal, BrakePinId, IsInvert, p_Timer, p_Config) (Pedals_T) \
{                                                                                                                                 \
    .P_STATE = (p_State),                                                                                                         \
    .THROTTLE_AIN =                                                                                                               \
    {                                                                                                                             \
        .P_EDGE_PIN = &USER_DIN_INIT_FROM((ThrottlePinHal), (ThrottlePinId), (IsInvert), &(p_State)->ThrottleEdgePinState, (p_Timer), 10U), \
        .P_STATE = &(p_State)->ThrottleAInState,                                                                                  \
        .P_NVM_CONFIG = &((p_Config)->ThrottleAInConfig),                                                                         \
        .FILTER_SHIFT = 0U,                                                                                                       \
    },                                                                                                                            \
    .BRAKE_AIN =                                                                                                                  \
    {                                                                                                                             \
        .P_EDGE_PIN = &USER_DIN_INIT_FROM((BrakePinHal), (BrakePinId), (IsInvert), &(p_State)->BrakeEdgePinState, (p_Timer), 10U),\
        .P_STATE = &(p_State)->BrakeAInState,                                                                                     \
        .P_NVM_CONFIG = &((p_Config)->BrakeAInConfig),                                                                            \
        .FILTER_SHIFT = 0U,                                                                                                       \
    },                                                                                                                            \
    .P_NVM_CONFIG = (p_Config),                                                                                                   \
}

/******************************************************************************/
/*
    Capture — call once per sample tick
*/
/******************************************************************************/
static inline void Pedals_CaptureThrottle(const Pedals_T * p_pedals, uint16_t throttle_Adcu)
{
    UserAIn_CaptureValue(&p_pedals->THROTTLE_AIN, throttle_Adcu);
}

static inline void Pedals_CaptureBrake(const Pedals_T * p_pedals, uint16_t brake_Adcu)
{
    UserAIn_CaptureValue(&p_pedals->BRAKE_AIN, brake_Adcu);
}

static inline void Pedals_Capture(const Pedals_T * p_pedals, uint16_t throttle_Adcu, uint16_t brake_Adcu)
{
    Pedals_CaptureThrottle(p_pedals, throttle_Adcu);
    Pedals_CaptureBrake(p_pedals, brake_Adcu);
}

/******************************************************************************/
/*
    Query — level
*/
/******************************************************************************/
static inline uint16_t Pedals_GetThrottle(const Pedals_T * p_pedals) { return UserAIn_GetValue(&p_pedals->THROTTLE_AIN); }
static inline uint16_t Pedals_GetBrake(const Pedals_T * p_pedals)    { return UserAIn_GetValue(&p_pedals->BRAKE_AIN); }

static inline bool Pedals_IsThrottleOn(const Pedals_T * p_pedals) { return UserAIn_IsOn(&p_pedals->THROTTLE_AIN); }
static inline bool Pedals_IsBrakeOn(const Pedals_T * p_pedals)    { return UserAIn_IsOn(&p_pedals->BRAKE_AIN); }

/******************************************************************************/
/*
    Query — edge
*/
/******************************************************************************/
static inline bool Pedals_IsThrottleRisingEdge(const Pedals_T * p_pedals)  { return UserAIn_IsRisingEdge(&p_pedals->THROTTLE_AIN); }
static inline bool Pedals_IsThrottleFallingEdge(const Pedals_T * p_pedals) { return UserAIn_IsFallingEdge(&p_pedals->THROTTLE_AIN); }
static inline bool Pedals_IsBrakeRisingEdge(const Pedals_T * p_pedals)     { return UserAIn_IsRisingEdge(&p_pedals->BRAKE_AIN); }
static inline bool Pedals_IsBrakeFallingEdge(const Pedals_T * p_pedals)    { return UserAIn_IsFallingEdge(&p_pedals->BRAKE_AIN); }

/******************************************************************************/
/*
    Diagnostics — bypasses EdgePin gating
*/
/******************************************************************************/
static inline uint16_t Pedals_GetThrottleRaw(const Pedals_T * p_pedals) { return _UserAIn_GetValue(p_pedals->THROTTLE_AIN.P_STATE); }
static inline uint16_t Pedals_GetBrakeRaw(const Pedals_T * p_pedals)    { return _UserAIn_GetValue(p_pedals->BRAKE_AIN.P_STATE); }

/******************************************************************************/
/*
    Extern
*/
/******************************************************************************/
extern void Pedals_Init(const Pedals_T * p_pedals);
extern void Pedals_SetThrottleRange(const Pedals_T * p_pedals, uint16_t zero_Adcu, uint16_t max_Adcu);
extern void Pedals_SetBrakeRange(const Pedals_T * p_pedals, uint16_t zero_Adcu, uint16_t max_Adcu);
extern void Pedals_SetThrottleZero(const Pedals_T * p_pedals, uint16_t zero_Adcu);
extern void Pedals_SetBrakeZero(const Pedals_T * p_pedals, uint16_t zero_Adcu);

/******************************************************************************/
/*
    VarId / ConfigId
*/
/******************************************************************************/
typedef enum Pedals_VarId
{
    PEDALS_VAR_THROTTLE,
    PEDALS_VAR_BRAKE,
    PEDALS_VAR_THROTTLE_RAW,
    PEDALS_VAR_BRAKE_RAW,
    PEDALS_VAR_THROTTLE_EDGE_PIN,
    PEDALS_VAR_BRAKE_EDGE_PIN,
}
Pedals_VarId_T;

typedef enum Pedals_ConfigId
{
    PEDALS_CONFIG_THROTTLE_ZERO_ADCU,
    PEDALS_CONFIG_THROTTLE_MAX_ADCU,
    PEDALS_CONFIG_BRAKE_ZERO_ADCU,
    PEDALS_CONFIG_BRAKE_MAX_ADCU,
}
Pedals_ConfigId_T;

extern int32_t Pedals_VarId_Get(const Pedals_T * p_pedals, Pedals_VarId_T id);
extern int32_t Pedals_ConfigId_Get(const Pedals_T * p_pedals, Pedals_ConfigId_T id);
extern void Pedals_ConfigId_Set(const Pedals_T * p_pedals, Pedals_ConfigId_T id, int32_t value);
