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
    @file   Shifter.h
    @author FireSourcery
    @brief  Forward/Neutral/Reverse direction pins. Wiring variant (FNR/FR/R) is
            applied at Init as per-pin UserDIn_Mode. Level and edge are exposed
            as separate queries — no mixed-enum.
*/
/******************************************************************************/
#include "Transducer/UserIn/UserDIn.h"
#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*
    Direction — level only. Edge queried separately.
*/
/******************************************************************************/
typedef enum Shifter_Direction
{
    SHIFTER_DIRECTION_NEUTRAL,
    SHIFTER_DIRECTION_FORWARD,
    SHIFTER_DIRECTION_REVERSE,
}
Shifter_Direction_T;

/* Wiring variants:
    FNR — three pins, one per gear, mutually exclusive
    FR  — two pins (forward + reverse). Neutral = neither pressed
    R   — one pin (reverse). Forward = not reverse. Neutral never emitted
*/
typedef enum Shifter_PinMode
{
    SHIFTER_PIN_MODE_FNR,
    SHIFTER_PIN_MODE_FR,
    SHIFTER_PIN_MODE_R,
}
Shifter_PinMode_T;

/******************************************************************************/
/*
    Config
*/
/******************************************************************************/
typedef struct Shifter_Config
{
    Shifter_PinMode_T PinMode;
}
Shifter_Config_T;

/******************************************************************************/
/*
    Runtime state
*/
/******************************************************************************/
typedef struct Shifter_State
{
    UserDIn_State_T ForwardState;
    UserDIn_State_T ReverseState;
    UserDIn_State_T NeutralState;
    Shifter_Direction_T LastDirection;  /* For change-edge query */
    Shifter_Config_T Config;
}
Shifter_State_T;

/******************************************************************************/
/*
    Compile-time context
*/
/******************************************************************************/
typedef const struct Shifter
{
    Shifter_State_T * P_STATE;
    UserDIn_T FORWARD_DIN;
    UserDIn_T REVERSE_DIN;
    UserDIn_T NEUTRAL_DIN;
    const Shifter_Config_T * P_NVM_CONFIG;
}
Shifter_T;

#define SHIFTER_STATE_ALLOC() (&(Shifter_State_T){0})

#define SHIFTER_INIT_FROM(p_State, ForwardHal, ForwardId, ReverseHal, ReverseId, NeutralHal, NeutralId, IsInvert, p_Timer, p_Config) (Shifter_T) \
{                                                                                                                                                 \
    .P_STATE = (p_State),                                                                                                                         \
    .FORWARD_DIN = USER_DIN_INIT_FROM((ForwardHal), (ForwardId), (IsInvert), &(p_State)->ForwardState, (p_Timer), 10U),                          \
    .REVERSE_DIN = USER_DIN_INIT_FROM((ReverseHal), (ReverseId), (IsInvert), &(p_State)->ReverseState, (p_Timer), 10U),                          \
    .NEUTRAL_DIN = USER_DIN_INIT_FROM((NeutralHal), (NeutralId), (IsInvert), &(p_State)->NeutralState, (p_Timer), 10U),                          \
    .P_NVM_CONFIG = (p_Config),                                                                                                                   \
}

/******************************************************************************/
/*
    Capture
*/
/******************************************************************************/
static inline void Shifter_Poll(const Shifter_T * p_shifter)
{
    UserDIn_PollEdge(&p_shifter->FORWARD_DIN);
    UserDIn_PollEdge(&p_shifter->REVERSE_DIN);
    UserDIn_PollEdge(&p_shifter->NEUTRAL_DIN);
}

/******************************************************************************/
/*
    Query — level
*/
/******************************************************************************/
static inline bool Shifter_IsForwardOn(const Shifter_T * p_shifter)
{
    switch (p_shifter->P_STATE->Config.PinMode)
    {
        case SHIFTER_PIN_MODE_R: return !UserDIn_GetState(&p_shifter->REVERSE_DIN);
        default:                 return UserDIn_GetState(&p_shifter->FORWARD_DIN);
    }
}

static inline bool Shifter_IsReverseOn(const Shifter_T * p_shifter)
{
    return UserDIn_GetState(&p_shifter->REVERSE_DIN);
}

static inline bool Shifter_IsNeutralOn(const Shifter_T * p_shifter)
{
    switch (p_shifter->P_STATE->Config.PinMode)
    {
        case SHIFTER_PIN_MODE_FNR: return UserDIn_GetState(&p_shifter->NEUTRAL_DIN);
        case SHIFTER_PIN_MODE_FR:  return (!UserDIn_GetState(&p_shifter->FORWARD_DIN)) && (!UserDIn_GetState(&p_shifter->REVERSE_DIN));
        default:                   return false;
    }
}

static inline Shifter_Direction_T Shifter_GetDirection(const Shifter_T * p_shifter)
{
    if      (Shifter_IsNeutralOn(p_shifter)) { return SHIFTER_DIRECTION_NEUTRAL; }
    else if (Shifter_IsReverseOn(p_shifter)) { return SHIFTER_DIRECTION_REVERSE; }
    else if (Shifter_IsForwardOn(p_shifter)) { return SHIFTER_DIRECTION_FORWARD; }
    else                                     { return SHIFTER_DIRECTION_NEUTRAL; }
}

/******************************************************************************/
/*
    Query — change edge
*/
/******************************************************************************/
static inline bool Shifter_IsDirectionChanged(const Shifter_T * p_shifter)
{
    return (Shifter_GetDirection(p_shifter) != p_shifter->P_STATE->LastDirection);
}

/* Call at end of tick after acting on the change. */
static inline Shifter_Direction_T Shifter_PollDirection(const Shifter_T * p_shifter)
{
    Shifter_Direction_T dir = Shifter_GetDirection(p_shifter);
    p_shifter->P_STATE->LastDirection = dir;
    return dir;
}

/******************************************************************************/
/*
    Extern
*/
/******************************************************************************/
extern void Shifter_Init(const Shifter_T * p_shifter);
extern void Shifter_SetPinMode(const Shifter_T * p_shifter, Shifter_PinMode_T mode);

/******************************************************************************/
/*
    VarId / ConfigId
*/
/******************************************************************************/
typedef enum Shifter_VarId
{
    SHIFTER_VAR_DIRECTION,
    SHIFTER_VAR_FORWARD_PIN,
    SHIFTER_VAR_REVERSE_PIN,
    SHIFTER_VAR_NEUTRAL_PIN,
}
Shifter_VarId_T;

typedef enum Shifter_ConfigId
{
    SHIFTER_CONFIG_PIN_MODE,
}
Shifter_ConfigId_T;

extern int32_t Shifter_VarId_Get(const Shifter_T * p_shifter, Shifter_VarId_T id);
extern int32_t Shifter_ConfigId_Get(const Shifter_T * p_shifter, Shifter_ConfigId_T id);
extern void Shifter_ConfigId_Set(const Shifter_T * p_shifter, Shifter_ConfigId_T id, int32_t value);
