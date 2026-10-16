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
    @brief  Forward/Reverse direction pins. Neutral pin by SHIFTER_NEUTRAL_PIN_ENABLE.
            [Shifter_PinMode_T] selects the pin-state -> Direction decode table.
            The table pointer is cached in state at Init / SetPinMode so the
            level queries are branchless dispatch.
*/
/******************************************************************************/
#include "Transducer/UserIn/UserDIn.h"
#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*
    Pin levels, packed as the decode index.
    Index = (Neutral<<2) | (Forward<<1) | (Reverse<<0).
    A pin without a HAL reads 0, as does Neutral without SHIFTER_NEUTRAL_PIN_ENABLE.
*/
/******************************************************************************/
typedef union Shifter_Pins
{
    struct
    {
        uint8_t Reverse : 1;
        uint8_t Forward : 1;
        uint8_t Neutral : 1;
    };
    uint8_t Value;
}
Shifter_Pins_T;

/*
    Wiring variant. Selects the decode table only; which pins exist is the board's (HAL, SHIFTER_NEUTRAL_PIN_ENABLE).
        FR  — forward + reverse pins. Neither = Neutral
        R   — reverse pin only. Released = Forward. Neutral never emitted
*/
typedef enum Shifter_PinMode
{
    SHIFTER_PIN_MODE_FR = 0,
    SHIFTER_PIN_MODE_R = 1, /* R-only wiring variant */
}
Shifter_PinMode_T;

/******************************************************************************/
/*
    Direction — level only. Edge queried separately.
*/
/******************************************************************************/
typedef enum Shifter_Direction
{
    SHIFTER_DIRECTION_REVERSE = -1,
    SHIFTER_DIRECTION_NEUTRAL = 0,
    SHIFTER_DIRECTION_FORWARD = 1,
}
Shifter_Direction_T;

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
#if defined(SHIFTER_NEUTRAL_PIN_ENABLE)
    UserDIn_State_T NeutralState;
#endif
    const Shifter_Direction_T * p_DecodeTable; /* by Config.PinMode */
    Shifter_Direction_T LastDirection;     /* For change-edge query */
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
#if defined(SHIFTER_NEUTRAL_PIN_ENABLE)
    UserDIn_T NEUTRAL_DIN;
#endif
    // const Shifter_Config_T * P_NVM_CONFIG;
}
Shifter_T;

#define SHIFTER_STATE_ALLOC() (&(Shifter_State_T){0})

#define SHIFTER_DIN_INIT(PinHal, PinId, IsInvert, p_State, p_Timer)                                                                         \
    { .PIN = PIN_INIT_INVERT((PinHal), (PinId), (IsInvert)), .P_STATE = (p_State), .P_TIMER = (p_Timer), .DEBOUNCE_TIME = 10U, }

#if defined(SHIFTER_NEUTRAL_PIN_ENABLE)
#define SHIFTER_INIT_FROM(p_State, ForwardHal, ForwardId, ReverseHal, ReverseId, NeutralHal, NeutralId, IsInvert, p_Timer) (Shifter_T)      \
{                                                                                                                                           \
    .P_STATE = (p_State),                                                                                                                   \
    .FORWARD_DIN = SHIFTER_DIN_INIT((ForwardHal), (ForwardId), (IsInvert), &(p_State)->ForwardState, (p_Timer)),                            \
    .REVERSE_DIN = SHIFTER_DIN_INIT((ReverseHal), (ReverseId), (IsInvert), &(p_State)->ReverseState, (p_Timer)),                            \
    .NEUTRAL_DIN = SHIFTER_DIN_INIT((NeutralHal), (NeutralId), (IsInvert), &(p_State)->NeutralState, (p_Timer)),                            \
}
#else
#define SHIFTER_INIT_FROM(p_State, ForwardHal, ForwardId, ReverseHal, ReverseId, IsInvert, p_Timer) (Shifter_T)                             \
{                                                                                                                                           \
    .P_STATE = (p_State),                                                                                                                   \
    .FORWARD_DIN = SHIFTER_DIN_INIT((ForwardHal), (ForwardId), (IsInvert), &(p_State)->ForwardState, (p_Timer)),                            \
    .REVERSE_DIN = SHIFTER_DIN_INIT((ReverseHal), (ReverseId), (IsInvert), &(p_State)->ReverseState, (p_Timer)),                            \
}
#endif

/******************************************************************************/
/*
    Capture
*/
/******************************************************************************/
static inline Shifter_Direction_T Shifter_Decode(Shifter_T * p_shifter, Shifter_Pins_T pins)
{
    return p_shifter->P_STATE->p_DecodeTable[pins.Value];
}

static inline void Shifter_Poll(Shifter_T * p_shifter)
{
    UserDIn_Modal_PollEdgeValue(&p_shifter->REVERSE_DIN);
    UserDIn_Modal_PollEdgeValue(&p_shifter->FORWARD_DIN);
#if defined(SHIFTER_NEUTRAL_PIN_ENABLE)
    UserDIn_Modal_PollEdgeValue(&p_shifter->NEUTRAL_DIN);
#endif
}

static inline Shifter_Pins_T Shifter_GetPins(Shifter_T * p_shifter)
{
    Shifter_Pins_T pins =
    {
        .Reverse = UserDIn_GetState(&p_shifter->REVERSE_DIN),
        .Forward = UserDIn_GetState(&p_shifter->FORWARD_DIN),
#if defined(SHIFTER_NEUTRAL_PIN_ENABLE)
        .Neutral = UserDIn_GetState(&p_shifter->NEUTRAL_DIN),
#endif
    };
    return pins;
}

/******************************************************************************/
/*
    Query — level. Pack the pin states into the decode index and look up.
*/
/******************************************************************************/
static inline Shifter_Direction_T Shifter_ResolveDirection(Shifter_T * p_shifter)
{
    return Shifter_Decode(p_shifter, Shifter_GetPins(p_shifter));
}

static inline Shifter_Direction_T Shifter_PollDirection(Shifter_T * p_shifter)
{
    Shifter_Poll(p_shifter);
    p_shifter->P_STATE->LastDirection = Shifter_ResolveDirection(p_shifter);
    return p_shifter->P_STATE->LastDirection;
}

static inline bool Shifter_PollDirectionEdge(Shifter_T * p_shifter)
{
    Shifter_Direction_T prev = p_shifter->P_STATE->LastDirection;
    return (prev != Shifter_PollDirection(p_shifter));
}

static inline Shifter_Direction_T Shifter_GetDirection(Shifter_T * p_shifter) { return p_shifter->P_STATE->LastDirection; }


/******************************************************************************/
/*
    Extern
*/
/******************************************************************************/
extern void Shifter_Init(Shifter_T * p_shifter);
extern void Shifter_InitFrom(const Shifter_T * p_shifter, const Shifter_Config_T * p_config);
extern void Shifter_SetPinMode(Shifter_T * p_shifter, Shifter_PinMode_T mode);

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

extern int32_t Shifter_VarId_Get(Shifter_T * p_shifter, Shifter_VarId_T id);
extern int32_t Shifter_ConfigId_Get(Shifter_T * p_shifter, Shifter_ConfigId_T id);

