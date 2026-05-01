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
    @brief  Forward/Neutral/Reverse direction pins.
            Wiring variant (FNR/FR/R) is selected as a Strategy that bundles a
            pin-state -> Direction Decoder with the per-pin UserDIn_Mode policy.
            The Strategy pointer is cached in state at Init / SetPinMode so the
            level queries are branchless dispatch.
*/
/******************************************************************************/
#include "Transducer/UserIn/UserDIn.h"
#include <stdint.h>
#include <stdbool.h>

#if !defined(SHIFTER_PINS_AVAILABLE_FNR) && !defined(SHIFTER_PINS_AVAILABLE_FR) && !defined(SHIFTER_PINS_AVAILABLE_R)
#define SHIFTER_PINS_AVAILABLE_FNR
#endif


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

/******************************************************************************/
/*
    Pin index — packs the three pin levels (and the SKU's pin-presence mask)
    into the same 3-bit shape.
    Index = (Neutral<<2) | (Forward<<1) | (Reverse<<0).

    The Config.PinMode field stores which pins are physically wired in the
    SKU (the presence mask). At runtime the same shape is reused to index
    the decode table. Pins absent from the mask are USER_DIN_MODE_DISABLED;
    Modal polling leaves their Debounce state at 0, so only the rows
    reachable for the SKU are visited. Unreachable rows are still filled
    defensively (Neutral, except R-only which defaults to Forward).
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
    Wiring variants. (Neutral<<2 | Forward<<1 | Reverse<<0) — assignable directly to Config.PinMode.Value.
        FNR — three pins, one per gear, mutually exclusive
        FR  — two pins (forward + reverse). Neutral = neither pressed
        R   — one pin (reverse). Forward = not reverse. Neutral never emitted
*/
typedef enum Shifter_PinMode
{
    SHIFTER_PIN_MODE_FNR = 0b111,
    SHIFTER_PIN_MODE_FR  = 0b011,
    SHIFTER_PIN_MODE_R   = 0b001,
}
Shifter_PinMode_T;


/******************************************************************************/
/*
    Config
*/
/******************************************************************************/
typedef struct Shifter_Config
{
    Shifter_Pins_T PinMode;
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
    // const Shifter_Direction_T * p_DecodeTable; /* if runtime switch to R only is needed */
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
    UserDIn_T NEUTRAL_DIN;
    // const Shifter_Config_T * P_NVM_CONFIG;
}
Shifter_T;

#define SHIFTER_STATE_ALLOC() (&(Shifter_State_T){0})

#define SHIFTER_INIT_FROM(p_State, ForwardHal, ForwardId, ReverseHal, ReverseId, NeutralHal, NeutralId, IsInvert, p_Timer ) (Shifter_T) \
{                                                                                                                                                 \
    .P_STATE = (p_State),                                                                                                                         \
    .FORWARD_DIN = USER_DIN_INIT_FROM((ForwardHal), (ForwardId), (IsInvert), &(p_State)->ForwardState, (p_Timer), 10U),                          \
    .REVERSE_DIN = USER_DIN_INIT_FROM((ReverseHal), (ReverseId), (IsInvert), &(p_State)->ReverseState, (p_Timer), 10U),                          \
    .NEUTRAL_DIN = USER_DIN_INIT_FROM((NeutralHal), (NeutralId), (IsInvert), &(p_State)->NeutralState, (p_Timer), 10U),                          \
}

/******************************************************************************/
/*
    Capture
*/
/******************************************************************************/
static inline Shifter_Direction_T Shifter_Decode(Shifter_T * p_shifter, Shifter_Pins_T pins)
{
    #define _N SHIFTER_DIRECTION_NEUTRAL
    #define _F SHIFTER_DIRECTION_FORWARD
    #define _R SHIFTER_DIRECTION_REVERSE

    (void)p_shifter; /* in case decode table is fully static */

#if defined(SHIFTER_PINS_AVAILABLE_FNR) || defined(SHIFTER_PINS_AVAILABLE_FR) // && !RUN_TIME_SWITCHABLE
    static const Shifter_Direction_T DECODE_TABLE[8] = { _N, _F, _R, _N, _N, _N, _N, _N };
#elif defined(SHIFTER_PINS_AVAILABLE_R)
    static const Shifter_Direction_T DECODE_TABLE[8] = { _F, _F, _R, _R, _F, _F, _R, _R };
#endif

    return DECODE_TABLE[pins.Value];
    // return p_shifter->P_STATE->p_DecodeTable[pins.Value];
}

static inline void Shifter_Poll(Shifter_T * p_shifter)
{
    UserDIn_Modal_PollEdgeValue(&p_shifter->REVERSE_DIN);
#if defined(SHIFTER_PINS_AVAILABLE_FNR) || defined(SHIFTER_PINS_AVAILABLE_FR)
    UserDIn_Modal_PollEdgeValue(&p_shifter->FORWARD_DIN);
#endif
#if defined(SHIFTER_PINS_AVAILABLE_FNR)
    UserDIn_Modal_PollEdgeValue(&p_shifter->NEUTRAL_DIN);
#endif
}
static inline Shifter_Pins_T Shifter_GetPins(Shifter_T * p_shifter)
{
    Shifter_Pins_T pins =
    {
        .Reverse = UserDIn_GetState(&p_shifter->REVERSE_DIN),
#if defined(SHIFTER_PINS_AVAILABLE_FNR) || defined(SHIFTER_PINS_AVAILABLE_FR)
        .Forward = UserDIn_GetState(&p_shifter->FORWARD_DIN),
#endif
#if defined(SHIFTER_PINS_AVAILABLE_FNR)
        .Neutral = UserDIn_GetState(&p_shifter->NEUTRAL_DIN),
#endif
    };
    return pins;
}

/******************************************************************************/
/*
    Query — level. Pack the three pin states into a 3-bit index and look up.
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
    return (p_shifter->P_STATE->LastDirection != Shifter_PollDirection(p_shifter));
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

// /******************************************************************************/
// /*
//     VarId / ConfigId
// */
// /******************************************************************************/
// typedef enum Shifter_VarId
// {
//     SHIFTER_VAR_DIRECTION,
//     SHIFTER_VAR_FORWARD_PIN,
//     SHIFTER_VAR_REVERSE_PIN,
//     SHIFTER_VAR_NEUTRAL_PIN,
// }
// Shifter_VarId_T;

// typedef enum Shifter_ConfigId
// {
//     SHIFTER_CONFIG_PIN_MODE,
// }
// Shifter_ConfigId_T;

// extern int32_t Shifter_VarId_Get(Shifter_T * p_shifter, Shifter_VarId_T id);
// extern int32_t Shifter_ConfigId_Get(Shifter_T * p_shifter, Shifter_ConfigId_T id);
// extern void Shifter_ConfigId_Set(Shifter_T * p_shifter, Shifter_ConfigId_T id, int32_t value);

