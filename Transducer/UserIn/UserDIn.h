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
    @file   UserDIn.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Debounce/Debounce.h"
#include "Peripheral/Pin/Pin.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum UserDIn_Edge
{
    USER_DIN_EDGE_FALLING = -1,
    USER_DIN_EDGE_NULL = 0,
    USER_DIN_EDGE_RISING = 1,
}
UserDIn_Edge_T;

// typedef enum UserDIn_Mode
// {
//     USER_DIN_MODE_NORMAL = 0,           /* Standard debounced input */
//     USER_DIN_MODE_TOGGLE = 1,           /* Toggle on each press */
//     USER_DIN_MODE_MOMENTARY = 2,        /* Active only while pressed */
//     USER_DIN_MODE_HOLD = 3,             /* Requires hold time */
// }
// UserDIn_Mode_T;

/******************************************************************************/
/*
    Runtime State
*/
/******************************************************************************/
typedef struct UserDIn_State
{
    Debounce_T Debounce;
    // bool IsEnabled;
    // bool IsInverted;                    /* Runtime invert config */
    // bool ToggleState;                   /* For toggle mode */
    // bool HoldState;                     /* For hold mode */
    // uint16_t HoldStartTime;             /* Hold timing */
    // UserDIn_Mode_T Mode;
    // uint16_t DebounceTime;
}
UserDIn_State_T;

/******************************************************************************/
/*
    Context as compile time constant
*/
/******************************************************************************/
typedef const struct UserDIn
{
    Pin_T PIN;
    UserDIn_State_T * P_STATE;
    const volatile uint32_t * P_TIMER;
    uint16_t DEBOUNCE_TIME;
}
UserDIn_T;

#define USER_DIN_STATE_ALLOC() (&(UserDIn_State_T){0})

#define USER_DIN_INIT(Pin, p_State, p_Timer, DebounceTime) \
    { .PIN = Pin, .P_STATE = (p_State), .P_TIMER = (p_Timer), .DEBOUNCE_TIME = (DebounceTime), }

#define USER_DIN_ALLOC(Pin, p_Timer, DebounceTime) \
    USER_DIN_INIT(Pin, USER_DIN_STATE_ALLOC(), p_Timer, DebounceTime)

#define USER_DIN_INIT_FROM(p_PinHal, PinId, PinIsInvert, p_State, p_Timer, DebounceTime) \
    USER_DIN_INIT(PIN_INIT_INVERT(p_PinHal, PinId, PinIsInvert), p_State, p_Timer, DebounceTime)

/******************************************************************************/
/*
    State Query Functions
*/
/******************************************************************************/
static inline bool UserDIn_GetState(const UserDIn_T * p_context) { return Debounce_GetState(&p_context->P_STATE->Debounce); }
static inline bool UserDIn_IsEdge(const UserDIn_T * p_context) { return Debounce_IsEdge(&p_context->P_STATE->Debounce); }
static inline bool UserDIn_IsRisingEdge(const UserDIn_T * p_context) { return Debounce_IsRisingEdge(&p_context->P_STATE->Debounce); }
static inline bool UserDIn_IsFallingEdge(const UserDIn_T * p_context) { return Debounce_IsFallingEdge(&p_context->P_STATE->Debounce); }
static inline UserDIn_Edge_T UserDIn_GetEdge(const UserDIn_T * p_context) { return (UserDIn_Edge_T)Debounce_GetEdge(&p_context->P_STATE->Debounce); }

/******************************************************************************/
/*
    Configuration Functions
*/
/******************************************************************************/
static inline void UserDIn_SetDebounceTime(const UserDIn_T * p_context, uint16_t millis) { Debounce_SetTime(&p_context->P_STATE->Debounce, millis); }
static inline uint16_t UserDIn_GetDebounceTime(const UserDIn_T * p_context) { return Debounce_GetTime(&p_context->P_STATE->Debounce); }


/******************************************************************************/
/*
    Configuration Functions
*/
/******************************************************************************/
extern void UserDIn_Init(const UserDIn_T * p_context);
extern bool UserDIn_PollEdge(const UserDIn_T * p_context);
extern bool UserDIn_PollRisingEdge(const UserDIn_T * p_context);
extern bool UserDIn_PollFallingEdge(const UserDIn_T * p_context);
extern UserDIn_Edge_T UserDIn_PollEdgeValue(const UserDIn_T * p_context);