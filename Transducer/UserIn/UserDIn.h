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
#include "Math/edge_fn.h"
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

typedef enum UserDIn_Mode
{
    USER_DIN_MODE_DISABLED,
    USER_DIN_MODE_NORMAL,           /* Standard debounced input */
    USER_DIN_MODE_TOGGLE,           /* Toggle on each press */
    USER_DIN_MODE_MOMENTARY,        /* Active only while pressed */
    USER_DIN_MODE_HOLD,             /* Requires hold time */
}
UserDIn_Mode_T;


// typedef struct UserDIn_Config
// {
//     UserDIn_Mode_T Mode;
//     size_t CmdId;
// }
// UserDIn_Config_T;

/******************************************************************************/
/*
    Runtime State
*/
/******************************************************************************/
typedef struct UserDIn_State
{
    Debounce_T Debounce;
    bool OutputPrev;
    // bool ToggleState;                   /* For toggle mode */
    // bool HoldState;                     /* For hold mode */
    // uint16_t HoldStartTime;             /* Hold timing */
    UserDIn_Mode_T Mode;
    // UserDIn_Cmd_T * p_OptCmd; /* Optional command to execute on edge */
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
    // UserDIn_Mode_T MODE; /* Default mode */
    // UserDIn_Cmd_T CMD; /* Fixed Cmd */
}
UserDIn_T;

#define USER_DIN_STATE_ALLOC() (&(UserDIn_State_T){0})

#define USER_DIN_INIT(Pin, p_State, p_Timer, DebounceTime) (UserDIn_T) \
    { .PIN = Pin, .P_STATE = (p_State), .P_TIMER = (p_Timer), .DEBOUNCE_TIME = (DebounceTime), }

#define USER_DIN_INIT_FROM(p_PinHal, PinId, PinIsInvert, p_State, p_Timer, DebounceTime) \
    USER_DIN_INIT(PIN_INIT_INVERT(p_PinHal, PinId, PinIsInvert), p_State, p_Timer, DebounceTime)

/******************************************************************************/
/*
    State Query Functions
*/
/******************************************************************************/
static inline bool UserDIn_GetState(UserDIn_T * p_dev) { return Debounce_GetState(&p_dev->P_STATE->Debounce); }
static inline bool UserDIn_IsEdge(UserDIn_T * p_dev) { return is_edge(p_dev->P_STATE->OutputPrev, UserDIn_GetState(p_dev)); }
static inline bool UserDIn_IsRisingEdge(UserDIn_T * p_dev) { return is_rising_edge(p_dev->P_STATE->OutputPrev, UserDIn_GetState(p_dev)); }
static inline bool UserDIn_IsFallingEdge(UserDIn_T * p_dev) { return is_falling_edge(p_dev->P_STATE->OutputPrev, UserDIn_GetState(p_dev)); }
static inline UserDIn_Edge_T UserDIn_GetEdge(UserDIn_T * p_dev) { return (UserDIn_Edge_T)edge_sign(p_dev->P_STATE->OutputPrev, UserDIn_GetState(p_dev)); }

static inline void UserDIn_Modal_Disable(UserDIn_T * p_dev) { p_dev->P_STATE->Mode = USER_DIN_MODE_DISABLED; }
static inline void UserDIn_Modal_Enable(UserDIn_T * p_dev) { p_dev->P_STATE->Mode = USER_DIN_MODE_NORMAL; }

/******************************************************************************/
/*

*/
/******************************************************************************/
extern void UserDIn_Init(UserDIn_T * p_dev);
extern bool UserDIn_PollEdge(UserDIn_T * p_dev);
extern bool UserDIn_PollRisingEdge(UserDIn_T * p_dev);
extern bool UserDIn_PollFallingEdge(UserDIn_T * p_dev);
extern UserDIn_Edge_T UserDIn_PollEdgeValue(UserDIn_T * p_dev);

