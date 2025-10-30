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
    @file   UserDIn.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
/******************************************************************************/
#include "UserDIn.h"

#include "Debounce/Debounce.h"
#include "Peripheral/Pin/Pin.h"
#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*
    Private Helper Functions
*/
/******************************************************************************/
static inline bool ReadPin(const UserDIn_T * p_context) { return Pin_Input_Read(&p_context->PIN); }
static inline uint32_t GetTime(const UserDIn_T * p_context) { return *p_context->P_TIMER; }

/******************************************************************************/
/*
    Public Functions
*/
/******************************************************************************/
void UserDIn_Init(const UserDIn_T * p_context)
{
    Pin_Input_Init(&p_context->PIN);

    Debounce_Init(&p_context->P_STATE->Debounce, p_context->DEBOUNCE_TIME);

    /* Initialize debounce state to current pin reading */
    p_context->P_STATE->Debounce.TimeStart = GetTime(p_context);
    p_context->P_STATE->Debounce.PinState = ReadPin(p_context);
    p_context->P_STATE->Debounce.Output = p_context->P_STATE->Debounce.PinState;
    p_context->P_STATE->Debounce.OutputPrev = p_context->P_STATE->Debounce.PinState;
}

/*! @return On/Off state */
bool UserDIn_PollState(const UserDIn_T * p_context) { return Debounce_Poll(&p_context->P_STATE->Debounce, GetTime(p_context), ReadPin(p_context)); }

/*! @return true on change */
bool UserDIn_PollEdge(const UserDIn_T * p_context) { return Debounce_PollEdge(&p_context->P_STATE->Debounce, GetTime(p_context), ReadPin(p_context)); }

bool UserDIn_PollRisingEdge(const UserDIn_T * p_context) { return Debounce_PollRisingEdge(&p_context->P_STATE->Debounce, GetTime(p_context), ReadPin(p_context)); }

bool UserDIn_PollFallingEdge(const UserDIn_T * p_context) { return Debounce_PollFallingEdge(&p_context->P_STATE->Debounce, GetTime(p_context), ReadPin(p_context)); }

UserDIn_Edge_T UserDIn_PollEdgeValue(const UserDIn_T * p_context) { return (UserDIn_Edge_T)Debounce_PollEdgeValue(&p_context->P_STATE->Debounce, GetTime(p_context), ReadPin(p_context)); }


// /******************************************************************************/
// /*
//     Mode Extensions - Toggle Mode
// */
// /******************************************************************************/
// static bool _UserDIn_ProcessToggleMode(const UserDIn_T * p_context)
// {
//     if (Debounce_IsRisingEdge(&p_context->P_STATE->Debounce) && _UserDIn_GetActiveState(p_context))
//         p_context->P_STATE->ToggleState = !p_context->P_STATE->ToggleState;
//     return p_context->P_STATE->ToggleState;
// }

// /******************************************************************************/
// /*
//     Mode Extensions - Momentary Mode
// */
// /******************************************************************************/
// bool _UserDIn_ProcessMomentaryMode(const UserDIn_T * p_context) { return _UserDIn_GetActiveState(p_context); }

// /******************************************************************************/
// /*
//     Mode Extensions - Hold Mode
// */
// /******************************************************************************/
// static bool _UserDIn_ProcessHoldMode(const UserDIn_T * p_context)
// {
//     bool activeState = _UserDIn_GetActiveState(p_context);
//     uint16_t currentTime = GetTime(p_context);
//     Debounce_T * p_debounce = &p_context->P_STATE->Debounce;

//     if (Debounce_IsRisingEdge(p_debounce) && activeState)
//     {
//         p_context->P_STATE->HoldStartTime = currentTime;
//         p_context->P_STATE->HoldState = false;
//     }
//     else if (Debounce_IsFallingEdge(p_debounce) && !activeState)
//     {
//         p_context->P_STATE->HoldState = false;
//     }
//     else if (activeState && !p_context->P_STATE->HoldState)
//     {
//         if (currentTime - p_context->P_STATE->HoldStartTime >= p_context->HOLD_TIME)
//             p_context->P_STATE->HoldState = true;
//     }

//     return p_context->P_STATE->HoldState;
// }
