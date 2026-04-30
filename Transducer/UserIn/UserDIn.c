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
static inline bool ReadPin(UserDIn_T * p_dev) { return Pin_Input_Read(&p_dev->PIN); }
static inline uint32_t GetTime(UserDIn_T * p_dev) { return *p_dev->P_TIMER; }

/******************************************************************************/
/*
    Public Functions
*/
/******************************************************************************/
void UserDIn_Init(UserDIn_T * p_dev)
{
    Pin_Input_Init(&p_dev->PIN);

    Debounce_Init(&p_dev->P_STATE->Debounce, p_dev->DEBOUNCE_TIME);

    /* Initialize debounce state to current pin reading */
    p_dev->P_STATE->Debounce.Time0 = GetTime(p_dev);
    p_dev->P_STATE->Debounce.State0 = ReadPin(p_dev);
    p_dev->P_STATE->Debounce.Output = p_dev->P_STATE->Debounce.State0;
    p_dev->P_STATE->OutputPrev = p_dev->P_STATE->Debounce.State0;
}

/*
    Polling Functions
*/
/*! @return On/Off state */
bool UserDIn_PollState(UserDIn_T * p_dev) { return Debounce_Poll(&p_dev->P_STATE->Debounce, GetTime(p_dev), ReadPin(p_dev)); }

/*! @return true on change */
bool UserDIn_PollEdge(UserDIn_T * p_dev)
{
    p_dev->P_STATE->OutputPrev = Debounce_GetState(&p_dev->P_STATE->Debounce);
    Debounce_Poll(&p_dev->P_STATE->Debounce, GetTime(p_dev), ReadPin(p_dev));
    return is_edge(p_dev->P_STATE->OutputPrev, Debounce_GetState(&p_dev->P_STATE->Debounce));
}

bool UserDIn_PollRisingEdge(UserDIn_T * p_dev) { return UserDIn_PollEdge(p_dev) && (UserDIn_GetState(p_dev) == true); }
bool UserDIn_PollFallingEdge(UserDIn_T * p_dev) { return UserDIn_PollEdge(p_dev) && (UserDIn_GetState(p_dev) == false); }
UserDIn_Edge_T UserDIn_PollEdgeValue(UserDIn_T * p_dev) { return UserDIn_PollEdge(p_dev) ? UserDIn_GetEdge(p_dev) : USER_DIN_EDGE_NULL; }


UserDIn_Edge_T UserDIn_Modal_PollEdgeValue(UserDIn_T * p_dev)
{
    switch (p_dev->P_STATE->Mode)
    {
        case USER_DIN_MODE_DISABLED: return USER_DIN_EDGE_NULL;
        case USER_DIN_MODE_NORMAL: return UserDIn_PollEdgeValue(p_dev);
        default: return USER_DIN_EDGE_NULL;
    }
}



// /******************************************************************************/
// /*
//     Mode Extensions - Toggle Mode
// */
// /******************************************************************************/
// static bool _UserDIn_ProcessToggleMode( UserDIn_T * p_dev)
// {
//     if (Debounce_IsRisingEdge(&p_dev->P_STATE->Debounce) && _UserDIn_GetActiveState(p_dev))
//         p_dev->P_STATE->ToggleState = !p_dev->P_STATE->ToggleState;
//     return p_dev->P_STATE->ToggleState;
// }

// /******************************************************************************/
// /*
//     Mode Extensions - Momentary Mode
// */
// /******************************************************************************/
// bool _UserDIn_ProcessMomentaryMode( UserDIn_T * p_dev) { return _UserDIn_GetActiveState(p_dev); }

// /******************************************************************************/
// /*
//     Mode Extensions - Hold Mode
// */
// /******************************************************************************/
// static bool _UserDIn_ProcessHoldMode( UserDIn_T * p_dev)
// {
//     bool activeState = _UserDIn_GetActiveState(p_dev);
//     uint16_t currentTime = GetTime(p_dev);
//     Debounce_T * p_debounce = &p_dev->P_STATE->Debounce;

//     if (Debounce_IsRisingEdge(p_debounce) && activeState)
//     {
//         p_dev->P_STATE->HoldStartTime = currentTime;
//         p_dev->P_STATE->HoldState = false;
//     }
//     else if (Debounce_IsFallingEdge(p_debounce) && !activeState)
//     {
//         p_dev->P_STATE->HoldState = false;
//     }
//     else if (activeState && !p_dev->P_STATE->HoldState)
//     {
//         if (currentTime - p_dev->P_STATE->HoldStartTime >= p_dev->HOLD_TIME)
//             p_dev->P_STATE->HoldState = true;
//     }

//     return p_dev->P_STATE->HoldState;
// }
