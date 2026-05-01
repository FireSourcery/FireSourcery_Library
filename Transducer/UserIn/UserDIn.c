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
/*
    Disable skips init, Modal Poll always returns 0
*/
void UserDIn_InitFrom(UserDIn_T * p_dev, UserDIn_Config_T * p_config)
{
    // if (p_config != NULL) { p_dev ->P_STATE.Confg = *p_config; }
    if (p_dev->PIN.P_HAL_PIN == NULL) { p_dev->P_STATE->Mode = USER_DIN_MODE_DISABLED; return; } /* runtime config for no pin connected. read as 0 */
    Pin_Input_Init(&p_dev->PIN);

    if (p_config != NULL)
    {
        p_dev->P_STATE->Mode = p_config->Mode;
    #ifdef USER_DIN_CMD_TABLE_ENABLE
        p_dev->P_STATE->OptCmd = p_dev->P_CMD_TABLE[p_config->CmdId];
    #else
        p_dev->P_STATE->OptCmd = UserDIn_CmdNull;
    #endif
    }
    else
    {
        p_dev->P_STATE->Mode = USER_DIN_MODE_NORMAL;
        p_dev->P_STATE->OptCmd = UserDIn_CmdNull;
    }

    Debounce_Init(&p_dev->P_STATE->Debounce, p_dev->DEBOUNCE_TIME);

    p_dev->P_STATE->Mode = USER_DIN_MODE_NORMAL;
    /* Initialize debounce state to current pin reading */
    p_dev->P_STATE->Debounce.Time0 = GetTime(p_dev);
    p_dev->P_STATE->Debounce.State0 = ReadPin(p_dev);
    p_dev->P_STATE->Debounce.Output = p_dev->P_STATE->Debounce.State0;
    p_dev->P_STATE->OutputPrev = p_dev->P_STATE->Debounce.State0;
}

void UserDIn_Init(UserDIn_T * p_dev) { UserDIn_InitFrom(p_dev, p_dev->P_NVM_CONFIG); }

/******************************************************************************/
/*
    Essential Polling Functions
*/
/******************************************************************************/
/*! @return On/Off state */
bool UserDIn_PollState(UserDIn_T * p_dev) { return Debounce_Poll(&p_dev->P_STATE->Debounce, GetTime(p_dev), ReadPin(p_dev)); }

/*! @return true on change */
bool UserDIn_PollEdge(UserDIn_T * p_dev)
{
    p_dev->P_STATE->OutputPrev = UserDIn_GetState(p_dev);
    UserDIn_PollState(p_dev);
    return UserDIn_IsEdge(p_dev);
}

bool UserDIn_PollRisingEdge(UserDIn_T * p_dev) { return UserDIn_PollEdge(p_dev) && (UserDIn_GetState(p_dev) == true); }
bool UserDIn_PollFallingEdge(UserDIn_T * p_dev) { return UserDIn_PollEdge(p_dev) && (UserDIn_GetState(p_dev) == false); }
UserDIn_Edge_T UserDIn_PollEdgeValue(UserDIn_T * p_dev) { return UserDIn_PollEdge(p_dev) ? UserDIn_GetEdge(p_dev) : USER_DIN_EDGE_NULL; }


/* run time select should not also be disabled.  */
UserDIn_Edge_T UserDIn_PollOptional(UserDIn_T * p_dev)
{
    if (p_dev == NULL) { return USER_DIN_EDGE_NULL; }
    return UserDIn_PollEdgeValue(p_dev);
}

/******************************************************************************/
/*

*/
/******************************************************************************/
/* with runtime disable */
UserDIn_Edge_T UserDIn_Modal_PollEdgeValue(UserDIn_T * p_dev)
{
    switch (p_dev->P_STATE->Mode)
    {
        case USER_DIN_MODE_DISABLED: return USER_DIN_EDGE_NULL;
        case USER_DIN_MODE_NORMAL: return UserDIn_PollEdgeValue(p_dev);
        default: return USER_DIN_EDGE_NULL;
    }
}

/*
    Call must provide context corresponding to function set
*/
void _UserDIn_Modal_PollEdgeCmd(UserDIn_T * p_dev, void * p_context)
{
    UserDIn_Edge_T edge = UserDIn_Modal_PollEdgeValue(p_dev);
    if (edge != USER_DIN_EDGE_NULL) { p_dev->P_STATE->OptCmd(p_context, edge); }
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
