/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

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
    @file    Debounce.c
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#include "Debounce.h"

void Debounce_Init(Debounce_T * p_debounce, uint16_t debounceTime)
{
    Pin_Input_Init(&p_debounce->Pin);
    p_debounce->DebounceTime         = debounceTime;
    p_debounce->DebouncedState         = Pin_Input_Read(&p_debounce->Pin);
    p_debounce->DebouncedStatePrev     = p_debounce->DebouncedState;
    p_debounce->RawStatePrev         = p_debounce->DebouncedState;
}

/*!
    @return true if state changed
*/
bool Debounce_CaptureState(Debounce_T * p_debounce)
{
    bool pinState = Pin_Input_Read(&p_debounce->Pin);

    /*
        check if state is the same for specified duration,
        vs preemptive lock out - noise may lock out real input
    */
    if(pinState != p_debounce->RawStatePrev)
    {
        p_debounce->TimePrev = *p_debounce->CONFIG.P_TIMER;
        p_debounce->RawStatePrev = pinState;
    }
    else
    {
        // if(p_debounce->DebouncedState != p_debounce->DebouncedStatePrev) // change polling state var
        {
            if(*p_debounce->CONFIG.P_TIMER - p_debounce->TimePrev > p_debounce->DebounceTime)
            {
                p_debounce->TimePrev = UINT32_MAX - p_debounce->DebounceTime; /* disable until next change in pin */
                p_debounce->DebouncedStatePrev = p_debounce->DebouncedState;
                p_debounce->DebouncedState = pinState;
            }
        }
    }

    return Debounce_GetIsEdge(p_debounce);
}


static inline void updateDebounceState(Debounce_T * p_debounce, bool isEdge) { if(isEdge == true) { p_debounce->DebouncedStatePrev = p_debounce->DebouncedState; } }

/* todo change state var */
bool Debounce_PollFallingEdge(Debounce_T * p_debounce)
{
    bool isEdge = Debounce_GetIsFallingEdge(p_debounce);
    updateDebounceState(p_debounce, isEdge);
    return isEdge;
}

bool Debounce_PollRisingEdge(Debounce_T * p_debounce)
{
    bool isEdge = Debounce_GetIsRisingEdge(p_debounce);
    updateDebounceState(p_debounce, isEdge);
    return isEdge;
}

bool Debounce_PollDualEdge(Debounce_T * p_debounce)
{
    bool isEdge = Debounce_GetIsEdge(p_debounce);
    updateDebounceState(p_debounce, isEdge);
    return isEdge;
}

Debounce_Edge_T Debounce_PollEdge(Debounce_T * p_debounce)
{
    Debounce_Edge_T edge = Debounce_GetEdge(p_debounce);
    updateDebounceState(p_debounce, edge != DEBOUNCE_EDGE_NULL);
    return edge;
}
