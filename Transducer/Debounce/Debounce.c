/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery / The Firebrand Forge Inc

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
        // if(p_debounce->DebouncedState != p_debounce->DebouncedStatePrev) //change polling state var
        {
            if(*p_debounce->CONFIG.P_TIMER - p_debounce->TimePrev > p_debounce->DebounceTime)
            {
                p_debounce->TimePrev = UINT32_MAX - p_debounce->DebounceTime; /* disable until next change in pin */
                p_debounce->DebouncedStatePrev = p_debounce->DebouncedState;
                p_debounce->DebouncedState = pinState;
            }
        }
    }

    return (p_debounce->DebouncedStatePrev ^ p_debounce->DebouncedState);
}

/* todo change state var */
bool Debounce_PollFallingEdge(Debounce_T * p_debounce)
{
    bool isEdge = ((p_debounce->DebouncedState == false) && (p_debounce->DebouncedStatePrev == true));
    if(isEdge == true) { p_debounce->DebouncedStatePrev = p_debounce->DebouncedState; }
    return isEdge;
}

bool Debounce_PollRisingEdge(Debounce_T * p_debounce)
{
    bool isEdge = ((p_debounce->DebouncedState == true) && (p_debounce->DebouncedStatePrev == false));
    if(isEdge == true) { p_debounce->DebouncedStatePrev = p_debounce->DebouncedState; }
    return isEdge;
}

bool Debounce_PollIsDualEdge(Debounce_T * p_debounce)
{
    bool isEdge = ((p_debounce->DebouncedState ^ p_debounce->DebouncedStatePrev) == true);
    if(isEdge == true) { p_debounce->DebouncedStatePrev = p_debounce->DebouncedState; }
    return isEdge;
}

Debounce_Edge_T Debounce_PollDualEdge(Debounce_T * p_debounce)
{
    return ((Debounce_PollIsDualEdge(p_debounce) == true) ? ((p_debounce->DebouncedState = true) ? DEBOUNCE_EDGE_RISING : DEBOUNCE_EDGE_FALLING) : DEBOUNCE_EDGE_NULL);
}
