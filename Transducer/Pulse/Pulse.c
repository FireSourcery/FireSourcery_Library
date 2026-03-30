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
    @file   Pulse.c
    @author FireSourcery
    @brief  Pulse - Init and stop detection config
*/
/******************************************************************************/
#include "Pulse.h"

/******************************************************************************/
/*
    Init
*/
/******************************************************************************/
void Pulse_Init(const Pulse_T * p_pulse)
{
    Pulse_State_T * p_state = p_pulse->P_STATE;

    /* Init timer HAL */
    HAL_Encoder_InitTimer(p_pulse->P_HAL_TIMER);
    HAL_Encoder_InitTimerFreq(p_pulse->P_HAL_TIMER, p_pulse->TIMER_FREQ);
    p_state->ExtendedTimerConversion = p_pulse->TIMER_FREQ / p_pulse->EXTENDED_TIMER_FREQ;
    p_state->DirectionComp = 1;

    Pulse_SetInitial(p_pulse);
}

void Pulse_SetInitial(const Pulse_T * p_pulse)
{
    Pulse_State_T * p_state = p_pulse->P_STATE;

    HAL_Encoder_ClearTimerOverflow(p_pulse->P_HAL_TIMER);
    HAL_Encoder_WriteTimer(p_pulse->P_HAL_TIMER, 0U);

    p_state->DeltaT = p_pulse->TIMER_FREQ; /* Initial as 1 second period => near-zero speed */
    p_state->ExtendedTimerPrev = *(p_pulse->P_EXTENDED_TIMER);

    /* Counter/FreqD state via math layer */
    Angle_Counter_Zero(p_pulse->P_COUNTER);
}

void Pulse_SetExtendedWatchStop_Millis(const Pulse_T * p_pulse, uint16_t stopTime_Millis)
{
    p_pulse->P_STATE->ExtendedDeltaTStop = stopTime_Millis * p_pulse->EXTENDED_TIMER_FREQ / 1000U;
}
