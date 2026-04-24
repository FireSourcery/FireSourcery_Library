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
    @file    Blinky.c
    @author FireSourcery
    @brief     Pin Indicator

*/
/******************************************************************************/
#include "Blinky.h"
#include "Peripheral/Pin/Pin.h"
#include "Framework/Timer/Timer.h"

#include <stddef.h>

/*!
    @brief initialize blink switch
*/
void Blinky_Init(const Blinky_T * p_blinky)
{
    Pin_Output_Init(&p_blinky->PIN);
    Pin_Output_Off(&p_blinky->PIN);
    TimerT_Init(&p_blinky->TIMER);
    p_blinky->P_STATE->PatternFunction = NULL;
}

static void Pattern_PeriodicToggle(const Blinky_T * p_blinky);

/*!
    @brief
    @param[in]
*/
//disabled, oneshot, periodic
void Blinky_Proc(const Blinky_T * p_blinky)
{
    Blinky_State_T * p_state = p_blinky->P_STATE;
    if (TimerT_Modal_Poll(&p_blinky->TIMER) == true)
    {
        switch (p_blinky->TIMER.P_STATE->Mode)
        {
            case TIMER_MODE_STOPPED: /* OneShot/CounterN completed */
                Pin_Output_Off(&p_blinky->PIN);
                TimerT_StartPeriodic(&p_blinky->TIMER, p_state->OffTime); /* Restore Periodic */
                break;
            case TIMER_MODE_ONE_SHOT_COUNTER: /* BlinkN in progress, Timer Counter handles repeat */
                Pattern_PeriodicToggle(p_blinky);
                break;
            case TIMER_MODE_PERIODIC:
                if (p_state->PatternFunction != NULL) { p_state->PatternFunction(p_blinky); }
                break;

            case TIMER_MODE_DISABLED: break;
            case TIMER_MODE_ONE_SHOT: break; /* Timer sets mode to stop */
            case TIMER_MODE_PERIODIC_COUNTER: break;/* Not used */
            default: break;
        }
    }
}


void Blinky_On(const Blinky_T * p_blinky) { p_blinky->P_STATE->IsOn = true; Pin_Output_High(&p_blinky->PIN); }
void Blinky_Off(const Blinky_T * p_blinky) { p_blinky->P_STATE->IsOn = false; Pin_Output_Low(&p_blinky->PIN); }
void _Blinky_Toggle(const Blinky_T * p_blinky) { Pin_Output_Toggle(&p_blinky->PIN); } /* with register state */
void Blinky_Toggle(const Blinky_T * p_blinky) { if (p_blinky->P_STATE->IsOn == true) { Blinky_Off(p_blinky); } else { Blinky_On(p_blinky); } }

void Blinky_Stop(const Blinky_T * p_blinky)
{
    TimerT_Stop(&p_blinky->TIMER);
    Blinky_Off(p_blinky);
    p_blinky->P_STATE->PatternFunction = NULL;
}

/* Start with On first */
void Blinky_Blink_OnOff(const Blinky_T * p_blinky, uint32_t duration)
{
    Blinky_On(p_blinky);
    TimerT_StartOneShot(&p_blinky->TIMER, duration);
}

/* Toggle */
void Blinky_Blink_Toggle(const Blinky_T * p_blinky, uint32_t duration)
{
    _Blinky_Toggle(p_blinky);
    TimerT_StartOneShot(&p_blinky->TIMER, duration);
}

void Blinky_Blink(const Blinky_T * p_blinky, uint32_t onTime)
{
    Blinky_Blink_OnOff(p_blinky, onTime);
}

void Blinky_BlinkN(const Blinky_T * p_blinky, uint32_t onTime, uint32_t offTime, uint8_t nRepeat)
{
    p_blinky->P_STATE->OnTime = onTime;
    p_blinky->P_STATE->OffTime = offTime;
    Blinky_On(p_blinky);
    TimerT_StartCounterN(&p_blinky->TIMER, onTime, nRepeat * 2U - 1U);
}


/******************************************************************************/
/*
*/
/******************************************************************************/
static void Pattern_PeriodicToggle(const Blinky_T * p_blinky)
{
    if (p_blinky->P_STATE->IsOn == true)
    {
        Blinky_Off(p_blinky);
        TimerT_SetPeriod(&p_blinky->TIMER, p_blinky->P_STATE->OffTime);
    }
    else
    {
        Blinky_On(p_blinky);
        TimerT_SetPeriod(&p_blinky->TIMER, p_blinky->P_STATE->OnTime);
    }
}

void Blinky_StartPeriodic(const Blinky_T * p_blinky, uint32_t onTime, uint32_t offTime)
{
    p_blinky->P_STATE->OnTime = onTime;
    p_blinky->P_STATE->OffTime = offTime;
    p_blinky->P_STATE->PatternFunction = Pattern_PeriodicToggle;
    TimerT_StartPeriodic(&p_blinky->TIMER, offTime);
}

/******************************************************************************/
/*

*/
/******************************************************************************/
// static inline void Blinky_Blink_Short(MotorController_T * p_mc) { Blinky_Blink(&p_mc->Buzzer, 500U); }
// static inline void Blinky_Blink_PeriodicType1(MotorController_T * p_mc) { Blinky_StartPeriodic(&p_mc->Buzzer, 500U, 500U); }