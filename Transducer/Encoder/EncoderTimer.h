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
    @file   EncoderTimer.h
    @author FireSourcery
    @brief  Thin encodertimer/timer HAL wrapper. Timer capture, extended timer,
            and stop detection. Counter math delegated to Angle_Counter_T.
*/
/******************************************************************************/
#include "HAL_Encoder.h"
#include "Math/Angle/Angle_Counter.h"

#include <stdint.h>
#include <stdbool.h>


/*
    Encoder_Base / TimerCounter / Sqaure wave counter
*/
/******************************************************************************/
/*
    State - Timer-specific only
    Counter/FreqD state lives in Angle_Counter_T (math layer)
*/
/******************************************************************************/
typedef struct EncoderTimer_State
{
    /* Timer Capture */
    uint32_t DeltaT;        /* Timer ticks between edges */

    /* Extended Timer */
    uint32_t ExtendedTimerPrev;
    uint32_t ExtendedTimerConversion;   /* Extended timer ticks to base timer ticks */

    /* Direction */
    int32_t DirectionComp;  /* Direction sign for unsigned capture modes */

    /* Stop detection threshold */
    uint16_t ExtendedDeltaTStop;        /* Extended timer ticks to determine stopped */

    Angle_Counter_T Counter; /* Embedded math state for count accumulation and speed estimation */
}
EncoderTimer_State_T;

/******************************************************************************/
/*
    Const Instance
*/
/******************************************************************************/
typedef const struct EncoderTimer
{
    HAL_Encoder_Timer_T * P_HAL_TIMER;      /* DeltaT Timer */
    uint32_t TIMER_FREQ;

    const volatile uint32_t * P_EXTENDED_TIMER; /* 32-bit extension for low-speed stop detection */
    uint32_t EXTENDED_TIMER_FREQ;

    uint32_t SAMPLE_FREQ;       /* Speed sample freq (e.g. 1kHz) */
    uint32_t SAMPLE_TIME;       /* TIMER_FREQ / SAMPLE_FREQ */

    EncoderTimer_State_T * P_STATE;
}
EncoderTimer_T;

#define ENCODER_TIMER_INIT(p_TimerHal, TimerFreq, p_ExtTimer, ExtTimerFreq, SampleFreq, p_State, p_Counter) \
{                                                           \
    .P_HAL_TIMER        = (p_TimerHal),                     \
    .TIMER_FREQ         = (TimerFreq),                      \
    .P_EXTENDED_TIMER   = (p_ExtTimer),                     \
    .EXTENDED_TIMER_FREQ = (ExtTimerFreq),                  \
    .SAMPLE_FREQ        = (SampleFreq),                     \
    .SAMPLE_TIME        = (TimerFreq) / (SampleFreq),       \
    .P_STATE            = (p_State),                        \
    .P_COUNTER          = (p_Counter),                      \
}

#define ENCODER_TIMER_STATE_ALLOC() (&(EncoderTimer_State_T){0})
#define ENCODER_TIMER_COUNTER_ALLOC() (&(Angle_Counter_T){0})
