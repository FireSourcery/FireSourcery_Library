#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2026 FireSourcery

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
    @file   ADC_Thread.h
    @author FireSourcery
    @brief  Base ISR and thread entry points. 1 ADC, no set and no handler.

    Software:       ADC_OnComplete_ISR, in the ADC ISR. ADC_ActivateMarked, in the thread that requests.
    Hw triggered:   ADC_ArmMarked, or ADC_Arm for a set as given, once per trigger, in place of ADC_ActivateMarked. ADC_OnCompleteArmed_ISR captures
                    the armed fifo, and continues nothing: every set converts on a trigger the thread arms it for.
    Hw sequenced:   ADC_ActivateSlots writes the sequencer. _ADC_MarkComplete, in the
                    transfer complete ISR, flags the set the caller armed.

    Both leave the same [CompleteFlags]. Everything that reads them is above this file.
*/
/******************************************************************************/
#include "_ADC.h"
#include "ADC.h"

/******************************************************************************/
/*
    Software activation
*/
/******************************************************************************/
/*!
    @brief  Capture the active channels, and continue with the remaining marked channels. Software started [ADC_ActivateMarked].
            Run in the ADC ISR. Higher priority than the thread calling ADC_ActivateMarked.
*/
static inline void ADC_OnComplete_ISR(ADC_T * p_adc)
{
    HAL_ADC_ClearConversionCompleteFlag(p_adc->P_HAL_ADC);
    if (_ADC_CaptureActive(p_adc) == true) /* Continue incrementing. Channels do not repeat until all marked channels have completed once */
    {
        if (p_adc->P_STATE->ChannelMarkers != 0UL) { _ADC_ActivateMarked(p_adc); }
        else { HAL_ADC_Deactivate(p_adc->P_HAL_ADC); }
    }
}

/*!
    @brief Capture by polling the status register, where the ISR is unavailable
*/
static inline void ADC_PollCompleteFifo(ADC_T * p_adc)
{
    if (HAL_ADC_ReadConversionCompleteFlag(p_adc->P_HAL_ADC) == true) { ADC_OnComplete_ISR(p_adc); }
}

/*!
    @brief  Start the marked channels. Run in the thread that requests conversions.
            Only 1 thread starts conversions, no critical section is needed.
            Not with a Hw trigger: its thread arms each set instead [ADC_ArmMarked].
*/
static inline void ADC_ActivateMarked(ADC_T * p_adc)
{
    /*
        While the ADC is active the remaining channels continue from the ISR.
        The ISR does not start within this block when a single thread calls it.
    */
    if ((p_adc->P_STATE->ChannelMarkers != 0UL) && (ADC_ReadIsActive(p_adc) == false))
    {
        _ADC_ActivateMarked(p_adc);
    }
#ifndef NDEBUG
    else if (ADC_ReadIsActive(p_adc) == true) { p_adc->P_STATE->IncompleteCycles++; }
#endif
}


/******************************************************************************/
/*
    Hw triggered activation
    1 fifo of marked channels per trigger, armed ahead: the set lands when the trigger places it, not when the thread runs.
    The trigger's route and delay are the Board's. Nothing converts by software start, and the completion continues nothing:
    a channel converts on a trigger the thread arms its set for. The trigger stays on from the first arm, so a trigger before
    the next arm finds the reset fifo and converts nothing. [ActiveChannelCount] is the armed state.
*/
/******************************************************************************/
/*!
    @brief  Capture the armed fifo. Continues nothing.
            Run in the ADC ISR, in place of ADC_OnComplete_ISR. Higher priority than the thread calling ADC_ArmMarked.
*/
static inline void ADC_OnCompleteArmed_ISR(ADC_T * p_adc)
{
    HAL_ADC_ClearConversionCompleteFlag(p_adc->P_HAL_ADC);
    /* The thread arms the next set on its own trigger [ADC_ArmMarked] */
    if (_ADC_CaptureActive(p_adc) == true) { HAL_ADC_Deactivate(p_adc->P_HAL_ADC); } /* Resets the fifo */
}


/* Its trigger did not come. The channels stay marked for the next arm */
static inline void _ADC_Disarm(ADC_T * p_adc)
{
    HAL_ADC_Deactivate(p_adc->P_HAL_ADC); /* Resets the fifo */
    p_adc->P_STATE->ActiveChannelCount = 0U;
#ifndef NDEBUG
    p_adc->P_STATE->MissedTriggers++;
#endif
}

/*!
    @brief  Arm [channels] on the next Hw trigger, as given, marked or not. In channel order, up to the fifo's depth.
            Run in the thread that requests conversions, at most once per trigger period. Higher priority ADC ISR.
            Still armed from the last call, no trigger came: disarmed, counted in MissedTriggers, and armed again.
            The caller's [channels] is not empty, and the ADC is idle [ADC_ReadIsActive].
*/
static inline void ADC_Arm(ADC_T * p_adc, adc_mask_t channels)
{
    assert(channels != 0UL);
    if (p_adc->P_STATE->ActiveChannelCount > 0U) { _ADC_Disarm(p_adc); }
    (void)_ADC_SetStateFrom(p_adc->P_STATE, channels);
    _ADC_Arm(p_adc);
}

/*!
    @brief  Arm the marked channels of [set], filtered [ADC_Arm]. The other marked channels wait for an arm of their own set.
            Nothing of [set] marked: a stale arm is still disarmed, and nothing armed.
            While the ADC converts, arms nothing.
    @return true where armed
*/
static inline bool ADC_ArmMarked(ADC_T * p_adc, adc_mask_t filter)
{
    adc_mask_t marked = p_adc->P_STATE->ChannelMarkers & filter;
    bool isArmed = false;

    if (ADC_ReadIsActive(p_adc) == false)
    {
        if (p_adc->P_STATE->ActiveChannelCount > 0U) { _ADC_Disarm(p_adc); } /* Here for the empty set. ADC_Arm's own is then dead, and compiles out */
        if (marked != 0UL) { ADC_Arm(p_adc, marked); isArmed = true; }
    }

    return isArmed;
}

// static inline void ADC_ActivateMarked_Thread(ADC_T * p_adc)
// {
// #if ADC_HW_TRIGGER_ENABLE
//     if (ADC_ReadIsArmed(p_adc) == true) { return; } /* The armed fifo converts first, and its completion continues the walk */
// #endif
//     ADC_ActivateMarked(p_adc);
// }

