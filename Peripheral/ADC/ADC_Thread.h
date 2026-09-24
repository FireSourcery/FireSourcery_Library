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
    @brief  ISR and thread entry points.

    Hw sequenced:   ADC_OnCompleteSequence_ISR, in the transfer complete ISR. 1 per ADC, per trigger.
    Software:       ADC_OnComplete_ISR, in the ADC ISR. ADC_ActivateMarked, in the thread that requests.

    _ADC_[Name](p_adc, p_state, ...) is private, and takes the resolved state.
    ADC_[Name]_ISR(p_adc) is the outermost call, for the Board to place in the vector.
    The batch entries are the same shape, over a trigger's ADCs. ADC_Batch.h
*/
/******************************************************************************/
#include "_ADC.h"
#include "ADC.h"


/******************************************************************************/
/*
    Software activation
*/
/******************************************************************************/
static void _ADC_ActivateMarked(ADC_T * p_adc, ADC_State_T * p_state)
{
    _ADC_SetStateFrom(p_state, &p_adc->P_CHANNELS[0U], p_state->ChannelMarkers);
    /* Nothing marked selects no channels, and the fifo push has no pin to write */
    if (p_state->ActiveChannelCount > 0U) { _ADC_Activate(p_adc->P_HAL_ADC, p_state); }
}

static inline void _ADC_OnComplete(ADC_T * p_adc, ADC_State_T * p_state)
{
#ifndef NDEBUG
    if (p_state->ActiveChannelCount == 0U) { HAL_ADC_Deactivate(p_adc->P_HAL_ADC); p_state->ErrorCount++; }
#if (ADC_FIFO_LENGTH_MAX > 1U)
    else if (p_state->ActiveChannelCount != HAL_ADC_ReadFifoCount(p_adc->P_HAL_ADC)) { p_state->FifoMismatch++; }
#endif
#endif

    if (p_state->ActiveChannelCount > 0U)
    {
        adc_mask_t captured = _ADC_Capture(p_adc->P_HAL_ADC, p_state, p_adc->P_CHANNEL_RESULTS);

        p_state->ChannelMarkers &= ~captured;

        /* The set captured its last channel. Before the continue, so a request made within it is picked up here */
        if (_ADC_IsSequenceComplete(p_state, captured) == true) { (void)_ADC_OnCompleteSequence(p_adc, p_state); }

        /* Continue incrementing. Channels do not repeat until all marked channels have completed once */
        if (p_state->ChannelMarkers != 0UL) { _ADC_ActivateMarked(p_adc, p_state); }
        else { HAL_ADC_Deactivate(p_adc->P_HAL_ADC); }
    }
}

/*!
    @brief  Capture the active channels, continue with the remaining marked channels.
            Run in the ADC ISR. Higher priority than the thread calling ADC_ActivateMarked.
*/
static inline void ADC_OnComplete_ISR(ADC_T * p_adc)
{
    HAL_ADC_ClearConversionCompleteFlag(p_adc->P_HAL_ADC);
    _ADC_OnComplete(p_adc, p_adc->P_STATE);
}

/*!
    @brief Capture by polling the status register, where the ISR is unavailable
*/
static inline void ADC_PollComplete(ADC_T * p_adc)
{
    if (HAL_ADC_ReadConversionCompleteFlag(p_adc->P_HAL_ADC) == true) { ADC_OnComplete_ISR(p_adc); }
}

/*!
    @brief  Start the marked channels. Run in the thread that requests conversions.
            Only 1 thread starts conversions, no critical section is needed.
*/
static inline void ADC_ActivateMarked(ADC_T * p_adc)
{
    /*
        While the ADC is active the remaining channels continue from the ISR.
        The ISR does not start within this block when a single thread calls it.
    */
    if ((p_adc->P_STATE->ChannelMarkers != 0UL) && (ADC_ReadIsActive(p_adc) == false))
    {
        _ADC_ActivateMarked(p_adc, p_adc->P_STATE);
    }
#ifndef NDEBUG
    else if (ADC_ReadIsActive(p_adc) == true) { p_adc->P_STATE->IncompleteCycles++; }
#endif
}


/******************************************************************************/
/*
    Sequence. The mechanism is in _ADC.h, shared with ADC_Batch.h
*/
/******************************************************************************/
/*!
    @brief  The transfer for the active sequence is complete, the results buffer holds it.
            Run in the transfer complete ISR. The caller clears the Hw flag.

    ADC ISR priority: a consumer that joins several ADCs needs their ISRs at 1 priority, for its own bookkeeping.
*/
static inline void ADC_OnCompleteSequence_ISR(ADC_T * p_adc)
{
    assert(p_adc->P_STATE->p_ActiveSequence != NULL); /* A sequence must be activated before the trigger is enabled */
    (void)_ADC_OnCompleteSequence(p_adc, p_adc->P_STATE);
}
