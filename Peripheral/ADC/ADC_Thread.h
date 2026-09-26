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
    Hw sequenced:   ADC_ActivateSlots writes the sequencer. ADC_OnCompleteTransfer_ISR, in the
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
static inline void _ADC_ActivateMarked(ADC_T * p_adc)
{
    _ADC_SetStateFrom(p_adc->P_STATE, p_adc->P_STATE->ChannelMarkers);
    assert(p_adc->P_STATE->ActiveChannelCount > 0U); /* Callers gate on ChannelMarkers, so the selection is never empty */
    _ADC_Activate(p_adc);
}

/*
    The captured channels leave the work queue and enter the completion flags, so the two words
    hold the whole conversion state: what is left to convert, and what has landed unconsumed.
*/
static inline void _ADC_OnComplete(ADC_T * p_adc)
{
    ADC_State_T * p_state = p_adc->P_STATE;

#ifndef NDEBUG
    if (p_state->ActiveChannelCount == 0U) { HAL_ADC_Deactivate(p_adc->P_HAL_ADC); p_state->ErrorCount++; }
#if (ADC_FIFO_LENGTH_MAX > 1U)
    else if (p_state->ActiveChannelCount != HAL_ADC_ReadFifoCount(p_adc->P_HAL_ADC)) { p_state->FifoMismatch++; }
#endif
#endif

    if (p_state->ActiveChannelCount > 0U)
    {
        adc_mask_t captured = _ADC_Capture(p_adc);

        p_state->ChannelMarkers &= ~captured;
        p_state->CompleteFlags  |=  captured;
        p_state->ActiveChannelCount = 0U;

        /* Continue incrementing. Channels do not repeat until all marked channels have completed once */
        if (p_state->ChannelMarkers != 0UL) { _ADC_ActivateMarked(p_adc); }
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
    _ADC_OnComplete(p_adc);
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
        _ADC_ActivateMarked(p_adc);
    }
#ifndef NDEBUG
    else if (ADC_ReadIsActive(p_adc) == true) { p_adc->P_STATE->IncompleteCycles++; }
#endif
}

/******************************************************************************/
/*
    Hw sequenced activation
*/
/******************************************************************************/
/******************************************************************************/
/*
    Route the trigger to [channelStart, channelStart + count). The Hw converts and transfers on each trigger.
    Board or platform provided, when ADC_HW_SEQUENCER_ENABLE is true.
*/
/*
    The sequencer routes the trigger to the slot range, and converts it on every trigger until
    another range is written. Software has no sequencer: its register write is the fifo push.
*/
static inline void ADC_ActivateSlots(ADC_T * p_adc, adc_channel_t channelStart, uint8_t count)
{
#ifdef HAL_ADC_DMA_ENABLE
    HAL_ADC_SelectDmaSequence(p_adc->P_HAL_ADC, channelStart, count);
#else
    (void)p_adc; (void)channelStart; (void)count;
#endif
}

/*!
    @brief  The transfer for [armed] is complete, the results buffer holds it.
            Run in the transfer complete ISR. The caller clears the Hw flag, and names the set
            it armed - the base has no sequencer state to read it back from.

    ADC ISR priority: a consumer that joins several ADCs needs their ISRs at 1 priority.
*/
static inline void ADC_OnCompleteTransfer_ISR(ADC_T * p_adc, adc_mask_t armed)
{
    p_adc->P_STATE->CompleteFlags |= armed;
}
