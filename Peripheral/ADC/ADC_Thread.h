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
    Software:       ADC_OnComplete_ISR, in the ADC ISR. ADC_ProcMarked, in the thread that requests.
*/
/******************************************************************************/
#include "_ADC.h"
#include "ADC.h"

/******************************************************************************/
/*
    Hw sequenced
*/
/******************************************************************************/
/*!
    @brief  The transfer for the active sequence is complete, the results buffer holds it.
            Run in the transfer complete ISR. The caller clears the Hw flag.

    ADC ISR priority: a consumer that joins several ADCs needs their ISRs at 1 priority, for its own bookkeeping.
*/
static inline void ADC_OnCompleteSequence_ISR(const ADC_T * p_adc)
{
    assert(p_adc->P_STATE->p_ActiveSequence != NULL); /* A sequence must be activated before the trigger is enabled */
    _ADC_CompleteSequence(p_adc);
}

/******************************************************************************/
/*
    Software activation
*/
/******************************************************************************/
/*!
    @brief  Capture the active channels, continue with the remaining marked channels.
            Run in the ADC ISR. Higher priority than the thread calling ADC_ProcMarked.
*/
static inline void ADC_OnComplete_ISR(const ADC_T * p_adc)
{
    ADC_State_T * p_state = p_adc->P_STATE;

    HAL_ADC_ClearConversionCompleteFlag(p_adc->P_HAL_ADC);

    if (p_state->ActiveChannelCount > 0U)
    {
        p_state->PendingMarkers &= ~_ADC_CaptureActive(p_adc);

        /* The active sequence captured its last channel */
        if ((p_state->PendingMarkers == 0UL) && (p_state->p_ActiveSequence != NULL)) { _ADC_CompleteSequence(p_adc); }

        /* Channels do not repeat until all marked channels have completed once */
        if (p_state->ChannelMarkers != 0UL) { _ADC_StartMarked(p_adc); }
        else { HAL_ADC_Deactivate(p_adc->P_HAL_ADC); }
    }
#ifndef NDEBUG
    else
    {
        HAL_ADC_Deactivate(p_adc->P_HAL_ADC);
        p_state->ErrorCount++;
    }
#endif
}

/*!
    @brief Capture by polling the status register, where the ISR is unavailable
*/
static inline void ADC_PollComplete(const ADC_T * p_adc)
{
    if (HAL_ADC_ReadConversionCompleteFlag(p_adc->P_HAL_ADC) == true) { ADC_OnComplete_ISR(p_adc); }
}

/*!
    @brief  Start the marked channels. Run in the thread that requests conversions.
            Only 1 thread starts conversions, no critical section is needed.
*/
static inline void ADC_ProcMarked(const ADC_T * p_adc)
{
    /*
        While the ADC is active the remaining channels continue from the ISR.
        The ISR does not start within this block when a single thread calls it.
    */
    if ((p_adc->P_STATE->ChannelMarkers != 0UL) && (ADC_ReadIsActive(p_adc) == false))
    {
        _ADC_StartMarked(p_adc);
    }
#ifndef NDEBUG
    else if (ADC_ReadIsActive(p_adc) == true) { p_adc->P_STATE->IncompleteCycles++; }
#endif
}

/******************************************************************************/
/*
    N ADCs. Uniform iteration over an ADC array, not a synchronized set. That is ADC_Batch_T
*/
/******************************************************************************/
static inline void ADC_N_ProcMarked(const ADC_T * p_adcs, uint8_t count)
{
    for (uint8_t iAdc = 0U; iAdc < count; iAdc++) { ADC_ProcMarked(&p_adcs[iAdc]); }
}

static inline void ADC_N_Init(const ADC_T * p_adcs, uint8_t count)
{
    for (uint8_t iAdc = 0U; iAdc < count; iAdc++) { ADC_Init(&p_adcs[iAdc]); }
}
