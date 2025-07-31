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
    @file   Analog_ADC_Thread.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "_Analog_ADC.h"
#include "Analog_ADC.h"


/******************************************************************************/
/*!
    Thread
*/
/******************************************************************************/
/*
    ActivateMarked
    Update State and Start

    ADC_SetStateInternal
*/
static void ADC_ProcStart(const Analog_ADC_T * p_adc, Analog_ADC_State_T * p_state)
{
    ADC_StartFrom(p_adc, &p_adc->P_CONVERSION_CHANNELS[0U], p_state->ChannelMarkers);

    // p_state->ChannelMarkers &= ~processed;
    // p_state->ChannelMarkers &= ~ADC_StartFrom(p_adc, &p_adc->P_CONVERSION_CHANNELS[0U], p_state->ChannelMarkers);
}

// from settable pointer
// static void ADC_ProcStartContext(const Analog_ADC_T * p_adc, Analog_ADC_State_T * p_state)
// {
//      p_adc->P_ADC_STATE->p_ConversionChannels = p_conversions;
//     ADC_StartConversions(p_state, &p_adc->P_ADC_STATE->p_ConversionChannels, p_state->ChannelMarkers);
// }

static inline void ADC_OnComplete(const Analog_ADC_T * p_adc, Analog_ADC_State_T * p_state)
{
    HAL_ADC_ClearConversionCompleteFlag(p_adc->P_HAL_ADC);

    if (p_state->ActiveConversionCount > 0U)
    {
        ADC_Capture(p_adc, p_state);
        // p_state->ChannelComplete =

        // if (p_state->p_ActiveContext != NULL)
        // {
         //     ADC_StartConversionsFrom(p_state, p_state->p_ActiveContext-> , p_state->p_ActiveContext->ChannelMarkers); //channel marker maped per adc
         //     ADC_ProcStartBatch
        //     // Analog_Batch_OnComplete(p_adc->p_Batch );
        // }

        /* Continue incrementing. Channels do not repeat until all marked channels have completed once */
        if (p_state->ChannelMarkers != 0UL) { ADC_ProcStart(p_adc, p_state); }
        else { HAL_ADC_Deactivate(p_adc->P_HAL_ADC); } /* alternatively flag for successful conversions */

        // optionally proc on complete in parallel, after ADC_Activate
        // if (p_conversion->ON_COMPLETE != NULL) { p_conversion->ON_COMPLETE(p_conversion->P_CONTEXT); }
    }
#ifndef NDEBUG
    else
    {
        HAL_ADC_Deactivate(p_adc->P_HAL_ADC);
        p_state->ErrorCount++;
    }
#endif
}

/******************************************************************************/
/*
    Inline for instanced versions
    Alternatively Keep all function in source file, and extern const/#define 'hook'
*/
/******************************************************************************/
/*!
    @brief  Capture ADC results, Process Channel OnComplete Functions.
            Run in corresponding ADC ISR

    ADC ISR should be higher priority than thread calling Analog_StartConversions()
*/
static inline void Analog_ADC_OnComplete_ISR(const Analog_ADC_T * p_adc)
{
    ADC_OnComplete(p_adc, p_adc->P_ADC_STATE);
}

/*!
    @brief Capture ADC results, by polling status register, if ISR is unavailable
*/
static inline void Analog_ADC_PollComplete(const Analog_ADC_T * p_adc)
{
    if (HAL_ADC_ReadConversionCompleteFlag(p_adc->P_HAL_ADC) == true) { Analog_ADC_OnComplete_ISR(p_adc); }
}

/*!
    @brief Activate marked channels
    Only a single thread starts the conversions. No Critical is needed.
*/
static inline void Analog_ADC_ProcMarked(const Analog_ADC_T * p_adc)
{
    /*
        If Adc is still active, remaining channels will continue processing until the next call.
        ISR should not start witin this block, when this function is called by a single thread
     */
    if (Analog_ADC_ReadIsActive(p_adc) == false)
    {
        ADC_ProcStart(p_adc, p_adc->P_ADC_STATE);
    }
#ifndef NDEBUG
    else
    {
        p_adc->P_ADC_STATE->IncompleteCycles++;
    }
#endif
}


/*
*/
// static inline void Analog_ADCN_ProcMarked(const Analog_ADC_T * const p_adcs, uint8_t count)
// {
//     for (uint8_t iAdc = 0U; iAdc < count; iAdc++) { Analog_ADC_ProcMarked(&p_adcs[iAdc]); }
// }

/* instanced to ensure inlining optimization */
// static inline void Analog_ADC0_ProcMarked(void)
// {
//     Analog_ADC_ProcMarked(&G_ANALOG_ADCS[0U]);
// }

