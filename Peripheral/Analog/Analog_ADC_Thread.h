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


/******************************************************************************/
/*!
    Thread
*/
/******************************************************************************/

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
    Only a single thread starts the conversions.
    No Critical is needed
*/
static inline void Analog_ADC_ProcMarked(const Analog_ADC_T * p_adc)
{
    /* if Adc is still active, channels will remain marked until the next call */
    /*
        no possibility of interrupt when this function is called by a single thread
    */
    if (Analog_ADC_ReadIsActive(p_adc) == false) /* ISR should not start witin this block */
    {
        if (p_adc->P_ADC_STATE->ChannelIndex >= p_adc->CHANNEL_COUNT)
        {
            /*
                State Error.
                ADC did not complete all Marked since previous called, and should be Active or in the ISR.
                ADC is not active, and also not inside the ISR.
            */
        #ifndef NDEBUG
            p_adc->P_ADC_STATE->ErrorCount++;
        #endif
            // assert(false);
            p_adc->P_ADC_STATE->ChannelIndex = 0U;
        }

        // /* Ensure no active conversions from previous incomplete cycle */
        // if (p_adc->P_ADC_STATE->ActiveConversionCount != 0U)
        // {
        // #ifndef NDEBUG
        //     p_adc->P_ADC_STATE->ErrorCount++;
        // #endif
        //     p_adc->P_ADC_STATE->ActiveConversionCount = 0U;
        // }

        ADC_ProcStart(p_adc, p_adc->P_ADC_STATE);
    }
}


/*
*/
static inline void Analog_ADCN_ProcMarked(const Analog_ADC_T * const p_adcs, uint8_t count)
{
    for (uint8_t iAdc = 0U; iAdc < count; iAdc++) { Analog_ADC_ProcMarked(&p_adcs[iAdc]); }
}

/* instanced to ensure inlining optimization */
// static inline void Analog_ADC0_ProcMarked(void)
// {
//     Analog_ADC_ProcMarked(&G_ANALOG_ADCS[0U]);
// }

