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
    @file   Analog.c
    @author FireSourcery
    @brief  Analog module conventional function definitions
    @version V0
*/
/******************************************************************************/
#include "Analog.h"

#include <assert.h>

/******************************************************************************/
/*!
    Private
*/
/******************************************************************************/
/******************************************************************************/
/* Per ADC */
/******************************************************************************/
static inline adc_t CaptureConversionResult(const Analog_ADC_T * p_adc, const Analog_Conversion_T * p_conversion)
{
    adc_t result = HAL_ADC_ReadResult(p_adc->P_HAL_ADC, p_conversion->PIN);
    /* Capture may incurr additional interrupt time, while eliminating double buffer */
    if (p_conversion->ON_COMPLETE != NULL) { p_conversion->ON_COMPLETE(p_conversion->P_CONTEXT, result); }
    return result;
}

static inline void CaptureActiveConversion(const Analog_ADC_T * p_adc, const Analog_Conversion_T * p_conversion)
{
    p_conversion->P_STATE->Result = CaptureConversionResult(p_adc, p_conversion); /* alternatively disable when ON_COMPLETE is defined */
    // p_conversion->P_STATE->IsMarked = false;
}

/*

*/
static inline void ADC_CaptureSingle(Analog_ADC_T * p_adc) { CaptureActiveConversion(p_adc, p_adc->ActiveConversions[0U]); }

/*
    Activate and wait for return
*/
static inline void ADC_StartSingle(Analog_ADC_T * p_adc) { HAL_ADC_Activate(p_adc->P_HAL_ADC, p_adc->ActiveConversions[0U]->PIN); }

/*
    FIFO Version
*/
static inline void ADC_CaptureFifo(Analog_ADC_T * p_adc)
{
    if (p_adc->ActiveConversionCount == HAL_ADC_ReadFifoCount(p_adc->P_HAL_ADC))
    {
        /* Read in the same way it was pushed */
        for (uint8_t iConversion = 0U; iConversion < p_adc->ActiveConversionCount; iConversion++)
            { CaptureActiveConversion(p_adc, p_adc->ActiveConversions[iConversion]); }
    }
#ifndef NDEBUG
    else { p_adc->ErrorCount++; }
#endif
}

/*
*/
static inline void ADC_StartFifo(Analog_ADC_T * p_adc)
{
    adc_pin_t pins[ADC_FIFO_LENGTH_MAX];
    for (uint8_t iConversion = 0U; iConversion < p_adc->ActiveConversionCount; iConversion++)
        { pins[iConversion] = p_adc->ActiveConversions[iConversion]->PIN; }

    HAL_ADC_WriteFifoCount(p_adc->P_HAL_ADC, p_adc->ActiveConversionCount);
    HAL_ADC_WriteFifo_ActivateOnLast(p_adc->P_HAL_ADC, pins, p_adc->ActiveConversionCount);
}


static inline void ADC_Capture(Analog_ADC_T * p_adc)
{
    // assert(p_adc->ActiveConversionCount > 0U);
#ifdef CONFIG_ANALOG_ADC_HW_FIFO_ENABLE
    ADC_CaptureFifo(p_adc);
#else
    ADC_CaptureSingle(p_adc);
#endif
}

static inline void ADC_Start(Analog_ADC_T * p_adc)
{
    if (p_adc->ActiveConversionCount > 0U)
    {
    #ifdef CONFIG_ANALOG_ADC_HW_FIFO_ENABLE
        ADC_StartFifo(p_adc);
    #else
        ADC_StartSingle(p_adc);
    #endif
    }
}

/*
    todo include batch
*/
/*
    Write Critical Buffer. single threaded access only
    Set active conversions for ADC
    from Analog_T buffers to Analog_ADC_T buffer

    No critical is needed, given:
    In a single thread while ADC is inactive
    In the ADC ISR
*/
// static inline void ADC_FillConversion(Analog_ADC_T * p_adc, const Analog_Conversion_T * p_entry, uint8_t index)
// {
//     p_adc->ActiveConversions[index] = p_entry;
//     p_adc->ActiveConversionCount += 1U;
//     p_entry->P_STATE->IsMarked = false;
// }

static inline void ADC_FillActiveConversions(Analog_ADC_T * p_adc, uint8_t adcId, const Analog_Conversion_T * const * pp_entries, uint8_t length)
{
    Analog_Conversion_T * p_entry;

    p_adc->ActiveConversionCount = 0U; // assert(p_adc->ActiveConversionCount == 0U);
    /*
        Find the next marked channel,
        Continue incrementing. Channels do not repeat until all marked channels have completed once
        Updates ActiveChannelIndex to last index
    */
    while (p_adc->ActiveConversionCount < ADC_FIFO_LENGTH_MAX && p_adc->ActiveChannelIndex < length)
    {
        p_entry = pp_entries[p_adc->ActiveChannelIndex];
        p_adc->ActiveChannelIndex += 1U;
        if (p_entry != NULL)
        {
            // todo remove id for hal pointer. if (p_entry->P_ADC == p_adc && p_entry->P_STATE->IsMarked == true)
            if (p_entry->ADC_ID == adcId && p_entry->P_STATE->IsMarked == true)
            {
                p_adc->ActiveConversions[p_adc->ActiveConversionCount] = p_entry;
                p_adc->ActiveConversionCount += 1U;
                p_entry->P_STATE->IsMarked = false;
            }
        }
    }
}

static inline void ADC_OnComplete(Analog_ADC_T * p_adc, uint8_t adcId, const Analog_Conversion_T * const * pp_entries, uint8_t length)
{
    HAL_ADC_ClearConversionCompleteFlag(p_adc->P_HAL_ADC);

    if (p_adc->ActiveConversionCount > 0U)
    {
        ADC_Capture(p_adc);
        p_adc->ActiveConversionCount = 0U;

        if (p_adc->ActiveChannelIndex < length) /* Continue until all marked are complete */
        {
            ADC_FillActiveConversions(p_adc, adcId, pp_entries, length);
            ADC_Start(p_adc);
        }
        else
        {
            HAL_ADC_Deactivate(p_adc->P_HAL_ADC);
        }

        // optionally proc on complete in parallel
    }
#ifndef NDEBUG
    else
    {
        HAL_ADC_Deactivate(p_adc->P_HAL_ADC);
        p_adc->ErrorCount++;
    }
#endif
}

// void ADC_WriteOptions(Analog_ADC_T * p_adc, const Analog_Options_T * p_options)
// {
//     // xor with previous
//     if(p_options->FLAGS.HwTriggerConversion == 1U)      { HAL_ADC_EnableHwTrigger(p_adc->CONST.P_HAL_ADC); }
//     else                                                { HAL_ADC_DisableHwTrigger(p_adc->CONST.P_HAL_ADC); }
// #ifdef CONFIG_ANALOG_HW_CONTINOUS_CONVERSION_ENABLE
//     if(p_options->FLAGS.ContinuousConversion == 1U)     { HAL_ADC_EnableContinuousConversion(p_adc->CONST.P_HAL_ADC): }
//     else                                                { HAL_ADC_DisableContinuousConversion(p_adc->CONST.P_HAL_ADC); }
// #endif
//     if(p_options->ON_OPTIONS != 0U) { p_options->ON_OPTIONS(p_options->P_CALLBACK_CONTEXT); }
// }


/******************************************************************************/
/*!
    Thread
*/
/******************************************************************************/
/*!
    @brief  Capture ADC results, Process Channel OnComplete Functions.
            Run in corresponding ADC ISR

    ADC ISR should be higher priority than thread calling Analog_StartConversions()
*/
void Analog_OnComplete_ISR(Analog_T * p_analog, uint8_t adcId)
{
    ADC_OnComplete(&p_analog->CONST.P_ADCS[adcId], adcId, p_analog->CONST.PP_CONVERSIONS, p_analog->CONST.CONVERSIONS_COUNT);
}

/*!
    @brief Capture ADC results, by polling status register, if ISR is unavailable
*/
void _Analog_PollComplete(Analog_T * p_analog, uint8_t adcId)
{
    if (HAL_ADC_ReadConversionCompleteFlag(p_analog->CONST.P_ADCS[adcId].P_HAL_ADC) == true) { Analog_OnComplete_ISR(p_analog, adcId); }
}

void Analog_PollComplete(Analog_T * p_analog)
{
    for (uint8_t iAdc = 0U; iAdc < p_analog->CONST.ADC_COUNT; iAdc++)
    {
        _Analog_PollComplete(p_analog, iAdc);
    }
}

// void StartConversions(Analog_T * p_analog, Analog_Conversion_T * p_conversions, uint8_t count)

/*
    No Critical is needed as long as -
        Only a single thread starts the conversions.
    Activate markedChannels
*/
void Analog_StartConversions(Analog_T * p_analog)
{
    Analog_ADC_T * p_adc;
    /* if Adc is still active, all channels will remain marked until the next start call */
    for (uint8_t iAdc = 0U; iAdc < p_analog->CONST.ADC_COUNT; iAdc++)
    {
        p_adc = &p_analog->CONST.P_ADCS[iAdc];
        if (_Analog_ADC_ReadIsActive(p_adc) == false) /* no possibility of interrupt when this function is called by a single thread */
        {
            if (p_adc->ActiveChannelIndex >= p_analog->CONST.CONVERSIONS_COUNT) { p_adc->ActiveChannelIndex = 0U; } /* Only reset to 0U if all channels have been processed */
            ADC_FillActiveConversions(p_adc, iAdc, p_analog->CONST.PP_CONVERSIONS, p_analog->CONST.CONVERSIONS_COUNT);
            ADC_Start(p_adc);
        }
    }

    // if (isComplete == true) { HAL_ADC_Deactivate(p_analog->CONST.P_HAL_ADC); }
}


// void Analog_StartConversionBatch(Analog_T * p_analog, const Analog_ConversionBatch_T * p_batch)
// {

// }

/******************************************************************************/
/*!
    Public
*/
/******************************************************************************/
void Analog_Init(Analog_T * p_analog)
{
    for(uint8_t iAdc = 0U; iAdc < p_analog->CONST.ADC_COUNT; iAdc++)
    {
        HAL_ADC_Init(p_analog->CONST.P_ADCS[iAdc].P_HAL_ADC);
        HAL_ADC_Deactivate(p_analog->CONST.P_ADCS[iAdc].P_HAL_ADC);
    }
}



// static inline void _Analog_CaptureLocalPeak(Analog_T * p_analog, adc_t result)
// adc_t result = HAL_ADC_ReadResult(p_analog->CONST.P_HAL_ADC, p_activeConversion->PIN);
// if(p_analog->ActiveOptions.CaptureLocalPeak == true)
// {
//     if(result > p_activeConversion->P_RESULTS_BUFFER[p_activeConversion->CHANNEL])
//     {
//         p_activeConversion->P_RESULTS_BUFFER[p_activeConversion->CHANNEL] = result;
//     }
//     else
//     {
//         p_analog->IsLocalPeakFound[p_activeConversion->CHANNEL] = true;
//     }
// }
