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
static inline analog_result_t CaptureConversionResult(const Analog_ADC_T * p_adc, const Analog_Conversion_T * p_conversion)
{
    analog_result_t result = HAL_Analog_ReadResult(p_adc->P_HAL_ANALOG, p_conversion->PIN);
    /* Capture may incurr additional interrupt time, while eliminating double buffer */
    if (p_conversion->ON_COMPLETE != NULL) { p_conversion->ON_COMPLETE(p_conversion->P_CONTEXT, result); }
    return result;
}

static inline void CaptureActiveConversion(Analog_ADC_T * p_adc, const Analog_Conversion_T * p_conversion)
{
    p_conversion->P_STATE->Result = CaptureConversionResult(p_adc, p_conversion); /* alternatively disable when ON_COMPLETE is defined */
    p_conversion->P_STATE->IsMarked = false;
}

/*
    ADC_ Context is split between ADC and Analog containing Analog_ConversionState_T
*/
static inline void ADC_CaptureChannel(Analog_ADC_T * p_adc)
{
    CaptureActiveConversion(p_adc, p_adc->ActiveConversions[0U]);
}

static inline void ADC_CaptureFifo(Analog_ADC_T * p_adc)
{
    Analog_ConversionState_T * p_entry;
    if (p_adc->ActiveConversionCount == HAL_Analog_ReadFifoCount(p_adc->P_HAL_ANALOG))
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
    Activate and wait for return
*/
static inline void ADC_StartChannel(Analog_ADC_T * p_adc)
{
    if (p_adc->ActiveConversionCount > 0U)
    {
        HAL_Analog_Activate(p_adc->P_HAL_ANALOG, p_adc->ActiveConversions[0U]->PIN);
    }
}
/*
*/
static inline void ADC_StartFifo(Analog_ADC_T * p_adc)
{
    Analog_Conversion_T * p_conversion;

    HAL_Analog_WriteFifoCount(p_adc->P_HAL_ANALOG, p_adc->ActiveConversionCount);

    // for (uint8_t iConversion = 0U; iConversion < p_adc->ActiveConversionCount; iConversion++) /* if active separate from last pin */
    for (uint8_t iConversion = 0U; iConversion < p_adc->ActiveConversionCount - 1U; iConversion++)
    {
        p_conversion = p_adc->ActiveConversions[iConversion];
        HAL_Analog_WriteFifoPin(p_adc->P_HAL_ANALOG, p_conversion->PIN);
    }

    p_conversion = p_adc->ActiveConversions[p_adc->ActiveConversionCount];
    HAL_Analog_ActivateFifo(p_adc->P_HAL_ANALOG, p_conversion->PIN);
}

/*
    Set active conversions for ADC
    from Analog_T buffers to Analog_ADC_T buffer
*/
static inline void ADC_FillActiveConversions(Analog_ADC_T * p_adc, uint8_t adcId, const Analog_Conversion_T * const * pp_entries, uint8_t length)
{
    Analog_Conversion_T * p_entry;

    p_adc->ActiveConversionCount = 0U;
    /*
        Find the next marked channel,
        Continue incrementing. Channels do not repeat until all marked channels have completed once
        Updates ActiveChannelIndex to last index
    */
    while (p_adc->ActiveConversionCount < ADC_FIFO_LENGTH_MAX && p_adc->ActiveChannelIndex < length)
    {
        p_entry = pp_entries[p_adc->ActiveChannelIndex];
        p_adc->ActiveChannelIndex += 1U;
        if (p_entry->ADC_ID == adcId && p_entry->P_STATE->IsMarked == true)
        {
            p_adc->ActiveConversions[p_adc->ActiveConversionCount] = p_entry;
            p_adc->ActiveConversionCount += 1U;
        }
    }
}

static inline void ADC_OnComplete(Analog_ADC_T * p_adc, uint8_t adcId, const Analog_Conversion_T * const * pp_entries, uint8_t length)
{
    if (HAL_Analog_ReadConversionCompleteFlag(p_adc->P_HAL_ANALOG) == true)
    {
        HAL_Analog_ClearConversionCompleteFlag(p_adc->P_HAL_ANALOG);
        /* includes handling mark entries */
    #ifdef CONFIG_ANALOG_HW_FIFO_ENABLE
        ADC_CaptureFifo(p_adc, p_entries);
    #else
        ADC_CaptureChannel(p_adc);
    #endif

        if (p_adc->ActiveChannelIndex < length)
        {
            ADC_FillActiveConversions(p_adc, adcId, pp_entries, length);
        #ifdef CONFIG_ANALOG_HW_FIFO_ENABLE
            ADC_StartFifo(p_adc);
        #else
            ADC_StartChannel(p_adc);
        #endif
        }

        // optionally proc on complete in parallel
    }
#ifndef NDEBUG
    else { p_adc->ErrorCount++; }
#endif
}

// void ADC_WriteOptions(Analog_ADC_T * p_adc, const Analog_Options_T * p_options)
// {
//     // xor with previous
//     if(p_options->FLAGS.HwTriggerConversion == 1U)      { HAL_Analog_EnableHwTrigger(p_adc->CONST.P_HAL_ANALOG); }
//     else                                                { HAL_Analog_DisableHwTrigger(p_adc->CONST.P_HAL_ANALOG); }
// #ifdef CONFIG_ANALOG_HW_CONTINOUS_CONVERSION_ENABLE
//     if(p_options->FLAGS.ContinuousConversion == 1U)     { HAL_Analog_EnableContinuousConversion(p_adc->CONST.P_HAL_ANALOG): }
//     else                                                { HAL_Analog_DisableContinuousConversion(p_adc->CONST.P_HAL_ANALOG); }
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
    if (HAL_Analog_ReadConversionCompleteFlag(p_analog->CONST.P_ADCS[adcId].P_HAL_ANALOG) == true) { Analog_OnComplete_ISR(p_analog, adcId); }
}

void Analog_PollComplete(Analog_T * p_analog)
{
    for (uint8_t iAdc = 0U; iAdc < p_analog->CONST.ADC_COUNT; iAdc++)
    {
        _Analog_PollComplete(p_analog, iAdc);
    }
}

/*
    No Critical is needed as long as -
        Only a single thread starts the conversions.
    Activate markedChannels
*/
void Analog_StartConversions(Analog_T * p_analog)
// void Analog_StartConversions(Analog_T * p_analog, Analog_Conversion_T * p_conversions, uint8_t count)
{
    Analog_ADC_T * p_adc;
    /* if Adc is still active, all channels will remain marked until the next start call */
    for (uint8_t iAdc = 0U; iAdc < p_analog->CONST.ADC_COUNT; iAdc++)
    {
        p_adc = &p_analog->CONST.P_ADCS[iAdc];
        if (_Analog_ADC_ReadIsActive(p_adc) == false) /* no possibility of interrupt when this function is called by a single thread */
        {
            p_adc->ActiveChannelIndex = 0U;
            ADC_FillActiveConversions(p_adc, iAdc, p_analog->CONST.PP_CONVERSIONS , p_analog->CONST.CONVERSIONS_COUNT);
        #ifdef CONFIG_ANALOG_HW_FIFO_ENABLE
            ADC_StartFifo(p_adc);
        #else
            ADC_StartChannel(p_adc);
        #endif
        }
    }

    // if (isComplete == true) { HAL_Analog_Deactivate(p_analog->CONST.P_HAL_ANALOG); }
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
        HAL_Analog_Init(p_analog->CONST.P_ADCS[iAdc].P_HAL_ANALOG);
        HAL_Analog_Deactivate(p_analog->CONST.P_ADCS[iAdc].P_HAL_ANALOG);
    }
}

/******************************************************************************/
/*!
    Channel Fags
*/
/******************************************************************************/
/*
    Add Conversion by marking its flag
    Analog_Conversion_T provides ADC select abstraction
*/
void Analog_MarkConversion(Analog_T * p_analog, const Analog_Conversion_T * p_conversion)
{
    p_conversion->P_STATE->IsMarked = true;
}

// void Analog_MarkConversionBatch(Analog_T * p_analog, const Analog_ConversionBatch_T * p_conversion)
// {
//     p_analog->CONST.P_BATCH_ENTRIES[p_conversion->BATCH].IsMarked = true;
// }


// static inline void _Analog_CaptureLocalPeak(Analog_T * p_analog, analog_result_t result)
// analog_result_t result = HAL_Analog_ReadResult(p_analog->CONST.P_HAL_ANALOG, p_activeConversion->PIN);
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
