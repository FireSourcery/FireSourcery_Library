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
    @file   Analog_ADC.h
    @author FireSourcery
    @brief
*/
/******************************************************************************/
#include "Analog.h"
#include "HAL_ADC.h"
#include "Config.h"

#include <stdint.h>
#include <stdbool.h>


#if defined(CONFIG_ANALOG_ADC_HW_FIFO_ENABLE)
#define ADC_FIFO_LENGTH_MAX HAL_ADC_FIFO_LENGTH_MAX
#else
#define ADC_FIFO_LENGTH_MAX 1U
#endif

/******************************************************************************/
/*
    ADC State
    Critical section buffer shared by ISR and StartConversions.
*/
/******************************************************************************/
typedef struct Analog_ADC_State
{
    analog_channel_t ChannelIndex; /* Index to marked Conversions */

    /*
        Fifo State
        maintained by software in case of fifo where direct id map is not available.
    */
    const Analog_ConversionChannel_T * ActiveConversions[ADC_FIFO_LENGTH_MAX]; /* Array of pointers */
    uint8_t ActiveConversionCount; /* Hw fifo only. Number of active channels being processed by ADC */

    /* Seperate execution from Per channel operation */
    // const Analog_ConversionBatch_T * ActiveConversionBatch;
    // int ActiveType; /* Single, Batch */
    // uint8_t BatchId; /* 255 for null /

// #ifdef CONFIG_ANALOG_ADC_HW_FIFO_ENABLE
//     Analog_ADC_FifoState_T FifoState; /* State of the ADC FIFO */
// #else
//     const Analog_ConversionChannel_T * p_ActiveConversion;
// #endif
#ifndef NDEBUG
    uint8_t ErrorCount;
#endif
}
Analog_ADC_State_T;

/******************************************************************************/
/*
    ADC Context - Context Per Thread
*/
/******************************************************************************/
typedef const struct Analog_ADC
{
    HAL_ADC_T * P_HAL_ADC;  /* ADC register map base address */
    analog_channel_t CHANNEL_COUNT; /* Number of channels in the ADC */
                                    /* allow repeat pin with different compile time set callback */

    Analog_ADC_State_T * P_ADC_STATE; /* State data not retained by registers */

    /*
        A writable buffer, marked channels, without critical section
        each possible channel id allocated.
        ADCs can be started independent of global state.

        associate with ADC, at a single point of reference, utilize memory locality
        direct iteration for marked
        using a buffer outside of ADC_State allows for the ADC state to remain unchanged through the entire conversion process.

        In this case, ADC Structs must be defined for each Board HAL.
    */
    const Analog_ConversionChannel_T * P_CONVERSION_CHANNELS; /* compile time associates callback */
    volatile Analog_ConversionState_T * P_CONVERSION_STATES;

    // const Analog_BatchPart_T * const * const P_BATCH_PARTS;
    /* alternatively results can be passed seperately */
    // volatile bool * const P_MARKERS;
    // volatile adc_result_t * const P_RESULTS;
}
Analog_ADC_T;

#define ANALOG_ADC_INIT(p_HalAnalog, ChannelCount, p_ConvChannels, p_ConvStates, p_AdcState) \
    { .P_HAL_ADC = p_HalAnalog, .CHANNEL_COUNT = ChannelCount, .P_CONVERSION_CHANNELS = p_ConvChannels, .P_CONVERSION_STATES = p_ConvStates, .P_ADC_STATE = p_AdcState, }

#define ANALOG_ADC_ALLOC(p_HalAnalog, ChannelCount, p_ConvChannels) \
    ANALOG_ADC_INIT(p_HalAnalog, ChannelCount, p_ConvChannels, (Analog_ConversionState_T[ChannelCount]){0}, &(Analog_ADC_State_T){0})


/******************************************************************************/
/*!

*/
/******************************************************************************/
static inline void _ADC_CaptureChannel(const Analog_ADC_T * p_adc, analog_channel_t channel, adc_result_t result) { p_adc->P_CONVERSION_STATES[channel].State = (uint32_t)result; }
static inline adc_result_t Analog_ADC_ResultOf(const Analog_ADC_T * p_adc, analog_channel_t channel) { return p_adc->P_CONVERSION_STATES[channel].Result; }
static inline void Analog_ADC_MarkConversion(const Analog_ADC_T * p_adc, analog_channel_t channel) { p_adc->P_CONVERSION_STATES[channel].IsMarked = true; }
static inline bool Analog_ADC_IsMarked(const Analog_ADC_T * p_adc, analog_channel_t channel) { return p_adc->P_CONVERSION_STATES[channel].IsMarked; }

/******************************************************************************/
/*! */
/******************************************************************************/
static inline bool Analog_ADC_ReadIsActive(const Analog_ADC_T * p_adc)
{
    /*
        sufficient For lower priority thread check. lower priority thread cannot override ISR update
        HAL_ADC_ReadConversionCompleteFlag will not be set if called from lower priority thread
    */
    return (HAL_ADC_ReadConversionActiveFlag(p_adc->P_HAL_ADC) == true);
}

static inline void Analog_ADC_Deactivate(const Analog_ADC_T * p_adc)
{
    HAL_ADC_Deactivate(p_adc->P_HAL_ADC);
}


/******************************************************************************/
/*!
    extern
*/
/******************************************************************************/
extern void Analog_ADC_Init(const Analog_ADC_T * p_adc);

