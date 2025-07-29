
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
    @file   Analog.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "HAL_ADC.h"
#include "Config.h"

#include <stdint.h>
#include <stdbool.h>


/******************************************************************************/
/*
    Module Common Defs
    Organized by unit of execution
*/
/******************************************************************************/
/******************************************************************************/
/*
    Channel
    Board HAL compile time define.
*/
/******************************************************************************/
typedef uint8_t analog_channel_t; /* Virtual Channel Index. resolve to Analog_Conversion_T */

typedef const struct Analog_Channel
{
    analog_channel_t ID;  /* Virtual Channel Index. Index into ADC.P_CHANNELS */
    adc_pin_t PIN;        /* Physical Id of the Pin */
    // const uint32_t ID_MASK;
}
Analog_Channel_T;

#define ANALOG_CHANNEL_INIT(Id, PinId) { .ID = Id, .PIN = PinId, }

/******************************************************************************/
/*
*/
/******************************************************************************/
typedef void (*Analog_Callback_T)(void * p_context);
typedef void (*Analog_Capture_T)(void * p_context, adc_result_t value);

/*

*/
typedef union Analog_ConversionState
{
    struct
    {
        uint32_t Result : 16U;
        uint32_t IsMarked : 1U;
        uint32_t Reserved : 15U;
        // uint32_t IsNewResult : 1U; // new result sync flag
        // volatile bool IsActive; // allow mark while active /* !IsComplete */
    };
    uint32_t State; /* setting State effectively clears IsMarked */
}
Analog_ConversionState_T;


/******************************************************************************/
/*
    Conversion Channel/Unit
    holds ADC_OnComplete context per channel
    granular unit of execution for ADC
*/
/******************************************************************************/
typedef const struct Analog_ConversionChannel
{
    Analog_Channel_T CHANNEL; /* Small enough to copy/store by value */
    Analog_Capture_T CAPTURE; /* Overwrite capture to ADC Buffer */
    void * P_CONTEXT;
    volatile Analog_ConversionState_T * P_CONVERSION_STATE; /* each channal allocates it own buffer. directly use by oncomplete */
    // Analog_ADC_T * P_ADC_STATE;
}
Analog_ConversionChannel_T;

#define ANALOG_CONVERSION_STATE_ALLOC() (&(Analog_ConversionState_T){0})
#define ANALOG_CONVERSION_CHANNEL_INIT_FROM(ChannelId, PinId, p_Context, CaptureFn, ...) \
    { .CHANNEL = { .ID = ChannelId, .PIN = PinId }, .CAPTURE = (Analog_Capture_T)CaptureFn, .P_CONTEXT = p_Context, .P_CONVERSION_STATE = ANALOG_CONVERSION_STATE_ALLOC(), }


/******************************************************************************/
/*
    Export
*/
/******************************************************************************/
#include "Analog_ADC.h"
#include "Analog_Conversion.h"



// typedef struct Analog_OptionsFlags
// {
//     uint32_t HwTriggerConversion      : 1U;
//     uint32_t ContinuousConversion     : 1U;
//     uint32_t CaptureLocalPeak         : 1U; /* for now, conversion stops on 1 local peak in channel set, user must also set ContinuousConversion */
//     uint32_t HwAveraging              : 1U;
//     uint32_t HwTriggerChannel         : 1U; /* Per Hw buffer complete. Per Channel if both are set*/
//     uint32_t Interrupt                : 1U;
//     uint32_t Dma                      : 1U;
//     uint8_t Priority
// }
// Analog_OptionsFlags_T;


/******************************************************************************/
/*
    Channel
    Board HAL compile time define.
    Implementation as mapping ADC Host to ADC Channel.
    This way ADC_T does not need to be defined per Board HAL.
    stuctured by caller source, alternatively split by threading
*/
/******************************************************************************/
// #define ANALOG_CHANNEL_INIT(Channel, p_AnalogAdc, PinId) { .ID = Channel, .P_ADC = p_AnalogAdc, .PIN = PinId, }