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
    @file   Analog.h
    @author FireSourcery
    @brief     ADC wrapper module. Implements run time configurable settings.
    @version V0
*/
/******************************************************************************/
#ifndef ANALOG_H
#define ANALOG_H

#include "HAL_ADC.h"
#include "Global_Analog.h"
#include "Config.h"

#include "Type/Array/void_array.h"

#include <stdint.h>
#include <stdbool.h>


#if defined(CONFIG_ANALOG_ADC_HW_FIFO_ENABLE)
#define ADC_FIFO_LENGTH_MAX HAL_ADC_FIFO_LENGTH_MAX
#else
#define ADC_FIFO_LENGTH_MAX 1U
#endif

typedef uint8_t analog_channel_t; /* Virtual Channel Index */

typedef void (*Analog_Callback_T)(void * p_context);
typedef void (*Analog_Setter_T)(void * p_context, adc_t value);
// typedef void (*Analog_Callback1_T)(Analog_Conversion_T * p_conversion, adc_t value);
// typedef void (*Analog_SetBatch_T)(void * p_context, uint8_t batchIndex, adc_t value);

typedef struct Analog_ConversionState
{
    volatile adc_t Result;
    volatile bool IsMarked;
}
Analog_ConversionState_T;

/*
    Conversion 'key'
    Compile time define, layer passing assignment. This way virtualization is not needed.
*/
typedef const struct Analog_Conversion
{
    /*
        Id, to associate Analog_T state, by index. Globally unique if buffer is global.
        Not needed if P_STATE is provided.
    */
    const analog_channel_t CHANNEL;

    /* HAL Map */
    const uint8_t ADC_ID; /* required to associate state from Analog_T */
    const adc_pin_t PIN;

    void * const P_CONTEXT;
    const Analog_Setter_T ON_COMPLETE; /* On complete, always runs if set */
    volatile Analog_ConversionState_T * const P_STATE;
}
Analog_Conversion_T;

#define _ANALOG_CONVERSION_INIT_HAL(AdcId, PinId)   \
    .ADC_ID     = AdcId,                            \
    .PIN        = PinId,

#define _ANALOG_CONVERSION_INIT_CALLBACK(p_Context, CaptureFn)  \
    .P_CONTEXT      = p_Context,                                \
    .ON_COMPLETE    = CaptureFn,

// #define ANALOG_CONVERSION_INIT(Channel, AdcId, PinId, p_Context, CaptureFn, ...)
#define ANALOG_CONVERSION_INIT(Channel, CaptureFn, p_Context, AdcId, PinId, ...) \
{                                                               \
    .CHANNEL    = Channel,                                      \
    _ANALOG_CONVERSION_INIT_HAL(AdcId, PinId)                   \
    _ANALOG_CONVERSION_INIT_CALLBACK(p_Context, CaptureFn)      \
    .P_STATE    =  &(Analog_ConversionState_T){},               \
}

// #define ANALOG_CONVERSION_INIT_STRUCT(...) ( (Analog_Conversion_T) { __VA_ARGS__ } )


/*
    To indicate marked channels without critical section

    Each ADC maintains its own state, ActiveChannelIndex, so ADCs can be started independent of global state.

    A writable buffer without critical section requires each possible channel id to be allocated.
    To map with global state, either:
        each channel id is globally unique, virtual global map
        ADC local channel id, defined by _Board_ per adc map, reverse pin map
*/
typedef struct Analog_ADC
{
    /* ADC_CONST */
    const struct
    {
        HAL_ADC_T * const P_HAL_ADC;  /*!< ADC register map base address */

        // alternatively point to shared channel table, or compile time define per ADC
        // const Analog_Conversion_T * const * const PP_CONVERSIONS; /* Array of pointers */
        // const uint8_t CONVERSIONS_COUNT;
    };

    /*
        ADC State
        Shared by ISR and StartConversions.
    */
    // const Analog_Conversion_T * pp_ConversionsList;
    volatile analog_channel_t ActiveChannelIndex; /* Index to shared PP_CONVERSIONS in Analog_T. */

    const Analog_Conversion_T * ActiveConversions[ADC_FIFO_LENGTH_MAX]; /* Critical section buffer. Access by StartConversions and ISR. Array of pointers */
// #ifdef CONFIG_ANALOG_ADC_HW_FIFO_ENABLE
    volatile uint8_t ActiveConversionCount; /*! Hw fifo only. Number of active channels being processed by ADC */
// #endif
#ifndef NDEBUG
    volatile uint8_t ErrorCount;
#endif
}
Analog_ADC_T;

#define ANALOG_ADC_INIT(p_HalAnalog)    \
{                                       \
    .P_HAL_ADC       = p_HalAnalog,     \
}

typedef const struct Analog_Const
{
    Analog_ADC_T * const P_ADCS;
    const uint8_t ADC_COUNT;

    // List of all conversions. Can be iterated to ensure all conversions run once before starting again.
    const Analog_Conversion_T * const * const PP_CONVERSIONS; /* Array of pointers */
    const uint8_t CONVERSIONS_COUNT;                /*!< also analog_channel_t end */

    // const uint8_t BATCH_COUNT;
    // Analog_BatchEntry_T * const P_BATCH_ENTRIES;
}
Analog_Const_T;

/*
    Per Analog Context Space,
*/
typedef struct Analog
{
    const Analog_Const_T CONST;
    volatile uint8_t ActiveBatchIndex;
}
Analog_T;

// #define ANALOG_INIT_CHANNELS(AdcArray, AdcCount, ChannelArrayBuffer, ChannelCount, BatchArrayBuffer, BatchCount) \

#define ANALOG_INIT(AdcArray, AdcCount, pp_ChannelTable, ChannelCount) \
{                                                                           \
    .CONST =                                                                \
    {                                                                       \
        .P_ADCS                     = AdcArray,                             \
        .ADC_COUNT                  = AdcCount,                             \
        .PP_CONVERSIONS             = pp_ChannelTable,                      \
        .CONVERSIONS_COUNT          = ChannelCount                          \
    }, \
}

// #define ANALOG_INIT(AdcArray, p_ChannelArray) ANALOG_INIT(AdcArray, sizeof(AdcArray)/sizeof(Analog_ADC_T), p_ChannelArray, sizeof(p_ChannelArray)/sizeof(Analog_Conversion_T *))

/******************************************************************************/
/*!

*/
/******************************************************************************/

/* Interface for void array this way */
static inline bool _Analog_ADC_ReadIsActive(const Analog_ADC_T * p_adc)
{
    /*
        sufficient For lower priority thread check. lower priority thread cannot override ISR update
        HAL_ADC_ReadConversionCompleteFlag will not be set if called from lower priority thread
    */
    return (HAL_ADC_ReadConversionActiveFlag(p_adc->P_HAL_ADC) == true);
}

static inline void _Analog_ADC_Deactivate(const Analog_ADC_T * p_adc)
{
    HAL_ADC_Deactivate(p_adc->P_HAL_ADC);
}

/*

*/
static inline bool Analog_ReadIsActive(const Analog_T * p_analog)
{
    return void_array_is_any(p_analog->CONST.P_ADCS, sizeof(Analog_ADC_T), p_analog->CONST.ADC_COUNT, (void_test_t)_Analog_ADC_ReadIsActive);
}

/*
    Needed to overwrite continuous conversion
*/
static inline void Analog_Deactivate(const Analog_T * p_analog)
{
    void_array_foreach(p_analog->CONST.P_ADCS, sizeof(Analog_ADC_T), p_analog->CONST.ADC_COUNT, (void_op_t)_Analog_ADC_Deactivate);
}

/******************************************************************************/
/*!

*/
/******************************************************************************/
static inline adc_t Analog_ResultOf(const Analog_Conversion_T * p_conversion) { return p_conversion->P_STATE->Result; }
static inline adc_t Analog_ResultOfChannel(const Analog_T * p_analog, analog_channel_t channel) { return Analog_ResultOf(p_analog->CONST.PP_CONVERSIONS[channel]); }


/*
    Add Conversion by marking its flag
    Analog_Conversion_T provides ADC select abstraction
*/
static inline void Analog_MarkConversion(const Analog_Conversion_T * p_conversion) { p_conversion->P_STATE->IsMarked = true; }
static inline void Analog_MarkChannel(const Analog_T * p_analog, analog_channel_t channel) { Analog_MarkConversion(p_analog->CONST.PP_CONVERSIONS[channel]); }

/******************************************************************************/
/*!
    extern
*/
/******************************************************************************/
extern void Analog_OnComplete_ISR(Analog_T * p_analog, uint8_t adcId);
extern void Analog_StartConversions(Analog_T * p_analog);
extern void Analog_Init(Analog_T * p_analog);

#endif

// typedef const struct
// {
//     const uint8_t BATCH;
//     // const Analog_Conversion_T * const  P_CONVERSIONS;
//     const Analog_Conversion_T * const * const PP_CONVERSIONS; /*  Array of pointers */
//     const uint8_t CONVERSION_COUNT;
//     const Analog_Callback_T COMPLETE; /* On group complete */
//     void * const P_CONTEXT;
//     volatile adc_t * const P_RESULTS_BUFFER;  /*!< Persistent ADC results buffer, virtual channel index.  */
// }
// Analog_ConversionBatch_T;
// void Analog_MarkConversionBatch(Analog_T * p_analog, const Analog_ConversionBatch_T * p_conversion)
// {
//     p_analog->CONST.P_BATCH_ENTRIES[p_conversion->BATCH].IsMarked = true;
// }
// extern void Analog_StartConversionBatch(Analog_T * p_analog, const Analog_ConversionBatch_T * p_group);

// static inline bool Analog_ADC_FlagAt(const Analog_ADC_T * p_adc, uint8_t index) { return (p_adc->ActiveChannelFlags.Value & (1U << index)); }
// typedef union Analog_ChannelFlags
// {
//     struct
//     {
//         uint32_t Channel0 : 1U;
//         uint32_t Channel1 : 1U;
//         uint32_t Channel2 : 1U;
//         uint32_t Channel3 : 1U;
//         uint32_t Channel4 : 1U;
//         uint32_t Channel5 : 1U;
//         uint32_t Channel6 : 1U;
//         uint32_t Channel7 : 1U;
//         // uint32_t Channel8 : 1U;
//         // uint32_t Channel9 : 1U;
//         // uint32_t Channel10 : 1U;
//         // uint32_t Channel11 : 1U;
//         // uint32_t Channel12 : 1U;
//         // uint32_t Channel13 : 1U;
//         // uint32_t Channel14 : 1U;
//         // uint32_t Channel15 : 1U;
//     };
//     uint32_t Value;
// }
// Analog_ChannelFlags_T;


// typedef union Analog_AdcFlags
// {
//     struct
//     {
//         uint32_t Adc0 : 1U;
//         uint32_t Adc1 : 1U;
//         uint32_t Adc2 : 1U;
//         uint32_t Adc3 : 1U;
//         uint32_t Adc4 : 1U;
//         uint32_t Adc5 : 1U;
//         uint32_t Adc6 : 1U;
//         uint32_t Adc7 : 1U;
//     };
//     uint8_t Flags;
// }
// Analog_AdcFlags_T;
