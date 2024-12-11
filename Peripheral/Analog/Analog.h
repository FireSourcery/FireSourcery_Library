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

#include "HAL_Analog.h"
#include "Global_Analog.h"
#include "Config.h"

#include "Type/Array/void_array.h"

#include <stdint.h>
#include <stdbool.h>

#ifdef CONFIG_ANALOG_CRITICAL_LIBRARY_ENABLE
#include "System/Critical/Critical.h"
#endif

/* Software side data storage */
#ifdef CONFIG_ANALOG_ADC_RESULT_UINT8
typedef uint8_t analog_result_t;
#elif defined(CONFIG_ANALOG_ADC_RESULT_UINT16)
typedef uint16_t analog_result_t;
#endif

/* ADC Pin Channel - May or may not need 32 bits */
#ifdef CONFIG_ANALOG_ADC_PIN_UINT8
typedef uint8_t analog_pin_t;
#elif defined(CONFIG_ANALOG_ADC_PIN_UINT32)
typedef uint32_t analog_pin_t;
#endif

#if defined(CONFIG_ANALOG_HW_FIFO_ENABLE)
#ifndef ANALOG_FIFO_LENGTH_MAX
#define ANALOG_FIFO_LENGTH_MAX 8U
#endif
#else
#define ANALOG_FIFO_LENGTH_MAX 1U
#endif

typedef uint8_t analog_channel_t; /* Virtual Channel Index */

typedef void (*Analog_Callback_T)(void * p_context);
typedef void (*Analog_Capture_T)(void * p_context, analog_result_t value);

/*
    Conversion 'key'
    Compile time define, layer passing assignment. This way virtualization is not needed.
*/
typedef const struct Analog_Conversion
{
    /* Defined by module */
    const analog_channel_t CHANNEL; /* Id, same as buffer index. Globally unique or partitioned by Analog_ConversionBatch_T */
    // const uint8_t CHANNEL_LOCAL;  /* Results Buffer Index */
    // volatile analog_result_t * const P_RESULTS;  /*!< Persistent ADC results buffer, virtual channel index.  */
    volatile analog_result_t * const P_RESULT;
    // const Analog_Callback_T ON_COMPLETE;    /* On channel complete */
    const Analog_Capture_T CAPTURE;
    void * const P_CONTEXT;

    /* HAL Map */  /* Defined by main */
    // HAL_Analog_T * const P_HAL_ADC;
    const uint8_t ADC_ID; /* required to associate state */
    const analog_pin_t PIN;
}
Analog_Conversion_T;

#define _ANALOG_CONVERSION_INIT_HAL(AdcId, PinId)   \
    .ADC_ID     = AdcId,                            \
    .PIN        = PinId,

#define _ANALOG_CONVERSION_INIT_CALLBACK(p_Context, CaptureFn)  \
    .P_CONTEXT  = p_Context,                                    \
    .CAPTURE    = CaptureFn,

// #define _ANALOG_CONVERSION_INIT_P_RESULT(p_Result, p_Analog) .P_RESULT = (p_Result != NULL) ? p_Result : &(p_Analog->CONST.P_CHANNEL_ENTRIES[p_Analog->CONST.CHANNEL_COUNT].Result),

#define ANALOG_CONVERSION_INIT(Channel, CaptureFn, p_Context, AdcId, PinId, ...) \
{                                                               \
    .CHANNEL    = Channel,                                      \
    _ANALOG_CONVERSION_INIT_HAL(AdcId, PinId)                   \
    _ANALOG_CONVERSION_INIT_CALLBACK(p_Context, CaptureFn)      \
    __VA_ARGS__                                                 \
}

// #define ANALOG_CONVERSION_INIT_STRUCT(...) ( (Analog_Conversion_T) { __VA_ARGS__ } )

typedef const struct
{
    const uint8_t BATCH;
    // const Analog_Conversion_T * const  P_CONVERSIONS;
    const Analog_Conversion_T * const * const PP_CONVERSIONS; /*  Array of pointers */
    const uint8_t CONVERSION_COUNT;
    const Analog_Callback_T COMPLETE; /* On group complete */
    void * const P_CONTEXT;
    volatile analog_result_t * const P_RESULTS_BUFFER;  /*!< Persistent ADC results buffer, virtual channel index.  */
}
Analog_ConversionBatch_T;


typedef struct Analog_ADC
{
    /* ADC_CONST */
    const struct
    {
        HAL_Analog_T * const P_HAL_ANALOG;  /*!< ADC register map base address */
    };

    /* ADC State */
    // volatile Analog_ChannelFlags_T ActiveChannelFlags; /* Dirty bits. */
    const Analog_Conversion_T * ActiveConversions[ADC_FIFO_LENGTH_MAX]; /* Array of pointers */
    volatile analog_channel_t ActiveChannelIndex; /* Index to shared P_CHANNEL_ENTRIES in Analog_T. ActiveChannelFirst */
// #ifdef CONFIG_ANALOG_HW_FIFO_ENABLE
    volatile uint8_t ActiveConversionCount; /*! Hw fifo only. Number of active channels being processed by ADC */
// #endif
#ifndef NDEBUG
    volatile uint8_t ErrorCount;
#endif
}
Analog_ADC_T;

#define ANALOG_ADC_INIT(p_HalAnalog) \
{                                                                   \
    .P_HAL_ANALOG       = p_HalAnalog,                              \
}


/* Conversion State */
typedef struct Analog_Entry
{
    const Analog_Conversion_T * p_Conversion; /* The selected conversion, effectively matching callback to pin. mutable for set callbacks */
    // const Analog_Capture_T Capture;
    volatile analog_result_t Result;
    volatile bool IsMarked;
}
Analog_Entry_T;

typedef struct Analog_BatchEntry
{
    const Analog_ConversionBatch_T * p_ConversionBatch;
    // Capture(p_context, index, value)
    volatile bool IsMarked;
    uint8_t CompleteFlags;
    // uint8_t AdcFlags; check all are disabled
}
Analog_BatchEntry_T;

typedef const struct Analog_Const
{
    Analog_ADC_T * const P_ADCS;
    const uint8_t ADC_COUNT;

    /* ADC common. This way all virtual channel ids are shared. */
    const uint8_t CHANNEL_COUNT;                /*!< also analog_channel_t max + 1 */
    Analog_Entry_T * const P_CHANNEL_ENTRIES;   /* Entries buffer */

    const uint8_t BATCH_COUNT;
    Analog_BatchEntry_T * const P_BATCH_ENTRIES;
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

#define ANALOG_INIT(AdcArray, AdcCount, ChannelArrayBuffer, ChannelCount, BatchArrayBuffer, BatchCount) \
{                                                                   \
    .CONST =                                                        \
    {                                                               \
        .P_ADCS             = AdcArray,                             \
        .ADC_COUNT          = AdcCount,  \
        .P_CHANNEL_ENTRIES  = ChannelArrayBuffer,                       \
        .CHANNEL_COUNT      = ChannelCount,      \
        .P_BATCH_ENTRIES    = BatchArrayBuffer,                                         \
        .BATCH_COUNT        = BatchCount,   \
    },                                                                                  \
}

// INIT_ALLOC
// #define ANALOG_INIT(AdcArray, ChannelCount, BatchCount) \
// {                                                                       \
//     .CONST =                                                            \
//     {                                                                   \
//         .P_ADCS             = AdcArray,                                 \
//         .ADC_COUNT          = (sizeof(AdcArray) / sizeof(Analog_ADC_T)),  \
//         .P_CHANNEL_ENTRIES  = (Analog_Entry_T [(ChannelCount)]){ },       \
//         .CHANNEL_COUNT      = ChannelCount,                             \
//         .P_BATCH_ENTRIES    = (Analog_BatchEntry_T [(BatchCount)]){ },    \
//         .BATCH_COUNT        = BatchCount,                               \
//     },                                                                  \
// }


/******************************************************************************/
/*!

*/
/******************************************************************************/
// static inline bool Analog_ADC_FlagAt(const Analog_ADC_T * p_adc, uint8_t index) { return (p_adc->ActiveChannelFlags.Value & (1U << index)); }

static inline analog_result_t Analog_ResultsOf(Analog_T * p_analog, const Analog_Conversion_T * p_conversion)
{
    return p_analog->CONST.P_CHANNEL_ENTRIES[p_conversion->CHANNEL].Result;
}

/* Interface for void array this way */
static inline bool _Analog_ADC_ReadIsActive(const Analog_ADC_T * p_adc)
{
    /*
        sufficient For lower priority thread check. lower priority thread cannot override ISR update
        HAL_Analog_ReadConversionCompleteFlag will not be set if called from lower priority thread
    */
    return (HAL_Analog_ReadConversionActiveFlag(p_adc->P_HAL_ANALOG) == true);
}

static inline void _Analog_ADC_Deactivate(const Analog_ADC_T * p_adc)
{
    HAL_Analog_Deactivate(p_adc->P_HAL_ANALOG);
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

extern void Analog_OnComplete_ISR(Analog_T * p_analog, uint8_t adcId);
extern void Analog_StartConversions(Analog_T * p_analog);
extern void Analog_MarkConversion(Analog_T * p_analog, const Analog_Conversion_T * p_conversion);
extern void Analog_StartConversionBatch(Analog_T * p_analog, const Analog_ConversionBatch_T * p_group);

extern void Analog_Init(Analog_T * p_analog);


#endif

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
