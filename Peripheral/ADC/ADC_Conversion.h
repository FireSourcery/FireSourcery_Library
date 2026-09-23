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
    @file   ADC_Conversion.h
    @author FireSourcery
    @brief  The application layer interface. What a feature consumes, and how it is requested.
*/
/******************************************************************************/
#include "ADC_Batch.h"
#include "ADC.h"


/******************************************************************************/
/*
    Application handle
    Feature module holds the pointer. ADC owns the state.
    handle requires at least one dereference, either P_ADC or P_CONVERSION_STATE
*/
/******************************************************************************/
typedef const struct ADC_Conversion
{
    const volatile adc_result_t * P_RESULT;
    // volatile adc_mask_t * P_COMPLETION;
    /* for starting. */
    ADC_T * P_ADC;
    adc_channel_t CHANNEL;
    // adc_mask_t CHANNEL_MASK;
}
ADC_Conversion_T;

#define ADC_CONVERSION(AdcArray, AdcId, ChannelIndex) (ADC_Conversion_T) \
        { .P_ADC = &(AdcArray[AdcId]), .CHANNEL = (ChannelIndex), .P_RESULT = &((AdcArray[AdcId]).P_CHANNEL_RESULTS[ChannelIndex])  }

/* From the ADC by name, where the Board holds it as a struct and not an array. P_RESULT resolves on read */
#define ADC_CONVERSION_FROM(AdcStruct, ChannelIndex) (ADC_Conversion_T) { .P_ADC = &(AdcStruct), .CHANNEL = (ChannelIndex), }

static inline void ADC_Conversion_Mark(ADC_Conversion_T * p_conv) { ADC_MarkChannel(p_conv->P_ADC, p_conv->CHANNEL); }
static inline bool ADC_Conversion_IsMarked(ADC_Conversion_T * p_conv) { return ADC_IsMarked(p_conv->P_ADC, p_conv->CHANNEL); }
static inline adc_result_t ADC_Conversion_GetResult(ADC_Conversion_T * p_conv) { return ADC_ResultOf(p_conv->P_ADC, p_conv->CHANNEL); }
static inline void ADC_Conversion_ClearResult(ADC_Conversion_T * p_conv) { p_conv->P_ADC->P_CHANNEL_RESULTS[p_conv->CHANNEL] = 0U; }


static inline void ADC_ConversionMap_Resolve(ADC_Conversion_T ** p_conv, uint8_t count, adc_result_t * p_dest)
{
    for (uint8_t index = 0U; index < count; index++) { p_dest[index] = ADC_Conversion_GetResult(p_conv[index]); }
}



/* Destination. The consumer's index */
/* Source. A slot in any ADC's results buffer */
// typedef const struct ADC_MapEntry { const volatile adc_result_t * P_RESULT; } ADC_MapEntry_T;
// typedef const struct ADC_MapEntry { uint8_t INDEX; const volatile adc_result_t * P_RESULT; } ADC_MapEntry_T;
// typedef const volatile adc_result_t * const ADC_MapEntry_T;
// typedef ADC_MapEntry_T * const ADC_Map_T;


/* Map[ConsumerChannel] -> ADC Source */
/* The index must be one the map names */
/* Per sequence */
/* p_dest[ConsumerChannel] -> consumer buffer */
// static inline void ADC_Map_Resolve(ADC_Map_T map, uint8_t count, adc_result_t * p_dest)
// {
//     for (uint8_t index = 0U; index < count; index++) { p_dest[index] = *map[index]; }
// }

// shared map
// /* p_dest[INDEX] = *P_RESULT, for each entry. Indexes outside the map are left unchanged */
// static inline void ADC_Map_Decode(const ADC_Map_T map, uint8_t count, adc_result_t * p_dest)
// {
//     for (uint8_t index = 0U; index < count; index++) { p_dest[map[index].INDEX] = *map[index].P_RESULT; }
// }


/* Map[source] -> destination */
/* ADC_Channel_T.CAPTURE */
// typedef ADC_Map_T ADC_Demux_T;
// typedef adc_result_t * const ADC_DemuxEntry_T;
// typedef ADC_DemuxEntry_T const ADC_Demux_T[];
// static inline  void ADC_Demux(const ADC_Demux_T map, adc_result_t * p_source, uint8_t count)
// {
//     for (uint8_t index = 0U; index < count; index++) { *map[index] = p_source[index]; }
// }
