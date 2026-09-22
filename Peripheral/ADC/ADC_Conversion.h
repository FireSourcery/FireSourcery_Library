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

    The Board layer defines ADC_Channel_T and ADC_Batch_T. The application layer holds conversions
    only. The same calls serve both forms, so a feature does not know which it was given:

        1 channel   - { P_ADC, CHANNEL }. Read directly from that ADC's results.
                      Request marks the channel when software activated, a sequence of 1.
                      When Hw sequenced a lone channel cannot be triggered; it rides the running scan.
        a batch     - { P_BATCH }. 1..N sequences over 1..N ADCs, read from the joined buffer.
                      The batch pushes the set to its consumer on each join, and tracks the parts itself.

        .HEAT_PCB_CONVERSION = { .P_ADC = &ADCS[0U], .CHANNEL = ADC0_HEAT_PCB },
        .PHASE_ANALOG = { .I = { .P_BATCH = &MOTOR0_I_BATCH }, .V = { .P_BATCH = &MOTOR0_V_BATCH } },

    A batch conversion reads in the batch buffer order: parts in declared order, then channel order.
*/
/******************************************************************************/
#include "ADC_Batch.h"
#include "ADC.h"

typedef const struct ADC_Conversion
{
    const ADC_Batch_T * P_BATCH;    /* A set. When set, P_ADC and CHANNEL are unused */
    const ADC_T * P_ADC;            /* Or 1 channel */
    adc_channel_t CHANNEL;
}
ADC_Conversion_T;

static inline bool _ADC_Conversion_IsBatch(const ADC_Conversion_T * p_conv) { return (p_conv->P_BATCH != NULL); }

static inline uint8_t ADC_Conversion_Count(const ADC_Conversion_T * p_conv) { return _ADC_Conversion_IsBatch(p_conv) ? p_conv->P_BATCH->LENGTH : 1U; }

static inline adc_result_t ADC_Conversion_ResultAt(const ADC_Conversion_T * p_conv, uint8_t index)
{
    return _ADC_Conversion_IsBatch(p_conv) ? ADC_Batch_ResultAt(p_conv->P_BATCH, index) : ADC_ResultOf(p_conv->P_ADC, p_conv->CHANNEL);
}

/* The value, for a single channel. The first, for a set */
static inline adc_result_t ADC_Conversion_GetResult(const ADC_Conversion_T * p_conv) { return ADC_Conversion_ResultAt(p_conv, 0U); }

/* The channel descriptor behind a position. e.g. its RESULT_SCALING */
static inline const ADC_Channel_T * ADC_Conversion_ChannelAt(const ADC_Conversion_T * p_conv, uint8_t index)
{
    return _ADC_Conversion_IsBatch(p_conv) ? ADC_Batch_ChannelAt(p_conv->P_BATCH, index) : ADC_ChannelOf(p_conv->P_ADC, p_conv->CHANNEL);
}

/*
    A batch: selects it, and it converts everything in it.
    1 channel: marks it when software activated. No op when Hw sequenced, the scan converts it.
*/
static inline void ADC_Conversion_Request(const ADC_Conversion_T * p_conv)
{
    if (_ADC_Conversion_IsBatch(p_conv)) { ADC_Batch_Select(p_conv->P_BATCH); }
#if !ADC_HW_SEQUENCER_ENABLE
    else { ADC_MarkChannel(p_conv->P_ADC, p_conv->CHANNEL); }
#endif
}

static inline bool ADC_Conversion_IsActive(const ADC_Conversion_T * p_conv) { return (_ADC_Conversion_IsBatch(p_conv) == false) || ADC_Batch_IsActive(p_conv->P_BATCH); }
