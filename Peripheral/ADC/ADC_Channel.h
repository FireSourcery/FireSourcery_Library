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
    @file   ADC_Channel.h
    @author FireSourcery
    @brief  The single channel collaborator. A channel bound to its ADC, with an optional push handler.

    [ADC_Channel_T] names its own ADC, so it is a complete handle: it marks, it reads, and it pushes.
    A table of them indexed by [ID] is the Board's map, and the dispatch is a walk of the channels
    that landed. Gaps in the table are zeroed rows, not NULL registrations.

        static ADC_Channel_T ADC0_CHANNELS[] =
        {
            [CHANNEL_VSOURCE]     = ADC_CHANNEL(&ADCS[0U], CHANNEL_VSOURCE, PIN_VSOURCE, &VBus, VBus_Analog_Capture),
            [CHANNEL_MOTOR0_IA]   = ADC_CHANNEL(&ADCS[0U], CHANNEL_MOTOR0_IA, PIN_IA, &Motors[0U], Motor_Analog_CaptureIa),
        };

        void BOARD_ADC0_ISR(void) { ADC_Channel_OnComplete_ISR(&ADCS[0U], ADC0_CHANNELS); }
*/
/******************************************************************************/
#include "ADC_Thread.h"  /* The per ADC base a channel composes. ADC.h, _ADC.h below it */

typedef const struct ADC_Channel
{
    adc_channel_t ID;           /* Index into ADC.P_CHANNELS, and into the results buffer */
    adc_pin_t PIN;              /* Physical Id of the Pin. Mirrors P_ADC->P_CHANNELS[ID] */
    ADC_Capture_T CAPTURE;      /* Optional. Pushed the result on completion. The buffer holds it either way */
    void * P_CONTEXT;
    ADC_T * P_ADC;
}
ADC_Channel_T;

/* Casts the handler to its registration type. Braces only, so a channel can nest in a table */
#define ADC_CHANNEL_FIELDS(p_Adc, ChannelId, PinId, p_Context, CaptureFn) \
    { .P_ADC = (p_Adc), .ID = (ChannelId), .PIN = (PinId), .CAPTURE = (ADC_Capture_T)(CaptureFn), .P_CONTEXT = (void *)(p_Context), }

#define ADC_CHANNEL(p_Adc, ChannelId, PinId, p_Context, CaptureFn) (ADC_Channel_T)ADC_CHANNEL_FIELDS(p_Adc, ChannelId, PinId, p_Context, CaptureFn)

static inline adc_mask_t ADC_Channel_Mask(ADC_Channel_T * p_channel) { return ADC_MaskOf(p_channel->ID); }

/******************************************************************************/
/*
    Single channel. Software activation
*/
/******************************************************************************/
static inline void ADC_Channel_Mark(ADC_Channel_T * p_channel) { ADC_MarkChannel(p_channel->P_ADC, p_channel->ID); }
static inline bool ADC_Channel_IsMarked(ADC_Channel_T * p_channel) { return ADC_IsMarked(p_channel->P_ADC, p_channel->ID); }

static inline void ADC_Channel_Activate(ADC_Channel_T * p_channel)
{
    assert(p_channel->PIN == ADC_PinOf(p_channel->P_ADC, p_channel->ID)); /* The table and the base's pin map are 1 declaration apart */
    ADC_Channel_Mark(p_channel);
    ADC_ActivateMarked(p_channel->P_ADC);
}

static inline adc_result_t ADC_Channel_Result(ADC_Channel_T * p_channel) { return ADC_ResultOf(p_channel->P_ADC, p_channel->ID); }

/* Landed. Consumes, so it is an edge */
static inline bool ADC_Channel_TakeComplete(ADC_Channel_T * p_channel) { return ADC_TakeComplete(p_channel->P_ADC, ADC_Channel_Mask(p_channel)); }

/******************************************************************************/
/*
    Push
    The results buffer is the canonical store, so CAPTURE is in addition, not instead.
    A pull read then sees the same value the push delivered.
*/
/******************************************************************************/
static inline void ADC_Channel_Capture(ADC_Channel_T * p_channel)
{
    if (p_channel->CAPTURE != NULL) { p_channel->CAPTURE(p_channel->P_CONTEXT, ADC_Channel_Result(p_channel)); }
}

/* [Channel] table. Pushes each channel of [landed] to its handler, in channel order */
static inline void ADC_Channel_CaptureEach(ADC_Channel_T * p_channels, adc_mask_t landed)
{
    for (; landed != 0UL; landed &= (landed - 1UL)) { ADC_Channel_Capture(&p_channels[__builtin_ctz(landed)]); }
}

/*!
    @brief  The base captures into the buffer, this pushes what landed.
            Run in the ADC ISR, in place of ADC_OnComplete_ISR.

    Takes every flag, so 1 dispatcher owns the ADC. A Board that also reads flags of its own
    names its set with ADC_TakeComplete first, or calls ADC_Channel_CaptureEach itself.
*/
static inline void ADC_Channel_OnComplete_ISR(ADC_T * p_adc, ADC_Channel_T * p_channels)
{
    ADC_OnComplete_ISR(p_adc);
    ADC_Channel_CaptureEach(p_channels, ADC_TakeFlags(p_adc, ADC_MASK_ALL));
}
