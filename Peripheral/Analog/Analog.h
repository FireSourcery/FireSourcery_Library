
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
#include "Analog_ADC.h"
#include "_Analog_ADC.h"    /* Analog_ADC_ActivateBatch resolves the Hw activation */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/******************************************************************************/
/*
    Application handle
    Feature module holds the pointer. ADC owns the state.
    handle requires at least one dereference, either P_ADC or P_CONVERSION_STATE
*/
/******************************************************************************/
/* Analog_VirtualChannel */
typedef const struct Analog_Conversion
{
    Analog_ADC_T * P_ADC;
    analog_channel_t CHANNEL;
    // analog_mask_t
    // Analog_Capture_T CAPTURE;
    // void * P_CONTEXT;
    /* reserve interface for extension */
    // Analog_Options_T OPTIONS;
}
Analog_Conversion_T;


static inline void Analog_Conversion_Mark(const Analog_Conversion_T * p_conv) { Analog_ADC_MarkConversion(p_conv->P_ADC, p_conv->CHANNEL); }
static inline bool Analog_Conversion_IsMarked(const Analog_Conversion_T * p_conv) { return Analog_ADC_IsMarked(p_conv->P_ADC, p_conv->CHANNEL); }

/* resolve at runtime */
static inline Analog_ConversionChannel_T * Analog_Conversion_ChannelOf(const Analog_Conversion_T * p_conv) { return Analog_ADC_ConversionOf(p_conv->P_ADC, p_conv->CHANNEL); }

static inline adc_result_t Analog_Conversion_GetResult(const Analog_Conversion_T * p_conv) { return Analog_Conversion_ChannelOf(p_conv)->P_CONVERSION_STATE->Result; }
static inline void Analog_Conversion_ClearResult(const Analog_Conversion_T * p_conv) { Analog_Conversion_ChannelOf(p_conv)->P_CONVERSION_STATE->Result = 0U; }

// uniform handling for dma
// static inline adc_result_t Analog_Conversion_GetResult(const Analog_Conversion_T * p_conv) { return Analog_ADC_ChannelResultOf(p_conv->P_ADC, p_conv->CHANNEL); }

/******************************************************************************/
/*
    Unsynchronize Activation
*/
/******************************************************************************/
// static void Analog_Conversion_Activate(const Analog_Conversion_T * p_conversion)
// {
//     _Analog_ADC_StartConversion(p_conversion->P_ADC, p_conversion->P_CONVERSION_CHANNEL, 1U);
// }

// static void Analog_ActivateConversions(const Analog_Conversion_T * p_conversions, uint32_t markers)
// {
//     // if (Analog_ADC_ReadIsActive(p_conversion->P_ADC) == false)
    // {
    //     Analog_ADC_StartConversions(p_conversion->P_ADC, p_conversion->P_CONVERSION_CHANNEL, markers);
    // }
    // else
    // {
    //     Analog_Conversion_Mark(p_conversion); // mark each
    // }
// }

