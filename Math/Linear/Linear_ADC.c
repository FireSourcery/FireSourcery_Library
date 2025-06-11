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
    @file   Linear_ADC.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
/******************************************************************************/
#include "Linear_ADC.h"

// typedef struct Linear_ADC_Config
// {
//     uint16_t AdcuZero;
//     uint16_t AdcuBits;
//     bool IsInverted;
// }
// Linear_ADC_Config_T;

// /* Linear_ADC_Init_Scalar(p_linear, &(Linear_ADC_Config_T){ }); */

// void Linear_ADC_InitFrom(Linear_T * p_linear, Linear_ADC_Config_T * p_config)
// {
//     p_linear->Slope = p_config->IsInverted ? -1 : 1;
//     p_linear->SlopeShift = 15U - p_config->AdcuBits; // Adjust the shift to match the fractional bits
//     p_linear->InvSlope = p_linear->Slope;
//     p_linear->InvSlopeShift = p_linear->SlopeShift;
//     p_linear->X0 = p_config->AdcuZero;
//     // unused
//     p_linear->XDelta = (1U << p_config->AdcuBits);
//     p_linear->XReference = p_config->AdcuZero + p_linear->XDelta;
//     // p_linear->Y0 = 0;
//     // p_linear->YDelta = 65536;
// }


/******************************************************************************/
/*!
    f([adcuZero - (2^adcBits - adcuZero)):adcuZero + (2^adcBits - adcuZero)]) = [-32768:32768]
    multiply +/- 1 and shift
    alternatively as const
*/
/******************************************************************************/
void Linear_ADC_Init_Fract16(Linear_T * p_linear, uint16_t adcuZero, uint8_t adcuBits, bool isInverted)
{
    p_linear->Slope = isInverted ? -1 : 1;
    p_linear->SlopeShift = 15U - adcuBits; // Adjust the shift to match the fractional bits
    p_linear->InvSlope = p_linear->Slope;
    p_linear->InvSlopeShift = p_linear->SlopeShift;
    p_linear->X0 = adcuZero;
    // ref only
    p_linear->XDelta = (1U << adcuBits) - adcuZero;
    p_linear->XReference = adcuZero + p_linear->XDelta;

    // *p_linear = (Linear_T)LINEAR_ADC_INIT(adcuBits, adcuZero, isInverted);
}


// void Linear_ADC_InitAsPercent16(Linear_T * p_linear, uint16_t adcuZero, uint8_t adcuRef, bool isInverted)
// {
//     Linear_Q16_Init(p_linear, adcuZero, adcuRef);
//     p_linear->Slope = isInverted ? -1 : 1;
// }


// void Linear_ADC_InitAsPhysical(Linear_T * p_linear, uint16_t adcuZero, uint8_t adcuRef, uint16_t unitRef, bool isInverted)
// {

// }


/******************************************************************************/
/*!
    f([adcuZero:adcuRef]) = [0:65536]
*/
/******************************************************************************/
void Linear_ADC_Init_ZeroToPeakMilliV(Linear_T * p_linear, uint16_t adcVRef_MilliV, uint16_t adcMax, uint16_t zero_MilliV, uint16_t max_MilliV)
{
    uint16_t adcuZero = (uint32_t)zero_MilliV * adcMax / adcVRef_MilliV;
    uint16_t adcuRef = (uint32_t)max_MilliV * adcMax / adcVRef_MilliV;
    Linear_Q16_Init(p_linear, adcuZero, adcuRef);
}
