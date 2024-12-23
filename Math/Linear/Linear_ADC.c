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
    @file   Linear_ADC.c
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#include "Linear_ADC.h"

/******************************************************************************/
/*!
    f(adcu) = fract16
    f(adcuZero) = 0
    f(adcuRef) = 65535
*/
/******************************************************************************/
void Linear_ADC_Init(Linear_T * p_linear, uint16_t adcuZero, uint16_t adcuRef)
{
    Linear_Q16_Init(p_linear, adcuZero, adcuRef);
}

void Linear_ADC_Init_ZeroToPeak(Linear_T * p_linear, uint16_t adcuZero, uint16_t adcuZtPRef)
{
    Linear_Q16_Init(p_linear, adcuZero, adcuZero + adcuZtPRef);
}

// void Linear_ADC_Init_MinMax(Linear_T * p_linear, uint16_t adcuMin, uint16_t adcuMax)
// {
//     Linear_Q16_Init(p_linear, (adcuMin + adcuMax) / 2, adcuMax);
// }

/******************************************************************************/
/*!
    Inverted pivot on 0
    f(adcuZero) = 0
    f(adcuRef) = -65536
    f(-adcuRef) = 65536
*/
/******************************************************************************/
void Linear_ADC_Init_Inverted(Linear_T * p_linear, uint16_t adcuZero, uint16_t adcuRef)
{
    Linear_ADC_Init(p_linear, adcuZero, adcuRef);
    Linear_ADC_SetInverted(p_linear);
}

void Linear_ADC_SetInverted(Linear_T * p_linear)
{
    p_linear->Slope = 0 - p_linear->Slope;
    p_linear->InvSlope = 0 - p_linear->InvSlope;
}

/******************************************************************************/
/*!

*/
/******************************************************************************/
void Linear_ADC_Init_PeakToPeakMilliV(Linear_T * p_linear, uint16_t adcVRef_MilliV, uint16_t adcMax, uint16_t min_MilliV, uint16_t max_MilliV)
{
    uint16_t adcuZero = ((uint32_t)max_MilliV + min_MilliV) * adcMax / 2U / adcVRef_MilliV;
    uint16_t adcuRef = (uint32_t)max_MilliV * adcMax / adcVRef_MilliV;
    Linear_ADC_Init(p_linear, adcuZero, adcuRef);
}

void Linear_ADC_Init_ZeroToPeakMilliV(Linear_T * p_linear, uint16_t adcVRef_MilliV, uint16_t adcMax, uint16_t zero_MilliV, uint16_t max_MilliV)
{
    uint16_t adcuZero = (uint32_t)zero_MilliV * adcMax / adcVRef_MilliV;
    uint16_t adcuRef = (uint32_t)max_MilliV * adcMax / adcVRef_MilliV;
    Linear_ADC_Init(p_linear, adcuZero, adcuRef);
}
