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
    @file   Linear_ADC.h
    @author FireSourcery
    @brief  Scale adcu to a reference value.
*/
/******************************************************************************/
#include "Math/Linear/Linear_Q16.h"

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*!
    Fract16. Zero referenced, signed both ways

    f(adcuZero) = 0
    f([adcuZero - (2^adcBits - adcuZero) : adcuZero + (2^adcBits - adcuZero)]) = [-32768:32768]

    A multiply by +/-1 and a shift.
*/
/******************************************************************************/
#define LINEAR_ADC_FRACT16_INIT(AdcBits, AdcuZero, IsInverted) (Linear_T) \
{                                                               \
    .Slope          = (IsInverted) ? -1 : 1,                    \
    .SlopeShift     = 15U - (AdcBits),                          \
    .InvSlope       = (IsInverted) ? -1 : 1,                    \
    .InvSlopeShift  = 15U - (AdcBits),                          \
    .X0             = (AdcuZero),                               \
    .XDelta         = (1L << (AdcBits)) - (AdcuZero),           \
    .Y0             = 0,                                        \
    .YDelta         = 32768 * ((IsInverted) ? -1 : 1),          \
}

static inline int16_t Linear_ADC_Normalize(const Linear_T * p_linear, uint16_t adcu) { return linear_shift_f_x0(p_linear->Slope, p_linear->SlopeShift, p_linear->X0, adcu); }
static inline uint16_t Linear_ADC_Of(const Linear_T * p_linear, int32_t normalized) { return linear_shift_invf_x0(p_linear->InvSlope, p_linear->InvSlopeShift, p_linear->X0, normalized); }

/* Runtime form of LINEAR_ADC_FRACT16_INIT */
static inline void Linear_ADC_Init_Fract16(Linear_T * p_linear, uint16_t adcuZero, uint8_t adcBits, bool isInverted)
{
    *p_linear = (Linear_T)LINEAR_ADC_FRACT16_INIT(adcBits, adcuZero, isInverted);
}

/******************************************************************************/
/*!
    Unsigned, over a millivolt window

    f([adcuOf(zero_MilliV) : adcuOf(max_MilliV)]) = [0:65536]
*/
/******************************************************************************/
static inline void Linear_ADC_Init_ZeroToPeakMilliV(Linear_T * p_linear, uint16_t adcVRef_MilliV, uint16_t adcMax, uint16_t zero_MilliV, uint16_t max_MilliV)
{
    Linear_Q16_Init(p_linear, ((uint32_t)zero_MilliV * adcMax / adcVRef_MilliV), ((uint32_t)max_MilliV * adcMax / adcVRef_MilliV));
}
