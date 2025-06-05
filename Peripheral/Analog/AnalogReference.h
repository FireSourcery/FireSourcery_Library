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
    @file   AnalogReference.h
    @author FireSourcery
    @brief

*/
/******************************************************************************/
#ifndef ANALOG_REFERENCE_H
#define ANALOG_REFERENCE_H

#include "Config.h"
#include <stdint.h>
#include <stdbool.h>

/* Global Static Const  */
typedef const struct AnalogReference
{
    const uint16_t ADC_BITS;
    const uint16_t ADC_MAX;
    const uint16_t ADC_VREF_MILLIV;
}
AnalogReference_T;

#define ANALOG_REFERENCE_ADC_MAX(AdcBits) ((1U << AdcBits) - 1U)

/* Global Static. Define in Main App */
extern const AnalogReference_T ANALOG_REFERENCE;

// static inline int16_t AnalogUnits_OpInvert_Fract16(uint16_t adcuZero, uint16_t adcu) { return ((adcu - adcuZero) * (-1 << 3)); }
// static inline int16_t AnalogUnits_Fract16(uint16_t adcuZero, uint16_t adcu) { return ((adcu - adcuZero) << 3); }

// static inline int16_t AnalogUnit16_OfResult(uint16_t adcuZero, uint16_t adcu) { return ((adcu - adcuZero) << 3); }
// static inline int16_t AnalogUnit16(int16_t adcu) { return ((adcu) << 3); }
// static inline int16_t scaled_adcu16(int16_t adcu) { return ((adcu) << (16-1-ADC_BITS)); }

#endif
