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
typedef const struct Analog_Reference
{
    uint16_t ADC_BITS;
    uint16_t ADC_MAX;
    uint16_t ADC_VREF_MILLIV;
}
Analog_Reference_T;

#define ANALOG_REFERENCE_ADC_MAX(AdcBits) ((1U << AdcBits) - 1U)
#define ANALOG_REFERENCE_INIT(AdcBits, AdcVrefMilliv) { AdcBits, ((1U << AdcBits) - 1U), AdcVrefMilliv }

/* Global Static. Define in Main App */
extern const Analog_Reference_T ANALOG_REFERENCE;


#endif
