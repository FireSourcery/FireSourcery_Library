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
    @file   Global_Analog.h
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef GLOBAL_ANALOG_H
#define GLOBAL_ANALOG_H

#include "Config.h"
#include <stdint.h>
#include <stdbool.h>

/* Global Static Const  */
typedef const struct Global_Analog
{
    const uint16_t ADC_BITS;
    const uint16_t ADC_MAX;
    const uint16_t ADC_VREF_MILLIV;
    const uint16_t ADC_FIFO_LENGTH;
}
Global_Analog_T;

#define GLOBAL_ANALOG_ADC_MAX(AdcBits) ((1U << AdcBits) - 1U)

/* MISRA violation */
/* Define in Main App */
extern const Global_Analog_T GLOBAL_ANALOG;

#endif
