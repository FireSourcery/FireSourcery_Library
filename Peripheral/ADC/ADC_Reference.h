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
    @file   ADC_Reference.h
    @author FireSourcery
    @brief  The converter's full scale, as the consumer sees it. 1 per application.
*/
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>

typedef const struct ADC_Reference
{
    uint16_t ADC_BITS;
    uint16_t ADC_MAX;
    uint16_t ADC_VREF_MILLIV;
}
ADC_Reference_T;

/* Derives ADC_MAX from the bits, at config time */
#define ADC_REFERENCE_INIT(AdcBits, AdcVrefMilliv) (ADC_Reference_T) \
    { .ADC_BITS = (AdcBits), .ADC_MAX = ((1U << (AdcBits)) - 1U), .ADC_VREF_MILLIV = (AdcVrefMilliv), }

/* Global static. Defined in the main App */
extern const ADC_Reference_T ADC_REFERENCE;
