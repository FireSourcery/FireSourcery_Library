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
    @file   ADC_Map.h
    @author FireSourcery
    @brief  Result map. Copies Hw ordered results into consumer defined indexes. Remap only, no conversion.

    A channel mask decodes only a compaction: the channels of 1 sequence, in channel order, packed.
    A map decodes any layout the consumer defines:
        - any order, by the consumer's own index enum
        - any ADC, per entry, so 1 map can span converters
        - sparse, a map writes only its own entries. Several maps can fill 1 destination
        - repeats, 1 source may feed several indexes

    Source is a pointer into an ADC's results buffer, resolved at config time. Decode is 1 load, 1 store per entry.

        enum { MOTOR_ANALOG_IA, MOTOR_ANALOG_IB, MOTOR_ANALOG_IC, MOTOR_ANALOG_VBUS, MOTOR_ANALOG_VA, MOTOR_ANALOG_VB, MOTOR_ANALOG_VC, MOTOR_ANALOG_COUNT };

        static const ADC_MapEntry_T MOTOR0_I_ENTRIES[] =
        {
            { .INDEX = MOTOR_ANALOG_IA,   .P_RESULT = &ADC1_RESULTS[BOARD_ADC1_SLOT_IA] },
            { .INDEX = MOTOR_ANALOG_IB,   .P_RESULT = &ADC1_RESULTS[BOARD_ADC1_SLOT_IB] },
            { .INDEX = MOTOR_ANALOG_IC,   .P_RESULT = &ADC0_RESULTS[BOARD_ADC0_SLOT_IC] },     // another converter
            { .INDEX = MOTOR_ANALOG_VBUS, .P_RESULT = &ADC1_RESULTS[BOARD_ADC1_SLOT_VSOURCE] },
        };

        static const ADC_Map_T MOTOR0_I_MAP = ADC_MAP_INIT(MOTOR0_I_ENTRIES);

        adc_result_t motorAnalog[MOTOR_ANALOG_COUNT];
        ADC_Map_Decode(&MOTOR0_I_MAP, motorAnalog);     // writes IA IB IC VBUS, leaves VA VB VC
*/
/******************************************************************************/
#include "ADC.h"
#include "Math/math_general.h"

/* Destination. The consumer's index */
/* Source. A slot in any ADC's results buffer */
// typedef const struct ADC_MapEntry { const volatile adc_result_t * P_RESULT; } ADC_MapEntry_T;
// typedef const struct ADC_MapEntry { uint8_t INDEX; const volatile adc_result_t * P_RESULT; } ADC_MapEntry_T;
typedef adc_result_t * ADC_MapEntry_T;
typedef ADC_MapEntry_T const ADC_Map_T[];
static inline adc_result_t ADC_Demap(const ADC_Map_T map, uint8_t destIndex) { return *map[destIndex]; }
// typedef const struct ADC_Map { const ADC_MapEntry_T * P_ENTRIES; uint8_t COUNT; } ADC_Map_T;


/* p_dest[INDEX] = *P_RESULT, for each entry. Indexes outside the map are left unchanged */
static inline void ADC_Map_Decode(const ADC_Map_T map, adc_result_t * p_dest, uint8_t count)
{
    for (uint8_t index = 0U; index < count; index++) { p_dest[index] = ADC_Demap(map, index); }
}

/* The destination length a map needs. 1 past its highest index */
// static inline uint8_t ADC_Map_Extent(const ADC_Map_T map, uint8_t count)
// {
//     uint8_t extent = 0U;
//     for (uint8_t index = 0U; index < count; index++) { extent = (uint8_t)math_max(extent, map[index].INDEX + 1U); }
//     return extent;
// }



