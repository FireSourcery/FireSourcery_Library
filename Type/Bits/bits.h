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
    @file   general.h
    @author FireSourcery
    @brief

*/
/******************************************************************************/
#ifndef BITS_H
#define BITS_H

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

typedef uint32_t bits_t;

static inline bool bit_at(uint32_t bits, uint8_t index) { return (bits & (1U << index)); }
static inline void fill_bit(uint32_t * p_bits, uint8_t index) { *p_bits |= (1U << index); }
static inline void clear_bit(uint32_t * p_bits, uint8_t index) { *p_bits &= ~(1U << index); }
static inline void set_bit(uint32_t * p_bits, uint8_t index, bool value) { (value) ? fill_bit(p_bits, index) : clear_bit(p_bits, index); }

static inline void set_bits(uint32_t * p_bits, uint8_t index, uint32_t value) { *p_bits = (*p_bits & ~(1U << index)) | (value << index); }

static inline void bits_foreach(uint32_t bits, uint8_t width, void (*fn)(uint8_t index))
{
    for (uint8_t i = 0U; i < width; i++) { if (bit_at(bits, i) == true) { fn(i); } }
}

static inline uint32_t bitmask_of(uint8_t index, uint32_t width) { return ((1U << width) - 1U) << index; }
static inline uint32_t bits_of(uint32_t bits, uint8_t index, uint8_t width) { return (bits & bitmask_of(width, index)) >> index; }


#endif


