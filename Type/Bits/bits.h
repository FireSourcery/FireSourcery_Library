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

static inline void set_bits(uint32_t * p_bits, uint8_t index, uint32_t value) { *p_bits = (*p_bits & ~(1UL << index)) | (value << index); }

static inline uint32_t bitmask_of(uint8_t width, uint8_t index) { return ((1UL << width) - 1UL) << index; }
static inline uint32_t bits_of(uint32_t bits, uint8_t index, uint8_t width) { return (bits & bitmask_of(width, index)) >> index; }


#if defined(__GNUC__)

static inline void bits_foreach(uint32_t bits, uint8_t width, void (*fn)(void * p_context, uint8_t index), void * p_context)
{
    while (bits != 0)
    {
        // int bit_pos = __builtin_ctz(bits); // Find lowest set bit // Maps to ARM RBIT + CLZ
        fn(p_context, __builtin_ctz(bits)); // Process this input
        bits &= (bits - 1);  // Clear rightmost set bit
    }
}

#else
static inline void _bits_foreach(uint32_t bits, uint8_t width, void (*fn)(uint8_t index))
{
    for (uint8_t i = 0U; i < width; i++) { if (bit_at(bits, i)) { fn(i); } }
}
static inline void bits_foreach(uint32_t bits, uint8_t width, void (*fn)(uint8_t index))
{
    if (bits != 0U)
    {
        if (bits & 0x0000FFFFUL != 0U)
        {
            if (bits & 0x000000FFUL != 0U) { _bits_foreach(bits & 0x000000FFUL, 8U, fn); }
            if (bits & 0x0000FF00UL != 0U) { _bits_foreach(bits & 0x0000FF00UL, 8U, fn); }
        }
        if (bits & 0xFFFF0000UL != 0U)
        {
            if (bits & 0x00FF0000UL != 0U) { _bits_foreach(bits & 0x00FF0000UL, 8U, fn); }
            if (bits & 0xFF000000UL != 0U) { _bits_foreach(bits & 0xFF000000UL, 8U, fn); }
        }
    }
}
#endif

#endif


