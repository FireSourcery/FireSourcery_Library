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
    @file   math_general.h
    @author FireSourcery
    @brief

*/
/******************************************************************************/
#ifndef MATH_BITS_H
#define MATH_BITS_H

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

/* pow2 only */
static inline uintptr_t align_down(uintptr_t value, size_t align) { return (value & (uintptr_t)(-align)); }
static inline uintptr_t align_up(uintptr_t value, size_t align) { return (-((-value) & (uintptr_t)(-align))); }
static inline bool is_aligned(uintptr_t value, size_t align) { return ((value & (uintptr_t)(align - 1U)) == (uintptr_t)0U); }

/* is_aligned_mask */
static inline bool is_masked(uintptr_t value, uintptr_t mask) { return ((value & mask) == (uintptr_t)0U); }


#endif


