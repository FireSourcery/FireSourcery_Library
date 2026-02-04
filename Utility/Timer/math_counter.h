#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery

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
    @file   counter_fn.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include <stdint.h>

static inline uint32_t counter_wrapped(uint32_t max, uint32_t prev, uint32_t count) { return (count < prev) ? (max - prev + count) : (count - prev); }
static inline bool counter_is_aligned(uint32_t mask, uint32_t count) { return ((count & mask) == 0UL); }

static inline uint32_t timer_elapsed_wrapped(uint32_t time_prev, uint32_t time) { return counter_wrapped(UINT32_MAX, time_prev, time); }
static inline uint32_t timer_elapsed_direct(uint32_t time_prev, uint32_t time) { return (time - time_prev); }
static inline bool timer_is_elapsed_wrapped(uint32_t period, uint32_t time_prev, uint32_t time) { return (timer_elapsed_wrapped(time_prev, time) >= period); }
static inline bool timer_is_elapsed_direct(uint32_t period, uint32_t time_prev, uint32_t time) { return (timer_elapsed_direct(time_prev, time) >= period); }