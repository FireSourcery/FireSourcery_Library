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
    @file   math_general_float.h
    @author FireSourcery
    @brief  float overloads of the [math_general.h] order helpers.
            Integer callers (ticks, counts, angles) keep the int32_t versions.
*/
/******************************************************************************/
#include "Math/math_general.h"
#include <math.h>

static inline float math_absf(float value) { return fabsf(value); }

static inline float math_maxf(float value1, float value2) { return ((value1 > value2) ? value1 : value2); }
static inline float math_minf(float value1, float value2) { return ((value1 < value2) ? value1 : value2); }
static inline float math_clampf(float value, float lower, float upper) { return math_minf(math_maxf(value, lower), upper); }

static inline sign_t math_signf(float value) { return (value > 0.0F) - (value < 0.0F); }
