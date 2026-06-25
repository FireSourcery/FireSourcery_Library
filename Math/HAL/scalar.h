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
    @file   scalar.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "math_general.h"
#include "Fixed/fract16.h"

/*
    HAL for math
    data abstraction layer for math value type, real_t
    wrap semantics over storage.
*/
#ifndef FLOATING_POINT
typedef accum32_t scalar_wide_t; /*  */
typedef fract16_t scalar_unit_t; /* < 1.0 */
typedef fract16e_t scalar_ext_t;
typedef fract16_t scalar_t;
#else
typedef float scalar_ext_t;
typedef float scalar_t;

typedef float fract_t;
typedef float accum_t;
#endif


static inline float _float_mul(float a, float b) { return a * b; }

#define scalar_mul(a, b) _Generic((a), \
    fract16_t: fract16_mul, \
    fract16e_t: fract16e_mul, \
    float : _float_mul \
)(a, b)


/******************************************************************************/
/*
    Number formats

    [Fract16]           [-1:1) <=> [-32768:32767] in Q1.15
    [UFract16]          [0:2) <=> [0:65535] in Q1.15
    [Accum32]           [-2:2] <=> [-65536:65536] in Q17.15     Max [INT32_MIN:INT32_MAX]
    [UQ16]              [0:1) <=> [0:65535] in Q0.16
    [Fixed16]           [-1:1] <=> [-256:256] in Q8.8           Max [-32768:32767]
    [Fixed32]           [-1:1] <=> [-65536:65536] in Q16.16     Max [INT32_MIN:INT32_MAX]
*/
/******************************************************************************/