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
typedef fract16e_t scalar_ext_t;
typedef fract16_t scalar_t;
#else
typedef float scalar_ext_t;
typedef float scalar_t;
#endif


static inline float _float_mul(float a, float b) { return a * b; }

#define scalar_mul(a, b) _Generic((a), \
    fract16_t: fract16_mul, \
    fract16e_t: fract16e_mul, \
    float : _float_mul \
)(a, b)


