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
    @file   real.h
    @author FireSourcery
    @brief  Numeric representation HAL. Selects [real_fixed.h] or [real_float.h].
*/
/******************************************************************************/
/*
    HAL for math
    data abstraction layer for math value type
    wrap semantics over storage.
    uniform handling in float <=> selective in fixed-point
*/
#if defined(FLOATING_POINT)
#include "real_float.h"
#else
#include "real_fixed.h"
#endif


