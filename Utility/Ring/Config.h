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
    @file   Config.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#if      defined(RING_INDEX_POW2_COUNTER)       /*! Power 2 length only. Mask on each access */
#elif    defined(RING_INDEX_POW2_WRAP)          /*! Power 2 length only. Mask once on update index. -1 capacity */
#elif    defined(RING_INDEX_LENGTH_COMPARE)     /*! Flexible: any buffer size */
#else
    #define RING_INDEX_POW2_COUNTER
    #warning "Ring Buffer: No index mode defined. Defaulting to RING_INDEX_POW2_COUNTER"
#endif

/* Critical at Ring module level */
#if     defined(RING_LOCAL_CRITICAL_ENABLE)
    #define RING_LOCAL_CRITICAL
#elif   defined(RING_LOCAL_CRITICAL_DISABLE)
#else
    #define RING_LOCAL_CRITICAL_DISABLE
#endif

// #if     defined(RING_ALIGNMENT_ENABLE)
// #endif



#define _CAT(a, b) a##b
#define CAT(a, b) _CAT(a, b)

#define _RING_IF_0(...)
#define _RING_IF_1(...) __VA_ARGS__
#define _RING_IF_FALSE(...)
#define _RING_IF_TRUE(...) __VA_ARGS__
#define RING_IF(condition) CAT(_RING_IF_, condition)




