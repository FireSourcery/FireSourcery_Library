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
    @file   generic_array.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/


/*
    Generic array implementation - using Macros with _Generic selection for type safety

    Compile time typed
    effectively void array expressed with type prior to what compiler may optimize
    alternative to handling size parameter, may be faster depending on compiler.
*/
/*
    Allow sub-module to configure type using a single parameter
*/
// #define as_pointer(T, p_buffer) ((T *)p_buffer)
#define as_pointer(T, p_buffer) \
    _Generic((T)0, \
        int8_t  : ((int8_t *)p_buffer), \
        int16_t : ((int16_t *)p_buffer), \
        int32_t : ((int32_t *)p_buffer), \
        int8_t *: ((int8_t *)p_buffer), \
        int16_t *: ((int16_t *)p_buffer), \
        int32_t *: ((int32_t *)p_buffer), \
        int64_t *: ((int64_t *)p_buffer) \
    )

// #define as_pointer(T, p_buffer) \
//     _Generic((T)0, \
//         int8_t    : ((int8_t *)p_buffer), \
//         uint8_t   : ((uint8_t *)p_buffer), \
//         int16_t   : ((int16_t *)p_buffer), \
//         uint16_t  : ((uint16_t *)p_buffer), \
//         int32_t   : ((int32_t *)p_buffer), \
//         uint32_t  : ((uint32_t *)p_buffer), \
//         int64_t   : ((int64_t *)p_buffer), \
//         uint64_t  : ((uint64_t *)p_buffer), \
//         float     : ((float *)p_buffer), \
//         double    : ((double *)p_buffer), \
//         default   : ((void *)p_buffer) \
//     )

#define as_ref(T, p_buffer) (as_pointer(T, p_buffer)[0U])

#define as_value(T, p_buffer) as_ref(T, p_buffer)

// #define as(T, p_buffer) \
//     _Generic((T)0, \
//         int8_t  : as_value(T, p_buffer), \
//         int16_t : as_value(T, p_buffer), \
//         int32_t : as_value(T, p_buffer), \
//         int8_t * : as_pointer(T, p_buffer), \
//         int16_t *: as_pointer(T, p_buffer) \
//     )


#define as_array_at(T, p_buffer, index) (as_pointer(T, p_buffer)[index])