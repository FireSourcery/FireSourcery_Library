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
    @file   void_pointer.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include <stdint.h>
#include <string.h>


/******************************************************************************/
/*!

*/
/******************************************************************************/
/*
    value operations should inline with type
*/
/*!
   generic switch copy / memcpy
*/
/* less function call when 'type' is not compile time const */
/* same as memcpy when type is compile time literal */
static inline void switch_copy(void * p_dest, const void * p_src, size_t size)
{
    if (!__builtin_constant_p(size)) { memcpy(p_dest, p_src, size); return; }

    switch (size)
    {
        case sizeof(uint8_t) : *((uint8_t  *)p_dest) = *((const uint8_t  *)p_src); break;
        case sizeof(uint16_t): *((uint16_t *)p_dest) = *((const uint16_t *)p_src); break;
        case sizeof(uint32_t): *((uint32_t *)p_dest) = *((const uint32_t *)p_src); break;
#if (REGISTER_SIZE_64)
        case sizeof(uint64_t) : *((uint64_t *)p_dest) = *((const uint64_t *)p_src); break;
#endif
        default: memcpy(p_dest, p_src, size); break;
    }
}

/* Copy as type */
// static inline void pointer_assign(size_t type, void * p_unit, const void * p_value) { switch_copy(p_unit, p_value, type); }
/* transparent when type is known at compile time */
static inline void pointer_assign(size_t type, void * p_unit, const void * p_value) { memcpy(p_unit, p_value, type); }


/* value sign extension */
static inline int pointer_value_as(size_t type, const void * p_unit)
{
    switch (type)
    {
        case sizeof(int8_t):  return *((const int8_t *)p_unit);
        case sizeof(int16_t): return *((const int16_t *)p_unit);
        case sizeof(int32_t): return *((const int32_t *)p_unit);
#if (REGISTER_SIZE_64)
        case sizeof(int64_t): return *((const int64_t *)p_unit);
#endif
        default:  return 0;
    }
}

/* value version signiture clamp with type */
/* preserves endianess */
static inline void pointer_assign_value_as(size_t type, void * p_unit, int value)
{
    switch (type)
    {
        case sizeof(int8_t):  *((int8_t *)p_unit)  = (int8_t)value;  break;
        case sizeof(int16_t): *((int16_t *)p_unit) = (int16_t)value; break;
        case sizeof(int32_t): *((int32_t *)p_unit) = (int32_t)value; break;
#if (REGISTER_SIZE_64)
        case sizeof(int64_t): *((int64_t *)p_unit) = (int64_t)value; break;
#endif
        default: break;
    }
}



/******************************************************************************/
/*!

*/
/******************************************************************************/
typedef  union
{
    // uint8_t * asU8;
    uint8_t * u8;
    uint16_t * u16;
    uint32_t * u32;
    uint64_t * u64;
}
pointer_cast_t;

static inline pointer_cast_t pointer_cast(const void * p_unit) { return (pointer_cast_t) { .u8 = (uint8_t *)p_unit }; }

#define pointer_value(T, p_unit) _Generic(((T)0), \
    uint8_t: *pointer_cast(p_unit).u8, \
    uint16_t: *pointer_cast(p_unit).u16, \
    uint32_t: *pointer_cast(p_unit).u32, \
    uint64_t: *pointer_cast(p_unit).u64, \
    default: 0 \
)

// typedef struct
// {
//     pointer_cast_t a;
//     size_t size;
//     bool valid;
// }
// pointer_cast_safe_t;
// pointer_cast_t pointer_cast_safe (const void * p_unit, size_t size) { return (pointer_cast_t) { .u8 = (uint8_t *)p_unit }; }