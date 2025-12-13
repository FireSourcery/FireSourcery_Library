#ifndef VOID_ARRAY_H
#define VOID_ARRAY_H

#include "../accessor.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <sys/types.h>

#ifndef REGISTER_SIZE_64
#define REGISTER_SIZE_64 (__SIZEOF_POINTER__ >= 8)
#endif

/******************************************************************************/
/*!
    @brief Void Array / Sized Array - Generic by type
    let compiler to optimize away [size_t type]
    alternatively _Generic select on literal type,
        Macro arguments lose type constraints
*/
/******************************************************************************/
/*  */
// static inline void * void_pointer_at(size_t type, const void * p_buffer, size_t index) { return ((uint8_t *)p_buffer + (index * type)); }
static inline void * void_pointer_at(const void * p_buffer, size_t type, size_t index) { return ((uint8_t *)p_buffer + (index * type)); }

/*!
    swtich copy
    @brief Copy data from source to destination based on the size.
    @param dest Pointer to the destination buffer.
    @param src Pointer to the source buffer.
    @param size Size of the data to copy.
*/
static inline void void_copy(void * p_dest, const void * p_src, size_t size)
{
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

static inline void void_pointer_assign(void * p_unit, size_t type, const void * p_value) { void_copy(p_unit, p_value, type); }

/* this should inline with type */
// static inline value_t as_value(size_t type, const void * p_unit)
static inline value_t void_pointer_as_value(const void * p_unit, size_t type)
{
    value_t value = 0;
    switch (type)
    {
        case sizeof(int8_t):  value = *((const int8_t *)p_unit);  break;
        case sizeof(int16_t): value = *((const int16_t *)p_unit); break;
        case sizeof(int32_t): value = *((const int32_t *)p_unit); break;
#if (REGISTER_SIZE_64)
        case sizeof(int64_t): value = *((const int64_t *)p_unit); break;
#endif
        default: break;
    }
    return value;
}

static inline void void_pointer_assign_as_value(void * p_unit, size_t type, value_t value)
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

// static inline void void_pointer_assign_as_cast(void * p_unit, size_t type, uintptr_t arg)
// {
//     switch (type)
//     {
//         case sizeof(uint8_t) : *((uint8_t  *)p_unit) = (uint8_t)arg; break;
//         case sizeof(uint16_t): *((uint16_t *)p_unit) = (uint16_t)arg; break;
//         case sizeof(uint32_t): *((uint32_t *)p_unit) = (uint32_t)arg; break;
// #if (REGISTER_SIZE_64)
//         case sizeof(uint64_t) : *((uint64_t *)p_unit) = (uint64_t)arg; break;
// #endif
//         default: memcpy(p_unit, (const void *)arg, type); break;
//     }
// }

/******************************************************************************/
/*
    Iteration
    struct array
*/
/******************************************************************************/
/* todo  static inline void array_foreach(size_t type, void * p_array, size_t length, proc_t func) */

/*
    0 argument accessors
*/
/*
    length in units
*/
static inline void void_array_foreach(void * p_buffer, size_t type, size_t length, proc_t unit_op)
{
    for (size_t index = 0U; index < length; index++) { unit_op(void_pointer_at(p_buffer, type, index)); }
}

/*!
    applies to every element
    @return true if all return true
*/
static inline bool void_array_for_every(void * p_buffer, size_t type, size_t length, try_proc_t unit_poll)
{
    bool is_every = true;
    for (size_t index = 0U; index < length; index++) { if (unit_poll(void_pointer_at(p_buffer, type, index)) == false) { is_every = false; } }
    return is_every;
}

/*!
    @return true if at least one return true
*/
static inline bool void_array_for_any(void * p_buffer, size_t type, size_t length, try_proc_t unit_poll)
{
    bool is_any = false;
    for (size_t index = 0U; index < length; index++) { if (unit_poll(void_pointer_at(p_buffer, type, index)) == true) { is_any = true; } }
    return is_any;
}

/*
    const and returns early
*/
static inline bool void_array_is_every(const void * p_buffer, size_t type, size_t length, test_t unit_test)
{
    bool is_every = true;
    for (size_t index = 0U; index < length; index++) { if (unit_test(void_pointer_at(p_buffer, type, index)) == false) { is_every = false; break; } }
    return is_every;
}

static inline bool void_array_is_any(const void * p_buffer, size_t type, size_t length, test_t unit_test)
{
    bool is_any = false;
    for (size_t index = 0U; index < length; index++) { if (unit_test(void_pointer_at(p_buffer, type, index)) == true) { is_any = true; break; } }
    return is_any;
}

/*
    register size value interface
*/
/*
    accessors with parameters of register width type
    using register size as parameter interface
*/
static inline void void_array_foreach_set(void * p_buffer, size_t type, size_t length, set_t unit_setter, value_t value)
{
    for (size_t index = 0U; index < length; index++) { unit_setter(void_pointer_at(p_buffer, type, index), value); }
}

static inline bool void_array_for_every_set(void * p_buffer, size_t type, size_t length, try_set_t unit_try, value_t value)
{
    bool is_every = true;
    for (size_t index = 0U; index < length; index++) { if (unit_try(void_pointer_at(p_buffer, type, index), value) == false) { is_every = false; } }
    return is_every;
}

static inline bool void_array_for_any_set(void * p_buffer, size_t type, size_t length, try_set_t unit_try, value_t value)
{
    bool is_any = false;
    for (size_t index = 0U; index < length; index++) { if (unit_try(void_pointer_at(p_buffer, type, index), value) == true) { is_any = true; } }
    return is_any;
}

static inline bool void_array_is_every_value(const void * p_buffer, size_t type, size_t length, test_value_t test, value_t value)
{
    bool is_every = true;
    for (size_t index = 0U; index < length; index++) { if (test(void_pointer_at(p_buffer, type, index), value) == false) { is_every = false; break; } }
    return is_every;
}

static inline bool void_array_is_any_value(const void * p_buffer, size_t type, size_t length, test_value_t test, value_t value)
{
    bool is_any = false;
    for (size_t index = 0U; index < length; index++) { if (test(void_pointer_at(p_buffer, type, index), value) == true) { is_any = true; break; } }
    return is_any;
}

/******************************************************************************/
/*
    value arary
*/
/******************************************************************************/
/*
    multiple units by pointer
*/
static inline void void_array_copy_to(const void * p_buffer, size_t type, void * p_to, size_t count) { memcpy(p_to, p_buffer, type * count); }
static inline void void_array_copy_from(void * p_buffer, size_t type, const void * p_from, size_t count) { memcpy(p_buffer, p_from, type * count); }

/*
    array
    single unit at index by value
*/
static inline value_t void_array_get(const void * p_buffer, size_t type, size_t index) { return void_pointer_as_value(void_pointer_at(p_buffer, type, index), type); }
static inline void void_array_set(void * p_buffer, size_t type, size_t index, value_t value) { void_pointer_assign_as_value(void_pointer_at(p_buffer, type, index), type, value); }

/******************************************************************************/
/*
    Value Array Iteration
*/
/******************************************************************************/
/*
    handles numeric types
*/
static inline void * void_array_min(const void * p_buffer, size_t type, size_t length)
{
    const void * p_min = p_buffer;
    void * p_unit;
    for (size_t index = 1U; index < length; index++)
    {
        p_unit = void_pointer_at(p_buffer, type, index);
        if (memcmp(p_unit, p_min, type) < 0) { p_min = p_unit; }
    }
    return (void *)p_min;
}

static inline void * void_array_max(const void * p_buffer, size_t type, size_t length)
{
    const void * p_max = p_buffer;
    void * p_unit;
    for (size_t index = 1U; index < length; index++)
    {
        p_unit = void_pointer_at(p_buffer, type, index);
        if (memcmp(p_unit, p_max, type) > 0) { p_max = p_unit; }
    }
    return (void *)p_max;
}

static inline value_t void_array_min_value(const void * p_buffer, size_t type, size_t length)
{
    return void_pointer_as_value(void_array_min(p_buffer, type, length), type);
}

static inline value_t void_array_max_value(const void * p_buffer, size_t type, size_t length)
{
    return void_pointer_as_value(void_array_max(p_buffer, type, length), type);
}

static inline int compare_int(const void * a, const void * b) { return (*(int *)a - *(int *)b); }

static inline void * void_array_min_with(void * p_buffer, size_t type, size_t length, compare_t compare)
{
    void * p_min = p_buffer;
    void * p_unit;
    for (size_t index = 1U; index < length; index++)
    {
        p_unit = void_pointer_at(p_buffer, type, index);
        if (compare(p_min, p_unit) > 0) { p_min = p_unit; }
    }
    return p_min;
}

static inline void * void_array_max_with(void * p_buffer, size_t type, size_t length, compare_t compare)
{
    void * p_max = p_buffer;
    void * p_unit;
    for (size_t index = 1U; index < length; index++)
    {
        p_unit = void_pointer_at(p_buffer, type, index);
        if (compare(p_max, p_unit) < 0) { p_max = p_unit; }
    }
    return p_max;
}

#endif // VOID_ARRAY_H

