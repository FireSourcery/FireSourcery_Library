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
        let compiler to optimize away [size_t][type]
        alternatively _Generic select on literal type,
            Macro arguments lose type constraints
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

/* Copy as type */
static inline void void_pointer_assign(size_t type, void * p_unit, const void * p_value) { void_copy(p_unit, p_value, type); }

/*
    Scalar value path
*/
/* value sign extension */
static inline value_t void_pointer_as_value(size_t type, const void * p_unit)
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

/* value version signiture clamp with type */
/* preserves endianess */
static inline void void_pointer_assign_as_value(size_t type, void * p_unit, value_t value)
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


/*!
    @param type size of the element type
*/
static inline void * void_array_at(size_t type, const void * p_buffer, size_t index) { return ((uint8_t *)p_buffer + (index * type)); }

/*
    array
    single unit at index by value
*/
static inline value_t void_array_get(size_t type, const void * p_buffer, size_t index) { return void_pointer_as_value(type, void_array_at(type, p_buffer, index)); }
static inline void void_array_set(size_t type, void * p_buffer, size_t index, value_t value) { void_pointer_assign_as_value(type, void_array_at(type, p_buffer, index), value); }

/*
    multiple units by pointer
*/
static inline void void_array_copy_to(size_t type, const void * p_buffer, void * p_to, size_t count) { memcpy(p_to, p_buffer, type * count); }
static inline void void_array_copy_from(size_t type, void * p_buffer, const void * p_from, size_t count) { memcpy(p_buffer, p_from, type * count); }

/******************************************************************************/
/*
    Iteration
    length in units
    For each doesnt need transparent type optimization to pass through to memcpy
*/
/******************************************************************************/
/*
    accessors with 0 additional arguments
*/
static inline void void_array_foreach(size_t type, void * p_buffer, size_t length, proc_t unit_op)
{
    for (size_t index = 0U; index < length; index++) { unit_op(void_array_at(type, p_buffer, index)); }
}

#define array_foreach(p_buffer, length, op) void_array_foreach(sizeof(*(p_buffer)), (void *)p_buffer, length, (proc_t)op)

/*!
    applies to every element
    @return true if all return true
*/
static inline bool void_array_for_every(size_t type, void * p_buffer, size_t length, try_proc_t unit_poll)
{
    bool is_every = true;
    for (size_t index = 0U; index < length; index++) { if (unit_poll(void_array_at(type, p_buffer, index)) == false) { is_every = false; } }
    return is_every;
}

/*!
    @return true if at least one return true
*/
static inline bool void_array_for_any(size_t type, void * p_buffer, size_t length, try_proc_t unit_poll)
{
    bool is_any = false;
    for (size_t index = 0U; index < length; index++) { if (unit_poll(void_array_at(type, p_buffer, index)) == true) { is_any = true; } }
    return is_any;
}

/*
    const and returns early
*/
static inline bool void_array_is_every(size_t type, const void * p_buffer, size_t length, test_t unit_test)
{
    bool is_every = true;
    for (size_t index = 0U; index < length; index++) { if (unit_test(void_array_at(type, p_buffer, index)) == false) { is_every = false; break; } }
    return is_every;
}

static inline bool void_array_is_any(size_t type, const void * p_buffer, size_t length, test_t unit_test)
{
    bool is_any = false;
    for (size_t index = 0U; index < length; index++) { if (unit_test(void_array_at(type, p_buffer, index)) == true) { is_any = true; break; } }
    return is_any;
}

/*
    register size value interface
    accessors with parameters of register width type
*/
static inline void void_array_foreach_set(size_t type, void * p_buffer, size_t length, set_t unit_setter, value_t value)
{
    for (size_t index = 0U; index < length; index++) { unit_setter(void_array_at(type, p_buffer, index), value); }
}

static inline bool void_array_for_every_set(size_t type, void * p_buffer, size_t length, try_set_t unit_try, value_t value)
{
    bool is_every = true;
    for (size_t index = 0U; index < length; index++) { if (unit_try(void_array_at(type, p_buffer, index), value) == false) { is_every = false; } }
    return is_every;
}

static inline bool void_array_for_any_set(size_t type, void * p_buffer, size_t length, try_set_t unit_try, value_t value)
{
    bool is_any = false;
    for (size_t index = 0U; index < length; index++) { if (unit_try(void_array_at(type, p_buffer, index), value) == true) { is_any = true; } }
    return is_any;
}

static inline bool void_array_is_every_value(size_t type, const void * p_buffer, size_t length, test_value_t test, value_t value)
{
    bool is_every = true;
    for (size_t index = 0U; index < length; index++) { if (test(void_array_at(type, p_buffer, index), value) == false) { is_every = false; break; } }
    return is_every;
}

static inline bool void_array_is_any_value(size_t type, const void * p_buffer, size_t length, test_value_t test, value_t value)
{
    bool is_any = false;
    for (size_t index = 0U; index < length; index++) { if (test(void_array_at(type, p_buffer, index), value) == true) { is_any = true; break; } }
    return is_any;
}

#define array_foreach_call(p_buffer, length, function, ...) \
    _Generic((function), \
        proc_t:   void_array_foreach,        \
        set_t:    void_array_foreach_set,    \
        default:  void_array_foreach         \
    )(sizeof(*(p_buffer)), p_buffer, length, function __VA_OPT__(,) __VA_ARGS__)


typedef int (*visitor2_t)(const void * p_context, value_t opt1, value_t opt2);

// static inline void _array_foreach(size_t type, void * p_buffer, size_t length, visitor2_t visitor, value_t value1, value_t value2)
// {
//     for (size_t index = 0U; index < length; index++) { visitor(void_array_at(type, p_buffer, index), value1, value2); }
// }
// static inline void void_array_foreach(size_t type, void * p_buffer, size_t length, proc_t unit_op)
// {
//     _array_foreach(type, p_buffer, length, (visitor2_t)unit_op, 0, 0);
// }



/******************************************************************************/
/*
    Value Array Iteration
*/
/******************************************************************************/
/*
    handles numeric types
*/
static inline void * void_array_min(size_t type, const void * p_buffer, size_t length)
{
    const void * p_min = p_buffer;
    void * p_unit;
    for (size_t index = 1U; index < length; index++)
    {
        p_unit = void_array_at(type, p_buffer, index);
        if (memcmp(p_unit, p_min, type) < 0) { p_min = p_unit; }
    }
    return (void *)p_min;
}

static inline void * void_array_max(size_t type, const void * p_buffer, size_t length)
{
    const void * p_max = p_buffer;
    void * p_unit;
    for (size_t index = 1U; index < length; index++)
    {
        p_unit = void_array_at(type, p_buffer, index);
        if (memcmp(p_unit, p_max, type) > 0) { p_max = p_unit; }
    }
    return (void *)p_max;
}

static inline value_t void_array_min_value(size_t type, const void * p_buffer, size_t length) { return void_pointer_as_value(type, void_array_min(type, p_buffer, length)); }
static inline value_t void_array_max_value(size_t type, const void * p_buffer, size_t length) { return void_pointer_as_value(type, void_array_max(type, p_buffer, length)); }

struct range { value_t min; value_t max; };

// static inline struct range void_array_min_max(size_t type, const void * p_buffer, size_t length)
// {
//     const void * p_min = p_buffer;
//     const void * p_max = p_buffer;
//     void * p_unit;
//     for (size_t index = 1U; index < length; index++)
//     {
//         p_unit = void_array_at(type, p_buffer, index);
//         if (memcmp(p_unit, p_min, type) < 0) { p_min = p_unit; }
//         else if (memcmp(p_unit, p_max, type) > 0) { p_max = p_unit; }
//     }
//     return (struct range){ .min = void_pointer_as_value(type, p_min), .max = void_pointer_as_value(type, p_max) };
// }


static inline int compare_int(const void * a, const void * b) { return (*(int *)a - *(int *)b); }

static inline void * void_array_min_with(size_t type, void * p_buffer, size_t length, compare_t compare)
{
    void * p_min = p_buffer;
    void * p_unit;
    for (size_t index = 1U; index < length; index++)
    {
        p_unit = void_array_at(type, p_buffer, index);
        if (compare(p_min, p_unit) > 0) { p_min = p_unit; }
    }
    return p_min;
}

static inline void * void_array_max_with(size_t type, void * p_buffer, size_t length, compare_t compare)
{
    void * p_max = p_buffer;
    void * p_unit;
    for (size_t index = 1U; index < length; index++)
    {
        p_unit = void_array_at(type, p_buffer, index);
        if (compare(p_max, p_unit) < 0) { p_max = p_unit; }
    }
    return p_max;
}

#endif // VOID_ARRAY_H
