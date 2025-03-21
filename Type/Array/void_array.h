#ifndef VOID_ARRAY_H
#define VOID_ARRAY_H

#include "../accessor.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <sys/types.h>


/******************************************************************************/
/*!
    @brief Void Array / Sized Array - Generic by type_size
*/
/******************************************************************************/
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
#if (__SIZEOF_POINTER__ >= 8)
        case sizeof(uint64_t) : *((uint64_t *)p_dest) = *((const uint64_t *)p_src); break;
#endif
        default: memcpy(p_dest, p_src, size); break;
    }
}

/*

*/
static inline void * void_pointer_at(const void * p_buffer, size_t type, size_t index) { return ((uint8_t *)p_buffer + (index * type)); }
static inline void void_pointer_assign(void * p_buffer, size_t type, const void * p_value) { void_copy(p_buffer, p_value, type); }
// static inline void void_pointer_assign(void * p_buffer, size_t type, const void * p_value) { memcpy(p_buffer, p_value, type); }


/*
    reister size value
    as value interface during access only, value upto register size
    ensure this is inline
*/
static inline value_t void_as_value(const void * p_unit, size_t type)
{
    value_t value = 0;
    switch (type)
    {
        case sizeof(int8_t):  value = *((const int8_t *)p_unit);  break;
        case sizeof(int16_t): value = *((const int16_t *)p_unit); break;
        case sizeof(int32_t): value = *((const int32_t *)p_unit); break;
#if (__SIZEOF_POINTER__ >= 8)
        case sizeof(int64_t): value = *((const int64_t *)p_unit); break;
#endif
        default: break;
    }
    return value;
}

static inline void void_assign_as_value(void * p_unit, size_t type, value_t value)
{
    switch (type)
    {
        case sizeof(int8_t):  *((int8_t *)p_unit)  = (int8_t)value;  break;
        case sizeof(int16_t): *((int16_t *)p_unit) = (int16_t)value; break;
        case sizeof(int32_t): *((int32_t *)p_unit) = (int32_t)value; break;
#if (__SIZEOF_POINTER__ >= 8)
        case sizeof(int64_t): *((int64_t *)p_unit) = (int64_t)value; break;
#endif
        default: break;
    }
}

/*
    array
    singlt unit at index
*/
static inline value_t void_array_at(const void * p_buffer, size_t type, size_t index)
{
    return void_as_value(void_pointer_at(p_buffer, type, index), type);
}

static inline void void_array_assign(void * p_buffer, size_t type, size_t index, value_t value)
{
    void_assign_as_value(void_pointer_at(p_buffer, type, index), type, value);
}

static inline void void_array_assign_from(void * p_buffer, size_t type, size_t index, const void * p_value)
{
    void_copy(void_pointer_at(p_buffer, type, index), p_value, type);
}

/*
    multiple units at start
*/
// static inline void void_copy_to(const void * p_buffer, size_t type, size_t count, void * p_result)
static inline void void_array_copy_to(const void * p_buffer, size_t type, void * p_results, size_t count) { memcpy(p_results, p_buffer, type * count); }
static inline void void_array_copy_from(void * p_buffer, size_t type, const void * p_units, size_t count) { memcpy(p_buffer, p_units, type * count); }


/*
    0 argument accessors
*/
/*
    length in units
*/
static inline void void_array_foreach(void * p_buffer, size_t type_size, size_t length, proc_t unit_op)
{
    for (size_t index = 0U; index < length; index++) { unit_op(void_pointer_at(p_buffer, type_size, index)); }
}


/*!
    foreach_is_every
    @return true if all return true
*/
static inline bool void_array_for_every(void * p_buffer, size_t type_size, size_t length, poll_t unit_poll)
{
    bool is_every = true;
    for (size_t index = 0U; index < length; index++) { if (unit_poll(void_pointer_at(p_buffer, type_size, index)) == false) { is_every = false; } }
    return is_every;
}

/*!
    @return true if at least one return true
*/
static inline bool void_array_for_any(void * p_buffer, size_t type_size, size_t length, poll_t unit_poll)
{
    bool is_any = false;
    for (size_t index = 0U; index < length; index++) { if (unit_poll(void_pointer_at(p_buffer, type_size, index)) == true) { is_any = true; } }
    return is_any;
}

/*
    const and returns early
*/
static inline bool void_array_is_every(const void * p_buffer, size_t type_size, size_t length, test_t unit_test)
{
    bool is_every = true;
    for (size_t index = 0U; index < length; index++) { if (unit_test(void_pointer_at(p_buffer, type_size, index)) == false) { is_every = false; break; } }
    return is_every;
}

static inline bool void_array_is_any(const void * p_buffer, size_t type_size, size_t length, test_t unit_test)
{
    bool is_any = false;
    for (size_t index = 0U; index < length; index++) { if (unit_test(void_pointer_at(p_buffer, type_size, index)) == true) { is_any = true; break; } }
    return is_any;
}


/*
    accessors with parameters of register width type
    using register size as parameter interface
*/
static inline void void_array_foreach_set(void * p_buffer, size_t type_size, size_t length, set_t unit_setter, value_t value)
{
    for (size_t index = 0U; index < length; index++) { unit_setter(void_pointer_at(p_buffer, type_size, index), value); }
}

static inline bool void_array_for_every_try(void * p_buffer, size_t type_size, size_t length, try_t unit_try, value_t value)
{
    bool is_every = true;
    for (size_t index = 0U; index < length; index++) { if (unit_try(void_pointer_at(p_buffer, type_size, index), value) == false) { is_every = false; } }
    return is_every;
}

static inline bool void_array_for_any_try(void * p_buffer, size_t type_size, size_t length, try_t unit_try, value_t value)
{
    bool is_any = false;
    for (size_t index = 0U; index < length; index++) { if (unit_try(void_pointer_at(p_buffer, type_size, index), value) == true) { is_any = true; } }
    return is_any;
}


// static inline bool switch_compare(void * p_dest, const void * p_src, size_t type_size)
/*
    handles numeric types
*/
static inline void * void_array_min(const void * p_buffer, size_t type_size, size_t length)
{
    const void * p_min = p_buffer;
    void * p_unit;
    for (size_t index = 1U; index < length; index++)
    {
        p_unit = void_pointer_at(p_buffer, type_size, index);
        if (memcmp(p_unit, p_min, type_size) < 0) { p_min = p_unit; }
    }
    return (void *)p_min;
}

static inline void * void_array_max(const void * p_buffer, size_t type_size, size_t length)
{
    const void * p_max = p_buffer;
    void * p_unit;
    for (size_t index = 1U; index < length; index++)
    {
        p_unit = void_pointer_at(p_buffer, type_size, index);
        if (memcmp(p_unit, p_max, type_size) > 0) { p_max = p_unit; }
    }
    return (void *)p_max;
}



/* int cases less than sizeof(int) */
static inline value_t void_array_min_value(const void * p_buffer, size_t type_size, size_t length)
{
    return void_as_value(void_array_min(p_buffer, type_size, length), type_size);
}

static inline value_t void_array_max_value(const void * p_buffer, size_t type_size, size_t length)
{
    return void_as_value(void_array_max(p_buffer, type_size, length), type_size);
}

// static inline int compare_int(const void * a, const void * b) { return (*(int *)a - *(int *)b); }

// static inline void * void_array_min_with(void * p_buffer, size_t type_size, size_t length, int (*compare)(const void *, const void *))
// {
//     void * p_min = p_buffer;
//     void * p_unit;
//     for (size_t index = 1U; index < length; index++)
//     {
//         p_unit = void_pointer_at(p_buffer, type_size, index);
//         if (compare(p_min, p_unit) > 0) { p_min = p_unit; }
//     }
//     return p_min;
// }

// static inline void * void_array_max_with(void * p_buffer, size_t type_size, size_t length, int (*compare)(const void *, const void *))
// {
//     void * p_max = p_buffer;
//     void * p_unit;
//     for (size_t index = 1U; index < length; index++)
//     {
//         p_unit = void_pointer_at(p_buffer, type_size, index);
//         if (compare(p_max, p_unit) < 0) { p_max = p_unit; }
//     }
//     return p_max;
// }





#endif // VOID_ARRAY_H