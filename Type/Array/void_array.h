#ifndef VOID_ARRAY_H
#define VOID_ARRAY_H

#include "array_generic.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

typedef void(*void_op_t)(void * p_unit);
typedef bool(*void_poll_t)(void * p_unit);
typedef bool(*void_test_t)(const void * p_unit);

/******************************************************************************/
/*!
    @brief Void Array / Sized Array - Generic by type_size
*/
/******************************************************************************/
/*!
    @brief Copy data from source to destination based on the size.
    @param dest Pointer to the destination buffer.
    @param src Pointer to the source buffer.
    @param size Size of the data to copy.
*/
static inline void void_copy(void * p_dest, const void * p_src, size_t type_size)
{
    switch(type_size)
    {
        case sizeof(uint8_t) : *((uint8_t  *)p_dest) = *((const uint8_t  *)p_src); break;
        case sizeof(uint16_t): *((uint16_t *)p_dest) = *((const uint16_t *)p_src); break;
        case sizeof(uint32_t): *((uint32_t *)p_dest) = *((const uint32_t *)p_src); break;
        case sizeof(uint64_t): *((uint64_t *)p_dest) = *((const uint64_t *)p_src); break;
        default: memcpy(p_dest, p_src, type_size); break;
    }
}

static inline void * void_pointer_at(const void * p_buffer, size_t type_size, size_t unit_index)
{
    return ((uint8_t *)p_buffer + (unit_index * type_size));
}

static inline void void_array_get(const void * p_buffer, size_t type_size, size_t index, void * p_result)
{
    void_copy(p_result, void_pointer_at(p_buffer, type_size, index), type_size);
}

static inline void void_array_set(void * p_buffer, size_t type_size, size_t index, const void * p_value)
{
    void_copy(void_pointer_at(p_buffer, type_size, index), p_value, type_size);
}



/*
    length in units
*/
static inline void void_array_foreach(void * p_buffer, size_t type_size, size_t length, void_op_t unit_op)
{
    for(size_t index = 0U; index < length; index++) { unit_op(void_pointer_at(p_buffer, type_size, index)); }
}


/*!
    @return true if all return true
*/
static inline bool void_array_for_every(void * p_buffer, size_t type_size, size_t length, void_poll_t unit_poll)
{
    bool is_every = true;
    for (size_t index = 0U; index < length; index++) { if (unit_poll(void_pointer_at(p_buffer, type_size, index)) == false) { is_every = false; } }
    return is_every;
}

/*!
    @return true if at least one return true
*/
static inline bool void_array_for_any(void * p_buffer, size_t type_size, size_t length, void_poll_t unit_poll)
{
    bool is_any = false;
    for (size_t index = 0U; index < length; index++) { if (unit_poll(void_pointer_at(p_buffer, type_size, index)) == true) { is_any = true; } }
    return is_any;
}

/*
    const and returns early
*/
static inline bool void_array_is_every(const void * p_buffer, size_t type_size, size_t length, void_test_t unit_test)
{
    bool is_every = true;
    for (size_t index = 0U; index < length; index++) { if (unit_test(void_pointer_at(p_buffer, type_size, index)) == false) { is_every = false; break; } }
    return is_every;
}

static inline bool void_array_is_any(const void * p_buffer, size_t type_size, size_t length, void_test_t unit_test)
{
    bool is_any = false;
    for (size_t index = 0U; index < length; index++) { if (unit_test(void_pointer_at(p_buffer, type_size, index)) == true) { is_any = true; break; } }
    return is_any;
}

// typedef void (*set_register_t)(void * p_struct, register_t value);
// typedef bool (*try_register_t)(void * p_struct, register_t value);

// static inline void void_array_foreach_set(const void * p_buffer, size_t unit_size, size_t length, set_register_t unit_setter, register_t value)
// {
//     for (size_t index = 0U; index < length; index++) { unit_setter(void_pointer_at(p_buffer, unit_size, index), value); }
// }

// static inline bool void_array_for_every_set(const void * p_buffer, size_t unit_size, size_t length, try_register_t unit_try, register_t value)
// {
//     bool is_every = true;
//     for (size_t index = 0U; index < length; index++) { if (unit_try(void_pointer_at(p_buffer, unit_size, index), value) == false) { is_every = false; } }
//     return is_every;
// }
// static inline bool void_array_for_any_set(const void * p_buffer, size_t unit_size, size_t length, try_register_t unit_try, register_t value)
// {
//     bool is_any = false;
//     for (size_t index = 0U; index < length; index++) { if (unit_try(void_pointer_at(p_buffer, unit_size, index), value) == true) { is_any = true; } }
//     return is_any;
// }


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

/*  int assignment should handle sign extension */
static inline int void_as_int(const void * p_unit, size_t type_size)
{
    int value = 0;
    switch (type_size)
    {
        case sizeof(int8_t):  value = *((const int8_t *)p_unit);  break;
        case sizeof(int16_t): value = *((const int16_t *)p_unit); break;
        case sizeof(int32_t): value = *((const int32_t *)p_unit); break;
        // case sizeof(int64_t) : value = *((const int64_t *)p_unit); break;
    }
    return value;
}

/* int cases less than sizeof(int) */
static inline int void_array_min_int(const void * p_buffer, size_t type_size, size_t length)
{
    return void_as_int(void_array_min(p_buffer, type_size, length), type_size);
}

static inline int void_array_max_int(const void * p_buffer, size_t type_size, size_t length)
{
    return void_as_int(void_array_max(p_buffer, type_size, length), type_size);
}

// static inline int compare_int(const void * a, const void * b) { return (*(int *)a - *(int *)b); }
// static inline int compare_int32(const void * a, const void * b) { return (*(int32_t *)a - *(int32_t *)b); }

// static inline void * void_array_min_with(void * p_buffer, size_t type_size, size_t length, int (*compare)(const void *, const void *))
// {
//     void * p_min = p_buffer;
//     void * p_unit;
//     for(size_t index = 1U; index < length; index++)
//     {
//         p_unit = void_pointer_at(p_buffer, type_size, index);
//         if(compare(p_min, p_unit) > 0) { p_min = p_unit; }
//     }
//     return p_min;
// }

// static inline void * void_array_max_with(void * p_buffer, size_t type_size, size_t length, int (*compare)(const void *, const void *))
// {
//     void * p_max = p_buffer;
//     void * p_unit;
//     for(size_t index = 1U; index < length; index++)
//     {
//         p_unit = void_pointer_at(p_buffer, type_size, index);
//         if(compare(p_max, p_unit) < 0) { p_max = p_unit; }
//     }
//     return p_max;
// }


#endif // VOID_ARRAY_H