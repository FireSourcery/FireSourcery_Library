#ifndef VOID_ARRAY_H
#define VOID_ARRAY_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "array_generic.h"

/******************************************************************************/
/*!
    @brief Void Array / Sized Array - Generic by unit_size
*/
/******************************************************************************/
/*!
    @brief Copy data from source to destination based on the size.
    @param dest Pointer to the destination buffer.
    @param src Pointer to the source buffer.
    @param size Size of the data to copy.
*/
static inline void void_copy(void * p_dest, const void * p_src, size_t unit_size)
{
    switch(unit_size)
    {
        case sizeof(uint8_t) : *((uint8_t *)p_dest) = *((const uint8_t *)p_src); break;
        case sizeof(uint16_t) : *((uint16_t *)p_dest) = *((const uint16_t *)p_src); break;
        case sizeof(uint32_t) : *((uint32_t *)p_dest) = *((const uint32_t *)p_src); break;
        case sizeof(uint64_t) : *((uint64_t *)p_dest) = *((const uint64_t *)p_src); break;
        default: memcpy(p_dest, p_src, unit_size); break;
    }
}

static inline void * void_pointer_at(const void * p_buffer, size_t unit_size, size_t unit_index)
{
    return ((uint8_t *)p_buffer + (unit_index * unit_size));
}


static inline void void_array_get(const void * p_buffer, size_t unit_size, size_t index, void * p_result)
{
    void_copy(p_result, void_pointer_at(p_buffer, unit_size, index), unit_size);
}

static inline void void_array_set(void * p_buffer, size_t unit_size, size_t index, const void * p_value)
{
    void_copy(void_pointer_at(p_buffer, unit_size, index), p_value, unit_size);
}


typedef void(*void_op_t)(void * p_unit);
typedef bool(*void_poll_t)(void * p_unit);

static inline void void_array_foreach(void * p_buffer, size_t unit_size, size_t length, void_op_t unit_op)
{
    for(size_t index = 0U; index < length; index++) { unit_op(void_pointer_at(p_buffer, unit_size, index)); }
}

/*!
    @return true if all are set
*/
static inline bool void_array_for_every(void * p_buffer, size_t unit_size, size_t length, void_poll_t unit_poll)
{
    bool is_every = true;
    for(size_t index = 0U; index < length; index++) { if(unit_poll(void_pointer_at(p_buffer, unit_size, index)) == false) { is_every = false; } }
    return is_every;
}

/*!
    @return true if at least one is set
*/
static inline bool void_array_for_any(void * p_buffer, size_t unit_size, size_t length, void_poll_t unit_poll)
{
    bool is_any = false;
    for(size_t index = 0U; index < length; index++) { if(unit_poll(void_pointer_at(p_buffer, unit_size, index)) == true) { is_any = true; } }
    return is_any;
}

/*
    const and returns early
*/
typedef bool(*void_test_t)(const void * p_unit);

static inline bool void_array_is_every(const void * p_buffer, size_t unit_size, size_t length, void_test_t unit_test)
{
    bool is_every = true;
    for(size_t index = 0U; index < length; index++) { if(unit_test(void_pointer_at(p_buffer, unit_size, index)) == false) { is_every = false; break; } }
    return is_every;
}

static inline bool void_array_is_any(const void * p_buffer, size_t unit_size, size_t length, void_test_t unit_test)
{
    bool is_any = false;
    for(size_t index = 0U; index < length; index++) { if(unit_test(void_pointer_at(p_buffer, unit_size, index)) == true) { is_any = true; break; } }
    return is_any;
}

/*
    handles numeric types
*/
static inline void * void_array_min(const void * p_buffer, size_t unit_size, size_t length)
{
    const void * p_min = p_buffer;
    void * p_unit;
    for (size_t index = 1U; index < length; index++)
    {
        p_unit = void_pointer_at(p_buffer, unit_size, index);
        if (memcmp(p_unit, p_min, unit_size) < 0) { p_min = p_unit; }
    }
    return (void *)p_min;
}

static inline void * void_array_max(const void * p_buffer, size_t unit_size, size_t length)
{
    const void * p_max = p_buffer;
    void * p_unit;
    for (size_t index = 1U; index < length; index++)
    {
        p_unit = void_pointer_at(p_buffer, unit_size, index);
        if (memcmp(p_unit, p_max, unit_size) > 0) { p_max = p_unit; }
    }
    return (void *)p_max;
}

/*  int assignment should handle sign extension */
static inline int void_as_int(const void * p_unit, size_t unit_size)
{
    int value = 0;
    switch (unit_size)
    {
        case sizeof(int8_t) : value = *((const int8_t *)p_unit); break;
        case sizeof(int16_t) : value = *((const int16_t *)p_unit); break;
        case sizeof(int32_t) : value = *((const int32_t *)p_unit); break;
        // case sizeof(int64_t) : value = *((const int64_t *)p_unit); break;
    }
    return value;
}

/* int cases less than sizeof(int) */
static inline int void_array_min_int(const void * p_buffer, size_t unit_size, size_t length)
{
    return void_as_int(void_array_min(p_buffer, unit_size, length), unit_size);
}

static inline int void_array_max_int(const void * p_buffer, size_t unit_size, size_t length)
{
    return void_as_int(void_array_max(p_buffer, unit_size, length), unit_size);
}

// static inline int compare_int(const void * a, const void * b) { return (*(int *)a - *(int *)b); }
// static inline int compare_int32(const void * a, const void * b) { return (*(int32_t *)a - *(int32_t *)b); }

// static inline void * void_array_min_with(void * p_buffer, size_t unit_size, size_t length, int (*compare)(const void *, const void *))
// {
//     void * p_min = p_buffer;
//     void * p_unit;
//     for(size_t index = 1U; index < length; index++)
//     {
//         p_unit = void_pointer_at(p_buffer, unit_size, index);
//         if(compare(p_min, p_unit) > 0) { p_min = p_unit; }
//     }
//     return p_min;
// }

// static inline void * void_array_max_with(void * p_buffer, size_t unit_size, size_t length, int (*compare)(const void *, const void *))
// {
//     void * p_max = p_buffer;
//     void * p_unit;
//     for(size_t index = 1U; index < length; index++)
//     {
//         p_unit = void_pointer_at(p_buffer, unit_size, index);
//         if(compare(p_max, p_unit) < 0) { p_max = p_unit; }
//     }
//     return p_max;
// }


#endif // VOID_ARRAY_H