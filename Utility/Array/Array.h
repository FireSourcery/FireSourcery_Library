#ifndef ARRAY_UTILITY_H
#define ARRAY_UTILITY_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "array_generic.h"
#include "void_array.h"
#include "struct_array.h"


/*!
    @brief Encapsulated wrap of void_array
*/

/*
    Array metadata
    a structure to hold a pointer to an p_array buffer and its length
    Generic array implementation using Unit Size. Run time generic.
*/
typedef const struct ArrayMeta
{
    void * const P_BUFFER;  // Pointer to the p_array buffer
    const size_t LENGTH;    // Length of the p_array
    const size_t UNIT_SIZE; // Size of each element in the p_array
}
Array_T;

/*!
    @brief Macro to initialize the Array_T structure.
    @param buffer Pointer to the p_array buffer.
    @param unitSize Size of each element in the p_array.
    @param length Length of the p_array.
*/
#define ARRAY_INIT(buffer, length, unitSize) { \
    .UNIT_SIZE = (unitSize), \
    .P_BUFFER = (buffer), \
    .LENGTH = (length) \
}

#define ARRAY_NEW(length, unitSize) { \
    .UNIT_SIZE = (unitSize), \
    .P_BUFFER = &(Array_T){0}, \
    .LENGTH = (length) \
}

#define ARRAY_INIT_AS(T, buffer, length) { \
    .P_BUFFER = (buffer), \
    .UNIT_SIZE = sizeof(T), \
    .LENGTH = (length) \
}

/*!
    Protected
*/
static inline void * _Array_At(const Array_T * p_array, size_t index)                       { return void_pointer_at(p_array->P_BUFFER, p_array->UNIT_SIZE, index); }
static inline void _Array_Get(const Array_T * p_array, size_t index, void * p_value)        { void_array_get(p_array->P_BUFFER, p_array->UNIT_SIZE, index, p_value); }
static inline void _Array_Set(const Array_T * p_array, size_t index, const void * p_value)  { void_array_set(p_array->P_BUFFER, p_array->UNIT_SIZE, index, p_value); }

/*!
    @brief Get the p_value of an element at a specific index.
    @param p_array Pointer to the Array_T structure.
    @param index Index of the element to get.
    @param p_value Pointer to store the retrieved p_value.
    @return True if the index is valid and the p_value was retrieved, false otherwise.
*/
static inline bool Array_Get(const Array_T * p_array, size_t index, void * p_value)
{
    if(index >= p_array->LENGTH) { return false; }
    _Array_Get(p_array, index, p_value);
    return true;
}

/*!
    @brief Set the p_value of an element at a specific index.
    @param p_array Pointer to the Array_T structure.
    @param index Index of the element to set.
    @param p_value Pointer to the p_value to set.
    @return True if the index is valid and the p_value was set, false otherwise.
*/
static inline bool Array_Set(const Array_T * p_array, size_t index, const void * p_value)
{
    if(index >= p_array->LENGTH) { return false; }
    _Array_Set(p_array, index, p_value);
    return true;
}

static inline void Array_ForEach(const Array_T * p_array, void_op_t unit_op)
{
    void_array_foreach(p_array->P_BUFFER, p_array->UNIT_SIZE, p_array->LENGTH, unit_op);
}


static inline void Array_SetAll_Int32(const Array_T * p_array, set_int32_t unit_op, int32_t value)
{
    struct_array_set_all_int32(p_array->P_BUFFER, p_array->UNIT_SIZE, p_array->LENGTH, unit_op, value);
}


/*
    Value Array.
    Wrapped version of array/value_array
    Macro generic - for primatives without swith_copy/memcpy
    call can wrap array without helper
    handle with _Generic.
*/
typedef const struct Array8 { uint8_t * const P_ARRAY; const size_t LENGTH; } Array8_T;
typedef const struct Array16 { uint16_t * const P_ARRAY; const size_t LENGTH; } Array16_T;
typedef const struct Array32 { uint32_t * const P_ARRAY; const size_t LENGTH; } Array32_T;
typedef const struct Array64 { uint64_t * const P_ARRAY; const size_t LENGTH; } Array64_T;
#define VALUE_ARRAY(T) const struct { T * const P_ARRAY; const size_t LENGTH; }

#define VALUE_ARRAY_INIT(p_buffer, length) { .P_ARRAY = (p_buffer), .LENGTH = (length), }
#define VALUE_ARRAY_INIT_UNNAMED(p_buffer, length) .P_ARRAY = (p_buffer), .LENGTH = (length)



#endif // ARRAY_H


