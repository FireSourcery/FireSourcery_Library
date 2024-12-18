#ifndef ARRAY_GENERIC_H
#define ARRAY_GENERIC_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

/*
    Generic array implementation - using Macros with _Generic selection for type safety

    Compile time typed
    alternative to handling size parameter, may be faster depending on compiler.

    Macro buffer pointer assumed to be origin.
*/
/******************************************************************************/
/*!
    @brief Value Array - primitive types only, copies value
    buffer must be typed for _Generic selection (NOT void*)
*/
/******************************************************************************/
// #define value_array_get(p_values, index, p_result) (*p_result = ((p_values) + index))
// #define value_array_set(p_values, index, p_value) (*((p_values) + index) = *p_value)

static inline void uint8_copy(uint8_t * p_dest, const uint8_t * p_source) { *p_dest = *p_source; }
static inline void uint16_copy(uint16_t * p_dest, const uint16_t * p_source) { *p_dest = *p_source; }
static inline void uint32_copy(uint32_t * p_dest, const uint32_t * p_source) { *p_dest = *p_source; }
static inline void uint64_copy(uint64_t * p_dest, const uint64_t * p_source) { *p_dest = *p_source; }
// match memcpy signature
// static inline void uint8_copy(uint8_t * p_dest, const uint8_t * p_source, size_t _void) { *p_dest = *p_source; }
// static inline void uint16_copy(uint16_t * p_dest, const uint16_t * p_source, size_t _void) { *p_dest = *p_source; }
// static inline void uint32_copy(uint32_t * p_dest, const uint32_t * p_source, size_t _void) { *p_dest = *p_source; }
// static inline void uint64_copy(uint64_t * p_dest, const uint64_t * p_source, size_t _void) { *p_dest = *p_source; }

// typed_value_copy
#define _INT_COPY(p_dest, p_source) \
_Generic(p_dest, \
    uint8_t *: uint8_copy, \
    uint16_t *: uint16_copy, \
    uint32_t *: uint32_copy, \
    uint64_t *: uint64_copy, \
    int8_t *: uint8_copy, \
    int16_t *: uint16_copy, \
    int32_t *: uint32_copy, \
    int64_t *: uint64_copy \
)(p_dest, p_source)

/*
    base for type checking This way other functions do not need to define typed selections
    p_typed_values
*/
#define TYPED_ARRAY_GET(p_typed, index, p_result) (_INT_COPY(p_result, (p_typed + index)))
#define TYPED_ARRAY_SET(p_typed, index, p_value) (_INT_COPY((p_typed + index), p_value))
// #define array_copy(p_typed, p_source, length) ({ for (size_t index = 0U; index < length; index++) { _int_copy((p_typed + index), (p_source + index)); } })


// addition casting for safety? caller can handle directly
// static inline uint8_t uint8_get(const uint8_t * p_source, size_t index) { return  p_source[index]; }
// static inline uint16_t uint16_get(const uint16_t * p_source, size_t index) { return p_source[index]; }
// static inline uint32_t uint32_get(const uint32_t * p_source, size_t index) { return p_source[index]; }
// static inline uint64_t uint64_get(const uint64_t * p_source, size_t index) { return p_source[index]; }
// #define value_array_at(p_typed, index) \
// _Generic(p_typed, \
//     uint8_t *: uint8_get, \
//     uint16_t *: uint16_get, \
//     uint32_t *: uint32_get, \
//     uint64_t *: uint64_get, \
//     int8_t *: uint8_get, \
//     int16_t *: uint16_get, \
//     int32_t *: uint32_get, \
//     int64_t *: uint64_get \
// )

// #define value_array_assign(p_typed, index, value) (_int_copy(&((p_typed)[index]), &value))

// typedef void(*void_op_t)(void * p_unit);
typedef void(*uint8_op_t)(uint8_t * p_value);
typedef void(*uint16_op_t)(uint16_t * p_value);
typedef void(*uint32_op_t)(uint32_t * p_value);
typedef void(*uint64_op_t)(uint64_t * p_value);


/*
    alternative to void_array.
    call directly or use a function code gen
*/


#define ARRAY_LENGTH(TypedArray) (sizeof(TypedArray) / sizeof(TypedArray[0U]))

/*!
    (TypedArray)[index] accounts for unit size, TypedArray is typed
    TypedArray and unit_op mismatch will result in compiler warning
    alternatively, directly passing array can omit length

    @param[in] unit_op void(*unit_op)(  * p_unit)
*/
#define ARRAY_FOREACH(TypedArray, unit_op) ({ for(size_t index = 0U; index < ARRAY_LENGTH(TypedArray); index++) { unit_op((TypedArray) + index); } })

// typedef bool(*void_test_t)(const void * p_unit);
// typedef bool(*void_poll_t)(void * p_unit);

#define ARRAY_FOR_EVERY(TypedArray, unit_poll) \
    ({ bool is_every = true; for (size_t index = 0U; index < ARRAY_LENGTH(TypedArray); index++) { if (unit_poll(((TypedArray) + index)) == false) { is_every = false; } } is_every; })

#define ARRAY_FOR_ANY(TypedArray, unit_poll) \
    ({ bool is_any = false; for (size_t index = 0U; index < ARRAY_LENGTH(TypedArray); index++) { if (unit_poll(((TypedArray) + index)) == true) { is_any = true; } } is_any; })

// #define array_is_every(TypedArray, unit_test) ({ \
//     bool result = true; \
//     for (size_t index = 0U; index < ARRAY_LENGTH(TypedArray); index++) { if (unit_test((TypedArray + index))) { result = false; break; } } \
//     result; \
// })

// #define array_is_any(TypedArray, unit_test) ({ \
//     bool result = true; \
//     for (size_t index = 0U; index < ARRAY_LENGTH(TypedArray); index++) { if (unit_test((TypedArray + index))) { result = false; break; } } \
//     result; \
// })

// add cast after TypedArray[index]?
// #define array_int_min(TypedArray, length) ({ \
//     int min_value = TypedArray[0U]; \
//     for (size_t index = 1U; index < ARRAY_LENGTH(TypedArray); index++) { if (TypedArray[index] < min_value) { min_value = TypedArray[index]; } } \
//     min_value; \
// })




#endif // ARRAY_GENERIC_H