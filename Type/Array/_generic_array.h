#ifndef ARRAY_GENERIC_H
#define ARRAY_GENERIC_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>


/*
    Generic array implementation - using Macros with _Generic selection for type safety

    Compile time typed
    effectively void array expressed with type prior to what compiler may optimize
    alternative to handling size parameter, may be faster depending on compiler.

    Macro buffer pointer assumed to be origin.
*/
/******************************************************************************/
/*!
    @brief Value Array - primitive types only, copies value
    buffer must be typed for _Generic selection
*/
/******************************************************************************/
#define _TYPED_COPY(p_dest, p_source) \
_Generic(p_dest, \
    uint8_t *: (*p_dest = *p_source), \
    uint16_t *: (*p_dest = *p_source), \
    uint32_t *: (*p_dest = *p_source), \
    uint64_t *: (*p_dest = *p_source), \
    int8_t *: (*p_dest = *p_source), \
    int16_t *: (*p_dest = *p_source), \
    int32_t *: (*p_dest = *p_source), \
    int64_t *: (*p_dest = *p_source), \
    void **: (*p_dest = *p_source), \
    default: memcpy(p_dest, p_src, sizeof(*p_dest)) \
)

/*
    base for type checking This way other functions do not need to define typed selections
    p_typed_values
*/
#define TYPED_ARRAY_GET(p_typed, index, p_result) (_TYPED_COPY(p_result, (p_typed + index)))
#define TYPED_ARRAY_SET(p_typed, index, p_value) (_TYPED_COPY((p_typed + index), p_value))
// #define array_copy(p_typed, p_source, length) ({ for (size_t index = 0U; index < length; index++) { _TYPED_COPY((p_typed + index), (p_source + index)); } })

/* value if value, pointer if struct  */
#define TYPED_ARRAY_ASSIGN(p_typed, index, value) \
_Generic(p_typed, \
    uint8_t *: p_typed[index] = value, \
    uint16_t *: p_typed[index] = value, \
    uint32_t *: p_typed[index] = value, \
    uint64_t *: p_typed[index] = value, \
    int8_t *: p_typed[index] = value, \
    int16_t *: p_typed[index] = value, \
    int32_t *: p_typed[index] = value, \
    int64_t *: p_typed[index] = value, \
    default: memcpy(&((p_typed)[index]), &value, sizeof(value)) \
)

// #define TYPED_ARRAY_SET(p_typed, index, p_value) (TYPED_ARRAY_ASSIGN(p_typed, index, p_value))



/*
    alternative to void_array.
    call directly or use a function code gen
*/
#define ARRAY_LENGTH(TypedArray) (sizeof(TypedArray) / sizeof(TypedArray[0U]))

/*!
    (TypedArray)[index] accounts for unit size, TypedArray is typed
    TypedArray and unit_op mismatch will result in compiler warning
    alternatively, directly passing array can omit length

    @param[in] unit_op void(*unit_op)(T * p_unit)
*/
#define ARRAY_FOREACH(TypedArray, unit_op) ({ for(size_t index = 0U; index < ARRAY_LENGTH(TypedArray); index++) { unit_op((TypedArray) + index); } })


#define ARRAY_FOR_EVERY(TypedArray, unit_poll) \
    ({ bool is_every = true; for (size_t index = 0U; index < ARRAY_LENGTH(TypedArray); index++) { if (unit_poll(((TypedArray) + index)) == false) { is_every = false; } } is_every; })

#define ARRAY_FOR_ANY(TypedArray, unit_poll) \
    ({ bool is_any = false; for (size_t index = 0U; index < ARRAY_LENGTH(TypedArray); index++) { if (unit_poll(((TypedArray) + index)) == true) { is_any = true; } } is_any; })

/*
    Typed Array.

    Wrapped version of array/value_array
    Macro generic - for primatives without swith_copy/memcpy
    call can wrap array without helper
    handle with _Generic.
*/
typedef const struct Array8 { uint8_t * const P_ARRAY; const size_t LENGTH; } Array8_T;
typedef const struct Array16 { uint16_t * const P_ARRAY; const size_t LENGTH; } Array16_T;
typedef const struct Array32 { uint32_t * const P_ARRAY; const size_t LENGTH; } Array32_T;
typedef const struct Array64 { uint64_t * const P_ARRAY; const size_t LENGTH; } Array64_T;
#define TYPED_ARRAY(T) const struct { T * const P_ARRAY; const size_t LENGTH; }

#define TYPED_ARRAY_INIT(p_buffer, length) { .P_ARRAY = (p_buffer), .LENGTH = (length), }
#define TYPED_ARRAY_INIT_UNNAMED(p_buffer, length) .P_ARRAY = (p_buffer), .LENGTH = (length)

#endif // ARRAY_GENERIC_H