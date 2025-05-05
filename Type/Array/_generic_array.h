#ifndef ARRAY_GENERIC_H
#define ARRAY_GENERIC_H


#include "../accessor.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

/*
    Generic array implementation - using Macros with _Generic selection for type safety

    Compile time typed
    effectively void array expressed with type prior to what compiler may optimize
    alternative to handling size parameter, may be faster depending on compiler.
*/

/******************************************************************************/
/*!
    @brief Caller cast void *
    type determined by calling function.
*/
/******************************************************************************/
/* shorthand *((T*)p_buffer) */
// static inline int8_t as_int8(const int8_t * p_buffer) { return p_buffer[0]; }
// static inline void int8_assign(int8_t * p_buffer, int8_t value) { p_buffer[0] = value; }
// static inline void int8_ptr_assign(int8_t * p_buffer, int8_t * value) { p_buffer[0] = *value; }

static inline int8_t array8_get(const int8_t * p_buffer, size_t index) { return p_buffer[index]; }
static inline void array8_set(int8_t * p_buffer, size_t index, int8_t value) { p_buffer[index] = value; }
static inline const int8_t * array8_at(const int8_t * p_buffer, size_t index) { return &p_buffer[index]; }


#define as(T, p_buffer) \
    _Generic((T)0, \
        int8_t  : *((int8_t *)p_buffer), \
        int16_t : *((int16_t *)p_buffer), \
        int32_t : *((int32_t *)p_buffer), \
        int8_t *: ((int8_t *)p_buffer) \
    )

#define _assign_as(T, p_buffer, p_value) \
    _Generic((T)0, \
        int8_t *:  (*as(T, p_buffer) = *as(T, p_value)), \
        uint8_t *: (*(uint8_t *)p_buffer = *(uint8_t *)p_value), \
        void **: (*p_buffer = *p_value), \
        default: memcpy(p_buffer, p_value, sizeof(T)) \
    )
    // uint16_t *: (*p_buffer = *p_value), \
    // uint32_t *: (*p_buffer = *p_value), \
    // uint64_t *: (*p_buffer = *p_value), \
    // int8_t *: (*p_buffer = *p_value), \
    // int16_t *: (*p_buffer = *p_value), \
    // int32_t *: (*p_buffer = *p_value), \
    // int64_t *: (*p_buffer = *p_value), \


// static inline int8_t as_int8_array_at(const int8_t * p_buffer, size_t index) { return p_buffer[index]; }

#define as_array(T, p_buffer, index) \
    _Generic((T)0, \
        int8_t  : ((int8_t *)p_buffer)[index], \
        int16_t : ((int16_t *)p_buffer)[index], \
        int32_t : ((int32_t *)p_buffer)[index], \
        int8_t * : &((int8_t *)p_buffer)[index] \
    )

/* cast the value, in case it is not the buffer type */
// #define _assign_as_array_value(T, p_buffer, index, value) \
//     _Generic((T)0, \
//         int8_t  : ((int8_t *)p_buffer), \
//         int16_t : ((int16_t *)p_buffer), \
//         int32_t : ((int32_t *)p_buffer)  \
//     )[index] = value

// #define _assign_as_array_ptr(T, p_buffer, index, p_value) \
//     _Generic((T)0, \
//         int8_t  *: ((int8_t *)p_buffer)[index] = *(intptr_t)p_value, \
//         int16_t *: ((int16_t *)p_buffer), \
//         int32_t *: ((int32_t *)p_buffer)  \
//     )


/* cast the value, in case it is not the end type */
// #define assign_as_array(T, p_buffer, index, value) \
//     _Generic((T)0, \
//         int8_t  : ((int8_t *)p_buffer)[index] = value, \
//         int16_t : ((int16_t *)p_buffer)[index] = value, \
//         int32_t : ((int32_t *)p_buffer)[index] = value, \
//         int8_t * : ((int8_t *)p_buffer)[index] = *((int8_t *)((intptr_t)value)) \
//     )

/* _assign_as_array_value */
#define _assign_as_array(T, p_buffer, index, value) (as_array(T, p_buffer, index) = value)



    // int8_t * : _assign_as_array_ptr(T, p_buffer, index, (intptr_t)value) \

// #define array_assign_as_value(p_buffer, index, value) array_assign_as(typeof(value), p_buffer, index, value)

// #define array_assign_as_value(p_buffer, index, value) \
//     _Generic((value), \
//         int8_t  : ((int8_t *)p_buffer)[index] = value, \
//         int8_t * : ((int8_t *)p_buffer)[index] = *value, \
//         int16_t : ((int16_t *)p_buffer)[index] = value, \
//         int32_t : ((int32_t *)p_buffer)[index] = value \
//     )



/******************************************************************************/
/*!
    @brief Typed/Value Array - primitive types only, copies value
    buffer must be typed for _Generic selection
*/
/******************************************************************************/

/* Short hand generation macro */
#define _ARRAY_FOREACH(p_typed, length, function) { for (size_t index = 0U; index < length; index++) { function(&value_array_at(p_typed, index)); } }

/* is there a lesser chance for size to optimize away? */
// #define _ARRAY_FOREACH(p_typed, length, function) { void_array_foreach(p_typed, sizeof(*p_typed), length, function); }

/* Typed function signitures. Individually selectable */
static inline void array_foreach_int8(int8_t * p_array, size_t length, proc_t unit_op) _ARRAY_FOREACH(p_array, length, unit_op)
static inline void array_foreach_ptr(void ** p_array, size_t length, proc_t unit_op) _ARRAY_FOREACH(p_array, length, unit_op)

/* Typed Array - _Generic wrap */
#define array_foreach(p_array, length, unit_op) \
    _Generic(p_array,                           \
        int8_t *: array_foreach_int8,    \
        void **: array_foreach_ptr       \
    )(p_array, length, unit_op)

static void proc_test(void * p_context){ }

static void test()
{
    void * buffer[10] = {0};
    void * buffer1[10] = {0};
    array_foreach(buffer1, 10, proc_test);
}


/*
    alternative to void_array.
    call directly or use a function code gen
*/
// #define ARRAY_LENGTH(TypedArray) (sizeof(TypedArray) / sizeof(TypedArray[0U]))

/*!
    (TypedArray)[index] accounts for unit size, TypedArray is typed
    TypedArray and unit_op mismatch will result in compiler warning
    alternatively, directly passing array can omit length

    @param[in] unit_op void(*unit_op)(T * p_unit)
*/
// #define ARRAY_FOREACH(TypedArray, unit_op) ({ for(size_t index = 0U; index < ARRAY_LENGTH(TypedArray); index++) { unit_op((TypedArray) + index); } })


// #define ARRAY_FOR_EVERY(TypedArray, unit_poll) \
//     ({ bool is_every = true; for (size_t index = 0U; index < ARRAY_LENGTH(TypedArray); index++) { if (unit_poll(((TypedArray) + index)) == false) { is_every = false; } } is_every; })

// #define ARRAY_FOR_ANY(TypedArray, unit_poll) \
//     ({ bool is_any = false; for (size_t index = 0U; index < ARRAY_LENGTH(TypedArray); index++) { if (unit_poll(((TypedArray) + index)) == true) { is_any = true; } } is_any; })

// // altnerntaively generate

// // #define ARRAY_FOR_EVERY_DEF(T)
// // static inline bool T##_array_for_every(T * p_buffer, size_t length, try_proc_t unit_poll)
// // {
// //     bool is_every = true;
// //     for (size_t index = 0U; index < length; index++) { if (unit_poll(p_buffer[index]) == false) { is_every = false; } }
// //     return is_every;
// // }

// // Using macros to handle struct type, accounts for [unit_size].
// #define _STRUCT_ARRAY_FOREACH_SET(p_structs, length, unit_set, unit_value) ({ for (size_t index = 0U; index < length; index++) { unit_set(((p_structs) + index), unit_value); } })

// #define _STRUCT_ARRAY_FOR_EVERY_SET(p_structs, length, unit_try, unit_value) ({ \
//     bool is_every = true; \
//     for (size_t index = 0U; index < length; index++) { if ( unit_try(((p_structs) + index), unit_value) == false) { is_every = false; } } \
//     is_every; \
// })

// #define _STRUCT_ARRAY_FOR_ANY_SET(p_structs, length, unit_try, unit_value) ({ \
//     bool is_any = false; \
//     for (size_t index = 0U; index < length; index++) { if ( unit_try(((p_structs) + index), unit_value) == true) { is_any = true; } } \
//     is_any; \
// })


// #define cast_accessor(unit_function) \
//     _Generic(unit_function, \
//         set_int8_t:   (set_int8_t)((void *)unit_function), \
//         set_int16_t:  (set_int16_t)((void *)unit_function), \
//         set_int32_t:  (set_int32_t)((void *)unit_function), \
//         set_uint8_t:  (set_uint8_t)((void *)unit_function), \
//         set_uint16_t: (set_uint16_t)((void *)unit_function), \
//         set_uint32_t: (set_uint32_t)((void *)unit_function)  \
//     )

/* function called through a non-compatible type */
// #define struct_array_foreach_set(p_structs, length, unit_setter, unit_value) _STRUCT_ARRAY_FOREACH_SET(p_structs, length, (unit_setter), unit_value)



#endif // ARRAY_GENERIC_H