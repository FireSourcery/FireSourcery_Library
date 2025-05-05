// #ifndef ARRAY_UTILITY_H
// #define ARRAY_UTILITY_H

// #include <stdint.h>
// #include <stdbool.h>
// #include <stddef.h>
// #include <string.h>

// #include "void_array.h"
// #include "struct_array.h"


// /*!
//     @brief Encapsulated wrap of void_array
// */

// /*
//     Array metadata
//     a structure to hold a pointer to an p_array buffer and its length
//     Generic array implementation using Unit Size. Run time generic.
// */
// // typedef struct array_type { size_t type; size_t length; } array_type_t;
// // typedef struct array_part { void * const p_array; size_t length; } array_part_t;

// typedef const struct ArrayMeta
// {
//     const size_t UNIT_SIZE; // or TYPE_SIZE, Size of each element in the p_array
//     const size_t LENGTH;    // Length of the p_array
//     void * const P_BUFFER;  // Pointer to the p_array buffer
// }
// Array_T;

// /*!
//     @brief Macro to initialize the Array_T structure.
//     @param p_buffer Pointer to the p_array buffer.
//     @param unitSize Size of each element in the p_array.
//     @param length Length of the p_array.
// */
// #define ARRAY_INIT(p_buffer, unitSize, length) { .P_BUFFER = (p_buffer), .UNIT_SIZE = (unitSize), .LENGTH = (length) }
// #define ARRAY_ALLOC(unitSize, length) ARRAY_ALLOC((uint8_t[(unitSize) * (length)]){}, unitSize, length)
// #define ARRAY_ALLOC_AS(T, length) ARRAY_ALLOC_AS(sizeof(T), length)

// #define ARRAY_ALLOC(UnitSize, Length) BUFFER_ALLOC((UnitSize)*(Length))
// #define ARRAY_ALLOC_AS(T, Length) ((void *)(T[(Length)]){})

// /*!
//     Protected
// */
// // static inline void * _Array_At(const array_type_t type, void * p_buffer, size_t index) { return void_pointer_at(p_buffer, type.type, index); }
// static inline void * _Array_At(const Array_T * p_array, size_t index)                       { return void_pointer_at(p_array->P_BUFFER, p_array->UNIT_SIZE, index); }
// // static inline void _Array_Get(const Array_T * p_array, size_t index, void * p_value)        { void_array_get(p_array->P_BUFFER, p_array->UNIT_SIZE, index, p_value); }
// // static inline void _Array_Set(const Array_T * p_array, size_t index, const void * p_value)  { void_array_set(p_array->P_BUFFER, p_array->UNIT_SIZE, index, p_value); }

// // /*!
// //     @brief Get the p_value of an element at a specific index.
// //     @param p_array Pointer to the Array_T structure.
// //     @param index Index of the element to get.
// //     @param p_value Pointer to store the retrieved p_value.
// //     @return True if the index is valid and the p_value was retrieved, false otherwise.
// // */
// // static inline bool Array_Get(const Array_T * p_array, size_t index, void * p_value)
// // {
// //     if(index >= p_array->LENGTH) { return false; }
// //     _Array_Get(p_array, index, p_value);
// //     return true;
// // }

// // /*!
// //     @brief Set the p_value of an element at a specific index.
// //     @param p_array Pointer to the Array_T structure.
// //     @param index Index of the element to set.
// //     @param p_value Pointer to the p_value to set.
// //     @return True if the index is valid and the p_value was set, false otherwise.
// // */
// // static inline bool Array_Set(const Array_T * p_array, size_t index, const void * p_value)
// // {
// //     if(index >= p_array->LENGTH) { return false; }
// //     _Array_Set(p_array, index, p_value);
// //     return true;
// // }

// static inline void Array_ForEach(const Array_T * p_array, proc_t unit_op)
// {
//     void_array_foreach(p_array->P_BUFFER, p_array->UNIT_SIZE, p_array->LENGTH, unit_op);
// }

// static inline void Array_SetEach(const Array_T * p_array, set_t unit_op, intptr_t value)
// {
//     void_array_foreach_set(p_array->P_BUFFER, p_array->UNIT_SIZE, p_array->LENGTH, unit_op, value);
// }



// /*
//     Typed Array.
//       predefine value types

//     Wrapped version of array/value_array
//     Macro generic - for primatives without swith_copy/memcpy
//     call can wrap array without helper
//     handle with _Generic.
// */
// typedef const struct Array8 { uint8_t * const P_ARRAY; const size_t LENGTH; } Array8_T;
// typedef const struct Array16 { uint16_t * const P_ARRAY; const size_t LENGTH; } Array16_T;
// typedef const struct Array32 { uint32_t * const P_ARRAY; const size_t LENGTH; } Array32_T;
// typedef const struct Array64 { uint64_t * const P_ARRAY; const size_t LENGTH; } Array64_T;
// typedef const struct _Array { void * const P_ARRAY; const size_t LENGTH; } _Array_T;
// typedef const struct ArrayPtr { void ** const P_ARRAY; const size_t LENGTH; } ArrayPtr_T;

// // typedef const struct Array
// // {
//     //     const size_t LENGTH;
//     //     T const Array[];
// // }
// // Array_T;

// // static inline void Array_ForEach(Array_T * p_array, size_t length, proc_t function)
// // {
// //     // void_array_foreach(p_array->P_ARRAY, sizeof(int8_t), p_array->LENGTH, function);
// //     for (size_t index = 0U; index < length; index++) { function(&p_array->Array[index]); }
// // }

// #define TYPED_ARRAY(T) const struct { T * const P_ARRAY; const size_t LENGTH; }

// #define TYPED_ARRAY_INIT(p_buffer, length) { .P_ARRAY = (p_buffer), .LENGTH = (length), }
// #define TYPED_ARRAY_INIT_UNNAMED(p_buffer, length) .P_ARRAY = (p_buffer), .LENGTH = (length)

// #define TYPED_ARRAY_ALLOC(T, length) ((T[length]){ })

// // static inline void Array8_ForEach(uint8_t * p_array, size_t length, proc_t function)
// // {
// //     // void_array_foreach(p_array->P_ARRAY, sizeof(int8_t), p_array->LENGTH, function);
// //     for (size_t index = 0U; index < length; index++) { function(&p_array[index]); }
// // }

// // static inline void Array8_ForEach(const Array8_T * p_array, proc_t function)
// // {
// //     for (size_t index = 0U; index < p_array->LENGTH; index++) { function(&p_array->P_ARRAY[index]); }
// // }

// // static inline void Array_ForEach_As8(const _Array_T * p_array, proc_t unit_op)
// // {
// //     Array8_ForEach((Array8_T *)p_array, unit_op);
// // }

// // #define Array_ForEach_As(p_array, unit_op); \
// //     _Generic(p_array, \
// //         uint8_t * : Array_ForEach_As8, \
// //         default : Array_ForEach \
// //     )(p_array, unit_op)

// #endif // ARRAY_H


