#ifndef STRUCT_ARRAY_H
#define STRUCT_ARRAY_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "array_generic.h"
#include "void_array.h"

/******************************************************************************/
/*!
    @brief Struct Array - typed getters/setters
    Extension of void_array.
    In most cases structs will have pre defined getters/setters.
    Collectively defined with various typed handlers.
        Otherwise caller would need to wrap established getters/setters with void handlers.
        typedef void (*set_void_t)(void * p_unit, void * p_value);
*/
/******************************************************************************/
typedef int32_t(*get_int32_t)(const void * p_struct);

/* Both signed and unsigned versions are needed. Casting parameter sign will result in compiler warning */
typedef void (*set_int8_t)(void * p_struct, int8_t value);
typedef void (*set_int16_t)(void * p_struct, int16_t value);
typedef void (*set_int32_t)(void * p_struct, int32_t value);
typedef void (*set_uint8_t)(void * p_struct, uint8_t value);
typedef void (*set_uint16_t)(void * p_struct, uint16_t value);
typedef void (*set_uint32_t)(void * p_struct, uint32_t value);

typedef bool (*try_int8_t)(void * p_struct, int8_t value);
typedef bool (*try_int16_t)(void * p_struct, int16_t value);
typedef bool (*try_int32_t)(void * p_struct, int32_t value);
typedef bool (*try_uint8_t)(void * p_struct, uint8_t value);
typedef bool (*try_uint16_t)(void * p_struct, uint16_t value);
typedef bool (*try_uint32_t)(void * p_struct, uint32_t value);


// typedef void (*set_t)(void * p_struct, register_t value);
// typedef register_t (*get_t)(void * p_struct, register_t value);

// typedef void (*set_entry_t)(void * p_struct, size_t key, register_t value);
// typedef register_t(*get_entry_t)(void * p_struct, size_t key);

/******************************************************************************/
/*!
    Macros version
    Using macros to handle struct type, accounts for [unit_size].
*/
/******************************************************************************/
// #define struct_array_foreach array_foreach

#define struct_array_set_all(p_typed_struct, length, unit_set, unit_value) ({ for (size_t index = 0U; index < length; index++) { unit_set(((p_typed_struct) + index), unit_value); } })

#define struct_array_is_every_set(p_typed_struct, length, unit_try, unit_value) ({ \
    bool is_every = true; \
    for(size_t index = 0U; index < length; index++) { if ( unit_try(((p_typed_struct) + index), unit_value) == false) { is_every = false; } } \
    is_every; \
})

#define struct_array_is_any_set(p_typed_struct, length, unit_try, unit_value) ({ \
    bool is_any = false; \
    for(size_t index = 0U; index < length; index++) { if ( unit_try(((p_typed_struct) + index), unit_value) == true) { is_any = true; } } \
    is_any; \
})

// #define STRUCT_ARRAY_IS_EVERY_SET_GEN(STRUCT_TYPE, VALUE_TYPE, p_typed_struct, length, unit_try, unit_value) struct_array_is_every_set(p_typed_struct, length, (set_##VALUE_TYPE)unit_try, (VALUE_TYPE)unit_value)

/******************************************************************************/
        // ensure calling convention is correct for function pointer containing a typed value parameter
/******************************************************************************/
static inline void struct_array_set_all_int32(const void * p_buffer, size_t unit_size, size_t length, set_int32_t unit_setter, int32_t value)
    { for (size_t index = 0U; index < length; index++) { unit_setter(void_pointer_at(p_buffer, unit_size, index), value); } }

static inline void struct_array_set_all_int16(const void * p_buffer, size_t unit_size, size_t length, set_int16_t unit_setter, int16_t value)
    { for (size_t index = 0U; index < length; index++) { unit_setter(void_pointer_at(p_buffer, unit_size, index), value); } }

static inline void struct_array_set_all_int8(const void * p_buffer, size_t unit_size, size_t length, set_int8_t unit_setter, int8_t value)
    { for (size_t index = 0U; index < length; index++) { unit_setter(void_pointer_at(p_buffer, unit_size, index), value); } }

static inline void struct_array_set_all_uint32(const void * p_buffer, size_t unit_size, size_t length, set_uint32_t unit_setter, uint32_t value)
    { for (size_t index = 0U; index < length; index++) { unit_setter(void_pointer_at(p_buffer, unit_size, index), value); } }

static inline void struct_array_set_all_uint16(const void * p_buffer, size_t unit_size, size_t length, set_uint16_t unit_setter, uint16_t value)
    { for (size_t index = 0U; index < length; index++) { unit_setter(void_pointer_at(p_buffer, unit_size, index), value); } }

static inline void struct_array_set_all_uint8(const void * p_buffer, size_t unit_size, size_t length, set_uint8_t unit_setter, uint8_t value)
    { for (size_t index = 0U; index < length; index++) { unit_setter(void_pointer_at(p_buffer, unit_size, index), value); } }

static inline bool struct_array_is_every_set_int32(const void * p_buffer, size_t unit_size, size_t length, try_int32_t unit_try, int32_t value)
{
    bool is_every = true;
    for (size_t index = 0U; index < length; index++) { if (unit_try(void_pointer_at(p_buffer, unit_size, index), value) == false) { is_every = false; } }
    return is_every;
}

static inline bool struct_array_is_every_set_int16(const void * p_buffer, size_t unit_size, size_t length, try_int16_t unit_try, int16_t value)
{
    bool is_every = true;
    for (size_t index = 0U; index < length; index++) { if (unit_try(void_pointer_at(p_buffer, unit_size, index), value) == false) { is_every = false; } }
    return is_every;
}

static inline bool struct_array_is_every_set_int8(const void * p_buffer, size_t unit_size, size_t length, try_int8_t unit_try, int8_t value)
{
    bool is_every = true;
    for (size_t index = 0U; index < length; index++) { if (unit_try(void_pointer_at(p_buffer, unit_size, index), value) == false) { is_every = false; } }
    return is_every;
}

static inline bool struct_array_is_every_set_uint32(const void * p_buffer, size_t unit_size, size_t length, try_uint32_t unit_try, uint32_t value)
{
    bool is_every = true;
    for (size_t index = 0U; index < length; index++) { if (unit_try(void_pointer_at(p_buffer, unit_size, index), value) == false) { is_every = false; } }
    return is_every;
}

static inline bool struct_array_is_every_set_uint16(const void * p_buffer, size_t unit_size, size_t length, try_uint16_t unit_try, uint16_t value)
{
    bool is_every = true;
    for (size_t index = 0U; index < length; index++) { if (unit_try(void_pointer_at(p_buffer, unit_size, index), value) == false) { is_every = false; } }
    return is_every;
}

static inline bool struct_array_is_every_set_uint8(const void * p_buffer, size_t unit_size, size_t length, try_uint8_t unit_try, uint8_t value)
{
    bool is_every = true;
    for (size_t index = 0U; index < length; index++) { if (unit_try(void_pointer_at(p_buffer, unit_size, index), value) == false) { is_every = false; } }
    return is_every;
}

static inline bool struct_array_is_any_set_int32(const void * p_buffer, size_t unit_size, size_t length, try_int32_t unit_try, int32_t value)
{
    bool is_any = false;
    for (size_t index = 0U; index < length; index++) { if (unit_try(void_pointer_at(p_buffer, unit_size, index), value) == true) { is_any = true; } }
    return is_any;
}

static inline bool struct_array_is_any_set_int16(const void * p_buffer, size_t unit_size, size_t length, try_int16_t unit_try, int16_t value)
{
    bool is_any = false;
    for (size_t index = 0U; index < length; index++) { if (unit_try(void_pointer_at(p_buffer, unit_size, index), value) == true) { is_any = true; } }
    return is_any;
}

static inline bool struct_array_is_any_set_int8(const void * p_buffer, size_t unit_size, size_t length, try_int8_t unit_try, int8_t value)
{
    bool is_any = false;
    for (size_t index = 0U; index < length; index++) { if (unit_try(void_pointer_at(p_buffer, unit_size, index), value) == true) { is_any = true; } }
    return is_any;
}

static inline bool struct_array_is_any_set_uint32(const void * p_buffer, size_t unit_size, size_t length, try_uint32_t unit_try, uint32_t value)
{
    bool is_any = false;
    for (size_t index = 0U; index < length; index++) { if (unit_try(void_pointer_at(p_buffer, unit_size, index), value) == true) { is_any = true; } }
    return is_any;
}

static inline bool struct_array_is_any_set_uint16(const void * p_buffer, size_t unit_size, size_t length, try_uint16_t unit_try, uint16_t value)
{
    bool is_any = false;
    for (size_t index = 0U; index < length; index++) { if (unit_try(void_pointer_at(p_buffer, unit_size, index), value) == true) { is_any = true; } }
    return is_any;
}

static inline bool struct_array_is_any_set_uint8(const void * p_buffer, size_t unit_size, size_t length, try_uint8_t unit_try, uint8_t value)
{
    bool is_any = false;
    for (size_t index = 0U; index < length; index++) { if (unit_try(void_pointer_at(p_buffer, unit_size, index), value) == true) { is_any = true; } }
    return is_any;
}

#endif // STRUCT_ARRAY_H