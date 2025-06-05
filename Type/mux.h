#pragma once

#include <stdint.h>
#include <stdbool.h>

/*!
    @brief Generic multiplexer (Mux) for selecting and managing multiple interface/resource instances.
*/
typedef const void entry_t;
typedef const entry_t * const entry_table_t;


// typedef const struct
// {
//     const entry_t * p_Active;
//     const Mux_Table_T * p_Table;
//     const uint8_t Error;
// }
// Mux_T;

static inline entry_t * mux_from(entry_table_t * pp_table, size_t length, size_t index) { return (index < length) ? pp_table[index] : NULL; }
static inline entry_t * mux_or_default(entry_table_t * pp_table, size_t length, size_t index) { return (index < length) ? pp_table[index] : pp_table[0U]; }

static inline bool mux_is_valid(entry_t * p_entry, entry_table_t * pp_table, uint8_t length, uint8_t index)
{
    return (p_entry != NULL) && (p_entry == mux_from(pp_table, length, index));
}

static inline bool mux_is_from(entry_t * p_entry, entry_table_t * pp_table, uint8_t length)
{
    bool isValid = false;
    for (uint8_t index = 0U; index < length; index++) { if (p_entry == pp_table[index]) { isValid = true; break; } }
    return isValid;
}

