#pragma once

#include <stdint.h>
#include <stdbool.h>

/*!
    @brief Array Mux
    Generic multiplexer (Mux) for selecting and managing multiple interface/resource instances.
*/
typedef const void entry_t;
typedef const entry_t * const entry_table_t;
// typedef const entry_t * const mux_t; entry_t ** pp_entry


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


// static inline VTable_T * Mux_Select(const Mux_T * p_mux, int select) { return (select < p_mux->SOURCE_COUNT) ? p_mux->PP_SOURCES[select] : NULL; }
// // directly set, can be templated for permanent selection
// static inline void Mux_SetField(const Mux_T * p_mux, int select, int fieldId, uintptr_t value) { VTable_SetField(p_mux->PP_SOURCES[select], p_mux->P_CONTEXT, fieldId, value); }