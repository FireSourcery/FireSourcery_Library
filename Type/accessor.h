#ifndef ACCESSOR_H
#define ACCESSOR_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <sys/types.h>

typedef intptr_t value_t;

typedef void(*proc_t)(void * p_context);
typedef void (*set_t)(void * p_context, value_t value);
typedef value_t(*get_t)(void * p_context);
typedef void (*set_entry_t)(void * p_context, size_t key, value_t value);
typedef value_t(*get_entry_t)(void * p_context, size_t key);

typedef bool (*test_t)(const void * p_unit);
typedef bool (*poll_t)(void * p_unit);
typedef bool (*try_t)(void * p_unit, value_t value);
// typedef void * (*try_t)(void * p_unit, value_t value);

static inline void nullcheck_call(proc_t proc, void * p_context) { if (proc != NULL) { proc(p_context); } }
static inline void nullcheck_call1(set_t proc, void * p_context, value_t value) { if (proc != NULL) { proc(p_context, value); } }
// static inline void maybe_set(set_t set, void * p_context, value_t value) { if (set != NULL) { set(p_context, value); } }

#endif