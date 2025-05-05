#ifndef ACCESSOR_H
#define ACCESSOR_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <sys/types.h>

/* Generic Interface. Generic Array/Memory Access */

typedef intptr_t value_t;
// typedef register_t value_t;

typedef void(*proc_t)(void * p_context);
typedef void (*set_t)(void * p_context, value_t value);
typedef value_t(*get_t)(const void * p_context);
typedef void (*set_entry_t)(void * p_context, size_t key, value_t value);
typedef value_t(*get_entry_t)(const void * p_context, size_t key);

typedef value_t * (*get_optional_t)(void * p_context);
typedef bool (*test_t)(const void * p_context);
typedef bool (*try_proc_t)(void * p_context);
typedef bool (*try_set_t)(void * p_context, value_t value);

typedef int(*compare_t)(const void * a, const void * b);
// typedef value_t(*transform_t)(const void * p_context);
// typedef value_t (*apply_t)(void * p_context, value_t value);

// static inline void nullcheck_call(proc_t proc, void * p_context) { if (proc != NULL) { proc(p_context); } }
// static inline void call_proc(proc_t proc, void * p_context) { if (proc != NULL) { proc(p_context); } }
// static inline void nullcheck_call1(set_t proc, void * p_context, value_t value) { if (proc != NULL) { proc(p_context, value); } }

#endif