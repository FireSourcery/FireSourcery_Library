#ifndef ACCESSOR_H
#define ACCESSOR_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <sys/types.h>

/* Generic Interface. Generic Array/Memory Access */

typedef int value_t;
// typedef intptr_t value_t;
// typedef register_t value_t;
// typedef int key_t;
// typedef size_t key_t;

typedef void(*proc_t)(void * p_context);
typedef value_t(*get_t)(const void * p_context);
typedef void (*set_t)(void * p_context, value_t value);

typedef value_t(*get_at_t)(const void * p_context, int index);
typedef void (*set_at_t)(void * p_context, int index, value_t value);

typedef bool (*test_t)(const void * p_context);
typedef bool (*test_entry_t)(const void * p_context, int idOrValue);

// typedef int (*poll_t)(void * p_context);
typedef bool (*try_proc_t)(void * p_context);
typedef value_t * (*get_optional_t)(void * p_context);
typedef bool (*try_set_t)(void * p_context, value_t value);

typedef int(*compare_t)(const void * a, const void * b);
// typedef value_t(*transform_t)(const void * p_context);
// typedef value_t (*apply_t)(void * p_context, value_t value);

// static inline void nullcheck_call(proc_t proc, void * p_context) { if (proc != NULL) { proc(p_context); } }
// static inline void call_proc(proc_t proc, void * p_context) { if (proc != NULL) { proc(p_context); } }
// static inline void nullcheck_call1(set_t proc, void * p_context, value_t value) { if (proc != NULL) { proc(p_context, value); } }

/* Shorthand */
static inline value_t call_get_at(get_at_t get, void * p_context, int index) { return (get != NULL) ? get(p_context, index) : 0; }
static inline void call_set_at(set_at_t set, void * p_context, int index, value_t value) { if (set != NULL) { set(p_context, index, value); } }

static inline bool call_test(test_t test, void * p_context) { return (test != NULL) ? test(p_context) : true; }
static inline bool call_test_set(test_entry_t test, void * p_context, int value) { return (test != NULL) ? test(p_context, value) : true; }


// #define Call_Accessor(Accessor, ...) \
//     _Generic(Accessor, \
//         get_t: call_get_at(__VA_ARGS__), \
//         set_t: call_set_at, \
//         get_at_t: call_get_at, \
//         set_at_t: call_set_at, \
//         test_entry_t: call_test_entry, \
//         test_t: call_test, \
//         try_proc_t: call_try_proc, \
//         try_set_t: call_try_set, \
//         compare_t: call_compare, \
//         transform_t: call_transform, \
//         apply_t: call_apply  \
//     )

#endif