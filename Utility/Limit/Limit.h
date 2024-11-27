#ifndef LIMIT_H
#define LIMIT_H

#include "../Array/Array.h"

#ifdef CONFIG_LIMIT_SIGNED
typedef int32_t limit_t;
#define LIMIT_MIN (-INT32_MAX)
#define LIMIT_MAX INT32_MAX
#define LIMIT_CLEAR INT32_MIN /* (LIMIT_MIN - 1) is reserved  */
#else
/*
    limits as a percent of uint16_t
*/
typedef uint16_t limit_t;
#define LIMIT_MIN 0U
#define LIMIT_MAX UINT16_MAX
#define LIMIT_CLEAR LIMIT_MAX /* UpperLimit unaffected. LowerLimit may incur additional logic  */
#endif

typedef uint8_t limit_id_t;

typedef struct
{
    // VALUE_ARRAY(limit_t);
    const struct { limit_t * const P_ARRAY; const size_t LENGTH; };
    // void * P_CALLBACK_CONTEXT;
    // void (* const ON_SET)(void * p_context, limit_t value);
    // void (* const ON_CLEAR)(void * p_context);

    size_t count;
    /*
        limits calculated on add/remove
        get is O(1)
    */
    limit_t valuesMin; /* Min of the array, upper of [Limit] */
    limit_t valuesMax; /* Max of the array, lower of [Limit] */
}
Limit_T;

#define LIMIT_INIT(p_buffer, length) \
{ \
    .P_ARRAY = (p_buffer), .LENGTH = (length) \
}
// VALUE_ARRAY_INIT_UNNAMED(p_buffer, length),             \
// .ARRAY = ARRAY_INIT(p_buffer, length, unitSize),        \


/*!
    @brief Get the arrayMinimum value from the p_limit control.
    @param p_limit Pointer to the Limit_T structure.
    @return The arrayMinimum value, or UINT16_MAX if the array is empty.
*/
static inline limit_t Limit_GetUpper(const Limit_T * p_limit) { return p_limit->valuesMin; }

/*!
    @brief Get the arrayMaximum value from the p_limit control.
    @param p_limit Pointer to the Limit_T structure.
    @return The arrayMaximum value, or 0 if the array is empty.
*/
static inline limit_t Limit_GetLower(const Limit_T * p_limit) { return p_limit->valuesMax; }

static inline bool Limit_IsActive(const Limit_T * p_limit) {  return (p_limit->valuesMin != LIMIT_MAX || p_limit->valuesMax != LIMIT_MIN); }
static inline bool Limit_IsUpperActive(const Limit_T * p_limit) {  return (p_limit->valuesMin != LIMIT_MAX); }
static inline bool Limit_IsLowerActive(const Limit_T * p_limit) {  return (p_limit->valuesMax != LIMIT_MIN); }

extern void Limit_Init(Limit_T * p_limit);
extern void Limit_ClearAll(Limit_T * p_limit);

extern bool Limit_Entry_Set(Limit_T * p_limit, uint8_t id, limit_t value);
extern bool Limit_Entry_Clear(Limit_T * p_limit, uint8_t id);
// extern bool Limit_Entry_SetUpper(Limit_T * p_limit, uint8_t id, limit_t value);

#endif // LIMIT_H