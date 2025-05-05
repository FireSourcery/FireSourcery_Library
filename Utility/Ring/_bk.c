
#include "Ring.h"

// typedef const struct Ring_Type
// {
//     const size_t UNIT_SIZE;
//     const size_t LENGTH;
// #if defined(CONFIG_RING_POW2_COUNTER) || defined(CONFIG_RING_POW2_WRAP)
//     const size_t POW2_MASK;
// #endif
// }
// Ring_Type_T;

// typedef struct Ring
// {
//     const Ring_Type_T Type;
//     volatile uint32_t Head;    /* FIFO Out/Front. */
//     volatile uint32_t Tail;    /* FIFO In/Back. */
// #if defined(CONFIG_RING_LOCAL_CRITICAL_ENABLE)
//     volatile critical_signal_t Lock;
// #endif
//     uint8_t Buffer[]; /* MISRA violation. Rationale: Compile-time allocated. */
// }
// Ring_T;

// const handle
// typedef const struct Ring_Const
// {
//     const Ring_Type_T TYPE;
//     Ring_T * const P_STATE;
// }
// Ring_Const_T;

// can compiler optimize as generic type when passing const to inline function?
// static inline void _Ring_PushBack(const Ring_Type_T * p_ringType, Ring_State_T * p_state, const void * p_unit)

#define IS_POW2(x) (((x) & ((x) - 1U)) == 0U)

/* alternatively use int/sizeof(int) in case of ascii fill */
#define BUFFER_ALLOC(Bytes) ((void *)(uint8_t[(Bytes)]){})

#define _RING_TYPE_INIT(UnitSize, Length) { .UNIT_SIZE = UnitSize, .LENGTH = Length,  _RING_INIT_POW2(Length - 1U) }
#define _RING_STATE_ALLOC(UnitSize, Length) ((Ring_State_T*)(BUFFER_ALLOC(sizeof(Ring_State_T) + (UnitSize)*(Length))))

#define RING_INIT(UnitSize, Length, p_State) { .TYPE = _RING_TYPE_INIT(UnitSize, Length), .P_STATE = p_State, }; static_assert(IS_POW2(Length), "Ring length must be power of 2");
#define RING_ALLOC(UnitSize, Length) RING_INIT(UnitSize, Length, _RING_STATE_ALLOC(UnitSize, Length))



/*
    Contigious RAM allocation,
    this implementation would still need to be referenced by a pointer when nested in another struct
    Incurs 1 pointer dereference, just as the ROM P_STATE case
*/
// typedef struct _Ring_RAM
// {
//     const Ring_Type_T Type;
//     Ring_State_T State;
//     uint8_t _Buffer[]; /* For compile time init. Overlap with State.Buffer  */
// }
// _Ring_RAM_T;

// #define _RING_RAM(UnitSize, Length)                 \
// {                                                   \
//     .Type = _RING_TYPE_INIT(UnitSize, Length),      \
//     ._Buffer[(UnitSize)*(Length)] = 0U,             \
// }

// /*
//     Const Control in ROM
// */
// typedef const struct _Ring_ROM
// {
//     const Ring_Type_T TYPE;
//     Ring_State_T * const P_STATE;
// }
// _Ring_ROM_T;

// #define _RING_ROM(UnitSize, Length)                 \
// {                                                   \
//     .TYPE = _RING_TYPE_INIT(UnitSize, Length),      \
//     .P_STATE = _RING_STATE_ALLOC(UnitSize, Length), \
// }

// typedef struct _Ring_Interface
// {
//     const Ring_Type_T Type; Ring_State_T * const p_State; // null for contiguos RAM
//     uint8_t _Interface[];
// }
// _Ring_Interface_T;


// #define _RING_INIT_AS_RAM(UnitSize, Length)\
// {\
//     .Type = _RING_TYPE_INIT(UnitSize, Length),  \
//     .p_State = NULL,                            \
//     ._Interface[(UnitSize) * (Length)] = 0U,    \
// }

// static inline Ring_State_T * _Ring_GetState(const Ring_T * p_ring)
// {
// #ifdef CONFIG_RING_RAM
//     return &(((_Ring_RAM_T*)p_ring)->State);
// #else
//     return ((_Ring_ROM_T*)p_ring)->TYPE.P_STATE;
// #endif
// }

/******************************************************************************/
/*!
    Experimental
*/
/******************************************************************************/
/*
    Give caller exclusive buffer write.
    Will flush buffer to start from index. Caller bypass virtual index.
*/
void * Ring_AcquireBuffer(Ring_T * p_ring)
{
    void * p_buffer = NULL;

    if (AcquireSignal(p_ring) == true)
    {
        Ring_Clear(p_ring);
        p_buffer = p_ring->CONST.P_BUFFER;
    }

    return p_buffer;
}

void Ring_ReleaseBuffer(Ring_T * p_ring, size_t writeSize)
{
    p_ring->Tail = writeSize;
    ReleaseSignal(p_ring);
}



#ifdef CONFIG_RING_DYNAMIC_MEMORY_ALLOCATION
Ring_T * Ring_New(Ring_Type_T type)
{
    Ring_T * p_ring = malloc(sizeof(Ring_T));
    void * p_buffer = malloc(unitCount * unitSize);

    if (p_ring != 0U && p_buffer != 0U)
    {
        *(void *)&p_ring->CONST.P_BUFFER = p_buffer; /* bypass const typedef */
        *(size_t)&p_ring->CONST.LENGTH = unitCount;
        *(size_t)&p_ring->CONST.UNIT_SIZE = unitSize;
        Ring_Init(p_ring);
    }

    return p_ring;
}

void Ring_Free(Ring_T * p_ring)
{
    free(p_ring);
}
#endif
/*
    Test
    Generic Implementation - withtout UNIT_SIZE
    Caller handle type
*/
#include "Type/Array/_generic_array.h"
/******************************************************************************/
/*!
    Cast Type
*/
/******************************************************************************/
static inline intptr_t _Ring_ValueOf(const Ring_T * p_ring, const void * p_unit)
{
    assert(p_ring->CONST.UNIT_SIZE <= sizeof(intptr_t));
    return void_pointer_as_value(p_unit, p_ring->CONST.UNIT_SIZE);
}

static inline void * array8_at(const void * p_buffer, size_t index) { return ((uint8_t *)p_buffer)[index]; }
static const int8_t Value8_At(const Ring_T * p_ring, size_t index) { return array8_at(p_ring->CONST.P_BUFFER, IndexAt(p_ring, index)); }
// static inline int8_t * _AsInt8Ptr(const Ring_T * p_ring, size_t index) { return as(int8_t *, p_ring->CONST.P_BUFFER) + IndexAt(p_ring, index); }
// static inline void _Assign_AsInt8(const Ring_T * p_ring, size_t index, int8_t value) { assign_as_array(int8_t, p_ring->CONST.P_BUFFER, IndexAt(p_ring, index), value); }

// static const int8_t Value8_At(const Ring_T * p_ring, size_t index) { return as_array(int8_t, p_ring->CONST.P_BUFFER, IndexAt(p_ring, index)); }
// const int8_t * _Ring8_At(const Ring_T * p_ring, size_t index) { return (index < Ring_GetFullCount(p_ring)) ? as(int8_t *, p_ring->CONST.P_BUFFER) + IndexAt(p_ring, index) : NULL; }
const int8_t * _Ring8_At(const Ring_T * p_ring, size_t index) { return (index < Ring_GetFullCount(p_ring)) ? as_array(int8_t *, p_ring->CONST.P_BUFFER, IndexAt(p_ring, index)) : NULL; }

#define RING_IMPL(T) \
    static inline T _Ring_At(const Ring_T * p_ring, size_t index) { return (index < Ring_GetFullCount(p_ring)) ? as_array(T, p_ring->CONST.P_BUFFER, IndexAt(p_ring, index)) : NULL; } \

// #define _AssignAs(T, p_ring, index, value) assign_as_array(T, p_ring->CONST.P_BUFFER, IndexAt(p_ring, index), value)
// void _Ring8_Assign(Ring_T * p_ring, size_t index, int8_t value) { if (index < Ring_GetFullCount(p_ring)) { _assign_as_array(int8_t, p_ring->CONST.P_BUFFER, IndexAt(p_ring, index), value); } }

// const void * _Ring_At_TypeWith(const Ring_T * p_ring, size_t index, get_entry_t arrayAccessor) { return (index < Ring_GetFullCount(p_ring)) ? (const void *)(arrayAccessor(p_ring, index)) : NULL; }
// const int8_t * _Ring8_At1(const Ring_T * p_ring, size_t index) { return _Ring_At_TypeWith(p_ring, index, (get_entry_t)as_array_int8_ptr); }

//alternatively return type defined at INIT, handles index offset, user calls to cast again
// void * _Ring_At1(const Ring_T * p_ring, size_t index) { return (index < Ring_GetFullCount(p_ring)) ? as_array(p_ring->CONST.P_BUFFER, IndexAt(p_ring, index)) : NULL; }