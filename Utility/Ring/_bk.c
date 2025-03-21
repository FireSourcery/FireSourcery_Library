
#include "Ring.h"

typedef const struct Ring_Type
{
    const size_t UNIT_SIZE;
    const size_t LENGTH;
#if defined(CONFIG_RING_POW2_COUNTER) || defined(CONFIG_RING_POW2_WRAP)
    const size_t POW2_MASK;
#endif
// ifndef RAM_ONLY
    Ring_State_T * const P_STATE; /* Null for RAM */
}
Ring_Type_T;

typedef struct Ring_State
{
    volatile uint32_t Head;    /* FIFO Out/Front. */
    volatile uint32_t Tail;    /* FIFO In/Back. */
#if defined(CONFIG_RING_LOCAL_CRITICAL_ENABLE)
    volatile critical_signal_t Lock;
#endif
    uint8_t Buffer[]; /* MISRA violation. Rationale: Compile-time allocated. */
}
Ring_State_T;


typedef const struct Ring
{
    const Ring_Type_T TYPE; // Ring_State_T * const P_STATE;
    // uint8_t _Interface[]; // if allocating as RAM at compile time
}
Ring_T;


// can compiler optimize as generic type when passing const to inline function?
// static inline void _Ring_PushBack(const Ring_Type_T * p_ringType, Ring_State_T * p_state, const void * p_unit)

#define IS_POW2(x) (((x) & ((x) - 1U)) == 0U)

/* alternatively use int/sizeof(int) in case of ascii fill */
#define BUFFER_ALLOC(Bytes) ((void *)(uint8_t[(Bytes)]){})
#define ARRAY_ALLOC(UnitSize, Length) BUFFER_ALLOC((UnitSize)*(Length))
#define ARRAY_ALLOC_AS(T, Length) ((void *)(T[(Length)]){})

#define _RING_TYPE_INIT(UnitSize, Length)   { .UNIT_SIZE = UnitSize, .LENGTH = Length,  _RING_INIT_POW2(Length - 1U) }
// #define _RING_TYPE_INIT(UnitSize, Length, p_State)   { .UNIT_SIZE = UnitSize, .LENGTH = Length, .P_STATE = p_State,  _RING_INIT_POW2(Length - 1U) }
#define _RING_STATE_ALLOC(UnitSize, Length) ((Ring_State_T*)(BUFFER_ALLOC(sizeof(Ring_State_T) + (UnitSize)*(Length))))

#define RING_INIT(UnitSize, Length, p_State) { .TYPE = _RING_TYPE_INIT(UnitSize, Length), .P_STATE = p_State, }; static_assert(IS_POW2(Length), "Ring length must be power of 2");
#define RING_ALLOC(UnitSize, Length) RING_INIT(UnitSize, Length, _RING_STATE_ALLOC(UnitSize, Length))



// _RING_ROM(UnitSize, Length); static_assert(IS_POW2(Length), "Ring length must be power of 2");


/*
    Contigious RAM allocation,
    this implementation would still need to be referenced by a pointer when nested in another struct
    Incurs 1 pointer dereference, just as the ROM P_STATE case
*/
typedef struct _Ring_RAM
{
    const Ring_Type_T Type;
    Ring_State_T State;
    uint8_t _Buffer[]; /* For compile time init. Overlap with State.Buffer  */
}
_Ring_RAM_T;

#define _RING_RAM(UnitSize, Length)                 \
{                                                   \
    .Type = _RING_TYPE_INIT(UnitSize, Length),      \
    ._Buffer[(UnitSize)*(Length)] = 0U,             \
}

/*
    Const Control in ROM
*/
typedef const struct _Ring_ROM
{
    const Ring_Type_T TYPE;
    // Ring_State_T * const P_STATE;
}
_Ring_ROM_T;

#define _RING_ROM(UnitSize, Length)                 \
{                                                   \
    .TYPE = _RING_TYPE_INIT(UnitSize, Length),      \
    .P_STATE = _RING_STATE_ALLOC(UnitSize, Length), \
}

typedef struct _Ring_Interface
{
    const Ring_Type_T Type; Ring_State_T * const p_State; // null for contiguos RAM
    uint8_t _Interface[];
}
_Ring_Interface_T;


#define _RING_INIT_AS_RAM(UnitSize, Length)\
{\
    .Type = _RING_TYPE_INIT(UnitSize, Length),  \
    .p_State = NULL,                            \
    ._Interface[(UnitSize) * (Length)] = 0U,    \
}

static inline Ring_State_T * _Ring_GetState(const Ring_T * p_ring)
{
#ifdef CONFIG_RING_RAM
    return &(((_Ring_RAM_T*)p_ring)->State);
#else
    return ((_Ring_ROM_T*)p_ring)->TYPE.P_STATE;
#endif
}

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
