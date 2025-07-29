/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

    This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
/******************************************************************************/
/******************************************************************************/
/*!
    @file
    @author FireSourcery
    @brief  Non Volatile Memory Operations. Interface Base, Cmd Pattern.
*/
/******************************************************************************/
#ifndef NV_MEMORY_H
#define NV_MEMORY_H

#include "Config.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <assert.h>

/* For placing functions into ram section */
#ifndef NV_MEMORY_ATTRIBUTE_RAM_SECTION
#warning 'Undefined NV_MEMORY_ATTRIBUTE_RAM_SECTION'
#define NV_MEMORY_ATTRIBUTE_RAM_SECTION
// #define NV_MEMORY_ATTRIBUTE_RAM_SECTION __attribute__((section(".code_ram")))
#endif


/******************************************************************************/
/*
    Common return status by public module functions
*/
/******************************************************************************/
typedef enum NvMemory_Status
{
    NV_MEMORY_STATUS_SUCCESS,
    NV_MEMORY_STATUS_PROCESSING,        /* NonBlocking Only */

    /* Hal Errors */
    // NV_MEMORY_STATUS_ERROR,
    NV_MEMORY_STATUS_ERROR_BUSY,
    NV_MEMORY_STATUS_ERROR_CMD,         /* Cmd Failed. Unparsed Error */
    NV_MEMORY_STATUS_ERROR_PROTECTION,

    // NV_MEMORY_STATUS_ERROR_INPUT,
    /* Input errors - destination, source, size */
    NV_MEMORY_STATUS_ERROR_BOUNDARY,    /* Dest + size, larger than partition */
    NV_MEMORY_STATUS_ERROR_ALIGNMENT,   /* Dest, or source size, not aligned */
    NV_MEMORY_STATUS_ERROR_BUFFER,      /* Virtual buffer size */
    NV_MEMORY_STATUS_ERROR_INVALID_OP,

    NV_MEMORY_STATUS_ERROR_VERIFY,      /* Verify cmd */
    NV_MEMORY_STATUS_ERROR_CHECKSUM,    /*  */
    NV_MEMORY_STATUS_ERROR_NOT_IMPLEMENTED,
    NV_MEMORY_STATUS_ERROR_OTHER, /* External error requiring common a return type */
}
NvMemory_Status_T;

/******************************************************************************/
/*
    HAL typedefs
*/
/******************************************************************************/
typedef bool (* const HAL_NvMemory_ReadFlags_T)(const void * p_hal);
typedef void (* const HAL_NvMemory_ClearFlags_T)(void * p_hal);

/* Generic with all parameters */
typedef void (*HAL_NvMemory_StartCmd_T)(void * p_hal, uintptr_t address, const uint8_t * p_data, size_t units);
typedef void (*HAL_NvMemory_FinalizeCmd_T)(void * p_hal, uintptr_t address, size_t units, uint8_t * p_buffer); /* Capture/OnComplete */

/* implementation mapping each operation type to vtable, instead of using generic [HAL_NvMemory_StartCmd_T] */
typedef void (*HAL_NvMemory_CmdReadUnit_T)(void * p_hal, uintptr_t address, uint8_t * p_result);
typedef void (*HAL_NvMemory_CmdWriteUnit_T)(void * p_hal, uintptr_t address, const uint8_t * p_data);
typedef void (*HAL_NvMemory_CmdEraseUnit_T)(void * p_hal, uintptr_t address);
typedef void (*HAL_NvMemory_CmdUnits_T)(void * p_hal, uintptr_t address, size_t units); /* Collective Erase/Verify */

/* status moves to hal */
typedef NvMemory_Status_T(*HAL_NvMemory_MapStatus_T)(const void * p_hal);
// typedef NvMemory_Status_T(*NvMemory_MapStatus_T)(NvMemory_T * p_this); // or nvm

/* Per Cmd VTable. NvMemory_Cmd */
typedef const struct NvMemory_OpControl
{
    HAL_NvMemory_StartCmd_T START_CMD;          /* Start HAL Cmd, multiple iterations per Op */
    HAL_NvMemory_FinalizeCmd_T FINALIZE_CMD;    /* On end per Cmd iteration */
    HAL_NvMemory_MapStatus_T PARSE_CMD_ERROR;   /* On end all Cmd iteration, Op Complete Error */
    size_t UNIT_SIZE;                           /* Align Size */
    size_t(*FORCE_ALIGN)(uintptr_t value, size_t align); /* Align up or down, alternatively as enum */

    // size_t UNITS_PER_CMD;    /* count of UNIT_SIZE. Used by EraseVerify for now */
    // size_t BytesPerCmd;      /* Used by Erase for now */ /* overwrite if not equal unit size. BytesPerCmd = unitsPerCmd * unitSize */
}
NvMemory_OpControl_T;

/******************************************************************************/
/*
    Partition Group
    Partition defines writable ranges. Checked on op cmd.
    Only provides range check. Caller account for overlapping address id
*/
/******************************************************************************/
typedef const struct NvMemory_Partition
{
    uintptr_t ADDRESS;
    size_t SIZE;
#ifdef CONFIG_NV_MEMORY_HW_OP_ADDRESS_RELATIVE /* or HAL functions handle offset */
    ptrdiff_t OP_ADDRESS_OFFSET;
#endif
    // uintptr_t RAM_ADDRESS;
}
NvMemory_Partition_T;

#if defined(CONFIG_NV_MEMORY_HW_OP_ADDRESS_RELATIVE)
#define _NV_MEMORY_INIT_PARTITION_OFFSET(OpAddressOffset) .OP_ADDRESS_OFFSET = OpAddressOffset,
#else
#define _NV_MEMORY_INIT_PARTITION_OFFSET(OpAddressOffset)
#endif

#define NV_MEMORY_INIT_PARTITION(AddressStart, SizeBytes, OpAddressOffset)  \
{                                                                           \
    .ADDRESS    = AddressStart,                                             \
    .SIZE       = SizeBytes,                                                \
    _NV_MEMORY_INIT_PARTITION_OFFSET(OpAddressOffset)                       \
}


/******************************************************************************/
/*
*/
/******************************************************************************/
/*  */
typedef void (*NvMemory_Callback_T)(void * p_callbackData);

/*
    NvMemory controller
*/
typedef struct NvMemory_State
{
    /* User Input Per Operation */
    /* Direct */
    const NvMemory_OpControl_T * p_OpControl;
    uintptr_t OpAddress;        /* The NV Memory address. The destination address in most cases */
    size_t OpSize;              /* Outer loop size. Retain for ContinueWrite. Total bytes at start, aligned */
    const void * p_OpData;      /* Op data source or read buffer. internal buffer, caller provided address, written only in read case */
    // uint8_t * p_DestBuffer, // alternatively keep separate pointer for buffer

    /* Derived */
    // const NvMemory_Partition_T * p_OpPartition; /* Op Dest boundary check */
    size_t OpSizeAligned;       /* Cached calculation. alternatively use getter */

    /* Config Options, todo as bits */
    bool IsOpBuffered;          /* copy to buffer first or use pointer to source */
    bool IsVerifyEnable;        /* Auto call verify, alternatively user call */
    bool IsForceAlignEnable;    /* if implement in this module, need buffer of UNIT_WRITE_SIZE for fill with erase pattern (Even if IsOpBuffered is not used ) */

    void * p_CallbackContext;
    NvMemory_Callback_T Yield;  /* On Block */
}
NvMemory_State_T;

/* Flash must store a copy in RAM */
typedef const struct NvMemory
{
    /* Per sub-type VTable, abstract functions provided by concrete child class */
    void * const P_HAL;
    const HAL_NvMemory_ReadFlags_T READ_COMPLETE_FLAG;      /* Must reside in RAM for Flash case */
    const HAL_NvMemory_ReadFlags_T READ_ERROR_FLAGS;        /* Must reside in RAM for Flash case */
    const HAL_NvMemory_ClearFlags_T CLEAR_ERROR_FLAGS;

    /* Instance. In most cases there will only be one instance per type */
    NvMemory_State_T * const P_STATE;

    /* Bounds of accessible partitions */
    const NvMemory_Partition_T * const P_PARTITIONS;
    const uint8_t PARTITION_COUNT;

    /* Allocated buffer for internal use */
    uint8_t * const P_BUFFER;
    const size_t BUFFER_SIZE;

    /* On Block */
    // void * P_YIELD_CONTEXT;
    // NvMemory_Callback_T YIELD;
}
NvMemory_T;

// BaseVTable
// typedef const struct NvMemory_OpCommon
// {
//     const HAL_NvMemory_ReadFlags_T READ_COMPLETE_FLAG;      /* Must reside in RAM, for Flash case */
//     const HAL_NvMemory_ReadFlags_T READ_ERROR_FLAGS;        /* Must reside in RAM, for Flash case */
//     const HAL_NvMemory_ClearFlags_T CLEAR_ERROR_FLAGS;
// }
// NvMemory_OpCommon_T;

/*
*/
#define _NV_MEMORY_INIT_HAL(p_Hal, ReadCompleteFlag, ReadErrorFlags, ClearErrorFlags)   \
    .P_HAL = p_Hal,                                                                     \
    .READ_COMPLETE_FLAG = (HAL_NvMemory_ReadFlags_T)ReadCompleteFlag,                   \
    .READ_ERROR_FLAGS   = (HAL_NvMemory_ReadFlags_T)ReadErrorFlags,                     \
    .CLEAR_ERROR_FLAGS  = (HAL_NvMemory_ClearFlags_T)ClearErrorFlags,

#define _NV_MEMORY_INIT_PARTITIONS(p_Partitions, PartitionsCount)   \
    .P_PARTITIONS       = p_Partitions,                             \
    .PARTITION_COUNT    = PartitionsCount,

#define _NV_MEMORY_INIT_BUFFER(p_Buffer, BufferSize)    \
    .P_BUFFER           = p_Buffer,                     \
    .BUFFER_SIZE        = BufferSize,

#define NV_MEMORY_STATE_ALLOC() (&(NvMemory_State_T){0})

/*
    ReadOnce must provide buffer here, or in calling function
*/
#define NV_MEMORY_INIT(p_Hal, p_HAL_ReadCompleteFlag, p_HAL_ReadErrorFlags, p_HAL_ClearErrorFlags, p_State, p_Partitions, PartitionsCount, p_Buffer, BufferSize) \
{   \
    _NV_MEMORY_INIT_HAL(p_Hal, p_HAL_ReadCompleteFlag, p_HAL_ReadErrorFlags, p_HAL_ClearErrorFlags)                                                 \
    .P_STATE = p_State,                                                                                                                             \
    _NV_MEMORY_INIT_PARTITIONS(p_Partitions, PartitionsCount)                                                                                       \
    _NV_MEMORY_INIT_BUFFER(p_Buffer, BufferSize)                                                                                                    \
}

/*!
    @param[in] value - address or size
    @param[in] align - unit must be power of 2
*/
static inline uintptr_t nvmemory_align_down(uintptr_t value, size_t align) { return (value & (-align)); }
static inline uintptr_t nvmemory_align_up(uintptr_t value, size_t align) { return (-((-value) & (-align))); }
static inline bool nvmemory_is_aligned(uintptr_t value, size_t align) { return ((value & (align - 1UL)) == (uintptr_t)0UL); }


/* Alternatively Cached on SetOp */
static inline size_t _NvMemory_GetOpSizeAligned(NvMemory_State_T * p_state)    { return p_state->p_OpControl->FORCE_ALIGN(p_state->OpSize, p_state->p_OpControl->UNIT_SIZE); }
static inline size_t _NvMemory_GetOpSizeRemainder(NvMemory_State_T * p_state)  { return p_state->OpSize - nvmemory_align_down(p_state->OpSize, p_state->p_OpControl->UNIT_SIZE); }

static inline void _NvMemory_EnableForceAlign(NvMemory_State_T * p_state) { p_state->IsForceAlignEnable = true; }
static inline void _NvMemory_DisableForceAlign(NvMemory_State_T * p_state) { p_state->IsForceAlignEnable = false; }

static inline void _NvMemory_SetYield(NvMemory_State_T * p_state, NvMemory_Callback_T yield, void * p_callbackData)
{
    p_state->Yield = yield;
    p_state->p_CallbackContext = p_callbackData;
}

static inline void NvMemory_EnableForceAlign(const NvMemory_T * p_context) { _NvMemory_EnableForceAlign(p_context->P_STATE); }
static inline void NvMemory_DisableForceAlign(const NvMemory_T * p_context) { _NvMemory_DisableForceAlign(p_context->P_STATE); }
static inline void NvMemory_SetYield(const NvMemory_T * p_context, NvMemory_Callback_T yield, void * p_callbackData) { _NvMemory_SetYield(p_context->P_STATE, yield, p_callbackData); }


/*
    Extern
*/
/*
    Blocking
*/
// extern NvMemory_Status_T NvMemory_ProcOp_Blocking(NvMemory_T * p_context) NV_MEMORY_ATTRIBUTE_RAM_SECTION;
extern NvMemory_Status_T NvMemory_ProcOp_Blocking(NvMemory_T * p_context);

extern void NvMemory_Init(NvMemory_T * p_context);

extern NvMemory_Status_T NvMemory_MemCompare(const void * p_dest, const void * p_source, size_t size);
// extern NvMemory_Status_T NvMemory_VerifyByMemCompare(NvMemory_T * p_context);
// extern bool NvMemory_CheckOpChecksum(const NvMemory_T * p_context, const void * p_source, size_t size);

/* Destination is not intended to be dereference, pass as uintptr_t type */
extern NvMemory_Status_T NvMemory_SetOpAddress(NvMemory_T * p_context, uintptr_t address, size_t size);
extern NvMemory_Status_T NvMemory_SetOpSize(NvMemory_T * p_context, size_t size);
extern NvMemory_Status_T NvMemory_SetOpData(NvMemory_T * p_context, const void * p_data, size_t size);
extern NvMemory_Status_T NvMemory_SetOpControl(NvMemory_T * p_context, const NvMemory_OpControl_T * p_opControl, uintptr_t address, size_t size);
extern NvMemory_Status_T NvMemory_SetOpControl_Read(NvMemory_T * p_context, const NvMemory_OpControl_T * p_opControl, uintptr_t address, size_t size, void * p_data);
extern NvMemory_Status_T NvMemory_SetOpControl_Write(NvMemory_T * p_context, const NvMemory_OpControl_T * p_opControl, uintptr_t address, const void * p_data, size_t size);

/*
    Non Blocking
*/

// typedef enum NvMemory_OpState
// {
//     NV_MEMORY_STATE_IDLE,
//     NV_MEMORY_STATE_ACTIVE,
//     NV_MEMORY_STATE_WRITE,
//     NV_MEMORY_STATE_VERIFY,
// }
// NvMemory_OpState_T;

/* Nonblocking use only */
// NvMemory_StateId_T State;
// NvMemory_Status_T Status;
// size_t OpIndex;     /* in bytes */
// NvMemory_Callback_T OnComplete;     /*!< OnComplete */

extern size_t NvMemory_GetOpBytesRemaining(NvMemory_T * p_context);
extern bool NvMemory_ProcOp(NvMemory_T * p_context);
extern NvMemory_Status_T NvMemory_StartOp(NvMemory_T * p_context);

#endif /* NV_MEMORY_H */


