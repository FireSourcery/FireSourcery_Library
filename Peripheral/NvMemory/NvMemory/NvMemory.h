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
    @brief  Non Volatile Memory Operations. Abstract Base Class. Template Pattern.
    @version V0
*/
/******************************************************************************/
#ifndef NV_MEMORY_H
#define NV_MEMORY_H

#include "Config.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <assert.h>

/*!
    @param[in] value - address or size
    @param[in] align - unit must be power of 2
*/
static inline uintptr_t nvmemory_align_down(uintptr_t value, size_t align) { return (value & (-align)); }
static inline uintptr_t nvmemory_align_up(uintptr_t value, size_t align) { return (-((-value) & (-align))); }
static inline bool nvmemory_is_aligned(uintptr_t value, size_t align) { return ((value & (align - 1UL)) == (uintptr_t)0UL); }

/*
    Common return status by public module functions
*/
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
    NV_MEMORY_STATUS_ERROR_OTHER, /* External error requring common a return type */
}
NvMemory_Status_T;

/*
    Partition Group
    Partition defines writable ranges. Checked on op cmd.
    Only provides range check. Caller account for overlapping address id
*/
typedef const struct NvMemory_Partition
{
    uintptr_t ADDRESS;
    size_t SIZE;
#ifdef CONFIG_NV_MEMORY_HW_OP_ADDRESS_RELATIVE /* or HAL functions handle offset */
    ptrdiff_t OP_ADDRESS_OFFSET;
#endif
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

// #define NV_MEMORY_PARTITION_INIT(AddressStart, SizeBytes, ...)  \
// {                                                               \
//     .ADDRESS    = AddressStart,                                 \
//     .SIZE       = SizeBytes,                                    \
//     _NV_MEMORY_INIT_PARTITION_OFFSET(__VA_ARGS__)               \
// }

/*
    typedefs
*/
typedef bool (* const HAL_NvMemory_ReadFlags_T)(const void * p_hal);
typedef void (* const HAL_NvMemory_ClearFlags_T)(void * p_hal);

typedef void (*HAL_NvMemory_StartCmd_T)(void * p_hal, uintptr_t address, const uint8_t * p_data, size_t units);
typedef void (*HAL_NvMemory_FinalizeCmd_T)(void * p_hal, uintptr_t address, size_t units, uint8_t * p_buffer);

// typedef bool(*HAL_NvMemory_ReadStatus_T)(const void * p_hal);
// typedef NvMemory_Status_T(*NvMemory_ParseStatus_T)(const void * p_hal);
typedef NvMemory_Status_T(*HAL_NvMemory_MapStatus_T)(const void * p_hal);

/* implementation mapping each operation type to vtable, instead of using generic startCmd */
typedef void (*HAL_NvMemory_CmdReadUnit_T)(void * p_hal, uintptr_t address, uint8_t * p_result);
typedef void (*HAL_NvMemory_CmdWriteUnit_T)(void * p_hal, uintptr_t address, const uint8_t * p_data);
typedef void (*HAL_NvMemory_CmdEraseUnit_T)(void * p_hal, uintptr_t address);
typedef void (*HAL_NvMemory_CmdUnits_T)(void * p_hal, uintptr_t address, size_t units);

typedef void (*NvMemory_Callback_T)(void * p_callbackData);

/* Per Cmd VTable */
typedef const struct NvMemory_OpControl
{
    HAL_NvMemory_StartCmd_T START_CMD;          /* Start HAL Cmd, multiple iterations per Op */
    HAL_NvMemory_FinalizeCmd_T FINALIZE_CMD;    /* On end per Cmd iteration */
    HAL_NvMemory_MapStatus_T PARSE_CMD_ERROR;   /* On end all Cmd iteration, Op Complete Error */
    size_t UNIT_SIZE;                           /* Align Size */
    size_t(*FORCE_ALIGN)(uintptr_t value, size_t align); /* Align up or down */
    // size_t CMD_UNITS;        /* Used by EraseVerify for now */
    // size_t BytesPerCmd;      /* Used by Erase for now */
    // size_t UnitsPerCmd;      /* count of UNIT_SIZE */
}
NvMemory_OpControl_T;

typedef enum NvMemory_State
{
    NV_MEMORY_STATE_IDLE,
    NV_MEMORY_STATE_ACTIVE,
    NV_MEMORY_STATE_WRITE,
    NV_MEMORY_STATE_VERIFY,
}
NvMemory_State_T;

/* Per sub-type VTable, abstract functions provided by concrete child class */
typedef const struct NvMemory_Const
{
    void * const P_HAL;
    const HAL_NvMemory_ReadFlags_T READ_COMPLETE_FLAG;      /* Must reside in RAM, for Flash case */
    const HAL_NvMemory_ReadFlags_T READ_ERROR_FLAGS;        /* Must reside in RAM, for Flash case */
    const HAL_NvMemory_ClearFlags_T CLEAR_ERROR_FLAGS;
    const NvMemory_Partition_T * const P_PARTITIONS;
    const uint8_t PARTITION_COUNT;
    uint8_t * const P_BUFFER;
    const size_t BUFFER_SIZE;
}
NvMemory_Const_T;

/*
    NvMemory controller
*/
typedef struct NvMemory
{
    NvMemory_Const_T CONST;

    /* Config Options */
    bool IsOpBuffered;          /* copy to buffer first or use pointer to source */
    bool IsVerifyEnable;        /* Auto call verify, alternatively user call */
    bool IsForceAlignEnable;    /* if implement in this module, need buffer of UNIT_WRITE_SIZE for fill with erase pattern (Even if IsOpBuffered is not used ) */

    void * p_CallbackContext;
    NvMemory_Callback_T Yield;  /* On Block */

    /* User Input Per Operation */
    /* Direct */
    const NvMemory_OpControl_T * p_OpControl;
    uintptr_t OpAddress;        /* The NV Memory address. The destination address in most cases */
    size_t OpSize;              /* Outer loop size. Retain for ContinueWrite. Total bytes at start, aligned */
    const uint8_t * p_OpData;   /* Op data source. internal buffer, caller provided address, written only in read case */

    /* Derived */
    const NvMemory_Partition_T * p_OpPartition; /* Op Dest boundary check */
    size_t OpSizeAligned;       /* Cached calculation. alternatively use getter */
    // uint8_t * p_buffer, // keep separate pointer for buffer?
    // size_t BytesPerCmd; overwrite if not equal unit size p_this->BytesPerCmd = unitsPerCmd * unitSize

    /* Nonblocking use only */
    NvMemory_State_T State;
    NvMemory_Status_T Status;
    size_t OpIndex;     /* in bytes */
    // NvMemory_Callback_T OnComplete;     /*!< OnComplete */
}
NvMemory_T;

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

/*
    ReadOnce must provide buffer here, or in calling function
*/
#define NV_MEMORY_INIT(p_Hal, p_HAL_ReadCompleteFlag, p_HAL_ReadErrorFlags, p_HAL_ClearErrorFlags, p_Partitions, PartitionsCount, p_Buffer, BufferSize) \
{                                                                                                                                                       \
    .CONST =                                                                                                                                            \
    {                                                                                                                                                   \
        _NV_MEMORY_INIT_HAL(p_Hal, p_HAL_ReadCompleteFlag, p_HAL_ReadErrorFlags, p_HAL_ClearErrorFlags)                                                 \
        _NV_MEMORY_INIT_PARTITIONS(p_Partitions, PartitionsCount)                                                                                       \
        _NV_MEMORY_INIT_BUFFER(p_Buffer, BufferSize)                                                                                                    \
    }                                                                                                                                                   \
}

/* Alternatively Cached on SetOp */
static size_t NvMemory_GetOpSizeAligned(NvMemory_T * p_this)    { p_this->p_OpControl->FORCE_ALIGN(p_this->OpSize, p_this->p_OpControl->UNIT_SIZE); }
static size_t NvMemory_GetOpSizeRemainder(NvMemory_T * p_this)  { p_this->OpSize - nvmemory_align_down(p_this->OpSize, p_this->p_OpControl->UNIT_SIZE); }

static inline void NvMemory_EnableForceAlign(NvMemory_T * p_this) { p_this->IsForceAlignEnable = true; }
static inline void NvMemory_DisableForceAlign(NvMemory_T * p_this) { p_this->IsForceAlignEnable = false; }

static inline void NvMemory_SetYield(NvMemory_T * p_this, NvMemory_Callback_T yield, void * p_callbackData)
{
    p_this->Yield = yield;
    p_this->p_CallbackContext = p_callbackData;
}


/*
    Extern
*/

extern void NvMemory_Init(NvMemory_T * p_this);
extern bool NvMemory_CheckOpChecksum(const NvMemory_T * p_this, const uint8_t * p_source, size_t size);

extern NvMemory_Status_T NvMemory_MemCompare(const uint8_t * p_dest, const uint8_t * p_source, size_t size);
/* Destination is not intended to be dereference, pass as uintptr_t type */
extern NvMemory_Status_T NvMemory_SetOpAddress(NvMemory_T * p_this, uintptr_t address, size_t opSize);
extern NvMemory_Status_T NvMemory_SetOpData(NvMemory_T * p_this, const uint8_t * p_data, size_t opSize);
extern NvMemory_Status_T NvMemory_SetOpSize(NvMemory_T * p_this, size_t opSize);
extern NvMemory_Status_T NvMemory_SetOpControl(NvMemory_T * p_this, const NvMemory_OpControl_T * p_opControl, uintptr_t address, size_t size);
extern NvMemory_Status_T NvMemory_SetOpControl_Read(NvMemory_T * p_this, const NvMemory_OpControl_T * p_opControl, uintptr_t address, size_t size, uint8_t * p_data);
extern NvMemory_Status_T NvMemory_SetOpControl_Write(NvMemory_T * p_this, const NvMemory_OpControl_T * p_opControl, uintptr_t address, const uint8_t * p_data, size_t size);


/*
    Blocking
*/
extern NvMemory_Status_T NvMemory_ProcOp_Blocking(NvMemory_T * p_this) CONFIG_NV_MEMORY_ATTRIBUTE_RAM_SECTION;

/*
    Non Blocking
*/
extern size_t NvMemory_GetOpBytesRemaining(NvMemory_T * p_this);
extern bool NvMemory_ProcOp(NvMemory_T * p_this);
extern NvMemory_Status_T NvMemory_StartOp(NvMemory_T * p_this);

#endif /* NV_MEMORY_H */


