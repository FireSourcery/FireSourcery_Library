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

/*
    Common return status by public module functions
*/
typedef enum NvMemory_Status
{
    NV_MEMORY_STATUS_SUCCESS,
    NV_MEMORY_STATUS_PROCESSING,        /* NonBlocking Only */
    NV_MEMORY_STATUS_START_VERIFY,      /* Follow up Op after complete */ //todo remove
    // NV_MEMORY_STATUS_ERROR,
    NV_MEMORY_STATUS_ERROR_BUSY,
    NV_MEMORY_STATUS_ERROR_INPUT,       /* op params, dest address or size */ //todo   parse error destination, align
    NV_MEMORY_STATUS_ERROR_BOUNDARY,    /* Dest, size, larger than partition */
    NV_MEMORY_STATUS_ERROR_ALIGNMENT,   /* Dest, size, not aligned */
    NV_MEMORY_STATUS_ERROR_CMD,         /* Cmd Failed. Unparsed Error */
    NV_MEMORY_STATUS_ERROR_VERIFY,      /* Verify cmd */
    NV_MEMORY_STATUS_ERROR_PROTECTION,
    NV_MEMORY_STATUS_ERROR_CHECKSUM,    /*  */
    NV_MEMORY_STATUS_ERROR_INVALID_OP,
    NV_MEMORY_STATUS_ERROR_BUFFER,      /* Virtual buffer */
    NV_MEMORY_STATUS_ERROR_OTHER,
}
NvMemory_Status_T;

/*
    Partition Group
    Partition defines writable ranges. Checked on op cmd.
    Only provides range check. Caller account for overlapping address id
*/
typedef const struct NvMemory_Partition
{
    uint8_t * P_START;
    size_t SIZE;
#ifdef CONFIG_NV_MEMORY_HW_OP_ADDRESS_RELATIVE /* or HAL functions handle offset */
    int32_t OP_ADDRESS_OFFSET;
#endif
    // uint8_t Alignment; per partition algin if not determined by cmd
}
NvMemory_Partition_T;

#if defined(CONFIG_NV_MEMORY_HW_OP_ADDRESS_RELATIVE)
#define _NV_MEMORY_INIT_PARTITION_OFFSET(OpAddressOffset) .OP_ADDRESS_OFFSET = OpAddressOffset,
#else
#define _NV_MEMORY_INIT_PARTITION_OFFSET(OpAddressOffset)
#endif

#define NV_MEMORY_INIT_PARTITION(p_Start, SizeBytes, OpAddressOffset)   \
{                                                                       \
    .P_START     = (void *)p_Start,                                     \
    .SIZE         = SizeBytes,                                          \
    _NV_MEMORY_INIT_PARTITION_OFFSET(OpAddressOffset)                   \
}

/*
    NvMemory typedefs
*/
typedef bool (* const HAL_NvMemory_ReadFlags_T) (const void * p_hal);
typedef void (* const HAL_NvMemory_ClearFlags_T) (void * p_hal);

typedef void (*HAL_NvMemory_StartCmd_T) (void * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units);
typedef void (*HAL_NvMemory_OnEndCmd_T) (void * p_hal, const uint8_t * p_cmdDest, uint8_t * p_cmdData, size_t units);
typedef NvMemory_Status_T(*HAL_NvMemory_CmdStatus_T) (const void * p_hal);

typedef void (*NvMemory_Callback_T)(void * p_callbackData);

/* VTable */
typedef const struct NvMemory_OpControl
{
    HAL_NvMemory_StartCmd_T START_CMD;          /* Start HAL Cmd, multiple iterations per Op */
    HAL_NvMemory_OnEndCmd_T FINALIZE_CMD;       /* On end per Cmd iteration */
    HAL_NvMemory_CmdStatus_T PARSE_CMD_ERROR;   /* On end all Cmd iteration, Op Complete Error */
    size_t UNIT_SIZE;                           /* Align Size */
    size_t(*FILL_ALIGN)(size_t opSize, size_t alignSize);

    // size_t CMD_UNITS;                           /* Used by EraseVerify for now */
    // size_t BytesPerCmd;     /* Used by Erase for now */
    // size_t UnitsPerCmd;     /* count of UNIT_SIZE */
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

typedef const struct NvMemory_Config
{
    void * const P_HAL;
    /* abstract functions provided by concrete child class */
    const HAL_NvMemory_ReadFlags_T READ_COMPLETE_FLAG;      /* Must reside in RAM, for Flash case*/
    const HAL_NvMemory_ReadFlags_T READ_ERROR_FLAGS;        /* Must reside in RAM, for Flash case*/
    const HAL_NvMemory_ClearFlags_T CLEAR_ERROR_FLAGS;
    const NvMemory_Partition_T * const P_PARTITIONS;
    const uint8_t PARTITION_COUNT;
    uint8_t * const P_BUFFER;
    const size_t BUFFER_SIZE;
}
NvMemory_Config_T;

/*
    NvMemory controller
*/
typedef struct NvMemory
{
    NvMemory_Config_T CONFIG;

    bool IsOpBuffered;      /* copy to buffer first or use pointer to source */
    bool IsVerifyEnable;    /* Auto call verify, alternatively user call */
    bool IsFillAlignEnable; /* if implement in this module, need buffer of UNIT_WRITE_SIZE for fill with erase pattern (Even if IsOpBuffered is not used ) */

    const NvMemory_OpControl_T * p_OpControl;
    const uint8_t * p_OpDest;
    const uint8_t * p_OpData;   /* Op data source. internal buffer, caller provided address */
    size_t OpSize;              /* Retain for ContinueWrite. Total bytes at start, aligned */
    size_t OpSizeAligned;
    // size_t OpSizeRemainder;     /* Remainder, retain total from user */
    // size_t BytesPerCmd; overwrite if not equal unit size p_this->BytesPerCmd = unitsPerCmd * unitSize
    const NvMemory_Partition_T * p_OpPartition; /* Op Dest boundary check */

    void * p_CallbackContext;
    NvMemory_Callback_T Yield;             /*!< On Block */

    /* Nonblocking use only */
    NvMemory_State_T State;
    NvMemory_Status_T Status;
    size_t OpIndex;     /* in bytes */
    // NvMemory_Callback_T OnComplete;     /*!< OnComplete */
}
NvMemory_T;

/*
    Vtable HAL so that parent class routines may be re used
    Alternatively template the calling function
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
    .CONFIG =                                                                                                                                           \
    {                                                                                                                                                   \
        _NV_MEMORY_INIT_HAL(p_Hal, p_HAL_ReadCompleteFlag, p_HAL_ReadErrorFlags, p_HAL_ClearErrorFlags)                                                 \
        _NV_MEMORY_INIT_PARTITIONS(p_Partitions, PartitionsCount)                                                                                       \
        _NV_MEMORY_INIT_BUFFER(p_Buffer, BufferSize)                                                                                                    \
    }                                                                                                                                                   \
}

/*!
    @param[in] num - address or size
    @param[in] align - unit must be power of 2
*/
static inline size_t NvMemory_AlignDown(size_t num, size_t align) { return ((num) & -(align)); }
static inline size_t NvMemory_AlignUp(size_t num, size_t align) { return (-(-(num) & -(align))); }
static inline bool NvMemory_IsAligned(size_t num, size_t align) { return ((num & (align - 1U)) == 0UL); }
static inline bool NvMemory_IsPtrAligned(const void * ptr, size_t align) { return NvMemory_IsAligned((uintptr_t)ptr, align); }

static inline void NvMemory_SetOpControl(NvMemory_T * p_this, NvMemory_OpControl_T * p_opControl) { p_this->p_OpControl = p_opControl; }

static inline void NvMemory_EnableFillAlign(NvMemory_T * p_this) { p_this->IsFillAlignEnable = true; }
static inline void NvMemory_DisableFillAlign(NvMemory_T * p_this) { p_this->IsFillAlignEnable = false; }

static inline void NvMemory_SetYield(NvMemory_T * p_this, NvMemory_Callback_T yield, void * p_callbackData)
{
    p_this->Yield = yield;
    p_this->p_CallbackContext = p_callbackData;
}
// extern void NvMemory_SetYield(NvMemory_T * p_this, NvMemory_Callback_T yield, void * p_callbackData);
// extern void NvMemory_EnableFillAlign(NvMemory_T * p_this);
// extern void NvMemory_DisableFillAlign(NvMemory_T * p_this);

extern void NvMemory_Init(NvMemory_T * p_this);
extern void NvMemory_SetOpCmdSize(NvMemory_T * p_this, size_t unitSize, uint8_t unitsPerCmd);
extern void NvMemory_SetOpControl(NvMemory_T * p_this, NvMemory_OpControl_T * p_opControl);
extern bool NvMemory_CheckOpChecksum(const NvMemory_T * p_this, const uint8_t * p_source, size_t size);
extern bool NvMemory_VerifyWrite(const NvMemory_T * p_this, const uint8_t * p_dest, const uint8_t * p_source, size_t size);

extern NvMemory_Status_T NvMemory_SetOpAddress(NvMemory_T * p_this, const uint8_t * p_dest, size_t opSize);
extern NvMemory_Status_T NvMemory_SetOpDataSource(NvMemory_T * p_this, const uint8_t * p_source, size_t opSize);
extern NvMemory_Status_T NvMemory_SetOpSize(NvMemory_T * p_this, size_t opSize);
// extern NvMemory_Status_T NvMemory_SetOpSizeAligned(NvMemory_T * p_this, size_t opSize, size_t unitSize);
// extern NvMemory_Status_T NvMemory_SetOpSizeAlignDown(NvMemory_T * p_this, size_t opSize, size_t unitSize);
// extern NvMemory_Status_T NvMemory_SetOpSizeAlignUp(NvMemory_T * p_this, size_t opSize, size_t unitSize);

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

// static void _HAL_Flash_LaunchCmd(HAL_Flash_T * p_regs);
// static inline void HAL_Flash_StartCmdEraseSector(HAL_Flash_T * p_regs, const uint8_t * p_dest);
// static inline void HAL_Flash_StartCmdEraseAll(HAL_Flash_T * p_regs);
// static inline void HAL_Flash_StartCmdWritePage(HAL_Flash_T * p_regs, const uint8_t * p_dest, const uint8_t * p_data);
// static inline void HAL_Flash_StartCmdVerifyWriteUnit(HAL_Flash_T * p_regs, const uint8_t * p_dest, const uint8_t * p_data);
// static inline void HAL_Flash_StartCmdWriteOnce(HAL_Flash_T * p_regs, const uint8_t * p_dest, const uint8_t * p_data);

// static inline void HAL_Flash_StartCmdReadOnce(HAL_Flash_T * p_regs, const uint8_t * p_dest);
// static inline void HAL_Flash_ReadOnceData(HAL_Flash_T * p_regs, uint8_t * p_result);
// static inline void HAL_Flash_StartCmdVerifyEraseUnit(HAL_Flash_T * p_regs, const uint8_t * p_dest);
// static inline void HAL_Flash_StartCmdVerifyEraseUnits(HAL_Flash_T * p_regs, const uint8_t * p_dest, uint16_t units);
