/******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSourcery / The Firebrand Forge Inc

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
	@brief	Non Volatile Memory Operations. Abstract Base Class. Template Pattern.
	@version V0
*/
/******************************************************************************/
#ifndef NV_MEMORY_H
#define NV_MEMORY_H

#include "Config.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/*
	Common return status by public module functions
*/
typedef enum NvMemory_Status_Tag
{
	NV_MEMORY_STATUS_SUCCESS,
	NV_MEMORY_STATUS_PROCESSING, 		/* NonBlocking Only */
	NV_MEMORY_STATUS_START_VERIFY,		 /* Follow up Op after complete */
	// NV_MEMORY_STATUS_ERROR,
	NV_MEMORY_STATUS_ERROR_BUSY,
	NV_MEMORY_STATUS_ERROR_INPUT,		/* op params, dest address or size */ //todo   parse error destination, align
	NV_MEMORY_STATUS_ERROR_BOUNDARY,	/* Dest, size, larger than partition */
	NV_MEMORY_STATUS_ERROR_ALIGNMENT,	/* dest, size, not aligned */
	NV_MEMORY_STATUS_ERROR_CMD,			/* Cmd Failed. Unparsed Error */
	NV_MEMORY_STATUS_ERROR_VERIFY,		/* Verify cmd */
	NV_MEMORY_STATUS_ERROR_PROTECTION,
	NV_MEMORY_STATUS_ERROR_BUFFER,		/* Virtual buffer */
	NV_MEMORY_STATUS_ERROR_CHECKSUM,	/*  */
	NV_MEMORY_STATUS_ERROR_INVALID_OP,
}
NvMemory_Status_T;

/*
	Partition Group
	Partition defines writable ranges. Checked on op cmd.
*/
typedef const struct NvMemory_Partition_Tag
{
	uint8_t * P_START;
	size_t SIZE;
#ifdef CONFIG_NV_MEMORY_HW_OP_ADDRESS_RELATIVE /* or HAL functions handle offset */
	int32_t OP_ADDRESS_OFFSET;
#endif
	//	uint8_t Alignment;
}
NvMemory_Partition_T;

#if defined(CONFIG_NV_MEMORY_HW_OP_ADDRESS_RELATIVE)
#define _NV_MEMORY_INIT_PARTITION_OFFSET(OpAddressOffset) .OP_ADDRESS_OFFSET = OpAddressOffset,
#else
#define _NV_MEMORY_INIT_PARTITION_OFFSET(OpAddressOffset)
#endif

#define NV_MEMORY_INIT_PARTITION(p_Start, SizeBytes, OpAddressOffset)	\
{																		\
	.P_START 	= (void *)p_Start,  									\
	.SIZE 		= SizeBytes, 											\
	_NV_MEMORY_INIT_PARTITION_OFFSET(OpAddressOffset)					\
}

/*
	NvMemory typedefs
*/
typedef bool (* const HAL_NvMemory_ReadFlags_T)(const void * p_hal);
typedef void (* const HAL_NvMemory_ClearFlags_T)(void * p_hal);
typedef void (*HAL_NvMemory_StartCmd_T)(void * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units);

struct NvMemory_Tag;
typedef NvMemory_Status_T(*NvMemory_Process_T)(struct NvMemory_Tag * p_this);
typedef NvMemory_Status_T(*NvMemory_FinalizeCmd_T)(struct NvMemory_Tag * p_this, size_t opIndex);
typedef void (*NvMemory_Callback_T)(void * p_callbackData);

typedef enum NvMemory_State_Tag
{
	NV_MEMORY_STATE_IDLE,
	NV_MEMORY_STATE_ACTIVE,
	NV_MEMORY_STATE_WRITE,
	NV_MEMORY_STATE_VERIFY,
}
NvMemory_State_T;

typedef const struct NvMemory_Config_Tag
{
	void * const P_HAL;
	/* template functions provided by concrete child class */
	const HAL_NvMemory_ReadFlags_T READ_COMPLETE_FLAG;
	const HAL_NvMemory_ReadFlags_T READ_ERROR_FLAGS;
	const HAL_NvMemory_ClearFlags_T CLEAR_ERROR_FLAGS;
	NvMemory_Partition_T * const P_PARTITIONS;
	const uint8_t PARTITION_COUNT;
	uint8_t * const P_BUFFER; /* IsForceAlignEnable need buffer of UNIT_WRITE_SIZE for fill with erase pattern. (Even if IsOpBuffered is not used ) */
	const size_t BUFFER_SIZE;
}
NvMemory_Config_T;

// typedef const struct NvMemory_OpControl_Tag
// {
// 	HAL_NvMemory_StartCmd_T Start_Cmd;
// 	NvMemory_FinalizeCmd_T Finalize_Cmd;
// 	NvMemory_Process_T Parse_Cmd_Error;
// 	NvMemory_Process_T Finalize_Op;
// 	size_t Units_Per_Cmd;
// }
// NvMemory_OpControl_T;

/*
	NvMemory controller
*/
typedef struct NvMemory_Tag
{
	NvMemory_Config_T CONFIG;

	bool IsVerifyEnable;
	bool IsOpBuffered; 	/* copy to buffer first or use pointer to source */
	bool IsForceAlignEnable;

	const uint8_t * p_OpDest;
	const uint8_t * p_OpData;
	size_t OpSize; 			/* Total bytes at start */
	size_t BytesPerCmd; 	/* Used by Erase for now */
	size_t UnitsPerCmd;
	size_t ForceAlignBytes;

	const NvMemory_Partition_T * p_OpPartition; /* Op Dest */

	HAL_NvMemory_StartCmd_T StartCmd;		/* Start HAL Cmd, many iterations per Op */
	NvMemory_FinalizeCmd_T FinalizeCmd; 	/* On end per Cmd iteration */
	NvMemory_Process_T ParseCmdError;		/* On end all Cmd iteration, Op Complete Error */
	NvMemory_Process_T FinalizeOp;			/* On end all Cmd iteration, Op Complete Sucess */

	void * p_CallbackData;
	NvMemory_Callback_T Yield; 			/*!< On Block */
	// NvMemory_Callback_T OnComplete; 	/*!< OnComplete */

	/* Nonblocking use only */
	NvMemory_State_T State;
	NvMemory_Status_T Status;
	size_t OpIndex; 	/* in bytes */
}
NvMemory_T;

/*
	Alternatively template the calling function
*/
#define _NV_MEMORY_INIT_HAL(p_Hal, ReadCompleteFlag, ReadErrorFlags, ClearErrorFlags)	\
	.P_HAL = p_Hal,  																	\
	.READ_COMPLETE_FLAG 	= (HAL_NvMemory_ReadFlags_T)ReadCompleteFlag,				\
	.READ_ERROR_FLAGS 		= (HAL_NvMemory_ReadFlags_T)ReadErrorFlags,					\
	.CLEAR_ERROR_FLAGS 		= (HAL_NvMemory_ClearFlags_T)ClearErrorFlags,

#define _NV_MEMORY_INIT_PARTITIONS(p_Partitions, PartitionsCount)	\
	.P_PARTITIONS 		= p_Partitions,  							\
	.PARTITION_COUNT 	= PartitionsCount,

#define _NV_MEMORY_INIT_BUFFER(p_Buffer, BufferSize)	\
	.P_BUFFER 		= p_Buffer,  						\
	.BUFFER_SIZE 	= BufferSize,

#define NV_MEMORY_INIT(p_Hal, p_HAL_ReadCompleteFlag, p_HAL_ReadErrorFlags, p_HAL_ClearErrorFlags, p_Partitions, PartitionsCount, p_Buffer, BufferSize)	\
{																																						\
	.CONFIG = 																																			\
	{																																					\
		_NV_MEMORY_INIT_HAL(p_Hal, p_HAL_ReadCompleteFlag, p_HAL_ReadErrorFlags, p_HAL_ClearErrorFlags)  												\
		_NV_MEMORY_INIT_PARTITIONS(p_Partitions, PartitionsCount) 																						\
		_NV_MEMORY_INIT_BUFFER(p_Buffer, BufferSize)			 																						\
	}																																					\
}


extern void NvMemory_Init(NvMemory_T * p_this);
extern void NvMemory_SetYield(NvMemory_T * p_this, void (*yield)(void *), void * p_callbackData);
extern void NvMemory_EnableForceAlign(NvMemory_T * p_this);
extern void NvMemory_DisableForceAlign(NvMemory_T * p_this);

extern void NvMemory_SetOpCmdSize(NvMemory_T * p_this, size_t unitSize, uint8_t unitsPerCmd);
extern void NvMemory_SetOpFunctions(NvMemory_T * p_this, HAL_NvMemory_StartCmd_T startCmd, NvMemory_Process_T finalizeOp, NvMemory_Process_T parseCmdError);
extern bool NvMemory_CheckOpChecksum(const NvMemory_T * p_this);

extern NvMemory_Status_T NvMemory_SetOpDest(NvMemory_T * p_this, const uint8_t * p_dest, size_t opSize, size_t unitSize);
extern NvMemory_Status_T NvMemory_SetOpSourceData(NvMemory_T * p_this, const uint8_t * p_source, size_t size);
extern NvMemory_Status_T NvMemory_SetOpSizeNoAlign(NvMemory_T * p_this, size_t opSize, size_t unitSize);
extern NvMemory_Status_T NvMemory_SetOpSizeAlignDown(NvMemory_T * p_this, size_t opSize, size_t unitSize);
extern NvMemory_Status_T NvMemory_SetOpSizeAlignUp(NvMemory_T * p_this, size_t opSize, size_t unitSize);

/*
	Blocking
*/
/* Store in RAM for case of Flash */
extern NvMemory_Status_T NvMemory_ProcOp_Blocking(NvMemory_T * p_this) CONFIG_NV_MEMORY_ATTRIBUTE_RAM_SECTION;

/*
	Non Blocking
*/
extern size_t NvMemory_GetOpBytesRemaining(NvMemory_T * p_this);
extern bool NvMemory_ProcOp(NvMemory_T * p_this) CONFIG_NV_MEMORY_ATTRIBUTE_RAM_SECTION;
extern NvMemory_Status_T NvMemory_StartOp(NvMemory_T * p_this);

#endif /* NV_MEMORY_H */

