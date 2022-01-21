/******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSoucery / The Firebrand Forge Inc

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
    @author FireSoucery
	@brief	Non Volatile Memory Write
    		Abstract Base Class Template Pattern
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
 * Partition defines writable ranges. Checked on write cmd.
 */
typedef const struct
{
	uint8_t * P_START;
	size_t SIZE;
#ifdef CONFIG_NV_MEMORY_HW_OP_ADDRESS_RELATIVE
	int32_t OP_ADDRESS_OFFSET;
#endif
//	uint8_t Alignment;
}
NvMemory_Partition_T;

#define NV_MEMORY_PARTITION(p_Start, Size)	\
{											\
	.P_START 	= p_Start,  				\
	.SIZE 		= Size, 					\
}

typedef bool (*const NvMemory_HalReadFlags_T) (const void * p_hal);
typedef void (*const NvMemory_HalClearFlags_T) (void * p_hal);

typedef const struct
{
	void * P_HAL;
	/* template functions provided by concrete child class */
	bool (*const READ_COMPLETE_FLAG)(const void * p_hal);
	bool (*const READ_ERROR_FLAGS)(const void * p_hal);
	void (*const CLEAR_ERROR_FLAGS)(void * p_hal);

	NvMemory_Partition_T * P_PARTITIONS;
	const uint8_t PARTITION_COUNT;

	uint8_t * P_BUFFER;
	size_t BUFFER_SIZE;
}
NvMemory_Config_T;

/*
 * Alternatively template the calling function
 */
#define NV_MEMORY_CONFIG_HAL(p_Hal, ReadCompleteFlag, ReadErrorFlags, ClearErrorFlags)	\
	.P_HAL 		= p_Hal,  																\
	.READ_COMPLETE_FLAG 	= (NvMemory_HalReadFlags_T)ReadCompleteFlag,				\
	.READ_ERROR_FLAGS 		= (NvMemory_HalReadFlags_T)ReadErrorFlags,					\
	.CLEAR_ERROR_FLAGS 		= (NvMemory_HalClearFlags_T)ClearErrorFlags,

#define NV_MEMORY_CONFIG_PARTITIONS(p_partitions, partitionCount)	\
	.P_PARTITIONS 		= p_partitions,  							\
	.PARTITION_COUNT 	= partitionCount, 							\

#define NV_MEMORY_CONFIG_BUFFER(p_Buffer, BufferSize)	\
	.P_BUFFER 		= p_Buffer,  						\
	.BUFFER_SIZE 	= BufferSize, 						\

#define NV_MEMORY_CONFIG(p_Hal, p_HAL_ReadCompleteFlag, p_HAL_ReadErrorFlags, p_HAL_ClearErrorFlags, p_Partitions, PartitionCount, p_Buffer, BufferSize)	\
{																																	\
	.CONFIG = 																														\
	{																																\
		NV_MEMORY_CONFIG_HAL(p_Hal, p_HAL_ReadCompleteFlag, p_HAL_ReadErrorFlags, p_HAL_ClearErrorFlags)  							\
		NV_MEMORY_CONFIG_PARTITIONS(p_Partitions, PartitionCount) 																	\
		NV_MEMORY_CONFIG_BUFFER(p_Buffer, BufferSize)			 																	\
	}																																\
}

typedef enum
{
	NV_MEMORY_STATUS_SUCCESS,
	NV_MEMORY_STATUS_PROCESSING,
	NV_MEMORY_STATUS_START_VERIFY,
	NV_MEMORY_STATUS_ERROR,
	NV_MEMORY_STATUS_ERROR_BUSY,
	NV_MEMORY_STATUS_ERROR_INPUT,		/* op param input */ //todo can parse eroor destination, align
	NV_MEMORY_STATUS_ERROR_CMD,			/* flash controller error */
	NV_MEMORY_STATUS_ERROR_VERIFY,		/* Verify cmd */
	NV_MEMORY_STATUS_ERROR_PROTECTION,
	NV_MEMORY_STATUS_ERROR_CHECKSUM,	/*  */
}
NvMemory_Status_T;

typedef enum
{
	NV_MEMORY_STATE_IDLE,
	NV_MEMORY_STATE_ACTIVE,
	NV_MEMORY_STATE_WRITE,
	NV_MEMORY_STATE_VERIFY,
}
NvMemory_State_T;

typedef void (*NvMemory_StartCmd_T)(void * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units);

/*
 * NvMemory controller
 */
typedef struct NvMemory_Tag
{
	NvMemory_Config_T CONFIG;
//	Queue_T Queue; //job queue for nonblocking operation

	bool IsVerifyEnable;
	bool IsOpBuffered; //copy to buffer first or use pointer
	//	bool IsForceAlignEnable;

	/* */
	const uint8_t * p_OpDest;
	const uint8_t * p_OpData;
	size_t OpSize; 			//total bytes at start
	size_t BytesPerCmd; 	//only for erase
	size_t UnitsPerCmd;

	const NvMemory_Partition_T * p_OpPartition; //op dest

	void (*StartCmd)(void * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units);
	NvMemory_Status_T (*FinalizeOp)(struct NvMemory_Tag * p_this);
//	NvMemory_Status_T (*ParseErrorCode)(void * p_this);

	void * p_CallbackData;
//	void (*OnComplete)(void * p_callbackData); /*!< OnComplete */
	void (*Yield)(void * p_callbackData); /*!<  On Block*/
//    void (* Callback)(void *);    	/*!<  Union*/

	//Nonblocking use only
	NvMemory_State_T State;
	NvMemory_Status_T Status;
	size_t OpIndex; 	//in page/phrase
	//	  NvMemory_Operation_T OpType;
} NvMemory_T;

extern void NvMemory_Init(NvMemory_T * p_this);
extern void NvMemory_SetYield(NvMemory_T * p_this, void (*yield)(void *), void * p_callbackData);

extern NvMemory_Status_T NvMemory_SetOpCommon(NvMemory_T * p_this, const uint8_t * p_dest, size_t opSize, size_t unitSize);
extern void NvMemory_SetOpData(NvMemory_T * p_this, const uint8_t * p_source, size_t size);
extern void NvMemory_SetOpCmdSize(NvMemory_T * p_this, size_t unitSize, uint8_t unitsPerCmd);
extern NvMemory_Status_T NvMemory_ProcOpCommon_Blocking(NvMemory_T * p_this);

extern bool NvMemory_ProcOp(NvMemory_T * p_this);
extern NvMemory_Status_T NvMemory_StartOpCommon(NvMemory_T * p_this);

#endif /* NV_MEMORY_H */

