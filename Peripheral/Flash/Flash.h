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
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef FLASH_H
#define FLASH_H

#include "HAL_Flash.h"
#include "Config.h"
//#include "System/Queue/Queue.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef const struct
{
	uint8_t * P_START;
	size_t SIZE;

#ifdef CONFIG_FLASH_HW_OP_SIZE_PER_PARITION
	size_t WRITE_SIZE;
	size_t ERASE_SIZE;
	size_t VERIFY_WRITE_SIZE;
	size_t VERIFY_ERASE_SIZE;
#endif

#ifdef CONFIG_FLASH_HW_OP_ADDRESS_RELATIVE
	int32_t OP_ADDRESS_OFFSET;
#endif
//	uint8_t Alignment;
}
Flash_Partition_T;

typedef const struct
{
	HAL_Flash_T * P_HAL_FLASH;	//flash controller registers
	Flash_Partition_T PARTITIONS[CONFIG_FLASH_PARTITION_COUNT];
}
Flash_Config_T;

typedef enum
{
	FLASH_STATUS_SUCCESS,
	FLASH_STATUS_PROCESSING,
	FLASH_STATUS_START_VERIFY,
	FLASH_STATUS_ERROR,
	FLASH_STATUS_ERROR_BUSY,
	FLASH_STATUS_ERROR_INPUT,		/* op param input */
	FLASH_STATUS_ERROR_CMD,			/* flash controller error */
	FLASH_STATUS_ERROR_VERIFY,		/* Verify cmd*/
	FLASH_STATUS_ERROR_CHECKSUM,	/*  */
}
Flash_Status_T;

typedef enum
{
	FLASH_OPERATION_WRITE,
	FLASH_OPERATION_ERASE,
	FLASH_OPERATION_VERIFY_WRITE,
	FLASH_OPERATION_VERIFY_ERASE,
	FLASH_OPERATION_WRITE_ONCE,
	FLASH_OPERATION_READ_ONCE,
}
Flash_Operation_T;

typedef enum
{
	FLASH_STATE_IDLE,
	FLASH_STATE_ACTIVE,
	FLASH_STATE_WRITE,
	FLASH_STATE_VERIFY,
}
Flash_State_T;

//Flash controller
typedef struct
{
	Flash_Config_T CONFIG;
//	Queue_T Queue; //data queue for nonblocking operation
	//job quee

	//for virtual buffer, non blocking op
	//static allocate size, only be 1 instance of flash is expected
	uint8_t Buffer[CONFIG_FLASH_BUFFER_SIZE];

	bool IsVerifyEnable;
	bool IsOpBuffered; //copy to buffer first or use pointer
	//	bool IsForceAlignEnable;

	const uint8_t * volatile p_OpDest;
	const uint8_t * volatile p_OpData;
	volatile size_t OpSize; 	//total bytes at start
	volatile size_t BytesPerCmd; //only for erase
	volatile size_t UnitsPerCmd;

	//Nonblocking use only
	volatile Flash_State_T State;
//	volatile Flash_Operation_T OpType;
	volatile Flash_Status_T Status;
	volatile size_t OpIndex; 	//in page/phrase

	const Flash_Partition_T * volatile p_OpPartition; //op dest

	void (*StartCmd)(HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units);
	Flash_Status_T (*FinalizeOp)(void * p_this);
//	Flash_Status_T (*ParseErrorCode)(void * p_this);


    void * p_CallbackData;
    void (* OnComplete)(void * p_callbackData);    /*!< OnComplete */
    void (* Yield)(void * p_callbackData);    		/*!<  On Block*/
//    void (* Callback)(void *);    	/*!<  Union*/
} Flash_T;

//extern Flash_Status_T Flash_StartVerifyWrite(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size);
//extern Flash_Status_T Flash_StartVerifyErase(Flash_T * p_flash, const uint8_t * p_destFlash, size_t size);

#endif /* FLASH_H */

