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

#include "Peripheral/NvMemory/NvMemory/NvMemory.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define FLASH_UNIT_ERASE_SIZE			HAL_FLASH_UNIT_ERASE_SIZE
#define FLASH_UNIT_WRITE_SIZE			HAL_FLASH_UNIT_WRITE_SIZE
#define FLASH_UNIT_VERIFY_ERASE_SIZE	HAL_FLASH_UNIT_VERIFY_ERASE_SIZE
#define FLASH_UNIT_VERIFY_WRITE_SIZE	HAL_FLASH_UNIT_VERIFY_WRITE_SIZE
#define FLASH_UNIT_WRITE_ONCE_SIZE		HAL_FLASH_UNIT_WRITE_ONCE_SIZE
#define FLASH_UNIT_READ_ONCE_SIZE		HAL_FLASH_UNIT_READ_ONCE_SIZE

typedef NvMemory_Partition_T Flash_Partition_T;
typedef NvMemory_T Flash_T;

#define FLASH_CONFIG(p_Hal, p_Partitions, PartitionCount, p_Buffer, BufferSize) \
	NV_MEMORY_CONFIG(p_Hal, HAL_Flash_ReadCompleteFlag, HAL_Flash_ReadErrorFlags, HAL_Flash_ClearErrorFlags, p_Partitions, PartitionCount, p_Buffer, BufferSize)														\

/*
 * Alias for NvMemory Status
 */
typedef enum
{
	FLASH_STATUS_SUCCESS 			= NV_MEMORY_STATUS_SUCCESS,
	FLASH_STATUS_PROCESSING 		= NV_MEMORY_STATUS_PROCESSING,
	FLASH_STATUS_START_VERIFY 		= NV_MEMORY_STATUS_START_VERIFY,
	FLASH_STATUS_ERROR 				= NV_MEMORY_STATUS_ERROR,
	FLASH_STATUS_ERROR_BUSY 		= NV_MEMORY_STATUS_ERROR_BUSY,
	FLASH_STATUS_ERROR_INPUT 		= NV_MEMORY_STATUS_ERROR_INPUT, 	/* op param input: address destination, align */
	FLASH_STATUS_ERROR_CMD 			= NV_MEMORY_STATUS_ERROR_CMD, 		/* flash controller error */
	FLASH_STATUS_ERROR_VERIFY 		= NV_MEMORY_STATUS_ERROR_VERIFY, 	/* Verify cmd */
	FLASH_STATUS_ERROR_PROTECTION 	= NV_MEMORY_STATUS_ERROR_PROTECTION,
	FLASH_STATUS_ERROR_CHECKSUM 	= NV_MEMORY_STATUS_ERROR_CHECKSUM, 	/*  */
} Flash_Status_T;

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

#endif /* FLASH_H */

