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

#define FLASH_UNIT_ERASE_SIZE           HAL_FLASH_UNIT_ERASE_SIZE
#define FLASH_UNIT_WRITE_SIZE           HAL_FLASH_UNIT_WRITE_SIZE
#define FLASH_UNIT_VERIFY_ERASE_SIZE    HAL_FLASH_UNIT_VERIFY_ERASE_SIZE
#define FLASH_UNIT_VERIFY_WRITE_SIZE    HAL_FLASH_UNIT_VERIFY_WRITE_SIZE
#define FLASH_UNIT_WRITE_ONCE_SIZE      HAL_FLASH_UNIT_WRITE_ONCE_SIZE
#define FLASH_UNIT_READ_ONCE_SIZE       HAL_FLASH_UNIT_READ_ONCE_SIZE
#define FLASH_UNIT_ERASE_PATTERN        HAL_FLASH_UNIT_ERASE_PATTERN

typedef enum Flash_Operation
{
    FLASH_OPERATION_WRITE,
    FLASH_OPERATION_ERASE,
    FLASH_OPERATION_VERIFY_WRITE,
    FLASH_OPERATION_VERIFY_ERASE,
    FLASH_OPERATION_WRITE_ONCE,
    FLASH_OPERATION_READ_ONCE,
}
Flash_Operation_T;

typedef NvMemory_Status_T Flash_Status_T;
typedef NvMemory_Partition_T Flash_Partition_T;
typedef NvMemory_T Flash_T; /* Flash struct must reside in RAM CONFIG_NV_MEMORY_ATTRIBUTE_RAM_SECTION */

#define FLASH_INIT(p_Hal, p_Partitions, PartitionCount, p_Buffer, BufferSize) \
    NV_MEMORY_INIT(p_Hal, HAL_Flash_ReadCompleteFlag, HAL_Flash_ReadErrorFlags, HAL_Flash_ClearErrorFlags, p_Partitions, PartitionCount, p_Buffer, BufferSize)

static inline bool Flash_ReadSecurityFlag(Flash_T * p_flash) { return HAL_Flash_ReadSecurityFlag(p_flash->CONST.P_HAL); }

/* Yield must point to RAM address for Flash case  */
static inline void Flash_SetYield(Flash_T * p_flash, NvMemory_Callback_T yield, void * p_callbackData) { NvMemory_SetYield(p_flash, yield, p_callbackData); }
static inline void Flash_EnableFillAlign(Flash_T * p_flash) { NvMemory_EnableForceAlign(p_flash); }
static inline void Flash_DisableFillAlign(Flash_T * p_flash) { NvMemory_DisableForceAlign(p_flash); }

/*
    extern
*/
extern void Flash_Init(Flash_T * p_flash);

extern Flash_Status_T Flash_SetContinueWrite(Flash_T * p_flash, uintptr_t flashAddress, size_t size);
extern Flash_Status_T Flash_ContinueWrite_Blocking(Flash_T * p_flash, const uint8_t * p_data, size_t size);

extern Flash_Status_T Flash_Write_Blocking(Flash_T * p_flash, uintptr_t flashAddress, const uint8_t * p_data, size_t size);
extern Flash_Status_T Flash_Erase_Blocking(Flash_T * p_flash, uintptr_t flashAddress, size_t size);
extern Flash_Status_T Flash_VerifyWrite_Blocking(Flash_T * p_flash, uintptr_t flashAddress, const uint8_t * p_data, size_t size);
extern Flash_Status_T Flash_VerifyErase_Blocking(Flash_T * p_flash, uintptr_t flashAddress, size_t size);
extern Flash_Status_T Flash_EraseAll_Blocking(Flash_T * p_flash);
extern Flash_Status_T Flash_WriteOnce_Blocking(Flash_T * p_flash, uintptr_t flashAddress, const uint8_t * p_data, size_t size);
extern Flash_Status_T Flash_ReadOnce_Blocking(Flash_T * p_flash, uintptr_t flashAddress, size_t size, uint8_t * p_resultBuffer);

// extern Flash_Status_T Flash_ProcOp_Blocking(Flash_T * p_flash, Flash_Operation_T opId, uintptr_t flashAddress, const uint8_t * p_data, size_t size);

// extern bool Flash_ProcOp(Flash_T * p_flash);
// extern size_t Flash_GetOpBytesRemaining(Flash_T * p_flash);
// extern bool Flash_ReadIsOpComplete(Flash_T * p_flash);
// extern Flash_Status_T Flash_StartWrite_NonBlocking(Flash_T * p_flash, uintptr_t flashAddress, const uint8_t * p_data, size_t size);
// extern Flash_Status_T Flash_StartErase_NonBlocking(Flash_T * p_flash, uintptr_t flashAddress, size_t size);
// extern Flash_Status_T Flash_StartVerifyWrite_NonBlocking(Flash_T * p_flash, uintptr_t flashAddress, const uint8_t * p_data, size_t size);
// extern Flash_Status_T Flash_StartVerifyErase_NonBlocking(Flash_T * p_flash, uintptr_t flashAddress, size_t size);
// extern Flash_Status_T Flash_StartWriteOnce_NonBlocking(Flash_T * p_flash, uintptr_t flashAddress, const uint8_t * p_data, size_t size);
// extern Flash_Status_T Flash_StartReadOnce_NonBlocking(Flash_T * p_flash, uint8_t * p_resultBuffer, uintptr_t flashAddressOnce, size_t size);
// extern void Flash_GetReadOnce(const Flash_T * p_flash, uint8_t * p_result);

#endif /* FLASH_H */

