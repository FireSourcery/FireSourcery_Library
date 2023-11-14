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
// /*
//     Alias for NvMemory Status
// */
// typedef enum Flash_Status
// {
//     FLASH_STATUS_SUCCESS = NV_MEMORY_STATUS_SUCCESS,
//     FLASH_STATUS_PROCESSING = NV_MEMORY_STATUS_PROCESSING,
//     FLASH_STATUS_START_VERIFY = NV_MEMORY_STATUS_START_VERIFY,
//     // FLASH_STATUS_ERROR                 = NV_MEMORY_STATUS_ERROR,
//     FLASH_STATUS_ERROR_BUSY = NV_MEMORY_STATUS_ERROR_BUSY,
//     FLASH_STATUS_ERROR_INPUT = NV_MEMORY_STATUS_ERROR_INPUT,
//     FLASH_STATUS_ERROR_CMD = NV_MEMORY_STATUS_ERROR_CMD,
//     FLASH_STATUS_ERROR_VERIFY = NV_MEMORY_STATUS_ERROR_VERIFY,
//     FLASH_STATUS_ERROR_PROTECTION = NV_MEMORY_STATUS_ERROR_PROTECTION,
//     FLASH_STATUS_ERROR_CHECKSUM = NV_MEMORY_STATUS_ERROR_CHECKSUM,
//     FLASH_STATUS_ERROR_INVALID_OP = NV_MEMORY_STATUS_ERROR_INVALID_OP,
// } Flash_Status_T;

//tod replace
#define FLASH_STATUS_SUCCESS            (NV_MEMORY_STATUS_SUCCESS)
#define FLASH_STATUS_PROCESSING         (NV_MEMORY_STATUS_PROCESSING)
#define FLASH_STATUS_START_VERIFY       (NV_MEMORY_STATUS_START_VERIFY)
// #define FLASH_STATUS_ERROR           (NV_MEMORY_STATUS_ERROR)
#define FLASH_STATUS_ERROR_BUSY         (NV_MEMORY_STATUS_ERROR_BUSY)
#define FLASH_STATUS_ERROR_INPUT        (NV_MEMORY_STATUS_ERROR_INPUT)
#define FLASH_STATUS_ERROR_CMD          (NV_MEMORY_STATUS_ERROR_CMD)
#define FLASH_STATUS_ERROR_VERIFY       (NV_MEMORY_STATUS_ERROR_VERIFY)
#define FLASH_STATUS_ERROR_PROTECTION   (NV_MEMORY_STATUS_ERROR_PROTECTION)
#define FLASH_STATUS_ERROR_CHECKSUM     (NV_MEMORY_STATUS_ERROR_CHECKSUM)
#define FLASH_STATUS_ERROR_INVALID_OP   (NV_MEMORY_STATUS_ERROR_INVALID_OP)

typedef NvMemory_Partition_T Flash_Partition_T;
typedef NvMemory_T Flash_T; /* Flash struct must reside in RAM */
// typedef NvMemory_T __attribute__((section(".data"))) Flash_T; /* Flash struct must reside in RAM */

#define FLASH_INIT(p_Hal, p_Partitions, PartitionCount, p_Buffer, BufferSize) \
    NV_MEMORY_INIT(p_Hal, HAL_Flash_ReadCompleteFlag, HAL_Flash_ReadErrorFlags, HAL_Flash_ClearErrorFlags, p_Partitions, PartitionCount, p_Buffer, BufferSize)

/*

*/
extern void Flash_Init(Flash_T * p_flash);
extern void Flash_SetYield(Flash_T * p_flash, void (*yield)(void *), void * p_callbackData);
extern bool Flash_ReadSecurityFlag(Flash_T * p_flash);
extern void Flash_EnableForceAlign(Flash_T * p_flash);
extern void Flash_DisableForceAlign(Flash_T * p_flash);

extern Flash_Status_T Flash_SetWrite(Flash_T * p_flash, const uint8_t * p_dest, const uint8_t * p_data, size_t size);
extern Flash_Status_T Flash_SetErase(Flash_T * p_flash, const uint8_t * p_dest, size_t size);
extern Flash_Status_T Flash_SetVerifyWrite(Flash_T * p_flash, const uint8_t * p_dest, const uint8_t * p_data, size_t size);
extern Flash_Status_T Flash_SetVerifyErase(Flash_T * p_flash, const uint8_t * p_dest, size_t size);
extern Flash_Status_T Flash_SetWriteOnce(Flash_T * p_flash, const uint8_t * p_dest, const uint8_t * p_data, size_t size);
extern Flash_Status_T Flash_SetReadOnce(Flash_T * p_flash, uint8_t * p_dataResult, const uint8_t * p_destOnce, size_t size);
extern Flash_Status_T Flash_SetEraseAll(Flash_T * p_flash);
extern Flash_Status_T Flash_SetOp(Flash_T * p_flash, const uint8_t * p_dest, const uint8_t * p_data, size_t size, Flash_Operation_T opId);
// extern void Flash_GetReadOnce(const Flash_T * p_flash, uint8_t * p_result);

extern Flash_Status_T Flash_ProcThisOp_Blocking(Flash_T * p_flash); //CONFIG_NV_MEMORY_ATTRIBUTE_RAM_SECTION;

extern Flash_Status_T Flash_Write_Blocking(Flash_T * p_flash, const uint8_t * p_dest, const uint8_t * p_data, size_t size);
extern Flash_Status_T Flash_ContinueWrite_Blocking(Flash_T * p_flash, const uint8_t * p_data, size_t size);
extern Flash_Status_T Flash_Erase_Blocking(Flash_T * p_flash, const uint8_t * p_dest, size_t size);
extern Flash_Status_T Flash_VerifyWrite_Blocking(Flash_T * p_flash, const uint8_t * p_dest, const uint8_t * p_data, size_t size);
extern Flash_Status_T Flash_VerifyErase_Blocking(Flash_T * p_flash, const uint8_t * p_dest, size_t size);
extern Flash_Status_T Flash_WriteOnce_Blocking(Flash_T * p_flash, const uint8_t * p_dest, const uint8_t * p_data, size_t size);
extern Flash_Status_T Flash_ReadOnce_Blocking(Flash_T * p_flash, uint8_t * p_dataResult, const uint8_t * p_once, size_t size);
extern Flash_Status_T Flash_EraseAll_Blocking(Flash_T * p_flash);
extern Flash_Status_T Flash_ProcOp_Blocking(Flash_T * p_flash, const uint8_t * p_dest, const uint8_t * p_data, size_t size, Flash_Operation_T opId);

extern bool Flash_ProcOp(Flash_T * p_flash);
extern size_t Flash_GetOpBytesRemaining(Flash_T * p_flash);
// extern bool Flash_ReadIsOpComplete(Flash_T * p_flash);
extern Flash_Status_T Flash_StartWrite_NonBlocking(Flash_T * p_flash, const uint8_t * p_dest, const uint8_t * p_data, size_t size);
extern Flash_Status_T Flash_StartErase_NonBlocking(Flash_T * p_flash, const uint8_t * p_dest, size_t size);
extern Flash_Status_T Flash_StartVerifyWrite_NonBlocking(Flash_T * p_flash, const uint8_t * p_dest, const uint8_t * p_data, size_t size);
extern Flash_Status_T Flash_StartVerifyErase_NonBlocking(Flash_T * p_flash, const uint8_t * p_dest, size_t size);
extern Flash_Status_T Flash_StartWriteOnce_NonBlocking(Flash_T * p_flash, const uint8_t * p_dest, const uint8_t * p_data, size_t size);
extern Flash_Status_T Flash_StartReadOnce_NonBlocking(Flash_T * p_flash, uint8_t * p_dataResult, const uint8_t * p_destOnce, size_t size);

#endif /* FLASH_H */

