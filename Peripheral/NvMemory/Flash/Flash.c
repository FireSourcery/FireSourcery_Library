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
/******************************************************************************/
/*!
	Abstraction details
	Shared completion and error status -> simultaneous operations not supported.
*/
/******************************************************************************/
#include "Flash.h"
#include "HAL_Flash.h"
#include "Config.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#define FLASH_UNIT_ERASE_SIZE			HAL_FLASH_UNIT_ERASE_SIZE
#define FLASH_UNIT_WRITE_SIZE			HAL_FLASH_UNIT_WRITE_SIZE
#define FLASH_UNIT_VERIFY_ERASE_SIZE	HAL_FLASH_UNIT_VERIFY_ERASE_SIZE
#define FLASH_UNIT_VERIFY_WRITE_SIZE	HAL_FLASH_UNIT_VERIFY_WRITE_SIZE
#define FLASH_UNIT_WRITE_ONCE_SIZE		HAL_FLASH_UNIT_WRITE_ONCE_SIZE
#define FLASH_UNIT_READ_ONCE_SIZE		HAL_FLASH_UNIT_READ_ONCE_SIZE

//static inline const uint8_t * CalcOpCmdAddress(const Flash_T * p_flash, const uint8_t * p_dest) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;
//static inline const uint8_t * CalcOpCmdAddress(const Flash_T * p_flash, const uint8_t * p_dest)
//{
//#ifdef CONFIG_FLASH_HW_OP_ADDRESS_RELATIVE
//	return (uint8_t *)((uint32_t)p_dest + p_flash->p_OpPartition->OP_ADDRESS_OFFSET);
//#elif defined(CONFIG_FLASH_HW_OP_ADDRESS_ABSOLUTE)
//	return p_dest;
//#endif
//}

static void StartCmdWritePage 			(HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;
static void StartCmdEraseSector			(HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;
static void StartCmdVerifyWriteUnit		(HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;
static void StartCmdVerifyEraseUnits	(HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;
static void StartCmdWriteOnce			(HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;
static void StartCmdReadOnce			(HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;

static void StartCmdWritePage			(HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) {HAL_Flash_StartCmdWritePage			(p_hal, p_cmdDest, p_cmdData);}
static void StartCmdEraseSector			(HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) {HAL_Flash_StartCmdEraseSector		(p_hal, p_cmdDest);}
static void StartCmdVerifyWriteUnit		(HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) {HAL_Flash_StartCmdVerifyWriteUnit	(p_hal, p_cmdDest, p_cmdData);}
static void StartCmdVerifyEraseUnits	(HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) {HAL_Flash_StartCmdVerifyEraseUnits	(p_hal, p_cmdDest, units);}
static void StartCmdWriteOnce			(HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) {HAL_Flash_StartCmdWriteOnce			(p_hal, p_cmdDest, p_cmdData);}
static void StartCmdReadOnce			(HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) {HAL_Flash_StartCmdReadOnce			(p_hal, p_cmdDest);}

/******************************************************************************/
/*!
	Public Functions
*/
/******************************************************************************/
void Flash_Init(Flash_T * p_flash)
{
	HAL_Flash_Init(p_flash->CONFIG.P_HAL);
	NvMemory_Init(p_flash);
}

void Flash_SetYield(Flash_T * p_flash, void (*yield)(void *), void * p_callbackData)
{
	NvMemory_SetYield(p_flash, yield, p_callbackData);
}

bool Flash_ReadSecurityFlag(Flash_T * p_flash)
{
	return HAL_Flash_ReadSecurityFlag(p_flash->CONFIG.P_HAL);
}


/******************************************************************************/
/*!
	Set - Common Blocking Non Blocking
*/
/******************************************************************************/
static inline size_t CalcVerifyEraseUnitsPerCmd(size_t bytes)
{
#ifdef CONFIG_FLASH_HW_VERIFY_ERASE_N_UNITS
	return bytes/FLASH_UNIT_VERIFY_ERASE_SIZE;
#elif defined(CONFIG_FLASH_HW_VERIFY_ERASE_1_UNIT)
	return 1U;
#endif
}

static inline Flash_Status_T FinalizeWrite(Flash_T * p_flash);
static inline Flash_Status_T FinalizeErase(Flash_T * p_flash);
static inline Flash_Status_T FinalizeVerify(Flash_T * p_flash);

static inline void SetWrite(Flash_T * p_flash)
{
	p_flash->StartCmd 	= StartCmdWritePage;
	p_flash->FinalizeOp = FinalizeWrite;
	NvMemory_SetOpCmdSize(p_flash, FLASH_UNIT_WRITE_SIZE, 1U);
}

static inline void SetErase(Flash_T * p_flash)
{
	p_flash->StartCmd 	= StartCmdEraseSector;
	p_flash->FinalizeOp = FinalizeErase;
	NvMemory_SetOpCmdSize(p_flash, FLASH_UNIT_ERASE_SIZE, 1U);
}

static inline void SetVerifyWrite(Flash_T * p_flash)
{
	p_flash->StartCmd 	= StartCmdVerifyWriteUnit;
	p_flash->FinalizeOp = FinalizeVerify;
	NvMemory_SetOpCmdSize(p_flash, FLASH_UNIT_VERIFY_WRITE_SIZE, 1U);
}

static inline void SetVerifyErase(Flash_T * p_flash)
{
	p_flash->StartCmd 	= StartCmdVerifyEraseUnits;
	p_flash->FinalizeOp = FinalizeVerify;
	NvMemory_SetOpCmdSize(p_flash, FLASH_UNIT_VERIFY_ERASE_SIZE, CalcVerifyEraseUnitsPerCmd(p_flash->OpSize));
}

Flash_Status_T Flash_SetWrite(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size)
{
	Flash_Status_T status = NvMemory_SetOpCommon(p_flash, p_destFlash, size, FLASH_UNIT_WRITE_SIZE);

	if (status == FLASH_STATUS_SUCCESS)
	{
		SetWrite(p_flash);
		NvMemory_SetOpData(p_flash, p_source, size);
	}

	return status;
}

Flash_Status_T Flash_SetErase(Flash_T * p_flash, const uint8_t * p_destFlash, size_t size)
{
	Flash_Status_T status = NvMemory_SetOpCommon(p_flash, p_destFlash, size, FLASH_UNIT_ERASE_SIZE);

	if (status == FLASH_STATUS_SUCCESS)
	{
		SetErase(p_flash);
		p_flash->OpSize = size;
	}

	return status;
}

Flash_Status_T Flash_SetVerifyWrite(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size)
{
	Flash_Status_T status = NvMemory_SetOpCommon(p_flash, p_destFlash, size, FLASH_UNIT_VERIFY_WRITE_SIZE);

	if (status == FLASH_STATUS_SUCCESS)
	{
		SetVerifyWrite(p_flash);
		NvMemory_SetOpData(p_flash, p_source, size);
	}

	return status;
}

Flash_Status_T Flash_SetVerifyErase(Flash_T * p_flash, const uint8_t * p_destFlash, size_t size)
{
	Flash_Status_T status = NvMemory_SetOpCommon(p_flash, p_destFlash, size, FLASH_UNIT_VERIFY_ERASE_SIZE);

	if (status == FLASH_STATUS_SUCCESS)
	{
		SetVerifyErase(p_flash);
		p_flash->OpSize = size;
	}

	return status;
}

Flash_Status_T Flash_SetWriteOnce(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size)
{
	Flash_Status_T status = NvMemory_SetOpCommon(p_flash, p_destFlash, size, FLASH_UNIT_WRITE_ONCE_SIZE);

	if (status == FLASH_STATUS_SUCCESS)
	{
		p_flash->StartCmd 	= StartCmdWriteOnce;
		p_flash->FinalizeOp = FinalizeWrite;
		NvMemory_SetOpCmdSize(p_flash, FLASH_UNIT_WRITE_ONCE_SIZE, 1U);
		NvMemory_SetOpData(p_flash, p_source, size);
	}

	return status;
}

Flash_Status_T Flash_SetReadOnce(Flash_T * p_flash, const uint8_t * p_destFlash, size_t size) //option to provide buffer?
{
	Flash_Status_T status = NvMemory_SetOpCommon(p_flash, p_destFlash, size, FLASH_UNIT_READ_ONCE_SIZE);

	if (status == FLASH_STATUS_SUCCESS)
	{
		p_flash->StartCmd 	= StartCmdReadOnce;
		p_flash->FinalizeOp = 0U; //todo
		NvMemory_SetOpCmdSize(p_flash, FLASH_UNIT_READ_ONCE_SIZE, 1U);
		p_flash->OpSize 	= size;
	}

	return status;
}

void Flash_GetReadOnce(const Flash_T * p_flash, uint8_t * p_result)
{
	memcpy(p_result, &p_flash->CONFIG.P_BUFFER[0], p_flash->OpSize);
}

Flash_Status_T Flash_SetOp(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size, Flash_Operation_T opType)
{
	Flash_Status_T status;

	switch(opType)
	{
		case FLASH_OPERATION_WRITE:			status = Flash_SetWrite			(p_flash, p_destFlash, p_source, size);		break;
		case FLASH_OPERATION_ERASE:			status = Flash_SetErase			(p_flash, p_destFlash, size); 				break;
		case FLASH_OPERATION_VERIFY_WRITE:	status = Flash_SetVerifyWrite	(p_flash, p_destFlash, p_source, size);		break;
		case FLASH_OPERATION_VERIFY_ERASE:	status = Flash_SetVerifyErase	(p_flash, p_destFlash, size); 				break;
		case FLASH_OPERATION_WRITE_ONCE:	status = Flash_SetWriteOnce		(p_flash, p_destFlash, p_source, size); 	break;
		case FLASH_OPERATION_READ_ONCE:		status = Flash_SetReadOnce		(p_flash, p_destFlash, size);				break;
		default:  break;
	}

	return status;
}

/******************************************************************************/
/*!
	Common Blocking Non Blocking
*/
/******************************************************************************/
static inline Flash_Status_T FinalizeWriteEraseCommon(Flash_T * p_flash)
{
	Flash_Status_T status;

	if (HAL_Flash_ReadErrorFlags(p_flash->CONFIG.P_HAL) == true)
	{
		if (HAL_Flash_ReadErrorProtectionFlag(p_flash->CONFIG.P_HAL) == true)
		{
			status = FLASH_STATUS_ERROR_PROTECTION;
		}
		else
		{
			status = FLASH_STATUS_ERROR_CMD;
		}
	}
	else
	{
		if (p_flash->IsVerifyEnable == true)
		{
			status = FLASH_STATUS_START_VERIFY;
		}
		else
		{
			status = FLASH_STATUS_SUCCESS;
		}
	}

	return status;
}

static inline Flash_Status_T FinalizeErase(Flash_T * p_flash)
{
	Flash_Status_T status = FinalizeWriteEraseCommon(p_flash);

	if (status == FLASH_STATUS_START_VERIFY)
	{
		SetVerifyErase(p_flash);
	}

	return status;
}

static inline Flash_Status_T FinalizeWrite(Flash_T * p_flash)
{
	Flash_Status_T status = FinalizeWriteEraseCommon(p_flash);

	if (status == FLASH_STATUS_START_VERIFY)
	{
		if (NvMemory_ChecksumOp(p_flash) == false)
		{
			status = FLASH_STATUS_ERROR_CHECKSUM;
		}
		else
		{
			SetVerifyWrite(p_flash);
		}
	}

	return status;
}

static inline Flash_Status_T FinalizeVerify(Flash_T * p_flash) //ensure hal where error flag is shared.
{
	Flash_Status_T status;

	if (HAL_Flash_ReadErrorFlags(p_flash->CONFIG.P_HAL) == true)
	{
		if (HAL_Flash_ReadErrorVerifyFlag(p_flash->CONFIG.P_HAL) == true)
		{
			status = FLASH_STATUS_ERROR_VERIFY;
		}
		else
		{
			status = FLASH_STATUS_ERROR_CMD;
		}
	}
	else
	{
		status = FLASH_STATUS_SUCCESS;
	}

	return status;
}

/******************************************************************************/
/*!
	Blocking Implementations
*/
/******************************************************************************/

Flash_Status_T Flash_ProcThis_Blocking(Flash_T * p_flash)
{
	return NvMemory_ProcOpCommon_Blocking(p_flash);
}

/*
	Set and Proc
 */
Flash_Status_T Flash_Write_Blocking(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size)
{
	return ((Flash_SetWrite(p_flash, p_destFlash, p_source, size) == FLASH_STATUS_SUCCESS) ? NvMemory_ProcOpCommon_Blocking(p_flash) : FLASH_STATUS_ERROR_INPUT);
}

/*
 * Append Write Operation
 */
Flash_Status_T Flash_ContinueWrite_Blocking(Flash_T * p_flash, const uint8_t * p_source, size_t size)
{
	Flash_Status_T status;

	NvMemory_SetOpData(p_flash, p_source, size);

	status = NvMemory_ProcOpCommon_Blocking(p_flash);

	if (status == FLASH_STATUS_SUCCESS)
	{
		p_flash->p_OpDest += size;
	}

	if (p_flash->FinalizeOp == FinalizeVerify)
	{
		SetWrite(p_flash); //restore write after verify
	}

	return status;
}

Flash_Status_T Flash_Erase_Blocking(Flash_T * p_flash, const uint8_t * p_destFlash, size_t size)
{
	return ((Flash_SetErase(p_flash, p_destFlash, size) == FLASH_STATUS_SUCCESS) ? NvMemory_ProcOpCommon_Blocking(p_flash) : FLASH_STATUS_ERROR_INPUT);
}

Flash_Status_T Flash_VerifyWrite_Blocking(Flash_T *p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size)
{
	return ((Flash_SetVerifyWrite(p_flash, p_destFlash, p_source, size) == FLASH_STATUS_SUCCESS) ? NvMemory_ProcOpCommon_Blocking(p_flash) : FLASH_STATUS_ERROR_INPUT);
}

Flash_Status_T Flash_VerifyErase_Blocking(Flash_T *p_flash, const uint8_t * p_destFlash, size_t size)
{
	return ((Flash_SetVerifyErase(p_flash, p_destFlash, size) == FLASH_STATUS_SUCCESS) ? NvMemory_ProcOpCommon_Blocking(p_flash) : FLASH_STATUS_ERROR_INPUT);
}

Flash_Status_T Flash_WriteOnce_Blocking(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size)
{
	return ((Flash_SetWriteOnce(p_flash, p_destFlash, p_source, size) == FLASH_STATUS_SUCCESS) ? NvMemory_ProcOpCommon_Blocking(p_flash) : FLASH_STATUS_ERROR_INPUT);
}

Flash_Status_T Flash_ReadOnce_Blocking(Flash_T * p_flash, const uint8_t * p_destFlash, size_t size)
{
	return ((Flash_SetReadOnce(p_flash, p_destFlash, size) == FLASH_STATUS_SUCCESS) ? NvMemory_ProcOpCommon_Blocking(p_flash) : FLASH_STATUS_ERROR_INPUT);
}

Flash_Status_T Flash_ProcOp_Blocking(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size, Flash_Operation_T opType)
{
	return ((Flash_SetOp(p_flash, p_destFlash, p_source, size, opType) == FLASH_STATUS_SUCCESS) ? NvMemory_ProcOpCommon_Blocking(p_flash) : FLASH_STATUS_ERROR_INPUT);
}

/*

 */
//Flash_Status_T Flash_EraseAll_Blocking(Flash_T *p_flash)
//{

//}


/******************************************************************************/
/*!
	Non Blocking
*/
/******************************************************************************/
/*
 * returns true when complete
 */
bool Flash_ProcOp(Flash_T * p_flash)
{
	return NvMemory_ProcOp(p_flash);
}

size_t Flash_GetOpBytesRemaining(Flash_T * p_flash)
{
	return NvMemory_GetOpBytesRemaining(p_flash);
}

bool Flash_GetIsOpComplete(Flash_T * p_flash)
{
	return NvMemory_GetIsOpComplete(p_flash);
}

Flash_Status_T Flash_StartWrite_NonBlocking(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size)
{
	return (p_flash->Status = (Flash_SetWrite(p_flash, p_destFlash, p_source, size) == FLASH_STATUS_SUCCESS ? NvMemory_StartOpCommon(p_flash) : FLASH_STATUS_ERROR_INPUT));
}

Flash_Status_T Flash_StartErase_NonBlocking(Flash_T * p_flash, const uint8_t * p_destFlash, size_t size)
{
	return (p_flash->Status = (Flash_SetErase(p_flash, p_destFlash, size) == FLASH_STATUS_SUCCESS ? NvMemory_StartOpCommon(p_flash) : FLASH_STATUS_ERROR_INPUT));
}

Flash_Status_T Flash_StartVerifyWrite_NonBlocking(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size)
{
	return (p_flash->Status = (Flash_SetVerifyWrite(p_flash, p_destFlash, p_source, size) == FLASH_STATUS_SUCCESS ? NvMemory_StartOpCommon(p_flash) : FLASH_STATUS_ERROR_INPUT));
}

Flash_Status_T Flash_StartVerifyErase_NonBlocking(Flash_T * p_flash, const uint8_t * p_destFlash, size_t size)
{
	return (p_flash->Status = (Flash_SetVerifyErase(p_flash, p_destFlash, size) == FLASH_STATUS_SUCCESS ? NvMemory_StartOpCommon(p_flash) : FLASH_STATUS_ERROR_INPUT));
}

Flash_Status_T Flash_StartWriteOnce_NonBlocking(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size)
{
	return (p_flash->Status = (Flash_SetWriteOnce(p_flash, p_destFlash, p_source, size) == FLASH_STATUS_SUCCESS ? NvMemory_StartOpCommon(p_flash) : FLASH_STATUS_ERROR_INPUT));
}

Flash_Status_T Flash_StartReadOnce_NonBlocking(Flash_T * p_flash, const uint8_t * p_destFlash, size_t size)
{
	return (p_flash->Status = (Flash_SetReadOnce(p_flash, p_destFlash, size) == FLASH_STATUS_SUCCESS ? NvMemory_StartOpCommon(p_flash) : FLASH_STATUS_ERROR_INPUT));
}

/******************************************************************************/
/*!
	Virtual
*/
/******************************************************************************/
/*
 * creates copy of flash in buffer
 */
//void Flash_OpenVirtual(Flash_T * p_flash, const uint8_t * p_physical, size_t size)
//{

//
//}
//
///*
// * dest of physical flash location
// */
//void Flash_WriteVirtual(Flash_T * p_flash, const uint8_t * p_physical, const uint8_t * p_src, size_t size)
//{

//}
//
//void Flash_ReadVirtual(Flash_T * p_flash, uint8_t * p_dest, const uint8_t * p_physical, size_t size)
//{

//}
//
//void Flash_CloseVirtual(Flash_T * p_flash)
//{
//	p_flash->State = FLASH_STATE_WRITE;
//}
//
//bool Flash_CloseVirtual_Blocking(Flash_T * p_flash)
//{

//}

