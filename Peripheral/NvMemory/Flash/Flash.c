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
	@brief
	@version V0
*/
/******************************************************************************/
#include "Flash.h"
#include "System/Critical/Critical.h"
#include <string.h>

/******************************************************************************/
/*!
	HAL Launch Cmd
*/
/******************************************************************************/
static void StartCmdWritePage			(HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;
static void StartCmdEraseSector			(HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;
static void StartCmdVerifyWriteUnit		(HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;
static void StartCmdVerifyEraseUnits	(HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;
static void StartCmdWriteOnce			(HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;
static void StartCmdReadOnce			(HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;
static void StartCmdEraseAll			(HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;

static void StartCmdWritePage			(HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) { (void)units; 					HAL_Flash_StartCmdWritePage(p_hal, p_cmdDest, p_cmdData);}
static void StartCmdEraseSector			(HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) { (void)p_cmdData; (void)units; 	HAL_Flash_StartCmdEraseSector(p_hal, p_cmdDest);}
static void StartCmdVerifyWriteUnit		(HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) { (void)units; 					HAL_Flash_StartCmdVerifyWriteUnit(p_hal, p_cmdDest, p_cmdData);}
static void StartCmdVerifyEraseUnits	(HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) { (void)p_cmdData; 				HAL_Flash_StartCmdVerifyEraseUnits(p_hal, p_cmdDest, units);}
static void StartCmdWriteOnce			(HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) { (void)units; 					HAL_Flash_StartCmdWriteOnce(p_hal, p_cmdDest, p_cmdData);}
static void StartCmdReadOnce			(HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) { (void)p_cmdData; (void)units; 	HAL_Flash_StartCmdReadOnce(p_hal, p_cmdDest);}
static void StartCmdEraseAll			(HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) { (void)p_cmdDest; (void)p_cmdData; (void)units; 	HAL_Flash_StartCmdEraseAll(p_hal);}

/******************************************************************************/
/*!
	Finalize Cmd
*/
/******************************************************************************/
// static Flash_Status_T FinalizeCmdNull(Flash_T * p_flash) { (void)p_flash; }
static Flash_Status_T FinalizeCmdReadOnce(Flash_T * p_flash, size_t opIndex)
{
	HAL_Flash_ReadOnceData(p_flash->CONFIG.P_HAL, &p_flash->CONFIG.P_BUFFER[opIndex]);
	return FLASH_STATUS_SUCCESS;
}

/******************************************************************************/
/*!
	Finalize Op
*/
/******************************************************************************/
static inline void SetWrite(Flash_T * p_flash);
static inline void SetErase(Flash_T * p_flash);
static inline void SetVerifyWrite(Flash_T * p_flash);
static inline void SetVerifyErase(Flash_T * p_flash);

static inline Flash_Status_T CheckStartVerify(Flash_T * p_flash)
{
	return (p_flash->IsVerifyEnable == true) ? FLASH_STATUS_START_VERIFY : FLASH_STATUS_SUCCESS;
}

static inline Flash_Status_T FinalizeErase(Flash_T * p_flash)
{
	Flash_Status_T status = CheckStartVerify(p_flash);
	if(status == FLASH_STATUS_START_VERIFY) { SetVerifyErase(p_flash); }
	return status;
}

static inline Flash_Status_T FinalizeWrite(Flash_T * p_flash)
{
	Flash_Status_T status = CheckStartVerify(p_flash);
	if(status == FLASH_STATUS_START_VERIFY)
	{
		if(NvMemory_CheckOpChecksum(p_flash) == false) { status = FLASH_STATUS_ERROR_CHECKSUM; }
		else { SetVerifyWrite(p_flash); }
	}
	return status;
}

static inline Flash_Status_T FinalizeVerify(Flash_T * p_flash)
{
	(void)p_flash;
	return FLASH_STATUS_SUCCESS;
}

static inline Flash_Status_T FinalizeWriteOnce(Flash_T * p_flash)
{
	(void)p_flash;
	return FLASH_STATUS_SUCCESS;
}

static inline Flash_Status_T FinalizeReadOnce(Flash_T * p_flash)
{
	(void)p_flash;
	return FLASH_STATUS_SUCCESS;
}

/******************************************************************************/
/*!
	Parse Error
	Cannot use common Parse Error function as register meaning might differ depending on Cmd
*/
/******************************************************************************/
static Flash_Status_T ParseCmdErrorWrite(Flash_T * p_flash)
{
	Flash_Status_T status;
	if(HAL_Flash_ReadErrorProtectionFlag(p_flash->CONFIG.P_HAL) == true) { status = FLASH_STATUS_ERROR_PROTECTION; }
	else { status = FLASH_STATUS_ERROR_CMD; }
	return status;
}

static Flash_Status_T ParseCmdErrorErase(Flash_T * p_flash)
{
	Flash_Status_T status;
	if(HAL_Flash_ReadErrorProtectionFlag(p_flash->CONFIG.P_HAL) == true) { status = FLASH_STATUS_ERROR_PROTECTION; }
	else { status = FLASH_STATUS_ERROR_CMD; }
	return status;
}

static Flash_Status_T ParseCmdErrorVerify(Flash_T * p_flash)
{
	Flash_Status_T status;
	if(HAL_Flash_ReadErrorVerifyFlag(p_flash->CONFIG.P_HAL) == true) { status = FLASH_STATUS_ERROR_VERIFY; }
	else { status = FLASH_STATUS_ERROR_CMD; }
	return status;
}

static Flash_Status_T ParseCmdErrorWriteOnce(Flash_T * p_flash)
{
	(void)p_flash;
	return FLASH_STATUS_ERROR_CMD; /* Cmd failed */
}

static Flash_Status_T ParseCmdErrorReadOnce(Flash_T * p_flash)
{
	(void)p_flash;
	return FLASH_STATUS_ERROR_CMD;
}

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
	return bytes / FLASH_UNIT_VERIFY_ERASE_SIZE;
#elif defined(CONFIG_FLASH_HW_VERIFY_ERASE_1_UNIT)
	return 1U;
#endif
}

static inline void SetWrite(Flash_T * p_flash)
{
	NvMemory_SetOpFunctions(p_flash, (NvMemory_StartCmd_T)StartCmdWritePage, (NvMemory_Process_T)FinalizeWrite, (NvMemory_Process_T)ParseCmdErrorWrite);
	NvMemory_SetOpCmdSize(p_flash, FLASH_UNIT_WRITE_SIZE, 1U);
}

static inline void SetErase(Flash_T * p_flash)
{
	NvMemory_SetOpFunctions(p_flash, (NvMemory_StartCmd_T)StartCmdEraseSector, (NvMemory_Process_T)FinalizeErase, (NvMemory_Process_T)ParseCmdErrorErase);
	NvMemory_SetOpCmdSize(p_flash, FLASH_UNIT_ERASE_SIZE, 1U);
}

static inline void SetVerifyWrite(Flash_T * p_flash)
{
	NvMemory_SetOpFunctions(p_flash, (NvMemory_StartCmd_T)StartCmdVerifyWriteUnit, (NvMemory_Process_T)FinalizeVerify, (NvMemory_Process_T)ParseCmdErrorVerify);
	NvMemory_SetOpCmdSize(p_flash, FLASH_UNIT_VERIFY_WRITE_SIZE, 1U);
}

static inline void SetVerifyErase(Flash_T * p_flash)
{
	NvMemory_SetOpFunctions(p_flash, (NvMemory_StartCmd_T)StartCmdVerifyEraseUnits, (NvMemory_Process_T)FinalizeVerify, (NvMemory_Process_T)ParseCmdErrorVerify);
	NvMemory_SetOpCmdSize(p_flash, FLASH_UNIT_VERIFY_ERASE_SIZE, CalcVerifyEraseUnitsPerCmd(p_flash->OpSize));
}

Flash_Status_T Flash_SetWrite(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size)
{
	Flash_Status_T status = (Flash_Status_T)NvMemory_SetOpDest(p_flash, p_destFlash, size, FLASH_UNIT_WRITE_SIZE);

	if(status == FLASH_STATUS_SUCCESS)
	{
		SetWrite(p_flash);
		NvMemory_SetOpSourceData(p_flash, p_source, size);
	}

	return status;
}

Flash_Status_T Flash_SetErase(Flash_T * p_flash, const uint8_t * p_destFlash, size_t size)
{
	Flash_Status_T status = (Flash_Status_T)NvMemory_SetOpDest(p_flash, p_destFlash, size, FLASH_UNIT_ERASE_SIZE);

	if(status == FLASH_STATUS_SUCCESS)
	{
		SetErase(p_flash);
		p_flash->OpSize = size;
	}

	return status;
}

Flash_Status_T Flash_SetVerifyWrite(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size)
{
	Flash_Status_T status = (Flash_Status_T)NvMemory_SetOpDest(p_flash, p_destFlash, size, FLASH_UNIT_VERIFY_WRITE_SIZE);

	if(status == FLASH_STATUS_SUCCESS)
	{
		SetVerifyWrite(p_flash);
		NvMemory_SetOpSourceData(p_flash, p_source, size);
	}

	return status;
}

Flash_Status_T Flash_SetVerifyErase(Flash_T * p_flash, const uint8_t * p_destFlash, size_t size)
{
	Flash_Status_T status = (Flash_Status_T)NvMemory_SetOpDest(p_flash, p_destFlash, size, FLASH_UNIT_VERIFY_ERASE_SIZE);

	if(status == FLASH_STATUS_SUCCESS)
	{
		SetVerifyErase(p_flash);
		p_flash->OpSize = size;
	}

	return status;
}

Flash_Status_T Flash_SetWriteOnce(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size)
{
	Flash_Status_T status = (Flash_Status_T)NvMemory_SetOpDest(p_flash, p_destFlash, size, FLASH_UNIT_WRITE_ONCE_SIZE);

	if(status == FLASH_STATUS_SUCCESS)
	{
		NvMemory_SetOpFunctions(p_flash, (NvMemory_StartCmd_T)StartCmdWriteOnce, (NvMemory_Process_T)FinalizeWriteOnce, (NvMemory_Process_T)ParseCmdErrorWriteOnce);
		NvMemory_SetOpCmdSize(p_flash, FLASH_UNIT_WRITE_ONCE_SIZE, 1U);
		NvMemory_SetOpSourceData(p_flash, p_source, size);
	}

	return status;
}

//option to provide buffer?
Flash_Status_T Flash_SetReadOnce(Flash_T * p_flash, const uint8_t * p_destFlash, size_t size)
{
	Flash_Status_T status = (Flash_Status_T)NvMemory_SetOpDest(p_flash, p_destFlash, size, FLASH_UNIT_READ_ONCE_SIZE);

	if(status == FLASH_STATUS_SUCCESS)
	{
		NvMemory_SetOpFunctions(p_flash, (NvMemory_StartCmd_T)StartCmdReadOnce, (NvMemory_Process_T)FinalizeReadOnce, (NvMemory_Process_T)ParseCmdErrorReadOnce);
		NvMemory_SetOpCmdSize(p_flash, FLASH_UNIT_READ_ONCE_SIZE, 1U);
		p_flash->FinalizeCmd = (NvMemory_FinalizeCmd_T)FinalizeCmdReadOnce;
		p_flash->OpSize = size;
	}

	return status;
}

// Flash_Status_T Flash_StartReadOnce_Direct_Blocking(Flash_T * p_flash, uint8_t * p_destResult, const uint8_t * p_destFlash, size_t size)
// {
// 	return = (Flash_SetReadOnce(p_flash, p_destFlash, size) == FLASH_STATUS_SUCCESS ? NvMemory_ProcOp_Blocking(p_flash) : FLASH_STATUS_ERROR_INPUT);
// }

Flash_Status_T Flash_SetEraseAll(Flash_T * p_flash)
{
	NvMemory_SetOpFunctions(p_flash, (NvMemory_StartCmd_T)StartCmdEraseAll, 0U, 0U);
	return FLASH_STATUS_SUCCESS;
}

Flash_Status_T Flash_SetOp(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size, Flash_Operation_T opId)
{
	Flash_Status_T status;

	switch(opId)
	{
		case FLASH_OPERATION_WRITE:			status = Flash_SetWrite(p_flash, p_destFlash, p_source, size); 			break;
		case FLASH_OPERATION_ERASE:			status = Flash_SetErase(p_flash, p_destFlash, size); 					break;
		case FLASH_OPERATION_VERIFY_WRITE:	status = Flash_SetVerifyWrite(p_flash, p_destFlash, p_source, size); 	break;
		case FLASH_OPERATION_VERIFY_ERASE:	status = Flash_SetVerifyErase(p_flash, p_destFlash, size); 				break;
		case FLASH_OPERATION_WRITE_ONCE:	status = Flash_SetWriteOnce(p_flash, p_destFlash, p_source, size); 		break;
		case FLASH_OPERATION_READ_ONCE:		status = Flash_SetReadOnce(p_flash, p_destFlash, size); 				break;
		default:  							status = FLASH_STATUS_ERROR_INVALID_OP; 								break;
	}

	return status;
}

void Flash_GetReadOnce(const Flash_T * p_flash, uint8_t * p_result)
{
	memcpy(p_result, &p_flash->CONFIG.P_BUFFER[0U], p_flash->OpSize);
}

/******************************************************************************/
/*!
	Blocking Implementations
	Set and Proc
*/
/******************************************************************************/
Flash_Status_T Flash_ProcThisOp_Blocking(Flash_T * p_flash)
{
	Flash_Status_T status;
	Critical_Enter();
	status = (Flash_Status_T)NvMemory_ProcOp_Blocking(p_flash);
	Critical_Exit();
	return status;
}

static inline Flash_Status_T ProcOpReturn_Blocking(Flash_T * p_flash, Flash_Status_T status)
{
	return (status == FLASH_STATUS_SUCCESS) ? Flash_ProcThisOp_Blocking(p_flash) : status;
}

Flash_Status_T Flash_Write_Blocking(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size)
{
	Flash_Status_T status = Flash_SetWrite(p_flash, p_destFlash, p_source, size);
	return ProcOpReturn_Blocking(p_flash, status);
}

/*
	Append Write Operation
*/
Flash_Status_T Flash_ContinueWrite_Blocking(Flash_T * p_flash, const uint8_t * p_source, size_t size)
{
	Flash_Status_T status;

	NvMemory_SetOpSourceData(p_flash, p_source, size);
	status = Flash_ProcThisOp_Blocking(p_flash);
	if(status == FLASH_STATUS_SUCCESS)
	{
		p_flash->p_OpDest += size;
		if((uint32_t)p_flash->FinalizeOp == (uint32_t)FinalizeVerify) { SetWrite(p_flash); } /* Restore write after verify */
	}

	return status;
}

Flash_Status_T Flash_Erase_Blocking(Flash_T * p_flash, const uint8_t * p_destFlash, size_t size)
{
	Flash_Status_T status = Flash_SetErase(p_flash, p_destFlash, size);
	return ProcOpReturn_Blocking(p_flash, status);
}

Flash_Status_T Flash_VerifyWrite_Blocking(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size)
{
	Flash_Status_T status = Flash_SetVerifyWrite(p_flash, p_destFlash, p_source, size);
	return ProcOpReturn_Blocking(p_flash, status);
}

Flash_Status_T Flash_VerifyErase_Blocking(Flash_T * p_flash, const uint8_t * p_destFlash, size_t size)
{
	Flash_Status_T status = Flash_SetVerifyErase(p_flash, p_destFlash, size);
	return ProcOpReturn_Blocking(p_flash, status);
}

Flash_Status_T Flash_WriteOnce_Blocking(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size)
{
	Flash_Status_T status = Flash_SetWriteOnce(p_flash, p_destFlash, p_source, size);
	return ProcOpReturn_Blocking(p_flash, status);
}

Flash_Status_T Flash_ReadOnce_Blocking(Flash_T * p_flash, const uint8_t * p_destFlash, size_t size)
{
	Flash_Status_T status = Flash_SetReadOnce(p_flash, p_destFlash, size);
	return ProcOpReturn_Blocking(p_flash, status);
}

// Flash_Status_T Flash_StartReadOnce_Direct_Blocking(Flash_T * p_flash, uint8_t * p_destResult, const uint8_t * p_destFlash, size_t size)
// {
// 	return = (Flash_SetReadOnce(p_flash, p_destFlash, size) == FLASH_STATUS_SUCCESS ? NvMemory_ProcOp_Blocking(p_flash) : FLASH_STATUS_ERROR_INPUT);
// }

Flash_Status_T Flash_EraseAll_Blocking(Flash_T *p_flash)
{
	Flash_Status_T status = Flash_SetEraseAll(p_flash);
	return ProcOpReturn_Blocking(p_flash, status);
}

Flash_Status_T Flash_ProcOp_Blocking(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size, Flash_Operation_T opId)
{
	Flash_Status_T status = Flash_SetOp(p_flash, p_destFlash, p_source, size, opId);
	return ProcOpReturn_Blocking(p_flash, status);
}



/*

*/



/******************************************************************************/
/*!
	Non Blocking - UNTESTED
*/
/******************************************************************************/
/*
	returns true when complete
*/
bool Flash_ProcOp(Flash_T * p_flash) { return NvMemory_ProcOp(p_flash); }

size_t Flash_GetOpBytesRemaining(Flash_T * p_flash) { return NvMemory_GetOpBytesRemaining(p_flash); }

// bool Flash_ReadIsOpComplete(Flash_T * p_flash) { return (Flash_Status_T)NvMemory_ReadIsOpComplete(p_flash); }

Flash_Status_T Flash_StartWrite_NonBlocking(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size)
{
	p_flash->Status = (Flash_SetWrite(p_flash, p_destFlash, p_source, size) == FLASH_STATUS_SUCCESS ? NvMemory_StartOp(p_flash) : FLASH_STATUS_ERROR_INPUT);
	return (Flash_Status_T)p_flash->Status;
}

Flash_Status_T Flash_StartErase_NonBlocking(Flash_T * p_flash, const uint8_t * p_destFlash, size_t size)
{
	p_flash->Status = (Flash_SetErase(p_flash, p_destFlash, size) == FLASH_STATUS_SUCCESS ? NvMemory_StartOp(p_flash) : FLASH_STATUS_ERROR_INPUT);
	return (Flash_Status_T)p_flash->Status;
}

Flash_Status_T Flash_StartVerifyWrite_NonBlocking(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size)
{
	p_flash->Status = (Flash_SetVerifyWrite(p_flash, p_destFlash, p_source, size) == FLASH_STATUS_SUCCESS ? NvMemory_StartOp(p_flash) : FLASH_STATUS_ERROR_INPUT);
	return (Flash_Status_T)p_flash->Status;
}

Flash_Status_T Flash_StartVerifyErase_NonBlocking(Flash_T * p_flash, const uint8_t * p_destFlash, size_t size)
{
	p_flash->Status = (Flash_SetVerifyErase(p_flash, p_destFlash, size) == FLASH_STATUS_SUCCESS ? NvMemory_StartOp(p_flash) : FLASH_STATUS_ERROR_INPUT);
	return (Flash_Status_T)p_flash->Status;
}

Flash_Status_T Flash_StartWriteOnce_NonBlocking(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size)
{
	p_flash->Status = (Flash_SetWriteOnce(p_flash, p_destFlash, p_source, size) == FLASH_STATUS_SUCCESS ? NvMemory_StartOp(p_flash) : FLASH_STATUS_ERROR_INPUT);
	return (Flash_Status_T)p_flash->Status;
}

Flash_Status_T Flash_StartReadOnce_NonBlocking(Flash_T * p_flash, const uint8_t * p_destFlash, size_t size)
{
	p_flash->Status = (Flash_SetReadOnce(p_flash, p_destFlash, size) == FLASH_STATUS_SUCCESS ? NvMemory_StartOp(p_flash) : FLASH_STATUS_ERROR_INPUT);
	return (Flash_Status_T)p_flash->Status;
}

// Flash_Status_T Flash_StartReadOnce_Direct_NonBlocking(Flash_T * p_flash, uint8_t * p_destResult, const uint8_t * p_destFlash, size_t size)
// {
// 	p_flash->Status = (Flash_SetReadOnce(p_flash, p_destFlash, size) == FLASH_STATUS_SUCCESS ? NvMemory_StartOp(p_flash) : FLASH_STATUS_ERROR_INPUT);
// 	return (Flash_Status_T)p_flash->Status;
// }
/******************************************************************************/
/*!
	Virtual
*/
/******************************************************************************/
/*
	creates copy of flash in buffer
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

