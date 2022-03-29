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
#include "EEPROM.h"
#include "HAL_EEPROM.h"

#include "../NvMemory/NvMemory.h"

#include <stdint.h>
#include <stdbool.h>


//
//static inline WriteUnit( uint8_t * p_destEeprom, const uint8_t * p_source)
//{
//#if (EEPROM_UNIT_WRITE_SIZE == 4U)
//	(*(uint32_t*)p_destEeprom) = (*(uint32_t*)p_source);
//#elif (EEPROM_UNIT_WRITE_SIZE == 2U)
//	(*(uint16_t*)p_destEeprom) = (*(uint16_t*)p_source);
//#elif (EEPROM_UNIT_WRITE_SIZE == 1U)
//	(*(uint8_t*)p_destEeprom) = (*(uint8_t*)p_source);
//#endif
//}

/*
	match start cmd function signature
 */
static void EepromStartCmdWrite(void * p_hal, const void * p_cmdDest, const void * p_cmdData, size_t units)
{
	(void)units;

	if(*(uint32_t*)p_cmdDest != (*(uint32_t*)p_cmdData))
	{
		HAL_EEPROM_StartCmdWriteUnit(p_hal, p_cmdDest, p_cmdData);
	}

	/* else finalizeOp will return no error and continue to next  */
}

//static void EepromStartCmdProgramPartition(void * p_hal, const void * p_cmdDest, const void * p_cmdData, size_t units)
//{
//	(void)units;
//	(void)p_cmdDest;
//	(void)p_cmdData;
//	HAL_EEPROM_ProgramPartition(p_hal);
//}
//
//static void EepromStartCmdInitHw(void * p_hal, const void * p_cmdDest, const void * p_cmdData, size_t units)
//{
//	(void)units;
//	(void)p_cmdDest;
//	(void)p_cmdData;
//	HAL_EEPROM_InitHw(p_hal);
//}

static inline NvMemory_Status_T EepromFinalizeWrite(EEPROM_T * p_eeprom)
{
	NvMemory_Status_T status;

	if (HAL_EEPROM_ReadErrorFlags(p_eeprom->CONFIG.P_HAL) == true)
	{
		if (HAL_EEPROM_ReadErrorProtectionFlag(p_eeprom->CONFIG.P_HAL) == true)
		{
			status = NV_MEMORY_STATUS_ERROR_PROTECTION;
		}
		else
		{
			status = NV_MEMORY_STATUS_ERROR_CMD;
		}
	}
	else
	{
//		if (p_eeprom->IsVerifyEnable == true)
//		{
//			status = NV_MEMORY_STATUS_START_VERIFY;
//		}
//		else
		{
			status = NV_MEMORY_STATUS_SUCCESS;
		}
	}

	return status;
}

void EEPROM_Init_Blocking(EEPROM_T * p_eeprom)
{
	//todo isfirsttime, use startcmd template
//	HAL_EEPROM_ReadIsFirstTime(p_eeprom->CONFIG.P_HAL);

	HAL_EEPROM_Init_Blocking(p_eeprom->CONFIG.P_HAL);
	NvMemory_Init(p_eeprom);
	p_eeprom->IsVerifyEnable 	= false;
	p_eeprom->IsOpBuffered 		= false;
    while (HAL_EEPROM_ReadCompleteFlag(p_eeprom->CONFIG.P_HAL) == false);
}


void EEPROM_ReadIsFirstTime(EEPROM_T * p_eeprom)
{
	HAL_EEPROM_ReadIsFirstTime(p_eeprom->CONFIG.P_HAL);
}

//void EEPROM_Init_NonBlocking(EEPROM_T * p_eeprom)
//{
//	HAL_EEPROM_Init_NonBlocking(p_eeprom->CONFIG.P_HAL);
//}

NvMemory_Status_T EEPROM_SetWrite(EEPROM_T * p_eeprom, const void * p_dest, const void * p_source, size_t sizeBytes)
{
	NvMemory_Status_T status = NvMemory_SetOpCommon(p_eeprom, p_dest, sizeBytes, EEPROM_UNIT_WRITE_SIZE);

	if (status == NV_MEMORY_STATUS_SUCCESS)
	{
		p_eeprom->StartCmd 		= EepromStartCmdWrite;
		p_eeprom->FinalizeOp 	= EepromFinalizeWrite;
		NvMemory_SetOpCmdSize(p_eeprom, EEPROM_UNIT_WRITE_SIZE, 1U);
		NvMemory_SetOpData(p_eeprom, p_source, sizeBytes);
	}

	return status;
}

NvMemory_Status_T EEPROM_Write_Blocking(EEPROM_T * p_eeprom, const void * p_dest, const void * p_source, size_t sizeBytes)
{
	return ((EEPROM_SetWrite(p_eeprom, p_dest, p_source, sizeBytes) == NV_MEMORY_STATUS_SUCCESS) ? NvMemory_ProcOpCommon_Blocking(p_eeprom) : NV_MEMORY_STATUS_ERROR_INPUT);
}

/*
 * Non Blocking
 */
bool EEPROM_ProcOp_NonBlocking(EEPROM_T * p_flash)
{
	return NvMemory_ProcOp(p_flash);
}

bool EEPROM_ReadIsOpComplete(EEPROM_T * p_eeprom)
{
	return HAL_EEPROM_ReadCompleteFlag(p_eeprom->CONFIG.P_HAL);
}

NvMemory_Status_T EEPROM_StartWrite_NonBlocking(EEPROM_T * p_eeprom, void * p_dest, const void * p_source, size_t sizeBytes)
{
	return (p_eeprom->Status = (EEPROM_SetWrite(p_eeprom, p_dest, p_source, sizeBytes) == NV_MEMORY_STATUS_SUCCESS ? NvMemory_StartOpCommon(p_eeprom) : NV_MEMORY_STATUS_ERROR_INPUT));
}


