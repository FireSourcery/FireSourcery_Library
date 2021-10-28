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

static inline uint32_t CalcAlignDown(uint32_t value, uint8_t align)
{
	return ((value) & -(align));
}

static inline uint32_t CalcAlignUp(uint32_t value, uint8_t align)
{
	return (-(-(value) & -(align)));
}

static inline bool CheckIsAligned(uint32_t value, uint8_t align)
{
	return ((align & (align - 1U)) == 0U); //align unit always power of 2
}

static inline bool CheckDestIsAligned(const uint8_t * p_destFlash, size_t size, uint8_t align)
{
	return (CheckIsAligned((uint32_t)p_destFlash, align) && CheckIsAligned(size, align) ? true : false);
}

static inline bool CheckIsBounded(uint32_t targetStart, size_t targetSize, uint32_t boundaryStart, size_t boundarySize)
{
	return ((targetStart >= boundaryStart) && ((targetStart + targetSize) <= (boundaryStart + boundarySize)));
}

static inline bool CheckDestIsBoundedPartition(const uint8_t * p_dest, size_t size, Flash_Partition_T * p_partition)
{
	return CheckIsBounded((uint32_t)p_dest, size, (uint32_t)p_partition->P_START, p_partition->SIZE);
}

static inline Flash_Partition_T * SearchParition(const Flash_Partition_T * p_partitionTable, const uint8_t * p_dest, size_t size)
{
	Flash_Partition_T * p_partition = 0U;

	for (uint8_t iPartition = 0U; iPartition < CONFIG_FLASH_PARTITION_COUNT; iPartition++)
	{
		if(CheckDestIsBoundedPartition(p_dest, size, &p_partitionTable[iPartition]) == true)
		{
			p_partition = &p_partitionTable[iPartition];
		}
	}

	return p_partition;
}

static inline bool SearchSetParition(Flash_T * p_flash, const uint8_t * p_dest, size_t size)
{
	p_flash->p_OpPartition = SearchParition(&p_flash->CONFIG.PARTITIONS[0U], p_dest, size);
	return ((p_flash->p_OpPartition != 0U)? true : false);
}

static inline uint32_t CalcChecksum(const uint8_t * p_data, size_t size)
{
	uint32_t sum = 0U;
	for (size_t iByte = 0U; iByte < size; iByte++)	{ sum += p_data[iByte]; }
	return sum;
}

static inline bool ChecksumOp(const Flash_T * p_flash)
{
	return (CalcChecksum(&p_flash->p_OpDest[0U], p_flash->OpSize) == CalcChecksum(&p_flash->Buffer[0U], p_flash->OpSize)) ? true : false;
}

static inline const uint8_t * CalcOpCmdAddress(const Flash_T * p_flash, const uint8_t * p_dest)
{
#ifdef CONFIG_FLASH_HW_OP_ADDRESS_RELATIVE
	return (uint8_t *)((uint32_t)p_dest + p_flash->p_OpPartition->OP_ADDRESS_OFFSET);
#elif defined(CONFIG_FLASH_HW_OP_ADDRESS_ABSOLUTE)
	return p_dest;
#endif
}

static void StartCmdWritePage			(HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) {HAL_Flash_StartCmdWritePage			(p_hal, p_cmdDest, p_cmdData);}
static void StartCmdEraseSector			(HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) {HAL_Flash_StartCmdEraseSector		(p_hal, p_cmdDest);}
static void StartCmdVerifyWriteUnit		(HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) {HAL_Flash_StartCmdVerifyWriteUnit	(p_hal, p_cmdDest, p_cmdData);}
static void StartCmdVerifyEraseUnits	(HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) {HAL_Flash_StartCmdVerifyEraseUnits	(p_hal, p_cmdDest, units);}
static void StartCmdWriteOnce			(HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) {HAL_Flash_StartCmdWriteOnce			(p_hal, p_cmdDest, p_cmdData);}
static void StartCmdReadOnce			(HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) {HAL_Flash_StartCmdReadOnce			(p_hal, p_cmdDest);}

static inline bool StartOpCmd(const Flash_T * p_flash, size_t opIndex)
{
	const uint8_t * p_cmdDest = CalcOpCmdAddress(p_flash, &p_flash->p_OpDest[opIndex]);
	const uint8_t * p_cmdData = &p_flash->p_OpData[opIndex];
	size_t units = p_flash->UnitsPerCmd;

	if(p_flash->StartCmd != 0U)
	{
		p_flash->StartCmd(p_flash->CONFIG.P_HAL_FLASH, p_cmdDest, p_cmdData, units);
	}

	return (HAL_Flash_ReadErrorFlags(p_flash->CONFIG.P_HAL_FLASH) == false) ? true : false;
}

/******************************************************************************/
/*!
	Public Functions
*/
/******************************************************************************/
void Flash_Init(Flash_T * p_flash)
{
	HAL_Flash_Init(p_flash->CONFIG.P_HAL_FLASH);
	p_flash->Status 			= FLASH_STATUS_SUCCESS;
	p_flash->State 				= FLASH_STATE_IDLE;
	p_flash->p_OpDest 			= 0U;
	p_flash->OpIndex	  		= 0U;
	p_flash->OpSize 			= 0U;
	p_flash->IsVerifyEnable 	= true;
	p_flash->IsOpBuffered 		= true;;
}

void Flash_SetYield(Flash_T * p_flash, void (*yield)(void *), void * p_callbackData)
{
	p_flash->Yield = yield;
	p_flash->p_CallbackData = p_callbackData;
}

void Flash_GetReadOnce(const Flash_T * p_flash, uint8_t * p_result)
{
	memcpy(p_result, &p_flash->Buffer[0], p_flash->OpSize);
}

/******************************************************************************/
/*!
	Set - Common Blocking Non Blocking
*/
/******************************************************************************/
/*
 * Callback on end
 */
static Flash_Status_T CheckSetVerifyWrite(Flash_T * p_flash)
{
	Flash_Status_T status;

	if(ChecksumOp(p_flash) == true)
	{
		status = FLASH_STATUS_START_VERIFY;
		p_flash->StartCmd = StartCmdVerifyWriteUnit;
	}
	else
	{
		status = FLASH_STATUS_ERROR_CHECKSUM;
	}

	return status;
}

static Flash_Status_T CheckSetVerifyErase(Flash_T * p_flash)
{
	Flash_Status_T status = FLASH_STATUS_START_VERIFY;
	p_flash->StartCmd = StartCmdVerifyEraseUnits;

	return status;
}

static inline Flash_Status_T ParseErrorFlags(const Flash_T * p_flash)
{
	return (HAL_Flash_ReadErrorVerifyFlag(p_flash->CONFIG.P_HAL_FLASH) == true) ? FLASH_STATUS_ERROR_VERIFY : FLASH_STATUS_ERROR_CMD;
}

static Flash_Status_T FinalizeOpCommon(Flash_T * p_flash)
{
	Flash_Status_T status;

	if (HAL_Flash_ReadErrorFlags(p_flash->CONFIG.P_HAL_FLASH) == false)
	{
		if(p_flash->CheckSetVerify != 0U)
		{
			p_flash->CheckSetVerify = 0U; //do not verify again after verify,
			status = p_flash->CheckSetVerify(p_flash);
		}
		else
		{
			status = FLASH_STATUS_SUCCESS;
		}
	}
	else
	{
		status = ParseErrorFlags(p_flash);
	}

	return status;
}

static inline void SetOpDataPtr(Flash_T * p_flash, const uint8_t * p_source, size_t size)
{
	p_flash->p_OpData	= p_source;
	p_flash->OpSize 	= size;
//	p_flash->OpIndex 	= 0U;
}

static inline void SetOpDataBuffer(Flash_T * p_flash, const uint8_t * p_source, size_t size)
{
	if(p_source != 0U)
	{
		memcpy(&p_flash->Buffer[0U], p_source, size);
	}

	SetOpDataPtr(p_flash, p_flash->Buffer, size);
}

void Flash_SetOpData(Flash_T * p_flash, const uint8_t * p_source, size_t size)
{
	(p_flash->IsOpBuffered == true) ? SetOpDataBuffer(p_flash, p_source, size) : SetOpDataPtr(p_flash, p_source, size);
}

void Flash_IncOpPtr(Flash_T * p_flash)
{
	p_flash->p_OpDest += p_flash->OpSize;
}

static bool SetOpCommon(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size,  size_t unitSize, uint8_t unitsPerCmd, void * p_startCmd, void * p_checkSetVerify)
{
	bool isSuccess;

	if ((SearchSetParition(p_flash, p_destFlash, size) == true) && (CheckDestIsAligned(p_destFlash, size, unitSize) == true))
	{
		p_flash->p_OpDest	= p_destFlash;
		Flash_SetOpData(p_flash, p_source, size);
		p_flash->UnitsPerCmd = unitsPerCmd;
		p_flash->BytesPerCmd = unitsPerCmd * unitSize;
		p_flash->StartCmd = p_startCmd;
		p_flash->CheckSetVerify = (p_flash->IsVerifyEnable == true) ? p_checkSetVerify : 0U;
		isSuccess = true;
	}
	else
	{
		isSuccess = false;
	}

	return isSuccess;
}

bool Flash_SetWrite(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size)
{
	return SetOpCommon(p_flash, p_destFlash, p_source, size, FLASH_UNIT_WRITE_SIZE, 1U, StartCmdWritePage, CheckSetVerifyWrite);
}

bool Flash_SetErase(Flash_T * p_flash, const uint8_t * p_destFlash, size_t size)
{
	return SetOpCommon(p_flash, p_destFlash, 0U, size, FLASH_UNIT_ERASE_SIZE, 1U, StartCmdEraseSector, CheckSetVerifyErase);
}

bool Flash_SetVerifyWrite(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size)
{
	return SetOpCommon(p_flash, p_destFlash, p_source, size, FLASH_UNIT_VERIFY_WRITE_SIZE, 1U, StartCmdVerifyWriteUnit,  0U);
}

static inline size_t CalcVerifyEraseUnitsPerCmd(size_t bytes)
{
#ifdef CONFIG_FLASH_HW_VERIFY_ERASE_N_UNITS
	return bytes/FLASH_UNIT_VERIFY_ERASE_SIZE;
#elif defined(CONFIG_FLASH_HW_VERIFY_ERASE_1_UNIT)
	return 1U;
#endif
}

bool Flash_SetVerifyErase(Flash_T * p_flash, const uint8_t * p_destFlash, size_t size)
{
	return SetOpCommon(p_flash, p_destFlash, 0U, size, FLASH_UNIT_VERIFY_ERASE_SIZE, CalcVerifyEraseUnitsPerCmd(size), StartCmdVerifyEraseUnits, 0U);
}

bool Flash_SetWriteOnce(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size)
{
	return SetOpCommon(p_flash, p_destFlash, p_source, size, FLASH_UNIT_WRITE_ONCE_SIZE, 1U, StartCmdWriteOnce, 0U);
}

bool Flash_SetReadOnce(Flash_T * p_flash, const uint8_t * p_destFlash, size_t size)
{
	return SetOpCommon(p_flash, p_destFlash, 0U, size, FLASH_UNIT_READ_ONCE_SIZE, 1U, StartCmdReadOnce, 0U);
}

bool Flash_SetOp(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size, Flash_Operation_T opType)
{
	bool isSuccess;

	switch(opType)
	{
		case FLASH_OPERATION_WRITE:			isSuccess = Flash_SetWrite			(p_flash, p_destFlash, p_source, size);		break;
		case FLASH_OPERATION_ERASE:			isSuccess = Flash_SetErase			(p_flash, p_destFlash, size); 				break;
		case FLASH_OPERATION_VERIFY_WRITE:	isSuccess = Flash_SetVerifyWrite	(p_flash, p_destFlash, p_source, size);		break;
		case FLASH_OPERATION_VERIFY_ERASE:	isSuccess = Flash_SetVerifyErase	(p_flash, p_destFlash, size); 				break;
		case FLASH_OPERATION_WRITE_ONCE:	isSuccess = Flash_SetWriteOnce		(p_flash, p_destFlash, p_source, size); 	break;
		case FLASH_OPERATION_READ_ONCE:		isSuccess = Flash_SetReadOnce		(p_flash, p_destFlash, size);				break;
		default:  break;
	}

	return isSuccess;
}


/******************************************************************************/
/*!
	Blocking Implementations
*/
/******************************************************************************/
/*
 * no finalize, verify etc
 */
static Flash_Status_T ProcOpCommon_Blocking(Flash_T * p_flash)
{
	Flash_Status_T status = FLASH_STATUS_ERROR_CMD;

	if (HAL_Flash_ReadCompleteFlag(p_flash->CONFIG.P_HAL_FLASH) == true)
	{
		HAL_Flash_ClearErrorFlags(p_flash->CONFIG.P_HAL_FLASH);

		for (size_t opIndex = 0U; opIndex < p_flash->OpSize; opIndex += p_flash->BytesPerCmd)
		{
			if (StartOpCmd(p_flash, opIndex) == true)
			{
				while (HAL_Flash_ReadCompleteFlag(p_flash->CONFIG.P_HAL_FLASH) == false)
				{
					if (HAL_Flash_ReadErrorFlags(p_flash->CONFIG.P_HAL_FLASH) == true)
					{
						break;
					}
					if (p_flash->Yield != 0U)
					{
						p_flash->Yield(p_flash->p_CallbackData);
					}
				}

//				if(p_flash->OpType == FLASH_OPERATION_READ_ONCE)
//				{
//					HAL_Flash_ReadOnceData(p_flash->CONFIG.P_HAL_FLASH, &p_flash->Buffer[opIndex]);
//				}
			}
			else
			{
				break;
			}
		}

		status = FinalizeOpCommon(p_flash);

		if(status == FLASH_STATUS_START_VERIFY)
		{
			status = ProcOpCommon_Blocking(p_flash); // misra violation, single recursive call
		}
	}
	else
	{
		status = FLASH_STATUS_ERROR_BUSY;
	}

    return status;
}

/*
	Set and Proc
 */
Flash_Status_T Flash_Write_Blocking(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size)
{
	return (Flash_SetWrite(p_flash, p_destFlash, p_source, size) ? ProcOpCommon_Blocking(p_flash) : FLASH_STATUS_ERROR_INPUT);
}

Flash_Status_T Flash_Erase_Blocking(Flash_T * p_flash, const uint8_t * p_destFlash, size_t size)
{
	return (Flash_SetErase(p_flash, p_destFlash, size) ? ProcOpCommon_Blocking(p_flash) : FLASH_STATUS_ERROR_INPUT);
}

Flash_Status_T Flash_VerifyWrite_Blocking(Flash_T *p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size)
{
	return (Flash_SetVerifyWrite(p_flash, p_destFlash, p_source, size) ? ProcOpCommon_Blocking(p_flash) : FLASH_STATUS_ERROR_INPUT);
}

Flash_Status_T Flash_VerifyErase_Blocking(Flash_T *p_flash, const uint8_t * p_destFlash, size_t size)
{
	return (Flash_SetVerifyErase(p_flash, p_destFlash, size) ? ProcOpCommon_Blocking(p_flash) : FLASH_STATUS_ERROR_INPUT);
}

Flash_Status_T Flash_WriteOnce_Blocking(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size)
{
	return (Flash_SetWriteOnce(p_flash, p_destFlash, p_source, size) ? ProcOpCommon_Blocking(p_flash) : FLASH_STATUS_ERROR_INPUT);
}

Flash_Status_T Flash_ReadOnce_Blocking(Flash_T * p_flash, const uint8_t * p_destFlash, size_t size)
{
	return (Flash_SetReadOnce(p_flash, p_destFlash, size) ? ProcOpCommon_Blocking(p_flash) : FLASH_STATUS_ERROR_INPUT);
}

/*
	Erase all unsecured
 */
//Flash_Status_T Flash_EraseAll_Blocking(Flash_T *p_flash)
//{
////	Flash_EraseBytes_Blocking( p_flash, p_flash->Start , -end );
//}

/*
 */
Flash_Status_T Flash_ProcOpThis_Blocking(Flash_T * p_flash)
{
	return ProcOpCommon_Blocking(p_flash);
}

Flash_Status_T Flash_ProcOp_Blocking(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size, Flash_Operation_T opType)
{
	return (Flash_SetOp(p_flash, p_destFlash, p_source, size, opType) ? Flash_ProcOpThis_Blocking(p_flash) : FLASH_STATUS_ERROR_INPUT);
}

/******************************************************************************/
/*!
	Non Blocking
*/
/******************************************************************************/
size_t Flash_GetOpBytesRemaining(Flash_T * p_flash)
{
	return p_flash->OpSize - p_flash->OpIndex;
}

bool Flash_GetIsOpComplete(Flash_T *p_flash)
{
	return (p_flash->State == FLASH_STATE_IDLE) && (HAL_Flash_ReadCompleteFlag(p_flash->CONFIG.P_HAL_FLASH) == true);
}

/*
 * returns true when complete
 */
bool Flash_ProcOp(Flash_T * p_flash)
{
	bool isComplete = false;

	switch (p_flash->State)
	{
		case FLASH_STATE_IDLE:
			break;

		case FLASH_STATE_ACTIVE:
			//multithread use mutex
			if (HAL_Flash_ReadErrorFlags(p_flash->CONFIG.P_HAL_FLASH) == false)
			{
				if (HAL_Flash_ReadCompleteFlag(p_flash->CONFIG.P_HAL_FLASH) == true)
				{
//					if(p_flash->OpType == FLASH_OPERATION_READ_ONCE)
//					{
//						HAL_Flash_ReadOnceData(p_flash->CONFIG.P_HAL_FLASH, &p_flash->Buffer[p_flash->OpIndex]);
//					}

					if(p_flash->OpIndex < p_flash->OpSize)
					{
						if (StartOpCmd(p_flash, p_flash->OpIndex) == true)
						{
							p_flash->OpIndex += p_flash->BytesPerCmd;
						}
					}
					else  //all pages complete
					{
						p_flash->Status = FinalizeOpCommon(p_flash);

						if(p_flash->Status == FLASH_STATUS_START_VERIFY)
						{
							p_flash->Status = FLASH_STATUS_PROCESSING;
							p_flash->OpIndex = 0U;
						}
						else
						{
							p_flash->State = FLASH_STATE_IDLE;
							isComplete = true;
						}
					}
				}
			}
			else //error occurred during command operation
			{
				p_flash->Status = FLASH_STATUS_ERROR_CMD;
				p_flash->State = FLASH_STATE_IDLE;
				isComplete = true;
			}
			break;

		default:
			break;
	}

	return isComplete;
}

static Flash_Status_T StartOpCommon(Flash_T * p_flash)
{
	if(Flash_GetIsOpComplete(p_flash) == true)
	{
		HAL_Flash_ClearErrorFlags(p_flash->CONFIG.P_HAL_FLASH);
		p_flash->State = FLASH_STATE_ACTIVE;
		p_flash->Status = FLASH_STATUS_PROCESSING;
		p_flash->OpIndex = 0U;
	}
	else
	{
		p_flash->Status = FLASH_STATUS_ERROR_BUSY;
	}

    return p_flash->Status;
}

Flash_Status_T Flash_StartWrite(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size)
{
	return (p_flash->Status = (Flash_SetWrite(p_flash, p_destFlash, p_source, size) ? StartOpCommon(p_flash) : FLASH_STATUS_ERROR_INPUT));
}

Flash_Status_T Flash_StartErase(Flash_T * p_flash, const uint8_t * p_destFlash, size_t size)
{
	return (p_flash->Status = (Flash_SetErase(p_flash, p_destFlash, size) ? StartOpCommon(p_flash) : FLASH_STATUS_ERROR_INPUT));
}

Flash_Status_T Flash_StartVerifyWrite(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size)
{
	return (p_flash->Status = (Flash_SetVerifyWrite(p_flash, p_destFlash, p_source, size) ? StartOpCommon(p_flash) : FLASH_STATUS_ERROR_INPUT));
}

Flash_Status_T Flash_StartVerifyErase(Flash_T * p_flash, const uint8_t * p_destFlash, size_t size)
{
	return (p_flash->Status = (Flash_SetVerifyErase(p_flash, p_destFlash, size) ? StartOpCommon(p_flash) : FLASH_STATUS_ERROR_INPUT));
}

Flash_Status_T Flash_StartWriteOnce(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size)
{
	return (p_flash->Status = (Flash_SetWriteOnce(p_flash, p_destFlash, p_source, size) ? StartOpCommon(p_flash) : FLASH_STATUS_ERROR_INPUT));
}

Flash_Status_T Flash_StartReadOnce(Flash_T * p_flash, const uint8_t * p_destFlash, size_t size)
{
	return (p_flash->Status = (Flash_SetReadOnce(p_flash, p_destFlash, size) ? StartOpCommon(p_flash) : FLASH_STATUS_ERROR_INPUT));
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
//	memcpy(&p_flash->Buffer[0U], p_physical, size);
//
//	p_flash->p_OpDest = CalcOpCmdAddress(p_flash, p_physical);
//	p_flash->OpIndex = 0U;
//	p_flash->OpSize = size;
//
//}
//
///*
// * dest of physical flash location
// */
//void Flash_WriteVirtual(Flash_T * p_flash, const uint8_t * p_physical, const uint8_t * p_src, size_t size)
//{
//	uint32_t offset;
//
//	if (p_physical >= p_flash->p_OpDest && p_physical + size < p_flash->p_OpDest + p_flash->OpSize)
//	{
//		offset = p_physical - p_flash->p_OpDest;
//		memcpy(&p_flash->Buffer[offset], p_src, size);
//	}
//}
//
//void Flash_ReadVirtual(Flash_T * p_flash, uint8_t * p_dest, const uint8_t * p_physical, size_t size)
//{
//	uint32_t offset;
//
//	if (p_physical >= p_flash->p_OpDest && p_physical <= p_flash->p_OpDest + p_flash->OpSize)
//	{
//		offset = p_physical - p_flash->p_OpDest;
//		memcpy(p_dest, &p_flash->Buffer[offset], size);
//	}
//}
//
//void Flash_CloseVirtual(Flash_T * p_flash)
//{
//	p_flash->State = FLASH_STATE_WRITE;
//}
//
//bool Flash_CloseVirtual_Blocking(Flash_T * p_flash)
//{
//	bool isSuccess = false;
//
//	if(Flash_Erase_Blocking(p_flash, p_flash->p_OpDest, p_flash->OpSize) == true)
//	{
//		if(Flash_Write_Blocking(p_flash, p_flash->p_OpDest, &p_flash->Buffer[0U], p_flash->OpSize) == true)
//		{
//			if (ChecksumOp(p_flash) == true)
//			{
//				if(p_flash->IsVerifyEnable == true)
//				{
//					if (Flash_VerifyWrite_Blocking(p_flash, p_flash->p_OpDest, &p_flash->Buffer[0U], p_flash->OpSize) == true)
//					{
//						isSuccess = true;
//					}
//				}
//				else
//				{
//					isSuccess = true;
//				}
//			}
//		}
//	}
//
//	return isSuccess;
//}
