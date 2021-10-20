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
#define FLASH_UNIT_VERIFY_ERASE_SIZE	1//HAL_FLASH_UNIT_VERIFY_ERASE_SIZE
#define FLASH_UNIT_VERIFY_WRITE_SIZE	1//HAL_FLASH_UNIT_VERIFY_ERASE_SIZE
#define FLASH_UNIT_WRITE_ONCE_SIZE		1//HAL_FLASH_UNIT_WRITE_ONCE_SIZE
#define FLASH_UNIT_READ_ONCE_SIZE		1//HAL_FLASH_UNIT_READ_ONCE_SIZE

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
//	(((uint32_t)p_destFlash > p_partition->P_START) && ((uint32_t)p_destFlash + size) < ((uint32_t)p_partition->P_START + p_partition->SIZE));
}

static inline uint32_t CalcChecksum(const uint8_t * p_data, size_t size)
{
	uint32_t sum = 0U;
	for (size_t iByte = 0U; iByte < size; iByte++)	{ sum += p_data[iByte]; }
	return sum;
}

static inline bool ChecksumOp(const Flash_T * p_flash)
{
	return (CalcChecksum(&p_flash->p_OpFlash[0U], p_flash->OpSize) == CalcChecksum(&p_flash->Buffer[0U], p_flash->OpSize)) ? true : false;
}

static inline const uint8_t * CalcOpCmdAddress(const Flash_T * p_flash, const uint8_t * p_dest)
{
#if CONFIG_FLASH_HW_OP_ADDRESS_RELATIVE
	//	if dataflash section
//	address += offset = (0x800000U - 0x10000000);
	return address - p_flash.PARTITION_START;
	if ((dest >= temp) && (dest < (temp + pSSDConfig->DFlashSize)))
    {
        dest += 0x800000U - temp;
    }
    else

    {

	if ((p_dest >= p_flash.PARTITION_START) && (dest < (p_flash.PARTITION_START + p_flash.PARTITION_SIZE)))
	{
		*pp_dest -= p_flash.PARTITION_SIZE;
	}

#elif defined(CONFIG_FLASH_HW_OP_ADDRESS_ABSOLUTE)
	return p_dest;
#endif
}

//or use uniform function pointer/wrapper
bool StartOpCmd(const Flash_T * p_flash, size_t opIndex)
{
	//HAL_Flash_ClearErrorFlags(p_flash->CONFIG.P_HAL_FLASH);

	switch(p_flash->OpType)
	{
		case FLASH_OPERATION_WRITE: 		HAL_Flash_StartCmdWritePage			(p_flash->CONFIG.P_HAL_FLASH, &p_flash->p_OpFlash[opIndex], &p_flash->p_OpData[opIndex]);	break;
		case FLASH_OPERATION_ERASE:			HAL_Flash_StartCmdEraseSector		(p_flash->CONFIG.P_HAL_FLASH, &p_flash->p_OpFlash[opIndex]); 								break;
		case FLASH_OPERATION_VERIFY_WRITE:	HAL_Flash_StartCmdVerifyWriteUnit	(p_flash->CONFIG.P_HAL_FLASH, &p_flash->p_OpFlash[opIndex], &p_flash->p_OpData[opIndex]); 	break;
		case FLASH_OPERATION_VERIFY_ERASE:	HAL_Flash_StartCmdVerifyEraseUnits	(p_flash->CONFIG.P_HAL_FLASH, &p_flash->p_OpFlash[opIndex], p_flash->UnitsPerCmd); 			break;
		case FLASH_OPERATION_WRITE_ONCE:	HAL_Flash_StartCmdWriteOnce			(p_flash->CONFIG.P_HAL_FLASH, &p_flash->p_OpFlash[opIndex], &p_flash->p_OpData[opIndex]);	break;
		case FLASH_OPERATION_READ_ONCE:		HAL_Flash_StartCmdReadOnce			(p_flash->CONFIG.P_HAL_FLASH, &p_flash->p_OpFlash[opIndex]); 								break;
		default: break;
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
	p_flash->Status 			= FLASH_STATUS_SUCCESS;
	p_flash->State 				= FLASH_STATE_IDLE;
	p_flash->p_OpFlash 			= 0U;
	p_flash->OpIndex	  		= 0U;
	p_flash->OpSize 			= 0U;
	p_flash->IsVerifyEnable 	= true;
	p_flash->IsForceAlignEnable = true;;
}

void Flash_SetYield(Flash_T * p_flash, void (* yield)(void *), void * p_callbackData)
{
	p_flash->Yield = yield;
	p_flash->p_CallbackData = p_callbackData;
}


bool Flash_CheckIsBounded(const Flash_T * p_flash, const uint8_t * p_dest, size_t size)
{
	bool isBounded = false;

	for (uint8_t iPartition = 0U; iPartition < CONFIG_FLASH_PARTITION_COUNT; iPartition++)
	{
		if(CheckDestIsBoundedPartition(p_dest, size, &p_flash->CONFIG.PARTITIONS[iPartition]) == true)
		{
			isBounded = true;
		}
	}

	return isBounded;
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
inline void Flash_SetOpDataBuffer(Flash_T * p_flash, const uint8_t * p_source, size_t size)
{
	if(p_source != 0U)
	{
		memcpy(&p_flash->Buffer[0U], p_source, size);
		p_flash->p_OpData	= p_flash->Buffer;
		p_flash->OpSize  	= size;
	}
}

inline void Flash_SetOpDataPtr(Flash_T * p_flash, const uint8_t * p_source, size_t size)
{
	p_flash->p_OpData	= p_source;
	p_flash->OpSize 	= size;
}

inline void Flash_SetOpData(Flash_T * p_flash, const uint8_t * p_source, size_t size)
{
	(p_flash->IsOpBuffered == true) ? Flash_SetOpDataBuffer(p_flash, p_source, size) : Flash_SetOpDataPtr(p_flash, p_source, size);
}

//inline void Flash_SetOpNextData(Flash_T * p_flash, const uint8_t * p_source, size_t size)
//{
//	p_flash->p_OpFlash	+= p_flash->OpSize;
//	p_flash->OpIndex 	= 0U;
//	Flash_SetOpData(p_flash, p_source, size);
//}

static bool SetOpCommon(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size, Flash_Operation_T opType, size_t unitSize, uint8_t unitsPerCmd)
{
	bool isSuccess;

	if ((Flash_CheckIsBounded(p_flash, p_destFlash, size) == true) && (CheckDestIsAligned(p_destFlash, size, unitSize) == true))
	{
		p_flash->p_OpFlash	= CalcOpCmdAddress(p_flash, p_destFlash);
		p_flash->OpIndex 	= 0U;
		p_flash->OpType		= opType;
		Flash_SetOpData(p_flash, p_source, size);
//		p_flash->Status 	= FLASH_STATUS_SUCCESS;

		p_flash->UnitsPerCmd = unitsPerCmd;
		p_flash->BytesPerCmd = unitsPerCmd * unitSize;
		isSuccess = true;
	}
	else
	{
//		p_flash->Status = FLASH_STATUS_ERROR_INPUT;
		isSuccess = false;
	}

	return isSuccess;
}

//static bool SetOpMultiUnit(Flash_T * p_flash, uint8_t unitSize, uint8_t unitsPerCmd)
//{
//	p_flash->UnitsPerCmd = unitsPerCmd;
//	p_flash->BytesPerCmd = unitsPerCmd * unitSize;
//}

bool Flash_SetWrite(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size)
{
	return SetOpCommon(p_flash, p_destFlash, p_source, size, FLASH_OPERATION_WRITE, FLASH_UNIT_WRITE_SIZE, 1U);
}

bool Flash_SetErase(Flash_T * p_flash, const uint8_t * p_destFlash, size_t size)
{
	return SetOpCommon(p_flash, p_destFlash, 0U, size, FLASH_OPERATION_ERASE, FLASH_UNIT_ERASE_SIZE, 1U);
}

bool Flash_SetVerifyWrite(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size)
{
	return SetOpCommon(p_flash, p_destFlash, p_source, size, FLASH_OPERATION_WRITE, FLASH_UNIT_VERIFY_WRITE_SIZE, 1U);
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
	return SetOpCommon(p_flash, p_destFlash, 0U, size, FLASH_OPERATION_VERIFY_ERASE, FLASH_UNIT_VERIFY_ERASE_SIZE, CalcVerifyEraseUnitsPerCmd(size));
}

bool Flash_SetWriteOnce(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size)
{
	return SetOpCommon(p_flash, p_destFlash, p_source, size, FLASH_OPERATION_WRITE_ONCE, FLASH_UNIT_WRITE_ONCE_SIZE, 1U);
}

bool Flash_SetReadOnce(Flash_T * p_flash, const uint8_t * p_destFlash, size_t size)
{
	return SetOpCommon(p_flash, p_destFlash, 0U, size, FLASH_OPERATION_READ_ONCE, FLASH_UNIT_READ_ONCE_SIZE, 1U);
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
	 Common Blocking Non Blocking
*/
/******************************************************************************/
static inline Flash_Status_T CheckEndNeedVerify(const Flash_T * p_flash)
{
	return (p_flash->IsVerifyEnable == true) ? FLASH_STATUS_START_VERIFY : FLASH_STATUS_SUCCESS;
}

static inline Flash_Status_T CheckEndWrite(const Flash_T * p_flash)
{
	return (ChecksumOp(p_flash) == true) ? CheckEndNeedVerify(p_flash) : FLASH_STATUS_ERROR_CHECKSUM;
}

static inline Flash_Status_T CheckEndErase(const Flash_T * p_flash)
{
	return CheckEndNeedVerify(p_flash);
}

static inline Flash_Status_T CheckEndVerify(const Flash_T * p_flash)
{
	return (HAL_Flash_ReadErrorVerifyFlag(p_flash->CONFIG.P_HAL_FLASH) == true) ? FLASH_STATUS_ERROR_VERIFY : FLASH_STATUS_ERROR_CMD;
}

//static inline Flash_Status_T CheckEndWriteErase(const Flash_T * p_flash)
//{
//	Flash_Status_T status;
//
//	switch(p_flash->OpType)
//	{
//		case FLASH_OPERATION_WRITE:	status = CheckEndWrite(p_flash);	break;
//		case FLASH_OPERATION_ERASE:	status = CheckEndErase(p_flash);	break;
//		default:  break;
//	}
//
//	return status;
//}

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

	if (HAL_Flash_ReadCompleteFlags(p_flash->CONFIG.P_HAL_FLASH) == true)
	{
		for (size_t opIndex = 0U; opIndex < p_flash->OpSize; opIndex += p_flash->BytesPerCmd)
		{
			if (StartOpCmd(p_flash, opIndex) == true)
			{
				while (HAL_Flash_ReadCompleteFlags(p_flash->CONFIG.P_HAL_FLASH) == false)
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

				if(p_flash->OpType == FLASH_OPERATION_READ_ONCE)
				{
					HAL_Flash_ReadOnceData(p_flash->CONFIG.P_HAL_FLASH, &p_flash->Buffer[opIndex]);
				}
			}
			else
			{
				break;
			}
		}

		if (HAL_Flash_ReadErrorFlags(p_flash->CONFIG.P_HAL_FLASH) == false)
		{
			status = FLASH_STATUS_SUCCESS;
		}
		else
		{
			status = FLASH_STATUS_ERROR; //error tbd
		}
	}
	else
	{
		status = FLASH_STATUS_ERROR_BUSY;
	}

    return status;
}

/*
 * Block Proc after set
 */
Flash_Status_T Flash_ProcVerify_Blocking(Flash_T * p_flash)
{
	Flash_Status_T status = ProcOpCommon_Blocking(p_flash);
	return ((status == FLASH_STATUS_ERROR) ? CheckEndVerify(p_flash) : status);
}

Flash_Status_T Flash_ProcWrite_Blocking(Flash_T * p_flash)
{
	Flash_Status_T status = ProcOpCommon_Blocking(p_flash);

	if(status == FLASH_STATUS_SUCCESS)
	{
		status = CheckEndWrite(p_flash); // (ChecksumOp(p_flash) == true) ? CheckEndNeedVerify(p_flash) : FLASH_STATUS_ERROR_CHECKSUM;

		if(status == FLASH_STATUS_START_VERIFY)
		{
			p_flash->OpType	= FLASH_OPERATION_VERIFY_WRITE;
			Flash_ProcVerify_Blocking(p_flash);
		}
	}

	return status;
}

Flash_Status_T Flash_ProcErase_Blocking(Flash_T * p_flash)
{
	Flash_Status_T status = ProcOpCommon_Blocking(p_flash);

	if(status == FLASH_STATUS_SUCCESS)
	{
		status = CheckEndNeedVerify(p_flash);

		if(status == FLASH_STATUS_START_VERIFY)
		{
			p_flash->OpType	= FLASH_OPERATION_VERIFY_ERASE;
			status = Flash_ProcVerify_Blocking(p_flash);
		}
	}

	return status;
}

/*
	Set and Proc
 */
Flash_Status_T Flash_Write_Blocking(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size)
{
	return (Flash_SetWrite(p_flash, p_destFlash, p_source, size) ? Flash_ProcWrite_Blocking(p_flash) : FLASH_STATUS_ERROR_INPUT);
}

Flash_Status_T Flash_Erase_Blocking(Flash_T * p_flash, const uint8_t * p_destFlash, size_t size)
{
	return (Flash_SetErase(p_flash, p_destFlash, size) ? Flash_ProcErase_Blocking(p_flash) : FLASH_STATUS_ERROR_INPUT);
}

Flash_Status_T Flash_VerifyWrite_Blocking(Flash_T *p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size)
{
	return (Flash_SetVerifyWrite(p_flash, p_destFlash, p_source, size) ? Flash_ProcVerify_Blocking(p_flash) : FLASH_STATUS_ERROR_INPUT);
}

Flash_Status_T Flash_VerifyErase_Blocking(Flash_T *p_flash, const uint8_t * p_destFlash, size_t size)
{
	return (Flash_SetVerifyErase(p_flash, p_destFlash, size) ? Flash_ProcVerify_Blocking(p_flash) : FLASH_STATUS_ERROR_INPUT);
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
 * Wrapper
 * Common with finalize, verify
 */
Flash_Status_T Flash_ProcOpThis_Blocking(Flash_T * p_flash)
{
	Flash_Status_T status;

	switch(p_flash->OpType)
	{
		case FLASH_OPERATION_WRITE: 		status = Flash_ProcWrite_Blocking	(p_flash); 	break;
		case FLASH_OPERATION_ERASE:			status = Flash_ProcErase_Blocking	(p_flash); 	break;
		case FLASH_OPERATION_VERIFY_WRITE:	status = Flash_ProcVerify_Blocking	(p_flash); 	break;
		case FLASH_OPERATION_VERIFY_ERASE:	status = Flash_ProcVerify_Blocking	(p_flash); 	break;
		case FLASH_OPERATION_WRITE_ONCE:	status = ProcOpCommon_Blocking		(p_flash); 	break;
		case FLASH_OPERATION_READ_ONCE:		status = ProcOpCommon_Blocking		(p_flash); 	break;
		default: break;
	}

	return status;
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
	return (p_flash->State == FLASH_STATE_IDLE) && (HAL_Flash_ReadCompleteFlags(p_flash->CONFIG.P_HAL_FLASH) == true);
}

static Flash_Status_T StartOpCommon(Flash_T * p_flash)
{
	if(Flash_GetIsOpComplete(p_flash) == true)
	{
		HAL_Flash_ClearErrorFlags(p_flash->CONFIG.P_HAL_FLASH);
		p_flash->State = FLASH_STATE_ACTIVE;
		p_flash->Status = FLASH_STATUS_PROCESSING;
	}
	else
	{
		p_flash->Status = FLASH_STATUS_ERROR_BUSY;
	}

    return p_flash->Status;
}

//Flash_Status_T FinalizeWriteEraseCommon(Flash_T * p_flash, Flash_Operation_T verifyOp)
//{
//		p_flash->OpType = verifyOp;
//		p_flash->OpIndex = 0U;
//		p_flash->Status = StartOpCommon(p_flash);
//
//	return (p_flash->Status);
//}

/*
 * returns ERROR_CHECKSUM, FLASH_STATUS_SUCCESS, FLASH_STATUS_PROCESSING, ERROR_INPUT
 *
 */
Flash_Status_T Flash_FinalizeWrite(Flash_T * p_flash)
{
	p_flash->Status = CheckEndWrite(p_flash);

	if (p_flash->Status == FLASH_STATUS_START_VERIFY)
	{
		p_flash->OpType = FLASH_OPERATION_VERIFY_WRITE;
		p_flash->OpIndex = 0U;
		p_flash->Status = StartOpCommon(p_flash);
	}
	else
	{
		p_flash->State = FLASH_STATE_IDLE;
	}

	return (p_flash->Status);

//	return ((p_flash->Status == FLASH_STATUS_START_VERIFY) ? FinalizeWriteEraseCommon(p_flash, FLASH_OPERATION_VERIFY_WRITE) : p_flash->Status);
}

Flash_Status_T Flash_FinalizeErase(Flash_T * p_flash)
{
	p_flash->Status = CheckEndErase(p_flash);

	if (p_flash->Status == FLASH_STATUS_START_VERIFY)
	{
		p_flash->OpType = FLASH_OPERATION_VERIFY_ERASE;
		p_flash->OpIndex = 0U;
		p_flash->Status = StartOpCommon(p_flash);
	}
	else
	{
		p_flash->State = FLASH_STATE_IDLE;
	}

	return (p_flash->Status);

//	return ((p_flash->Status == FLASH_STATUS_START_VERIFY) ? FinalizeWriteEraseCommon(p_flash, FLASH_OPERATION_VERIFY_ERASE) : p_flash->Status);
}

Flash_Status_T Flash_FinalizeVerify(Flash_T * p_flash)
{
	p_flash->Status = CheckEndVerify(p_flash);
	p_flash->State = FLASH_STATE_IDLE;
	return (p_flash->Status);
}

/*
 * returns true when complete
 */
bool Flash_ProcOpCommon(Flash_T * p_flash)
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
				if (HAL_Flash_ReadCompleteFlags(p_flash->CONFIG.P_HAL_FLASH) == true)
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
						else
						{
							p_flash->State = FLASH_STATE_IDLE;
							p_flash->Status = FLASH_STATUS_ERROR_CMD;
							isComplete = true;
						}
					}
					else  //all pages complete
					{
						//user finalize to return to idle state// go to idle if multi threaded?
						p_flash->State = FLASH_STATE_IDLE;
						p_flash->Status = FLASH_STATUS_SUCCESS;
						isComplete = true;
					}
				}
			}
			else
			{
				p_flash->State = FLASH_STATE_IDLE;
				p_flash->Status = FLASH_STATUS_ERROR_CMD;
				isComplete = true;
			}
			break;

		default:
			break;
	}

	return isComplete;
}

/*
 * Nonblock uses wrapper proc function by default
 */
bool Flash_ProcOp(Flash_T * p_flash)
{
	bool isComplete = false;

	if(Flash_ProcOpCommon(p_flash) == true)
	{
		if (p_flash->Status == FLASH_STATUS_SUCCESS)
		{
			switch(p_flash->OpType)
			{
				case FLASH_OPERATION_WRITE: 		Flash_FinalizeWrite			(p_flash); 	break;
				case FLASH_OPERATION_ERASE:			Flash_FinalizeErase			(p_flash); 	break;
				case FLASH_OPERATION_WRITE_ONCE:	  	break;
				case FLASH_OPERATION_READ_ONCE:		  	break;
				default: break;
			}
		}
		else
		{
			switch(p_flash->OpType)
			{
				case FLASH_OPERATION_VERIFY_WRITE:	Flash_FinalizeVerify	(p_flash); 	break;
				case FLASH_OPERATION_VERIFY_ERASE:	Flash_FinalizeVerify	(p_flash); 	break;
				default: break;
			}
		}

		if(p_flash->Status != FLASH_STATUS_PROCESSING)
		{
//				p_flash->State = FLASH_STATE_IDLE;
			isComplete = true;
		}
	}

	return isComplete;
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
//	p_flash->p_OpFlash = CalcOpCmdAddress(p_flash, p_physical);
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
//	if (p_physical >= p_flash->p_OpFlash && p_physical + size < p_flash->p_OpFlash + p_flash->OpSize)
//	{
//		offset = p_physical - p_flash->p_OpFlash;
//		memcpy(&p_flash->Buffer[offset], p_src, size);
//	}
//}
//
//void Flash_ReadVirtual(Flash_T * p_flash, uint8_t * p_dest, const uint8_t * p_physical, size_t size)
//{
//	uint32_t offset;
//
//	if (p_physical >= p_flash->p_OpFlash && p_physical <= p_flash->p_OpFlash + p_flash->OpSize)
//	{
//		offset = p_physical - p_flash->p_OpFlash;
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
//	if(Flash_Erase_Blocking(p_flash, p_flash->p_OpFlash, p_flash->OpSize) == true)
//	{
//		if(Flash_Write_Blocking(p_flash, p_flash->p_OpFlash, &p_flash->Buffer[0U], p_flash->OpSize) == true)
//		{
//			if (ChecksumOp(p_flash) == true)
//			{
//				if(p_flash->IsVerifyEnable == true)
//				{
//					if (Flash_VerifyWrite_Blocking(p_flash, p_flash->p_OpFlash, &p_flash->Buffer[0U], p_flash->OpSize) == true)
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
