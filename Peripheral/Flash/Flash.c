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
#include "Flash.h"

#include "HAL_Flash.h"

#include <stdint.h>
#include <stdbool.h>

#define FLASH_UNIT_SIZE_ERASE	HAL_FLASH_UNIT_SIZE_ERASE
#define FLASH_UNIT_SIZE_WRITE	HAL_FLASH_UNIT_SIZE_WRITE


uint8_t * CalcRemapAddress(const Flash_T * p_flash, uint8_t * p_dest)
{
#if CONFIG_FLASH_HAL_USE_ADDRESS_RELATIVE
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

#elif defined(CONFIG_FLASH_HAL_USE_ADDRESS_ABSOLUTE)
	return address;
#endif
	//	if dataflash section
//	address += offset = (0x800000U - 0x10000000);
}

#ifndef FORCE_ALIGN_DOWN
#define FORCE_ALIGN_DOWN(size, align) ((size) & -(align))
#endif
#ifndef FORCE_ALIGN_UP
#define FORCE_ALIGN_UP(size, align) (-(-(size) & -(align)))
#endif

static inline bool ProcForceAlign(const Flash_T * p_flash, const uint8_t * p_dest)
{

}



static inline bool CheckIsAligned(const Flash_T * p_flash, const uint8_t * p_destFlash, size_t sizeBytes)
{
	//p_destFlash%FLASH_UNIT_SIZE_WRITE == 0, unit size always power of 2
	return (((uint32_t)p_destFlash & (FLASH_UNIT_SIZE_WRITE - 1U)) == 0U) &&
			(((uint32_t)sizeBytes & (FLASH_UNIT_SIZE_WRITE - 1U)) == 0U);
}

static bool StartWritePage(const Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_data)
{
	bool isStarted = false;

	HAL_Flash_ClearErrorFlags(p_flash->CONFIG.P_HAL_FLASH);
	HAL_Flash_PrepCmdWrite(p_flash->CONFIG.P_HAL_FLASH, p_destFlash, p_data); //Chip unique procedures
	HAL_Flash_WriteCmdWritePage(p_flash->CONFIG.P_HAL_FLASH);
	HAL_Flash_WriteCmdWriteDest(p_flash->CONFIG.P_HAL_FLASH, p_destFlash);
	HAL_Flash_WriteCmdWriteData(p_flash->CONFIG.P_HAL_FLASH, p_data);
	HAL_Flash_WriteCmdWriteStart(p_flash->CONFIG.P_HAL_FLASH);
	if (HAL_Flash_ReadErrorFlags(p_flash->CONFIG.P_HAL_FLASH) != true)	{isStarted = true;}

    return isStarted;
}

static bool StartEraseBlock(const Flash_T * p_flash, const uint8_t * p_destFlash)
{
	bool isStarted = false;

	HAL_Flash_ClearErrorFlags(p_flash->CONFIG.P_HAL_FLASH);
	HAL_Flash_PrepCmdErase(p_flash->CONFIG.P_HAL_FLASH, p_destFlash); //Chip unique procedures
	HAL_Flash_WriteCmdEraseBlock(p_flash->CONFIG.P_HAL_FLASH);
	HAL_Flash_WriteCmdEraseDest(p_flash->CONFIG.P_HAL_FLASH, p_destFlash);
	HAL_Flash_WriteCmdEraseStart(p_flash->CONFIG.P_HAL_FLASH);
	if (HAL_Flash_ReadErrorFlags(p_flash->CONFIG.P_HAL_FLASH) != true)	{isStarted = true;}

    return isStarted;
}

static bool StartVerifyWritePage(const Flash_T * p_flash, const uint8_t * p_destFlash)
{
	bool isStarted = false;

	HAL_Flash_ClearErrorFlags(p_flash->CONFIG.P_HAL_FLASH);
	HAL_Flash_PrepCmdVerifyWrite(p_flash->CONFIG.P_HAL_FLASH, p_destFlash); //Chip unique procedures
	HAL_Flash_WriteCmdVerifyWrite(p_flash->CONFIG.P_HAL_FLASH);
	HAL_Flash_WriteCmdVerifyWriteDest(p_flash->CONFIG.P_HAL_FLASH, p_destFlash);
	HAL_Flash_WriteCmdVerifyWriteStart(p_flash->CONFIG.P_HAL_FLASH);
	if (HAL_Flash_ReadErrorFlags(p_flash->CONFIG.P_HAL_FLASH) != true)	{isStarted = true;}

    return isStarted;
}

static bool StartVerifyEraseBlock(const Flash_T * p_flash, const uint8_t * p_destFlash)
{
	bool isStarted = false;

	HAL_Flash_ClearErrorFlags(p_flash->CONFIG.P_HAL_FLASH);
	HAL_Flash_PrepCmdVerifyErase(p_flash->CONFIG.P_HAL_FLASH, p_destFlash); //Chip unique procedures
	HAL_Flash_WriteCmdVerifyBlock(p_flash->CONFIG.P_HAL_FLASH);
	HAL_Flash_WriteCmdVerifyEraseDest(p_flash->CONFIG.P_HAL_FLASH, p_destFlash);
	HAL_Flash_WriteCmdVerifyEraseStart(p_flash->CONFIG.P_HAL_FLASH);
	if (HAL_Flash_ReadErrorFlags(p_flash->CONFIG.P_HAL_FLASH) != true)	{isStarted = true;}

    return isStarted;
}

void Flash_Init(Flash_T * p_flash)
{
	Queue_Init(&p_flash->Queue);
	p_flash->Status = FLASH_STATUS_SUCCESS;
	p_flash->State 	= FLASH_STATE_IDLE;
	p_flash->p_Write 		= 0U;
	p_flash->WriteIndex  	= 0U;
	p_flash->WriteSize 		= 0U;
	p_flash->IsVerifyEnable 	= true;
	p_flash->IsForceAlignEnable = true;;
}

  bool Flash_CheckBoundary(const Flash_T * p_flash, const uint8_t * p_dest)
{
	//for each partition in flash, check dest
}

void Flash_SetBlockingYield(Flash_T * p_flash, void (* yield)(void *), void * p_callbackData)
{
	p_flash->Yield = yield;
	p_flash->p_CallbackData = p_callbackData;
}

//returns true when "blocking" is over
bool Flash_PollWrite(Flash_T * p_flash)
{
	bool isCompleteNow = false;

	switch (p_flash->State)
	{
		case FLASH_STATE_IDLE:
			break;

		case FLASH_STATE_WRITE:
			//multithread use mutex
			if (HAL_Flash_ReadErrorFlags(p_flash->CONFIG.P_HAL_FLASH) == false)
			{
				if (HAL_Flash_ReadCompleteWriteFlag(p_flash->CONFIG.P_HAL_FLASH) == true)
				{
					Queue_Remove(&p_flash->Queue, 1U);
					p_flash->WriteIndex++;

					if (Queue_PeekPtrFront(&p_flash->Queue) != 0U)
					{
						if (StartWritePage(p_flash, &p_flash->p_Write[p_flash->WriteIndex], Queue_PeekPtrFront(&p_flash->Queue)) == false)
						{
							p_flash->State = FLASH_STATE_IDLE;
							p_flash->Status = FLASH_STATUS_ERROR;
							isCompleteNow = true;
						}
					}
					else  //all pages complete
					{
						if(p_flash->IsVerifyEnable == true)
						{
							p_flash->State = FLASH_STATE_VERIFY;
						}
						else
						{
							p_flash->State = FLASH_STATE_IDLE;
							p_flash->Status = FLASH_STATUS_SUCCESS;
							isCompleteNow = true;
							//oncomplete
						}
					}
				}
			}
			else
			{
				p_flash->Status == FLASH_STATUS_ERROR;
				p_flash->State = FLASH_STATE_IDLE;
			}
			break;

		case FLASH_STATE_VERIFY:

			break;
		default:
			break;
	}

	return isCompleteNow;
}

//bool Flash_GetIsWriteComplete(Flash_T *p_flash)
//{
////	return HAL_Flash_ReadCompleteFlag(p_flash->CONFIG.P_HAL_FLASH);
////	{
////	return (p_flash->IsWriteComplete);
//}

bool Flash_GetIsSuccess(Flash_T * p_flash)
{
	return (p_flash->Status == FLASH_STATUS_SUCCESS) ? true : false;
}

bool Flash_PollWriteSuccess(Flash_T *p_flash)
{
	if(Flash_PollWrite(p_flash))
	{
		return Flash_GetIsSuccess(p_flash);
	}
}

size_t Flash_GetWriteBytesRemaining(Flash_T * p_flash)
{
	return (Queue_GetFullCount(&p_flash->Queue) * FLASH_UNIT_SIZE_WRITE);
}

static inline uint32_t GetCheckSum(uint8_t * p_data, uint16_t sizeBytes)
{
	uint32_t sum = 0U;

	for (size_t iByte = 0U; iByte < sizeBytes; iByte++)
	{
		sum += p_data[iByte];
	}

	return sum;
}

uint32_t Flash_GetCheckSumFlash(Flash_T * p_flash)
{
	return GetCheckSum(p_flash->p_Write, p_flash->WriteSize);
}

uint32_t Flash_GetCheckSumBuffer(Flash_T * p_flash)
{
	return GetCheckSum(p_flash->CONFIG.QUEUE.P_BUFFER, p_flash->WriteSize);
}


bool Flash_StartWritePages(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t countPages)
{
	bool isSuccess = false;

    return isSuccess;
}

bool Flash_StartWriteBytes(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t sizeBytes)
{
	bool isSuccess = false;
	Flash_Status_T status = FLASH_STATUS_ERROR;
	//check partition boundarys and align address up if needed

	if((p_flash->State == FLASH_STATE_IDLE) && (HAL_Flash_ReadCompleteWriteFlag(p_flash->CONFIG.P_HAL_FLASH) == true))
	{
		if (CheckIsAligned(p_flash, p_destFlash, p_source))
		{
			if(Queue_EnqueueBytes(&p_flash->Queue, p_source, sizeBytes))
			{
				if (Queue_PeekPtrFront(&p_flash->Queue) != 0U)
				{
					if (StartWritePage(p_flash, p_destFlash, Queue_PeekPtrFront(&p_flash->Queue)) == true)
					{
						p_flash->p_Write  	= p_destFlash;
						p_flash->WriteIndex = 0U;
						p_flash->WriteSize 	= sizeBytes;

						p_flash->State = FLASH_STATE_WRITE;
						status = FLASH_STATUS_SUCCESS;
						isSuccess = true;
					}
				}

			}
		}
    }

	p_flash->Status = status;

    return isSuccess;
}

bool Flash_EraseBlocks(Flash_T * p_flash, const uint8_t * p_destFlash, size_t sizeUnits)
{
	bool isSuccess = false;

    return isSuccess;
}

bool Flash_EraseBytes(Flash_T * p_flash, const uint8_t * p_destFlash, size_t sizeBytes)
{
	bool isSuccess = false;

    return isSuccess;
}

bool Flash_StartVerifyWritePages(Flash_T * p_flash, const uint8_t * p_destFlash, size_t sizeBytes)
{
	bool isSuccess = false;

    return isSuccess;
}

bool Flash_StartVerifyEraseBlocks(Flash_T * p_flash, const uint8_t * p_destFlash, size_t sizeBytes)
{
	bool isSuccess = false;

    return isSuccess;
}



bool Flash_WriteBytes_Blocking(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t sizeBytes)
{
	bool isSuccess = false;

	if (CheckIsAligned(p_flash, p_destFlash, sizeBytes))
	{
		//check partition boundaries and update address
		if (HAL_Flash_ReadCompleteWriteFlag(p_flash->CONFIG.P_HAL_FLASH) == true)
		{
			for (uint32_t index = 0U; index < sizeBytes; index += FLASH_UNIT_SIZE_WRITE)
			{
				if (StartWritePage(p_flash->CONFIG.P_HAL_FLASH, &p_destFlash[index], &p_source[index]) == true)
				{
					while (HAL_Flash_ReadCompleteWriteFlag(p_flash->CONFIG.P_HAL_FLASH) == false)
					{
						if (HAL_Flash_ReadErrorFlags(p_flash->CONFIG.P_HAL_FLASH) == true)
						{
							break;
						}
						if (p_flash->Yield)
						{
							p_flash->Yield(p_flash->p_CallbackData);
						}
					}
				}
				else
				{
					break;
				}
			}

			if (HAL_Flash_ReadErrorFlags(p_flash->CONFIG.P_HAL_FLASH) == false)
			{
				isSuccess = true;
			}
		}
    }

    return isSuccess;
}

bool Flash_EraseBytes_Blocking(Flash_T *p_flash, const uint8_t * p_destFlash, size_t sizeBytes)
{
	bool isSuccess = false;

	if (CheckIsAligned(p_flash, p_destFlash, sizeBytes))
	{

		if (HAL_Flash_ReadCompleteEraseFlag(p_flash->CONFIG.P_HAL_FLASH) == true)
		{
			for (uint32_t index = 0U; index < sizeBytes; index += FLASH_UNIT_SIZE_WRITE)
			{
				if (StartEraseBlock(p_flash->CONFIG.P_HAL_FLASH, &p_destFlash[index]) == true)
				{
					while (HAL_Flash_ReadCompleteEraseFlag(p_flash->CONFIG.P_HAL_FLASH) == false)
					{
						if (HAL_Flash_ReadErrorFlags(p_flash->CONFIG.P_HAL_FLASH) == true)
						{
							break;
						}
						if (p_flash->Yield)
						{
							p_flash->Yield(p_flash->p_CallbackData);
						}
					}
				}
				else
				{
					break;
				}
			}

			if (HAL_Flash_ReadErrorFlags(p_flash->CONFIG.P_HAL_FLASH) == false)
			{
				isSuccess = true;
			}
		}
    }

    return isSuccess;
}

bool Flash_VerifyWriteBytes_Blocking(Flash_T *p_flash, const uint8_t * p_destFlash, size_t sizeBytes)
{
	bool isSuccess = false;

	if (CheckIsAligned(p_flash, p_destFlash, sizeBytes))
	{

		if (HAL_Flash_ReadCompleteVerifyWriteFlag(p_flash->CONFIG.P_HAL_FLASH) == true)
		{
			for (uint32_t index = 0U; index < sizeBytes; index += FLASH_UNIT_SIZE_WRITE)
			{
				if (StartVerifyWritePage(p_flash->CONFIG.P_HAL_FLASH, &p_destFlash[index]) == true)
				{
					while (HAL_Flash_ReadCompleteVerifyWriteFlag(p_flash->CONFIG.P_HAL_FLASH) == false)
					{
						if (HAL_Flash_ReadErrorFlags(p_flash->CONFIG.P_HAL_FLASH) == true)
						{
							break;
						}
						if (p_flash->Yield)
						{
							p_flash->Yield(p_flash->p_CallbackData);
						}
					}
				}
				else
				{
					break;
				}
			}

			if (HAL_Flash_ReadErrorFlags(p_flash->CONFIG.P_HAL_FLASH) == false)
			{
				isSuccess = true;
			}
		}
    }

    return isSuccess;
}

bool Flash_VerifyEraseBytes_Blocking(Flash_T *p_flash, const uint8_t * p_destFlash, size_t sizeBytes)
{
	bool isSuccess = false;

	if (CheckIsAligned(p_flash, p_destFlash, sizeBytes))
	{

		if (HAL_Flash_ReadCompleteVerifyEraseFlag(p_flash->CONFIG.P_HAL_FLASH) == true)
		{
			for (uint32_t index = 0U; index < sizeBytes; index += FLASH_UNIT_SIZE_WRITE)
			{
				if (StartVerifyEraseBlock(p_flash->CONFIG.P_HAL_FLASH, &p_destFlash[index]) == true)
				{
					while (HAL_Flash_ReadCompleteVerifyEraseFlag(p_flash->CONFIG.P_HAL_FLASH) == false)
					{
						if (HAL_Flash_ReadErrorFlags(p_flash->CONFIG.P_HAL_FLASH) == true)
						{
							break;
						}
						if (p_flash->Yield)
						{
							p_flash->Yield(p_flash->p_CallbackData);
						}
					}
				}
				else
				{
					break;
				}
			}

			if (HAL_Flash_ReadErrorFlags(p_flash->CONFIG.P_HAL_FLASH) == false)
			{
				isSuccess = true;
			}
		}
    }

    return isSuccess;
}

/*
 * creates copy of flash in buffer
 */
bool Flash_OpenVirtual(Flash_T * p_flash, const uint8_t * p_physical, size_t sizeBytes)
{
	Queue_Clear(&p_flash->Queue);

	if(Queue_EnqueueN(&p_flash->Queue, p_physical, sizeBytes))
	{
		p_flash->p_Write = p_flash;
		p_flash->WriteIndex = 0U;
		p_flash->WriteSize = sizeBytes;
	}
}

/*
 * dest of physical flash location
 */
bool Flash_WriteVirtual(Flash_T * p_flash, const uint8_t * p_physical, const uint8_t * p_src, size_t sizeBytes)
{
	uint32_t offset;

	if (p_physical >= p_flash->p_Write && p_physical < p_flash->p_Write + p_flash->WriteSize)
	{
		offset = p_physical - p_flash->p_Write;
		memcpy(&p_flash->CONFIG.QUEUE.P_BUFFER[offset], p_src, sizeBytes);
//		for (uint32_t index = 0U; index < sizeBytes; index++)
//		{
////			p_flash->Buffer[offset + index] = p_src[index];
//			((uint8_t *)p_flash->CONFIG.QUEUE.P_BUFFER)[offset + index] = p_src[index];
//		}
	}
}

bool Flash_ReadVirtual(Flash_T * p_flash, uint8_t * p_dest, const uint8_t * p_physical, size_t sizeBytes)
{
	uint32_t offset;

	if (p_physical >= p_flash->p_Write && p_physical < p_flash->p_Write + p_flash->WriteSize)
	{
		offset = p_physical - p_flash->p_Write;
		memcpy(p_dest, &p_flash->CONFIG.QUEUE.P_BUFFER[offset], sizeBytes);
	}
}

bool Flash_CloseVirtual(Flash_T * p_flash)
{
	p_flash->State = FLASH_STATE_WRITE;
}

bool Flash_CloseVirtual_Blocking(Flash_T * p_flash)
{
	bool isSucess = false;

	if(Flash_EraseBytes_Blocking(p_flash, p_flash->p_Write, p_flash->WriteSize) ==true)
	{
		if(Flash_WriteBytes_Blocking(p_flash, p_flash->p_Write, &p_flash->Buffer, p_flash->WriteSize) == true)
		{
			if (Flash_GetCheckSumFlash(p_flash) == Flash_GetCheckSumBuffer(p_flash))
			{
				if(p_flash->IsVerifyEnable == true)
				{
					if (Flash_VerifyBytes_Blocking(p_flash, p_flash->p_Write, p_flash->WriteSize) == true)
					{
						isSucess = true;
					}
				}
				else
				{
					isSucess = true;
				}
			}
		}
	}

	return isSucess;
}
