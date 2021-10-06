/*******************************************************************************/
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
/*******************************************************************************/
/*******************************************************************************/
/*!
    @file
    @author FireSoucery
    @brief
    @version V0
*/
/*******************************************************************************/

#include "EEPROM.h"

#include "HAL_EEPROM.h" //Shared file for address definitions

#include "System/Queue/Queue.h"

#include <stdint.h>
#include <stdbool.h>




static inline bool CheckIsAligned(const EEPROM_T * p_eeprom, const uint8_t * p_destFlash, size_t sizeBytes)
{
	//p_destFlash%FLASH_UNIT_SIZE_WRITE == 0, unit size always power of 2
	return (((uint32_t)p_destFlash & (EEPROM_UNIT_SIZE_WRITE - 1U)) == 0U) && (((uint32_t)sizeBytes & (EEPROM_UNIT_SIZE_WRITE - 1U)) == 0U);
}

static inline WriteAlignedBytes(EEPROM_T * p_eeprom, uint8_t * p_destEeprom, const uint8_t * p_source)
{
#if (EEPROM_UNIT_SIZE_WRITE == 4U)
	(*(uint32_t*)p_destEeprom) = (*(uint32_t*)p_source);
#elif (EEPROM_UNIT_SIZE_WRITE == 2U)
	(*(uint16_t*)p_destEeprom) = (*(uint16_t*)p_source);
#elif (EEPROM_UNIT_SIZE_WRITE == 1U)
	(*(uint8_t*)p_destEeprom) = (*(uint8_t*)p_source);
#endif
}


 bool EEPROM_CheckBoundary(const EEPROM_T * p_eeprom, const uint8_t * p_dest)
{
	//for each partition in flash, check dest
}


bool EEPROM_StartWriteUnit(EEPROM_T * p_eeprom, uint8_t * p_destEeprom, const uint8_t * p_source)
{
//	bool isSuccess = false;
//
//	if (CheckIsAligned(p_eeprom, p_destEeprom))
//    {
//		if (HAL_EEPROM_ReadCompleteFlag(p_eeprom->CONFIG.P_HAL_EEPROM) != true)
//		{
//			switch (EEPROM_UNIT_SIZE_WRITE)
//			{
//				case 4U: *((volatile uint32_t*) p_destEeprom) = ((uint32_t*) p_source); break;
//				case 2U: *((volatile uint16_t*) p_destEeprom) = ((uint16_t*) p_source); break;
//				case 1U: *((volatile uint8_t*) p_destEeprom) = ((uint8_t*) p_source); break;
//				default: break;
//			}
//
//			if (HAL_EEPROM_ReadErrorFlags(p_eeprom->CONFIG.P_HAL_EEPROM) == false)
//			{
//				isSuccess = true;
//
//			}
//		}
//    }
//
//    return isSuccess;
}

bool EEPROM_StartWriteBytes(EEPROM_T * p_eeprom, uint8_t * p_destEeprom, const uint8_t * p_source, size_t sizeBytes)
{
	bool isSuccess = false;

//	switch (EEPROM_UNIT_SIZE_WRITE)
//	{
//#if	((EEPROM_UNIT_SIZE_WRITE == 4U) || (EEPROM_UNIT_SIZE_WRITE == 2U) || (EEPROM_UNIT_SIZE_WRITE == 1U))
//		case 4U: (*(volatile uint32_t*) p_destEeprom) = ((uint32_t*) p_source); break;
//#elif ((EEPROM_UNIT_SIZE_WRITE == 2U) || (EEPROM_UNIT_SIZE_WRITE == 1U))
//		case 2U: (*(volatile uint16_t*) p_destEeprom) = ((uint16_t*) p_source); break;
//#elif (EEPROM_UNIT_SIZE_WRITE == 1U)
//		case 1U: (*(volatile uint8_t*) p_destEeprom) = ((uint8_t*) p_source); break;
//#endif
//		default: break;
//	}
    return isSuccess;
}

/*
 * reject if dest not aligned to min write
 */
bool EEPROM_WriteAlignedBytes_Blocking(EEPROM_T * p_eeprom, uint8_t * p_destEeprom, const uint8_t * p_source, size_t sizeBytes)
{
	bool isSuccess = false;

//    if ((p_destEeprom < HAL_EEPROM_START) || ((p_destEeprom + sizeBytes) > (HAL_EEPROM_START + HAL_EEPROM_SIZE)))
//    {
//    	isSuccess = false;
//    }
//    else

	//#ifdefine CONFIG_EEPROM_WRITE_ALIGNED_ONLY

	//align

	if (CheckIsAligned(p_eeprom, p_destEeprom, sizeBytes))

    {
    	for (uint32_t index = 0; index < sizeBytes; index += EEPROM_UNIT_SIZE_WRITE)
    	{
    		switch (EEPROM_UNIT_SIZE_WRITE)
    		{
			case 4U: ((volatile uint32_t*) p_destEeprom)[index] = ((uint32_t*) p_source)[index]; break;
    		case 2U: ((volatile uint16_t*) p_destEeprom)[index] = ((uint16_t*) p_source)[index]; break;
    		case 1U: ((volatile uint8_t*) p_destEeprom)[index] = ((uint8_t*) p_source)[index]; break;
    		default: break;
    		}

			while (HAL_EEPROM_ReadCompleteFlag(p_eeprom->CONFIG.P_HAL_EEPROM) != true)
			{
				if (p_eeprom->Yield)
				{
					p_eeprom->Yield(p_eeprom->p_CallbackData);
				}
				if (HAL_EEPROM_ReadErrorFlags(p_eeprom->CONFIG.P_HAL_EEPROM) == true)
				{
					break;
				}
			}

			if (HAL_EEPROM_ReadErrorFlags(p_eeprom->CONFIG.P_HAL_EEPROM) ==  false)
			{
				isSuccess = true;
				break;
			}
    	}
    }

    return isSuccess;
}


//uint32_t BuildAlignedData(EEPROM_T * p_eeprom,   uint8_t * p_destEeprom, const uint8_t * p_source, size_t sizeBytes)
//{
//
//}
//
////if dest not align with unit min, load then write
////align unaligned bytes
//bool EEPROM_WriteBytes_Blocking(EEPROM_T * p_eeprom,   uint8_t * p_destEeprom, const uint8_t * p_source, size_t sizeBytes)
//{
//	uint8_t unitWrite[EEPROM_UNIT_SIZE_WRITE];
//
//	uint8_t align;
//
//    if ((p_destEeprom & 3U) == 0U)
//    {
//    	align = 4U;
//    }
//    else if ((p_destEeprom & 1U) == 0U)
//    {
//    	align = 2U;
//    }
//    else
//    {
//    	align = 1U;
//    }
//
//    switch (EEPROM_UNIT_SIZE_WRITE)
//    {
//		switch (align)
//		{
//			case 4U: ((  uint32_t*) (p_destEeprom)) = ((uint32_t*) p_source); break;
//			case 2U: ((  uint32_t*) (p_destEeprom)) = ((uint16_t*) p_source); break;
//			case 1U: ((  uint32_t*) (p_destEeprom)) = ((uint8_t*) p_source); break;
//			default: break;
//		}
//    }
//
//	switch (sizeBytes)
//	{
//	case 4U: ((  uint32_t*) p_destEeprom) = ((uint32_t*) p_source); break;
//	case 2U: ((  uint16_t*) p_destEeprom) = ((uint16_t*) p_source); break;
//	case 1U: ((  uint8_t*) p_destEeprom) = ((uint8_t*) p_source); break;
//	default: break;
//	}
//
//	//unaligned writes
//
//
//    if (0x04U == step)
//    {
//        temp =  (uint32_t)(pData[3]) << 24U;
//        temp |= (uint32_t)(pData[2]) << 16U;
//        temp |= (uint32_t)(pData[1]) << 8U;
//        temp |= (uint32_t)(pData[0]);
//        *(volatile uint32_t *)dest = temp;
//    }
//    if (0x01U == step)
//    {
//        *(uint8_t *)dest = *pData;
//    }
//    if (0x02U == step)
//    {
//        temp = (uint32_t)(pData[1]) << 8U;
//        temp |= (uint32_t)(pData[0]);
//        *(volatile uint16_t *)dest = (uint16_t)temp;
//    }
//}




/*
 * creates copy of flash in buffer
 */
bool EEPROM_OpenVirtual(EEPROM_T * p_eeprom, const uint8_t * p_physical, size_t sizeBytes)
{
	Queue_Clear(&p_eeprom->Queue);

	if(Queue_EnqueueN(&p_eeprom->Queue, p_physical, sizeBytes))
	{
		p_eeprom->p_Write = p_eeprom;
		p_eeprom->WriteIndex = 0U;
		p_eeprom->WriteSize = sizeBytes;
	}
}

/*
 * dest of physical flash location
 */
bool EEPROM_WriteVirtual(EEPROM_T * p_eeprom, const uint8_t * p_physical, const uint8_t * p_src, size_t sizeBytes)
{
	uint32_t offset;

	if (p_physical >= p_eeprom->p_Write && p_physical < p_eeprom->p_Write + p_eeprom->WriteSize)
	{
		offset = p_physical - p_eeprom->p_Write;
		memcpy(&p_eeprom->CONFIG.QUEUE.P_BUFFER[offset], p_src, sizeBytes);
	}
}

bool EEPROM_ReadVirtual(EEPROM_T * p_eeprom, uint8_t * p_dest, const uint8_t * p_physical, size_t sizeBytes)
{
	uint32_t offset;

	if (p_physical >= p_eeprom->p_Write && p_physical < p_eeprom->p_Write + p_eeprom->WriteSize)
	{
		offset = p_physical - p_eeprom->p_Write;
		memcpy(p_dest, &p_eeprom->CONFIG.QUEUE.P_BUFFER[offset], sizeBytes);
	}
}

//bool EEPROM_CloseVirtual(EEPROM_T * p_eeprom)
//{
//	p_eeprom->State = FLASH_STATE_WRITE;
//}

bool EEPROM_CloseVirtual_Blocking(EEPROM_T * p_eeprom)
{
	bool isSucess = false;

	if(EEPROM_WriteAlignedBytes_Blocking(p_eeprom, p_eeprom->p_Write, p_eeprom->CONFIG.QUEUE.P_BUFFER, p_eeprom->WriteSize) == true)
	{
//		if (EEPROM_GetCheckSumFlash(p_eeprom) == EEPROM_GetCheckSumBuffer(p_eeprom))
//		{
				isSucess = true;
//		}
	}

	return isSucess;
}


//uint8_t EEPROM_ReadBytes(EEPROM_T * p_eeprom, uint8_t * p_dest, const uint8_t * p_sourceEeprom, uint32_t sizeBytes)
//{
//	for (uint32_t index = 0; index < sizeBytes; index++)
//	{
//		p_dest[index] = p_sourceEeprom[index];
//	}
//}
//
//uint8_t EEPROM_ReadAlignedBytes(EEPROM_T * p_eeprom, uint8_t * p_dest, const uint8_t * p_sourceEeprom, uint32_t sizeBytes)
//{
//	for (uint32_t index = 0; index < sizeBytes; index += 4U)
//	{
//		((volatile uint32_t*)p_dest)[index] = ((uint32_t*)p_sourceEeprom)[index];
//	}
//}



