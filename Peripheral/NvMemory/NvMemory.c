
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
#include "NvMemory.h"

//#include "System/Queue/Queue.h"

#include <stdint.h>
#include <stdbool.h>

static inline NvMemory_Id_T GetParitionId(const NvMemory_T * p_nvMemory, const uint8_t * p_dest)
{
	NvMemory_Id_T id;

	//for each partition in flash, check dest
	for (uint8_t iPartition = 0; iPartition < 1U; iPartition++) //CONFIG_NV_MEMORY_PARTITION_COUNT
	{
		if (Flash_CheckBoundary(p_nvMemory->CONFIG.P_FLASH, p_dest) == true)
		{
			id = NV_MEMORY_ID_FLASH;
		}
		else if (EEPROM_CheckBoundary(p_nvMemory->CONFIG.P_EEPROM, p_dest) == true)
		{
			id = NV_MEMORY_ID_EEPROM;
		}
	}

	return id;
}

static inline bool CheckIsAligned(const NvMemory_T * p_nvMemory, const uint8_t * p_destFlash, size_t sizeBytes)
{
//	//p_destFlash%FLASH_UNIT_SIZE_WRITE == 0, unit size always power of 2
//	return (((uint32_t)p_destFlash & (NvMemory_UNIT_SIZE_WRITE - 1U)) == 0U) && (((uint32_t)sizeBytes & (NvMemory_UNIT_SIZE_WRITE - 1U)) == 0U);
}

static inline WriteAlignedBytes(NvMemory_T * p_nvMemory, uint8_t * p_destEeprom, const uint8_t * p_source)
{
//#if (NvMemory_UNIT_SIZE_WRITE == 4U)
//	((uint32_t*) p_destEeprom) = ((uint32_t*) p_source)
//#elif (NvMemory_UNIT_SIZE_WRITE == 2U)
//	((uint16_t*) p_destEeprom) = ((uint16_t*) p_source)
//#elif (NvMemory_UNIT_SIZE_WRITE == 1U)
//	((uint8_t*) p_destEeprom) = ((uint8_t*) p_source)
//#endif
}

bool NvMemory_StartWriteUnit(NvMemory_T * p_nvMemory, uint8_t * p_destEeprom, const uint8_t * p_source)
{
//	bool isSuccess = false;
//
//	if (CheckIsAligned(p_nvMemory, p_destEeprom, sizeBytes))
//    {
//		if (HAL_NvMemory_ReadCompleteFlag(p_nvMemory->CONFIG.P_HAL_NvMemory) != true)
//		{
//			switch (NvMemory_UNIT_SIZE_WRITE)
//			{
//				case 4U: ((volatile uint32_t*) p_destEeprom) = ((uint32_t*) p_source); break;
//				case 2U: ((volatile uint16_t*) p_destEeprom) = ((uint16_t*) p_source); break;
//				case 1U: ((volatile uint8_t*) p_destEeprom) = ((uint8_t*) p_source); break;
//				default: break;
//			}
//
//			if (HAL_NvMemory_ReadErrorFlags(p_nvMemory->CONFIG.P_HAL_NvMemory) == false)
//			{
//				isSuccess = true;
//
//			}
//		}
//    }
//
//    return isSuccess;
}

bool NvMemory_StartWriteBytes(NvMemory_T * p_nvMemory, uint8_t * p_destEeprom, const uint8_t * p_source, size_t sizeBytes)
{
//	bool isSuccess = false;
//
//	switch (NvMemory_UNIT_SIZE_WRITE)
//	{
//#if	((NvMemory_UNIT_SIZE_WRITE == 4U) || (NvMemory_UNIT_SIZE_WRITE == 2U) || (NvMemory_UNIT_SIZE_WRITE == 1U))
//		case 4U: ((volatile uint32_t*) p_destEeprom) = ((uint32_t*) p_source); break;
//#elif ((NvMemory_UNIT_SIZE_WRITE == 2U) || (NvMemory_UNIT_SIZE_WRITE == 1U))
//		case 2U: ((volatile uint16_t*) p_destEeprom) = ((uint16_t*) p_source); break;
//#elif (NvMemory_UNIT_SIZE_WRITE == 1U)
//		case 1U: ((volatile uint8_t*) p_destEeprom) = ((uint8_t*) p_source); break;
//#endif
//		default: break;
//	}
//    return isSuccess;
}

/*
 * reject if dest not aligned to min write
 */
bool NvMemory_WriteBytes_Blocking(NvMemory_T * p_nvMemory, uint8_t * p_destEeprom, const uint8_t * p_source, size_t sizeBytes)
{
//	bool isSuccess = false;
//
////    if ((p_destEeprom < HAL_NvMemory_START) || ((p_destEeprom + sizeBytes) > (HAL_NvMemory_START + HAL_NvMemory_SIZE)))
////    {
////    	isSuccess = false;
////    }
////    else
//
//	//#ifdefine CONFIG_NvMemory_WRITE_ALIGNED_ONLY
//
//	//align
//
//	if (CheckIsAligned(p_nvMemory, p_destEeprom, sizeBytes))
//
//    {
//    	for (uint32_t index = 0; index < sizeBytes; index += NvMemory_UNIT_SIZE_WRITE)
//    	{
//    		switch (NvMemory_UNIT_SIZE_WRITE)
//    		{
//			case 4U: ((volatile uint32_t*) p_destEeprom)[index] = ((uint32_t*) p_source)[index]; break;
//    		case 2U: ((volatile uint16_t*) p_destEeprom)[index] = ((uint16_t*) p_source)[index]; break;
//    		case 1U: ((volatile uint8_t*) p_destEeprom)[index] = ((uint8_t*) p_source)[index]; break;
//    		default: break;
//    		}
//
//			while (HAL_NvMemory_ReadCompleteFlag(p_nvMemory->CONFIG.P_HAL_NvMemory) != true)
//			{
//				if (p_nvMemory->Yield)
//				{
//					p_nvMemory->Yield(p_nvMemory->p_CallbackData);
//				}
//				if (HAL_NvMemory_ReadErrorFlags(p_nvMemory->CONFIG.P_HAL_NvMemory) == true)
//				{
//					break;
//				}
//			}
//
//			if (HAL_NvMemory_ReadErrorFlags(p_nvMemory->CONFIG.P_HAL_NvMemory) ==  false)
//			{
//				isSuccess = true;
//				break;
//			}
//    	}
//    }
//
//    return isSuccess;
}


//uint32_t BuildAlignedData(NvMemory_T * p_nvMemory,   uint8_t * p_destEeprom, const uint8_t * p_source, size_t sizeBytes)
//{
//
//}
//
////if dest not align with unit min, load then write
////align unaligned bytes
//bool NvMemory_WriteBytes_Blocking(NvMemory_T * p_nvMemory,   uint8_t * p_destEeprom, const uint8_t * p_source, size_t sizeBytes)
//{
//	uint8_t unitWrite[NvMemory_UNIT_SIZE_WRITE];
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
//    switch (NvMemory_UNIT_SIZE_WRITE)
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
bool NvMemory_OpenVirtual(NvMemory_T * p_nvMemory, const uint8_t * p_physical, size_t sizeBytes)
{

}

/*
 * dest of physical flash location
 */
bool NvMemory_WriteVirtual(NvMemory_T * p_nvMemory, const uint8_t * p_physical, const uint8_t * p_src, size_t sizeBytes)
{

}

bool NvMemory_ReadVirtual(NvMemory_T * p_nvMemory, uint8_t * p_dest, const uint8_t * p_physical, size_t sizeBytes)
{

}

//bool NvMemory_CloseVirtual(NvMemory_T * p_nvMemory)
//{
//	p_nvMemory->State = FLASH_STATE_WRITE;
//}

bool NvMemory_CloseVirtual_Blocking(NvMemory_T * p_nvMemory)
{

}


//uint8_t NvMemory_ReadBytes(NvMemory_T * p_nvMemory, uint8_t * p_dest, const uint8_t * p_sourceEeprom, uint32_t sizeBytes)
//{
//	for (uint32_t index = 0; index < sizeBytes; index++)
//	{
//		p_dest[index] = p_sourceEeprom[index];
//	}
//}
//
//uint8_t NvMemory_ReadAlignedBytes(NvMemory_T * p_nvMemory, uint8_t * p_dest, const uint8_t * p_sourceEeprom, uint32_t sizeBytes)
//{
//	for (uint32_t index = 0; index < sizeBytes; index += 4U)
//	{
//		((volatile uint32_t*)p_dest)[index] = ((uint32_t*)p_sourceEeprom)[index];
//	}
//}



