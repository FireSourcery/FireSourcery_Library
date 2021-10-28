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
#ifndef MEMORY_H
#define MEMORY_H


#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

//#include "System/Queue/Queue.h"
//#include "Peripheral/EEPROM/EEPROM.h"
//#include "Peripheral/Flash/Flash.h"

////id address
//typedef enum
//{
//	NV_MEMORY_ID_FLASH,
//	NV_MEMORY_ID_EEPROM,
////	NV_MEMORY__ONCE,
//}
//Memory_Id_T;

//typedef const struct
//{
//// 	uint8_t * P_ADDRESS;
////	uint32_t SIZE;
//// 	void * P_PARTITION; //flash/eeprom
////	Memory_Id_T ID;
//}
//Memory_Partition_T;

//typedef const struct
//{
////	Flash_T * P_FLASH;
////	EEPROM_T * P_EEPROM;
//
//	Memory_Partition_T PARTITION_TABLE[1U]; //CONFIG_NV_MEMORY_PARTITION_COUNT
//}
//Memory_Config_T;

typedef enum
{
//	NV_MEMORY_STATUS_IDLE,
	NV_MEMORY_STATUS_SUCCESS,
	NV_MEMORY_STATUS_ERROR,
}
Memory_Status_T;

//
//typedef enum
//{
//	NV_MEMORY_STATE_IDLE,
//	NV_MEMORY_STATE_WRITE,
//	NV_MEMORY_STATE_VERIFY,
//}
//Memory_State_T;

typedef struct
{
//	Memory_Config_T CONFIG;

//	volatile Memory_Status_T Status;
//	volatile Memory_State_T State;
	void * p_Context;
	Memory_Status_T (*Init)		(void * p_context);
	Memory_Status_T (*Write)	(void * p_context, uint8_t * p_destMemory, const uint8_t * p_source, size_t sizeBytes);
	Memory_Status_T (*Erase)	(void * p_context, uint8_t * p_destMemory, size_t sizeBytes);
	Memory_Status_T (*EraseAll)	(void * p_context);
//	Memory_Status_T (*Config)	(void * p_context, void * p_buffer);
//	Memory_Status_T (*Fill)		(void * p_context, uint8_t * p_destMemory, const uint8_t * p_source, size_t sizeBytes);
//	Memory_Status_T (*Flush)	(void * p_context);
//	Memory_Status_T (*Read)		(void * p_context, uint8_t * p_destBuffer, const uint8_t * p_source, size_t sizeBytes);
} Memory_T;

//static inline Memory_Id_T GetParitionId(const Memory_T * p_nvMemory, const uint8_t * p_dest)
//{
//	Memory_Id_T id;
//
//	//for each partition in flash, check dest
//	for (uint8_t iPartition = 0; iPartition < 1U; iPartition++) //CONFIG_NV_MEMORY_PARTITION_COUNT
//	{
//		if (Flash_CheckBoundary(p_nvMemory->CONFIG.P_FLASH, p_dest) == true)
//		{
//			id = NV_MEMORY_ID_FLASH;
//		}
//		else if (EEPROM_CheckBoundary(p_nvMemory->CONFIG.P_EEPROM, p_dest) == true)
//		{
//			id = NV_MEMORY_ID_EEPROM;
//		}
//	}
//
//	return id;
//}




bool Memory_StartWriteBytes(Memory_T * p_nvMemory, uint8_t * p_destMemory, const uint8_t * p_source, size_t sizeBytes)
{

}

/*
 * reject if dest not aligned to min write
 */
bool Memory_WriteBytes_Blocking(Memory_T * p_nvMemory, uint8_t * p_destEeprom, const uint8_t * p_source, size_t sizeBytes)
{

}


//uint32_t BuildAlignedData(Memory_T * p_nvMemory,   uint8_t * p_destEeprom, const uint8_t * p_source, size_t sizeBytes)
//{
//
//}
//
////if dest not align with unit min, load then write
////align unaligned bytes
//bool Memory_WriteBytes_Blocking(Memory_T * p_nvMemory,   uint8_t * p_destEeprom, const uint8_t * p_source, size_t sizeBytes)
//{

//}




/*
 * creates copy of flash in buffer
 */
bool Memory_OpenVirtual(Memory_T * p_nvMemory, const uint8_t * p_physical, size_t sizeBytes)
{

}

/*
 * dest of physical flash location
 */
bool Memory_WriteVirtual(Memory_T * p_nvMemory, const uint8_t * p_physical, const uint8_t * p_src, size_t sizeBytes)
{

}

bool Memory_ReadVirtual(Memory_T * p_nvMemory, uint8_t * p_dest, const uint8_t * p_physical, size_t sizeBytes)
{

}

//bool Memory_CloseVirtual(Memory_T * p_nvMemory)
//{
//	p_nvMemory->State = FLASH_STATE_WRITE;
//}

bool Memory_CloseVirtual_Blocking(Memory_T * p_nvMemory)
{

}


//uint8_t Memory_ReadBytes(Memory_T * p_nvMemory, uint8_t * p_dest, const uint8_t * p_sourceEeprom, uint32_t sizeBytes)
//{

//}
//
//uint8_t Memory_ReadAlignedBytes(Memory_T * p_nvMemory, uint8_t * p_dest, const uint8_t * p_sourceEeprom, uint32_t sizeBytes)
//{

//}

#endif /* NV_MEMORY_H */

