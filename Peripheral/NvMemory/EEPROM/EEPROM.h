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
#ifndef EEPROM_H
#define EEPROM_H

#include "HAL_EEPROM.h"
#include "Peripheral/NvMemory/NvMemory/NvMemory.h"

#define EEPROM_START			HAL_EEPROM_START
#define EEPROM_SIZE				HAL_EEPROM_SIZE
#define EEPROM_UNIT_WRITE_SIZE 	HAL_EEPROM_UNIT_WRITE_SIZE

typedef NvMemory_Partition_T EEPROM_Partition_T;
typedef NvMemory_T EEPROM_T;

#define EEPROM_CONFIG(p_Hal, p_Partitions, PartitionCount, p_Buffer, BufferSize) \
	NV_MEMORY_CONFIG(p_Hal, HAL_EEPROM_ReadCompleteFlag, HAL_EEPROM_ReadErrorFlags, HAL_EEPROM_ClearErrorFlags, p_Partitions, PartitionCount, p_Buffer, BufferSize)														\

// extern bool EEPROM_ReadIsOpComplete(EEPROM_T * p_eeprom);
extern NvMemory_Status_T EEPROM_SetWrite		(EEPROM_T * p_eeprom, const void * p_dest, const void * p_source, size_t sizeBytes);

extern void EEPROM_Init_Blocking(EEPROM_T * p_eeprom);
extern NvMemory_Status_T EEPROM_Write_Blocking	(EEPROM_T * p_eeprom, const void * p_dest, const void * p_source, size_t sizeBytes);

// extern void EEPROM_Init_NonBlocking(EEPROM_T * p_eeprom);
extern bool EEPROM_ProcOp_NonBlocking(EEPROM_T * p_flash);
extern NvMemory_Status_T EEPROM_StartWrite_NonBlocking(EEPROM_T * p_eeprom, void * p_dest, const void * p_source, size_t sizeBytes);

#endif /*   */

