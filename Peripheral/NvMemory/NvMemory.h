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
#ifndef NV_MEMORY_H
#define NV_MEMORY_H

#include "System/Queue/Queue.h"
#include "Peripheral/EEPROM/EEPROM.h"
#include "Peripheral/Flash/Flash.h"

//id address
typedef enum
{
	NV_MEMORY_ID_FLASH,
	NV_MEMORY_ID_EEPROM,
//	NV_MEMORY__ONCE,
}
NvMemory_Id_T;

typedef const struct
{
// 	uint8_t * P_ADDRESS;
//	uint32_t SIZE;
 	void * P_PARTITION; //flash/eeprom
//	NvMemory_Id_T ID;
}
NvMemory_Partition_T;

typedef const struct
{
	Flash_T * P_FLASH;
	EEPROM_T * P_EEPROM;

	NvMemory_Partition_T PARTITION_TABLE[1U]; //CONFIG_NV_MEMORY_PARTITION_COUNT
}
NvMemory_Config_T;

typedef enum
{
//	NV_MEMORY_STATUS_IDLE,
	NV_MEMORY_STATUS_SUCCESS,
	NV_MEMORY_STATUS_ERROR,
}
NvMemory_Status_T;

//
//typedef enum
//{
//	NV_MEMORY_STATE_IDLE,
//	NV_MEMORY_STATE_WRITE,
//	NV_MEMORY_STATE_VERIFY,
//}
//NvMemory_State_T;

typedef struct
{
	NvMemory_Config_T CONFIG;

	volatile NvMemory_Status_T Status;
//	volatile NvMemory_State_T State;

} NvMemory_T;


#endif /* NV_MEMORY_H */

