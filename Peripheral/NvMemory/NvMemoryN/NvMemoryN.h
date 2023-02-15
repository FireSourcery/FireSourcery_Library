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
#ifndef NV_MEMORY_N_H
#define NV_MEMORY_N_H


#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

//#include "System/Queue/Queue.h"
//#include "Peripheral/EEPROM/EEPROM.h"
//#include "Peripheral/Flash/Flash.h"


typedef enum
{
    NV_MEMORY_ID_FLASH,
    NV_MEMORY_ID_EEPROM,
//    NV_MEMORY__ONCE,
}
NvMemoryN_Id_T;

//static inline Memory_Id_T GetParitionId(const Memory_T * p_nvMemory, const uint8_t * p_dest)
//{
//    Memory_Id_T id;
//
//    //for each partition in flash, check dest
//    for (uint8_t iPartition = 0; iPartition < 1U; iPartition++) //CONFIG_NV_MEMORY_PARTITION_COUNT
//    {
//        if (Flash_CheckBoundary(p_nvMemory->CONFIG.P_FLASH, p_dest) == true)
//        {
//            id = NV_MEMORY_ID_FLASH;
//        }
//        else if (EEPROM_CheckBoundary(p_nvMemory->CONFIG.P_EEPROM, p_dest) == true)
//        {
//            id = NV_MEMORY_ID_EEPROM;
//        }
//    }
//
//    return id;
//}

#endif /* NV_MEMORY_H */

