#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2026 FireSourcery

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
    @file   NvMemory_Map.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "NvMemory.h"

/* Config Mem descriptor table. */
// typedef const struct NvMemory_Entry
// {
//     const void * NVM_ADDRESS;     /* NVM/Flash address */
//     void * RAM_ADDRESS;           /* RAM address */
//     size_t SIZE;                  /* Size in bytes */
//     // const uint32_t checksum_seed;    /* For validation */
// }
// NvMemory_Entry_T;

// typedef const struct
// {
//     NvMemory_Entry_T * P_PARTITIONS; /* Config Partition decriptors */
//     size_t PARTITION_COUNT;        /* Number of NVM Partitions */
// }
// NvMemory_Map_T;

// NvMemory_Status_T NvMemory_SaveAll_Blocking(NvMemory_T * p_nvm, const NvMemory_Map_T * p_map)
// {
//     NvMemory_Status_T status;

//     /* Flash Erase Full block */
//     // status = Flash_Erase_Blocking(p_map->P_FLASH, p_map->MAIN_CONFIG_ADDRESS, p_map->MAIN_CONFIG_SIZE);
//     // if (status != NV_MEMORY_STATUS_SUCCESS) { return status; }

//     for (size_t i = 0U; i < p_map->PARTITION_COUNT; i++)
//     {
//         status = SaveEntry_Blocking(p_map, &p_map->P_PARTITIONS[i]);
//         if (status != NV_MEMORY_STATUS_SUCCESS) { break; }
//     }
//     //   Critical_EnableIrq();
//     return status;
// }

// void NvMemory_LoadAll(const NvMemory_Map_T * p_nvm, const NvMemory_Map_T * p_map)
// {
//     for (size_t i = 0U; i < p_map->PARTITION_COUNT; i++)
//     {
//         memcpy(p_map->P_PARTITIONS[i].RAM_ADDRESS, p_map->P_PARTITIONS[i].NVM_ADDRESS, p_map->P_PARTITIONS[i].SIZE);
//     }
// }