/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

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
    @file   MemMapBoot.h
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef MEM_MAP_BOOT_H
#define MEM_MAP_BOOT_H

#define MEM_MAP_BOOT ((MemMapBoot_T *)CONFIG_MEM_MAP_BOOT_ADDRESS) /* User define externally */

/*
    Calling apps are compiled to the same platform, endianess should be consistent
*/

/* use 2 bits do differentiate unwritten memory patterns */
typedef enum MemMapBoot_IsValid_Tag
{
    MEM_MAP_BOOT_IS_VALID_01 = 0b01,
    MEM_MAP_BOOT_IS_VALID_10 = 0b10,
    MEM_MAP_BOOT_IS_INVALID_00 = 0b00,
    MEM_MAP_BOOT_IS_INVALID_11 = 0b11,
}
MemMapBoot_IsValid_T;

typedef union MemMapBoot_Tag
{
    struct
    {
        volatile uint32_t IsValid       : 2U;
        volatile uint32_t FastBoot      : 1U;
        volatile uint32_t Beep          : 1U;
        volatile uint32_t Blink         : 1U;
        volatile uint32_t LoadDefault   : 1U;
        volatile uint32_t ProtocolId    : 1U;
    };
    volatile uint32_t Register;
}
MemMapBoot_T;

static inline bool MemMapBoot_GetIsValid(void)
{
    return ((MEM_MAP_BOOT->IsValid == MEM_MAP_BOOT_IS_VALID_01) || (MEM_MAP_BOOT->IsValid == MEM_MAP_BOOT_IS_VALID_10));
}

/* Load Default by default */
static inline bool MemMapBoot_GetLoadDefault(void) { return ((MemMapBoot_GetIsValid() == false) || (MEM_MAP_BOOT->LoadDefault == 1U)); }

/* Beep by default */
static inline bool MemMapBoot_GetBeep(void) { return ((MemMapBoot_GetIsValid() == false) || (MEM_MAP_BOOT->Beep == 1U)); }

/* No FastBoot by default */
static inline bool MemMapBoot_GetFastBoot(void) { return ((MemMapBoot_GetIsValid() == true) && (MEM_MAP_BOOT->FastBoot == 1U)); }

#endif
