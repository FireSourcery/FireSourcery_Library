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
    @file   BootRef.h
    @author FireSourcery
    @brief

*/
/******************************************************************************/
#ifndef BOOT_REF_H
#define BOOT_REF_H

#include <stdint.h>
#include <stdbool.h>

/*
    Calling apps are compiled to the same platform, endianess should be consistent
*/

/* use 2 bits do differentiate unwritten memory patterns */
/* IsFirstTime check for first time set */
typedef enum BootRef_IsValid
{
    BOOT_REF_IS_INVALID_00 = 0b00,
    BOOT_REF_IS_VALID_01 = 0b01,
    BOOT_REF_IS_VALID_10 = 0b10,
    BOOT_REF_IS_INVALID_11 = 0b11,
}
BootRef_IsValid_T;

typedef union BootRef
{
    struct
    {
        volatile uint32_t IsValid       : 2U;
        volatile uint32_t FastBoot      : 1U;
        volatile uint32_t Beep          : 1U;
        volatile uint32_t Blink         : 1U;
        // volatile uint32_t LoadDefault   : 1U;
        volatile uint32_t ProtocolId    : 2U;
    };
    volatile uint32_t Word;
}
BootRef_T;

#ifndef BOOT_REF_ADDRESS
    #error "BOOT_REF_ADDRESS not defined"
#endif

/*
    User define externally
    Align to minium NvM write size
*/
#define BOOT_REF ((BootRef_T *)BOOT_REF_ADDRESS)

/* !IsFirstTime */
static inline bool BootRef_IsValid(void)
{
    return ((BOOT_REF->IsValid == BOOT_REF_IS_VALID_01) || (BOOT_REF->IsValid == BOOT_REF_IS_VALID_10));
}

/* No FastBoot by default */
static inline bool BootRef_IsFastBoot(void) { return ((BootRef_IsValid() == true) && (BOOT_REF->FastBoot == 1U)); }

/* Beep by default */
static inline bool BootRef_IsBeep(void) { return ((BootRef_IsValid() == false) || (BOOT_REF->Beep == 1U)); }

/* Load Default by default */
// static inline bool BootRef_IsLoadDefault(void) { return ((BootRef_IsValid() == false) || (BOOT_REF->LoadDefault == 1U)); }


#endif
