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
    @file
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef HAL_EEPROM_PLATFORM_H
#define HAL_EEPROM_PLATFORM_H

#include "HAL_Flash.h" /* Includes common defs */

typedef HAL_Flash_T HAL_EEPROM_T; /* Flash/EEPROM use same controller */

#if     defined(KE06Z4_SERIES) // No EEPROM on KE06

#define HAL_EEPROM_START                0U
#define HAL_EEPROM_END                  0U
#define HAL_EEPROM_SIZE                 0U
#define HAL_EEPROM_UNIT_WRITE_SIZE      0U

static inline void HAL_EEPROM_ClearErrorFlags(HAL_EEPROM_T * p_hal) { (void)p_hal; }
static inline bool HAL_EEPROM_ReadErrorFlags(const HAL_EEPROM_T * p_hal) { (void)p_hal; }
static inline bool HAL_EEPROM_ReadErrorProtectionFlag(const HAL_EEPROM_T * p_hal) { (void)p_hal; }
static inline bool HAL_EEPROM_ReadCompleteFlag(const HAL_EEPROM_T * p_hal) { (void)p_hal; }
static inline void HAL_EEPROM_StartCmdWriteUnit(HAL_EEPROM_T * p_hal, const uint8_t * p_dest, const uint8_t * p_data) { (void)p_hal; }
static inline bool HAL_EEPROM_ReadIsFirstTime(const HAL_EEPROM_T * p_hal) { (void)p_hal; }
static inline void HAL_EEPROM_Init_Blocking(HAL_EEPROM_T * p_hal) { (void)p_hal; }

#elif   defined(KE02Z4_SERIES)

#define HAL_EEPROM_START                0U
#define HAL_EEPROM_END                  0U
#define HAL_EEPROM_SIZE                 HAL_FLASH_UNIT_ERASE_SIZE
#define HAL_EEPROM_UNIT_WRITE_SIZE      HAL_FLASH_UNIT_WRITE_SIZE

static inline void HAL_EEPROM_ClearErrorFlags(HAL_EEPROM_T * p_hal) { HAL_Flash_ClearErrorFlags(p_hal); }
static inline bool HAL_EEPROM_ReadErrorFlags(const HAL_EEPROM_T * p_hal) { return HAL_Flash_ReadErrorFlags(p_hal); }
static inline bool HAL_EEPROM_ReadErrorProtectionFlag(const HAL_EEPROM_T * p_hal) { return HAL_Flash_ReadErrorProtectionFlag(p_hal); }
static inline bool HAL_EEPROM_ReadCompleteFlag(const HAL_EEPROM_T * p_hal) { return HAL_Flash_ReadCompleteFlag(p_hal); }
static inline void HAL_EEPROM_StartCmdWriteUnit(HAL_EEPROM_T * p_hal, const uint8_t * p_dest, const uint8_t * p_data) { HAL_Flash_StartCmdWritePage(p_hal, p_dest, p_data); }
static inline bool HAL_EEPROM_ReadIsFirstTime(const HAL_EEPROM_T * p_hal) { (void)p_hal; }
static inline void HAL_EEPROM_Init_Blocking(HAL_EEPROM_T * p_hal) { (void)p_hal; }
#endif



#endif
