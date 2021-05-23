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
#ifndef HAL_FLASH_PLATFORM_H
#define HAL_FLASH_PLATFORM_H

#include "External/S32K142/include/S32K142.h"

#include <stdint.h>
#include <stdbool.h>

/*
	Chip Boundaries
	0000_0000	0003_FFFF	 	Program / code flash
	1000_0000	1000_FFFF	 	FlexNVM / code flash
	1400_0000	1400_0FFF		FlexRAM
 */

#define S32K_PROGRAM_FLASH_START	0x00000000	/* */
#define S32K_PROGRAM_FLASH_END		0x0003FFFF	/* */
#define S32K_PROGRAM_FLASH_SIZE		0x00040000

#define S32K_FLEX_NVM_START		0x10000000	/* */
#define S32K_FLEX_NVM_END		0x1000FFFF	/* */
#define S32K_FLEX_NVM_SIZE		0x00010000

#define S32K_FLEX_RAM_START		0x14000000	/* */
#define S32K_FLEX_RAM_END		0x14000FFF	/* */
#define S32K_FLEX_RAM_SIZE		0x00001000

#define S32K_FLASH_RESERVED_CONFIG_START	0x0400	/* */
#define S32K_FLASH_RESERVED_CONFIG_END		0x040F	/* */

#define S32K_FLASH_SECTOR_SIZE				0x00000800	/* 2KB */
#define S32K_FLASH_PHRASE_SIZE				0x00000008	/* 64-bits */

#define S32K_FLASH_EEPROM_UNIT_SIZE_ERASE	0x01
#define S32K_FLASH_EEPROM_UNIT_SIZE_WRITE	0x01

/*
 * User Defined
 *
 * all flex ram allocated as eeprom?
 */

#define HAL_FLASH_START				S32K_FLEX_NVM_START	/* */
#define HAL_FLASH_END				S32K_FLEX_NVM_END	/* */
#define HAL_FLASH_SIZE				S32K_FLEX_NVM_SIZE	/* */
#define HAL_FLASH_UNIT_SIZE_ERASE	S32K_FLASH_SECTOR_SIZE
#define HAL_FLASH_UNIT_SIZE_WRITE	S32K_FLASH_PHRASE_SIZE

#define HAL_FLASH_EEPROM_START					S32K_FLEX_RAM_START	/* */
#define HAL_FLASH_EEPROM_END					S32K_FLEX_RAM_END	/* */
#define HAL_FLASH_EEPROM_SIZE					S32K_FLEX_RAM_SIZE	/* */
#define HAL_FLASH_EEPROM_UNIT_SIZE_ERASE		0x04  /* User enforce 4 byte aligned */
#define HAL_FLASH_EEPROM_UNIT_SIZE_WRITE		0x04

/*
 *
 */
#define HAL_FLASH_USER_DATA_START			HAL_FLASH_EEPROM_START	/* */
#define HAL_FLASH_USER_DATA_END				HAL_FLASH_EEPROM_END	/* */
#define HAL_FLASH_USER_DATA_SIZE			HAL_FLASH_EEPROM_SIZE	/* */
#define HAL_FLASH_USER_DATA_SIZE_ERASE		HAL_FLASH_EEPROM_UNIT_SIZE_ERASE
#define HAL_FLASH_USER_DATA_SIZE_WRITE		HAL_FLASH_EEPROM_UNIT_SIZE_WRITE

/*
 * S32K  FLASH/EEPROM use same controller, same HAL
 */
//typedef struct
//{
//
//
//
//}HAL_Flash_T;
typedef FTFC_Type HAL_Flash_T;
typedef FTFC_Type HAL_Flash_EEPROM_T;

static inline void HAL_Flash_ClearError(HAL_Flash_T *p_hal_flash)
{

}

static inline void HAL_Flash_WriteCmdPrep(HAL_Flash_T *p_hal_flash)
{

}

static inline void HAL_Flash_WriteCmdLength(HAL_Flash_T *p_hal_flash)
{

} //FLASH_UNIT_SIZE_WRITE

static inline void HAL_Flash_WriteCmdDest(HAL_Flash_T *p_hal_flash, uint8_t *p_dest)
{

}

static inline void HAL_Flash_WriteCmdSource(HAL_Flash_T *p_hal_flash, uint8_t *p_source)
{

}

static inline void HAL_Flash_WriteCmdStart(HAL_Flash_T *p_hal_flash)
{

}

static inline bool HAL_Flash_ReadCompleteFlag(HAL_Flash_T *p_hal_flash)
{
//	FTFC->FERSTAT
}

static inline bool HAL_Flash_ReadErrorFlags(HAL_Flash_T *p_hal_flash)
{
//	FTFC->FERSTAT
}

static inline void HAL_Flash_ClearErrorFlags(HAL_Flash_T *p_hal_flash)
{

}

static inline void HAL_Flash_Prep(HAL_Flash_T *p_hal_flash)
{

}

static inline void HAL_Flash_Complete(HAL_Flash_T *p_hal_flash)
{

}


static inline bool HAL_Flash_EEPROM_ReadCompleteFlag(HAL_Flash_T *p_hal_flash)
{

}

static inline bool HAL_Flash_EEPROM_ReadErrorFlags(HAL_Flash_EEPROM_T *p_hal_flash)
{
	return HAL_Flash_ReadErrorFlags(p_hal_flash);  //FLASH/EEPROM use same controller, same HAL
}

static inline void HAL_Flash_EEPROM_ClearErrorFlags(HAL_Flash_T *p_hal_flash)
{

}

#endif /* HAL_FLASH_H */
