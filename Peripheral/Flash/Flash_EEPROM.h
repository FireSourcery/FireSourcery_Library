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
#ifndef FLASH_EEPROM_H
#define FLASH_EEPROM_H

#include "Flash.h"

#include "HAL_Flash.h" //Shared file for address definitions

#define FLASH_EEPROM_START				HAL_FLASH_EEPROM_START
#define FLASH_EEPROM_SIZE				HAL_FLASH_EEPROM_SIZE
#define FLASH_EEPROM_UNIT_SIZE_WRITE 	HAL_FLASH_EEPROM_UNIT_SIZE_WRITE

//private
//static inline bool WriteFlashEepromAlignedUnit(Flash_T *p_flash, uint8_t *p_destEeprom, uint8_t *p_source)
//{
//	switch (FLASH_EEPROM_UNIT_SIZE_WRITE)
//	{
//	case 4U: ((volatile uint32_t*) p_destEeprom) = ((uint32_t*) p_source); break;
//	case 2U: ((volatile uint16_t*) p_destEeprom) = ((uint16_t*) p_source); break;
//	case 1U: ((volatile uint8_t*) p_destEeprom) = ((uint8_t*) p_source); break;
//	default: break;
//	}
//
//}

static inline bool Flash_EEPROM_WriteAlignedBytes_NonBlocking(Flash_T *p_flash, uint8_t *p_destEeprom, uint8_t *p_source, uint32_t sizeBytes)
{

}

/*
 * May write excess data. maintains dest data address correctly
 */
static inline bool Flash_EEPROM_WriteAlignedBytes_Blocking(Flash_T *p_flash, uint8_t *p_destEeprom, uint8_t *p_source, uint32_t sizeBytes)
{
	bool isSuccess = true;

//    if ((p_destEeprom < HAL_FLASH_EEPROM_START) || ((p_destEeprom + sizeBytes) > (HAL_FLASH_EEPROM_START + HAL_FLASH_EEPROM_SIZE)))
//    {
//    	isSuccess = false;
//    }
//    else

	//#ifdefine CONFIG_FLASH_WRITE_ALIGNED_ONLY (p_dest | 0x3 == p_source | 0x3)
//	if(p_destEeprom % FLASH_EEPROM_UNIT_SIZE_WRITE != 0U)
	if ((p_destEeprom & (FLASH_UNIT_SIZE_WRITE - 1U)) != 0U)
	{
		isSuccess = false;
	}
	else
    {
    	for (uint32_t index = 0; index < sizeBytes; index += FLASH_EEPROM_UNIT_SIZE_WRITE)
    	{
    		switch (FLASH_EEPROM_UNIT_SIZE_WRITE)
    		{
			case 4U: ((volatile uint32_t*) p_destEeprom)[index] = ((uint32_t*) p_source)[index]; break;
    		case 2U: ((volatile uint16_t*) p_destEeprom)[index] = ((uint16_t*) p_source)[index]; break;
    		case 1U: ((volatile uint8_t*) p_destEeprom)[index] = ((uint8_t*) p_source)[index]; break;
    		default: break;
    		}

			while (HAL_Flash_EEPROM_ReadCompleteFlag(p_flash->p_HAL_Flash) != true)
			{
				if (p_flash->Yield)
				{
					p_flash->Yield(p_flash->p_CallbackData);
				}
				if (HAL_Flash_EEPROM_ReadErrorFlags(p_flash->p_HAL_Flash) == true)
				{
					isSuccess = false;
					break;
				}
			}

			if (HAL_Flash_EEPROM_ReadErrorFlags(p_flash->p_HAL_Flash) == true)
			{
				isSuccess = false;
				break;
			}
    	}
    }

    return isSuccess;
}

static inline bool Flash_EEPROM_WriteAlignedBytes(Flash_T *p_flash, uint8_t *p_destEeprom, uint8_t *p_source, uint32_t sizeBytes)
{
#ifdef CONFIG_FLASH_WRITE_BLOCKING
	Flash_EEPROM_WriteAlignedBytes_Blocking();
#elif defined(CONFIG_FLASH_WRITE_NONBLOCKING)
	Flash_EEPROM_WriteAlignedBytes_NonBlocking();
#endif
}

//static inline bool  Flash_EEPROM_WriteUpdate_Blocking(Flash_T *p_flash, uint32_t *p_dest, uint32_t *p_source, uint32_t sizeBytes)
//{
//	for (uint32_t index = 0; index < sizeBytes; index += FLASH_EEPROM_UNIT_SIZE_WRITE)
//	{
//		if (p_dest[index] != p_source[index])
//		{
//			Flash_Write(p_flash, p_dest, p_source, 4U);
//		}
//	}
//}

static inline bool Flash_EEPROM_WriteRegion_Blocking(Flash_T *p_flash, Flash_Region_T *p_dest, uint8_t *p_sourcApp)
{

}

static inline bool Flash_EEPROM_ReadRegion_Blocking(Flash_T *p_flash, uint8_t *p_destApp, Flash_Region_T *p_sourc)
{

}

static inline uint8_t Flash_EEPROM_EraseUnit(Flash_T *p_flash, uint8_t *p_addressEeprom)
{

}

static inline uint8_t Flash_EEPROM_EraseUnits(Flash_T *p_flash, uint8_t *p_addressEeprom, uint32_t sizeUnits)
{

}

static inline uint8_t Flash_EEPROM_EraseBytes(Flash_T *p_flash, uint8_t *p_addressEeprom, uint32_t sizeBytes)
{

}

#endif /*   */

//unaligned writes

//		if (0U == (dest & 3U)
//		{
//			*(volatile uint32_t*) p_destEeprom = (uint32_t) (p_source[3]) << 24U | (uint32_t) (p_source[2]) << 16U | (uint32_t) (p_source[1]) << 8U | (uint32_t) (p_source[0]);
//		}
//		else ((0U == (dest & 1U))
//		{
//			*(volatile uint16_t*) p_destEeprom = (uint16_t) (p_source[1]) << 8U | (uint16_t) (p_source[0]);
//		}
//		else if (0x01U == writeszie)
//		{
//			*(uint8_t*) p_destEeprom = *p_source;
//		}
