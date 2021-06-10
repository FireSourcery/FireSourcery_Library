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
#ifndef FLASH_H
#define FLASH_H


#include "HAL_Flash.h"

#define FLASH_START				HAL_FLASH_START	/* */
#define FLASH_SIZE				HAL_FLASH_SIZE	/* */

#define FLASH_UNIT_SIZE_ERASE	HAL_FLASH_UNIT_SIZE_ERASE
#define FLASH_UNIT_SIZE_WRITE	HAL_FLASH_UNIT_SIZE_WRITE

//Flash controller
typedef struct
{
	HAL_Flash_T * p_HAL_Flash;	//flash controller registers

    void (* Callback)(void *);    	/*!<  Union*/
    void (* Yield)(void *);    		/*!<  On Block*/
    void (* OnComplete)(void *);    /*!< OnComplete */

    void * p_CallbackData;
}Flash_T;

typedef const struct
{
	uint8_t * p_Address;
	uint32_t Size;
//	uint8_t Alignment;
}
Flash_Partition_T;


//little endian
typedef union
{
	uint32_t UInt32;
	struct
	{
		uint16_t UInt16_Low;
		uint16_t UInt16_High;
	};
	struct
	{
		uint8_t UInt8_Low;
		uint8_t UInt8_LowHigh;
		uint8_t UInt8_HighLow;
		uint8_t UInt8_High;
	};
} Interger32_T;
//else big endian
//typedef union
//{
//	uint32_t UInt32;
//	struct
//	{
//		uint16_t UInt16_High;
//		uint16_t UInt16_Low;
//	};
//	struct
//	{
//		uint8_t UInt8_High;
//		uint8_t UInt8_HighLow;
//		uint8_t UInt8_LowHigh;
//		uint8_t UInt8_Low;
//	};
//} Interger32_T;
////


//static inline void FlashWriteUnit(Flash_T *p_flash, uint8_t *p_destFlash, uint8_t *p_source)
//{
//	HAL_Flash_ClearErrorFlags(p_flash->p_HAL_Flash);
//	HAL_Flash_WriteCmdPrep(p_flash->p_HAL_Flash);
//	HAL_Flash_WriteCmdLength(p_flash->p_HAL_Flash); //FLASH_UNIT_SIZE_WRITE
//	HAL_Flash_WriteCmdDest(p_flash->p_HAL_Flash, &p_destFlash);
//	HAL_Flash_WriteCmdSource(p_flash->p_HAL_Flash, &p_source);
//	HAL_Flash_WriteCmdStart(p_flash->p_HAL_Flash);
//}

static inline uint8_t Flash_Write_NonBlocking(Flash_T *p_flash, uint8_t *p_destFlash, uint8_t *p_source, uint16_t sizeBytes)
{

}

static inline uint8_t Flash_WriteBytes_Blocking(Flash_T *p_flash, uint8_t *p_destFlash, uint8_t *p_source, uint16_t sizeBytes)
{
	bool isSuccess = true;

	if ((((uint32_t)p_destFlash & (FLASH_UNIT_SIZE_WRITE - 1U)) != 0U) || (((uint32_t)sizeBytes & (FLASH_UNIT_SIZE_WRITE - 1U)) != 0U)) //p_destFlash%FLASH_UNIT_SIZE_WRITE == 0
	{
		isSuccess = false;
	}
	else
	{

		for (uint32_t index = 0; index < sizeBytes; index += FLASH_UNIT_SIZE_WRITE)
		{

			if (HAL_Flash_ReadCompleteFlag(p_flash->p_HAL_Flash) == false)
			{
				isSuccess = false;
				break;
			}
			else
			{
				HAL_Flash_ClearErrorFlags(p_flash->p_HAL_Flash);

				HAL_Flash_Prep(p_flash->p_HAL_Flash); //Chip unique procedures

				HAL_Flash_WriteCmdLength(p_flash->p_HAL_Flash); //FLASH_UNIT_SIZE_WRITE
				HAL_Flash_WriteCmdDest(p_flash->p_HAL_Flash, &p_destFlash[index]);
				HAL_Flash_WriteCmdSource(p_flash->p_HAL_Flash, &p_source[index]);
				HAL_Flash_WriteCmdStart(p_flash->p_HAL_Flash);

				while (HAL_Flash_ReadCompleteFlag(p_flash->p_HAL_Flash) != true)
				{
					if (p_flash->Yield)
					{
						p_flash->Yield(p_flash->p_CallbackData);
					}
					if (HAL_Flash_ReadErrorFlags(p_flash->p_HAL_Flash) == true)
					{
						isSuccess = false;
						break;
					}
				}

				HAL_Flash_ReadCompleteFlag(p_flash->p_HAL_Flash); //Chip unique procedures

				if (HAL_Flash_ReadErrorFlags(p_flash->p_HAL_Flash) == true)
				{
					isSuccess = false;
					break;
				}
			}
		}
    }

    return isSuccess;
}

static inline uint8_t Flash_EraseUnit(Flash_T *p_flash, uint8_t *p_addressFlash)
{

}

static inline uint8_t Flash_EraseUnits(Flash_T *p_flash, uint8_t *p_addressFlash, uint32_t sizeUnits)
{

}

//round up or down to nearest unit or return error
static inline uint8_t Flash_EraseBytes(Flash_T *p_flash, uint8_t *p_addressFlash, uint32_t sizeBytes)
{

}

static inline void  Flash_Init
(
	Flash_T * p_flash,
	HAL_Flash_T * p_hal_flash,
	void (* flashCallback)(void *),
	void * p_callbackData
)
{
	p_flash->p_HAL_Flash = p_hal_flash;
	p_flash->Callback = flashCallback;
	p_flash->p_CallbackData = p_callbackData;
}


#endif /* FLASH_H */

