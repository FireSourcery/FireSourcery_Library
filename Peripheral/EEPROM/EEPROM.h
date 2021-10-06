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

#include "System/Queue/Queue.h"

#define EEPROM_START			HAL_EEPROM_START
#define EEPROM_SIZE				HAL_EEPROM_SIZE
#define EEPROM_UNIT_SIZE_WRITE 	HAL_EEPROM_UNIT_SIZE_WRITE

typedef const struct
{
	uint8_t * P_ADDRESS;
	uint32_t SIZE;
//	uint8_t Alignment;
	//#define EEPROM_START				HAL_EEPROM_START	/* */
	//#define EEPROM_SIZE				HAL_EEPROM_SIZE	/* */
}
EEPROM_Partition_T;

typedef const struct
{
	HAL_EEPROM_T * P_HAL_EEPROM;	//eeprom controller registers
	uint8_t	UNIT_SIZE_WRITE;
//#ifdef CONFIG_EEPROM_NON_BLOCKING
	Queue_Config_T QUEUE;
//#endif
}
EEPROM_Config_T;

typedef enum
{
	EEPROM_STATUS_IDLE,
	EEPROM_STATUS_SUCESS,
	EEPROM_STATUS_ERROR,

}
EEPROM_Status_T;

//EEPROM controller
typedef struct
{
	EEPROM_Config_T CONFIG;

//#ifdef CONFIG_EEPROM_NON_BLOCKING
	Queue_T Queue; //write queue for nonblocking operation
	volatile EEPROM_Status_T Status;
	const uint8_t * volatile p_Write;
	volatile size_t WriteIndex; 		//in page/phrase
	volatile size_t WriteSize; 	//total bytes at start


	bool IsActive;
//#endif

	//Blocking
    void (* Yield)(void *);    		/*!<  On Block*/
//    void (* Callback)(void *);    	/*!<  Union*/

    void (* OnComplete)(void *);    /*!< OnComplete */
    void * p_CallbackData;
} EEPROM_T;


#endif /*   */

