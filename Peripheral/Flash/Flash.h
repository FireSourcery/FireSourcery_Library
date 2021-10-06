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
#ifndef FLASH_H
#define FLASH_H

#include "HAL_Flash.h"

#include "System/Queue/Queue.h"

#define FLASH_UNIT_SIZE_ERASE	HAL_FLASH_UNIT_SIZE_ERASE
#define FLASH_UNIT_SIZE_WRITE	HAL_FLASH_UNIT_SIZE_WRITE
#define FLASH_ALIGN_MASK		(HAL_FLASH_UNIT_SIZE_WRITE - 1U)	/* */

//#define QUEUE_INIT_BUFFER(flash) (flash.CONFIG.QUEUE.P_BUFFER = &flash.Buffer)

typedef const struct
{
	uint8_t * P_ADDRESS;
	uint32_t SIZE;
//	uint8_t Alignment;
	//#define FLASH_START				HAL_FLASH_START	/* */
	//#define FLASH_SIZE				HAL_FLASH_SIZE	/* */
}
Flash_Partition_T;

typedef const struct
{
	HAL_Flash_T * P_HAL_FLASH;	//flash controller registers


	Queue_Config_T QUEUE; //

	Flash_Partition_T PARTITIONS[1U]; //CONFIG_FLASH_PARTITION_COUNT
}
Flash_Config_T;

typedef enum
{
//	FLASH_STATUS_IDLE,
	FLASH_STATUS_SUCCESS,
	FLASH_STATUS_ERROR,
}
Flash_Status_T;


typedef enum
{
	FLASH_STATE_IDLE,
	FLASH_STATE_WRITE,
	FLASH_STATE_VERIFY,
}
Flash_State_T;

//Flash controller
typedef struct
{
	Flash_Config_T CONFIG;

	//for virtual write,
	//static allocate size, only be 1 instance of flash is expected
	uint8_t Buffer[512U]; //CONFIG_FLASH_BUFFER_SIZE

	bool IsVerifyEnable;
	bool IsForceAlignEnable;

	uint32_t CheckSum; //compare if checksum is set

	Queue_T Queue; //data queue for nonblocking operation

	//job quee
	volatile Flash_Status_T Status;
	volatile Flash_State_T State;

	const uint8_t * volatile p_Write;
	volatile size_t WriteIndex; 		//in page/phrase
	volatile size_t WriteSize; 	//total bytes at start


	//Blocking
    void (* Yield)(void *);    		/*!<  On Block*/
//    void (* Callback)(void *);    	/*!<  Union*/

    void (* OnComplete)(void *);    /*!< OnComplete */
    void * p_CallbackData;
} Flash_T;



#endif /* FLASH_H */

