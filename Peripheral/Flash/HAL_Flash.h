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
#ifndef HAL_FLASH_H
#define HAL_FLASH_H

#include "Path/Path.h"

#if 	defined(CONFIG_HAL_FLASH_USER_DEFINED) || defined(CONFIG_HAL_USER_DEFINED)
	#include PATH_USER(HAL_Flash.h)
#elif 	defined(CONFIG_HAL_FLASH_LIBRARY_DEFINED) || defined(CONFIG_HAL_LIBRARY_DEFINED)
	#include PATH_PLATFORM(Peripheral/HAL, HAL_Flash.h)
#else

#include <stdint.h>
#include <stdbool.h>

typedef void HAL_Flash_T;

//static inline void HAL_Flash_WriteCmdWritePage(HAL_Flash_T * p_hal_flash){}
//static inline void HAL_Flash_WriteCmdEraseSector(HAL_Flash_T * p_hal_flash){}
//static inline void HAL_Flash_WriteCmdVerifyEraseSector(HAL_Flash_T * p_hal_flash){}
//static inline void HAL_Flash_WriteCmdVerifyWritePage(HAL_Flash_T * p_hal_flash){}
//static inline void HAL_Flash_WriteCmdWriteOnce(HAL_Flash_T * p_hal_flash){}
//static inline void HAL_Flash_WriteCmdReadOnce(HAL_Flash_T * p_hal_flash){}

//static inline void HAL_Flash_WriteCmdWriteDest(HAL_Flash_T * p_hal_flash, const uint8_t * p_dest){}
//static inline void HAL_Flash_WriteCmdWriteData(HAL_Flash_T * p_hal_flash, const uint8_t * p_data){}
//static inline void HAL_Flash_WriteCmdWriteStart(HAL_Flash_T * p_hal_flash){}
//static inline bool HAL_Flash_ReadCompleteWriteFlag(HAL_Flash_T * p_hal_flash){}


//static inline void HAL_Flash_WriteCmdEraseDest(HAL_Flash_T * p_hal_flash, const uint8_t * p_dest){}
//static inline void HAL_Flash_WriteCmdEraseStart(HAL_Flash_T * p_hal_flash){}
//static inline bool HAL_Flash_ReadCompleteEraseFlag(HAL_Flash_T * p_hal_flash){}

//static inline void HAL_Flash_WriteCmdVerifyWriteDest(HAL_Flash_T * p_hal_flash, const uint8_t * p_dest){}
//static inline void HAL_Flash_WriteCmdVerifyWriteStart(HAL_Flash_T * p_hal_flash){}
//static inline bool HAL_Flash_ReadCompleteVerifyWriteFlag(HAL_Flash_T * p_hal_flash){}

//static inline void HAL_Flash_WriteCmdVerifyEraseDest(HAL_Flash_T * p_hal_flash, const uint8_t * p_dest){}
//static inline void HAL_Flash_WriteCmdVerifyEraseStart(HAL_Flash_T * p_hal_flash){}
//static inline bool HAL_Flash_ReadCompleteVerifyEraseFlag(HAL_Flash_T * p_hal_flash){}

//static inline void HAL_Flash_WriteCmdWriteOnceDest(HAL_Flash_T * p_hal_flash, const uint8_t * p_dest){}
//static inline void HAL_Flash_WriteCmdWriteOnceData(HAL_Flash_T *p_hal_flash, const uint8_t * p_data){}
//static inline void HAL_Flash_WriteCmdWriteOnceStart(HAL_Flash_T * p_hal_flash){}
//static inline bool HAL_Flash_ReadCompleteWriteOnceFlag(HAL_Flash_T * p_hal_flash){}

//static inline void HAL_Flash_WriteCmdReadOnceDest(HAL_Flash_T * p_hal_flash, const uint8_t * p_dest){}
//static inline void HAL_Flash_WriteCmdReadOnceStart(HAL_Flash_T * p_hal_flash){}
//static inline bool HAL_Flash_ReadCompleteReadOnceFlag(HAL_Flash_T * p_hal_flash){}
#include <stdint.h>
#include <stdbool.h>

static inline void __attribute__((weak)) HAL_Flash_StartCmdWritePage(HAL_Flash_T * p_hal_flash, const uint8_t * p_dest, const uint8_t * p_data) {}
static inline void __attribute__((weak)) HAL_Flash_StartCmdEraseSector(HAL_Flash_T * p_hal_flash, const uint8_t * p_dest){}
static inline void __attribute__((weak)) HAL_Flash_StartCmdVerifyWriteUnit(HAL_Flash_T * p_hal_flash, const uint8_t * p_dest){}
static inline void __attribute__((weak)) HAL_Flash_StartCmdVerifyEraseUnit(HAL_Flash_T * p_hal_flash, const uint8_t * p_dest){}
static inline void __attribute__((weak)) HAL_Flash_StartCmdVerifyEraseUnits(HAL_Flash_T * p_hal_flash, const uint8_t * p_dest, uint8_t units){}
static inline void __attribute__((weak)) HAL_Flash_StartCmdWriteOnce(HAL_Flash_T * p_hal_flash, const uint8_t * p_dest, const uint8_t * p_data){}
static inline void __attribute__((weak)) HAL_Flash_StartCmdReadOnce(HAL_Flash_T * p_hal_flash, const uint8_t * p_dest){}
static inline void __attribute__((weak)) HAL_Flash_ReadOnceData(HAL_Flash_T *p_hal_flash, uint8_t * p_result){}
__attribute__((weak)) static inline bool HAL_Flash_ReadErrorFlags(HAL_Flash_T *p_hal_flash){}
__attribute__((weak)) static inline void HAL_Flash_ClearErrorFlags(HAL_Flash_T *p_hal_flash){}
__attribute__((weak)) static inline bool HAL_Flash_ReadCompleteFlag(HAL_Flash_T * p_hal_flash){}
__attribute__((weak)) static inline bool HAL_Flash_ReadErrorVerifyFlag(HAL_Flash_T *p_hal_flash){}
#endif




#endif /* HAL_FLASH_H */
