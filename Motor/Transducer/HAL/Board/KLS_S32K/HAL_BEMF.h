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
	@file 	HAL_Hall.h
	@author FireSoucery
	@brief
	@version V0
*/
/******************************************************************************/
#ifndef HAL_BEMF_BOARD_H
#define HAL_BEMF_BOARD_H

//#include "External/S32K142/include/S32K142.h"
#include "SDK/platform/devices/S32K142/include/S32K142.h"

#include "SDK/platform/drivers/inc/pins_driver.h"
#include "SDK/platform/drivers/inc/interrupt_manager.h"

#include <stdint.h>
#include <stdbool.h>

typedef const struct
{
//	uint32_t InstanceID;
} HAL_BEMF_T;

//
//
//
//
///// User app provides timer. Use to track commutation time, reset each commutation cycle
//static inline uint8_t HAL_BEMF_ReadTimer(const HAL_BEMF_T * p_hal_bemf)
//{
//	(void)p_hal_bemf;
//
//}
//
///// set timer time till next isr, reset timer.
//static inline void HAL_BEMF_StartTimerInterrupt(const HAL_BEMF_T * p_hal_bemf, uint32_t ticks)
//{
//	(void)p_hal_bemf;
//
//
//}
//
//// 16-bit timer, Freq = 50 kHz: Peroid = 20 uS, Overflow 1.3 seconds,  rpm min for 10 pole pairs
//static inline void HAL_BEMF_Init(const HAL_BEMF_T * p_hal_bemf)
//{
//	(void)p_hal_bemf;
//
//
//}


#endif
