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
#ifndef HAL_HALL_BOARD_H
#define HAL_HALL_BOARD_H

//#include "../../Platform/S32K/HAL_Hall.h"

#include "External/S32K142/include/S32K142.h"
//#include "SDK/platform/devices/S32K142/include/S32K142.h"

#include <stdint.h>
#include <stdbool.h>


typedef const struct
{

} HAL_Hall_T;

/*
 * Only 1 set of hall sensor on KLS_S32K. Can use hard coded value
 */
static inline volatile uint8_t HAL_Hall_ReadSensors(const HAL_Hall_T * p_hall)
{
	(void)p_hall;

	/*
	 * 	PTE4 -> SA
	 * 	PTE5 -> SB
	 * 	PTE10 -> SC
	 */
//	return ((((PTE->PDIR >> 4)&0x01) | ((PTE->PDIR >> 4)&0x02) | ((PTE->PDIR >> 8)&0x04)));
	return (((PTE->PDIR >> 4) & 0b0011) | ((PTE->PDIR >> 8) & 0b0100));
}

static inline bool HAL_Hall_ReadSensorA(const HAL_Hall_T * p_hall)
{
	(void)p_hall;
	return (bool)(PTE->PDIR & ((uint32_t)1U << 4U));
}

static inline bool HAL_Hall_ReadSensorB(const HAL_Hall_T * p_hall)
{
	(void)p_hall;
	return (bool)(PTE->PDIR & ((uint32_t)1U << 5U));
}

static inline bool HAL_Hall_ReadSensorC(const HAL_Hall_T * p_hall)
{
	(void)p_hall;
	return (bool)(PTE->PDIR & ((uint32_t)1U << 10U));
}

#include "SDK/platform/drivers/inc/pins_driver.h"
#include "SDK/platform/drivers/inc/interrupt_manager.h"

static inline void HAL_Hall_Init(const HAL_Hall_T * p_hall)
{
	(void)p_hall;

	const pin_settings_config_t pin_mux_InitConfigArr0[3U] = {
		{
			.base            = PORTE,
			.pinPortIdx      = 4U,
			.pullConfig      = PORT_INTERNAL_PULL_NOT_ENABLED,
			.driveSelect     = PORT_LOW_DRIVE_STRENGTH,
			.passiveFilter   = false,
			.mux             = PORT_MUX_AS_GPIO,
			.pinLock         = false,
			.intConfig       = PORT_DMA_INT_DISABLED,
			.clearIntFlag    = false,
			.gpioBase        = PTE,
			.direction       = GPIO_INPUT_DIRECTION,
			.digitalFilter   = false,
			.initValue       = 0U,
		},
		{
			.base            = PORTE,
			.pinPortIdx      = 5U,
			.pullConfig      = PORT_INTERNAL_PULL_NOT_ENABLED,
			.driveSelect     = PORT_LOW_DRIVE_STRENGTH,
			.passiveFilter   = false,
			.mux             = PORT_MUX_AS_GPIO,
			.pinLock         = false,
			.intConfig       = PORT_DMA_INT_DISABLED,
			.clearIntFlag    = false,
			.gpioBase        = PTE,
			.direction       = GPIO_INPUT_DIRECTION,
			.digitalFilter   = false,
			.initValue       = 0U,
		},
		{
			.base            = PORTE,
			.pinPortIdx      = 10U,
			.pullConfig      = PORT_INTERNAL_PULL_NOT_ENABLED,
			.driveSelect     = PORT_LOW_DRIVE_STRENGTH,
			.passiveFilter   = false,
			.mux             = PORT_MUX_AS_GPIO,
			.pinLock         = false,
			.intConfig       = PORT_DMA_INT_DISABLED,
			.clearIntFlag    = false,
			.gpioBase        = PTE,
			.direction       = GPIO_INPUT_DIRECTION,
			.digitalFilter   = false,
			.initValue       = 0U,
		}
	};

	PINS_DRV_Init(3U, pin_mux_InitConfigArr0);
}


#endif
