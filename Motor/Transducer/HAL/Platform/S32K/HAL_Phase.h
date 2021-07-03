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
	@file 	HAL_Phase.h
	@author FireSoucery
	@brief
	@version V0
*/
/******************************************************************************/
#ifndef HAL_PHASE_PLATFORM_H
#define HAL_PHASE_PLATFORM_H

#include "External/S32K142/include/S32K142.h"

#include <stdint.h>
#include <stdbool.h>

typedef const struct
{
	FTM_Type * P_FTM_A;
	uint8_t FTM_CHANNEL_A;
	GPIO_Type * P_GPIO_A;
	uint32_t GPIO_PIN_MASK_A;

	FTM_Type * P_FTM_B;
	uint8_t FTM_CHANNEL_B;
	GPIO_Type * P_GPIO_B;
	uint32_t GPIO_PIN_MASK_B;

	FTM_Type * P_FTM_C;
	uint8_t FTM_CHANNEL_C;
	GPIO_Type * P_GPIO_C;
	uint32_t GPIO_PIN_MASK_C;
} HAL_Phase_T;


#endif
