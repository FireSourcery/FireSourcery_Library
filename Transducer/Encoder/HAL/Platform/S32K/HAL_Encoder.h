/******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSoucery / The Firebrand Forge Inc

	This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).

	This program is free software: you can redistribute it and/or modify
	it under the terupdateInterval of the GNU General Public License as published by
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
	@file 	HAL_Encoder.h
	@author FireSoucery
	@brief 	Encoder Timer Counter HAL for S32K
	@version V0
*/
/******************************************************************************/
#ifndef HAL_ENCODER_PLATFORM_H
#define HAL_ENCODER_PLATFORM_H

#include "External/S32K142/include/S32K142.h"

#include <stdint.h>
#include <stdbool.h>

typedef const struct
{
	FTM_Type * p_FtmBase;
	uint8_t FtmChannel;

	GPIO_Type * p_GpioBasePhaseA;
	uint32_t GpioPinMaskPhaseA;

	GPIO_Type * p_GpioBasePhaseB;
	uint32_t GpioPinMaskPhaseB;
} HAL_Encoder_T;


static inline uint32_t HAL_Encoder_ReadTimerCounter(const HAL_Encoder_T * p_encoder)
{
	return p_encoder->p_FtmBase->CONTROLS[p_encoder->FtmChannel].CnV;
}

static inline bool HAL_Encoder_ReadPhaseA(const HAL_Encoder_T * p_encoder)
{
	return p_encoder->p_GpioBasePhaseA->PDIR & p_encoder->GpioPinMaskPhaseA;
}

static inline bool HAL_Encoder_ReadPhaseB(const HAL_Encoder_T * p_encoder)
{
	return p_encoder->p_GpioBasePhaseB->PDIR & p_encoder->GpioPinMaskPhaseB;
}

static inline bool HAL_Encoder_ReadDirection(const HAL_Encoder_T * p_encoder)
{
	if((HAL_Encoder_ReadPhaseA(p_encoder) == true) && (HAL_Encoder_ReadPhaseB(p_encoder) == false))
	{
		return true;
	}
	else
	{
		return false;
	}
}

#endif
