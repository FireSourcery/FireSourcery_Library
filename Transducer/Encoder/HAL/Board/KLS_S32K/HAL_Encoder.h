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
	@file 	HAL_Encoder.h
	@author FireSoucery
	@brief 	Encoder Timer Counter HAL for S32K
	@version V0
*/
/******************************************************************************/
#ifndef HAL_ENCODER_BOARD_H
#define HAL_ENCODER_BOARD_H

#include "External/S32K142/include/S32K142.h"

#include <stdint.h>
#include <stdbool.h>

typedef const struct
{
	uint32_t InstanceID;
} HAL_Encoder_T;

static inline uint32_t HAL_Encoder_ReadTimerCounter(const HAL_Encoder_T * p_encoder)
{
	(void)p_encoder; /* KLS has only 1 possible encoder */
	return FTM2->CNT;
}

static inline bool HAL_Encoder_ReadDirection(const HAL_Encoder_T * p_encoder)
{
	(void)p_encoder;
	return (bool)(FTM2->QDCTRL & FTM_QDCTRL_QUADIR_MASK);
}

static inline bool HAL_Encoder_ReadPhaseA(const HAL_Encoder_T * p_encoder)
{
	(void)p_encoder;
	return (bool)(PTE->PDIR & ((uint32_t)1U << 5));
}

static inline bool HAL_Encoder_ReadPhaseB(const HAL_Encoder_T * p_encoder)
{
	(void)p_encoder;
	return (bool)(PTE->PDIR & ((uint32_t)1U << 4));
}

#endif
