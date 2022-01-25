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
#ifndef HAL_ENCODER_PLATFORM_H
#define HAL_ENCODER_PLATFORM_H

#include "External/S32K142/include/S32K142.h"

#include "Peripheral/HAL/Platform/S32K/Chip.h"

#include <stdint.h>
#include <stdbool.h>

typedef FTM_Type HAL_Encoder_T;

static inline bool HAL_Encoder_ReadTimerCounterOverflow(const HAL_Encoder_T * p_encoder)
{
	return p_encoder->SC & FTM_SC_TOF_MASK;
}

/* Clear interrupt */
static inline void HAL_Encoder_ClearTimerCounterOverflow(HAL_Encoder_T * p_encoder)
{
	p_encoder->SC &= ~FTM_SC_TOF_MASK;
	p_encoder->SC;	/* Read-after-write sequence to guarantee required serialization of memory operations */
}

static inline uint32_t HAL_Encoder_ReadTimerCounter(const HAL_Encoder_T * p_encoder)
{
	return p_encoder->CNT;
}

static inline void HAL_Encoder_WriteTimerCounter(HAL_Encoder_T * p_encoder, uint32_t count)
{
	p_encoder->CNT = FTM_CNT_COUNT(count);
}

static inline void HAL_Encoder_WriteTimerCounterMax(HAL_Encoder_T * p_encoder, uint32_t max)
{
	p_encoder->MOD = FTM_MOD_MOD(max);
}

/*
 * S32K use pin read Phase
 */
//static inline bool HAL_Encoder_ReadQuadraturePhaseA(const HAL_Encoder_T * p_encoder){(void)p_encoder;}
//static inline bool HAL_Encoder_ReadQuadraturePhaseB(const HAL_Encoder_T * p_encoder){(void)p_encoder;}

static inline bool HAL_Encoder_ReadQuadratureCounterDirection(const HAL_Encoder_T * p_encoder)
{
	return ((p_encoder->QDCTRL & FTM_QDCTRL_QUADIR_MASK) != 0U) ? true : false;
}

/*
 * Return true if counter over flow on increment
 */
static inline bool HAL_Encoder_ReadQuadratureCounterOverflowIncrement(const HAL_Encoder_T * p_encoder)
{
	return ((p_encoder->QDCTRL & FTM_QDCTRL_TOFDIR_MASK) != 0U) ? true : false;
}

static inline bool HAL_Encoder_ReadQuadratureCounterOverflowDecrement(const HAL_Encoder_T * p_encoder)
{
	return ((p_encoder->QDCTRL & FTM_QDCTRL_TOFDIR_MASK) == 0U) ? true : false;
}

extern void Board_Encoder_InitCaptureTime(void);
extern void Board_Encoder_InitCaptureCount(void);

/*
	Config SW Polling capture mode
	On S32K Hw capture mode disables gpio pin read
 */
static inline void HAL_Encoder_InitCaptureTime(HAL_Encoder_T * p_encoder, void * p_phaseAPinHal, uint32_t phaseAPinId, void * p_phaseBPinHal, uint32_t phaseBPinId)
{
//	(p_encoder->QDCTRL) &= ~(1UL << FTM_QDCTRL_QUADEN_SHIFT);
	Board_Encoder_InitCaptureTime();
}

/*
 * Enocder Res still needs to be configured.
 */
static inline void HAL_Encoder_InitCaptureCount(HAL_Encoder_T * p_encoder, void * p_phaseAPinHal, uint32_t phaseAPinId, void * p_phaseBPinHal, uint32_t phaseBPinId)
{
//	(p_encoder->QDCTRL) |= (1UL << FTM_QDCTRL_QUADEN_SHIFT);
	Board_Encoder_InitCaptureCount();
}

static inline void HAL_Encoder_Init(HAL_Encoder_T * p_encoder)
{

}

#endif
