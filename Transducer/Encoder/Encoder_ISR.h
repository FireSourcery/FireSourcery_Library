/******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2022 FireSourcery / The Firebrand Forge Inc

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
	@file  	Encoder_ISR.h
	@author FireSourcery
	@version V0
	@brief
*/
/******************************************************************************/
#ifndef ENCODER_ISR_H
#define ENCODER_ISR_H

#include "Encoder.h"
#include "Encoder_DeltaD.h"
#include "Encoder_DeltaT.h"
#include "Encoder_Motor.h"

/******************************************************************************/
/*!
	ISRs
	emulated capture D, or callback hook
*/
/*! @{ */
/******************************************************************************/
/*
	Index Pin
*/
static inline void Encoder_OnIndex_ISR(Encoder_T * p_encoder)
{
	HAL_Encoder_ClearPhaseFlag(p_encoder->CONFIG.P_HAL_ENCODER_Z, p_encoder->CONFIG.PHASE_Z_ID);
#if 	defined(CONFIG_ENCODER_HW_EMULATED)
#else 	defined(CONFIG_ENCODER_HW_DECODER)
	HAL_Encoder_WriteTimerCounter(p_encoder->CONFIG.P_HAL_ENCODER, 0U);
#endif
	p_encoder->AngularD = 0U;
}

//todo check freq for auto capture speed
static inline void Encoder_OnPhaseA_ISR(Encoder_T * p_encoder)
{
	HAL_Encoder_ClearPhaseFlag(p_encoder->CONFIG.P_HAL_ENCODER_A, p_encoder->CONFIG.PHASE_A_ID);
// #if 	defined(CONFIG_ENCODER_HW_EMULATED)
// 	_Encoder_CaptureAngularD(p_encoder);
// #else /* DeltaT ISR version */
	Encoder_DeltaT_Capture(p_encoder); /* superfluous deltaT capture for emulated D mode for now */
// #endif
}

static inline void Encoder_OnPhaseB_ISR(Encoder_T * p_encoder)
{
	HAL_Encoder_ClearPhaseFlag(p_encoder->CONFIG.P_HAL_ENCODER_B, p_encoder->CONFIG.PHASE_B_ID);
// #if 	defined(CONFIG_ENCODER_HW_EMULATED)
// 	_Encoder_CaptureAngularD(p_encoder);
// #else
	Encoder_DeltaT_Capture(p_encoder);
// #endif
}

static inline void Encoder_Motor_OnPhaseC_ISR(Encoder_T * p_encoder)
{
	HAL_Encoder_ClearPhaseFlag(p_encoder->CONFIG.P_HAL_ENCODER_Z, p_encoder->CONFIG.PHASE_Z_ID);
// #if 	defined(CONFIG_ENCODER_HW_EMULATED)
// 	_Encoder_CaptureAngularD(p_encoder);
// #else
	Encoder_DeltaT_Capture(p_encoder);
// #endif
}

/* Shared A, B Thread */
static inline void Encoder_OnPhaseAB_ISR(Encoder_T * p_encoder)
{
	if 		(HAL_Encoder_ReadPhaseFlag(p_encoder->CONFIG.P_HAL_ENCODER_A, p_encoder->CONFIG.PHASE_A_ID) == true) { Encoder_OnPhaseA_ISR(p_encoder); }
	else if	(HAL_Encoder_ReadPhaseFlag(p_encoder->CONFIG.P_HAL_ENCODER_B, p_encoder->CONFIG.PHASE_B_ID) == true) { Encoder_OnPhaseB_ISR(p_encoder); }
}

/* Shared A, B, Index Thread */
static inline void Encoder_OnPhaseABZ_ISR(Encoder_T * p_encoder)
{
	if 		(HAL_Encoder_ReadPhaseFlag(p_encoder->CONFIG.P_HAL_ENCODER_A, p_encoder->CONFIG.PHASE_A_ID) == true) { Encoder_OnPhaseA_ISR(p_encoder); }
	else if	(HAL_Encoder_ReadPhaseFlag(p_encoder->CONFIG.P_HAL_ENCODER_B, p_encoder->CONFIG.PHASE_B_ID) == true) { Encoder_OnPhaseB_ISR(p_encoder); }
	else if	(HAL_Encoder_ReadPhaseFlag(p_encoder->CONFIG.P_HAL_ENCODER_Z, p_encoder->CONFIG.PHASE_Z_ID) == true) { Encoder_OnIndex_ISR(p_encoder); }
}
/******************************************************************************/
/*! @} */
/******************************************************************************/

#endif