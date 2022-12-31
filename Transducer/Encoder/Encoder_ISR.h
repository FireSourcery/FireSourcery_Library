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

/******************************************************************************/
/*!
	@brief 	SW Capture Functions -
	CONFIG_ENCODER_HW_EMULATED mode DeltaD, ModeDT; DeltaT ISR Mode
*/
/******************************************************************************/
/******************************************************************************/
/*
	Quadrature, Signed Direction
	Captures as ALeadB is Positive, compensate on user Get
*/
/******************************************************************************/
static inline uint8_t _Encoder_CapturePhasesState(Encoder_T * p_encoder)
{
	p_encoder->Phases.PrevA = p_encoder->Phases.A;
	p_encoder->Phases.PrevB = p_encoder->Phases.B;
	p_encoder->Phases.A = Pin_Input_ReadPhysical(&p_encoder->PinA);
	p_encoder->Phases.B = Pin_Input_ReadPhysical(&p_encoder->PinB);
	return p_encoder->Phases.State;
}

static inline void _Encoder_CaptureCount(Encoder_T * p_encoder, int8_t count)
{
	p_encoder->CounterD += count;
	// p_encoder->TotalD += count;
	p_encoder->Angle32 += ((int32_t)count * (int32_t)p_encoder->UnitAngularD);
}

static inline void _Encoder_CaptureCount_Quadrature(Encoder_T * p_encoder, int8_t count)
{
	if(count == _ENCODER_TABLE_ERROR) 	{ p_encoder->ErrorCount++; }
	else 								{ _Encoder_CaptureCount(p_encoder, count); }
}

static inline void _Encoder_CapturePulse_Quadrature(Encoder_T * p_encoder)
{
	_Encoder_CaptureCount_Quadrature(p_encoder, _ENCODER_TABLE[_Encoder_CapturePhasesState(p_encoder)]);
}

static inline void _Encoder_CapturePulse_QuadraturePhaseA(Encoder_T * p_encoder)
{
	_Encoder_CaptureCount_Quadrature(p_encoder, _ENCODER_TABLE_PHASE_A[_Encoder_CapturePhasesState(p_encoder)]);
}

static inline void _Encoder_CapturePulse_QuadraturePhaseARisingEdge(Encoder_T * p_encoder)
{
	_Encoder_CaptureCount_Quadrature(p_encoder, ((Pin_Input_ReadPhysical(&p_encoder->PinB) == false) ? 1 : -1));
}

/******************************************************************************/
/*
	Single Phase, Unsigned Direction
*/
/******************************************************************************/
static inline void _Encoder_CapturePulse_SinglePhase(Encoder_T * p_encoder)
{
	_Encoder_CaptureCount(p_encoder, 1);
}

/******************************************************************************/
/*
	User compile time implement mode
*/
/******************************************************************************/
static inline void Encoder_CapturePulse_Quadrature(Encoder_T * p_encoder)
{
	_Encoder_CapturePulse_Quadrature(p_encoder);
	Encoder_DeltaT_CaptureExtended(p_encoder);
	Encoder_DeltaT_ZeroInterpolateAngle(p_encoder);
}

static inline void Encoder_CapturePulse_SinglePhase(Encoder_T * p_encoder)
{
	_Encoder_CapturePulse_SinglePhase(p_encoder);
	Encoder_DeltaT_CaptureExtended(p_encoder);
	Encoder_DeltaT_ZeroInterpolateAngle(p_encoder);
}

/******************************************************************************/
/*
	Quadrature On/Off Switch
*/
/******************************************************************************/
static inline void _Encoder_CapturePulse(Encoder_T * p_encoder)
{
	Encoder_ProcCaptureModeFunction(p_encoder, _Encoder_CapturePulse_Quadrature, _Encoder_CapturePulse_SinglePhase);
}

/*
	Quadrature and Single Phase select via IsQuadratureMode flag
*/
static inline void Encoder_CapturePulse(Encoder_T * p_encoder)
{
	_Encoder_CapturePulse(p_encoder);
	Encoder_DeltaT_CaptureExtended(p_encoder);
	Encoder_DeltaT_ZeroInterpolateAngle(p_encoder);
}

/******************************************************************************/
/*!
	ISRs
*/
/*! @{ */
/******************************************************************************/
/******************************************************************************/
/*!
	Upper layer configures ISRs
*/
/******************************************************************************/
static inline void _Encoder_OnPhaseA_ISR(Encoder_T * p_encoder)
{
	HAL_Encoder_ClearPhaseFlag(p_encoder->CONFIG.P_HAL_ENCODER_A, p_encoder->CONFIG.PHASE_A_ID);
}

static inline void _Encoder_OnPhaseB_ISR(Encoder_T * p_encoder)
{
	HAL_Encoder_ClearPhaseFlag(p_encoder->CONFIG.P_HAL_ENCODER_B, p_encoder->CONFIG.PHASE_B_ID);
}

static inline void _Encoder_OnPhaseZ_ISR(Encoder_T * p_encoder)
{
	HAL_Encoder_ClearPhaseFlag(p_encoder->CONFIG.P_HAL_ENCODER_Z, p_encoder->CONFIG.PHASE_Z_ID);
}

/* Shared A, B ISR */
static inline void _Encoder_OnPhaseAB_ISR(Encoder_T * p_encoder)
{
	if 		(HAL_Encoder_ReadPhaseFlag(p_encoder->CONFIG.P_HAL_ENCODER_A, p_encoder->CONFIG.PHASE_A_ID) == true) { _Encoder_OnPhaseA_ISR(p_encoder); }
	else if	(HAL_Encoder_ReadPhaseFlag(p_encoder->CONFIG.P_HAL_ENCODER_B, p_encoder->CONFIG.PHASE_B_ID) == true) { _Encoder_OnPhaseB_ISR(p_encoder); }
}

/******************************************************************************/
/*!
	Configured using IsQuadratureMode flag
	Superfluous capture DeltaT for Emulated DeltaD only mode
*/
/******************************************************************************/
static inline void Encoder_OnPhaseA_ISR(Encoder_T * p_encoder)
{
	HAL_Encoder_ClearPhaseFlag(p_encoder->CONFIG.P_HAL_ENCODER_A, p_encoder->CONFIG.PHASE_A_ID);
	Encoder_CapturePulse(p_encoder);
}

static inline void Encoder_OnPhaseB_ISR(Encoder_T * p_encoder)
{
	HAL_Encoder_ClearPhaseFlag(p_encoder->CONFIG.P_HAL_ENCODER_B, p_encoder->CONFIG.PHASE_B_ID);
	Encoder_CapturePulse(p_encoder);
}

/*
	Index Pin
*/
static inline void Encoder_OnIndex_ISR(Encoder_T * p_encoder)
{
	HAL_Encoder_ClearPhaseFlag(p_encoder->CONFIG.P_HAL_ENCODER_Z, p_encoder->CONFIG.PHASE_Z_ID);
	p_encoder->IndexCount++;
// #if 		defined(CONFIG_ENCODER_HW_EMULATED)
// #elif 	defined(CONFIG_ENCODER_HW_DECODER)
// 	HAL_Encoder_WriteCounter(p_encoder->CONFIG.P_HAL_ENCODER_COUNTER, 0U);
// #endif
}

static inline void Encoder_OnPhaseC_Hall_ISR(Encoder_T * p_encoder)
{
	HAL_Encoder_ClearPhaseFlag(p_encoder->CONFIG.P_HAL_ENCODER_Z, p_encoder->CONFIG.PHASE_Z_ID);
	Encoder_CapturePulse(p_encoder);
}

/* Shared A, B ISR */
static inline void Encoder_OnPhaseAB_ISR(Encoder_T * p_encoder)
{
	if 		(HAL_Encoder_ReadPhaseFlag(p_encoder->CONFIG.P_HAL_ENCODER_A, p_encoder->CONFIG.PHASE_A_ID) == true) { Encoder_OnPhaseA_ISR(p_encoder); }
	else if	(HAL_Encoder_ReadPhaseFlag(p_encoder->CONFIG.P_HAL_ENCODER_B, p_encoder->CONFIG.PHASE_B_ID) == true) { Encoder_OnPhaseB_ISR(p_encoder); }
}

/* Shared A, B, Index ISR */
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