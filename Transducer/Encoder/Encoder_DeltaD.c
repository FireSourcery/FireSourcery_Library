/******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSourcery / The Firebrand Forge Inc

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
	@file  	Encoder_DeltaD.c
	@author FireSourcery
	@brief
	@version V0
*/
/******************************************************************************/
#include "Encoder_DeltaD.h"
#include <string.h>

/*!

*/
void Encoder_DeltaD_Init(Encoder_T * p_encoder)
{
#if 	defined(CONFIG_ENCODER_HW_DECODER)
	// HAL_Encoder_InitCounter(p_encoder->CONFIG.P_HAL_ENCODER_COUNTER);
#elif 	defined(CONFIG_ENCODER_HW_EMULATED)
	HAL_Encoder_EnablePhaseInterrupt(p_encoder->CONFIG.P_HAL_ENCODER_A, p_encoder->CONFIG.PHASE_A_ID);
	HAL_Encoder_EnablePhaseInterrupt(p_encoder->CONFIG.P_HAL_ENCODER_B, p_encoder->CONFIG.PHASE_B_ID);
	HAL_Encoder_EnablePhaseInterrupt(p_encoder->CONFIG.P_HAL_ENCODER_Z, p_encoder->CONFIG.PHASE_Z_ID);
	#ifdef CONFIG_ENCODER_QUADRATURE_MODE_ENABLE
	if(p_encoder->Params.IsQuadratureCaptureEnabled == true)
	{
		Pin_Deinit(&p_encoder->PinA);
		Pin_Deinit(&p_encoder->PinB);
	}
	#endif
#endif

	if(p_encoder->CONFIG.P_PARAMS != 0U) { memcpy(&p_encoder->Params, p_encoder->CONFIG.P_PARAMS, sizeof(Encoder_Params_T)); }

#if 	defined(CONFIG_ENCODER_HW_DECODER)
	HAL_Encoder_WriteTimerCounterMax(p_encoder->CONFIG.P_HAL_ENCODER, p_encoder->Params.CountsPerRevolution - 1U);
#elif 	defined(CONFIG_ENCODER_HW_EMULATED)

#endif

	p_encoder->UnitT_Freq = p_encoder->CONFIG.SPEED_SAMPLE_FREQ;
	_Encoder_ResetUnitsAngular(p_encoder);
	_Encoder_ResetUnitsLinear(p_encoder);
	_Encoder_ResetUnitsFrac16Speed(p_encoder);

	if(p_encoder->Params.CountsPerRevolution > (UINT32_MAX / p_encoder->UnitAngularSpeed)) { p_encoder->UnitAngularSpeed = 0U; }

	Encoder_Zero(p_encoder);
}

void Encoder_DeltaD_SetInitial(Encoder_T * p_encoder)
{
#if 	defined(CONFIG_ENCODER_HW_DECODER)
	HAL_Encoder_WriteTimerCounter(p_encoder->CONFIG.P_HAL_ENCODER, 0U);
	p_encoder->TimerCounterSaved = HAL_Encoder_ReadTimerCounter(p_encoder->CONFIG.P_HAL_ENCODER);
#elif 	defined(CONFIG_ENCODER_HW_EMULATED)
#endif
	p_encoder->AngularD = 0U;
}

/*
	Run on calibration routine start
*/
void Encoder_DeltaD_CalibrateQuadratureReference(Encoder_T * p_encoder)
{
#if 	defined(CONFIG_ENCODER_HW_DECODER)
	p_encoder->TimerCounterSaved = HAL_Encoder_ReadTimerCounter(p_encoder->CONFIG.P_HAL_ENCODER);
#elif 	defined(CONFIG_ENCODER_HW_EMULATED)
#endif
}

/*
	call after having moved in the positive direction
*/
void Encoder_DeltaD_CalibrateQuadraturePositive(Encoder_T * p_encoder)
{
#if 	defined(CONFIG_ENCODER_HW_DECODER)
	uint32_t counterValue = HAL_Encoder_ReadTimerCounter(p_encoder->CONFIG.P_HAL_ENCODER);
	#ifdef CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_INCREMENT
	p_encoder->Params.IsALeadBPositive = (counterValue > p_encoder->TimerCounterSaved);
	#elif defined(CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_DECREMENT)
	p_encoder->Params.IsALeadBPositive = !(counterValue > p_encoder->TimerCounterSaved);
	#endif
#elif 	defined(CONFIG_ENCODER_HW_EMULATED)
#endif

//	Encoder_DeltaD_SetQuadratureDirectionCalibration(p_encoder, (counterValue > p_encoder->TimerCounterSaved));
}

// void Encoder_DeltaD_SetQuadratureDirectionCalibration(Encoder_T * p_encoder, bool isALeadBDirectionPositive)
// {
// #ifdef CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_INCREMENT
// 	p_encoder->Params.DirectionCalibration = (isALeadBDirectionPositive) ? ENCODER_DIRECTION_DIRECT : ENCODER_DIRECTION_REVERSE;
// #elif defined(CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_DECREMENT)
// 	p_encoder->Params.DirectionCalibration = (!isALeadBDirectionPositive) ? ENCODER_DIRECTION_DIRECT : ENCODER_DIRECTION_REVERSE;
// #endif
// }
