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
	Uses periodic timer ISR.
*/
void Encoder_DeltaD_Init(Encoder_T * p_encoder)
{
	//	Pin_Deinit(&p_encoder->PhaseA);
	//	Pin_Deinit(&p_encoder->PhaseB);

	HAL_Encoder_InitCaptureCount
	(
		p_encoder->CONFIG.P_HAL_ENCODER,
		p_encoder->PhaseA.CONFIG.P_HAL_PIN,
		p_encoder->PhaseA.CONFIG.ID,
		p_encoder->PhaseB.CONFIG.P_HAL_PIN,
		p_encoder->PhaseB.CONFIG.ID
	);

	if (p_encoder->CONFIG.P_PARAMS != 0U)
	{
		memcpy(&p_encoder->Params, p_encoder->CONFIG.P_PARAMS, sizeof(Encoder_Params_T));
	}

//todo
// #ifdef CONFIG_ENCODER_HW_QUADRATURE_CAPABLE
// 	if (p_encoder->Params.IsQuadratureCaptureEnabled == true)
// 	{
// //		HAL_Encoder_ConfigCaptureQuadrature(p_encoder);
// 	}
// 	else
// #endif
// 	{
// //		HAL_Encoder_ConfigCaptureDualEdge(p_encoder);
// 	}

	HAL_Encoder_WriteTimerCounterMax(p_encoder->CONFIG.P_HAL_ENCODER, p_encoder->Params.CountsPerRevolution - 1U);

	/*
		e.g.
		CountsPerRevolution = 8192

		SampleFreq = 20000
		UnitAngularSpeed = 160,000 => Max DeltaD = 26,843
		10k RPM => DeltaD = 10000 / 60 * 8192 / 20000 = 68

		SampleFreq = 1000
		UnitAngularSpeed = 8,000 => Max DeltaD = 536,870
		10k RPM => DeltaD = 10000 / 60 * 8192 / 1000 = 1,532
	*/
	// p_encoder->UnitT_Freq = p_encoder->CONFIG.DELTA_D_SAMPLE_FREQ;
	p_encoder->UnitT_Freq = p_encoder->CONFIG.SPEED_SAMPLE_FREQ;
	// _Encoder_ResetTimerFreq(p_encoder);
	_Encoder_ResetUnitsAngular(p_encoder);
	_Encoder_ResetUnitsLinear(p_encoder);
	_Encoder_ResetUnitsScalarSpeed(p_encoder);

	if(p_encoder->Params.CountsPerRevolution > (UINT32_MAX / p_encoder->UnitAngularSpeed))
	{
		p_encoder->UnitAngularSpeed = 0U;
	}

	Encoder_Zero(p_encoder);
}

void Encoder_DeltaD_SetInitial(Encoder_T * p_encoder)
{
	HAL_Encoder_WriteTimerCounter(p_encoder->CONFIG.P_HAL_ENCODER, 0U);
	p_encoder->TimerCounterSaved = HAL_Encoder_ReadTimerCounter(p_encoder->CONFIG.P_HAL_ENCODER);
	p_encoder->AngularD = HAL_Encoder_ReadTimerCounter(p_encoder->CONFIG.P_HAL_ENCODER);
}

/*
	Run on calibration routine
*/
void Encoder_DeltaD_CalibrateQuadratureReference(Encoder_T * p_encoder)
{
	p_encoder->TimerCounterSaved = HAL_Encoder_ReadTimerCounter(p_encoder->CONFIG.P_HAL_ENCODER);
}

//call after having moved in the positive direction
void Encoder_DeltaD_CalibrateQuadraturePositive(Encoder_T * p_encoder)
{
	uint32_t counterValue = HAL_Encoder_ReadTimerCounter(p_encoder->CONFIG.P_HAL_ENCODER);

#ifdef CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_INCREMENT
	p_encoder->Params.IsALeadBPositive = (counterValue > p_encoder->TimerCounterSaved);
#elif defined(CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_DECREMENT)
	p_encoder->Params.IsALeadBPositive = !(counterValue > p_encoder->TimerCounterSaved);
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
