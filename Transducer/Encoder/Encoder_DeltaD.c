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
	if(p_encoder->CONFIG.P_PARAMS != 0U) { memcpy(&p_encoder->Params, p_encoder->CONFIG.P_PARAMS, sizeof(Encoder_Params_T)); }

#if 	defined(CONFIG_ENCODER_HW_DECODER)
	// Pin_Deinit(&p_encoder->PinA);
	// Pin_Deinit(&p_encoder->PinB);
	HAL_Encoder_InitCounter(p_encoder->CONFIG.P_HAL_ENCODER_COUNTER);
	HAL_Encoder_WriteCounterMax(p_encoder->CONFIG.P_HAL_ENCODER_COUNTER, p_encoder->Params.CountsPerRevolution - 1U);
#elif 	defined(CONFIG_ENCODER_HW_EMULATED)
	#ifdef CONFIG_ENCODER_QUADRATURE_MODE_ENABLE
	if(p_encoder->Params.IsQuadratureCaptureEnabled == true)
	{
		Pin_Input_Init(&p_encoder->PinA);
		Pin_Input_Init(&p_encoder->PinB);
	}
	// HAL_Encoder_EnablePhaseInterrupt(p_encoder->CONFIG.P_HAL_ENCODER_A, p_encoder->CONFIG.PHASE_A_ID);
	// HAL_Encoder_EnablePhaseInterrupt(p_encoder->CONFIG.P_HAL_ENCODER_B, p_encoder->CONFIG.PHASE_B_ID);
	// HAL_Encoder_EnablePhaseInterrupt(p_encoder->CONFIG.P_HAL_ENCODER_Z, p_encoder->CONFIG.PHASE_Z_ID);
	#endif
#endif

	p_encoder->UnitT_Freq = p_encoder->CONFIG.D_SPEED_FREQ;
	_Encoder_ResetUnitsAngular(p_encoder);
	_Encoder_ResetUnitsLinear(p_encoder);
	_Encoder_ResetUnitsScalarSpeed(p_encoder);
	if(p_encoder->Params.CountsPerRevolution > (UINT32_MAX / p_encoder->UnitAngularSpeed)) { p_encoder->UnitAngularSpeed = 0U; }

	p_encoder->DeltaT = 1U;
	Encoder_DeltaD_SetInitial(p_encoder);
}

void Encoder_DeltaD_SetInitial(Encoder_T * p_encoder)
{
#if 	defined(CONFIG_ENCODER_HW_DECODER)
	HAL_Encoder_ClearCounterOverflow(p_encoder->CONFIG.P_HAL_ENCODER_COUNTER);
	HAL_Encoder_WriteCounter(p_encoder->CONFIG.P_HAL_ENCODER_COUNTER, 0U);
	p_encoder->CounterD = HAL_Encoder_ReadCounter(p_encoder->CONFIG.P_HAL_ENCODER_COUNTER);
#elif 	defined(CONFIG_ENCODER_HW_EMULATED)
	_Encoder_ZeroAngle(p_encoder);
#endif
}


#if defined(CONFIG_ENCODER_QUADRATURE_MODE_ENABLE) || defined(CONFIG_ENCODER_QUADRATURE_MODE_DECODER_ONLY)
/*
	Run on calibration routine start
*/
void Encoder_DeltaD_CalibrateQuadratureReference(Encoder_T * p_encoder)
{
#if 	defined(CONFIG_ENCODER_HW_DECODER)
	p_encoder->CounterD = HAL_Encoder_ReadCounter(p_encoder->CONFIG.P_HAL_ENCODER_COUNTER);
#elif 	defined(CONFIG_ENCODER_HW_EMULATED)
	p_encoder->CounterD = 0;
#endif
}

/*
	call after having moved in the positive direction
*/
void Encoder_DeltaD_CalibrateQuadraturePositive(Encoder_T * p_encoder)
{
#if 	defined(CONFIG_ENCODER_HW_DECODER)
	uint32_t counterValue = HAL_Encoder_ReadCounter(p_encoder->CONFIG.P_HAL_ENCODER_COUNTER);
	#ifdef CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_INCREMENT
	p_encoder->Params.IsALeadBPositive = (counterValue > p_encoder->CounterD);
	#elif defined(CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_DECREMENT)
	p_encoder->Params.IsALeadBPositive = !(counterValue > p_encoder->CounterD);
	#endif
#elif 	defined(CONFIG_ENCODER_HW_EMULATED)
	p_encoder->Params.IsALeadBPositive = (p_encoder->CounterD > 0);
#endif
}

#endif

