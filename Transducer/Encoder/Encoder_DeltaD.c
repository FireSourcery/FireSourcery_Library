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
	@file  	Encoder_DeltaD.c
	@author FireSourcery
	@brief
	@version V0
 */
/******************************************************************************/
#include "Encoder.h"
#include "HAL_Encoder.h"

#include <stdint.h>
#include <stdbool.h>


/*!
	Uses periodic timer ISR.
 */
void Encoder_DeltaD_Init(Encoder_T * p_encoder, uint32_t encoderCountsPerRevolution, uint32_t encoderDistancePerCount)
{
//	Pin_Deinit(&p_encoder->CONFIG.PIN_PHASE_A);
//	Pin_Deinit(&p_encoder->CONFIG.PIN_PHASE_B);

	HAL_Encoder_InitCaptureCount
	(
		p_encoder->CONFIG.P_HAL_ENCODER,
		p_encoder->CONFIG.PIN_PHASE_A.P_HAL_PIN,
		p_encoder->CONFIG.PIN_PHASE_A.ID,
		p_encoder->CONFIG.PIN_PHASE_B.P_HAL_PIN,
		p_encoder->CONFIG.PIN_PHASE_B.ID
	);

	HAL_Encoder_WriteTimerCounterMax(p_encoder->CONFIG.P_HAL_ENCODER, encoderCountsPerRevolution - 1U);
	_Encoder_Init(p_encoder, encoderCountsPerRevolution, encoderDistancePerCount, p_encoder->CONFIG.POLLING_FREQ);

#ifdef CONFIG_ENCODER_HW_QUADRATURE_CAPABLE
	p_encoder->IsQuadratureCaptureEnabled = true;
#endif
}

void Encoder_DeltaD_InitQuadratureDirection(Encoder_T * p_encoder, bool isALeadBDirectionPositive)
{
	p_encoder->IsALeadBDirectionPositive = isALeadBDirectionPositive;
}

void Encoder_CalibrateAngularD(Encoder_T * p_encoder)
{
	HAL_Encoder_WriteTimerCounter(p_encoder->CONFIG.P_HAL_ENCODER, 0U);
	p_encoder->AngularD = HAL_Encoder_ReadTimerCounter(p_encoder->CONFIG.P_HAL_ENCODER);
}

/*
 *  Calibration step 1
 */
void Encoder_DeltaD_CalibrateQuadratureReference(Encoder_T * p_encoder)
{
	HAL_Encoder_WriteTimerCounter(p_encoder->CONFIG.P_HAL_ENCODER, p_encoder->EncoderResolution / 2U);
	p_encoder->TimerCounterSaved = HAL_Encoder_ReadTimerCounter(p_encoder->CONFIG.P_HAL_ENCODER);
}

//call after having moved in the positive direction
void Encoder_DeltaD_CalibrateQuadraturePositive(Encoder_T * p_encoder)
{
	uint32_t counterValue = HAL_Encoder_ReadTimerCounter(p_encoder->CONFIG.P_HAL_ENCODER);

#ifdef CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_INCREMENT
	bool isALeadBIncrement = true;
#elif defined(CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_DECREMENT)
	bool isALeadBIncrement = false;
#endif

	if (!((counterValue > p_encoder->TimerCounterSaved) ^ isALeadBIncrement))
	{
		p_encoder->IsALeadBDirectionPositive = true;
	}
	else
	{
		p_encoder->IsALeadBDirectionPositive = false;
	}
}
