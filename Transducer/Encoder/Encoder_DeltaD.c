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
void Encoder_DeltaD_Init
(
	Encoder_T * p_encoder,
	HAL_Encoder_T * p_hal_encoder,
	uint32_t pollingFreq,
	uint32_t encoderDistancePerCount,
	uint32_t encoderCountsPerRevolution
//	uint8_t angleDataBits
)
{
	Encoder_Init
	(
		p_encoder,
		p_hal_encoder,
		encoderCountsPerRevolution - 1U,
		pollingFreq,
		pollingFreq,
		encoderDistancePerCount,
		encoderCountsPerRevolution
//		angleDataBits
	);

	HAL_Encoder_InitCaptureCount(p_hal_encoder);
	//	HAL_Encoder_ConfigCaptureCount(p_hal_encoder);
	//	HAL_Encoder_WriteTimerCounterMax(p_hal_encoder, timerCounterMax);
}

void Encoder_CalibrateAngularD(Encoder_T * p_encoder)
{
	HAL_Encoder_WriteTimerCounter(p_encoder->p_HAL_Encoder, 0U);
	p_encoder->AngularD = HAL_Encoder_ReadTimerCounter(p_encoder->p_HAL_Encoder);
}

void Encoder_DeltaD_CalibrateQuadratureReference(Encoder_T * p_encoder)
{
	HAL_Encoder_WriteTimerCounter(p_encoder->p_HAL_Encoder, p_encoder->EncoderResolution/2U);
	p_encoder->TimerCounterSaved = HAL_Encoder_ReadTimerCounter(p_encoder->p_HAL_Encoder);

//	p_encoder->DeltaD = 1U;
//	p_encoder->DeltaT = 1U;
//	p_encoder->TotalD = 0U;
//	p_encoder->TotalT = 0U;
//	p_encoder->AngularD = 0U;
}

//call after having moved in the positive direction
void Encoder_DeltaD_CalibrateQuadratureDirectionPositive(Encoder_T * p_encoder)
{
	uint32_t counterValue = HAL_Encoder_ReadTimerCounter(p_encoder->p_HAL_Encoder);

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

//	if (counterValue > p_encoder->TimerCounterSaved)
//	{
////		p_encoder->IsCounterIncrementDirectionPositive = true;
//#ifdef CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_INCREMENT
//		p_encoder->IsALeadBDirectionPositive = true;
//#elif defined(CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_DECREMENT)
//		p_encoder->IsALeadBDirectionPositive = false;
//#endif
//	}
//	else
//	{
//
////		p_encoder->IsCounterIncrementDirectionPositive = false;
//#ifdef CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_INCREMENT
//		p_encoder->IsALeadBDirectionPositive = false;
//#elif defined(CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_DECREMENT)
//		p_encoder->IsALeadBDirectionPositive  = true;
//#endif
//	}
}
