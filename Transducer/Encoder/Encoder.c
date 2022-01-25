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
	@file  	Encoder.c
	@author FireSourcery
	@brief 	Encoder module conventional function definitions
	@version V0
 */
/******************************************************************************/
#include "Encoder.h"
#include "HAL_Encoder.h"
#include "Config.h"

#include <stdint.h>
#include <stdbool.h>

/*!
	highest precision (factor << leftShift / divisor) without overflow
 */
static inline uint32_t MaxLeftShiftDivide(uint32_t factor, uint32_t divisor, uint8_t targetShift)
{
	uint32_t result = 0;
	uint32_t shiftedValue = (1U << targetShift);

	if (shiftedValue % divisor == 0U) /* when divisor is a whole number factor of shifted. */
	{
		result = factor * (shiftedValue / divisor);
	}
	else
	{
		for (uint8_t maxShift = targetShift; maxShift > 0U; maxShift--)
		{
			if ( factor <= (UINT32_MAX >> maxShift) )
			{
				result = (factor << maxShift) / divisor;

				if ( result <= (UINT32_MAX >> (targetShift - maxShift)) ) //check overflow
				{
					result = result << (targetShift - maxShift);
				}
				else  /* error, will overflow 32 bit even using ((factor << 0)/divisor) << leftShift */
				{
					result = 0U;
				}
				break;
			}
		}

	}

	return result;
}

/*!
	@brief Init with provided parameters.
	Default capture mode. HAL function responsible for all corresponding settings
	EncoderResolution, EncoderCounterMax + 1
 */

void _Encoder_SetUnitConversion(Encoder_T * p_encoder, uint32_t encoderCountsPerRevolution, uint32_t encoderDistancePerCount, uint32_t unitTFreq)
{
//	HAL_Encoder_Init(p_encoder->CONFIG.P_HAL_ENCODER);
//	p_encoder->EncoderResolution = encoderCountsPerRevolution;
	p_encoder->UnitT_Freq = unitTFreq;

	p_encoder->UnitLinearD = encoderDistancePerCount;
	/*
	 * Possible 32 bit overflow
	 *
	 * For case of CaptureDeltaT(), DeltaD == 1: unitDeltaT_Freq will be large, constraint on unitDeltaD
	 * Max unitDeltaD will be UINT32_MAX / (unitDeltaT_Freq)
	 * unitDeltaD ~14,000, for 300,000 unitDeltaT_Freq
	 *
	 * For case of CaptureDeltaD(), DeltaT == 1: constraint on unitDeltaD, and deltaD
	 * Max deltaD will be UINT32_MAX / (unitDeltaD * unitDeltaT_Freq)
	 * deltaD ~14,000, for 300,000 (unitDeltaD * unitDeltaT_Freq)
	 */
	p_encoder->UnitLinearSpeed = encoderDistancePerCount * unitTFreq;

	/*
	 * Angle Calc
	 * Angle = UnitAngle_Factor * DeltaD >> UnitAngle_Divisor
	 *
	 * uneven UnitD = unitAngle_DataBits/unitAngle_SensorResolution divide results in loss of precision
	 */
	p_encoder->UnitAngularD_Factor = 0xFFFFFFFFU / encoderCountsPerRevolution + 1U;

	/*
	 * Angular Speed Calc
	 *
	 * ((DeltaD * UnitT_Freq) << unitAngle_DataBits / (unitAngle_SensorResolution * DeltaT)) overflow
	 *
	 * 		Speed = (DeltaD * UnitD) / ANGLE_DEGREES_BITS * UnitT_Freq / Delta
	 * Use: speed = DeltaD * [UnitSpeed] / DeltaT = DeltaD * [UnitD * UnitT_Freq / ANGLE_DEGREES_BITS] / DeltaT
	 *
	 * most cases: UnitT_Freq > DeltaD
	 *
	 * e.g.
	 * Angular Speed =
	 * Real:   				DeltaD * UnitAngularD_Factor * UnitT_Freq / ANGLE_DEGREES_BITS / DeltaT
	 * 						8000 * 429,497{encoderRes == 10k} * 20000 / 65536 / 1 == 1,048,576,660.16
	 *
	 * Primitive:	  		(DeltaD * UnitAngularD_Factor) / ANGLE_DEGREES_BITS * UnitT_Freq / DeltaT == 1,048,560,000
	 * UnitAngularSpeed: 	DeltaD * [UnitAngularD_Factor * UnitT_Freq / ANGLE_DEGREES_BITS] / DeltaT == 1,048,576,000
	 */
	p_encoder->UnitAngularSpeed = MaxLeftShiftDivide(unitTFreq, encoderCountsPerRevolution, CONFIG_ENCODER_ANGLE_DEGREES_BITS);

//	p_encoder->IsUnitAngularSpeedOverflow = !p_encoder->UnitAngularSpeed;

	p_encoder->UnitInterpolateAngle = MaxLeftShiftDivide(unitTFreq, p_encoder->CONFIG.POLLING_FREQ * encoderCountsPerRevolution, CONFIG_ENCODER_ANGLE_DEGREES_BITS);

	Encoder_Reset(p_encoder);
}


//reset runtime variables
void Encoder_Reset(Encoder_T * p_encoder)
{
	p_encoder->DeltaD = 1U;
	p_encoder->DeltaT = 1U;
	p_encoder->TotalD = 0U;
	p_encoder->TotalT = 0U;
	p_encoder->AngularD = 0U;
//	p_encoder->UserD 	= 0;
//	p_encoder->UserT 	= 0;

	p_encoder->SpeedSaved = 0U;
	p_encoder->DeltaSpeed = 0U;

	HAL_Encoder_WriteTimerCounter(p_encoder->CONFIG.P_HAL_ENCODER, 0U);; /* reset for angularD */
	p_encoder->TimerCounterSaved = HAL_Encoder_ReadTimerCounter(p_encoder->CONFIG.P_HAL_ENCODER);
	p_encoder->ExtendedTimerSaved = *(p_encoder->CONFIG.P_EXTENDED_TIMER);
	HAL_Encoder_ClearTimerCounterOverflow(p_encoder->CONFIG.P_HAL_ENCODER);
}

//must use encoder init first
void Encoder_SetQuadratureMode(Encoder_T * p_encoder, bool isEnabled)
{
	p_encoder->Params.IsQuadratureCaptureEnabled = isEnabled;
}

/*!
 * isALeadBIncrement - Match to HAL/unused for deltaT Mode
 * isALeadBPositive - User runtime calibrate
 */
void Encoder_SetQuadratureDirectionCalibration(Encoder_T * p_encoder, bool isALeadBPositive)
{
	p_encoder->Params.IsALeadBPositive = isALeadBPositive;
}

