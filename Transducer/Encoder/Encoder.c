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

void _Encoder_SetUnitConversion(Encoder_T * p_encoder, uint32_t countsPerRevolution, uint32_t distancePerCount, uint32_t unitTFreq)
{
	p_encoder->UnitT_Freq = unitTFreq;

	p_encoder->UnitLinearD = distancePerCount;
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
	p_encoder->UnitLinearSpeed = distancePerCount * unitTFreq;

	/*
	 * Angle Calc
	 * Angle = UnitAngle_Factor * DeltaD >> UnitAngle_Divisor
	 *
	 * uneven UnitD = unitAngle_DataBits/unitAngle_SensorResolution divide results in loss of precision
	 *
	 * UnitAngularD_ShiftDivisor == CONFIG_ENCODER_ANGLE_DEGREES_BITS
	 */
	p_encoder->UnitAngularD_Factor = 0xFFFFFFFFU / countsPerRevolution + 1U;

	/*
	 * Angular Speed Calc
	 * Angular Speed = DeltaD * [UnitAngularD_Factor * UnitT_Freq / (32-ANGLE_DEGREES_BITS)] / DeltaT
	 *
	 * most cases: UnitT_Freq > DeltaD
	 *
	 * e.g.
	 * Real:   				8000 * 429,497{encoderRes == 10k} * 20000 / 65536 / 1 == 1,048,576,660.16
	 *
	 * Primitive:	  		(DeltaD * UnitAngularD_Factor) / (32-ANGLE_DEGREES_BITS) * UnitT_Freq / DeltaT == 1,048,560,000
	 * UnitAngularSpeed: 	DeltaD * [UnitAngularD_Factor * UnitT_Freq / (32-ANGLE_DEGREES_BITS)] / DeltaT == 1,048,576,000
	 */
	p_encoder->UnitAngularSpeed = MaxLeftShiftDivide(unitTFreq, countsPerRevolution, CONFIG_ENCODER_ANGLE_DEGREES_BITS);

	//todo overflow boundary
	//overflow boundary = UINT32_MAX / p_encoder->UnitAngularSpeed
//	if(unitTFreq > (UINT32_MAX >> CONFIG_ENCODER_ANGLE_DEGREES_BITS))
//	{
//		p_encoder->UnitAngularSpeed = 0U;
//	}
//	else
//	{
//		p_encoder->UnitAngularSpeed = (unitTFreq << CONFIG_ENCODER_ANGLE_DEGREES_BITS) / encoderCountsPerRevolution;
//	}

	p_encoder->UnitInterpolateAngle = MaxLeftShiftDivide(unitTFreq, p_encoder->CONFIG.POLLING_FREQ * countsPerRevolution, CONFIG_ENCODER_ANGLE_DEGREES_BITS);

//	p_encoder->UnitRefSpeed = p_encoder->UnitT_Freq * 65535U * 60 / p_encoder->Params.CountsPerRevolution / p_encoder->Params.SpeedRef_Rpm;
//	p_encoder->UnitRefSpeed_ShiftDivisor = 0U;
//	speed = (deltaD_Ticks * p_encoder->UnitT_Freq * 60) / (p_encoder->Params.CountsPerRevolution * deltaT_Ticks);
//	Speed_Frac16 = (speed * 65535U / SpeedRefMax_RPM);


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

