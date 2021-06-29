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

				if ( result <= (UINT32_MAX >> (targetShift - maxShift)) )
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
 */
void Encoder_Init
(
	Encoder_T * p_encoder,
	HAL_Encoder_T * p_hal_encoder,
	uint32_t timerCounterMax,
	uint32_t unitT_Freq,					/* UnitT_Freq  */
	uint32_t pollingFreq,					/* PollingFreq.  CaptureDeltaT Mode polling and InterpolateD */
	uint32_t encoderDistancePerCount,		/* UnitLinearD */
	uint32_t encoderCountsPerRevolution	/* UnitAngularD_Factor = [0xFFFFFFFFU/encoderCountsPerRevolution + 1] */
//	uint8_t angleDataBits					/* UnitAngularD_DivisorShift = [32 - unitAngle_DataBits] */ //remove todo
)
{
	HAL_Encoder_Init(p_hal_encoder);

	p_encoder->p_HAL_Encoder = p_hal_encoder;
	p_encoder->TimerCounterMax = timerCounterMax;
	p_encoder->EncoderResolution = encoderCountsPerRevolution;
	p_encoder->PollingFreq = pollingFreq;  	//	p_encoder->UnitInterpolateD = p_encoder->UnitSpeed / pollingFreq;

	p_encoder->UnitT_Freq = unitT_Freq;
	p_encoder->UnitD = encoderDistancePerCount;

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
	p_encoder->UnitSpeed = encoderDistancePerCount * unitT_Freq;

	/*
	 * Angle Calc
	 *
	 * DeltaD_Units = DeltaD * UnitD
	 * uneven UnitD = unitAngle_DataBits/unitAngle_SensorResolution divide results in loss of precision
	 *
	 * must multiply before unit correction for precision
	 * deltaD_Units = UnitAngle_Factor * DeltaD >> UnitAngle_Divisor
	 */
	/*
	 * p_encoder->UnitD is set as UnitAngle_Factor
	 * DeltaD, TotalD max PulsePerRevolution in GetDeltaD_Units(), GetTotalD_Units()
	 * Multiplication overflow should wrap angle, maintain angle position correctness
	 */
	p_encoder->UnitAngularD_Factor			= 0xFFFFFFFFU/encoderCountsPerRevolution + 1;
//	p_encoder->UnitAngularD_DivisorShift 	= (32 - angleDataBits);

	/*
	 * Speed calc
	 *
	 * ((DeltaD * UnitT_Freq) << unitAngle_DataBits / (unitAngle_SensorResolution * DeltaT)) overflow
	 *
	 * 		Speed = (DeltaD * UnitD) / Correction * UnitT_Freq / Delta
	 * Use: speed = DeltaD * [UnitSpeed] / DeltaT = DeltaD * [UnitD * UnitT_Freq / Correction] / DeltaT
	 *
	 * most cases: UnitT_Freq > DeltaD
	 *
	 * e.g.
	 * Real: speed 	== DeltaD * UnitD * UnitT_Freq / Correction / DeltaT
	 * 				== 8000 * 429,497{sensorRes == 10k} * 20000 / 65536 / 1 == 1,048,576,660.16
	 *
	 * speed = (DeltaD * UnitD) / Correction * UnitT_Freq / DeltaT == 1,048,560,000
	 * speed = DeltaD * [UnitD * UnitT_Freq / Correction] / DeltaT == 1,048,576,000
	 */
	p_encoder->UnitAngularSpeed = MaxLeftShiftDivide(unitT_Freq, encoderCountsPerRevolution, CONFIG_ENCODER_ANGLE_RESOLUTION_BITS);

	p_encoder->IsUnitAngularSpeedOverflow = !p_encoder->UnitAngularSpeed;

	p_encoder->UnitInterpolateAngle = MaxLeftShiftDivide(unitT_Freq, pollingFreq*encoderCountsPerRevolution, CONFIG_ENCODER_ANGLE_RESOLUTION_BITS);
	//e.g. 48,761

	Encoder_Reset(p_encoder);
}


//reset volatile variables
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

	HAL_Encoder_WriteTimerCounter(p_encoder->p_HAL_Encoder, 0U);; /* reset for angularD */
	p_encoder->TimerCounterSaved = HAL_Encoder_ReadTimerCounter(p_encoder->p_HAL_Encoder);

	p_encoder->ExtendedDeltaTimerSaved = *p_encoder->p_ExtendedDeltaTimer;
}

//must use encoder init first
void Encoder_SetQuadratureMode(Encoder_T * p_encoder, bool isEnabled)
{
	p_encoder->IsQuadratureCounterEnabled = isEnabled;
}

/*!
 * isALeadBIncrement - Match to HAL/unused for deltaT Mode
 * isALeadBPositive - User runtime calibrate
 */
void Encoder_SetQuadratureDirectionCalibration(Encoder_T * p_encoder, bool isALeadBPositive)
{
	p_encoder-> IsALeadBDirectionPositive = isALeadBPositive; //for deltaT mode and UI output

	//	p_encoder->IsCounterIncrementDirectionPositive = !(isALeadBIncrement ^ isALeadBPositive);
//#ifdef CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_INCREMENT
//	p_encoder->IsCounterIncrementDirectionPositive = isALeadBPositive;
//#elif defined(CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_DECREMENT)
//	p_encoder->IsCounterIncrementDirectionPositive = !isALeadBPositive;
//#endif

}





//volatile uint32_t * Encoder_GetPtrDelta(Encoder_T *p_encoder)
//{
//	return &p_encoder->DeltaT;
//}

