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
	@file  	Encoder.c
	@author FireSourcery
	@brief 	Encoder module conventional function definitions
	@version V0
*/
/******************************************************************************/
#include "Encoder.h"

/*!
	highest precision (factor << targetShift / divisor) without overflow
*/
static uint32_t MaxLeftShiftDivide(uint32_t factor, uint32_t divisor, uint8_t targetShift)
{
	uint32_t result = 0;
	uint32_t shiftedValue = (1U << targetShift);

	if(shiftedValue % divisor == 0U) /* when divisor is a whole number factor of shifted. */
	{
		result = factor * (shiftedValue / divisor);
	}
	else
	{
		for(uint8_t maxShift = targetShift; maxShift > 0U; maxShift--)
		{
			if(factor <= (UINT32_MAX >> maxShift))
			{
				result = (factor << maxShift) / divisor;

				if(result <= (UINT32_MAX >> (targetShift - maxShift)))
				{
					result = result << (targetShift - maxShift);
				}
				else /* error, will overflow 32 bit even using ((factor << 0)/divisor) << leftShift */
				{
					result = 0U;
				}
				break;
			}
		}
	}

	return result;
}

void _Encoder_ResetUnitsAngular(Encoder_T * p_encoder)
{
	/*
	 * Angle Calc
	 * Angle = UnitAngle_Factor * DeltaD >> UnitAngle_Divisor
	 *
	 * uneven UnitD = unitAngle_DataBits/unitAngle_SensorResolution divide results in loss of precision
	 *
	 * UnitAngularD_ShiftDivisor == CONFIG_ENCODER_ANGLE_DEGREES_BITS
	 */
	p_encoder->UnitAngularD_Factor = 0xFFFFFFFFU / p_encoder->Params.CountsPerRevolution + 1U;

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
	p_encoder->UnitAngularSpeed = MaxLeftShiftDivide(p_encoder->UnitT_Freq, p_encoder->Params.CountsPerRevolution, CONFIG_ENCODER_ANGLE_DEGREES_BITS);

	//	//set boundary using speed ref
	//	if(p_encoder->UnitAngularSpeed > UINT32_MAX / ref)	{p_encoder->UnitAngularSpeed = 0U;}

	/*
	 * set boundary using 1 revolution minimum
	 */
	 //	if(unitTFreq > (UINT32_MAX >> CONFIG_ENCODER_ANGLE_DEGREES_BITS))
	 //	{
	 //		p_encoder->UnitAngularSpeed = 0U;
	 //	}
	 //	else
	 //	{
	 //		p_encoder->UnitAngularSpeed = (unitTFreq << CONFIG_ENCODER_ANGLE_DEGREES_BITS) / countsPerRevolution;
	 //	}

	p_encoder->UnitInterpolateAngle = MaxLeftShiftDivide(p_encoder->UnitT_Freq, p_encoder->CONFIG.POLLING_FREQ * p_encoder->Params.CountsPerRevolution, CONFIG_ENCODER_ANGLE_DEGREES_BITS);

	/*
	 *  UnitFrac16Speed = unitTFreq * 65535U * 60U / CountsPerRevolution / Frac16SpeedRef_Rpm;
	 */
	p_encoder->UnitFrac16Speed = MaxLeftShiftDivide(p_encoder->UnitT_Freq * 60U, p_encoder->Params.CountsPerRevolution * p_encoder->Params.Frac16SpeedRef_Rpm, 16U);
	// e.g. no truncate = 16,384,000
}

void _Encoder_ResetUnitsLinear(Encoder_T * p_encoder)
{
	p_encoder->UnitLinearD = p_encoder->Params.DistancePerCount;
	/*
		Overflow Caution

		DeltaT Mode, DeltaD == 1:  largeunitDeltaT_Freq, constraint on unitDeltaD
		Max unitDeltaD => UINT32_MAX / (unitDeltaT_Freq)
		e.g. unitDeltaD ~14,000, for 300,000 unitDeltaT_Freq

		DeltaD Mode, DeltaT == 1: constraint on unitDeltaD, and deltaD
		Max deltaD => UINT32_MAX / (unitDeltaD * unitDeltaT_Freq)
		e.g. deltaD ~14,000, for 300,000 (unitDeltaD * unitDeltaT_Freq)
	*/
	p_encoder->UnitLinearSpeed = p_encoder->Params.DistancePerCount * p_encoder->UnitT_Freq;
}

void _Encoder_ResetUnitsFrac16Speed(Encoder_T * p_encoder)
{
	/*
		UnitFrac16Speed = unitTFreq * 65535U * 60U / CountsPerRevolution / Frac16SpeedRef_Rpm;

		e.g.  unitTFreq = 625000, CountsPerRevolution = 60, Frac16SpeedRef_Rpm =
		 = 16,384,000 (no truncate)
	*/
	p_encoder->UnitFrac16Speed = MaxLeftShiftDivide(p_encoder->UnitT_Freq * 60U, p_encoder->Params.CountsPerRevolution * p_encoder->Params.Frac16SpeedRef_Rpm, 16U);
}


void _Encoder_ResetTimerFreq(Encoder_T * p_encoder)
{
	/*
		RPM * CPR / 60[Seconds] = CPS
		CPS = T_FREQ [Hz] / deltaT_ticks [timerticks/Count]
		RPM * CPR / 60[Seconds] = T_FREQ [Hz] / deltaT_ticks

		Error ~ 1, deltaT_ticks = 100
		=> T_FREQ/CPS >= 100, (CPS/T_FREQ <= .01)
			T_FREQ /(RPM * CPR / 60) >= 100
		eg. RPM = 10000
			T_FREQ >= 100*(10000RPM * CPR / 60)
			T_FREQ >= 16666 * CPR

		Min: deltaT_ticks = 65535
			RPM = (T_FREQ / CPR) * (60 / 65535)
			=> 15 ~= 16666 * (60 / 65535)

		DELTA_T_TIMER_FREQ ~= 10000 * CPR
			=> 10000RPM error ~1%
			=> RPM Min ~= 10RPM
	*/
	p_encoder->UnitT_Freq = HAL_Encoder_ConfigTimerCounterFreq(p_encoder, p_encoder->Params.CountsPerRevolution * 16666U);
	// p_encoder->ExtendedTimerThreshold = ((uint32_t)CONFIG_ENCODER_HW_TIMER_COUNTER_MAX + 1UL) * p_encoder->CONFIG.EXTENDED_TIMER_FREQ / p_encoder->UnitT_Freq;
	p_encoder->ExtendedTimerConversion = p_encoder->UnitT_Freq / p_encoder->CONFIG.EXTENDED_TIMER_FREQ;
}

/*
	reset runtime variables
	todo check necessity, split t/d mode
*/
void Encoder_Zero(Encoder_T * p_encoder)
{
	p_encoder->DeltaD = 1U;
	p_encoder->DeltaT = 1U;
	p_encoder->TotalD = 0U;
	p_encoder->TotalT = 0U;
	p_encoder->AngularD = 0U;
	p_encoder->SpeedSaved = 0U;
	p_encoder->DeltaSpeed = 0U;

	HAL_Encoder_ClearTimerCounterOverflow(p_encoder->CONFIG.P_HAL_ENCODER);
	HAL_Encoder_WriteTimerCounter(p_encoder->CONFIG.P_HAL_ENCODER, 0U);
}

void Encoder_SetQuadratureMode(Encoder_T * p_encoder, bool isEnabled)
{
	p_encoder->Params.IsQuadratureCaptureEnabled = isEnabled;
}

/*!
	isALeadBPositive - User runtime calibrate
*/
void Encoder_SetQuadratureDirectionCalibration(Encoder_T * p_encoder, bool isALeadBPositive)
{
	p_encoder->Params.IsALeadBPositive = isALeadBPositive;
}

void Encoder_SetCountsPerRevolution(Encoder_T * p_encoder, uint16_t countsPerRevolution)
{
	p_encoder->Params.CountsPerRevolution = countsPerRevolution;
	_Encoder_ResetUnitsAngular(p_encoder);
	_Encoder_ResetTimerFreq(p_encoder);
}

void Encoder_SetDistancePerCount(Encoder_T * p_encoder, uint16_t distancePerCount)
{
	p_encoder->Params.DistancePerCount = distancePerCount;
	_Encoder_ResetUnitsLinear(p_encoder);
}

void Encoder_SetScalarSpeedRef(Encoder_T * p_encoder, uint16_t speedRef)
{
	p_encoder->Params.Frac16SpeedRef_Rpm = speedRef;
	_Encoder_ResetUnitsFrac16Speed(p_encoder);
}
