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
	Perform highest precision (factor << targetShift / divisor) without overflow
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
				if(result <= (UINT32_MAX >> (targetShift - maxShift))) { result = result << (targetShift - maxShift); }
				else { result = 0U; } /* error, will overflow 32 bit even using ((factor << 0) / divisor) << leftShift */
				break;
			}
		}
	}

	return result;
}

void _Encoder_ResetUnitsAngular(Encoder_T * p_encoder)
{
	/*
		Angle = CounterD * (1 << DEGREES_BITS) / CountsPerRevolution
		Angle = CounterD * [((1 << DEGREES_BITS) << UnitAngularD_ShiftDivisor) / CountsPerRevolution] >> UnitAngularD_ShiftDivisor
			UnitAngularD_ShiftDivisor == (32 - DEGREES_BITS)
	*/
	p_encoder->UnitAngularD_Factor = 0xFFFFFFFFU / p_encoder->Params.CountsPerRevolution + 1U;

	/*
		AngularSpeed = DeltaD * [(1 << DEGREES_BITS) * UnitT_Freq / CountsPerRevolution] / DeltaT
			UnitAngularSpeed == [UnitAngularD_Factor * UnitT_Freq >> (32-DEGREES_BITS)]

		e.g.
			UnitAngularSpeed = 160,000 		{ DEGREES_BITS = 16, UnitT_Freq = 20000, CountsPerRevolution = 8192 }
			UnitAngularSpeed = 8,000 		{ DEGREES_BITS = 16, UnitT_Freq = 1000, CountsPerRevolution = 8192 }
			UnitAngularSpeed = 819,200,000 	{ DEGREES_BITS = 16, UnitT_Freq = 750000, CountsPerRevolution = 60 }
			UnitAngularSpeed = 131,072		{ DEGREES_BITS = 16, UnitT_Freq = 20000, CountsPerRevolution = 10000 }
	*/
	p_encoder->UnitAngularSpeed = MaxLeftShiftDivide(p_encoder->UnitT_Freq, p_encoder->Params.CountsPerRevolution, CONFIG_ENCODER_ANGLE_DEGREES_BITS);

	/*
		DeltaT side overflow boundary set by MaxLeftShiftDivide
	*/

	p_encoder->UnitInterpolateAngle = MaxLeftShiftDivide(p_encoder->UnitT_Freq, p_encoder->CONFIG.POLLING_FREQ * p_encoder->Params.CountsPerRevolution, CONFIG_ENCODER_ANGLE_DEGREES_BITS);
}

void _Encoder_ResetUnitsLinear(Encoder_T * p_encoder)
{
	/*
		SurfaceToEncoder Ratio <=> gearRatio_Factor/gearRatio_Divisor

		SurfaceSpeed_DistancePerHour = (RPM * gearRatio_Divisor/gearRatio_Factor) * surfaceDiameter_Mm * 314 / 100 * 60 / DistanceUserConversion
		RPM = 60 *(deltaD_Ticks * UnitT_Freq) / (CountsPerRevolution * deltaT_Ticks)

		SurfaceSpeed_UserDPerSecond = (deltaD_Ticks * UnitT_Freq * gearRatio_Divisor * surfaceDiameter_Mm * 314) / (CountsPerRevolution * deltaT_Ticks * gearRatio_Factor * 100))
		SurfaceSpeed_UserDPerSecond = deltaD_Ticks * [UnitT_Freq * gearRatio_Divisor * surfaceDiameter_Mm * 314] / (deltaT_Ticks * [CountsPerRevolution * gearRatio_Factor * 100])
		SurfaceSpeed_UserDPerSecond = deltaD_Ticks * [[UnitT_Freq * gearRatio_Divisor * surfaceDiameter_Mm * 314] / [CountsPerRevolution * gearRatio_Factor * 100]] / deltaT_Ticks

		SurfaceSpeed_DistancePerHour = ((60 * (deltaD_Ticks * UnitT_Freq) * gearRatio_Divisor * surfaceDiameter_Mm * 314 * 60) / (CountsPerRevolution * deltaT_Ticks * gearRatio_Factor * 100 * DistanceUserConversion ))

		UserD => mm
		DistanceUserConversion:
		mph => 1,609,344
		kmh => 1,000,000

		e.g.
			UnitT_Freq = 1000, GearRatio = 8, SurfaceDiameter = 20in, 508mm, CountsPerRevolution = 8192
			UnitLinearSpeed = 24.3395996094
			1000 RPM => 7.44 mph, 11.97352 kmh
				=> DeltaD = 136.533333333
			mph = 136.533333333 * 24.3395996094 * 60 * 60 / 1609344 = 7.4337121212
			mph = 136 * 24 * 60 * 60 / 1609344 = 7.3
			kmh = 136 * 24 * 60 * 60 / 1000000 = 11.7504

	*/
	p_encoder->UnitLinearSpeed = (p_encoder->UnitT_Freq * p_encoder->Params.GearRatio_Divisor * p_encoder->Params.SurfaceDiameter * 314) / (p_encoder->Params.CountsPerRevolution * p_encoder->Params.GearRatio_Factor * 100);

	// DistancePerRevolution = (gearRatio_Divisor * surfaceDiameter_Mm * 314 / gearRatio_Factor * 100)
	// p_encoder->UnitLinearD_Factor = p_encoder->Params.DistancePerRevolution;
	// p_encoder->UnitLinearSpeed = p_encoder->Params.DistancePerRevolution * p_encoder->UnitT_Freq;
}

void _Encoder_ResetUnitsFrac16Speed(Encoder_T * p_encoder)
{
	/*
		UnitFrac16Speed = unitTFreq * 65535U * 60U / CountsPerRevolution / Frac16SpeedRef_Rpm

		e.g.  unitTFreq = 625000, CountsPerRevolution = 60,
			Frac16SpeedRef_Rpm = 2500 => 16,384,000
			Frac16SpeedRef_Rpm = 10000 => 4,095,937.5
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
	p_encoder->UnitT_Freq = HAL_Encoder_ConfigTimerCounterFreq(p_encoder->CONFIG.P_HAL_ENCODER, p_encoder->Params.CountsPerRevolution * 16666U);
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
	_Encoder_ResetUnitsFrac16Speed(p_encoder);
}

void Encoder_SetFrac16SpeedRef(Encoder_T * p_encoder, uint16_t speedRef)
{
	p_encoder->Params.Frac16SpeedRef_Rpm = speedRef;
	_Encoder_ResetUnitsFrac16Speed(p_encoder);
}

void Encoder_SetDistancePerRevolution(Encoder_T * p_encoder, uint16_t distancePerRevolution)
{
	p_encoder->Params.DistancePerRevolution = distancePerRevolution;
	_Encoder_ResetUnitsLinear(p_encoder);
}

void Encoder_Motor_SetSurfaceRatio(Encoder_T * p_encoder, uint32_t surfaceDiameter, uint32_t surfaceToMotorRatio_Factor, uint32_t surfaceToMotorRatio_Divisor)
{
	p_encoder->Params.SurfaceDiameter = surfaceDiameter;
	p_encoder->Params.GearRatio_Factor = surfaceToMotorRatio_Factor;
	p_encoder->Params.GearRatio_Divisor = surfaceToMotorRatio_Divisor;
	_Encoder_ResetUnitsLinear(p_encoder);

	// Encoder_SetDistancePerRevolution(p_encoder, surfaceDiameter * surfaceToMotorRatio_Divisor * 314 / (100 * surfaceToMotorRatio_Factor));
}

void Encoder_Motor_SetGroundRatio_US(Encoder_T * p_encoder, uint32_t wheelDiameter_Inch10, uint32_t wheelToMotorRatio_Factor, uint32_t wheelToMotorRatio_Divisor)
{
	Encoder_Motor_SetSurfaceRatio(p_encoder, wheelDiameter_Inch10 * 254 / 100, wheelToMotorRatio_Factor, wheelToMotorRatio_Divisor);
}

void Encoder_Motor_SetGroundRatio_Metric(Encoder_T * p_encoder, uint32_t wheelDiameter_Mm, uint32_t wheelToMotorRatio_Factor, uint32_t wheelToMotorRatio_Divisor)
{
	Encoder_Motor_SetSurfaceRatio(p_encoder, wheelDiameter_Mm, wheelToMotorRatio_Factor, wheelToMotorRatio_Divisor);
}




