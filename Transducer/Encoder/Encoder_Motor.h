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
	@file 	Encoder_Motor.h
	@author FireSourcery
	@brief 	Encoder + pole pairs, electrical angle calculation
			Capture DeltaD, fixed DeltaT
			Angle defined to be  16 bits
	@version V0
 */
/******************************************************************************/
#ifndef ENCODER_MOTOR_H
#define ENCODER_MOTOR_H

#include "Encoder.h"

#include <stdint.h>
#include <stdbool.h>
/******************************************************************************/
/*!
	DeltaD/TotalD Function - Essentially Speed_GetDeltaD_Units
 */
/******************************************************************************/

/*!
	Electrical Delta Angle, change in Angle [Degree16s] per Control Period.
 */
static inline uint32_t Encoder_Motor_GetElectricalDelta(Encoder_T * p_encoder)
{
	/*
	 * Multiply by PolePairs first can be more accurate in cases of uneven encoder division.
	 * Encoder CPR == 8192, PolePairs == 10: DeltaD is Max is 1 electrical cycle, ~682. Multiplication overflow should loop angle maintain correct angular position.
	 */
	return (p_encoder->DeltaD * p_encoder->UnitAngularD_Factor * p_encoder->PolePairs) >> p_encoder->UnitAngularD_DivisorShift;

	/*
	 * Capture DeltaT Mode
	 * [TimerFreq * 0x10000 * p_encoder->PolePairs / p_encoder->PulsePerRevolution] / p_encoder->DeltaT;
	 */
}

/*!
	Electrical Theta Angle, position Angle [Degree16s]
 */
static inline uint32_t Encoder_Motor_GetElectricalTheta(Encoder_T * p_encoder)
{
	/*
	 * Multiply by PolePairs first can be more accurate in cases of uneven encoder division.
	 * Encoder CPR == 8192, PolePairs == 10: TotalAngularD [0:8192], overflow at 1 electrical cycle, ~819. Multiplication overflow should loop angle, and maintain correct angular position.
	 */
	return (p_encoder->AngularD * p_encoder->UnitAngularD_Factor * p_encoder->PolePairs) >> p_encoder->UnitAngularD_DivisorShift;
}

static inline uint32_t Encoder_Motor_GetMechanicalDelta(Encoder_T * p_encoder)
{
	return Encoder_GetDeltaAngle(p_encoder);
}

static inline uint32_t Encoder_Motor_GetMechanicalTheta(Encoder_T * p_encoder)
{
	return Encoder_GetTotalAngle(p_encoder);
}

/*!
	DeltaD measures mechanical angle
 */
static inline uint32_t Encoder_Motor_ConvertDeltaDToElectricalAngle(Encoder_T * p_encoder, uint32_t deltaD_Ticks)
{
	return ((deltaD_Ticks * p_encoder->UnitAngularD_Factor * p_encoder->PolePairs) >> p_encoder->UnitAngularD_DivisorShift);
}

/*!
	DeltaD measures mechanical angle
 */
static inline uint32_t Encoder_Motor_ConvertElectricalAngleToDeltaD(Encoder_T * p_encoder, uint16_t electricalAngle_Degrees16)
{
	return (electricalAngle_Degrees16 << p_encoder->UnitAngularD_DivisorShift) / p_encoder->PolePairs / p_encoder->UnitAngularD_Factor;
}

/******************************************************************************/
/*!
	Speed - can be updated with DeltaT for both modes
 */
/******************************************************************************/
/*!
	Get Electrical Angular Speed [Degree16s Per Second]
 */
static inline uint32_t Encoder_Motor_GetElectricalSpeed(Encoder_T * p_encoder)
{
	/* Max DeltaD = 2,236, for UnitSpeed = 160000, PolerPairs = 12 */
	return p_encoder->DeltaD * p_encoder->UnitAngularSpeed * p_encoder->PolePairs; //DeltaT mode will not use, skip div p_encoder->Delta
}
static inline uint32_t Encoder_Motor_GetElectricalRpm(Encoder_T * p_encoder)
{
	return (p_encoder->DeltaD * p_encoder->UnitAngularSpeed * 60 >> (32 - p_encoder->UnitAngularD_DivisorShift)) * p_encoder->PolePairs;
}

/*!
	Get Mechanical Angular Speed [Degree16s Per Second]
 */
static inline uint32_t Encoder_Motor_GetMechanicalSpeed(Encoder_T * p_encoder)
{
	return Encoder_GetAngularSpeed(p_encoder);
//	return p_encoder->DeltaD * p_encoder->UnitAngularSpeed;
}

static inline uint32_t Encoder_Motor_GetMechanicalRpm(Encoder_T * p_encoder)
{
	/* Max DeltaD = 447, for UnitSpeed = 160000, PolerPairs = 12 */
	return Encoder_GetRotationalSpeed_RPM(p_encoder);
	//return p_encoder->DeltaD * p_encoder->UnitAngularSpeed * 60 >> (32 - p_encoder->UnitAngularD_DivisorShift) ;
}

/*
	When UnitAngularSpeed is too high. Large timer freq (> 16 bits), angle res, small countsPerRevolution
*/
static inline uint32_t Encoder_Motor_GetMechanicalSpeed_Coarse(Encoder_T * p_encoder)
{
	 return (p_encoder->DeltaD * p_encoder->UnitT_Freq) / (p_encoder->EncoderResolution * p_encoder->DeltaT) << (32U - p_encoder->UnitAngularD_DivisorShift);
}

static inline uint32_t Encoder_Motor_GetMechanicalRpm_Coarse(Encoder_T * p_encoder)
{
//	UnitRotationalSpeed = (p_encoder->UnitT_Freq) / (p_encoder->EncoderResolution)
	return (p_encoder->DeltaD * p_encoder->UnitT_Freq * 60) / (p_encoder->EncoderResolution * p_encoder->DeltaT);
}

static inline uint32_t Encoder_Motor_ConvertMechanicalRpmToDeltaD(Encoder_T * p_encoder, uint16_t mechRpm)
{
	return Encoder_ConvertRotationalSpeedToDeltaD_RPM(p_encoder, mechRpm);
}

static inline uint32_t Encoder_Motor_ConvertDeltaDToMechanicalRpm(Encoder_T * p_encoder, uint16_t deltaD_Ticks)
{
	return Encoder_ConvertDeltaDToRotationalSpeed_RPM(p_encoder, deltaD_Ticks);
}

/*!
	Convert Mechanical Angular Speed [Revolutions per Minute] to Electrical Delta [Degree16s Per Control Period]

	Skips conversion through DeltaD
 */
static inline uint32_t Encoder_Motor_ConvertMechanicalRpmToElectricalDelta(Encoder_T * p_encoder, uint16_t mechRpm)
{
	return (mechRpm << (32U - p_encoder->UnitAngularD_DivisorShift)) / 60U * p_encoder->PolePairs / p_encoder->PollingFreq;
}

static inline uint32_t Encoder_Motor_ConvertElectricalRpmToElectricalDelta(Encoder_T * p_encoder, uint16_t elecRpm)
{
	return (elecRpm << (32U - p_encoder->UnitAngularD_DivisorShift)) / 60U / p_encoder->PollingFreq;
}

/*
	Control Period per Delta Period capture
	CaptureDeltaT only, CaptureDeltaD returns 1
 */
static inline uint32_t Encoder_Motor_GetControlPeriods(Encoder_T *p_encoder)
{
	return p_encoder->PollingFreq * p_encoder->DeltaT / p_encoder->UnitT_Freq; //	return p_encoder->PollingFreq / (p_encoder->UnitT_Freq / p_encoder->DeltaT);
}

/*
	Control periods per tick
 */
static inline uint32_t Encoder_Motor_ConvertMechanicalRpmToControlPeriods(Encoder_T * p_encoder, uint16_t mechRpm)
{
//	return Encoder_ConvertRotationalSpeedToDeltaT_RPM(p_encoder, mechRpm);
	return p_encoder->PollingFreq * 60 / (p_encoder->EncoderResolution * mechRpm);
}

static inline uint32_t Encoder_Motor_ConvertControlPeriodsToMechanicalRpm(Encoder_T * p_encoder, uint16_t controlPeriods)
{
	return p_encoder->PollingFreq * 60 / (p_encoder->EncoderResolution * controlPeriods);
}

/*!
	Estimate speed in between ticks

	AngleControlIndex * 1(DeltaD) * AngleSize * UnitT_Freq / DeltaT / PollingFreq;

	angle index ranges from 0 to (AngleControlCyclesPerDeltaCycle)
 */
static inline uint32_t Encoder_Motor_InterpolateElectricalDelta(Encoder_T * p_encoder, uint32_t index)
{
	return index * p_encoder->UnitInterpolateAngle / p_encoder->DeltaT; //	AngleControlIndex * [AngleSize[0x10000] * (UnitDeltaT_Freq / PollingFreq)] / DeltaT;


	//UnitT_Freq * AngleData(0x10000) is too large
//	return index * Encoder_GetAngularSpeed(p_encoder) / p_encoder->PollingFreq;
//		[ DeltaD * (UnitT_Freq * 0x10000 *  PolePairs /  PulsePerRevolution) /  DeltaT] / PollingFreq;
//		[1 * (UnitT_Freq * 0x10000 * 1 / 1) /  DeltaT] / PollingFreq;
}

//static inline uint32_t Encoder_Motor_InterpolateAngle_Slope(Encoder_T * p_encoder, uint32_t index, uint32_t domain)
//{
//	return Encoder_InterpolateAngle(p_encoder, index, domain);
//}
//
//static inline uint32_t Encoder_Motor_InterpolateElectricalDelta(Encoder_T * p_encoder, uint32_t index)
//{
//	return Encoder_InterpolateAngle_TimeDomain(p_encoder, index);
//}

/*

 */
static inline int32_t Encoder_Motor_GetGroundSpeed(Encoder_T * p_encoder)
{
	return Encoder_GetLinearSpeed(p_encoder);
}



/*!
	Integrates speed to position
 */
//static inline void Encoder_Motor_IntegrateSpeed(Encoder_T * p_encoder, int32_t * p_position, uint16_t speed)
//{
//	*p_position += speed;
//}
//handle inside module or outside?
//static inline void Encoder_Motor_IntegrateSpeed_RPM(Encoder_T * p_encoder, int32_t * p_theta, uint16_t speed_Rpm)
//{
//	uint32_t electricalDelta = Encoder_Motor_ConvertMechanicalRpmToElectricalDelta(p_encoder, speed_Rpm);
//	*p_theta += electricalDelta;
////	Encoder_Motor_IntegrateSpeed(p_encoder, p_theta, electricalDelta);
//}


//extern void Encoder_Motor_Init
//(
//	Encoder_T * p_encoder,
//	const HAL_Encoder_T * p_HAL_encoder,
//	uint32_t controlFreq_Hz,				/* UnitT_Freq */
//	uint32_t encoderDistancePerCount,		/* UnitLinearD */
//	uint32_t encoderCountsPerRevolution,	/* UnitAngularD_Factor = [0xFFFFFFFFU/encoderCountsPerRevolution + 1] */
//	uint8_t angleDataBits,					/* UnitAngularD_DivisorShift = [32 - unitAngle_DataBits] */
//	uint8_t polePairs
//);

#endif
