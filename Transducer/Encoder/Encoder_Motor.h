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

#include "Encoder_DeltaT.h"
#include "Encoder_DeltaD.h"

/******************************************************************************/
/*!
	Position - Both Capture Modes
	todo freq auto DT mode, or split
*/
/******************************************************************************/
static inline uint16_t Encoder_Motor_GetMechanicalTheta(Encoder_T * p_encoder)
{
	return Encoder_GetAngle(p_encoder);
}

/*!
	Electrical Theta Angle, position Angle [Degree16s]
		=> MechanicalTheta * PolePairs % 65536
*/
static inline uint16_t Encoder_Motor_GetElectricalTheta(Encoder_T * p_encoder)
{
	uint16_t angle;
	// return Encoder_ConvertCounterDToAngle(p_encoder, (uint32_t)p_encoder->CounterD * (uint32_t)p_encoder->Params.MotorPolePairs);
#if 	defined(CONFIG_ENCODER_HW_DECODER)
	// update angle p_encoder->Angle32
#elif 	defined(CONFIG_ENCODER_HW_EMULATED)
	angle = ((_Encoder_GetAngle32(p_encoder) >> 6U) * p_encoder->Params.MotorPolePairs) >> 10U; /* MotorPolePairs less than 64 */
#endif
	return (p_encoder->Params.IsALeadBPositive == true) ? angle : 0 - angle;
}

/******************************************************************************/
/*!
	Capture DeltaD Mode
	CounterD Angle Functions
	DeltaD measures mechanical angle
*/
/******************************************************************************/
// static inline uint32_t Encoder_Motor_GetMechanicalDelta(Encoder_T * p_encoder)
// {
// 	return Encoder_DeltaD_GetDeltaAngle(p_encoder);
// }

/*!
	Electrical Delta Angle, change in Angle [Degree16s] per Control Period.
	MechanicalDelta * PolePairs
	Overflow threshold > 1 electrical cycle
*/
// static inline uint32_t Encoder_Motor_GetElectricalDelta(Encoder_T * p_encoder)
// {
// 	return Encoder_ConvertCounterDToAngle(p_encoder, p_encoder->DeltaD * p_encoder->Params.MotorPolePairs);
// }

/******************************************************************************/
/*!
	Capture DeltaT Mode
	Control Periods / Encoder Pulse > 1
	Interpolate Delta Angle
*/
/******************************************************************************/
/*!
	Hall => Use CountPerRevolution = polePairs*6. Capture on Commutation Step
*/
static inline uint32_t Encoder_Motor_InterpolateMechanicalDelta(Encoder_T * p_encoder, uint32_t pollingIndex)
{
	return Encoder_DeltaT_InterpolateAngle(p_encoder, pollingIndex);
}

static inline uint32_t Encoder_Motor_InterpolateElectricalDelta(Encoder_T * p_encoder, uint32_t pollingIndex)
{

	//todo check overflow boundaries
//	if (pollingIndex > UINT32_MAX / (p_encoder->UnitInterpolateAngle * p_encoder->Params.MotorPolePairs))
//	{
//		return Encoder_InterpolateAngle(p_encoder, pollingIndex) * p_encoder->Params.MotorPolePairs;
//	}
//	else
	{
		return Encoder_DeltaT_InterpolateAngle(p_encoder, pollingIndex * p_encoder->Params.MotorPolePairs);
	}
	// if(electricalDelta > 65536U / 6U) { electricalDelta = 65536U / 6U; }
}

static inline uint32_t Encoder_Motor_InterpolateHallDelta(Encoder_T * p_encoder, uint32_t pollingIndex)
{
	uint32_t electricalDelta;

	//todo check overflow boundaries
//	if (pollingIndex > UINT32_MAX / (p_encoder->UnitInterpolateAngle * p_encoder->Params.MotorPolePairs))
//	{
//		return Encoder_InterpolateAngle(p_encoder, pollingIndex) * p_encoder->Params.MotorPolePairs;
//	}
//	else
	{
		electricalDelta = Encoder_DeltaT_InterpolateAngle(p_encoder, pollingIndex * p_encoder->Params.MotorPolePairs);
	}
	if(electricalDelta > 65536U / 6U) { electricalDelta = 65536U / 6U; }
	return electricalDelta;
}

/*
	Control periods per encoder pulse, hall phase
*/
static inline uint32_t Encoder_Motor_ConvertMechanicalRpmToInterpolationFreq(Encoder_T * p_encoder, uint16_t mechRpm)
{
	return Encoder_DeltaT_ConvertRotationalSpeedToInterpolationCount_RPM(p_encoder, mechRpm);
}

static inline uint32_t Encoder_Motor_ConvertInterpolationFreqToMechanicalRpm(Encoder_T * p_encoder, uint16_t controlPeriods)
{
	return Encoder_DeltaT_ConvertInterpolationCountToRotationalSpeed_RPM(p_encoder, controlPeriods);
}

static inline uint32_t Encoder_Motor_GetInterpolationFreq(Encoder_T *p_encoder)
{
	return Encoder_DeltaT_GetInterpolationCount(p_encoder);
}

/******************************************************************************/
/*!
	Speed - Both Capture Modes
		todo freq auto DT mode, or split
*/
/******************************************************************************/
/* Mechanical Scalar Speed */
static inline uint32_t Encoder_Motor_GetScalarSpeed(Encoder_T * p_encoder)
{
	// return Encoder_GetScalarSpeed(p_encoder);
}

/*!
	Get Mechanical Angular Speed [Degree16s Per Second]
*/
static inline uint32_t Encoder_Motor_GetMechanicalSpeed(Encoder_T * p_encoder)
{
	// return Encoder_GetAngularSpeed(p_encoder);
}

static inline uint32_t Encoder_Motor_GetMechanicalRpm(Encoder_T * p_encoder)
{
	/* Max DeltaD = 447, for UnitSpeed = 160000, PolerPairs = 12 */
	// return Encoder_GetRotationalSpeed_RPM(p_encoder);
}

/*!
	Get Electrical Angular Speed [Degree16s Per Second]
 */
static inline uint32_t Encoder_Motor_GetElectricalSpeed(Encoder_T * p_encoder)
{
	return Encoder_Motor_GetMechanicalSpeed(p_encoder) * p_encoder->Params.MotorPolePairs;

	/* Max DeltaD = 2,236, for UnitSpeed = 160000, PolerPairs = 12 */
	//	return p_encoder->DeltaD * p_encoder->UnitAngularSpeed * p_encoder->Params.MotorPolePairs / p_encoder->DeltaT;
	//	return p_encoder->DeltaD * (p_encoder->UnitAngularSpeed  / p_encoder->DeltaT) * p_encoder->Params.MotorPolePairs
}

static inline uint32_t Encoder_Motor_GetElectricalRpm(Encoder_T * p_encoder)
{
	return Encoder_Motor_GetMechanicalRpm(p_encoder) * p_encoder->Params.MotorPolePairs;
//	return ((p_encoder->DeltaD * p_encoder->UnitAngularSpeed * p_encoder->Params.MotorPolePairs >> (32 - p_encoder->UnitAngularD_DivisorShift) - 6U) * 60U >> 6U)/ p_encoder->DeltaT;
}


/******************************************************************************/
/*!
	@brief 	Extern Declarations
*/
/*! @{ */
/******************************************************************************/
extern void Encoder_Motor_InitModeD(Encoder_T * p_encoder);
extern void Encoder_Motor_InitModeT(Encoder_T * p_encoder);
extern void Encoder_Motor_SetHallCountsPerRevolution(Encoder_T * p_encoder, uint8_t motorPolePairs);
extern void Encoder_Motor_SetPolePairs(Encoder_T * p_encoder, uint8_t motorPolePairs);
/******************************************************************************/
/*! @} */
/******************************************************************************/

#endif
