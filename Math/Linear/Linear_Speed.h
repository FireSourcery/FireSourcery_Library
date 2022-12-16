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
	@file 	Linear_Speed.h
	@author FireSourcery
	@brief
	@version V0
*/
/******************************************************************************/
#ifndef LINEAR_SPEED_H
#define LINEAR_SPEED_H

#include "Linear_Frac16.h"
#include <stdbool.h>

static inline uint32_t speed_angle16torpm(uint16_t angle16, uint32_t sampleFreq)
{
	uint32_t rpm;
	if(angle16 < (UINT32_MAX / sampleFreq * 60U)) 	{ rpm = (angle16 * sampleFreq * 60U >> 16U); }
	else 											{ rpm = (angle16 * sampleFreq >> 16U) * 60U; }
	return rpm;
}

// if(rpm > (UINT32_MAX >> 16U)) { angle = (1U << 16U) / 60U * rpm / sampleFreq; }
static inline uint32_t speed_rpmtoangle16(uint16_t rpm, uint32_t sampleFreq)
{
	return (rpm << 16U) / (60U * sampleFreq);
}
/******************************************************************************/
/*!
	Speed position conversions
	Independent from Delta Captures
	Use for integrate Speed to position
*/
/******************************************************************************/
/*
	Overflow caution:
	rpm < 65536
	Alternatively, 	angle = (1U << CONFIG_ENCODER_ANGLE_DEGREES_BITS) / 60U * rpm / sampleFreq;
*/
// static inline uint32_t Encoder_ConvertRotationalSpeedToAngle_RPM(uint32_t rpm, uint32_t sampleFreq)
// {
// 	return (rpm << CONFIG_ENCODER_ANGLE_DEGREES_BITS) / (60U * sampleFreq);
// }

// static inline uint32_t Encoder_ConvertAngleToRotationalSpeed_RPM(uint32_t angle_UserDegrees, uint32_t sampleFreq)
// {
// 	return (angle_UserDegrees * sampleFreq >> CONFIG_ENCODER_ANGLE_DEGREES_BITS) * 60U;
// }

// /*
// 	D/T Common
// 	delta angle for speed position integration => POLLING_FREQ e.g. 20000Hz
// */
// static inline uint32_t Encoder_ConvertRotationalSpeedToControlAngle_RPM(Encoder_T * p_encoder, uint32_t rpm)
// {
// 	return Encoder_ConvertRotationalSpeedToAngle_RPM(rpm, p_encoder->CONFIG.POLLING_FREQ);
// }

// static inline uint32_t Encoder_ConvertControlAngleToRotationalSpeed_RPM(Encoder_T * p_encoder, uint32_t angle_UserDegrees)
// {
// 	return Encoder_ConvertAngleToRotationalSpeed_RPM(angle_UserDegrees, p_encoder->CONFIG.POLLING_FREQ);
// }

// /*
// 	D only
// 	Avg Speed sampling
// */
// static inline uint32_t Encoder_ConvertRotationalSpeedToSampleAngle_RPM(Encoder_T * p_encoder, uint32_t rpm)
// {
// 	return Encoder_ConvertRotationalSpeedToAngle_RPM(rpm, p_encoder->CONFIG.D_SPEED_FREQ);
// }

// static inline uint32_t Encoder_ConvertSampleAngleToRotationalSpeed_RPM(Encoder_T * p_encoder, uint32_t angle_UserDegrees)
// {
// 	return Encoder_ConvertAngleToRotationalSpeed_RPM(angle_UserDegrees, p_encoder->CONFIG.D_SPEED_FREQ);
// }

/******************************************************************************/
/*!
	Integrate  speed to position, use ticks
*/
/******************************************************************************/
/*
	10k rpm, 20kHz => 546 ~= .83 percent of revolution
*/
// static inline uint32_t Encoder_Motor_ConvertMechanicalRpmToMechanicalDelta(Encoder_T * p_encoder, uint16_t mechRpm)
// {
// 	return Encoder_ConvertRotationalSpeedToControlAngle_RPM(p_encoder, mechRpm);
// }

// /*!
// 	Convert Mechanical Angular Speed [Revolutions per Minute] to Electrical Delta [Degree16s Per Control Period]

// 	Skips conversion through DeltaD
// 	angle = (rpm * MotorPolePairs << CONFIG_ENCODER_ANGLE_DEGREES_BITS) / (60U * controlFreq)
// */
// static inline uint32_t Encoder_Motor_ConvertMechanicalRpmToElectricalDelta(Encoder_T * p_encoder, uint16_t mechRpm)
// {
// 	return Encoder_ConvertRotationalSpeedToControlAngle_RPM(p_encoder, mechRpm * p_encoder->Params.MotorPolePairs);
// }

//handle inside module or outside?
//static inline void Encoder_Motor_IntegrateMechanicalRpm(Encoder_T * p_encoder, int32_t * p_theta, uint16_t speed_Rpm)
//{
//	uint32_t electricalDelta = Encoder_Motor_ConvertMechanicalRpmToElectricalDelta(p_encoder, speed_Rpm);
//	*p_theta += electricalDelta;
//}

//static inline uint32_t Encoder_Motor_ConvertMechanicalRpmToDeltaD(Encoder_T * p_encoder, uint16_t mechRpm)
//{
//	return Encoder_ConvertRotationalSpeedToDeltaD_RPM(p_encoder, mechRpm);
//}
//
//static inline uint32_t Encoder_Motor_ConvertDeltaDToMechanicalRpm(Encoder_T * p_encoder, uint16_t deltaD_Ticks)
//{
//	return Encoder_ConvertDeltaDToRotationalSpeed_RPM(p_encoder, deltaD_Ticks);
//}


static inline int32_t Linear_Speed_CalcAngleRpm(const Linear_T * p_linear, uint16_t angle)
{
	return Linear_Frac16_Units(p_linear, angle);
}

static inline int32_t Linear_Speed_CalcRpmAngle(const Linear_T * p_linear, uint32_t rpm)
{
	return Linear_Frac16_InvUnits(p_linear, rpm);
}

static inline int32_t Linear_Speed_CalcAngleRpmFrac16(const Linear_T * p_linear, int16_t angle)
{
	return Linear_Frac16(p_linear, angle);
}

static inline int32_t Linear_Speed_CalcRpmFrac16Angle(const Linear_T * p_linear, uint32_t rpm)
{
	return Linear_Frac16_Inv(p_linear, rpm);
}

extern void Linear_Speed_InitAngleRpm(Linear_T * p_linear, uint32_t sampleFreq, uint8_t angleBits, uint16_t speedRef_Rpm);
extern void Linear_Speed_InitElectricalAngleRpm(Linear_T * p_linear, uint32_t sampleFreq, uint8_t angleBits, uint8_t polePairs, uint16_t speedRef_Rpm);

#endif
