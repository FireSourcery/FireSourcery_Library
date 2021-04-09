/******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSoucery / The Firebrand Forge Inc

	This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).

	This program is free software: you can redistribute it and/or modify
	it under the terupdateInterval of the GNU General Public License as published by
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
	@brief 	Encoder module conventional function definitions
	@version V0
 */
/******************************************************************************/
#include "Encoder_Motor.h"
#include "Encoder.h"
#include "Private.h"

#include <stdint.h>
#include <stdbool.h>

/*!
	@param controlFreq pwm timer freq
 */
void Encoder_Motor_Init
(
	Encoder_T * p_encoder,
	const HAL_Encoder_T * p_encoderCounter,
	uint32_t encoderCounterMax,
	uint32_t controlFreq_Hz,				/* UnitT_Freq */
	uint32_t encoderDistancePerCount,		/* UnitLinearD */
	uint32_t encoderCountsPerRevolution,	/* UnitAngularD_Factor = [0xFFFFFFFFU/encoderCountsPerRevolution + 1] */
	uint8_t angleDataBits,					/* UnitAngularD_DivisorShift = [32 - unitAngle_DataBits] */
	uint8_t polePairs
)
{
	Encoder_Init
	(
		p_encoder,
		p_encoderCounter,
		encoderCounterMax,
		controlFreq_Hz,
		controlFreq_Hz,
		encoderDistancePerCount,
		encoderCountsPerRevolution,
		angleDataBits
	);

	p_encoder->PolePairs = polePairs;

	/*
	 * UnitD set to reflect mechanical angle => set for electrical angle to be derived value.
	 * Allow TotalD of at least PulsePerRevolution
	 *
	 * UnitD set to reflect electrical angle
	 * Allow CapturedD, TotalD of at least 1 electrical revolution
	 *
	 * 8192ppr
	 * 12 pole pairs
	 * 682 ticks per electrical cycle
	 */

	/*
	 * UnitSpeed = 160,000 => Max DeltaD = 26,843
	 * UnitSpeed = 1,600,000 => Max DeltaD = 2684
	 *
	 * 20k RPM => DeltaD = 136
	 * 10k RPM => DeltaD = 10000/60 * 8192 /20000 = 68
	 */
//	p_speed->UnitAngularSpeed = MaxLeftShiftDivide(controlFreq_Hz * nPolePairs, pulsePerRevolution, 16);

}

//// calculate distance per revolution using components
//uint32_t Speed_SetUnitGroundSpeed(Speed_T *p_speed, uint32_t wheelDiameter, uint32_t wheeltoMotorGearRatio)
//{
//	//p_speed->DistancePerRevolution = wheelDiameter * 314 / 100 / wheeltoMotorGearRatio;
//}
