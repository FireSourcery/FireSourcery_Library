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
	@brief 	Encoder module conventional function definitions
	@version V0
 */
/******************************************************************************/
#include "Encoder_Motor.h"
#include "Encoder.h"

#include <stdint.h>
#include <stdbool.h>

/*!
	Init to CaptureDeltaD Mode

	UnitAngularD set to reflect mechanical angle => set for electrical angle to be derived value.
	Allow TotalD of at least CountsPerRevolution

	@param controlFreq_Hz pwm timer freq
 */
void Encoder_Motor_InitCaptureCount(Encoder_T * p_encoder, uint32_t encoderCountsPerRevolution, uint32_t encoderDistancePerCount, uint8_t polePairs)
{
	Encoder_DeltaD_Init(p_encoder, encoderCountsPerRevolution, encoderDistancePerCount);
	p_encoder->PolePairs = polePairs;

	/*
	 * e.g.
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
}



/**************************************************************************/
/*!
	Init to CaptureDeltaT Mode
	e.g.   hall sensors  as speed encoder.

	Hall => Use CountPerRevolution = polePairs*6. Capture on Commutation Step

	// Capture Pulse Per Hall Cycle, 	PulsePerRevolution = 1 				=> Speed_GetRotationalSpeed_RPM reflects electrical speed
	// Capture Pulse Per Hall Cycle, 	PulsePerRevolution = PolePairs 		=> Speed_GetRotationalSpeed_RPM reflects mechanical speed
	// Capture Pulse Per Commutation, 	PulsePerRevolution = PolePairs*6 	=> Speed_GetRotationalSpeed_RPM reflects mechanical speed
	 *
  	deltaT = hall period = 1 electric revolution

	MECH_R = ELECTRIC_R / N_POLE_PAIRS
	ELECTRIC_R = 6 COMMUTATION_STEPS
	1 POLE_PAIRS -> 6 COMMUTATION_STEPS = ELECTRIC_R = MECH_R
	2 POLE_PAIRS -> 12 COMMUTATION_STEPS = 2 ELECTRIC_R = MECH_R

	Delta[s] 	= Delta[Ticks]/TimerFreq[Hz]
	ERPM		= 1[Delta]*60[s]/Delta[s]
	RPM 		= ERPM/PolePairs
	RPM 		= 1[Delta]*60[s]*TimerFreq[Hz]/PolePairs/Delta[Ticks]

	16-bit timer, Freq = 312,500 Hz: Peroid = 3.2 uS, Overflow 209,712 us, 209 ms, 28 rpm min for 10 pole pairs
	16-bit timer, Freq = 625,000 Hz: Peroid = 1.6 uS, Overflow 104,856 us, 104.5 ms, 56 rpm min for 10 pole pairs


 */
/**************************************************************************/
void Encoder_Motor_InitCaptureTime(Encoder_T * p_encoder, uint32_t encoderCountsPerRevolution, uint32_t encoderDistancePerCount, uint8_t polePairs)
{
	Encoder_DeltaT_Init(p_encoder, encoderCountsPerRevolution, encoderDistancePerCount);
	Encoder_DeltaT_InitExtendedTimer(p_encoder, 1000U);	//motor must use for stop poll
	p_encoder->PolePairs = polePairs;
}


//// calculate distance per revolution using components
//uint32_t Speed_SetUnitGroundSpeed(Speed_T *p_speed, uint32_t wheelDiameter, uint32_t wheeltoMotorGearRatio)
//{
//	//p_speed->DistancePerRevolution = wheelDiameter * 314 / 100 / wheeltoMotorGearRatio;
//}
