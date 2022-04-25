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
 */
void Encoder_Motor_InitCaptureCount(Encoder_T * p_encoder)
{
	Encoder_DeltaD_Init(p_encoder);
}

/**************************************************************************/
/*!
	Init to CaptureDeltaT Mode
	e.g.   hall sensors as speed encoder.

	Hall => Use CountPerRevolution = PolePairs*6. Capture on Commutation Step

	e.g.
	PolePairs = 10, TimerMax = 0xFFFF
	Freq = 312,500 Hz: Peroid = 3.2 uS, Overflow 209,712 us, 209 ms		=>	28 rpm min for 10 pole pairs
	Freq = 625,000 Hz: Peroid = 1.6 uS, Overflow 104,856 us, 104.5 ms 	=> 	56 rpm min for 10 pole pairs

	Capture Pulse Per Commutation, 	CPR = PolePairs*6 	=> GetRotationalSpeed reflects mechanical speed
 	Capture Pulse Per Commutation, 	CPR = PolePairs  	=> GetRotationalSpeed reflects electrical speed
	Capture Pulse Per Hall Cycle, 	CPR = PolePairs 	=> GetRotationalSpeed reflects mechanical speed
	Capture Pulse Per Hall Cycle, 	CPR = 1 			=> GetRotationalSpeed reflects electrical speed

	MECH_R = ELECTRIC_R / N_POLE_PAIRS
	ELECTRIC_R = 6 COMMUTATION_STEPS
	10 POLE_PAIRS -> 60 COMMUTATION_STEPS = 10 ELECTRIC_R = MECH_R

	Delta[s] 	= Delta[Ticks]/TimerFreq[Hz]
	ERPM		= 1[Delta]*60[s]/ECPR/Delta[s]
	MRPM 		= ERPM/PolePairs
	MRPM 		= 1[Delta]*60[s]*TimerFreq[Hz]/PolePairs/Delta[Ticks]

 */
/**************************************************************************/
void Encoder_Motor_InitCaptureTime(Encoder_T * p_encoder)
{
	Encoder_DeltaT_Init(p_encoder);
	Encoder_DeltaT_SetUnitConversion(p_encoder, p_encoder->Params.CountsPerRevolution, p_encoder->Params.DistancePerCount);
}

void Encoder_Motor_CaptureTime_SetPolePairs(Encoder_T * p_encoder, uint8_t motorPolePairs)
{
	p_encoder->Params.MotorPolePairs = motorPolePairs;
	Encoder_SetCountsPerRevolution(p_encoder, motorPolePairs * 6U);
}

void Encoder_Motor_SetLinearUnits(Encoder_T * p_encoder, uint32_t wheelDiameter, uint32_t wheeltoMotorRatio_Factor, uint32_t wheeltoMotorRatio_Divisor)
{
	uint32_t distancePerRevolution = wheelDiameter * wheeltoMotorRatio_Factor * 314 / 100 / wheeltoMotorRatio_Divisor;
	p_encoder->Params.DistancePerCount = distancePerRevolution / p_encoder->Params.CountsPerRevolution;
}
