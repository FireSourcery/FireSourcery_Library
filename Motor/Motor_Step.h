/**************************************************************************/
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
/**************************************************************************/
/**************************************************************************/
/*!
    @file 	Motor_Step.h
    @author FireSoucery
    @brief  Motor Six Step submodule.
    		Six Step commutation strategy. Polar PWM
    @version V0
*/
/**************************************************************************/
#ifndef MOTOR_STEP_H
#define MOTOR_STEP_H

#include "Motor.h"
#include "Config.h"

#include "Transducer/Hall/Hall.h"

//#include "Transducer/Encoder/Encoder_IO.h"
//#include "Transducer/Encoder/Encoder_Motor.h"
//#include "Transducer/Encoder/Encoder.h"

#include <stdint.h>
#include <stdbool.h>


static inline void Motor_Step_Commutate(Motor_T * p_motor, Motor_SectorId_T sectorId)
{
	switch(sectorId)
	{
	case MOTOR_SECTOR_ID_0:
		//set error
		break;
	case MOTOR_SECTOR_ID_1: //Phase AC
		Phase_Polar_ActivateAC(&p_motor->Phase);
		break;
	case MOTOR_SECTOR_ID_2:
	case MOTOR_SECTOR_ID_3:
	case MOTOR_SECTOR_ID_4:
	case MOTOR_SECTOR_ID_5:
	case MOTOR_SECTOR_ID_6:
	case MOTOR_SECTOR_ID_7:
	default: break;
	}

}


static inline void Motor_Step_HallDrive(Motor_T * p_motor)
{
	if (Hall_PollPhase_IO(&p_motor->Hall) //no hall isr
	{
//		Encoder_CaptureDeltaT_IO(&p_motor->Encoder); //using hall pin as encoder
//		PhaseID = Hall_GetPhaseID(&p_motor->Hall);

		Motor_Step_Commutate(p_motor, Hall_GetPhaseID(&p_motor->Hall));

//		if (SinusodialModulation)
//			{
//
//			ElectricalAngle = Angle(PhaseID);
//			}

	}
//	if (SinusodialModulation)
//		Waveform_ModulateAngleISR(&p_motor->Waveform1, Motor1.PWM, 0);
}


// Run in timer ISR. 30 degree after zero crossing.
//void Sensorless_CommutationISR(Sensorless_t * p_sensorless)
//{
//
//
//
//}
void Motor_Step_SensorlessDrive()
{

}


#endif
