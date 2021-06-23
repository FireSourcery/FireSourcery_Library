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
#ifndef MOTOR_SIXSTEP_H
#define MOTOR_SIXSTEP_H

#include "Motor.h"
#include "Config.h"

#include "Transducer/Hall/Hall.h"
#include "Transducer/BEMF/BEMF.h"
#include "Transducer/Phase/Phase.h"

#include "Transducer/Encoder/Encoder_IO.h"
#include "Transducer/Encoder/Encoder_Motor.h"
#include "Transducer/Encoder/Encoder.h"

#include "Math/Linear/Linear_Voltage.h"
#include "Math/Linear/Linear_ADC.h"
#include "Math/Linear/Linear_Ramp.h"
#include "Math/Linear/Linear.h"

#include <stdint.h>
#include <stdbool.h>





static inline void Motor_SixStep_SetBrakeRegenOptimal(Motor_T * p_motor)
{
	p_motor->RampCmd = Linear_Voltage_CalcUnsignedFraction16(&p_motor->UnitVabc, BEMF_GetVPhase(&p_motor->Bemf)) / 2U;
}

static inline void Motor_SixStep_SetBrakeRegenProportional(Motor_T * p_motor, uint16_t intensity)
{
	p_motor->RampCmd = Linear_Voltage_CalcUnsignedFraction16(&p_motor->UnitVabc, BEMF_GetVPhase(&p_motor->Bemf)) * (65536U - intensity) >> 16U;

	//braking 0% -> pwm 100% of back emf;
	//braking 10% -> pwm 90% of back emf;
	//braking 50% -> pwm 50% of back emf;
	//braking 90% -> pwm 10% of back emf;
}

static inline void Motor_SixStep_SetBrakeRegenScalar(Motor_T * p_motor, uint16_t intensity)
{
	p_motor->RampCmd = Linear_Voltage_CalcUnsignedFraction16(&p_motor->UnitVabc, BEMF_GetVPhase(&p_motor->Bemf)) * (32768U - ((intensity + 32768U) / p_motor->Parameters.BrakeCoeffcient)) >> 16U;

	// e.g
	// RegenCoeffcientInput = 4, RegenCoeffcient -> fract16(1,4)

	//braking 0% -> pwm 62.5% of back emf;
	//braking 10% -> pwm 60% of back emf;
	//braking 50% -> pwm 50% of back emf;
	//braking 90% -> pwm 40% of back emf;
	//braking 100% -> pwm 37.5% of back emf;
}

static inline void Motor_SixStep_ProcRamp(Motor_T * p_motor)
{
	switch (p_motor->Parameters.BrakeMode)
	{
	case MOTOR_BRAKE_MODE_SCALAR:
		Motor_ProcRamp(p_motor);
		break;
	case MOTOR_BRAKE_MODE_REGEN_OPTIMAL:
		Motor_SixStep_SetBrakeRegenOptimal(p_motor);
		break;
	case MOTOR_BRAKE_MODE_REGEN_PROPRTIONAL:
		Motor_SixStep_SetBrakeRegenProportional(p_motor, p_motor->UserCmd);
		break;
	case MOTOR_BRAKE_MODE_REGEN_SCALAR:
		Motor_SixStep_SetBrakeRegenScalar(p_motor, p_motor->UserCmd);
		break;
	default:
		break;
	}

}

static inline void Motor_SixStep_ProcControlVariable(Motor_T * p_motor)
{
//	if (brake )
//	{
//	Motor_SixStep_SetRegenBrake(p_motor);
//	}
//	else
//	{

	Motor_ProcRamp(p_motor);
//	}

	Motor_ProcControlVariable(p_motor);
}


static inline void Motor_SixStep_ActivateSector(Motor_T * p_motor, Motor_SectorId_T sectorId)
{
	Phase_Polar_SetDutyCyle(&p_motor->Phase, p_motor->VPwm);

	//hall: 	phase
	//openloop: phase, sector
	//bemf: 	phase, sector, bemfmap

	switch (sectorId)
	{
	case MOTOR_SECTOR_ID_0:
		//set error
		break;
	case MOTOR_SECTOR_ID_1: //Phase AC
		Phase_Polar_ActivateAC(&p_motor->Phase);
		if (p_motor->Direction == MOTOR_DIRECTION_CCW) 	{p_motor->NextSector = MOTOR_SECTOR_ID_2; BEMF_MapCcwPhaseAC_IO(&p_motor->Bemf);}
		else											{p_motor->NextSector = MOTOR_SECTOR_ID_6; BEMF_MapCwPhaseAC_IO(&p_motor->Bemf);}

		break;
	case MOTOR_SECTOR_ID_2:
		Phase_Polar_ActivateBC(&p_motor->Phase);
		if (p_motor->Direction == MOTOR_DIRECTION_CCW) 	{p_motor->NextSector = MOTOR_SECTOR_ID_3; BEMF_MapCcwPhaseBC_IO(&p_motor->Bemf);}
		else											{p_motor->NextSector = MOTOR_SECTOR_ID_1; BEMF_MapCwPhaseBC_IO(&p_motor->Bemf);}

		break;
	case MOTOR_SECTOR_ID_3:
		Phase_Polar_ActivateBA(&p_motor->Phase);
		if (p_motor->Direction == MOTOR_DIRECTION_CCW) 	{p_motor->NextSector = MOTOR_SECTOR_ID_4; BEMF_MapCcwPhaseBA_IO(&p_motor->Bemf);}
		else											{p_motor->NextSector = MOTOR_SECTOR_ID_2; BEMF_MapCwPhaseBA_IO(&p_motor->Bemf);}

		break;
	case MOTOR_SECTOR_ID_4:
		Phase_Polar_ActivateCA(&p_motor->Phase);
		if (p_motor->Direction == MOTOR_DIRECTION_CCW) 	{p_motor->NextSector = MOTOR_SECTOR_ID_5; BEMF_MapCcwPhaseCA_IO(&p_motor->Bemf);}
		else											{p_motor->NextSector = MOTOR_SECTOR_ID_3; BEMF_MapCwPhaseCA_IO(&p_motor->Bemf);}

		break;
	case MOTOR_SECTOR_ID_5:
		Phase_Polar_ActivateCB(&p_motor->Phase);
		if (p_motor->Direction == MOTOR_DIRECTION_CCW) 	{p_motor->NextSector = MOTOR_SECTOR_ID_6; BEMF_MapCcwPhaseCA_IO(&p_motor->Bemf);}
		else											{p_motor->NextSector = MOTOR_SECTOR_ID_4; BEMF_MapCwPhaseCA_IO(&p_motor->Bemf);}

		break;
	case MOTOR_SECTOR_ID_6:
		Phase_Polar_ActivateAB(&p_motor->Phase);
		if (p_motor->Direction == MOTOR_DIRECTION_CCW) 	{p_motor->NextSector = MOTOR_SECTOR_ID_1; BEMF_MapCcwPhaseAB_IO(&p_motor->Bemf);}
		else											{p_motor->NextSector = MOTOR_SECTOR_ID_5; BEMF_MapCwPhaseAB_IO(&p_motor->Bemf);}

		break;
	case MOTOR_SECTOR_ID_7:
		//set error
		break;
	default:
		break;
	}

}

///*
// * Seperate state for OpenLoop start? maps to different inputs
// */
//static inline void Motor_SixStep_ProcOpenLoop(Motor_T *p_motor)
//{
//
//}


//state loop in 20khz pwm thread
static inline void Motor_SixStep_ProcSectorControl(Motor_T * p_motor)
{
	bool commutation = false;
	bool updatePwm = false;
	switch (p_motor->Parameters.SensorMode)
	{
	case MOTOR_SENSOR_MODE_OPEN_LOOP:
		if(Thread_PollTimerCompletePeriodic(&p_motor->ControlTimerThread) == true)
		{
			p_motor->Openloop_RPM = Linear_Ramp_CalcTarget_IncIndex(&p_motor->OpenLoopRamp, &p_motor->OpenLoopRampIndex, p_motor->CommutationPeriodCmd);
			p_motor->CommutationPeriodCmd = Encoder_Motor_ConvertMechanicalRpmToControlPeriods(&p_motor->Encoder, p_motor->Openloop_RPM );
			Thread_SetTimerPeriod(&p_motor->ControlTimerThread, p_motor->CommutationPeriodCmd);

			p_motor->CommutationSector = p_motor->NextSector;
			commutation = true;
		}
		break;

	case MOTOR_SENSOR_MODE_BEMF:
		if (Thread_PollTimerCompletePeriodic(&p_motor->ControlTimerThread) == true) //20KHz 59 hour overflow
		{
//			BEMF_SetNewCycle_IO(&p_motor->Bemf);

			p_motor->CommutationPeriodCmd = BEMF_GetTimeAngle60(&p_motor->Bemf);
			Thread_SetTimerPeriod(&p_motor->ControlTimerThread, p_motor->CommutationPeriodCmd);

			p_motor->CommutationSector = p_motor->NextSector;
			commutation = true;
		}
		else
		{
			if (BEMF_PollZeroCrossingDetection_IO(&p_motor->Bemf))
			{
				p_motor->CommutationPeriodCmd = BEMF_GetTimeAngle30(&p_motor->Bemf);
				Thread_SetTimerPeriod(&p_motor->ControlTimerThread, p_motor->CommutationPeriodCmd);
			}
		}
		break;

	case MOTOR_SENSOR_MODE_HALL:
		if(Hall_PollEdge_IO(&p_motor->Hall))
		{
			p_motor->CommutationSector = Hall_GetCommutationId(&p_motor->Hall);
			commutation = true;
		}
		break;

	case MOTOR_SENSOR_MODE_ENCODER:
		//encoder use FOC
		break;

	default:
		break;
	}


//	switch (p_motor->PositionFeedback)
//	{
//
//	case MOTOR_POSITION_FEEDBACK_OPEN_LOOP:		//openloop drive common
//		if(Thread_PollTimerCompletePeriodic(&p_motor->ControlTimerThread) == true)
//		{
//			p_motor->Openloop_RPM = Linear_Ramp_CalcTarget_IncIndex(&p_motor->OpenLoopRamp, &p_motor->OpenLoopRampIndex, p_motor->CommutationPeriodCmd);
//			p_motor->CommutationPeriodCmd = Encoder_Motor_ConvertMechanicalRpmToControlPeriods(&p_motor->Encoder, p_motor->Openloop_RPM );
//			Thread_SetTimerPeriod(&p_motor->ControlTimerThread, p_motor->CommutationPeriodCmd);
//
//			p_motor->CommutationSector = p_motor->NextSector;
//			commutation = true;
//		}
//		break;
//
//	case MOTOR_POSITION_FEEDBACK_BEMF:		//if bemf reliable
//
//		break;
//
//	case MOTOR_POSITION_FEEDBACK_HALL:
//
//		break;
//
////	case MOTOR_POSITION_FEEDBACK_ENCODER: //encoder use foc
////
////		break;
//
//	default:
//		break;
//	}

	if(commutation)
	{
		BEMF_SetNewCycle_IO(&p_motor->Bemf); //always observe bemf
		Encoder_CaptureDeltaT_IO(&p_motor->Encoder);
		Encoder_CaptureExtendedDeltaT_IO(&p_motor->Encoder);
	}

//	if(updatePwm)
//	{
		Motor_ProcRamp(p_motor);
		Motor_ProcControlVariable(p_motor);
		Motor_SixStep_ActivateSector(p_motor, p_motor->CommutationSector);
//	}
//	if(Thread_PollTimerCompletePeriodic(&p_motor->MillisTimerThread) == true)
//	{
//		Encoder_PollDeltaTStop_IO(&p_motor->Encoder);
//	}
}




//run state entry
static inline void Motor_SixStep_StartSectorControl(Motor_T * p_motor) //Prep
{
	if(Motor_GetSpeed(p_motor) == 0)
	{
		p_motor->ControlTimerBase = 0U;

		Encoder_SetZeroSpeed(&p_motor->Encoder);
		p_motor->SpeedFeedback_RPM = 1;

	}
	else
	{

	}

	switch (p_motor->Parameters.SensorMode)
	{

	case MOTOR_SENSOR_MODE_OPEN_LOOP:
		p_motor->PositionFeedback = MOTOR_POSITION_FEEDBACK_OPEN_LOOP;
		Linear_Ramp_InitMillis(&p_motor->OpenLoopRamp, 20000U, 5U, 100U, 1000U);
		break;

	case MOTOR_SENSOR_MODE_BEMF:
//		p_motor->PositionFeedback = MOTOR_POSITION_FEEDBACK_OPEN_LOOP;

//		if (p_motor->PositionFeedback = MOTOR_POSITION_FEEDBACK_OPEN_LOOP)
//		{
			p_motor->OpenLoopRampIndex = 0U;
			p_motor->NextSector = MOTOR_SECTOR_ID_1;
			p_motor->CommutationPeriodCmd = 0U;
			Thread_SetTimerPeriod(&p_motor->ControlTimerThread, 1U); //next pwm will commutate

			//openloop speed ramp
			// must start at sufficient speed for fixed angle displacements
			Linear_Ramp_InitMillis(&p_motor->OpenLoopRamp, 20000U, 5U, 100U, 1000U);
//		}
//		else if (p_motor->PositionFeedback = MOTOR_POSITION_FEEDBACK_BEMF)
//		{
//
//		}

		break;

	case MOTOR_SENSOR_MODE_HALL:
		p_motor->PositionFeedback = MOTOR_POSITION_FEEDBACK_HALL;
		Hall_CaptureSensors_IO(&p_motor->Hall);
		p_motor->CommutationSector = Hall_GetCommutationId(&p_motor->Hall);

		if (Motor_GetSpeed(p_motor) == 1U)
		{
			Encoder_StartDeltaT(&p_motor->Encoder, 20U);
		}

		break;

//	case MOTOR_SENSOR_MODE_ENCODER: //encoder use foc
//		p_motor->PositionFeedback = MOTOR_POSITION_FEEDBACK_ENCODER;
//		break;

	default:
		break;
	}

}

static inline void Motor_SixStep_ProcFreewheel(Motor_T * p_motor)
{
	bool commutation = false;


	//	p_motor->VPwm = speed

	switch (p_motor->Parameters.SensorMode)
	{

//	case MOTOR_SENSOR_MODE_NONE:

	case MOTOR_SENSOR_MODE_BEMF:
		//	if(p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_BEMF)
		//	{
		//		if (BEMF_PollZeroCrossingDetection_IO(&p_motor->Bemf) == true)
		//		{
		//			//poll is bemf reliable
		//			if (p_motor->PositionFeedback == MOTOR_POSITION_FEEDBACK_OPEN_LOOP)
		//			{ //todo move to bemf module
		//				if (p_motor->CommutationPeriodCmd >> 4U == BEMF_GetZeroCrossingPeriod(&p_motor->Bemf) >> 4U) //within 15 ticks or 750us
		//				{
		//					p_motor->OpenLoopZcdCount++;
		//					if (p_motor->OpenLoopZcdCount > 10U)
		//					{
		//						//transition from openloop to bemf
		//						p_motor->PositionFeedback = MOTOR_POSITION_FEEDBACK_BEMF;
		//						p_motor->CommutationPeriodCmd = BEMF_GetTimeAngle30(&p_motor->Bemf); //commutation time using same 20khz timer
		//						Thread_SetTimerPeriod(&p_motor->ControlTimerThread, p_motor->CommutationPeriodCmd);
		//					}
		//				}
		//			} //passive MOTOR_POSITION_FEEDBACK_OBSERVE
		//			else
		//			{
		//				p_motor->NextSector = MOTOR_SECTOR_ID_2;
		//				p_motor->CommutationPeriodCmd = BEMF_GetZeroCrossingPeriod(&p_motor->Bemf)/6U;
		//				Thread_SetTimerPeriod(&p_motor->ControlTimerThread, p_motor->CommutationPeriodCmd);//?
		//			}
		//		}
		//
		//
		//
		////		else			//		if bemf unreliable,  //start from observe mode
		////		{
		////				p_motor->PositionFeedback = MOTOR_POSITION_FEEDBACK_OBSERVE;
		////		if (p_motor->Direction == MOTOR_DIRECTION_CCW) 	{BEMF_MapCcwPhaseAC_IO(&p_motor->Bemf);}
		////		else											{BEMF_MapCwPhaseAC_IO(&p_motor->Bemf);
		//		//Phase_Float()
		//		//report errot to statemechiane
		////		}
		////		}
		//	}

		break;

	case MOTOR_SENSOR_MODE_HALL:
		if(Hall_PollEdge_IO(&p_motor->Hall))
		{
			commutation = true;
		}

	default:
		break;
	}

	if(commutation)
	{
		BEMF_SetNewCycle_IO(&p_motor->Bemf);
		Encoder_CaptureDeltaT_IO(&p_motor->Encoder);
		Encoder_CaptureExtendedDeltaT_IO(&p_motor->Encoder);
	}

}

static inline void Motor_SixStep_StartFreewheel(Motor_T * p_motor)
{

}


//void Motor_SixStep_Jog(Motor_T *p_motor)
//{
//	if (p_motor->JogSteps)
//	{
//		p_motor->JogSteps--;
//	Motor_SixStep_ActivateSector(p_motor, p_motor->NextSector);
//	}
//}

#endif
