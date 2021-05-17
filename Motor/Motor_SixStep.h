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


static inline void Motor_SixStep_CommutateSector(Motor_T * p_motor, Motor_SectorId_T sectorId)
{
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

static inline void Motor_SixStep_ActivateCommutation(Motor_T * p_motor)
{
	//if not set from outside
	p_motor->VReq = Linear_ADC_CalcUnsignedFraction16(&p_motor->UnitThrottle, p_motor->AnalogChannelResults[MOTOR_ANALOG_CHANNEL_THROTTLE]);

	switch (p_motor->ControlMode)
	{

	case MOTOR_CONTROL_MODE_OPEN_LOOP:
		p_motor->VCtrl = p_motor->VReq / 4;
		break;

	case MOTOR_CONTROL_MODE_CONSTANT_VOLTAGE:
		p_motor->VCtrl = p_motor->VReq;
		break;
	case MOTOR_CONTROL_MODE_SCALAR_VOLTAGE_FREQ:
		//p_motor->VCmd = p_motor->VReq * p_motor->SpeedReq_RPM * p_motor->VRpmGain;
		break;
	case MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE:
		Motor_PollSpeedLoop(p_motor);
		p_motor->VCtrl = 00000; //get from PID
		break;
	default:
		break;
	}

	//testing
	p_motor->VCtrl = PHASE_DUTY_CYCLE_MAX / 10 / 4;

	Phase_Polar_SetDutyCyle(&p_motor->Phase, p_motor->VCtrl);
//		Motor_SixStep_CommutateSector(p_motor, p_motor->NextSector);

	BEMF_SetNewCycle_IO(&p_motor->Bemf);

	Encoder_CaptureDeltaT_IO(&p_motor->Encoder); // track delta using 1 hall phase as encoder //todo check calibrated for commutation and not hall cycle
	p_motor->SpeedFeedback_RPM = Encoder_Motor_GetMechanicalRpm(&p_motor->Encoder); //Always calc speed? //invaldate low speeds
}


/*
 * Seperate state for OpenLoop start? maps to different inputs
 */
static inline void Motor_SixStep_ProcOpenLoop(Motor_T *p_motor)
{

}



static inline void Motor_SixStep_SetCommutationControl(Motor_T * p_motor)
{

//	if (p_motor->HwSensor == MOTOR_SENSOR_MODE_BEMF)
//	{
//		p_motor->SensorMode = MOTOR_SENSOR_MODE_OPEN_LOOP;
//	}
//	else if( p_motor->HwSensor ==  MOTOR_SENSOR_MODE_HALL)
//	{
//		p_motor->SensorMode = MOTOR_SENSOR_MODE_HALL;
//	}


	p_motor->ControlTimerBase = 0U;

	switch (p_motor->SensorMode)
	{
	case MOTOR_SENSOR_MODE_OPEN_LOOP:
		p_motor->NextSector = MOTOR_SECTOR_ID_1;

		//testing
//		p_motor->VCmd = PHASE_DUTY_CYCLE_MAX / 10 / 4; //todo prop to throttle
//		p_motor->SpeedCmd_RPM = 10U;
//		p_motor->OpenLoopPeriod = Encoder_Motor_ConvertMechanicalRpmToControlPeriods(&p_motor->Encoder, p_motor->SpeedCmd_RPM);

		/* Reset */
		Linear_Ramp_InitMillis(&p_motor->Ramp, 4U, 10U, 1000U, 20000U); // must start at sufficent speed for fixed angle displacements
		p_motor->RampIndex = 0U;
		p_motor->VCtrl = 0U;
//		p_motor->SpeedCmd_RPM = 0U;
		p_motor->CommutationPeriodCmd = 0U;
		Thread_SetTimer(&p_motor->ControlTimerThread, 1U); //next pwm will commutate

		break;

	case MOTOR_SENSOR_MODE_BEMF:
		//transition from openloop to bemf
		p_motor->CommutationPeriodCmd = BEMF_GetCommutationTime(&p_motor->Bemf); //commutation time using same 20khz timer
		Thread_SetTimer(&p_motor->ControlTimerThread, p_motor->CommutationPeriodCmd);

		break;

	case MOTOR_SENSOR_MODE_HALL:

		break;

	default:
		break;
	}
}



//20khz pwm thread
static inline void Motor_SixStep_ProcCommutationControl(Motor_T * p_motor)
{
	bool commutate = false;

	switch (p_motor->SensorMode)
	{
	case MOTOR_SENSOR_MODE_OPEN_LOOP:
		//openloop drive common
		if(Thread_PollTimer(&p_motor->ControlTimerThread) == true)
		{
			p_motor->SpeedReq_RPM = Linear_Ramp_CalcTarget_IncIndex(&p_motor->Ramp, &p_motor->RampIndex, p_motor->CommutationPeriodCmd);
			p_motor->CommutationPeriodCmd = Encoder_Motor_ConvertMechanicalRpmToControlPeriods(&p_motor->Encoder, p_motor->SpeedReq_RPM);
			Thread_SetTimer(&p_motor->ControlTimerThread, p_motor->CommutationPeriodCmd);

			commutate = true;
		}

		//open loop state
		if (BEMF_PollZeroCrossingDetection_IO(&p_motor->Bemf) == true)
		{
			if (p_motor->CommutationPeriodCmd >> 4U == BEMF_GetZeroCrossingPeriod(&p_motor->Bemf) >> 4U) //within 15 ticks or 750us
			{
				p_motor->OpenLoopZcdCount++;
				if (p_motor->OpenLoopZcdCount > 10U)
				{
					//transition from openloop to bemf
					p_motor->CommutationPeriodCmd = BEMF_GetCommutationTime(&p_motor->Bemf);
					Thread_SetTimer(&p_motor->ControlTimerThread, p_motor->CommutationPeriodCmd);
					p_motor->SensorMode = MOTOR_SENSOR_MODE_BEMF;
				}
			}
		}

		break;

	case MOTOR_SENSOR_MODE_BEMF:
		//ifbemf reliable
		if (Thread_PollTimer(&p_motor->ControlTimerThread) == true) //20KHz 59 hour overflow
		{
			BEMF_SetNewCycle_IO(&p_motor->Bemf);

			//back up timer
			p_motor->CommutationPeriodCmd = BEMF_GetCommutationTime(&p_motor->Bemf);
			Thread_SetTimer(&p_motor->ControlTimerThread, p_motor->CommutationPeriodCmd);
			commutate = true;
		}
		else
		{
			if (BEMF_PollZeroCrossingDetection_IO(&p_motor->Bemf))
			{
				p_motor->CommutationPeriodCmd = BEMF_GetCommutationTime(&p_motor->Bemf);
				Thread_SetTimer(&p_motor->ControlTimerThread, p_motor->CommutationPeriodCmd);
			}
		}

//		if bemf unreliable
//		p_motor->SensorMode = MOTOR_SENSOR_MODE_OPEN_LOOP;
		//start from observe mode
		break;

	case MOTOR_SENSOR_MODE_HALL:
		if(Hall_PollSector_IO(&p_motor->Hall))
		{
			p_motor->NextSector = Hall_GetSectorId(&p_motor->Hall); //overwrite next sector
			commutate = true;
		}
		break;

	default:
		break;
	}

	if(commutate)
	{
		Motor_SixStep_ActivateCommutation(p_motor);
	}

}

static inline void Motor_SixStep_BrakeRegenOptimal(Motor_T * p_motor)
{
	p_motor->VReq = Linear_Voltage_CalcUnsignedFraction16(&p_motor->UnitVabc, BEMF_GetVPhase(&p_motor->Bemf)) / 2U;
}

static inline void Motor_SixStep_BrakeRegenProportional(Motor_T * p_motor, uint16_t intensity)
{
	p_motor->VReq = Linear_Voltage_CalcUnsignedFraction16(&p_motor->UnitVabc, BEMF_GetVPhase(&p_motor->Bemf)) * (65536U - intensity) >> 16U;

	//braking 0% -> pwm 100% of back emf;
	//braking 10% -> pwm 90% of back emf;
	//braking 50% -> pwm 50% of back emf;
	//braking 90% -> pwm 10% of back emf;
}

static inline void Motor_SixStep_BrakeRegenScalar(Motor_T * p_motor, uint16_t intensity)
{
//	p_motor->VReq = Linear_Voltage_CalcUnsignedFraction16(&p_motor->UnitVabc, BEMF_GetVPhase(&p_motor->Bemf)) * (32768U - ((intensity + 32768U) / p_motor->RegenCoeffcient)) >> 16U;

	// e.g
	// RegenCoeffcientInput = 4, RegenCoeffcient -> fract16(1,4)

	//braking 0% -> pwm 62.5% of back emf;
	//braking 10% -> pwm 60% of back emf;
	//braking 50% -> pwm 50% of back emf;
	//braking 90% -> pwm 40% of back emf;
	//braking 100% -> pwm 37.5% of back emf;
}

//void Motor_Jog(Motor_T *p_motor)
//{
//	if (p_motor->JogSteps)
//	{
//		p_motor->JogSteps--;
//Motor_SixStep_CommutateSector(p_motor, p_motor->NextSector);
//	}
//

#endif

//static inline void Motor_SixStep_ProcHallControl(Motor_T * p_motor)
//{
//	if(Hall_PollSector_IO(&p_motor->Hall))
//	{
//		Phase_Polar_Commutate(p_motor, (Phase_Id_T)Hall_GetSectorId(&p_motor->Hall));		//		Motor_SixStep_Commutate(p_motor, (Motor_SectorId_T)Hall_GetSectorId(&p_motor->Hall));
//	}
//
//	if (SinusodialModulation)
//	{
//		if(Hall_PollSector_IO(&p_motor->Hall))
//		{
//			p_motor->ElectricalAngle = Hall_GetRotorAngle_Degrees16(&p_motor->Hall);
//		}
//		else
//		{
//			p_motor->ElectricalAngle += Encoder_Motor_InterpolateElectricalDelta(&p_motor->Encoder);
//		}
//
//		//use precomputed table  svpwm

//	}
//}
