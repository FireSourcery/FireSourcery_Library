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

#include "Transducer/Encoder/Encoder_DeltaT.h"
#include "Transducer/Encoder/Encoder_DeltaD.h"
#include "Transducer/Encoder/Encoder_Motor.h"
#include "Transducer/Encoder/Encoder.h"

#include "Math/Linear/Linear_Voltage.h"
#include "Math/Linear/Linear_ADC.h"
#include "Math/Linear/Linear_Ramp.h"
#include "Math/Linear/Linear.h"

#include <stdint.h>
#include <stdbool.h>


//call at end of adc
static inline bool Motor_SixStep_ProcBemf_IO(Motor_T * p_motor)
{
	BEMF_CaptureSample_IO(&p_motor->Bemf); //must be inside adc isr for capture time
	BEMF_PollZeroCrossingDetection(&p_motor->Bemf); //inside adc isr is shared between spin and freewheel state
}

static inline bool Motor_SixStep_CapturePeakCurrent_IO(Motor_T * p_motor)
{

}


static inline void Motor_SixStep_ActivateSector(Motor_T * p_motor, Motor_SectorId_T sectorId, bool actuatePwm)
{
	if(actuatePwm) {Phase_Polar_SetDutyCyle(&p_motor->Phase, p_motor->VPwm);}

	//hall: 	phase
	//openloop: phase, sector
	//bemf: 	phase, sector, bemfmap

	switch (sectorId)
	{
	case MOTOR_SECTOR_ID_0:
		//set error
		break;

	case MOTOR_SECTOR_ID_1: //Phase AC
		if(actuatePwm) {Phase_Polar_ActivateAC(&p_motor->Phase);}
		if (p_motor->Direction == MOTOR_DIRECTION_CCW) 	{p_motor->NextSector = MOTOR_SECTOR_ID_2; BEMF_MapCcwPhaseAC_IO(&p_motor->Bemf);}
		else											{p_motor->NextSector = MOTOR_SECTOR_ID_6; BEMF_MapCwPhaseAC_IO(&p_motor->Bemf);}

		p_motor->PhaseCurrent_ADCU = *p_motor->p_Ia_ADCU;
		break;

	case MOTOR_SECTOR_ID_2:
		if(actuatePwm) {Phase_Polar_ActivateBC(&p_motor->Phase);}
		if (p_motor->Direction == MOTOR_DIRECTION_CCW) 	{p_motor->NextSector = MOTOR_SECTOR_ID_3; BEMF_MapCcwPhaseBC_IO(&p_motor->Bemf);}
		else											{p_motor->NextSector = MOTOR_SECTOR_ID_1; BEMF_MapCwPhaseBC_IO(&p_motor->Bemf);}

		p_motor->PhaseCurrent_ADCU = *p_motor->p_Ib_ADCU;
		break;

	case MOTOR_SECTOR_ID_3:
		if(actuatePwm) {Phase_Polar_ActivateBA(&p_motor->Phase);}
		if (p_motor->Direction == MOTOR_DIRECTION_CCW) 	{p_motor->NextSector = MOTOR_SECTOR_ID_4; BEMF_MapCcwPhaseBA_IO(&p_motor->Bemf);}
		else											{p_motor->NextSector = MOTOR_SECTOR_ID_2; BEMF_MapCwPhaseBA_IO(&p_motor->Bemf);}

		p_motor->PhaseCurrent_ADCU = *p_motor->p_Ib_ADCU;
		break;

	case MOTOR_SECTOR_ID_4:
		if(actuatePwm) {Phase_Polar_ActivateCA(&p_motor->Phase);}
		if (p_motor->Direction == MOTOR_DIRECTION_CCW) 	{p_motor->NextSector = MOTOR_SECTOR_ID_5; BEMF_MapCcwPhaseCA_IO(&p_motor->Bemf);}
		else											{p_motor->NextSector = MOTOR_SECTOR_ID_3; BEMF_MapCwPhaseCA_IO(&p_motor->Bemf);}

		p_motor->PhaseCurrent_ADCU = *p_motor->p_Ic_ADCU;
		break;

	case MOTOR_SECTOR_ID_5:
		if(actuatePwm) {Phase_Polar_ActivateCB(&p_motor->Phase);}
		if (p_motor->Direction == MOTOR_DIRECTION_CCW) 	{p_motor->NextSector = MOTOR_SECTOR_ID_6; BEMF_MapCcwPhaseCA_IO(&p_motor->Bemf);}
		else											{p_motor->NextSector = MOTOR_SECTOR_ID_4; BEMF_MapCwPhaseCA_IO(&p_motor->Bemf);}

		p_motor->PhaseCurrent_ADCU = *p_motor->p_Ic_ADCU;

		break;

	case MOTOR_SECTOR_ID_6:
		if(actuatePwm) {Phase_Polar_ActivateAB(&p_motor->Phase);}
		if (p_motor->Direction == MOTOR_DIRECTION_CCW) 	{p_motor->NextSector = MOTOR_SECTOR_ID_1; BEMF_MapCcwPhaseAB_IO(&p_motor->Bemf);}
		else											{p_motor->NextSector = MOTOR_SECTOR_ID_5; BEMF_MapCwPhaseAB_IO(&p_motor->Bemf);}

		p_motor->PhaseCurrent_ADCU = *p_motor->p_Ia_ADCU;
		break;

	case MOTOR_SECTOR_ID_7:
		//set error
		break;

	default:
		break;
	}

}

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

//static inline void Motor_SixStep_ProcRamp(Motor_T * p_motor)
//{
//	switch (p_motor->Parameters.BrakeMode)
//	{
//	case MOTOR_BRAKE_MODE_SCALAR:
//		Motor_ProcRamp(p_motor);
//		break;
//	case MOTOR_BRAKE_MODE_REGEN_OPTIMAL:
//		Motor_SixStep_SetBrakeRegenOptimal(p_motor);
//		break;
//	case MOTOR_BRAKE_MODE_REGEN_PROPRTIONAL:
//		Motor_SixStep_SetBrakeRegenProportional(p_motor, p_motor->UserCmd);
//		break;
//	case MOTOR_BRAKE_MODE_REGEN_SCALAR:
//		Motor_SixStep_SetBrakeRegenScalar(p_motor, p_motor->UserCmd);
//		break;
//	default:
//		break;
//	}
//
//}

static inline void Motor_SixStep_ProcControlVariable(Motor_T * p_motor)
{
//	if (brake )
//	{
//	Motor_SixStep_SetRegenBrake(p_motor);
//	}
//	else
//	{

//	Motor_ProcRamp(p_motor);
////	}
//
//	Motor_ProcControlVariable_LimitCurrent(p_motor);


}

static inline bool Motor_SixStep_PollOpenLoop(Motor_T * p_motor)
{
	bool commutation ;

	if (Thread_PollTimerCompletePeriodic(&p_motor->ControlTimerThread) == true)
	{
		p_motor->OpenLoopSpeed_RPM = Linear_Ramp_CalcTarget_IncIndex(&p_motor->OpenLoopRamp, &p_motor->OpenLoopRampIndex, p_motor->OpenLoopCommutationPeriod);
		p_motor->OpenLoopCommutationPeriod = Encoder_Motor_ConvertMechanicalRpmToControlResolution(&p_motor->Encoder, p_motor->OpenLoopSpeed_RPM);
		Thread_SetTimerPeriod(&p_motor->ControlTimerThread, p_motor->OpenLoopCommutationPeriod);
		commutation = true;
	}
	else
	{
		commutation = false;
	}

	return commutation;
}

static inline bool Motor_SixStep_ProcSensorFeedback(Motor_T * p_motor, Motor_SensorMode_T liveSensorMode)
{
	bool commutation = false;


	switch (liveSensorMode)	//active sensor feedback
	{
	case MOTOR_SENSOR_MODE_OPEN_LOOP:
		if(p_motor->IsActiveControl)
		{
		if(Motor_SixStep_PollOpenLoop(p_motor)) //todo fix state
		{
			p_motor->CommutationSector = p_motor->NextSector;
			commutation = true;
		}
		}
		break;

	case MOTOR_SENSOR_MODE_BEMF:
		if (BEMF_PollCycleTimer(&p_motor->Bemf))
		{
			p_motor->CommutationSector = p_motor->NextSector;
			commutation = true;
		}
		break;

	case MOTOR_SENSOR_MODE_HALL:
		if(Hall_PollSensorsEdge_IO(&p_motor->Hall))
		{
			p_motor->CommutationSector = Hall_GetCommutation(&p_motor->Hall);
			commutation = true;
		}
		break;

	case MOTOR_SENSOR_MODE_ENCODER:
		break;

	default:
		break;
	}

	return commutation;
}


/*
 * State Threads
 */

//state loop in 20khz pwm thread
//spin and freewheel state
static inline void Motor_SixStep_ProcPhaseControl(Motor_T * p_motor)
{
	bool commutation = false;
//	bool updatePwm = false;

	Motor_SensorMode_T liveSensorMode;

	//start ADC in later abstraction

	switch (p_motor->Parameters.SensorMode) //active sensor feedback
	{
	case MOTOR_SENSOR_MODE_OPEN_LOOP:
		liveSensorMode = MOTOR_SENSOR_MODE_OPEN_LOOP;
		p_motor->VPwm = p_motor->UserCmd/4U;
		break;

	case MOTOR_SENSOR_MODE_BEMF:
		if (p_motor->IsStartUp) //eliminate with separate state?
		{
			if(BEMF_GetZeroCrossingCounter(&p_motor->Bemf) > 10U)
			{
				p_motor->IsStartUp = false;
			}
			liveSensorMode = MOTOR_SENSOR_MODE_OPEN_LOOP;
			p_motor->VPwm = p_motor->UserCmd/4U;
		}
		else
		{
			liveSensorMode = MOTOR_SENSOR_MODE_BEMF;
		}
		break;

	case MOTOR_SENSOR_MODE_HALL:
		liveSensorMode = MOTOR_SENSOR_MODE_HALL;
		break;

	case MOTOR_SENSOR_MODE_ENCODER:
		//encoder use FOC
		break;

	default:
		break;
	}

	commutation = Motor_SixStep_ProcSensorFeedback(p_motor, liveSensorMode);

	if(commutation)
	{
		Encoder_DeltaT_Capture_IO(&p_motor->Encoder);
		Encoder_DeltaT_CaptureExtension_IO(&p_motor->Encoder);
		BEMF_StartCycle_IO(&p_motor->Bemf);
	}

	if(p_motor->IsActiveControl && !(liveSensorMode==MOTOR_SENSOR_MODE_OPEN_LOOP)) //todo fix
	{
		Motor_ProcRamp(p_motor);
		Motor_ProcControlVariable_Thread(p_motor);
	}

	Motor_SixStep_ActivateSector(p_motor, p_motor->CommutationSector, p_motor->IsActiveControl);

	//filterconver phase current
//	p_motor->PhaseCurrent_ADCU = *p_motor->p_Ia_ADCU;
}

//static inline void Motor_ConvertIBat(Motor_T * p_motor)
//{
//	p_motor->PhaseCurrent_ADCU = MovAvg
//}

//run state entry
static inline void Motor_SixStep_StartSpin(Motor_T * p_motor) //start active control
{
#define OPEN_LOOP_START_SPEED 5U
#define OPEN_LOOP_SPEED 100U
	p_motor->IsActiveControl = true;
	BEMF_SetObserveMode(&p_motor->Bemf, BEMF_MODE_COMMUTATION);



	if(Motor_GetSpeed(p_motor) == 0)
	{
		p_motor->ControlTimerBase = 0U;
//		Encoder_DeltaT_SetInitial(&p_motor->Encoder, 5U);

		BEMF_Reset(&p_motor->Bemf); //ret zcdCount
	}

	switch (p_motor->Parameters.SensorMode)
	{

	case MOTOR_SENSOR_MODE_OPEN_LOOP:
		if(Motor_GetSpeed(p_motor) == 0)
		{
//			Linear_Ramp_InitAcceleration(&p_motor->OpenLoopRamp, 20000U, OPEN_LOOP_START_SPEED, 100U, 100U);

			Linear_Ramp_InitMillis(&p_motor->OpenLoopRamp, 20000U, OPEN_LOOP_START_SPEED, OPEN_LOOP_SPEED, 1000U);
			p_motor->OpenLoopRampIndex = 0U;

			p_motor->OpenLoopCommutationPeriod = 0U;
			Thread_SetTimerPeriod(&p_motor->ControlTimerThread, 0U);

			p_motor->NextSector = MOTOR_SECTOR_ID_2;
		}
		else
		{
//			allow openloop transition from freewheel state?
//			Linear_Ramp_InitMillis(&p_motor->OpenLoopRamp, 20000U, Motor_GetSpeed(p_motor), Motor_GetSpeed(p_motor), 1000U);
		}
		break;

	case MOTOR_SENSOR_MODE_BEMF:

		if(Motor_GetSpeed(p_motor) == 0)
		{
			p_motor->IsStartUp = true;
			//openloop speed ramp	// must start at sufficient speed for timer driven fixed angle displacements
			Linear_Ramp_InitMillis(&p_motor->OpenLoopRamp, 20000U, OPEN_LOOP_START_SPEED, OPEN_LOOP_SPEED, 1000U);
			p_motor->OpenLoopRampIndex = 0U;

			p_motor->OpenLoopCommutationPeriod = 0U;
			Thread_SetTimerPeriod(&p_motor->ControlTimerThread, 0U); //next pwm will commutate

//			if aligned 		//motor aligned to phase A
//			if (p_motor->Direction == MOTOR_DIRECTION_CCW) 	{p_motor->NextSector = MOTOR_SECTOR_ID_2;}
//			else											{p_motor->NextSector = MOTOR_SECTOR_ID_6;}

			p_motor->NextSector = MOTOR_SECTOR_ID_2;
		}
		else //transition to bemf from freewheel
		{
			//bemf module already set commutation time
		}
		break;

	case MOTOR_SENSOR_MODE_HALL:
		Hall_CaptureSensors_IO(&p_motor->Hall);
		p_motor->CommutationSector = Hall_GetCommutation(&p_motor->Hall);

		break;

//	case MOTOR_SENSOR_MODE_ENCODER: //encoder use foc
//		break;

	default:
		break;
	}

}


static inline void Motor_SixStep_StartFreewheel(Motor_T * p_motor) //start observe
{
	Motor_Float(p_motor);
	p_motor->IsActiveControl = false;
	BEMF_Reset(&p_motor->Bemf);
	BEMF_SetObserveMode(&p_motor->Bemf, BEMF_MODE_PASSIVE);

//	switch (p_motor->Parameters.SensorMode)
//	{
//	case MOTOR_SENSOR_MODE_OPEN_LOOP:
//		break;
//
//	case MOTOR_SENSOR_MODE_BEMF:
//
//		break;
//
//	case MOTOR_SENSOR_MODE_HALL:
//		break;
//
////	case MOTOR_SENSOR_MODE_ENCODER: //encoder use foc
//	default:
//		break;
//	}
}


static inline bool Motor_SixStep_SetBemfState(Motor_T * p_motor)
{



}

static inline void Motor_SixStep_PollTransition(Motor_T * p_motor)
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



//static inline void Motor_SixStep_ProcFreewheel(Motor_T * p_motor)
//{
//	bool isBoundary = false;
//
//	switch (p_motor->Parameters.SensorMode)
//	{
//
//	case MOTOR_SENSOR_MODE_OPEN_LOOP:
//
//	case MOTOR_SENSOR_MODE_BEMF:
//		if(BEMF_PollCycle(&p_motor->Bemf))
//		{
//			p_motor->CommutationSector = p_motor->NextSector;
//
//			isBoundary = true;
//		}
//		break;
//
//	case MOTOR_SENSOR_MODE_HALL:
//		if(Hall_PollEdge_IO(&p_motor->Hall))
//		{
//			isBoundary = true;
//		}
//
//	default:
//		break;
//	}
//
//	if(isBoundary)
//	{
//		Encoder_DeltaT_Capture_IO(&p_motor->Encoder);
//		Encoder_DeltaT_CaptureExtension_IO(&p_motor->Encoder);
//
//		Motor_SixStep_ActivateSector(p_motor, p_motor->CommutationSector,  0U); //observe bemf in hall mode?
//	}
//}
