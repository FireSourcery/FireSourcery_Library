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
#include "BEMF/BEMF.h"

#include "Transducer/Encoder/Encoder_IO.h"
#include "Transducer/Encoder/Encoder_Motor.h"
#include "Transducer/Encoder/Encoder.h"

#include <stdint.h>
#include <stdbool.h>


static inline void Motor_SixStep_Commutate(Motor_T * p_motor, Motor_SectorId_T sectorId)
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

/*
 * Seprate state for openloop start? maps to different inputs
 */
static inline void Motor_SixStep_RunOpenLoop(Motor_T *p_motor)
{
	if(Thread_PollTimer(&p_motor->ThreadTimer) == true)
	{
		p_motor->VCmd = 65536U / 10 / 4; //todo prop to throttle
		Phase_Polar_SetDutyCyle(&p_motor->Phase, p_motor->VCmd);
		Motor_SixStep_Commutate(p_motor, p_motor->NextSector);

		p_motor->Speed_RPM = 60U;	///p_motor->Speed_RPM = Linear_Ramp_ConvertIndexPlus(&p_motor->Ramp, &p_motor->RampIndex);
		p_motor->OpenLoopPeriod = Encoder_Motor_ConvertMechanicalRpmToControlPeriods(&p_motor->Encoder, p_motor->Speed_RPM);
		Thread_SetTimer(&p_motor->ThreadTimer, p_motor->OpenLoopPeriod);
	}
}

static inline bool Motor_SixStep_PollZcd(Motor_T *p_motor)
{
	bool isReliable = false;

	if(BEMF_PollTimeZeroCrossing_IO(&p_motor->Bemf) == true)
	{
		if (p_motor->OpenLoopPeriod >> 1U == BEMF_GetZeroCrossingPeriod(&p_motor->Bemf) >> 1U) //within 100us
		{
			p_motor->OpenLoopZcd++;
			if (p_motor->OpenLoopZcd > 3U)
			{
				p_motor->OpenLoopZcd = 0U;
				isReliable = true;
			}
		}
	}

	return isReliable;
}


//static inline bool Motor_SixStep_PrepObserveBemf(Motor_T *p_motor)
//{
//	MapNextSector();
//	if (p_motor->Direction == MOTOR_DIRECTION_CCW) 	{p_motor->NextSector = MOTOR_SECTOR_ID_2; BEMF_MapCcwPhaseAC_IO(&p_motor->Bemf);}
//	else											{p_motor->NextSector = MOTOR_SECTOR_ID_6; BEMF_MapCwPhaseAC_IO(&p_motor->Bemf);}
//
//
////	p_motor->Speed_RPM = 60U;			///p_motor->Speed_RPM = Linear_Ramp_ConvertIndexPlus(&p_motor->Ramp, &p_motor->RampIndex);
////	p_motor->OpenLoopPeriod = Encoder_Motor_ConvertMechanicalRpmToControlPeriods(&p_motor->Encoder, p_motor->Speed_RPM);
////	Thread_SetTimer(&p_motor->ThreadTimer, p_motor->OpenLoopPeriod);
//}

//static inline bool Motor_SixStep_ObserveBemf(Motor_T *p_motor)
//{
//	if(BEMF_PollZeroCrossingDetection_IO(&p_motor->Bemf) == true)
//	{
//		BEMF_ZeroTimer();
//		MapNextSector();
//		p_motor->OpenLoopZcd++;
//		if (p_motor->OpenLoopZcd > 3U)
//		{
//			p_motor->OpenLoopZcd = 0U;
//			isReliable = true;
//		}
//	}
//}

//sixstep read returns sectors id
//static inline Motor_SectorId_T Motor_SixStep_ReadSector(Motor_T *p_motor)
//{
//	switch (p_motor->SensorMode)
//	{
//
//	case MOTOR_SENSOR_MODE_BEMF:
//		break;
//
//	case MOTOR_SENSOR_MODE_HALL:
//
//		break;
//
//	default:
//		break;
//	}
//}

static inline void Motor_SixStep_PrepCommutationControl(Motor_T * p_motor)
{

//	Thread_ResetTimer(&p_motor->ThreadTimer);
	p_motor->ControlTimer = 0U;
	switch (p_motor->SensorMode)
	{
	case MOTOR_SENSOR_MODE_OPEN_LOOP:
		p_motor->NextSector = MOTOR_SECTOR_ID_1;
		p_motor->Speed_RPM = 60U;			///p_motor->Speed_RPM = Linear_Ramp_ConvertIndexPlus(&p_motor->Ramp, &p_motor->RampIndex);
		p_motor->OpenLoopPeriod = Encoder_Motor_ConvertMechanicalRpmToControlPeriods(&p_motor->Encoder, p_motor->Speed_RPM);
		Thread_SetTimer(&p_motor->ThreadTimer, p_motor->OpenLoopPeriod);
		break;

	case MOTOR_SENSOR_MODE_BEMF:

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

//	Encoder_CaptureDeltaT_IO(&p_motor->Encoder); //if openloop is seprate state factor out

	switch (p_motor->SensorMode)
	{
	case MOTOR_SENSOR_MODE_OPEN_LOOP:
		if(Thread_PollTimer(&p_motor->ThreadTimer) == true)
		{
			//	speed_RPM = Linear_Ramp_ConvertIndex(&p_motor->Ramp, p_motor->RampIndex);
			//p_motor->Speed_RPM = Linear_Ramp_ConvertIndexPlus(&p_motor->Ramp, &p_motor->RampIndex);
			p_motor->OpenLoopPeriod = Encoder_Motor_ConvertMechanicalRpmToControlPeriods(&p_motor->Encoder, p_motor->Speed_RPM);
			Thread_SetTimer(&p_motor->ThreadTimer, p_motor->OpenLoopPeriod);
			commutate = true;
		}

		break;

	case MOTOR_SENSOR_MODE_BEMF:
		if (Thread_PollTimer(&p_motor->ThreadTimer) == true)
		{
			p_motor->ControlTimer = 0U; //reset timer every time to simplify bemf zcd calc for now
			BEMF_OnCommutation_IO(&p_motor->Bemf); //reset blanktime
			commutate = true;
		}
		else
		{
			if (BEMF_PollTimeZeroCrossing_IO(&p_motor->Bemf))
			{
				Thread_SetTimer(&p_motor->ThreadTimer, BEMF_GetCommutationTime(&p_motor->Bemf));
			}
		}

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

	if(commutate) //&& not openloop
	{
		switch (p_motor->ControlMode)
		{
		case MOTOR_CONTROL_MODE_CONSTANT_VOLTAGE:
			p_motor->VCmd = 65536U / 10 / 4; //todo prop to throttle
			break;
		case MOTOR_CONTROL_MODE_SCALAR_VOLTAGE_FREQ:
			//p_motor->VCmd = p_motor->Speed_RPM * p_motor->VRpmGain;
			break;
		case MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE:
			Motor_PollSpeedLoop(p_motor);
			p_motor->VCmd = 00000; //get from PID
			break;
		default:
			break;
		}
		Phase_Polar_SetDutyCyle(&p_motor->Phase, p_motor->VCmd);
		Motor_SixStep_Commutate(p_motor, p_motor->NextSector);

		Encoder_CaptureDeltaT_IO(&p_motor->Encoder); // track delta using 1 hall phase as encoder //todo check calibrated for commutation and not hall cycle
		//	p_motor->Speed_RPM = Encoder_Motor_GetMechanicalRpm(&p_motor->Encoder); //Always calc speed?
	}

}

// Run in timer ISR. 30 degree after zero crossing.
//void Motor_SixStep_ProcBemfCommutation(Motor_T * p_motor)
//{
////	Motor_SixStep_Commutate(p_motor, (Motor_SectorId_T)Bemf_GetSectorId(&p_motor->Bemf));
//
//}

//static inline void Motor_SixStep_ProcHallControl(Motor_T * p_motor)
//{
//
////	if(Hall_PollSector_IO(&p_motor->Hall))
////	{
////		Phase_Polar_Commutate(p_motor, (Phase_Id_T)Hall_GetSectorId(&p_motor->Hall));		//		Motor_SixStep_Commutate(p_motor, (Motor_SectorId_T)Hall_GetSectorId(&p_motor->Hall));
////	}
//
////	if (SinusodialModulation)
////	{
////		if(Hall_PollSector_IO(&p_motor->Hall))
////		{
////			p_motor->ElectricalAngle = Hall_GetRotorAngle_Degrees16(&p_motor->Hall);
////		}
////		else
////		{
////			p_motor->ElectricalAngle += Encoder_Motor_InterpolateElectricalDelta(&p_motor->Encoder);
////		}
//
////		//use precomputed table or svpwm?
//		//FOC_SetVq(&p_motor->Foc, p_motor->VCmd);
////		FOC_SetTheta(&p_motor->Foc, p_motor->ElectricalAngle);
////		FOC_ProcInvParkInvClarkeSvpwm(&p_motor->Foc);
////		Phase_SetDutyCyle_15(&p_motor->Phase, FOC_GetDutyA(&p_motor->Foc), FOC_GetDutyB(&p_motor->Foc), FOC_GetDutyC(&p_motor->Foc));
////		Phase_Actuate(&p_motor->Phase);
////	}
//}

#endif
