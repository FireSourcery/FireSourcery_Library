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
    @file 	Motor_SixStep.h
    @author FireSoucery
    @brief  Motor Six Step submodule.
    		Six Step commutation strategy. Polar PWM

    		Current design - Only SixStep module tracks phase

    @version V0
*/
/******************************************************************************/
#ifndef MOTOR_SIXSTEP_H
#define MOTOR_SIXSTEP_H

#include "Motor.h"
#include "Config.h"

#include "Transducer/Hall/Hall.h"
#include "Transducer/BEMF/BEMF.h"
#include "Transducer/Phase/Phase.h"

#include "Transducer/Encoder/Encoder_DeltaT.h"
#include "Transducer/Encoder/Encoder_Motor.h"
#include "Transducer/Encoder/Encoder.h"

#include "Peripheral/Analog/AnalogN/AnalogN.h"

#include "Math/Linear/Linear_Ramp.h"
#include "Math/Linear/Linear.h"

#include <stdint.h>
#include <stdbool.h>


#include "Utility/Debug/Debug.h"


/*
 * Case Observe: Adc
 * Case Control: Adc, pwm, cv
 * Case Observe Commutation: bemf
 * Case Control Commutation: bemf, phase, cv, pwm
 *
 * let complier optimize
 */

/*
 *  Once per commutation, both obeserve and control modes
 */
static inline void MapMotorSixStepBemfPhase(Motor_T * p_motor)
{
 	switch (p_motor->CommutationPhase)
	{
		case MOTOR_PHASE_ERROR_0:
			//set error
			break;

		case MOTOR_PHASE_AC:
			if (p_motor->Direction == MOTOR_DIRECTION_CCW) 	{p_motor->NextPhase = MOTOR_PHASE_BC; BEMF_MapCcwPhaseAC(&p_motor->Bemf);}
			else											{p_motor->NextPhase = MOTOR_PHASE_AB; BEMF_MapCwPhaseAC(&p_motor->Bemf);}
			break;

		case MOTOR_PHASE_BC:
			if (p_motor->Direction == MOTOR_DIRECTION_CCW) 	{p_motor->NextPhase = MOTOR_PHASE_BA; BEMF_MapCcwPhaseBC(&p_motor->Bemf);}
			else											{p_motor->NextPhase = MOTOR_PHASE_AC; BEMF_MapCwPhaseBC(&p_motor->Bemf);}
			break;

		case MOTOR_PHASE_BA:
			if (p_motor->Direction == MOTOR_DIRECTION_CCW) 	{p_motor->NextPhase = MOTOR_PHASE_CA; BEMF_MapCcwPhaseBA(&p_motor->Bemf);}
			else											{p_motor->NextPhase = MOTOR_PHASE_BC; BEMF_MapCwPhaseBA(&p_motor->Bemf);}
			break;

		case MOTOR_PHASE_CA:
			if (p_motor->Direction == MOTOR_DIRECTION_CCW) 	{p_motor->NextPhase = MOTOR_PHASE_CB; BEMF_MapCcwPhaseCA(&p_motor->Bemf);}
			else											{p_motor->NextPhase = MOTOR_PHASE_BA; BEMF_MapCwPhaseCA(&p_motor->Bemf);}
			break;

		case MOTOR_PHASE_CB:
			if (p_motor->Direction == MOTOR_DIRECTION_CCW) 	{p_motor->NextPhase = MOTOR_PHASE_AB; BEMF_MapCcwPhaseCB(&p_motor->Bemf);}
			else											{p_motor->NextPhase = MOTOR_PHASE_CA; BEMF_MapCwPhaseCB(&p_motor->Bemf);}
			break;

		case MOTOR_PHASE_AB:
			if (p_motor->Direction == MOTOR_DIRECTION_CCW) 	{p_motor->NextPhase = MOTOR_PHASE_AC; BEMF_MapCcwPhaseAB(&p_motor->Bemf);}
			else											{p_motor->NextPhase = MOTOR_PHASE_CB; BEMF_MapCwPhaseAB(&p_motor->Bemf);}
			break;

		case MOTOR_PHASE_ERROR_7:
			//set error
			break;

		default:
			break;
	}

 	//alternatively
//	if (p_motor->Direction == MOTOR_DIRECTION_CCW)
//	{
//		p_motor->NextSector = (p_motor->CommutationSector + 1U) % 6U;
//	}
//	else
//	{
//		p_motor->NextSector = (p_motor->CommutationSector - 1U) % 6U;
//		p_motor->Bemf.IsBemfRising = !p_motor->Bemf.IsBemfRising;

 	// 	p_motor->NextSector = (p_motor->NextSector + 4U) % 6U;
 	// 	p_motor->Bemf.IsBemfRising = !p_motor->Bemf.IsBemfRising;
//	}
}

/*
 *  Once per commutation, control mode only
 */
static inline void ActivateMotorSixStepPhase(Motor_T * p_motor)
{
 	switch (p_motor->CommutationPhase)
	{
		case MOTOR_PHASE_ERROR_0: 	break;
		case MOTOR_PHASE_AC: Phase_Polar_ActivateAC(&p_motor->Phase, p_motor->VPwm); break;
		case MOTOR_PHASE_BC: Phase_Polar_ActivateBC(&p_motor->Phase, p_motor->VPwm); break;
		case MOTOR_PHASE_BA: Phase_Polar_ActivateBA(&p_motor->Phase, p_motor->VPwm); break;
		case MOTOR_PHASE_CA: Phase_Polar_ActivateCA(&p_motor->Phase, p_motor->VPwm); break;
		case MOTOR_PHASE_CB: Phase_Polar_ActivateCB(&p_motor->Phase, p_motor->VPwm); break;
		case MOTOR_PHASE_AB: Phase_Polar_ActivateAB(&p_motor->Phase, p_motor->VPwm); break;
		case MOTOR_PHASE_ERROR_7: 	break;
		default: break;
	}
}

/*
 *  Every Pwm, control mode only
 */
static inline void ActivateMotorSixStepPhaseDuty(Motor_T * p_motor)
{
	switch (p_motor->CommutationPhase)
	{
		case MOTOR_PHASE_ERROR_0: break;
		case MOTOR_PHASE_AC: Phase_Polar_ActivateDutyAC(&p_motor->Phase, p_motor->VPwm); break;
		case MOTOR_PHASE_BC: Phase_Polar_ActivateDutyBC(&p_motor->Phase, p_motor->VPwm); break;
		case MOTOR_PHASE_BA: Phase_Polar_ActivateDutyBA(&p_motor->Phase, p_motor->VPwm); break;
		case MOTOR_PHASE_CA: Phase_Polar_ActivateDutyCA(&p_motor->Phase, p_motor->VPwm); break;
		case MOTOR_PHASE_CB: Phase_Polar_ActivateDutyCB(&p_motor->Phase, p_motor->VPwm); break;
		case MOTOR_PHASE_AB: Phase_Polar_ActivateDutyAB(&p_motor->Phase, p_motor->VPwm); break;
		case MOTOR_PHASE_ERROR_7: break;
		default: break;
	}
}

/*
 * All cases
 */
static inline void ActivateMotorSixStepAnalog(Motor_T * p_motor)
{
	if (BEMF_CheckBlankTime(&p_motor->Bemf) == true)
	{
		Debug_CapturePeriod(0); //expected 50
		Debug_CaptureRef(); //begin pwm cycle

		switch (p_motor->CommutationPhase)
		{
			case MOTOR_PHASE_ERROR_0: break;
			case MOTOR_PHASE_AC: AnalogN_EnqueueFrontConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_BEMF_B); break;
			case MOTOR_PHASE_BC: AnalogN_EnqueueFrontConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_BEMF_A); break;
			case MOTOR_PHASE_BA: AnalogN_EnqueueFrontConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_BEMF_C); break;
			case MOTOR_PHASE_CA: AnalogN_EnqueueFrontConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_BEMF_B); break;
			case MOTOR_PHASE_CB: AnalogN_EnqueueFrontConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_BEMF_A); break;
			case MOTOR_PHASE_AB: AnalogN_EnqueueFrontConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_BEMF_C); break;
			case MOTOR_PHASE_ERROR_7: break;
			default: break;
		}
	}
}

/*
 * Open Loop Functions
 */
static inline bool Motor_SixStep_PollOpenLoop(Motor_T * p_motor)
{
	bool commutation = Timer_Poll(&p_motor->ControlTimer);

	if (commutation == true)
	{
		p_motor->OpenLoopSpeed_RPM = Linear_Ramp_CalcTargetIncIndex(&p_motor->OpenLoopRamp, &p_motor->OpenLoopRampIndex, p_motor->OpenLoopCommutationPeriod);
		p_motor->OpenLoopCommutationPeriod = Encoder_Motor_ConvertMechanicalRpmToInterpolationFreq(&p_motor->Encoder, p_motor->OpenLoopSpeed_RPM);
		Timer_SetPeriod(&p_motor->ControlTimer, p_motor->OpenLoopCommutationPeriod);
	}

	return commutation;
}

static inline void Motor_SixStep_StartOpenLoop(Motor_T * p_motor)
{

	// if pwm update is only on commutation cycle, must start at sufficient speed for timer driven fixed angle displacements
//	Linear_Ramp_Init_Millis(&p_motor->OpenLoopRamp, 20000U, p_motor->Parameters.OpenLoopSpeedStart, p_motor->Parameters.OpenLoopSpeedFinal, 1000U);
	Linear_Ramp_Init_Acceleration(&p_motor->OpenLoopRamp, 20000U, p_motor->Parameters.OpenLoopSpeedStart, p_motor->Parameters.OpenLoopSpeedFinal, 100U);
	p_motor->OpenLoopRampIndex = 0U;

	p_motor->OpenLoopSpeed_RPM = Linear_Ramp_CalcTargetIncIndex(&p_motor->OpenLoopRamp, &p_motor->OpenLoopRampIndex, 0U);
	p_motor->OpenLoopCommutationPeriod = Encoder_Motor_ConvertMechanicalRpmToInterpolationFreq(&p_motor->Encoder, p_motor->OpenLoopSpeed_RPM);
	Timer_SetPeriod(&p_motor->ControlTimer, p_motor->OpenLoopCommutationPeriod);

	//motor aligned to phase A
	p_motor->NextPhase = (p_motor->Direction == MOTOR_DIRECTION_CCW) ? MOTOR_PHASE_BC : MOTOR_PHASE_AB;
}

static inline void Motor_SixStep_ProcOpenLoop(Motor_T * p_motor)
{
	p_motor->VPwm = p_motor->RampCmd; //p_motor->Parameters.OpenLoopVPwm_Frac16 +

	// bound to 2.5 to 10 percent
	if 		(p_motor->VPwm < p_motor->Parameters.OpenLoopVPwmMin)	{ p_motor->VPwm = p_motor->Parameters.OpenLoopVPwmMin;}
	else if (p_motor->VPwm > p_motor->Parameters.OpenLoopVPwmMax)	{ p_motor->VPwm = p_motor->Parameters.OpenLoopVPwmMax;	}

	if(Motor_SixStep_PollOpenLoop(p_motor) == true)
	{
		p_motor->CommutationPhase = p_motor->NextPhase;

//		Encoder_DeltaT_Capture(&p_motor->Encoder);
//		Encoder_DeltaT_CaptureExtendedTimer(&p_motor->Encoder);

		MapMotorSixStepBemfPhase(p_motor);
		BEMF_CapturePhaseReference(&p_motor->Bemf); //set reference
		ActivateMotorSixStepPhase(p_motor);
	}
	else
	{
		BEMF_ProcZeroCrossingDetection(&p_motor->Bemf); //  poll zcd with prev capture
	}

	ActivateMotorSixStepAnalog(p_motor);
}

static inline bool Motor_SixStep_GetBemfReliable(Motor_T * p_motor) {return BEMF_GetIsReliable(&p_motor->Bemf);}

static inline bool PollMotorSixStepCommutation(Motor_T * p_motor)
{
	bool commutation = false;
	switch(p_motor->Parameters.SensorMode)
	{
//		case MOTOR_SENSOR_MODE_OPEN_LOOP :
//			if(Motor_SixStep_PollOpenLoop(p_motor) == true)
//			{
//				p_motor->CommutationPhase = p_motor->NextPhase;
//				commutation = true;
//			}
//			break;

		case MOTOR_SENSOR_MODE_BEMF:
//			if(Motor_SixStep_GetBemfReliable(p_motor) == true)
//			{
				if(BEMF_CheckPhasePeriod(&p_motor->Bemf) == true)
				{
					p_motor->CommutationPhase = p_motor->NextPhase;

					if (BEMF_GetIsZeroCrossingDetectionComplete(&p_motor->Bemf) == true)
					{
						Encoder_DeltaT_Capture(&p_motor->Encoder);
						Encoder_DeltaT_CaptureExtendedTimer(&p_motor->Encoder);
					}

					MapMotorSixStepBemfPhase(p_motor);
					BEMF_CapturePhaseReference(&p_motor->Bemf); //set reference
					commutation = true;
				}
				else
				{
					BEMF_ProcZeroCrossingDetection(&p_motor->Bemf); //  poll zcd with prev capture
				}
//			}
			break;

		case MOTOR_SENSOR_MODE_HALL :
			if(Hall_PollCaptureSensors(&p_motor->Hall) == true)
			{
				p_motor->CommutationPhase = (Motor_SectorId_T)Hall_GetCommutationId(&p_motor->Hall); //hall id match motor id
 				Encoder_DeltaT_Capture(&p_motor->Encoder);
				Encoder_DeltaT_CaptureExtendedTimer(&p_motor->Encoder);

				MapMotorSixStepBemfPhase(p_motor);
				BEMF_CapturePhaseReference(&p_motor->Bemf); //set reference
				commutation = true;
			}
			break;

//	case MOTOR_SENSOR_MODE_ENCODER:	 //encoder use foc
//		break;

		default :
			break;
	}

	return commutation;
}


/*
 * State loop in 20khz pwm thread
 * Active Control
 */
static inline void Motor_SixStep_ProcPhaseControl(Motor_T * p_motor)
{
	Debug_CapturePeriod(0);
	Debug_CaptureRef(); //begin pwm cycle

	Motor_ProcControlVariable(p_motor);

	if(PollMotorSixStepCommutation(p_motor) == true)
	{
		ActivateMotorSixStepPhase(p_motor);
	}
	else
	{
		ActivateMotorSixStepPhaseDuty(p_motor); /* update pwm with control variable value every pwm cycle */
	}

	ActivateMotorSixStepAnalog(p_motor);
}


static inline void Motor_SixStep_ResumePhaseControl(Motor_T * p_motor)
{
	switch(p_motor->Parameters.SensorMode)
	{
//		case MOTOR_SENSOR_MODE_OPEN_LOOP :
//			break;

		case MOTOR_SENSOR_MODE_BEMF:
			break;

		case MOTOR_SENSOR_MODE_HALL:
			Hall_ResetCapture(&p_motor->Hall);
			break;

//	case MOTOR_SENSOR_MODE_ENCODER:
//		resume =  true;
//		break;

		default :
			break;
	}

	BEMF_SetCycleMode(&p_motor->Bemf, BEMF_CYCLE_MODE_COMMUTATION);
}

static inline bool Motor_SixStep_CheckResumePhaseControl(Motor_T * p_motor)
{
	bool resume = false;

	switch(p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_OPEN_LOOP :
//			resume = false;
//			break;

		case MOTOR_SENSOR_MODE_BEMF :
			resume = Motor_SixStep_GetBemfReliable(p_motor);
			break;

		case MOTOR_SENSOR_MODE_HALL :
			resume = true;
			break;

//	case MOTOR_SENSOR_MODE_ENCODER:
//		resume =  true;
//		break;

		default :
			break;
	}

	return resume;
}

/*
 * From 0 speed always
 */
static inline void Motor_SixStep_StartPhaseControl(Motor_T * p_motor)
{
	Encoder_Reset(&p_motor->Encoder);
	p_motor->ControlTimerBase = 0U; //overflow at 20Khz, 59 hours
//	Encoder_DeltaT_SetInitial(&p_motor->Encoder, 5U);

	p_motor->Bemf.ZeroCrossingCounter  = 0U;

	switch (p_motor->Parameters.SensorMode)
	{
//	case MOTOR_SENSOR_MODE_OPEN_LOOP:
//		Motor_SixStep_StartOpenLoop(p_motor);
//		break;

	case MOTOR_SENSOR_MODE_BEMF:
		BEMF_SetCycleMode(&p_motor->Bemf, BEMF_CYCLE_MODE_STARTUP);
		Motor_SixStep_StartOpenLoop(p_motor);
//			p_motor->IsOpenLoopStartUp = true;
		break;

	case MOTOR_SENSOR_MODE_HALL:
		BEMF_SetCycleMode(&p_motor->Bemf, BEMF_CYCLE_MODE_COMMUTATION);
		Hall_ResetCapture(&p_motor->Hall);
		break;

//	case MOTOR_SENSOR_MODE_ENCODER: //encoder always use foc
//		break;

	default:
		break;
	}
}

/*
 * Passive Observe
 */
static inline void Motor_SixStep_ProcPhaseObserve(Motor_T * p_motor)
{
	PollMotorSixStepCommutation(p_motor);
	ActivateMotorSixStepAnalog(p_motor);
}

static inline void Motor_SixStep_StartPhaseObserve(Motor_T * p_motor)
{
	Motor_Float(p_motor); //fix redundancy
	BEMF_SetCycleMode(&p_motor->Bemf, BEMF_CYCLE_MODE_PASSIVE); //no blank time
}


extern void Motor_SixStep_CaptureBemfA(Motor_T * p_motor);
extern void Motor_SixStep_CaptureBemfB(Motor_T * p_motor);
extern void Motor_SixStep_CaptureBemfC(Motor_T * p_motor);

//void Motor_SixStep_Jog(Motor_T *p_motor)
//{
//	if (p_motor->JogSteps)
//	{
//		p_motor->JogSteps--;
//	Motor_SixStep_ActivateSector(p_motor, p_motor->NextSector);
//	}
//}

#endif
