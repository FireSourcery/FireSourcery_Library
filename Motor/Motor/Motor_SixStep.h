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
    @file 	Motor_Step.h
    @author FireSoucery
    @brief  Motor Six Step submodule.
    		Six Step commutation strategy. Polar PWM
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
 * On Adc measure of phase complete
 */

static inline void Motor_SixStep_CaptureBemfA(Motor_T * p_motor)
{
	Debug_CaptureElapsed(3);

}

static inline void Motor_SixStep_CaptureBemfB(Motor_T * p_motor)
{

}
static inline void Motor_SixStep_CaptureBemfC(Motor_T * p_motor)
{

}

static inline bool Motor_SixStep_GetBemfReliable(Motor_T * p_motor) {return (BEMF_GetZeroCrossingCounter(&p_motor->Bemf) > 10U) ? true : false;}

/*
 * Map once per commutation
 *
 */
static inline void Motor_SixStep_MapBemf(Motor_T * p_motor)
{
 	switch (p_motor->CommutationSector)
	{
	case MOTOR_SECTOR_ID_0:
		//set error
		break;

	case MOTOR_PHASE_AC: //Phase AC
		if (p_motor->Direction == MOTOR_DIRECTION_CCW) 	{p_motor->NextSector = MOTOR_SECTOR_ID_2; BEMF_MapCcwPhaseAC(&p_motor->Bemf);}
		else											{p_motor->NextSector = MOTOR_SECTOR_ID_6; BEMF_MapCwPhaseAC(&p_motor->Bemf);}

//		Phase_Polar_ActivateAC(&p_motor->Phase, p_motor->CommutationSector, p_motor->VPwm); 	//change sector and update pwm



//		p_motor->p_BemfConversionActive = &p_motor->CONFIG.CONVERSION_BEMF_B; //sets as conversion set for commutation cycle
//		AnalogN_EnqueueFrontConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_BEMF_B);
//		AnalogN_EnqueueFrontConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_BEMF_B_REPEAT);
		break;

	case MOTOR_PHASE_BC:
		if (p_motor->Direction == MOTOR_DIRECTION_CCW) 	{p_motor->NextSector = MOTOR_SECTOR_ID_3; BEMF_MapCcwPhaseBC(&p_motor->Bemf);}
		else											{p_motor->NextSector = MOTOR_SECTOR_ID_1; BEMF_MapCwPhaseBC(&p_motor->Bemf);}

//		p_motor->p_BemfConversionActive = &p_motor->CONFIG.CONVERSION_BEMF_A;
//		AnalogN_EnqueueFrontConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_BEMF_A);
//		AnalogN_EnqueueFrontConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_BEMF_A_REPEAT);
		break;

	case MOTOR_PHASE_BA:
		if (p_motor->Direction == MOTOR_DIRECTION_CCW) 	{p_motor->NextSector = MOTOR_SECTOR_ID_4; BEMF_MapCcwPhaseBA(&p_motor->Bemf);}
		else											{p_motor->NextSector = MOTOR_SECTOR_ID_2; BEMF_MapCwPhaseBA(&p_motor->Bemf);}

		p_motor->p_BemfConversionActive = &p_motor->CONFIG.CONVERSION_BEMF_C;
		break;

	case MOTOR_PHASE_CA:
		if (p_motor->Direction == MOTOR_DIRECTION_CCW) 	{p_motor->NextSector = MOTOR_SECTOR_ID_5; BEMF_MapCcwPhaseCA(&p_motor->Bemf);}
		else											{p_motor->NextSector = MOTOR_SECTOR_ID_3; BEMF_MapCwPhaseCA(&p_motor->Bemf);}

		p_motor->p_BemfConversionActive = &p_motor->CONFIG.CONVERSION_BEMF_B;
		break;

	case MOTOR_PHASE_CB:
		if (p_motor->Direction == MOTOR_DIRECTION_CCW) 	{p_motor->NextSector = MOTOR_SECTOR_ID_6; BEMF_MapCcwPhaseCB(&p_motor->Bemf);}
		else											{p_motor->NextSector = MOTOR_SECTOR_ID_4; BEMF_MapCwPhaseCB(&p_motor->Bemf);}

//		p_motor->p_BemfConversionActive = &p_motor->CONFIG.CONVERSION_BEMF_A;
//		AnalogN_EnqueueFrontConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_BEMF_A);
//		AnalogN_EnqueueFrontConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_BEMF_A_REPEAT);
		//inclue dequeu if need to account for aborted conversions
		break;

	case MOTOR_PHASE_AB:
		if (p_motor->Direction == MOTOR_DIRECTION_CCW) 	{p_motor->NextSector = MOTOR_SECTOR_ID_1; BEMF_MapCcwPhaseAB(&p_motor->Bemf);}
		else											{p_motor->NextSector = MOTOR_SECTOR_ID_5; BEMF_MapCwPhaseAB(&p_motor->Bemf);}

		p_motor->p_BemfConversionActive = &p_motor->CONFIG.CONVERSION_BEMF_C;
		break;

	case MOTOR_SECTOR_ID_7:
		//set error
		break;

	default:
		break;
	}

 	//alternatively
//	if (p_motor->Direction == MOTOR_DIRECTION_CCW)
//	{
//		p_motor->NextSector = (p_motor->CommutationSector + 1U)
// 		if (p_motor->NextSector > 6) p_motor->NextSector = 1;
//	}
//	else
//	{
//		p_motor->NextSector = (p_motor->CommutationSector - 1U) % 6U;
//		p_motor->Bemf.IsBemfRising = !p_motor->Bemf.IsBemfRising;
//	}

// 	p_motor->NextSector = (p_motor->NextSector + 4U) % 6U;
// 	p_motor->Bemf.IsBemfRising = !p_motor->Bemf.IsBemfRising;
}

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

static inline void Motor_SixStep_StartOpenLoop(Motor_T * p_motor) //start active control
{
	#define OPEN_LOOP_START_SPEED 20U
	#define OPEN_LOOP_SPEED 300U

	// if pwm update is only on commutation cycle, must start at sufficient speed for timer driven fixed angle displacements
	Linear_Ramp_Init_Millis(&p_motor->OpenLoopRamp, 20000U, OPEN_LOOP_START_SPEED, OPEN_LOOP_SPEED, 2000U);
	p_motor->OpenLoopRampIndex = 0U;

	p_motor->OpenLoopSpeed_RPM = Linear_Ramp_CalcTargetIncIndex(&p_motor->OpenLoopRamp, &p_motor->OpenLoopRampIndex, 0U);
	p_motor->OpenLoopCommutationPeriod = Encoder_Motor_ConvertMechanicalRpmToInterpolationFreq(&p_motor->Encoder, p_motor->OpenLoopSpeed_RPM);
	Timer_SetPeriod(&p_motor->ControlTimer, p_motor->OpenLoopCommutationPeriod);

	//motor aligned to phase A
	p_motor->NextSector = (p_motor->Direction == MOTOR_DIRECTION_CCW) ? MOTOR_SECTOR_ID_2 : MOTOR_SECTOR_ID_6;
}

/*
 * State loop in 20khz pwm thread
 */
static inline bool Motor_SixStep_PollCommutation(Motor_T * p_motor, Motor_SensorMode_T liveSensorMode)
{
	bool commutation = false;

	switch (liveSensorMode) //active sensor feedback
	{
	case MOTOR_SENSOR_MODE_OPEN_LOOP:
		if(Motor_SixStep_PollOpenLoop(p_motor) == true)
		{
			p_motor->CommutationSector = p_motor->NextSector;
			commutation = true;
		}

		p_motor->VPwm =  (p_motor->UserCmd / 2U);

		// bound to 2.5 to 10 percent
		if (p_motor->VPwm < (65536U/10U/4U))
		{
			p_motor->VPwm = (65536U/10U/4U);
		}
		else if (p_motor->VPwm > 65536U/10U)
		{
			p_motor->VPwm = (65536U/10U);
		}

		break;

	case MOTOR_SENSOR_MODE_BEMF:
		if (BEMF_PollPhasePeriod(&p_motor->Bemf) == true)
		{
			p_motor->CommutationSector = p_motor->NextSector;
			commutation = true;
		}
		break;

	case MOTOR_SENSOR_MODE_HALL:
		if(Hall_PollCaptureSensors(&p_motor->Hall) == true)
		{
			BEMF_CapturePhasePeriod(&p_motor->Bemf);
			p_motor->CommutationSector = (Motor_SectorId_T)Hall_GetCommutationId(&p_motor->Hall); //hall id match motor id
			commutation = true;
		}
		break;

//	case MOTOR_SENSOR_MODE_ENCODER:	 //encoder always use foc
//		break;

	default:
		break;
	}

	if (commutation == true)
	{
		Encoder_DeltaT_Capture(&p_motor->Encoder);
		Encoder_DeltaT_ProcExtendedTimer(&p_motor->Encoder);

		Motor_SixStep_MapBemf(p_motor);
		BEMF_StartPhase(&p_motor->Bemf);
		Debug_CapturePeriod(1);
	}
	else
	{
		BEMF_PollZeroCrossingDetection(&p_motor->Bemf); //always poll zcd with prev capture
		BEMF_ResetCaptureVPhase(&p_motor->Bemf);
	}


	return commutation;
}



extern volatile uint16_t BemfDebugIndex ;

//us 8800
//deltat 5500
/*
 * Active Control
 */
static inline void Motor_SixStep_ProcPhaseControl(Motor_T * p_motor, Motor_SensorMode_T liveSensorMode)
{

	Debug_CapturePeriod(0);
	Debug_CaptureRef(); //begin pwm cycle
//	BemfDebugIndex = 0;

	Motor_SixStep_PollCommutation(p_motor, liveSensorMode);

	if (liveSensorMode != MOTOR_SENSOR_MODE_OPEN_LOOP)
	{
		Motor_ProcRamp(p_motor);
		Motor_ProcControlVariable(p_motor); //todo coordinate openloop
	}

	Phase_Polar_Activate(&p_motor->Phase, p_motor->CommutationSector, p_motor->VPwm); 	//update pwm with cv every pwm cycle

	// if(Motor_ProcControlVariable)
//	Phase_Polar_ActivatePwm(&p_motor->Phase, p_motor->VPwm); 	//update pwm with cv every pwm cycle
}

//Start Spin from Stop
static inline void Motor_SixStep_StartPhaseControl(Motor_T * p_motor, Motor_SensorMode_T liveSensorMode) //start active control
{
	Encoder_Reset(&p_motor->Encoder);
	BEMF_SetObserveMode(&p_motor->Bemf, BEMF_MODE_COMMUTATION);
	p_motor->ControlTimerBase = 0U; //overflow at 20Khz, 59 hours
//	Encoder_DeltaT_SetInitial(&p_motor->Encoder, 5U);

	switch (liveSensorMode)
	{
	case MOTOR_SENSOR_MODE_OPEN_LOOP:
		Motor_SixStep_StartOpenLoop(p_motor);
		break;

	case MOTOR_SENSOR_MODE_BEMF:
		//bemf poll uses timer reference, auto sets timer cycle period - (current time - commutation time)
		//variable set from timer mode
//			p_motor->IsStartUp = true;

		break;

	case MOTOR_SENSOR_MODE_HALL:
		Hall_CaptureSensors(&p_motor->Hall);
		p_motor->CommutationSector = Hall_GetCommutationId(&p_motor->Hall);
		break;

//	case MOTOR_SENSOR_MODE_ENCODER: //encoder always use foc
//		break;

	default:
		break;
	}
	//		Motor_SixStep_MapBemf(p_motor);
	//		BEMF_StartPhase(&p_motor->Bemf);
}

/*
 * Passive Observe
 */
static inline void Motor_SixStep_ProcPhaseObserve(Motor_T * p_motor, Motor_SensorMode_T liveSensorMode)
{
	Motor_SixStep_PollCommutation(p_motor, liveSensorMode);

}

static inline void Motor_SixStep_StartPhaseObserve(Motor_T * p_motor)
{
	BEMF_SetObserveMode(&p_motor->Bemf, BEMF_MODE_PASSIVE); //no blank time
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
