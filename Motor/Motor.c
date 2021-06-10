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
    @file 	Motor.c
    @author FireSoucery
    @brief  Motor module conventional function definitions.
    @version V0
*/
/**************************************************************************/
#include "Motor.h"

#include "Config.h"
//#include "Default.h"

#include "System/MotorFlash.h"
#include "System/MotorStateMachine.h"


#include "System/StateMachine/StateMachine.h"
#include "System/Thread/Thread.h"

#include "Transducer/Encoder/Encoder_Motor.h"
#include "Transducer/Encoder/Encoder.h"
#include "Transducer/Phase/Phase.h"
#include "Transducer/Phase/HAL.h"

#include "Math/Linear/Linear_ADC.h"
#include "Math/Linear/Linear.h"

/*
 * General Init
 */
void Motor_Init(Motor_T * p_motor, const Motor_Init_T * p_motorInit)
{
	p_motor->p_Init = p_motorInit;

	//if first time boot, use serial com instead

	MotorStateMachine_Init(p_motor);
}

//state machine init-state run
void Motor_InitReboot(Motor_T * p_motor)
{
	const Motor_Init_T * p_motorInit = p_motor->p_Init; //Load Compile time consts


	/*
	 * Motor Utility Instance Init, 1 for all motor
	 */

	//call from outside since there is only 1
//	MotorFlash_Init
//	(
//		&p_motorInit->HAL_FLASH
//	);
	//	MotorShell_Init();

	MotorFlash_LoadParameterAll(p_motor);

	/*
	 * HW Wrappers Init
	 */
	Phase_Init
	(
		&p_motor->Phase,
		&p_motorInit->HAL_PHASE,
		p_motorInit->PHASE_PWM_PERIOD
	);

	//ALL FOC modes except hall sensor use standard encoder
//	Encoder_Motor_Init
//	(
//		&p_motor->Encoder,
//		&p_motorInit->HAL_ENCODER,
//		p_motorInit->MOTOR_PWM_FREQ, //firmware const
//		p_motorInit->ENCODER_ANGLE_RES_BITS, //firmware const
//		7, //run time variable
//		8192, //runtime variable
//		1 //runtime variable
//	);

	//all sixstep modes and hall foc mode use CaptureTime
	Encoder_Motor_InitCaptureTime
	(
		&p_motor->Encoder,
		&p_motorInit->HAL_ENCODER,
		p_motorInit->HALL_ENCODER_TIMER_COUNTER_MAX,
		p_motorInit->HALL_ENCODER_TIMER_COUNTER_FREQ,
		p_motorInit->MOTOR_PWM_FREQ,
		16U, //always 16 bits
		p_motor->Parameters.PolePairs,
		p_motor->Parameters.PolePairs*6U,  // use PolePairs * 6 for count per commutation or PolePairs for per erotation
		p_motor->Parameters.EncoderDistancePerCount
	);

	Encoder_InitExtendedDeltaT
	(
		&p_motor->Encoder,
		&p_motor->MillisTimerBase,
		1000U,
		1000U
	);


	Hall_Init
	(
		&p_motor->Hall,
		&p_motorInit->HAL_HALL,
		MOTOR_SECTOR_ID_1,
		MOTOR_SECTOR_ID_2,
		MOTOR_SECTOR_ID_3,
		MOTOR_SECTOR_ID_4,
		MOTOR_SECTOR_ID_5,
		MOTOR_SECTOR_ID_6,
		p_motor->Parameters.HallVirtualSensorInvBMap,
		p_motor->Parameters.HallVirtualSensorAMap,
		p_motor->Parameters.HallVirtualSensorInvCMap,
		p_motor->Parameters.HallVirtualSensorBMap,
		p_motor->Parameters.HallVirtualSensorInvAMap,
		p_motor->Parameters.HallVirtualSensorCMap,
		MOTOR_SECTOR_ERROR_000,
		MOTOR_SECTOR_ERROR_111
	);

	//	BEMF_Init
	//	(
	//		&p_motor->Bemf,
	//
	//	);
	//

	Debounce_Init(&p_motor->PinBrake, 		&p_motorInit->HAL_PIN_BRAKE, 		&p_motor->MillisTimerBase, 5U);	//5millis
	Debounce_Init(&p_motor->PinThrottle,	&p_motorInit->HAL_PIN_THROTTLE, 	&p_motor->MillisTimerBase, 5U);	//5millis
	Debounce_Init(&p_motor->PinForward, 	&p_motorInit->HAL_PIN_FORWARD, 		&p_motor->MillisTimerBase, 5U);	//5millis
	Debounce_Init(&p_motor->PinReverse, 	&p_motorInit->HAL_PIN_REVERSE, 		&p_motor->MillisTimerBase, 5U);	//5millis

	//	if analog module algo supports matching channel to adc virtualization layer
	//	Analog_Init
	//	(
	//	);

	FOC_Init(&p_motor->Foc);

	Thread_InitThreadPeriodic_Period //Timer only mode
	(
		&p_motor->ControlTimerThread,
		&p_motor->ControlTimerBase,
		p_motorInit->MOTOR_PWM_FREQ,
		1U,
		0U,
		0U
	);

	Thread_InitThreadPeriodic_Period //Timer only mode
	(
		&p_motor->MillisTimerThread,
		&p_motor->MillisTimerBase,
		1000U,
		1U,
		0U,
		0U
	);

	Thread_InitThreadPeriodic_Period //Timer only mode
	(
		&p_motor->SecondsTimerThread,
		&p_motor->MillisTimerBase,
		1000U,
		1000U,
		0U,
		0U
	);


	//initial runtime config settings
	Phase_Polar_ActivateMode(&p_motor->Phase, PHASE_MODE_UNIPOLAR_1);

	//uncalibrated default
	Linear_ADC_Init(&p_motor->UnitThrottle, 0U, 4095U, 100U);
	Linear_ADC_Init(&p_motor->UnitBrake, 0U, 4095U, 100U);

	/*
	 * Run calibration later, default zero to middle adc
	 */
	Linear_ADC_Init(&p_motor->UnitIa, 2048U, 4095U, 120U); //temp 120amp
	Linear_ADC_Init(&p_motor->UnitIb, 2048U, 4095U, 120U);
	Linear_ADC_Init(&p_motor->UnitIc, 2048U, 4095U, 120U);

	Linear_Voltage_Init(&p_motor->UnitVBus, 42U, p_motorInit->LINEAR_V_BUS_R1, p_motorInit->LINEAR_V_BUS_R2, 50U, 12U);
	Linear_Voltage_Init(&p_motor->UnitVabc, 42U, p_motorInit->LINEAR_V_ABC_R1, p_motorInit->LINEAR_V_ABC_R2, 50U, 12U);

	//Linear_Init(&(p_Motor->VFMap), vPerRPM, 1, vOffset); //f(freq) = voltage

	p_motor->Direction 		= MOTOR_DIRECTION_CCW;
	p_motor->DirectionInput = MOTOR_DIRECTION_CCW;
	p_motor->SpeedFeedback_RPM = 0U;

}


void Motor_ActivateAlign(Motor_T * p_motor)
{
	Phase_ActuateDutyCycle(&p_motor->Phase, 6553U/4U, 0, 0); /* Align Phase A 10% pwm */
	Phase_ActuateState(&p_motor->Phase, 1, 1, 1);
}

void Motor_StartAlign(Motor_T * p_motor)
{
	Thread_SetTimer(&p_motor->ControlTimerThread, 20000U);
	Motor_ActivateAlign(p_motor);
}

bool Motor_WaitAlign(Motor_T *p_motor)
{
	bool status;
	if (Thread_PollTimer(&p_motor->ControlTimerThread) == true)	{status = true;}
	else														{status = false;}
	return status;
}

Motor_AlignMode_T Motor_GetAlignMode(Motor_T *p_motor)
{
	Motor_AlignMode_T alignMode;

	if (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_HALL)
	{
		alignMode = MOTOR_ALIGN_MODE_DISABLE;
	}
//	else
//	{
//		//		if useHFI alignMode= MOTOR_ALIGN_MODE_HFI;
//		//		else
//				alignMode = MOTOR_ALIGN_MODE_ALIGN;
//	}
//

	return alignMode;
}

void Motor_SetDirection(Motor_T * p_motor, Motor_Direction_T direction)
{
	p_motor->Direction = direction;

	if (direction = MOTOR_DIRECTION_CW)
	{
		Hall_SetDirection(&p_motor->Hall, HALL_DIRECTION_CW);
	}
	else
	{
		Hall_SetDirection(&p_motor->Hall, HALL_DIRECTION_CCW);
	}


//	switch (direction)
//	{
//
//	case MOTOR_DIRECTION_CW:
//		Hall_SetDirection(&p_motor->Hall, HALL_DIRECTION_CW);
//		//	 BEMF_SetDirection(&p_motor->Bemf, direction);
//		//	 FOC_SetDirection(&p_motor);
//		break;
//
//	case MOTOR_DIRECTION_CCW:
//		Hall_SetDirection(&p_motor->Hall, HALL_DIRECTION_CCW);
//		//	 BEMF_SetDirection(&p_motor->Bemf, direction);
//		//	 FOC_SetDirection(&p_motor);
//		break;
//
////	case	MOTOR_DIRECTION_NEUTRAL:
////		break;
//
//	default:
//		break;
//	}


}


void Motor_StartCalibrateHall(Motor_T * p_motor)
{
	p_motor->ControlTimerBase = 0U;
	Thread_SetTimer(&p_motor->ControlTimerThread, 20000U); //Parameter.HallCalibrationTime
}

//120 degree hall aligned with phase
bool Motor_CalibrateHall(Motor_T * p_motor)
{
	static uint8_t state = 0; //limits calibration to 1 at a time;

	const uint16_t duty = 65536 / 10/4;

	bool isComplete = false;

	if (Thread_PollTimer(&p_motor->ControlTimerThread) == true)
	{
		switch (state)
		{
		case 0U:
			Phase_ActuateState(&p_motor->Phase, true, true, true);

			Phase_ActuateDutyCycle(&p_motor->Phase, duty, 0U, 0U);
			state++;
			break;

		case 1U:
			Hall_CalibrateSensorAPhaseBC(&p_motor->Hall, MOTOR_SECTOR_ID_2);

			Phase_ActuateDutyCycle(&p_motor->Phase, duty, duty, 0U);
			state++;
			break;

		case 2U:
			Hall_CalibrateSensorInvCPhaseBA(&p_motor->Hall, MOTOR_SECTOR_ID_3);

			Phase_ActuateDutyCycle(&p_motor->Phase, 0U, duty, 0);
			state++;
			break;

		case 3U:
			Hall_CalibrateSensorBPhaseCA(&p_motor->Hall, MOTOR_SECTOR_ID_4);

			Phase_ActuateDutyCycle(&p_motor->Phase, 0U, duty, duty);
			state++;
			break;

		case 4U:
			Hall_CalibrateSensorInvAPhaseCB(&p_motor->Hall, MOTOR_SECTOR_ID_5);

			Phase_ActuateDutyCycle(&p_motor->Phase, 0U, 0U, duty);
			state++;
			break;

		case 5U:
			Hall_CalibrateSensorCPhaseAB(&p_motor->Hall, MOTOR_SECTOR_ID_6);

			Phase_ActuateDutyCycle(&p_motor->Phase, duty, 0U, duty);
			state++;
			break;

		case 6U:
			Hall_CalibrateSensorInvBPhaseAC(&p_motor->Hall, MOTOR_SECTOR_ID_1);

			Phase_ActuateState(&p_motor->Phase, false, false, false);
			state = 0;
			isComplete = true;
			break;

		default:
			break;

		}
	}

	return isComplete;
}




//void Motor_OnBlock(Motor_T * p_motor)
//{
//
//}
