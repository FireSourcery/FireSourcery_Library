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
#include "Motor_SixStep.h"
#include "Motor_FOC.h"

#include "Config.h"
#include "Default.h"

//#include "Motor/Instance/MotorAnalog.h"
#include "Motor/Instance/MotorStateMachine.h"
//#include "OS/StateMachine/StateMachine.h"

#include "System/Thread/Thread.h"

#include "Transducer/Encoder/Encoder_Motor.h"
#include "Transducer/Encoder/Encoder.h"
#include "Transducer/Phase/Phase.h"
#include "Transducer/Phase/HAL.h"

//#include "Math/Q/Q.h"
//#define CONFIG_MOTOR_COUNT 1

//Motor_T Motors[CONFIG_MOTOR_COUNT];

//Motor_Common_T MotorCommon //all motor shared variables
//{
//
//}

/*
 * General Init
 */
void Motor_Init(Motor_T * p_motor, const Motor_Init_T * p_motorInit)
{
	/*HW Wrappers Init */
	Phase_Init
	(
		&p_motor->Phase,
		&p_motorInit->HAL_PHASE,
		p_motorInit->MOTOR_PWM_FREQ  //firmware const
//		0,	0,	0,	0,	0,	0
	);

	Phase_Polar_ActivateMode(&p_motor->Phase, PHASE_MODE_UNIPOLAR_1);

	//	BEMF_Init
	//	(
	//		&p_motor->Bemf,
	//
	//	);
	//
	//	Hall_Init
	//	(
	//		&p_motor->Hall,
	//
	//	);

	FOC_Init(&p_motor->Foc);

	p_motor->ControlMode = MOTOR_CONTROL_MODE_OPEN_LOOP;
	p_motor->SensorMode = MOTOR_SENSOR_MODE_OPEN_LOOP;

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

	//all sixstep modes and hall foc mode user hall version
	Encoder_Motor_InitHall
	(
		&p_motor->Encoder,
		&p_motorInit->HAL_ENCODER,
		p_motorInit->HALL_ENCODER_TIMER_COUNTER_MAX,
		p_motorInit->HALL_ENCODER_TIMER_COUNTER_FREQ,
		p_motorInit->MOTOR_PWM_FREQ,
		p_motorInit->ENCODER_ANGLE_RES_BITS,
		7,
		0
	);



	p_motor->Direction = MOTOR_DIRECTION_CW;
	Hall_SetDirection(&p_motor->Hall, HALL_DIRECTION_CW);



	Thread_InitThreadPeriodic_Period
	(
		&p_motor->ThreadTimer,
		&p_motor->ControlTimer,
		20000U,
		0,
		0,
		1U
	);


//	iff anlog module algo supports matching channel to adc virtualization layer
//	Analog_Init
//	(
//		&p_motor->Analog,
//		p_motorInit->p_AdcRegMap,
//		p_motorInit->N_Adc,
//		p_motorInit->M_HwBuffer,
//		MOTOR_ANALOG_ADC_CHANNEL_COUNT,
//		p_motorInit->p_AdcChannelPinMap,
//		p_motor->AnalogChannelMapResults,
//		p_motor->AnalogIndexesBuffer,
//		p_motor
//	);

	//MotorStateMachine_Init(p_Motor);
	//Linear_Init(&(p_Motor->VFMap), vPerRPM, 1, vOffset); //f(freq) = voltage
	//Linear_Voltage_Init(&DividerBatteryVoltage, R1, R2, 5, 12);
	//load param from flash?


//testing
//	Motor_StartAlign(p_motor);
//	Motor_SixStep_Start(p_motor);
//	Motor_FOC_StartAngleControlMode(p_motor);
//	p_motor->ControlMode = MOTOR_CONTROL_MODE_OPEN_LOOP;
//	p_motor->SensorMode = MOTOR_SENSOR_MODE_OPEN_LOOP;

//	Phase_ActuateDutyCycle_Ticks(&p_motor->Phase, 100, 100, 100);
//	Phase_ActuateState(&p_motor->Phase, 1, 1, 1);
}

void Motor_Align(Motor_T * p_motor)
{
	Phase_ActuateDutyCycle(&p_motor->Phase, 6553U/4U, 0, 0); /* Align Phase A 10% pwm */
	Phase_ActuateState(&p_motor->Phase, 1, 1, 1);
}

void Motor_StartAlign(Motor_T * p_motor)
{
	Motor_Align(p_motor);
	//should this be in state wrapper?
	p_motor->ControlTimer = DEFAULT_CONTROL_FREQ_HZ; /* set timer for 1 second */
}

void Motor_SetDirection(Motor_T * p_motor)
{
//	 Hall_SetDirection(&p_motor->Hall);
//	 BEMF_SetDirection(&p_motor->Bemf);
//	 Motor_FOC_SetDirection(&p_motor);
}

/*!
	@brief Load from flash
	@param p_motor
 */
void Motor_LoadParameters(Motor_T * p_motor)
{

//	p_motor->Parameters.FocOpenLoopVq = DEFAULT_FOC_OPEN_LOOP_VQ;
	/*
	 * Load default via HAL
	 */
//#ifdef CONFIG_MOTOR_LOAD_PARAMETERS_DEFAULT
//	p_motor->Parameters.FocOpenLoopVq = DEFAULT_FOC_OPEN_LOOP_VQ;
//
//#elif defined(CONFIG_MOTOR_LOAD_PARAMETERS_FLASH)
//	p_motor->Parameters.FocOpenLoopVq = Flash_Read(&p_motor->Flash, MEMORY_ADDRESS_FOC_OPEN_LOOP_VOLTAGE);
//#endif
}


