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
#include "Default.h"

//#include "Motor/Instance/MotorAnalog.h"
#include "Motor/Instance/MotorStateMachine.h"
//#include "OS/StateMachine/StateMachine.h"

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
void Motor_Init(Motor_T * p_motor, const Motor_Init_T * p_motorInitStruct)
{
	/*HW Wrappers Init */
	Phase_Init
	(
		&p_motor->Phase,
		&p_motorInitStruct->HAL_PHASE,
		p_motorInitStruct->PWM_PERIOD,  //firmware const, change to init struct?
		0,	0,	0,	0,	0,	0
	);

	Encoder_Motor_Init
	(
		&p_motor->Encoder,
		&p_motorInitStruct->HAL_ENCODER,
		p_motorInitStruct->PWM_FREQ, //firmware const
		1, //DEFAULT_ENCODER_DISTANCE_PER_COUNT, //runtime variable
		8192, //p_motorInitStruct->ENCODER_TIMER_COUNTER_MAX_ENCODER + 1, //runtime variable
		p_motorInitStruct->ANGLE_RES_BITS, //firmware const
		7 //DEFAULT_POLE_PAIRS //run time variable
	);

	/*
	 * breaking encapsulation for mapping
	 */
	FOC_Init(&p_motor->Foc, p_motorInitStruct->PWM_PERIOD, &p_motor->Phase.PeriodA, &p_motor->Phase.PeriodB, &p_motor->Phase.PeriodC);


//	Analog_Init
//	(
//		&p_motor->Analog,
//		p_motorInitStruct->p_AdcRegMap,
//		p_motorInitStruct->N_Adc,
//		p_motorInitStruct->M_HwBuffer,
//		MOTOR_ANALOG_ADC_CHANNEL_COUNT,
//		p_motorInitStruct->p_AdcChannelPinMap,
//		p_motor->AnalogChannelMapResults,
//		p_motor->AnalogIndexesBuffer,
//		p_motor
//	);

	//temp foc init

	//MotorStateMachine_Init(p_Motor);
	//Linear_Init(&(p_Motor->VFMap), vPerRPM, 1, vOffset); //f(freq) = voltage
	//Linear_Voltage_Init(&DividerBatteryVoltage, R1, R2, 5, 12);
	//load param from flash?
}




//#define DEFAULT_ALIGN_VOLTAGE           (30) //12% pwm 30/255
//#define DEFAULT_ALIGN_DURATION          (1000) //1000ms

void Motor_Align(Motor_T * p_motor)
{
//	Phase_ActuatePeriod_DutyCycle(&p_motor->Phase, 6553, 0, 0); /* Align Phase A. 10% pwm */
//	Phase_ActuateState(&p_motor->Phase, 1, 1, 1);

	Phase_ActuatePeriod(&p_motor->Phase, 100, 0, 0); /* Align Phase A. 10% pwm */
	Phase_ActuateState(&p_motor->Phase, 1, 0, 0);
}

void Motor_StartAlign(Motor_T * p_motor)
{
	Motor_Align(p_motor);
	//should this be in state wrapper?
	p_motor->TimerCounter = DEFAULT_CONTROL_FREQ_HZ; /* set timer for 1 second */
}


