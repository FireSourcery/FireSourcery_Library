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
    @file 	Motor.h
    @author FireSoucery
    @brief  1 instance for all motor, input output control
    @version V0
*/
/**************************************************************************/
#ifndef MOTOR_USER_H
#define MOTOR_USER_H

#include "Motor/Config.h"

#include "Peripheral/Pin/Debounce.h"
#include "Peripheral/Pin/Pin.h"
#include "Peripheral/Serial/Serial.h"

#include "Math/Linear/Linear_ADC.h"
#include "Math/Linear/Linear.h"

#include <stdint.h>
#include <stdbool.h>


typedef enum
{
	MOTOR_INPUT_MODE_ANALOG,
	MOTOR_INPUT_MODE_SERIAL,
	MOTOR_INPUT_MODE_CAN,
}
MotorUser_InputMode_T;

typedef struct
{
	Linear_T UnitThrottle;
	Linear_T UnitBrake;

	Debounce_T PinBrake;
	Debounce_T PinThrottle;
	Debounce_T PinForward;
	Debounce_T PinReverse;
}
MotorUser_Analog_T; //analog in


////per motor outputs
//typedef struct
//{
//
//
//}
//MotorUser_Ouputs_T;


#ifdef CONFIG_MOTOR_ADC_8
	typedef uint16_t adc_t;
#elif defined(CONFIG_MOTOR_ADC_16)
	typedef uint16_t adc_t;
#endif

typedef const struct
{
	//analog in
	const HAL_Pin_T HAL_PIN_BRAKE;
	const HAL_Pin_T HAL_PIN_THROTTLE;
	const HAL_Pin_T HAL_PIN_FORWARD;
	const HAL_Pin_T HAL_PIN_REVERSE;
	volatile const uint32_t * const P_DEBOUNCE_TIMER; //millis timer
	volatile const adc_t * const P_THROTTLE_ADCU;
	volatile const adc_t * const P_BRAKE_ADCU;
	const uint32_t LINEAR_V_ADC_VREF;
	const uint32_t LINEAR_V_ADC_BITS;

	//per controller sensors?
//	volatile const adc_t * const P_VACC_ADCU;
//	volatile const adc_t * const P_VSENSE_ADCU;
//	volatile const adc_t * const P_HEAT_PCB_ADCU;

}
MotorUser_Consts_T;

typedef struct
{
	const MotorUser_Consts_T * P_MOTOR_USER_CONST;

	MotorUser_Analog_T InputAnalog;
//	Serial_T * p_Serial; //serial maybe shared
	//MotorProtocol_T ;

	MotorUser_InputMode_T InputMode;

	//input values buffer
	volatile bool InputSwitchBrake;
	volatile bool InputSwitchThrottle;
	volatile bool InputSwitchForward;
	volatile bool InputSwitchReverse;
	volatile bool InputSwitchNeutral;
	volatile bool InputIsThrottleRelease;

	volatile uint16_t InputValueThrottle;
	volatile uint16_t InputValueThrottlePrev;
	volatile uint16_t InputValueBrake;
	volatile uint16_t InputValueBrakePrev;

	//output
	//todo alarm Blinky_T Alarm

}
MotorUser_T;



//priority brake thread
//static inline void MotorUser_PollBrake(MotorUser_T * p_motorUser, Motor_T * p_motor)
//{
//	Debounce_CaptureState_IO(p_motorUser->MotorUserAnalog.PinBrake);
//	p_motorUser->InputSwitchBrake 		= Debounce_GetState(p_motorUser->MotorUserAnalog.PinBrake);
//
//	if (p_motorUser->InputSwitchBrake == true)
//	{
//			Motor_SetUserCmd(p_motor, (p_motorUser->InputValueBrake + p_motorUser->InputValueBrake) / 2U);
//			Motor_SetRampDecelerate(p_motor, 0);
//	}
//}


extern void MotorUser_Init(MotorUser_T * p_motorUser, MotorUser_Consts_T * p_consts);
#endif
