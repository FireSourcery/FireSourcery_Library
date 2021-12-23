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
    @file 	Motor_User.h
    @author FireSoucery
    @brief  User access scope. Functions include error checking.
    @version V0
*/
/******************************************************************************/
#ifndef MOTOR_USER_H
#define MOTOR_USER_H

#include "Motor.h"


//#include "Config.h"
////#include "Default.h"
//
//#include "MotorStateMachine.h"
//
//#include "Transducer/Phase/Phase.h"
//#include "Transducer/Hall/Hall.h"
//#include "Transducer/BEMF/BEMF.h"
//
//#include "Transducer/Encoder/Encoder_Motor.h"
//#include "Transducer/Encoder/Encoder_DeltaT.h"
//#include "Transducer/Encoder/Encoder_DeltaD.h"
//#include "Transducer/Encoder/Encoder.h"
//
//#include "System/StateMachine/StateMachine.h"
//#include "System/Thread/Thread.h"
//
//#include "Math/Linear/Linear_ADC.h"
//#include "Math/Linear/Linear_Voltage.h"
//#include "Math/Linear/Linear.h"

#include <stdint.h>
#include <stdbool.h>


//void Motor_User_StartControl(Motor_T * p_motor)
//{
//	StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_RUN);
//	p_motor->IsActiveControl = true;
//}

static inline void Motor_User_SetThrottle(Motor_T * p_motor, uint16_t throttle)
{
 //		if throttle > p_motor->VPwm)
	Motor_SetUserCmd(p_motor, throttle);
	Motor_SetRampUp(p_motor, throttle);
	p_motor->IsActiveControl = true;
}


static inline void Motor_User_SetBrake(Motor_T * p_motor, uint16_t brake)
{
	if (p_motor->Parameters.BrakeMode == MOTOR_BRAKE_MODE_PASSIVE)
	{
		p_motor->IsActiveControl = false; //or set state?
	}
	else
	{
		Motor_SetUserCmd(p_motor, brake);
		Motor_SetRampDown(p_motor, brake);
		p_motor->IsActiveControl = true;
	}
}

static inline void Motor_User_Disable(Motor_T * p_motor)
{
	Phase_Float(&p_motor->Phase);
	p_motor->IsActiveControl = false;
}

//set buffered direction, check on state machine run
static inline void Motor_User_SetDirection(Motor_T * p_motor, Motor_Direction_T direction)
{
//		if (p_motor->Direction != p_motor->InputDirection)//direction reversed
//		{
//			Blinky_SetOnTime(&p_Motor->Alarm, 1000)
//		}
	p_motor->DirectionInput = direction;
}

static inline void Motor_User_SetNThrottle(Motor_T * p_motor, uint8_t motorCount, uint16_t throttle)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_SetThrottle(&p_motor[iMotor], throttle);
	}
}



static inline uint32_t Motor_User_GetBemf_Frac16(Motor_T * p_motor)	{return Linear_Voltage_CalcFractionUnsigned16(&p_motor->CONFIG.UNIT_V_ABC, BEMF_GetVBemfPeak_ADCU(&p_motor->Bemf));}
static inline uint32_t Motor_User_GetBemf_MilliV(Motor_T * p_motor)	{return Linear_Voltage_CalcMilliV(&p_motor->CONFIG.UNIT_V_ABC, BEMF_GetVBemfPeak_ADCU(&p_motor->Bemf));}

static inline uint32_t Motor_User_GetVPos_MilliV(Motor_T * p_motor)	{return Linear_Voltage_CalcMilliV(&p_motor->CONFIG.UNIT_V_POS, p_motor->AnalogResults[MOTOR_ANALOG_CHANNEL_VPOS]);}
static inline uint32_t Motor_User_GetSpeed_RPM(Motor_T *p_motor) 	{return p_motor->Speed_RPM;}

#endif
