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
    @brief
    @version V0
*/
/**************************************************************************/
#include "MotorUser.h"

#include "Peripheral/Pin/Debounce.h"
#include "Peripheral/Pin/Pin.h"

#include "Math/Linear/Linear_ADC.h"
#include "Math/Linear/Linear.h"

#include <stdint.h>
#include <stdbool.h>

void MotorUser_Init(MotorUser_T * p_motorUser, const MotorUser_Consts_T * p_consts)
{
	p_motorUser->P_MOTOR_USER_CONST = p_consts;

	Debounce_Init(&p_motorUser->InputAnalog.PinBrake, 		&p_consts->HAL_PIN_BRAKE, 		&p_consts->P_DEBOUNCE_TIMER, 5U);	//5millis
	Debounce_Init(&p_motorUser->InputAnalog.PinThrottle,	&p_consts->HAL_PIN_THROTTLE, 		&p_consts->P_DEBOUNCE_TIMER, 5U);	//5millis
	Debounce_Init(&p_motorUser->InputAnalog.PinForward, 	&p_consts->HAL_PIN_FORWARD, 		&p_consts->P_DEBOUNCE_TIMER, 5U);	//5millis
	Debounce_Init(&p_motorUser->InputAnalog.PinReverse, 	&p_consts->HAL_PIN_REVERSE, 		&p_consts->P_DEBOUNCE_TIMER, 5U);	//5millis

	//uncalibrated default
	Linear_ADC_Init(&p_motorUser->InputAnalog.UnitThrottle, 	0U, 4095U, 50U);
	Linear_ADC_Init(&p_motorUser->InputAnalog.UnitBrake,		0U, 4095U, 50U);
}

//static inline bool MotorUser_Analog_GetThrottleRelease(MotorUser_T * p_motorUser)
//{
//	bool isReleased;
//
//	if (p_motorUser->InputValueThrottle < p_motorUser->InputValueThrottlePrev)
//	{
//		if ((p_motorUser->InputValueThrottlePrev - p_motorUser->InputValueThrottle) > (65535U/10U))
//		{
//			isReleased == true;
//		}
//	}
//	else
//	{
//		isReleased == false;
//	}
//
//	return isReleased;
//}

void MotorUser_Analog_CaptureInput_IO(MotorUser_T * p_motorUser)
{
	Debounce_CaptureState_IO(&p_motorUser->InputAnalog.PinBrake);
	Debounce_CaptureState_IO(&p_motorUser->InputAnalog.PinThrottle);
	Debounce_CaptureState_IO(&p_motorUser->InputAnalog.PinForward);
	Debounce_CaptureState_IO(&p_motorUser->InputAnalog.PinReverse);

	p_motorUser->InputSwitchBrake 		= Debounce_GetState(&p_motorUser->InputAnalog.PinBrake);
	p_motorUser->InputSwitchThrottle 	= Debounce_GetState(&p_motorUser->InputAnalog.PinThrottle);
	p_motorUser->InputSwitchForward 	= Debounce_GetState(&p_motorUser->InputAnalog.PinForward);
	p_motorUser->InputSwitchReverse 	= Debounce_GetState(&p_motorUser->InputAnalog.PinReverse);
	//p_motorUser->InputSwitchNeutral = Debounce_GetState(p_motorUser->MotorUserAnalog.PinNeutral);
	p_motorUser->InputSwitchNeutral = p_motorUser->InputSwitchForward | p_motorUser->InputSwitchReverse ? false : true;

	p_motorUser->InputValueThrottlePrev 	= p_motorUser->InputValueThrottle;
	p_motorUser->InputValueBrakePrev 		= p_motorUser->InputValueBrake;
	p_motorUser->InputValueThrottle 		= Linear_ADC_CalcFractionUnsigned16(&p_motorUser->InputAnalog.UnitThrottle, 	*p_motorUser->P_MOTOR_USER_CONST->P_THROTTLE_ADCU);
	p_motorUser->InputValueBrake 			= Linear_ADC_CalcFractionUnsigned16(&p_motorUser->InputAnalog.UnitBrake, 		*p_motorUser->P_MOTOR_USER_CONST->P_BRAKE_ADCU);

//	p_motorUser->IsThrottleRelease = MotorUser_Analog_GetThrottleRelease();


}

void MotorUser_Serial_CaptureInput_IO(MotorUser_T * p_motorUser)
{
//	if(Thread_PollTimerCompletePeriodic(&TimerSeconds) == true)
//	{
//		Serial_RestartRx_IO(&Serial1);
//	}
}


void MotorUser_CaptureInput_IO(MotorUser_T * p_motorUser)
{
	switch (p_motorUser->InputMode)
	{
	case MOTOR_INPUT_MODE_ANALOG:
		break;

	case MOTOR_INPUT_MODE_SERIAL:
		break;


//	case MOTOR_INPUT_MODE_DISABLED:
//		break;

	default:
		break;
	}

}


void MotorUser_WriteOutput(const MotorUser_T * p_motorUser)
{
//alarm

}


//Motor Functions
#include "Motor/Motor.h" //MotorUser_SetInput(Motor_T * p_motorDest)

//write to motor
void MotorUser_SetMotorInput(const MotorUser_T * p_motorUser, Motor_T * p_motorDest)
{
	//proc inputs
	//	if(GetSwitchEdge(DirectionButton)) {StateMachine_Semisynchronous_ProcInput(p_motorUser->StateMachine, PIN_FUNCTION_FORWARD);}
	//	if(GetSwitchEdge(DirectionButton)) {StateMachine_Semisynchronous_ProcInput(p_motorUser->StateMachine, PIN_FUNCTION_REVERSE);}

	bool activeControl;

	if (
			!p_motorUser->InputSwitchNeutral //&&
//			!MotorUser_Analog_GetThrottleRelease()
		)
	{
		if (p_motorUser->InputSwitchBrake == true)
		{
			if (p_motorDest->Parameters.BrakeMode == MOTOR_BRAKE_MODE_PASSIVE)
			{
				activeControl = false;
			}
			else
			{
//				 (Motor_GetSpeed(p_motorDest) == 0)
				activeControl = true;
			}
		}
		else
		{
			if (p_motorUser->InputValueThrottle < 5U) // less then threshold
			{
				activeControl = false;
			}
			else
			{
				activeControl = true;
			}
		}
	}
	else
	{
		activeControl = false;
	}

	if (activeControl == false)
	{
//		Motor_SetActiveControl(p_motorDest, false);
//		p_motorDest->IsDirectionNeutral = true;
		p_motorDest->IsActiveControl = false;
//		Motor_SetUserCmd(p_motorDest, 0U);
	}
	else
	{
//		p_motorDest->IsDirectionNeutral = false;
		p_motorDest->IsActiveControl = true;

		if (p_motorUser->InputSwitchForward == true) // ^ reverse direction
		{
			Motor_SetDirectionInput(p_motorDest, MOTOR_DIRECTION_CCW);
		}
		else if (p_motorUser->InputSwitchReverse == true)
		{
			Motor_SetDirectionInput(p_motorDest, MOTOR_DIRECTION_CW);
		}

		//set ramp per second
		if (p_motorUser->InputSwitchBrake == true)
		{
	//			if (p_motor->Parameters.BrakeMode == MOTOR_BRAKE_MODE_SCALAR)
	//			{
			Motor_SetUserCmd(p_motorDest, (p_motorUser->InputValueBrake + p_motorUser->InputValueBrakePrev) / 2U);
			Motor_SetRampDecelerate(p_motorDest, 0);
	//			}
		}
		else
		{
			Motor_SetUserCmd(p_motorDest, (p_motorUser->InputValueThrottle + p_motorUser->InputValueThrottlePrev) / 2U);
			Motor_SetRampAccelerate(p_motorDest, 0);
		}
	}
}



void MotorUser_SetUserOutput(MotorUser_T * p_motorUser, const Motor_T * p_motorSrc)
{

}

void MotorUser_SetUserOutput_Units(MotorUser_T * p_motorUser, const Motor_T * p_motorSrc)
{

}

void MotorUser_SetUserOutput_ADCU(MotorUser_T * p_motorUser, const Motor_T * p_motorSrc)
{

}
