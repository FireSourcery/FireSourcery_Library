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

//#include "Config.h"

#include "Peripheral/Pin/Debounce.h"
#include "Peripheral/Pin/Pin.h"

#include "Math/Linear/Linear_ADC.h"
#include "Math/Linear/Linear.h"

#include <stdint.h>
#include <stdbool.h>

MotorUser_T MotorUserMain;

void MotorUser_Init(MotorUser_Init_T * p_init)
{
	MotorUserMain.P_MOTOR_USER_CONST = p_init;

	Debounce_Init(&MotorUserMain.MotorUserAnalog.PinBrake, 		&p_init->HAL_PIN_BRAKE, 		&p_init->P_DEBOUNCE_TIMER, 5U);	//5millis
	Debounce_Init(&MotorUserMain.MotorUserAnalog.PinThrottle,	&p_init->HAL_PIN_THROTTLE, 		&p_init->P_DEBOUNCE_TIMER, 5U);	//5millis
	Debounce_Init(&MotorUserMain.MotorUserAnalog.PinForward, 	&p_init->HAL_PIN_FORWARD, 		&p_init->P_DEBOUNCE_TIMER, 5U);	//5millis
	Debounce_Init(&MotorUserMain.MotorUserAnalog.PinReverse, 	&p_init->HAL_PIN_REVERSE, 		&p_init->P_DEBOUNCE_TIMER, 5U);	//5millis

	//uncalibrated default
	Linear_ADC_Init(&MotorUserMain.MotorUserAnalog.UnitThrottle, 	0U, 4095U, 50U);
	Linear_ADC_Init(&MotorUserMain.MotorUserAnalog.UnitBrake,		0U, 4095U, 50U);
}

//write to motor
void MotorUser_SetInput(Motor_T * p_motorDest)
{
	//proc inputs
	//	if(GetSwitchEdge(DirectionButton)) {StateMachine_Semisynchronous_ProcInput(&MotorUserMain.StateMachine, PIN_FUNCTION_FORWARD);}
	//	if(GetSwitchEdge(DirectionButton)) {StateMachine_Semisynchronous_ProcInput(&MotorUserMain.StateMachine, PIN_FUNCTION_REVERSE);}

	bool activeControl;

	if (
			!MotorUserMain.InputSwitchNeutral //&&
//			!MotorUser_Analog_GetThrottleRelease()
		)
	{
		if (MotorUserMain.InputSwitchBrake == true)
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
			if (MotorUserMain.InputValueThrottle < 5U) // less then threshold
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

		if (MotorUserMain.InputSwitchForward == true) // ^ reverse direction
		{
			Motor_SetDirectionInput(p_motorDest, MOTOR_DIRECTION_CCW);
		}
		else if (MotorUserMain.InputSwitchReverse == true)
		{
			Motor_SetDirectionInput(p_motorDest, MOTOR_DIRECTION_CW);
		}

		//set ramp per second
		if (MotorUserMain.InputSwitchBrake == true)
		{
	//			if (p_motor->Parameters.BrakeMode == MOTOR_BRAKE_MODE_SCALAR)
	//			{
			Motor_SetUserCmd(p_motorDest, (MotorUserMain.InputValueBrake + MotorUserMain.InputValueBrakePrev) / 2U);
			Motor_SetRampDecelerate(p_motorDest, 0);
	//			}
		}
		else
		{
			Motor_SetUserCmd(p_motorDest, (MotorUserMain.InputValueThrottle + MotorUserMain.InputValueThrottlePrev) / 2U);
			Motor_SetRampAccelerate(p_motorDest, 0);
		}
	}
}




  void MotorUser_SetOutput_Units(const Motor_T * p_motorSrc)
{

}

  void MotorUser_SetOutput_ADCU(const Motor_T * p_motorSrc)
{

}
