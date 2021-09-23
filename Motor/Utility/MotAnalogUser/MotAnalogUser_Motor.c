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
    @file 	MotAnalogUser_Motor.h
    @author FireSoucery
    @brief
    @version V0
*/
/**************************************************************************/
#include "MotAnalogUser_Motor.h"
#include "MotAnalogUser.h"

#include "Motor/Motor/Motor.h"

//#include "Peripheral/Pin/Debounce.h"
//#include "Peripheral/Pin/Pin.h"

//#include "Math/Linear/Linear_ADC.h"
//#include "Math/Linear/Linear.h"

#include <stdint.h>
#include <stdbool.h>

void MotAnalogUser_Motor_Write(const MotAnalogUser_T * p_user, Motor_T * p_motorDest, uint8_t motorCount)
{
	bool activeControl;

	if (p_user->InputSwitchNeutral == true)
	{
		activeControl = false;

//		if (input->InputSwitchBrake == true)
//		{
//			//todo remove, check brake mode inside, or set brake mode in ui
//			if (p_motorDest->Parameters.BrakeMode == MOTOR_BRAKE_MODE_PASSIVE)
//			{
//				activeControl = false;
//			}
//			else
//			{
//				activeControl = true;
//			}
//		}
//		else
//		{
//			if (input->InputValueThrottle < 5U) // less then threshold //p_input->InputValueThrottleThreshold
//			{
//				activeControl = false;
//			}
//			else
//			{
//				activeControl = true;
//			}
//		}
		if (p_user->InputSwitchForward == true)
		{
			for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
			{
				Motor_SetDirectionInput(p_motorDest, MOTOR_DIRECTION_CCW);
			}
		}
		else if (p_user->InputSwitchReverse == true)
		{
			for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
			{
				Motor_SetDirectionInput(p_motorDest, MOTOR_DIRECTION_CW);
			}
		}
	}
	else if ((p_user->InputSwitchBrake == false) && (p_user->InputValueThrottle < 5U))
	{
		activeControl = false;
	}
	else if (p_user->IsThrottleRelease == true)
	{
		activeControl = false;
	}
	else
	{
		activeControl = true;
	}

	if (activeControl == false)
	{
		p_motorDest->IsActiveControl = false;

		for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
		{
//			Motor_User_Disable(p_motorDest);
		}
	}
	else
	{
		p_motorDest->IsActiveControl = true;

		//set ramp per second
		if (p_user->InputSwitchBrake == true)
		{
			for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
			{
				Motor_SetUserCmd(p_motorDest, (p_user->InputValueBrake + p_user->InputValueBrakePrev) / 2U);
				Motor_SetRampDecelerate(p_motorDest, 0);
	//			Motor_User_SetDecelerate(p_motorDest, (input->InputValueBrake + input->InputValueBrakePrev) / 2U);
			}
		}
		else
		{
			for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
			{
				Motor_SetUserCmd(p_motorDest, (p_user->InputValueThrottle + p_user->InputValueThrottlePrev) / 2U);
				Motor_SetRampAccelerate(p_motorDest, 0);
	//			Motor_User_SetAccelerate(p_motorDest, (input->InputValueThrottle + input->InputValueThrottlePrev) / 2U);
			}
		}
	}
}


void MotAnalogUser_Motor_WriteMotor(const MotAnalogUser_T * input, Motor_T * p_motorDest)
{
	MotAnalogUser_Motor_Write(input, p_motorDest, 1U);
}
