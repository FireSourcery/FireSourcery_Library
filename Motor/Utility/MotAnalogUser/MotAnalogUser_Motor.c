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
    @brief	Parse conditions and actuate
    @version V0
*/
/**************************************************************************/
#include "MotAnalogUser_Motor.h"
#include "MotAnalogUser.h"

#include "Motor/Motor/Motor.h"
#include "Motor/Motor/Motor_User.h"

#include <stdint.h>
#include <stdbool.h>



void MotAnalogUser_Motor_Write(const MotAnalogUser_T * p_user, Motor_T * p_motorDest, uint8_t motorCount)
{
	bool activeControl = false;


	if (p_user->InputSwitchNeutral == false)
	{
		if ((p_user->InputSwitchBrake == true) || ((p_user->InputValueThrottle > 5U) && (p_user->IsThrottleRelease == false)))
		{
			activeControl = true;
		}
	}

	if (activeControl == false)
	{
		if (p_user->InputSwitchForward == true)
		{
			for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
			{
				Motor_User_SetDirection(&p_motorDest[iMotor], MOTOR_DIRECTION_CCW);
			}
		}
		else if (p_user->InputSwitchReverse == true)
		{
			for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
			{
				Motor_User_SetDirection(&p_motorDest[iMotor], MOTOR_DIRECTION_CW);
			}
		}

		for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
		{
			Motor_User_Disable(&p_motorDest[iMotor]);	//set state freewheel
		}
	}
	else
	{
		//set ramp slope per second
		if (p_user->InputSwitchBrake == true)
		{
			for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
			{
				Motor_User_SetBrake(&p_motorDest[iMotor], p_user->InputValueBrake);
			}
		}
		else
		{
			for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
			{
				Motor_User_SetThrottle(&p_motorDest[iMotor], p_user->InputValueThrottle);
			}
		}
	}
}



void MotAnalogUser_Motor_WriteMotor(const MotAnalogUser_T * input, Motor_T * p_motorDest)
{
	MotAnalogUser_Motor_Write(input, p_motorDest, 1U);
}
