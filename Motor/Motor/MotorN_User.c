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
	@file 	MotorN_User.c
	@author FireSoucery
	@brief
	@version V0
*/
/******************************************************************************/
#include "MotorN_User.h"

/******************************************************************************/
/*
	N Motor Array functions
*/
/******************************************************************************/
//void ProcN(Motor_T * p_motor, uint8_t motorCount, uint32_t (*op)(Motor_T*, uint32_t), uint16_t var)
//{
//	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
//	{
//		op(&p_motor[iMotor], var);
//	}
//}

//alternatively use check stop state?
bool MotorN_User_CheckStop(Motor_T * p_motor, uint8_t motorCount)
{
	bool isStop = true;

	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		if(p_motor->SpeedFeedback_Frac16 > 0) { isStop = false; break; }
	}

	return isStop;
}

void MotorN_User_DisableControl(Motor_T * p_motor, uint8_t motorCount)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_DisableControl(&p_motor[iMotor]);
	}
}

void MotorN_User_Ground(Motor_T * p_motor, uint8_t motorCount)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_Ground(&p_motor[iMotor]);
	}
}

bool MotorN_User_SetDirectionForward(Motor_T * p_motor, uint8_t motorCount)
{
	bool isSet = true;

	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		if(Motor_User_SetDirectionForward(&p_motor[iMotor]) == false) { isSet = false; }
	}

	return isSet;
}

bool MotorN_User_SetDirectionReverse(Motor_T * p_motor, uint8_t motorCount)
{
	bool isSet = true;

	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		if(Motor_User_SetDirectionReverse(&p_motor[iMotor]) == false) { isSet = false; }
	}

	return isSet;
}

bool MotorN_User_CheckFault(Motor_T * p_motor, uint8_t motorCount)
{
	bool isFault = false;

	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		if(Motor_User_CheckFault(&p_motor[iMotor]) == true) { isFault = true; }
	}

	return isFault;
}

bool MotorN_User_ClearFault(Motor_T * p_motor, uint8_t motorCount)
{
	bool isClear = true;

	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		if(Motor_User_ClearFault(&p_motor[iMotor]) == false) { isClear = false; }
	}

	return isClear;
}

void MotorN_User_SetUserFeedbackModeCmd(Motor_T * p_motor, uint8_t motorCount, int32_t cmd)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_SetUserFeedbackModeCmd(&p_motor[iMotor], cmd);
	}
}

void MotorN_User_SetThrottleCmd(Motor_T * p_motor, uint8_t motorCount, uint16_t throttle)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_SetThrottleCmd(&p_motor[iMotor], throttle);
	}
}

void MotorN_User_SetBrakeCmd(Motor_T * p_motor, uint8_t motorCount, uint16_t brake)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_SetBrakeCmd(&p_motor[iMotor], brake);
	}
}

// void MotorN_User_SetVoltageBrakeCmd(Motor_T * p_motor, uint8_t motorCount)
// {
// 	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
// 	{
// 		Motor_User_SetVoltageBrakeCmd(&p_motor[iMotor]);
// 	}
// }

void MotorN_User_SetRegenCmd(Motor_T * p_motor, uint8_t motorCount, uint16_t brake)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_SetRegenCmd(&p_motor[iMotor], brake);
	}
}

void MotorN_User_SetSpeedLimitActive(Motor_T * p_motor, uint8_t motorCount, uint16_t limit_frac16)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_SetSpeedLimitActive(&p_motor[iMotor], limit_frac16);
	}
}

void MotorN_User_ClearSpeedLimit(Motor_T * p_motor, uint8_t motorCount)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_ClearSpeedLimitActive(&p_motor[iMotor]);
	}
}

/*! @return true if at least one is set */
bool MotorN_User_SetILimitActive(Motor_T * p_motor, uint8_t motorCount, uint16_t limit_frac16, Motor_ILimitActiveId_T id)
{
	bool isSet = false;

	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		if(Motor_User_SetILimitActive(&p_motor[iMotor], limit_frac16, id) == true) { isSet = true; };
	}

	return isSet;
}

/*! @return true if all are cleared, todo change to are clear not on clear[ed] */
bool MotorN_User_ClearILimit(Motor_T * p_motor, uint8_t motorCount, Motor_ILimitActiveId_T id)
{
	bool isClear = true;

	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		if(Motor_User_ClearILimitActive(&p_motor[iMotor], id) == false) { isClear = false; };
	}

	return isClear;
}