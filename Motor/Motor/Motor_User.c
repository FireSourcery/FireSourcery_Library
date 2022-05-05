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
	@file 	Motor_User.c
	@author FireSoucery
	@brief
	@version V0
*/
/******************************************************************************/
#include "Motor_User.h"

/******************************************************************************/
/*!
	State Machine Error checked inputs
*/
/******************************************************************************/
/*
	Motor State Machine Thread Safety

	SemiSync Mode -
	State Proc in PWM thread. User Input in Main Thread. May need critical section during input.

	Set async to proc, sync issue can recover?,
	control proc may overwrite pid state set, but last to complete is always user input

	Sync Mode
	Must check input flags every pwm cycle
*/

/*
	Disable control, motor may remain spinning
*/
void Motor_User_DisableControl(Motor_T * p_motor)
{
	Phase_Float(&p_motor->Phase);
	StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_FLOAT);
}

void Motor_User_Ground(Motor_T * p_motor)
{
	Phase_Ground(&p_motor->Phase);

	//alternatively only ground in stop state
//	StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_GROUND);
}

/*
	set buffered direction, check on state machine
*/
bool Motor_User_SetDirection(Motor_T * p_motor, Motor_Direction_T direction)
{
	p_motor->UserDirection = direction;
	StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_DIRECTION);
	//StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_DIRECTION, direction);

	return (p_motor->UserDirection == p_motor->Direction);
}

bool Motor_User_SetDirectionForward(Motor_T * p_motor)
{
	bool isSet;

	if(p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW)
	{
		isSet = Motor_User_SetDirection(p_motor, MOTOR_DIRECTION_CCW) ? true : false;
	}
	else
	{
		isSet = Motor_User_SetDirection(p_motor, MOTOR_DIRECTION_CW) ? true : false;
	}

	return isSet;
}

bool Motor_User_SetDirectionReverse(Motor_T * p_motor)
{
	bool isSet;

	if(p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW)
	{
		isSet = Motor_User_SetDirection(p_motor, MOTOR_DIRECTION_CW) ? true : false;
	}
	else
	{
		isSet = Motor_User_SetDirection(p_motor, MOTOR_DIRECTION_CCW) ? true : false;
	}

	return isSet;
}

/******************************************************************************/
/*!
	UserCmd functions
	
	Call regularly to update cmd value
	Cmd value sets without checking state machine.

	User input sign +/- indicates along or against Direction selected. NOT virtual CW/CCW.
	Convert sign to direction here. Called less frequently than control loop, 1/Millis.
*/
/******************************************************************************/
//set ramps and limits using mode
void Motor_User_SetCmdSigned(Motor_T * p_motor, int32_t cmd)
{



	int32_t rampTarget = cmd >> 1U;
	//set output limit here, user params, set to pid or ramp?
	if(p_motor->Direction == MOTOR_DIRECTION_CW) { rampTarget = 0 - rampTarget; }
	Motor_SetRamp(p_motor, rampTarget);
}

void Motor_User_SetCmdUnsigned(Motor_T * p_motor, uint16_t cmd)
{
	//set output limit here, user params, set to pid or ramp?
	Motor_SetRamp(p_motor, cmd);
}

/*!
	@param[in] userCmd [-65536:65535]
*/
void Motor_User_SetCmd(Motor_T * p_motor, int32_t userCmd)
{
	if(p_motor->ControlModeFlags.Speed == 1U)
	{
		Motor_User_SetCmdUnsigned(p_motor, userCmd); 
	}
	else
	{
		Motor_User_SetCmdSigned(p_motor, userCmd);
	}
}


void Motor_User_SetCmdMode(Motor_T * p_motor, Motor_CmdMode_T mode)
{
	if(mode != p_motor->CmdMode) 
	{
	// reset limits
	}

	// if
	// (
	// 	(RampTarget > RampCmd) && (p_motor.CmdMode == MOTOR_CMD_MODE_GENERATING) ||
	// 	(RampTarget < RampCmd) && (p_motor.CmdMode == MOTOR_CMD_MODE_MOTORING)
	// )
	// {
	// 	reset limits
	// }
 
	if(Motor_CheckCmdMode(p_motor ) == true)
	{
		Critical_Enter();

		Motor_SetControlMode(p_motor, mode);
		StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_CONTROL_MODE);
		p_motor->ControlModeFlags.Update = 0U;

		Critical_Exit();
	}
}


/******************************************************************************/
/*!
	User control modes, torque/speed/position, above foc/sixstep commutation layer

	SetMode functions sets control mode only
	SetCmd functions check/sets control mode, and sets cmd value
*/
/******************************************************************************/
void Motor_User_SetControlMode(Motor_T * p_motor, Motor_ControlMode_T mode)
{
	if(Motor_CheckControlMode(p_motor, mode) == true)
	{
		Critical_Enter();

		Motor_SetControlMode(p_motor, mode);
		StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_CONTROL_MODE);
		p_motor->ControlModeFlags.Update = 0U;

		Critical_Exit();
	}
}



/******************************************************************************/
/*
	N Motor Array functions
*/
/******************************************************************************/
//void ProcMotorUserN(Motor_T * p_motor, uint8_t motorCount, uint32_t (*op)(Motor_T*, uint32_t), uint16_t var)
//{
//	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
//	{
//		op(&p_motor[iMotor], var);
//	}
//}

void Motor_UserN_SetCmd(Motor_T * p_motor, uint8_t motorCount, int32_t cmd)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_SetCmd(&p_motor[iMotor], cmd);
	}
}

void Motor_UserN_SetThrottleCmd(Motor_T * p_motor, uint8_t motorCount, uint16_t throttle)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_SetThrottleCmd(&p_motor[iMotor], throttle);
	}
}

void Motor_UserN_SetBrakeCmd(Motor_T * p_motor, uint8_t motorCount, uint16_t brake)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_SetBrakeCmd(&p_motor[iMotor], brake);
	}
}

void Motor_UserN_SetVoltageBrakeCmd(Motor_T * p_motor, uint8_t motorCount)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_SetVoltageBrakeCmd(&p_motor[iMotor]);
	}
}

bool Motor_UserN_SetDirectionForward(Motor_T * p_motor, uint8_t motorCount)
{
	bool isSet = true;

	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		if(Motor_User_SetDirectionForward(&p_motor[iMotor]) == false)
		{
			isSet = false;
		}
	}

	return isSet;
}

bool Motor_UserN_SetDirectionReverse(Motor_T * p_motor, uint8_t motorCount)
{
	bool isSet = true;

	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		if(Motor_User_SetDirectionReverse(&p_motor[iMotor]) == false)
		{
			isSet = false;
		}
	}

	return isSet;
}

void Motor_UserN_DisableControl(Motor_T * p_motor, uint8_t motorCount)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_DisableControl(&p_motor[iMotor]);
	}
}

void Motor_UserN_Ground(Motor_T * p_motor, uint8_t motorCount)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_Ground(&p_motor[iMotor]);
	}
}

bool Motor_UserN_CheckStop(Motor_T * p_motor, uint8_t motorCount)
{
	bool isStop = true;

	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		if(Motor_User_GetSpeed_RPM(&p_motor[iMotor]) > 0U)
		{
			isStop = false;
			break;
		}
	}

	return isStop;
}

bool Motor_UserN_CheckErrorFlags(Motor_T * p_motor, uint8_t motorCount)
{
	bool isError = false;

	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		// if(Motor_User_GetErrorFlags(&p_motor[iMotor]).State != 0U)
		// {
		// 	isError = true;
		// 	break;
		// }
	}

	return isError;
}

bool Motor_UserN_CheckFault(Motor_T * p_motor, uint8_t motorCount)
{
	bool isFault = false;

	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		if(Motor_User_CheckFault(&p_motor[iMotor]) == true) { isFault = true; }
	}

	return isFault;
}

bool Motor_UserN_ClearFault(Motor_T * p_motor, uint8_t motorCount)
{
	bool isClear = true;

	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		if(Motor_User_ClearFault(&p_motor[iMotor]) == false) { isClear = false; }
	}

	return isClear;
}
