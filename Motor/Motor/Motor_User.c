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
	Motor_Direction_T direction = (p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW) ?
		MOTOR_DIRECTION_CCW : MOTOR_DIRECTION_CW;

	return Motor_User_SetDirection(p_motor, direction);
}

bool Motor_User_SetDirectionReverse(Motor_T * p_motor)
{
	Motor_Direction_T direction = (p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW) ?
		MOTOR_DIRECTION_CW : MOTOR_DIRECTION_CCW;

	return Motor_User_SetDirection(p_motor, direction);
}

/*
	Change Feedback mode, match Ramp and PID state to output
*/
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

// //set ramps and limits using mode
// void Motor_User_SetCmdSigned(Motor_T * p_motor, int32_t cmd)
// {



// 	int32_t rampTarget = cmd >> 1U;
// 	//set output limit here, user params, set to pid or ramp?
// 	if(p_motor->Direction == MOTOR_DIRECTION_CW) { rampTarget = 0 - rampTarget; }
// 	Motor_SetRamp(p_motor, rampTarget); //ramp is set to cc/ccw direction


// }

// void Motor_User_SetCmdUnsigned(Motor_T * p_motor, uint16_t cmd)
// {
// 	//set output limit here, user params, set to pid or ramp?
// 	Motor_SetRamp(p_motor, cmd);
// }

// /*!
// 	@param[in] userCmd [-65536:65535] 
// */
// void Motor_User_SetCmd(Motor_T * p_motor, int32_t userCmd)
// {
// 	int32_t userCmdNew;

// 	if(p_motor->ControlModeFlags.Speed == 1U)
// 	{
// 		userCmdNew = userCmd * p_motor->SpeedLimit_Frac16 / 65536U;
// 		Motor_User_SetCmdUnsigned(p_motor, userCmdNew);

	


// 	}
// 	else
// 	{
// 		if(p_motor->ControlModeFlags.Current == 1U)
// 		{
// 			// error on input, p_motor->RampCmd, or feedback p_motor->SpeedFeedback_Frac16

// 			//check if changed
// 			// if(userCmdNew >= p_motor->SpeedFeedback_Frac16) 	{ p_motor->TorqueDirection = MOTOR_TORQUE_MODE_MOTORING; }
// 			// else 												{ p_motor->TorqueDirection = MOTOR_TORQUE_MODE_GENERATING; }

// 			if(userCmd > 0)
// 			{
// 				// p_motor->TorqueDirection = MOTOR_TORQUE_MODE_MOTORING;
// 				userCmdNew = userCmd * p_motor->ILimitMotoring_Frac16 / 65536U;	
// 			}
// 			else
// 			{
// 				// p_motor->TorqueDirection = MOTOR_TORQUE_MODE_GENERATING;
// 				userCmdNew = userCmd * p_motor->ILimitGenerating_Frac16 / 65536U;	
// 			}


			  
// 		}
// 		else
// 		{
// 			if(userCmd > 0)
// 			{ 
// 				userCmdNew = userCmd * p_motor->ILimitMotoring_Frac16 / 65536U;	
// 			} 
// 			else
// 			{
// 				userCmdNew = 0U;
// 			}

// 		}

// 		if(p_motor->Direction == MOTOR_DIRECTION_CW) { userCmdNew = 0 - userCmdNew; }
// 		userCmdNew = userCmdNew >> 1U; 
// 		Motor_SetRamp(p_motor, userCmdNew); //ramp is set to cc/ccw direction 
// 		Motor_User_SetCmdSigned(p_motor, userCmd);
// 	}
// }








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

bool Motor_UserN_CheckStop(Motor_T * p_motor, uint8_t motorCount)
{
	bool isStop = true;

	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		if(p_motor->SpeedFeedback_Frac16 > 0U) { isStop = false; break; }
	}

	return isStop;
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
		if(Motor_User_SetDirectionForward(&p_motor[iMotor]) == false) { isSet = false; }
	}

	return isSet;
}

bool Motor_UserN_SetDirectionReverse(Motor_T * p_motor, uint8_t motorCount)
{
	bool isSet = true;

	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		if(Motor_User_SetDirectionReverse(&p_motor[iMotor]) == false) { isSet = false; }
	}

	return isSet;
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

// bool Motor_UserN_CheckErrorFlags(Motor_T * p_motor, uint8_t motorCount)
// {
// 	bool isError = false;

// 	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
// 	{
// 		// if(Motor_User_GetErrorFlags(&p_motor[iMotor]).State != 0U)
// 		// {
// 		// 	isError = true;
// 		// 	break;
// 		// }
// 	}

// 	return isError;
// }