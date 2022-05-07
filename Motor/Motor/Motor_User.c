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
	Change Feedback mode, match Ramp and PID state to output
*/
void _Motor_User_SetControlMode(Motor_T * p_motor, Motor_ControlMode_T mode)
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
//	StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_GROUND);	//alternatively only ground in stop state
}

/*
	set buffered direction, check on state machine
*/
bool Motor_User_SetDirection(Motor_T * p_motor, Motor_Direction_T direction)
{ 
	p_motor->UserDirection = direction;

	if(p_motor->Direction != direction) //neutral to same direction?
	{
		StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_DIRECTION);
		//StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_DIRECTION, direction);
	}

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
	Active speed limit
	
	SpeedRefMax -> UserParam -> Direction -> Scalar

	SpeedRefMax_Rpm => 5000
	Parameters.SpeedLimitCcw_Frac16 == 32767 => 2500 RPM
	scalar_frac16 == 52428 => SpeedLimitCcw_Frac16[Active] == 26213 => 2000 RPM
	
	SpeedLimitCcw_Frac16[Active] is applied every SetUserCmd
*/
void Motor_User_SetSpeedLimitActive_Scalar(Motor_T * p_motor, uint16_t scalar_frac16)
{
	p_motor->SpeedLimitCcw_Frac16 = (uint32_t)scalar_frac16 * p_motor->Parameters.SpeedLimitCcw_Frac16 / 65536U;
	p_motor->SpeedLimitCw_Frac16 = (uint32_t)scalar_frac16 * p_motor->Parameters.SpeedLimitCw_Frac16 / 65536U;  
	p_motor->SpeedLimit_Frac16 = (p_motor->Direction == MOTOR_DIRECTION_CCW) ? p_motor->SpeedLimitCcw_Frac16 : p_motor->SpeedLimitCw_Frac16;
}

void Motor_User_ClearSpeedLimitActive(Motor_T * p_motor)
{
	Motor_ResetSpeedLimits(p_motor); 
	p_motor->SpeedLimit_Frac16 = (p_motor->Direction == MOTOR_DIRECTION_CCW) ? p_motor->SpeedLimitCcw_Frac16 : p_motor->SpeedLimitCw_Frac16;
}

void Motor_User_SetILimitActive_Scalar(Motor_T * p_motor, uint16_t scalar_frac16)
{
	p_motor->ILimitMotoring_Frac16 = (uint32_t)scalar_frac16 * p_motor->Parameters.ILimitMotoring_Frac16 / 65536U;
	p_motor->ILimitGenerating_Frac16 = (uint32_t)scalar_frac16 * p_motor->Parameters.ILimitGenerating_Frac16 / 65536U;

	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
	{
		(p_motor->Direction == MOTOR_DIRECTION_CCW) ? Motor_FOC_SetOutputLimitsCcw(p_motor) : Motor_FOC_SetOutputLimitsCw(p_motor);
	}
}

void Motor_User_ClearILimitActive(Motor_T * p_motor)
{
	Motor_ResetILimits(p_motor); 

	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
	{
		(p_motor->Direction == MOTOR_DIRECTION_CCW) ? Motor_FOC_SetOutputLimitsCcw(p_motor) : Motor_FOC_SetOutputLimitsCw(p_motor);
	}
}

uint16_t Motor_User_GetMechanialAngle(Motor_T * p_motor)
{
	uint16_t angle;

	switch(p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_SENSORLESS: 	angle = 0; 	break;
		case MOTOR_SENSOR_MODE_HALL: 		angle = Encoder_Motor_GetMechanicalAngle(&p_motor->Encoder);	break;
		case MOTOR_SENSOR_MODE_ENCODER: 	angle = Encoder_Motor_GetMechanicalAngle(&p_motor->Encoder);	break;
		case MOTOR_SENSOR_MODE_SIN_COS: 	angle = SinCos_GetMechanicalAngle(&p_motor->SinCos); 			break;
		default: 	break;
	}

	return angle;
}

/******************************************************************************/
/*
	Nvm Param function
*/
/******************************************************************************/ 
/* Persistent ILimit */
/* refmax must be set */
void Motor_User_SetILimitParam_Amp(Motor_T * p_motor, uint16_t motoring_Amp, uint16_t generating_Amp)
{
	uint32_t limitMotoring_Frac16 = Motor_ConvertToIFrac16(p_motor, motoring_Amp);
	uint32_t limitGenerating_Frac16 = Motor_ConvertToIFrac16(p_motor, generating_Amp);

	if(limitMotoring_Frac16 > 65535U) 		{ limitMotoring_Frac16 = 65535U; }
	if(limitGenerating_Frac16 > 65535U) 	{ limitGenerating_Frac16 = 65535U; }

	p_motor->Parameters.ILimitMotoring_Frac16 = limitMotoring_Frac16;
	p_motor->Parameters.ILimitGenerating_Frac16 = limitGenerating_Frac16; 
#ifndef CONFIG_MOTOR_PROPOGATE_SET_PARAM_DISABLE
	Motor_User_ClearILimitActive(p_motor);
#endif		
}

/* Persistent SpeedLimit - effective speed control mode only */
void Motor_User_SetSpeedLimitParam_Rpm(Motor_T * p_motor, uint16_t forward_Rpm, uint16_t reverse_Rpm)
{
	uint32_t forward_Frac16 = Motor_ConvertToSpeedFrac16(p_motor, forward_Rpm);
	uint32_t reverse_Frac16 = Motor_ConvertToSpeedFrac16(p_motor, reverse_Rpm); 

	if(forward_Frac16 > 65535U) { forward_Frac16 = 65535U; }
	if(reverse_Frac16 > 65535U) { reverse_Frac16 = 65535U; }

	if(p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW)
	{
		p_motor->Parameters.SpeedLimitCcw_Frac16 = forward_Frac16;
		p_motor->Parameters.SpeedLimitCw_Frac16 = reverse_Frac16;
	}
	else
	{
		p_motor->Parameters.SpeedLimitCcw_Frac16 = reverse_Frac16;
		p_motor->Parameters.SpeedLimitCw_Frac16 = forward_Frac16;
	}
#ifndef CONFIG_MOTOR_PROPOGATE_SET_PARAM_DISABLE
	Motor_User_ClearSpeedLimitActive(p_motor);
#endif	
}

void Motor_User_SetDirectionCalibration(Motor_T * p_motor, Motor_DirectionCalibration_T mode)
{
#ifndef CONFIG_MOTOR_PROPOGATE_SET_PARAM_DISABLE
	uint32_t ccw, cw;

	if(p_motor->Parameters.DirectionCalibration != mode)
	{
		ccw = p_motor->Parameters.SpeedLimitCcw_Frac16;
		cw = p_motor->Parameters.SpeedLimitCw_Frac16;
		p_motor->Parameters.SpeedLimitCcw_Frac16 = cw;
		p_motor->Parameters.SpeedLimitCw_Frac16 = ccw;
	}
#endif

	p_motor->Parameters.DirectionCalibration = mode;
}



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

void Motor_UserN_SetUserControlCmd(Motor_T * p_motor, uint8_t motorCount, int32_t cmd)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_SetUserControlCmd(&p_motor[iMotor], cmd);
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

void Motor_UserN_SetSpeedLimit_Scalar(Motor_T * p_motor, uint8_t motorCount, uint16_t limit_frac16)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_SetSpeedLimitActive_Scalar(&p_motor[iMotor], limit_frac16);
	}
}

void Motor_UserN_ClearSpeedLimit(Motor_T * p_motor, uint8_t motorCount)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_ClearSpeedLimitActive(&p_motor[iMotor]);
	}
} 

void Motor_UserN_SetILimit_Scalar(Motor_T * p_motor, uint8_t motorCount, uint16_t limit_frac16)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_SetILimitActive_Scalar(&p_motor[iMotor], limit_frac16);
	}
}

void Motor_UserN_ClearILimit(Motor_T * p_motor, uint8_t motorCount)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_ClearILimitActive(&p_motor[iMotor]);
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