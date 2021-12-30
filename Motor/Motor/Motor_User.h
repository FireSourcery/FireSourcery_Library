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

#include "Motor_StateMachine.h"
#include "Motor.h"
#include "Utility/StateMachine/StateMachine.h"

#include <stdint.h>
#include <stdbool.h>



static inline void Motor_User_EnableControl(Motor_T * p_motor)
{

}

/*
 * Disable control, motor may remain spinning
 */
static inline void Motor_User_DisableControl(Motor_T * p_motor)
{
	Phase_Float(&p_motor->Phase);
	StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_FLOAT);
}

static inline void Motor_User_SetCmdAccelerate(Motor_T * p_motor, uint16_t throttle)
{
	Motor_SetRampUp(p_motor, throttle);
	StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_ACCELERATE);
}


static inline void Motor_User_SetCmdDecelerate(Motor_T * p_motor, uint16_t brake)
{
	if (p_motor->Parameters.BrakeMode == MOTOR_BRAKE_MODE_PASSIVE)
	{
		Motor_User_DisableControl(p_motor);
	}
	else
	{
		Motor_SetRampDown(p_motor, brake);
		StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_DECELERATE);
	}
}

//or set buffered direction, check on state machine  ?
static inline bool Motor_User_SetDirection(Motor_T * p_motor, Motor_Direction_T direction)
{
	bool isSet = false;

//	if (p_motor->Direction != direction) //direction reversed
//	{
	if (p_motor->Speed_RPM == 0U)
	{
		Motor_SetDirection(p_motor, direction);
//		StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MSM_INPUT_DIRECTION);
		isSet = true;
	}
//	}

	return isSet;
}


static inline bool Motor_User_GetDirectionCalibration(Motor_T * p_motor)
{
//	return p_motor->Parameters.IsCcwForward;
	return true;
}

static inline bool Motor_User_SetDirectionForward(Motor_T * p_motor)
{
	bool isSet;

	if(Motor_User_GetDirectionCalibration(p_motor) == true) //ccw is forward
	{
		isSet = Motor_User_SetDirection(p_motor, MOTOR_DIRECTION_CCW) ? true : false;
	}
	else
	{
		isSet = Motor_User_SetDirection(p_motor, MOTOR_DIRECTION_CW) ? true : false;
	}

	return isSet;
}

static inline bool Motor_User_SetDirectionReverse(Motor_T * p_motor)
{
	bool isSet;

	if(Motor_User_GetDirectionCalibration(p_motor) == false) //ccw is forward
	{
		isSet = Motor_User_SetDirection(p_motor, MOTOR_DIRECTION_CCW) ? true : false;
	}
	else
	{
		isSet = Motor_User_SetDirection(p_motor, MOTOR_DIRECTION_CW) ? true : false;
	}

	return isSet;
}

static inline uint32_t Motor_User_GetBemf_Frac16(Motor_T * p_motor)	{return Linear_Voltage_CalcFractionUnsigned16(&p_motor->CONFIG.UNIT_V_ABC, BEMF_GetVBemfPeak_ADCU(&p_motor->Bemf));}
static inline uint32_t Motor_User_GetBemf_MilliV(Motor_T * p_motor)	{return Linear_Voltage_CalcMilliV(&p_motor->CONFIG.UNIT_V_ABC, BEMF_GetVBemfPeak_ADCU(&p_motor->Bemf));}

static inline uint32_t Motor_User_GetVPos_MilliV(Motor_T * p_motor)	{return Linear_Voltage_CalcMilliV(&p_motor->CONFIG.UNIT_V_POS, p_motor->AnalogResults.VPos_ADCU);}
static inline uint32_t Motor_User_GetSpeed_RPM(Motor_T *p_motor) 	{return p_motor->Speed_RPM;}

/*
 * Run Calibration functions
 */
static inline void Motor_User_ActivateCalibrationHall(Motor_T * p_motor)
{
	Motor_SetCalibrationStateHall(p_motor);
	StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION);
}

static inline void Motor_User_ActivateCalibrationEncoder(Motor_T * p_motor)
{
	Motor_SetCalibrationStateEncoder(p_motor);
	StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION);
}

static inline void Motor_User_ActivateCalibrationAdc(Motor_T * p_motor)
{
	Motor_SetCalibrationStateAdc(p_motor);
	StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION);
}


/*
 * Set Motor Array functions
 */
static inline void Motor_UserN_SetCmdAccelerate(Motor_T * p_motor, uint8_t motorCount, uint16_t throttle)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_SetCmdAccelerate(&p_motor[iMotor], throttle);
	}
}

static inline void Motor_UserN_SetCmdDecelerate(Motor_T * p_motor, uint8_t motorCount, uint16_t brake)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_SetCmdDecelerate(&p_motor[iMotor], brake);
	}
}

static inline void Motor_UserN_DisableControl(Motor_T * p_motor, uint8_t motorCount)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_DisableControl(&p_motor[iMotor]);
	}
}

static inline bool Motor_UserN_CheckStop(Motor_T * p_motor, uint8_t motorCount)
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

#endif
