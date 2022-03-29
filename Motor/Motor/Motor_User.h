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
    @brief  User Interface. Functions include error checking.
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

//disable pwm interrupt of this motor only
//static inline void Motor_User_EnterCriticalLocal(Motor_T * p_motor)
//{
//
//}


/*
 * State Machine Error checked inputs
 */

/*
 * Motor State Machine Thread Safety
 *
 * SemiSync Mode -
 * State Proc in PWM thread. User Input in Main Thread. May need critical section during input.
 * Sync error only occurs in way of running new states function, using false validated function of old state.
 * Can always recover? Stop to Run transition always at 0 speed.
 *
 * Sync Mode
 * Must check input flags every pwm cycle
 */


/*
 * Disable control, motor may remain spinning
 */
static inline void Motor_User_DisableControl(Motor_T * p_motor)
{
	Phase_Float(&p_motor->Phase);
	StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_FLOAT);
}

static inline void Motor_User_SetCmd(Motor_T * p_motor, uint16_t cmd)
{
//	Critical_Enter();
	Motor_SetRamp(p_motor, cmd);
	StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_ACCELERATE);
//	Critical_Exit();
}

/*
 * Always request opposite current
 */
static inline void Motor_User_SetCmdBrake(Motor_T * p_motor, uint16_t intensity)
{
//	p_motor->ControlMode.Current = 1U;

//	Critical_Enter();
	Motor_SetRamp(p_motor, intensity);
	StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_DECELERATE);
//	Critical_Exit();
}

//static inline void Motor_User_SetCmdSpeed(Motor_T * p_motor, uint16_t speed)
//{
//setcontrolmode
//	Motor_SetRamp(p_motor, cmd);
//	p_motor->IsBrake = false;
//	StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_ACCELERATE);
//}

//  set buffered direction, check on state machine
static inline bool Motor_User_SetDirection(Motor_T * p_motor, Motor_Direction_T direction)
{
	p_motor->UserDirection = direction; //pass to state machine
	StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_DIRECTION);
//	return StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_DIRECTION, direction);

	return (p_motor->UserDirection == p_motor->Direction);
}


static inline bool Motor_User_SetDirectionForward(Motor_T * p_motor)
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

static inline bool Motor_User_SetDirectionReverse(Motor_T * p_motor)
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
 * Conversion Function only called if called by the user. Not called regularly
 */
//static inline uint16_t Motor_User_GetBemf_Frac16(Motor_T * p_motor)	{return Linear_Voltage_CalcFractionUnsigned16(&p_motor->CONFIG.UNIT_V_ABC, BEMF_GetVBemfPeak_ADCU(&p_motor->Bemf));}
//static inline uint32_t Motor_User_GetBemf_MilliV(Motor_T * p_motor)	{return Linear_Voltage_CalcMilliV(&p_motor->CONFIG.UNIT_V_ABC, BEMF_GetVBemfPeak_ADCU(&p_motor->Bemf));}
//static inline uint32_t Motor_User_GetVPos_MilliV(Motor_T * p_motor)	{return Linear_Voltage_CalcMilliV(&p_motor->CONFIG.UNIT_V_POS, p_motor->AnalogResults.VPos_ADCU);}


//static inline float Motor_User_GetHeat_DegCFloat(Motor_T *p_motor)						{return Thermistor_ConvertToDegC_Float(&p_motor->Thermistor, p_motor->AnalogResults.Heat_ADCU);}
//static inline uint16_t Motor_User_GetHeat_DegCRound(Motor_T *p_motor)					{return Thermistor_ConvertToDegC_Round(&p_motor->Thermistor, p_motor->AnalogResults.Heat_ADCU);}
//static inline uint16_t Motor_User_GetHeat_DegCNDecimal(Motor_T *p_motor, uint8_t n) 	{return Thermistor_ConvertToDegC_NDecimal(&p_motor->Thermistor, p_motor->AnalogResults.Heat_ADCU, n);}
//static inline uint16_t Motor_User_GetHeat_DegCScalar(Motor_T * p_motor, uint16_t scalar) 	{return Thermistor_ConvertToDegC_Scalar(&p_motor->Thermistor, p_motor->AnalogResults.Heat_ADCU, scalar);}
//static inline uint16_t Motor_User_GetHeat_Fixed32(Motor_T *p_motor) 					{return Thermistor_ConvertToDegC_Fixed32(&p_motor->Thermistor, p_motor->AnalogResults.Heat_ADCU);}

static inline uint16_t Motor_User_GetSpeed_RPM(Motor_T * p_motor) 	{return p_motor->Speed_RPM;}
static inline Motor_DirectionCalibration_T Motor_User_GetDirectionCalibration(Motor_T *p_motor) 	{return p_motor->Parameters.DirectionCalibration;}
static inline Hall_Sensors_T Motor_User_GetHall(Motor_T * p_motor) 		{return Hall_GetSensors(&p_motor->Hall);}
static inline uint16_t Motor_User_GetHallRotorAngle(Motor_T * p_motor) 	{return Hall_GetRotorAngle_Degrees16(&p_motor->Hall);}

static inline Motor_ErrorFlags_T Motor_User_GetErrorFlags(Motor_T * p_motor) 	{return  p_motor->ErrorFlags;}

//static inline void Motor_User_ActivatePhase(Motor_T * p_motor)
//{
//
//}

/*
 * Set Motor Array functions
 */
static inline void Motor_UserN_SetCmd(Motor_T * p_motor, uint8_t motorCount, uint16_t throttle)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_SetCmd(&p_motor[iMotor], throttle);
	}
}

static inline void Motor_UserN_SetCmdBrake(Motor_T * p_motor, uint8_t motorCount, uint16_t brake)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_SetCmdBrake(&p_motor[iMotor], brake);
	}
}


static inline bool Motor_UserN_SetDirectionForward(Motor_T * p_motor, uint8_t motorCount)
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

static inline bool Motor_UserN_SetDirectionReverse(Motor_T * p_motor, uint8_t motorCount)
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

static inline bool Motor_UserN_CheckErrorFlags(Motor_T * p_motor, uint8_t motorCount)
{
	bool isError = false;

	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		if(Motor_User_GetErrorFlags(&p_motor[iMotor]).State != 0U)
		{
			isError = true;
			break;
		}
	}

	return isError;
}

#endif
