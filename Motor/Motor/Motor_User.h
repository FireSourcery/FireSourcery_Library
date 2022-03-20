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


//static inline uint16_t Motor_User_GetBemf_Frac16(Motor_T * p_motor)	{return Linear_Voltage_CalcFractionUnsigned16(&p_motor->CONFIG.UNIT_V_ABC, BEMF_GetVBemfPeak_ADCU(&p_motor->Bemf));}
//static inline uint32_t Motor_User_GetBemf_MilliV(Motor_T * p_motor)	{return Linear_Voltage_CalcMilliV(&p_motor->CONFIG.UNIT_V_ABC, BEMF_GetVBemfPeak_ADCU(&p_motor->Bemf));}
static inline uint32_t Motor_User_GetVPos_MilliV(Motor_T * p_motor)	{return Linear_Voltage_CalcMilliV(&p_motor->CONFIG.UNIT_V_POS, p_motor->AnalogResults.VPos_ADCU);}
static inline uint16_t Motor_User_GetSpeed_RPM(Motor_T * p_motor) 	{return p_motor->Speed_RPM;}

// Linear_Voltage_CalcMilliV(&p_motor->CONFIG.UNIT_V_POS, p_motor->AnalogResults.VPos_ADCU);
// Linear_Voltage_CalcMilliV(&p_motor->CONFIG.UNIT_V_ABC, p_motor->AnalogResults.Va_ADCU);


//static inline float Motor_User_GetHeat_DegCFloat(Motor_T *p_motor)						{return Thermistor_ConvertToDegC_Float(&p_motor->Thermistor, p_motor->AnalogResults.Heat_ADCU);}
//static inline uint16_t Motor_User_GetHeat_DegCRound(Motor_T *p_motor)					{return Thermistor_ConvertToDegC_Round(&p_motor->Thermistor, p_motor->AnalogResults.Heat_ADCU);}
//static inline uint16_t Motor_User_GetHeat_DegCNDecimal(Motor_T *p_motor, uint8_t n) 	{return Thermistor_ConvertToDegC_NDecimal(&p_motor->Thermistor, p_motor->AnalogResults.Heat_ADCU, n);}
static inline uint16_t Motor_User_GetHeat_DegCScalar(Motor_T * p_motor, uint16_t scalar) 	{return Thermistor_ConvertToDegC_Scalar(&p_motor->Thermistor, p_motor->AnalogResults.Heat_ADCU, scalar);}
//static inline uint16_t Motor_User_GetHeat_Fixed32(Motor_T *p_motor) 					{return Thermistor_ConvertToDegC_Fixed32(&p_motor->Thermistor, p_motor->AnalogResults.Heat_ADCU);}
static inline Motor_DirectionCalibration_T Motor_User_GetDirectionCalibration(Motor_T *p_motor) 	{return p_motor->Parameters.DirectionCalibration;}
static inline Hall_Sensors_T Motor_User_GetHall(Motor_T * p_motor) 	{return Hall_GetSensors(&p_motor->Hall);}
static inline uint16_t Motor_User_GetHallRotorAngle(Motor_T * p_motor) 	{return Hall_GetRotorAngle_Degrees16(&p_motor->Hall);}


//static inline void Motor_User_ActivatePhase(Motor_T * p_motor)
//{
//
//}

/*
 * Should Motor module proc state machine in non Pwm thread?
 * Pwm interrupt proc state StateMachine_Semisynchronous_ProcOutput
 * todo change to synchronous?
 */


/*
 * Disable control, motor may remain spinning
 */
static inline void Motor_User_DisableControl(Motor_T * p_motor)
{
	Phase_Float(&p_motor->Phase);
//	p_motor->Regen = 0U;
	StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_FLOAT);
}

static inline void Motor_User_SetCmd(Motor_T * p_motor, uint16_t cmd)
{
	if (p_motor->Regen == 1U)
	{
		Critical_Enter();
		p_motor->Regen = 0U;
		Motor_SetRamp(p_motor, cmd);
		//match resume speed/current?
//		Motor_ResumeSpeedFeedback(p_motor);
		Critical_Exit();
	}
	else
	{
		Motor_SetRamp(p_motor, cmd);
	}

	StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_ACCELERATE);
}

//static inline void Motor_User_SetCmdSpeed(Motor_T * p_motor, uint16_t speed)
//{
//	Motor_SetRamp(p_motor, cmd);
//	p_motor->IsBrake = false;
//	StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_ACCELERATE);
//}

static inline void Motor_User_SetCmdBrake(Motor_T * p_motor, uint16_t intensity)
{
	if (p_motor->Regen == 0U)
	{
		Critical_Enter();
		p_motor->Regen = 1U;
		p_motor->RampCmd = 0U; //ramp starts at 0 current for foc mode.


		//		if (p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
		//		{
		//			if (((p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_CURRENT) || (p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT)))
		//			{
		//				PID_SetIntegral(&p_motor->PidIq, FOC_GetIq(&p_motor->Foc));
		//			}
		//		}
		//		else //p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP
		//		{
		//			if ((p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_CURRENT) || (p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT))
		//			{
		//				PID_SetIntegral(&p_motor->PidIBus, p_motor->IBus_Frac16); //set vpwm proportional to current ibus
		//			}
		//		}

		//		if((p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT) || (p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE))
		//		{
		//			Motor_ResumeSpeedFeedback(p_motor);
		//		}

		Critical_Exit();
	}
	else
	{
		Motor_SetRamp(p_motor, intensity);
	}

///UserSemaphore = 0;
//	p_motor->ControlMode.Current = 1U;
//	Motor_SetRamp(p_motor, intensity);
	//UserSemaphore = 1;

	switch (p_motor->Parameters.BrakeMode)
	{
		case MOTOR_BRAKE_MODE_PASSIVE :
			Motor_User_DisableControl(p_motor);
			break;

//		case MOTOR_BRAKE_MODE_PROPRTIONAL :
//			Motor_SetRamp(p_motor, ((uint32_t)p_motor->SpeedFeedback_Frac16 * (65536U - intensity)) >> 16U);
//			//todo offset
//			//braking 0% -> pwm 100% of back emf;
//			//braking 10% -> pwm 90% of back emf;
//			//braking 50% -> pwm 50% of back emf;
//			//braking 90% -> pwm 10% of back emf;
//			break;

		case MOTOR_BRAKE_MODE_SCALAR:
			//minus 5% for zero
//			if (p_motor->SpeedFeedback_Frac16 < 65536U/20U)
//			{
//				Motor_SetRamp(p_motor, 0U);
//			}
//			else
//			{
//	//			p_motor->Parameters.BrakeCoeffcient
//				Motor_SetRamp(p_motor, (p_motor->SpeedFeedback_Frac16/2)); // - (1- intensity*25/100)
//			}

			//BrakeCoeffcient = 25 -> fract16(1/4)

			//braking 0% 	-> pwm 62.5% of back emf;
			//braking 10% 	-> pwm 60% of back emf;
			//braking 50% 	-> pwm 50% of back emf;
			//braking 90% 	-> pwm 40% of back emf;
			//braking 100% 	-> pwm 37.5% of back emf;

			break;

		default :
			break;
	}

	StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_DECELERATE);
}

//or set buffered direction, check on state machine  ?
static inline bool Motor_User_SetDirection(Motor_T * p_motor, Motor_Direction_T direction)
{
	bool isSet = false;

	if (p_motor->Speed_RPM == 0U)
	{
		Motor_SetDirection(p_motor, direction);
//		StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MSM_INPUT_DIRECTION);
		isSet = true;
	}

	return isSet;
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

static inline bool Motor_UserN_CheckIOverLimit(Motor_T * p_motor, uint8_t motorCount)
{
	bool isOverLimit = false;

	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		if(p_motor[iMotor].IOverLimitFlag == true)
		{
			isOverLimit = true;
			break;
		}
	}

	return isOverLimit;
}

#endif
