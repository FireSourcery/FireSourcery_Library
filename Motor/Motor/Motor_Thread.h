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
	@file 	Motor_Thread.h
	@author FireSoucery
	@brief  Motor module functions must be placed into corresponding user app threads
			Most outer functions to call from main app
	@version V0
*/
/******************************************************************************/
#ifndef MOTOR_THREAD_H
#define MOTOR_THREAD_H

#include "Motor_StateMachine.h"
// #include "System/SysTime/SysTime.h"

/*
	Default 50us
	Calling function must clear interrupt flag
*/
static inline void Motor_PWM_Thread(Motor_T * p_motor)
{
	// p_motor->MicrosRef = SysTime_GetMicros();
	p_motor->ControlTimerBase++;

	if (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_SIN_COS)
	{
		//todo group
		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_SIN);
		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_COS);
	}

	//  _Motor_Analog_Thread( p_motor); use analog select mode to implement prefered order

	//	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
	//	{
	//		Motor_FOC_ProcAngleObserve(p_motor);
	//	}
	//	else /* p_motor->CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP */
	//	{
	//
	//	}

	StateMachine_Semi_ProcOutput(&p_motor->StateMachine);
}

/* THERMISTOR_WARNING repeatedly checks if heat is a lower ILimit when another ILimit is active */
static inline void Motor_Heat_Thread(Motor_T * p_motor)
{
	if(Thermistor_GetIsMonitorEnable(&p_motor->Thermistor) == true)
	{
		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_HEAT);

		switch(Thermistor_PollMonitor(&p_motor->Thermistor, p_motor->AnalogResults.Heat_Adcu))
		{
			case THERMISTOR_STATUS_OK: 	Motor_User_ClearILimitActive(p_motor, MOTOR_I_LIMIT_ACTIVE_HEAT);												break;
			case THERMISTOR_SHUTDOWN: 	Motor_User_SetFault(p_motor); 																					break;
			case THERMISTOR_WARNING: 	Motor_User_SetILimitActive(p_motor, p_motor->Parameters.ILimitScalarHeat_Frac16, MOTOR_I_LIMIT_ACTIVE_HEAT); 	break;
			default: break;
		}
	}
}

#endif


