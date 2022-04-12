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
    		Most outer functions to call from MCU app
    @version V0
*/
/******************************************************************************/
#ifndef MOTOR_THREAD_H
#define MOTOR_THREAD_H

#include "Motor_StateMachine.h"
#include "Motor.h"

#include "System/SysTime/SysTime.h"

/*
    Default 50us
    Calling function must clear interrupt flag
 */
static inline void Motor_PWM_Thread(Motor_T * p_motor)
{
	p_motor->MicrosRef = SysTime_GetMicros();
	p_motor->ControlTimerBase++;
	StateMachine_Semisynchronous_ProcState(&p_motor->StateMachine);
}

//1ms isr priority
static inline void Motor_Timer1Ms_Thread(Motor_T * p_motor)
{

}


static inline void Motor_Heat_Thread(Motor_T * p_motor)
{
	if(Thermistor_GetIsEnable(&p_motor->Thermistor))
	{
		AnalogN_PauseQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ADCS_ACTIVE_PWM_THREAD);
		AnalogN_EnqueueConversion_Group(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_HEAT );
		AnalogN_ResumeQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ADCS_ACTIVE_PWM_THREAD);

		if (Thermistor_ProcThreshold(&p_motor->Thermistor, p_motor->AnalogResults.Heat_ADCU) != THERMISTOR_THRESHOLD_OK)
		{
			p_motor->ErrorFlags.OverHeat = 1U;
			StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_FAULT);
		}
	}
}
#endif
