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
#include "Motor_FOC.h"
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

	p_motor->DebugTime[0] = SysTime_GetMicros() - p_motor->MicrosRef;

	AnalogN_Group_PauseQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_PWM);
	if (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_SIN_COS)
	{
		AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_SIN);
		AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_COS);
	}

	if(StateMachine_GetActiveStateId(&p_motor->StateMachine) == MSM_STATE_ID_RUN)
	{
		AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IA);
		AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IB);
#if defined(CONFIG_MOTOR_I_SENSORS_ABC) && !defined(CONFIG_MOTOR_I_SENSORS_AB)
		AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IC);
#endif
	}
	else if((StateMachine_GetActiveStateId(&p_motor->StateMachine) == MSM_STATE_ID_FREEWHEEL) || (StateMachine_GetActiveStateId(&p_motor->StateMachine) == MSM_STATE_ID_STOP)) //todo flag
	{
		//if observe bemf
#if !defined(CONFIG_MOTOR_V_SENSORS_ISOLATED) &&defined(CONFIG_MOTOR_V_SENSORS_ADC)
		AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_VA);
		AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_VB);
		AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_VC);
#endif
	}
	AnalogN_Group_ResumeQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_PWM);


	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
	{
		Motor_FOC_ProcAngleObserve(p_motor);
	}
	else /* p_motor->CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP */
	{

	}

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
		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_HEAT);

		if (Thermistor_ProcThreshold(&p_motor->Thermistor, p_motor->AnalogResults.Heat_ADCU) != THERMISTOR_THRESHOLD_OK)
		{
			p_motor->ErrorFlags.OverHeat = 1U;
			StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_FAULT);
		}
	}
}
#endif
