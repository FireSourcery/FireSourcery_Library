/**************************************************************************/
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
/**************************************************************************/
/**************************************************************************/
/*!
    @file 	Motor_IO.h
    @author FireSoucery
    @brief  Motor module functions must be placed into corresponding user app threads
    		Most outer functions to call from MCU app
    @version V0
*/
/**************************************************************************/
#include "Motor.h"
#include "Motor_FOC.h"
#include "Motor_SixStep.h"

#include "HAL.h"

#include "Peripheral/Analog/Analog_IO.h"

#include "System/StateMachine/StateMachine.h"


/*
    Default 50us
 */
static inline void Motor_PWM_Thread(Motor_T * p_motor)
{
	p_motor->ControlTimer++;

	//StateMachine_Semisynchronous_ProcState(&p_motor->StateMachine);
	Motor_SixStep_ProcCommutationControl(p_motor);



//		Motor_FOC_ProcAngleControl(p_motor);


}


/*
	ADC on complete. Possibly multiple per 1 PWM
	todo adc algo to coordinate multiple adc using 1 channel samples list
 */
static inline void Motor_ADC_Thread(Motor_T * p_motor)
{
//	Analog_CaptureResults_IO(&p_motor->Analog);
	//if complete == Iabc Motor_FOC_ProcCurrentControl(Motor_T * p_motor)
}

static inline void Motor_Timer1Ms_Thread(Motor_T * p_motor) //1ms
{
	//Motor_SpeedLoop();
	//Monitor
}

/*
 * SixStep openloop and sensorless
 */
static inline void Motor_TimerCommutation_Thread(Motor_T * p_motor)
{
//	BEMF_ProcTimer_IO(&p_motor->Bemf);
}

static inline void Motor_Main_Thread(Motor_T * p_motor)
{
	//UI
	//Motor_SetRamp(user input)
}
