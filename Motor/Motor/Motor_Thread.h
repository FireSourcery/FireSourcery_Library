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
#include "Motor.h"
//#include "HAL_Motor.h"
#include "Motor/Motor/Transducer/Phase/Phase.h"
#include "Utility/StateMachine/StateMachine.h"
//#include "Utility/Thread/Thread.h"
//#include "Utility/Timer/Timer.h"


/******************************************************************************/
/*!
    @name 	MotorThread
    @brief  Motor module functions must be placed into corresponding user app threads
    		Most outer functions to call from MCU app
*/
/* @{ */
/******************************************************************************/
/*
    Default 50us
 */
static inline void Motor_PWM_Thread(Motor_T * p_motor)
{
	p_motor->ControlTimerBase++;
	StateMachine_Semisynchronous_ProcState(&p_motor->StateMachine);
	//	Analog_LoadConversion(p_motor->CONFIG.P_ANALOG , p_motor->p_ConversionActive);
	Phase_ClearInterrupt(&p_motor->Phase); //proc after, clear interrupt flag that may be set up pwm update
}

static inline void Motor_Timer1Ms_Thread(Motor_T * p_motor) //1ms isr priority
{

	Motor_PollStop(p_motor); //todo transition freewheel state

	//high prioirty brake decel

//	if (p_motor->Parameters.BrakeMode == MOTOR_BRAKE_MODE_SCALAR)
//	{
//
//	}
	//Motor_SpeedLoop();
	//Monitor
	//		if(Motor_PollFault(p_motor))	{ StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_FAULT);	}
}


//Low Freq Main //user/monitor thread 1ms low priority
static inline void Motor_Main1Ms_Thread(Motor_T * p_motor)
{

//	HAL_Motor_EnqueueConversionIdle(p_motor); //handle in stop state
}

//low priotiy, max freq
static inline void Motor_Main_Thread(Motor_T * p_motor)
{
	if (Timer_Poll(&p_motor->MillisTimer) == true)
	{
		Motor_CaptureSpeed(p_motor);
		Motor_ProcUnitOutputs(p_motor);
	}
}

/******************************************************************************/
/*! @} */
/******************************************************************************/
///*
// * SixStep openloop and sensorless hw timer
// */
//static inline void Motor_TimerCommutation_Thread(Motor_T * p_motor)
//{
////	BEMF_ProcTimer_IO(&p_motor->Bemf);
//}

