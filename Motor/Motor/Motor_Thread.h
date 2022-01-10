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

#include "Motor.h"
#include "Utility/StateMachine/StateMachine.h"
//#include "Utility/Timer/Timer.h"

/*
    Default 50us
    Calling function must clear interrupt flag
 */
static inline void Motor_PWM_Thread(Motor_T * p_motor)
{
	p_motor->ControlTimerBase++;
	StateMachine_Semisynchronous_ProcOutput(&p_motor->StateMachine);
}

//1ms isr priority
static inline void Motor_Timer1Ms_Thread(Motor_T * p_motor)
{

}

//Low Freq Main //user/monitor thread 1ms low priority
static inline void Motor_Main1Ms_Thread(Motor_T * p_motor)
{

}

//low priotiy, max freq
static inline void Motor_Main_Thread(Motor_T * p_motor)
{
////Low Freq Main //user/monitor thread 1ms low priority
//	if (Timer_Poll(&p_motor->MillisTimer) == true)
//	{
//
//	}
}

#endif
