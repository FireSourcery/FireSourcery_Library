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
	p_motor->ControlTimerBase++;

	StateMachine_Semisynchronous_ProcState(&p_motor->StateMachine);

}

/*
	ADC on complete. Possibly multiple per 1 PWM
	todo adc algo to coordinate multiple adc using 1 channel samples list. handle outside for now
 */
//static inline void Motor_ADC_Thread(Motor_T * p_motor)
//{
////	Analog_CaptureResults_IO(&p_motor->Analog);
//	//if Iabc complete, Motor_FOC_ProcCurrentControl(Motor_T * p_motor)
//}

static inline void Motor_Timer1Ms_Thread(Motor_T * p_motor) //1ms isr priotiy
{
	//Motor_SpeedLoop();
	//Monitor
	//		if(Motor_PollFault(p_motor))	{ StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_FAULT);	}
}

///*
// * SixStep openloop and sensorless
// */
//static inline void Motor_TimerCommutation_Thread(Motor_T * p_motor)
//{
////	BEMF_ProcTimer_IO(&p_motor->Bemf);
//}

static inline void Motor_Main_Thread(Motor_T * p_motor)
{
	//UI
	if (Thread_PollTimer(&p_motor->Timer1Ms) == true)
	{
		Thread_RestartTimer(&p_motor->Timer1Ms);

		p_motor->VReq = Linear_ADC_CalcUnsignedFraction16(&p_motor->UnitThrottle, p_motor->AnalogChannelResults[MOTOR_ANALOG_CHANNEL_THROTTLE]);

		//Motor_SetRamp(user input)
//		Linear_Ramp_InitMillis(&p_motor->Ramp, 4U, 10U, 1000U, 20000U);
//		p_motor->SpeedReq_RPM = Linear_Ramp_CalcTarget_IncIndex(&p_motor->Ramp, &p_motor->RampIndex, p_motor->CommutationPeriodCmd);

		p_motor->SpeedFeedback_RPM = Encoder_Motor_GetMechanicalRpm(&p_motor->Encoder); //speed UI

	}
}
