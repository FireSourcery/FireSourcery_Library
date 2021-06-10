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

#include "HAL_Motor.h"

#include "Peripheral/Analog/Analog_IO.h"

#include "System/StateMachine/StateMachine.h"


/*
    Default 50us
 */
static inline void Motor_PWM_Thread(Motor_T * p_motor)
{
//	HAL_Phase_ClearInterrupt(&p_motor->Phase);
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
	p_motor->MillisTimerBase++;

	Motor_PollStop(p_motor);
	Motor_CalcSpeed(p_motor);
	//high prioirty brake decel
//	if (MotorUser_PollBrake(p_motor))
//	if (p_motor->InputSwitchBrake == true)
//	{
//		if (p_motor->Parameters.BrakeMode == MOTOR_BRAKE_MODE_SCALAR)
//		{
////				p_motor->UserCmdDelta = (int32_t) p_motor->UserCmd - (int32_t)p_motor->InputValueBrake;
//
//			p_motor->UserCmd =  p_motor->InputValueBrake;
//			Motor_SetRampDecel(p_motor);
//		}
////			else 	//set flag
////			{
////				if (p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
////				{
////
////				}
////				else //p_motor->CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP
////				{
////
////				}
////			}
//
//	}


	if (p_motor->Parameters.BrakeMode == MOTOR_BRAKE_MODE_SCALAR)
	{

	}


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


static inline bool MotorUser_GetCmdRelease(Motor_T * p_motor)
{
	bool isReleased;

	if (p_motor->InputValueThrottle < p_motor->InputValueThrottlePrev)
	{
		if ((p_motor->InputValueThrottlePrev - p_motor->InputValueThrottle) > (65535/100))
		{
			isReleased == true;
		}
	}
	else
	{
		isReleased == false;
	}

	return isReleased;
}

static inline void Motor_Main_Thread(Motor_T * p_motor)
{

	if (Thread_PollTimer(&p_motor->MillisTimerThread) == true)
	{

		//UI

		//read inputs
		switch (p_motor->Parameters.InputMode)
		{
		case MOTOR_INPUT_MODE_ANALOG:
			Debounce_CaptureState_IO(&p_motor->PinBrake);
			Debounce_CaptureState_IO(&p_motor->PinThrottle);
			Debounce_CaptureState_IO(&p_motor->PinForward);
			Debounce_CaptureState_IO(&p_motor->PinReverse);

			p_motor->InputSwitchBrake 		= Debounce_GetState(&p_motor->PinBrake);
			p_motor->InputSwitchThrottle 	= Debounce_GetState(&p_motor->PinThrottle);
			p_motor->InputSwitchForward 	= Debounce_GetState(&p_motor->PinForward);
			p_motor->InputSwitchReverse 	= Debounce_GetState(&p_motor->PinReverse);
			//p_motor->InputSwitchNeutral = Debounce_GetState(&p_motor->PinNeutral);
//			p_motor->InputSwitchNeutral = p_motor->InputSwitchForward | p_motor->InputSwitchReverse ? false : true;

			p_motor->InputValueThrottlePrev = p_motor->InputValueThrottle;
			p_motor->InputValueBrakePrev = p_motor->InputValueBrake;
			p_motor->InputValueThrottle 	= Linear_ADC_CalcUnsignedFraction16(&p_motor->UnitThrottle, p_motor->AnalogChannelResults[MOTOR_ANALOG_CHANNEL_THROTTLE]);
			p_motor->InputValueBrake 		= Linear_ADC_CalcUnsignedFraction16(&p_motor->UnitBrake, p_motor->AnalogChannelResults[MOTOR_ANALOG_CHANNEL_BRAKE]);



			break;

		case MOTOR_INPUT_MODE_SERIAL:
			break;

//		case MOTOR_INPUT_MODE_SHELL:
//			break;

		default:
			break;
		}

		//proc inputs

		//	if(GetSwitchEdge(DirectionButton)) {StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, PIN_FUNCTION_FORWARD);}
		//	if(GetSwitchEdge(DirectionButton)) {StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, PIN_FUNCTION_REVERSE);}

//		p_motor->IsDirectionNeutral = p_motor->InputSwitchForward | p_motor->InputSwitchReverse ? false : true;
//
//		if(p_motor->IsDirectionNeutral == false)
//		{
			p_motor->IsDirectionNeutral = false;
			if (p_motor->InputSwitchForward  == true) // ^ reverse direction
			{
				p_motor->DirectionInput = MOTOR_DIRECTION_CCW;
			}
			else if (p_motor->InputSwitchReverse == true)
			{
				p_motor->DirectionInput = MOTOR_DIRECTION_CW;
			}
			else
			{
				p_motor->IsDirectionNeutral = true;
			}
//		}

//		if (MotorUser_GetCmdRelease()) {SetMotorReleaseFlag()}

		//set ramp per second
		if (p_motor->InputSwitchBrake == true)
		{
			if (p_motor->Parameters.BrakeMode == MOTOR_BRAKE_MODE_SCALAR)
			{


//				if (p_motor->InputValueBrake < p_motor->InputValueBrakePrev)
//				{
//					p_motor->IsThrottleRelease == true;
//				}
//				else
//				{
//					p_motor->IsThrottleRelease == false;
//				}
				p_motor->UserCmd =  p_motor->InputValueBrake;
				Motor_SetRampDecel(p_motor);
			}
//			else
//			{
//				if (p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
//				{
//
//				}
//				else //p_motor->CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP
//				{
//
//				}
//			}
		}
		else
		{
			if (p_motor->InputValueThrottle < p_motor->InputValueThrottlePrev)
			{
				if ((p_motor->InputValueThrottlePrev - p_motor->InputValueThrottle) > (65535/2000))
				{
					p_motor->IsThrottleRelease == true;
				}
			}
			else
			{
					p_motor->IsThrottleRelease == false;

			}

//			if (Thread_PollTimer(&p_motor->SecondsTimerThread) == true)
			{
//			if (p_motor->InputValueThrottle > feedback)
				p_motor->UserCmd = p_motor->InputValueThrottle;
				Motor_SetRampAccel(p_motor);
			}
		}


	}
}
