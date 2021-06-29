

typedef struct
{
	//UI - change to 1 per all motor?
	Debounce_T PinBrake;
	Debounce_T PinThrottle;
	Debounce_T PinForward;
	Debounce_T PinReverse;
	volatile bool InputSwitchBrake;
	volatile bool InputSwitchThrottle;
	volatile bool InputSwitchForward;
	volatile bool InputSwitchReverse;
	volatile bool InputSwitchNeutral;
	volatile uint16_t InputValueThrottle;
	volatile uint16_t InputValueBrake;

} MotorUser_T;

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


//static inline void Motor_User_Thread(Motor_T * p_motor)
//{
//
//	if (Thread_PollTimerCompletePeriodic(&p_motor->MillisTimerThread) == true)
//	{
//
//		//UI
//
//		//read inputs
//		switch (p_motor->Parameters.InputMode)
//		{
//		case MOTOR_INPUT_MODE_ANALOG:
//			Debounce_CaptureState_IO(&p_motor->PinBrake);
//			Debounce_CaptureState_IO(&p_motor->PinThrottle);
//			Debounce_CaptureState_IO(&p_motor->PinForward);
//			Debounce_CaptureState_IO(&p_motor->PinReverse);
//
//			p_motor->InputSwitchBrake 		= Debounce_GetState(&p_motor->PinBrake);
//			p_motor->InputSwitchThrottle 	= Debounce_GetState(&p_motor->PinThrottle);
//			p_motor->InputSwitchForward 	= Debounce_GetState(&p_motor->PinForward);
//			p_motor->InputSwitchReverse 	= Debounce_GetState(&p_motor->PinReverse);
//			//p_motor->InputSwitchNeutral = Debounce_GetState(&p_motor->PinNeutral);
////			p_motor->InputSwitchNeutral = p_motor->InputSwitchForward | p_motor->InputSwitchReverse ? false : true;
//
//			p_motor->InputValueThrottlePrev = p_motor->InputValueThrottle;
//			p_motor->InputValueBrakePrev = p_motor->InputValueBrake;
//			p_motor->InputValueThrottle 	= Linear_ADC_CalcFractionUnsigned16(&p_motor->UnitThrottle, p_motor->AnalogChannelResults[MOTOR_ANALOG_CHANNEL_THROTTLE]);
//			p_motor->InputValueBrake 		= Linear_ADC_CalcFractionUnsigned16(&p_motor->UnitBrake, p_motor->AnalogChannelResults[MOTOR_ANALOG_CHANNEL_BRAKE]);
//
//			break;
//
//		case MOTOR_INPUT_MODE_SERIAL:
//			break;
//
////		case MOTOR_INPUT_MODE_SHELL:
////			break;
//
//		default:
//			break;
//		}
//
//		//proc inputs
//		//	if(GetSwitchEdge(DirectionButton)) {StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, PIN_FUNCTION_FORWARD);}
//		//	if(GetSwitchEdge(DirectionButton)) {StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, PIN_FUNCTION_REVERSE);}
//
////		p_motor->IsDirectionNeutral = p_motor->InputSwitchForward | p_motor->InputSwitchReverse ? false : true;
////
////		if(p_motor->IsDirectionNeutral == false)
////		{
//			p_motor->IsDirectionNeutral = false;
//			if (p_motor->InputSwitchForward  == true) // ^ reverse direction
//			{
//				p_motor->DirectionInput = MOTOR_DIRECTION_CCW;
//			}
//			else if (p_motor->InputSwitchReverse == true)
//			{
//				p_motor->DirectionInput = MOTOR_DIRECTION_CW;
//			}
//			else
//			{
//				p_motor->IsDirectionNeutral = true;
//			}
////		}
//
////		if (MotorUser_GetCmdRelease()) {SetMotorReleaseFlag()}
//
//		//set ramp per second
//		if (p_motor->InputSwitchBrake == true)
//		{
//			if (p_motor->Parameters.BrakeMode == MOTOR_BRAKE_MODE_SCALAR)
//			{
////				if (p_motor->InputValueBrake < p_motor->InputValueBrakePrev)
////				{
////					p_motor->IsThrottleRelease == true;
////				}
////				else
////				{
////					p_motor->IsThrottleRelease == false;
////				}
//
//				Motor_SetUserCmd(p_motor, p_motor->InputValueBrake);
//				Motor_SetRampDecelerate(p_motor, p_motor->InputValueBrake);
//			}
//		}
//		else
//		{
//			if (p_motor->InputValueThrottle < p_motor->InputValueThrottlePrev)
//			{
//				if ((p_motor->InputValueThrottlePrev - p_motor->InputValueThrottle) > (65535/2000))
//				{
//					p_motor->IsThrottleRelease == true;
//				}
//			}
//			else
//			{
//					p_motor->IsThrottleRelease == false;
//			}
//
//			Motor_SetUserCmd(p_motor, p_motor->InputValueThrottle);
//			Motor_SetRampAccelerate(p_motor, p_motor->InputValueThrottle);
//		}
//
//
//	}
//}
