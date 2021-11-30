#include "Motor.h"

//#include "Config.h"
////#include "Default.h"
//
//#include "MotorStateMachine.h"
//
//#include "Transducer/Phase/Phase.h"
//#include "Transducer/Hall/Hall.h"
//#include "Transducer/BEMF/BEMF.h"
//
//#include "Transducer/Encoder/Encoder_Motor.h"
//#include "Transducer/Encoder/Encoder_DeltaT.h"
//#include "Transducer/Encoder/Encoder_DeltaD.h"
//#include "Transducer/Encoder/Encoder.h"
//
//#include "System/StateMachine/StateMachine.h"
//#include "System/Thread/Thread.h"
//
//#include "Math/Linear/Linear_ADC.h"
//#include "Math/Linear/Linear_Voltage.h"
//#include "Math/Linear/Linear.h"

//todo user access scope, wrapper functions

//void Motor_User_StartControl(Motor_T * p_motor)
//{
//	StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_RUN);
//	p_motor->IsActiveControl = true;
//}

void Motor_User_SetThrottle(Motor_T * p_motor, uint16_t throttle)
{
 //		if throttle > p_motor->VPwm)

	Motor_SetUserCmd(p_motor, throttle);
	Motor_SetRampThrottle(p_motor, throttle);
	p_motor->IsActiveControl = true;
}

void Motor_User_SetBrake(Motor_T * p_motor, uint16_t brake)
{
	if (p_motor->Parameters.BrakeMode == MOTOR_BRAKE_MODE_PASSIVE)
	{
		p_motor->IsActiveControl = false; //or set state?
	}
	else
	{
		Motor_SetUserCmd(p_motor, brake);
		Motor_SetRampBrake(p_motor, brake);
		p_motor->IsActiveControl = true;
	}
}

void Motor_User_Disable(Motor_T * p_motor)
{
//	Phase_Float(&p_motor->Phase);
	p_motor->IsActiveControl = false;
}

////set buffered direction, check on state machine run
void Motor_User_SetDirection(Motor_T * p_motor, Motor_Direction_T direction)
{
	p_motor->DirectionInput = direction;
}

void Motor_User_ProcDirection(Motor_T * p_motor, Motor_Direction_T direction)
{
		//		 if (p_motor->Direction != p_motor->InputDirection)//direction reversed
		//		{
}

//static inline bool Motor_SetDirection_ (Motor_T * p_motor)
//{
//	//proc direction
//	//		 if (p_motor->Direction != p_motor->InputDirection)//direction reversed
//	//		{
//	//			Blinky_SetOnTime(&p_Motor->Alarm, 1000)
//	//		}
//}
//
//static inline bool Motor_SetDirection_ (Motor_T * p_motor)
//{
//	if (p_motor->Direction != p_motor->DirectionInput)
//	{
//		Motor_SetDirection(p_motor->DirectionInput);
//	}
//}

