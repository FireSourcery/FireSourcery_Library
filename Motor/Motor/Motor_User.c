#include "Motor.h"

//#include "Config.h"
////#include "Default.h"
//
//#include "MotorStateMachine.h" //circular modular dependency only for MotorStateMachine_Init
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

//user access scope, wrapper functions
void Motor_User_SetAccelerate(Motor_T * p_motor, uint16_t acceration)
{
	Motor_SetRampAccelerate(p_motor, acceration);
}

void Motor_User_SetDecelerate(Motor_T * p_motor, uint16_t deceleration)
{
	if (p_motor->Parameters.BrakeMode == MOTOR_BRAKE_MODE_PASSIVE)
	{
		p_motor->IsActiveControl = false;
	}
	else
	{
		p_motor->IsActiveControl = true;
		Motor_SetRampDecelerate(p_motor, deceleration);
	}
}

void Motor_User_SetDirection(Motor_T * p_motor, Motor_Direction_T direction)
{

}

void Motor_User_Disable(Motor_T * p_motor)
{
//	Phase_Float(&p_motor->Phase);
	p_motor->IsActiveControl = false;
}
