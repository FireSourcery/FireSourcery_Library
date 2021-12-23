/*
 * Six Step commutation strategy
 */
#include "Motor.h"
#include "MotorAnalog.h"

#include "Transducer/BEMF/BEMF.h"

/*
 * On Adc measure of phase complete
 */
void Motor_SixStep_CaptureBemfA(Motor_T * p_motor)
{
//	Debug_CaptureElapsed(3);
	Bemf_CaptureVPos(&p_motor->Bemf, p_motor->AnalogResults[MOTOR_ANALOG_CHANNEL_VPOS]);
	Bemf_CaptureVPhase(&p_motor->Bemf, p_motor->AnalogResults[MOTOR_ANALOG_CHANNEL_VA]);
}

void Motor_SixStep_CaptureBemfB(Motor_T * p_motor)
{
	Bemf_CaptureVPos(&p_motor->Bemf, p_motor->AnalogResults[MOTOR_ANALOG_CHANNEL_VPOS]);
	Bemf_CaptureVPhase(&p_motor->Bemf, p_motor->AnalogResults[MOTOR_ANALOG_CHANNEL_VB]);
}

void Motor_SixStep_CaptureBemfC(Motor_T * p_motor)
{
	Bemf_CaptureVPos(&p_motor->Bemf, p_motor->AnalogResults[MOTOR_ANALOG_CHANNEL_VPOS]);
	Bemf_CaptureVPhase(&p_motor->Bemf, p_motor->AnalogResults[MOTOR_ANALOG_CHANNEL_VC]);
}
