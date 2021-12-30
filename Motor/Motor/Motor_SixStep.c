/*
 * Six Step commutation strategy
 */
#include "Motor.h"
#include "MotorAnalog.h"

#include "Transducer/BEMF/BEMF.h"

#include "Utility/Debug/Debug.h"

void Motor_SixStep_CaptureBemfA_DEBUG(Motor_T * p_motor)
{
	Debug_CaptureElapsed(8);
}

/*
 * On Adc measure of phase complete
 */
void Motor_SixStep_CaptureBemfA(Motor_T * p_motor)
{
//	Debug_CaptureElapsed(9);
	Bemf_CaptureVPos(&p_motor->Bemf, p_motor->AnalogResults.VPos_ADCU);
	Bemf_CaptureVPhase(&p_motor->Bemf, p_motor->AnalogResults.Va_ADCU);
}

//void Motor_SixStep_CaptureBemfAPwmOn(Motor_T * p_motor)
//{
////	Debug_CaptureElapsed(9);
//	Bemf_CaptureVPos(&p_motor->Bemf, p_motor->AnalogResults.VPos_ADCU);
//	Bemf_CaptureVPhase(&p_motor->Bemf, p_motor->AnalogResults.Va_ADCU);
//}
//
//void Motor_SixStep_CaptureBemfAPwmOff(Motor_T * p_motor)
//{
//	Bemf_CaptureVPhasePwmOff(&p_motor->Bemf, p_motor->AnalogResults.Va_ADCU);
//}

void Motor_SixStep_CaptureBemfB(Motor_T * p_motor)
{
	Bemf_CaptureVPos(&p_motor->Bemf, p_motor->AnalogResults.VPos_ADCU);
	Bemf_CaptureVPhase(&p_motor->Bemf, p_motor->AnalogResults.Vb_ADCU);
}

void Motor_SixStep_CaptureBemfC(Motor_T * p_motor)
{
	Bemf_CaptureVPos(&p_motor->Bemf, p_motor->AnalogResults.VPos_ADCU);
	Bemf_CaptureVPhase(&p_motor->Bemf, p_motor->AnalogResults.Vc_ADCU);
}
