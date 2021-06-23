//#include "MotorUser.h"
//
//MotorUser_T MotorUserMain;
//
//static inline void MotorUser_CaptureInput( )
//{
//	//UI
//	switch (MotorUserMain.Mode)
//	{
//	case MOTOR_INPUT_MODE_ANALOG:
//		Debounce_CaptureState_IO(&MotorUserMain.PinBrake);
//		Debounce_CaptureState_IO(&MotorUserMain.PinThrottle);
//		Debounce_CaptureState_IO(&MotorUserMain.PinForward);
//		Debounce_CaptureState_IO(&MotorUserMain.PinReverse);
//
//		MotorUserMain->InputSwitchBrake 	= Debounce_GetState(&MotorUserMain.PinBrake);
//		MotorUserMain->InputSwitchThrottle 	= Debounce_GetState(&MotorUserMain.PinThrottle);
//		MotorUserMain->InputSwitchForward 	= Debounce_GetState(&MotorUserMain.PinForward);
//		MotorUserMain->InputSwitchReverse 	= Debounce_GetState(&MotorUserMain.PinReverse);
//		MotorUserMain->InputSwitchNeutral = MotorUserMain->InputSwitchForward | MotorUserMain->InputSwitchReverse ? false : true;
//
//
//		MotorUserMain->InputValueThrottle 	= Linear_ADC_CalcUnsignedFraction16(&MotorUserMain.UnitThrottle, *MotorUserMain.p_AdcChannelThrottle );
//		MotorUserMain->InputValueBrake 		= Linear_ADC_CalcUnsignedFraction16(&MotorUserMain.UnitBrake,  *MotorUserMain.p_AdcChannelBrake );
//
//
////			if (Debounce_GetState(&MotorUserMain.PinForward) == true)
////			{
////				p_motor->InputDirection = MOTOR_DIRECTION_CW;
////			}
////			else if (Debounce_GetState(&MotorUserMain.PinReverse) == true)
////			{
////				p_motor->InputDirection = MOTOR_DIRECTION_CCW;
////			}
////			else
////			{
////				p_motor->InputDirection = MOTOR_DIRECTION_NEUTRAL;
////			}
//
//		break;
//
//	case MOTOR_INPUT_MODE_SERIAL:
//		break;
//
////		case MOTOR_INPUT_MODE_SHELL:
////			break;
//
//	default:
//		break;
//}
//
//	//proc inputs
//static inline void MotorUser_ProcInput(Motor_T * p_motor)
//{
//
////	if(GetSwitchEdge(DirectionButton)) {StateMachine_Semisynchronous_ProcInput(&MotorUserMain.StateMachine, PIN_FUNCTION_FORWARD);}
////	if(GetSwitchEdge(DirectionButton)) {StateMachine_Semisynchronous_ProcInput(&MotorUserMain.StateMachine, PIN_FUNCTION_REVERSE);}
//
//
//
//  Motor_SetSetpointScalarBrake(p_motor);
//
//	//set ramp to p_motor->InputValueThrottle
//	//Setsetpoint ramp
//}
//

//Motor_OutputUnits()
//{
//
//}
//
//Motor_OutputADCU()
//{
//}
