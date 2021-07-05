RAMP + PID FUNCTIONS

//void  ControlVariable_Proc(ControlVar_T * p_controlVar, uint32_t cmd)
//uint32_t Motor_CalcControlVariable(Motor_T * p_motor, Motor_ControlMode_T liveControlVarMode, uint32_t var) // input voltage/speed cmd
//{
//	switch (liveControlVarMode)
//	{
//	case MOTOR_CONTROL_MODE_OPEN_LOOP:
//		p_motor->VPwm = var/4U;
//		break;
//
//	case MOTOR_CONTROL_MODE_CONSTANT_VOLTAGE:
//
//
//
//		p_motor->VPwm = var;
//		break;
//
//	case MOTOR_CONTROL_MODE_SCALAR_VOLTAGE_FREQ:
//		//p_motor->VPwm = p_motor->RampCmd * p_motor->Speed_RPM * p_motor->VRpmGain;
//		break;
//
//	case MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE:
//		if (Thread_PollTimerCompletePeriodic(&p_motor->MillisTimerThread) == true)
//		{
//			 Motor_ProcSpeedFeedback(p_motor); //get from PID
//		}
//
//		break;
//
//	case MOTOR_CONTROL_MODE_CONSTANT_CURRENT:
//
//		//proc constant current pid
//		break;
//
//	default:
//		break;
//	}
//
//}
