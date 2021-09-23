


//Filter acceptable input states
//limit io scope
// data members to fit all input / output requirements/peripherals

//#include "MotorInterface.h"
//
//#include "Motor/Motor/Motor.h"
//
////#include "Math/Linear/Linear_ADC.h"
//#include "Math/Linear/Linear_Voltage.h"
//#include "Math/Linear/Linear.h"
//

//
///*
// * Sw check throttle release
// */
//static inline bool CheckThrottleRelease(uint16_t  inputValueThrottle, uint16_t  inputValueThrottlePrev)
//{
//	bool isReleased;
//
//	if (inputValueThrottle < inputValueThrottlePrev)
//	{
//		if ((inputValueThrottlePrev - inputValueThrottle) > (65535U/10U))
//		{
//			isReleased == true;
//		}
//	}
//	else
//	{
//		isReleased == false;
//	}
//
//	return isReleased;
//}
//
//void MotorInterface_Input_SetThrottle(MotorInterface_Input_T * p_input, uint16_t throttle)
//{
//	p_input->InputValueThrottlePrev 	= p_input->InputValueThrottle;
//	p_input->InputValueThrottle 		= throttle;
//
//	p_input->InputIsThrottleRelease = CheckThrottleRelease(p_input->InputValueThrottle, p_input->InputValueThrottlePrev);
//}
//
//void MotorInterface_Input_SetBrake(MotorInterface_Input_T * p_input, uint16_t throttle)
//{
//	p_input->InputValueThrottlePrev 	= p_input->InputValueThrottle;
//	p_input->InputValueThrottle 		= throttle;
//}
//
//

//
//void MotorInterface_OutputMotor_GetVBus_mV		(const MotorInterface_Output_Motor_T * p_output){return p_output->VBus_mV;}
//void MotorInterface_OutputMotor_GetVBus_ADCU	(const MotorInterface_Output_Motor_T * p_output){return p_output->p_Motor->p_Constants->P_VBUS_ADCU;}
//
//void MotorInterface_OutputMotor_GetVBemfPeak_mV	(const MotorInterface_Output_Motor_T * p_output){return p_output->VBemfPeak_mV;}
//
//
//void MotorInterface_OutputMotor_GetVBus_mV			(const MotorInterface_Output_Map_T * p_map, uint8_t motorId){return MotorInterface_OutputMotor_GetVBus_mV(&p_map->P_OUTPUT_MOTORS[motorId]);}
//void MotorInterface_OutputMotors_GetVBemfPeak_mV	(const MotorInterface_Output_Map_T * p_map, uint8_t motorId){return MotorInterface_OutputMotor_GetVBemfPeak_mV(&p_map->P_OUTPUT_MOTORS[motorId]);}
//
////void MotorInterface_OutputMotors_GetVBemfPeak_ADCU	(const MotorInterface_Output_Map_T * p_map, uint8_t motorId){return MotorInterface_OutputMotor_GetVBemfPeak_mV(&p_map->P_OUTPUT_MOTORS[motorId]);}
//
//
