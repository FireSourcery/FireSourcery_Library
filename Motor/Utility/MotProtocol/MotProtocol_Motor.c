#include "MotProtocol.h"

#include <stdint.h>
#include <stdbool.h>

void MotProtocol_Motor_WriteInputMain(const MotProtocol_Input_T * input, Motor_T * p_motorDest, uint8_t motorCount)
{
//	switch(WriteMode)
//	{
//		if (input->InputSwitchForward == true) // ^ reverse direction
//		{
//			Motor_SetDirectionInput(p_motorDest, MOTOR_DIRECTION_CCW);
//		}
//		else if (input->InputSwitchReverse == true)
//		{
//			Motor_SetDirectionInput(p_motorDest, MOTOR_DIRECTION_CW);
//		}
//
//		//set ramp per second
//		if (input->InputSwitchBrake == true)
//		{
//			Motor_SetUserCmd(p_motorDest, (input->InputValueBrake + input->InputValueBrakePrev) / 2U);
//			Motor_SetRampDecelerate(p_motorDest, 0);
//
//	//			Motor_User_SetDecelerate(p_motorDest, (input->InputValueBrake + input->InputValueBrakePrev) / 2U);
//		}
//		else
//		{
//			Motor_SetUserCmd(p_motorDest, (input->InputValueThrottle + input->InputValueThrottlePrev) / 2U);
//			Motor_SetRampAccelerate(p_motorDest, 0);
//
//	//			Motor_User_SetAccelerate(p_motorDest, (input->InputValueThrottle + input->InputValueThrottlePrev) / 2U);
//		}
//	}
}

void MotProtocol_Motor_WriteInput(const MotProtocol_Input_T * input, Motor_T * p_motorDest, uint8_t motorCount)
{
	MotProtocol_InputProcess_T cmd = input->FunctionCmd;

	//error checking input->FunctionCmd

	switch (cmd)
	{

	case MOT_PROTOCOL_INPUT_PROCESS_A:
		for (uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
		{
			if(input->UseMain[iMotor] == true)
			{
				Motor_User_SetAccelerate(&p_motorDest[iMotor], input->MainValueThrottle);
			}
			else
			{
				Motor_User_SetAccelerate(&p_motorDest[iMotor], input->Motors[iMotor].ValueThrottle);
			}
		}
		break;

	default: break;

	}

}


//void MotProtocol_Output_ReadFromMotor(MotProtocol_Output_T * p_output, uint8_t motorBufferId, const Motor_T * p_motorSrc)
//{
//	p_output->Motors[motorBufferId]->VBus_mV 		= Linear_Voltage_CalcMilliV(&p_motorSrc->UnitVBus, *p_motorSrc->p_Constants->P_VBUS_ADCU);
//	p_output->Motors[motorBufferId]->VBemfPeak_mV 	= Linear_Voltage_CalcMilliV(&p_motorSrc->UnitVabc, BEMF_GetVBemfPeak_ADCU(&p_motorSrc->Bemf));
//}


void MotProtocol_Motor_ReadOutput(MotProtocol_Output_T * p_output, const Motor_T * p_motorSrc, uint8_t motorCount)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		p_output->Motors[iMotor].VBus_mV 		= Linear_Voltage_CalcMilliV(&p_motorSrc->UnitVBus, *p_motorSrc->p_Constants->P_VBUS_ADCU);
		p_output->Motors[iMotor].VBemfPeak_mV 	= Linear_Voltage_CalcMilliV(&p_motorSrc->UnitVabc, BEMF_GetVBemfPeak_ADCU(&p_motorSrc->Bemf));
	}
}

