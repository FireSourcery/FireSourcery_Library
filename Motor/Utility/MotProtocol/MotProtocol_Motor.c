#include "MotProtocol.h"

#include <stdint.h>
#include <stdbool.h>

void MotProtocol_Motor_SetBrake(const MotProtocol_Input_T * input, Motor_T * p_motorDest )
{
	bool activeControl = false;

	if (p_user->InputSwitchNeutral == false)
	{
		if ((p_user->InputSwitchBrake == true) || ((p_user->InputValueThrottle > 5U) && (p_user->IsThrottleRelease == false)))
		{
			activeControl = true;
		}
	}

	if (activeControl == false)
	{
		if (p_user->InputSwitchForward == true)
		{
			for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
			{
				Motor_User_SetDirection(&p_motorDest[iMotor], MOTOR_DIRECTION_CCW);
			}
		}
		else if (p_user->InputSwitchReverse == true)
		{
			for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
			{
				Motor_User_SetDirection(&p_motorDest[iMotor], MOTOR_DIRECTION_CW);
			}
		}

		for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
		{
			Motor_User_Disable(&p_motorDest[iMotor]);	//set state freewheel
		}
	}
	else
	{
		//set ramp slope per second
		if (p_user->InputSwitchBrake == true)
		{
			for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
			{
				Motor_User_SetBrake(&p_motorDest[iMotor], p_user->InputValueBrake);
			}
		}
		else
		{
			for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
			{
				Motor_User_SetThrottle(&p_motorDest[iMotor], p_user->InputValueThrottle);
			}
		}
	}
}

void MotProtocol_Motor_SetBrakeAll(const MotProtocol_Input_T * input, Motor_T * p_motorDest )
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_SetBrake(&p_motorDest[iMotor], p_user->InputValueBrake);
	}
}


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

//void MotProtocol_Motor_WriteInput(const MotProtocol_Input_T * input, Motor_T * p_motorDest, uint8_t motorCount)
//{
//	MotProtocol_InputProcess_T cmd = input->FunctionCmd;
//
//	//error checking input->FunctionCmd
//
//	switch (cmd)
//	{
//
//	case MOT_PROTOCOL_INPUT_PROCESS_A:
//		for (uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
//		{
//			if(input->UseMain[iMotor] == true)
//			{
//				Motor_User_SetAccelerate(&p_motorDest[iMotor], input->MainValueThrottle);
//			}
//			else
//			{
//				Motor_User_SetAccelerate(&p_motorDest[iMotor], input->Motors[iMotor].ValueThrottle);
//			}
//		}
//		break;
//
//	default: break;
//
//	}
//
//}




