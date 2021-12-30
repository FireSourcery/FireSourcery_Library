/******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSoucery / The Firebrand Forge Inc

	This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
/******************************************************************************/
/******************************************************************************/
/*!
    @file 	MotorController_Thread.h
    @author FireSoucery
    @brief  MotorController module functions must be placed into corresponding user app threads.
    		Most outer functions to call from MCU app.
    @version V0
*/
/******************************************************************************/
#ifndef MOTOR_CONTROLLER_THREAD_H
#define MOTOR_CONTROLLER_THREAD_H

#include "MotorController.h"
#include "MotorController_User.h"
#include "MotorController_StateMachine.h"
#include "Config.h"

#include "Motor/Utility/MotAnalogUser/MotAnalogUser.h"
#include "Motor/Utility/MotAnalogMonitor/MotAnalogMonitor.h"
//#include "Motor/Utility/MotShell/MotShell.h"

#include "Motor/Motor/Motor_Thread.h"

#include "Protocol/Protocol/Protocol.h"

#include "Peripheral/Analog/AnalogN/AnalogN.h"
#include "Peripheral/Serial/Serial.h"

//#include "Utility/Shell/Shell.h"
#include "Utility/StateMachine/StateMachine.h"
#include "Utility/Timer/Timer.h"

//static uint32_t count;

static inline void MotorControllerAnalogUserThread(MotorController_T * p_controller)
{
	MotAnalogUser_Cmd_T 		cmd 		= MotAnalogUser_GetCmd(&p_controller->AnalogUser);
	MotAnalogUser_Direction_T 	direction 	= MotAnalogUser_PollDirection(&p_controller->AnalogUser);

	MotAnalogUser_CaptureInput(&p_controller->AnalogUser, p_controller->AnalogResults.Throttle_ADCU, p_controller->AnalogResults.Brake_ADCU);
	AnalogN_EnqueueConversion(p_controller->CONFIG.P_ANALOG_N, &p_controller->CONFIG.CONVERSION_ANALOG_USER);

	if (direction == MOT_ANALOG_USER_DIRECTION_NEUTRAL)
	{
		StateMachine_Semisynchronous_ProcInput(&p_controller->StateMachine, MCSM_INPUT_CHECK_STOP);
	}
	else
	{
		switch(direction)
		{
			case MOT_ANALOG_USER_DIRECTION_NEUTRAL_EDGE: MotorController_User_ProcDisableControl(p_controller); break;
			case MOT_ANALOG_USER_DIRECTION_FORWARD_EDGE: MotorController_User_ProcDirection(p_controller, MOTOR_CONTROLLER_DIRECTION_FORWARD); 		break;
			case MOT_ANALOG_USER_DIRECTION_REVERSE_EDGE: //MotorController_User_ProcDirection(p_controller, MOTOR_CONTROLLER_DIRECTION_REVERSE); 	break;
				Motor_User_ActivateCalibrationHall(&p_controller->CONFIG.P_MOTORS[0]);
				if (p_controller->Parameters.IsBuzzerOnReverseEnable == true) {MotorController_Beep(p_controller);}
				break;
			default: break;
		}

		if ((direction == MOT_ANALOG_USER_DIRECTION_FORWARD) || (direction == MOT_ANALOG_USER_DIRECTION_REVERSE))
		{
			switch(cmd)
			{
				case MOT_ANALOG_USER_CMD_THROTTLE:			MotorController_User_ProcCmdAccelerate(p_controller, MotAnalogUser_GetThrottle(&p_controller->AnalogUser));	break;
				case MOT_ANALOG_USER_CMD_BRAKE: 			MotorController_User_ProcCmdDecelerate(p_controller, MotAnalogUser_GetBrake(&p_controller->AnalogUser)); 	break;
				case MOT_ANALOG_USER_CMD_THROTTLE_RELEASE:
					//check throttle release param
					MotorController_User_ProcDisableControl(p_controller); break;
				case MOT_ANALOG_USER_CMD_THROTTLE_ZERO_EDGE: 	MotorController_User_ProcDisableControl(p_controller); 	break;
				case MOT_ANALOG_USER_CMD_THROTTLE_ZERO:			StateMachine_Semisynchronous_ProcInput(&p_controller->StateMachine, MCSM_INPUT_CHECK_STOP);		break;
				default: break;
			}
		}
	}
}

static inline void MotorController_Main_Thread(MotorController_T * p_controller)
{
	//Med Freq -  1 ms, Low Priority - Main
	if (Timer_Poll(&p_controller->TimerMillis) == true)
	{
		StateMachine_Semisynchronous_ProcState(&p_controller->StateMachine);

		switch(p_controller->Parameters.InputMode)
		{
			case MOTOR_CONTROLLER_INPUT_MODE_ANALOG:
				MotorControllerAnalogUserThread(p_controller);
				break;

	//		case MOTOR_CONTROLLER_INPUT_MODE_SERIAL:
	//			break;
	//
	//		case MOTOR_CONTROLLER_INPUT_MODE_CAN:
	//			break;

			default :
				break;
		}

//		if (MotAnalogMonitor_CheckHeat_ADCU(&p_controller->AnalogMonitor, p_controller->AnalogResults.HeatPcb_ADCU, p_controller->AnalogResults.HeatMosfetsTop_ADCU, p_controller->AnalogResults.HeatMosfetsBot_ADCU) != MOT_ANALOG_MONITOR_OK)
//		{
//			StateMachine_Semisynchronous_ProcTransition(&p_controller->StateMachine, MCSM_TRANSITION_FAULT);
//		}

//		AnalogN_EnqueueConversion(p_controller->CONFIG.P_ANALOG_N, &p_controller->CONFIG.CONVERSION_ANALOG_MONITOR);

//		Protocol_Slave_Proc(&p_controller->MotProtocol);

//		for(uint8_t iProtocol = 0U; iProtocol < CONFIG_MOTOR_CONTROLLER_AUX_PROTOCOL_COUNT; iProtocol++)
//		{
//			Protocol_Slave_Proc(&p_controller->AuxProtocols[iProtocol]);
//		}
	}

	if (Timer_Poll(&p_controller->TimerMillis10) == true) 	//Low Freq, Low Priority 10 ms, Main
	{
//		Shell_Proc(&p_controller->MotShell);
		Blinky_Proc(&p_controller->Buzzer);
	}

	if (Timer_Poll(&p_controller->TimerSeconds) == true)
	{
		//In case of Serial Rx Overflow Timeout
//		for (uint8_t iSerial = 0U; iSerial < CONFIG_MOTOR_CONTROLLER_SERIAL_COUNT; iSerial++)
//		{
//			Serial_PollRestartRxIsr(&p_motorController->Serials[iSerial]);
//		}
	}

	/*
	 * High Freq, Low Priority, Main
	 */
//	for (uint8_t iMotor = 0U; iMotor < p_controller->CONFIG.MOTOR_COUNT; iMotor++)
//	{
//		Motor_Main_Thread(&p_controller->CONFIG.P_MOTORS[iMotor]);
//	}
}

/*
 * Wrappers
 */
static inline void MotorController_PWM_Thread(MotorController_T * p_motorController)
{
	for (uint8_t iMotor = 0U; iMotor < p_motorController->CONFIG.MOTOR_COUNT; iMotor++)
	{
		Motor_PWM_Thread(&p_motorController->CONFIG.P_MOTORS[iMotor]);
	}
}

//Med Freq, High Priority 1 ms
static inline void MotorController_Timer1Ms_Thread(MotorController_T * p_motorController)
{
//	MotorController_PollBrake(p_motorController);

	for (uint8_t iMotor = 0U; iMotor < p_motorController->CONFIG.MOTOR_COUNT; iMotor++)
	{
		Motor_Timer1Ms_Thread(&p_motorController->CONFIG.P_MOTORS[iMotor]);
	}
}

//static inline void MotorController_Serial_Thread(MotorController_T * p_motorController, uint8_t serialId)
//{
//	Serial_RxData_ISR(&p_motorController->CONFIG.P_SERIALS[serialId]);
//	Serial_TxData_ISR(&p_motorController->CONFIG.P_SERIALS[serialId]);
//}

#endif
