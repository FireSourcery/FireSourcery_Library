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
#include "Motor/Motor/Motor_Thread.h"

#include "Protocol/Protocol/Protocol.h"

#include "Peripheral/Analog/AnalogN/AnalogN.h"
#include "Peripheral/Serial/Serial.h"

#include "Utility/StateMachine/StateMachine.h"
#include "Utility/Timer/Timer.h"
#include "Utility/Shell/Shell.h"

static inline void MotorControllerAnalogUserThread(MotorController_T * p_mc)
{
	MotAnalogUser_Cmd_T 		cmd 		= MotAnalogUser_GetCmd(&p_mc->AnalogUser);
	MotAnalogUser_Direction_T 	direction 	= MotAnalogUser_PollDirection(&p_mc->AnalogUser);

	MotAnalogUser_CaptureInput(&p_mc->AnalogUser, p_mc->AnalogResults.Throttle_ADCU, p_mc->AnalogResults.Brake_ADCU);

	AnalogN_PauseQueue(p_mc->CONFIG.P_ANALOG_N, p_mc->CONFIG.ADCS_ACTIVE_MAIN_THREAD); //todo disable adc only, or fix critical within critical
	AnalogN_EnqueueConversion_Group(p_mc->CONFIG.P_ANALOG_N, &p_mc->CONFIG.CONVERSION_THROTTLE);
	AnalogN_EnqueueConversion_Group(p_mc->CONFIG.P_ANALOG_N, &p_mc->CONFIG.CONVERSION_BRAKE);
	AnalogN_ResumeQueue(p_mc->CONFIG.P_ANALOG_N, p_mc->CONFIG.ADCS_ACTIVE_MAIN_THREAD);

	if (cmd == MOT_ANALOG_USER_CMD_BRAKE)
	{
		MotorController_User_SetCmdBrake(p_mc, MotAnalogUser_GetBrake(&p_mc->AnalogUser));
	}
	else if ((direction == MOT_ANALOG_USER_DIRECTION_FORWARD) || (direction == MOT_ANALOG_USER_DIRECTION_REVERSE))
	{
		switch(cmd)
		{
			//			case MOT_ANALOG_USER_CMD_BRAKE: 				MotorController_User_SetCmdBrake	(p_mc, MotAnalogUser_GetBrake(&p_mc->AnalogUser)); 		break;
			case MOT_ANALOG_USER_CMD_THROTTLE:				MotorController_User_SetCmdThrottle	(p_mc, MotAnalogUser_GetThrottle(&p_mc->AnalogUser));	break;
			case MOT_ANALOG_USER_CMD_THROTTLE_RELEASE: 		MotorController_User_DisableControl(p_mc); break; // todo check throttle release param
			case MOT_ANALOG_USER_CMD_THROTTLE_ZERO_EDGE: 	MotorController_User_DisableControl(p_mc); break;
			case MOT_ANALOG_USER_CMD_THROTTLE_ZERO:			StateMachine_Semisynchronous_ProcInput(&p_mc->StateMachine, MCSM_INPUT_CHECK_STOP);		break;
			default: break;
		}
	}
	else
	{
		switch(direction)
		{
			case MOT_ANALOG_USER_DIRECTION_NEUTRAL: 		StateMachine_Semisynchronous_ProcInput(&p_mc->StateMachine, MCSM_INPUT_CHECK_STOP); break;
			case MOT_ANALOG_USER_DIRECTION_NEUTRAL_EDGE: 	MotorController_User_DisableControl(p_mc); break;
			case MOT_ANALOG_USER_DIRECTION_FORWARD_EDGE: 	MotorController_User_SetDirection(p_mc, MOTOR_CONTROLLER_DIRECTION_FORWARD); 		break;
			case MOT_ANALOG_USER_DIRECTION_REVERSE_EDGE: 	MotorController_User_SetDirection(p_mc, MOTOR_CONTROLLER_DIRECTION_REVERSE); 		break;
			default: break;
		}
	}
}

static inline void MotorController_Main_Thread(MotorController_T * p_mc)
{
	//Med Freq -  1 ms, Low Priority - Main
	if (Timer_Poll(&p_mc->TimerMillis) == true)
	{
		StateMachine_Semisynchronous_ProcOutput(&p_mc->StateMachine);

		switch(p_mc->Parameters.InputMode)
		{
			case MOTOR_CONTROLLER_INPUT_MODE_ANALOG:
				MotorControllerAnalogUserThread(p_mc);
				break;

	//		case MOTOR_CONTROLLER_INPUT_MODE_SERIAL:
	//			break;
	//
	//		case MOTOR_CONTROLLER_INPUT_MODE_CAN:
	//			break;

			default :
				break;
		}

		for(uint8_t iProtocol = 0U; iProtocol < p_mc->CONFIG.PROTOCOL_COUNT; iProtocol++)
		{
			Protocol_Slave_Proc(&p_mc->CONFIG.P_PROTOCOLS[iProtocol]);
		}

//		for (uint8_t iSerial = 0U; iSerial < p_mc->CONFIG.SERIAL_COUNT; iSerial++)
//		{
//			Serial_PollRxData(&p_mc->CONFIG.P_SERIALS[iSerial]);
//			Serial_PollTxData(&p_mc->CONFIG.P_SERIALS[iSerial]);
//		}
	}

	if (Timer_Poll(&p_mc->TimerMillis10) == true) 	//Low Freq, Low Priority 10 ms, Main
	{
		Shell_Proc(&p_mc->Shell);
		Blinky_Proc(&p_mc->Buzzer);
	}

	if (Timer_Poll(&p_mc->TimerSeconds) == true)
	{
		//In case of Serial Rx Overflow Timeout
		for (uint8_t iSerial = 0U; iSerial < p_mc->CONFIG.SERIAL_COUNT; iSerial++)
		{
			Serial_PollRestartRxIsr(&p_mc->CONFIG.P_SERIALS[iSerial]);
		}

		//main thread conversion remainder, motor heat
//#ifdef DEBUG

//#endif
	}

	/*
	 * High Freq, Low Priority, Main
	 */
//	for (uint8_t iMotor = 0U; iMotor < p_mc->CONFIG.MOTOR_COUNT; iMotor++)
//	{
//		Motor_Main_Thread(&p_mc->CONFIG.P_MOTORS[iMotor]);
//	}
}



/*
 * Wrappers
 */
static inline void MotorController_PWM_Thread(MotorController_T * p_mc)
{
	for (uint8_t iMotor = 0U; iMotor < p_mc->CONFIG.MOTOR_COUNT; iMotor++)
	{
		Motor_PWM_Thread(&p_mc->CONFIG.P_MOTORS[iMotor]);
	}

	//pwm thread conversions remainder, kls hs temp
}

//Med Freq, High Priority 1 ms
static inline void MotorController_Timer1Ms_Thread(MotorController_T * p_mc)
{
//	MotorController_PollBrake(p_mc);

	if(Motor_UserN_CheckIOverLimit(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT) == true)
	{
		Blinky_Blink(&p_mc->Buzzer, 500U);
	}

//		if (MotAnalogMonitor_CheckHeat_ADCU(&p_mc->AnalogMonitor,  p_mc->AnalogResults.HeatMosfetsTop_ADCU, p_mc->AnalogResults.HeatMosfetsBot_ADCU) != MOT_ANALOG_MONITOR_OK)
//		{
//
//		}

//		if (Thermistor_ProcThreshold(&p_mc->ThermistorPcb, p_mc->AnalogResults.HeatPcb_ADCU) != THERMISTOR_THRESHOLD_OK)
//		{
//			StateMachine_Semisynchronous_ProcInput(&p_mc->StateMachine, MCSM_INPUT_FAULT);
//		}

//		AnalogN_EnqueueConversion(p_mc->CONFIG.P_ANALOG_N, &p_mc->CONFIG.CONVERSION_HEAT_PCB);
//		AnalogN_EnqueueConversion(p_mc->CONFIG.P_ANALOG_N, &p_mc->CONFIG.CONVERSION_HEAT_MOSFETS_TOP);
//		AnalogN_EnqueueConversion(p_mc->CONFIG.P_ANALOG_N, &p_mc->CONFIG.CONVERSION_HEAT_MOSFETS_BOT);
//		AnalogN_EnqueueConversion(p_mc->CONFIG.P_ANALOG_N, &p_mc->CONFIG.CONVERSION_VACC);
//		AnalogN_EnqueueConversion(p_mc->CONFIG.P_ANALOG_N, &p_mc->CONFIG.CONVERSION_VSENSE);

//	for (uint8_t iMotor = 0U; iMotor < p_mc->CONFIG.MOTOR_COUNT; iMotor++)
//	{
//		Motor_Timer1Ms_Thread(&p_mc->CONFIG.P_MOTORS[iMotor]);
//	}
}

//static inline void MotorController_Serial_Thread(MotorController_T * p_mc, uint8_t serialId)
//{
//	Serial_RxData_ISR(&p_mc->CONFIG.P_SERIALS[serialId]);
//	Serial_TxData_ISR(&p_mc->CONFIG.P_SERIALS[serialId]);
//}

#endif
