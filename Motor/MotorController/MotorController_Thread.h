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

#include "Config.h"
#include "MotorController.h"

//#include "Motor/Utility/MotShell/MotShell.h"
//#include "Motor/Utility/MotProtocol/MotProtocol.h"
//#include "Motor/Utility/MotProtocol/MotProtocol_Motor.h"
#include "Motor/Utility/MotAnalogUser/MotAnalogUser.h"
#include "Motor/Utility/MotAnalogUser/MotAnalogUser_Motor.h"

#include "Motor/Motor/Motor_Thread.h"
//#include "Motor/Motor/HAL_Motor.h"

#include "Peripheral/Serial/Serial.h"
#include "Peripheral/Analog/AnalogN/AnalogN.h"
//#include "Utility/Shell/Shell.h"

#include "Utility/Timer/Timer.h"

#include "Protocol/Protocol/Protocol.h"

static uint32_t count;

static inline void MotorController_Main_Thread(MotorController_T * p_controller)
{
	if (Timer_Poll(&p_controller->TimerMillis) == true) 	//Med Freq, Low Priority 1 ms, Main
	{
//		for (uint8_t iMotor = 0U; iMotor < p_controller->CONFIG.MOTOR_COUNT; iMotor++)
//		{
//			Motor_Main1Ms_Thread(&p_controller->CONFIG.P_MOTORS[iMotor]);
//		}

//		if(p_controller->Parameters.AnalogUserEnable == true)
		{
			MotAnalogUser_CaptureInput(&p_controller->AnalogUser);
			MotAnalogUser_Motor_Write(&p_controller->AnalogUser, p_controller->CONFIG.P_MOTORS, p_controller->CONFIG.MOTOR_COUNT);

			AnalogN_EnqueueConversion(p_controller->CONFIG.P_ANALOG_N, &p_controller->CONFIG.CONVERSION_ANALOG_USER);
			count++;
		}

//		for(uint8_t iProtocol = 0U; iProtocol < CONFIG_MOTOR_CONTROLLER_AUX_PROTOCOL_COUNT; iProtocol++)
//		{
//			Protocol_Slave_Proc(&p_controller->AuxProtocols[iProtocol]);
//		}

//		Protocol_Slave_Proc(&p_controller->MotProtocol);
//		MotProtocol_Motor_WriteInput(&p_controller->MotProtocolInput,  &p_controller->Motors, CONFIG_MOTOR_CONTROLLER_MOTOR_COUNT);  //copy UI input to motor
//		MotProtocol_Acc_WriteInput(&p_motorController->Interface.Input,  &p_motorController->Motors[iMotor]);  //copy UI input to peripherals
//		MotProtocol_Motor_ReadOutput(&p_controller->MotProtocolOutput, &p_controller->Motors, CONFIG_MOTOR_CONTROLLER_MOTOR_COUNT);	//copy motor output to UI


//		MotorInterface_Input_ProcWriteToMotor(&p_motorController->Interface.Input,  &p_motorController->Motors[iMotor]);  //copy UI input to motor
//		MotorInterface_Output_ProcReadFromMotor(&p_motorController->Interface.Output, &p_motorController->Motors[iMotor]);	//copy motor output to UI
//		MotorInterface_Output_WriteTo(&p_motorController->MotorUser); //actuate hw

	}

	if (Timer_Poll(&p_controller->TimerMillis10) == true) 	//Low Freq, Low Priority 10 ms, Main
	{
//		Shell_Proc(&p_controller->MotShell);
	}

	if (Timer_Poll(&p_controller->TimerSeconds) == true)
	{
		count = 0;
		//Incase of Serial Rx Overflow Timeout
//		for (uint8_t iSerial = 0U; iSerial < CONFIG_MOTOR_CONTROLLER_SERIAL_COUNT; iSerial++)
//		{
//			Serial_PollRestartRxIsr(&p_motorController->Serials[iSerial]);
//		}

		Blinky_Proc(&p_controller->Buzzer);
	}


	/*
	 * High Freq, Low Priority, Main
	 */
	for (uint8_t iMotor = 0U; iMotor < p_controller->CONFIG.MOTOR_COUNT; iMotor++)
	{
		Motor_Main_Thread(&p_controller->CONFIG.P_MOTORS[iMotor]);
	}

	//	HAL_Motor_EnqueueConversionUser();
	//HAL_MotoAnalogr_EnqueueConversionUser();
	//HAL_MotoAnalogr_EnqueueConversionMonitor();

}

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
	for (uint8_t iMotor = 0U; iMotor < p_motorController->CONFIG.MOTOR_COUNT; iMotor++)
	{
//		Motor_Timer1Ms_Thread(&p_motorController->CONFIG.P_MOTORS[iMotor]);
	}
}

//static inline void MotorController_Serial_Thread(MotorController_T * p_motorController, uint8_t serialId)
//{
//	Serial_RxData_ISR(&p_motorController->CONFIG.P_SERIALS[serialId]);
//	Serial_TxData_ISR(&p_motorController->CONFIG.P_SERIALS[serialId]);
//}

#endif
