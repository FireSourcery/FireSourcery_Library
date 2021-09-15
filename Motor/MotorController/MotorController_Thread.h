/**************************************************************************/
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
/**************************************************************************/
/**************************************************************************/
/*!
    @file 	MotorController_Thread.h
    @author FireSoucery
    @brief  MotorController module functions must be placed into corresponding user app threads.
    		Most outer functions to call from MCU app.
    @version V0
*/
/**************************************************************************/
#ifndef MOTOR_CONTROLLER_THREAD_H
#define MOTOR_CONTROLLER_THREAD_H

#include "Config.h"

#include "MotorController.h"

#include "Motor/Utility/MotorUser.h"

#include "Motor/Motor_Thread.h"
#include "Motor/HAL_Motor.h"

#include "Peripheral/Serial/Serial.h"
#include "System/Shell/Shell.h"

static inline void MotorController_Main_Thread(MotorController_T * p_motorController)
{
	if (Thread_PollTimerCompletePeriodic(&p_motorController->TimerMillis) == true) 	//Med Freq, Low Priority 1 ms, Main
	{
		MotorUser_Analog_CaptureInput_IO(&p_motorController->MotorUser);
		MotorUser_WriteOutput(&p_motorController->MotorUser); //actuate hw

		for (uint8_t iMotor = 0U; iMotor < CONFIG_MOTOR_CONTROLLER_MOTOR_COUNT; iMotor++)
		{
			MotorUser_SetMotorInput(&p_motorController->MotorUser, &p_motorController->Motors[iMotor]); //copy UI input to motor
			MotorUser_SetUserOutput(&p_motorController->MotorUser, &p_motorController->Motors[iMotor]); //copy motor output to UI

			Motor_Main1Ms_Thread(&p_motorController->Motors[iMotor]);
		}

		HAL_Motor_EnqueueConversionUser();
		//HAL_MotorController_EnqueueConversionUser();
		//HAL_MotorController_EnqueueConversionMonitor();

		//		Shell_Proc(&p_motorController->MotorControllerShell);
	}

	if (Thread_PollTimerCompletePeriodic(&p_motorController->TimerMillis10) == true) 	//Low Freq, Low Priority 1 ms, Main
	{
		Shell_Proc(&p_motorController->MotorShell);
	}


	if (Thread_PollTimerCompletePeriodic(&p_motorController->TimerSeconds) == true)
	{
		//Incase of Serial Rx Overflow Timeout
//		for (uint8_t iSerial = 0U; iSerial < CONFIG_MOTOR_CONTROLLER_SERIAL_COUNT; iSerial++)
//		{
//			Serial_PollRestartRx_IO(&p_motorController->Serials[iSerial]);
//		}

//			Serial_SendChar(&p_motorController->Serials[1U], 'a');
//			Blinky_Toggle(&p_motorController->BlinkyAlarm);

	}

	//High Freq, Low Priority 1 ms, Main
}

static inline void MotorController_PWM_Thread(MotorController_T * p_motorController)
{
	for(uint8_t iMotor = 0U; iMotor < CONFIG_MOTOR_CONTROLLER_MOTOR_COUNT; iMotor++)
	{
		Motor_PWM_Thread(&p_motorController->Motors[iMotor]);
	}
}

//Med Freq, High Priority 1 ms
static inline void MotorController_Timer1Ms_Thread(MotorController_T * p_motorController)
{
	for(uint8_t iMotor = 0U; iMotor < CONFIG_MOTOR_CONTROLLER_MOTOR_COUNT; iMotor++)
	{
		Motor_Timer1Ms_Thread(&p_motorController->Motors[iMotor]);
	}
}

static inline void MotorController_Serial_Thread(MotorController_T * p_motorController, uint8_t serialId)
{
//	Serial_RxData_ISR(&p_motorController->Serials[serialId]);
	Serial_TxData_ISR(&p_motorController->Serials[serialId]);
}



#endif
