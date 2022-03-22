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
    @file 	MotorController.h
    @author FireSoucery
    @brief	Facade Wrapper
    @version V0
*/
/******************************************************************************/
#include "MotorController.h"
#include "Config.h"

#include "Motor/Utility/MotAnalogUser/MotAnalogUser.h"
#include "Motor/Utility/MotAnalogMonitor/MotAnalogMonitor.h"

#include "Motor/Motor/Motor.h"
#include "Motor/Motor/Motor_User.h"

#include "Transducer/Blinky/Blinky.h"
#include "Transducer/Thermistor/Thermistor.h"

#include "Protocol/Protocol/Protocol.h"

#include "Peripheral/Serial/Serial.h"
#include "Peripheral/NvMemory/Flash/Flash.h"
#include "Peripheral/NvMemory/EEPROM/EEPROM.h"

#include "Utility/Shell/Shell.h"
#include "Utility/Timer/Timer.h"
#include "Utility/StateMachine/StateMachine.h"

#include <stdint.h>
#include <string.h>

/*

 */
void MotorController_Init(MotorController_T * p_controller)
{
	StateMachine_Init(&p_controller->StateMachine);

	//init reboot
//	Flash_Init(&p_controller->CONFIG.P_FLASH);
	EEPROM_Init_Blocking(&p_controller->Eeprom);

	if (p_controller->CONFIG.P_PARAMS_NVM != 0U)
	{
		memcpy(&p_controller->Parameters, p_controller->CONFIG.P_PARAMS_NVM, sizeof(MotorController_Params_T));
	}

	for (uint8_t iMotor = 0U; iMotor < p_controller->CONFIG.MOTOR_COUNT; iMotor++)
	{
		Motor_Init(&p_controller->CONFIG.P_MOTORS[iMotor]);
	}

	for (uint8_t iSerial = 0U; iSerial < p_controller->CONFIG.SERIAL_COUNT; iSerial++)
	{
		Serial_Init(&p_controller->CONFIG.P_SERIALS[iSerial]);
	}

	MotAnalogUser_Init(&p_controller->AnalogUser);
	MotAnalogMonitor_Init(&p_controller->AnalogMonitor);
	Blinky_Init(&p_controller->Buzzer);
	Debounce_Init(&p_controller->DIn, 5U);	//5millis
	Pin_Output_Init(&p_controller->CONFIG.PIN_COIL);
	Pin_Output_Init(&p_controller->CONFIG.PIN_METER);

	Timer_InitPeriodic(&p_controller->TimerSeconds,		1000U);
	Timer_InitPeriodic(&p_controller->TimerMillis, 		1U);
	Timer_InitPeriodic(&p_controller->TimerMillis10, 	10U);

	for (uint8_t iProtocol = 0U; iProtocol < p_controller->CONFIG.PROTOCOL_COUNT; iProtocol++)
	{
		Protocol_Init(&p_controller->CONFIG.P_PROTOCOLS[iProtocol]);
	}

	Shell_Init(&p_controller->Shell);

	p_controller->MainDirection = MOTOR_CONTROLLER_DIRECTION_FORWARD;

	//	p_controller->SignalBufferAnalogMonitor.AdcFlags 	= 0U;
	//	p_controller->SignalBufferAnalogUser.AdcFlags 		= 0U;
}

//void MotorController_LoadDefault(MotorController_T * p_controller)
//{
//	if (p_controller->CONFIG.P_PARAMS_DEFAULT != 0U)
//	{
//		memcpy(&p_controller->Parameters, p_controller->CONFIG.P_PARAMS_DEFAULT, sizeof(MotorController_Params_T));
//	}
//
//	for (uint8_t iMotor = 0U; iMotor < p_controller->CONFIG.MOTOR_COUNT; iMotor++)
//	{
//		Motor_LoadDefault(&p_controller->CONFIG.P_MOTORS[iMotor]);
//	}
//
////	for (uint8_t iProtocol = 0U; iProtocol < p_controller->CONFIG.PROTOCOL_COUNT; iProtocol++)
////	{
////		Protocol_LoadDefault(&p_controller->CONFIG.P_PROTOCOLS[iProtocol]);
////	}
////
////	Shell_LoadDefault(&p_controller->Shell);
////	MotAnalogUser_LoadDefault(&p_controller->AnalogUser);
////	MotAnalogMonitor_LoadDefault(&p_controller->AnalogMonitor);
////	Thermistor_LoadDefault(&p_controller->ThermistorPcb);
////	Thermistor_LoadDefault(&p_controller->ThermistorMosfetsHigh);
////	Thermistor_LoadDefault(&p_controller->ThermistorMosfetsLow);
//
//
////	memcpy(Protocols[0].CONFIG.P_PARAMS, &MAIN_PROTOCOL_PARAMS_DEFAULT, sizeof(Protocol_Params_T));
////
////	memcpy(MotorControllerMain.CONFIG.P_PARAMS_NVM, 			&MOTOR_CONTROLLER_PARAMS_DEFAULT, sizeof(MotorController_Params_T));
////	memcpy(MotorControllerMain.AnalogUser.CONFIG.P_PARAMS, 		&MOT_ANALOG_USER_PARAMS_DEFAULT, sizeof(MotAnalogUser_Params_T));
////	memcpy(MotorControllerMain.AnalogMonitor.CONFIG.P_PARAMS, 	&MOT_ANALOG_MONITOR_PARAMS_DEFAULT, sizeof(MotAnalogMonitor_Params_T));
////	memcpy(MotorControllerMain.Shell.CONFIG.P_PARAMS, 			&SHELL_PARAMS_DEFAULT, sizeof(Shell_Params_T));
////
////	for (uint8_t iMotor = 0U; iMotor < BOARD_MOTOR_COUNT; iMotor++)
////	{
////		memcpy(Motors[iMotor].CONFIG.P_PARAMS_NVM, &MOTOR_PARAMETERS_DEFAULT, sizeof(Motor_Params_T));
////		memcpy(Motors[iMotor].Hall.CONFIG.P_PARAMS_NVM, &HALL_DEFAULT, sizeof(Hall_Params_T));
////		memcpy(Motors[iMotor].Encoder.CONFIG.P_PARAMS, &ENCODER_DEFAULT, sizeof(Encoder_Params_T));
////
////		memcpy(Motors[iMotor].PidSpeed.CONFIG.P_PARAMS, &PID_SPEED_DEFAULT, sizeof(PID_Params_T));
////	}
//}


