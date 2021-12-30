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
#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "Config.h"

#include "Motor/Utility/MotAnalog/MotAnalog.h"
#include "Motor/Utility/MotAnalogUser/MotAnalogUser.h"
#include "Motor/Utility/MotAnalogMonitor/MotAnalogMonitor.h"
//#include "Motor/Utility/MotProtocol/MotProtocol.h"
//#include "Motor/Utility/MotShell/MotShell.h"

#include "Motor/Motor/Motor.h"
#include "Motor/Motor/Motor_User.h"

#include "Transducer/Blinky/Blinky.h"
#include "Transducer/Thermistor/Thermistor.h"

#include "Protocol/Protocol/Protocol.h"

#include "Peripheral/Serial/Serial.h"
#include "Peripheral/Analog/AnalogN/AnalogN.h"
#include "Peripheral/NvMemory/Flash/Flash.h"
#include "Peripheral/NvMemory/EEPROM/EEPROM.h"

//#include "Utility/Shell/Shell.h"
#include "Utility/Thread/Thread.h"
#include "Utility/StateMachine/StateMachine.h"

#include <stdint.h>

typedef enum
{
	MOTOR_CONTROLLER_INPUT_MODE_ANALOG,
	MOTOR_CONTROLLER_INPUT_MODE_SERIAL,
	MOTOR_CONTROLLER_INPUT_MODE_CAN,
}
MotorController_InputMode_T;

typedef enum
{
	MOTOR_CONTROLLER_DIRECTION_FORWARD,
	MOTOR_CONTROLLER_DIRECTION_REVERSE,
}
MotorController_Direction_T;


typedef struct __attribute__((aligned (4U)))
{
	//	const uint8_t application_config;
	//	const uint8_t startup_wait_time;
//	uint8_t ShellConnectId;
//	uint8_t ProtocolDataLinkId[1]; //per protocol
//	uint8_t AuxProtocolSpecsId[CONFIG_MOTOR_CONTROLLER_AUX_PROTOCOL_COUNT];
	uint8_t MainProtocolSpecsId;
	MotorController_InputMode_T InputMode;
	bool IsBuzzerOnReverseEnable;
}
MotorController_Params_T;

/*
 * allocated memory outside for less CONFIG define redundancy
 */
typedef const struct
{
	const MotorController_Params_T * const P_PARAMETERS; //nvm copy

	Motor_T * const P_MOTORS;
	const uint8_t MOTOR_COUNT;

	Serial_T * const P_SERIALS; //simultaneous active serial
	const uint8_t SERIAL_COUNT;

	AnalogN_T * const P_ANALOG_N;
	const AnalogN_Conversion_T CONVERSION_ANALOG_USER;
	const AnalogN_Conversion_T CONVERSION_ANALOG_MONITOR;

	//	Protocol_T * const P_AUX_PROTOCOLS; 	//Simultaneously active protocols
	//	const uint8_t AUX_PROTOCOL_COUNT;

	const Protocol_Specs_T ** const P_PROTOCOL_SPECS_TABLE;
	const uint8_t PROTOCOL_SPECS_COUNT;

	const Pin_T PIN_METER;
	const Pin_T PIN_COIL;
}
MotorController_Config_T;

/*   */
typedef struct
{
	const MotorController_Config_T CONFIG;
	MotorController_Params_T Parameters; //ram copy
	StateMachine_T StateMachine;

	Flash_T Flash;
	EEPROM_T Eeprom;
	MotAnalogUser_T AnalogUser;
	MotAnalogMonitor_T AnalogMonitor;
	Blinky_T Buzzer;
	Debounce_T DIn; //configurable input
	Thermistor_T ThermistorPcb;
	Thermistor_T ThermistorMosfets;

	Timer_T TimerMillis;
	Timer_T TimerMillis10;
	Timer_T TimerSeconds;

	MotAnalog_Map_T AnalogResults;
 	AnalogN_AdcFlags_T SignalBufferAnalogMonitor;
 	AnalogN_AdcFlags_T SignalBufferAnalogUser;

	Protocol_T		MainProtocol;
//	Shell_T 		MotShell;

 	MotorController_Direction_T MainDirection;

 	uint16_t UserCmd;
 	MotorController_Direction_T UserDirection;
}
MotorController_T;


static inline Motor_T * MotorController_GetPtrMotor(MotorController_T * p_motorController, uint8_t motorIndex) {return &(p_motorController->CONFIG.P_MOTORS[motorIndex]);}

static inline void MotorController_ProcSaveParameters_Blocking(MotorController_T * p_motorController)
{
	for(uint8_t iMotor = 0U; iMotor < p_motorController->CONFIG.MOTOR_COUNT; iMotor++)
	{
		EEPROM_Write_Blocking(&p_motorController->Eeprom, MotorController_GetPtrMotor(p_motorController, iMotor)->CONFIG.P_PARAMETERS, &MotorController_GetPtrMotor(p_motorController, iMotor)->Parameters, sizeof(Motor_Params_T));
	}

	EEPROM_Write_Blocking(&p_motorController->Eeprom, p_motorController->AnalogMonitor.CONFIG.P_PARAMS, &p_motorController->AnalogMonitor.Params, sizeof(MotAnalogMonitor_Params_T));
//	EEPROM_Write_Blocking(&p_motorController->Eeprom, p_motorController->AnalogMonitor.CONFIG.P_PARAMS, &p_motorController->AnalogMonitor.Params, sizeof(MotAnalogMonitor_Params_T));
}

static inline void MotorController_Beep(MotorController_T * p_motorController)
{
	Blinky_Blink(&p_motorController->Buzzer, 500U);
}

/*
 * Wrappers for State Machine
 */
static inline void MotorController_ProcAlarm(MotorController_T * p_motorController)
{
//	Blinky_Blink(&p_motorController->Buzzer, 500U); set alarm type
}

static inline void MotorController_ProcUserCmdDecelerate(MotorController_T * p_motorController)
{
	Motor_UserN_SetCmdDecelerate(p_motorController->CONFIG.P_MOTORS, p_motorController->CONFIG.MOTOR_COUNT, p_motorController->UserCmd);
}

static inline void MotorController_ProcUserCmdAccelerate(MotorController_T * p_motorController)
{
	Motor_UserN_SetCmdAccelerate(p_motorController->CONFIG.P_MOTORS, p_motorController->CONFIG.MOTOR_COUNT, p_motorController->UserCmd);
}

static inline bool MotorController_ProcDirection(MotorController_T * p_motorController)
{
	bool status = true;

	p_motorController->MainDirection = p_motorController->UserDirection;

	if(p_motorController->UserDirection == MOTOR_CONTROLLER_DIRECTION_FORWARD)
	{
		for(uint8_t iMotor = 0U; iMotor < p_motorController->CONFIG.MOTOR_COUNT; iMotor++)
		{
			if(Motor_User_SetDirectionForward(&p_motorController->CONFIG.P_MOTORS[iMotor]) == false)
			{
				Blinky_Blink(&p_motorController->Buzzer, 500U);
				status = false;
			}
		}
	}
	else
	{
		for(uint8_t iMotor = 0U; iMotor < p_motorController->CONFIG.MOTOR_COUNT; iMotor++)
		{
			if(Motor_User_SetDirectionReverse(&p_motorController->CONFIG.P_MOTORS[iMotor]) == false)
			{
				Blinky_Blink(&p_motorController->Buzzer, 500U);
				status = false;
			}

			Motor_User_ActivateCalibrationHall(&p_motorController->CONFIG.P_MOTORS[iMotor]);
		}
	}

	return status;
}

static inline void MotorController_DisableMotorAll(MotorController_T * p_motorController)
{
//	for (uint8_t iMotor = 0U; iMotor < p_motorController->CONFIG.MOTOR_COUNT; iMotor++)
//	{
//		Motor_User_Disable(&p_motorController->CONFIG.P_MOTORS[iMotor]);
//	}

	Motor_UserN_DisableControl(p_motorController->CONFIG.P_MOTORS, p_motorController->CONFIG.MOTOR_COUNT);
}

static inline bool MotorController_CheckMotorAllStop(MotorController_T * p_motorController)
{
	return Motor_UserN_CheckStop(p_motorController->CONFIG.P_MOTORS, p_motorController->CONFIG.MOTOR_COUNT);
}

extern void MotorController_Init(MotorController_T * p_controller);

#endif
