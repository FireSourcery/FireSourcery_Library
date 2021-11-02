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
    @file 	MotorController.h
    @author FireSoucery
    @brief
    @version V0
*/
/**************************************************************************/
#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "Config.h"
//#include "MotorController_Params.h"

#include "Motor/Utility/MotShell/MotShell.h"
#include "Motor/Utility/MotProtocol/MotProtocol.h"
#include "Motor/Utility/MotAnalogUser/MotAnalogUser.h"

#include "Motor/Motor/Motor.h"
#include "Motor/Motor/HAL_Motor.h"

#include "Transducer/Blinky/Blinky.h"

#include "System/Protocol/Protocol/Protocol.h"

#include "Peripheral/Serial/Serial.h"
#include "Peripheral/Flash/Flash.h"
#include "Peripheral/EEPROM/EEPROM.h"

#include "System/Shell/Shell.h"
#include "System/Thread/Thread.h"

#include <stdint.h>

#ifdef CONFIG_MOTOR_ADC_8
	typedef uint16_t adc_t;
#elif defined(CONFIG_MOTOR_ADC_16)
	typedef uint16_t adc_t;
#endif


	typedef enum
	{
		MOTOR_INPUT_MODE_ANALOG,
		MOTOR_INPUT_MODE_SERIAL,
		MOTOR_INPUT_MODE_CAN,
	}
	MotorController_InputMode_T;

//typedef enum
//{
//	MOTOR_CONTROLLER_INPUT_MODE_ANALOG,
//} MotorController_InputMode_T;

//todo replace with user options
	typedef  __attribute__ ((aligned (4U))) const volatile struct
	{
		const uint8_t SHELL_CONNECT_ID;

		uint8_t ShellConnectId;
	//	uint8_t ProtocolDataLinkId[1]; //per protocol
		uint8_t AuxProtocolSpecsId[CONFIG_MOTOR_CONTROLLER_AUX_PROTOCOL_COUNT];

		uint8_t MotProtocolSpecsId;

		bool AnalogUserEnable;
	}
	MotorController_Parameters_T;



typedef const struct
{
	const HAL_Pin_T HAL_PIN_ALARM;
	////		.HAL_PIN_METER 		= {.P_GPIO_BASE = MTR_OUT_PORT,		.GPIO_PIN_MASK = (uint32_t)1U << MTR_OUT_PIN,	},
	////		.HAL_PIN_COIL	 	= {.P_GPIO_BASE = COIL_OUT_PORT,	.GPIO_PIN_MASK = (uint32_t)1U << COIL_OUT_PIN,	},
	////		.HAL_PIN_AUX1	 	= {.P_GPIO_BASE = ECO_IN_PORT,		.GPIO_PIN_MASK = (uint32_t)1U << ECO_IN_PIN,	},

	//	const HAL_Flash_T * const P_HAL_FLASH;

	const Motor_Constants_T 			MOTOR_INITS[CONFIG_MOTOR_CONTROLLER_MOTOR_COUNT];
	const Protocol_Specs_T * const 	P_PROTOCOL_SPECS[1];	//protocols available, not necessary simultaneous
	const Protocol_Config_T 			AUX_PROTOCOL_CONFIG[CONFIG_MOTOR_CONTROLLER_AUX_PROTOCOL_COUNT]; 	//Simultaneously active protocols
	const Protocol_Config_T 			MOT_PROTOCOL_CONFIG;
	const MotProtocol_Config_T			MOT_PROTOCOL_CONFIG_REGISTERS;
	const MotShell_Config_T 			MOT_SHELL_CONFIG;
	const MotAnalogUser_Config_T 		MOT_ANALOG_USER_CONFIG;

	volatile const uint32_t * const 	P_MILLIS_TIMER;
	//todo alarm, thermistor

	const MotorController_Parameters_T * const P_PARAMS;
}
MotorController_Config_T;

/* Compile time preprocessor define array allocation, only 1 instance of MotorController_T is expected. */
typedef struct
{
	const MotorController_Config_T * p_Config;
	MotorController_Parameters_T Parameters;

	Motor_T Motors[CONFIG_MOTOR_CONTROLLER_MOTOR_COUNT];
	Serial_T Serials[CONFIG_MOTOR_CONTROLLER_SERIAL_COUNT]; //simultaneous active serial


	Flash_T Flash;
	EEPROM_T Eeprom;

	//	void * p_Coms[]; 			//list of all datalink structs id by index
	Protocol_T				AuxProtocols[CONFIG_MOTOR_CONTROLLER_AUX_PROTOCOL_COUNT]; 	//Simultaneously active protocols

	Protocol_T				MotProtocol;
	MotProtocol_Output_T 	MotProtocolOutput;
	MotProtocol_Input_T 	MotProtocolInput;

	Shell_T MotShell;

	MotAnalogUser_T AnalogUser;
	Blinky_T BlinkyAlarm;


	Thread_T TimerSeconds;
	Thread_T TimerMillis;
	Thread_T TimerMillis10;

	volatile MotorController_InputMode_T InputMode; //arbitrate input overwrite
}
MotorController_T;

//priority brake thread
//static inline void MotorUser_PollBrake(MotorUser_T * p_motorUser, Motor_T * p_motor)
//{
//	Debounce_CaptureState_IO(p_motorUser->MotorUserAnalog.PinBrake);
//	p_motorUser->InputSwitchBrake 		= Debounce_GetState(p_motorUser->MotorUserAnalog.PinBrake);
//
//	if (p_motorUser->InputSwitchBrake == true)
//	{
//			Motor_SetUserCmd(p_motor, (p_motorUser->InputValueBrake + p_motorUser->InputValueBrake) / 2U);
//			Motor_SetRampDecelerate(p_motor, 0);
//	}
//}

#endif
