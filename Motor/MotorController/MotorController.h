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

#include "Motor/Motor/Motor.h"
#include "Motor/Motor/Motor_User.h"

#include "Transducer/Blinky/Blinky.h"
#include "Transducer/Thermistor/Thermistor.h"

#include "Protocol/Protocol/Protocol.h"

#include "Peripheral/Serial/Serial.h"
#include "Peripheral/Analog/AnalogN/AnalogN.h"
#include "Peripheral/NvMemory/Flash/Flash.h"
#include "Peripheral/NvMemory/EEPROM/EEPROM.h"

#include "Utility/Timer/Timer.h"
#include "Utility/StateMachine/StateMachine.h"
#include "Utility/Shell/Shell.h"

#include "System/MemMapBoot/MemMapBoot.h"

#include <stdint.h>

typedef enum
{
	MOTOR_CONTROLLER_INPUT_MODE_ANALOG,
	MOTOR_CONTROLLER_INPUT_MODE_SERIAL,
	MOTOR_CONTROLLER_INPUT_MODE_CAN,
}
MotorController_InputMode_T;

//typedef enum
//{
//	MOTOR_CONTROLLER_BRAKE_MODE_PASSIVE,
//	MOTOR_CONTROLLER_BRAKE_MODE_CONSTANT,
//	MOTOR_CONTROLLER_BRAKE_MODE_PROPRTIONAL,
//	MOTOR_CONTROLLER_BRAKE_MODE_SCALAR,
//}
//MotorController_BrakeMode_T;

typedef enum
{
	MOTOR_CONTROLLER_DIRECTION_FORWARD,
	MOTOR_CONTROLLER_DIRECTION_REVERSE,
}
MotorController_Direction_T;

typedef const struct __attribute__((aligned (FLASH_UNIT_WRITE_ONCE_SIZE)))
{
	const uint8_t NAME[8U];
	const uint8_t NAME_EXT[8U];
	const uint8_t MANUFACTURE_DAY;
	const uint8_t MANUFACTURE_MONTH;
	const uint8_t MANUFACTURE_YEAR;
	const uint8_t MANUFACTURE_RESV;
	const uint32_t SERIAL_NUMBER;
}
MotorController_Once_T;

typedef struct __attribute__((aligned (4U))) //CONFIG_PARAMS_ALIGN_SIZE
{
	MotorController_InputMode_T InputMode;
	bool IsBuzzerOnReverseEnable;
}
MotorController_Params_T;

/*
 * allocated memory outside for less CONFIG define redundancy
 */
typedef const struct
{
	const MotorController_Params_T 	* const P_PARAMS_NVM;
	const MotorController_Params_T 	* const P_PARAMS_DEFAULT;
	const MotorController_Once_T 	* const P_ONCE;
	const MemMapBoot_T 				* const P_MEM_MAP_BOOT;

	Motor_T * const P_MOTORS;
	const uint8_t MOTOR_COUNT;

	Serial_T * const P_SERIALS; //simultaneous active serial
	const uint8_t SERIAL_COUNT;

	Flash_T * const P_FLASH; 	/* ensure flash config/params are in RAM */

	AnalogN_T * const P_ANALOG_N;
//	const AnalogN_Conversion_T CONVERSION_ANALOG_USER;
//	const AnalogN_Conversion_T CONVERSION_ANALOG_MONITOR;
	const AnalogN_Conversion_T CONVERSION_HEAT_PCB;
	const AnalogN_Conversion_T CONVERSION_HEAT_MOSFETS_TOP;
	const AnalogN_Conversion_T CONVERSION_HEAT_MOSFETS_BOT;
	const AnalogN_Conversion_T CONVERSION_VACC;
	const AnalogN_Conversion_T CONVERSION_VSENSE;
	const AnalogN_Conversion_T CONVERSION_THROTTLE;
	const AnalogN_Conversion_T CONVERSION_BRAKE;
	const AnalogN_AdcFlags_T ADCS_ACTIVE_MAIN_THREAD;

	Protocol_T * const P_PROTOCOLS; //Simultaneously active protocols
	const uint8_t PROTOCOL_COUNT;

	/* set function use only */
	//todo move to ProtocolN module
	const Protocol_Specs_T ** const P_PROTOCOL_SPECS_TABLE;
	const uint8_t PROTOCOL_SPECS_COUNT;

	const Pin_T PIN_METER;
	const Pin_T PIN_COIL;

	const Linear_T UNIT_VSENSE;
	const Linear_T UNIT_VACC;

	const uint8_t SOFTWARE_VERSION[4U];
	const uint16_t CONTROLLER_VOLTAGE;
	const uint16_t SENSOR_RATED_I;
//	const uint16_t SENSOR_RATED_AD;
}
MotorController_Config_T;

/*   */
typedef struct
{
	const MotorController_Config_T CONFIG;
	MotorController_Params_T Parameters; //ram copy
	MemMapBoot_T MemMapBoot; //temp buffer

	StateMachine_T StateMachine;
	volatile MotAnalog_Results_T AnalogResults;
// 	AnalogN_AdcFlags_T SignalBufferAnalogMonitor;
// 	AnalogN_AdcFlags_T SignalBufferAnalogUser;

//	Flash_T Flash;
	EEPROM_T Eeprom;
	MotAnalogUser_T AnalogUser;
	MotAnalogMonitor_T AnalogMonitor;
	Debounce_T DIn; //configurable input

	Blinky_T Buzzer;

	Thermistor_T ThermistorPcb;
	Thermistor_T ThermistorMosfetsTop;
	Thermistor_T ThermistorMosfetsBot;

	Timer_T TimerMillis;
	Timer_T TimerMillis10;
	Timer_T TimerSeconds;

	Shell_T Shell;
 	MotorController_Direction_T MainDirection;

 	uint16_t UserCmd;
 	MotorController_Direction_T UserDirection;
}
MotorController_T;


static inline Motor_T * MotorController_GetPtrMotor(const MotorController_T * p_mc, uint8_t motorIndex) {return &(p_mc->CONFIG.P_MOTORS[motorIndex]);}

static inline void MotorController_SaveParameters_Blocking(MotorController_T * p_mc)
{
	Motor_T * p_motor;
	Protocol_T * p_protocol;

	for(uint8_t iMotor = 0U; iMotor < p_mc->CONFIG.MOTOR_COUNT; iMotor++)
	{
		p_motor = MotorController_GetPtrMotor(p_mc, iMotor);
		EEPROM_Write_Blocking(&p_mc->Eeprom, p_motor->CONFIG.P_PARAMS_NVM, 			&p_motor->Parameters, 			sizeof(Motor_Params_T));
		EEPROM_Write_Blocking(&p_mc->Eeprom, p_motor->Hall.CONFIG.P_PARAMS_NVM, 	&p_motor->Hall.Params, 			sizeof(Hall_Params_T));
		EEPROM_Write_Blocking(&p_mc->Eeprom, p_motor->Encoder.CONFIG.P_PARAMS, 		&p_motor->Encoder.Params, 		sizeof(Encoder_Params_T));
		EEPROM_Write_Blocking(&p_mc->Eeprom, p_motor->PidSpeed.CONFIG.P_PARAMS, 	&p_motor->PidSpeed.Params, 		sizeof(PID_Params_T));
		EEPROM_Write_Blocking(&p_mc->Eeprom, p_motor->PidIq.CONFIG.P_PARAMS, 		&p_motor->PidIq.Params, 		sizeof(PID_Params_T));
		EEPROM_Write_Blocking(&p_mc->Eeprom, p_motor->PidId.CONFIG.P_PARAMS, 		&p_motor->PidId.Params, 		sizeof(PID_Params_T));
		EEPROM_Write_Blocking(&p_mc->Eeprom, p_motor->PidIBus.CONFIG.P_PARAMS, 		&p_motor->PidIBus.Params, 		sizeof(PID_Params_T));
		//	EEPROM_Write_Blocking(&p_mc->Eeprom, p_motor->Thermistor .CONFIG.P_PARAMS, &p_motor->Thermistor .Params, sizeof(Thermistor_Params_T));
	}

	EEPROM_Write_Blocking(&p_mc->Eeprom, p_mc->CONFIG.P_PARAMS_NVM, &p_mc->Parameters, sizeof(MotorController_Params_T));

	for (uint8_t iProtocol = 0U; iProtocol < p_mc->CONFIG.PROTOCOL_COUNT; iProtocol++)
	{
		p_protocol = &p_mc->CONFIG.P_PROTOCOLS[iProtocol];
		EEPROM_Write_Blocking(&p_mc->Eeprom, p_protocol->CONFIG.P_PARAMS, &p_protocol->Params, sizeof(Protocol_Params_T));
	}

	EEPROM_Write_Blocking(&p_mc->Eeprom, p_mc->Shell.CONFIG.P_PARAMS, 			&p_mc->Shell.Params, 			sizeof(Shell_Params_T));
	EEPROM_Write_Blocking(&p_mc->Eeprom, p_mc->AnalogUser.CONFIG.P_PARAMS, 		&p_mc->AnalogUser.Params, 		sizeof(MotAnalogUser_Params_T));
	EEPROM_Write_Blocking(&p_mc->Eeprom, p_mc->AnalogMonitor.CONFIG.P_PARAMS, 	&p_mc->AnalogMonitor.Params, 	sizeof(MotAnalogMonitor_Params_T));
	//	EEPROM_Write_Blocking(&p_mc->Eeprom, p_mc->ThermistorPcb.CONFIG.P_PARAMS, &p_mc->ThermistorPcb.Params, sizeof(Thermistor_Params_T));
	//	EEPROM_Write_Blocking(&p_mc->Eeprom, p_mc->ThermistorMosfetsTop.CONFIG.P_PARAMS, &p_mc->ThermistorMosfetsTop.Params, sizeof(Thermistor_Params_T));
	//	EEPROM_Write_Blocking(&p_mc->Eeprom, p_mc->ThermistorMosfetsBot.CONFIG.P_PARAMS, &p_mc->ThermistorMosfetsBot.Params, sizeof(Thermistor_Params_T));
	EEPROM_Write_Blocking(&p_mc->Eeprom, p_mc->CONFIG.P_MEM_MAP_BOOT, &p_mc->MemMapBoot, sizeof(MemMapBoot_T));
}

static inline void MotorController_Beep(MotorController_T * p_mc)
{
	Blinky_Blink(&p_mc->Buzzer, 500U);
}

/*
 * Wrappers for State Machine
 */
//static inline void MotorController_ProcAlarm(MotorController_T * p_mc)
//{
//	Blinky_Blink(&p_mc->Buzzer, 500U); //set alarm type
//}

static inline void MotorController_ProcUserCmdBrake(MotorController_T * p_mc)
{
	Motor_UserN_SetCmdBrake(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, p_mc->UserCmd);
}

static inline void MotorController_ProcUserCmdThrottle(MotorController_T * p_mc)
{
	Motor_UserN_SetCmd(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, p_mc->UserCmd);
}

static inline bool MotorController_ProcDirection(MotorController_T * p_mc)
{
	bool status;

	if(p_mc->UserDirection == MOTOR_CONTROLLER_DIRECTION_FORWARD)
	{
		status = Motor_UserN_SetDirectionForward(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT);
	}
	else
	{
		status = Motor_UserN_SetDirectionReverse(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT);
	}

	if(status == true)
	{
		p_mc->MainDirection = p_mc->UserDirection;
	}
//	else
//	{
////		  //todo set error state, direction partial sett
//	}


	return status;
}

static inline void MotorController_DisableMotorAll(MotorController_T * p_mc)
{
	Motor_UserN_DisableControl(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT);
}

static inline bool MotorController_CheckMotorAllStop(MotorController_T * p_mc)
{
	return Motor_UserN_CheckStop(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT);
}

extern void MotorController_Init(MotorController_T * p_controller);

#endif
