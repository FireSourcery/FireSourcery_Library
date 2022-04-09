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

#include "MotAnalog/MotAnalog.h"
#include "MotAnalogUser/MotAnalogUser.h"

#include "Motor/Motor/Motor.h"
#include "Motor/Motor/Motor_User.h"

#include "Transducer/Blinky/Blinky.h"
#include "Transducer/Thermistor/Thermistor.h"
#include "Transducer/VMonitor/VMonitor.h"

#include "Protocol/Protocol/Protocol.h"

#include "Peripheral/Serial/Serial.h"
#include "Peripheral/Analog/AnalogN/AnalogN.h"
#include "Peripheral/NvMemory/Flash/Flash.h"
#include "Peripheral/NvMemory/EEPROM/EEPROM.h"

#include "Utility/Timer/Timer.h"
#include "Utility/StateMachine/StateMachine.h"
#include "Utility/Shell/Shell.h"

#include "System/MemMapBoot/MemMapBoot.h"

#include "Math/Linear/Linear_Voltage.h"
#include "Math/Linear/Linear.h"

#include <stdint.h>
#include <string.h>

typedef enum
{
	MOTOR_CONTROLLER_INPUT_MODE_ANALOG,
	MOTOR_CONTROLLER_INPUT_MODE_SERIAL,
	MOTOR_CONTROLLER_INPUT_MODE_CAN,
//	MOTOR_CONTROLLER_INPUT_MODE_CAN,
}
MotorController_InputMode_T;

typedef enum
{
	MOTOR_CONTROLLER_COAST_MODE_FLOAT,
	MOTOR_CONTROLLER_COAST_MODE_REGEN,
}
MotorController_CoastMode_T;

//typedef enum
//{
//	MOTOR_CONTROLLER_STOP_MODE_FLOAT,
//	MOTOR_CONTROLLER_STOP_MODE_GROUND,
//}
//MotorController_StopMode_T;


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
//	MOTOR_CONTROLLER_DIRECTION_NEUTRAL,
}
MotorController_Direction_T;

typedef union
{
	struct
	{
		uint32_t PcbOverHeat 			:1;
		uint32_t MosfetsTopOverHeat 	:1;
		uint32_t MosfetsBotOverHeat 	:1;
		uint32_t VPosLimit 				:1;
		uint32_t VSenseLimit 			:1;
		uint32_t VAccLimit 				:1;
	};
	uint32_t State;
}
MotorController_ErrorFlags_T;

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
//	MotorController_StopMode_T StopMode;
	MotorController_CoastMode_T CoastMode;
	bool IsBuzzerOnReverseEnable;
	uint16_t BatteryZero_ADCU;
	uint16_t BatteryFull_ADCU;
}
MotorController_Params_T;

/*
 * allocated memory outside for less CONFIG define redundancy
 */
typedef const struct
{
	const MotorController_Params_T 	* const P_PARAMS_NVM;
	const MotorController_Once_T 	* const P_ONCE;
	const MemMapBoot_T 				* const P_MEM_MAP_BOOT;

	Motor_T * const P_MOTORS;
	const uint8_t MOTOR_COUNT;

	Serial_T * const P_SERIALS; /* Simultaneous active serial */
	const uint8_t SERIAL_COUNT;

	Flash_T * const P_FLASH; 	/* Flash defined outside module, ensure flash config/params are in RAM */

	AnalogN_T * const P_ANALOG_N;
	const AnalogN_Conversion_T CONVERSION_HEAT_PCB;
	const AnalogN_Conversion_T CONVERSION_HEAT_MOSFETS_TOP;
	const AnalogN_Conversion_T CONVERSION_HEAT_MOSFETS_BOT;
	const AnalogN_Conversion_T CONVERSION_VPOS;
	const AnalogN_Conversion_T CONVERSION_VACC;
	const AnalogN_Conversion_T CONVERSION_VSENSE;
	const AnalogN_Conversion_T CONVERSION_THROTTLE;
	const AnalogN_Conversion_T CONVERSION_BRAKE;
	const AnalogN_AdcFlags_T ADCS_ACTIVE_MAIN_THREAD;

	const Pin_T PIN_METER;
	const Pin_T PIN_COIL;

	Protocol_T * const P_PROTOCOLS; /* Simultaneously active protocols */
	const uint8_t PROTOCOL_COUNT;

	/* set function use only */
	//todo move to ProtocolN module
	const Protocol_Specs_T ** const P_PROTOCOL_SPECS_TABLE;
	const uint8_t PROTOCOL_SPECS_COUNT;

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
	MotorController_ErrorFlags_T ErrorFlags;

	volatile MotAnalog_Results_T AnalogResults;
	MotAnalog_Results_T FaultAnalogRecord;

	EEPROM_T Eeprom;
	MotAnalogUser_T AnalogUser;
	MotAnalogUser_Cmd_T AnalogUserCmd;
	Debounce_T DIn; //configurable input

	Blinky_T Buzzer;

	VMonitor_T VMonitorPos; 	//Controller Supply
	VMonitor_T VMonitorSense; 	//5V
	VMonitor_T VMonitorAcc; 	//12V

	Linear_T Battery; //battery percentage

	Thermistor_T ThermistorPcb;
	Thermistor_T ThermistorMosfetsTop;
	Thermistor_T ThermistorMosfetsBot;

	Timer_T TimerMillis;
	Timer_T TimerMillis10;
	Timer_T TimerSeconds;

	Timer_T StateTimer;

	Shell_T Shell;

	MotorController_Direction_T MainDirection;
 	MotorController_Direction_T UserDirection;
 	uint16_t UserCmd;
}
MotorController_T;


static inline Motor_T * MotorController_GetPtrMotor(const MotorController_T * p_mc, uint8_t motorIndex) {return &(p_mc->CONFIG.P_MOTORS[motorIndex]);}

static inline void MotorController_DisableMotorAll(MotorController_T * p_mc)
{
	Motor_UserN_DisableControl(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT);
}

static inline void MotorController_GroundMotorAll(MotorController_T * p_mc)
{
	Motor_UserN_Ground(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT);
}


static inline void MotorController_Beep(MotorController_T * p_mc)
{
	Blinky_Blink(&p_mc->Buzzer, 500U);
}

static inline void MotorController_CalibrateBattery_MilliV(MotorController_T * p_mc, uint32_t zero_mV, uint32_t max_mV)
{
	p_mc->Parameters.BatteryZero_ADCU = VMonitor_ConvertMilliVToAdcu(&p_mc->VMonitorPos, zero_mV);
	p_mc->Parameters.BatteryFull_ADCU = VMonitor_ConvertMilliVToAdcu(&p_mc->VMonitorPos, max_mV);
	Linear_ADC_Init(&p_mc->Battery, p_mc->Parameters.BatteryZero_ADCU, p_mc->Parameters.BatteryFull_ADCU, 1000U);
}

static inline uint32_t MotorController_GetBatteryCharge_Base10(MotorController_T * p_mc)
{
	return Linear_ADC_CalcPhysical(&p_mc->Battery, p_mc->AnalogResults.VPos_ADCU);
}

static inline uint32_t MotorController_GetBatteryCharge_Frac16(MotorController_T * p_mc)
{
	return Linear_ADC_CalcFraction16(&p_mc->Battery, p_mc->AnalogResults.VPos_ADCU);
}

static inline void MotorController_SetFaultRecord(MotorController_T * p_mc)
{
	memcpy(&p_mc->FaultAnalogRecord, (void *)&p_mc->AnalogResults, sizeof(MotAnalog_Results_T));
}

/*
 * Mapped to State Machine
 */
static inline void MotorController_ProcUserCmdBrake(MotorController_T * p_mc)
{
	Motor_UserN_SetBrakeCmd(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, p_mc->UserCmd);
}

static inline void MotorController_ProcUserCmdThrottle(MotorController_T * p_mc)
{
	Motor_UserN_SetThrottleCmd(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, p_mc->UserCmd);
}

static inline void MotorController_ProcUserCmdRegen(MotorController_T * p_mc)
{
	Motor_UserN_SetRegenCmd(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT);
}

static inline bool MotorController_ProcDirection(MotorController_T * p_mc)
{
	bool isSucess;

	if(p_mc->UserDirection == MOTOR_CONTROLLER_DIRECTION_FORWARD)
	{
		isSucess = Motor_UserN_SetDirectionForward(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT);
	}
	else
	{
		isSucess = Motor_UserN_SetDirectionReverse(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT);
	}

	if(isSucess == true)
	{
		p_mc->MainDirection = p_mc->UserDirection;
	}

	return isSucess;
}

static inline bool MotorController_CheckMotorStopAll(MotorController_T * p_mc)
{
	//user stop state instead?
	return Motor_UserN_CheckStop(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT);
}

static inline bool MotorController_CheckMotorErrorFlagsAll(MotorController_T * p_mc)
{
	return Motor_UserN_CheckErrorFlags(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT);
}

static inline void MotorController_SaveParameters_Blocking(MotorController_T * p_mc)
{
	Motor_T * p_motor;
	Protocol_T * p_protocol;

#ifndef CONFIG_MOTOR_CONTROLLER_PARAMETERS_FLASH
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
		EEPROM_Write_Blocking(&p_mc->Eeprom, p_motor->Thermistor.CONFIG.P_PARAMS, 	&p_motor->Thermistor.Params, 	sizeof(Thermistor_Params_T));
	}

	EEPROM_Write_Blocking(&p_mc->Eeprom, p_mc->CONFIG.P_PARAMS_NVM, &p_mc->Parameters, sizeof(MotorController_Params_T));
	EEPROM_Write_Blocking(&p_mc->Eeprom, p_mc->CONFIG.P_MEM_MAP_BOOT, &p_mc->MemMapBoot, sizeof(MemMapBoot_T));
	for (uint8_t iProtocol = 0U; iProtocol < p_mc->CONFIG.PROTOCOL_COUNT; iProtocol++)
	{
		p_protocol = &p_mc->CONFIG.P_PROTOCOLS[iProtocol];
		EEPROM_Write_Blocking(&p_mc->Eeprom, p_protocol->CONFIG.P_PARAMS, &p_protocol->Params, sizeof(Protocol_Params_T));
	}

	EEPROM_Write_Blocking(&p_mc->Eeprom, p_mc->AnalogUser.CONFIG.P_PARAMS, 				&p_mc->AnalogUser.Params, 			sizeof(MotAnalogUser_Params_T));
	EEPROM_Write_Blocking(&p_mc->Eeprom, p_mc->ThermistorPcb.CONFIG.P_PARAMS, 			&p_mc->ThermistorPcb.Params, 		sizeof(Thermistor_Params_T));
	EEPROM_Write_Blocking(&p_mc->Eeprom, p_mc->ThermistorMosfetsTop.CONFIG.P_PARAMS, 	&p_mc->ThermistorMosfetsTop.Params, sizeof(Thermistor_Params_T));
	EEPROM_Write_Blocking(&p_mc->Eeprom, p_mc->ThermistorMosfetsBot.CONFIG.P_PARAMS, 	&p_mc->ThermistorMosfetsBot.Params, sizeof(Thermistor_Params_T));
	//vmonitor
	EEPROM_Write_Blocking(&p_mc->Eeprom, p_mc->Shell.CONFIG.P_PARAMS, &p_mc->Shell.Params, sizeof(Shell_Params_T));

#endif
}

static inline void MotorController_WriteSerialNumber_Blocking(MotorController_T * p_mc, uint32_t serialNumber)
{
	Flash_WriteOnce_Blocking(p_mc->CONFIG.P_FLASH, p_mc->CONFIG.P_ONCE, &serialNumber, 4U);
}

extern void MotorController_Init(MotorController_T * p_controller);

#endif
