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
#include "MotorControllerAnalog.h"
#include "MotAnalogUser/MotAnalogUser.h"

// #include "Motor/Motor/Motor.h"
#include "Motor/Motor/Motor_User.h"

#include "Transducer/Blinky/Blinky.h"
#include "Transducer/Thermistor/Thermistor.h"
#include "Transducer/VMonitor/VMonitor.h"

#include "Peripheral/Analog/AnalogN/AnalogN.h"
#include "Peripheral/NvMemory/Flash/Flash.h"
#include "Peripheral/NvMemory/EEPROM/EEPROM.h"
#include "Peripheral/Serial/Serial.h"
#include "Peripheral/CanBus/CanBus.h"

#include "Utility/Timer/Timer.h"
#include "Utility/StateMachine/StateMachine.h"
#include "Utility/Protocol/Protocol.h"
#include "Utility/Shell/Shell.h"

#include "System/MemMapBoot/MemMapBoot.h"

#include "Math/Linear/Linear_Voltage.h"
#include "Math/Linear/Linear.h"

#include <stdint.h>
#include <string.h>

#define MOT_SOFTWARE_VERSION_OPT 		0U
#define MOT_SOFTWARE_VERSION_MAJOR 		0U
#define MOT_SOFTWARE_VERSION_MINOR 		0U
#define MOT_SOFTWARE_VERSION_BUGFIX 	1U
#define MOT_SOFTWARE_VERSION_ID 		((MOT_SOFTWARE_VERSION_OPT << 24U) | (MOT_SOFTWARE_VERSION_MAJOR << 16U) | (MOT_SOFTWARE_VERSION_MINOR << 8U) | (MOT_SOFTWARE_VERSION_BUGFIX))

typedef enum MotorController_InputMode_Tag
{
	MOTOR_CONTROLLER_INPUT_MODE_ANALOG,
	MOTOR_CONTROLLER_INPUT_MODE_SERIAL,
	MOTOR_CONTROLLER_INPUT_MODE_CAN,
}
MotorController_InputMode_T;

typedef enum MotorController_CoastMode_Tag
{
	MOTOR_CONTROLLER_COAST_MODE_FLOAT,
	MOTOR_CONTROLLER_COAST_MODE_REGEN,
}
MotorController_CoastMode_T;

typedef enum
{
	MOTOR_CONTROLLER_BRAKE_MODE_PASSIVE,
	MOTOR_CONTROLLER_BRAKE_MODE_TORQUE,
	MOTOR_CONTROLLER_BRAKE_MODE_VFREQ_SCALAR,
}
MotorController_BrakeMode_T;

typedef enum MotorController_Direction_Tag
{
	MOTOR_CONTROLLER_DIRECTION_PARK,
	MOTOR_CONTROLLER_DIRECTION_NEUTRAL,
	MOTOR_CONTROLLER_DIRECTION_FORWARD,
	MOTOR_CONTROLLER_DIRECTION_REVERSE,
}
MotorController_Direction_T;

typedef enum MotorController_Substate_Tag
{
	MOTOR_CONTROLLER_NVM_ALL,
	MOTOR_CONTROLLER_NVM_BOOT,
	MOTOR_CONTROLLER_CALIBRATION,
}
MotorController_Substate_T; //calibration/stop substate

/*
	Fault substate flags
*/
typedef union MotorController_FaultFlags_Tag
{
	struct
	{
		uint32_t PcbOverHeat 		: 1U;
		uint32_t MosfetsTopOverHeat : 1U;
		uint32_t MosfetsBotOverHeat : 1U;
		uint32_t VPosLimit 			: 1U;
		uint32_t VSenseLimit 		: 1U;
		uint32_t VAccLimit 			: 1U;
		uint32_t StopStateSync 		: 1U;
		// uint32_t ThrottleOnInit 	: 1U;
		uint32_t RxLost 			: 1U;
	};
	uint32_t State;
}
MotorController_FaultFlags_T;

typedef const struct __attribute__((aligned(FLASH_UNIT_WRITE_ONCE_SIZE))) MotorController_Once_Tag
{
	const uint8_t NAME[8U];
	const uint8_t NAME_EXT[8U];
	const uint8_t MANUFACTURE_DAY;
	const uint8_t MANUFACTURE_MONTH;
	const uint8_t MANUFACTURE_YEAR;
	const uint8_t MANUFACTURE_RESV;
	union
	{
		const uint32_t SERIAL_NUMBER;
		const uint8_t SERIAL_NUMBER_BYTES[4U];
	};
}
MotorController_Once_T;

typedef enum OptDinFunction_Tag
{
	MOTOR_CONTROLLER_OPT_DIN_DISABLE,
 	MOTOR_CONTROLLER_OPT_DIN_SPEED_LIMIT,
}
MotorController_OptDinFunction_T;

typedef union MotorController_BuzzerFlags_Tag
{
	struct
	{
		uint32_t BeepThrottleOnInit 	: 1U;
		uint32_t BeepOnReverse			: 1U;
	};
	uint32_t State;
}
MotorController_BuzzerFlags_T;

// typedef enum MotorController_SpeedLimitActiveId_Tag
// {
// 	MOTOR_CONTROLLER_SPEED_LIMIT_ACTIVE_DISABLE = 0U,
// 	MOTOR_CONTROLLER_SPEED_LIMIT_ACTIVE_OPT = 1U, /* From parent class */
// }
// MotorController_SpeedLimitActiveId_T;

typedef enum MotorController_ILimitActiveId_Tag
{
	MOTOR_CONTROLLER_I_LIMIT_ACTIVE_DISABLE = 0U,
	MOTOR_CONTROLLER_I_LIMIT_ACTIVE_HEAT = 1U,
	MOTOR_CONTROLLER_I_LIMIT_ACTIVE_LOW_V = 2U,
}
MotorController_ILimitActiveId_T;

typedef struct __attribute__((aligned(4U))) MotorController_Params_Tag
{
	uint16_t AdcVRef_MilliV;
	uint16_t VSupply;
	MotorController_InputMode_T InputMode;
	MotorController_BrakeMode_T BrakeMode;
	MotorController_CoastMode_T CoastMode;

	uint16_t BatteryZero_Adcu;
	uint16_t BatteryFull_Adcu;
	uint8_t CanServicesId;
	bool IsCanEnable;
	//todo
	// bool FaultThrottleOnInit;
	// bool FaultThrottleOnBrakeCmd;
	// bool FaultThrottleOnBrakeRelease;
	// bool FaultThrottleOnNeutralRelease;

	MotorController_BuzzerFlags_T BuzzerFlagsEnable; /* which options are enabled for use */

	MotorController_OptDinFunction_T OptDinFunction;
	uint16_t OptDinSpeedLimit_Frac16;

	uint16_t ILimitScalarOnLowV_Frac16;
	uint16_t ILimitScalarOnHeat_Frac16;
	//battery max current

	// bool IsFixedFreqUserOutput; /* limits conversion freq regardless of polling freq */
}
MotorController_Params_T;

/*
	allocated memory outside for less CONFIG define redundancy
*/
typedef const struct MotorController_Config_Tag
{
	const MotorController_Params_T * const P_PARAMS_NVM;
	const MotorController_Once_T * const P_ONCE;
	const MemMapBoot_T * const P_MEM_MAP_BOOT;

	Motor_T * const 	P_MOTORS;
	const uint8_t 		MOTOR_COUNT;
	Serial_T * const 	P_SERIALS; 	/* Simultaneous active serial */
	const uint8_t 		SERIAL_COUNT;
	CanBus_T * const 	P_CAN_BUS;
	Flash_T * const 	P_FLASH; 	/* Flash defined outside module, ensure flash config/params are in RAM */
	EEPROM_T * const 	P_EEPROM;	/* defined outside for regularity */

	AnalogN_T * const P_ANALOG_N;
	const MotAnalog_Conversions_T ANALOG_CONVERSIONS;

	Protocol_T * const P_PROTOCOLS; /* Simultaneously active protocols */
	const uint8_t PROTOCOL_COUNT;

	const uint16_t V_MAX;
	const uint16_t I_MAX;

	const uint16_t ADC_VREF_MAX_MILLIV;
	const uint16_t ADC_VREF_MIN_MILLIV;

	const uint8_t SOFTWARE_VERSION[4U];
}
MotorController_Config_T;

/*   */
typedef struct MotorController_Tag
{
	const MotorController_Config_T CONFIG;
	MotorController_Params_T Parameters; //ram copy
	MemMapBoot_T MemMapBoot; //temp buffer

	volatile MotAnalog_Results_T AnalogResults;

	MotAnalogUser_T AnalogUser;

	Blinky_T Buzzer;
	MotorController_BuzzerFlags_T BuzzerFlagsActive; /* active conditions requesting buzzer */

	Debounce_T OptDin; 	/* Configurable input */
	Pin_T Meter;
	Pin_T Relay;

	Thermistor_T ThermistorPcb;
	Thermistor_T ThermistorMosfetsTop;
	Thermistor_T ThermistorMosfetsBot;
	VMonitor_T VMonitorPos; 	//Controller Supply
	VMonitor_T VMonitorSense; 	//5V
	VMonitor_T VMonitorAcc; 	//12V
	Linear_T BatteryLife; 		//Battery Life percentage

	Timer_T TimerMillis;
	Timer_T TimerMillis10;
	Timer_T TimerSeconds;
	Timer_T TimerIsrDividerSeconds;
	Timer_T TimerState;

	Shell_T Shell;
	uint16_t ShellSubstate;

	StateMachine_T StateMachine;
	volatile MotAnalog_Results_T FaultAnalogRecord;
	volatile MotorController_FaultFlags_T FaultFlags; /* Fault Substate*/
	MotorController_Direction_T MainDirection;
	MotorController_Direction_T UserDirection;
	MotorController_Substate_T StopSubstate;
	NvMemory_Status_T NvmStatus;

	uint16_t UserCmd;

	// MotorController_SpeedLimitActiveId_T SpeedLimitActiveId;
	// MotorController_ILimitActiveId_T ILimitActiveId;
}
MotorController_T; // Motor_Ctrlr_T;

static inline Motor_T * MotorController_GetPtrMotor(const MotorController_T * p_mc, uint8_t motorIndex) { return &(p_mc->CONFIG.P_MOTORS[motorIndex]); }

static inline void MotorController_BeepShort(MotorController_T * p_mc) { Blinky_Blink(&p_mc->Buzzer, 500U); }

static inline void MotorController_BeepPeriodicType1(MotorController_T * p_mc)
{
	Blinky_StartPeriodic(&p_mc->Buzzer, 500U, 500U);
}

// static inline void MotorController_BeepType1(MotorController_T * p_mc)
// {
// 	Blinky_Blink(&p_mc->Buzzer, Type1OnTime, Type1OffTime);
// }

/******************************************************************************/
/*
	On check by StateMachine
*/
/******************************************************************************/

/*
	Assume edge type input. todo inputs into staemachine
*/
static inline bool MotorController_ProcDirection(MotorController_T * p_mc)
{
	bool isSucess;

	switch(p_mc->UserDirection)
	{
		case MOTOR_CONTROLLER_DIRECTION_PARK: 		isSucess = true; break;
		case MOTOR_CONTROLLER_DIRECTION_NEUTRAL: 	isSucess = true; break;
		case MOTOR_CONTROLLER_DIRECTION_FORWARD: 	isSucess = Motor_UserN_SetDirectionForward(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT); break;
		case MOTOR_CONTROLLER_DIRECTION_REVERSE: 	isSucess = Motor_UserN_SetDirectionReverse(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT); break;
		default: isSucess = true; break;
	}

	if(isSucess == true) { p_mc->MainDirection = p_mc->UserDirection; }

	// if((p_mc->Parameters.BuzzerFlagsEnable.BeepOnReverse == true) && isSucess)
	// {
	// 	if(p_mc->MainDirection == MOTOR_CONTROLLER_DIRECTION_REVERSE)
	// 	{
	// 		MotorController_BeepPeriodicType1(p_mc);
	// 	}
	// 	else
	// 	{
	// 		Blinky_Stop(&p_mc->Buzzer);
	// 	}
	// }

	return isSucess;
}


static inline void MotorController_ProcUserCmdBrake(MotorController_T * p_mc)
{
	if(p_mc->Parameters.BrakeMode == MOTOR_CONTROLLER_BRAKE_MODE_TORQUE)
	{
		Motor_UserN_SetBrakeCmd(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, p_mc->UserCmd);
	}
	else if(p_mc->Parameters.BrakeMode == MOTOR_CONTROLLER_BRAKE_MODE_VFREQ_SCALAR)
	{
		Motor_UserN_SetRegenCmd(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, p_mc->UserCmd);
	}
}

static inline bool MotorController_ProcMotorDirectionAll(MotorController_T * p_mc)
{
	bool isSucess;

	if(p_mc->UserDirection == MOTOR_CONTROLLER_DIRECTION_FORWARD)
	{
		isSucess = Motor_UserN_SetDirectionForward(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT);
	}
	else if(p_mc->UserDirection == MOTOR_CONTROLLER_DIRECTION_REVERSE)
	{
		isSucess = Motor_UserN_SetDirectionReverse(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT);
	}

	if(isSucess == true) { p_mc->MainDirection = p_mc->UserDirection; }

	return isSucess;
}

static inline void MotorController_DisableMotorAll(MotorController_T * p_mc) 			{ Motor_UserN_DisableControl(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT); }
static inline void MotorController_GroundMotorAll(MotorController_T * p_mc) 			{ Motor_UserN_Ground(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT); }
static inline void MotorController_ProcUserCmdThrottle(MotorController_T * p_mc) 		{ Motor_UserN_SetThrottleCmd(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, p_mc->UserCmd); }
// static inline void MotorController_ProcUserCmdVoltageBrake(MotorController_T * p_mc) 	{ Motor_UserN_SetVoltageBrakeCmd(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT); }
static inline bool MotorController_CheckMotorStopAll(MotorController_T * p_mc) 			{ return Motor_UserN_CheckStop(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT); } /* Checks 0 speed. alternatively check stop state. */
static inline bool MotorController_CheckMotorFaultAll(MotorController_T * p_mc) 		{ return Motor_UserN_CheckFault(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT); }

static inline void MotorController_SetMotorSpeedLimitAll(MotorController_T * p_mc, uint16_t limit_frac16) 	{ Motor_UserN_SetSpeedLimitActive(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, limit_frac16); }
static inline void MotorController_ClearMotorSpeedLimitAll(MotorController_T * p_mc) 						{ Motor_UserN_ClearSpeedLimit(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT); }

static inline void MotorController_SetMotorILimitAll(MotorController_T * p_mc, uint16_t limit_frac16, MotorController_ILimitActiveId_T id)
{
	Motor_UserN_SetILimitActive(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, limit_frac16, (id + MOTOR_I_LIMIT_ACTIVE_SYSTEM));
}

static inline void MotorController_ClearMotorILimitAll(MotorController_T * p_mc, MotorController_ILimitActiveId_T id)
{
	Motor_UserN_ClearILimit(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, (id + MOTOR_I_LIMIT_ACTIVE_SYSTEM));
}

/*
	Extern
*/
extern void MotorController_Init(MotorController_T * p_controller);
extern bool MotorController_SaveParameters_Blocking(MotorController_T * p_mc);
extern void MotorController_SaveBootReg_Blocking(MotorController_T * p_mc);

#endif
