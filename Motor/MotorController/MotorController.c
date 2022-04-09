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

#include <string.h>

/*

 */

void MotorController_InitLoadDefaultCheck(MotorController_T * p_mc)
{

}




void MotorController_Init(MotorController_T * p_mc)
{
	StateMachine_Init(&p_mc->StateMachine);

//	Flash_Init(&p_mc->CONFIG.P_FLASH);
	EEPROM_Init_Blocking(&p_mc->Eeprom);

	if (p_mc->CONFIG.P_PARAMS_NVM != 0U)
	{
		memcpy(&p_mc->Parameters, p_mc->CONFIG.P_PARAMS_NVM, sizeof(MotorController_Params_T));
	}

	if (p_mc->CONFIG.P_MEM_MAP_BOOT != 0U)
	{
		p_mc->MemMapBoot.Register = p_mc->CONFIG.P_MEM_MAP_BOOT->Register;
	}

	for (uint8_t iMotor = 0U; iMotor < p_mc->CONFIG.MOTOR_COUNT; iMotor++)
	{
		Motor_Init(&p_mc->CONFIG.P_MOTORS[iMotor]);
	}

	for (uint8_t iSerial = 0U; iSerial < p_mc->CONFIG.SERIAL_COUNT; iSerial++)
	{
		Serial_Init(&p_mc->CONFIG.P_SERIALS[iSerial]);
	}

	MotAnalogUser_Init(&p_mc->AnalogUser);
	Debounce_Init(&p_mc->DIn, 5U);	//5millis

	Thermistor_Init(&p_mc->ThermistorPcb);
	Thermistor_Init(&p_mc->ThermistorMosfetsTop);
	Thermistor_Init(&p_mc->ThermistorMosfetsBot);

	VMonitor_Init(&p_mc->VMonitorPos);
	VMonitor_Init(&p_mc->VMonitorSense);
	VMonitor_Init(&p_mc->VMonitorAcc);

	/* set values to not enter fault state */
	p_mc->AnalogResults.HeatPcb_ADCU = p_mc->ThermistorPcb.Params.Threshold_ADCU;
	p_mc->AnalogResults.HeatMosfetsTop_ADCU = p_mc->ThermistorMosfetsTop.Params.Threshold_ADCU;
	p_mc->AnalogResults.HeatMosfetsBot_ADCU = p_mc->ThermistorMosfetsBot.Params.Threshold_ADCU;

	p_mc->AnalogResults.VPos_ADCU 		= p_mc->VMonitorPos.Params.LimitLower_ADCU + 1U;
	p_mc->AnalogResults.VSense_ADCU 	= p_mc->VMonitorSense.Params.LimitLower_ADCU + 1U;
	p_mc->AnalogResults.VAcc_ADCU 		= p_mc->VMonitorAcc.Params.LimitLower_ADCU + 1U;

	Linear_ADC_Init(&p_mc->Battery, p_mc->Parameters.BatteryZero_ADCU, p_mc->Parameters.BatteryFull_ADCU, 1000U);

	Blinky_Init(&p_mc->Buzzer);


	Pin_Output_Init(&p_mc->CONFIG.PIN_COIL);
	Pin_Output_Init(&p_mc->CONFIG.PIN_METER);

	Timer_InitPeriodic(&p_mc->TimerSeconds,		1000U);
	Timer_InitPeriodic(&p_mc->TimerMillis, 		1U);
	Timer_InitPeriodic(&p_mc->TimerMillis10, 	10U);
	Timer_InitPeriodic(&p_mc->StateTimer,		1000U);

	/* todo invalid xcvr set issues */
	for (uint8_t iProtocol = 0U; iProtocol < p_mc->CONFIG.PROTOCOL_COUNT; iProtocol++)
	{
		Protocol_Init(&p_mc->CONFIG.P_PROTOCOLS[iProtocol]);
	}

	Shell_Init(&p_mc->Shell);

	p_mc->MainDirection = MOTOR_CONTROLLER_DIRECTION_FORWARD;
	p_mc->UserDirection = MOTOR_CONTROLLER_DIRECTION_FORWARD;
	//	p_mc->SignalBufferAnalogMonitor.AdcFlags 	= 0U;
	//	p_mc->SignalBufferAnalogUser.AdcFlags 		= 0U;
}


