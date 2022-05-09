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
	@brief
	@version V0
*/
/******************************************************************************/
#include "MotorController.h"
#include <string.h>

void MotorController_Init(MotorController_T * p_mc)
{
	Flash_Init(p_mc->CONFIG.P_FLASH);
	EEPROM_Init_Blocking(p_mc->CONFIG.P_EEPROM);

	if(p_mc->CONFIG.P_PARAMS_NVM != 0U) 	{ memcpy(&p_mc->Parameters, p_mc->CONFIG.P_PARAMS_NVM, sizeof(MotorController_Params_T)); }
	if(p_mc->CONFIG.P_MEM_MAP_BOOT != 0U) 	{ p_mc->MemMapBoot.Register = p_mc->CONFIG.P_MEM_MAP_BOOT->Register; }

	AnalogN_Init(p_mc->CONFIG.P_ANALOG_N);

	for(uint8_t iSerial = 0U; iSerial < p_mc->CONFIG.SERIAL_COUNT; iSerial++) { Serial_Init(&p_mc->CONFIG.P_SERIALS[iSerial]); }
	// if(p_mc->Parameters.IsCanEnable == true) { CanBus_Init(p_mc->CONFIG.P_CAN_BUS, p_mc->Parameters.CanServicesId); } //move enable

	MotAnalogUser_Init(&p_mc->AnalogUser);

	VMonitor_InitAdcVRef_MilliV(p_mc->Parameters.AdcVRef_MilliV);
	VMonitor_Init(&p_mc->VMonitorPos);
	VMonitor_Init(&p_mc->VMonitorSense);
	VMonitor_Init(&p_mc->VMonitorAcc);

	Thermistor_InitAdcVRef_Scalar(p_mc->Parameters.AdcVRef_MilliV);
	Thermistor_Init(&p_mc->ThermistorPcb);
	Thermistor_Init(&p_mc->ThermistorMosfetsTop);
	Thermistor_Init(&p_mc->ThermistorMosfetsBot);

	/* set values to not enter fault state */
	p_mc->AnalogResults.VPos_Adcu 		= p_mc->VMonitorPos.Params.LimitLower_Adcu + 1U;
	p_mc->AnalogResults.VSense_Adcu 	= p_mc->VMonitorSense.Params.LimitLower_Adcu + 1U;
	p_mc->AnalogResults.VAcc_Adcu 		= p_mc->VMonitorAcc.Params.LimitLower_Adcu + 1U;
	p_mc->AnalogResults.HeatPcb_Adcu 	= p_mc->ThermistorPcb.Params.Threshold_Adcu; //todo wrap
	if(Thermistor_GetIsEnable(&p_mc->ThermistorMosfetsTop)) { p_mc->AnalogResults.HeatMosfetsTop_Adcu = p_mc->ThermistorMosfetsTop.Params.Threshold_Adcu; }
	if(Thermistor_GetIsEnable(&p_mc->ThermistorMosfetsBot)) { p_mc->AnalogResults.HeatMosfetsBot_Adcu = p_mc->ThermistorMosfetsBot.Params.Threshold_Adcu; }

	Motor_InitAdcVRef_MilliV(p_mc->Parameters.AdcVRef_MilliV);
	Motor_InitVRefSupply_V(VMonitor_GetVInRefMax(&p_mc->VMonitorPos));
	for(uint8_t iMotor = 0U; iMotor < p_mc->CONFIG.MOTOR_COUNT; iMotor++) { Motor_Init(&p_mc->CONFIG.P_MOTORS[iMotor]); }

	Blinky_Init(&p_mc->Buzzer);
	Pin_Output_Init(&p_mc->Relay);
	Pin_Output_Init(&p_mc->Meter);
	Debounce_Init(&p_mc->OptDin, 5U);

	Timer_InitPeriodic(&p_mc->TimerSeconds, 		1000U);
	Timer_InitPeriodic(&p_mc->TimerMillis, 			1U);
	Timer_InitPeriodic(&p_mc->TimerMillis10, 		10U);
	Timer_InitPeriodic(&p_mc->TimerIsrDividerSeconds, 	1000U);
	Timer_Init(&p_mc->TimerState);

	for(uint8_t iProtocol = 0U; iProtocol < p_mc->CONFIG.PROTOCOL_COUNT; iProtocol++) { Protocol_Init(&p_mc->CONFIG.P_PROTOCOLS[iProtocol]); }

	Shell_Init(&p_mc->Shell);

	Linear_ADC_Init(&p_mc->BatteryLife, p_mc->Parameters.BatteryZero_Adcu, p_mc->Parameters.BatteryFull_Adcu, 1000U);

	p_mc->MainDirection = MOTOR_CONTROLLER_DIRECTION_PARK;
	p_mc->UserDirection = MOTOR_CONTROLLER_DIRECTION_PARK;

	StateMachine_Init(&p_mc->StateMachine);

	//
}

bool MotorController_SaveParameters_Blocking(MotorController_T * p_mc)
{
	NvMemory_Status_T status = NV_MEMORY_STATUS_SUCCESS;
	Motor_T * p_motor;
	Protocol_T * p_protocol;

#if defined(CONFIG_MOTOR_CONTROLLER_PARAMETERS_EEPROM)
	for(uint8_t iMotor = 0U; iMotor < p_mc->CONFIG.MOTOR_COUNT; iMotor++)
	{
		p_motor = MotorController_GetPtrMotor(p_mc, iMotor);
		EEPROM_Write_Blocking(p_mc->CONFIG.P_EEPROM, p_motor->CONFIG.P_PARAMS_NVM, 			&p_motor->Parameters, 			sizeof(Motor_Params_T));
		EEPROM_Write_Blocking(p_mc->CONFIG.P_EEPROM, p_motor->Hall.CONFIG.P_PARAMS_NVM, 	&p_motor->Hall.Params, 			sizeof(Hall_Params_T));
		EEPROM_Write_Blocking(p_mc->CONFIG.P_EEPROM, p_motor->Encoder.CONFIG.P_PARAMS, 		&p_motor->Encoder.Params, 		sizeof(Encoder_Params_T));
		EEPROM_Write_Blocking(p_mc->CONFIG.P_EEPROM, p_motor->PidSpeed.CONFIG.P_PARAMS, 	&p_motor->PidSpeed.Params, 		sizeof(PID_Params_T));
		EEPROM_Write_Blocking(p_mc->CONFIG.P_EEPROM, p_motor->PidIq.CONFIG.P_PARAMS, 		&p_motor->PidIq.Params, 		sizeof(PID_Params_T));
		EEPROM_Write_Blocking(p_mc->CONFIG.P_EEPROM, p_motor->PidId.CONFIG.P_PARAMS, 		&p_motor->PidId.Params, 		sizeof(PID_Params_T));
		EEPROM_Write_Blocking(p_mc->CONFIG.P_EEPROM, p_motor->PidIBus.CONFIG.P_PARAMS, 		&p_motor->PidIBus.Params, 		sizeof(PID_Params_T));
		EEPROM_Write_Blocking(p_mc->CONFIG.P_EEPROM, p_motor->Thermistor.CONFIG.P_PARAMS, 	&p_motor->Thermistor.Params, 	sizeof(Thermistor_Params_T));
	}

	EEPROM_Write_Blocking(p_mc->CONFIG.P_EEPROM, p_mc->CONFIG.P_PARAMS_NVM, 					&p_mc->Parameters, 						sizeof(MotorController_Params_T));
	EEPROM_Write_Blocking(p_mc->CONFIG.P_EEPROM, p_mc->CONFIG.P_MEM_MAP_BOOT, 					&p_mc->MemMapBoot, 						sizeof(MemMapBoot_T));
	EEPROM_Write_Blocking(p_mc->CONFIG.P_EEPROM, p_mc->AnalogUser.CONFIG.P_PARAMS, 				&p_mc->AnalogUser.Params, 				sizeof(MotAnalogUser_Params_T));
	EEPROM_Write_Blocking(p_mc->CONFIG.P_EEPROM, p_mc->ThermistorPcb.CONFIG.P_PARAMS, 			&p_mc->ThermistorPcb.Params, 			sizeof(Thermistor_Params_T));
	EEPROM_Write_Blocking(p_mc->CONFIG.P_EEPROM, p_mc->ThermistorMosfetsTop.CONFIG.P_PARAMS, 	&p_mc->ThermistorMosfetsTop.Params, 	sizeof(Thermistor_Params_T));
	EEPROM_Write_Blocking(p_mc->CONFIG.P_EEPROM, p_mc->ThermistorMosfetsBot.CONFIG.P_PARAMS, 	&p_mc->ThermistorMosfetsBot.Params, 	sizeof(Thermistor_Params_T));
 	EEPROM_Write_Blocking(p_mc->CONFIG.P_EEPROM, p_mc->VMonitorPos.CONFIG.P_PARAMS, 			&p_mc->VMonitorPos.Params, 				sizeof(VMonitor_Params_T));
	EEPROM_Write_Blocking(p_mc->CONFIG.P_EEPROM, p_mc->VMonitorAcc.CONFIG.P_PARAMS, 			&p_mc->VMonitorAcc.Params, 				sizeof(VMonitor_Params_T));
	EEPROM_Write_Blocking(p_mc->CONFIG.P_EEPROM, p_mc->VMonitorSense.CONFIG.P_PARAMS, 			&p_mc->VMonitorSense.Params, 			sizeof(VMonitor_Params_T));

	for(uint8_t iProtocol = 0U; iProtocol < p_mc->CONFIG.PROTOCOL_COUNT; iProtocol++)
	{
		p_protocol = &p_mc->CONFIG.P_PROTOCOLS[iProtocol];
		EEPROM_Write_Blocking(p_mc->CONFIG.P_EEPROM, p_protocol->CONFIG.P_PARAMS, &p_protocol->Params, sizeof(Protocol_Params_T));
	}

	EEPROM_Write_Blocking(p_mc->CONFIG.P_EEPROM, p_mc->Shell.CONFIG.P_PARAMS, &p_mc->Shell.Params, sizeof(Shell_Params_T));
#elif defined(CONFIG_MOTOR_CONTROLLER_PARAMETERS_FLASH)

#endif

	return (status == NV_MEMORY_STATUS_SUCCESS) ? true : false;
}

void MotorController_SaveBootReg_Blocking(MotorController_T * p_mc)
{
#if defined(CONFIG_MOTOR_CONTROLLER_PARAMETERS_EEPROM)
	EEPROM_Write_Blocking(p_mc->CONFIG.P_EEPROM, p_mc->CONFIG.P_MEM_MAP_BOOT, &p_mc->MemMapBoot, sizeof(MemMapBoot_T));
#elif defined(CONFIG_MOTOR_CONTROLLER_PARAMETERS_FLASH)

#endif
}

