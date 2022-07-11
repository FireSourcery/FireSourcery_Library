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

	MotorController_ReadOnce_Blocking(p_mc);

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

	Motor_InitAdcVRef_MilliV(p_mc->Parameters.AdcVRef_MilliV);
	Motor_InitVRefSupply_V(p_mc->Parameters.VSupply);
	for(uint8_t iMotor = 0U; iMotor < p_mc->CONFIG.MOTOR_COUNT; iMotor++) { Motor_Init(&p_mc->CONFIG.P_MOTORS[iMotor]); }

	Blinky_Init(&p_mc->Buzzer);
	Pin_Output_Init(&p_mc->Relay);
	Pin_Output_Init(&p_mc->Meter);
	Debounce_Init(&p_mc->OptDin, 5U);

	Timer_InitPeriodic(&p_mc->TimerSeconds, 			1000U);
	Timer_InitPeriodic(&p_mc->TimerMillis, 				1U);
	Timer_InitPeriodic(&p_mc->TimerMillis10, 			10U);
	Timer_InitPeriodic(&p_mc->TimerIsrDividerSeconds, 	1000U);
	Timer_Init(&p_mc->TimerState);

	for(uint8_t iProtocol = 0U; iProtocol < p_mc->CONFIG.PROTOCOL_COUNT; iProtocol++) { Protocol_Init(&p_mc->CONFIG.P_PROTOCOLS[iProtocol]); }

	Shell_Init(&p_mc->Shell);

	Linear_ADC_Init(&p_mc->BatteryLife, p_mc->Parameters.BatteryZero_Adcu, p_mc->Parameters.BatteryFull_Adcu, 1000U);

	/*
		Shutdown_Adcu [~2176], => ILimitHeat_Frac16
		Warning_Adcu [~2800],  => 65535
		slope = 860,343
		Heat_Adcu Lower is higher heat
	*/
	Linear_Frac16_Init_Map
	(
		&p_mc->ILimitHeatRate,
		p_mc->ThermistorMosfetsTop.Params.Shutdown_Adcu,
		p_mc->ThermistorMosfetsTop.Params.Warning_Adcu,
		p_mc->Parameters.ILimitHeat_Frac16,
		0U /* Param not used */
	);

	// p_mc->MainDirection = MOTOR_CONTROLLER_DIRECTION_PARK;
	// p_mc->UserDirection = MOTOR_CONTROLLER_DIRECTION_PARK;
	// p_mc->SpeedLimitActiveId = MOTOR_SPEED_LIMIT_ACTIVE_DISABLE;
	// p_mc->ILimitActiveId = MOTOR_I_LIMIT_ACTIVE_DISABLE;

	p_mc->ThermistorPcb.Adcu = p_mc->ThermistorPcb.Params.WarningThreshold_Adcu + 1U;
	p_mc->ThermistorMosfetsTop.Adcu = p_mc->ThermistorMosfetsTop.Params.WarningThreshold_Adcu + 1U;
	p_mc->ThermistorMosfetsBot.Adcu = p_mc->ThermistorMosfetsBot.Params.WarningThreshold_Adcu + 1U;

	p_mc->AnalogResults.VPos_Adcu = p_mc->VMonitorPos.Params.WarningLower_Adcu + 1U;
	p_mc->AnalogResults.VAcc_Adcu = p_mc->VMonitorAcc.Params.WarningLower_Adcu + 1U;
	p_mc->AnalogResults.VSense_Adcu = p_mc->VMonitorSense.Params.WarningLower_Adcu + 1U;

	p_mc->AnalogResults.HeatPcb_Adcu = p_mc->ThermistorPcb.Params.WarningThreshold_Adcu + 1U;
	p_mc->AnalogResults.HeatMosfetsTop_Adcu = p_mc->ThermistorMosfetsTop.Params.WarningThreshold_Adcu + 1U;
	p_mc->AnalogResults.HeatMosfetsBot_Adcu = p_mc->ThermistorMosfetsBot.Params.WarningThreshold_Adcu + 1U;

	p_mc->AnalogResults.VPos_Adcu = p_mc->VMonitorPos.Params.WarningLower_Adcu + 1U;
	p_mc->AnalogResults.VAcc_Adcu = p_mc->VMonitorAcc.Params.WarningLower_Adcu + 1U;
	p_mc->AnalogResults.VSense_Adcu = p_mc->VMonitorSense.Params.WarningLower_Adcu + 1U;

	StateMachine_Init(&p_mc->StateMachine);
}

// void MotorController_InitDefault(MotorController_T * p_mc)
// {
// #define SPEED_MAX_DEFAULT 	2000U
// #define ADC_VREF_DEFAULT 	5100U

// 	Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);

// 	MotAnalogUser_InvertPins_T invertPins = { .State = 0U };

// 	MotorController_User_SetAdcVRef(p_mc, ADC_VREF_DEFAULT);
// 	MotorController_User_SetVSupply(p_mc, 42U);
// 	// MotorController_User_SetVSupply(p_mc, p_mc->CONFIG.V_MAX);

// 	//  MotorController_User_SetInputMode(p_mc, MOTOR_CONTROLLER_INPUT_MODE_ANALOG);
// 	//  MotorController_User_SetCoastMode(p_mc, MOTOR_CONTROLLER_COAST_MODE_FLOAT);
// 	// MotorController_User_SetCanBusServicesId(p_mc, 0U);
// 	//  MotorController_User_DisableCanBusId(p_mc, 0U);

// 	// MotorController_BuzzerFlags_T BuzzerFlagsEnable; /* which options are enabled for use */

// 	// MotorController_User_SetOptDinSpeedLimit(p_mc, 65536U / 2U);
// 	MotorController_User_DisableOptDin(p_mc);
// 	MotorController_User_SetILimitOnLowVParam(p_mc, 65536U / 2U);
// 	MotorController_User_SetILimitOnHeatParam(p_mc, 65536U / 2U);

// 	// VMonitor_SetLimitUpper_MilliV(&p_mc->VMonitorPos, p_mc->CONFIG.V_MAX * 1000U + p_mc->CONFIG.V_MAX * 200U);
// 	// VMonitor_SetLimitLower_MilliV(&p_mc->VMonitorPos, p_mc->CONFIG.V_MAX * 1000U - p_mc->CONFIG.V_MAX * 200U);
// 	// VMonitor_SetWarningUpper_MilliV(&p_mc->VMonitorPos, p_mc->CONFIG.V_MAX * 1000U + p_mc->CONFIG.V_MAX * 100U);
// 	// VMonitor_SetWarningLower_MilliV(&p_mc->VMonitorPos, p_mc->CONFIG.V_MAX * 1000U - p_mc->CONFIG.V_MAX * 100U);
// 	// VMonitor_Enable(&p_mc->VMonitorPos);
// 	// MotorController_User_SetBatteryLife_MilliV(p_mc, p_mc->CONFIG.V_MAX * 1000U - p_mc->CONFIG.V_MAX * 250U, p_mc->CONFIG.V_MAX * 1000U);

// 	VMonitor_SetLimits_MilliV(&p_mc->VMonitorPos, 30000U, 45000U, 35000U, 43000U);
// 	VMonitor_Enable(&p_mc->VMonitorPos);
// 	MotorController_User_SetBatteryLife_MilliV(p_mc, 30000U, 42000U); //need to set vmonitor for conversion

// 	VMonitor_SetLimits_MilliV(&p_mc->VMonitorSense, 4500U, 5500U, 4800U, 5200U);
// 	VMonitor_Enable(&p_mc->VMonitorSense);

// 	VMonitor_SetLimits_MilliV(&p_mc->VMonitorAcc, 10000U, 14000U, 11000U, 13000U);
// 	VMonitor_Enable(&p_mc->VMonitorAcc);

// 	Thermistor_SetNtc(&p_mc->ThermistorPcb, BOARD_THERM_PCB_R0, BOARD_THERM_PCB_T0_DEG_C, BOARD_THERM_PCB_B);
// 	Thermistor_SetVInRef_MilliV(&p_mc->ThermistorPcb, ADC_VREF_DEFAULT);
// 	Thermistor_SetLimits_DegC(&p_mc->ThermistorPcb, 100U, 90U, 80U, 78U);
// 	Thermistor_SetMonitorEnable(&p_mc->ThermistorPcb, BOARD_THERM_PCB_PRESENT);

// 	Thermistor_SetNtc(&p_mc->ThermistorMosfetsTop, BOARD_THERM_MOSFETS_TOP_R0, BOARD_THERM_MOSFETS_TOP_T0_DEG_C, BOARD_THERM_MOSFETS_TOP_B);
// 	Thermistor_SetVInRef_MilliV(&p_mc->ThermistorMosfetsTop, ADC_VREF_DEFAULT); /* Same as adc by default, but may be different */
// 	Thermistor_SetLimits_DegC(&p_mc->ThermistorMosfetsTop, 100U, 90U, 80U, 78U);
// 	Thermistor_SetMonitorEnable(&p_mc->ThermistorMosfetsTop, BOARD_THERM_MOSFETS_TOP_PRESENT);

// 	Thermistor_SetNtc(&p_mc->ThermistorMosfetsBot, BOARD_THERM_MOSFETS_BOT_R0, BOARD_THERM_MOSFETS_BOT_T0_DEG_C, BOARD_THERM_MOSFETS_BOT_B);
// 	Thermistor_SetVInRef_MilliV(&p_mc->ThermistorMosfetsBot, ADC_VREF_DEFAULT);
// 	Thermistor_SetLimits_DegC(&p_mc->ThermistorMosfetsBot, 100U, 90U, 80U, 78U);
// 	Thermistor_SetMonitorEnable(&p_mc->ThermistorMosfetsBot, BOARD_THERM_MOSFETS_BOT_PRESENT);

// 	Thermistor_SetNtc(&p_motor->Thermistor, 0U, 0U, 0U);
// 	Thermistor_SetVInRef_MilliV(&p_motor->Thermistor, ADC_VREF_DEFAULT);
// 	Thermistor_SetLimits_DegC(&p_motor->Thermistor, 100U, 90U, 80U, 78U);
// 	Thermistor_DisableMonitor(&p_motor->Thermistor);

// 	MotAnalogUser_SetBrakeAdc(&p_mc->AnalogUser, 0U, 4095U, true);
// 	MotAnalogUser_SetThrottleAdc(&p_mc->AnalogUser, 0U, 4095U, true);
// 	MotAnalogUser_SetBistateBrake(&p_mc->AnalogUser, false, 65536U / 10U);
// 	MotAnalogUser_SetDirectionPins(&p_mc->AnalogUser, MOT_ANALOG_USER_DIRECTION_PINS_FR);
// 	MotAnalogUser_SetPinInvert(&p_mc->AnalogUser, invertPins);

// 	Motor_User_SetPolePairs(p_motor, 8U);
// 	Motor_User_SetSensorMode(p_motor, MOTOR_SENSOR_MODE_HALL);
// 	// Motor_User_SetAlignMode(p_motor, MOTOR_ALIGN_MODE_DISABLE);
// 	Motor_User_SetCommutationMode(p_motor, MOTOR_COMMUTATION_MODE_FOC);
// 	// Motor_User_SetFeedbackModeParam(p_motor, MOTOR_FEEDBACK_MODE_CONSTANT_SPEED_CURRENT);

// 	// Motor_User_SetSpeedFeedbackRef_VRpm(p_motor, BOARD_V_MAX, SPEED_MAX_DEFAULT);
// 	// Motor_User_SetSpeedFeedbackRef_VRpm(p_motor, 42U, SPEED_MAX_DEFAULT);
// 	Motor_User_SetSpeedFeedbackRef_Rpm(p_motor, SPEED_MAX_DEFAULT);
// 	// Motor_User_SetIPeakRef_Adcu(p_motor, BOARD_I_SENSOR_LIMIT * 9U / 10U);
// 	// Motor_User_SetIPeakRef_MilliV(p_motor,  min_MilliV,  max_MilliV);
// 	// Motor_User_SetIaIbIcZero_Adcu(p_motor, ADC_MAX / 2U, ADC_MAX / 2U, ADC_MAX / 2U);

// 	Motor_User_SetDirectionCalibration(p_motor, MOTOR_FORWARD_IS_CCW);
// 	Motor_User_SetSpeedLimitParam_Rpm(p_motor, SPEED_MAX_DEFAULT, SPEED_MAX_DEFAULT / 2U);
// 	Motor_User_SetILimitParam_Amp(p_motor, BOARD_I_MAX, BOARD_I_MAX / 2U);
// 	Motor_User_SetILimitHeatParam(p_motor, 65536U / 2U);
// 	// Motor_User_VoltageBrakeScalar_Frac16(p_motor, 65536U / 4U);
// 	// Motor_User_SetPhaseModeParam(p_motor, PHASE_MODE_UNIPOLAR_1);
// 	Motor_User_SetAlignVoltage(p_motor, 65536U / 20U);
// 	Motor_User_SetAlignTime_Millis(p_motor, 1000U);

// 	// PID_SetFreq(&p_motor->PidSpeed, 1000U);
// 	// PID_SetFreq(&p_motor->PidIq, 20000U);
// 	// PID_SetFreq(&p_motor->PidId, 20000U);
// 	PID_SetTunings(&p_motor->PidSpeed, 1U, 1U, 1U, 2U, 0U, 0U);
// 	PID_SetTunings(&p_motor->PidIq, 1U, 1U, 1U, 2U, 0U, 0U);
// 	PID_SetTunings(&p_motor->PidId, 1U, 1U, 1U, 2U, 0U, 0U);

// 	// .CountsPerRevolution 			= 8192U,	//currently unused for Hall captureT, should be MotorPolePairs*6
// 	// .DistancePerCount 				= 1U,
// 	// .IsQuadratureCaptureEnabled 		= true,
// 	// .IsALeadBPositive 				= true,
// 	// .ExtendedTimerDeltaTStop 		= 1000U,	//ExtendedTimer time read at deltaT stopped
// 	// .ScalarSpeedRef_Rpm;

// // Motor_User_ActivateCalibrationHall(p_motor);
// // Hall_MapSensorsTable(&p_motor->Hall, HALL_VIRTUAL_SENSORS_A, HALL_VIRTUAL_SENSORS_INV_C, HALL_VIRTUAL_SENSORS_B, HALL_VIRTUAL_SENSORS_INV_A, HALL_VIRTUAL_SENSORS_C, HALL_VIRTUAL_SENSORS_INV_B);
// // Motor_User_ActivateCalibrationEncoder(p_motor);
// // Motor_User_ActivateCalibrationAdc(p_motor);
// // Motor_User_ActivateCalibrationSinCos(p_motor);

// 	// Protocol_SetSpecs(&p_mc->CONFIG.P_PROTOCOLS[0U], 0U);
// 	// Protocol_SetXcvr(&p_mc->CONFIG.P_PROTOCOLS[0U], 1U);
// 	// // Protocol_EnableOnInit(&p_mc->CONFIG.P_PROTOCOLS[0U]);
// 	// Protocol_DisableOnInit(&p_mc->CONFIG.P_PROTOCOLS[0U]);

// 	// Shell_SetXcvr(&p_mc->Shell, 1U);
// 	// Shell_ConfigBaudRate(&p_mc->Shell, 19200U);
// 	// // Shell_SetXcvr(&p_mc->Shell, 0U);
// 	// Shell_EnableOnInit(&p_mc->Shell);
// 	// Shell_DisableOnInit(&p_mc->Shell);
// }

NvMemory_Status_T MotorController_SaveParameters_Blocking(MotorController_T * p_mc)
{
	NvMemory_Status_T status = NV_MEMORY_STATUS_SUCCESS;
	Motor_T * p_motor;
	Protocol_T * p_protocol;

#if defined(CONFIG_MOTOR_CONTROLLER_PARAMETERS_EEPROM)
	EEPROM_T * p_eeprom = p_mc->CONFIG.P_EEPROM;

	for(uint8_t iMotor = 0U; iMotor < p_mc->CONFIG.MOTOR_COUNT; iMotor++)
	{
		p_motor = MotorController_GetPtrMotor(p_mc, iMotor);
		if (status == NV_MEMORY_STATUS_SUCCESS) { status = EEPROM_Write_Blocking(p_eeprom, p_motor->CONFIG.P_PARAMS_NVM, 			&p_motor->Parameters, 			sizeof(Motor_Params_T)); }
		if (status == NV_MEMORY_STATUS_SUCCESS) { status = EEPROM_Write_Blocking(p_eeprom, p_motor->Hall.CONFIG.P_PARAMS_NVM, 		&p_motor->Hall.Params, 			sizeof(Hall_Params_T)); }
		if (status == NV_MEMORY_STATUS_SUCCESS) { status = EEPROM_Write_Blocking(p_eeprom, p_motor->Encoder.CONFIG.P_PARAMS, 		&p_motor->Encoder.Params, 		sizeof(Encoder_Params_T)); }
		if (status == NV_MEMORY_STATUS_SUCCESS) { status = EEPROM_Write_Blocking(p_eeprom, p_motor->PidSpeed.CONFIG.P_PARAMS, 		&p_motor->PidSpeed.Params, 		sizeof(PID_Params_T)); }
		if (status == NV_MEMORY_STATUS_SUCCESS) { status = EEPROM_Write_Blocking(p_eeprom, p_motor->PidIq.CONFIG.P_PARAMS, 			&p_motor->PidIq.Params, 		sizeof(PID_Params_T)); }
		if (status == NV_MEMORY_STATUS_SUCCESS) { status = EEPROM_Write_Blocking(p_eeprom, p_motor->PidId.CONFIG.P_PARAMS, 			&p_motor->PidId.Params, 		sizeof(PID_Params_T)); }
		if (status == NV_MEMORY_STATUS_SUCCESS) { status = EEPROM_Write_Blocking(p_eeprom, p_motor->PidIBus.CONFIG.P_PARAMS, 		&p_motor->PidIBus.Params, 		sizeof(PID_Params_T)); }
		if (status == NV_MEMORY_STATUS_SUCCESS) { status = EEPROM_Write_Blocking(p_eeprom, p_motor->Thermistor.CONFIG.P_PARAMS, 	&p_motor->Thermistor.Params, 	sizeof(Thermistor_Params_T)); }
		if (status != NV_MEMORY_STATUS_SUCCESS) { break; }
	}

	if (status == NV_MEMORY_STATUS_SUCCESS) { status = EEPROM_Write_Blocking(p_eeprom, p_mc->CONFIG.P_PARAMS_NVM, 					&p_mc->Parameters, 						sizeof(MotorController_Params_T)); };
	if (status == NV_MEMORY_STATUS_SUCCESS) { status = EEPROM_Write_Blocking(p_eeprom, p_mc->CONFIG.P_MEM_MAP_BOOT, 				&p_mc->MemMapBoot, 						sizeof(MemMapBoot_T)); };
	if (status == NV_MEMORY_STATUS_SUCCESS) { status = EEPROM_Write_Blocking(p_eeprom, p_mc->AnalogUser.CONFIG.P_PARAMS, 			&p_mc->AnalogUser.Params, 				sizeof(MotAnalogUser_Params_T)); };
	if (status == NV_MEMORY_STATUS_SUCCESS) { status = EEPROM_Write_Blocking(p_eeprom, p_mc->ThermistorPcb.CONFIG.P_PARAMS, 		&p_mc->ThermistorPcb.Params, 			sizeof(Thermistor_Params_T)); };
	if (status == NV_MEMORY_STATUS_SUCCESS) { status = EEPROM_Write_Blocking(p_eeprom, p_mc->ThermistorMosfetsTop.CONFIG.P_PARAMS, 	&p_mc->ThermistorMosfetsTop.Params, 	sizeof(Thermistor_Params_T)); };
	if (status == NV_MEMORY_STATUS_SUCCESS) { status = EEPROM_Write_Blocking(p_eeprom, p_mc->ThermistorMosfetsBot.CONFIG.P_PARAMS, 	&p_mc->ThermistorMosfetsBot.Params, 	sizeof(Thermistor_Params_T)); };
 	if (status == NV_MEMORY_STATUS_SUCCESS) { status = EEPROM_Write_Blocking(p_eeprom, p_mc->VMonitorPos.CONFIG.P_PARAMS, 			&p_mc->VMonitorPos.Params, 				sizeof(VMonitor_Params_T)); };
	if (status == NV_MEMORY_STATUS_SUCCESS) { status = EEPROM_Write_Blocking(p_eeprom, p_mc->VMonitorAcc.CONFIG.P_PARAMS, 			&p_mc->VMonitorAcc.Params, 				sizeof(VMonitor_Params_T)); };
	if (status == NV_MEMORY_STATUS_SUCCESS) { status = EEPROM_Write_Blocking(p_eeprom, p_mc->VMonitorSense.CONFIG.P_PARAMS, 		&p_mc->VMonitorSense.Params, 			sizeof(VMonitor_Params_T)); };
	if (status == NV_MEMORY_STATUS_SUCCESS) { status = EEPROM_Write_Blocking(p_eeprom, p_mc->Shell.CONFIG.P_PARAMS, 				&p_mc->Shell.Params, 					sizeof(Shell_Params_T)); };

	for(uint8_t iProtocol = 0U; iProtocol < p_mc->CONFIG.PROTOCOL_COUNT; iProtocol++)
	{
		p_protocol = &p_mc->CONFIG.P_PROTOCOLS[iProtocol];
		status = EEPROM_Write_Blocking(p_eeprom, p_protocol->CONFIG.P_PARAMS, &p_protocol->Params, sizeof(Protocol_Params_T));
		if (status != NV_MEMORY_STATUS_SUCCESS) { break; }
	}
#elif defined(CONFIG_MOTOR_CONTROLLER_PARAMETERS_FLASH)

#endif
	// p_mc->NvmStatus = status;
	return status;
}

NvMemory_Status_T MotorController_SaveBootReg_Blocking(MotorController_T * p_mc)
{
#if defined(CONFIG_MOTOR_CONTROLLER_PARAMETERS_EEPROM)
	return EEPROM_Write_Blocking(p_mc->CONFIG.P_EEPROM, p_mc->CONFIG.P_MEM_MAP_BOOT, &p_mc->MemMapBoot, sizeof(MemMapBoot_T));
#elif defined(CONFIG_MOTOR_CONTROLLER_PARAMETERS_FLASH)

#endif
}

NvMemory_Status_T MotorController_ReadOnce_Blocking(MotorController_T * p_mc)
{
// #if defined(CONFIG_MOTOR_CONTROLLER_MANUFACTURE_PARAMS_ONCE)
	NvMemory_Status_T status = (NvMemory_Status_T)Flash_ReadOnce_Blocking(p_mc->CONFIG.P_FLASH, (uint8_t *)p_mc->CONFIG.P_ONCE, sizeof(MotorController_Manufacture_T));
	Flash_GetReadOnce(p_mc->CONFIG.P_FLASH, (uint8_t *)&p_mc->OnceBuffer);
	return status;
// #elif defined(CONFIG_MOTOR_CONTROLLER_MANUFACTURE_PARAMS_FLASH)
// (void) p_mc;
// #endif
}

NvMemory_Status_T MotorController_SaveOnce_Blocking(MotorController_T * p_mc)
{
// #if defined(CONFIG_MOTOR_CONTROLLER_MANUFACTURE_PARAMS_ONCE)
	return (NvMemory_Status_T)Flash_WriteOnce_Blocking(p_mc->CONFIG.P_FLASH, (uint8_t *)p_mc->CONFIG.P_ONCE, (uint8_t *)&p_mc->OnceBuffer, sizeof(MotorController_Manufacture_T));
// #elif defined(CONFIG_MOTOR_CONTROLLER_MANUFACTURE_PARAMS_FLASH)

// #endif
}

