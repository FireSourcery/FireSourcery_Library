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
	@file 	MotorController_User.h
	@author FireSoucery
	@brief 	UI Wrappers. User accessor functions, error checking applied
	@version V0
*/
/******************************************************************************/
#ifndef MOTOR_CONTROLLER_USER_H
#define MOTOR_CONTROLLER_USER_H

#include "MotorController_StateMachine.h"

/*
	Common Input Interface
	Subject to StateMachine process
	Need to save to temporary variable, as StateMachine functions pass 1 context variable only

	todo update function to pass through to motor state machine, eliminate redundancy
	throttle, brake, direction,
*/

static inline void MotorController_User_DisableControl(MotorController_T * p_mc) 		{ MotorController_DisableMotorAll(p_mc); }

/* State machine handles ignore while braking */
static inline void MotorController_User_SetNeutral(MotorController_T * p_mc) 			{ StateMachine_Semisynchronous_ProcInput(&p_mc->StateMachine, MCSM_INPUT_SET_NEUTRAL); }
static inline void MotorController_User_ProcNeutral(MotorController_T * p_mc) 			{ StateMachine_Semisynchronous_ProcInput(&p_mc->StateMachine, MCSM_INPUT_NEUTRAL); }
static inline void MotorController_User_SetReleaseThrottle(MotorController_T * p_mc) 	{ StateMachine_Semisynchronous_ProcInput(&p_mc->StateMachine, MCSM_INPUT_RELEASE_THROTTLE); }
static inline void MotorController_User_SetReleaseBrake(MotorController_T * p_mc) 		{ StateMachine_Semisynchronous_ProcInput(&p_mc->StateMachine, MCSM_INPUT_RELEASE_BRAKE); }
static inline void MotorController_User_ProcRelease(MotorController_T * p_mc) 			{ StateMachine_Semisynchronous_ProcInput(&p_mc->StateMachine, MCSM_INPUT_NULL); }

static inline void MotorController_User_SetCmdThrottle(MotorController_T * p_mc, uint16_t userCmd)
{
	p_mc->UserCmd = userCmd;
	StateMachine_Semisynchronous_ProcInput(&p_mc->StateMachine, MCSM_INPUT_THROTTLE);
}

static inline void MotorController_User_SetCmdBrake(MotorController_T * p_mc, uint16_t userCmd)
{
	p_mc->UserCmd = userCmd;
	StateMachine_Semisynchronous_ProcInput(&p_mc->StateMachine, MCSM_INPUT_BRAKE);
}

// static inline void MotorController_User_SetCmdBrakeAlt(MotorController_T * p_mc, uint16_t userCmd)
// {
// 	p_mc->UserCmd = userCmd;
// 	StateMachine_Semisynchronous_ProcInput(&p_mc->StateMachine, MCSM_INPUT_BRAKE_ALT);
// }

static inline void MotorController_User_SetDirection(MotorController_T * p_mc, MotorController_Direction_T direction)
{
	p_mc->UserDirection = direction;
	if(p_mc->MainDirection != direction) { StateMachine_Semisynchronous_ProcInput(&p_mc->StateMachine, MCSM_INPUT_DIRECTION); }
}

static inline void MotorController_User_SaveParameters_Blocking(MotorController_T * p_mc)
{
	p_mc->NvmSubstate = MOTOR_CONTROLLER_NVM_ALL;
	StateMachine_Semisynchronous_ProcInput(&p_mc->StateMachine, MCSM_INPUT_SAVE_PARAMS);
}

static inline void MotorController_User_SaveBootReg_Blocking(MotorController_T * p_mc)
{
	p_mc->NvmSubstate = MOTOR_CONTROLLER_NVM_BOOT;
	StateMachine_Semisynchronous_ProcInput(&p_mc->StateMachine, MCSM_INPUT_SAVE_PARAMS);
}

static inline bool MotorController_User_CheckFault(MotorController_T * p_mc)
{
	return (StateMachine_GetActiveStateId(&p_mc->StateMachine) == MCSM_STATE_ID_FAULT);
}

static inline bool MotorController_User_ClearFault(MotorController_T * p_mc)
{
	if(StateMachine_GetActiveStateId(&p_mc->StateMachine) == MCSM_STATE_ID_FAULT)
	{
		StateMachine_Semisynchronous_ProcInput(&p_mc->StateMachine, MCSM_INPUT_FAULT);
	}

	return (StateMachine_GetActiveStateId(&p_mc->StateMachine) != MCSM_STATE_ID_FAULT);
}

static inline void MotorController_User_SetFault(MotorController_T * p_mc)
{
	if(StateMachine_GetActiveStateId(&p_mc->StateMachine) != MCSM_STATE_ID_FAULT)
	{
		StateMachine_Semisynchronous_ProcInput(&p_mc->StateMachine, MCSM_INPUT_FAULT);
	}
}

static inline void MotorController_User_ToggleFault(MotorController_T * p_mc)
{
	StateMachine_Semisynchronous_ProcInput(&p_mc->StateMachine, MCSM_INPUT_FAULT);
}

static inline void MotorController_User_BeepN(MotorController_T * p_mc, uint32_t onTime, uint32_t offTime, uint8_t n) { Blinky_BlinkN(&p_mc->Buzzer, onTime, offTime, n); }
static inline void MotorController_User_BeepStart(MotorController_T * p_mc, uint32_t onTime, uint32_t offTime) { Blinky_StartPeriodic(&p_mc->Buzzer, onTime, offTime); }
static inline void MotorController_User_BeepStop(MotorController_T * p_mc) { Blinky_Stop(&p_mc->Buzzer); }

/*
	Input mode voluntarily call checked function, avoids intermediate buffer
*/
//static inline void MotorController_User_Serial_SetCmdValue(MotorController_T * p_mc, uint16_t userCmd) 	{ if (p_mc->Parameters.InputMode == MOTOR_CONTROLLER_INPUT_MODE_SERIAL) {p_mc->UserCmd = userCmd;} }
//static inline void MotorController_User_CanBus_SetCmdValue(MotorController_T * p_mc, uint16_t userCmd) 	{ if (p_mc->Parameters.InputMode == MOTOR_CONTROLLER_INPUT_MODE_CAN) {p_mc->UserCmd = userCmd;} }

/*
	Controller RAM Variables
*/
static inline MotorController_StateMachine_StateId_T MotorController_User_GetStateId(MotorController_T * p_mc) { return StateMachine_GetActiveStateId(&p_mc->StateMachine); }
static inline MotorController_Direction_T MotorController_User_GetDirection(MotorController_T * p_mc) { return p_mc->MainDirection; }
static inline MotorController_InputMode_T MotorController_User_GetInputMode(MotorController_T * p_mc) { return p_mc->Parameters.InputMode; }
static inline uint16_t MotorController_User_GetAdcu(MotorController_T * p_mc, MotAnalog_Channel_T adcChannel) { return p_mc->AnalogResults.Channels[adcChannel]; }
static inline uint16_t MotorController_User_GetAdcu_Msb8(MotorController_T * p_mc, MotAnalog_Channel_T adcChannel) { return MotorController_User_GetAdcu(p_mc, adcChannel) >> (ADC_BITS - 8U); }
static inline uint16_t MotorController_User_GetFaultAdcu(MotorController_T * p_mc, MotAnalog_Channel_T adcChannel) { return p_mc->FaultAnalogRecord.Channels[adcChannel]; }
static inline uint16_t MotorController_User_GetCmdValue(MotorController_T * p_mc) { return p_mc->UserCmd; }

static inline uint32_t MotorController_User_GetVPos(MotorController_T * p_mc, uint16_t vScalar) 				{ return VMonitor_ConvertToV(&p_mc->VMonitorPos, p_mc->AnalogResults.VPos_Adcu, vScalar); }
static inline uint32_t MotorController_User_GetVSense(MotorController_T * p_mc, uint16_t vScalar) 				{ return VMonitor_ConvertToV(&p_mc->VMonitorSense, p_mc->AnalogResults.VSense_Adcu, vScalar); }
static inline uint32_t MotorController_User_GetVAcc(MotorController_T * p_mc, uint16_t vScalar) 				{ return VMonitor_ConvertToV(&p_mc->VMonitorAcc, p_mc->AnalogResults.VAcc_Adcu, vScalar); }
static inline int32_t MotorController_User_GetHeatPcb_DegC(MotorController_T * p_mc, uint8_t scalar) 			{ return Thermistor_ConvertToDegC_Int(&p_mc->ThermistorPcb, p_mc->AnalogResults.HeatPcb_Adcu, scalar); }
static inline int32_t MotorController_User_GetHeatMosfetsTop_DegC(MotorController_T * p_mc, uint8_t scalar) 	{ return Thermistor_ConvertToDegC_Int(&p_mc->ThermistorMosfetsTop, p_mc->AnalogResults.HeatMosfetsTop_Adcu, scalar); }
static inline int32_t MotorController_User_GetHeatMosfetsBot_DegC(MotorController_T * p_mc, uint8_t scalar) 	{ return Thermistor_ConvertToDegC_Int(&p_mc->ThermistorMosfetsBot, p_mc->AnalogResults.HeatMosfetsBot_Adcu, scalar); }

static inline uint32_t MotorController_User_GetFaultVPos(MotorController_T * p_mc, uint16_t vScalar) 				{ return VMonitor_ConvertToV(&p_mc->VMonitorPos, p_mc->FaultAnalogRecord.VPos_Adcu, vScalar); }
static inline uint32_t MotorController_User_GetFaultVSense(MotorController_T * p_mc, uint16_t vScalar) 				{ return VMonitor_ConvertToV(&p_mc->VMonitorSense, p_mc->FaultAnalogRecord.VSense_Adcu, vScalar); }
static inline uint32_t MotorController_User_GetFaultVAcc(MotorController_T * p_mc, uint16_t vScalar) 				{ return VMonitor_ConvertToV(&p_mc->VMonitorAcc, p_mc->FaultAnalogRecord.VAcc_Adcu, vScalar); }
static inline int32_t MotorController_User_GetFaultHeatPcb_DegC(MotorController_T * p_mc, uint8_t scalar) 			{ return Thermistor_ConvertToDegC_Int(&p_mc->ThermistorPcb, p_mc->FaultAnalogRecord.HeatPcb_Adcu, scalar); }
static inline int32_t MotorController_User_GetFaultHeatMosfetsTop_DegC(MotorController_T * p_mc, uint8_t scalar) 	{ return Thermistor_ConvertToDegC_Int(&p_mc->ThermistorMosfetsTop, p_mc->FaultAnalogRecord.HeatMosfetsTop_Adcu, scalar); }
static inline int32_t MotorController_User_GetFaultHeatMosfetsBot_DegC(MotorController_T * p_mc, uint8_t scalar) 	{ return Thermistor_ConvertToDegC_Int(&p_mc->ThermistorMosfetsBot, p_mc->FaultAnalogRecord.HeatMosfetsBot_Adcu, scalar); }

// static inline uint32_t MotorController_User_GetVPosFault_MilliV(MotorController_T * p_mc)	{return VMonitor_ConvertToMilliV(&p_mc->VMonitorPos, p_mc->FaultAnalogRecord.VPos_Adcu);}
// static inline uint16_t MotorController_User_GetVSenseFault_MilliV(MotorController_T * p_mc)	{return VMonitor_ConvertToMilliV(&p_mc->VMonitorSense, p_mc->FaultAnalogRecord.VSense_Adcu);}
// static inline uint16_t MotorController_User_GetVAccFault_MilliV(MotorController_T * p_mc)	{return VMonitor_ConvertToMilliV(&p_mc->VMonitorAcc, p_mc->FaultAnalogRecord.VAcc_Adcu);}
// static inline uint16_t MotorController_User_GetHeatPcbFault_DegC(MotorController_T * p_mc, uint8_t scalar) 		{ return Thermistor_ConvertToDegC_Int(&p_mc->ThermistorPcb, p_mc->FaultAnalogRecord.HeatPcb_Adcu, scalar); }


/*
	Controller NvM Variables
*/


static inline uint16_t MotorController_User_GetAdcVRef(MotorController_T * p_mc)  { return p_mc->Parameters.AdcVRef_MilliV; }

/* VSupply   */
static inline uint16_t MotorController_User_GetVSupply(MotorController_T * p_mc)  { return VMonitor_GetVInRefMax(&p_mc->VMonitorPos); }
static inline void MotorController_User_SetVSupply(MotorController_T * p_mc, uint16_t volts)
{
	(volts > p_mc->CONFIG.V_MAX) ? VMonitor_SetVInRefMax(&p_mc->VMonitorPos, p_mc->CONFIG.V_MAX) : VMonitor_SetVInRefMax(&p_mc->VMonitorPos, volts*1000U);

	//propagate paarams?
}

static inline void MotorController_User_SetBatteryLife_MilliV(MotorController_T * p_mc, uint32_t zero_mV, uint32_t max_mV)
{
	p_mc->Parameters.BatteryZero_Adcu = VMonitor_ConvertMilliVToAdcu(&p_mc->VMonitorPos, zero_mV);
	p_mc->Parameters.BatteryFull_Adcu = VMonitor_ConvertMilliVToAdcu(&p_mc->VMonitorPos, max_mV);
	Linear_ADC_Init(&p_mc->BatteryLife, p_mc->Parameters.BatteryZero_Adcu, p_mc->Parameters.BatteryFull_Adcu, 1000U);
}

static inline void MotorController_User_SetFastBoot(MotorController_T * p_mc, bool isEnable) 		{ p_mc->MemMapBoot.FastBoot = isEnable; }
static inline void MotorController_User_SetLoadDefault(MotorController_T * p_mc, bool isEnable) 	{ p_mc->MemMapBoot.LoadDefault = isEnable; }

static inline void MotorController_User_SetOptDinSpeedLimit(MotorController_T * p_mc, uint16_t scalar_Frac16) 	{ p_mc->Parameters.OptDinFunction = MOTOR_CONTROLLER_OPT_DIN_SPEED_LIMIT; p_mc->Parameters.OptDinSpeedLimit_Frac16 = scalar_Frac16; }
static inline void MotorController_User_DisableOptDin(MotorController_T * p_mc) 								{ p_mc->Parameters.OptDinFunction = MOTOR_CONTROLLER_OPT_DIN_DISABLE; }

static inline void MotorController_User_SetILimitOnLowVParam(MotorController_T * p_mc, uint16_t scalar_Frac16) 	{ p_mc->Parameters.ILimitScalarOnLowV_Frac16 = scalar_Frac16; }
static inline void MotorController_User_SetILimitOnHeatParam(MotorController_T * p_mc, uint16_t scalar_Frac16) 	{ p_mc->Parameters.ILimitScalarOnHeat_Frac16 = scalar_Frac16; }

// change to Linear_Init flexible scalar output
// static inline uint32_t MotorController_User_GetBatteryCharge_Base10(MotorController_T * p_mc, uint8_t scalar) 	{ return Linear_ADC_CalcPhysical(&p_mc->BatteryLife, p_mc->AnalogResults.VPos_Adcu); }
static inline uint32_t MotorController_User_GetBatteryCharge_Unit1000(MotorController_T * p_mc) 				{ return Linear_ADC_CalcPhysical(&p_mc->BatteryLife, p_mc->AnalogResults.VPos_Adcu); }
static inline uint32_t MotorController_User_GetBatteryCharge_Frac16(MotorController_T * p_mc) 					{ return Linear_ADC_CalcFraction16(&p_mc->BatteryLife, p_mc->AnalogResults.VPos_Adcu); }

/*
	ReadOnly
*/
static inline uint32_t MotorController_User_GetLibraryVersion(MotorController_T * p_mc) { return MOT_SOFTWARE_VERSION_ID; }
static inline uint8_t MotorController_User_GetLibraryVersionIndex(MotorController_T * p_mc, uint8_t charIndex)
{
	static const uint8_t version[4U] = { MOT_SOFTWARE_VERSION_BUGFIX, MOT_SOFTWARE_VERSION_MINOR, MOT_SOFTWARE_VERSION_MAJOR, MOT_SOFTWARE_VERSION_OPT };
	return version[charIndex];
}

static inline uint32_t MotorController_User_GetMainVersion(MotorController_T * p_mc) { return *((uint32_t *)(&p_mc->CONFIG.SOFTWARE_VERSION[0U])); }
static inline uint8_t MotorController_User_GetMainVersionIndex(MotorController_T * p_mc, uint8_t charIndex) { return p_mc->CONFIG.SOFTWARE_VERSION[charIndex]; }

static inline uint32_t MotorController_User_GetVMax(MotorController_T * p_mc) { return p_mc->CONFIG.V_MAX; }
static inline uint32_t MotorController_User_GetIMax(MotorController_T * p_mc) { return p_mc->CONFIG.I_MAX; }

/*
	WriteOnce Variables, manufacture use, no state machine error check
*/
static inline uint8_t MotorController_User_GetName(MotorController_T * p_mc, uint8_t charIndex) { return p_mc->CONFIG.P_ONCE->NAME[charIndex]; }
static inline uint8_t MotorController_User_WriteName_Blocking(MotorController_T * p_mc, uint8_t charIndex, uint8_t nameChar) {};
static inline void MotorController_WriteSerialNumber_Blocking(MotorController_T * p_mc, uint32_t serialNumber)
{
	// Flash_WriteOnce_Blocking(p_mc->CONFIG.P_FLASH, p_mc->CONFIG.P_ONCE, &serialNumber, 4U);
}

/*
	Motor Wrapper, no implementation of motor wrappers
*/
static inline Motor_T * MotorController_User_GetPtrMotor(const MotorController_T * p_mc, uint8_t motorIndex)
{
	return (motorIndex < p_mc->CONFIG.MOTOR_COUNT) ? MotorController_GetPtrMotor(p_mc, motorIndex) : MotorController_GetPtrMotor(p_mc, 0U);
}

/*
	Module additional wrappers, ram and nvm
*/
static inline uint16_t MotorController_User_GetThrottle(MotorController_T * p_mc) 		{ return MotAnalogUser_GetThrottle(&p_mc->AnalogUser); }
static inline uint8_t MotorController_User_GetThrottle_Frac8(MotorController_T * p_mc) 	{ return MotAnalogUser_GetThrottle(&p_mc->AnalogUser) >> 8U; }
static inline uint16_t MotorController_User_GetBrake(MotorController_T * p_mc) 			{ return MotAnalogUser_GetBrake(&p_mc->AnalogUser); }
static inline uint8_t MotorController_User_GetBrake_Frac8(MotorController_T * p_mc) 	{ return MotAnalogUser_GetBrake(&p_mc->AnalogUser) >> 8U; }
// static inline bool MotorController_User_GetThrottleSwitch(MotorController_T * p_mc) 	{return MotAnalogUser_GetIsThrottleOn(&p_mc->AnalogUser);}
// static inline bool MotorController_User_GetBrakeSwitch(MotorController_T * p_mc) 		{return MotAnalogUser_GetIsBrakeOn(&p_mc->AnalogUser);}
// static inline bool MotorController_User_GetForwardSwitch(MotorController_T * p_mc) 		{return MotAnalogUser_GetForwardSwitch(&p_mc->AnalogUser);}
// static inline bool MotorController_User_GetReverseSwitch(MotorController_T * p_mc) 		{return MotAnalogUser_GetReverseSwitch(&p_mc->AnalogUser);}
static inline bool MotorController_User_GetOptDinSwitch(MotorController_T * p_user) 	{ return Debounce_GetState(&p_user->OptDin); }

static inline uint16_t MotorController_User_GetHeatPcbLimit_DegC(MotorController_T * p_mc, uint16_t scalar) 			{ return Thermistor_GetLimitShutdown_DegCInt(&p_mc->ThermistorPcb, scalar); }
static inline uint16_t MotorController_User_GetHeatPcbThreshold_DegC(MotorController_T * p_mc, uint16_t scalar) 		{ return Thermistor_GetLimitThreshold_DegCInt(&p_mc->ThermistorPcb, scalar); }
static inline void MotorController_User_SetHeatPcbLimit_DegC(MotorController_T * p_mc, uint8_t limit_degreesC) 			{ Thermistor_SetLimitShutdown_DegC(&p_mc->ThermistorPcb, limit_degreesC); }
static inline void MotorController_User_SetHeatPcbThreshold_DegC(MotorController_T * p_mc, uint8_t threshold_degreesC) 	{ Thermistor_SetLimitThreshold_DegC(&p_mc->ThermistorPcb, threshold_degreesC); }

static inline void MotorController_User_SetVPosLimitUpper_MilliV(MotorController_T * p_mc, uint32_t limit) { VMonitor_SetLimitUpper_MilliV(&p_mc->VMonitorPos, limit); }
static inline void MotorController_User_SetVPosLimitLower_MilliV(MotorController_T * p_mc, uint32_t limit) { VMonitor_SetLimitLower_MilliV(&p_mc->VMonitorPos, limit); }
static inline void MotorController_User_SetVSenseLimitUpper_MilliV(MotorController_T * p_mc, uint32_t limit) { VMonitor_SetLimitUpper_MilliV(&p_mc->VMonitorSense, limit); }
static inline void MotorController_User_SetVSenseLimitLower_MilliV(MotorController_T * p_mc, uint32_t limit) { VMonitor_SetLimitLower_MilliV(&p_mc->VMonitorSense, limit); }
static inline void MotorController_User_SetVAccLimitUpper_MilliV(MotorController_T * p_mc, uint32_t limit) { VMonitor_SetLimitUpper_MilliV(&p_mc->VMonitorAcc, limit); }
static inline void MotorController_User_SetVAccLimitLower_MilliV(MotorController_T * p_mc, uint32_t limit) { VMonitor_SetLimitLower_MilliV(&p_mc->VMonitorAcc, limit); }

//static inline uint32_t MotorController_User_GetVPosLimitUpper_V(MotorController_T * p_mc) 	{return p_mc->Parameters.VPosLimitUpper_Adcu;}
//static inline uint32_t MotorController_User_GetVPosLimitLower_V(MotorController_T * p_mc) 	{return p_mc->Parameters.VPosLimitLower_Adcu;}
//static inline uint16_t MotorController_User_GetVPosLimitUpper_Adcu(MotorController_T * p_mc) 		{return p_mc->Parameters.VPosLimitUpper_Adcu;}
//static inline uint16_t MotorController_User_GetVPosLimitLower_Adcu(MotorController_T * p_mc) 		{return p_mc->Parameters.VPosLimitLower_Adcu;}

static inline void MotorController_User_SetMainProtocolXcvr(MotorController_T * p_mc, uint8_t id) { Protocol_SetSpecs(&p_mc->CONFIG.P_PROTOCOLS[0U], id); }
static inline void MotorController_User_SetMainProtoclSpecs(MotorController_T * p_mc, uint8_t id) { Protocol_SetXcvr(&p_mc->CONFIG.P_PROTOCOLS[0U], id); }

#endif
