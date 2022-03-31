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
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef MOTOR_CONTROLLER_USER_H
#define MOTOR_CONTROLLER_USER_H

#include "MotorController_StateMachine.h"
#include "MotorController.h"
#include "Config.h"


/*
 * Controller Nvm Variables
 */
static inline uint8_t MotorController_User_GetName(MotorController_T * p_mc, uint8_t charIndex) {return p_mc->CONFIG.P_ONCE->NAME[charIndex];}
static inline void MotorController_User_SetFastBoot(MotorController_T * p_mc, bool isEnable)	{p_mc->MemMapBoot.FastBoot = isEnable;}

/*
 * Controller ram variables
 */
static inline uint16_t MotorController_User_GetAdcu(MotorController_T * p_mc, MotAnalog_Channel_T adcChannel)		{return p_mc->AnalogResults.Channels[adcChannel];}
static inline uint16_t MotorController_User_GetAdcu_Msb8(MotorController_T * p_mc, MotAnalog_Channel_T adcChannel)	{return MotorController_User_GetAdcu(p_mc, adcChannel) >> (CONFIG_MOTOR_CONTROLLER_ADCU_BITS_N - 8U);}

static inline uint16_t MotorController_User_GetCmdValue(MotorController_T * p_mc)	{return p_mc->UserCmd;}
static inline MotorController_Direction_T MotorController_User_GetDirection(MotorController_T * p_mc) {return p_mc->MainDirection;}
static inline MotorController_InputMode_T MotorController_User_GetInputMode(MotorController_T * p_mc) {return p_mc->Parameters.InputMode;}


/*
 * Motor Controller Module Wrapper
 */
static inline uint16_t MotorController_User_GetThrottle(MotorController_T * p_mc)		{return MotAnalogUser_GetThrottle(&p_mc->AnalogUser);}
static inline uint8_t MotorController_User_GetThrottle_Frac8(MotorController_T * p_mc)	{return MotAnalogUser_GetThrottle(&p_mc->AnalogUser) >> 8U;}
static inline bool MotorController_User_GetThrottleSwitch(MotorController_T * p_mc) 	{return MotAnalogUser_GetThrottleSwitch(&p_mc->AnalogUser);}
static inline uint16_t MotorController_User_GetBrake(MotorController_T * p_mc)			{return MotAnalogUser_GetBrake(&p_mc->AnalogUser);}
static inline uint8_t MotorController_User_GetBrake_Frac8(MotorController_T * p_mc)		{return MotAnalogUser_GetBrake(&p_mc->AnalogUser) >> 8U;}
static inline bool MotorController_User_GetBrakeSwitch(MotorController_T * p_mc) 		{return MotAnalogUser_GetBrakeSwitch(&p_mc->AnalogUser);}
static inline bool MotorController_User_GetForwardSwitch(MotorController_T * p_mc) 		{return MotAnalogUser_GetForwardSwitch(&p_mc->AnalogUser);}
static inline bool MotorController_User_GetReverseSwitch(MotorController_T * p_mc) 		{return MotAnalogUser_GetReverseSwitch(&p_mc->AnalogUser);}
static inline bool MotorController_User_GetDInSwitch(MotorController_T * p_user) 		{return Debounce_GetState(&p_user->DIn);}

static inline uint16_t MotorController_User_GetHeatPcb_DegCInt(MotorController_T * p_mc, uint8_t scalar)
{
	return Thermistor_ConvertToDegC_Int(&p_mc->ThermistorPcb, p_mc->AnalogResults.HeatPcb_ADCU, scalar);
}

static inline uint16_t MotorController_User_GetHeatPcbLimit_DegCInt(MotorController_T * p_mc, uint16_t scalar)
{
	return Thermistor_GetHeatLimit_DegCInt(&p_mc->ThermistorPcb, scalar);
}

static inline uint16_t MotorController_User_GetHeatPcbThreshold_DegCInt(MotorController_T * p_mc, uint16_t scalar)
{
	return Thermistor_GetHeatThreshold_DegCInt(&p_mc->ThermistorPcb, scalar);
}

static inline void MotorController_User_SetHeatPcbLimit_DegC(MotorController_T * p_mc, uint8_t limit_degreesC)
{
	return Thermistor_SetHeatLimit_DegC(&p_mc->ThermistorPcb, limit_degreesC);
}

static inline void MotorController_User_SetHeatPcbThreshold_DegC(MotorController_T * p_mc, uint8_t threshold_degreesC)
{
	return Thermistor_SetHeatThreshold_DegC(&p_mc->ThermistorPcb, threshold_degreesC);
}

static inline uint16_t MotorController_User_GetHeatMosfetsTop_DegCInt(MotorController_T * p_mc, uint8_t scalar)
{
	return Thermistor_ConvertToDegC_Int(&p_mc->ThermistorMosfetsTop, p_mc->AnalogResults.HeatMosfetsTop_ADCU, scalar);
}

static inline uint16_t MotorController_User_GetHeatMosfetsBot_DegCInt(MotorController_T * p_mc, uint8_t scalar)
{
	return Thermistor_ConvertToDegC_Int(&p_mc->ThermistorMosfetsBot, p_mc->AnalogResults.HeatMosfetsBot_ADCU, scalar);
}


static inline uint32_t MotorController_User_GetVPos_MilliV(MotorController_T * p_mc)	{return VMonitor_ConvertToMilliV(&p_mc->VMonitorPos, p_mc->AnalogResults.VPos_ADCU);}
static inline uint16_t MotorController_User_GetVSense_MilliV(MotorController_T * p_mc)	{return VMonitor_ConvertToMilliV(&p_mc->VMonitorSense, p_mc->AnalogResults.VSense_ADCU);}
static inline uint16_t MotorController_User_GetVAcc_MilliV(MotorController_T * p_mc)	{return VMonitor_ConvertToMilliV(&p_mc->VMonitorAcc, p_mc->AnalogResults.VAcc_ADCU);}

static inline void MotorController_User_SetVPosLimitUpper_MilliV(MotorController_T * p_mc, uint32_t limit) 		{VMonitor_SetLimitUpper_MilliV(&p_mc->VMonitorPos, limit);}
static inline void MotorController_User_SetVPosLimitLower_MilliV(MotorController_T * p_mc, uint32_t limit) 		{VMonitor_SetLimitLower_MilliV(&p_mc->VMonitorPos, limit);}
static inline void MotorController_User_SetVSenseLimitUpper_MilliV(MotorController_T * p_mc, uint32_t limit) 	{VMonitor_SetLimitUpper_MilliV(&p_mc->VMonitorSense, limit);}
static inline void MotorController_User_SetVSenseLimitLower_MilliV(MotorController_T * p_mc, uint32_t limit) 	{VMonitor_SetLimitLower_MilliV(&p_mc->VMonitorSense, limit);}
static inline void MotorController_User_SetVAccLimitUpper_MilliV(MotorController_T * p_mc, uint32_t limit) 		{VMonitor_SetLimitUpper_MilliV(&p_mc->VMonitorAcc, limit);}
static inline void MotorController_User_SetVAccLimitLower_MilliV(MotorController_T * p_mc, uint32_t limit) 		{VMonitor_SetLimitLower_MilliV(&p_mc->VMonitorAcc, limit);}

//static inline void MotorController_User_SetVPosLimitUpper_ADCU(MotorController_T * p_mc, uint16_t limit) {p_mc->Parameters.VPosLimitUpper_ADCU = limit;}
//static inline void MotorController_User_SetVPosLimitLower_ADCU(MotorController_T * p_mc, uint16_t limit) {p_mc->Parameters.VPosLimitLower_ADCU = limit;}
//static inline uint32_t MotorController_User_GetVPosLimitUpper_MilliV(MotorController_T * p_mc) 	{return p_mc->Parameters.VPosLimitUpper_ADCU;}
//static inline uint32_t MotorController_User_GetVPosLimitLower_MilliV(MotorController_T * p_mc) 	{return p_mc->Parameters.VPosLimitLower_ADCU;}
//static inline uint16_t MotorController_User_GetVPosLimitUpper_ADCU(MotorController_T * p_mc) 		{return p_mc->Parameters.VPosLimitUpper_ADCU;}
//static inline uint16_t MotorController_User_GetVPosLimitLower_ADCU(MotorController_T * p_mc) 		{return p_mc->Parameters.VPosLimitLower_ADCU;}

//static inline void MotorController_User_SetMainProtocolXcvr(MotorController_T * p_mc, uint8_t id) 	{  Protocol_SetSpecs(p_mc->MainProtocol, p_mc->CONFIG.P_PROTOCOL_SPECS_TABLE[id]); }
//static inline void MotorController_User_SetMainProtoclSpecs(MotorController_T * p_mc, uint8_t id) 	{  Protocol_SetPort(p_mc->MainProtocol, p_mc->CONFIG.P_XCVR_TABLE[id]); }



/*
 * Common Input Interface
 * Subject to StateMachine process
 * Need to save to temporary variable, as StateMachine functions pass 1 context variable only
 *
 * todo update function to pass through to motor state machine, eliminate redundancy
 * throttle, brake, direction,
 */
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

static inline void MotorController_User_SetDirection(MotorController_T * p_mc, MotorController_Direction_T direction)
{
	p_mc->UserDirection = direction;
	StateMachine_Semisynchronous_ProcInput(&p_mc->StateMachine, MCSM_INPUT_DIRECTION);
}

static inline void MotorController_User_StartCoast(MotorController_T * p_mc)
{
	StateMachine_Semisynchronous_ProcInput(&p_mc->StateMachine, MCSM_INPUT_START_COAST);

// 	if(p_mc->Parameters.CoastMode == MOTOR_CONTROLLER_COAST_MODE_REGEN)
// 	{
// 		//set regen
// 	}
// 	else
// 	{
// 		MotorController_DisableMotorAll(p_mc);
// 	}
}

static inline void MotorController_User_ProcCoast(MotorController_T * p_mc)
{
// 	if(p_mc->Parameters.CoastMode == MOTOR_CONTROLLER_COAST_MODE_REGEN)
// 	{
//
// 	}
// 	else
// 	{
//
// 	}

 	StateMachine_Semisynchronous_ProcInput(&p_mc->StateMachine, MCSM_INPUT_PROC_COAST);
}

static inline void MotorController_User_DisableControl(MotorController_T * p_mc)
{
	MotorController_DisableMotorAll(p_mc);
}

static inline void MotorController_User_SaveParameters_Blocking(MotorController_T * p_mc)
{
	StateMachine_Semisynchronous_ProcInput(&p_mc->StateMachine, MCSM_INPUT_SAVE_PARAMS);
}

/*
 * Input mode voluntarily call checked function, avoids intermediate buffer
 */
//static inline void MotorController_UserSerial_SetCmdValue(MotorController_T * p_mc, uint16_t userCmd) 	{ if (p_mc->Parameters.InputMode == MOTOR_CONTROLLER_INPUT_MODE_SERIAL) {p_mc->UserCmd = userCmd;} }
//static inline void MotorController_UserCanBus_SetCmdValue(MotorController_T * p_mc, uint16_t userCmd) 	{ if (p_mc->Parameters.InputMode == MOTOR_CONTROLLER_INPUT_MODE_CAN) {p_mc->UserCmd = userCmd;} }


/*
 * Motor Wrapper
 */
static inline Motor_T * MotorController_User_GetPtrMotor(const MotorController_T * p_mc, uint8_t motorIndex) {return &(p_mc->CONFIG.P_MOTORS[motorIndex]);}


#endif



//static inline uint16_t MotorController_User_GetMotorAdcu(MotorController_T * p_mc, uint8_t motorIndex, MotorAnalog_Channel_T adcChannel)
//{
//	return p_mc->CONFIG.P_MOTORS[motorIndex].AnalogResults.Channels[adcChannel];
//}
//
//static inline uint16_t MotorController_User_GetMotorAdcu_Msb8(MotorController_T * p_mc, uint8_t motorIndex, MotorAnalog_Channel_T adcChannel)
//{
//	return MotorController_User_GetMotorAdcu(p_mc, motorIndex, adcChannel) >> (CONFIG_MOTOR_CONTROLLER_ADCU_BITS_N - 8U);
//}
//
//static inline Hall_Sensors_T MotorController_User_GetHall(MotorController_T * p_mc, uint8_t motorIndex) 	{return Motor_User_GetHall(MotorController_GetPtrMotor(p_mc, motorIndex));}
//static inline bool MotorController_User_GetHallA(MotorController_T * p_mc, uint8_t motorIndex) 	{return Motor_User_GetHall(MotorController_GetPtrMotor(p_mc, motorIndex)).A;}
//static inline bool MotorController_User_GetHallB(MotorController_T * p_mc, uint8_t motorIndex) 	{return Motor_User_GetHall(MotorController_GetPtrMotor(p_mc, motorIndex)).B;}
//static inline bool MotorController_User_GetHallC(MotorController_T * p_mc, uint8_t motorIndex) 	{return Motor_User_GetHall(MotorController_GetPtrMotor(p_mc, motorIndex)).C;}
//
//
//static inline Motor_DirectionCalibration_T MotorController_User_GetDirectionCalibration(MotorController_T * p_mc, uint8_t motorIndex) {return Motor_User_GetDirectionCalibration(MotorController_GetPtrMotor(p_mc, motorIndex));}
//static inline bool MotorController_User_GetIsRevMode(MotorController_T * p_mc, uint8_t motorIndex) {return (Motor_User_GetDirectionCalibration(MotorController_GetPtrMotor(p_mc, motorIndex)) ==  MOTOR_FORWARD_IS_CW);}
//
////static inline uint32_t MotorController_User_GetBemfAvg_MilliV(MotorController_T * p_mc, uint8_t motorIndex)	{return Motor_User_GetBemf_Frac16(MotorController_GetPtrMotor(p_mc, motorIndex));}
//static inline uint16_t MotorController_User_GetSpeed_RPM(MotorController_T * p_mc, uint8_t motorIndex)		{return Motor_User_GetSpeed_RPM(MotorController_GetPtrMotor(p_mc, motorIndex));}
//static inline uint16_t MotorController_User_GetMotorHeat_DegCScalar(MotorController_T * p_mc, uint8_t motorIndex, uint8_t scalar) {return Motor_User_GetHeat_DegCScalar(MotorController_GetPtrMotor(p_mc, motorIndex), scalar);}
//
//static inline void MotorController_User_CalibrateHall(MotorController_T * p_mc, uint8_t motorIndex) {Motor_User_ActivateCalibrationHall(MotorController_GetPtrMotor(p_mc, motorIndex));}
//
//static inline void MotorController_User_SetPidIqKp(MotorController_T * p_mc, uint8_t motorIndex, uint16_t kpFactor)
//{
//	p_mc->CONFIG.P_MOTORS[motorIndex].PidIq.Params.KpFactor = kpFactor;
//}
