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

#include "Utility/StateMachine/StateMachine.h"
//#include "Motor/Utility/MotMemMap/MotMemMap.h"
#include "Motor/Motor/Motor.h"

#include <stdint.h>

static inline uint16_t MotorController_User_GetThrottle_Adcu(MotorController_T * p_mc)		{return p_mc->AnalogResults.Throttle_ADCU;}
static inline uint16_t MotorController_User_GetThrottle_Adcu8(MotorController_T * p_mc)		{return MotorController_User_GetThrottle_Adcu(p_mc) >> (CONFIG_MOTOR_CONTROLLER_ADCU_BITS_N - 8U);}
static inline uint16_t MotorController_User_GetThrottle_Frac16(MotorController_T * p_mc)	{return MotAnalogUser_GetThrottle(&p_mc->AnalogUser);}
static inline uint16_t MotorController_User_GetThrottle_Frac8(MotorController_T * p_mc)		{return MotAnalogUser_GetThrottle(&p_mc->AnalogUser) >> 8U;}
static inline uint16_t MotorController_User_GetThrottleValue(MotorController_T * p_mc)		{return MotAnalogUser_GetThrottle(&p_mc->AnalogUser);}
static inline bool MotorController_User_GetThrottleSwitch(MotorController_T * p_mc) 		{return MotAnalogUser_GetThrottleSwitch(&p_mc->AnalogUser);}

static inline uint16_t 	MotorController_User_GetCmdValue(MotorController_T * p_mc)	{return p_mc->UserCmd;}

static inline uint32_t MotorController_User_GetBemf_Frac16(MotorController_T * p_mc, uint8_t motorIndex)	{return Motor_User_GetBemf_Frac16(MotorController_GetPtrMotor(p_mc, motorIndex));}
static inline uint32_t MotorController_User_GetVPos_MilliV(MotorController_T * p_mc, uint8_t motorIndex)	{return Motor_User_GetVPos_MilliV(MotorController_GetPtrMotor(p_mc, motorIndex));}
static inline uint32_t MotorController_User_GetSpeed_RPM(MotorController_T * p_mc, uint8_t motorIndex)		{return Motor_User_GetSpeed_RPM(MotorController_GetPtrMotor(p_mc, motorIndex));}

//uint16_t MotorController_User_GetIa_Adcu(MotorController_T * p_mc, uint8_t motorIndex)		{return  ;}
//uint16_t MotorController_User_GetIa_Frac16(MotorController_T * p_mc, uint8_t motorIndex)	{return  ;}
//uint16_t MotorController_User_GetIa_Frac8(MotorController_T * p_mc, uint8_t motorIndex)	{return MotorController_GetPtrMotor(p_mc, motorIndex);}

/*
 * Nvm Saved Functions
 */
//static inline uint8_t MotorController_User_GetNameChar(MotorController_T * p_mc, uint8_t charId) {return MOT_MEM_ONCE->NAME[charId];}

//static inline uint16_t MotorController_User_GetVLimitUpper(MotorController_T * p_mc) 	{return p_mc->AnalogMonitor.Params.VPosLimitUpper_V;}
//static inline uint16_t MotorController_User_GetVLimitLower(MotorController_T * p_mc) 	{return p_mc->AnalogMonitor.Params.VPosLimitLower_V;}
//
//static inline void MotorController_User_SetVLimitUpper(MotorController_T * p_mc, uint16_t limit) {p_mc->AnalogMonitor.Params.VPosLimitUpper_V = limit;}
//static inline void MotorController_User_SetVLimitLower(MotorController_T * p_mc, uint16_t limit) {p_mc->AnalogMonitor.Params.VPosLimitLower_V = limit;}
//
////can use macro
//#define SET_VAR16_BYTE_HIGH(var, valueByte) 	(var = (var | 0x0FU) & (valueByte << 8U))
//#define SET_VAR16_BYTE_LOW(var, valueByte) 		(var = (var | 0xF0U) & valueByte)
//
//static inline void MotorController_User_SetVLimitUpperByteHigh(MotorController_T * p_mc, uint8_t limitByteHigh) {p_mc->AnalogMonitor.Params.VPosLimitUpper_V = (p_mc->AnalogMonitor.Params.VPosLimitUpper_V | 0x0F) & (limitByteHigh << 8U);}
//static inline void MotorController_User_SetVLimitUpperByteLow(MotorController_T * p_mc, uint8_t limitByteLow) 	{p_mc->AnalogMonitor.Params.VPosLimitUpper_V = (p_mc->AnalogMonitor.Params.VPosLimitUpper_V | 0xF0) & limitByteLow;}


/*
 * Common Input Interface
 * Subject to StateMachine process
 * Need to save to temporary buffer, as StateMachine functions pass 1 context variable only
 */
static inline void MotorController_User_SetCmdValue(MotorController_T * p_mc, uint16_t userCmd) {p_mc->UserCmd = userCmd;}
static inline void MotorController_User_SetDirection(MotorController_T * p_mc, MotorController_Direction_T direction) {p_mc->UserDirection = direction;}

static inline void MotorController_User_ProcCmdAccelerate(MotorController_T * p_mc, uint16_t userCmd) {p_mc->UserCmd = userCmd; StateMachine_Semisynchronous_ProcInput(&p_mc->StateMachine, MCSM_INPUT_ACCELERATE);}
static inline void MotorController_User_ProcCmdDecelerate(MotorController_T * p_mc, uint16_t userCmd) {p_mc->UserCmd = userCmd; StateMachine_Semisynchronous_ProcInput(&p_mc->StateMachine, MCSM_INPUT_DECELERATE);}

static inline void MotorController_User_ProcDirection(MotorController_T * p_mc, MotorController_Direction_T direction)
{
	if (direction != p_mc->MainDirection)
	{
		p_mc->UserDirection = direction;
		StateMachine_Semisynchronous_ProcInput(&p_mc->StateMachine, MCSM_INPUT_DIRECTION);
	}
}

static inline void MotorController_User_ProcDisableControl(MotorController_T * p_mc)
{
//	if(p_mc->UserCmd > 0U)
//	{
//		p_mc->UserCmd = 0U;
		MotorController_DisableMotorAll(p_mc); //proc on edge only
//	}
}


static inline void MotorController_User_ProcSaveParameters(MotorController_T * p_mc) { StateMachine_Semisynchronous_ProcInput(&p_mc->StateMachine, MCSM_INPUT_SAVE_PARAMS);}

static inline MotorController_InputMode_T MotorController_User_GetInputMode(MotorController_T * p_mc) 	{ return p_mc->Parameters.InputMode; }

static inline void MotorController_UserSerial_SetCmdValue(MotorController_T * p_mc, uint16_t userCmd) 	{ if (p_mc->Parameters.InputMode == MOTOR_CONTROLLER_INPUT_MODE_SERIAL) {p_mc->UserCmd = userCmd;} }
static inline void MotorController_UserCanBus_SetCmdValue(MotorController_T * p_mc, uint16_t userCmd) 	{ if (p_mc->Parameters.InputMode == MOTOR_CONTROLLER_INPUT_MODE_CAN) {p_mc->UserCmd = userCmd;} }


#endif
