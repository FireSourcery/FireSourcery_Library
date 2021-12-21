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

#include "MotorController.h"
#include "Config.h"

#include "Motor/Utility/MotMemMap/MotMemMap.h"

#include "Motor/Motor/Motor.h"

#include <stdint.h>

static inline uint16_t MotorController_User_GetThrottle_Adcu(MotorController_T * p_motorController)	{return p_motorController->AnalogResults[MOTOR_ANALOG_CHANNEL_THROTTLE];}
static inline uint16_t MotorController_User_GetThrottle_Adcu8(MotorController_T * p_motorController)	{return MotorController_User_GetThrottle_Adcu(p_motorController) >> (CONFIG_MOTOR_CONTROLLER_ADCU_BITS_N - 8U);}
static inline uint16_t MotorController_User_GetThrottle_Frac16(MotorController_T * p_motorController)	{return p_motorController->AnalogUser.InputValueThrottle;}
static inline uint16_t MotorController_User_GetThrottle_Frac8(MotorController_T * p_motorController)	{return MotorController_User_GetThrottle_Frac16(p_motorController) >> 8U;}


static inline void MotorController_User_SetThrottleAll(MotorController_T * p_motorController, uint16_t throttle)
{
	Motor_User_SetNThrottle((MotorController_GetPtrMotor(p_motorController, 0U)), p_motorController->CONFIG.MOTOR_COUNT, throttle);
}

static inline void MotorController_User_SetThrottleMotor(MotorController_T * p_motorController, uint8_t motorIndex, uint16_t throttle)
{
	Motor_User_SetThrottle((MotorController_GetPtrMotor(p_motorController, motorIndex)), throttle);
}

//MotorController_User_GetMotorSpeed_RPM

static inline uint32_t MotorController_User_GetBemf_Frac16(MotorController_T * p_motorController, uint8_t motorIndex)	{return Motor_User_GetBemf_Frac16(MotorController_GetPtrMotor(p_motorController, motorIndex));}
static inline uint32_t MotorController_User_GetVPos_MilliV(MotorController_T * p_motorController, uint8_t motorIndex)	{return Motor_User_GetVPos_MilliV(MotorController_GetPtrMotor(p_motorController, motorIndex));}
static inline uint32_t MotorController_User_GetSpeed_RPM(MotorController_T * p_motorController, uint8_t motorIndex)	{return Motor_User_GetSpeed_RPM(MotorController_GetPtrMotor(p_motorController, motorIndex));}

//uint16_t MotorController_User_GetIa_Adcu(MotorController_T * p_motorController, uint8_t motorIndex)		{return  ;}
//uint16_t MotorController_User_GetIa_Frac16(MotorController_T * p_motorController, uint8_t motorIndex)	{return  ;}
//uint16_t MotorController_User_GetIa_Frac8(MotorController_T * p_motorController, uint8_t motorIndex)	{return MotorController_GetPtrMotor(p_motorController, motorIndex);}


static inline uint16_t 	MotorController_User_GetThrottleValue(MotorController_T * p_motorController)	{return MotAnalogUser_GetThrottleValue(&p_motorController->AnalogUser);}
static inline bool 		MotorController_User_GetThrottleSwitch(MotorController_T * p_motorController) 	{return MotAnalogUser_GetThrottleSwitch(&p_motorController->AnalogUser);}


/*
 * Flash Data Functions
 */
static inline uint8_t MotorController_User_GetNameChar(MotorController_T * p_motorController, uint8_t charId) {return MOT_MEM_ONCE->NAME[charId];}

static inline uint16_t MotorController_User_GetVLimitUpper(MotorController_T * p_motorController) 	{return p_motorController->AnalogMonitor.Params.VPosLimitUpperV;}
static inline uint16_t MotorController_User_GetVLimitLower(MotorController_T * p_motorController) 	{return p_motorController->AnalogMonitor.Params.VPosLimitLowerV;}

static inline void MotorController_User_SetVLimitUpper(MotorController_T * p_motorController, uint16_t limit) {p_motorController->AnalogMonitor.Params.VPosLimitUpperV = limit;}
static inline void MotorController_User_SetVLimitLower(MotorController_T * p_motorController, uint16_t limit) {p_motorController->AnalogMonitor.Params.VPosLimitLowerV = limit;}

//can use macro
#define SET_VAR16_BYTE_HIGH(var, valueByte) 	(var = (var | 0x0FU) & (valueByte << 8U))
#define SET_VAR16_BYTE_LOW(var, valueByte) 		(var = (var | 0xF0U) & valueByte)

static inline void MotorController_User_SetVLimitUpperByteHigh(MotorController_T * p_motorController, uint8_t limitByteHigh) {p_motorController->AnalogMonitor.Params.VPosLimitUpperV = (p_motorController->AnalogMonitor.Params.VPosLimitUpperV | 0x0F) & (limitByteHigh << 8U);}
static inline void MotorController_User_SetVLimitUpperByteLow(MotorController_T * p_motorController, uint8_t limitByteLow) {p_motorController->AnalogMonitor.Params.VPosLimitUpperV = (p_motorController->AnalogMonitor.Params.VPosLimitUpperV | 0xF0) & limitByteLow;}


static inline void MotorController_User_SaveParameters_Blocking(MotorController_T * p_motorController)
{
	for(uint8_t iMotor = 0U; iMotor < p_motorController->CONFIG.MOTOR_COUNT; iMotor++)
	{
		EEPROM_Write_Blocking(&p_motorController->Eeprom, MotorController_GetPtrMotor(p_motorController, iMotor)->CONFIG.P_PARAMETERS, &MotorController_GetPtrMotor(p_motorController, iMotor)->Parameters, sizeof(Motor_Parameters_T));

	}

	EEPROM_Write_Blocking(&p_motorController->Eeprom, p_motorController->AnalogMonitor.CONFIG.P_PARAMS, &p_motorController->AnalogMonitor.Params, sizeof(MotAnalogMonitor_Params_T));
//	EEPROM_Write_Blocking(&p_motorController->Eeprom, p_motorController->AnalogMonitor.CONFIG.P_PARAMS, &p_motorController->AnalogMonitor.Params, sizeof(MotAnalogMonitor_Params_T));

}

#endif
