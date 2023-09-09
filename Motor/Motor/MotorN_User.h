/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

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
    @file   MotorN_User.h
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef MOTOR_N_USER_H
#define MOTOR_N_USER_H

#include "Motor_User.h"
#include <stdint.h>
#include <stdbool.h>

typedef void(*Motor_User_ProcVoid_T)(Motor_T * p_motor);
typedef void(*Motor_User_SetCmd_T)(Motor_T * p_motor, int16_t cmd);
typedef void(*Motor_User_SetId_T)(Motor_T * p_motor, uint32_t enumValue);
typedef bool(*Motor_User_ProcStatus_T)(Motor_T * p_motor );

static inline void MotorN_User_ProcFunction(Motor_T * p_motor, uint8_t motorCount, Motor_User_ProcVoid_T cmdFunction)
{
    for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { cmdFunction(&p_motor[iMotor]); }
}

static inline void MotorN_User_SetCmd(Motor_T * p_motor, uint8_t motorCount, Motor_User_SetCmd_T cmdFunction, int16_t cmdValue)
{
    for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { cmdFunction(&p_motor[iMotor], cmdValue); }
}

void MotorN_User_SetInt16(Motor_T * p_motor, uint8_t motorCount, Motor_User_SetCmd_T cmdFunction, int16_t cmdValue)
{
    for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++) { cmdFunction(&p_motor[iMotor], cmdValue); }
}

/******************************************************************************/
/*!
    Extern
*/
/******************************************************************************/
extern void MotorN_User_ApplyInt16(Motor_T * p_motor, uint8_t motorCount, Motor_User_SetCmd_T cmdFunction, int16_t cmdValue);
extern void MotorN_User_SetTorqueCmd(Motor_T * p_motor, uint8_t motorCount, int16_t cmdValue);
extern void MotorN_User_SetDefaultCmd(Motor_T * p_motor, uint8_t motorCount, int16_t cmdValue);
extern void MotorN_User_SetThrottleCmd(Motor_T * p_motor, uint8_t motorCount, uint16_t throttle);
extern void MotorN_User_SetBrakeCmd(Motor_T * p_motor, uint8_t motorCount, uint16_t brake);
// extern void MotorN_User_SetVoltageBrakeCmd(Motor_T * p_motor, uint8_t motorCount);
// extern void MotorN_User_SetCruise(Motor_T * p_motor, uint8_t motorCount);

extern bool MotorN_User_CheckStop(Motor_T * p_motor, uint8_t motorCount);
extern void MotorN_User_ReleaseControl(Motor_T * p_motor, uint8_t motorCount);
extern void MotorN_User_DisableControl(Motor_T * p_motor, uint8_t motorCount);
extern void MotorN_User_Ground(Motor_T * p_motor, uint8_t motorCount);
extern bool MotorN_User_SetDirectionForward(Motor_T * p_motor, uint8_t motorCount);
extern bool MotorN_User_SetDirectionReverse(Motor_T * p_motor, uint8_t motorCount);

extern bool MotorN_User_CheckFault(Motor_T * p_motor, uint8_t motorCount);
extern bool MotorN_User_ClearFault(Motor_T * p_motor, uint8_t motorCount);

extern void MotorN_User_SetSpeedLimitActive(Motor_T * p_motor, uint8_t motorCount, uint16_t limit_frac16);
extern void MotorN_User_ClearSpeedLimit(Motor_T * p_motor, uint8_t motorCount);
extern bool MotorN_User_SetILimitActive(Motor_T * p_motor, uint8_t motorCount, uint16_t limit_frac16, Motor_ILimitActiveId_T id);
extern bool MotorN_User_ClearILimit(Motor_T * p_motor, uint8_t motorCount, Motor_ILimitActiveId_T id);

#endif
