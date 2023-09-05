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
    @file   MotorController_Var.h
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef MOTOR_CONTROLLER_VAR_H
#define MOTOR_CONTROLLER_VAR_H

#include "MotorController.h"
#include "MotVarId.h"

typedef enum MotPacket_Status_Tag
{
    MOT_VAR_STATUS_OK = 0x00U,
    MOT_VAR_STATUS_ERROR = 0x01U,
    MOT_VAR_STATUS_ERROR_READ_ONLY = 0x11U,
    MOT_VAR_STATUS_ERROR_INVALID_ID = 0x12U,
    MOT_VAR_STATUS_RESERVED = 0xFFU,
}
MotVar_Status_T;

typedef union { uint32_t Unsigned; int32_t Signed; } var32_t;
typedef union { uint16_t Unsigned; int16_t Signed; } var16_t;
typedef union { uint8_t Unsigned; int8_t Signed; } var8_t;

extern int32_t MotorController_Var_Get(const MotorController_T * p_mc, MotVarId_T varId);
extern MotVar_Status_T MotorController_Var_Set(MotorController_T * p_mc, MotVarId_T varId, int32_t varValue);

#endif