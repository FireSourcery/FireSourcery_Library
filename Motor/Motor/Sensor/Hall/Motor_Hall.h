#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery

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
    @file   Motor_Hall.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Hall.h"

#include "../../Motor_StateMachine.h"
#include "../../Motor.h"
#include "../../Motor_Config.h"
// #include "../../Motor_FOC.h"

#include "Utility/Var/VarAccess.h"

extern void Motor_Hall_Calibrate(const Motor_T * p_motor);

extern int32_t _Motor_VarConfig_Hall_Get(const Motor_State_T * p_motor, Hall_ConfigId_T varId);
extern void _Motor_VarConfig_Hall_Set(Motor_State_T * p_motor, Hall_ConfigId_T varId, int32_t varValue);

extern int Motor_VarConfig_Hall_Get(const Motor_State_T * p_motor, int varId);
extern void Motor_VarConfig_Hall_Set(Motor_State_T * p_motor, int varId, int varValue);

extern const VarAccess_VTable_T MOTOR_VAR_CONFIG_HALL;


#define MOTOR_HALL_VAR_ACCESS_INIT(p_Motor) VAR_ACCESS_INIT(p_Motor, &MOTOR_VAR_CONFIG_HALL, &((p_Motor)->VarAccessOuputState))
