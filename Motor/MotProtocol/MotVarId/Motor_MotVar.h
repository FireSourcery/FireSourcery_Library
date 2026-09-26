#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2026 FireSourcery

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
    @file   Motor_MotVar.h
    @author FireSourcery
    @brief  [Motor_T] dispatch by [MotVarId_T] Type. Protocol-side routing to [Motor_Var] accessors.
*/
/******************************************************************************/
#include "Motor/Motor/Motor.h"
#include "MotVarId.h"

/*
    [Motor_T] routing by [MotVarId_T] Type nibble. Caller handles access control.
*/
extern int  Motor_VarType_Base_Get(Motor_T * p_motor, Motor_VarType_Base_T typeId, int varId);
extern void Motor_VarType_Base_Set(Motor_T * p_motor, Motor_VarType_Base_T typeId, int varId, int varValue);

extern int  Motor_VarType_SubModule_Get(Motor_T * p_motor, Motor_VarType_SubModule_T typeId, int varId);
extern void Motor_VarType_SubModule_Set(Motor_T * p_motor, Motor_VarType_SubModule_T typeId, int varId, int varValue);

extern int  Motor_VarType_Sensor_Get(Motor_T * p_motor, Motor_VarType_Sensor_T typeId, int varId);
extern void Motor_VarType_Sensor_Set(Motor_T * p_motor, Motor_VarType_Sensor_T typeId, int varId, int varValue);

/*
    [MotVarId_T] entry - routes the Motor Prefix block to the VarType dispatchers above.
    Caller resolves the [Motor_T] instance; NULL is reported as an invalid id.
*/
extern int               Motor_MotVar_Get(Motor_T * p_motor, MotVarId_T varId);
extern MotVarId_Status_T Motor_MotVar_Set(Motor_T * p_motor, MotVarId_T varId, int varValue);
