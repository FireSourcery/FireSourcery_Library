/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2021 FireSourcery / The Firebrand Forge Inc

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
    @file     MotorController_Var.h
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef MOTOR_CONTROLLER_VAR_H
#define MOTOR_CONTROLLER_VAR_H

#include "MotorController.h"
#include "Motor/MotProtocol/MotVarId.h"

extern int32_t MotorController_Var_Get(const MotorController_T * p_mc, MotVarId_T varId);
extern uint8_t MotorController_Var_Set(MotorController_T * p_mc, MotVarId_T varId, uint32_t varValue);

#endif