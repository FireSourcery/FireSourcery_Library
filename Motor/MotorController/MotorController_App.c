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
    @file   MotorController_App.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "MotorController_App.h"
#include "MotorController_StateMachine.h"
#include "MotorController.h"
#include "Utility/StateMachine/StateMachine.h"
#include "Utility/StateMachine/_StateMachine_Tree.h"


State_T * MotorController_App_MainStateOf(MotorController_MainMode_T mode)
{
    switch (mode)
    {
        case MOTOR_CONTROLLER_MAIN_MODE_MOTOR_CMD:  return &MC_STATE_MAIN_MOTOR_CMD;
        case MOTOR_CONTROLLER_MAIN_MODE_VEHICLE:    return &MC_STATE_MAIN_VEHICLE;
        default: return &MC_STATE_MAIN_MOTOR_CMD;
    }
}

State_T * MotorController_App_GetMainState(MotorController_T * p_context)
{
    return MotorController_App_MainStateOf(p_context->P_MC_STATE->Config.InitMode);
}

MotorController_App_T * MotorController_App_Get(MotorController_T * p_context)
{
    switch (p_context->P_MC_STATE->Config.InitMode)
    {
        case MOTOR_CONTROLLER_MAIN_MODE_MOTOR_CMD:  return &p_context->APPS.MOTOR_CMD;
        case MOTOR_CONTROLLER_MAIN_MODE_VEHICLE:    return &p_context->APPS.VEHICLE;
        default: return &p_context->APPS.MOTOR_CMD;
    }
}


