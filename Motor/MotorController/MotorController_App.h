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
#include "Utility/StateMachine/StateMachine.h"

// #define APP_SUBSTATE_ID(appId, subId) (() | ((appId) << 8U) | (subId))

/* Part of MotorController */
struct MotorController;
typedef const struct MotorController MotorController_T;

typedef void (*MotorController_App_Proc_T)(MotorController_T * p_mc);

/*
    All apps include independent AnalogUser handlers. Interpretation based on App handled separately from State
    Protocol freely maps,
*/
/******************************************************************************/
/*!
    Interface
    Around [MotorController_T] for StateMachine Access

    Includes Main substate.
    Common States including Park must be handled outside
*/
/******************************************************************************/
typedef const struct MotorController_App
{
    MotorController_App_Proc_T PROC_ANALOG_USER;
    MotorController_App_Proc_T INIT;
    State_Input_T ENTER_MAIN;
}
MotorController_App_T;


