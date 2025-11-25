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
// #include "MotorController.h"
#include "Utility/StateMachine/StateMachine.h"
#include "Utility/StateMachine/_StateMachine_Tree.h"


/*
    All apps include independent AnalogUser handlers.
    Protocol freely maps,

    UserApp -> MotorController_App
    MotorControllerVar

*/

/* Part of MotorController */
struct MotorController; // forward declare
typedef const struct MotorController MotorController_T;

/******************************************************************************/
/*!
    Interface
*/
/******************************************************************************/
typedef const struct MotorController_App
{
    // void (*INIT)(const MotorController_T * p_context);
    // void (*PROC)(const MotorController_T * p_context);
    void (*PROC_ANALOG_USER)(MotorController_T * p_context);
    State_T * P_INITIAL_STATE;
    const void * P_APP_CONTEXT;
}
MotorController_App_T;


typedef const struct MotorController_AppTable
{
    MotorController_App_T MOTOR_CMD;
    MotorController_App_T VEHICLE;
}
MotorController_AppTable_T;


/******************************************************************************/
/*!
    App Table/Repo
*/
/******************************************************************************/
/* Operation Mode. Common as config and StateMachine Input */
typedef enum MotorController_MainMode
{
    MOTOR_CONTROLLER_MAIN_MODE_MOTOR_CMD,
    MOTOR_CONTROLLER_MAIN_MODE_VEHICLE,
}
MotorController_MainMode_T;

extern State_T * MotorController_App_MainStateOf(MotorController_MainMode_T mode);

extern State_T * MotorController_App_GetMainState(MotorController_T * p_context);

extern MotorController_App_T * MotorController_App_Get(MotorController_T * p_context);
