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
// #include "MotorController.h"
#include "Vehicle/MotorController_Vehicle.h"

#include "Utility/StateMachine/StateMachine.h"
#include "Utility/StateMachine/_StateMachine_Tree.h"

/* part of MotorController */
/* Part of MotorController */
struct MotorController; // forward declare
typedef const struct MotorController MotorController_T;


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

// typedef const struct MotorController_AppTable
// {
//     MotorController_App_T MOTOR_CMD;
//     MotorController_App_T VEHICLE;
// }
// MotorController_AppTable_T;

extern MotorController_App_T * MotorController_App_Get(MotorController_T * p_context);

extern State_T * MotorController_App_GetMainState(MotorController_T * p_context);
extern MotorController_App_Proc_T MotorController_App_GetProcAnalogUser(MotorController_T * p_context);