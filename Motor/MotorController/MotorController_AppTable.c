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
    @file   MotorController_App.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "MotorController_AppTable.h"
#include "Vehicle/MotorController_Vehicle.h"

#include "MotorController_App.h"
// #include "MotorController_StateMachine.h"
#include "MotorController_User.h"

/******************************************************************************/
/*!
    default App
*/
/******************************************************************************/
static inline void MotorCmdApp_ProcAnalogUser(const MotorController_T * p_context)
{
    switch (MotAnalogUser_GetDirectionEdge(&p_context->ANALOG_USER))
    {
        case MOT_ANALOG_USER_DIRECTION_FORWARD_EDGE:  MotorController_SetDirection(p_context, MOTOR_DIRECTION_CCW);   break;
        case MOT_ANALOG_USER_DIRECTION_REVERSE_EDGE:  MotorController_SetDirection(p_context, MOTOR_DIRECTION_CW);   break;
        case MOT_ANALOG_USER_DIRECTION_NEUTRAL_EDGE:  MotorController_SetDirection(p_context, MOTOR_DIRECTION_NULL);      break;
        default: break;
    }

    MotorController_SetCmdValue(p_context, MotAnalogUser_GetThrottle(&p_context->ANALOG_USER));
    // if (p_context->P_MC_STATE->CmdInput.CmdValue == 0U)
    // {
    //     MotorController_SetControlState(p_context, PHASE_OUTPUT_FLOAT);
    // }
}

MotorController_App_T MC_APP_MOTOR_CMD =
{
    .PROC_ANALOG_USER = MotorCmdApp_ProcAnalogUser,
    .P_INITIAL_STATE = &MC_STATE_MAIN_MOTOR_CMD,
};



/******************************************************************************/
/*!
    Singleton App handlers
*/
/******************************************************************************/
MotorController_App_T * MotorController_App_Get(MotorController_T * p_context)
{
    switch (p_context->P_MC_STATE->Config.InitMode)
    {
        case MOTOR_CONTROLLER_MAIN_MODE_MOTOR_CMD:  return &MC_APP_MOTOR_CMD;
        case MOTOR_CONTROLLER_MAIN_MODE_VEHICLE:    return &MC_APP_VEHICLE;
        default: return &MC_APP_MOTOR_CMD;
    }
}

State_T * MotorController_App_GetMainState(MotorController_T * p_context) { return MotorController_App_Get(p_context)->P_INITIAL_STATE; }

MotorController_App_Proc_T MotorController_App_GetAnalogUserProc(MotorController_T * p_context) { return MotorController_App_Get(p_context)->PROC_ANALOG_USER; }