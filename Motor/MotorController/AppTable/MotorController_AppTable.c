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
// #include "Traction/MotorController_Traction.h"

#include "../MotorController_App.h"
// #include "MotorController_StateMachine.h"
#include "../MotorController_User.h"

/******************************************************************************/
/*!
    default App
*/
/******************************************************************************/
static inline void MotorCmdApp_ProcAnalogUser(const MotorController_T * p_dev)
{
    // switch (MotAnalogUser_GetDirectionEdge(&p_dev->ANALOG_USER))
    // {
    //     case MOT_ANALOG_USER_DIRECTION_FORWARD_EDGE:  MotorController_SetDirection(p_dev, MOTOR_DIRECTION_CCW);   break;
    //     case MOT_ANALOG_USER_DIRECTION_REVERSE_EDGE:  MotorController_SetDirection(p_dev, MOTOR_DIRECTION_CW);    break;
    //     case MOT_ANALOG_USER_DIRECTION_NEUTRAL_EDGE:  MotorController_SetDirection(p_dev, MOTOR_DIRECTION_NULL);      break;
    //     default: break;
    // }

    MotorController_SetCmdValue(p_dev, MotAnalogUser_GetThrottle(&p_dev->ANALOG_USER));
    // if (p_dev->P_MC->CmdInput.CmdValue == 0U)
    // {
    //     MotorController_SetControlState(p_dev, PHASE_VOUT_Z);
    // }
}

static State_T * EnterMain(const MotorController_T * p_mc, state_value_t fromPark)
{
    (void)p_mc; (void)fromPark;
    return &MC_STATE_MAIN_MOTOR_CMD;
}

MotorController_App_T MC_APP_MOTOR_CMD =
{
    .PROC_ANALOG_USER = MotorCmdApp_ProcAnalogUser,
    // .P_INITIAL_STATE = &MC_STATE_MAIN_MOTOR_CMD,
    .ENTER_MAIN = (State_Input_T)EnterMain,
};


/******************************************************************************/
/*!
    Singleton App handlers
*/
/******************************************************************************/
// MotorController_App_T * MotorController_App(MotorController_T * p_dev)
// {
//     switch (p_dev->P_MC->Config.InitMode)
//     {
//         case MOTOR_CONTROLLER_MAIN_MODE_MOTOR_CMD:  return &MC_APP_MOTOR_CMD;
//         case MOTOR_CONTROLLER_MAIN_MODE_TRACTION:    return &MC_APP_TRACTION;
//         default: return &MC_APP_MOTOR_CMD;
//     }
// }

// // alternatively need subid scheme
// MotorController_MainMode_T MotorController_App_GetActiveMode(MotorController_T * p_dev)
// {
//     if (StateMachine_IsActiveBranch(p_dev->STATE_MACHINE.P_ACTIVE, &MC_STATE_MAIN_MOTOR_CMD)) { return MOTOR_CONTROLLER_MAIN_MODE_MOTOR_CMD; }
//     // else if (StateMachine_IsActiveBranch(p_dev->STATE_MACHINE.P_ACTIVE, &MC_STATE_MAIN_TRACTION)) { return MOTOR_CONTROLLER_MAIN_MODE_TRACTION; }
//     return -1;
// }


// State_T * MotorController_App_EnterMain(MotorController_T * p_dev) { return MotorController_App(p_dev)->ENTER_MAIN((void *)p_dev, 0); }

// void MotorController_App_ProcAnalogUser(MotorController_T * p_dev) { return MotorController_App(p_dev)->PROC_ANALOG_USER(p_dev); }


// void MotorController_AppTableVar(MotorController_T * p_dev) {
//    case MOT_VAR_TYPE_TRACTION_CONTROL:     return MotorController_Traction_VarId_Get(p_dev, varId.Base);
//         case MOT_VAR_TYPE_TRACTION_CONFIG:      return MotorController_Traction_ConfigId_Get(p_dev, varId.Base);
//      }