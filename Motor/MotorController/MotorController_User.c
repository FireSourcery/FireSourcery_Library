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
    @file   MotorController_User.c
    @author FireSourcery
    @brief

*/
/******************************************************************************/
#include "MotorController_User.h"


/******************************************************************************/
/*   */
/******************************************************************************/
/******************************************************************************/
/*
    Call via Key/Value, State/SubStates Cmds
    vars not stored in host view cache
*/
/******************************************************************************/
int MotorController_User_Call(const MotorController_T * p_context, MotorController_User_SystemCmd_T id, int value)
{
    MotorController_State_T * p_mc = p_context->P_ACTIVE;

    int status = 0; // MotorController_User_GenericStatus_T
    bool isSuccess = true;

    switch (id)
    {
        // case MOT_USER_SYSTEM_RESRV:                           break;
        case MOT_USER_SYSTEM_BEEP:          MotorController_BeepShort(p_context);                               break;
        // case MOT_USER_SYSTEM_BEEP:          Blinky_BlinkN(&p_context->BUZZER, 250U, 250U, 1U);                break;
        case MOT_USER_SYSTEM_BEEP_STOP:     MotorController_BeepStop(p_context);                                break; /* Stop active periodic. does not disable */

        case MOT_USER_SYSTEM_CLEAR_FAULT:   MotorController_StateMachine_ClearFault(p_context, value);          break;
        case MOT_USER_SYSTEM_FORCE_DISABLE_CONTROL: MotorController_User_ForceDisableControl(p_context);        break;

        // case MOT_USER_SYSTEM_DISABLE_CONTROL: MotorController_User_DisableControl(p_context);        break;

        /* Non Blocking function, host/caller poll Async return status after. */
        /* Blocking functions can directly return status. */
        /* MOTOR_CONTROLLER_LOCK_NVM_SAVE_CONFIG will block */
        case MOT_USER_SYSTEM_LOCK_STATE_INPUT:
            // MotorController_User_SetDirection(p_mc, MOT_DRIVE_DIRECTION_PARK);
            // _StateMachine_ProcAsyncInput(&p_mc->StateMachine, MOT_DRIVE_STATE_INPUT_DIRECTION, MOT_DRIVE_DIRECTION_PARK);
            // MotDrive_SetDirection(&p_mc->MotDrive, MOT_DRIVE_DIRECTION_PARK);
            // checks the park state
            MotorController_User_InputLock(p_context, (MotorController_LockId_T)value);
            if (MotorController_User_IsEnterLockError(p_context, (MotorController_LockId_T)value) == true) { MotorController_BeepShort(p_context); }

            status = MotorController_User_GetLockOpStatus(p_context);
            break;

        // LOCK_STATE_ACTIVCE
        //  (MotorController_LockId_T)
        case MOT_USER_SYSTEM_LOCK_STATE_STATUS:
            status = MotorController_User_GetLockState(p_context);
            break;

        case MOT_USER_SYSTEM_LOCK_ASYNC_STATUS: // union status, 0 as success
            // if (MotorController_User_IsLockOpComplete(p_context) == true)
            status = MotorController_User_GetLockOpStatus(p_context);
            break;

        case MOT_USER_SYSTEM_MAIN_MODE_INPUT:
            MotorController_User_InputMainMode(p_context, (MotorController_MainMode_T)value);
            status = 0;
            break;

        case MOT_USER_SYSTEM_RX_WATCHDOG:   MotorController_User_SetRxWatchdog(p_context, value);               break;

        // include for convenience
        // case MOT_USER_SYSTEM_STATE:
        // case MOT_USER_SYSTEM_PARK: isSuccess = MotorController_User_ProcDirection(p_mc, MOT_DRIVE_DIRECTION_PARK);
        // case MOT_USER_SYSTEM_SERVO: MotorController_User_InputServoMode(p_mc, (MotorController_ServoMode_T)value); break;
        default: break;
    }

    if (isSuccess == false) { status = -1; }
    return status;
}


/******************************************************************************/
/*
    Config
*/
/******************************************************************************/
/*! @param[in] volts < MOTOR_ANALOG_REFERENCE.VMAX and Config.VSupplyRef */
void MotorController_User_SetVSupplyRef(const MotorController_T * p_context, uint16_t volts)
{
    MotorController_State_T * p_mc = p_context->P_ACTIVE;
    p_mc->Config.VSupplyRef = math_min(volts, MotorAnalogRef_GetVRated_V());
    MotorController_ResetVSourceMonitorDefaults(p_context); /* may overwrite fault/warning if called in the same packet */
    MotorController_CaptureVSource(p_context); /* optionally */
}

// void MotorController_User_SetILimit_DC(const MotorController_T * p_context, uint16_t dc)
// {
// }

void MotorController_User_SetInputMode(const MotorController_T * p_context, MotorController_InputMode_T mode)
{
    MotorController_State_T * p_mc = p_context->P_ACTIVE;

    p_mc->Config.InputMode = mode;

    // switch (p_mc->Config.InputMode)
    // {
    //     case MOTOR_CONTROLLER_INPUT_MODE_ANALOG:
    //         // MotMotors_ForEach(&p_context->MOTORS, Motor_Var_Cmd_Disable);
    //         break;
    //     case MOTOR_CONTROLLER_INPUT_MODE_SERIAL:
    //         // MotMotors_ForEach(&p_context->MOTORS, Motor_Var_Cmd_Enable);
    //         break;
    //     case MOTOR_CONTROLLER_INPUT_MODE_CAN:
    //         break;
    //     default:  break;
    // }
}
