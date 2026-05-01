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
/*
    Call State/SubStates Cmds
    vars not stored in host view cache
*/
/******************************************************************************/
int MotorController_CallSystemCmd(const MotorController_T * p_dev, MotorController_SystemCmd_T id, int value)
{
    MotorController_State_T * p_mc = p_dev->P_MC;

    int status = 0; // MotorController_GenericStatus_T
    bool isSuccess = true;

    switch (id)
    {
        case MOT_USER_SYSTEM_BEEP:          MotBuzzer_Short(MotorController_Buzzer(p_dev));          break;
        case MOT_USER_SYSTEM_BEEP_STOP:     MotBuzzer_Stop(MotorController_Buzzer(p_dev));           break;
        case MOT_USER_SYSTEM_CLEAR_FAULT:           MotorController_ClearFault(p_dev, (MotorController_FaultFlags_T) { .Value = value });    break;
        case MOT_USER_SYSTEM_FORCE_DISABLE_CONTROL: MotorController_ForceDisableControl(p_dev);         break;
        /* Blocking functions can directly return status. */
        /* MOTOR_CONTROLLER_LOCK_NVM_SAVE_CONFIG will block */
        /* Non Blocking function, host/caller poll Async return status after. */
        case MOT_USER_SYSTEM_LOCK_STATE_INPUT:
            MotorController_InputLock(p_dev, (MotorController_LockId_T)value);
            if (MotorController_IsEnterLockError(p_dev, (MotorController_LockId_T)value) == true) { MotBuzzer_Short(MotorController_Buzzer(p_dev)); }
            status = MotorController_GetLockOpStatus(p_dev);
            break;

        case MOT_USER_SYSTEM_LOCK_STATE_STATUS:     status = MotorController_IsLockOpComplete(p_dev);               break;
        case MOT_USER_SYSTEM_LOCK_ASYNC_STATUS:     status = MotorController_GetLockOpStatus(p_dev);            break;
        case MOT_USER_SYSTEM_STATE_COMMAND:         MotorController_InputStateCommand(p_dev, (MotorController_StateCmd_T)value);            break;
        case MOT_USER_SYSTEM_RX_WATCHDOG:           MotorController_SetRxWatchdog(p_dev, value);               break;
        // case MOT_USER_SYSTEM_DIRECTION_COMMAND:
        //     MotorController_ApplyDirectionCmd(p_dev, (int)value);
        //     break;
        // case MOT_USER_SYSTEM_MAIN_MODE_INPUT:
        //     MotorController_InputMainMode(p_dev, (MotorController_MainMode_T)value);
        //     break;
        default: break;
    }

    if (isSuccess == false) { status = -1; }
    return status;
}

/* optionally host side implement */
/* Separate Check direction with alarm, so Motor set can use SetSyncInput */
/* effective on motor async transition only */
bool MotorController_CheckDirection(MotorController_T * p_dev, sign_t direction)
{
    if (MotorController_GetDirection(p_dev) != direction) { Blinky_Blink(&p_dev->BUZZER, 500U); return false; }
    return true;
}


