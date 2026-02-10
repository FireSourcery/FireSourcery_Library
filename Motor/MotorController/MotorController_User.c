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
int MotorController_CallSystemCmd(const MotorController_T * p_context, MotorController_SystemCmd_T id, int value)
{
    MotorController_State_T * p_mc = p_context->P_MC_STATE;

    int status = 0; // MotorController_GenericStatus_T
    bool isSuccess = true;

    switch (id)
    {
        // case MOT_USER_SYSTEM_RESRV:                           break;
        case MOT_USER_SYSTEM_BEEP:          MotorController_BeepShort(p_context);                               break;
        // case MOT_USER_SYSTEM_BEEP:          Blinky_BlinkN(&p_context->BUZZER, 250U, 250U, value);                break;
        case MOT_USER_SYSTEM_BEEP_STOP:     MotorController_BeepStop(p_context);                                break; /* Stop active periodic. does not disable */

        case MOT_USER_SYSTEM_CLEAR_FAULT:   MotorController_ClearFault(p_context, value);                   break;
        case MOT_USER_SYSTEM_FORCE_DISABLE_CONTROL: MotorController_ForceDisableControl(p_context);         break;
        // case MOT_USER_SYSTEM_DISABLE_CONTROL: MotorController_DisableControl(p_context);        break;

        /* Non Blocking function, host/caller poll Async return status after. */
        /* Blocking functions can directly return status. */
        /* MOTOR_CONTROLLER_LOCK_NVM_SAVE_CONFIG will block */
        case MOT_USER_SYSTEM_LOCK_STATE_INPUT:
            // checks the park state
            MotorController_InputLock(p_context, (MotorController_LockId_T)value);
            if (MotorController_IsEnterLockError(p_context, (MotorController_LockId_T)value) == true) { MotorController_BeepShort(p_context); }

            status = MotorController_GetLockOpStatus(p_context);
            break;

        case MOT_USER_SYSTEM_LOCK_STATE_STATUS:
            status = MotorController_GetLockState(p_context);
            break;

        case MOT_USER_SYSTEM_LOCK_ASYNC_STATUS: // union status, 0 as success
            // if (MotorController_IsLockOpComplete(p_context) == true)
            status = MotorController_GetLockOpStatus(p_context);
            break;

        case MOT_USER_SYSTEM_STATE_COMMAND:
            MotorController_InputStateCommand(p_context, (MotorController_StateCmd_T)value);
            break;

        // case MOT_USER_SYSTEM_DIRECTION_COMMAND:
        //     MotorController_ApplyDirectionCmd(p_context, (int)value);
        //     break;

        // case MOT_USER_SYSTEM_MAIN_MODE_INPUT:
        //     MotorController_InputMainMode(p_context, (MotorController_MainMode_T)value);
        //     status = 0;
        //     break;

        case MOT_USER_SYSTEM_RX_WATCHDOG:   MotorController_SetRxWatchdog(p_context, value);               break;

        // include for convenience
        // case MOT_USER_SYSTEM_STATE:
        // case MOT_USER_SYSTEM_PARK: isSuccess = MotorController_ProcDirection(p_mc, MOTOR_CONTROLLER_DIRECTION_PARK);
        // case MOT_USER_SYSTEM_SERVO: MotorController_InputServoMode(p_mc, (MotorController_ServoMode_T)value); break;
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
/*! @param[in] volts < PHASE_CALIBRATION.VMAX and Config.VSupplyRef */
void MotorController_SetVSupplyRef(const MotorController_T * p_context, uint16_t volts)
{
    MotorController_State_T * p_mc = p_context->P_MC_STATE;
    p_mc->Config.VSupplyRef = math_min(volts, Phase_Calibration_GetVRated_V());
    MotorController_ResetVSourceMonitorDefaults(p_context);
    /* todo as bound limits */
    /* may overwrite fault/warning if called in the same packet */
}

// void MotorController_SetILimit_DC(const MotorController_T * p_context, uint16_t dc)
// {
// }

void MotorController_SetInputMode(const MotorController_T * p_context, MotorController_InputMode_T mode)
{
    MotorController_State_T * p_mc = p_context->P_MC_STATE;

    p_mc->Config.InputMode = mode;

    // switch (p_mc->Config.InputMode)
    // {
    //     case MOTOR_CONTROLLER_INPUT_MODE_ANALOG:    //         break;
    //     case MOTOR_CONTROLLER_INPUT_MODE_SERIAL:    //         break;
    //     case MOTOR_CONTROLLER_INPUT_MODE_CAN:    //         break;
    //     default:  break;
    // }
}

// static inline bool _Vehicle_ProcOnDirection(const Vehicle_T * p_vehicle, Vehicle_Direction_T direction)
// {
//     // if((p_vehicle->P_VEHICLE_STATE->Config.BuzzerFlagsEnable.OnReverse == true))
//     // {
//     //     if(p_this->DriveDirection == MOTOR_DIRECTION_CW)
//     //     {
//     //         Vehicle_BeepPeriodicType1(p_this);
//     //     }
//     //     else
//     //     {
//     //         Blinky_Stop(&p_this->Buzzer);
//     //     }
//     // }
// }
    // if (Vehicle_StateMachine_GetDirection(p_vehicle) != direction)
    // {
    // Vehicle_User_CheckDirection(p_vehicle, direction); /* effective on motor async transition only */
    // bool isSuccess = (Vehicle_User_GetDirection(p_vehicle) == direction);
        // if (isSuccess == false) { Blinky_Blink(p_vehicle->P_BUZZER, 500U); }
    // }
/* optionally host side implement */
// /* Separate Check direction with alarm, so Motor set can use SetSyncInput */
// bool Vehicle_User_CheckDirection(const Vehicle_T * p_vehicle, sign_t direction)
// {
//     bool isSuccess = (Vehicle_User_GetDirection(p_vehicle) == direction);
//     if (isSuccess == false) { Blinky_Blink(p_vehicle->P_BUZZER, 500U); }
//     return isSuccess;
// }
