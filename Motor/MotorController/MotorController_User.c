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
*/
/******************************************************************************/
bool MotorController_User_SetSpeedLimitAll(const MotorController_T * p_context, uint16_t limit_fract16)
{
    LimitArray_SetEntry(&p_context->MOT_SPEED_LIMITS, MOT_SPEED_LIMIT_USER, limit_fract16);
    MotMotors_ApplySpeedLimit(&p_context->MOTORS, &p_context->MOT_SPEED_LIMITS);
}

bool MotorController_User_ClearSpeedLimitAll(const MotorController_T * p_context)
{
    LimitArray_ClearEntry(&p_context->MOT_SPEED_LIMITS, MOT_SPEED_LIMIT_USER);
    MotMotors_ApplySpeedLimit(&p_context->MOTORS, &p_context->MOT_SPEED_LIMITS);
}

bool MotorController_User_SetILimitAll(const MotorController_T * p_context, uint16_t limit_fract16)
{
    LimitArray_SetEntry(&p_context->MOT_I_LIMITS, MOT_I_LIMIT_USER, limit_fract16);
    MotMotors_ApplyILimit(&p_context->MOTORS, &p_context->MOT_I_LIMITS);
}

bool MotorController_User_ClearILimitAll(const MotorController_T * p_context)
{
    LimitArray_ClearEntry(&p_context->MOT_I_LIMITS, MOT_I_LIMIT_USER);
    MotMotors_ApplyILimit(&p_context->MOTORS, &p_context->MOT_I_LIMITS);
}

/******************************************************************************/
/*   */
/******************************************************************************/
void MotorController_User_SetOptSpeedLimitOnOff(const MotorController_T * p_context, bool isEnable)
{
    if (isEnable == true) { MotorController_User_SetSpeedLimitAll(p_context, p_context->P_ACTIVE->Config.OptSpeedLimit_Fract16); }
    else { MotorController_User_ClearSpeedLimitAll(p_context); }
}

void MotorController_User_SetOptILimitOnOff(const MotorController_T * p_context, bool isEnable)
{
    if (isEnable == true) { MotorController_User_SetILimitAll(p_context, p_context->P_ACTIVE->Config.OptILimit_Fract16); }
    else { MotorController_User_ClearILimitAll(p_context); }
}

/******************************************************************************/
/* Config */
/******************************************************************************/
/*! @param[in] volts < MOTOR_ANALOG_REFERENCE.VMAX and Config.VSupplyRef */
void MotorController_User_SetVSupplyRef(const MotorController_T * p_context, uint16_t volts)
{
    MotorController_State_T * p_mc = p_context->P_ACTIVE;
    // p_mc->Config.VSupplyRef = Motor_VSourceLimitOf(volts);
    p_mc->Config.VSupplyRef = volts;
    MotorController_ResetVSourceMonitorDefaults(p_context);
    MotorController_CaptureVSource(p_context);
}

// void MotorController_User_SetILimit_DC(const MotorController_T * p_context, uint16_t dc)
// {
// }

void MotorController_User_SetInputMode(const MotorController_T * p_context, MotorController_InputMode_T mode)
{
    MotorController_State_T * p_mc = p_context->P_ACTIVE;

    p_mc->Config.InputMode = mode;

    switch (p_mc->Config.InputMode)
    {
        case MOTOR_CONTROLLER_INPUT_MODE_ANALOG:
            // MotMotors_ForEach(&p_context->MOTORS, Motor_Var_Cmd_Disable);
            for (uint8_t iMotor = 0U; iMotor < p_context->MOTORS.LENGTH; iMotor++)
            {
                Motor_Var_DisableInput(MotMotors_StateAt(&p_context->MOTORS, iMotor));
            }
            break;
        case MOTOR_CONTROLLER_INPUT_MODE_SERIAL:
            // MotMotors_ForEach(&p_context->MOTORS, Motor_Var_Cmd_Enable);
            for (uint8_t iMotor = 0U; iMotor < p_context->MOTORS.LENGTH; iMotor++)
            {
                Motor_Var_EnableInput(MotMotors_StateAt(&p_context->MOTORS, iMotor));
            }
            break;
        case MOTOR_CONTROLLER_INPUT_MODE_CAN:
            break;
        default:  break;
    }
}



/******************************************************************************/
/*
    Call via Key/Value, State/SubStates Cmds
    vars not stored in host view cache
    todo regularize return status
*/
/******************************************************************************/
uint32_t MotorController_User_Call(const MotorController_T * p_context, MotorController_User_SystemCmd_T id, int32_t value)
{
    MotorController_State_T * p_mc = p_context->P_ACTIVE;

    uint32_t status = 0;
    bool isSuccess = true;

    switch (id)
    {
        // case MOT_USER_SYSTEM_RESRV:                           break;
        case MOT_USER_SYSTEM_BEEP:          MotorController_BeepShort(p_context);                               break;
        // case MOT_USER_SYSTEM_BEEP:          Blinky_BlinkN(&p_context->BUZZER, 250U, 250U, 1U);                break;
        case MOT_USER_SYSTEM_BEEP_STOP:     MotorController_BeepStop(p_context);                                break;
        case MOT_USER_SYSTEM_CLEAR_FAULT:   MotorController_StateMachine_ClearFault(p_context, value);          break;
        case MOT_USER_SYSTEM_RX_WATCHDOG:   MotorController_User_SetRxWatchdog(p_context, value);               break;

        /* Non Blocking function, host/caller poll Async return status after. */
        /* Blocking functions can directly return status. */
        /* MOTOR_CONTROLLER_LOCK_NVM_SAVE_CONFIG will block */
        case MOT_USER_SYSTEM_LOCK_STATE_INPUT:
            // MotorController_User_SetDirection(p_mc, MOT_DRIVE_DIRECTION_PARK);
            // _StateMachine_ProcAsyncInput(&p_mc->StateMachine, MOT_DRIVE_STATE_INPUT_DIRECTION, MOT_DRIVE_DIRECTION_PARK);
            // MotDrive_SetDirection(&p_mc->MotDrive, MOT_DRIVE_DIRECTION_PARK);
            MotorController_User_InputLock(p_context, (MotorController_LockId_T)value);

            /* failed to enter lock */
            if (((MotorController_LockId_T)value != MOTOR_CONTROLLER_LOCK_EXIT) && (MotorController_User_IsLockState(p_context) == false))
                { MotorController_BeepShort(p_context); }

            status = MotorController_User_GetLockOpStatus(p_context); // returns block status, async op always returns 0
            break;
        case MOT_USER_SYSTEM_LOCK_STATE_STATUS:
            status = MotorController_User_GetLockState(p_context);
            break;
        case MOT_USER_SYSTEM_LOCK_ASYNC_STATUS: // union status, 0 as success
            // if (MotorController_User_IsLockOpComplete(p_context) == true)
            status = MotorController_User_GetLockOpStatus(p_context);
            break;

        // include for convenience
        // case MOT_USER_SYSTEM_PARK: isSuccess = MotorController_User_ProcDirection(p_mc, MOT_DRIVE_DIRECTION_PARK);
        // case MOT_USER_SYSTEM_SERVO: MotorController_User_InputServoMode(p_mc, (MotorController_ServoMode_T)value); break;
        default: break;
    }

    if (isSuccess == false) { status = -1; }
    return status;
}


