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
    @version V0
*/
/******************************************************************************/
#include "MotorController_User.h"

/******************************************************************************/
/*
    Real Time StateMachine Functions
*/
/******************************************************************************/
/* Drive Direction */
MotorController_Direction_T MotorController_User_GetDirection(const MotorController_T * p_mc)
{
    MotorController_Direction_T direction;
    switch (StateMachine_GetActiveStateId(&p_mc->StateMachine))
    {
        case MCSM_STATE_ID_LOCK:        direction = MOTOR_CONTROLLER_DIRECTION_PARK;            break;
        case MCSM_STATE_ID_PARK:        direction = MOTOR_CONTROLLER_DIRECTION_PARK;            break;
        case MCSM_STATE_ID_NEUTRAL:     direction = MOTOR_CONTROLLER_DIRECTION_NEUTRAL;         break;
        case MCSM_STATE_ID_DRIVE:
            if      (Motor_Array_IsEvery(&p_mc->CONST.MOTORS, Motor_IsDirectionForward) == true) { direction = MOTOR_CONTROLLER_DIRECTION_FORWARD; }
            else if (Motor_Array_IsEvery(&p_mc->CONST.MOTORS, Motor_IsDirectionReverse) == true) { direction = MOTOR_CONTROLLER_DIRECTION_REVERSE; }
            else                                                        { direction = MOTOR_CONTROLLER_DIRECTION_ERROR; }
            break;
        default: direction = MOTOR_CONTROLLER_DIRECTION_ERROR; break;
    }
    return direction;
}

void MotorController_User_SetDirection(MotorController_T * p_mc, MotorController_Direction_T direction)
{
    if (MotorController_User_GetDirection(p_mc) != direction)
    {
        _StateMachine_ProcAsyncInput(&p_mc->StateMachine, MCSM_INPUT_DIRECTION, direction);
        if (MotorController_User_GetDirection(p_mc) != direction) { MotorController_BeepShort(p_mc); } /* effective on motor async proc only */
    }
}

/* Separate Check direction with alarm, so Motor set can use SetSyncInput */
bool MotorController_User_CheckDirection(MotorController_T * p_mc, MotorController_Direction_T direction)
{
    bool isSuccess = (MotorController_User_GetDirection(p_mc) == direction);
    if (isSuccess == false) { MotorController_BeepShort(p_mc); }
    return isSuccess;
}

/*
    Motor must also use Async_ProcInput
*/
// bool MotorController_User_ProcDirection(MotorController_T * p_mc, MotorController_Direction_T direction)
// {
//     bool isSuccess;
//     if (MotorController_User_GetDirection(p_mc) != direction) { _StateMachine_ProcAsyncInput(&p_mc->StateMachine, MCSM_INPUT_DIRECTION, direction); }
//     // else { MotorController_BeepDouble(p_mc); }
//     isSuccess = (MotorController_User_GetDirection(p_mc) == direction);
//     if (isSuccess == false) { MotorController_BeepShort(p_mc); }
//     return isSuccess;
// }

bool MotorController_User_SetSpeedLimitAll(MotorController_T * p_mc, uint16_t limit_fract16)
{
    LimitArray_SetEntry(&p_mc->MotLimits.SpeedLimit, MOT_SPEED_LIMIT_USER, limit_fract16);
    Motor_Array_ApplySpeedLimit(&p_mc->CONST.MOTORS, &p_mc->MotLimits.SpeedLimit);
}

bool MotorController_User_ClearSpeedLimitAll(MotorController_T * p_mc)
{
    LimitArray_ClearEntry(&p_mc->MotLimits.SpeedLimit, MOT_SPEED_LIMIT_USER);
    Motor_Array_ApplySpeedLimit(&p_mc->CONST.MOTORS, &p_mc->MotLimits.SpeedLimit);
}

bool MotorController_User_SetILimitAll(MotorController_T * p_mc, uint16_t limit_fract16)
{
    LimitArray_SetEntry(&p_mc->MotLimits.ILimit, MOT_I_LIMIT_USER, limit_fract16);
    Motor_Array_ApplyILimit(&p_mc->CONST.MOTORS, &p_mc->MotLimits.ILimit);
}

bool MotorController_User_ClearILimitAll(MotorController_T * p_mc)
{
    LimitArray_ClearEntry(&p_mc->MotLimits.ILimit, MOT_I_LIMIT_USER);
    Motor_Array_ApplyILimit(&p_mc->CONST.MOTORS, &p_mc->MotLimits.ILimit);
}

void MotorController_User_SetOptSpeedLimitOnOff(MotorController_T * p_mc, bool isEnable)
{
    if (isEnable == true)   { MotorController_User_SetSpeedLimitAll(p_mc, p_mc->Config.OptSpeedLimit_Fract16); }
    else                    { MotorController_User_ClearSpeedLimitAll(p_mc); }
}

void MotorController_User_SetOptILimitOnOff(MotorController_T * p_mc, bool isEnable)
{
    if (isEnable == true)   { MotorController_User_SetILimitAll(p_mc, p_mc->Config.OptILimit_Fract16); }
    else                    { MotorController_User_ClearILimitAll(p_mc); }
}

/******************************************************************************/
/* Config */
/******************************************************************************/
/*! @param[in] volts < MOTOR_ANALOG_REFERENCE.VMAX and Config.VSourceRef */
void MotorController_User_SetVSourceRef(MotorController_T * p_mc, uint16_t volts)
{
    // p_mc->Config.VSourceRef = Motor_VSourceLimitOf(volts);
    p_mc->Config.VSourceRef = volts;
    MotorController_CaptureVSourceActiveRef(p_mc);
    MotorController_ResetVSourceMonitorDefaults(p_mc);
#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
    MotorController_ResetBatteryLifeDefault(p_mc);
#endif
}

// void MotorController_User_SetILimit_DC(MotorController_T * p_mc, uint16_t dc)
// {
// }

#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
void MotorController_User_SetBatteryLife_MilliV(MotorController_T * p_mc, uint32_t zero_mV, uint32_t max_mV)
{
    p_mc->Config.BatteryZero_Adcu = VMonitor_AdcuOfMilliV(&p_mc->VMonitorSource, zero_mV);
    p_mc->Config.BatteryFull_Adcu = VMonitor_AdcuOfMilliV(&p_mc->VMonitorSource, max_mV);
    MotorController_ResetUnitsBatteryLife(p_mc);
}
#endif

/******************************************************************************/
/* Manufacture */
/******************************************************************************/
/*
    Multi variable StateMachine call
    Use outer layer StateMachine check, simplifies handling of signature type.
*/
NvMemory_Status_T MotorController_User_ReadManufacture_Blocking(MotorController_T * p_mc, uintptr_t onceAddress, uint8_t size, uint8_t * p_destBuffer)
{
    NvMemory_Status_T status = NV_MEMORY_STATUS_ERROR_OTHER;
    if (MotorController_User_IsConfigState(p_mc) == true) { status = MotNvm_ReadManufacture_Blocking(&p_mc->CONST.MOT_NVM, onceAddress, size, p_destBuffer); }
    return status;
}

NvMemory_Status_T MotorController_User_WriteManufacture_Blocking(MotorController_T * p_mc, uintptr_t onceAddress, const uint8_t * p_source, uint8_t size)
{
    NvMemory_Status_T status = NV_MEMORY_STATUS_ERROR_OTHER;
    if (MotorController_User_IsLockState(p_mc) == true) { status = MotNvm_WriteManufacture_Blocking(&p_mc->CONST.MOT_NVM, onceAddress, p_source, size); }
    return status;
}


/******************************************************************************/
/*
    Call via Key/Value, State/SubStates Cmds
    vars not stored in host view cache
    todo regularize return status
*/
/******************************************************************************/
uint32_t MotorController_User_Call(MotorController_T * p_mc, MotorController_User_CallId_T id, int32_t value)
{
    uint32_t status = 0;
    bool isSuccess = true;

    switch (id)
    {
        // case MOT_USER_SYSTEM_RESRV:                           break;
        case MOT_USER_SYSTEM_BEEP:          MotorController_User_BeepN(p_mc, 500U, 500U, value);                break;
        case MOT_USER_SYSTEM_BEEP_STOP:     MotorController_User_BeepStop(p_mc);                                break;
        case MOT_USER_SYSTEM_CLEAR_FAULT:   isSuccess = MotorController_StateMachine_ClearFault(p_mc, value);   break;
        case MOT_USER_SYSTEM_RX_WATCHDOG:   MotorController_User_SetRxWatchdog(p_mc, value);                    break;

        /* Non Blocking function, host/caller poll Async return status after. */
        /* Blocking functions can directly return status. */
        /* MOTOR_CONTROLLER_LOCK_NVM_SAVE_CONFIG will block */
        case MOT_USER_SYSTEM_LOCK_STATE_INPUT:
            // MotorController_User_SetDirection(p_mc, MOTOR_CONTROLLER_DIRECTION_PARK);
            _StateMachine_ProcAsyncInput(&p_mc->StateMachine, MCSM_INPUT_DIRECTION, MOTOR_CONTROLLER_DIRECTION_PARK);
            MotorController_User_InputLock(p_mc, (MotorController_LockId_T)value);
            if (((MotorController_LockId_T)value != MOTOR_CONTROLLER_LOCK_EXIT) && (MotorController_User_IsLockState(p_mc) == false))
                { MotorController_BeepShort(p_mc); }
            status = MotorController_User_GetLockOpStatus(p_mc); // returns block status, async op always returns 0
            break;
        case MOT_USER_SYSTEM_LOCK_STATE_STATUS:
            status = MotorController_User_GetLockState(p_mc);
            break;
        case MOT_USER_SYSTEM_LOCK_ASYNC_STATUS: // union status, 0 as success
            // if (MotorController_User_IsLockOpComplete(p_mc) == true)
            status = MotorController_User_GetLockOpStatus(p_mc);
            break;
        // include for convenience
        // case MOT_USER_SYSTEM_PARK: isSuccess = MotorController_User_ProcDirection(p_mc, MOTOR_CONTROLLER_DIRECTION_PARK);
        // case MOT_USER_SYSTEM_SERVO: MotorController_User_InputServoMode(p_mc, (MotorController_ServoMode_T)value); break;
        default: break;
    }

    if (isSuccess == false) { status = -1; }
    return status;
}


