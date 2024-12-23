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
    Real Time
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
            if      (MotorController_IsEveryMotorForward(p_mc) == true) { direction = MOTOR_CONTROLLER_DIRECTION_FORWARD; }
            else if (MotorController_IsEveryMotorReverse(p_mc) == true) { direction = MOTOR_CONTROLLER_DIRECTION_REVERSE; }
            else                                                        { direction = MOTOR_CONTROLLER_DIRECTION_ERROR; }
            break;
        default: direction = MOTOR_CONTROLLER_DIRECTION_ERROR; break;
    }
    return direction;
}

bool MotorController_User_SetDirection(MotorController_T * p_mc, MotorController_Direction_T direction)
{
    bool isSuccess;
    if (MotorController_User_GetDirection(p_mc) != direction) { StateMachine_ProcInput(&p_mc->StateMachine, MCSM_INPUT_DIRECTION, direction); }
    // else { MotorController_BeepDouble(p_mc); }
    isSuccess = (MotorController_User_GetDirection(p_mc) == direction);
    if (isSuccess == false) { MotorController_BeepShort(p_mc); }
    return isSuccess;
}

// static inline MotorController_MainMode_T MotorController_User_GetMainMode(MotorController_T * p_mc)
// {
//     return (StateMachine_GetActiveStateId(&p_mc->StateMachine) == MCSM_STATE_ID_SERVO) ? MOTOR_CONTROLLER_MAIN_MODE_SERVO : MOTOR_CONTROLLER_MAIN_MODE_DRIVE;
// }

/******************************************************************************/
/*
    Config
*/
/******************************************************************************/
/*! @param[in] volts < MOTOR_STATIC.VMAX and Config.VSourceRef */
void MotorController_User_SetVSourceRef(MotorController_T * p_mc, uint16_t volts)
{
    p_mc->Config.VSourceRef = Motor_VSourceLimitOf(volts);
    MotorController_ResetVSourceActiveRef(p_mc);
    MotorController_ResetVSourceMonitorDefaults(p_mc);
#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
    MotorController_ResetBatteryLifeDefault(p_mc);
#endif
}

// void MotorController_User_SetILimit_DC(MotorController_T * p_mc, uint16_t dc)
// {
//     uint16_t iPeakAc = dc;
//     uint16_t motoring = iPeakAc;
//     for(uint8_t iMotor = 0U; iMotor < p_mc->CONST.MOTOR_COUNT; iMotor++)
//     {
//         Motor_User_SetILimitMotoringParam_Amp(&p_mc->CONST.P_MOTORS[iMotor], motoring);
//     }
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
    if (MotorController_User_IsConfigState(p_mc) == true) { status = MotorController_ReadManufacture_Blocking(p_mc, onceAddress, size, p_destBuffer); }
    return status;
}

NvMemory_Status_T MotorController_User_WriteManufacture_Blocking(MotorController_T * p_mc, uintptr_t onceAddress, const uint8_t * p_source, uint8_t size)
{
    NvMemory_Status_T status = NV_MEMORY_STATUS_ERROR_OTHER;
    if (MotorController_User_IsLockState(p_mc) == true) { status = MotorController_WriteManufacture_Blocking(p_mc, onceAddress, p_source, size); }
    return status;
}

/******************************************************************************/
/*
    VarId Key/Value based I/O
*/
/******************************************************************************/
/* with status */
static inline uint32_t _MotorController_User_InputLockOp(MotorController_T * p_mc, MotorController_LockId_T opId)
{
    uint32_t status = MOT_VAR_STATUS_OK;
    bool isSuccess = true;

    // status = MotorController_User_InputLock(p_mc, opId); // match return status

    switch (opId) /* StateMachine will check for invalid Id */
    {
        case MOTOR_CONTROLLER_LOCK_PARK:      isSuccess = MotorController_User_SetDirection(p_mc, MOTOR_CONTROLLER_DIRECTION_PARK);  break;
        case MOTOR_CONTROLLER_LOCK_ENTER:     isSuccess = MotorController_User_EnterLockState(p_mc);  break;
        case MOTOR_CONTROLLER_LOCK_EXIT:      isSuccess = MotorController_User_ExitLockState(p_mc);   break;
            /* Non Blocking function, host/caller poll Async return status after. */ // calibration status todo
        case MOTOR_CONTROLLER_LOCK_CALIBRATE_SENSOR:    MotorController_User_InputLock(p_mc, MOTOR_CONTROLLER_LOCK_CALIBRATE_SENSOR);   break;
        case MOTOR_CONTROLLER_LOCK_CALIBRATE_ADC:       MotorController_User_InputLock(p_mc, MOTOR_CONTROLLER_LOCK_CALIBRATE_ADC);      break;
            /* Blocking functions can directly return status. */
        case MOTOR_CONTROLLER_LOCK_NVM_SAVE_CONFIG:     status = MotorController_User_SaveConfig_Blocking(p_mc);                        break;
        case MOTOR_CONTROLLER_LOCK_REBOOT:              MotorController_User_InputLock(p_mc, MOTOR_CONTROLLER_LOCK_REBOOT); break; /* No return */
        default: break;
    }

    if (isSuccess == false) { status = MOT_VAR_STATUS_ERROR; }
    return status;
}


uint32_t MotorController_User_InputSystem(MotorController_T * p_mc, MotorController_User_System_T id, int32_t value)
{
    uint32_t status = MOT_VAR_STATUS_OK;
    bool isSuccess = true;

    switch (id)
    {
        case MOT_USER_SYSTEM_BEEP:                  MotorController_User_BeepN(p_mc, 500U, 500U, value);                                  break;
        case MOT_USER_SYSTEM_CLEAR_FAULT:           isSuccess = MotorController_StateMachine_ClearFault(p_mc, value);                     break;
        case MOT_USER_SYSTEM_RX_WATCHDOG:           MotorController_User_SetRxWatchdog(p_mc, value);                                      break;
        case MOT_USER_SYSTEM_LOCK_STATE:            status = _MotorController_User_InputLockOp(p_mc, (MotorController_LockId_T)value); break;
        case MOT_USER_SYSTEM_SERVO:                 MotorController_User_InputServoMode(p_mc, (MotorController_ServoMode_T)value);        break;
        // case MOT_USER_SYSTEM_SYSTEM_GENERAL:     MotorController_User_InputSystemGeneral(p_mc, (MotorController_SystemGeneral_T)value); break;
        default: break;
    }

    if (isSuccess == false) { status = MOT_VAR_STATUS_ERROR; }
    return status;
}


/******************************************************************************/
/* Inputs disabled on Analog Mode */
/******************************************************************************/
/* SetIO */
uint32_t MotorController_User_InputControl(MotorController_T * p_mc, MotVarId_Control_General_T id, int32_t value)
{
    bool isSet = true;
    if (p_mc->Config.InputMode == MOTOR_CONTROLLER_INPUT_MODE_SERIAL)
    {
        switch (id)
        {
            case MOT_VAR_DIRECTION:  isSet = MotorController_User_SetDirection(p_mc, (MotorController_Direction_T)value);    break;
            default: break;
        }
    }

    return (isSet == true) ? MOT_VAR_STATUS_OK : MOT_VAR_STATUS_ERROR;
}

uint32_t _MotorController_User_InputDriveCmd(MotorController_T * p_mc, MotVarId_Cmd_General_T id, int32_t value)
{
    bool isSet = (p_mc->Config.InputMode == MOTOR_CONTROLLER_INPUT_MODE_SERIAL);

    if (isSet)
    {
        switch (id)
        {
            case MOT_VAR_THROTTLE:  MotorController_User_SetCmdThrottle(p_mc, value);        break;
            case MOT_VAR_BRAKE:     MotorController_User_SetCmdBrake(p_mc, value);           break;
            // case MOT_VAR_DRIVE_DIRECTION:  isSet = MotorController_User_SetDirection(p_mc, (MotorController_Direction_T)value);    break;
            default: break;
        }
    }

    return  (isSet == true) ? MOT_VAR_STATUS_OK : MOT_VAR_STATUS_ERROR_PROTOCOL_CONTROL_DISABLED;
}

uint32_t MotorController_User_InputCmd(MotorController_T * p_mc, MotVarId_Cmd_General_T id, int32_t value)
{
   volatile bool isSuccess = true;

    switch (id)
    {
        // case MOT_VAR_BEEP:                      MotorController_User_BeepN(p_mc, 500U, 500U, varValue);     break;
        // case MOT_VAR_CONTROLLER_MODE:           MotorController_User_SetMainMode(p_mc, varValue);           break;
        // case MOT_VAR_CLEAR_FAULT:               isSuccess = MotorController_StateMachine_ClearFault(p_mc, varValue);   break;

        case MOT_VAR_THROTTLE:                  _MotorController_User_InputDriveCmd(p_mc, id, value);            break;
        case MOT_VAR_BRAKE:                     _MotorController_User_InputDriveCmd(p_mc, id, value);            break;

        case MOT_VAR_USER_CMD:                  MotorController_User_SetCmdValue(p_mc, value);               break;
        case MOT_VAR_USER_FEEDBACK_MODE:        MotorController_User_SetFeedbackMode_Cast(p_mc, value);      break;

        case MOT_VAR_OPT_SPEED_LIMIT_ON_OFF:    MotorController_User_SetOptSpeedLimitOnOff(p_mc, value);     break;
        case MOT_VAR_OPT_I_LIMIT_ON_OFF:        MotorController_User_SetOptILimitOnOff(p_mc, value);         break;

        // case MOT_VAR_TRY_HOLD:                  break;
        // case MOT_VAR_TRY_RELEASE:               break;
        // case MOT_VAR_FORCE_DISABLE_CONTROL:     break;
        default: break;
    }

    return (isSuccess == true) ? MOT_VAR_STATUS_OK : MOT_VAR_STATUS_ERROR;
}

int32_t MotorController_User_OutputVar(MotorController_T * p_mc, MotVarId_Monitor_General_T varKey)
{

}