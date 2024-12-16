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
/* Real Time */
/******************************************************************************/
/* Drive Direction */
MotorController_Direction_T MotorController_User_GetDirection(const MotorController_T * p_mc)
{
    MotorController_Direction_T direction;
    switch(StateMachine_GetActiveStateId(&p_mc->StateMachine))
    {
        case MCSM_STATE_ID_LOCK:        direction = MOTOR_CONTROLLER_DIRECTION_PARK;            break;
        case MCSM_STATE_ID_PARK:        direction = MOTOR_CONTROLLER_DIRECTION_PARK;            break;
        case MCSM_STATE_ID_NEUTRAL:     direction = MOTOR_CONTROLLER_DIRECTION_NEUTRAL;         break;
        case MCSM_STATE_ID_DRIVE:
            if      (MotorController_IsEveryMotorForward(p_mc) == true) { direction = MOTOR_CONTROLLER_DIRECTION_FORWARD; }
            else if (MotorController_IsEveryMotorReverse(p_mc) == true) { direction = MOTOR_CONTROLLER_DIRECTION_REVERSE; }
            else                                                    { direction = MOTOR_CONTROLLER_DIRECTION_ERROR; }
            break;
        default: direction = MOTOR_CONTROLLER_DIRECTION_ERROR; break;
    }
    return direction;
}

bool MotorController_User_SetDirection(MotorController_T * p_mc, MotorController_Direction_T direction)
{
    bool isSuccess;
    if(MotorController_User_GetDirection(p_mc) != direction) { StateMachine_ProcInput(&p_mc->StateMachine, MCSM_INPUT_DIRECTION, direction); }
    // else { MotorController_BeepDouble(p_mc); }
    isSuccess = (MotorController_User_GetDirection(p_mc) == direction);
    if(isSuccess == false) { MotorController_BeepShort(p_mc); }
    return isSuccess;
}

/******************************************************************************/
/* Config */
/******************************************************************************/
/*! @param[in] volts < MOTOR_STATIC.VMAX and Config.VSourceRef */
void MotorController_User_SetVSourceRef(MotorController_T * p_mc, uint16_t volts)
{
    /* VSource Monitor Ref */
    p_mc->Config.VSourceRef = Motor_VSourceLimitOf(volts);
    VMonitor_SetNominal_MilliV(&p_mc->VMonitorSource, p_mc->Config.VSourceRef * 1000U);
    VMonitor_ResetLimitsDefault(&p_mc->VMonitorSource);

    /* VSource Active Ref *//* todo Set Motor Ref using read Value */
    // MotorController_ResetVSourceActiveRef(p_mc);
    Motor_Static_InitVSourceRef_V(volts);
    for(uint8_t iMotor = 0U; iMotor < p_mc->CONST.MOTOR_COUNT; iMotor++) { Motor_ResetUnitsVabc(&p_mc->CONST.P_MOTORS[iMotor]); }
#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
    MotorController_User_SetBatteryLifeDefault(p_mc);
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
/*
*/
void MotorController_User_SetBatteryLifeDefault(MotorController_T * p_mc)
{
    p_mc->Config.BatteryZero_Adcu = VMonitor_GetFaultLower(&p_mc->VMonitorSource);
    p_mc->Config.BatteryFull_Adcu = VMonitor_AdcuOfMilliV(&p_mc->VMonitorSource, (uint32_t)p_mc->Config.VSourceRef * 1000U);
    MotorController_ResetUnitsBatteryLife(p_mc);
}

void MotorController_User_SetBatteryLife_MilliV(MotorController_T * p_mc, uint32_t zero_mV, uint32_t max_mV)
{
    p_mc->Config.BatteryZero_Adcu = VMonitor_AdcuOfMilliV(&p_mc->VMonitorSource, zero_mV);
    p_mc->Config.BatteryFull_Adcu = VMonitor_AdcuOfMilliV(&p_mc->VMonitorSource, max_mV);
    MotorController_ResetUnitsBatteryLife(p_mc);
}
#endif

/******************************************************************************/
/* Manufacture */
// use outer layer StateMachine check, simplifies handling of signature type.
// Multi variable StateMachine call
/******************************************************************************/
/* Caller clears buffer */
NvMemory_Status_T MotorController_User_ReadManufacture_Blocking(MotorController_T * p_mc, uintptr_t onceAddress, uint8_t size, uint8_t * p_destBuffer)
{
    NvMemory_Status_T status;

    if (MotorController_User_IsConfigState(p_mc) == true)
    {
        status = MotorController_ReadManufacture_Blocking(p_mc, onceAddress, size, p_destBuffer);
    }
    else
    {
        status = NV_MEMORY_STATUS_ERROR_OTHER;
    }

    return status;
}

NvMemory_Status_T MotorController_User_WriteManufacture_Blocking(MotorController_T * p_mc, uintptr_t onceAddress, const uint8_t * p_source, uint8_t size)
{
    NvMemory_Status_T status;

    if(MotorController_User_IsLockState(p_mc) == true)
    {
        status = MotorController_WriteManufacture_Blocking(p_mc, onceAddress, p_source, size);
    }
    else
    {
        status = NV_MEMORY_STATUS_ERROR_OTHER;
    }

    return status;
}

// alternatively write to buffer

/*! @param[in] opId MOTOR_CONTROLLER_NVM_BOOT, MOTOR_CONTROLLER_NVM_WRITE_ONCE, MOTOR_CONTROLLER_LOCK_NVM_SAVE_CONFIG */
// static inline NvMemory_Status_T _MotorController_User_SaveNvm_Blocking(MotorController_T * p_mc, MotorController_LockId_T opId)
// {
//     MotorController_User_InputLock(p_mc, opId);
//     return p_mc->NvmStatus;
// }

// static inline NvMemory_Status_T MotorController_User_SaveManufacture_Blocking(MotorController_T * p_mc)
// {
//     return _MotorController_User_SaveNvm_Blocking(p_mc, MOTOR_CONTROLLER_BLOCKING_NVM_WRITE_ONCE);
//     // return StateMachine_ProcInput(p_mc, MCSM_INPUT_LOCK, MOTOR_CONTROLLER_LOCK_NVM_SAVE_CONFIG);
//     // return p_mc->NvmStatus;
// }

// static inline NvMemory_Status_T MotorController_User_ReadManufacture_Blocking(MotorController_T * p_mc)
// {
//     MotorController_User_InputLock(p_mc, MOTOR_CONTROLLER_LOCKED_NVM_READ_ONCE);
//     return p_mc->NvmStatus;
// }
