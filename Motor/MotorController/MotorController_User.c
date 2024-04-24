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
MotorController_Direction_T MotorController_User_GetDirection(const MotorControllerPtr_T p_mc)
{
    MotorController_Direction_T direction;
    switch(StateMachine_GetActiveStateId(&p_mc->StateMachine))
    {
        case MCSM_STATE_ID_LOCK:        direction = MOTOR_CONTROLLER_DIRECTION_PARK;            break;
        case MCSM_STATE_ID_PARK:        direction = MOTOR_CONTROLLER_DIRECTION_PARK;            break;
        case MCSM_STATE_ID_NEUTRAL:     direction = MOTOR_CONTROLLER_DIRECTION_NEUTRAL;         break;
        case MCSM_STATE_ID_DRIVE:
            if(MotorController_CheckForwardAll(p_mc) == true) { direction = MOTOR_CONTROLLER_DIRECTION_FORWARD; }
            else if(MotorController_CheckReverseAll(p_mc) == true) { direction = MOTOR_CONTROLLER_DIRECTION_REVERSE; }
            else { direction = MOTOR_CONTROLLER_DIRECTION_ERROR; }
            break;
        default: direction = MOTOR_CONTROLLER_DIRECTION_ERROR; break;
    }
    return direction;
}

bool MotorController_User_SetDirection(MotorControllerPtr_T p_mc, MotorController_Direction_T direction)
{
    bool isSuccess;
    if(MotorController_User_GetDirection(p_mc) != direction) { StateMachine_ProcAsyncInput(&p_mc->StateMachine, MCSM_INPUT_DIRECTION, direction); }
    else { MotorController_BeepDouble(p_mc); }
    isSuccess = (MotorController_User_GetDirection(p_mc) == direction);
    if(isSuccess == false) { MotorController_BeepShort(p_mc); }
    return isSuccess;
}

/******************************************************************************/
/* Parameters */
/******************************************************************************/
/*! @param[in] volts < GLOBAL_MOTOR.VMAX and Parameters.VSourceRef */
void MotorController_User_SetVSourceRef(MotorControllerPtr_T p_mc, uint16_t volts)
{
    Global_Motor_InitVSourceRef_V(volts);
    p_mc->Parameters.VSourceRef = Global_Motor_GetVSource_V();
    for(uint8_t iMotor = 0U; iMotor < p_mc->CONFIG.MOTOR_COUNT; iMotor++) { Motor_ResetUnitsVabc(&p_mc->CONFIG.P_MOTORS[iMotor]); }
    VMonitor_SetNominal_MilliV(&p_mc->VMonitorSource, volts * 1000U);
    VMonitor_ResetLimitsDefault(&p_mc->VMonitorSource);
#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
    MotorController_User_SetBatteryLifeDefault(p_mc);
#endif
}

// void MotorController_User_SetILimit_DC(MotorControllerPtr_T p_mc, uint16_t dc)
// {
//     uint16_t iPeakAc = dc;
//     uint16_t motoring = iPeakAc;

//     for(uint8_t iMotor = 0U; iMotor < p_mc->CONFIG.MOTOR_COUNT; iMotor++)
//     {
//         Motor_User_SetILimitMotoringParam_Amp(&p_mc->CONFIG.P_MOTORS[iMotor], motoring);
//     }
// }

#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
/*
    e.g. 42V Source:
    VSource 100% 42V
    VSource 0% 0V
    VSource VInRef 42V
    VSource FaultUpper 52.5V
    VSource WarningUpper 47.25V
    VSource WarningLower 36.75V - begin ILimitLowV
    VSource FaultLower 31.5V
    VBattery 100% 42V
    VBattery 0% 31.5V
*/
void MotorController_User_SetBatteryLifeDefault(MotorControllerPtr_T p_mc)
{
    p_mc->Parameters.BatteryZero_Adcu = VMonitor_GetFaultLower(&p_mc->VMonitorSource);
    p_mc->Parameters.BatteryFull_Adcu = VMonitor_AdcuOfMilliV(&p_mc->VMonitorSource, (uint32_t)p_mc->Parameters.VSourceRef * 1000U);
    MotorController_ResetUnitsBatteryLife(p_mc);
}

void MotorController_User_SetBatteryLife_MilliV(MotorControllerPtr_T p_mc, uint32_t zero_mV, uint32_t max_mV)
{
    p_mc->Parameters.BatteryZero_Adcu = VMonitor_AdcuOfMilliV(&p_mc->VMonitorSource, zero_mV);
    p_mc->Parameters.BatteryFull_Adcu = VMonitor_AdcuOfMilliV(&p_mc->VMonitorSource, max_mV);
    MotorController_ResetUnitsBatteryLife(p_mc);
}
#endif