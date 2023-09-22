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


/*
    Controller NvM Variables Parameters Set
*/
/* VSource is a value < GLOBAL_MOTOR.VMAX and Parameters.VSourceRef */
void MotorController_User_SetVSourceRef(MotorControllerPtr_T p_mc, uint16_t volts)
{
    Global_Motor_InitVSourceRef_V(volts);
    p_mc->Parameters.VSourceRef = Global_Motor_GetVSource_V();
    for(uint8_t iMotor = 0U; iMotor < p_mc->CONFIG.MOTOR_COUNT; iMotor++) { Motor_ResetUnitsVabc(&p_mc->CONFIG.P_MOTORS[iMotor]); }
    VMonitor_SetVInRef(&p_mc->VMonitorSource, volts);

//propagate set //todo restore ratio set
    VMonitor_SetLimitsDefault(&p_mc->VMonitorSource);
    MotorController_User_SetBatteryLifeDefault(p_mc);
}

void MotorController_User_SetBatteryLifeDefault(MotorControllerPtr_T p_mc)
{
    p_mc->Parameters.BatteryZero_Adcu = VMonitor_GetFaultLower(&p_mc->VMonitorSource);
    p_mc->Parameters.BatteryFull_Adcu = VMonitor_ConvertMilliVToAdcu(&p_mc->VMonitorSource, (uint32_t)p_mc->Parameters.VSourceRef * 1000U);
    MotorController_ResetUnitsBatteryLife(p_mc);
}

void MotorController_User_SetBatteryLife_MilliV(MotorControllerPtr_T p_mc, uint32_t zero_mV, uint32_t max_mV)
{
    p_mc->Parameters.BatteryZero_Adcu = VMonitor_ConvertMilliVToAdcu(&p_mc->VMonitorSource, zero_mV);
    p_mc->Parameters.BatteryFull_Adcu = VMonitor_ConvertMilliVToAdcu(&p_mc->VMonitorSource, max_mV);
    MotorController_ResetUnitsBatteryLife(p_mc);
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