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
/*! @param[in] volts < GLOBAL_MOTOR.VMAX and Parameters.VSourceRef */
void MotorController_User_SetVSourceRef(MotorControllerPtr_T p_mc, uint16_t volts)
{
    Global_Motor_InitVSourceRef_V(volts);
    p_mc->Parameters.VSourceRef = Global_Motor_GetVSource_V();
    for(uint8_t iMotor = 0U; iMotor < p_mc->CONFIG.MOTOR_COUNT; iMotor++) { Motor_ResetUnitsVabc(&p_mc->CONFIG.P_MOTORS[iMotor]); }
    VMonitor_SetVInRef(&p_mc->VMonitorSource, volts);

    // propagate set //todo restore ratio set
    VMonitor_SetLimitsDefault(&p_mc->VMonitorSource);
#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
    MotorController_User_SetBatteryLifeDefault(p_mc);
#endif
}

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
    p_mc->Parameters.BatteryFull_Adcu = VMonitor_ConvertMilliVToAdcu(&p_mc->VMonitorSource, (uint32_t)p_mc->Parameters.VSourceRef * 1000U);
    MotorController_ResetUnitsBatteryLife(p_mc);
}

void MotorController_User_SetBatteryLife_MilliV(MotorControllerPtr_T p_mc, uint32_t zero_mV, uint32_t max_mV)
{
    p_mc->Parameters.BatteryZero_Adcu = VMonitor_ConvertMilliVToAdcu(&p_mc->VMonitorSource, zero_mV);
    p_mc->Parameters.BatteryFull_Adcu = VMonitor_ConvertMilliVToAdcu(&p_mc->VMonitorSource, max_mV);
    MotorController_ResetUnitsBatteryLife(p_mc);
}
#endif

// void MotorController_User_SetILimit_DC(MotorControllerPtr_T p_mc, uint16_t dc)
// {
//     uint16_t iPeakAc = dc;
//     uint16_t motoring = iPeakAc;

//     for(uint8_t iMotor = 0U; iMotor < p_mc->CONFIG.MOTOR_COUNT; iMotor++)
//     {
//         Motor_User_SetILimitMotoringParam_Amp(&p_mc->CONFIG.P_MOTORS[iMotor], motoring);
//     }
// }

/*
    WriteOnce Variables
*/
// #if defined(CONFIG_MOTOR_CONTROLLER_FLASH_LOADER_ENABLE)
/* Save Once does not need state machine */
// static inline void MotorController_User_WriteManufacture_Blocking(MotorControllerPtr_T p_mc, const MotorController_Manufacture_T * p_data)
// {
// #if defined(CONFIG_MOTOR_CONTROLLER_MANUFACTURE_PARAMS_RAM_COPY_ENABLE)
//     memcpy(&p_mc->Manufacture, p_data, sizeof(MotorController_Manufacture_T));
// #endif
//     MotorController_User_ProcCalibration_Blocking(p_mc, MOTOR_CONTROLLER_NVM_WRITE_ONCE);
// }

// static inline void MotorController_User_ReadManufacture_Blocking(MotorControllerPtr_T p_mc)
// {
//     MotorController_User_ProcCalibration_Blocking(p_mc, MOTOR_CONTROLLER_NVM_READ_ONCE);
// }

// #if defined(CONFIG_MOTOR_CONTROLLER_MANUFACTURE_PARAMS_RAM_COPY_ENABLE)

// static inline void MotorController_User_GetName(MotorControllerPtr_T p_mc, uint8_t * p_stringBuffer) { memcpy(p_stringBuffer, &p_mc->Manufacture.NAME[0U], 8U); }
// static inline char * MotorController_User_GetPtrName(MotorControllerPtr_T p_mc) { return &p_mc->Manufacture.NAME[0U]; }
// // static inline char MotorController_User_GetNameIndex(MotorControllerPtr_T p_mc, uint8_t charIndex) { return p_mc->Manufacture.NAME[charIndex]; }

// static inline void MotorController_User_GetManufacture(MotorControllerPtr_T p_mc, MotorController_Manufacture_T * p_dest) { memcpy(p_dest, &p_mc->Manufacture, sizeof(MotorController_Manufacture_T)); }
// static inline uint32_t MotorController_User_GetSerialNumber(MotorControllerPtr_T p_mc) { return p_mc->Manufacture.SERIAL_NUMBER_WORD; }
// static inline uint32_t MotorController_User_GetManufactureDate(MotorControllerPtr_T p_mc) { return p_mc->Manufacture.MANUFACTURE_NUMBER_WORD; }
// // static inline void MotorController_User_GetIdExt(MotorControllerPtr_T p_mc, uint8_t * p_stringBuffer) { memcpy(p_stringBuffer, &p_mc->Manufacture.ID_EXT[0U], 8U); }
// #endif

// static inline Flash_Status_T MotorController_User_ReadName_Blocking(MotorControllerPtr_T p_mc, uint8_t charIndex)
// {
//     //set read once, goto statemachine
// // #if defined(CONFIG_MOTOR_CONTROLLER_ONCE_USE_FLASH)
// //     return p_mc->CONFIG.P_MANUFACTURE->NAME[charIndex];
// // #elif defined(CONFIG_MOTOR_CONTROLLER_ONCE_USE_ONCE)
//     return Flash_ReadOnce_Blocking(p_mc->CONFIG.P_FLASH, &p_mc->CONFIG.P_MANUFACTURE->NAME[charIndex], 8U);
// // #endif
// }
// static inline Flash_Status_T MotorController_User_WriteName_Blocking(MotorControllerPtr_T p_mc, const uint8_t * p_nameString)
// {
//     return Flash_WriteOnce_Blocking(p_mc->CONFIG.P_FLASH, &p_mc->CONFIG.P_MANUFACTURE->NAME[0U], p_nameString, 8U);
// };
// #endif