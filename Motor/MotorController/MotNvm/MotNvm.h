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
    @file   .h
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef MOT_NVM_H
#define MOT_NVM_H

#include "Motor/Motor/Transducer/MotorAnalog/MotorAnalogRef.h"
#include "Utility/BootRef/BootRef.h"

// #if     defined(CONFIG_MOT_NVM_USER_EEPROM)
// #if     defined(CONFIG_MOT_NVM_USER_FLASH)

/* externally define */
struct HAL_Nvm_Manufacturer;
// struct HAL_MemMap_MainConfig;
// typedef struct HAL_Nvm_Manufacturer HAL_Nvm_Manufacturer_T;

// extern uint32_t HAL_Nvm_Manufacturer_GetVMaxVolts(const struct HAL_Nvm_Manufacturer * p_this);
// extern uint32_t HAL_Nvm_Manufacturer_GetIMaxAmps(const struct HAL_Nvm_Manufacturer * p_this);
// extern uint32_t HAL_Nvm_Manufacturer_GetVRated(const struct HAL_Nvm_Manufacturer * p_this);
// extern uint32_t HAL_Nvm_Manufacturer_GetIRatedRms(const struct HAL_Nvm_Manufacturer * p_this);
extern uint32_t HAL_Nvm_Manufacturer_GetVRated_Fract16(const struct HAL_Nvm_Manufacturer * p_this);
extern uint32_t HAL_Nvm_Manufacturer_GetIRatedPeak_Fract16(const struct HAL_Nvm_Manufacturer * p_this);


typedef const struct
{
    Flash_T * const P_FLASH;
#if defined(CONFIG_MOTOR_CONTROLLER_USER_NVM_EEPROM)
    EEPROM_T * const P_EEPROM;
#endif
    /* Flash params start. */
    const uintptr_t MAIN_CONFIG_ADDRESS;
    const uint16_t MAIN_CONFIG_SIZE;
    // alternatively use  P_FLASH->P_PARTITIONS[id]
    const uintptr_t MANUFACTURE_ADDRESS;
    const uint8_t MANUFACTURE_SIZE;

    const BootRef_T * const P_BOOT_REF;
}
const MotNvm_T;

/*
*/
extern void MotNvm_Init(MotNvm_T * p_this);
extern NvMemory_Status_T MotNvm_WriteConfig_Blocking(MotNvm_T * p_this, const void * p_nvm, const void * p_ram, size_t sizeBytes);
extern NvMemory_Status_T MotNvm_ReadManufacture_Blocking(MotNvm_T * p_this, uintptr_t onceAddress, uint8_t size, uint8_t * p_destBuffer);
extern NvMemory_Status_T MotNvm_WriteManufacture_Blocking(MotNvm_T * p_this, uintptr_t onceAddress, const uint8_t * p_sourceBuffer, uint8_t size);
extern NvMemory_Status_T MotNvm_SaveBootReg_Blocking(MotNvm_T * p_this);
extern NvMemory_Status_T MotNvm_LoadMotorRefFrom(MotNvm_T * p_this, const struct HAL_Nvm_Manufacturer * p_source);
extern NvMemory_Status_T MotNvm_LoadMotorRef(MotNvm_T * p_this);


#endif