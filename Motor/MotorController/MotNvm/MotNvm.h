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

*/
/******************************************************************************/
#ifndef MOT_NVM_H
#define MOT_NVM_H

#include "Motor/Motor/Phase_Input/Phase_Analog.h"
#include "Motor/Motor/Phase_Input/Phase_Calibration.h"
#include "Utility/BootRef/BootRef.h"
#include "Peripheral/NvMemory/Flash/Flash.h"
#include "Peripheral/NvMemory/EEPROM/EEPROM.h"

// #if     defined(MOT_NVM_USER_EEPROM)
// #elif   defined(MOT_NVM_USER_FLASH)
// #else
//     #define MOT_NVM_USER_FLASH
// #endif

#if     defined(MOTOR_CONTROLLER_USER_NVM_EEPROM)
#elif   defined(MOTOR_CONTROLLER_USER_NVM_FLASH)
#else
#define MOTOR_CONTROLLER_USER_NVM_FLASH
#endif

#if     defined(MOTOR_CONTROLLER_MANUFACTURE_NVM_ONCE)
#elif   defined(MOTOR_CONTROLLER_MANUFACTURE_NVM_FLASH)
#else
#define MOTOR_CONTROLLER_MANUFACTURE_NVM_ONCE
#endif

#ifndef MOT_NVM_MANUFACTURE_SIZE
#define MOT_NVM_MANUFACTURE_SIZE (64U)
#endif

/* For Protocol Flash Only */
#if     defined(MOTOR_CONTROLLER_FLASH_LOADER_ENABLE)
#elif   defined(MOTOR_CONTROLLER_FLASH_LOADER_DISABLE)
#else
#define MOTOR_CONTROLLER_FLASH_LOADER_DISABLE
#endif

/* Config Mem descriptor table. */
typedef const struct MotNvm_Entry
{
    const void * NVM_ADDRESS;     /* NVM/Flash address */
    void * RAM_ADDRESS;           /* RAM address */
    size_t SIZE;                  /* Size in bytes */
    // const uint32_t checksum_seed;    /* For validation */
}
MotNvm_Entry_T;

typedef const struct
{
    Flash_T * P_FLASH;
#if defined(MOTOR_CONTROLLER_USER_NVM_EEPROM)
    EEPROM_T * P_EEPROM;
#endif

    /* Flash params start. */
    uintptr_t MAIN_CONFIG_ADDRESS;
    uint16_t MAIN_CONFIG_SIZE;
    MotNvm_Entry_T * P_PARTITIONS; /* Config Partition decriptors */
    size_t PARTITION_COUNT;        /* Number of NVM Partitions */

    // alternatively use  P_FLASH->P_PARTITIONS[id]
    uintptr_t MANUFACTURE_ADDRESS;
    uint8_t MANUFACTURE_SIZE;

    const BootRef_T * P_BOOT_REF;
}
MotNvm_T;

/*
*/
extern void MotNvm_Init(const MotNvm_T * p_motNvm);
extern NvMemory_Status_T MotNvm_Write_Blocking(const MotNvm_T * p_motNvm, const void * p_rom, const void * p_ram, size_t size);
extern NvMemory_Status_T MotNvm_ReadManufacture_Blocking(const MotNvm_T * p_motNvm, uintptr_t address, uint8_t size, void * p_dest);
extern NvMemory_Status_T MotNvm_WriteManufacture_Blocking(const MotNvm_T * p_motNvm, uintptr_t address, const void * p_source, uint8_t size);
extern NvMemory_Status_T MotNvm_SaveBootReg_Blocking(const MotNvm_T * p_motNvm);
extern NvMemory_Status_T MotNvm_SaveConfigAll_Blocking(const MotNvm_T * p_motNvm);


extern NvMemory_Status_T MotNvm_WriteConstRef(const MotNvm_T * p_motNvm);

/* externally define */
// struct HAL_Nvm_Once_T;
struct HAL_Nvm_Manufacturer;
typedef const struct HAL_Nvm_Manufacturer HAL_Nvm_Manufacturer_T;

extern void HAL_Nvm_MapPhaseCalibration(const HAL_Nvm_Manufacturer_T * p_manufacture, Phase_Calibration_T * p_buffer);
extern void HAL_Nvm_MapPhaseAnalogCalibration(const HAL_Nvm_Manufacturer_T * p_manufacture, Phase_AnalogCalibration_T * p_buffer);

NvMemory_Status_T MotNvm_WriteConstFrom(const MotNvm_T * p_motNvm, HAL_Nvm_Manufacturer_T * p_source);

#endif