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

    MotNvm_Entry_T * P_PARTITIONS; /* NVM Partitions */
    size_t PARTITION_COUNT;        /* Number of NVM Partitions */

    // alternatively use  P_FLASH->P_PARTITIONS[id]
    uintptr_t MANUFACTURE_ADDRESS;
    uint8_t MANUFACTURE_SIZE;

    // Phase_Calibration_T * P_PHASE_ANALOG_REF; /* Motor Analog Reference */
    const BootRef_T * P_BOOT_REF;
}
MotNvm_T;

/*
*/
extern void MotNvm_Init(const MotNvm_T * p_motNvm);
extern NvMemory_Status_T MotNvm_Write_Blocking(const MotNvm_T * p_motNvm, const void * p_rom, const void * p_ram, size_t sizeBytes);
extern NvMemory_Status_T MotNvm_ReadManufacture_Blocking(const MotNvm_T * p_motNvm, uintptr_t onceAddress, uint8_t size, void * p_destBuffer);
extern NvMemory_Status_T MotNvm_WriteManufacture_Blocking(const MotNvm_T * p_motNvm, uintptr_t onceAddress, const void * p_sourceBuffer, uint8_t size);
extern NvMemory_Status_T MotNvm_SaveBootReg_Blocking(const MotNvm_T * p_motNvm);
extern NvMemory_Status_T MotNvm_SaveConfigAll_Blocking(const MotNvm_T * p_motNvm);

extern NvMemory_Status_T MotNvm_LoadConstRef(const MotNvm_T * p_motNvm);

/* externally define */
// struct HAL_Nvm_Once_T;
struct HAL_Nvm_Manufacturer;
typedef const struct HAL_Nvm_Manufacturer HAL_Nvm_Manufacturer_T;

extern NvMemory_Status_T MotNvm_MapPhaseCalibrationRef(const HAL_Nvm_Manufacturer_T * p_manufacture, Phase_Calibration_T * p_buffer);
extern NvMemory_Status_T MotNvm_MapPhaseAnalogSensorRef(const HAL_Nvm_Manufacturer_T * p_manufacture, Phase_AnalogSensor_T * p_buffer);

#endif