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
#include "MotNvm.h"

#include <string.h>

void MotNvm_Init(const MotNvm_T * p_motNvm)
{
    Flash_Init(p_motNvm->P_FLASH);
#if defined(CONFIG_MOTOR_CONTROLLER_USER_NVM_EEPROM)
    EEPROM_Init_Blocking(p_motNvm->P_EEPROM);
#endif
}

NvMemory_Status_T MotNvm_Write_Blocking(const MotNvm_T * p_motNvm, const void * p_rom, const void * p_ram, size_t sizeBytes)
{
#if     defined(CONFIG_MOTOR_CONTROLLER_USER_NVM_EEPROM)
    return EEPROM_Write_Blocking(p_motNvm->P_EEPROM, (uintptr_t)p_rom, p_ram, sizeBytes);
#elif   defined(CONFIG_MOTOR_CONTROLLER_USER_NVM_FLASH)
    assert(nvmemory_is_aligned((uintptr_t)p_rom, FLASH_UNIT_WRITE_SIZE));
    // assert(nvmemory_is_aligned((uintptr_t)p_ram, FLASH_UNIT_WRITE_SIZE));
    return Flash_Write_Blocking(p_motNvm->P_FLASH, (uintptr_t)p_rom, p_ram, sizeBytes);
#endif
}

NvMemory_Status_T MotNvm_ReadManufacture_Blocking(const MotNvm_T * p_motNvm, uintptr_t onceAddress, uint8_t size, void * p_destBuffer)
{
#if     defined(CONFIG_MOTOR_CONTROLLER_MANUFACTURE_NVM_ONCE)
    return Flash_ReadOnce_Blocking(p_motNvm->P_FLASH, onceAddress, size, p_destBuffer);  // if(p_motNvm->MANUFACTURE_ADDRESS != 0) handle offset
#elif   defined(CONFIG_MOTOR_CONTROLLER_MANUFACTURE_NVM_FLASH)
    return (size <= p_motNvm->MANUFACTURE_SIZE) ? ({ memcpy(p_destBuffer, onceAddress, size); true; }) : false;
#endif
}

NvMemory_Status_T _MotNvm_WriteManufacture_Blocking(const MotNvm_T * p_motNvm, uintptr_t onceAddress, const void * p_sourceBuffer, uint8_t size)
{
#if     defined(CONFIG_MOTOR_CONTROLLER_MANUFACTURE_NVM_ONCE)
    return Flash_WriteOnce_Blocking(p_motNvm->P_FLASH, onceAddress, p_sourceBuffer, size);
#elif   defined(CONFIG_MOTOR_CONTROLLER_MANUFACTURE_NVM_FLASH)
    return Flash_Write_Blocking(p_motNvm->P_FLASH, onceAddress, p_dataBuffer, size);
#endif
}

NvMemory_Status_T MotNvm_WriteManufacture_Blocking(const MotNvm_T * p_motNvm, uintptr_t onceAddress, const void * p_sourceBuffer, uint8_t size)
{
    NvMemory_Status_T status = _MotNvm_WriteManufacture_Blocking(p_motNvm, onceAddress, p_sourceBuffer, size);
    if (status == NV_MEMORY_STATUS_SUCCESS)
    {
        /* Load the motor analog reference after writing the manufacture data */
        /* shared check for BoardRef */
        if (Phase_Calibration_IsLoaded() == false) { status = MotNvm_LoadConstRef(p_motNvm); }
    }
    return status;
}

/******************************************************************************/
/*!
    User Config
    with defined parameters
*/
/******************************************************************************/
/* eeprom only. Save on first init. */
NvMemory_Status_T MotNvm_SaveBootReg_Blocking(const MotNvm_T * p_motNvm)
{
#if     defined(CONFIG_MOTOR_CONTROLLER_USER_NVM_EEPROM)
    return EEPROM_Write_Blocking(p_motNvm->P_EEPROM, (uintptr_t)p_motNvm->P_BOOT_REF, &p_mc->BootRef, sizeof(BootRef_T));
#elif   defined(CONFIG_MOTOR_CONTROLLER_USER_NVM_FLASH)
    return NV_MEMORY_STATUS_ERROR_OTHER; /* Save on all params */
#endif
}

static NvMemory_Status_T SaveEntry_Blocking(const MotNvm_T * p_motNvm, const MotNvm_Entry_T * p_entry)
{
    assert(p_entry != NULL);
    assert(p_entry->NVM_ADDRESS != NULL);
    assert(p_entry->RAM_ADDRESS != NULL);
    assert(p_entry->SIZE > 0U);

    return MotNvm_Write_Blocking(p_motNvm, p_entry->NVM_ADDRESS, p_entry->RAM_ADDRESS, p_entry->SIZE);
}

NvMemory_Status_T MotNvm_SaveConfigAll_Blocking(const MotNvm_T * p_motNvm)
{
    NvMemory_Status_T status;

#if defined(CONFIG_MOTOR_CONTROLLER_USER_NVM_FLASH)
    /* Flash Erase Full block */
    status = Flash_Erase_Blocking(p_motNvm->P_FLASH, p_motNvm->MAIN_CONFIG_ADDRESS, p_motNvm->MAIN_CONFIG_SIZE);
    if (status != NV_MEMORY_STATUS_SUCCESS) { return status; }
#endif

    for (size_t i = 0U; i < p_motNvm->PARTITION_COUNT; i++)
    {
        status = SaveEntry_Blocking(p_motNvm, &p_motNvm->P_PARTITIONS[i]);
        if (status != NV_MEMORY_STATUS_SUCCESS) { break; }
    }

    return status;
}

/******************************************************************************/
/*!

*/
/******************************************************************************/
NvMemory_Status_T MotNvm_LoadPhaseCalibrationRefFrom(const MotNvm_T * p_motNvm, HAL_Nvm_Manufacturer_T * p_source)
{
    Phase_Calibration_T buffer = { 0 };
    MotNvm_MapPhaseCalibrationRef(p_source, &buffer);
    return Flash_Write_Blocking(p_motNvm->P_FLASH, (uintptr_t)&PHASE_CALIBRATION, (const void *)&buffer, sizeof(Phase_Calibration_T));
}

NvMemory_Status_T MotNvm_LoadPhaseSensorRefFrom(const MotNvm_T * p_motNvm, HAL_Nvm_Manufacturer_T * p_source)
{
    Phase_AnalogSensor_T buffer = { 0 };
    MotNvm_MapPhaseAnalogSensorRef(p_source, &buffer); // callee cast away const
    return Flash_Write_Blocking(p_motNvm->P_FLASH, (uintptr_t)&PHASE_ANALOG_SENSOR_REF, (const void *)&buffer, sizeof(Phase_AnalogSensor_T));
}

/******************************************************************************/
/*!
    with defined parameters
*/
/******************************************************************************/
NvMemory_Status_T MotNvm_LoadConstRef(const MotNvm_T * p_motNvm)
{
    HAL_Nvm_Manufacturer_T result;
    NvMemory_Status_T status = MotNvm_ReadManufacture_Blocking(p_motNvm, (uintptr_t)0U, sizeof(HAL_Nvm_Manufacturer_T), (void *)&result);
    if (status == NV_MEMORY_STATUS_SUCCESS) { status = MotNvm_LoadPhaseSensorRefFrom(p_motNvm, &result); }
    if (status == NV_MEMORY_STATUS_SUCCESS) { status = MotNvm_LoadPhaseCalibrationRefFrom(p_motNvm, &result); }
    return status;
}



