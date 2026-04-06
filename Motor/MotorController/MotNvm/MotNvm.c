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
#if defined(MOTOR_CONTROLLER_USER_NVM_EEPROM)
    EEPROM_Init_Blocking(p_motNvm->P_EEPROM);
#endif
}

NvMemory_Status_T MotNvm_Write_Blocking(const MotNvm_T * p_motNvm, const void * p_rom, const void * p_ram, size_t size)
{
#if     defined(MOTOR_CONTROLLER_USER_NVM_EEPROM)
    return EEPROM_Write_Blocking(p_motNvm->P_EEPROM, (uintptr_t)p_rom, p_ram, size);
#elif   defined(MOTOR_CONTROLLER_USER_NVM_FLASH)
    assert(nvmemory_is_aligned((uintptr_t)p_rom, FLASH_UNIT_WRITE_SIZE));
    return Flash_Write_Blocking(p_motNvm->P_FLASH, (uintptr_t)p_rom, p_ram, size);
#endif
}

NvMemory_Status_T MotNvm_Read_Blocking(const MotNvm_T * p_motNvm, const void * p_rom, size_t size, void * p_dest)
{
    memcpy(p_dest, (void *)p_rom, size);
    return NV_MEMORY_STATUS_SUCCESS;
}

NvMemory_Status_T MotNvm_ReadManufacture_Blocking(const MotNvm_T * p_motNvm, uintptr_t address, uint8_t size, void * p_dest)
{
#if     defined(MOTOR_CONTROLLER_MANUFACTURE_NVM_ONCE)
    return Flash_ReadOnce_Blocking(p_motNvm->P_FLASH, address, size, p_dest);  // if(p_motNvm->MANUFACTURE_ADDRESS != 0) handle offset
#elif   defined(MOTOR_CONTROLLER_MANUFACTURE_NVM_FLASH)
    return (size <= p_motNvm->MANUFACTURE_SIZE) ? ({ memcpy(p_dest, (void *)address, size); NV_MEMORY_STATUS_SUCCESS; }) : NV_MEMORY_STATUS_ERROR_OTHER;
#endif
}

NvMemory_Status_T _MotNvm_WriteManufacture_Blocking(const MotNvm_T * p_motNvm, uintptr_t address, const void * p_source, uint8_t size)
{
#if     defined(MOTOR_CONTROLLER_MANUFACTURE_NVM_ONCE)
    return Flash_WriteOnce_Blocking(p_motNvm->P_FLASH, address, p_source, size);
#elif   defined(MOTOR_CONTROLLER_MANUFACTURE_NVM_FLASH)
    return Flash_Write_Blocking(p_motNvm->P_FLASH, address, p_source, size);
#endif
}

/* Writable if each byte is either erased or matches the source. */
static bool IsManufactureWritable(uint8_t * p_image, const uint8_t * p_source, uint8_t size)
{
    for (uint8_t i = 0U; i < size; i++) { if (p_image[i] != FLASH_UNIT_ERASE_PATTERN && p_image[i] != p_source[i]) { return false; } }
    return true;
}

NvMemory_Status_T MotNvm_WriteManufacture_Blocking(const MotNvm_T * p_motNvm, uintptr_t address, const void * p_source, uint8_t size)
{
    uint8_t read[64U] = { 0 };
    NvMemory_Status_T status = MotNvm_ReadManufacture_Blocking(p_motNvm, address, size, &read[0U]);

    if (memcmp(&read[0U], p_source, size) == 0U) { status = NV_MEMORY_STATUS_SUCCESS; } /* Already matches. */
    else if (IsManufactureWritable(read, (const uint8_t *)p_source, size))
    {
        status = _MotNvm_WriteManufacture_Blocking(p_motNvm, address, p_source, size);
    }

    /* WriteManufacture may be called over partial address ranges */
    /* Assume sequential write, propagate on last */
    if (address + size == p_motNvm->MANUFACTURE_ADDRESS + p_motNvm->MANUFACTURE_SIZE)
    {
        /* read its entirety */
        status = MotNvm_ReadManufacture_Blocking(p_motNvm, p_motNvm->MANUFACTURE_ADDRESS, p_motNvm->MANUFACTURE_SIZE, &read[0U]);
        if (status == NV_MEMORY_STATUS_SUCCESS) { status = MotNvm_WriteConstFrom(p_motNvm, (HAL_Nvm_Manufacturer_T *)&read[0U]); }
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
#if     defined(MOTOR_CONTROLLER_USER_NVM_EEPROM)
    return EEPROM_Write_Blocking(p_motNvm->P_EEPROM, (uintptr_t)p_motNvm->P_BOOT_REF, &p_mc->BootRef, sizeof(BootRef_T));
#elif   defined(MOTOR_CONTROLLER_USER_NVM_FLASH)
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

#if defined(MOTOR_CONTROLLER_USER_NVM_FLASH)
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

void MotNvm_LoadConfigAll(const MotNvm_T * p_motNvm)
{
    for (size_t i = 0U; i < p_motNvm->PARTITION_COUNT; i++)
    {
        memcpy(p_motNvm->P_PARTITIONS[i].RAM_ADDRESS, p_motNvm->P_PARTITIONS[i].NVM_ADDRESS, p_motNvm->P_PARTITIONS[i].SIZE);
    }
}

/******************************************************************************/
/*!

*/
/******************************************************************************/
NvMemory_Status_T MotNvm_WritePhaseCalibrationFrom(const MotNvm_T * p_motNvm, HAL_Nvm_Manufacturer_T * p_source)
{
    Phase_Calibration_T buffer = { 0 };
    HAL_Nvm_MapPhaseCalibration(p_source, &buffer);
    return Flash_Write_Blocking(p_motNvm->P_FLASH, (uintptr_t)&PHASE_CALIBRATION, (const void *)&buffer, sizeof(Phase_Calibration_T));
}

NvMemory_Status_T MotNvm_WritePhaseSensorRefFrom(const MotNvm_T * p_motNvm, HAL_Nvm_Manufacturer_T * p_source)
{
    Phase_AnalogSensor_T buffer = { 0 };
    HAL_Nvm_MapPhaseAnalogSensorRef(p_source, &buffer); // callee cast away const
    return Flash_Write_Blocking(p_motNvm->P_FLASH, (uintptr_t)&PHASE_ANALOG_SENSOR_REF, (const void *)&buffer, sizeof(Phase_AnalogSensor_T));
}

NvMemory_Status_T MotNvm_WriteConstFrom(const MotNvm_T * p_motNvm, HAL_Nvm_Manufacturer_T * p_source)
{
    NvMemory_Status_T status = NV_MEMORY_STATUS_SUCCESS;
    status = Flash_Erase_Blocking(p_motNvm->P_FLASH, (uintptr_t)&PHASE_CALIBRATION, FLASH_UNIT_ERASE_SIZE); /* assume contigous start from PHASE_CALIBRATION  */

    if (status == NV_MEMORY_STATUS_SUCCESS) { status = MotNvm_WritePhaseSensorRefFrom(p_motNvm, p_source); }
    if (status == NV_MEMORY_STATUS_SUCCESS) { status = MotNvm_WritePhaseCalibrationFrom(p_motNvm, p_source); }
    return status;
}

/******************************************************************************/
/*!
    with defined parameters
*/
/******************************************************************************/
NvMemory_Status_T MotNvm_WriteConstRef(const MotNvm_T * p_motNvm)
{
    // NvMemory_Status_T status = NV_MEMORY_STATUS_SUCCESS;
    uint8_t buffer[64U] = { 0 };
    NvMemory_Status_T status = MotNvm_ReadManufacture_Blocking(p_motNvm, p_motNvm->MANUFACTURE_ADDRESS, p_motNvm->MANUFACTURE_SIZE, (void *)&buffer[0U]);

    // if (HAL_Nvm_IsValidManufacture(p_motNvm->MANUFACTURE_ADDRESS, p_motNvm->MANUFACTURE_SIZE) == true)

    status = Flash_Erase_Blocking(p_motNvm->P_FLASH, (uintptr_t)&PHASE_CALIBRATION, FLASH_UNIT_ERASE_SIZE);

    if (status == NV_MEMORY_STATUS_SUCCESS) { status = MotNvm_WritePhaseSensorRefFrom(p_motNvm, (HAL_Nvm_Manufacturer_T *)&buffer[0U]); }
    if (status == NV_MEMORY_STATUS_SUCCESS) { status = MotNvm_WritePhaseCalibrationFrom(p_motNvm, (HAL_Nvm_Manufacturer_T *)&buffer[0U]); }
    return status;
}
