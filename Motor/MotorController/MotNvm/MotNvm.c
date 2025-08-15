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
        if (MotorAnalogRef_IsLoaded() == false) { status = MotNvm_LoadRef(p_motNvm); }
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
// alternatively extern from here
// extern NvMemory_Status_T MotNvm_LoadRef(const MotNvm_T * p_motNvm);

NvMemory_Status_T MotNvm_LoadAnalogRefFrom(const MotNvm_T * p_motNvm, const struct HAL_Nvm_Manufacturer * p_source)
{
    MotorAnalogRef_T motorRef =
    {
        .V_MAX_VOLTS = HAL_Nvm_Manufacturer_GetVMaxVolts(p_source),
        .I_MAX_AMPS =  HAL_Nvm_Manufacturer_GetIMaxAmps(p_source),
        .V_RATED_FRACT16 = HAL_Nvm_Manufacturer_GetVRated_Fract16(p_source),
        .I_RATED_PEAK_FRACT16 = HAL_Nvm_Manufacturer_GetIRatedPeak_Fract16(p_source),
    };

    return Flash_Write_Blocking(p_motNvm->P_FLASH, (uintptr_t)&MOTOR_ANALOG_REFERENCE, (const void *)&motorRef, sizeof(MotorAnalogRef_T));
}

// NvMemory_Status_T MotNvm_MapBoardRef(const void * p_manufacture, MotorAnalogRef_Board_T * p_buffer)

NvMemory_Status_T MotNvm_LoadBoardRefFrom(const MotNvm_T * p_motNvm, const struct HAL_Nvm_Manufacturer * p_source)
{
    //todo getter wrappers
    MotorAnalogRef_Board_T boardRef =
    {
        .V_RATED = p_source->V_RATED,
        .I_RATED_RMS = p_source->I_RATED_RMS,
        .I_PHASE_R_BASE = p_source->I_PHASE_R_BASE,
        .I_PHASE_R_MOSFETS = p_source->I_PHASE_R_MOSFETS,
        .I_PHASE_GAIN = p_source->I_PHASE_GAIN,
        .V_PHASE_R1 = p_source->V_PHASE_R1,
        .V_PHASE_R2 = p_source->V_PHASE_R2,
    };

    return Flash_Write_Blocking(p_motNvm->P_FLASH, (uintptr_t)&MOTOR_ANALOG_REFERENCE_BOARD, (const void *)&boardRef, sizeof(MotorAnalogRef_Board_T));
}

/******************************************************************************/
/*!
    with defined parameters
*/
/******************************************************************************/
NvMemory_Status_T MotNvm_LoadRef(const MotNvm_T * p_motNvm)
{
    struct HAL_Nvm_Manufacturer buffer;
    NvMemory_Status_T status = MotNvm_ReadManufacture_Blocking(p_motNvm, (uintptr_t)0U, sizeof(struct HAL_Nvm_Manufacturer), &buffer);
    if (status == NV_MEMORY_STATUS_SUCCESS) { status = MotNvm_LoadBoardRefFrom(p_motNvm, &buffer); }
    if (status == NV_MEMORY_STATUS_SUCCESS) { status = MotNvm_LoadAnalogRefFrom(p_motNvm, &buffer); }
    return status;
}



