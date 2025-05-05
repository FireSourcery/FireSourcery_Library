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
#include "MotNvm.h"

void MotNvm_Init(MotNvm_T * p_this)
{
    Flash_Init(p_this->P_FLASH);
#if defined(CONFIG_MOTOR_CONTROLLER_USER_NVM_EEPROM)
    EEPROM_Init_Blocking(p_this->P_EEPROM);
#endif
}

NvMemory_Status_T MotNvm_WriteConfig_Blocking(MotNvm_T * p_this, const void * p_nvm, const void * p_ram, size_t sizeBytes)
{
#if     defined(CONFIG_MOTOR_CONTROLLER_USER_NVM_EEPROM)
    return EEPROM_Write_Blocking(p_this->P_EEPROM, (uintptr_t)p_nvm, p_ram, sizeBytes);
#elif   defined(CONFIG_MOTOR_CONTROLLER_USER_NVM_FLASH)
    assert(nvmemory_is_aligned((uintptr_t)p_nvm, FLASH_UNIT_WRITE_SIZE));
    return Flash_Write_Blocking(p_this->P_FLASH, (uintptr_t)p_nvm, p_ram, sizeBytes);
#endif
}

NvMemory_Status_T MotNvm_ReadManufacture_Blocking(MotNvm_T * p_this, uintptr_t onceAddress, uint8_t size, uint8_t * p_destBuffer)
{
#if     defined(CONFIG_MOTOR_CONTROLLER_MANUFACTURE_NVM_ONCE)
    return Flash_ReadOnce_Blocking(p_this->P_FLASH, onceAddress, size, p_destBuffer);  // if(p_this->MANUFACTURE_ADDRESS != 0) handle offset
#elif   defined(CONFIG_MOTOR_CONTROLLER_MANUFACTURE_NVM_FLASH)
    if (size < p_this->MANUFACTURE_SIZE) { memcpy(p_destBuffer, onceAddress, size); }
#endif
}

NvMemory_Status_T MotNvm_WriteManufacture_Blocking(MotNvm_T * p_this, uintptr_t onceAddress, const uint8_t * p_sourceBuffer, uint8_t size)
{
#if     defined(CONFIG_MOTOR_CONTROLLER_MANUFACTURE_NVM_ONCE)
    return Flash_WriteOnce_Blocking(p_this->P_FLASH, onceAddress, p_sourceBuffer, size);
#elif   defined(CONFIG_MOTOR_CONTROLLER_MANUFACTURE_NVM_FLASH)
    return Flash_Write_Blocking(p_this->P_FLASH, onceAddress, p_dataBuffer, size);
#endif
}

/* eeprom only. Save on first init. */
NvMemory_Status_T MotNvm_SaveBootReg_Blocking(MotNvm_T * p_this)
{
#if     defined(CONFIG_MOTOR_CONTROLLER_USER_NVM_EEPROM)
    return EEPROM_Write_Blocking(p_this->P_EEPROM, (uintptr_t)p_this->P_BOOT_REF, &p_mc->BootRef, sizeof(BootRef_T));
#elif   defined(CONFIG_MOTOR_CONTROLLER_USER_NVM_FLASH)
    return NV_MEMORY_STATUS_ERROR_OTHER; /* Save on all params */
#endif
}

NvMemory_Status_T MotNvm_LoadMotorRefFrom(MotNvm_T * p_this, const struct HAL_Nvm_Manufacturer * p_source)
{
    MotorAnalogRef_T buffer =
    {
        .V_MAX_VOLTS = p_source->V_MAX_VOLTS,
        .I_MAX_AMPS = p_source->I_MAX_AMPS,
        .V_RATED_FRACT16 = HAL_Nvm_Manufacturer_GetVRated_Fract16(p_source),
        .I_RATED_PEAK_FRACT16 = HAL_Nvm_Manufacturer_GetIRatedPeak_Fract16(p_source),
    };

    return Flash_Write_Blocking(p_this->P_FLASH, (uintptr_t)&MOTOR_ANALOG_REFERENCE, (uint8_t *)&buffer, sizeof(MotorAnalogRef_T));
}

NvMemory_Status_T MotNvm_LoadMotorRef(MotNvm_T * p_this)
{
    struct HAL_Nvm_Manufacturer buffer;
    MotNvm_ReadManufacture_Blocking(p_this, (uintptr_t)0U, sizeof(struct HAL_Nvm_Manufacturer), (uint8_t *)&buffer);
    return MotNvm_LoadMotorRefFrom(p_this, &buffer);
}

NvMemory_Status_T MotNvm_LoadMotorBoardRefFrom(MotNvm_T * p_this, const struct HAL_Nvm_Manufacturer * p_source)
{
    MotorAnalogRef_Board_T buffer =
    {
        .V_RATED = p_source->V_RATED,
        .I_RATED_RMS = p_source->I_RATED_RMS,
        .I_PHASE_R_BASE = p_source->I_PHASE_R_BASE,
        .I_PHASE_R_MOSFETS = p_source->I_PHASE_R_MOSFETS,
        .I_PHASE_GAIN = p_source->I_PHASE_GAIN,
        .V_PHASE_R1 = p_source->V_PHASE_R1,
        .V_PHASE_R2 = p_source->V_PHASE_R2,
    };

    return Flash_Write_Blocking(p_this->P_FLASH, (uintptr_t)&MOTOR_ANALOG_REFERENCE_BOARD, (uint8_t *)&buffer, sizeof(MotorAnalogRef_Board_T));
}

NvMemory_Status_T MotNvm_LoadMotorBoardRef(MotNvm_T * p_this)
{
    struct HAL_Nvm_Manufacturer buffer;
    MotNvm_ReadManufacture_Blocking(p_this, (uintptr_t)0U, sizeof(struct HAL_Nvm_Manufacturer), (uint8_t *)&buffer);
    return MotNvm_LoadMotorBoardRefFrom(p_this, &buffer);
}

