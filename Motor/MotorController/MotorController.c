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
    @file   MotorController.c
    @author FireSourcery
    @version V0
    @brief
*/
/******************************************************************************/
#include "MotorController.h"
#include <string.h>

void MotorController_Init(MotorController_T * p_mc)
{
    Flash_Init(p_mc->CONST.P_FLASH);
#if defined(CONFIG_MOTOR_CONTROLLER_USER_CONFIG_EEPROM)
    EEPROM_Init_Blocking(p_mc->CONST.P_EEPROM);
#endif

    if(p_mc->CONST.P_NVM_CONFIG != 0U) { memcpy(&p_mc->Config, p_mc->CONST.P_NVM_CONFIG, sizeof(MotorController_Config_T)); }
    if(p_mc->CONST.P_BOOT_REF != 0U) { p_mc->BootRef.Word = p_mc->CONST.P_BOOT_REF->Word; }

    // MotorController_LoadConfigDefault(p_mc);

    Analog_Init(p_mc->CONST.P_ANALOG);
    for(uint8_t iSerial = 0U; iSerial < p_mc->CONST.SERIAL_COUNT; iSerial++) { Serial_Init(&p_mc->CONST.P_SERIALS[iSerial]); }
#if defined(CONFIG_MOTOR_CONTROLLER_CAN_BUS_ENABLE)
    // if(p_mc->Config.IsCanEnable == true) { CanBus_Init(p_mc->CONST.P_CAN_BUS, p_mc->Config.CanServicesId); }
#endif

    MotAnalogUser_Init(&p_mc->AnalogUser);

    VMonitor_Init(&p_mc->VMonitorSource);
    VMonitor_Init(&p_mc->VMonitorSense);
    VMonitor_Init(&p_mc->VMonitorAccs);

    Thermistor_Init(&p_mc->ThermistorPcb);
    for (uint8_t iMosfets = 0U; iMosfets < MOTOR_CONTROLLER_HEAT_MOSFETS_COUNT; iMosfets++) { Thermistor_Init(&p_mc->MosfetsThermistors[iMosfets]); }

    Motor_Static_InitVSourceRef_V(p_mc->Config.VSourceRef);
    for (uint8_t iMotor = 0U; iMotor < p_mc->CONST.MOTOR_COUNT; iMotor++) { Motor_Init(&p_mc->CONST.P_MOTORS[iMotor]); }

    Blinky_Init(&p_mc->Buzzer);
    Blinky_Init(&p_mc->Meter);
    Pin_Output_Init(&p_mc->Relay);
    Debounce_Init(&p_mc->OptDin, 5U);

    Timer_Periodic_Init(&p_mc->TimerMillis, 1U);

    for(uint8_t iProtocol = 0U; iProtocol < p_mc->CONST.PROTOCOL_COUNT; iProtocol++) { Protocol_Init(&p_mc->CONST.P_PROTOCOLS[iProtocol]); }

#ifdef CONFIG_MOTOR_CONTROLLER_SHELL_ENABLE
    Shell_Init(&p_mc->Shell);
#endif

#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
    MotorController_ResetUnitsBatteryLife(p_mc);
#endif

    // /* Load derived values to RAM */
    // if (BootRef_IsValid() == false)
    // {
    //     MotorController_LoadConfigDefault(&MotorControllerMain);  /* Load runtime calculated */
    //     /* or prompt user, resets every boot until user saves params */
    // }
    StateMachine_Init(&p_mc->StateMachine);
}

/******************************************************************************/
/*!
    Set/Reset
*/
/******************************************************************************/
/*
    Set runtime Config (RAM copy) via abstraction layer functions (in user units)
    Convience function over p_mc->Config compile time initializers
    On first time boot up. propagate defaults
*/
void MotorController_LoadConfigDefault(MotorController_T * p_mc)
{
    VMonitor_SetNominal_MilliV(&p_mc->VMonitorSource, (uint32_t)p_mc->Config.VSourceRef * 1000U);
    VMonitor_ResetLimitsDefault(&p_mc->VMonitorSource);
    VMonitor_Enable(&p_mc->VMonitorSource);
    // VMonitor_ResetLimitsDefault(&p_mc->VMonitorAccs);
    // VMonitor_ResetLimitsDefault(&p_mc->VMonitorSense);
#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
    MotorController_ResetUnitsBatteryLife(p_mc);
#endif
    // for(uint8_t iMotor = 0U; iMotor < p_mc->CONST.MOTOR_COUNT; iMotor++)
    // {
    //     PID_SetTunings(&(p_mc->CONST.P_MOTORS[iMotor].PidSpeed), 1U, 1U, 1U, 2U, 0U, 0U);
    //     PID_SetTunings(&(p_mc->CONST.P_MOTORS[iMotor].PidIq), 1U, 1U, 1U, 2U, 0U, 0U);
    //     PID_SetTunings(&(p_mc->CONST.P_MOTORS[iMotor].PidId), 1U, 1U, 1U, 2U, 0U, 0U);
    // }
    MotorController_ResetBootDefault(p_mc); /* Set Boot Options Buffer in RAM */
    /* following boots will still reload defaults until user save */
    //  save to nvm? or wait user confirmation in gui
}

void MotorController_ResetBootDefault(MotorController_T * p_mc)
{
    static const BootRef_T BOOT_REF_DEFAULT = { .IsValid = BOOT_REF_IS_VALID_01, .FastBoot = 0U, .Beep = 1U, .Blink = 1U, }; /* Overwrite after first time boot */
    p_mc->BootRef.Word = BOOT_REF_DEFAULT.Word;
}

void MotorController_ResetVSourceActiveRef(MotorController_T * p_mc)
{
    Motor_Static_InitVSourceRef_Adcu(p_mc->CONST.CONVERSION_VSOURCE.P_STATE->Result); /* Set Motor Ref using read Value */
    for (uint8_t iMotor = 0U; iMotor < p_mc->CONST.MOTOR_COUNT; iMotor++) { Motor_ResetUnitsVabc(&p_mc->CONST.P_MOTORS[iMotor]); }
#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
    MotorController_User_SetBatteryLifeDefault(p_mc);
#endif
}

#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
void MotorController_ResetUnitsBatteryLife(MotorController_T * p_mc)
{
    Linear_ADC_Init(&p_mc->BatteryLife, p_mc->Config.BatteryZero_Adcu, p_mc->Config.BatteryFull_Adcu);
}
#endif


/******************************************************************************/
/*!
    Nvm Wrappers Call from State Machine
*/
/******************************************************************************/

/* Write Config */
static NvMemory_Status_T WriteNvm_Blocking(MotorController_T * p_mc, const void * p_nvm, const void * p_ram, size_t sizeBytes)
{
#if     defined(CONFIG_MOTOR_CONTROLLER_USER_CONFIG_EEPROM)
    return EEPROM_Write_Blocking(p_mc->CONST.P_EEPROM, (uintptr_t)p_nvm, p_ram, sizeBytes);
#elif   defined(CONFIG_MOTOR_CONTROLLER_USER_CONFIG_FLASH)
    assert(nvmemory_is_aligned((uintptr_t)p_nvm, FLASH_UNIT_WRITE_SIZE)); /* Compile time config ensure all structs are write unit size aligned */
    return Flash_Write_Blocking(p_mc->CONST.P_FLASH, (uintptr_t)p_nvm, p_ram, sizeBytes);
#endif
}

NvMemory_Status_T MotorController_SaveConfig_Blocking(MotorController_T * p_mc)
{
    NvMemory_Status_T status = NV_MEMORY_STATUS_SUCCESS;
    Motor_T * p_motor;
    Protocol_T * p_protocol;

#if defined(CONFIG_MOTOR_CONTROLLER_USER_CONFIG_FLASH)
    status = Flash_Erase_Blocking(p_mc->CONST.P_FLASH, p_mc->CONST.CONFIG_ADDRESS, p_mc->CONST.CONFIG_SIZE); /* Flash must erase parameters flash region before write */
#endif

    if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_mc, p_mc->CONST.P_NVM_CONFIG, &p_mc->Config, sizeof(MotorController_Config_T)); };
    if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_mc, p_mc->CONST.P_BOOT_REF, &p_mc->BootRef, sizeof(BootRef_T)); };
    if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_mc, p_mc->AnalogUser.CONST.P_CONFIG, &p_mc->AnalogUser.Config, sizeof(MotAnalogUser_Config_T)); };

    if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_mc, p_mc->VMonitorSource.CONST.P_CONFIG, &p_mc->VMonitorSource.Config, sizeof(VMonitor_Config_T)); };
    if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_mc, p_mc->VMonitorAccs.CONST.P_CONFIG, &p_mc->VMonitorAccs.Config, sizeof(VMonitor_Config_T)); };
    if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_mc, p_mc->VMonitorSense.CONST.P_CONFIG, &p_mc->VMonitorSense.Config, sizeof(VMonitor_Config_T)); };

    if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_mc, p_mc->ThermistorPcb.CONST.P_CONFIG, &p_mc->ThermistorPcb.Config, sizeof(Thermistor_Config_T)); };

    if (status == NV_MEMORY_STATUS_SUCCESS)
    {
        for (uint8_t iMosfets = 0U; iMosfets < MOTOR_CONTROLLER_HEAT_MOSFETS_COUNT; iMosfets++)
        {
            status = WriteNvm_Blocking(p_mc, p_mc->MosfetsThermistors[iMosfets].CONST.P_CONFIG, &p_mc->MosfetsThermistors[iMosfets].Config, sizeof(Thermistor_Config_T));
            if (status != NV_MEMORY_STATUS_SUCCESS) { break; }
        }
    };

    if (status == NV_MEMORY_STATUS_SUCCESS)
    {
        for (uint8_t iProtocol = 0U; iProtocol < p_mc->CONST.PROTOCOL_COUNT; iProtocol++)
        {
            p_protocol = &p_mc->CONST.P_PROTOCOLS[iProtocol];
            status = WriteNvm_Blocking(p_mc, p_protocol->CONST.P_CONFIG, &p_protocol->Config, sizeof(Protocol_Config_T));
            if (status != NV_MEMORY_STATUS_SUCCESS) { break; }
        }
    }

    if (status == NV_MEMORY_STATUS_SUCCESS)
    {
        for (uint8_t iMotor = 0U; iMotor < p_mc->CONST.MOTOR_COUNT; iMotor++)
        {
            p_motor = MotorController_GetPtrMotor(p_mc, iMotor);
            if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_mc, p_motor->CONST.P_NVM_CONFIG, &p_motor->Config, sizeof(Motor_Config_T)); }
            if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_mc, p_motor->Hall.CONST.P_NVM_CONFIG, &p_motor->Hall.Config, sizeof(Hall_Config_T)); }
            if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_mc, p_motor->Encoder.CONST.P_CONFIG, &p_motor->Encoder.Config, sizeof(Encoder_Config_T)); }
            if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_mc, p_motor->PidSpeed.CONST.P_CONFIG, &p_motor->PidSpeed.Config, sizeof(PID_Config_T)); }
            if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_mc, p_motor->PidIq.CONST.P_CONFIG, &p_motor->PidIq.Config, sizeof(PID_Config_T)); }
            if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_mc, p_motor->PidId.CONST.P_CONFIG, &p_motor->PidId.Config, sizeof(PID_Config_T)); }
            // if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_motor->PidIBus.CONST.P_CONFIG, &p_motor->PidIBus.Config, sizeof(PID_Config_T)); }
            if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_mc, p_motor->Thermistor.CONST.P_CONFIG, &p_motor->Thermistor.Config, sizeof(Thermistor_Config_T)); }
            if (status != NV_MEMORY_STATUS_SUCCESS) { break; }
        }
    }

#ifdef CONFIG_MOTOR_CONTROLLER_SHELL_ENABLE
    if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_mc, p_mc->Shell.CONST.P_CONFIG, &p_mc->Shell.Config, sizeof(Shell_Config_T)); };
#endif

    return status;
}

// eeprom only
NvMemory_Status_T MotorController_SaveBootReg_Blocking(MotorController_T * p_mc)
{
#if     defined(CONFIG_MOTOR_CONTROLLER_USER_CONFIG_EEPROM)
    return EEPROM_Write_Blocking(p_mc->CONST.P_EEPROM, (uintptr_t)p_mc->CONST.P_BOOT_REF, &p_mc->BootRef, sizeof(BootRef_T));
#elif   defined(CONFIG_MOTOR_CONTROLLER_USER_CONFIG_FLASH)
    return NV_MEMORY_STATUS_ERROR_OTHER; /* Must erase page. Save on all params */
#endif
}

NvMemory_Status_T MotorController_ReadManufacture_Blocking(MotorController_T * p_mc, uintptr_t onceAddress, uint8_t size, uint8_t * p_destBuffer)
{
#if     defined(CONFIG_MOTOR_CONTROLLER_MANUFACTURE_CONFIG_ONCE)
    // if(p_mc->CONST.MANUFACTURE_ADDRESS != 0) handle offset
    return Flash_ReadOnce_Blocking(p_mc->CONST.P_FLASH, onceAddress, size, p_destBuffer);
#elif   defined(CONFIG_MOTOR_CONTROLLER_MANUFACTURE_CONFIG_FLASH)
    if(size < p_mc->CONST.MANUFACTURE_SIZE) { memcpy(p_destBuffer, onceAddress, size); }
#endif
}

NvMemory_Status_T MotorController_WriteManufacture_Blocking(MotorController_T * p_mc, uintptr_t onceAddress, const uint8_t * p_sourceBuffer, uint8_t size)
{
#if     defined(CONFIG_MOTOR_CONTROLLER_MANUFACTURE_CONFIG_ONCE)
    return Flash_WriteOnce_Blocking(p_mc->CONST.P_FLASH, onceAddress, p_sourceBuffer, size);
#elif   defined(CONFIG_MOTOR_CONTROLLER_MANUFACTURE_CONFIG_FLASH)
    return Flash_Write_Blocking(p_mc->CONST.P_FLASH, onceAddress, p_dataBuffer, size);
#endif
}



