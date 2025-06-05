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

    @brief
*/
/******************************************************************************/
#include "MotorController.h"
#include <string.h>

void MotorController_Init(const MotorController_T * p_context)
{
    MotorController_State_T * p_mc = p_context->P_ACTIVE;

    MotNvm_Init(&p_context->MOT_NVM);

    if (p_context->P_NVM_CONFIG != NULL) { memcpy(&p_mc->Config, p_context->P_NVM_CONFIG, sizeof(MotorController_Config_T)); }
    if (p_context->MOT_NVM.P_BOOT_REF != NULL) { p_mc->BootRef.Word = p_context->MOT_NVM.P_BOOT_REF->Word; }

    for (uint8_t iAnalog = 0U; iAnalog < p_context->ADC_COUNT; iAnalog++) { Analog_ADC_Init(&p_context->P_ANALOG_ADCS[iAnalog]); }
    for (uint8_t iSerial = 0U; iSerial < p_context->SERIAL_COUNT; iSerial++) { Serial_Init(&p_context->P_SERIALS[iSerial]); }
#if defined(CONFIG_MOTOR_CONTROLLER_CAN_BUS_ENABLE)
    // if(p_mc->Config.IsCanEnable == true) { CanBus_Init(p_context->P_CAN_BUS, p_mc->Config.CanServicesId); }
#endif

    MotAnalogUser_Init(&p_context->ANALOG_USER);

    VMonitor_Init(&p_context->V_SOURCE);
    VMonitor_Init(&p_context->V_ACCESSORIES);
    VMonitor_Init(&p_context->V_ANALOG);

    HeatMonitor_Init(&p_context->HEAT_PCB);
    HeatMonitor_Group_Init(&p_context->HEAT_MOSFETS);

    MotorAnalog_InitVSource_V(p_mc->Config.VSupplyRef);
    for (uint8_t iMotor = 0U; iMotor < p_context->MOTORS.LENGTH; iMotor++) { Motor_Init(&p_context->MOTORS.P_CONTEXTS[iMotor]); }

    Blinky_Init(&p_context->BUZZER);
    Blinky_Init(&p_context->METER);
    Pin_Output_Init(&p_context->RELAY_PIN);
    // UserDIn_Init(&p_context->OPT_DIN, 5U);

    Timer_Periodic_Init(&p_mc->TimerMillis, 1U);

    for(uint8_t iProtocol = 0U; iProtocol < p_context->PROTOCOL_COUNT; iProtocol++) { Protocol_Init(&p_context->P_PROTOCOLS[iProtocol]); }

#ifdef CONFIG_MOTOR_CONTROLLER_SHELL_ENABLE
    Shell_Init(&p_mc->Shell);
#endif

    /* Load derived values to RAM */
    // if (BootRef_IsValid() == false)
    // {
    //     MotorController_LoadConfigDefault(p_mc);  /* Load runtime calculated */
    //     /* or prompt user, resets every boot until user saves params */
    // }

    StateMachine_Init(&p_mc->StateMachine);
}

/******************************************************************************/
/*!
    Set/Reset
    Runtime only
*/
/******************************************************************************/
/*
    Set runtime Config (RAM copy) via abstraction layer functions (in user units)
    Convience function over p_mc->Config compile time initializers
    On first time boot up. propagate defaults
*/
void MotorController_LoadConfigDefault(const MotorController_T * p_context)
{
    RangeMonitor_Enable(&p_context->V_SOURCE);
    MotorController_ResetVSourceMonitorDefaults(p_context);

    // VMonitor_ResetLimitsDefault(&p_mc->VMonitorAccs);
    // VMonitor_ResetLimitsDefault(&p_mc->VMonitorSense);

    // for (uint8_t iMotor = 0U; iMotor < p_context->MOTORS.LENGTH; iMotor++)
    // {
    //     // optionall sync limits
    //     // reset sensor units
    // }

    /*
        following boots will still reload defaults until user save
    */
    MotorController_ResetBootDefault(p_context); /* Set Boot Options Buffer in RAM */
}

void MotorController_ResetBootDefault(MotorController_State_T * p_mc)
{
    static const BootRef_T BOOT_REF_DEFAULT = { .IsValid = BOOT_REF_IS_VALID_01, .FastBoot = 0U, .Beep = 1U, .Blink = 1U, }; /* Overwrite after first time boot */
    p_mc->BootRef.Word = BOOT_REF_DEFAULT.Word;
}


void MotorController_ResetVSourceMonitorDefaults(const MotorController_T * p_context)
{
    VMonitor_InitLimitsDefault(p_context->V_SOURCE.P_STATE, Linear_Voltage_AdcuOfV(p_context->V_SOURCE.P_LINEAR, p_context->P_ACTIVE->Config.VSupplyRef), 25, 15, 5);
}



/******************************************************************************/
/*!

*/
/******************************************************************************/
/* shorthand */
static inline NvMemory_Status_T WriteNvm(const MotorController_T * p_context, const void * p_nvm, const void * p_ram, size_t sizeBytes)
{
    return MotNvm_WriteConfig_Blocking(&p_context->MOT_NVM, p_nvm, p_ram, sizeBytes);
}


/* todo map */
NvMemory_Status_T MotorController_SaveConfig_Blocking(const MotorController_T * p_context)
{
    MotorController_State_T * p_mc = p_context->P_ACTIVE;

    NvMemory_Status_T status = NV_MEMORY_STATUS_SUCCESS;
    Motor_T * p_motorContext;
    Motor_State_T * p_motor;
    Protocol_T * p_protocol;

#if defined(CONFIG_MOTOR_CONTROLLER_USER_NVM_FLASH)
    status = Flash_Erase_Blocking(p_context->MOT_NVM.P_FLASH, p_context->MOT_NVM.MAIN_CONFIG_ADDRESS, p_context->MOT_NVM.MAIN_CONFIG_SIZE);
#endif

    if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm(p_context, p_context->MOT_NVM.P_BOOT_REF, &p_mc->BootRef, sizeof(BootRef_T)); }
    if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm(p_context, p_context->P_NVM_CONFIG, &p_mc->Config, sizeof(MotorController_Config_T)); }
    if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm(p_context, p_context->ANALOG_USER.P_NVM_CONFIG, &p_context->ANALOG_USER.P_STATE->Config, sizeof(MotAnalogUser_Config_T)); }

    /* Fixed: Use the new monitor structure members */
    if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm(p_context, p_context->V_SOURCE.P_NVM_CONFIG, &p_context->V_SOURCE.P_STATE->Config, sizeof(VMonitor_Config_T)); }
    if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm(p_context, p_context->V_ACCESSORIES.P_NVM_CONFIG, &p_context->V_ACCESSORIES.P_STATE->Config, sizeof(VMonitor_Config_T)); }
    if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm(p_context, p_context->V_ANALOG.P_NVM_CONFIG, &p_context->V_ANALOG.P_STATE->Config, sizeof(VMonitor_Config_T)); }

    if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm(p_context, p_context->HEAT_PCB.P_NVM_CONFIG, &p_context->HEAT_PCB.P_STATE->Config, sizeof(HeatMonitor_Config_T)); }

    /* Fixed: Use the new group structure for MOSFET heat monitors */
    if (status == NV_MEMORY_STATUS_SUCCESS)
    {
        for (uint8_t iMosfet = 0U; iMosfet < p_context->HEAT_MOSFETS.COUNT; iMosfet++)
        {
            status = WriteNvm(p_context, p_context->HEAT_MOSFETS.P_CONTEXTS[iMosfet].P_NVM_CONFIG, &p_context->HEAT_MOSFETS.P_CONTEXTS[iMosfet].P_STATE->Config, sizeof(HeatMonitor_Config_T));
            if (status != NV_MEMORY_STATUS_SUCCESS) { break; }
        }
    }

    if (status == NV_MEMORY_STATUS_SUCCESS)
    {
        for (uint8_t iProtocol = 0U; iProtocol < p_context->PROTOCOL_COUNT; iProtocol++)
        {
            p_protocol = &p_context->P_PROTOCOLS[iProtocol];
            status = WriteNvm(p_context, p_protocol->CONST.P_CONFIG, &p_protocol->Config, sizeof(Protocol_Config_T));
            if (status != NV_MEMORY_STATUS_SUCCESS) { break; }
        }
    }

    if (status == NV_MEMORY_STATUS_SUCCESS)
    {
        for (uint8_t iMotor = 0U; iMotor < p_context->MOTORS.LENGTH; iMotor++)
        {
            p_motor = MotorController_MotorStateAt(p_context, iMotor);
            p_motorContext = MotorController_MotorContextAt(p_context, iMotor);
            if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm(p_context, p_motorContext[iMotor].P_NVM_CONFIG, &p_motor->Config, sizeof(Motor_Config_T)); }
            if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm(p_context, p_motorContext[iMotor].SENSOR_TABLE.HALL.HALL.P_NVM_CONFIG, &p_motorContext[iMotor].SENSOR_TABLE.HALL.HALL.P_STATE->Config, sizeof(Motor_Config_T)); }

            // if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm(p_context, &p_motor->Config.PidSpeed, &p_motor->PidSpeed.Config, sizeof(PID_Config_T)); }
            // if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm(p_context, &p_motor->Config.PidI, &p_motor->PidIq.Config, sizeof(PID_Config_T)); }
            // if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm(p_context, &p_motor->Config.PidI, & p_motor->PidId.Config, sizeof(PID_Config_T)); }
            if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm(p_context, p_motorContext->HEAT_MONITOR_CONTEXT.P_NVM_CONFIG, &p_motorContext->HEAT_MONITOR_CONTEXT.P_STATE->Config, sizeof(HeatMonitor_Config_T)); }
            if (status != NV_MEMORY_STATUS_SUCCESS) { break; }
        }
    }

#ifdef CONFIG_MOTOR_CONTROLLER_SHELL_ENABLE
    if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm(p_context, p_mc->Shell.CONST.P_CONFIG, &p_mc->Shell.Config, sizeof(Shell_Config_T)); }
#endif

    return status;
}