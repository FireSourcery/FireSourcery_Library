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

void MotorController_Init(MotorControllerPtr_T p_mc)
{
    Flash_Init(p_mc->CONFIG.P_FLASH);
#if defined(CONFIG_MOTOR_CONTROLLER_PARAMETERS_EEPROM)
    EEPROM_Init_Blocking(p_mc->CONFIG.P_EEPROM);
#endif

    if(p_mc->CONFIG.P_PARAMS_NVM != 0U) { memcpy(&p_mc->Parameters, p_mc->CONFIG.P_PARAMS_NVM, sizeof(MotorController_Params_T)); }
    if(p_mc->CONFIG.P_MEM_MAP_BOOT != 0U) { p_mc->MemMapBoot.Word = p_mc->CONFIG.P_MEM_MAP_BOOT->Word; }

    // MotorController_LoadParamsDefault(p_mc);

    AnalogN_Init(p_mc->CONFIG.P_ANALOG_N);
    for(uint8_t iSerial = 0U; iSerial < p_mc->CONFIG.SERIAL_COUNT; iSerial++) { Serial_Init(&p_mc->CONFIG.P_SERIALS[iSerial]); }
#if defined(CONFIG_MOTOR_CONTROLLER_CAN_BUS_ENABLE)
    // if(p_mc->Parameters.IsCanEnable == true) { CanBus_Init(p_mc->CONFIG.P_CAN_BUS, p_mc->Parameters.CanServicesId); }
#endif

    MotAnalogUser_Init(&p_mc->AnalogUser);
    VMonitor_Init(&p_mc->VMonitorSource);
    VMonitor_Init(&p_mc->VMonitorSense);
    VMonitor_Init(&p_mc->VMonitorAccs);

    // todo as array
    // for(uint8_t iTherm = 0U; iTherm < p_mc->CONFIG.THERMISTOR_COUNT; iTherm++) { Thermistor_Init(&p_mc->CONFIG.P_THERMISTORS[iTherm]); }
    Thermistor_Init(&p_mc->ThermistorPcb);
#if defined(CONFIG_MOTOR_CONTROLLER_HEAT_MOSFETS_TOP_BOT_ENABLE)
    Thermistor_Init(&p_mc->ThermistorMosfetsTop);
    Thermistor_Init(&p_mc->ThermistorMosfetsBot);
#else
    Thermistor_Init(&p_mc->ThermistorMosfets);
#endif
    // p_mc->AnalogResults.VAccs_Adcu = VMonitor_GetNominal(&p_mc->VMonitorAccs);
    // p_mc->AnalogResults.VSense_Adcu =
    // p_mc->AnalogResults.VSource_Adcu =
    if(Thermistor_IsMonitorEnable(&p_mc->ThermistorPcb) == true) { p_mc->AnalogResults.HeatPcb_Adcu = Thermistor_GetWarningThreshold_Adcu(&p_mc->ThermistorPcb); };
    if(Thermistor_IsMonitorEnable(&p_mc->ThermistorMosfets) == true) { p_mc->AnalogResults.HeatMosfets_Adcu = Thermistor_GetWarningThreshold_Adcu(&p_mc->ThermistorMosfets); };

    Global_Motor_InitVSourceRef_V(p_mc->Parameters.VSourceRef);
    for(uint8_t iMotor = 0U; iMotor < p_mc->CONFIG.MOTOR_COUNT; iMotor++) { Motor_Init(&p_mc->CONFIG.P_MOTORS[iMotor]); }

    Blinky_Init(&p_mc->Buzzer);
    Blinky_Init(&p_mc->Meter);
    Pin_Output_Init(&p_mc->Relay);
    Debounce_Init(&p_mc->OptDin, 5U);

    Timer_Periodic_Init(&p_mc->TimerMillis, 1U);

    for(uint8_t iProtocol = 0U; iProtocol < p_mc->CONFIG.PROTOCOL_COUNT; iProtocol++) { Protocol_Init(&p_mc->CONFIG.P_PROTOCOLS[iProtocol]); }

#ifdef CONFIG_MOTOR_CONTROLLER_SHELL_ENABLE
    Shell_Init(&p_mc->Shell);
#endif

#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
    MotorController_ResetUnitsBatteryLife(p_mc);
#endif

    StateMachine_Init(&p_mc->StateMachine);
}

void MotorController_PollFaultFlags(MotorController_T * p_mc)
{
    // p_mc->FaultFlags.Motors         = (MotorController_ClearMotorsFaultAll(p_mc) == false);
    p_mc->FaultFlags.VSenseLimit    = VMonitor_GetIsFault(&p_mc->VMonitorSense);
    p_mc->FaultFlags.VAccsLimit     = VMonitor_GetIsFault(&p_mc->VMonitorAccs);
    p_mc->FaultFlags.VSourceLimit   = VMonitor_GetIsFault(&p_mc->VMonitorSource);
    p_mc->FaultFlags.PcbOverheat    = Thermistor_GetIsFault(&p_mc->ThermistorPcb);
#if defined(CONFIG_MOTOR_CONTROLLER_HEAT_MOSFETS_TOP_BOT_ENABLE)
    p_mc->FaultFlags.MosfetsTopOverHeat = Thermistor_GetIsFault(&p_mc->ThermistorMosfetsTop);
    p_mc->FaultFlags.MosfetsBotOverHeat = Thermistor_GetIsFault(&p_mc->ThermistorMosfetsBot);
#else
    p_mc->FaultFlags.MosfetsOverheat = Thermistor_GetIsFault(&p_mc->ThermistorMosfets);
#endif
}



#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
void MotorController_ResetUnitsBatteryLife(MotorControllerPtr_T p_mc)
{
    Linear_ADC_Init(&p_mc->BatteryLife, p_mc->Parameters.BatteryZero_Adcu, p_mc->Parameters.BatteryFull_Adcu, 0U, 1000U);
}
#endif

/*
    Set runtime Params (RAM copy) via abstraction layer functions (in user units)
    Convience function over p_mc->Parameters compile time initializers
    On first time boot up. propagate defaults
*/
void MotorController_LoadParamsDefault(MotorController_T * p_mc)
{
    VMonitor_SetNominal_MilliV(&p_mc->VMonitorSource, (uint32_t)p_mc->Parameters.VSourceRef * 1000U);
    VMonitor_ResetLimitsDefault(&p_mc->VMonitorSource);
    // VMonitor_ResetLimitsDefault(&p_mc->VMonitorAccs);
    // VMonitor_ResetLimitsDefault(&p_mc->VMonitorSense);
#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
    // MotorController_User_SetBatteryLifeDefault(p_mc); //todo
#endif
    // Calculate values when adcu default is not provided
    // Thermistor_SetLimits_DegC(&p_mc->ThermistorPcb, 100U, 90U, 80U, 78U);
    // Thermistor_SetLimits_DegC(&p_mc->ThermistorMosfets, 100U, 90U, 80U, 78U);
    // for(uint8_t iMotor = 0U; iMotor < p_mc->CONFIG.MOTOR_COUNT; iMotor++) { Thermistor_SetLimits_DegC(&p_mc->CONFIG.P_MOTORS[iMotor].Thermistor, 100U, 90U, 80U, 78U); }
//     PID_SetTunings(&p_motor->PidSpeed, 1U, 1U, 1U, 2U, 0U, 0U);
//     PID_SetTunings(&p_motor->PidIq, 1U, 1U, 1U, 2U, 0U, 0U);
//     PID_SetTunings(&p_motor->PidId, 1U, 1U, 1U, 2U, 0U, 0U);
}


/******************************************************************************/
/*!
    Call from State Machine
*/
/******************************************************************************/
/******************************************************************************/
/*!
    Nvm Wrappers Call from State Machine
*/
/******************************************************************************/
static NvMemory_Status_T WriteNvm_Blocking(MotorControllerPtr_T p_mc, const void * p_dest, const void * p_source, size_t sizeBytes)
{
#if     defined(CONFIG_MOTOR_CONTROLLER_PARAMETERS_EEPROM)
    return EEPROM_Write_Blocking(p_mc->CONFIG.P_EEPROM, p_dest, p_source, sizeBytes);
#elif   defined(CONFIG_MOTOR_CONTROLLER_PARAMETERS_FLASH)
    assert(NvMemory_IsPtrAligned(p_dest, FLASH_UNIT_WRITE_SIZE));
    return Flash_Write_Blocking(p_mc->CONFIG.P_FLASH, p_dest, p_source, sizeBytes);
#endif
}

/*
    Compile time config ensure all structs are write unit size aligned
*/
NvMemory_Status_T MotorController_SaveParameters_Blocking(MotorControllerPtr_T p_mc)
{
    volatile  NvMemory_Status_T status = NV_MEMORY_STATUS_SUCCESS;
    MotorPtr_T p_motor;
    Protocol_T * p_protocol;

#if defined(CONFIG_MOTOR_CONTROLLER_PARAMETERS_FLASH)
   status = Flash_Erase_Blocking(p_mc->CONFIG.P_FLASH, p_mc->CONFIG.P_PARAMS_START, p_mc->CONFIG.PARAMS_SIZE); /* Flash must erase before write */
#endif

    if(status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_mc, p_mc->CONFIG.P_PARAMS_NVM, &p_mc->Parameters, sizeof(MotorController_Params_T)); };
    if(status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_mc, p_mc->CONFIG.P_MEM_MAP_BOOT, &p_mc->MemMapBoot, sizeof(MemMapBoot_T)); };
    if(status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_mc, p_mc->AnalogUser.CONFIG.P_PARAMS, &p_mc->AnalogUser.Params, sizeof(MotAnalogUser_Params_T)); };

    if(status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_mc, p_mc->VMonitorSource.CONFIG.P_PARAMS, &p_mc->VMonitorSource.Params, sizeof(VMonitor_Params_T)); };
    if(status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_mc, p_mc->VMonitorAccs.CONFIG.P_PARAMS, &p_mc->VMonitorAccs.Params, sizeof(VMonitor_Params_T)); };
    if(status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_mc, p_mc->VMonitorSense.CONFIG.P_PARAMS, &p_mc->VMonitorSense.Params, sizeof(VMonitor_Params_T)); };
#ifdef CONFIG_MOTOR_CONTROLLER_SHELL_ENABLE
    if(status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_mc, p_mc->Shell.CONFIG.P_PARAMS, &p_mc->Shell.Params, sizeof(Shell_Params_T)); };
#endif

    if(status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_mc, p_mc->ThermistorPcb.CONFIG.P_PARAMS, &p_mc->ThermistorPcb.Params, sizeof(Thermistor_Params_T)); };
#if defined(CONFIG_MOTOR_CONTROLLER_HEAT_MOSFETS_TOP_BOT_ENABLE)
    if(status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_mc, p_mc->ThermistorMosfetsTop.CONFIG.P_PARAMS, &p_mc->ThermistorMosfetsTop.Params, sizeof(Thermistor_Params_T)); };
    if(status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_mc, p_mc->ThermistorMosfetsBot.CONFIG.P_PARAMS, &p_mc->ThermistorMosfetsBot.Params, sizeof(Thermistor_Params_T)); };
#else
    if(status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_mc, p_mc->ThermistorMosfets.CONFIG.P_PARAMS, &p_mc->ThermistorMosfets.Params, sizeof(Thermistor_Params_T)); };
#endif

    if(status == NV_MEMORY_STATUS_SUCCESS)
    {
        for(uint8_t iProtocol = 0U; iProtocol < p_mc->CONFIG.PROTOCOL_COUNT; iProtocol++)
        {
            p_protocol = &p_mc->CONFIG.P_PROTOCOLS[iProtocol];
            status = WriteNvm_Blocking(p_mc, p_protocol->CONFIG.P_PARAMS, &p_protocol->Params, sizeof(Protocol_Params_T));
            if(status != NV_MEMORY_STATUS_SUCCESS) { break; }
        }
    }

    if(status == NV_MEMORY_STATUS_SUCCESS)
    {
        for(uint8_t iMotor = 0U; iMotor < p_mc->CONFIG.MOTOR_COUNT; iMotor++)
        {
            p_motor = MotorController_GetPtrMotor(p_mc, iMotor);
            if(status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_mc, p_motor->CONFIG.P_PARAMS_NVM, &p_motor->Parameters, sizeof(Motor_Params_T)); }
            if(status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_mc, p_motor->Hall.CONFIG.P_PARAMS_NVM, &p_motor->Hall.Params, sizeof(Hall_Params_T)); }
            if(status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_mc, p_motor->Encoder.CONFIG.P_PARAMS, &p_motor->Encoder.Params, sizeof(Encoder_Params_T)); }
            if(status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_mc, p_motor->PidSpeed.CONFIG.P_PARAMS, &p_motor->PidSpeed.Params, sizeof(PID_Params_T)); }
            if(status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_mc, p_motor->PidIq.CONFIG.P_PARAMS, &p_motor->PidIq.Params, sizeof(PID_Params_T)); }
            if(status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_mc, p_motor->PidId.CONFIG.P_PARAMS, &p_motor->PidId.Params, sizeof(PID_Params_T)); }
            // if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_motor->PidIBus.CONFIG.P_PARAMS, &p_motor->PidIBus.Params, sizeof(PID_Params_T)); }
            if(status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_mc, p_motor->Thermistor.CONFIG.P_PARAMS, &p_motor->Thermistor.Params, sizeof(Thermistor_Params_T)); }
            if(status != NV_MEMORY_STATUS_SUCCESS) { break; }
        }
    }

    return status;
}

NvMemory_Status_T MotorController_SaveBootReg_Blocking(MotorControllerPtr_T p_mc)
{
#if     defined(CONFIG_MOTOR_CONTROLLER_PARAMETERS_EEPROM)
    return WriteNvm_Blocking(p_mc, p_mc->CONFIG.P_MEM_MAP_BOOT, &p_mc->MemMapBoot, sizeof(MemMapBoot_T));
#elif   defined(CONFIG_MOTOR_CONTROLLER_PARAMETERS_FLASH)
    return NV_MEMORY_STATUS_ERROR_OTHER; /* Save on all params */
#endif
}


NvMemory_Status_T MotorController_ReadOnce_Blocking(MotorControllerPtr_T p_mc, uint8_t * p_destBuffer)
{
#if defined(CONFIG_MOTOR_CONTROLLER_MANUFACTURE_PARAMS_ONCE)
    return (NvMemory_Status_T)Flash_ReadOnce_Blocking(p_mc->CONFIG.P_FLASH, p_destBuffer, (uint8_t *)p_mc->CONFIG.P_MANUFACTURE, sizeof(MotorController_Manufacture_T));
#elif defined(CONFIG_MOTOR_CONTROLLER_MANUFACTURE_PARAMS_FLASH)
#endif
}

NvMemory_Status_T MotorController_SaveOnce_Blocking(MotorControllerPtr_T p_mc, const uint8_t * p_sourceBuffer)
{
#if defined(CONFIG_MOTOR_CONTROLLER_MANUFACTURE_PARAMS_ONCE)
    return (NvMemory_Status_T)Flash_WriteOnce_Blocking(p_mc->CONFIG.P_FLASH, (uint8_t *)p_mc->CONFIG.P_MANUFACTURE, p_sourceBuffer, sizeof(MotorController_Manufacture_T));
#elif defined(CONFIG_MOTOR_CONTROLLER_MANUFACTURE_PARAMS_FLASH)
    return (NvMemory_Status_T)Flash_Write_Blocking(p_mc->CONFIG.P_FLASH, (uint8_t *)p_mc->CONFIG.P_MANUFACTURE, p_dataBuffer, sizeof(MotorController_Manufacture_T));
#endif
}






