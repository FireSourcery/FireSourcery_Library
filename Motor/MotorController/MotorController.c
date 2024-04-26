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
    if(p_mc->CONFIG.P_BOOT_REF != 0U) { p_mc->BootRef.Word = p_mc->CONFIG.P_BOOT_REF->Word; }

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

    // /* Load derived values to RAM */
    // if(BootRef_IsValid() == false)
    // {
    //     MotorController_LoadParamsDefault(&MotorControllerMain);  /* Load runtime calculated */
    //     /* or prompt user, resets every boot until user saves params */
    // }
    MotorController_SetAdcResultsNominal(p_mc);
    StateMachine_Init(&p_mc->StateMachine);
}

/******************************************************************************/
/*!
    Set/Reset
*/
/******************************************************************************/
/*
    Set runtime Params (RAM copy) via abstraction layer functions (in user units)
    Convience function over p_mc->Parameters compile time initializers
    On first time boot up. propagate defaults
*/
void MotorController_LoadParamsDefault(MotorControllerPtr_T p_mc)
{
    VMonitor_SetNominal_MilliV(&p_mc->VMonitorSource, (uint32_t)p_mc->Parameters.VSourceRef * 1000U);
    VMonitor_ResetLimitsDefault(&p_mc->VMonitorSource);
    VMonitor_Enable(&p_mc->VMonitorSource);
    // VMonitor_ResetLimitsDefault(&p_mc->VMonitorAccs);
    // VMonitor_ResetLimitsDefault(&p_mc->VMonitorSense);
#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
    MotorController_ResetUnitsBatteryLife(p_mc);
#endif
    // for(uint8_t iMotor = 0U; iMotor < p_mc->CONFIG.MOTOR_COUNT; iMotor++)
    // {
    //     PID_SetTunings(&(p_mc->CONFIG.P_MOTORS[iMotor].PidSpeed), 1U, 1U, 1U, 2U, 0U, 0U);
    //     PID_SetTunings(&(p_mc->CONFIG.P_MOTORS[iMotor].PidIq), 1U, 1U, 1U, 2U, 0U, 0U);
    //     PID_SetTunings(&(p_mc->CONFIG.P_MOTORS[iMotor].PidId), 1U, 1U, 1U, 2U, 0U, 0U);
    // }
    MotorController_ResetBootDefault(p_mc); /* Set Boot Options Buffer in RAM */
    /* following boots will still reload defaults until user save */
    //  save to nvm? or wait user confirmation in gui
}

void MotorController_ResetBootDefault(MotorControllerPtr_T p_mc)
{
    static const BootRef_T BOOT_REF_DEFAULT = { .IsValid = BOOT_REF_IS_VALID_01, .FastBoot = 0U, .Beep = 1U, .Blink = 1U, }; /* Overwrite after first time boot */
    p_mc->BootRef.Word = BOOT_REF_DEFAULT.Word;
}


#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
void MotorController_ResetUnitsBatteryLife(MotorControllerPtr_T p_mc)
{
    Linear_ADC_Init(&p_mc->BatteryLife, p_mc->Parameters.BatteryZero_Adcu, p_mc->Parameters.BatteryFull_Adcu, 0U, 1000U);
}
#endif

/******************************************************************************/
/*!
    Call from State Machine
*/
/******************************************************************************/
void MotorController_SetAdcResultsNominal(MotorControllerPtr_T p_mc)
{
    // alternatively enqueue adc during init
    // handle via init state wait
    if(VMonitor_IsEnable(&p_mc->VMonitorSense) == true) { p_mc->AnalogResults.VSense_Adcu = VMonitor_GetNominal(&p_mc->VMonitorSense); };
    if(VMonitor_IsEnable(&p_mc->VMonitorAccs) == true) { p_mc->AnalogResults.VAccs_Adcu = VMonitor_GetNominal(&p_mc->VMonitorAccs); };
    if(VMonitor_IsEnable(&p_mc->VMonitorSource) == true) { p_mc->AnalogResults.VSource_Adcu = VMonitor_GetNominal(&p_mc->VMonitorSource); };
    if(Thermistor_IsMonitorEnable(&p_mc->ThermistorPcb) == true) { p_mc->AnalogResults.HeatPcb_Adcu = Thermistor_GetWarningThreshold_Adcu(&p_mc->ThermistorPcb); };
    if(Thermistor_IsMonitorEnable(&p_mc->ThermistorMosfets) == true) { p_mc->AnalogResults.HeatMosfets_Adcu = Thermistor_GetWarningThreshold_Adcu(&p_mc->ThermistorMosfets); };

    //updating mosfets to array
    // if(VMonitor_IsEnable(&p_mc->VMonitorSense) == true) { p_mc->AnalogResults.Channels[MOT_ANALOG_CHANNEL_VSENSE] = VMonitor_GetNominal(&p_mc->VMonitorSense); };

    // for(uint8_t iMotor = 0U; iMotor < p_mc->CONFIG.MOTOR_COUNT; iMotor++) { Motor_PollAdcFaultFlags(&p_mc->CONFIG.P_MOTORS[iMotor]); }
}

void MotorController_PollAdcFaultFlags(MotorControllerPtr_T p_mc)
{
    // p_mc->FaultFlags.Motors         = (MotorController_ClearMotorsFaultAll(p_mc) == false);
    p_mc->FaultFlags.VSenseLimit = VMonitor_GetIsFault(&p_mc->VMonitorSense);
    p_mc->FaultFlags.VAccsLimit = VMonitor_GetIsFault(&p_mc->VMonitorAccs);
    p_mc->FaultFlags.VSourceLimit = VMonitor_GetIsFault(&p_mc->VMonitorSource);
    p_mc->FaultFlags.PcbOverheat = Thermistor_GetIsFault(&p_mc->ThermistorPcb);
#if defined(CONFIG_MOTOR_CONTROLLER_HEAT_MOSFETS_TOP_BOT_ENABLE)
    p_mc->FaultFlags.MosfetsTopOverHeat = Thermistor_GetIsFault(&p_mc->ThermistorMosfetsTop);
    p_mc->FaultFlags.MosfetsBotOverHeat = Thermistor_GetIsFault(&p_mc->ThermistorMosfetsBot);
#else
    p_mc->FaultFlags.MosfetsOverheat = Thermistor_GetIsFault(&p_mc->ThermistorMosfets);
#endif

    for(uint8_t iMotor = 0U; iMotor < p_mc->CONFIG.MOTOR_COUNT; iMotor++) { Motor_PollAdcFaultFlags(&p_mc->CONFIG.P_MOTORS[iMotor]); }
}

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
    assert(NvMemory_IsAligned((uintptr_t)p_dest, FLASH_UNIT_WRITE_SIZE));
    return Flash_Write_Blocking(p_mc->CONFIG.P_FLASH, p_dest, p_source, sizeBytes);
#endif
}

/*
    Compile time config ensure all structs are write unit size aligned
*/
NvMemory_Status_T MotorController_SaveParameters_Blocking(MotorControllerPtr_T p_mc)
{
    NvMemory_Status_T status = NV_MEMORY_STATUS_SUCCESS;
    MotorPtr_T p_motor;
    Protocol_T * p_protocol;

#if defined(CONFIG_MOTOR_CONTROLLER_PARAMETERS_FLASH)
   status = Flash_Erase_Blocking(p_mc->CONFIG.P_FLASH, p_mc->CONFIG.P_PARAMS_START, p_mc->CONFIG.PARAMS_SIZE); /* Flash must erase, all parameters in most cases, before write */
//    status = Flash_Erase_Blocking(p_mc->CONFIG.P_FLASH, p_mc->CONFIG.P_FLASH->CONFIG.P_PARAMS_PARTITION->P_START, p_mc->CONFIG.P_FLASH->CONFIG.P_PARAMS_PARTITION->SIZE);
#endif

    if(status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_mc, p_mc->CONFIG.P_PARAMS_NVM, &p_mc->Parameters, sizeof(MotorController_Params_T)); };
    if(status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_mc, p_mc->CONFIG.P_BOOT_REF, &p_mc->BootRef, sizeof(BootRef_T)); };
    if(status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_mc, p_mc->AnalogUser.CONFIG.P_PARAMS, &p_mc->AnalogUser.Params, sizeof(MotAnalogUser_Params_T)); };

    if(status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_mc, p_mc->VMonitorSource.CONFIG.P_PARAMS, &p_mc->VMonitorSource.Params, sizeof(VMonitor_Params_T)); };
    if(status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_mc, p_mc->VMonitorAccs.CONFIG.P_PARAMS, &p_mc->VMonitorAccs.Params, sizeof(VMonitor_Params_T)); };
    if(status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_mc, p_mc->VMonitorSense.CONFIG.P_PARAMS, &p_mc->VMonitorSense.Params, sizeof(VMonitor_Params_T)); };

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

#ifdef CONFIG_MOTOR_CONTROLLER_SHELL_ENABLE
    if(status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm_Blocking(p_mc, p_mc->Shell.CONFIG.P_PARAMS, &p_mc->Shell.Params, sizeof(Shell_Params_T)); };
#endif

    return status;
}

NvMemory_Status_T MotorController_SaveBootReg_Blocking(MotorControllerPtr_T p_mc)
{
#if     defined(CONFIG_MOTOR_CONTROLLER_PARAMETERS_EEPROM)
    return WriteNvm_Blocking(p_mc, p_mc->CONFIG.P_BOOT_REF, &p_mc->BootRef, sizeof(BootRef_T));
#elif   defined(CONFIG_MOTOR_CONTROLLER_PARAMETERS_FLASH)
    return NV_MEMORY_STATUS_ERROR_OTHER; /* Save on all params */
#endif
}

NvMemory_Status_T MotorController_ReadOnce_Blocking(MotorControllerPtr_T p_mc, uint8_t * p_destBuffer, uint8_t size)
{
#if     defined(CONFIG_MOTOR_CONTROLLER_MANUFACTURE_PARAMS_ONCE)
    return Flash_ReadOnce_Blocking(p_mc->CONFIG.P_FLASH, p_destBuffer, (uint8_t *)p_mc->CONFIG.P_MANUFACTURE, size);
#elif   defined(CONFIG_MOTOR_CONTROLLER_MANUFACTURE_PARAMS_FLASH)
    if(size < p_mc->CONFIG.MANUFACTURE_SIZE) { memcpy(p_destBuffer, p_mc->CONFIG.P_MANUFACTURE, size); }
#endif
}

NvMemory_Status_T MotorController_SaveOnce_Blocking(MotorControllerPtr_T p_mc, const uint8_t * p_sourceBuffer, uint8_t size)
{
#if     defined(CONFIG_MOTOR_CONTROLLER_MANUFACTURE_PARAMS_ONCE)
    return Flash_WriteOnce_Blocking(p_mc->CONFIG.P_FLASH, (uint8_t *)p_mc->CONFIG.P_MANUFACTURE, p_sourceBuffer, size);
#elif   defined(CONFIG_MOTOR_CONTROLLER_MANUFACTURE_PARAMS_FLASH)
    return Flash_Write_Blocking(p_mc->CONFIG.P_FLASH, (uint8_t *)p_mc->CONFIG.P_MANUFACTURE, p_dataBuffer, size);
#endif
}



