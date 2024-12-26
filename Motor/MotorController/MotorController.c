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
#if defined(CONFIG_MOTOR_CONTROLLER_USER_NVM_EEPROM)
    EEPROM_Init_Blocking(p_mc->CONST.P_EEPROM);
#endif

    if(p_mc->CONST.P_NVM_CONFIG != 0U) { memcpy(&p_mc->Config, p_mc->CONST.P_NVM_CONFIG, sizeof(MotorController_Config_T)); }
    if(p_mc->CONST.P_BOOT_REF != 0U) { p_mc->BootRef.Word = p_mc->CONST.P_BOOT_REF->Word; }

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
    MotorController_ResetVSourceMonitorDefaults(p_mc);
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
    /*
        following boots will still reload defaults until user save
        No need to save to Nvm until user confirmation
    */
}

void MotorController_ResetBootDefault(MotorController_T * p_mc)
{
    static const BootRef_T BOOT_REF_DEFAULT = { .IsValid = BOOT_REF_IS_VALID_01, .FastBoot = 0U, .Beep = 1U, .Blink = 1U, }; /* Overwrite after first time boot */
    p_mc->BootRef.Word = BOOT_REF_DEFAULT.Word;
}

void MotorController_ResetVSourceActiveRef(MotorController_T * p_mc)
{
    Motor_Static_InitVSourceRef_V(p_mc->Config.VSourceRef);
    // Motor_Static_InitVSourceRef_Adcu(p_mc->CONST.CONVERSION_VSOURCE.P_STATE->Result); /* Set Motor Ref using read Value */
    for (uint8_t iMotor = 0U; iMotor < p_mc->CONST.MOTOR_COUNT; iMotor++) { Motor_ResetUnitsVabc(&p_mc->CONST.P_MOTORS[iMotor]); }
}

void MotorController_ResetVSourceMonitorDefaults(MotorController_T * p_mc)
{
    VMonitor_SetNominal_MilliV(&p_mc->VMonitorSource, (uint32_t)p_mc->Config.VSourceRef * 1000U);
    VMonitor_ResetLimitsDefault(&p_mc->VMonitorSource);
}

#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
void MotorController_ResetUnitsBatteryLife(MotorController_T * p_mc)
{
    Linear_ADC_Init(&p_mc->BatteryLife, p_mc->Config.BatteryZero_Adcu, p_mc->Config.BatteryFull_Adcu);
}

void MotorController_ResetBatteryLifeDefault(MotorController_T * p_mc)
{
    p_mc->Config.BatteryZero_Adcu = VMonitor_GetFaultLower(&p_mc->VMonitorSource);
    p_mc->Config.BatteryFull_Adcu = VMonitor_AdcuOfMilliV(&p_mc->VMonitorSource, (uint32_t)p_mc->Config.VSourceRef * 1000U);
    MotorController_ResetUnitsBatteryLife(p_mc);
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
#if     defined(CONFIG_MOTOR_CONTROLLER_USER_NVM_EEPROM)
    return EEPROM_Write_Blocking(p_mc->CONST.P_EEPROM, (uintptr_t)p_nvm, p_ram, sizeBytes);
#elif   defined(CONFIG_MOTOR_CONTROLLER_USER_NVM_FLASH)
    assert(nvmemory_is_aligned((uintptr_t)p_nvm, FLASH_UNIT_WRITE_SIZE)); /* Compile time config ensure all structs are write unit size aligned */
    return Flash_Write_Blocking(p_mc->CONST.P_FLASH, (uintptr_t)p_nvm, p_ram, sizeBytes);
#endif
}

NvMemory_Status_T MotorController_SaveConfig_Blocking(MotorController_T * p_mc)
{
    NvMemory_Status_T status = NV_MEMORY_STATUS_SUCCESS;
    Motor_T * p_motor;
    Protocol_T * p_protocol;

#if defined(CONFIG_MOTOR_CONTROLLER_USER_NVM_FLASH)
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

// void MotorController_ReloadConfig_Blocking(MotorController_T * p_mc)
// {
//     if (p_mc->CONST.P_NVM_CONFIG != 0U) { memcpy(&p_mc->Config, p_mc->CONST.P_NVM_CONFIG, sizeof(MotorController_Config_T)); }
//     if (p_mc->CONST.P_BOOT_REF != 0U) { p_mc->BootRef.Word = p_mc->CONST.P_BOOT_REF->Word; }
// }

// eeprom only
NvMemory_Status_T MotorController_SaveBootReg_Blocking(MotorController_T * p_mc)
{
#if     defined(CONFIG_MOTOR_CONTROLLER_USER_NVM_EEPROM)
    return EEPROM_Write_Blocking(p_mc->CONST.P_EEPROM, (uintptr_t)p_mc->CONST.P_BOOT_REF, &p_mc->BootRef, sizeof(BootRef_T));
    // return WriteNvm_Blocking(p_mc, p_mc->CONST.P_BOOT_REF, &p_mc->BootRef, sizeof(BootRef_T));
#elif   defined(CONFIG_MOTOR_CONTROLLER_USER_NVM_FLASH)
    return NV_MEMORY_STATUS_ERROR_OTHER; /* Save on all params */
#endif
}

NvMemory_Status_T MotorController_ReadManufacture_Blocking(MotorController_T * p_mc, uintptr_t onceAddress, uint8_t size, uint8_t * p_destBuffer)
{
#if     defined(CONFIG_MOTOR_CONTROLLER_MANUFACTURE_NVM_ONCE)
    // if(p_mc->CONST.MANUFACTURE_ADDRESS != 0) handle offset
    return Flash_ReadOnce_Blocking(p_mc->CONST.P_FLASH, onceAddress, size, p_destBuffer);
#elif   defined(CONFIG_MOTOR_CONTROLLER_MANUFACTURE_NVM_FLASH)
    if (size < p_mc->CONST.MANUFACTURE_SIZE) { memcpy(p_destBuffer, onceAddress, size); }
#endif
}

NvMemory_Status_T MotorController_WriteManufacture_Blocking(MotorController_T * p_mc, uintptr_t onceAddress, const uint8_t * p_sourceBuffer, uint8_t size)
{
#if     defined(CONFIG_MOTOR_CONTROLLER_MANUFACTURE_NVM_ONCE)
    return Flash_WriteOnce_Blocking(p_mc->CONST.P_FLASH, onceAddress, p_sourceBuffer, size);
#elif   defined(CONFIG_MOTOR_CONTROLLER_MANUFACTURE_NVM_FLASH)
    return Flash_Write_Blocking(p_mc->CONST.P_FLASH, onceAddress, p_dataBuffer, size);
#endif
}


/******************************************************************************/
/*!
    Calibrate
*/
/******************************************************************************/
void MotorController_CalibrateAdc(MotorController_T * p_mc)
{
    Analog_MarkConversion(&p_mc->CONST.CONVERSION_THROTTLE);
    Analog_MarkConversion(&p_mc->CONST.CONVERSION_BRAKE);
    void_array_foreach(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (void_op_t)Motor_User_CalibrateAdc);
    // MotAnalogUser_SetThrottleZero(&p_mc->AnalogUser, MotorController_Analog_GetThrottle(p_mc)); // todo wait filter state
    // MotAnalogUser_SetBrakeZero(&p_mc->AnalogUser, MotorController_Analog_GetBrake(p_mc));
    MotAnalogUser_SetThrottleZero(&p_mc->AnalogUser,  p_mc->CONST.CONVERSION_THROTTLE.P_STATE->Result);
    MotAnalogUser_SetBrakeZero(&p_mc->AnalogUser, p_mc->CONST.CONVERSION_BRAKE.P_STATE->Result);
}

void MotorController_CalibrateSensorAll(MotorController_T * p_mc)
{
    void_array_foreach(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (void_op_t)Motor_User_CalibrateSensor);
}


/******************************************************************************/
/*
   MotorN Array Functions - Proc by StateMachine
*/
/******************************************************************************/
bool MotorController_IsEveryMotorStopState(const MotorController_T * p_mc)  { return void_array_is_every(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (void_test_t)Motor_User_IsStopState); }
bool MotorController_IsEveryMotorRunState(const MotorController_T * p_mc)   { return void_array_is_every(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (void_test_t)Motor_User_IsRunState); }
bool MotorController_IsEveryMotorForward(const MotorController_T * p_mc)    { return void_array_is_every(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (void_test_t)Motor_IsDirectionForward); }
bool MotorController_IsEveryMotorReverse(const MotorController_T * p_mc)    { return void_array_is_every(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (void_test_t)Motor_IsDirectionReverse); }

bool MotorController_IsAnyMotorFault(const MotorController_T * p_mc)  { return void_array_is_any(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (void_test_t)Motor_StateMachine_IsFault); }
bool MotorController_ForEveryMotorExitFault(MotorController_T * p_mc) { return void_array_for_every(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (void_poll_t)Motor_StateMachine_ExitFault); }

void MotorController_ForceDisableAll(MotorController_T * p_mc)  { void_array_foreach(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (void_op_t)Motor_User_ForceDisableControl); }
void MotorController_SetReleaseAll(MotorController_T * p_mc)    { void_array_foreach(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (void_op_t)Motor_User_SetRelease); }
void MotorController_SetHoldAll(MotorController_T * p_mc)       { void_array_foreach(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (void_op_t)Motor_User_SetHold); }
void MotorController_SetDirectionForwardAll(MotorController_T * p_mc) { void_array_foreach(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (void_op_t)Motor_User_SetDirectionForward); }
void MotorController_SetDirectionReverseAll(MotorController_T * p_mc) { void_array_foreach(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (void_op_t)Motor_User_SetDirectionReverse); }

// bool MotorController_TryReleaseAll(MotorController_T * p_mc) { return void_array_for_every(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (void_poll_t)Motor_User_TryRelease); }
// bool MotorController_TryHoldAll(MotorController_T * p_mc)    { return void_array_for_every(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (void_poll_t)Motor_User_TryHold); }
// bool MotorController_TryDirectionForwardAll(MotorController_T * p_mc) { return void_array_for_every(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (void_poll_t)Motor_User_TryDirectionForward); }
// bool MotorController_TryDirectionReverseAll(MotorController_T * p_mc) { return void_array_for_every(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (void_poll_t)Motor_User_TryDirectionReverse); }

/*
    current no void_array functions for key/value set
*/
void MotorController_SetSpeedLimitAll(MotorController_T * p_mc, Motor_SpeedLimitId_T id, uint16_t limit_fract16)
    { for (uint8_t iMotor = 0U; iMotor < p_mc->CONST.MOTOR_COUNT; iMotor++) { Motor_SetSpeedLimitEntry(&p_mc->CONST.P_MOTORS[iMotor], id, limit_fract16); } }

void MotorController_ClearSpeedLimitAll(MotorController_T * p_mc, Motor_SpeedLimitId_T id)
    { for (uint8_t iMotor = 0U; iMotor < p_mc->CONST.MOTOR_COUNT; iMotor++) { Motor_ClearSpeedLimitEntry(&p_mc->CONST.P_MOTORS[iMotor], id); } }

// void MotorController_SetSpeedLimitAll_Scalar(MotorController_T * p_mc, Motor_SpeedLimitId_T id, uint16_t scalar_fract16)
//     { for (uint8_t iMotor = 0U; iMotor < p_mc->CONST.MOTOR_COUNT; iMotor++) { Motor_SetSpeedLimitEntry_Scalar(&p_mc->CONST.P_MOTORS[iMotor], id, scalar_fract16); } }

void MotorController_SetILimitAll(MotorController_T * p_mc, Motor_ILimitId_T id, uint16_t limit_fract16)
    { for (uint8_t iMotor = 0U; iMotor < p_mc->CONST.MOTOR_COUNT; iMotor++) { Motor_SetILimitMotoringEntry(&p_mc->CONST.P_MOTORS[iMotor], id, limit_fract16); } }

void MotorController_SetILimitAll_Scalar(MotorController_T * p_mc, Motor_ILimitId_T id, uint16_t scalar_fract16)
    { for (uint8_t iMotor = 0U; iMotor < p_mc->CONST.MOTOR_COUNT; iMotor++) { Motor_SetILimitMotoringEntry_Scalar(&p_mc->CONST.P_MOTORS[iMotor], id, scalar_fract16); } }

void MotorController_ClearILimitAll(MotorController_T * p_mc, Motor_ILimitId_T id)
    { for (uint8_t iMotor = 0U; iMotor < p_mc->CONST.MOTOR_COUNT; iMotor++) { Motor_ClearILimitMotoringEntry(&p_mc->CONST.P_MOTORS[iMotor], id); } }


void MotorController_SetCmdValueAll(MotorController_T * p_mc, int16_t userCmd)                          { struct_array_foreach_set_int16(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (set_int16_t)Motor_User_SetActiveCmdValue, userCmd); }
void MotorController_StartControlModeAll(MotorController_T * p_mc, Motor_FeedbackMode_T feedbackMode)   { struct_array_foreach_set_uint8(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (set_uint8_t)Motor_User_StartControl_Cast, feedbackMode.Value); }
void MotorController_SetFeedbackModeAll_Cast(MotorController_T * p_mc, uint8_t feedbackMode)            { struct_array_foreach_set_uint8(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (set_uint8_t)Motor_User_SetFeedbackMode_Cast, feedbackMode); }

void MotorController_StartThrottleMode(MotorController_T * p_mc)
{
    switch (p_mc->Config.ThrottleMode)
    {
        case MOTOR_CONTROLLER_THROTTLE_MODE_SPEED:  void_array_foreach(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (void_op_t)Motor_User_StartSpeedMode);     break;
        case MOTOR_CONTROLLER_THROTTLE_MODE_TORQUE: void_array_foreach(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (void_op_t)Motor_User_StartTorqueMode);    break;
        default: break;
    }
}

void MotorController_SetThrottleValue(MotorController_T * p_mc, uint16_t userCmdThrottle)
{
    int16_t cmdValue = (int32_t)userCmdThrottle / 2;
    switch (p_mc->Config.ThrottleMode)
    {
        case MOTOR_CONTROLLER_THROTTLE_MODE_SPEED:  struct_array_foreach_set_int16(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (set_int16_t)Motor_User_SetSpeedCmd_Scalar, cmdValue);     break;
        case MOTOR_CONTROLLER_THROTTLE_MODE_TORQUE: struct_array_foreach_set_int16(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (set_int16_t)Motor_User_SetTorqueCmd_Scalar, cmdValue);    break;
        default: break;
    }
}

void MotorController_StartBrakeMode(MotorController_T * p_mc)
{
    switch (p_mc->Config.BrakeMode)
    {
        case MOTOR_CONTROLLER_BRAKE_MODE_TORQUE: void_array_foreach(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (void_op_t)Motor_User_StartTorqueMode); break;
        case MOTOR_CONTROLLER_BRAKE_MODE_VOLTAGE: void_array_foreach(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (void_op_t)Motor_User_StartVoltageMode); break;
        default: break;
    }

    // MotorController_SetBrakeValue(p_mc, value); T
}

/*!
    Always request opposite direction current
    req opposite iq, bound vq to 0 for no plugging brake

    transition from accelerating to decelerating,
    use signed ramp to transition through 0 without discontinuity
    ramp from in-direction torque to 0 to counter-direction torque

    @param[in] brake [0:32767]
*/
void MotorController_SetBrakeValue(MotorController_T * p_mc, uint16_t userCmdBrake)
{
    int16_t cmdValue = 0 - (userCmdBrake / 2); // 32767 max

    switch (p_mc->Config.BrakeMode)
    {
        case MOTOR_CONTROLLER_BRAKE_MODE_TORQUE: struct_array_foreach_set_int16(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (set_int16_t)Motor_User_SetTorqueCmd_Scalar, cmdValue); break;
        case MOTOR_CONTROLLER_BRAKE_MODE_VOLTAGE: struct_array_foreach_set_int16(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (set_int16_t)Motor_User_SetVSpeedScalarCmd, 0); break;
        default: break;
    }
}

void MotorController_StartDriveZero(MotorController_T * p_mc)
{
    switch (p_mc->Config.DriveZeroMode)
    {
        case MOTOR_CONTROLLER_DRIVE_ZERO_MODE_FLOAT: MotorController_SetReleaseAll(p_mc); break;
        case MOTOR_CONTROLLER_DRIVE_ZERO_MODE_REGEN: /* MotorController_SetRegenMotorAll(p_mc); */ break;
        case MOTOR_CONTROLLER_DRIVE_ZERO_MODE_CRUISE: void_array_foreach(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (void_op_t)Motor_User_StartTorqueMode); break;
        default: break;
    }
}

/*
    Check Stop / Zero Throttle
    Eventually release for stop transition
*/
void MotorController_ProcDriveZero(MotorController_T * p_mc)
{
    switch (p_mc->Config.DriveZeroMode)
    {
        case MOTOR_CONTROLLER_DRIVE_ZERO_MODE_FLOAT: break;
        case MOTOR_CONTROLLER_DRIVE_ZERO_MODE_REGEN: /* MotorController_ProcRegenMotorAll(p_mc); */ break;
        case MOTOR_CONTROLLER_DRIVE_ZERO_MODE_CRUISE: struct_array_foreach_set_int16(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (set_int16_t)Motor_User_SetTorqueCmd_Scalar, 0); break;
        default: break;
    }
}



// static inline bool MotorController_ProcOnDirection(MotorController_T * p_mc, MotorController_Direction_T direction)
// {
//     // if((p_mc->Config.BuzzerFlagsEnable.OnReverse == true))
//     // {
//     //     if(p_mc->DriveDirection == MOTOR_CONTROLLER_DIRECTION_REVERSE)
//     //     {
//     //         MotorController_BeepPeriodicType1(p_mc);
//     //     }
//     //     else
//     //     {
//     //         Blinky_Stop(&p_mc->Buzzer);
//     //     }
//     // }
//
// }

// on Init poll
// if(p_mc->InitFlags.Word != 0U) { wait = true; }   // indirectly poll inputs

//     if((p_mc->Config.BuzzerFlagsEnable.ThrottleOnInit == true) && (p_mc->BuzzerFlagsActive.ThrottleOnInit == 0U))
//     {
//         p_mc->BuzzerFlagsActive.ThrottleOnInit = 1U;
        // MotorController_BeepShort(p_mc);
//     }