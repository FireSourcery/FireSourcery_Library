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
    MotNvm_Init(&p_mc->CONST.MOT_NVM);

    if (p_mc->CONST.P_CONFIG != NULL) { memcpy(&p_mc->Config, p_mc->CONST.P_CONFIG, sizeof(MotorController_Config_T)); }
    if (p_mc->CONST.MOT_NVM.P_BOOT_REF != NULL) { p_mc->BootRef.Word = p_mc->CONST.MOT_NVM.P_BOOT_REF->Word; }

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

    MotorAnalogRef_InitVSource_V(p_mc->Config.VSourceRef);
    for (uint8_t iMotor = 0U; iMotor < p_mc->CONST.MOTORS.LENGTH; iMotor++) { Motor_Init(&p_mc->CONST.MOTORS.P_ARRAY[iMotor]); }

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
    // for(uint8_t iMotor = 0U; iMotor < p_mc->CONST.MOTORS.LENGTH; iMotor++)
    // {
    //     PID_SetTunings(&(p_mc->CONST.MOTORS.P_ARRAY[iMotor].PidSpeed), 1U, 1U, 1U, 2U, 0U, 0U);
    //     PID_SetTunings(&(p_mc->CONST.MOTORS.P_ARRAY[iMotor].PidIq), 1U, 1U, 1U, 2U, 0U, 0U);
    //     PID_SetTunings(&(p_mc->CONST.MOTORS.P_ARRAY[iMotor].PidId), 1U, 1U, 1U, 2U, 0U, 0U);
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

void MotorController_CaptureVSourceActiveRef(MotorController_T * p_mc)
{
    MotorAnalogRef_CaptureVSource_Adcu(p_mc->CONST.CONVERSION_VSOURCE.P_STATE->Result); /* Set Motor Ref using read Value */
}

void MotorController_ResetVSourceMonitorDefaults(MotorController_T * p_mc)
{
    VMonitor_SetNominal_MilliV(&p_mc->VMonitorSource, (uint32_t)p_mc->Config.VSourceRef * 1000U);
    VMonitor_ResetLimitsDefault(&p_mc->VMonitorSource);
}

#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
void MotorController_ResetUnitsBatteryLife(MotorController_T * p_mc)
{
    Linear_ADC_Init_Scalar(&p_mc->BatteryLife, p_mc->Config.BatteryZero_Adcu, p_mc->Config.BatteryFull_Adcu);
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

*/
/******************************************************************************/
/* shorthand */
static inline NvMemory_Status_T WriteNvm(MotorController_T * p_mc, const void * p_nvm, const void * p_ram, size_t sizeBytes)
{
    MotNvm_WriteConfig_Blocking(&p_mc->CONST.MOT_NVM, p_nvm, p_ram, sizeBytes);
}

NvMemory_Status_T MotorController_SaveConfig_Blocking(MotorController_T * p_mc)
{
    NvMemory_Status_T status = NV_MEMORY_STATUS_SUCCESS;
    Motor_T * p_motor;
    Protocol_T * p_protocol;

#if defined(CONFIG_MOTOR_CONTROLLER_USER_NVM_FLASH)
    status = Flash_Erase_Blocking(p_mc->CONST.MOT_NVM.P_FLASH, p_mc->CONST.MOT_NVM.MAIN_CONFIG_ADDRESS, p_mc->CONST.MOT_NVM.MAIN_CONFIG_SIZE); /* Flash must erase parameters flash region before write */
#endif

    if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm(p_mc, p_mc->CONST.MOT_NVM.P_BOOT_REF, &p_mc->BootRef, sizeof(BootRef_T)); };
    if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm(p_mc, p_mc->CONST.P_CONFIG, &p_mc->Config, sizeof(MotorController_Config_T)); };
    if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm(p_mc, p_mc->AnalogUser.CONST.P_CONFIG, &p_mc->AnalogUser.Config, sizeof(MotAnalogUser_Config_T)); };

    if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm(p_mc, p_mc->VMonitorSource.CONST.P_CONFIG, &p_mc->VMonitorSource.Config, sizeof(VMonitor_Config_T)); };
    if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm(p_mc, p_mc->VMonitorAccs.CONST.P_CONFIG, &p_mc->VMonitorAccs.Config, sizeof(VMonitor_Config_T)); };
    if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm(p_mc, p_mc->VMonitorSense.CONST.P_CONFIG, &p_mc->VMonitorSense.Config, sizeof(VMonitor_Config_T)); };

    if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm(p_mc, p_mc->ThermistorPcb.CONST.P_CONFIG, &p_mc->ThermistorPcb.Config, sizeof(Thermistor_Config_T)); };

    if (status == NV_MEMORY_STATUS_SUCCESS)
    {
        for (uint8_t iMosfets = 0U; iMosfets < MOTOR_CONTROLLER_HEAT_MOSFETS_COUNT; iMosfets++)
        {
            status = WriteNvm(p_mc, p_mc->MosfetsThermistors[iMosfets].CONST.P_CONFIG, &p_mc->MosfetsThermistors[iMosfets].Config, sizeof(Thermistor_Config_T));
            if (status != NV_MEMORY_STATUS_SUCCESS) { break; }
        }
    };

    if (status == NV_MEMORY_STATUS_SUCCESS)
    {
        for (uint8_t iProtocol = 0U; iProtocol < p_mc->CONST.PROTOCOL_COUNT; iProtocol++)
        {
            p_protocol = &p_mc->CONST.P_PROTOCOLS[iProtocol];
            status = WriteNvm(p_mc, p_protocol->CONST.P_CONFIG, &p_protocol->Config, sizeof(Protocol_Config_T));
            if (status != NV_MEMORY_STATUS_SUCCESS) { break; }
        }
    }

    if (status == NV_MEMORY_STATUS_SUCCESS)
    {
        for (uint8_t iMotor = 0U; iMotor < p_mc->CONST.MOTORS.LENGTH; iMotor++)
        {
            p_motor = MotorController_GetPtrMotor(p_mc, iMotor);
            if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm(p_mc, p_motor->CONST.P_NVM_CONFIG, &p_motor->Config, sizeof(Motor_Config_T)); }
            if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm(p_mc, p_motor->Hall.CONST.P_NVM_CONFIG, &p_motor->Hall.Config, sizeof(Hall_Config_T)); }
            if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm(p_mc, p_motor->Encoder.CONST.P_CONFIG, &p_motor->Encoder.Config, sizeof(Encoder_Config_T)); }
            if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm(p_mc, p_motor->PidSpeed.CONST.P_CONFIG, &p_motor->PidSpeed.Config, sizeof(PID_Config_T)); }
            if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm(p_mc, p_motor->PidIq.CONST.P_CONFIG, &p_motor->PidIq.Config, sizeof(PID_Config_T)); }
            if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm(p_mc, p_motor->PidId.CONST.P_CONFIG, &p_motor->PidId.Config, sizeof(PID_Config_T)); }
            // if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm(p_motor->PidIBus.CONST.P_CONFIG, &p_motor->PidIBus.Config, sizeof(PID_Config_T)); }
            if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm(p_mc, p_motor->Thermistor.CONST.P_CONFIG, &p_motor->Thermistor.Config, sizeof(Thermistor_Config_T)); }
            if (status != NV_MEMORY_STATUS_SUCCESS) { break; }
        }
    }

#ifdef CONFIG_MOTOR_CONTROLLER_SHELL_ENABLE
    if (status == NV_MEMORY_STATUS_SUCCESS) { status = WriteNvm(p_mc, p_mc->Shell.CONST.P_CONFIG, &p_mc->Shell.Config, sizeof(Shell_Config_T)); };
#endif

    return status;
}




/******************************************************************************/
/*
   Drive Mode
*/
/******************************************************************************/
void MotorController_StartThrottleMode(MotorController_T * p_mc)
{
    switch (p_mc->Config.ThrottleMode)
    {
        case MOTOR_CONTROLLER_THROTTLE_MODE_SPEED:  Motor_Array_ForEach(&p_mc->CONST.MOTORS, Motor_User_StartSpeedMode);     break;
        case MOTOR_CONTROLLER_THROTTLE_MODE_TORQUE: Motor_Array_ForEach(&p_mc->CONST.MOTORS, Motor_User_StartTorqueMode);    break;
        default: break;
    }
}

// todo change to interface set int
void MotorController_SetThrottleValue(MotorController_T * p_mc, uint16_t userCmdThrottle)
{
    int16_t cmdValue = (int32_t)userCmdThrottle / 2;
    switch (p_mc->Config.ThrottleMode)
    {
        case MOTOR_CONTROLLER_THROTTLE_MODE_SPEED:  struct_array_foreach_set_int16(p_mc->CONST.MOTORS.P_ARRAY, sizeof(Motor_T), p_mc->CONST.MOTORS.LENGTH, (set_int16_t)Motor_User_SetSpeedCmd_Scalar, cmdValue);     break;
        case MOTOR_CONTROLLER_THROTTLE_MODE_TORQUE: struct_array_foreach_set_int16(p_mc->CONST.MOTORS.P_ARRAY, sizeof(Motor_T), p_mc->CONST.MOTORS.LENGTH, (set_int16_t)Motor_User_SetTorqueCmd_Scalar, cmdValue);    break;
        // case MOTOR_CONTROLLER_THROTTLE_MODE_SPEED:  Motor_Array_ForEachSet(&p_mc->CONST.MOTORS, Motor_User_SetSpeedCmd_Scalar, cmdValue);              break;
        // case MOTOR_CONTROLLER_THROTTLE_MODE_TORQUE: Motor_Array_ForEachSet(&p_mc->CONST.MOTORS, Motor_User_SetTorqueCmd_Scalar_Cast, cmdValue);        break;
        default: break;
    }
}

void MotorController_StartBrakeMode(MotorController_T * p_mc)
{
    switch (p_mc->Config.BrakeMode)
    {
        case MOTOR_CONTROLLER_BRAKE_MODE_TORQUE:  Motor_Array_ForEach(&p_mc->CONST.MOTORS, Motor_User_StartTorqueMode); break;
        case MOTOR_CONTROLLER_BRAKE_MODE_VOLTAGE: Motor_Array_ForEach(&p_mc->CONST.MOTORS, Motor_User_StartVoltageMode); break;
        default: break;
    }

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
    int16_t cmdValue = 0 - ((int32_t)userCmdBrake / 2); // 32767 max

    switch (p_mc->Config.BrakeMode)
    {
        case MOTOR_CONTROLLER_BRAKE_MODE_TORQUE: struct_array_foreach_set_int16(p_mc->CONST.MOTORS.P_ARRAY, sizeof(Motor_T), p_mc->CONST.MOTORS.LENGTH, (set_int16_t)Motor_User_SetTorqueCmd_Scalar, cmdValue); break;
        // case MOTOR_CONTROLLER_BRAKE_MODE_VOLTAGE: struct_array_foreach_set_int16(p_mc->CONST.MOTORS.P_ARRAY, sizeof(Motor_T), p_mc->CONST.MOTORS.LENGTH, (set_int16_t)Motor_User_SetVSpeedScalarCmd, 0); break;
        // case MOTOR_CONTROLLER_BRAKE_MODE_TORQUE: Motor_Array_ForEachSet(&p_mc->CONST.MOTORS, Motor_User_SetTorqueCmdScalar, cmdValue); break
        default: break;
    }
}

/* an alternate cmd for float is required */
void MotorController_StartDriveZero(MotorController_T * p_mc)
{
    switch (p_mc->Config.DriveZeroMode)
    {
        case MOTOR_CONTROLLER_DRIVE_ZERO_MODE_FLOAT: Motor_Array_ForEach(&p_mc->CONST.MOTORS, Motor_User_Release); break;
        case MOTOR_CONTROLLER_DRIVE_ZERO_MODE_CRUISE: Motor_Array_ForEach(&p_mc->CONST.MOTORS, Motor_User_StartTorqueMode); break;
        case MOTOR_CONTROLLER_DRIVE_ZERO_MODE_REGEN: /* MotorController_SetRegenMotorAll(p_mc); */ break;
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
        case MOTOR_CONTROLLER_DRIVE_ZERO_MODE_CRUISE: struct_array_foreach_set_int16(p_mc->CONST.MOTORS.P_ARRAY, sizeof(Motor_T), p_mc->CONST.MOTORS.LENGTH, (set_int16_t)Motor_User_SetTorqueCmd_Scalar, 0); break;
        case MOTOR_CONTROLLER_DRIVE_ZERO_MODE_REGEN: /* MotorController_ProcRegenMotorAll(p_mc); */ break;
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