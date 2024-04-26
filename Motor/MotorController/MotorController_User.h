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
    @file   MotorController_User.h
    @author FireSourcery
    @brief  UI Wrappers. End User accessor functions (chip external inputs),
                includes error checking
    @version V0
*/
/******************************************************************************/
#ifndef MOTOR_CONTROLLER_USER_H
#define MOTOR_CONTROLLER_USER_H

#include "MotorController_StateMachine.h"

/******************************************************************************/
/*
    Live Control User Input Interface; into StateMachine process
*/
/******************************************************************************/
/******************************************************************************/
/* Common */
/******************************************************************************/
// static inline void MotorController_User_ReleaseControl(MotorControllerPtr_T p_mc) { StateMachine_ProcInput(&p_mc->StateMachine, MCSM_INPUT_RELEASE, STATE_MACHINE_INPUT_VALUE_NULL); }

/******************************************************************************/
/* UserCmd - Common input, various handling */
/*! @param[in] userCmd [-32767:32767] */
/******************************************************************************/
// static inline int32_t MotorController_User_GetCmdValue(const MotorControllerPtr_T p_mc) { return p_mc->UserCmdValue; }
static inline void MotorController_User_SetCmdValue(MotorControllerPtr_T p_mc, int16_t userCmd) { StateMachine_ProcInput(&p_mc->StateMachine, MCSM_INPUT_CMD, userCmd); }

/******************************************************************************/
/* Drive Mode */
/******************************************************************************/
/******************************************************************************/
/* DriveCmd - Throttle, Brake, Zero */
/*! @param[in] driveCmd Throttle, Brake, Zero */
/*! @param[in] cmdValue [0:65535] */
/******************************************************************************/
// static inline void MotorController_User_SetDriveCmd(MotorControllerPtr_T p_mc, MotorController_DriveId_T cmdId, uint16_t cmdValue)
// {
//     // StateMachine_ProcInput(&p_mc->StateMachine, MCSM_INPUT_DRIVE, cmdId);
//     // p_mc->UserCmdValue = cmdValue;
//     StateMachine_ProcInput(&p_mc->StateMachine, cmdId, cmdValue);
// }
// /* Same as Brake/Throttle 0 */
// static inline void MotorController_User_SetDriveCmdZero(MotorControllerPtr_T p_mc)                   { MotorController_User_SetDriveCmd(p_mc, MOTOR_CONTROLLER_DRIVE_ZERO, 0U); }
// static inline void MotorController_User_SetCmdThrottle(MotorControllerPtr_T p_mc, uint16_t userCmd)  { MotorController_User_SetDriveCmd(p_mc, MOTOR_CONTROLLER_DRIVE_THROTTLE, userCmd); }
// static inline void MotorController_User_SetCmdBrake(MotorControllerPtr_T p_mc, uint16_t userCmd)     { MotorController_User_SetDriveCmd(p_mc, MOTOR_CONTROLLER_DRIVE_BRAKE, userCmd); }

static inline void MotorController_User_SetDriveCmdZero(MotorControllerPtr_T p_mc)                  { StateMachine_ProcInput(&p_mc->StateMachine, MCSM_INPUT_BRAKE, 0U); }
static inline void MotorController_User_SetCmdThrottle(MotorControllerPtr_T p_mc, uint16_t userCmd) { StateMachine_ProcInput(&p_mc->StateMachine, MCSM_INPUT_THROTTLE, userCmd); }
static inline void MotorController_User_SetCmdBrake(MotorControllerPtr_T p_mc, uint16_t userCmd)    { StateMachine_ProcInput(&p_mc->StateMachine, MCSM_INPUT_BRAKE, userCmd); }

/******************************************************************************/
/* Blocking */
/******************************************************************************/
/* Maybe blocking */
static inline void MotorController_User_InputLocked(MotorControllerPtr_T p_mc, MotorController_LockedId_T opId)
{
    StateMachine_ProcInput(&p_mc->StateMachine, MCSM_INPUT_LOCK, opId);
}

static inline bool MotorController_User_IsLockedState(MotorControllerPtr_T p_mc)
{
    return (StateMachine_GetActiveStateId(&p_mc->StateMachine) == MCSM_STATE_ID_LOCK);
}

static inline bool MotorController_User_EnterLockedState(MotorControllerPtr_T p_mc)
{
    MotorController_User_InputLocked(p_mc, MOTOR_CONTROLLER_LOCKED_ENTER);
    return MotorController_User_IsLockedState(p_mc);
}

static inline bool MotorController_User_ExitLockedState(MotorControllerPtr_T p_mc)
{
    MotorController_User_InputLocked(p_mc, MOTOR_CONTROLLER_LOCKED_EXIT);
    return (StateMachine_GetActiveStateId(&p_mc->StateMachine) == MCSM_STATE_ID_PARK);
}

static inline NvMemory_Status_T MotorController_User_SaveParameters_Blocking(MotorControllerPtr_T p_mc)
{
    MotorController_User_InputLocked(p_mc, MOTOR_CONTROLLER_LOCKED_NVM_SAVE_PARAMS);
    return p_mc->NvmStatus;
}

// return _MotorController_User_SaveNvm_Blocking(p_mc, MOTOR_CONTROLLER_LOCKED_NVM_SAVE_PARAMS);
/*! @param[in] opId MOTOR_CONTROLLER_NVM_BOOT, MOTOR_CONTROLLER_NVM_WRITE_ONCE, MOTOR_CONTROLLER_LOCKED_NVM_SAVE_PARAMS */
// static inline NvMemory_Status_T _MotorController_User_SaveNvm_Blocking(MotorControllerPtr_T p_mc, MotorController_LockedId_T opId)
// {
//     MotorController_User_InputLocked(p_mc, opId);
//     return p_mc->NvmStatus;
// }
// static inline NvMemory_Status_T MotorController_User_SaveManufacture_Blocking(MotorControllerPtr_T p_mc)
// {
//     return _MotorController_User_SaveNvm_Blocking(p_mc, MOTOR_CONTROLLER_BLOCKING_NVM_WRITE_ONCE);
//     // return StateMachine_ProcInput(p_mc, MCSM_INPUT_LOCK, MOTOR_CONTROLLER_LOCKED_NVM_SAVE_PARAMS);
//     // return p_mc->NvmStatus;
// }

// static inline NvMemory_Status_T MotorController_User_ReadManufacture_Blocking(MotorControllerPtr_T p_mc)
// {
//     MotorController_User_InputLocked(p_mc, MOTOR_CONTROLLER_LOCKED_NVM_READ_ONCE);
//     return p_mc->NvmStatus;
// }

/******************************************************************************/
/* Servo */
/******************************************************************************/
#ifdef CONFIG_MOTOR_CONTROLLER_SERVO_ENABLE
static inline void MotorController_User_SetServoCmd(MotorControllerPtr_T p_mc, int16_t userCmd) { StateMachine_ProcInput(&p_mc->StateMachine, MCSM_INPUT_SERVO, userCmd); }
static inline void MotorController_User_EnterServoMode(MotorControllerPtr_T p_mc)   { StateMachine_ProcInput(&p_mc->StateMachine, MCSM_INPUT_SERVO, STATE_MACHINE_INPUT_VALUE_NULL); }
static inline void MotorController_User_ExitServoMode(MotorControllerPtr_T p_mc)    { StateMachine_ProcInput(&p_mc->StateMachine, MCSM_INPUT_SERVO, STATE_MACHINE_INPUT_VALUE_NULL); }
#endif

/******************************************************************************/
/*
    Non StateMachine Checked Direct Inputs
*/
/******************************************************************************/
/* StateMachine unchecked disable motors, use with caution */
static inline void MotorController_User_DisableControl(MotorControllerPtr_T p_mc) { MotorController_DisableAll(p_mc); }

static inline void MotorController_User_BeepN(MotorControllerPtr_T p_mc, uint32_t onTime, uint32_t offTime, uint8_t n)   { Blinky_BlinkN(&p_mc->Buzzer, onTime, offTime, n); }
static inline void MotorController_User_BeepStart(MotorControllerPtr_T p_mc, uint32_t onTime, uint32_t offTime)          { Blinky_StartPeriodic(&p_mc->Buzzer, onTime, offTime); }
static inline void MotorController_User_BeepStop(MotorControllerPtr_T p_mc)                                              { Blinky_Stop(&p_mc->Buzzer); }

static inline void MotorController_User_SetSpeedLimitAll(MotorControllerPtr_T p_mc, uint16_t limit_scalar16)
    { MotorN_User_SetScalar16(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, Motor_User_SetSpeedLimitActive, limit_scalar16); }

static inline void MotorController_User_ClearSpeedLimitAll(MotorControllerPtr_T p_mc)
    { MotorN_User_ProcFunction(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, Motor_User_ClearSpeedLimitActive); }

static inline bool MotorController_User_SetILimitAll(MotorControllerPtr_T p_mc, uint16_t limit_scalar16)
    { return MotorN_User_SetLimit(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, Motor_User_SetILimitActive_Id, limit_scalar16, MOTOR_I_LIMIT_ACTIVE_USER); }

static inline bool MotorController_User_ClearILimitAll(MotorControllerPtr_T p_mc)
    { return MotorN_User_ClearLimit(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, Motor_User_ClearILimitActive_Id, MOTOR_I_LIMIT_ACTIVE_USER); }


/******************************************************************************/
/*
    Motor Controller Struct Variables
*/
/******************************************************************************/
/*
    Controller RAM Variables
*/
static inline MotorController_StateMachine_StateId_T MotorController_User_GetStateId(const MotorControllerPtr_T p_mc)   { return StateMachine_GetActiveStateId(&p_mc->StateMachine); }
static inline MotorController_StatusFlags_T MotorController_User_GetStatusFlags(const MotorControllerPtr_T p_mc)        { return p_mc->StatusFlags; }
static inline MotorController_FaultFlags_T MotorController_User_GetFaultFlags(const MotorControllerPtr_T p_mc)          { return p_mc->FaultFlags; }

static inline uint16_t MotorController_User_GetAdcu(const MotorControllerPtr_T p_mc, MotAnalog_Channel_T adcChannel)     { return p_mc->AnalogResults.Channels[adcChannel]; }
static inline uint8_t MotorController_User_GetAdcu_Msb8(const MotorControllerPtr_T p_mc, MotAnalog_Channel_T adcChannel) { return MotorController_User_GetAdcu(p_mc, adcChannel) >> (GLOBAL_ANALOG.ADC_BITS - 8U); }

static inline uint32_t MotorController_User_GetVSource_V(const MotorControllerPtr_T p_mc, uint16_t vScalar) { return VMonitor_ScalarVOf(&p_mc->VMonitorSource, p_mc->AnalogResults.VSource_Adcu, vScalar); }
static inline uint32_t MotorController_User_GetVSense_V(const MotorControllerPtr_T p_mc, uint16_t vScalar)  { return VMonitor_ScalarVOf(&p_mc->VMonitorSense, p_mc->AnalogResults.VSense_Adcu, vScalar); }
static inline uint32_t MotorController_User_GetVAccs_V(const MotorControllerPtr_T p_mc, uint16_t vScalar)   { return VMonitor_ScalarVOf(&p_mc->VMonitorAccs, p_mc->AnalogResults.VAccs_Adcu, vScalar); }

static inline uint16_t MotorController_User_GetHeatPcb_Adcu(const MotorControllerPtr_T p_mc)        { return p_mc->AnalogResults.HeatPcb_Adcu; }
static inline uint16_t MotorController_User_GetHeatMosfets_Adcu(const MotorControllerPtr_T p_mc)    { return p_mc->AnalogResults.HeatMosfets_Adcu; }
// static inline uint16_t MotorController_User_GetHeat_Adcu(const MotorControllerPtr_T p_mc, uint8_t index)    {  return p_mc->AnalogResultsThermal[index]; }

#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
    static inline uint32_t MotorController_User_GetBatteryCharge_Scalar16(const MotorControllerPtr_T p_mc)              { return Linear_ADC_CalcFracU16(&p_mc->BatteryLife, p_mc->AnalogResults.VSource_Adcu); }
    static inline uint32_t MotorController_User_GetBatteryCharge_Scalar1000(const MotorControllerPtr_T p_mc)            { return Linear_ADC_CalcPhysical(&p_mc->BatteryLife, p_mc->AnalogResults.VSource_Adcu); }
    static inline int32_t MotorController_User_GetHeatPcb_DegC(const MotorControllerPtr_T p_mc, uint8_t scalar)         { return Thermistor_ConvertToDegC_Scalar(&p_mc->ThermistorPcb, p_mc->AnalogResults.HeatPcb_Adcu, scalar); }
    static inline int32_t MotorController_User_GetHeatMosfets_DegC(const MotorControllerPtr_T p_mc, uint8_t scalar)     { return Thermistor_ConvertToDegC_Scalar(&p_mc->ThermistorMosfets, p_mc->AnalogResults.HeatMosfets_Adcu, scalar); }
#if defined(CONFIG_MOTOR_CONTROLLER_HEAT_MOSFETS_TOP_BOT_ENABLE)
    static inline int32_t MotorController_User_GetHeatMosfetsTop_DegC(const MotorControllerPtr_T p_mc, uint8_t scalar)  { return Thermistor_ConvertToDegC_Scalar(&p_mc->ThermistorMosfetsTop, p_mc->AnalogResults.HeatMosfetsTop_Adcu, scalar); }
    static inline int32_t MotorController_User_GetHeatMosfetsBot_DegC(const MotorControllerPtr_T p_mc, uint8_t scalar)  { return Thermistor_ConvertToDegC_Scalar(&p_mc->ThermistorMosfetsBot, p_mc->AnalogResults.HeatMosfetsBot_Adcu, scalar); }
#endif
#endif

#if    defined(CONFIG_MOTOR_CONTROLLER_DEBUG_ENABLE)
static inline uint16_t MotorController_User_GetFaultAdcu(const MotorControllerPtr_T p_mc, MotAnalog_Channel_T adcChannel)   { return p_mc->FaultAnalogRecord.Channels[adcChannel]; }
static inline uint32_t MotorController_User_GetFaultVSource(const MotorControllerPtr_T p_mc, uint16_t vScalar)              { return VMonitor_ScalarVOf(&p_mc->VMonitorSource, p_mc->FaultAnalogRecord.VSource_Adcu, vScalar); }
static inline uint32_t MotorController_User_GetFaultVSense(const MotorControllerPtr_T p_mc, uint16_t vScalar)               { return VMonitor_ScalarVOf(&p_mc->VMonitorSense, p_mc->FaultAnalogRecord.VSense_Adcu, vScalar); }
static inline uint32_t MotorController_User_GetFaultVAcc(const MotorControllerPtr_T p_mc, uint16_t vScalar)                 { return VMonitor_ScalarVOf(&p_mc->VMonitorAccs, p_mc->FaultAnalogRecord.VAccs_Adcu, vScalar); }
#if  defined(CONFIG_MOTOR_UNIT_CONVERSION_LOCAL)
static inline int32_t MotorController_User_GetFaultHeatPcb_DegC(const  MotorControllerPtr_T p_mc, uint8_t scalar)           { return Thermistor_ConvertToDegC_Scalar(&p_mc->ThermistorPcb, p_mc->FaultAnalogRecord.HeatPcb_Adcu, scalar); }
static inline int32_t MotorController_User_GetFaultHeatMosfets_DegC(const  MotorControllerPtr_T p_mc, uint8_t scalar)       { return Thermistor_ConvertToDegC_Scalar(&p_mc->ThermistorMosfets, p_mc->FaultAnalogRecord.HeatMosfets_Adcu, scalar); }
#if defined(CONFIG_MOTOR_CONTROLLER_HEAT_MOSFETS_TOP_BOT_ENABLE)
static inline int32_t MotorController_User_GetFaultHeatMosfetsTop_DegC(const MotorControllerPtr_T p_mc, uint8_t scalar)     { return Thermistor_ConvertToDegC_Scalar(&p_mc->ThermistorMosfetsTop, p_mc->FaultAnalogRecord.HeatMosfetsTop_Adcu, scalar); }
static inline int32_t MotorController_User_GetFaultHeatMosfetsBot_DegC(const MotorControllerPtr_T p_mc, uint8_t scalar)     { return Thermistor_ConvertToDegC_Scalar(&p_mc->ThermistorMosfetsBot, p_mc->FaultAnalogRecord.HeatMosfetsBot_Adcu, scalar); }
#endif
#endif
#endif

/*
    Boot Buffer
*/
static inline BootRef_T MotorController_User_GetBootReg(const MotorControllerPtr_T p_mc)                { return p_mc->BootRef; }
static inline void MotorController_User_SetBootReg(MotorControllerPtr_T p_mc, BootRef_T bootReg)        { p_mc->BootRef.Word = bootReg.Word; }
// static inline void MotorController_User_SetFastBoot(MotorControllerPtr_T p_mc, bool isEnable)        { p_mc->BootRef.FastBoot = isEnable; }
// static inline void MotorController_User_SetLoadDefault(MotorControllerPtr_T p_mc, bool isEnable)     { p_mc->BootRef.LoadDefault = isEnable; }

/*
    Controller NvM Variables Parameters
*/
static inline uint16_t MotorController_User_GetVSourceRef(const MotorControllerPtr_T p_mc)           { return p_mc->Parameters.VSourceRef; }

static inline Motor_FeedbackMode_T MotorController_User_GetDefaultFeedbackMode(const MotorControllerPtr_T p_mc)     { return p_mc->Parameters.DefaultCmdMode; }
static inline void MotorController_User_SetDefaultFeedbackMode(MotorControllerPtr_T p_mc, uint16_t wordValue)       { p_mc->Parameters.DefaultCmdMode.Word = wordValue; }

static inline MotorController_InitMode_T MotorController_User_GetInitMode(const MotorControllerPtr_T p_mc)           { return p_mc->Parameters.InitMode; }
static inline MotorController_InputMode_T MotorController_User_GetInputMode(const MotorControllerPtr_T p_mc)         { return p_mc->Parameters.InputMode; }
static inline MotorController_BrakeMode_T MotorController_User_GetBrakeMode(const MotorControllerPtr_T p_mc)         { return p_mc->Parameters.BrakeMode; }
static inline MotorController_ThrottleMode_T MotorController_User_GetThrottleMode(const MotorControllerPtr_T p_mc)   { return p_mc->Parameters.ThrottleMode; }
static inline MotorController_DriveZeroMode_T MotorController_User_GetDriveZeroMode(const MotorControllerPtr_T p_mc) { return p_mc->Parameters.DriveZeroMode; }

static inline void MotorController_User_SetInitMode(MotorControllerPtr_T p_mc, MotorController_InitMode_T mode)             { p_mc->Parameters.InitMode = mode; }
static inline void MotorController_User_SetInputMode(MotorControllerPtr_T p_mc, MotorController_InputMode_T mode)           { p_mc->Parameters.InputMode = mode; }
static inline void MotorController_User_SetBrakeMode(MotorControllerPtr_T p_mc, MotorController_BrakeMode_T mode)           { p_mc->Parameters.BrakeMode = mode; }
static inline void MotorController_User_SetThrottleMode(MotorControllerPtr_T p_mc, MotorController_ThrottleMode_T mode)     { p_mc->Parameters.ThrottleMode = mode; }
static inline void MotorController_User_SetDriveZeroMode(MotorControllerPtr_T p_mc, MotorController_DriveZeroMode_T mode)   { p_mc->Parameters.DriveZeroMode = mode; }
static inline void MotorController_User_SetILimitOnLowV(MotorControllerPtr_T p_mc, uint16_t scalar_Frac16)                  { p_mc->Parameters.ILimitLowV_Scalar16 = scalar_Frac16; }

static inline void MotorController_User_SetOptDinMode(MotorControllerPtr_T p_mc, MotorController_OptDinMode_T mode) { p_mc->Parameters.OptDinMode = mode; }
static inline void MotorController_User_DisableOptDin(MotorControllerPtr_T p_mc)                                    { p_mc->Parameters.OptDinMode = MOTOR_CONTROLLER_OPT_DIN_DISABLE; }
static inline bool MotorController_User_SetOptDinSpeedLimit(MotorControllerPtr_T p_mc, uint16_t scalar_Frac16)
{
    bool isSuccess = (p_mc->Parameters.OptDinMode == MOTOR_CONTROLLER_OPT_DIN_SPEED_LIMIT);
    if(isSuccess) { p_mc->Parameters.OptDinSpeedLimit_Scalar16 = scalar_Frac16; }
    return isSuccess;
}

/*
    Read Only Params
*/
static inline uint32_t MotorController_User_GetBoardVersion(void)
{
    uint8_t version[4U] = { GLOBAL_MOTOR.I_MAX_ADCU, GLOBAL_MOTOR.I_MAX_ADCU >> 8U, GLOBAL_MOTOR.I_MAX_AMPS, GLOBAL_MOTOR.V_MAX_VOLTS };
    return *((uint32_t *)(&version[0U]));
}

static inline uint16_t MotorController_User_GetVMax(void) { return GLOBAL_MOTOR.V_MAX_VOLTS; }
static inline uint16_t MotorController_User_GetIMax(void) { return GLOBAL_MOTOR.I_MAX_AMPS; }
static inline uint16_t MotorController_User_GetIMaxAdcu(void) { return GLOBAL_MOTOR.I_MAX_ADCU; }

static inline uint32_t MotorController_User_GetLibraryVersion(void) { return MOTOR_LIBRARY_VERSION_ID; }
static inline uint8_t MotorController_User_GetLibraryVersionIndex(uint8_t charIndex)
{
    uint8_t versionChar;
    switch(charIndex)
    {
        case 0U: versionChar = MOTOR_LIBRARY_VERSION_FIX; break;
        case 1U: versionChar = MOTOR_LIBRARY_VERSION_MINOR;  break;
        case 2U: versionChar = MOTOR_LIBRARY_VERSION_MAJOR;  break;
        case 3U: versionChar = MOTOR_LIBRARY_VERSION_OPT;    break;
        default: versionChar = 0U; break;
    }
    return versionChar;
}

static inline uint32_t MotorController_User_GetMainVersion(const MotorControllerPtr_T p_mc) { return *((uint32_t *)(&p_mc->CONFIG.MAIN_VERSION[0U])); }
static inline uint8_t MotorController_User_GetMainVersionIndex(const MotorControllerPtr_T p_mc, uint8_t charIndex) { return p_mc->CONFIG.MAIN_VERSION[charIndex]; }

/******************************************************************************/
/*!
    Instance Select
    @return may return null
*/
/******************************************************************************/
static inline MotorPtr_T MotorController_User_GetPtrMotor(const MotorControllerPtr_T p_mc, uint8_t motorIndex)
{
    return (motorIndex < p_mc->CONFIG.MOTOR_COUNT) ? MotorController_GetPtrMotor(p_mc, motorIndex) : NULL;
}

static inline Protocol_T * MotorController_User_GetPtrProtocol(const MotorControllerPtr_T p_mc, uint8_t protocolIndex)
{
    return (protocolIndex < p_mc->CONFIG.PROTOCOL_COUNT) ? &p_mc->CONFIG.P_PROTOCOLS[protocolIndex] : NULL;
}

static inline Thermistor_T * MotorController_User_GetPtrThermistor(const MotorControllerPtr_T p_mc, uint8_t index)
{
    Thermistor_T * p_thermistor;
    switch(index)
    {
        case 0U: p_thermistor = &p_mc->ThermistorPcb; break;
        case 1U: p_thermistor = &p_mc->ThermistorMosfets; break;
        //todo
        case 2U: p_thermistor = &p_mc->ThermistorMosfets; break;
        case 3U: p_thermistor = &p_mc->ThermistorMosfets; break;
        case 4U: p_thermistor = &p_mc->ThermistorMosfets; break;
        default: p_thermistor = NULL; break;
    }
    return p_thermistor;
}

static inline VMonitor_T * MotorController_User_GetPtrVMonitor(const MotorControllerPtr_T p_mc, uint8_t index)
{
    VMonitor_T * p_vMonitor;
    switch(index)
    {
        case 0U: p_vMonitor = &p_mc->VMonitorSource; break;
        case 1U: p_vMonitor = &p_mc->VMonitorSense;  break;
        case 2U: p_vMonitor = &p_mc->VMonitorAccs;   break;
        default: p_vMonitor = NULL; break;
    }
    return p_vMonitor;
}

/*
    Wrappers for MainProtocol aka Protocol[0]
*/
// static inline void MotorController_User_SetMainProtocolXcvr(MotorControllerPtr_T p_mc, uint8_t id) { Protocol_SetSpecs(&p_mc->CONFIG.P_PROTOCOLS[0U], id); }
// static inline void MotorController_User_SetMainProtoclSpecs(MotorControllerPtr_T p_mc, uint8_t id) { Protocol_SetXcvr(&p_mc->CONFIG.P_PROTOCOLS[0U], id); }

/******************************************************************************/
/*
    Extern
*/
/******************************************************************************/
extern MotorController_Direction_T MotorController_User_GetDirection(const MotorControllerPtr_T p_mc);
extern bool MotorController_User_SetDirection(MotorControllerPtr_T p_mc, MotorController_Direction_T direction);
extern void MotorController_User_SetVSourceRef(MotorControllerPtr_T p_mc, uint16_t volts);

#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
extern void MotorController_User_SetBatteryLifeDefault(MotorControllerPtr_T p_mc);
extern void MotorController_User_SetBatteryLife_MilliV(MotorControllerPtr_T p_mc, uint32_t zero_mV, uint32_t max_mV);
#endif

#endif
