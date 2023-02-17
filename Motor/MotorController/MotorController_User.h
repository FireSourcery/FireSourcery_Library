/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2021 FireSourcery / The Firebrand Forge Inc

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
    @file     MotorController_User.h
    @author FireSourcery
    @brief     UI Wrappers. End User accessor functions (chip external inputs),
                includes error checking
    @version V0
*/
/******************************************************************************/
#ifndef MOTOR_CONTROLLER_USER_H
#define MOTOR_CONTROLLER_USER_H

#include "MotorController_StateMachine.h"

/******************************************************************************/
/*
    Live Control User Input Interface subject to StateMachine process
*/
/******************************************************************************/

/******************************************************************************/
/*
    UserCmd - determine state machine input mode here,
        common input SubState proc across state machine input modes
*/
/******************************************************************************/
static inline void MotorController_User_SetCmdValue(MotorController_T * p_mc, int16_t userCmd)      { StateMachine_Semi_ProcInput(&p_mc->StateMachine, MCSM_INPUT_CMD, userCmd); }
static inline void MotorController_User_SetCmdThrottle(MotorController_T * p_mc, uint16_t userCmd)  { StateMachine_Semi_ProcInput(&p_mc->StateMachine, MCSM_INPUT_THROTTLE, userCmd); }
static inline void MotorController_User_SetCmdBrake(MotorController_T * p_mc, uint16_t userCmd)     { StateMachine_Semi_ProcInput(&p_mc->StateMachine, MCSM_INPUT_BRAKE, userCmd); }
static inline uint16_t MotorController_User_GetCmdValue(const MotorController_T * p_mc)             { return p_mc->UserCmd; }

/* Zero Throttle or Brake *///remove MotorController_User_SetCmdValue(p_mc, 0);
static inline void MotorController_User_SetCmdZero(MotorController_T * p_mc)    { StateMachine_Semi_ProcInput(&p_mc->StateMachine, MCSM_INPUT_ZERO, STATE_MACHINE_INPUT_VALUE_NULL); }
/* StateMachine unchecked disable motors, use with caution */
static inline void MotorController_User_ReleaseControl(MotorController_T * p_mc) { MotorController_User_SetCmdZero(p_mc); }
// static inline void MotorController_User_ReleaseControl(MotorController_T * p_mc) { MotorController_User_SetCmdValue(p_mc, 0); }
static inline void MotorController_User_DisableControl(MotorController_T * p_mc) { MotorController_DisableAll(p_mc); MotorController_User_ReleaseControl(p_mc); }

/*
    Input mode voluntarily call checked function, avoids intermediate buffer
*/
//static inline void MotorController_User_Serial_SetCmdValue(MotorController_T * p_mc, uint16_t userCmd)     { if (p_mc->Parameters.UserInputMode == MOTOR_CONTROLLER_INPUT_MODE_PROTOCOL) {p_mc->UserCmd = userCmd;} }
//static inline void MotorController_User_CanBus_SetCmdValue(MotorController_T * p_mc, uint16_t userCmd)     { if (p_mc->Parameters.UserInputMode == MOTOR_CONTROLLER_INPUT_MODE_CAN) {p_mc->UserCmd = userCmd;} }

/******************************************************************************/
/* Direction */
/******************************************************************************/
static inline void MotorController_User_SetDirection(MotorController_T * p_mc, MotorController_Direction_T direction)
{
    if(p_mc->ActiveDirection != direction) { StateMachine_Semi_ProcInput(&p_mc->StateMachine, MCSM_INPUT_SET_DIRECTION, direction); }
}

static inline MotorController_Direction_T MotorController_User_GetDirection(const MotorController_T * p_mc) { return p_mc->ActiveDirection; }

/******************************************************************************/
/* Servo  */
/******************************************************************************/
#ifdef CONFIG_MOTOR_CONTROLLER_SERVO_ENABLE
static inline void MotorController_User_EnterServoMode(MotorController_T * p_mc) { StateMachine_Semi_ProcInput(&p_mc->StateMachine, MCSM_INPUT_SERVO, STATE_MACHINE_INPUT_VALUE_NULL); }
static inline void MotorController_User_ExitServoMode(MotorController_T * p_mc) { StateMachine_Semi_ProcInput(&p_mc->StateMachine, MCSM_INPUT_SET_DIRECTION, STATE_MACHINE_INPUT_VALUE_NULL); }
#endif

/******************************************************************************/
/* Save NvMemory sections */
/******************************************************************************/
static inline void MotorController_User_SaveParameters_Blocking(MotorController_T * p_mc)
{
    StateMachine_Semi_ProcInput(&p_mc->StateMachine, MCSM_INPUT_CALIBRATION, MOTOR_CONTROLLER_NVM_PARAMS_ALL);
}

static inline void MotorController_User_SaveBootReg_Blocking(MotorController_T * p_mc)
{
    StateMachine_Semi_ProcInput(&p_mc->StateMachine, MCSM_INPUT_CALIBRATION, MOTOR_CONTROLLER_NVM_BOOT);
}

static inline void MotorController_User_SaveOnce_Blocking(MotorController_T * p_mc)
{
    StateMachine_Semi_ProcInput(&p_mc->StateMachine, MCSM_INPUT_CALIBRATION, MOTOR_CONTROLLER_NVM_WRITE_ONCE);
}

static inline void MotorController_User_ProcCalibration_Blocking(MotorController_T * p_mc, MotorController_OperationId_T operationId)
{
    StateMachine_Semi_ProcInput(&p_mc->StateMachine, MCSM_INPUT_CALIBRATION, operationId);
}

static inline void MotorController_User_SetInputMode_Blocking(MotorController_T * p_mc, MotorController_InputMode_T inputMode)
{
    // switch((MotorController_InputMode_T)varValue)
    // {
    //     case MOTOR_CONTROLLER_INPUT_MODE_ANALOG:    break;
    //     case MOTOR_CONTROLLER_INPUT_MODE_PROTOCOL:  break;
    //     case MOTOR_CONTROLLER_INPUT_MODE_DISABLE:   break;
    //     case MOTOR_CONTROLLER_INPUT_MODE_CAN:       break;
    //     default:     break;
    // }
    // MotorController_User_ProcCalibration_Blocking(p_mc, MOTOR_CONTROLLER_TOGGLE_USER_INPUT_MODE);
}

/******************************************************************************/
/* Fault */
/******************************************************************************/
static inline bool MotorController_User_CheckFault(MotorController_T * p_mc)
{
    return (StateMachine_GetActiveStateId(&p_mc->StateMachine) == MCSM_STATE_ID_FAULT);
}

/* returns true if no fault active at the end of function */
static inline bool MotorController_User_ClearFault(MotorController_T * p_mc)
{
    if(StateMachine_GetActiveStateId(&p_mc->StateMachine) == MCSM_STATE_ID_FAULT) { StateMachine_Semi_ProcInput(&p_mc->StateMachine, MCSM_INPUT_FAULT, STATE_MACHINE_INPUT_VALUE_NULL); }
    return (StateMachine_GetActiveStateId(&p_mc->StateMachine) != MCSM_STATE_ID_FAULT);
}

/* system fault - may move */
static inline void MotorController_User_SetFault(MotorController_T * p_mc)
{
    if(StateMachine_GetActiveStateId(&p_mc->StateMachine) != MCSM_STATE_ID_FAULT) { StateMachine_Semi_ProcInput(&p_mc->StateMachine, MCSM_INPUT_FAULT, STATE_MACHINE_INPUT_VALUE_NULL); }
}

/* user activate fault */
static inline void MotorController_User_SetUserFault(MotorController_T * p_mc)      { p_mc->FaultFlags.User = 1U; MotorController_User_SetFault(p_mc); }
static inline bool MotorController_User_ClearUserFault(MotorController_T * p_mc)    { p_mc->FaultFlags.User = 0U; return MotorController_User_ClearFault(p_mc); }

static inline void MotorController_User_ToggleUserFault(MotorController_T * p_mc)
{
    if(p_mc->FaultFlags.User == 1U) { MotorController_User_SetUserFault(p_mc); }
    else                            { MotorController_User_ClearUserFault(p_mc); }
}

/******************************************************************************/
/*
    Non StateMachine Checked Direct Inputs
*/
/******************************************************************************/
static inline void MotorController_User_BeepN(MotorController_T * p_mc, uint32_t onTime, uint32_t offTime, uint8_t n)   { Blinky_BlinkN(&p_mc->Buzzer, onTime, offTime, n); }
static inline void MotorController_User_BeepStart(MotorController_T * p_mc, uint32_t onTime, uint32_t offTime)          { Blinky_StartPeriodic(&p_mc->Buzzer, onTime, offTime); }
static inline void MotorController_User_BeepStop(MotorController_T * p_mc)                                              { Blinky_Stop(&p_mc->Buzzer); }

/******************************************************************************/
/*
    Motor Controller Struct Variables
*/
/******************************************************************************/
/*
    Controller RAM Variables
*/
static inline MotorController_StateMachine_StateId_T MotorController_User_GetStateId(const MotorController_T * p_mc)    { return StateMachine_GetActiveStateId(&p_mc->StateMachine); }

static inline uint16_t MotorController_User_GetAdcu(const MotorController_T * p_mc, MotAnalog_Channel_T adcChannel)     { return p_mc->AnalogResults.Channels[adcChannel]; }
static inline uint8_t MotorController_User_GetAdcu_Msb8(const MotorController_T * p_mc, MotAnalog_Channel_T adcChannel) { return MotorController_User_GetAdcu(p_mc, adcChannel) >> (GLOBAL_ANALOG.ADC_BITS - 8U); }
static inline uint32_t MotorController_User_GetVSource(const MotorController_T * p_mc, uint16_t vScalar)                { return VMonitor_ConvertToV(&p_mc->VMonitorSource, p_mc->AnalogResults.VSource_Adcu, vScalar); }
static inline uint32_t MotorController_User_GetVSense(const MotorController_T * p_mc, uint16_t vScalar)                 { return VMonitor_ConvertToV(&p_mc->VMonitorSense, p_mc->AnalogResults.VSense_Adcu, vScalar); }
static inline uint32_t MotorController_User_GetVAcc(const MotorController_T * p_mc, uint16_t vScalar)                   { return VMonitor_ConvertToV(&p_mc->VMonitorAcc, p_mc->AnalogResults.VAcc_Adcu, vScalar); }
static inline int32_t MotorController_User_GetHeatPcb_DegC(const MotorController_T * p_mc, uint8_t scalar)              { return Thermistor_ConvertToDegC_Int(&p_mc->ThermistorPcb, p_mc->AnalogResults.HeatPcb_Adcu, scalar); }
static inline int32_t MotorController_User_GetHeatMosfets_DegC(const MotorController_T * p_mc, uint8_t scalar)          { return Thermistor_ConvertToDegC_Int(&p_mc->ThermistorMosfets, p_mc->AnalogResults.HeatMosfets_Adcu, scalar); }
#if defined(CONFIG_MOTOR_CONTROLLER_HEAT_MOSFETS_TOP_BOT_ENABLE)
static inline int32_t MotorController_User_GetHeatMosfetsTop_DegC(const MotorController_T * p_mc, uint8_t scalar) { return Thermistor_ConvertToDegC_Int(&p_mc->ThermistorMosfetsTop, p_mc->AnalogResults.HeatMosfetsTop_Adcu, scalar); }
static inline int32_t MotorController_User_GetHeatMosfetsBot_DegC(const MotorController_T * p_mc, uint8_t scalar) { return Thermistor_ConvertToDegC_Int(&p_mc->ThermistorMosfetsBot, p_mc->AnalogResults.HeatMosfetsBot_Adcu, scalar); }
#endif

#if    defined(CONFIG_MOTOR_CONTROLLER_DEBUG_ENABLE) /* change to fault records ? */
static inline uint16_t MotorController_User_GetFaultAdcu(const MotorController_T * p_mc, MotAnalog_Channel_T adcChannel)    { return p_mc->FaultAnalogRecord.Channels[adcChannel]; }
static inline uint32_t MotorController_User_GetFaultVSource(const MotorController_T * p_mc, uint16_t vScalar)               { return VMonitor_ConvertToV(&p_mc->VMonitorSource, p_mc->FaultAnalogRecord.VSource_Adcu, vScalar); }
static inline uint32_t MotorController_User_GetFaultVSense(const MotorController_T * p_mc, uint16_t vScalar)                { return VMonitor_ConvertToV(&p_mc->VMonitorSense, p_mc->FaultAnalogRecord.VSense_Adcu, vScalar); }
static inline uint32_t MotorController_User_GetFaultVAcc(const MotorController_T * p_mc, uint16_t vScalar)                  { return VMonitor_ConvertToV(&p_mc->VMonitorAcc, p_mc->FaultAnalogRecord.VAcc_Adcu, vScalar); }
static inline int32_t MotorController_User_GetFaultHeatPcb_DegC(const  MotorController_T * p_mc, uint8_t scalar)            { return Thermistor_ConvertToDegC_Int(&p_mc->ThermistorPcb, p_mc->FaultAnalogRecord.HeatPcb_Adcu, scalar); }
static inline int32_t MotorController_User_GetFaultHeatMosfets_DegC(const  MotorController_T * p_mc, uint8_t scalar)        { return Thermistor_ConvertToDegC_Int(&p_mc->ThermistorMosfets, p_mc->FaultAnalogRecord.HeatMosfets_Adcu, scalar); }
// static inline int32_t MotorController_User_GetFaultHeatMosfetsTop_DegC(const MotorController_T * p_mc, uint8_t scalar)     { return Thermistor_ConvertToDegC_Int(&p_mc->ThermistorMosfetsTop, p_mc->FaultAnalogRecord.HeatMosfetsTop_Adcu, scalar); }
// static inline int32_t MotorController_User_GetFaultHeatMosfetsBot_DegC(const MotorController_T * p_mc, uint8_t scalar)     { return Thermistor_ConvertToDegC_Int(&p_mc->ThermistorMosfetsBot, p_mc->FaultAnalogRecord.HeatMosfetsBot_Adcu, scalar); }
#endif

/*
    Controller NvM Variables Parameters
*/
static inline uint16_t MotorController_User_GetVSourceRef(const MotorController_T * p_mc)           { return p_mc->Parameters.VSourceRef; }

static inline MemMapBoot_T MotorController_User_GetBootReg(const MotorController_T * p_mc)          { return p_mc->MemMapBoot; }
static inline void MotorController_User_SetBootReg(MotorController_T * p_mc, MemMapBoot_T bootReg)  { p_mc->MemMapBoot = bootReg; }
static inline void MotorController_User_SetFastBoot(MotorController_T * p_mc, bool isEnable)        { p_mc->MemMapBoot.FastBoot = isEnable; }
static inline void MotorController_User_SetLoadDefault(MotorController_T * p_mc, bool isEnable)     { p_mc->MemMapBoot.LoadDefault = isEnable; }

static inline MotorController_InputMode_T MotorController_User_GetInputMode(const MotorController_T * p_mc)             { return p_mc->Parameters.UserInputMode; }
static inline void MotorController_User_SetBrakeMode(MotorController_T * p_mc, MotorController_BrakeMode_T brakeMode)   { p_mc->Parameters.BrakeMode = brakeMode; }
static inline void MotorController_User_SetOptDinSpeedLimit(MotorController_T * p_mc, uint16_t scalar_Frac16)           { p_mc->Parameters.OptDinFunction = MOTOR_CONTROLLER_OPT_DIN_SPEED_LIMIT; p_mc->Parameters.OptDinSpeedLimit_Scalar16 = scalar_Frac16; }
static inline void MotorController_User_DisableOptDin(MotorController_T * p_mc)                                         { p_mc->Parameters.OptDinFunction = MOTOR_CONTROLLER_OPT_DIN_DISABLE; }
static inline void MotorController_User_SetILimitOnLowVParam(MotorController_T * p_mc, uint16_t scalar_Frac16)          { p_mc->Parameters.ILimitLowV_Scalar16 = scalar_Frac16; }

static inline uint32_t MotorController_User_GetBatteryCharge_Scalar16(const MotorController_T * p_mc) { return Linear_ADC_CalcFracU16(&p_mc->BatteryLife, p_mc->AnalogResults.VSource_Adcu); }
static inline uint32_t MotorController_User_GetBatteryCharge_Unit1000(const MotorController_T * p_mc) { return Linear_ADC_CalcPhysical(&p_mc->BatteryLife, p_mc->AnalogResults.VSource_Adcu); }

/******************************************************************************/
/*
    Read Only Params
*/
/******************************************************************************/
/*
    Read Only
*/
static inline uint32_t MotorController_User_GetLibraryVersion(void) { return MOT_SOFTWARE_VERSION_ID; }
static inline uint8_t MotorController_User_GetLibraryVersionIndex(uint8_t charIndex)
{
    uint8_t versionChar;
    switch(charIndex)
    {
        case 0U: versionChar = MOT_SOFTWARE_VERSION_BUGFIX; break;
        case 1U: versionChar = MOT_SOFTWARE_VERSION_MINOR;  break;
        case 2U: versionChar = MOT_SOFTWARE_VERSION_MAJOR;  break;
        case 3U: versionChar = MOT_SOFTWARE_VERSION_OPT;    break;
        default: versionChar = 0U; break;
    }
    return versionChar;
}

static inline uint32_t MotorController_User_GetMainVersion(const MotorController_T * p_mc) { return *((uint32_t *)(&p_mc->CONFIG.SOFTWARE_VERSION[0U])); }
static inline uint8_t MotorController_User_GetMainVersionIndex(const MotorController_T * p_mc, uint8_t charIndex) { return p_mc->CONFIG.SOFTWARE_VERSION[charIndex]; }

static inline uint32_t MotorController_User_GetVMax(const MotorController_T * p_mc) { return GLOBAL_MOTOR.V_MAX_VOLTS; }
static inline uint32_t MotorController_User_GetIMax(const MotorController_T * p_mc) { return GLOBAL_MOTOR.I_UNITS_AMPS; }

/*
    WriteOnce Variables
*/
// #if defined(CONFIG_MOTOR_CONTROLLER_FLASH_LOADER_ENABLE)
/* Save Once does not need state machine */
// static inline void MotorController_User_WriteManufacture_Blocking(MotorController_T * p_mc, const MotorController_Manufacture_T * p_data)
// {
// #if defined(CONFIG_MOTOR_CONTROLLER_MANUFACTURE_PARAMS_RAM_COPY_ENABLE)
//     memcpy(&p_mc->Manufacture, p_data, sizeof(MotorController_Manufacture_T));
// #endif
//     MotorController_User_ProcCalibration_Blocking(p_mc, MOTOR_CONTROLLER_NVM_WRITE_ONCE);
// }

// static inline void MotorController_User_ReadManufacture_Blocking(MotorController_T * p_mc)
// {
//     MotorController_User_ProcCalibration_Blocking(p_mc, MOTOR_CONTROLLER_NVM_READ_ONCE);
// }

// #if defined(CONFIG_MOTOR_CONTROLLER_MANUFACTURE_PARAMS_RAM_COPY_ENABLE)

// static inline void MotorController_User_GetName(MotorController_T * p_mc, uint8_t * p_stringBuffer) { memcpy(p_stringBuffer, &p_mc->Manufacture.NAME[0U], 8U); }
// static inline char * MotorController_User_GetPtrName(MotorController_T * p_mc) { return &p_mc->Manufacture.NAME[0U]; }
// // static inline char MotorController_User_GetNameIndex(MotorController_T * p_mc, uint8_t charIndex) { return p_mc->Manufacture.NAME[charIndex]; }

// static inline void MotorController_User_GetManufacture(MotorController_T * p_mc, MotorController_Manufacture_T * p_dest) { memcpy(p_dest, &p_mc->Manufacture, sizeof(MotorController_Manufacture_T)); }
// static inline uint32_t MotorController_User_GetSerialNumber(MotorController_T * p_mc) { return p_mc->Manufacture.SERIAL_NUMBER_REG; }
// static inline uint32_t MotorController_User_GetManufactureDate(MotorController_T * p_mc) { return p_mc->Manufacture.MANUFACTURE_NUMBER_REG; }
// // static inline void MotorController_User_GetIdExt(MotorController_T * p_mc, uint8_t * p_stringBuffer) { memcpy(p_stringBuffer, &p_mc->Manufacture.ID_EXT[0U], 8U); }
// #endif

// static inline Flash_Status_T MotorController_User_ReadName_Blocking(MotorController_T * p_mc, uint8_t charIndex)
// {
//     //set read once, goto statemachine
// // #if defined(CONFIG_MOTOR_CONTROLLER_ONCE_USE_FLASH)
// //     return p_mc->CONFIG.P_ONCE->NAME[charIndex];
// // #elif defined(CONFIG_MOTOR_CONTROLLER_ONCE_USE_ONCE)
//     return Flash_ReadOnce_Blocking(p_mc->CONFIG.P_FLASH, &p_mc->CONFIG.P_ONCE->NAME[charIndex], 8U);
// // #endif
// }
// static inline Flash_Status_T MotorController_User_WriteName_Blocking(MotorController_T * p_mc, const uint8_t * p_nameString)
// {
//     return Flash_WriteOnce_Blocking(p_mc->CONFIG.P_FLASH, &p_mc->CONFIG.P_ONCE->NAME[0U], p_nameString, 8U);
// };
// #endif

/******************************************************************************/
/*
    Wrappers
*/
/******************************************************************************/
/*
    Motor Wrapper, no implementation of motor wrappers
*/
static inline Motor_T * MotorController_User_GetPtrMotor(const MotorController_T * p_mc, uint8_t motorIndex)
{
    return (motorIndex < p_mc->CONFIG.MOTOR_COUNT) ? MotorController_GetPtrMotor(p_mc, motorIndex) : MotorController_GetPtrMotor(p_mc, 0U);
}

static inline Protocol_T * MotorController_User_GetPtrProtocol(const MotorController_T * p_mc, uint8_t protocolIndex)
{
    return (protocolIndex < p_mc->CONFIG.PROTOCOL_COUNT) ? &p_mc->CONFIG.P_PROTOCOLS[protocolIndex] : &p_mc->CONFIG.P_PROTOCOLS[0U];
}

/*
    Wrappers for MainProtocol aka Protocol[0]
*/
static inline void MotorController_User_SetMainProtocolXcvr(MotorController_T * p_mc, uint8_t id) { Protocol_SetSpecs(&p_mc->CONFIG.P_PROTOCOLS[0U], id); }
static inline void MotorController_User_SetMainProtoclSpecs(MotorController_T * p_mc, uint8_t id) { Protocol_SetXcvr(&p_mc->CONFIG.P_PROTOCOLS[0U], id); }

/******************************************************************************/
/*
    Extern
*/
/******************************************************************************/
extern void MotorController_User_SetAdcVRef(MotorController_T * p_mc, uint16_t adcVRef_MilliV);
extern void MotorController_User_SetVSource(MotorController_T * p_mc, uint16_t volts);
extern void MotorController_User_SetBatteryLifeDefault(MotorController_T * p_mc);
extern void MotorController_User_SetBatteryLife_MilliV(MotorController_T * p_mc, uint32_t zero_mV, uint32_t max_mV);

#endif


/*
    Module additional wrappers, ram and nvm
*/
// static inline uint16_t MotorController_User_GetThrottle(MotorController_T * p_mc)         { return MotAnalogUser_GetThrottle(&p_mc->AnalogUser); }
// static inline uint8_t MotorController_User_GetThrottle_Frac8(MotorController_T * p_mc)     { return MotAnalogUser_GetThrottle(&p_mc->AnalogUser) >> 8U; }
// static inline uint16_t MotorController_User_GetBrake(MotorController_T * p_mc)             { return MotAnalogUser_GetBrake(&p_mc->AnalogUser); }
// static inline uint8_t MotorController_User_GetBrake_Frac8(MotorController_T * p_mc)     { return MotAnalogUser_GetBrake(&p_mc->AnalogUser) >> 8U; }
// static inline bool MotorController_User_GetThrottleSwitch(MotorController_T * p_mc)     {return MotAnalogUser_GetIsThrottleOn(&p_mc->AnalogUser);}
// static inline bool MotorController_User_GetBrakeSwitch(MotorController_T * p_mc)         {return MotAnalogUser_GetIsBrakeOn(&p_mc->AnalogUser);}
// static inline bool MotorController_User_GetForwardSwitch(MotorController_T * p_mc)         {return MotAnalogUser_GetForwardSwitch(&p_mc->AnalogUser);}
// static inline bool MotorController_User_GetReverseSwitch(MotorController_T * p_mc)         {return MotAnalogUser_GetReverseSwitch(&p_mc->AnalogUser);}
// static inline bool MotorController_User_GetOptDinSwitch(MotorController_T * p_user)     { return Debounce_GetState(&p_user->OptDin); }

// static inline uint16_t MotorController_User_GetHeatPcbLimit_DegC(MotorController_T * p_mc, uint16_t scalar)             { return Thermistor_GetShutdown_DegCInt(&p_mc->ThermistorPcb, scalar); }
// static inline uint16_t MotorController_User_GetHeatPcbThreshold_DegC(MotorController_T * p_mc, uint16_t scalar)         { return Thermistor_GetShutdownThreshold_DegCInt(&p_mc->ThermistorPcb, scalar); }
// static inline void MotorController_User_SetHeatPcbLimit_DegC(MotorController_T * p_mc, uint8_t limit_degreesC)             { Thermistor_SetShutdown_DegC(&p_mc->ThermistorPcb, limit_degreesC); }
// static inline void MotorController_User_SetHeatPcbThreshold_DegC(MotorController_T * p_mc, uint8_t threshold_degreesC)     { Thermistor_SetLimitThreshold_DegC(&p_mc->ThermistorPcb, threshold_degreesC); }

// static inline void MotorController_User_SetVSourceLimitUpper_MilliV(MotorController_T * p_mc, uint32_t limit) { VMonitor_SetLimitUpper_MilliV(&p_mc->VMonitorPos, limit); }
// static inline void MotorController_User_SetVSourceLimitLower_MilliV(MotorController_T * p_mc, uint32_t limit) { VMonitor_SetLimitLower_MilliV(&p_mc->VMonitorPos, limit); }
// static inline void MotorController_User_SetVSenseLimitUpper_MilliV(MotorController_T * p_mc, uint32_t limit) { VMonitor_SetLimitUpper_MilliV(&p_mc->VMonitorSense, limit); }
// static inline void MotorController_User_SetVSenseLimitLower_MilliV(MotorController_T * p_mc, uint32_t limit) { VMonitor_SetLimitLower_MilliV(&p_mc->VMonitorSense, limit); }
// static inline void MotorController_User_SetVAccLimitUpper_MilliV(MotorController_T * p_mc, uint32_t limit) { VMonitor_SetLimitUpper_MilliV(&p_mc->VMonitorAcc, limit); }
// static inline void MotorController_User_SetVAccLimitLower_MilliV(MotorController_T * p_mc, uint32_t limit) { VMonitor_SetLimitLower_MilliV(&p_mc->VMonitorAcc, limit); }

//static inline uint32_t MotorController_User_GetVSourceLimitUpper_V(MotorController_T * p_mc)     {return p_mc->Parameters.VSourceLimitUpper_Adcu;}
//static inline uint32_t MotorController_User_GetVSourceLimitLower_V(MotorController_T * p_mc)     {return p_mc->Parameters.VSourceLimitLower_Adcu;}
//static inline uint16_t MotorController_User_GetVSourceLimitUpper_Adcu(MotorController_T * p_mc)         {return p_mc->Parameters.VSourceLimitUpper_Adcu;}
//static inline uint16_t MotorController_User_GetVSourceLimitLower_Adcu(MotorController_T * p_mc)         {return p_mc->Parameters.VSourceLimitLower_Adcu;}