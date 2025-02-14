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
    @file   MotorController_Var.c
    @author FireSourcery
    @version V0

    @brief
*/
/******************************************************************************/
#include "MotorController_Var.h"

/******************************************************************************/
/*!

*/
/******************************************************************************/
int32_t MotorController_VarOutput_Get(const MotorController_T * p_mc, MotorController_VarOutput_T id)
{
    int32_t value = 0;
    switch (id)
    {
        case MOT_VAR_ZERO:                  value = 0;                                                      break;
        case MOT_VAR_MILLIS:                value = Millis();                                               break;
        case MOT_VAR_MC_STATE:              value = MotorController_User_GetStateId(p_mc);                  break;
        // case MOT_VAR_MC_STATE_FLAGS:       value = MotorController_User_GetStateFlags(p_mc).Word;      break;
        case MOT_VAR_MC_FAULT_FLAGS:        value = MotorController_User_GetFaultFlags(p_mc).Value;         break;
        case MOT_VAR_MC_STATUS_FLAGS:       value = MotorController_User_GetStatusFlags(p_mc).Value;        break;

        case MOT_VAR_V_SOURCE:              value = MotorController_Analog_GetVSource(p_mc);                break;
        case MOT_VAR_V_SENSOR:              value = MotorController_Analog_GetVSense(p_mc);                 break;
        case MOT_VAR_V_ACCS:                value = MotorController_Analog_GetVAccs(p_mc);                  break;
        case MOT_VAR_HEAT_PCB:              value = MotorController_Analog_GetHeatPcb(p_mc);                break;
        case MOT_VAR_HEAT_MOSFETS:          value = MotorController_Analog_GetHeatMosfets(p_mc, 0);         break;
        #ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
        case MOT_VAR_BATTERY_CHARGE:        value = MotorController_User_GetBatteryCharge_Percent16(p_mc);  break;
        // case MOT_VAR_HEAT_PCB_DEG_C:        value = MotorController_User_GetHeatPcb_DegC(p_mc, 1U);         break;
        // case MOT_VAR_HEAT_MOSFETS_DEG_C:    value = MotorController_User_GetHeatMosfets_DegC(p_mc, 1U);     break;
        #endif
    }
    return value;
}

int32_t MotorController_VarOutput_AnalogUser_Get(const MotorController_T * p_mc, MotorController_VarOutput_AnalogUser_T id)
{
    int32_t value = 0;
    switch (id)
    {
        // case MOT_VAR_ANALOG_THROTTLE:       value = MotAnalogUser_GetThrottle(&p_mc->AnalogUser);               break;
        // case MOT_VAR_ANALOG_BRAKE:          value = MotAnalogUser_GetBrake(&p_mc->AnalogUser);                  break;
        case MOT_VAR_ANALOG_THROTTLE:           value = MotorController_Analog_GetThrottle(&p_mc->AnalogUser);      break;
        case MOT_VAR_ANALOG_BRAKE:              value = MotorController_Analog_GetBrake(&p_mc->AnalogUser);         break;
        case MOT_VAR_ANALOG_THROTTLE_DIN:       value = p_mc->AnalogUser.ThrottleAIn.EdgePin.DebouncedState;        break;
        case MOT_VAR_ANALOG_BRAKE_DIN:          value = p_mc->AnalogUser.BrakeAIn.EdgePin.DebouncedState;           break;
        case MOT_VAR_ANALOG_BISTATE_BRAKE_DIN:  value = p_mc->AnalogUser.ThrottleAIn.EdgePin.DebouncedState;        break;
        case MOT_VAR_ANALOG_FORWARD_DIN:        value = p_mc->AnalogUser.ForwardPin.DebouncedState;                 break;
        case MOT_VAR_ANALOG_NEUTRAL_DIN:        value = p_mc->AnalogUser.NeutralPin.DebouncedState;                 break;
        case MOT_VAR_ANALOG_REVERSE_DIN:        value = p_mc->AnalogUser.ReversePin.DebouncedState;                 break;
        case MOT_VAR_ANALOG_OPT_DIN:            value = p_mc->OptDin.DebouncedState;                                break;
    }
    return value;
}

int32_t MotorController_VarOutput_Debug_Get(const MotorController_T * p_mc, MotorController_VarOutput_Debug_T id)
{
    int32_t value = 0;
    Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0);
    switch (id)
    {
        case MOT_VAR_DEBUG0: value = p_motor->DebugTime[4];    break;
        case MOT_VAR_DEBUG1: value = 0;    break;
        case MOT_VAR_DEBUG2: value = 0;    break;
        case MOT_VAR_DEBUG3: value = 0;    break;
        case MOT_VAR_DEBUG4: value = 0;    break;
        case MOT_VAR_DEBUG5: value = 0;    break;
        case MOT_VAR_DEBUG6: value = 0;    break;
        case MOT_VAR_DEBUG7: value = 0;    break;
    }
    return value;
}

/******************************************************************************/
/* Inputs disabled on Analog Mode */
/******************************************************************************/
static uint32_t SetDriveCmd(MotorController_T * p_mc, MotorController_VarInput_T id, int32_t value)
{
    bool isSet = (p_mc->Config.InputMode == MOTOR_CONTROLLER_INPUT_MODE_SERIAL);

    if (isSet)
    {
        switch (id)
        {
            case MOT_VAR_THROTTLE:  MotorController_User_SetCmdThrottle(p_mc, value);        break;
            case MOT_VAR_BRAKE:     MotorController_User_SetCmdBrake(p_mc, value);           break;
            // case MOT_VAR_DRIVE_DIRECTION:    isSet = MotorController_User_SetDirection(p_mc, (MotorController_Direction_T)value);    break;
            // case MOT_VAR_USER_CMD:           MotorController_User_SetCmdValue(p_mc, value);                                          break;
        }
    }

    return  (isSet == true) ? 0 : -1;
}

uint32_t MotorController_VarInput_Set(MotorController_T * p_mc, MotorController_VarInput_T id, int32_t value)
{
    bool isSuccess = true;
    // bool isSet = (p_mc->Config.InputMode == MOTOR_CONTROLLER_INPUT_MODE_SERIAL);

    switch (id)
    {
        case MOT_VAR_USER_CMD:                  MotorController_User_SetCmdValue(p_mc, value);              break;
        case MOT_VAR_USER_FEEDBACK_MODE:        MotorController_User_SetFeedbackMode(p_mc, value);          break;
        case MOT_VAR_THROTTLE:                  SetDriveCmd(p_mc, id, value);       break;
        case MOT_VAR_BRAKE:                     SetDriveCmd(p_mc, id, value);       break;
        case MOT_VAR_OPT_SPEED_LIMIT_ON_OFF:    MotorController_User_SetOptSpeedLimitOnOff(p_mc, value);    break;
        case MOT_VAR_OPT_I_LIMIT_ON_OFF:        MotorController_User_SetOptILimitOnOff(p_mc, value);        break;

        case MOT_VAR_RELAY_TOGGLE:                 break;
        case MOT_VAR_METER_TOGGLE:                 break;
    }

    return (isSuccess == true) ? 0 : -1;
}

/******************************************************************************/
/* */
/******************************************************************************/
int32_t MotorController_VarIO_Get(const MotorController_T * p_mc, MotorController_VarIO_T id)
{
    int32_t value = 0;
    switch (id)
    {
        case MOT_VAR_DIRECTION:     value = MotorController_User_GetDirection(p_mc);    break;
        // case MOT_VAR_USER_SET_POINT: value = MotorController_User_GetCmdValue(p_mc);     break;
    }
    return value;
}

uint32_t MotorController_VarIO_Set(MotorController_T * p_mc, MotorController_VarIO_T id, int32_t value)
{
    bool isSet = (p_mc->Config.InputMode == MOTOR_CONTROLLER_INPUT_MODE_SERIAL);

    if (isSet)
    {
        switch (id)
        {
            case MOT_VAR_DIRECTION: MotorController_User_SetDirection(p_mc, (MotorController_Direction_T)value);    break;
            default: break;
        }
    }

    return (isSet == true) ? 0 : -1;
}

/******************************************************************************/
/* Config */
/******************************************************************************/
int32_t MotorController_Config_Get(const MotorController_T * p_mc, MotorController_ConfigId_T id)
{
    int32_t value = 0;
    switch (id)
    {
        case MOT_VAR_V_SOURCE_REF_VOLTS:            value = p_mc->Config.VSourceRef;                        break;
        case MOT_VAR_USER_INIT_MODE:                value = p_mc->Config.InitMode;                          break;
        case MOT_VAR_USER_INPUT_MODE:               value = p_mc->Config.InputMode;                         break;
        case MOT_VAR_THROTTLE_MODE:                 value = p_mc->Config.ThrottleMode;                      break;
        case MOT_VAR_BRAKE_MODE:                    value = p_mc->Config.BrakeMode;                         break;
        case MOT_VAR_DRIVE_ZERO_MODE:               value = p_mc->Config.DriveZeroMode;                     break;
        case MOT_VAR_I_LIMIT_LOW_V:                 value = p_mc->Config.VLowILimit_Fract16;                break;
        case MOT_VAR_OPT_DIN_FUNCTION:              value = p_mc->Config.OptDinMode;                        break;
        case MOT_VAR_OPT_SPEED_LIMIT:               value = p_mc->Config.OptSpeedLimit_Fract16;             break;
        case MOT_VAR_OPT_I_LIMIT:                   value = p_mc->Config.OptILimit_Fract16;                 break;
        // case MOT_VAR_BUZZER_FLAGS_ENABLE:           value = p_mc->Config.BuzzerFlags;                    break;
        // case MOT_VAR_DEFAULT_FEEDBACK_MODE:         value = MotorController_User_GetDefaultFeedbackMode_Cast(p_mc); break;
        #ifdef CONFIG_MOTOR_CONTROLLER_CAN_BUS_ENABLE
        case MOT_VAR_CAN_SERVICES_ID:               value = p_mc->Config.CanServicesId;                     break;
        case MOT_VAR_CAN_IS_ENABLE:                 value = p_mc->Config.CanIsEnable;                       break;
        #endif
        #ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
        case MOT_VAR_BATTERY_ZERO_ADCU:             value = p_mc->Config.BatteryZero_Adcu;                  break;
        case MOT_VAR_BATTERY_FULL_ADCU:             value = p_mc->Config.BatteryFull_Adcu;                  break;
        #endif
    }
    return value;
}

/* Serialization of MotorController_Config */
uint32_t MotorController_Config_Set(MotorController_T * p_mc, MotorController_ConfigId_T id, int32_t value)
{
    switch (id)
    {
        case MOT_VAR_V_SOURCE_REF_VOLTS:            MotorController_User_SetVSourceRef(p_mc, value);                         break;
        case MOT_VAR_USER_INIT_MODE:                p_mc->Config.InitMode = (MotorController_MainMode_T)value;               break;
        case MOT_VAR_USER_INPUT_MODE:               p_mc->Config.InputMode = (MotorController_InputMode_T)value;             break;
        case MOT_VAR_THROTTLE_MODE:                 p_mc->Config.ThrottleMode = (MotorController_ThrottleMode_T)value;       break;
        case MOT_VAR_BRAKE_MODE:                    p_mc->Config.BrakeMode = (MotorController_BrakeMode_T)value;             break;
        case MOT_VAR_DRIVE_ZERO_MODE:               p_mc->Config.DriveZeroMode = (MotorController_DriveZeroMode_T)value;     break;
        case MOT_VAR_I_LIMIT_LOW_V:                 p_mc->Config.VLowILimit_Fract16 = value;                                 break;
        case MOT_VAR_OPT_DIN_FUNCTION:              p_mc->Config.OptDinMode = (MotorController_OptDinMode_T)value;           break;
        case MOT_VAR_OPT_SPEED_LIMIT:               p_mc->Config.OptSpeedLimit_Fract16 = value;                              break;
        case MOT_VAR_OPT_I_LIMIT:                   p_mc->Config.OptILimit_Fract16 = value;                                  break;
            // case MOT_VAR_BUZZER_FLAGS_ENABLE:           p_mc->Config.BuzzerFlags. = (MotorController_BuzzerFlags_T)value;        break;
            // case MOT_VAR_DEFAULT_FEEDBACK_MODE:         MotorController_User_SetDefaultFeedbackMode_Cast(p_mc, value);           break;
        #ifdef CONFIG_MOTOR_CONTROLLER_CAN_BUS_ENABLE
        case MOT_VAR_CAN_SERVICES_ID:               p_mc->Config.CanServicesId = value;                                      break;
        case MOT_VAR_CAN_IS_ENABLE:                 p_mc->Config.CanIsEnable = value;                                        break;
        #endif
        #ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
        case MOT_VAR_BATTERY_ZERO_ADCU:             p_mc->Config.BatteryZero_Adcu = value;                                   break;
        case MOT_VAR_BATTERY_FULL_ADCU:             p_mc->Config.BatteryFull_Adcu = value;                                   break;
        #endif
    }
}

int32_t MotorController_Config_AnalogUser_Get(const MotorController_T * p_mc, MotorController_Config_AnalogUser_T id)
{
    int32_t value = 0;
    switch (id)
    {
        case MOT_VAR_ANALOG_THROTTLE_ZERO_ADCU:             value = p_mc->AnalogUser.Config.ThrottleZero_Adcu;              break;
        case MOT_VAR_ANALOG_THROTTLE_MAX_ADCU:              value = p_mc->AnalogUser.Config.ThrottleMax_Adcu;               break;
        case MOT_VAR_ANALOG_THROTTLE_EDGE_PIN_IS_ENABLE:    value = p_mc->AnalogUser.Config.UseThrottleEdgePin;             break;
        case MOT_VAR_ANALOG_BRAKE_ZERO_ADCU:                value = p_mc->AnalogUser.Config.BrakeZero_Adcu;                 break;
        case MOT_VAR_ANALOG_BRAKE_MAX_ADCU:                 value = p_mc->AnalogUser.Config.BrakeMax_Adcu;                  break;
        case MOT_VAR_ANALOG_BRAKE_EDGE_PIN_IS_ENABLE:       value = p_mc->AnalogUser.Config.UseBrakeEdgePin;                break;
        case MOT_VAR_ANALOG_DIN_BRAKE_VALUE:                value = p_mc->AnalogUser.Config.BistateBrakeValue_Percent16;    break;
        case MOT_VAR_ANALOG_DIN_BRAKE_IS_ENABLE:            value = p_mc->AnalogUser.Config.UseBistateBrakePin;             break;
        // case MOT_VAR_ANALOG_DIRECTION_PINS:                 value = p_mc->AnalogUser.Config.PinsSelect;                     break;
    }
    return value;
}

uint32_t MotorController_Config_AnalogUser_Set(MotorController_T * p_mc, MotorController_Config_AnalogUser_T id, int32_t value)
{
    switch (id)
    {
        case MOT_VAR_ANALOG_THROTTLE_ZERO_ADCU:             p_mc->AnalogUser.Config.ThrottleZero_Adcu = value;              break;
        case MOT_VAR_ANALOG_THROTTLE_MAX_ADCU:              p_mc->AnalogUser.Config.ThrottleMax_Adcu = value;               break;
        case MOT_VAR_ANALOG_THROTTLE_EDGE_PIN_IS_ENABLE:    p_mc->AnalogUser.Config.UseThrottleEdgePin = value;             break;
        case MOT_VAR_ANALOG_BRAKE_ZERO_ADCU:                p_mc->AnalogUser.Config.BrakeZero_Adcu = value;                 break;
        case MOT_VAR_ANALOG_BRAKE_MAX_ADCU:                 p_mc->AnalogUser.Config.BrakeMax_Adcu = value;                  break;
        case MOT_VAR_ANALOG_BRAKE_EDGE_PIN_IS_ENABLE:       p_mc->AnalogUser.Config.UseBrakeEdgePin = value;                break;
        case MOT_VAR_ANALOG_DIN_BRAKE_VALUE:                p_mc->AnalogUser.Config.BistateBrakeValue_Percent16 = value;    break;
        case MOT_VAR_ANALOG_DIN_BRAKE_IS_ENABLE:            p_mc->AnalogUser.Config.UseBistateBrakePin = value;             break;
        // case MOT_VAR_ANALOG_DIRECTION_PINS:                 p_mc->AnalogUser.Config.PinsSelect = value;                     break;
    }

    return 0;
}

int32_t MotorController_Config_BootRef_Get(const MotorController_T * p_mc, MotorController_Config_BootRef_T id)
{
    int32_t value = 0;
    switch (id)
    {
        case MOT_VAR_BOOT_REF_FAST_BOOT:    value = p_mc->BootRef.FastBoot;     break;
        case MOT_VAR_BOOT_REF_BEEP:         value = p_mc->BootRef.Beep;         break;
        case MOT_VAR_BOOT_REF_BLINK:        value = p_mc->BootRef.Blink;        break;
            // case MOT_VAR_BOOT_REF_PROTOCOL_INDEX:    value = p_mc->BootRef.ProtocolIndex;    break;
    }
    return value;
}

uint32_t MotorController_Config_BootRef_Set(MotorController_T * p_mc, MotorController_Config_BootRef_T id, int32_t value)
{
    switch (id)
    {
        case MOT_VAR_BOOT_REF_FAST_BOOT:    MotorController_User_SetFastBoot(p_mc, value);     break;
        case MOT_VAR_BOOT_REF_BEEP:         MotorController_User_SetBeep(p_mc, value);         break;
        case MOT_VAR_BOOT_REF_BLINK:        MotorController_User_SetBlink(p_mc, value);        break;
        // case MOT_VAR_BOOT_REF_PROTOCOL_INDEX:    MotorController_User_SetProtocolIndex(p_mc, value);    break;
    }
}


/******************************************************************************/
/*!
    Combined Id Fields
*/
/******************************************************************************/
/******************************************************************************/
/*!
    RealTime
*/
/******************************************************************************/
static inline int32_t GetRealTime(const MotorController_T * p_mc, MotVarId_T varId)
{
    int32_t value = 0;
    Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, varId.Instance); /* for convience, invalid if varId.NameBase is not Motor type */

    switch((MotVarId_Type_RealTime_T)varId.NameType)
    {
        case MOT_VAR_ID_TYPE_MONITOR_GENERAL:       value = MotorController_VarOutput_Get(p_mc, (MotorController_VarOutput_T)varId.NameBase);                           break;
        case MOT_VAR_ID_TYPE_MONITOR_ANALOG_USER:   value = MotorController_VarOutput_AnalogUser_Get(p_mc, (MotorController_VarOutput_AnalogUser_T)varId.NameBase);     break;
        case MOT_VAR_ID_TYPE_DEBUG:                 value = MotorController_VarOutput_Debug_Get(p_mc, (MotorController_VarOutput_Debug_T)varId.NameBase);               break;
        case MOT_VAR_ID_TYPE_CONTROL_GENERAL:       value = MotorController_VarIO_Get(p_mc, (MotorController_VarIO_T)varId.NameBase);                                   break;
        // case MOT_VAR_ID_TYPE_CMD_SYSTEM:         value = MotorController_User_OutputSystem(p_mc, (MotorController_User_CallId_T)varId.NameBase);         break;
        case MOT_VAR_ID_TYPE_MONITOR_MOTOR:         value = Motor_VarOutput_Get(p_motor, (Motor_VarOuput_T)varId.NameBase);                                 break;
        case MOT_VAR_ID_TYPE_MONITOR_MOTOR_FOC:     value = Motor_VarOutput_Foc_Get(p_motor, (Motor_VarOuput_Foc_T)varId.NameBase);                         break;
        case MOT_VAR_ID_TYPE_MONITOR_MOTOR_SENSOR:  value = Motor_VarOutput_PositionSensor_Get(p_motor, (Motor_VarOutput_PositionSensor_T)varId.NameBase);  break;
        case MOT_VAR_ID_TYPE_CONTROL_MOTOR:         value = Motor_VarIO_Get(p_motor, (Motor_VarIO_T)varId.NameBase);                                        break;

        /* Write-Only */
        case MOT_VAR_ID_TYPE_CMD_GENERAL:           value = 0; break;
        case MOT_VAR_ID_TYPE_CMD_MOTOR:             value = 0; break;
    }

    return value;
}

/*!

*/
/******************************************************************************/
/*!
    Per Motor Instance Input with MotorController StateMachine check
    private helper, check state
*/
/******************************************************************************/
static inline uint32_t MotorInstanceInput_Set(MotorController_T * p_mc, uint8_t motorInstance, Motor_VarInput_T id, int32_t value)
{
    Motor_T * p_motor;

    //  if (MotorController_User_IsActiveState(p_mc)) //or set buffer
    {
        p_motor = MotorController_User_GetPtrMotor(p_mc, motorInstance);
        if (p_motor != NULL) { Motor_VarInput_Set(p_motor, id, value); }
        // MotorController_User_SetMotorValue(p_mc, motorInstance value);
    }

    return 0;
}

static inline uint32_t MotorInstanceIO_Set(MotorController_T * p_mc, uint8_t motorInstance, Motor_VarIO_T id, int32_t value)
{
    Motor_T * p_motor;

    // if (MotorController_User_IsActiveState(p_mc))
    {
        p_motor = MotorController_User_GetPtrMotor(p_mc, motorInstance);
        if (p_motor != NULL) { Motor_VarIO_Set(p_motor, id, value); }
    }

    return 0;
}

static inline MotVarId_Status_T SetRealTime(MotorController_T * p_mc, MotVarId_T varId, int32_t varValue)
{
    MotVarId_Status_T status = MOT_VAR_STATUS_OK;
    bool isSuccess = true;

    switch((MotVarId_Type_RealTime_T)varId.NameType)
    {
        case MOT_VAR_ID_TYPE_MONITOR_GENERAL:       status = MOT_VAR_STATUS_ERROR_READ_ONLY; break;
        case MOT_VAR_ID_TYPE_MONITOR_ANALOG_USER:   status = MOT_VAR_STATUS_ERROR_READ_ONLY; break;
        case MOT_VAR_ID_TYPE_DEBUG:                 status = MOT_VAR_STATUS_ERROR_READ_ONLY; break;
        case MOT_VAR_ID_TYPE_CMD_GENERAL:           status = MotorController_VarInput_Set(p_mc, (MotorController_VarInput_T)varId.NameBase, varValue);      break;
        case MOT_VAR_ID_TYPE_CONTROL_GENERAL:       status = MotorController_VarIO_Set(p_mc, (MotorController_VarIO_T)varId.NameBase, varValue);            break;
        // case MOT_VAR_ID_TYPE_CMD_SYSTEM:         status = MotorController_User_Call(p_mc, (MotorController_User_CallId_T)varId.NameBase, varValue);  break;
        case MOT_VAR_ID_TYPE_MONITOR_MOTOR:         status = MOT_VAR_STATUS_ERROR_READ_ONLY; break;
        case MOT_VAR_ID_TYPE_MONITOR_MOTOR_FOC:     status = MOT_VAR_STATUS_ERROR_READ_ONLY; break;
        case MOT_VAR_ID_TYPE_MONITOR_MOTOR_SENSOR:  status = MOT_VAR_STATUS_ERROR_READ_ONLY; break;
        case MOT_VAR_ID_TYPE_CMD_MOTOR:             status = MotorInstanceInput_Set(p_mc, varId.Instance, (Motor_VarInput_T)varId.NameBase, varValue);  break;
        case MOT_VAR_ID_TYPE_CONTROL_MOTOR:         status = MotorInstanceIO_Set(p_mc, varId.Instance, (Motor_VarIO_T)varId.NameBase, varValue);        break;
    }

    if (isSuccess == false) { status = MOT_VAR_STATUS_ERROR; };

    return status;
}

/******************************************************************************/
/*!
    Config
*/
/******************************************************************************/
static int32_t MotorInstance_GetConfig(const MotorController_T * p_mc, uint8_t motorInstance, MotVarId_Type_Config_T varType, uint8_t idBase)
{
    int32_t value = 0;
    Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, motorInstance);

    if (p_motor != NULL)
    {
        switch (varType)
        {
            case MOT_VAR_ID_TYPE_CONFIG_MOTOR_PRIMARY:      value = Motor_VarConfig_Calibration_Get(p_motor, (Motor_VarConfig_Calibration_T)idBase);    break;
            case MOT_VAR_ID_TYPE_CONFIG_MOTOR_SECONDARY:    value = Motor_VarConfig_Actuation_Get(p_motor, (Motor_VarConfig_Actuation_T)idBase);        break;
            case MOT_VAR_ID_TYPE_CONFIG_MOTOR_HALL:         value = Motor_VarConfig_Hall_Get(p_motor, (Motor_VarConfig_Hall_T)idBase);                  break;
            case MOT_VAR_ID_TYPE_CONFIG_MOTOR_ENCODER:      value = Motor_VarConfig_Encoder_Get(p_motor, (Motor_VarConfig_Encoder_T)idBase);            break;
            case MOT_VAR_ID_TYPE_CONFIG_MOTOR_THERMISTOR:   value = Thermistor_ConfigId_Get(&p_motor->Thermistor, (Thermistor_ConfigId_T)idBase);       break;
            case MOT_VAR_ID_TYPE_CONFIG_MOTOR_PID:          value = Motor_VarConfig_Pid_Get(p_motor, (Motor_VarConfig_Pid_T)idBase);                    break;
            // case MOT_VAR_TYPE_SIN_COS:                   value = Motor_VarConfig_SinCos_Get(p_motor, (Motor_VarConfig_SinCos_T)idBase);              break;
        }
    }

    return value;
}

static int32_t GetConfig(const MotorController_T * p_mc, MotVarId_T varId)
{
    int32_t value = 0;
    const Protocol_T * p_protocol;
    const Thermistor_T * p_thermistor;
    const VMonitor_T * p_vMonitor;

    switch ((MotVarId_Type_Config_T)varId.NameType)
    {
        case MOT_VAR_ID_TYPE_CONFIG_MOTOR_PRIMARY:      value = MotorInstance_GetConfig(p_mc, varId.Instance, (MotVarId_Type_Config_T)varId.NameType, varId.NameBase);  break;
        case MOT_VAR_ID_TYPE_CONFIG_MOTOR_SECONDARY:    value = MotorInstance_GetConfig(p_mc, varId.Instance, (MotVarId_Type_Config_T)varId.NameType, varId.NameBase);  break;
        case MOT_VAR_ID_TYPE_CONFIG_MOTOR_HALL:         value = MotorInstance_GetConfig(p_mc, varId.Instance, (MotVarId_Type_Config_T)varId.NameType, varId.NameBase);  break;
        case MOT_VAR_ID_TYPE_CONFIG_MOTOR_ENCODER:      value = MotorInstance_GetConfig(p_mc, varId.Instance, (MotVarId_Type_Config_T)varId.NameType, varId.NameBase);  break;
        case MOT_VAR_ID_TYPE_CONFIG_MOTOR_THERMISTOR:   value = MotorInstance_GetConfig(p_mc, varId.Instance, (MotVarId_Type_Config_T)varId.NameType, varId.NameBase);  break;
        case MOT_VAR_ID_TYPE_CONFIG_MOTOR_PID:          value = MotorInstance_GetConfig(p_mc, varId.Instance, (MotVarId_Type_Config_T)varId.NameType, varId.NameBase);  break;
        // case MOT_VAR_TYPE_SIN_COS:                   value = MotorInstance_GetConfig(p_mc, varId.Instance, (MotVarId_Type_Config_T)varId.NameType, varId.NameBase);  break;

        case MOT_VAR_ID_TYPE_CONFIG_GENERAL:            value = MotorController_Config_Get(p_mc, (MotorController_ConfigId_T)varId.NameBase);                       break;
        case MOT_VAR_ID_TYPE_CONFIG_ANALOG_USER:        value = MotorController_Config_AnalogUser_Get(p_mc, (MotorController_Config_AnalogUser_T)varId.NameBase);   break;
        case MOT_VAR_ID_TYPE_CONFIG_BOOT_REF:           value = MotorController_Config_BootRef_Get(p_mc, (MotorController_Config_BootRef_T)varId.NameBase);         break;

        case MOT_VAR_ID_TYPE_CONFIG_BOARD_THERMISTOR:
            p_thermistor = MotorController_User_GetPtrThermistor(p_mc, varId.Instance);
            if (p_thermistor != NULL) { value = Thermistor_ConfigId_Get(p_thermistor, (Thermistor_ConfigId_T)varId.NameBase); }
            break;

        case MOT_VAR_ID_TYPE_CONFIG_VMONITOR:
            p_vMonitor = MotorController_User_GetPtrVMonitor(p_mc, varId.Instance);
            if (p_vMonitor != NULL) { value = VMonitor_ConfigId_Get(p_vMonitor, (VMonitor_ConfigId_T)varId.NameBase); }
            break;

        case MOT_VAR_ID_TYPE_CONFIG_PROTOCOL:
            p_protocol = MotorController_User_GetPtrProtocol(p_mc, varId.Instance);
            if (p_protocol != NULL) { value = Protocol_ConfigId_Get(p_protocol, (Protocol_ConfigId_T)varId.NameBase); }
            break;

        default: break;
    }

    return value;
}

static void MotorInstance_SetConfig(MotorController_T * p_mc, uint8_t motorInstance, MotVarId_Type_Config_T varType, uint8_t idBase, int32_t value)
{
    Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, motorInstance);

    if (p_motor != NULL)
    {
        switch (varType)
        {
            case MOT_VAR_ID_TYPE_CONFIG_MOTOR_PRIMARY:      Motor_VarConfig_Calibration_Set(p_motor, (Motor_VarConfig_Calibration_T)idBase, value);    break;
            case MOT_VAR_ID_TYPE_CONFIG_MOTOR_SECONDARY:    Motor_VarConfig_Actuation_Set(p_motor, (Motor_VarConfig_Actuation_T)idBase, value);        break;
            case MOT_VAR_ID_TYPE_CONFIG_MOTOR_HALL:         Motor_VarConfig_Hall_Set(p_motor, (Motor_VarConfig_Hall_T)idBase, value);                  break;
            case MOT_VAR_ID_TYPE_CONFIG_MOTOR_ENCODER:      Motor_VarConfig_Encoder_Set(p_motor, (Motor_VarConfig_Encoder_T)idBase, value);            break;
            case MOT_VAR_ID_TYPE_CONFIG_MOTOR_THERMISTOR:   Thermistor_ConfigId_Set(&p_motor->Thermistor, (Thermistor_ConfigId_T)idBase, value);       break;
            case MOT_VAR_ID_TYPE_CONFIG_MOTOR_PID:          Motor_VarConfig_Pid_Set(p_motor, (Motor_VarConfig_Pid_T)idBase, value);                    break;
            // case MOT_VAR_TYPE_SIN_COS:                      Motor_VarConfig_SinCos_Set(p_motor, (Motor_VarConfig_SinCos_T)idBase, value);           break;
        }
    }
}

static MotVarId_Status_T SetConfig(MotorController_T * p_mc, MotVarId_T varId, int32_t varValue)
{
    MotVarId_Status_T status = MOT_VAR_STATUS_OK;
    bool isSuccess = true;
    Protocol_T * p_protocol;
    Thermistor_T * p_thermistor;
    VMonitor_T * p_vMonitor;

    switch((MotVarId_Type_Config_T)varId.NameType)
    {
        case MOT_VAR_ID_TYPE_CONFIG_MOTOR_PRIMARY:      MotorInstance_SetConfig(p_mc, varId.Instance, (MotVarId_Type_Config_T)varId.NameType, varId.NameBase, varValue); break;
        case MOT_VAR_ID_TYPE_CONFIG_MOTOR_SECONDARY:    MotorInstance_SetConfig(p_mc, varId.Instance, (MotVarId_Type_Config_T)varId.NameType, varId.NameBase, varValue); break;
        case MOT_VAR_ID_TYPE_CONFIG_MOTOR_HALL:         MotorInstance_SetConfig(p_mc, varId.Instance, (MotVarId_Type_Config_T)varId.NameType, varId.NameBase, varValue); break;
        case MOT_VAR_ID_TYPE_CONFIG_MOTOR_ENCODER:      MotorInstance_SetConfig(p_mc, varId.Instance, (MotVarId_Type_Config_T)varId.NameType, varId.NameBase, varValue); break;
        case MOT_VAR_ID_TYPE_CONFIG_MOTOR_THERMISTOR:   MotorInstance_SetConfig(p_mc, varId.Instance, (MotVarId_Type_Config_T)varId.NameType, varId.NameBase, varValue); break;
        case MOT_VAR_ID_TYPE_CONFIG_MOTOR_PID:          MotorInstance_SetConfig(p_mc, varId.Instance, (MotVarId_Type_Config_T)varId.NameType, varId.NameBase, varValue); break;
        // case MOT_VAR_TYPE_SIN_COS:                   MotorInstance_SetConfig(p_mc, varId.Instance, (MotVarId_Type_Config_T)varId.NameType, varId.NameBase, varValue); break;

        case MOT_VAR_ID_TYPE_CONFIG_GENERAL:        MotorController_Config_Set(p_mc, (MotorController_ConfigId_T)varId.NameBase, varValue);                         break;
        case MOT_VAR_ID_TYPE_CONFIG_ANALOG_USER:    MotorController_Config_AnalogUser_Set(p_mc, (MotorController_Config_AnalogUser_T)varId.NameBase, varValue);     break;
        case MOT_VAR_ID_TYPE_CONFIG_BOOT_REF:       MotorController_Config_BootRef_Set(p_mc, (MotorController_Config_BootRef_T)varId.NameBase, varValue);           break;

        case MOT_VAR_ID_TYPE_CONFIG_BOARD_THERMISTOR:
            if ((Thermistor_ConfigId_T)varId.NameBase == THERMISTOR_CONFIG_R0) break;
            if ((Thermistor_ConfigId_T)varId.NameBase == THERMISTOR_CONFIG_T0) break;
            if ((Thermistor_ConfigId_T)varId.NameBase == THERMISTOR_CONFIG_B)  break;
            p_thermistor = MotorController_User_GetPtrThermistor(p_mc, varId.Instance);
            if (p_thermistor != NULL) { Thermistor_ConfigId_Set(p_thermistor, (Thermistor_ConfigId_T)varId.NameBase, varValue); }
            break;

        case MOT_VAR_ID_TYPE_CONFIG_VMONITOR:
            p_vMonitor = MotorController_User_GetPtrVMonitor(p_mc, varId.Instance);
            if (p_vMonitor != NULL) { VMonitor_ConfigId_Set(p_vMonitor, (VMonitor_ConfigId_T)varId.NameBase, varValue); }
            break;

        case MOT_VAR_ID_TYPE_CONFIG_PROTOCOL:
            p_protocol = MotorController_User_GetPtrProtocol(p_mc, varId.Instance);
            if (p_protocol != NULL) { Protocol_ConfigId_Set(p_protocol, (Protocol_ConfigId_T)varId.NameBase, varValue); }
            break;
    }

    return status;
}

/******************************************************************************/
/*!
    Public
*/
/******************************************************************************/
/*
    Var Reg Read Write Interface
    Variables, Single-Argument Functions
*/
int32_t MotorController_Var_Get(const MotorController_T * p_mc, MotVarId_T varId)
{
    int32_t value = 0;
    switch((MotVarId_TypeType_T)varId.NameTypeType)
    {
        case MOT_VAR_ID_TYPE_REAL_TIME: value = GetRealTime(p_mc, varId);   break;
        case MOT_VAR_ID_TYPE_CONFIG:    value = GetConfig(p_mc, varId);     break;
    }
    return value;
}

MotVarId_Status_T MotorController_Var_Set(MotorController_T * p_mc, MotVarId_T varId, int32_t varValue)
{
    MotVarId_Status_T status = MOT_VAR_STATUS_OK;
    switch((MotVarId_TypeType_T)varId.NameTypeType)
    {
        case MOT_VAR_ID_TYPE_REAL_TIME:
            status = SetRealTime(p_mc, varId, varValue);
            break;
        case MOT_VAR_ID_TYPE_CONFIG:
            if ((MotorController_User_IsConfigState(p_mc) == true) || (MotVarId_Type_Config_T)varId.NameType == (MOT_VAR_ID_TYPE_CONFIG_MOTOR_PID))
                { status = SetConfig(p_mc, varId, varValue); }
            else
                { status = MOT_VAR_STATUS_ERROR_RUNNING; }
            break;
        default: break;
    }
    return status;
}

// uint8_t MotorController_Var_GetSize(MotorController_T * p_mc, uint16_t varId)
// {
// }
