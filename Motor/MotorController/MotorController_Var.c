/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery

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
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "MotorController_Var.h"
#include "../Motor/Sensor/Motor_Sensor.h"

/*
*/
static inline Motor_T * MotorAt(const MotorController_T * p_context, uint8_t motor) { return (motor < p_context->MOTORS.LENGTH) ? MotMotors_ContextAt(&p_context->MOTORS, motor) : NULL; }


/******************************************************************************/
/*
    MotorController Module General
*/
/******************************************************************************/
int32_t MotorController_VarOutput_Get(const MotorController_T * p_context, MotorController_VarOutput_T id)
{
    int32_t value = 0;
    switch (id)
    {
        case MOT_VAR_ZERO:              value = 0;                                                              break;
        case MOT_VAR_MILLIS:            value = Millis();                                                       break;
        case MOT_VAR_MC_STATE:          value = MotorController_User_GetStateId(p_context->P_ACTIVE);           break;
        case MOT_VAR_MC_FAULT_FLAGS:    value = MotorController_User_GetFaultFlags(p_context->P_ACTIVE).Value;  break;
        case MOT_VAR_MC_STATUS_FLAGS:   value = MotorController_User_GetStatusFlags(p_context).Value;           break;
    }
    return value;
}

int32_t MotorController_VarOutput_Debug_Get(const MotorController_T * p_context, MotorController_VarOutput_Debug_T id)
{
    int32_t value = 0;
    Motor_State_T * p_motor = MotorAt(p_context, 0)->P_ACTIVE;
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
void MotorController_VarInput_Set(const MotorController_T * p_context, MotorController_VarInput_T id, int32_t value)
{
    // bool isSuccess = true;
    switch (id)
    {
        case MOT_VAR_USER_MOTOR_SET_POINT:          MotorController_User_SetCmdValue(p_context, value);                     break;
        case MOT_VAR_USER_MOTOR_FEEDBACK_MODE:      MotorController_User_SetFeedbackMode_Cast(p_context, value);            break;
        case MOT_VAR_USER_MOTOR_DIRECTION:          MotorController_User_SetDirection(p_context, math_sign(value));         break;
        case MOT_VAR_USER_MOTOR_PHASE_OUTPUT:       MotorController_User_SetControlState(p_context, (Phase_Output_T)value); break;

        case MOT_VAR_USER_OPT_SPEED_LIMIT_ON_OFF:    MotorController_User_SetOptSpeedLimitOnOff(p_context, value);          break;
        case MOT_VAR_USER_OPT_I_LIMIT_ON_OFF:        MotorController_User_SetOptILimitOnOff(p_context, value);              break;

        case MOT_VAR_USER_RELAY_TOGGLE:                 break;
        case MOT_VAR_USER_METER_TOGGLE:                 break;
    }

    // return (isSuccess == true) ? 0 : -1;
}


/******************************************************************************/
/* Config */
/******************************************************************************/
int32_t MotorController_Config_Get(const MotorController_T * p_context, MotorController_ConfigId_T id)
{
    MotorController_State_T * p_mc = p_context->P_ACTIVE;

    int32_t value = 0;
    switch (id)
    {
        case MOT_VAR_V_SOURCE_REF_VOLTS:            value = p_mc->Config.VSupplyRef;                        break;
        case MOT_VAR_I_LIMIT_LOW_V:                 value = p_mc->Config.VLowILimit_Fract16;                break;
        case MOT_VAR_USER_INIT_MODE:                value = p_mc->Config.InitMode;                          break;
        case MOT_VAR_USER_INPUT_MODE:               value = p_mc->Config.InputMode;                         break;
        // case MOT_VAR_BUZZER_FLAGS_ENABLE:           value = p_mc->Config.BuzzerFlags;                    break;
        case MOT_VAR_OPT_DIN_FUNCTION:              value = p_mc->Config.OptDinMode;                        break;
        case MOT_VAR_OPT_SPEED_LIMIT:               value = p_mc->Config.OptSpeedLimit_Fract16;             break;
        case MOT_VAR_OPT_I_LIMIT:                   value = p_mc->Config.OptILimit_Fract16;                 break;

        // case MOT_DRIVE_CONFIG_THROTTLE_MODE:     value = p_mc->Config.ThrottleMode;          break;
        // case MOT_DRIVE_CONFIG_BRAKE_MODE:        value = p_mc->Config.BrakeMode;             break;
        // case MOT_DRIVE_CONFIG_ZERO_MODE:         value = p_mc->Config.ZeroMode;              break;

        // MOTOR_CONTROLLER_MOT_DRIVE_CONFIG_THROTTLE_MODE
        // MOTOR_CONTROLLER_MOT_DRIVE_CONFIG_BRAKE_MODE
        // MOTOR_CONTROLLER_MOT_DRIVE_CONFIG_ZERO_MODE
    }
    return value;
}

void MotorController_Config_Set(const MotorController_T * p_context, MotorController_ConfigId_T id, int32_t value)
{
    MotorController_State_T * p_mc = p_context->P_ACTIVE;
    switch (id)
    {
        case MOT_VAR_V_SOURCE_REF_VOLTS:            MotorController_User_SetVSupplyRef(p_context, value);                               break;
        case MOT_VAR_I_LIMIT_LOW_V:                 p_mc->Config.VLowILimit_Fract16 = value;                                            break;
        case MOT_VAR_USER_INIT_MODE:                p_mc->Config.InitMode = (MotorController_MainMode_T)value;                          break;
        case MOT_VAR_USER_INPUT_MODE:               MotorController_User_SetInputMode(p_context, (MotorController_InputMode_T)value);   break;
        // case MOT_VAR_BUZZER_FLAGS_ENABLE:           p_mc->Config.BuzzerFlags. = (MotorController_BuzzerFlags_T)value;        break;

        case MOT_VAR_OPT_DIN_FUNCTION:              p_mc->Config.OptDinMode = (MotorController_OptDinMode_T)value;           break;
        case MOT_VAR_OPT_SPEED_LIMIT:               p_mc->Config.OptSpeedLimit_Fract16 = value;                              break;
        case MOT_VAR_OPT_I_LIMIT:                   p_mc->Config.OptILimit_Fract16 = value;                                  break;

        // case MOT_DRIVE_CONFIG_THROTTLE_MODE:                 p_mc->Config.ThrottleMode = (MotDrive_ThrottleMode_T)value;       break;
        // case MOT_DRIVE_CONFIG_BRAKE_MODE:                    p_mc->Config.BrakeMode = (MotDrive_BrakeMode_T)value;             break;
        // case MOT_DRIVE_CONFIG_ZERO_MODE:               p_mc->Config.ZeroMode = (MotDrive_ZeroMode_T)value;     break;
    }
}


int32_t MotorController_Config_BootRef_Get(const MotorController_T * p_context, MotorController_Config_BootRef_T id)
{
    MotorController_State_T * p_mc = p_context->P_ACTIVE;
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

void MotorController_Config_BootRef_Set(const MotorController_T * p_context, MotorController_Config_BootRef_T id, int32_t value)
{
    MotorController_State_T * p_mc = p_context->P_ACTIVE;
    switch (id)
    {
        case MOT_VAR_BOOT_REF_FAST_BOOT:    MotorController_User_SetFastBoot(p_mc, value);     break;
        case MOT_VAR_BOOT_REF_BEEP:         MotorController_User_SetBeep(p_mc, value);         break;
        case MOT_VAR_BOOT_REF_BLINK:        MotorController_User_SetBlink(p_mc, value);        break;
            // case MOT_VAR_BOOT_REF_PROTOCOL_INDEX:    MotorController_User_SetProtocolIndex(p_mc, value);    break;
    }
}

int32_t MotorController_InstancesRef_Get(const MotorController_T * p_context, MotorController_InstanceRef_T nameBase)
{
    switch (nameBase)
    {
        case MOT_VAR_REF_MOTOR_COUNT:               return (int32_t)MotorController_Var_GetMotorCount(p_context);
        case MOT_VAR_REF_V_MONITOR_COUNT:           return (int32_t)MotorController_Var_GetVMonitorCount(p_context);
        case MOT_VAR_REF_THERMISTOR_MOSFETS_COUNT:  return (int32_t)MotorController_Var_GetHeatMosfetCount(p_context);
        case MOT_VAR_REF_PROTOCOL_SOCKET_COUNT:     return (int32_t)MotorController_Var_GetProtocolCount(p_context);
        // case MOT_VAR_REF_CAN_NODE_COUNT:            return (int32_t) ;
        default: return 0;
    }
}


/******************************************************************************/
/*!
    @brief Handle MOTOR_VAR variables
*/
/******************************************************************************/
static int32_t _HandleMotorVar_Get(const MotorController_T * p_mc, MotVarId_T varId)
{
    const Motor_T * p_motor = MotorAt(p_mc, varId.Instance);
    if (p_motor == NULL) return 0;

    switch (varId.NameType)
    {
        case MOT_VAR_ID_TYPE_MOTOR_VAR_USER_OUT:        return _Motor_Var_UserOut_Get(p_motor->P_ACTIVE, varId.NameBase);
        case MOT_VAR_ID_TYPE_MOTOR_VAR_FOC_OUT:         return _Motor_Var_Foc_Get(p_motor->P_ACTIVE, varId.NameBase);
        case MOT_VAR_ID_TYPE_MOTOR_VAR_SENSOR_OUT:      return Motor_Var_Sensor_Get(p_motor->P_ACTIVE, varId.NameBase);
        case MOT_VAR_ID_TYPE_MOTOR_VAR_PID_TUNNING_IO:  return _Motor_Var_PidTuning_Get(p_motor->P_ACTIVE, varId.NameBase);
        // case MOT_VAR_ID_TYPE_MOTOR_VAR_HEAT_MONITOR_OUT: return HeatMonitor_VarId_Get(&p_motor->HEAT_MONITOR_CONTEXT, varId.NameBase);
        case MOT_VAR_ID_TYPE_MOTOR_VAR_USER_IO:         return _Motor_Var_UserIO_Get(p_motor, varId.NameBase);
        case MOT_VAR_ID_TYPE_MOTOR_VAR_CMD_IN:          return 0;
        default: return 0;
    }
}

static MotVarId_Status_T _HandleMotorVar_Set(const MotorController_T * p_mc, MotVarId_T varId, int32_t value)
{
    Motor_T * p_motor = MotorAt(p_mc, varId.Instance);
    MotVarId_Status_T status = MOT_VAR_STATUS_OK;

    if (p_motor == NULL) return MOT_VAR_STATUS_ERROR;

    switch (varId.NameType)
    {
        case MOT_VAR_ID_TYPE_MOTOR_VAR_USER_OUT:
        case MOT_VAR_ID_TYPE_MOTOR_VAR_FOC_OUT:
        case MOT_VAR_ID_TYPE_MOTOR_VAR_SENSOR_OUT: status = MOT_VAR_STATUS_ERROR_READ_ONLY; break;

        /* Wrap with enable check */
        case MOT_VAR_ID_TYPE_MOTOR_VAR_PID_TUNNING_IO:  VarAccess_SetAt(&p_motor->VAR_ACCESS.IO_PID_TUNNING, varId.NameBase, value);    break;
        case MOT_VAR_ID_TYPE_MOTOR_VAR_USER_IO:         VarAccess_SetAt(&p_motor->VAR_ACCESS.IO, varId.NameBase, value);                break;
        case MOT_VAR_ID_TYPE_MOTOR_VAR_CMD_IN:          VarAccess_SetAt(&p_motor->VAR_ACCESS.IN, varId.NameBase, value);                break;
        default: status = MOT_VAR_STATUS_ERROR; break;
    }

    return status;
}


/******************************************************************************/
/*!
    @brief Handle MOTOR_CONFIG variables
*/
/******************************************************************************/
static int32_t _HandleMotorConfig_Get(const MotorController_T * p_mc, MotVarId_T varId)
{
    Motor_T * p_motor = MotorAt(p_mc, varId.Instance);
    if (p_motor == NULL) return 0;

    switch (varId.NameType)
    {
        case MOT_VAR_ID_TYPE_MOTOR_CONFIG_CALIBRATION:          return _Motor_VarConfig_Calibration_Get(p_motor->P_ACTIVE, varId.NameBase);
        case MOT_VAR_ID_TYPE_MOTOR_CONFIG_CALIBRATION_ALIAS:    return _Motor_VarConfig_CalibrationAlias_Get(p_motor->P_ACTIVE, varId.NameBase);
        case MOT_VAR_ID_TYPE_MOTOR_CONFIG_ACTUATION:            return _Motor_VarConfig_Actuation_Get(p_motor->P_ACTIVE, varId.NameBase);
        case MOT_VAR_ID_TYPE_MOTOR_CONFIG_PID:                  return _Motor_VarConfig_Pid_Get(p_motor->P_ACTIVE, varId.NameBase);
        case MOT_VAR_ID_TYPE_MOTOR_CONFIG_HEAT_MONITOR:         return HeatMonitor_ConfigId_Get(&p_motor->HEAT_MONITOR_CONTEXT, varId.NameBase);
        case MOT_VAR_ID_TYPE_MOTOR_CONFIG_THERMISTOR:           return HeatMonitor_ConfigId_Thermistor_Get(&p_motor->HEAT_MONITOR_CONTEXT, varId.NameBase);
        case MOT_VAR_ID_TYPE_MOTOR_CONFIG_BOARD_REF:            return Motor_VarRef_Get(varId.NameBase);
        case MOT_VAR_ID_TYPE_MOTOR_CONFIG_CMD: break;
        default: return 0;
    }
}

static MotVarId_Status_T _HandleMotorConfig_Set(const MotorController_T * p_mc, MotVarId_T varId, int32_t value)
{
    // Check if in config state
    // if (!MotorController_IsConfigState(p_mc))
    // {
    //     return MOT_VAR_STATUS_ERROR_RUNNING;
    // }

    Motor_T * p_motor = MotorAt(p_mc, varId.Instance);
    if (p_motor == NULL) return MOT_VAR_STATUS_ERROR_INVALID_ID;

    /* handler check */
    // if (!Motor_StateMachine_IsConfig(p_motor)) { return MOT_VAR_STATUS_ERROR_RUNNING; }

    switch (varId.NameType)
    {
        case MOT_VAR_ID_TYPE_MOTOR_CONFIG_CALIBRATION:      _Motor_VarConfig_Calibration_Set(p_motor->P_ACTIVE, varId.NameBase, value);                     break;
        case MOT_VAR_ID_TYPE_MOTOR_CONFIG_ACTUATION:        _Motor_VarConfig_Actuation_Set(p_motor->P_ACTIVE, varId.NameBase, value);                       break;
        case MOT_VAR_ID_TYPE_MOTOR_CONFIG_PID:              _Motor_VarConfig_Pid_Set(p_motor->P_ACTIVE, varId.NameBase, value);                             break;
        case MOT_VAR_ID_TYPE_MOTOR_CONFIG_HEAT_MONITOR:     HeatMonitor_ConfigId_Set(&p_motor->HEAT_MONITOR_CONTEXT, varId.NameBase, value);                break;
        case MOT_VAR_ID_TYPE_MOTOR_CONFIG_THERMISTOR:       HeatMonitor_ConfigId_Thermistor_Set(&p_motor->HEAT_MONITOR_CONTEXT, varId.NameBase, value);     break;
        case MOT_VAR_ID_TYPE_MOTOR_CONFIG_CMD:              _Motor_VarConfig_Cmd_Call(p_motor, varId.NameBase, value);                                      break;

        case MOT_VAR_ID_TYPE_MOTOR_CONFIG_CALIBRATION_ALIAS: break;
        case MOT_VAR_ID_TYPE_MOTOR_CONFIG_BOARD_REF: return MOT_VAR_STATUS_ERROR_READ_ONLY; // Board ref is read-only

        default: return MOT_VAR_STATUS_ERROR_INVALID_ID;
    }

    return MOT_VAR_STATUS_OK;
}

/******************************************************************************/
/*!
    @brief Handle MOTOR_SENSOR variables
*/
/******************************************************************************/
static int32_t _HandleMotorSensor_Get(const MotorController_T * p_mc, MotVarId_T varId)
{
    Motor_T * p_motor = MotorAt(p_mc, varId.Instance);
    if (p_motor == NULL) return 0;

    switch (varId.NameType)
    {
        // case MOT_VAR_ID_TYPE_MOTOR_HALL_STATE:     return Motor_Hall_Var_Get(p_motor, varId.NameBase);
        case MOT_VAR_ID_TYPE_MOTOR_HALL_CONFIG:    return Motor_Hall_Config_Get(p_motor, varId.NameBase);
        // case MOT_VAR_ID_TYPE_MOTOR_ENCODER_STATE:  return Motor_Encoder_State_Get(p_motor, varId.NameBase);
        case MOT_VAR_ID_TYPE_MOTOR_ENCODER_CONFIG: return Motor_Encoder_Config_Get(p_motor, varId.NameBase);
        default: return 0;
    }
}

static MotVarId_Status_T _HandleMotorSensor_Set(const MotorController_T * p_mc, MotVarId_T varId, int32_t value)
{
    Motor_T * p_motor = MotorAt(p_mc, varId.Instance);
    if (p_motor == NULL) return MOT_VAR_STATUS_ERROR_INVALID_ID;

    MotVarId_Status_T status = MOT_VAR_STATUS_OK;

    switch (varId.NameType)
    {
        // case MOT_VAR_ID_TYPE_MOTOR_HALL_STATE:    break;
        // case MOT_VAR_ID_TYPE_MOTOR_ENCODER_STATE: break;
        case MOT_VAR_ID_TYPE_MOTOR_HALL_CONFIG:         Motor_Hall_Config_Set(p_motor, varId.NameBase, value);      break;
        case MOT_VAR_ID_TYPE_MOTOR_ENCODER_CONFIG:      Motor_Encoder_Config_Set(p_motor, varId.NameBase, value);   break;
        default: return MOT_VAR_STATUS_ERROR_INVALID_ID;
    }

    return status;
}



/******************************************************************************/
/*
    System Service Handler
*/
/******************************************************************************/
static inline Protocol_T * ProtocolAt(const MotorController_T * p_context, uint8_t protocolIndex)
{
    return (protocolIndex < p_context->PROTOCOL_COUNT) ? &p_context->P_PROTOCOLS[protocolIndex] : NULL;
}

static int32_t _HandleSystemService_Get(const MotorController_T * p_mc, MotVarId_T varId)
{
    switch ((MotVarId_Type_SystemService_T)varId.NameType)
    {
        case MOT_VAR_ID_TYPE_GENERAL_VAR_OUT:       return MotorController_VarOutput_Get(p_mc, varId.NameBase);
        case MOT_VAR_ID_TYPE_GENERAL_CONFIG:        return MotorController_Config_Get(p_mc, varId.NameBase);
        case MOT_VAR_ID_TYPE_BOOT_REF_CONFIG:       return MotorController_Config_BootRef_Get(p_mc, varId.NameBase);

        case MOT_VAR_ID_TYPE_ANALOG_USER_VAR_OUT:   return MotAnalogUser_VarId_Get(&p_mc->ANALOG_USER, varId.NameBase);
        case MOT_VAR_ID_TYPE_ANALOG_USER_CONFIG:    return MotAnalogUser_ConfigId_Get(&p_mc->ANALOG_USER, varId.NameBase);

        case MOT_VAR_ID_TYPE_PROTOCOL_CONFIG:       return Protocol_ConfigId_Get(ProtocolAt(p_mc, varId.Instance), varId.NameBase);
        // case MOT_VAR_ID_TYPE_CAN_BUS_CONFIG:        return CanBus_Config_Get(p_mc, varId.NameBase);

        case MOT_VAR_ID_TYPE_INSTANCES_REF:         return MotorController_InstancesRef_Get(p_mc, varId.NameBase);
        case MOT_VAR_ID_TYPE_DEBUG:                 return MotorController_VarOutput_Debug_Get(p_mc, varId.NameBase);
        default: return 0;
    }
}

static MotVarId_Status_T _HandleSystemService_Set(const MotorController_T * p_mc, MotVarId_T varId, int32_t value)
{
    switch (varId.NameType)
    {
        case MOT_VAR_ID_TYPE_GENERAL_VAR_OUT:       return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_ID_TYPE_ANALOG_USER_VAR_OUT:   return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_ID_TYPE_INSTANCES_REF:         return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_ID_TYPE_DEBUG:                 return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_ID_TYPE_GENERAL_CONFIG:        MotorController_Config_Set(p_mc, varId.NameBase, value);                            break;
        case MOT_VAR_ID_TYPE_BOOT_REF_CONFIG:       MotorController_Config_BootRef_Set(p_mc, varId.NameBase, value);                    break;
        case MOT_VAR_ID_TYPE_ANALOG_USER_CONFIG:    MotAnalogUser_ConfigId_Set(&p_mc->ANALOG_USER, varId.NameBase, value);              break;
        case MOT_VAR_ID_TYPE_PROTOCOL_CONFIG:       Protocol_ConfigId_Set(ProtocolAt(p_mc, varId.Instance), varId.NameBase, value);     break;
        // case MOT_VAR_ID_TYPE_CAN_BUS_CONFIG:        _HandleSystemService_CanBusConfig_Set(p_mc, varId.NameBase, value);
        default: return MOT_VAR_STATUS_ERROR;
    }

    return MOT_VAR_STATUS_OK;
}

/******************************************************************************/
/*
    Monitor Handlers
*/
/******************************************************************************/
static inline HeatMonitor_Context_T * MosfetsHeatContextAt(const MotorController_T * p_context, uint8_t index) { return HeatMonitor_Group_GetInstance(&p_context->HEAT_MOSFETS, index); }
static inline HeatMonitor_T * MosfetsHeatMonitorAt(const MotorController_T * p_context, uint8_t index) { return HeatMonitor_Group_GetMonitor(&p_context->HEAT_MOSFETS, index); }
static inline Thermistor_T * MosfetsThermistorAt(const MotorController_T * p_context, uint8_t index) { return HeatMonitor_Group_GetThermistor(&p_context->HEAT_MOSFETS, index); }

static inline VMonitor_Context_T * VMonitorContextAt(const MotorController_T * p_context, uint8_t index)
{
    const VMonitor_Context_T * p_vMonitor;
    switch (index)
    {
        case MOT_VAR_ID_V_MONITOR_SOURCE: p_vMonitor = &p_context->V_SOURCE; break;
        case MOT_VAR_ID_V_MONITOR_ACCS: p_vMonitor = &p_context->V_ACCESSORIES; break;
        case MOT_VAR_ID_V_MONITOR_ANALOG: p_vMonitor = &p_context->V_ANALOG; break;
        default: p_vMonitor = NULL; break;
    }
    return (VMonitor_Context_T *)p_vMonitor;
}

static inline VMonitor_T * VMonitorAt(const MotorController_T * p_context, uint8_t index) { return VMonitor_GetState(VMonitorContextAt(p_context, index)); }
static inline VDivider_T * VMonitorRefAt(const MotorController_T * p_context, uint8_t index) { return VMonitor_GetVDivider(VMonitorContextAt(p_context, index)); }


/******************************************************************************/
/*
    Monitor Handlers
*/
/******************************************************************************/
static int32_t _HandleMonitor_Get(const MotorController_T * p_mc, MotVarId_T varId)
{
    switch (varId.NameType)
    {
        case MOT_VAR_ID_TYPE_V_MONITOR_INSTANCE_STATE:          return RangeMonitor_VarId_Get(VMonitorAt(p_mc, varId.Instance), varId.NameBase);
        case MOT_VAR_ID_TYPE_V_MONITOR_INSTANCE_CONFIG:         return RangeMonitor_ConfigId_Get(VMonitorAt(p_mc, varId.Instance), varId.NameBase);
        case MOT_VAR_ID_TYPE_V_MONITOR_INSTANCE_VDIVIDER_REF:   return VDivider_RefId_Get(VMonitorRefAt(p_mc, varId.Instance), varId.NameBase);

        case MOT_VAR_ID_TYPE_HEAT_MONITOR_PCB_STATE:            return Monitor_VarId_Get(p_mc->HEAT_PCB.P_STATE, varId.NameBase);
        case MOT_VAR_ID_TYPE_HEAT_MONITOR_PCB_CONFIG:           return Monitor_ConfigId_Get(p_mc->HEAT_PCB.P_STATE, varId.NameBase);
        case MOT_VAR_ID_TYPE_HEAT_MONITOR_PCB_THERMISTOR_REF:   return Thermistor_ConfigId_Get(&p_mc->HEAT_PCB.THERMISTOR, varId.NameBase);

        case MOT_VAR_ID_TYPE_HEAT_MONITOR_MOSFETS_STATE:                    return HeatMonitor_Group_MonitorVar_Get(&p_mc->HEAT_MOSFETS, varId.NameBase);
        case MOT_VAR_ID_TYPE_HEAT_MONITOR_MOSFETS_CONFIG:                   return HeatMonitor_Group_MonitorConfig_Get(&p_mc->HEAT_MOSFETS, varId.NameBase);
        case MOT_VAR_ID_TYPE_HEAT_MONITOR_MOSFETS_INSTANCE_STATE:           return Monitor_VarId_Get(MosfetsHeatMonitorAt(p_mc, varId.Instance), varId.NameBase);
        case MOT_VAR_ID_TYPE_HEAT_MONITOR_MOSFETS_INSTANCE_THERMISTOR_REF:  return Thermistor_ConfigId_Get(MosfetsThermistorAt(p_mc, varId.Instance), varId.NameBase);
        default: return 0;
    }
}

static MotVarId_Status_T _HandleMonitor_Set(const MotorController_T * p_mc, MotVarId_T varId, int32_t value)
{
    if (!MotorController_User_IsConfigState(p_mc)) return MOT_VAR_STATUS_ERROR_RUNNING;

    switch (varId.NameType)
    {
        case MOT_VAR_ID_TYPE_V_MONITOR_INSTANCE_STATE:                      return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_ID_TYPE_HEAT_MONITOR_PCB_STATE:                        return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_ID_TYPE_HEAT_MONITOR_MOSFETS_STATE:                    return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_ID_TYPE_HEAT_MONITOR_MOSFETS_INSTANCE_STATE:           return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_ID_TYPE_HEAT_MONITOR_MOSFETS_INSTANCE_THERMISTOR_REF:  return MOT_VAR_STATUS_ERROR_READ_ONLY;

        case MOT_VAR_ID_TYPE_V_MONITOR_INSTANCE_CONFIG:     RangeMonitor_ConfigId_Set(VMonitorAt(p_mc, varId.Instance), varId.NameBase, value); break;
        case MOT_VAR_ID_TYPE_HEAT_MONITOR_PCB_CONFIG:       Monitor_ConfigId_Set(p_mc->HEAT_PCB.P_STATE, varId.NameBase, value);                break;
        case MOT_VAR_ID_TYPE_HEAT_MONITOR_MOSFETS_CONFIG:   HeatMonitor_Group_MonitorConfig_Set(&p_mc->HEAT_MOSFETS, varId.NameBase, value);    break;
        default: return MOT_VAR_STATUS_ERROR;
    }

    return MOT_VAR_STATUS_OK;
}

/******************************************************************************/
/*
    Command Handlers
*/
/******************************************************************************/
static MotVarId_Status_T _HandleInput_Set(const MotorController_T * p_mc, MotVarId_T varId, int32_t value)
{
    switch (varId.NameType)
    {
        //input rename
        case MOT_VAR_ID_TYPE_INPUT_USER:        MotorController_VarInput_Set(p_mc, varId.NameBase, value); break;
        case MOT_VAR_ID_TYPE_INPUT_MOT_DRIVE:   MotDrive_VarId_Set(&p_mc->MOT_DRIVE, varId.NameBase, value); break;
        default: return MOT_VAR_STATUS_ERROR;
    }

    return MOT_VAR_STATUS_OK;
}

/******************************************************************************/
/*
    System Command Handlers
*/
/******************************************************************************/
// static MotVarId_Status_T _HandleSystemCommand_Set(const MotorController_T * p_mc, MotVarId_T varId, int32_t value)
// {
//     switch (varId.NameType)
//     {
//         case MOT_VAR_ID_TYPE_COMMAND_NVM:           return _HandleSystemCommand_Nvm_Set(p_mc, varId.NameBase, value);
//         case MOT_VAR_ID_TYPE_COMMAND_CALIBRATION:   return _HandleSystemCommand_Calibration_Set(p_mc, varId.NameBase, value);
//         case MOT_VAR_ID_TYPE_COMMAND_BUZZER:        return _HandleSystemCommand_Buzzer_Set(p_mc, varId.NameBase, value);
//         case MOT_VAR_ID_TYPE_COMMAND_CONFIG:        return _HandleSystemCommand_Config_Set(p_mc, varId.NameBase, value);
//         default: return MOT_VAR_STATUS_ERROR;
//     }
// }

/******************************************************************************/
/*
    Main Dispatcher Functions - Public Interface
*/
/******************************************************************************/
/******************************************************************************/
/*!
    @brief Get variable value by ID
*/
/******************************************************************************/
int32_t MotorController_Var_Get(const MotorController_T * p_mc, MotVarId_T varId)
{
    switch ((MotVarId_HandlerType_T)varId.HandlerType)
    {
        case MOT_VAR_ID_HANDLER_TYPE_MOTOR_VAR:             return _HandleMotorVar_Get(p_mc, varId);
        case MOT_VAR_ID_HANDLER_TYPE_MOTOR_CONFIG:          return _HandleMotorConfig_Get(p_mc, varId);
        case MOT_VAR_ID_HANDLER_TYPE_MOTOR_SENSOR:          return _HandleMotorSensor_Get(p_mc, varId);
        case MOT_VAR_ID_HANDLER_TYPE_SYSTEM_SERVICE:        return _HandleSystemService_Get(p_mc, varId);
        case MOT_VAR_ID_HANDLER_TYPE_MONITOR:               return _HandleMonitor_Get(p_mc, varId);
        case MOT_VAR_ID_HANDLER_TYPE_COMMAND:               return 0;
        case MOT_VAR_ID_HANDLER_TYPE_SYSTEM_COMMAND:        return 0;
        default:            return 0;
    }
}

/******************************************************************************/
/*!
    @brief Set variable value by ID
*/
/******************************************************************************/
MotVarId_Status_T MotorController_Var_Set(const MotorController_T * p_mc, MotVarId_T varId, int32_t value)
{
    // Access control check using NameType pattern
    // if (MotVarId_IsReadOnly(varId)) { return MOT_VAR_STATUS_ERROR_READ_ONLY; }

    // // Check protocol control state for non-command variables
    // if (!MotorController_IsProtocolControlEnabled(p_mc) &&
    //     varId.HandlerType != MOT_VAR_ID_HANDLER_TYPE_COMMAND &&
    //     varId.HandlerType != MOT_VAR_ID_HANDLER_TYPE_SYSTEM_COMMAND)
    // {
    //     return MOT_VAR_STATUS_ERROR_PROTOCOL_CONTROL_DISABLED;
    // }

    // if (MotVarId_IsConfig(varId) && !MotorController_IsConfigState(p_mc))
    // {
    //     return MOT_VAR_STATUS_ERROR_RUNNING; // Config variables can only be set in config state
    // }

    switch ((MotVarId_HandlerType_T)varId.HandlerType)
    {
        case MOT_VAR_ID_HANDLER_TYPE_MOTOR_VAR:         return _HandleMotorVar_Set(p_mc, varId, value);
        case MOT_VAR_ID_HANDLER_TYPE_MOTOR_CONFIG:      return _HandleMotorConfig_Set(p_mc, varId, value);
        case MOT_VAR_ID_HANDLER_TYPE_MOTOR_SENSOR:      return _HandleMotorSensor_Set(p_mc, varId, value);
        case MOT_VAR_ID_HANDLER_TYPE_SYSTEM_SERVICE:    return _HandleSystemService_Set(p_mc, varId, value);
        case MOT_VAR_ID_HANDLER_TYPE_MONITOR:           return _HandleMonitor_Set(p_mc, varId, value);
        case MOT_VAR_ID_HANDLER_TYPE_COMMAND:           return _HandleInput_Set(p_mc, varId, value);
        // case MOT_VAR_ID_HANDLER_TYPE_SYSTEM_COMMAND:    return _HandleSystemCommand_Set(p_mc, varId, value);
        default:    return MOT_VAR_STATUS_ERROR;
    }
}