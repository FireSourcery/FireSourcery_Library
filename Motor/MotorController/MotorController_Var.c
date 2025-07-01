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


/******************************************************************************/
/*
    MotorController Module General
*/
/******************************************************************************/
/* UserInput */
int MotorController_VarOutput_Get(const MotorController_T * p_context, MotorController_VarOutput_T id)
{
    int value = 0;
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

/* Inputs disabled on Analog Mode */
void MotorController_VarInput_Set(const MotorController_T * p_context, MotorController_VarInput_T id, int value)
{
    // if (p_context->P_ACTIVE->Config.InputMode == MOTOR_CONTROLLER_INPUT_MODE_ANALOG) { return; }

    switch (id)
    {
        case MOT_VAR_USER_MOTOR_SET_POINT:          MotorController_User_SetCmdValue(p_context, (int16_t)value);                break;
        case MOT_VAR_USER_MOTOR_FEEDBACK_MODE:      MotorController_User_SetFeedbackMode_Cast(p_context, value);                break;
        case MOT_VAR_USER_MOTOR_DIRECTION:          MotorController_User_SetDirection(p_context, (sign_t)value);                break;
        case MOT_VAR_USER_MOTOR_PHASE_OUTPUT:       MotorController_User_SetControlState(p_context, (Phase_Output_T)value);     break;

        case MOT_VAR_USER_OPT_SPEED_LIMIT_ON_OFF:    MotorController_User_SetOptSpeedLimitOnOff(p_context, (bool)value);        break;
        case MOT_VAR_USER_OPT_I_LIMIT_ON_OFF:        MotorController_User_SetOptILimitOnOff(p_context, (bool)value);            break;

        case MOT_VAR_USER_RELAY_TOGGLE:                 break;
        case MOT_VAR_USER_METER_TOGGLE:                 break;
    }
}

int MotorController_VarOutput_Debug_Get(const MotorController_T * p_context, MotorController_VarOutput_Debug_T id)
{
    int value = 0;
    // #ifndef NDEBUG
    Motor_State_T * p_motor = MotMotors_ContextAt(&p_context->MOTORS, 0)->P_MOTOR_STATE;
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
    // #endif
    return value;
}

/******************************************************************************/
/*
    Config
*/
/******************************************************************************/
int MotorController_Config_Get(const MotorController_T * p_context, MotorController_ConfigId_T id)
{
    MotorController_State_T * p_state = p_context->P_ACTIVE;

    int value = 0;
    switch (id)
    {
        case MOT_VAR_V_SOURCE_REF_VOLTS:            value = p_state->Config.VSupplyRef;                        break;
        case MOT_VAR_I_LIMIT_LOW_V:                 value = p_state->Config.VLowILimit_Fract16;                break;
        case MOT_VAR_USER_INIT_MODE:                value = p_state->Config.InitMode;                          break;
        case MOT_VAR_USER_INPUT_MODE:               value = p_state->Config.InputMode;                         break;
        // case MOT_VAR_BUZZER_FLAGS_ENABLE:           value = p_state->Config.BuzzerFlags;                    break;
        case MOT_VAR_OPT_DIN_FUNCTION:              value = p_state->Config.OptDinMode;                        break;
        case MOT_VAR_OPT_SPEED_LIMIT:               value = p_state->Config.OptSpeedLimit_Fract16;             break;
        case MOT_VAR_OPT_I_LIMIT:                   value = p_state->Config.OptILimit_Fract16;                 break;

        // case MOT_DRIVE_CONFIG_THROTTLE_MODE:     value = p_state->Config.ThrottleMode;          break;
        // case MOT_DRIVE_CONFIG_BRAKE_MODE:        value = p_state->Config.BrakeMode;             break;
        // case MOT_DRIVE_CONFIG_ZERO_MODE:         value = p_state->Config.ZeroMode;              break;
    }
    return value;
}

void MotorController_Config_Set(const MotorController_T * p_context, MotorController_ConfigId_T id, int value)
{
    MotorController_State_T * p_state = p_context->P_ACTIVE;

    switch (id)
    {
        case MOT_VAR_V_SOURCE_REF_VOLTS:    MotorController_User_SetVSupplyRef(p_context, value);                                   break;
        case MOT_VAR_I_LIMIT_LOW_V:         p_state->Config.VLowILimit_Fract16 = value;                                             break;
        case MOT_VAR_USER_INIT_MODE:        p_state->Config.InitMode = (MotorController_MainMode_T)value;                           break;
        case MOT_VAR_USER_INPUT_MODE:       MotorController_User_SetInputMode(p_context, (MotorController_InputMode_T)value);       break;
        // case MOT_VAR_BUZZER_FLAGS_ENABLE:    p_state->Config.BuzzerFlags. = (MotorController_BuzzerFlags_T)value;        break;
        case MOT_VAR_OPT_DIN_FUNCTION:      p_state->Config.OptDinMode = (MotorController_OptDinMode_T)value;           break;
        case MOT_VAR_OPT_SPEED_LIMIT:       p_state->Config.OptSpeedLimit_Fract16 = value;                              break;
        case MOT_VAR_OPT_I_LIMIT:           p_state->Config.OptILimit_Fract16 = value;                                  break;

        // case MOT_DRIVE_CONFIG_THROTTLE_MODE:                 p_state->Config.ThrottleMode = (MotDrive_ThrottleMode_T)value;       break;
        // case MOT_DRIVE_CONFIG_BRAKE_MODE:                    p_state->Config.BrakeMode = (MotDrive_BrakeMode_T)value;             break;
        // case MOT_DRIVE_CONFIG_ZERO_MODE:               p_state->Config.ZeroMode = (MotDrive_ZeroMode_T)value;     break;
    }
}

int MotorController_Config_BootRef_Get(const MotorController_T * p_context, MotorController_Config_BootRef_T id)
{
    MotorController_State_T * p_state = p_context->P_ACTIVE;
    int value = 0;

    switch (id)
    {
        case MOT_VAR_BOOT_REF_FAST_BOOT:    value = p_state->BootRef.FastBoot;     break;
        case MOT_VAR_BOOT_REF_BEEP:         value = p_state->BootRef.Beep;         break;
        case MOT_VAR_BOOT_REF_BLINK:        value = p_state->BootRef.Blink;        break;
            // case MOT_VAR_BOOT_REF_PROTOCOL_INDEX:    value = p_state->BootRef.ProtocolIndex;    break;
    }
    return value;
}

void MotorController_Config_BootRef_Set(const MotorController_T * p_context, MotorController_Config_BootRef_T id, int value)
{
    MotorController_State_T * p_state = p_context->P_ACTIVE;
    switch (id)
    {
        case MOT_VAR_BOOT_REF_FAST_BOOT:    MotorController_User_SetFastBoot(p_state, value);     break;
        case MOT_VAR_BOOT_REF_BEEP:         MotorController_User_SetBeep(p_state, value);         break;
        case MOT_VAR_BOOT_REF_BLINK:        MotorController_User_SetBlink(p_state, value);        break;
            // case MOT_VAR_BOOT_REF_PROTOCOL_INDEX:    MotorController_User_SetProtocolIndex(p_state, value);    break;
    }
}

int MotorController_InstancesRef_Get(const MotorController_T * p_context, MotorController_InstanceRef_T nameBase)
{
    switch (nameBase)
    {
        case MOT_VAR_REF_MOTOR_COUNT:               return MotorController_Var_GetMotorCount(p_context);
        case MOT_VAR_REF_V_MONITOR_COUNT:           return MotorController_Var_GetVMonitorCount(p_context);
        case MOT_VAR_REF_THERMISTOR_MOSFETS_COUNT:  return MotorController_Var_GetHeatMosfetCount(p_context);
        case MOT_VAR_REF_PROTOCOL_SOCKET_COUNT:     return MotorController_Var_GetProtocolCount(p_context);
        // case MOT_VAR_REF_CAN_NODE_COUNT:            return  ;
        default: return 0;
    }
}


/******************************************************************************/
/*!
    @brief Handlers
*/
/******************************************************************************/
/******************************************************************************/
/*!
    Motor_T Helper
*/
/******************************************************************************/
static inline Motor_T * MotorAt(const MotorController_T * p_context, uint8_t motor)
{
    return (motor < p_context->MOTORS.LENGTH) ? MotMotors_ContextAt(&p_context->MOTORS, motor) : NULL;
}

/******************************************************************************/
/*!
    @brief Handle MOTOR_VAR variables
*/
/******************************************************************************/
static int _HandleMotorVar_Get(const MotorController_T * p_context, MotVarId_T varId)
{
    return Motor_VarType_Get(MotorAt(p_context, varId.Instance), varId.NameType, varId.NameBase);
}

static MotVarId_Status_T _HandleMotorVar_Set(const MotorController_T * p_context, MotVarId_T varId, int value)
{
    if (MotorAt(p_context, varId.Instance) == NULL) return MOT_VAR_STATUS_ERROR;
    // Analog mode does not allow these variables to be set
    if (p_context->P_ACTIVE->Config.InputMode == MOTOR_CONTROLLER_INPUT_MODE_ANALOG) { return MOT_VAR_STATUS_ERROR_READ_ONLY; }
    /* access control handled by Motor_VarType_Set */
    Motor_VarType_Set(MotorAt(p_context, varId.Instance), varId.NameType, varId.NameBase, value);
    return MOT_VAR_STATUS_OK;
}

/******************************************************************************/
/*!
    @brief Handle MOTOR_CONFIG variables
*/
/******************************************************************************/
static int _HandleMotorConfig_Get(const MotorController_T * p_context, MotVarId_T varId)
{
    return Motor_VarType_Config_Get(MotorAt(p_context, varId.Instance), varId.NameType, varId.NameBase);
}

static MotVarId_Status_T _HandleMotorConfig_Set(const MotorController_T * p_context, MotVarId_T varId, int value)
{
    if (MotorAt(p_context, varId.Instance) == NULL) return MOT_VAR_STATUS_ERROR;
    if (!MotorController_User_IsConfigState(p_context)) return MOT_VAR_STATUS_ERROR_RUNNING;

    Motor_VarType_Config_Set(MotorAt(p_context, varId.Instance), varId.NameType, varId.NameBase, value);
    return MOT_VAR_STATUS_OK;
}

/******************************************************************************/
/*!
    @brief Handle MOTOR_SENSOR variables
*/
/******************************************************************************/
static int _HandleRotorSensorState_Get(const MotorController_T * p_context, MotVarId_T varId)
{
    return Motor_VarType_SensorState_Get(MotorAt(p_context, varId.Instance), varId.NameType, varId.NameBase);
}

static MotVarId_Status_T _HandleRotorSensorState_Set(const MotorController_T * p_context, MotVarId_T varId, int value)
{
    return MOT_VAR_STATUS_ERROR_READ_ONLY; // Sensor state variables are read-only
}

static int _HandleRotorSensorConfig_Get(const MotorController_T * p_context, MotVarId_T varId)
{
    return Motor_VarType_SensorConfig_Get(MotorAt(p_context, varId.Instance), varId.NameType, varId.NameBase);
}

static MotVarId_Status_T _HandleRotorSensorConfig_Set(const MotorController_T * p_context, MotVarId_T varId, int value)
{
    if (MotorAt(p_context, varId.Instance) == NULL) return MOT_VAR_STATUS_ERROR;
    if (!MotorController_User_IsConfigState(p_context)) return MOT_VAR_STATUS_ERROR_RUNNING;

    Motor_VarType_SensorConfig_Set(MotorAt(p_context, varId.Instance), varId.NameType, varId.NameBase, value);
    return MOT_VAR_STATUS_OK;
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

static int _HandleSystemService_Get(const MotorController_T * p_context, MotVarId_T varId)
{
    switch ((MotorController_VarType_SystemService_T)varId.NameType)
    {
        case MOT_VAR_TYPE_GENERAL_VAR_OUT:       return MotorController_VarOutput_Get(p_context, varId.NameBase);
        case MOT_VAR_TYPE_GENERAL_CONFIG:        return MotorController_Config_Get(p_context, varId.NameBase);
        case MOT_VAR_TYPE_BOOT_REF_CONFIG:       return MotorController_Config_BootRef_Get(p_context, varId.NameBase);

        case MOT_VAR_TYPE_ANALOG_USER_VAR_OUT:   return MotAnalogUser_VarId_Get(&p_context->ANALOG_USER, varId.NameBase);
        case MOT_VAR_TYPE_ANALOG_USER_CONFIG:    return MotAnalogUser_ConfigId_Get(&p_context->ANALOG_USER, varId.NameBase);

        case MOT_VAR_TYPE_PROTOCOL_CONFIG:       return Protocol_ConfigId_Get(ProtocolAt(p_context, varId.Instance), varId.NameBase);
        case MOT_VAR_TYPE_CAN_BUS_CONFIG:        return 0; //return CanBus_Config_Get(p_mc, varId.NameBase);

        /* read only for now */
        // case MOT_VAR_TYPE_USER_INPUT:            return MotorController_VarInput_Get(p_mc, varId.NameBase);
        case MOT_VAR_TYPE_MOT_DRIVE_CONTROL:     return MotDrive_VarId_Get(&p_context->MOT_DRIVE, varId.NameBase);
        case MOT_VAR_TYPE_MOT_DRIVE_CONFIG:      return MotDrive_ConfigId_Get(p_context->MOT_DRIVE.P_MOT_DRIVE_STATE, varId.NameBase);

        case MOT_VAR_TYPE_INSTANCES_REF:         return MotorController_InstancesRef_Get(p_context, varId.NameBase);
        case MOT_VAR_TYPE_DEBUG:                 return MotorController_VarOutput_Debug_Get(p_context, varId.NameBase);
        default: return 0;
    }
}

static MotVarId_Status_T _HandleSystemService_Set(const MotorController_T * p_context, MotVarId_T varId, int value)
{

    switch ((MotorController_VarType_SystemService_T)varId.NameType)
    {
        case MOT_VAR_TYPE_GENERAL_CONFIG:
        case MOT_VAR_TYPE_BOOT_REF_CONFIG:
        case MOT_VAR_TYPE_ANALOG_USER_CONFIG:
        case MOT_VAR_TYPE_PROTOCOL_CONFIG:
        case MOT_VAR_TYPE_CAN_BUS_CONFIG:
            if (!MotorController_User_IsConfigState(p_context)) return MOT_VAR_STATUS_ERROR_RUNNING;
            break;

        case MOT_VAR_TYPE_USER_INPUT:
        case MOT_VAR_TYPE_MOT_DRIVE_CONTROL:
            if (p_context->P_ACTIVE->Config.InputMode == MOTOR_CONTROLLER_INPUT_MODE_ANALOG) return MOT_VAR_STATUS_ERROR_READ_ONLY;
            break;
        default: break;
    }


    switch ((MotorController_VarType_SystemService_T)varId.NameType)
    {
        case MOT_VAR_TYPE_GENERAL_VAR_OUT:       return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_TYPE_ANALOG_USER_VAR_OUT:   return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_TYPE_INSTANCES_REF:         return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_TYPE_DEBUG:                 return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_TYPE_GENERAL_CONFIG:        MotorController_Config_Set(p_context, varId.NameBase, value);                            break;
        case MOT_VAR_TYPE_BOOT_REF_CONFIG:       MotorController_Config_BootRef_Set(p_context, varId.NameBase, value);                    break;
        case MOT_VAR_TYPE_ANALOG_USER_CONFIG:    MotAnalogUser_ConfigId_Set(&p_context->ANALOG_USER, varId.NameBase, value);              break;
        case MOT_VAR_TYPE_PROTOCOL_CONFIG:       Protocol_ConfigId_Set(ProtocolAt(p_context, varId.Instance), varId.NameBase, value);     break;
        // case MOT_VAR_TYPE_CAN_BUS_CONFIG:     CanBusConfig_Set(p_context, varId.NameBase, value);    break;

        case MOT_VAR_TYPE_USER_INPUT:           MotorController_VarInput_Set(p_context, varId.NameBase, value);     break;
        case MOT_VAR_TYPE_MOT_DRIVE_CONTROL:    MotDrive_VarId_Set(&p_context->MOT_DRIVE, varId.NameBase, value);   break;
        case MOT_VAR_TYPE_MOT_DRIVE_CONFIG:     MotDrive_ConfigId_Set(p_context->MOT_DRIVE.P_MOT_DRIVE_STATE, varId.NameBase, value); break;

        default: return MOT_VAR_STATUS_ERROR_INVALID_ID;
    }

    return MOT_VAR_STATUS_OK;
}

/******************************************************************************/
/*
    Monitor Handlers
*/
/******************************************************************************/
static inline HeatMonitor_Context_T * MosfetsHeatContextAt(const MotorController_T * p_context, uint8_t index) { return HeatMonitor_Group_GetContext(&p_context->HEAT_MOSFETS, index); }

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

/*
    remove
*/
static inline HeatMonitor_T * MosfetsHeatMonitorAt(const MotorController_T * p_context, uint8_t index) { return HeatMonitor_Group_GetMonitor(&p_context->HEAT_MOSFETS, index); }
static inline Thermistor_T * MosfetsThermistorAt(const MotorController_T * p_context, uint8_t index) { return HeatMonitor_Group_GetThermistor(&p_context->HEAT_MOSFETS, index); }

static inline VMonitor_T * VMonitorAt(const MotorController_T * p_context, uint8_t index) { return VMonitor_GetState(VMonitorContextAt(p_context, index)); }
static inline VDivider_T * VMonitorRefAt(const MotorController_T * p_context, uint8_t index) { return VMonitor_GetVDivider(VMonitorContextAt(p_context, index)); }


/******************************************************************************/
/*
    Monitor Handlers
*/
/******************************************************************************/
static int _HandleMonitor_Get(const MotorController_T * p_context, MotVarId_T varId)
{
    switch ((MotorController_VarType_Monitor_T) varId.NameType)
    {
        case MOT_VAR_TYPE_V_MONITOR_INSTANCE_STATE:          return RangeMonitor_VarId_Get(VMonitorAt(p_context, varId.Instance), varId.NameBase);
        case MOT_VAR_TYPE_V_MONITOR_INSTANCE_CONFIG:         return RangeMonitor_ConfigId_Get(VMonitorAt(p_context, varId.Instance), varId.NameBase);
        case MOT_VAR_TYPE_V_MONITOR_INSTANCE_VDIVIDER_REF:   return VDivider_RefId_Get(VMonitorRefAt(p_context, varId.Instance), varId.NameBase);
        // case MOT_VAR_TYPE_V_MONITOR_INSTANCE_STATE:          return VMonitor_VarId_Get(VMonitorContextAt(p_context, varId.Instance), varId.NameBase);

        case MOT_VAR_TYPE_HEAT_MONITOR_PCB_STATE:            return Monitor_VarId_Get(p_context->HEAT_PCB.P_STATE, varId.NameBase);
        case MOT_VAR_TYPE_HEAT_MONITOR_PCB_CONFIG:           return Monitor_ConfigId_Get(p_context->HEAT_PCB.P_STATE, varId.NameBase);
        case MOT_VAR_TYPE_HEAT_MONITOR_PCB_THERMISTOR_REF:   return Thermistor_ConfigId_Get(&p_context->HEAT_PCB.THERMISTOR, varId.NameBase);

        case MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_STATE:                    return HeatMonitor_Group_VarId_Get(&p_context->HEAT_MOSFETS, varId.NameBase);
        case MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_CONFIG:                   return HeatMonitor_Group_ConfigId_Get(&p_context->HEAT_MOSFETS, varId.NameBase);
        case MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_INSTANCE_STATE:           return Monitor_VarId_Get(MosfetsHeatMonitorAt(p_context, varId.Instance), varId.NameBase);
        case MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_INSTANCE_THERMISTOR_REF:  return Thermistor_ConfigId_Get(MosfetsThermistorAt(p_context, varId.Instance), varId.NameBase);
        default: return 0;
    }
}

static MotVarId_Status_T _HandleMonitor_Set(const MotorController_T * p_context, MotVarId_T varId, int value)
{
    if (!MotorController_User_IsConfigState(p_context)) return MOT_VAR_STATUS_ERROR_RUNNING;

    switch (varId.NameType)
    {
        case MOT_VAR_TYPE_V_MONITOR_INSTANCE_STATE:                      return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_TYPE_HEAT_MONITOR_PCB_STATE:                        return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_STATE:                    return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_INSTANCE_STATE:           return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_INSTANCE_THERMISTOR_REF:  return MOT_VAR_STATUS_ERROR_READ_ONLY;

        case MOT_VAR_TYPE_V_MONITOR_INSTANCE_CONFIG:     RangeMonitor_ConfigId_Set(VMonitorAt(p_context, varId.Instance), varId.NameBase, value); break;
        case MOT_VAR_TYPE_HEAT_MONITOR_PCB_CONFIG:       Monitor_ConfigId_Set(p_context->HEAT_PCB.P_STATE, varId.NameBase, value);                break;
        case MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_CONFIG:   HeatMonitor_Group_ConfigId_Set(&p_context->HEAT_MOSFETS, varId.NameBase, value);    break;
        default: return MOT_VAR_STATUS_ERROR;
    }

    return MOT_VAR_STATUS_OK;
}

/******************************************************************************/
/*
    System Command Handlers
*/
/******************************************************************************/
// static MotVarId_Status_T _HandleSystemCommand_Set(const MotorController_T * p_context, MotVarId_T varId, int value)
// {
//     switch (varId.NameType)
//     {
//         case MOT_VAR_TYPE_COMMAND_NVM:           return _HandleSystemCommand_Nvm_Set(p_context, varId.NameBase, value);
//         case MOT_VAR_TYPE_COMMAND_CALIBRATION:   return _HandleSystemCommand_Calibration_Set(p_context, varId.NameBase, value);
//         case MOT_VAR_TYPE_COMMAND_BUZZER:        return _HandleSystemCommand_Buzzer_Set(p_context, varId.NameBase, value);
//         case MOT_VAR_TYPE_COMMAND_CONFIG:        return _HandleSystemCommand_Config_Set(p_context, varId.NameBase, value);
//         default: return MOT_VAR_STATUS_ERROR;
//     }

//     MotorController_User_Call(p_context, varId.NameType, varId.NameBase, value);
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
int MotorController_Var_Get(const MotorController_T * p_context, MotVarId_T varId)
{
    switch ((MotVarId_HandlerType_T)varId.HandlerType)
    {
        case MOT_VAR_ID_HANDLER_TYPE_MOTOR_VAR:             return _HandleMotorVar_Get(p_context, varId);
        case MOT_VAR_ID_HANDLER_TYPE_MOTOR_CONFIG:          return _HandleMotorConfig_Get(p_context, varId);
        case MOT_VAR_ID_HANDLER_TYPE_MOTOR_SENSOR_STATE:    return _HandleRotorSensorState_Get(p_context, varId);
        case MOT_VAR_ID_HANDLER_TYPE_MOTOR_SENSOR_CONFIG:   return _HandleRotorSensorConfig_Get(p_context, varId);
        case MOT_VAR_ID_HANDLER_TYPE_SYSTEM_SERVICE:        return _HandleSystemService_Get(p_context, varId);
        case MOT_VAR_ID_HANDLER_TYPE_MONITOR:               return _HandleMonitor_Get(p_context, varId);
        default: return 0;
    }
}

/******************************************************************************/
/*!
    @brief Set variable value by ID
*/
/******************************************************************************/
MotVarId_Status_T MotorController_Var_Set(const MotorController_T * p_context, MotVarId_T varId, int value)
{
    // Access control check using NameType pattern
    // if (MotVarId_IsReadOnly(varId)) { return MOT_VAR_STATUS_ERROR_READ_ONLY; }

    // // Check protocol control state for non-command variables
    // if (!MotorController_IsProtocolControlEnabled(p_context) &&
    //     varId.HandlerType != MOT_VAR_ID_HANDLER_TYPE_USER_INPUT &&
    //     varId.HandlerType != MOT_VAR_ID_HANDLER_TYPE_SYSTEM_COMMAND)
    // {
    //     return MOT_VAR_STATUS_ERROR_PROTOCOL_CONTROL_DISABLED;
    // }

    // if (MotVarId_IsConfig(varId) && !MotorController_IsConfigState(p_context))
    // {
    //     return MOT_VAR_STATUS_ERROR_RUNNING; // Config variables can only be set in config state
    // }

    switch ((MotVarId_HandlerType_T)varId.HandlerType)
    {
        case MOT_VAR_ID_HANDLER_TYPE_MOTOR_VAR:             return _HandleMotorVar_Set(p_context, varId, value);
        case MOT_VAR_ID_HANDLER_TYPE_MOTOR_CONFIG:          return _HandleMotorConfig_Set(p_context, varId, value);
        case MOT_VAR_ID_HANDLER_TYPE_MOTOR_SENSOR_STATE:    return _HandleRotorSensorState_Set(p_context, varId, value);
        case MOT_VAR_ID_HANDLER_TYPE_MOTOR_SENSOR_CONFIG:   return _HandleRotorSensorConfig_Set(p_context, varId, value);
        case MOT_VAR_ID_HANDLER_TYPE_SYSTEM_SERVICE:        return _HandleSystemService_Set(p_context, varId, value);
        case MOT_VAR_ID_HANDLER_TYPE_MONITOR:               return _HandleMonitor_Set(p_context, varId, value);
        default:    return MOT_VAR_STATUS_ERROR_INVALID_ID;
    }
}