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
int MotorController_Var_Output_Get(const MotorController_T * p_context, MotorController_Var_Output_T id)
{
    int value = 0;
    switch (id)
    {
        case MOT_VAR_ZERO:                  value = 0;                                                                  break;
        case MOT_VAR_MILLIS:                value = Millis();                                                           break;
        case MOT_VAR_MC_STATE:              value = MotorController_GetStateId(p_context->P_MC_STATE);              break;
        case MOT_VAR_MC_SUB_STATE:          value = _MotorController_GetSubStateId(p_context->P_MC_STATE);          break;
        case MOT_VAR_MC_FAULT_FLAGS:        value = MotorController_GetFaultFlags(p_context->P_MC_STATE).Value;     break;
        // case MOT_VAR_MC_STATUS_FLAGS:       value = MotorController_GetStatusFlags(p_context).Value;               break;
        case MOT_VAR_MC_DIRECTION:          value = MotorController_GetDirection(p_context);                        break;
    }
    return value;
}

/* Inputs disabled on Analog Mode */
void MotorController_Var_Input_Set(const MotorController_T * p_context, MotorController_Var_Input_T id, int value)
{
    switch (id)
    {
        case MOT_VAR_USER_GENERAL_SET_POINT:        MotorController_SetCmdValue(p_context, (int16_t)value);                                            break;
        case MOT_VAR_USER_GENERAL_FEEDBACK_MODE:    MotorController_SetFeedbackMode(p_context, (Motor_FeedbackMode_T) { .Value = (uint8_t)value });    break;
        case MOT_VAR_USER_GENERAL_DIRECTION:        MotorController_SetDirection(p_context, (int16_t)value);                                            break;
        case MOT_VAR_USER_GENERAL_PHASE_OUTPUT:     MotorController_SetControlState(p_context, (Phase_Output_T)value);                                 break;

        // case MOT_VAR_USER_OPT_SPEED_LIMIT_ON_OFF:   MotorController_SetOptSpeedLimitOnOff(p_context, (bool)value);        break;
        // case MOT_VAR_USER_OPT_I_LIMIT_ON_OFF:       MotorController_SetOptILimitOnOff(p_context, (bool)value);            break;
        // case MOT_VAR_USER_RELAY_TOGGLE:                 break;
        // case MOT_VAR_USER_METER_TOGGLE:                 break;
    }
}

int MotorController_Var_OutputDebug_Get(const MotorController_T * p_context, MotorController_Var_OutputDebug_T id)
{
    int value = 0;
    // #ifndef NDEBUG
    Motor_State_T * p_motor = Motor_Table_ContextAt(&p_context->MOTORS, 0)->P_MOTOR_STATE;
    switch (id)
    {
        case MOT_VAR_CONTROL_LOOP_PROFILE:  value = p_context->P_MC_STATE->ControlLoopProfile;  break;
        case MOT_VAR_DEBUG0: value = 0;    break;
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
int MotorController_Config_Get(const MotorController_T * p_context, MotorController_Var_Config_T id)
{
    MotorController_State_T * p_state = p_context->P_MC_STATE;
    int value = 0;
    switch (id)
    {
        case MOT_VAR_V_SUPPLY_VOLTS:           value = p_state->Config.VSupplyRef;                        break;
        case MOT_VAR_I_LIMIT_LOW_V:            value = p_state->Config.VLowILimit_Fract16;                break;
        case MOT_VAR_MAIN_MODE:                value = p_state->Config.InitMode;                          break;
        case MOT_VAR_INPUT_MODE:               value = p_state->Config.InputMode;                         break;
        case MOT_VAR_OPT_DIN_FUNCTION:         value = p_state->Config.OptDinMode;                        break;
        case MOT_VAR_OPT_SPEED_LIMIT:          value = p_state->Config.OptSpeedLimit_Fract16;             break;
        case MOT_VAR_OPT_I_LIMIT:              value = p_state->Config.OptILimit_Fract16;                 break;

        case MOT_VAR_BOOT_REF_FAST_BOOT:    value = p_state->BootRef.FastBoot;     break;
        case MOT_VAR_BOOT_REF_BEEP:         value = p_state->BootRef.Beep;         break;
        case MOT_VAR_BOOT_REF_BLINK:        value = p_state->BootRef.Blink;        break;
        // case MOT_VAR_BOOT_REF_PROTOCOL_INDEX:    value = p_state->BootRef.ProtocolIndex;    break;
        // case MOT_VAR_BUZZER_FLAGS_ENABLE:        value = p_state->Config.BuzzerFlags;                       break;
    }
    return value;
}

void MotorController_Config_Set(const MotorController_T * p_context, MotorController_Var_Config_T id, int value)
{
    MotorController_State_T * p_state = p_context->P_MC_STATE;
    switch (id)
    {
        case MOT_VAR_V_SUPPLY_VOLTS:        MotorController_SetVSupplyRef(p_context, value);                                   break;
        case MOT_VAR_I_LIMIT_LOW_V:         p_state->Config.VLowILimit_Fract16 = value;                                             break;
        case MOT_VAR_MAIN_MODE:             p_state->Config.InitMode = (MotorController_MainMode_T)value;                           break;
        case MOT_VAR_INPUT_MODE:            MotorController_SetInputMode(p_context, (MotorController_InputMode_T)value);       break;
        case MOT_VAR_OPT_DIN_FUNCTION:      p_state->Config.OptDinMode = (MotorController_OptDinMode_T)value;           break;
        case MOT_VAR_OPT_SPEED_LIMIT:       p_state->Config.OptSpeedLimit_Fract16 = value;                              break;
        case MOT_VAR_OPT_I_LIMIT:           p_state->Config.OptILimit_Fract16 = value;                                  break;

        case MOT_VAR_BOOT_REF_FAST_BOOT:    MotorController_SetFastBoot(p_state, value);     break;
        case MOT_VAR_BOOT_REF_BEEP:         MotorController_SetBeep(p_state, value);         break;
        case MOT_VAR_BOOT_REF_BLINK:        MotorController_SetBlink(p_state, value);        break;
        // case MOT_VAR_BUZZER_FLAGS_ENABLE:    p_state->Config.BuzzerFlags. = (MotorController_BuzzerFlags_T)value;        break;
    }
}



/******************************************************************************/
/*!
*/
/******************************************************************************/
static inline uint8_t MotorController_Var_GetMotorCount(const MotorController_T * p_context) { return p_context->MOTORS.LENGTH; }
static inline uint8_t MotorController_Var_GetHeatMosfetCount(const MotorController_T * p_context) { return HeatMonitor_Group_GetInstanceCount(&p_context->HEAT_MOSFETS); }
static inline uint8_t MotorController_Var_GetVMonitorCount(const MotorController_T * p_context) { return (p_context->V_SOURCE.P_STATE != NULL) + (p_context->V_ACCESSORIES.P_STATE != NULL) + (p_context->V_ANALOG.P_STATE != NULL); }
static inline uint8_t MotorController_Var_GetProtocolCount(const MotorController_T * p_context) { return p_context->PROTOCOL_COUNT; }

int MotorController_InstancesRef_Get(const MotorController_T * p_context, MotorController_Var_StaticRef_T nameBase)
{
    switch (nameBase)
    {
        case MOT_VAR_REF_MOTOR_COUNT:               return MotorController_Var_GetMotorCount(p_context);
        case MOT_VAR_REF_V_MONITOR_COUNT:           return MotorController_Var_GetVMonitorCount(p_context);
        case MOT_VAR_REF_THERMISTOR_MOSFETS_COUNT:  return MotorController_Var_GetHeatMosfetCount(p_context);
        case MOT_VAR_REF_PROTOCOL_SOCKET_COUNT:     return MotorController_Var_GetProtocolCount(p_context);
        // case MOT_VAR_REF_CAN_MODE_COUNT:            return  ;
        default: return 0;
    }
}

/******************************************************************************/
/*
    [MotVarId] [VarType] Interface
*/
/******************************************************************************/
/******************************************************************************/
/*!
    @brief Handle MOTOR_VAR variables
*/
/******************************************************************************/
static inline Motor_T * MotorAt(const MotorController_T * p_context, uint8_t motor) { return (motor < p_context->MOTORS.LENGTH) ? Motor_Table_ContextAt(&p_context->MOTORS, motor) : NULL; }

static int _HandleMotorVar_Get(const MotorController_T * p_context, MotVarId_T varId) { return Motor_VarType_Get(MotorAt(p_context, varId.Instance), (Motor_VarType_T)varId.InnerType, varId.Base); }

static MotVarId_Status_T _HandleMotorVar_Set(const MotorController_T * p_context, MotVarId_T varId, int value)
{
    // if (MotorAt(p_context, varId.Instance) == NULL) return MOT_VAR_STATUS_ERROR_INVALID_ID;
    if (!Motor_VarType_CheckSet(MotorAt(p_context, varId.Instance), (Motor_VarType_T)varId.InnerType, varId.Base, value)) return MOT_VAR_STATUS_ERROR_READ_ONLY;
    Motor_VarType_Set(MotorAt(p_context, varId.Instance), (Motor_VarType_T)varId.InnerType, varId.Base, value);
    return MOT_VAR_STATUS_OK;
}


/******************************************************************************/
/*
    System Service Handler
*/
/******************************************************************************/
static inline Socket_T * SocketAt(const MotorController_T * p_context, uint8_t protocolIndex)
{
    return (protocolIndex < p_context->PROTOCOL_COUNT) ? &p_context->P_PROTOCOLS[protocolIndex] : NULL;
}

static int _HandleSystem_Get(const MotorController_T * p_context, MotVarId_T varId)
{
    switch ((MotorController_VarType_T)varId.InnerType)
    {
        case MOT_VAR_TYPE_GENERAL_USER_OUT:      return MotorController_Var_Output_Get(p_context, varId.Base);
        case MOT_VAR_TYPE_GENERAL_CONFIG:        return MotorController_Config_Get(p_context, varId.Base);
        case MOT_VAR_TYPE_GENERAL_USER_IN:       return 0; //return MotorController_Var_Input_Get(p_mc, varId.Base);
        // case MOT_VAR_TYPE_GENERAL_USER_CONTROL:  return 0; //return MotorController_Var_Control_Get(p_mc, varId.Base);
        case MOT_VAR_TYPE_GENERAL_STATIC_REF:    return MotorController_InstancesRef_Get(p_context, varId.Base);
        case MOT_VAR_TYPE_GENERAL_DEBUG:         return MotorController_Var_OutputDebug_Get(p_context, varId.Base);

        case MOT_VAR_TYPE_ANALOG_USER_VAR_OUT:   return MotAnalogUser_VarId_Get(&p_context->ANALOG_USER, varId.Base);
        case MOT_VAR_TYPE_ANALOG_USER_CONFIG:    return MotAnalogUser_ConfigId_Get(&p_context->ANALOG_USER, varId.Base);

        case MOT_VAR_TYPE_V_MONITOR_SOURCE_STATE:           return VMonitor_VarId_Get(&p_context->V_SOURCE, varId.Base);
        case MOT_VAR_TYPE_V_MONITOR_SOURCE_CONFIG:          return VMonitor_ConfigId_Get(&p_context->V_SOURCE, varId.Base);
        case MOT_VAR_TYPE_V_MONITOR_SOURCE_VDIVIDER_REF: // override this for now
            switch (varId.Base)
            {
                case VDIVIDER_REF_R1: return Motor_Var_StaticRef_Get(MOTOR_VAR_REF_V_PHASE_R1);
                case VDIVIDER_REF_R2: return Motor_Var_StaticRef_Get(MOTOR_VAR_REF_V_PHASE_R2);
                default: return 0;
            }
        case MOT_VAR_TYPE_V_MONITOR_ACCS_STATE:             return VMonitor_VarId_Get(&p_context->V_ACCESSORIES, varId.Base);
        case MOT_VAR_TYPE_V_MONITOR_ACCS_CONFIG:            return VMonitor_ConfigId_Get(&p_context->V_ACCESSORIES, varId.Base);
        case MOT_VAR_TYPE_V_MONITOR_ACCS_VDIVIDER_REF:      return VMonitor_VDivider_RefId_Get(&p_context->V_ACCESSORIES, varId.Base);
        case MOT_VAR_TYPE_V_MONITOR_ANALOG_STATE:           return VMonitor_VarId_Get(&p_context->V_ANALOG, varId.Base);
        case MOT_VAR_TYPE_V_MONITOR_ANALOG_CONFIG:          return VMonitor_ConfigId_Get(&p_context->V_ANALOG, varId.Base);
        case MOT_VAR_TYPE_V_MONITOR_ANALOG_VDIVIDER_REF:    return VMonitor_VDivider_RefId_Get(&p_context->V_ANALOG, varId.Base);

        case MOT_VAR_TYPE_HEAT_MONITOR_PCB_STATE:                           return HeatMonitor_VarId_Get(&p_context->HEAT_PCB, varId.Base);
        case MOT_VAR_TYPE_HEAT_MONITOR_PCB_CONFIG:                          return HeatMonitor_ConfigId_Get(&p_context->HEAT_PCB, varId.Base);
        case MOT_VAR_TYPE_HEAT_MONITOR_PCB_THERMISTOR_REF:                  return HeatMonitor_Thermistor_ConfigId_Get(&p_context->HEAT_PCB, varId.Base);
        case MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_STATE:                       return HeatMonitor_Group_VarId_Get(&p_context->HEAT_MOSFETS, varId.Base);
        case MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_CONFIG:                      return HeatMonitor_Group_ConfigId_Get(&p_context->HEAT_MOSFETS, varId.Base);
        case MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_INSTANCE_STATE:              return HeatMonitor_GroupInstance_VarId_Get(&p_context->HEAT_MOSFETS, varId.Instance, varId.Base);
        case MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_INSTANCE_THERMISTOR_REF:     return HeatMonitor_GroupInstance_ThermistorConfigId_Get(&p_context->HEAT_MOSFETS, varId.Instance, varId.Base);

        case MOT_VAR_TYPE_SOCKET_STATE:             return 0;
        case MOT_VAR_TYPE_CAN_BUS_STATE:            return 0;
        case MOT_VAR_TYPE_SOCKET_CONFIG:         return Socket_ConfigId_Get(SocketAt(p_context, varId.Instance), varId.Base);
        case MOT_VAR_TYPE_CAN_BUS_CONFIG:        return 0; //return CanBus_Config_Get(p_mc, varId.Base);

        case MOT_VAR_TYPE_VEHICLE_CONTROL:     return Vehicle_VarId_Get(&p_context->VEHICLE, varId.Base);
        case MOT_VAR_TYPE_VEHICLE_CONFIG:      return Vehicle_ConfigId_Get(p_context->VEHICLE.P_VEHICLE_STATE, varId.Base);
        default: return 0;
    }
}

static MotVarId_Status_T _HandleSystem_Set(const MotorController_T * p_context, MotVarId_T varId, int value)
{
    switch ((MotorController_VarType_T)varId.InnerType)
    {
        case MOT_VAR_TYPE_GENERAL_USER_OUT:         return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_TYPE_ANALOG_USER_VAR_OUT:      return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_TYPE_GENERAL_STATIC_REF:       return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_TYPE_GENERAL_DEBUG:            return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_TYPE_GENERAL_USER_IN:          MotorController_Var_Input_Set(p_context, varId.Base, value);                           break;
        // case MOT_VAR_TYPE_GENERAL_USER_CONTROL:     MotorController_Var_Control_Set(p_context, varId.Base, value);                         break;
        case MOT_VAR_TYPE_GENERAL_CONFIG:           MotorController_Config_Set(p_context, varId.Base, value);                              break;
        case MOT_VAR_TYPE_ANALOG_USER_CONFIG:       MotAnalogUser_ConfigId_Set(&p_context->ANALOG_USER, varId.Base, value);                break;

        case MOT_VAR_TYPE_V_MONITOR_SOURCE_STATE:           return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_TYPE_V_MONITOR_SOURCE_VDIVIDER_REF:    return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_TYPE_V_MONITOR_ACCS_STATE:             return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_TYPE_V_MONITOR_ACCS_VDIVIDER_REF:      return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_TYPE_V_MONITOR_ANALOG_STATE:           return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_TYPE_V_MONITOR_ANALOG_VDIVIDER_REF:    return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_TYPE_V_MONITOR_SOURCE_CONFIG:          VMonitor_ConfigId_Set(&p_context->V_SOURCE, varId.Base, value); break;
        case MOT_VAR_TYPE_V_MONITOR_ACCS_CONFIG:            VMonitor_ConfigId_Set(&p_context->V_ACCESSORIES, varId.Base, value); break;
        case MOT_VAR_TYPE_V_MONITOR_ANALOG_CONFIG:          VMonitor_ConfigId_Set(&p_context->V_ANALOG, varId.Base, value); break;

        case MOT_VAR_TYPE_HEAT_MONITOR_PCB_STATE:                        return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_STATE:                    return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_INSTANCE_STATE:           return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_INSTANCE_THERMISTOR_REF:  return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_TYPE_HEAT_MONITOR_PCB_CONFIG:                          HeatMonitor_ConfigId_Set(&p_context->HEAT_PCB, varId.Base, value);                 break;
        case MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_CONFIG:                      HeatMonitor_Group_ConfigId_Set(&p_context->HEAT_MOSFETS, varId.Base, value);       break;

        case MOT_VAR_TYPE_SOCKET_STATE:             return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_TYPE_SOCKET_CONFIG:            Socket_ConfigId_Set(SocketAt(p_context, varId.Instance), varId.Base, value);     break;
        case MOT_VAR_TYPE_CAN_BUS_STATE:            return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_TYPE_CAN_BUS_CONFIG:           break; // CanBusConfig_Set(p_context, varId.Base, value);

        case MOT_VAR_TYPE_VEHICLE_CONTROL:        Vehicle_VarId_Set(&p_context->VEHICLE, varId.Base, value);                       break;
        case MOT_VAR_TYPE_VEHICLE_CONFIG:         Vehicle_ConfigId_Set(p_context->VEHICLE.P_VEHICLE_STATE, varId.Base, value);     break;
        default: return MOT_VAR_STATUS_ERROR_INVALID_ID;
        // case MOT_VAR_TYPE_COMMAND_NVM:           return _HandleSystemCommand_Nvm_Set(p_context, varId.Base, value);
        // case MOT_VAR_TYPE_COMMAND_CALIBRATION:   return _HandleSystemCommand_Calibration_Set(p_context, varId.Base, value);
        // case MOT_VAR_TYPE_COMMAND_BUZZER:        return _HandleSystemCommand_Buzzer_Set(p_context, varId.Base, value);
        // case MOT_VAR_TYPE_COMMAND_CONFIG:        return _HandleSystemCommand_Config_Set(p_context, varId.Base, value);
    }

    return MOT_VAR_STATUS_OK;
}

/******************************************************************************/
/*
    Main Dispatcher Functions - Public Interface
*/
/******************************************************************************/


/******************************************************************************/
/*!
    @brief Set Helpers
*/
/******************************************************************************/
// Analog mode does not allow these variables to be set
static inline bool IsProtocolControlMode(const MotorController_T * p_context)
{
    switch (p_context->P_MC_STATE->Config.InputMode)
    {
        case MOTOR_CONTROLLER_INPUT_MODE_SERIAL:    return true;
        case MOTOR_CONTROLLER_INPUT_MODE_CAN:       return true;
        case MOTOR_CONTROLLER_INPUT_MODE_ANALOG:    return false;
        default: return false;
    }
}

/*
    Input Guard
    Checks for Input and Config policies before variable access
*/
static MotVarId_Status_T CheckInputPolicy(const MotorController_T * p_context, MotVarId_T varId)
{
    // Check read/write policies based on var type
    switch ((MotVarId_HandlerType_T)varId.OuterType)
    {
        case MOT_VAR_ID_HANDLER_TYPE_MOTOR:
            switch ((Motor_VarType_T)varId.InnerType)
            {
                case MOTOR_VAR_TYPE_USER_CONTROL:
                case MOTOR_VAR_TYPE_USER_SETPOINT:
                    if (!IsProtocolControlMode(p_context)) return MOT_VAR_STATUS_ERROR_ACCESS_DISABLED;
                    // todo if (!MotorController_IsMotorCmd(p_context)) return MOT_VAR_STATUS_ERROR_ACCESS_DISABLED;
                    break;
                // case MOTOR_VAR_TYPE_CONFIG_CMD:
                //     if (!MotorController_IsConfig(p_context)) return MOT_VAR_STATUS_ERROR_NOT_CONFIG_STATE;
                //     break;
                default:
                    break;
            }
            break;

        case MOT_VAR_ID_HANDLER_TYPE_SYSTEM:
            switch ((MotorController_VarType_T)varId.InnerType)
            {
                case MOT_VAR_TYPE_GENERAL_USER_IN:
                    if (!IsProtocolControlMode(p_context)) return MOT_VAR_STATUS_ERROR_ACCESS_DISABLED;
                    break;

                case MOT_VAR_TYPE_GENERAL_CONFIG:
                case MOT_VAR_TYPE_ANALOG_USER_CONFIG:
                case MOT_VAR_TYPE_V_MONITOR_SOURCE_CONFIG:
                case MOT_VAR_TYPE_V_MONITOR_ACCS_CONFIG:
                case MOT_VAR_TYPE_V_MONITOR_ANALOG_CONFIG:
                case MOT_VAR_TYPE_HEAT_MONITOR_PCB_CONFIG:
                case MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_CONFIG:
                case MOT_VAR_TYPE_SOCKET_CONFIG:
                case MOT_VAR_TYPE_CAN_BUS_CONFIG:
                    if (!MotorController_IsConfig(p_context)) return MOT_VAR_STATUS_ERROR_NOT_CONFIG_STATE;
                    break;
                case MOT_VAR_TYPE_VEHICLE_CONTROL:
                    if (!IsProtocolControlMode(p_context)) return MOT_VAR_STATUS_ERROR_ACCESS_DISABLED;
                    break;
                case MOT_VAR_TYPE_VEHICLE_CONFIG:
                    if (!MotorController_IsConfig(p_context)) return MOT_VAR_STATUS_ERROR_NOT_CONFIG_STATE;
                    break;
                default:
                    break;
            }
            break;
        // case MOT_VAR_ID_HANDLER_TYPE_SYSTEM_COMMAND:
        //     if (!IsProtocolControlMode(p_context)) return MOT_VAR_STATUS_ERROR_ACCESS_DISABLED;
        default:
            break;
    }
    // Access control check using InnerType pattern
    // if (MotVarId_IsReadOnly(varId)) { return MOT_VAR_STATUS_ERROR_READ_ONLY; }
    // if (MotVarId_IsConfig(varId) && !MotorController_IsConfigState(p_context)) { return MOT_VAR_STATUS_ERROR_NOT_CONFIG_STATE; }
    return MOT_VAR_STATUS_OK;
}

/******************************************************************************/
/*!
    @brief Get variable value by ID
*/
/******************************************************************************/
int MotorController_Var_Get(const MotorController_T * p_context, MotVarId_T varId)
{
    switch ((MotVarId_HandlerType_T)varId.OuterType)
    {
        case MOT_VAR_ID_HANDLER_TYPE_MOTOR:         return _HandleMotorVar_Get(p_context, varId);
        case MOT_VAR_ID_HANDLER_TYPE_SYSTEM:        return _HandleSystem_Get(p_context, varId);
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
    MotVarId_Status_T accessStatus = CheckInputPolicy(p_context, varId);
    if (accessStatus != MOT_VAR_STATUS_OK) return accessStatus;

    switch ((MotVarId_HandlerType_T)varId.OuterType)
    {
        case MOT_VAR_ID_HANDLER_TYPE_MOTOR:             return _HandleMotorVar_Set(p_context, varId, value);
        case MOT_VAR_ID_HANDLER_TYPE_SYSTEM:            return _HandleSystem_Set(p_context, varId, value);
        default:                                        return MOT_VAR_STATUS_ERROR_INVALID_ID;
    }
}


// Check if a variable requires protocol control authority
// static inline bool MotVarId_IsProtocolControl(MotVarId_T varId)
// {
//     switch ((MotVarId_HandlerType_T)varId.OuterType)
//     {
//         case MOT_VAR_ID_HANDLER_TYPE_MOTOR_CONTROL: return true;
//         case MOT_VAR_ID_HANDLER_TYPE_GENERAL: return (varId.InnerType == MOT_VAR_TYPE_GENERAL_USER_IN);
//         case MOT_VAR_ID_HANDLER_TYPE_APPLICATION_COMMAND: return (varId.InnerType == MOT_VAR_TYPE_VEHICLE_CONTROL);
//         default:  return false;  // Config, monitoring, etc. don't need mux check
//     }
// }

// static inline bool MotVarId_IsMotorDirect(MotVarId_T varId)
// {
//     switch ((MotVarId_HandlerType_T)varId.OuterType)
//     {
//         case MOT_VAR_ID_HANDLER_TYPE_MOTOR_CONTROL: return true;
//         case MOT_VAR_ID_HANDLER_TYPE_MOTOR_CONFIG:  return ((varId.InnerType == MOTOR_VAR_TYPE_CONFIG_CMD) || (varId.InnerType == MOTOR_VAR_TYPE_CONFIG_SENSOR_CMD));
//             /* ConfigCmds are also blocked during non StopState */
//         default:  return false;
//     }
// }
