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

/* only point of coupling to AppTable */
#include "AppTable/MotorController_AppTable.h"


/******************************************************************************/
/*
    MotorController Module General
*/
/******************************************************************************/
int MotorController_Var_Output_Get(const MotorController_T * p_dev, MotorController_Var_Output_T id)
{
    int value = 0;
    switch (id)
    {
        case MOT_VAR_ZERO:                  value = 0;                                                                  break;
        case MOT_VAR_MILLIS:                value = Millis();                                                           break;
        case MOT_VAR_SYSTEM_STATE:              value = MotorController_GetStateId(p_dev->P_MC);              break;
        case MOT_VAR_SYSTEM_SUB_STATE:          value = _MotorController_GetSubStateId(p_dev->P_MC);          break;
        case MOT_VAR_SYSTEM_FAULT_FLAGS:        value = MotorController_GetFaultFlags(p_dev->P_MC).Value;     break;
        // case MOT_VAR_SYSTEM_STATUS_FLAGS:       value = MotorController_GetStatusFlags(p_dev).Value;               break;
        case MOT_VAR_SYSTEM_DIRECTION:          value = MotorController_GetDirection(p_dev);                        break;
    }
    return value;
}

/* Inputs disabled on Analog Mode */
void MotorController_Var_Input_Set(const MotorController_T * p_dev, MotorController_Var_Input_T id, int value)
{
    switch (id)
    {
        case MOT_VAR_USER_GENERAL_SET_POINT:        MotorController_SetCmdValue(p_dev, (int16_t)value);                                            break;
        case MOT_VAR_USER_GENERAL_FEEDBACK_MODE:    MotorController_SetFeedbackMode(p_dev, (Motor_FeedbackMode_T) { .Value = (uint8_t)value });    break;
        case MOT_VAR_USER_GENERAL_DIRECTION:        MotorController_SetDirection(p_dev, (int16_t)value);                                            break;
        case MOT_VAR_USER_GENERAL_PHASE_OUTPUT:     MotorController_SetControlState(p_dev, (Phase_Output_T)value);                                 break;

        // case MOT_VAR_USER_OPT_SPEED_LIMIT_ON_OFF:   MotorController_SetOptSpeedLimitOnOff(p_dev, (bool)value);        break;
        // case MOT_VAR_USER_OPT_I_LIMIT_ON_OFF:       MotorController_SetOptILimitOnOff(p_dev, (bool)value);            break;
        // case MOT_VAR_USER_RELAY_TOGGLE:                 break;
        // case MOT_VAR_USER_METER_TOGGLE:                 break;
    }
}

int MotorController_Var_OutputDebug_Get(const MotorController_T * p_dev, MotorController_Var_OutputDebug_T id)
{
    int value = 0;
    // #ifndef NDEBUG
    Motor_T * p_motor = Motor_Table_ContextAt(&p_dev->MOTORS, 0) ;
    Motor_State_T * p_motorState = Motor_Table_ContextAt(&p_dev->MOTORS, 0)->P_MOTOR;
    switch (id)
    {
        case MOT_VAR_CONTROL_LOOP_PROFILE:  value = p_dev->P_MC->ControlLoopProfile;            break;
        case MOT_VAR_DEBUG0: value = 0;             break;
        case MOT_VAR_DEBUG1: value = 0;             break;
        case MOT_VAR_DEBUG2: value = 0;     break;
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
int MotorController_Config_Get(const MotorController_T * p_dev, MotorController_Var_Config_T id)
{
    MotorController_State_T * p_state = p_dev->P_MC;
    int value = 0;
    switch (id)
    {
        case MOT_VAR_V_SUPPLY_VOLTS:           value = p_state->Config.VBusConfig.VSupplyNominal_V;       break;
        case MOT_VAR_I_LIMIT_RESV:             value = p_state->Config.VBusConfig.IDerateUnderVFloor_Fract16; break;
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

void MotorController_Config_Set(const MotorController_T * p_dev, MotorController_Var_Config_T id, int value)
{
    MotorController_State_T * p_state = p_dev->P_MC;
    switch (id)
    {
        case MOT_VAR_V_SUPPLY_VOLTS:        MotorController_SetVSupplyRef(p_dev, value);                                            break;
        case MOT_VAR_I_LIMIT_RESV:          p_state->Config.VBusConfig.IDerateUnderVFloor_Fract16 = value;                          break;
        case MOT_VAR_MAIN_MODE:             p_state->Config.InitMode = (MotorController_MainMode_T)value;                           break;
        case MOT_VAR_INPUT_MODE:            MotorController_SetInputMode(p_dev, (MotorController_InputMode_T)value);                     break;
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
static inline uint8_t MotorController_Var_GetMotorCount(const MotorController_T * p_dev) { return p_dev->MOTORS.LENGTH; }
static inline uint8_t MotorController_Var_GetHeatMosfetCount(const MotorController_T * p_dev) { return HeatMonitor_Group_GetInstanceCount(&p_dev->HEAT_MOSFETS); }
static inline uint8_t MotorController_Var_GetVMonitorCount(const MotorController_T * p_dev) { return (p_dev->P_VBUS != NULL) + (p_dev->V_ACCESSORIES.P_STATE != NULL) + (p_dev->V_ANALOG.P_STATE != NULL); }
static inline uint8_t MotorController_Var_GetProtocolCount(const MotorController_T * p_dev) { return p_dev->PROTOCOL_COUNT; }

int MotorController_InstancesRef_Get(const MotorController_T * p_dev, MotorController_Var_Board_T nameBase)
{
    switch (nameBase)
    {
        case MOT_VAR_BOARD_MOTOR_COUNT:               return MotorController_Var_GetMotorCount(p_dev);
        case MOT_VAR_BOARD_V_MONITOR_COUNT:           return MotorController_Var_GetVMonitorCount(p_dev);
        case MOT_VAR_BOARD_THERMISTOR_MOSFETS_COUNT:  return MotorController_Var_GetHeatMosfetCount(p_dev);
        case MOT_VAR_BOARD_PROTOCOL_SOCKET_COUNT:     return MotorController_Var_GetProtocolCount(p_dev);
        case MOT_VAR_BOARD_CAN_SOCKET_COUNT:          return 0; /* not yet implemented */
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
    @brief Motor Prefix Handlers
*/
/******************************************************************************/
static inline Motor_T * MotorAt(const MotorController_T * p_dev, uint8_t motor) { return (motor < p_dev->MOTORS.LENGTH) ? Motor_Table_ContextAt(&p_dev->MOTORS, motor) : NULL; }

static int _HandleMotor_Get(const MotorController_T * p_dev, MotVarId_T varId) { return Motor_VarType_Base_Get(MotorAt(p_dev, varId.Instance), (Motor_VarType_Base_T)varId.Type, varId.Base); }

static MotVarId_Status_T _HandleMotor_Set(const MotorController_T * p_dev, MotVarId_T varId, int value)
{
    Motor_T * p_motor = MotorAt(p_dev, varId.Instance);
    if (p_motor == NULL) return MOT_VAR_STATUS_ERROR_INVALID_ID;
    if (!Motor_VarType_Base_CheckSet(p_motor, (Motor_VarType_Base_T)varId.Type)) return MOT_VAR_STATUS_ERROR_READ_ONLY;
    Motor_VarType_Base_Set(p_motor, (Motor_VarType_Base_T)varId.Type, varId.Base, value);
    return MOT_VAR_STATUS_OK;
}

static int _HandleMotorSubModule_Get(const MotorController_T * p_dev, MotVarId_T varId) { return Motor_VarType_SubModule_Get(MotorAt(p_dev, varId.Instance), (Motor_VarType_SubModule_T)varId.Type, varId.Base); }

static MotVarId_Status_T _HandleMotorSubModule_Set(const MotorController_T * p_dev, MotVarId_T varId, int value)
{
    Motor_T * p_motor = MotorAt(p_dev, varId.Instance);
    if (p_motor == NULL) return MOT_VAR_STATUS_ERROR_INVALID_ID;
    if (!Motor_VarType_SubModule_CheckSet(p_motor, (Motor_VarType_SubModule_T)varId.Type)) return MOT_VAR_STATUS_ERROR_READ_ONLY;
    Motor_VarType_SubModule_Set(p_motor, (Motor_VarType_SubModule_T)varId.Type, varId.Base, value);
    return MOT_VAR_STATUS_OK;
}

static int _HandleMotorSensor_Get(const MotorController_T * p_dev, MotVarId_T varId) { return Motor_VarType_Sensor_Get(MotorAt(p_dev, varId.Instance), (Motor_VarType_Sensor_T)varId.Type, varId.Base); }

static MotVarId_Status_T _HandleMotorSensor_Set(const MotorController_T * p_dev, MotVarId_T varId, int value)
{
    Motor_T * p_motor = MotorAt(p_dev, varId.Instance);
    if (p_motor == NULL) return MOT_VAR_STATUS_ERROR_INVALID_ID;
    if (!Motor_VarType_Sensor_CheckSet(p_motor, (Motor_VarType_Sensor_T)varId.Type)) return MOT_VAR_STATUS_ERROR_READ_ONLY;
    Motor_VarType_Sensor_Set(p_motor, (Motor_VarType_Sensor_T)varId.Type, varId.Base, value);
    return MOT_VAR_STATUS_OK;
}

/******************************************************************************/
/*!
    @brief System Prefix Handlers
*/
/******************************************************************************/
static inline Socket_T * SocketAt(const MotorController_T * p_dev, uint8_t protocolIndex)
{
    return (protocolIndex < p_dev->PROTOCOL_COUNT) ? &p_dev->P_PROTOCOLS[protocolIndex] : NULL;
}

static int _HandleGeneral_Get(const MotorController_T * p_dev, MotVarId_T varId)
{
    switch ((MotorController_VarType_General_T)varId.Type)
    {
        case MOT_VAR_TYPE_GENERAL_USER_OUT:      return MotorController_Var_Output_Get(p_dev, varId.Base);
        case MOT_VAR_TYPE_GENERAL_CONFIG:        return MotorController_Config_Get(p_dev, varId.Base);
        case MOT_VAR_TYPE_GENERAL_USER_IN:       return 0;
        case MOT_VAR_TYPE_GENERAL_BOARD_CONST:    return MotorController_InstancesRef_Get(p_dev, varId.Base);
        case MOT_VAR_TYPE_GENERAL_DEBUG:         return MotorController_Var_OutputDebug_Get(p_dev, varId.Base);
        case MOT_VAR_TYPE_ANALOG_USER_VAR_OUT:   return MotAnalogUser_VarId_GetAsInput(&p_dev->ANALOG_USER, varId.Base);
        case MOT_VAR_TYPE_ANALOG_USER_CONFIG:    return MotAnalogUser_ConfigId_Get(&p_dev->ANALOG_USER, varId.Base);
        default: return 0;
    }
}

static MotVarId_Status_T _HandleGeneral_Set(const MotorController_T * p_dev, MotVarId_T varId, int value)
{
    switch ((MotorController_VarType_General_T)varId.Type)
    {
        case MOT_VAR_TYPE_GENERAL_USER_OUT:         return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_TYPE_ANALOG_USER_VAR_OUT:      return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_TYPE_GENERAL_BOARD_CONST:       return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_TYPE_GENERAL_DEBUG:            return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_TYPE_GENERAL_USER_IN:          MotorController_Var_Input_Set(p_dev, varId.Base, value);                           break;
        case MOT_VAR_TYPE_GENERAL_CONFIG:           MotorController_Config_Set(p_dev, varId.Base, value);                              break;
        case MOT_VAR_TYPE_ANALOG_USER_CONFIG:       MotAnalogUser_ConfigId_Set(&p_dev->ANALOG_USER, varId.Base, value);                break;
        default: return MOT_VAR_STATUS_ERROR_INVALID_ID;
    }
    return MOT_VAR_STATUS_OK;
}

static int _HandleVMonitor_Get(const MotorController_T * p_dev, MotVarId_T varId)
{
    switch ((MotorController_VarType_VMonitor_T)varId.Type)
    {
        case MOT_VAR_TYPE_V_MONITOR_SOURCE_STATE:           return RangeMonitor_VarId_Get(VBus_Monitor(p_dev->P_VBUS), varId.Base);
        case MOT_VAR_TYPE_V_MONITOR_SOURCE_CONFIG:          return RangeMonitor_ConfigId_Get(VBus_Monitor(p_dev->P_VBUS), varId.Base);
        case MOT_VAR_TYPE_V_MONITOR_SOURCE_VDIVIDER:
            switch (varId.Base)
            {
                case VDIVIDER_BOARD_R1: return Motor_Var_Board_Get(MOTOR_VAR_BOARD_V_PHASE_R1);
                case VDIVIDER_BOARD_R2: return Motor_Var_Board_Get(MOTOR_VAR_BOARD_V_PHASE_R2);
                default: return 0;
            }
        case MOT_VAR_TYPE_V_MONITOR_ACCS_STATE:             return VMonitor_VarId_Get(&p_dev->V_ACCESSORIES, varId.Base);
        case MOT_VAR_TYPE_V_MONITOR_ACCS_CONFIG:            return VMonitor_ConfigId_Get(&p_dev->V_ACCESSORIES, varId.Base);
        case MOT_VAR_TYPE_V_MONITOR_ACCS_VDIVIDER:          return VMonitor_VDivider_ConfigId_Get(&p_dev->V_ACCESSORIES, varId.Base);
        case MOT_VAR_TYPE_V_MONITOR_ANALOG_STATE:           return VMonitor_VarId_Get(&p_dev->V_ANALOG, varId.Base);
        case MOT_VAR_TYPE_V_MONITOR_ANALOG_CONFIG:          return VMonitor_ConfigId_Get(&p_dev->V_ANALOG, varId.Base);
        case MOT_VAR_TYPE_V_MONITOR_ANALOG_VDIVIDER:        return VMonitor_VDivider_ConfigId_Get(&p_dev->V_ANALOG, varId.Base);
        default: return 0;
    }
}

static MotVarId_Status_T _HandleVMonitor_Set(const MotorController_T * p_dev, MotVarId_T varId, int value)
{
    switch ((MotorController_VarType_VMonitor_T)varId.Type)
    {
        case MOT_VAR_TYPE_V_MONITOR_SOURCE_STATE:           return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_TYPE_V_MONITOR_SOURCE_VDIVIDER:        return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_TYPE_V_MONITOR_ACCS_STATE:             return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_TYPE_V_MONITOR_ACCS_VDIVIDER:          return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_TYPE_V_MONITOR_ANALOG_STATE:           return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_TYPE_V_MONITOR_ANALOG_VDIVIDER:        return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_TYPE_V_MONITOR_SOURCE_CONFIG:          RangeMonitor_ConfigId_Set(VBus_Monitor(p_dev->P_VBUS), varId.Base, value); break;
        case MOT_VAR_TYPE_V_MONITOR_ACCS_CONFIG:            VMonitor_ConfigId_Set(&p_dev->V_ACCESSORIES, varId.Base, value); break;
        case MOT_VAR_TYPE_V_MONITOR_ANALOG_CONFIG:          VMonitor_ConfigId_Set(&p_dev->V_ANALOG, varId.Base, value); break;
        default: return MOT_VAR_STATUS_ERROR_INVALID_ID;
    }
    return MOT_VAR_STATUS_OK;
}

static int _HandleHeatMonitor_Get(const MotorController_T * p_dev, MotVarId_T varId)
{
    switch ((MotorController_VarType_HeatMonitor_T)varId.Type)
    {
        case MOT_VAR_TYPE_HEAT_MONITOR_PCB_STATE:                           return HeatMonitor_VarId_Get(&p_dev->HEAT_PCB, varId.Base);
        case MOT_VAR_TYPE_HEAT_MONITOR_PCB_CONFIG:                          return HeatMonitor_ConfigId_Get(&p_dev->HEAT_PCB, varId.Base);
        case MOT_VAR_TYPE_HEAT_MONITOR_PCB_THERMISTOR:                      return HeatMonitor_Thermistor_ConfigId_Get(&p_dev->HEAT_PCB, varId.Base);
        case MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_STATE:                       return HeatMonitor_Group_VarId_Get(&p_dev->HEAT_MOSFETS, varId.Base);
        case MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_CONFIG:                      return HeatMonitor_Group_ConfigId_Get(&p_dev->HEAT_MOSFETS, varId.Base);
        case MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_INSTANCE_STATE:              return HeatMonitor_GroupInstance_VarId_Get(&p_dev->HEAT_MOSFETS, varId.Instance, varId.Base);
        case MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_INSTANCE_THERMISTOR:         return HeatMonitor_GroupInstance_ThermistorConfigId_Get(&p_dev->HEAT_MOSFETS, varId.Instance, varId.Base);
        default: return 0;
    }
}

static MotVarId_Status_T _HandleHeatMonitor_Set(const MotorController_T * p_dev, MotVarId_T varId, int value)
{
    switch ((MotorController_VarType_HeatMonitor_T)varId.Type)
    {
        case MOT_VAR_TYPE_HEAT_MONITOR_PCB_STATE:                       return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_STATE:                   return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_INSTANCE_STATE:          return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_INSTANCE_THERMISTOR:     return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_TYPE_HEAT_MONITOR_PCB_THERMISTOR:                  return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_TYPE_HEAT_MONITOR_PCB_CONFIG:                      HeatMonitor_ConfigId_Set(&p_dev->HEAT_PCB, varId.Base, value);                 break;
        case MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_CONFIG:                  HeatMonitor_Group_ConfigId_Set(&p_dev->HEAT_MOSFETS, varId.Base, value);       break;
        default: return MOT_VAR_STATUS_ERROR_INVALID_ID;
    }
    return MOT_VAR_STATUS_OK;
}

static int _HandleCommunication_Get(const MotorController_T * p_dev, MotVarId_T varId)
{
    switch ((MotorController_VarType_Communication_T)varId.Type)
    {
        case MOT_VAR_TYPE_SOCKET_STATE:             return 0;
        case MOT_VAR_TYPE_SOCKET_CONFIG:            return Socket_ConfigId_Get(SocketAt(p_dev, varId.Instance), varId.Base);
        case MOT_VAR_TYPE_CAN_BUS_STATE:            return 0;
        case MOT_VAR_TYPE_CAN_BUS_CONFIG:           return 0;
        default: return 0;
    }
}

static MotVarId_Status_T _HandleCommunication_Set(const MotorController_T * p_dev, MotVarId_T varId, int value)
{
    switch ((MotorController_VarType_Communication_T)varId.Type)
    {
        case MOT_VAR_TYPE_SOCKET_STATE:             return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_TYPE_CAN_BUS_STATE:            return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_TYPE_SOCKET_CONFIG:            Socket_ConfigId_Set(SocketAt(p_dev, varId.Instance), varId.Base, value);     break;
        case MOT_VAR_TYPE_CAN_BUS_CONFIG:           break;
        default: return MOT_VAR_STATUS_ERROR_INVALID_ID;
    }
    return MOT_VAR_STATUS_OK;
}

static int _HandleAppUser_Get(const MotorController_T * p_dev, MotVarId_T varId)
{
    switch ((MotorController_VarType_AppUser_T)varId.Type)
    {
        case MOT_VAR_TYPE_TRACTION_CONTROL:     return MotorController_Traction_VarId_Get(p_dev, varId.Base);
        case MOT_VAR_TYPE_TRACTION_CONFIG:      return MotorController_Traction_ConfigId_Get(p_dev, varId.Base);
        default: return 0;
    }
}

static MotVarId_Status_T _HandleAppUser_Set(const MotorController_T * p_dev, MotVarId_T varId, int value)
{
    switch ((MotorController_VarType_AppUser_T)varId.Type)
    {
        case MOT_VAR_TYPE_TRACTION_CONTROL:        MotorController_Traction_VarId_Set(p_dev, varId.Base, value);                       break;
        case MOT_VAR_TYPE_TRACTION_CONFIG:         MotorController_Traction_ConfigId_Set(p_dev, varId.Base, value);     break;
        default: return MOT_VAR_STATUS_ERROR_INVALID_ID;
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
static inline bool IsProtocolControlMode(const MotorController_T * p_dev)
{
    MotorController_InputMode_T mode = p_dev->P_MC->Config.InputMode;
    return (mode == MOTOR_CONTROLLER_INPUT_MODE_SERIAL) || (mode == MOTOR_CONTROLLER_INPUT_MODE_CAN);
}

/*
    Input Guard
    Checks for Input and Config policies before variable access
*/
static MotVarId_Status_T CheckInputPolicy(const MotorController_T * p_dev, MotVarId_T varId)
{
    switch ((MotVarId_Prefix_T)varId.Prefix)
    {
        case MOT_VAR_ID_PREFIX_MOTOR:
            switch ((Motor_VarType_Base_T)varId.Type)
            {
                case MOTOR_VAR_TYPE_USER_CONTROL:
                case MOTOR_VAR_TYPE_USER_SETPOINT:
                    if (!IsProtocolControlMode(p_dev)) return MOT_VAR_STATUS_ERROR_ACCESS_DISABLED;
                    if (!MotorController_IsMotorCmdState(p_dev)) return MOT_VAR_STATUS_ERROR_ACCESS_DISABLED;
                    break;
                case MOTOR_VAR_TYPE_STATE_CMD:
                    if (!MotorController_IsMotorCmdState(p_dev)) return MOT_VAR_STATUS_ERROR_ACCESS_DISABLED;
                    break;
                case MOTOR_VAR_TYPE_CONFIG_CALIBRATION:
                case MOTOR_VAR_TYPE_CONFIG_ACTUATION:
                case MOTOR_VAR_TYPE_CONFIG_PID:
                    if (!MotorController_IsConfig(p_dev)) return MOT_VAR_STATUS_ERROR_NOT_CONFIG_STATE;
                    break;
                case MOTOR_VAR_TYPE_CALIBRATION_CMD:
                    if (!MotorController_IsLock(p_dev)) return MOT_VAR_STATUS_ERROR_NOT_CONFIG_STATE;
                    break;
                default:
                    break;
            }
            break;

        case MOT_VAR_ID_PREFIX_MOTOR_SUB_MODULE:
            switch ((Motor_VarType_SubModule_T)varId.Type)
            {
                case MOTOR_VAR_TYPE_PID_TUNING_IO:
                    if (!MotorController_IsMotorCmdState(p_dev)) return MOT_VAR_STATUS_ERROR_ACCESS_DISABLED;
                    break;
                case MOTOR_VAR_TYPE_HEAT_MONITOR_CONFIG:
                case MOTOR_VAR_TYPE_THERMISTOR_CONFIG:
                    if (!MotorController_IsConfig(p_dev)) return MOT_VAR_STATUS_ERROR_NOT_CONFIG_STATE;
                    break;
                default:
                    break;
            }
            break;

        case MOT_VAR_ID_PREFIX_MOTOR_SENSOR:
            switch ((Motor_VarType_Sensor_T)varId.Type)
            {
                case MOTOR_VAR_TYPE_HALL_CONFIG:
                case MOTOR_VAR_TYPE_ENCODER_CONFIG:
                    if (!MotorController_IsConfig(p_dev)) return MOT_VAR_STATUS_ERROR_NOT_CONFIG_STATE;
                    break;
                case MOTOR_VAR_TYPE_HALL_CMD:
                case MOTOR_VAR_TYPE_ENCODER_CMD:
                    if (!MotorController_IsLock(p_dev)) return MOT_VAR_STATUS_ERROR_NOT_CONFIG_STATE;
                    break;
                default:
                    break;
            }
            break;

        case MOT_VAR_ID_PREFIX_GENERAL:
            switch ((MotorController_VarType_General_T)varId.Type)
            {
                case MOT_VAR_TYPE_GENERAL_USER_IN:
                    if (!IsProtocolControlMode(p_dev)) return MOT_VAR_STATUS_ERROR_ACCESS_DISABLED;
                    break;
                case MOT_VAR_TYPE_GENERAL_CONFIG:
                case MOT_VAR_TYPE_ANALOG_USER_CONFIG:
                    if (!MotorController_IsConfig(p_dev)) return MOT_VAR_STATUS_ERROR_NOT_CONFIG_STATE;
                    break;
                default:
                    break;
            }
            break;

        case MOT_VAR_ID_PREFIX_V_MONITOR:
            switch ((MotorController_VarType_VMonitor_T)varId.Type)
            {
                case MOT_VAR_TYPE_V_MONITOR_SOURCE_CONFIG:
                case MOT_VAR_TYPE_V_MONITOR_ACCS_CONFIG:
                case MOT_VAR_TYPE_V_MONITOR_ANALOG_CONFIG:
                    if (!MotorController_IsConfig(p_dev)) return MOT_VAR_STATUS_ERROR_NOT_CONFIG_STATE;
                    break;
                default:
                    break;
            }
            break;

        case MOT_VAR_ID_PREFIX_HEAT_MONITOR:
            switch ((MotorController_VarType_HeatMonitor_T)varId.Type)
            {
                case MOT_VAR_TYPE_HEAT_MONITOR_PCB_CONFIG:
                case MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_CONFIG:
                    if (!MotorController_IsConfig(p_dev)) return MOT_VAR_STATUS_ERROR_NOT_CONFIG_STATE;
                    break;
                default:
                    break;
            }
            break;

        case MOT_VAR_ID_PREFIX_COMMUNICATION:
            switch ((MotorController_VarType_Communication_T)varId.Type)
            {
                case MOT_VAR_TYPE_SOCKET_CONFIG:
                case MOT_VAR_TYPE_CAN_BUS_CONFIG:
                    if (!MotorController_IsConfig(p_dev)) return MOT_VAR_STATUS_ERROR_NOT_CONFIG_STATE;
                    break;
                default:
                    break;
            }
            break;

        case MOT_VAR_ID_PREFIX_APP_USER:
            switch ((MotorController_VarType_AppUser_T)varId.Type)
            {
                case MOT_VAR_TYPE_TRACTION_CONTROL:
                    if (!IsProtocolControlMode(p_dev)) return MOT_VAR_STATUS_ERROR_ACCESS_DISABLED;
                    break;
                case MOT_VAR_TYPE_TRACTION_CONFIG:
                    if (!MotorController_IsConfig(p_dev)) return MOT_VAR_STATUS_ERROR_NOT_CONFIG_STATE;
                    break;
                default:
                    break;
            }
            break;

        default:
            break;
    }
    return MOT_VAR_STATUS_OK;

    // VarAccess_Table TABLE[] =
    // {
    //     [MACRO(MOT_VAR_ID_PREFIX_MOTOR, MOTOR_VAR_TYPE_STATE_CMD)] = {.CheckSet = MotorController_IsMotorCmdState, .Getter = _Motor_Var_StateCmd_Get, .Setter = _Motor_Var_StateCmd_Set },
    // }
}

/******************************************************************************/
/*!
    @brief Get variable value by ID
*/
/******************************************************************************/
int MotorController_Var_Get(const MotorController_T * p_dev, MotVarId_T varId)
{
    switch ((MotVarId_Prefix_T)varId.Prefix)
    {
        case MOT_VAR_ID_PREFIX_MOTOR:             return _HandleMotor_Get(p_dev, varId);
        case MOT_VAR_ID_PREFIX_MOTOR_SUB_MODULE:  return _HandleMotorSubModule_Get(p_dev, varId);
        case MOT_VAR_ID_PREFIX_MOTOR_SENSOR:      return _HandleMotorSensor_Get(p_dev, varId);
        case MOT_VAR_ID_PREFIX_GENERAL:           return _HandleGeneral_Get(p_dev, varId);
        case MOT_VAR_ID_PREFIX_V_MONITOR:         return _HandleVMonitor_Get(p_dev, varId);
        case MOT_VAR_ID_PREFIX_HEAT_MONITOR:      return _HandleHeatMonitor_Get(p_dev, varId);
        case MOT_VAR_ID_PREFIX_COMMUNICATION:     return _HandleCommunication_Get(p_dev, varId);
        case MOT_VAR_ID_PREFIX_APP_USER:          return _HandleAppUser_Get(p_dev, varId);
        default: return 0;
    }
}

/******************************************************************************/
/*!
    @brief Set variable value by ID
*/
/******************************************************************************/
MotVarId_Status_T MotorController_Var_Set(const MotorController_T * p_dev, MotVarId_T varId, int value)
{
    MotVarId_Status_T accessStatus = CheckInputPolicy(p_dev, varId);
    if (accessStatus != MOT_VAR_STATUS_OK) return accessStatus;

    switch ((MotVarId_Prefix_T)varId.Prefix)
    {
        case MOT_VAR_ID_PREFIX_MOTOR:             return _HandleMotor_Set(p_dev, varId, value);
        case MOT_VAR_ID_PREFIX_MOTOR_SUB_MODULE:  return _HandleMotorSubModule_Set(p_dev, varId, value);
        case MOT_VAR_ID_PREFIX_MOTOR_SENSOR:      return _HandleMotorSensor_Set(p_dev, varId, value);
        case MOT_VAR_ID_PREFIX_GENERAL:           return _HandleGeneral_Set(p_dev, varId, value);
        case MOT_VAR_ID_PREFIX_V_MONITOR:         return _HandleVMonitor_Set(p_dev, varId, value);
        case MOT_VAR_ID_PREFIX_HEAT_MONITOR:      return _HandleHeatMonitor_Set(p_dev, varId, value);
        case MOT_VAR_ID_PREFIX_COMMUNICATION:     return _HandleCommunication_Set(p_dev, varId, value);
        case MOT_VAR_ID_PREFIX_APP_USER:          return _HandleAppUser_Set(p_dev, varId, value);
        default:                                  return MOT_VAR_STATUS_ERROR_INVALID_ID;
    }
}
// Check if a variable requires protocol control authority
// static inline bool MotVarId_IsProtocolControl(MotVarId_T varId)
// {
//     switch ((MotVarId_Handler_T)varId.OuterType)
//     {
//         case MOT_VAR_ID_HANDLER_MOTOR_CONTROL: return true;
//         case MOT_VAR_ID_HANDLER_GENERAL: return (varId.InnerType == MOT_VAR_TYPE_GENERAL_USER_IN);
//         case MOT_VAR_ID_HANDLER_APPLICATION_COMMAND: return (varId.InnerType == MOT_VAR_TYPE_TRACTION_CONTROL);
//         default:  return false;  // Config, monitoring, etc. don't need mux check
//     }
// }

// static inline bool MotVarId_IsMotorDirect(MotVarId_T varId)
// {
//     switch ((MotVarId_Handler_T)varId.OuterType)
//     {
//         case MOT_VAR_ID_HANDLER_MOTOR_CONTROL: return true;
//         case MOT_VAR_ID_HANDLER_MOTOR_CONFIG:  return ((varId.InnerType == MOTOR_VAR_TYPE_CALIBRATION_CMD) || (varId.InnerType == MOTOR_VAR_TYPE_CONFIG_RESV));
//             /* ConfigCmds are also blocked during non StopState */
//         default:  return false;
//     }
// }