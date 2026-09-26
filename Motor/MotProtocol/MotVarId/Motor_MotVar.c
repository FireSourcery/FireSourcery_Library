/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2026 FireSourcery

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
    @file   Motor_MotVar.c
    @author FireSourcery
    @brief  [Motor_T] dispatch by [MotVarId_T] Type. Protocol-side routing to [Motor_Var] accessors.
*/
/******************************************************************************/
#include "Motor_MotVar.h"

#include "Motor/Motor/Motor_Var.h"
#include "Motor/Motor/StateMachine/Motor_StateMachine.h"

/******************************************************************************/
/*
    Type Dispatchers
*/
/******************************************************************************/
/******************************************************************************/
/*
    [VarType_Base]
*/
/******************************************************************************/
int Motor_VarType_Base_Get(Motor_T * p_motor, Motor_VarType_Base_T typeId, int varId)
{
    if (p_motor == NULL) { return 0; }
    switch (typeId)
    {
        case MOTOR_VAR_TYPE_USER_OUT:               return _Motor_Var_UserOut_Get(p_motor, varId);
        case MOTOR_VAR_TYPE_USER_CONTROL:           return _Motor_Var_UserControl_Get(p_motor, varId);
        case MOTOR_VAR_TYPE_USER_SETPOINT:          return 0;
        case MOTOR_VAR_TYPE_CONFIG_CALIBRATION:     return _Motor_Var_ConfigCalibration_Get(&p_motor->P_MOTOR->Config, varId);
        case MOTOR_VAR_TYPE_CONFIG_DEBUG:           return _Motor_Var_ConfigDebug_Get(p_motor, varId);
        case MOTOR_VAR_TYPE_CONFIG_ACTUATION:       return _Motor_Var_ConfigActuation_Get(&p_motor->P_MOTOR->Config, varId);
        case MOTOR_VAR_TYPE_CONFIG_PID:             return _Motor_Var_ConfigPid_Get(&p_motor->P_MOTOR->Config, varId);
        case MOTOR_VAR_TYPE_STATE_CMD:              return 0;
        case MOTOR_VAR_TYPE_OPEN_LOOP_CMD:          return 0;
        case MOTOR_VAR_TYPE_CALIBRATION_CMD:        return 0;
        case MOTOR_VAR_TYPE_CONFIG_RESV:            return 0;
        default: break;
    }
    return 0;
}

/* caller handle Access Control */
void Motor_VarType_Base_Set(Motor_T * p_motor, Motor_VarType_Base_T typeId, int varId, int varValue)
{
    switch (typeId)
    {
        case MOTOR_VAR_TYPE_USER_OUT:               break;
        case MOTOR_VAR_TYPE_USER_CONTROL:           _Motor_Var_UserControl_Set(p_motor, varId, varValue);     break;
        case MOTOR_VAR_TYPE_USER_SETPOINT:          _Motor_Var_UserSetpoint_Set(p_motor, varId, varValue);    break;
        case MOTOR_VAR_TYPE_STATE_CMD:              _Motor_Var_StateCmd_Call(p_motor, varId, varValue);        break;
        case MOTOR_VAR_TYPE_OPEN_LOOP_CMD:          _Motor_Var_OpenLoopCmd_Call(p_motor, varId, varValue);   break;
        case MOTOR_VAR_TYPE_CALIBRATION_CMD:        _Motor_Var_CalibrationCmd_Call(p_motor, varId, varValue);                   break;
        case MOTOR_VAR_TYPE_CMD_RESV:               break;
        case MOTOR_VAR_TYPE_CONFIG_CALIBRATION:     _Motor_Var_ConfigCalibration_Set(&p_motor->P_MOTOR->Config, varId, varValue);  break;
        case MOTOR_VAR_TYPE_CONFIG_ACTUATION:       _Motor_Var_ConfigActuation_Set(&p_motor->P_MOTOR->Config, varId, varValue);    break;
        case MOTOR_VAR_TYPE_CONFIG_PID:             _Motor_Var_ConfigPid_Set(&p_motor->P_MOTOR->Config, varId, varValue);          break;
        case MOTOR_VAR_TYPE_CONFIG_DEBUG:           break;
        // case MOTOR_VAR_TYPE_CONFIG_RESV:         Motor_Sensor_CalibrationCmd_Call(p_motor, (RotorSensor_Id_T)varId, varValue);           break;
        default: break;
    }
}

/******************************************************************************/
/*
    [VarType_SubModule]
*/
/******************************************************************************/
#include "Motor/Motor/Math/FOC_Sensorless.h"

int Motor_VarType_SubModule_Get(Motor_T * p_motor, Motor_VarType_SubModule_T typeId, int varId)
{
    if (p_motor == NULL) { return 0; }
    switch (typeId)
    {
        case MOTOR_VAR_TYPE_BOARD_CONST:                return Motor_Var_Board_Get(varId);
        // case MOTOR_VAR_TYPE_PHASE:                      return Phase_VOutVar_Get( &p_motor->PHASE, varId);
        case MOTOR_VAR_TYPE_PHASE_INPUT:                return Phase_Input_Var_Get((Phase_Input_T *)&p_motor->P_MOTOR->PhaseInput, varId);
        case MOTOR_VAR_TYPE_ROTOR_OUT:                  return _Motor_Var_Rotor_Get(p_motor->P_MOTOR->p_ActiveSensor, varId);
        case MOTOR_VAR_TYPE_HEAT_MONITOR_OUT:           return HeatMonitor_VarId_Get(&p_motor->HEAT_MONITOR, varId);
        case MOTOR_VAR_TYPE_HEAT_MONITOR_CONFIG:        return HeatMonitor_ConfigId_Get(&p_motor->HEAT_MONITOR, varId);
        case MOTOR_VAR_TYPE_THERMISTOR_CONFIG:          return HeatMonitor_Thermistor_ConfigId_Get(&p_motor->HEAT_MONITOR, varId);
        case MOTOR_VAR_TYPE_PID_TUNING_IO:              return _Motor_Var_PidTuning_Get(p_motor, varId);

        case MOTOR_VAR_TYPE_FOC_OUT:                    return FOC_Var_Get(&p_motor->P_MOTOR->Foc, varId);
        case MOTOR_VAR_TYPE_FOC_CONFIG:                 return FOC_Config_Get(&p_motor->P_MOTOR->Foc.Config, varId);
#if defined(MOTOR_SENSOR_SENSORLESS_ENABLE)
        /* Read the table entry, not p_ActiveSensor — observer telemetry stays visible while tuning against another selected sensor. */
        case MOTOR_VAR_TYPE_FOC_SENSORLESS:             return FOC_Sensorless_GetVar(p_motor->SENSOR_TABLE.SENSORLESS.P_OBSERVER, varId);
        case MOTOR_VAR_TYPE_FOC_SENSORLESS_CONFIG:      return FOC_SensorlessConfig_Get(&p_motor->SENSOR_TABLE.SENSORLESS.P_OBSERVER->Config, varId);
#endif
        default: break;
    }
    return 0;
}

void Motor_VarType_SubModule_Set(Motor_T * p_motor, Motor_VarType_SubModule_T typeId, int varId, int varValue)
{
    switch (typeId)
    {
        case MOTOR_VAR_TYPE_BOARD_CONST:                break;
        case MOTOR_VAR_TYPE_PHASE:                      break;
        case MOTOR_VAR_TYPE_PHASE_INPUT:                break;
        case MOTOR_VAR_TYPE_FOC_OUT:                    break;
        case MOTOR_VAR_TYPE_FOC_CONFIG:                 FOC_Config_Set(&p_motor->P_MOTOR->Foc.Config, varId, varValue);          break;
        case MOTOR_VAR_TYPE_ROTOR_OUT:                  break;
        case MOTOR_VAR_TYPE_HEAT_MONITOR_OUT:           break;
        case MOTOR_VAR_TYPE_HEAT_MONITOR_CONFIG:        HeatMonitor_ConfigId_Set(&p_motor->HEAT_MONITOR, varId, varValue);              break;
        case MOTOR_VAR_TYPE_THERMISTOR_CONFIG:          HeatMonitor_Thermistor_ConfigId_Set(&p_motor->HEAT_MONITOR, varId, varValue);   break;
        case MOTOR_VAR_TYPE_PID_TUNING_IO:              _Motor_Var_PidTuning_Set(p_motor, varId, varValue);                             break;
        default: break;
    }
}



/******************************************************************************/
/*
    [VarType_Sensor]
*/
/******************************************************************************/
/*
    Include all compile time sensor options
*/
#include "Motor/Motor/Sensor/Motor_Sensor.h"

int Motor_VarType_Sensor_Get(Motor_T * p_motor, Motor_VarType_Sensor_T typeId, int varId)
{
    if (p_motor == NULL) { return 0; }
    switch (typeId)
    {
        // #if defined(MOTOR_SENSOR_HALL_ENABLE)
        case MOTOR_VAR_TYPE_HALL_STATE:     return Hall_VarId_Get(&p_motor->SENSOR_TABLE.HALL.HALL, varId);
        case MOTOR_VAR_TYPE_HALL_CONFIG:    return _Hall_ConfigId_Get(&p_motor->SENSOR_TABLE.HALL.HALL.P_STATE->Config, varId);
        case MOTOR_VAR_TYPE_HALL_CMD:       return 0;
        // #endif
        #if defined(MOTOR_SENSOR_ENCODER_ENABLE)
        case MOTOR_VAR_TYPE_ENCODER_STATE:  return Encoder_VarId_Get(p_motor->SENSOR_TABLE.ENCODER.ENCODER.P_STATE, varId);
        case MOTOR_VAR_TYPE_ENCODER_CONFIG: return _Encoder_ConfigId_Get(&p_motor->SENSOR_TABLE.ENCODER.ENCODER.P_STATE->Config, varId);
        #endif
        default: return 0;
    }
}

void Motor_VarType_Sensor_Set(Motor_T * p_motor, Motor_VarType_Sensor_T typeId, int varId, int varValue)
{
    switch (typeId)
    {
        case MOTOR_VAR_TYPE_HALL_CONFIG:      _Hall_ConfigId_Set(&p_motor->SENSOR_TABLE.HALL.HALL.P_STATE->Config, varId, varValue);        break;
        case MOTOR_VAR_TYPE_HALL_CMD:         Motor_Hall_Cmd(p_motor, varId, varValue);                                                     break;
        case MOTOR_VAR_TYPE_HALL_STATE:                  break;
        #if defined(MOTOR_SENSOR_ENCODER_ENABLE)
        case MOTOR_VAR_TYPE_ENCODER_CONFIG:   _Encoder_ConfigId_Set(&p_motor->SENSOR_TABLE.ENCODER.ENCODER.P_STATE->Config, varId, varValue);   break;
        case MOTOR_VAR_TYPE_ENCODER_CMD:                 break;
        case MOTOR_VAR_TYPE_ENCODER_STATE:               break;
        #endif
        default: break;
    }
}

/******************************************************************************/
/*
    [MotVarId_T] Entry
    Caller resolves the instance; every Motor Prefix shares that one [Motor_T].
*/
/******************************************************************************/
int Motor_MotVar_Get(Motor_T * p_motor, MotVarId_T varId)
{
    switch ((MotVarId_Prefix_T)varId.Prefix)
    {
        case MOT_VAR_ID_PREFIX_MOTOR:            return Motor_VarType_Base_Get(p_motor, (Motor_VarType_Base_T)varId.Type, varId.Base);
        case MOT_VAR_ID_PREFIX_MOTOR_SUB_MODULE: return Motor_VarType_SubModule_Get(p_motor, (Motor_VarType_SubModule_T)varId.Type, varId.Base);
        case MOT_VAR_ID_PREFIX_MOTOR_SENSOR:     return Motor_VarType_Sensor_Get(p_motor, (Motor_VarType_Sensor_T)varId.Type, varId.Base);
        default: return 0;
    }
}

/* Caller handles access control */
MotVarId_Status_T Motor_MotVar_Set(Motor_T * p_motor, MotVarId_T varId, int varValue)
{
    if (p_motor == NULL) { return MOT_VAR_STATUS_ERROR_INVALID_ID; }

    switch ((MotVarId_Prefix_T)varId.Prefix)
    {
        case MOT_VAR_ID_PREFIX_MOTOR:            Motor_VarType_Base_Set(p_motor, (Motor_VarType_Base_T)varId.Type, varId.Base, varValue);           break;
        case MOT_VAR_ID_PREFIX_MOTOR_SUB_MODULE: Motor_VarType_SubModule_Set(p_motor, (Motor_VarType_SubModule_T)varId.Type, varId.Base, varValue); break;
        case MOT_VAR_ID_PREFIX_MOTOR_SENSOR:     Motor_VarType_Sensor_Set(p_motor, (Motor_VarType_Sensor_T)varId.Type, varId.Base, varValue);      break;
        default: return MOT_VAR_STATUS_ERROR_INVALID_ID;
    }
    return MOT_VAR_STATUS_OK;
}


/******************************************************************************/
/*
    Write Admission - optional, not wired
    Keyed on (Prefix, Type) so the three Motor Prefixs share one switch.
    Grouped by outcome - this is the TEST_SET column of a descriptor row.
*/
/******************************************************************************/
static bool Motor_MotVar_CheckSet(Motor_T * p_motor, MotVarId_T varId)
{
    if (p_motor == NULL) { return false; }
    switch (MOT_VAR_ID_TYPE_ID(varId.Prefix, varId.Type))
    {
        /* Read-only - no setter */
        case MOT_VAR_ID_TYPE_ID(MOT_VAR_ID_PREFIX_MOTOR, MOTOR_VAR_TYPE_USER_OUT):
        case MOT_VAR_ID_TYPE_ID(MOT_VAR_ID_PREFIX_MOTOR, MOTOR_VAR_TYPE_CONFIG_DEBUG):
        case MOT_VAR_ID_TYPE_ID(MOT_VAR_ID_PREFIX_MOTOR_SUB_MODULE, MOTOR_VAR_TYPE_ROTOR_OUT):
        case MOT_VAR_ID_TYPE_ID(MOT_VAR_ID_PREFIX_MOTOR_SUB_MODULE, MOTOR_VAR_TYPE_FOC_OUT):
        case MOT_VAR_ID_TYPE_ID(MOT_VAR_ID_PREFIX_MOTOR_SUB_MODULE, MOTOR_VAR_TYPE_HEAT_MONITOR_OUT):
        case MOT_VAR_ID_TYPE_ID(MOT_VAR_ID_PREFIX_MOTOR_SUB_MODULE, MOTOR_VAR_TYPE_BOARD_CONST):
        case MOT_VAR_ID_TYPE_ID(MOT_VAR_ID_PREFIX_MOTOR_SUB_MODULE, MOTOR_VAR_TYPE_PHASE):
        case MOT_VAR_ID_TYPE_ID(MOT_VAR_ID_PREFIX_MOTOR_SENSOR, MOTOR_VAR_TYPE_HALL_STATE):
        case MOT_VAR_ID_TYPE_ID(MOT_VAR_ID_PREFIX_MOTOR_SENSOR, MOTOR_VAR_TYPE_ENCODER_STATE):
            return false;

        /* Unrestricted - Cmd/Setpoint IO */
        case MOT_VAR_ID_TYPE_ID(MOT_VAR_ID_PREFIX_MOTOR, MOTOR_VAR_TYPE_USER_CONTROL):
        case MOT_VAR_ID_TYPE_ID(MOT_VAR_ID_PREFIX_MOTOR, MOTOR_VAR_TYPE_USER_SETPOINT):
        case MOT_VAR_ID_TYPE_ID(MOT_VAR_ID_PREFIX_MOTOR, MOTOR_VAR_TYPE_STATE_CMD):
        case MOT_VAR_ID_TYPE_ID(MOT_VAR_ID_PREFIX_MOTOR, MOTOR_VAR_TYPE_OPEN_LOOP_CMD):
            return true;

        /* Config state */
        case MOT_VAR_ID_TYPE_ID(MOT_VAR_ID_PREFIX_MOTOR, MOTOR_VAR_TYPE_CALIBRATION_CMD):
        case MOT_VAR_ID_TYPE_ID(MOT_VAR_ID_PREFIX_MOTOR, MOTOR_VAR_TYPE_CONFIG_CALIBRATION):
        case MOT_VAR_ID_TYPE_ID(MOT_VAR_ID_PREFIX_MOTOR, MOTOR_VAR_TYPE_CONFIG_ACTUATION):
        case MOT_VAR_ID_TYPE_ID(MOT_VAR_ID_PREFIX_MOTOR, MOTOR_VAR_TYPE_CONFIG_PID):
        case MOT_VAR_ID_TYPE_ID(MOT_VAR_ID_PREFIX_MOTOR, MOTOR_VAR_TYPE_CONFIG_RESV):
        case MOT_VAR_ID_TYPE_ID(MOT_VAR_ID_PREFIX_MOTOR_SUB_MODULE, MOTOR_VAR_TYPE_HEAT_MONITOR_CONFIG):
        case MOT_VAR_ID_TYPE_ID(MOT_VAR_ID_PREFIX_MOTOR_SUB_MODULE, MOTOR_VAR_TYPE_THERMISTOR_CONFIG):
        case MOT_VAR_ID_TYPE_ID(MOT_VAR_ID_PREFIX_MOTOR_SENSOR, MOTOR_VAR_TYPE_HALL_CONFIG):
        case MOT_VAR_ID_TYPE_ID(MOT_VAR_ID_PREFIX_MOTOR_SENSOR, MOTOR_VAR_TYPE_ENCODER_CONFIG):
            // return Motor_IsConfig(p_motor);

        /* Calibration state */
        case MOT_VAR_ID_TYPE_ID(MOT_VAR_ID_PREFIX_MOTOR_SENSOR, MOTOR_VAR_TYPE_HALL_CMD):
        case MOT_VAR_ID_TYPE_ID(MOT_VAR_ID_PREFIX_MOTOR_SENSOR, MOTOR_VAR_TYPE_ENCODER_CMD):
            // return Motor_IsState(p_motor, &MOTOR_STATE_CALIBRATION);

        /* Run state */
        case MOT_VAR_ID_TYPE_ID(MOT_VAR_ID_PREFIX_MOTOR_SUB_MODULE, MOTOR_VAR_TYPE_PID_TUNING_IO):
            // return Motor_IsState(p_motor, &MOTOR_STATE_RUN);

        default: return false;
    }
}
