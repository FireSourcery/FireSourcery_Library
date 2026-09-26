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
    @file   Motor_Var.c
    @author FireSourcery
    @brief
*/
/******************************************************************************/
#include "Motor_Var.h"

#include "Analog/Motor_Analog.h" /* for calibration cmd */
#include "Motor_User.h"
#include "Motor_Config.h"
#include "Motor.h"


/******************************************************************************/
/*
    Var Runtime telemetry
*/
/******************************************************************************/
int _Motor_Var_UserOut_Get(Motor_T * p_motor, Motor_Var_UserOut_T varId)
{
    const Motor_Context_T * p_state = p_motor->P_MOTOR;
    int value = 0;
    switch (varId)
    {
        case MOTOR_VAR_SPEED:                       value = Motor_User_GetSpeed_Fract16(p_state);           break;
        case MOTOR_VAR_I_PHASE:                     value = Motor_GetIPhase_Fract16(p_state);               break;
        case MOTOR_VAR_V_PHASE:                     value = Motor_GetVPhase_Fract16(p_state);               break;
        case MOTOR_VAR_STATE:                       value = Motor_GetStateId(p_state);                      break;
        case MOTOR_VAR_SUB_STATE:                   value = Motor_GetPathId(p_state);                       break;
        case MOTOR_VAR_FAULT_FLAGS:                 value = Motor_GetFaultFlags(p_state).Value;             break;
        case MOTOR_VAR_HEAT:                        value = Motor_GetHeat_Adcu(p_state);                    break;
        case MOTOR_VAR_SPEED_REQ:                   value = Motor_GetSpeedSetpoint(p_state);                break;
        case MOTOR_VAR_TORQUE_I_REQ:                value = Motor_GetISetpoint(p_state);                    break;
        case MOTOR_VAR_TORQUE_V_REQ:                value = Motor_GetVSetpoint(p_state);                    break;
        case MOTOR_VAR_V_SPEED_EFFECTIVE:           value = Motor_User_GetVSpeed_Fract16(p_motor);          break;
        case MOTOR_VAR_POWER:                       value = Motor_GetElectricalPower_Fract16(p_state);      break;
        case MOTOR_VAR_I_BUS:                       value = Motor_GetIBus_Fract16(p_motor);                 break;
        default: break;
    }
    return value;
}


/******************************************************************************/
/*
    Input/Cmds
    Full context
*/
/******************************************************************************/
int _Motor_Var_UserControl_Get(Motor_T * p_motor, Motor_Var_UserControl_T varId)
{
    const Motor_Context_T * p_state = p_motor->P_MOTOR;
    int value = 0;
    switch (varId)
    {
        case MOTOR_VAR_USER_DIRECTION:          value = Motor_GetUserDirection(p_state);                    break;
        case MOTOR_VAR_USER_FEEDBACK_MODE:      value = Motor_GetFeedbackMode(p_state).Value;               break;
        case MOTOR_VAR_USER_PHASE_OUTPUT:       value = Motor_GetPhaseState(p_motor);                       break;

        case MOTOR_VAR_USER_SPEED_LIMIT:        value = Motor_User_SpeedLimit(p_state);                     break;
        case MOTOR_VAR_USER_I_LIMIT_MOTORING:   value = Motor_User_ILimitMotoring(p_state);                 break;
        case MOTOR_VAR_USER_I_LIMIT_GENERATING: value = Motor_User_ILimitGenerating(p_state);               break;
        default: break;
        /* RAMP_ON_OFF */
    }
    return value;
}

void _Motor_Var_UserControl_Set(Motor_T * p_motor, Motor_Var_UserControl_T varId, int varValue)
{
    switch (varId)
    {
        /* todo cast enum bounds */
        case MOTOR_VAR_USER_DIRECTION:          Motor_ApplyUserDirection(p_motor, (Motor_Direction_T)varValue);         break;
        case MOTOR_VAR_USER_FEEDBACK_MODE:      Motor_ApplyFeedbackMode(p_motor, Motor_FeedbackMode_Cast(varValue));    break;
        case MOTOR_VAR_USER_PHASE_OUTPUT:       Motor_ApplyControlState(p_motor, (Phase_VOutMode_T)varValue);             break;
        /* if local user Set user index */
        // case MOTOR_VAR_USER_SPEED_LIMIT:         Motor_TrySpeedLimit(p_motor->P_MOTOR, varValue);                  break;
        // case MOTOR_VAR_USER_I_LIMIT_MOTORING:    Motor_TryILimit(p_motor->P_MOTOR, varValue);                      break;
        default: break;
    }
}


/* caller handle switch logic */
void _Motor_Var_UserSetpoint_Set(Motor_T * p_motor, Motor_Var_UserSetpoint_T varId, int varValue)
{
    switch (varId)
    {
        case MOTOR_VAR_USER_SETPOINT_SCALAR:      Motor_SetActiveCmd_Norm(p_motor, varValue);    break;
        case MOTOR_VAR_USER_SETPOINT_SPEED:       Motor_SetSpeedCmd(p_motor->P_MOTOR, varValue);           break;
        case MOTOR_VAR_USER_SETPOINT_TORQUE:      Motor_SetTorqueCmd(p_motor->P_MOTOR, varValue);          break;
        case MOTOR_VAR_USER_SETPOINT_CURRENT:     Motor_SetICmd(p_motor->P_MOTOR, varValue);               break;
        case MOTOR_VAR_USER_SETPOINT_VOLTAGE:     Motor_SetVoltageCmd(p_motor->P_MOTOR, varValue);         break;
        // case MOTOR_VAR_USER_SETPOINT_ANGLE:    Motor_SetPositionCmd(p_motor, varValue);        break;
        default: break;
    }
}


/******************************************************************************/
/*
    Cmds
*/
/******************************************************************************/
void _Motor_Var_StateCmd_Call(Motor_T * p_motor, Motor_Var_StateCmd_T varId, int varValue)
{
    switch (varId)
    {
        case MOTOR_VAR_CLEAR_FAULT:             Motor_StateMachine_ClearFault(p_motor, (Motor_FaultFlags_T) { .Value = varValue });  break;
        case MOTOR_VAR_FORCE_DISABLE_CONTROL:   Motor_ForceDisableControl(p_motor);                                                  break;
        case MOTOR_VAR_USER_ENABLE:             Motor_Enable(p_motor);                                                                 break;
        case MOTOR_VAR_USER_DISABLE:            Motor_Disable(p_motor);                                                                break;
        default: break;
    }
}

void _Motor_Var_OpenLoopCmd_Call(Motor_T * p_motor, Motor_Var_OpenLoopCmd_T varId, int varValue)
{
    switch (varId)
    {

        case MOTOR_VAR_OPEN_LOOP_ENTER:         Motor_OpenLoop_Enter(p_motor);                                          break;
        case MOTOR_VAR_OPEN_LOOP_PHASE_OUTPUT:  Motor_OpenLoop_SetPhaseOutput(p_motor, (Phase_VOutMode_T)varValue);     break;
        // case MOTOR_VAR_OPEN_LOOP_PHASE_ALIGN:   Motor_OpenLoop_SetPhaseAlign(p_motor, (Phase_Id_T)varValue);        break;
        case MOTOR_VAR_OPEN_LOOP_ANGLE_ALIGN:   Motor_OpenLoop_SetAngleAlign(p_motor, varValue);                        break;
        case MOTOR_VAR_OPEN_LOOP_JOG:           Motor_OpenLoop_SetJog(p_motor, varValue);                               break;
        case MOTOR_VAR_OPEN_LOOP_RUN:           Motor_OpenLoop_StartRunChain(p_motor);                                  break;
        default: break;
            // case MOTOR_VAR_OPEN_LOOP_HOMING:     break;
    }
}

void _Motor_Var_CalibrationCmd_Call(Motor_T * p_motor, Motor_Var_CalibrationCmd_T varId, int varValue)
{
    (void)varValue; /* some cmds may use varValue as param */
    switch (varId)
    {
        case MOTOR_VAR_CALIBRATION_ENTER:               Motor_Calibration_Enter(p_motor);       break;
        case MOTOR_VAR_CALIBRATION_CMD_ADC:             Motor_Analog_Calibrate(p_motor);        break;
        case MOTOR_VAR_CALIBRATION_CMD_VIRTUAL_HOME:    Motor_Calibration_StartHome(p_motor);   break;
        case MOTOR_VAR_CALIBRATION_CMD_ELECTRICAL:      Motor_Calibration_StartElectrical(p_motor);   break;
        // case MOTOR_VAR_CALIBRATION_CMD_PID_TUNING:      Motor_Calibration_StartPidTuning(p_motor);    break;
        default: break;
    }
}


/******************************************************************************/
/*
    Runtime Tuning version
*/
/******************************************************************************/
int _Motor_Var_PidTuning_Get(Motor_T * p_motor, Motor_Var_ConfigPid_T varId)
{
    const Motor_Context_T * p_state = p_motor->P_MOTOR;
    int value = 0;
    switch (varId)
    {
        case MOTOR_VAR_PID_SPEED_SAMPLE_FREQ:     value = PID_GetSampleFreq(&p_state->PidSpeed);    break;
        case MOTOR_VAR_PID_SPEED_KP:              value = PID_GetKp_Fixed32(&p_state->PidSpeed);    break; // alternatively _PID_GetKp_Runtime
        case MOTOR_VAR_PID_SPEED_KI:              value = PID_GetKi_Fixed32(&p_state->PidSpeed);    break;
        case MOTOR_VAR_PID_CURRENT_SAMPLE_FREQ:   value = PID_GetSampleFreq(&p_state->Foc.PidIq);   break;
        case MOTOR_VAR_PID_CURRENT_KP:            value = PID_GetKp_Fixed32(&p_state->Foc.PidIq);   break;
        case MOTOR_VAR_PID_CURRENT_KI:            value = PID_GetKi_Fixed32(&p_state->Foc.PidIq);   break;
        default: break;
    }
    return value;
}

/* Sets runtime and config */
void _Motor_Var_PidTuning_Set(Motor_T * p_motor, Motor_Var_ConfigPid_T varId, int varValue)
{
    Motor_Context_T * p_state = p_motor->P_MOTOR;
    switch (varId)
    {
        case MOTOR_VAR_PID_SPEED_SAMPLE_FREQ: break;
        case MOTOR_VAR_PID_SPEED_KP:          _Motor_Tuning_SetSpeedKp(p_state, varValue);    break;
        case MOTOR_VAR_PID_SPEED_KI:          _Motor_Tuning_SetSpeedKi(p_state, varValue);    break;
        case MOTOR_VAR_PID_CURRENT_SAMPLE_FREQ: break;
        case MOTOR_VAR_PID_CURRENT_KP:       _Motor_Tuning_SetIKp(p_state, varValue); break;
        case MOTOR_VAR_PID_CURRENT_KI:       _Motor_Tuning_SetIKi(p_state, varValue); break;
        default: break;
    }
}

// int _Motor_Var_PidTuning_Get_Fixed16(Motor_T * p_motor, Motor_Var_ConfigPid_T varId)
// {
//     const Motor_Context_T * p_state = p_motor->P_MOTOR;
//     int value = 0;
//     switch (varId)
//     {
//         case MOTOR_VAR_PID_SPEED_SAMPLE_FREQ:     value = PID_GetSampleFreq(&p_state->PidSpeed);    break;
//         case MOTOR_VAR_PID_SPEED_KP:              value = PID_GetKp_Fixed16(&p_state->PidSpeed);    break;
//         case MOTOR_VAR_PID_SPEED_KI:              value = PID_GetKi_Fixed16(&p_state->PidSpeed);    break;
//         case MOTOR_VAR_PID_CURRENT_SAMPLE_FREQ:   value = PID_GetSampleFreq(&p_state->Foc.PidIq);   break;
//         case MOTOR_VAR_PID_CURRENT_KP:            value = PID_GetKp_Fixed16(&p_state->Foc.PidIq);   break;
//         case MOTOR_VAR_PID_CURRENT_KI:            value = PID_GetKi_Fixed16(&p_state->Foc.PidIq);   break;
//         default: break;
//     }
//     return value;
// }

// void _Motor_Var_PidTuning_Set_Fixed16(Motor_T * p_motor, Motor_Var_ConfigPid_T varId, int varValue)
// {
//     Motor_Context_T * p_state = p_motor->P_MOTOR;
//     switch (varId)
//     {
//         case MOTOR_VAR_PID_SPEED_SAMPLE_FREQ: break;
//         case MOTOR_VAR_PID_SPEED_KP:          _Motor_Tuning_SetSpeedKp_Fixed16(p_state, varValue);    break;
//         case MOTOR_VAR_PID_SPEED_KI:          _Motor_Tuning_SetSpeedKi_Fixed16(p_state, varValue);    break;
//         case MOTOR_VAR_PID_CURRENT_SAMPLE_FREQ: break;
//         case MOTOR_VAR_PID_CURRENT_KP:       _Motor_Tuning_SetIKp_Fixed16(p_state, varValue); break;
//         case MOTOR_VAR_PID_CURRENT_KI:       _Motor_Tuning_SetIKi_Fixed16(p_state, varValue); break;
//         default: break;
//     }
// }


/******************************************************************************/
/*

*/
/******************************************************************************/
/*
    const
    Alternate access to board reference values
*/
int Motor_Var_Board_Get(Motor_Var_Board_T varId)
{
    int value = 0;
    switch (varId)
    {
        case MOTOR_VAR_BOARD_V_RATED:                 value = Phase_Calibration_GetVRated_Fract16();                 break;
        case MOTOR_VAR_BOARD_I_RATED:                 value = Phase_Calibration_GetIRatedPeak_Fract16();             break;
        case MOTOR_VAR_BOARD_V_MAX:                   value = Phase_Calibration_GetVMaxVolts();                      break;
        case MOTOR_VAR_BOARD_I_MAX:                   value = Phase_Calibration_GetIMaxAmps();                       break;
        case MOTOR_VAR_BOARD_V_PHASE_R1:              value = PHASE_ANALOG_BOARD.V_PHASE_R1;            break;
        case MOTOR_VAR_BOARD_V_PHASE_R2:              value = PHASE_ANALOG_BOARD.V_PHASE_R2;            break;
        case MOTOR_VAR_BOARD_I_PHASE_R_BASE:          value = PHASE_ANALOG_BOARD.I_PHASE_R_BASE;        break;
        case MOTOR_VAR_BOARD_I_PHASE_R_MOSFETS:       value = PHASE_ANALOG_BOARD.I_PHASE_R_MOSFETS;     break;
        case MOTOR_VAR_BOARD_I_PHASE_R_SHUNT:         value = PHASE_ANALOG_BOARD.I_PHASE_R_SHUNT;       break;
        case MOTOR_VAR_BOARD_I_PHASE_GAIN:            value = PHASE_ANALOG_BOARD.I_PHASE_GAIN;          break;
        // case MOTOR_VAR_BOARD_CONTROL_FREQ:                 value =                   break;
        /* Precompile Options */
        case MOTOR_VAR_BOARD_ROTOR_SENSOR_OPTION:     value = ROTOR_SENSOR_ENABLED.ALL;                        break;
        // case MOTOR_VAR_BOARD_VERSION_FLAGS:           value = MOTOR_VERSION_FLAGS.Value;                        break;
        default: break;
    }
    return value;
}


/******************************************************************************/
/*

*/
/******************************************************************************/
//diagnostics
int _Motor_Var_ConfigDebug_Get(const Motor_T * p_motor, Motor_Var_ConfigDebug_T varId)
{
    int value = 0;
    switch (varId)
    {
        case MOTOR_VAR_SPEED_RATED_RPM:                 value = Motor_SpeedRated_Rpm(p_motor);             break;
        case MOTOR_VAR_SPEED_V_REF_RPM:                 value = Motor_GetSpeedVNominalRef_Rpm(p_motor);              break;
        case MOTOR_VAR_SPEED_V_REF_DEG_PER_CYCLE:       value = Motor_GetSpeedVNominalRef_Angle(p_motor);             break;
        // case MOTOR_VAR_SPEED_V_MATCH_REF_RPM:           value = Motor_Config_GetSpeedVMatchRef_Rpm(p_motor);         break;
        case MOTOR_VAR_V_SPEED_RATED_FRACT16:           value = Motor_SpeedRated_Fract16(p_motor);               break;
        default: break;
    }
    return value;
}



/******************************************************************************/
/*
    Cross module
*/
/******************************************************************************/
static inline int Motor_FocConfig_GetSi(Motor_T * p_motor, FOC_ConfigId_T var)
{
    FOC_Config_T * p_config = &p_motor->P_MOTOR->Foc.Config;
    switch (var)
    {
        case FOC_CONFIG_FW_ID_LIMIT:    return p_config->FieldWeakening.IdLimit;
        case FOC_CONFIG_FW_ID_GAIN:     return p_config->FieldWeakening.IdGain;
        case FOC_CONFIG_ELECTRICAL_LD:  return l_h_of_pu_rpm(Phase_Calibration_GetVMaxVolts(), Phase_Calibration_GetIMaxAmps(), Motor_SpeedTypeMax_Rpm(p_motor), p_motor->P_MOTOR->Config.SpeedRating.PolePairs, p_config->Electrical.Ld, 1000000UL);
        case FOC_CONFIG_ELECTRICAL_LQ:  return l_h_of_pu_rpm(Phase_Calibration_GetVMaxVolts(), Phase_Calibration_GetIMaxAmps(), Motor_SpeedTypeMax_Rpm(p_motor), p_motor->P_MOTOR->Config.SpeedRating.PolePairs, p_config->Electrical.Lq, 1000000UL);
        case FOC_CONFIG_ELECTRICAL_RS:  return rs_mohm_of_pu(Phase_Calibration_GetVMaxVolts(), Phase_Calibration_GetIMaxAmps(), p_config->Electrical.Rs);
        case FOC_CONFIG_ELECTRICAL_PSI: return psi_wb_of_pu_rads(Phase_Calibration_GetVMaxVolts(), Motor_SpeedTypeMax_Rpm(p_motor), p_config->Electrical.Psi, 1000000UL);
        default: return 0;
    }
}

static inline void Motor_FocConfig_SetSi(Motor_T * p_motor, FOC_ConfigId_T var, int value)
{
    FOC_Config_T * p_config = &p_motor->P_MOTOR->Foc.Config;

    switch (var)
    {
        case FOC_CONFIG_FW_ID_LIMIT:    p_config->FieldWeakening.IdLimit = value;        break;
        case FOC_CONFIG_FW_ID_GAIN:     p_config->FieldWeakening.IdGain = value;         break;
        case FOC_CONFIG_ELECTRICAL_LD:  p_config->Electrical.Ld = l_pu_rpm_of_h(Phase_Calibration_GetVMaxVolts(), Phase_Calibration_GetIMaxAmps(), Motor_SpeedTypeMax_Rpm(p_motor), p_motor->P_MOTOR->Config.SpeedRating.PolePairs, value, 1000000UL);    break;
        case FOC_CONFIG_ELECTRICAL_LQ:  p_config->Electrical.Lq = l_pu_rpm_of_h(Phase_Calibration_GetVMaxVolts(), Phase_Calibration_GetIMaxAmps(), Motor_SpeedTypeMax_Rpm(p_motor), p_motor->P_MOTOR->Config.SpeedRating.PolePairs, value, 1000000UL);    break;
        case FOC_CONFIG_ELECTRICAL_RS:  p_config->Electrical.Rs = rs_pu_of_mohm(Phase_Calibration_GetVMaxVolts(), Phase_Calibration_GetIMaxAmps(), value);    break;
        case FOC_CONFIG_ELECTRICAL_PSI: p_config->Electrical.Psi = psi_pu_rads_of_wb(Phase_Calibration_GetVMaxVolts(), Motor_SpeedTypeMax_Rpm(p_motor), value, 1000000UL);   break;
        default: break;
    }
}


