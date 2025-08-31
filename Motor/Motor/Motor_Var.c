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

#include "Sensor/Motor_Sensor.h" /* for calibration cmd */
#include "Analog/Motor_Analog.h" /* for calibration cmd */
#include "Motor_User.h"
#include "Motor_Config.h"
#include "Motor.h"


/******************************************************************************/
/*
    Var RealTime/RunTime
*/
/******************************************************************************/
int _Motor_Var_UserOut_Get(const Motor_State_T * p_motor, Motor_Var_UserOut_T varId)
{
    int value = 0;
    switch (varId)
    {
        case MOTOR_VAR_SPEED:                       value = Motor_User_GetSpeed_UFract16(p_motor);              break;
        case MOTOR_VAR_I_PHASE:                     value = Motor_User_GetIPhase_UFract16(p_motor);             break;
        case MOTOR_VAR_V_PHASE:                     value = Motor_User_GetVPhase_UFract16(p_motor);             break;
        case MOTOR_VAR_STATE:                       value = Motor_User_GetStateId(p_motor);                     break;
        case MOTOR_VAR_SUB_STATE:                   value = Motor_User_GetSubStateId(p_motor);                  break;
        case MOTOR_VAR_FAULT_FLAGS:                 value = Motor_User_GetFaultFlags(p_motor).Value;            break;
        // case MOTOR_VAR_STATUS_FLAGS:                value = Motor_User_GetStatusFlags(p_motor).Value;           break;
        case MOTOR_VAR_HEAT:                        value = Motor_User_GetHeat_Adcu(p_motor);                   break;
        case MOTOR_VAR_V_SPEED_EFFECTIVE:           value = Motor_User_GetVSpeedEffective_Fract16(p_motor);     break;
        case MOTOR_VAR_POWER:                       value = Motor_User_GetElectricalPower_UFract16(p_motor);    break;
        case MOTOR_VAR_I_DC:                        value = Motor_User_GetIdc_UFract16(p_motor);                break;
    }
    return value;
}

int _Motor_Var_Rotor_Get(const Motor_State_T * p_motor, Motor_Var_Rotor_T varId)
{
    switch (varId)
    {
        case MOTOR_VAR_ROTOR_ELECTRICAL_ANGLE:   return _RotorSensor_GetElecticalAngle(&p_motor->SensorState);
        case MOTOR_VAR_ROTOR_ELECTRICAL_SPEED:   return _RotorSensor_GetElectricalSpeed(&p_motor->SensorState);
        case MOTOR_VAR_ROTOR_SPEED_FRACT16:      return _RotorSensor_GetSpeed_Fract16(&p_motor->SensorState);
        case MOTOR_VAR_ROTOR_MECHANICAL_ANGLE:   return _RotorSensor_GetMechanicalAngle(&p_motor->SensorState);
        case MOTOR_VAR_ROTOR_DIRECTION:          return _RotorSensor_GetDirection(&p_motor->SensorState);
        case MOTOR_VAR_ROTOR_SPEED_REQ:          return Ramp_GetOutput(&p_motor->SpeedRamp); /* Speed Setpoint */
        default: return 0;
    }
}

int _Motor_Var_Foc_Get(const Motor_State_T * p_motor, Motor_Var_Foc_T varId)
{
    int value = 0;
    switch (varId)
    {
        case MOTOR_VAR_FOC_IA:      value = p_motor->PhaseInput.Iabc.A;        break;
        case MOTOR_VAR_FOC_IB:      value = p_motor->PhaseInput.Iabc.B;        break;
        case MOTOR_VAR_FOC_IC:      value = p_motor->PhaseInput.Iabc.C;        break;
        case MOTOR_VAR_FOC_ID:      value = p_motor->Foc.Id;        break;
        case MOTOR_VAR_FOC_IQ:      value = p_motor->Foc.Iq;        break;
        case MOTOR_VAR_FOC_VD:      value = p_motor->Foc.Vd;        break;
        case MOTOR_VAR_FOC_VQ:      value = p_motor->Foc.Vq;        break;
        case MOTOR_VAR_FOC_VA:      value = p_motor->Foc.Va;        break;
        case MOTOR_VAR_FOC_VB:      value = p_motor->Foc.Vb;        break;
        case MOTOR_VAR_FOC_VC:      value = p_motor->Foc.Vc;        break;
        // case MOTOR_VAR_FOC_REQ_D:   value = p_motor->Foc.ReqD;                          break;
        case MOTOR_VAR_FOC_REQ_Q:   value = Ramp_GetOutput(&p_motor->TorqueRamp);          break;
        case MOTOR_VAR_FOC_INTEGRAL_D:    value = PID_GetIntegral(&p_motor->PidId);        break;
        case MOTOR_VAR_FOC_INTEGRAL_Q:    value = PID_GetIntegral(&p_motor->PidIq);        break;
    }
    return value;
}

/******************************************************************************/
/*
    IO
*/
/******************************************************************************/
/******************************************************************************/
/*
    Input/Cmds
    Full context
*/
/******************************************************************************/
int _Motor_Var_UserControl_Get(const Motor_T * p_motor, Motor_Var_UserControl_T varId)
{
    int value = 0;
    switch (varId)
    {
        case MOTOR_VAR_USER_DIRECTION:          value = Motor_User_GetDirection(p_motor->P_MOTOR_STATE);                break;
        case MOTOR_VAR_USER_ROTARY_DIRECTION:   value = Motor_User_GetRotaryDirection(p_motor->P_MOTOR_STATE);          break;
        case MOTOR_VAR_USER_FEEDBACK_MODE:      value = Motor_User_GetFeedbackMode(p_motor->P_MOTOR_STATE).Value;       break;
        case MOTOR_VAR_USER_PHASE_OUTPUT:       value = Motor_User_GetPhaseState(p_motor);                              break;

        case MOTOR_VAR_USER_SPEED_LIMIT:        value = Motor_User_GetSpeedLimitActive(p_motor->P_MOTOR_STATE);         break;
        // case MOTOR_VAR_USER_I_LIMIT:         value = Motor_User_GetILimit(p_motor);                  break;
        case MOTOR_VAR_USER_I_LIMIT_MOTORING:   value = Motor_User_GetILimitMotoring(p_motor->P_MOTOR_STATE);           break;
        case MOTOR_VAR_USER_I_LIMIT_GENERATING: value = Motor_User_GetILimitGenerating(p_motor->P_MOTOR_STATE);         break;
        /* RAMP_ON_OFF */
    }
    return value;
}

// alternatively var interface calls use sync buffer
void _Motor_Var_UserControl_Set(const Motor_T * p_motor, Motor_Var_UserControl_T varId, int varValue)
{
    switch (varId)
    {
        /* Calls to StateMachine use full context */
        /* todo StateMachine validates input */
        case MOTOR_VAR_USER_DIRECTION:          Motor_User_ApplyDirectionSign(p_motor, (Motor_User_Direction_T)varValue);  break;
        case MOTOR_VAR_USER_ROTARY_DIRECTION:   Motor_User_ApplyRotaryDirection(p_motor, (Motor_Direction_T)varValue);     break;
        case MOTOR_VAR_USER_FEEDBACK_MODE:      Motor_User_SetFeedbackMode_Cast(p_motor, varValue);                        break;
        case MOTOR_VAR_USER_PHASE_OUTPUT:       Motor_User_ActivatePhaseOutput(p_motor, (Phase_Output_T)varValue);         break;
        /* Set user index */
        // case MOTOR_VAR_USER_SPEED_LIMIT:         Motor_User_TrySpeedLimit(p_motor->P_MOTOR_STATE, varValue);                  break;
        // case MOTOR_VAR_USER_I_LIMIT_MOTORING:    Motor_User_TryILimit(p_motor->P_MOTOR_STATE, varValue);                      break;
    }
}

/* caller handle switch logic */
void _Motor_Var_UserSetpoint_Set(const Motor_T * p_motor, Motor_Var_UserSetpoint_T varId, int varValue)
{
    switch (varId)
    {
        case MOTOR_VAR_USER_SETPOINT:             Motor_User_SetActiveCmdValue_Scalar(p_motor->P_MOTOR_STATE, varValue);     break;
        // case MOTOR_VAR_USER_SETPOINT:             Motor_User_SetActiveCmdValue(p_motor->P_MOTOR_STATE, varValue);  break; /* mixed units */
        case MOTOR_VAR_USER_SETPOINT_SPEED:       Motor_User_SetSpeedCmd_Fract16(p_motor->P_MOTOR_STATE, varValue);                  break;
        case MOTOR_VAR_USER_SETPOINT_CURRENT:     Motor_User_SetICmd(p_motor->P_MOTOR_STATE, varValue);                      break;
        case MOTOR_VAR_USER_SETPOINT_VOLTAGE:     Motor_User_SetVoltageCmd(p_motor->P_MOTOR_STATE, varValue);                break;
        // case MOTOR_VAR_USER_SETPOINT_ANGLE:   Motor_User_SetPositionCmd(p_motor->P_MOTOR_STATE, varValue);     break;
        // case MOTOR_VAR_USER_SETPOINT_OPEN_LOOP_TORQUE: Motor_User_SetOpenLoopCmd(p_motor->P_MOTOR_STATE, varValue); break;
        default: break;
    }
}

void _Motor_Var_StateCmd_Set(const Motor_T * p_motor, Motor_Var_StateCmd_T varId, int varValue)
{
    switch (varId)
    {
        case MOTOR_VAR_CLEAR_FAULT:             Motor_StateMachine_ClearFault(p_motor, (Motor_FaultFlags_T) { .Value = varValue }); break;
        case MOTOR_VAR_FORCE_DISABLE_CONTROL:   Motor_User_ForceDisableControl(p_motor);                                            break;

        case MOTOR_VAR_OPEN_LOOP_ENTER:         Motor_OpenLoop_Enter(p_motor);                                      break;
        case MOTOR_VAR_OPEN_LOOP_PHASE_OUTPUT:  Motor_OpenLoop_SetPhaseOutput(p_motor, (Phase_Output_T)varValue);   break;
        case MOTOR_VAR_OPEN_LOOP_PHASE_ALIGN:   Motor_OpenLoop_SetPhaseAlign(p_motor, (Phase_Id_T)varValue);        break;
        case MOTOR_VAR_OPEN_LOOP_ANGLE_ALIGN:   Motor_OpenLoop_SetAngleAlign(p_motor, varValue);                    break;
        case MOTOR_VAR_OPEN_LOOP_JOG:           Motor_OpenLoop_SetJog(p_motor, varValue);                           break;
        case MOTOR_VAR_OPEN_LOOP_RUN:           Motor_OpenLoop_StartRunChain(p_motor);                              break;
            // case MOTOR_VAR_OPEN_LOOP_HOMING:     break;
    }
}
/******************************************************************************/
/*
    Config
*/
/******************************************************************************/
int _Motor_Var_ConfigCalibration_Get(const Motor_State_T * p_motor, Motor_Var_ConfigCalibration_T varId)
{
    int value = 0;
    switch (varId)
    {
        case MOTOR_VAR_COMMUTATION_MODE:        value = Motor_Config_GetCommutationMode(p_motor);           break;
        case MOTOR_VAR_SENSOR_MODE:             value = Motor_Config_GetSensorMode(p_motor);                break;
        case MOTOR_VAR_DIRECTION_CALIBRATION:   value = Motor_Config_GetDirectionCalibration(p_motor);      break;
        case MOTOR_VAR_POLE_PAIRS:              value = Motor_Config_GetPolePairs(p_motor);                 break;
        case MOTOR_VAR_KV:                      value = Motor_Config_GetKv(p_motor);                        break;
        case MOTOR_VAR_SPEED_RATED:             value = Motor_Config_GetSpeedRated(p_motor);                break;
        case MOTOR_VAR_V_SPEED_SCALAR:          value = Motor_Config_GetVSpeedScalar_UFract16(p_motor);     break;
        case MOTOR_VAR_IA_ZERO_ADCU:            value = Motor_Config_GetIaZero_Adcu(p_motor);               break;
        case MOTOR_VAR_IB_ZERO_ADCU:            value = Motor_Config_GetIbZero_Adcu(p_motor);               break;
        case MOTOR_VAR_IC_ZERO_ADCU:            value = Motor_Config_GetIcZero_Adcu(p_motor);               break;
        // case MOTOR_VAR_I_PEAK_REF_ADCU:               value = Motor_Config_GetIPeakRef_Adcu(p_motor);             break;
    }
    return value;
}

void _Motor_Var_ConfigCalibration_Set(Motor_State_T * p_motor, Motor_Var_ConfigCalibration_T varId, int varValue)
{
    switch (varId)
    {
        case MOTOR_VAR_COMMUTATION_MODE:              Motor_Config_SetCommutationMode(p_motor, varValue);           break;
        case MOTOR_VAR_SENSOR_MODE:                   Motor_Config_SetSensorMode(p_motor, varValue);                break;
        case MOTOR_VAR_DIRECTION_CALIBRATION:         Motor_Config_SetDirectionCalibration(p_motor, varValue);      break;
        case MOTOR_VAR_POLE_PAIRS:                    Motor_Config_SetPolePairs(p_motor, varValue);                 break;
        case MOTOR_VAR_KV:                            Motor_Config_SetKv(p_motor, varValue);                        break;
        case MOTOR_VAR_SPEED_RATED:               Motor_Config_SetSpeedRated(p_motor, varValue);                break;
        case MOTOR_VAR_V_SPEED_SCALAR:                Motor_Config_SetVSpeedScalar_UFract16(p_motor, varValue);     break;
        case MOTOR_VAR_IA_ZERO_ADCU:                  Motor_Config_SetIaZero_Adcu(p_motor, varValue);               break;
        case MOTOR_VAR_IB_ZERO_ADCU:                  Motor_Config_SetIbZero_Adcu(p_motor, varValue);               break;
        case MOTOR_VAR_IC_ZERO_ADCU:                  Motor_Config_SetIcZero_Adcu(p_motor, varValue);               break;
        // case MOTOR_VAR_I_PEAK_REF_ADCU:               Motor_Config_SetIPeakRef_Adcu(p_motor, varValue);             break;
    }
}

int _Motor_Var_ConfigCalibrationAlias_Get(const Motor_State_T * p_motor, Motor_Var_ConfigCalibrationAlias_T varId)
{
    int value = 0;
    switch (varId)
    {
        case MOTOR_VAR_SPEED_RATED_RPM:                 value = Motor_Config_GetSpeedRated_Rpm(p_motor);             break;
        case MOTOR_VAR_SPEED_V_REF_RPM:                 value = Motor_Config_GetSpeedVRef_Rpm(p_motor);              break;
        case MOTOR_VAR_SPEED_V_MATCH_REF_RPM:           value = Motor_Config_GetSpeedVMatchRef_Rpm(p_motor);         break;
        case MOTOR_VAR_SPEED_V_SVPWM_REF_RPM:           value = Motor_Config_GetSpeedVSvpwmRef_Rpm(p_motor);         break;
        case MOTOR_VAR_SPEED_V_REF_DEG_PER_CYCLE:       value = Motor_GetSpeedVRef_DegPerCycle(p_motor);             break;
        case MOTOR_VAR_SPEED_V_SVPWM_REF_DEG_PER_CYCLE: value = Motor_GetSpeedVRefSvpwm_DegPerCycle(p_motor);        break;
        case MOTOR_VAR_V_SPEED_RATED_FRACT16:           value = Motor_GetVSpeedRated_Fract16(p_motor);               break;
    }
    return value;
}

/* Rates */
int _Motor_Var_ConfigActuation_Get(const Motor_State_T * p_motor, Motor_Var_ConfigActuation_T varId)
{
    int value = 0;
    switch (varId)
    {
        case MOTOR_VAR_BASE_SPEED_LIMIT_FORWARD:    value = Motor_Config_GetSpeedLimitForward_Fract16(p_motor);     break;
        case MOTOR_VAR_BASE_SPEED_LIMIT_REVERSE:    value = Motor_Config_GetSpeedLimitReverse_Fract16(p_motor);     break;
        case MOTOR_VAR_BASE_I_LIMIT_MOTORING:       value = Motor_Config_GetILimitMotoring_Fract16(p_motor);        break;
        case MOTOR_VAR_BASE_I_LIMIT_GENERATING:     value = Motor_Config_GetILimitGenerating_Fract16(p_motor);      break;
        case MOTOR_VAR_SPEED_RAMP_TIME:             value = Motor_Config_GetSpeedRampTime_Millis(p_motor);          break;
        case MOTOR_VAR_TORQUE_RAMP_TIME:            value = Motor_Config_GetTorqueRampTime_Millis(p_motor);         break;
        case MOTOR_VAR_OPEN_LOOP_POWER_LIMIT:       value = Motor_Config_GetOpenLoopScalarLimit(p_motor);           break;
        case MOTOR_VAR_ALIGN_POWER:                 value = Motor_Config_GetAlignPowerScalar(p_motor);              break;
        case MOTOR_VAR_ALIGN_TIME:                  value = Motor_Config_GetAlignTime_Millis(p_motor);              break;
    // #if defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE) || defined(CONFIG_MOTOR_SENSOR_SENSORLESS_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)
        case MOTOR_VAR_OPEN_LOOP_RAMP_I_FINAL:      value = Motor_Config_GetOpenLoopIFinal_Fract16(p_motor);        break;
        case MOTOR_VAR_OPEN_LOOP_RAMP_I_TIME:       value = Motor_Config_GetOpenLoopIRamp_Millis(p_motor);          break;
        case MOTOR_VAR_OPEN_LOOP_RAMP_SPEED_FINAL:  value = Motor_Config_GetOpenLoopSpeedFinal_Fract16(p_motor);    break;
        case MOTOR_VAR_OPEN_LOOP_RAMP_SPEED_TIME:   value = Motor_Config_GetOpenLoopSpeedRamp_Millis(p_motor);      break;
    // #endif
    // #if defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
        // case MOTOR_VAR_PHASE_POLAR_MODE:         value = Motor_Config_GetPhasePolarMode(p_motor);                break;
    // #endif
    }
    return value;
}

void _Motor_Var_ConfigActuation_Set(Motor_State_T * p_motor, Motor_Var_ConfigActuation_T varId, int varValue)
{
    switch (varId)
    {
        case MOTOR_VAR_BASE_SPEED_LIMIT_FORWARD:    Motor_Config_SetSpeedLimitForward_Fract16(p_motor, varValue);       break;
        case MOTOR_VAR_BASE_SPEED_LIMIT_REVERSE:    Motor_Config_SetSpeedLimitReverse_Fract16(p_motor, varValue);       break;
        case MOTOR_VAR_BASE_I_LIMIT_MOTORING:       Motor_Config_SetILimitMotoring_Fract16(p_motor, varValue);          break;
        case MOTOR_VAR_BASE_I_LIMIT_GENERATING:     Motor_Config_SetILimitGenerating_Fract16(p_motor, varValue);        break;
        case MOTOR_VAR_SPEED_RAMP_TIME:             Motor_Config_SetSpeedRampTime_Millis (p_motor, varValue);           break;
        case MOTOR_VAR_TORQUE_RAMP_TIME:            Motor_Config_SetTorqueRampTime_Millis (p_motor, varValue);          break;
        case MOTOR_VAR_OPEN_LOOP_POWER_LIMIT:       Motor_Config_SetOpenLoopScalarLimit(p_motor, varValue);             break;
        case MOTOR_VAR_ALIGN_POWER:                 Motor_Config_SetAlignPowerScalar (p_motor, varValue);               break;
        case MOTOR_VAR_ALIGN_TIME:                  Motor_Config_SetAlignTime_Millis(p_motor, varValue);                break;
        case MOTOR_VAR_OPEN_LOOP_RAMP_SPEED_FINAL:  Motor_Config_SetOpenLoopRampSpeedFinal_Fract16(p_motor, varValue);  break;
        case MOTOR_VAR_OPEN_LOOP_RAMP_SPEED_TIME:   Motor_Config_SetOpenLoopRampSpeedTime_Millis (p_motor, varValue);   break;
        case MOTOR_VAR_OPEN_LOOP_RAMP_I_FINAL:      Motor_Config_SetOpenLoopRampIFinal_Fract16(p_motor, varValue);      break;
        case MOTOR_VAR_OPEN_LOOP_RAMP_I_TIME:       Motor_Config_SetOpenLoopRampITime_Millis(p_motor, varValue);        break;
        // case MOTOR_VAR_PHASE_POLAR_MODE:           Motor_Config_SetPhaseModeParam(p_motor,  varValue);  break;
    }
}


int _Motor_Var_ConfigPid_Get(const Motor_State_T * p_motor, Motor_Var_ConfigPid_T varId)
{
    int value = 0;
    switch (varId)
    {
        case MOTOR_VAR_PID_SPEED_SAMPLE_FREQ:     value = _PID_GetSampleFreq(&p_motor->Config.PidSpeed);   break;
        case MOTOR_VAR_PID_SPEED_KP_FIXED16:      value = _PID_GetKp_Fixed16(&p_motor->Config.PidSpeed);   break;
        case MOTOR_VAR_PID_SPEED_KI_FIXED16:      value = _PID_GetKi_Fixed16(&p_motor->Config.PidSpeed);   break;
        // case MOTOR_VAR_PID_SPEED_KD_FIXED16:      value = PID_GetKd_Fixed16(&p_motor->Config.PidSpeed);   break;
        case MOTOR_VAR_PID_CURRENT_SAMPLE_FREQ:    value = _PID_GetSampleFreq(&p_motor->Config.PidI);     break;
        case MOTOR_VAR_PID_CURRENT_KP_FIXED16:     value = _PID_GetKp_Fixed16(&p_motor->Config.PidI);     break;
        case MOTOR_VAR_PID_CURRENT_KI_FIXED16:     value = _PID_GetKi_Fixed16(&p_motor->Config.PidI);     break;
        // case MOTOR_VAR_PID_CURRENT_KD_FIXED16:     value = PID_GetKd_Fixed16(&p_motor->Config.PidI);     break;
    }
    return value;
}


/* Coeffcients in 9.7 */
void _Motor_Var_ConfigPid_Set(Motor_State_T * p_motor, Motor_Var_ConfigPid_T varId, int varValue)
{
    switch (varId)
    {
        case MOTOR_VAR_PID_SPEED_SAMPLE_FREQ:   break;
        case MOTOR_VAR_PID_SPEED_KP_FIXED16:
            _PID_SetKp_Fixed16(&p_motor->Config.PidSpeed, varValue);
            PID_SetKp_Fixed16(&p_motor->PidSpeed, varValue); /* optionally set run time */
            break;
        case MOTOR_VAR_PID_SPEED_KI_FIXED16:
            _PID_SetKi_Fixed16(&p_motor->Config.PidSpeed, varValue);
            PID_SetKi_Fixed16(&p_motor->PidSpeed, varValue);
            break;
        case MOTOR_VAR_PID_CURRENT_SAMPLE_FREQ: break;
        case MOTOR_VAR_PID_CURRENT_KP_FIXED16:
            _PID_SetKp_Fixed16(&p_motor->Config.PidI, varValue);
            PID_SetKp_Fixed16(&p_motor->PidIq, varValue);
            PID_SetKp_Fixed16(&p_motor->PidId, varValue);
            break;
        case MOTOR_VAR_PID_CURRENT_KI_FIXED16:
            _PID_SetKi_Fixed16(&p_motor->Config.PidI, varValue);
            PID_SetKi_Fixed16(&p_motor->PidIq, varValue);
            PID_SetKi_Fixed16(&p_motor->PidId, varValue);
            break;
    }
}


/*

*/
void _Motor_Var_ConfigCmd_Call(const Motor_T * p_motor, Motor_Var_ConfigCmd_T varId, int varValue)
{
    switch (varId)
    {
        case MOTOR_VAR_CONFIG_CMD_ADC_CALIBRATION:  Motor_Analog_Calibrate(p_motor);        break;
        case MOTOR_VAR_CONFIG_CMD_VIRTUAL_HOME:     Motor_Calibration_StartHome(p_motor);   break;
        case MOTOR_VAR_CONFIG_ENTER_CALIBRATION:    Motor_Calibration_EnterTuning(p_motor); break;
        // case MOTOR_VAR_CONFIG_CMD_ENCODER_HOME:  Motor_Encoder_StartHoming(p_motor);     break; // move to sensor cmds
    }
}

/*
    Runtime Tuning version
*/
int _Motor_Var_PidTuning_Get(const Motor_State_T * p_motor, Motor_Var_ConfigPid_T varId)
{
    int value = 0;
    switch (varId)
    {
        case MOTOR_VAR_PID_SPEED_SAMPLE_FREQ:     value = PID_GetSampleFreq(&p_motor->PidSpeed);    break;
        case MOTOR_VAR_PID_SPEED_KP_FIXED16:      value = PID_GetKp_Fixed16(&p_motor->PidSpeed);    break;
        case MOTOR_VAR_PID_SPEED_KI_FIXED16:      value = PID_GetKi_Fixed16(&p_motor->PidSpeed);    break;
        case MOTOR_VAR_PID_CURRENT_SAMPLE_FREQ:   value = PID_GetSampleFreq(&p_motor->PidIq);       break;
        case MOTOR_VAR_PID_CURRENT_KP_FIXED16:    value = PID_GetKp_Fixed16(&p_motor->PidIq);       break;
        case MOTOR_VAR_PID_CURRENT_KI_FIXED16:    value = PID_GetKi_Fixed16(&p_motor->PidIq);       break;
    }
    return value;
}

void _Motor_Var_PidTuning_Set(Motor_State_T * p_motor, Motor_Var_ConfigPid_T varId, int varValue)
{
    if (!_Motor_StateMachine_IsState(p_motor, MSM_STATE_ID_CALIBRATION)) { return; }

    switch (varId)
    {
        case MOTOR_VAR_PID_SPEED_SAMPLE_FREQ: break;
        case MOTOR_VAR_PID_SPEED_KP_FIXED16:  PID_SetKp_Fixed16(&p_motor->PidSpeed, varValue);    break;
        case MOTOR_VAR_PID_SPEED_KI_FIXED16:  PID_SetKi_Fixed16(&p_motor->PidSpeed, varValue);    break;
        case MOTOR_VAR_PID_CURRENT_SAMPLE_FREQ: break;
        case MOTOR_VAR_PID_CURRENT_KP_FIXED16: PID_SetKp_Fixed16(&p_motor->PidIq, varValue); PID_SetKp_Fixed16(&p_motor->PidId, varValue); break;
        case MOTOR_VAR_PID_CURRENT_KI_FIXED16: PID_SetKi_Fixed16(&p_motor->PidIq, varValue); PID_SetKi_Fixed16(&p_motor->PidId, varValue); break;
    }
}

/*
    const
    Alternate access to board reference values
*/
int Motor_Var_StaticRef_Get(Motor_Var_StaticRef_T varId)
{
    int value = 0;
    switch (varId)
    {
        // case MOTOR_VAR_REF_CONTROL_FREQ:                 value =                   break;
        // case MOTOR_VAR_REF_SPEED_FEEDBACK_FREQ:          value =                   break;
        // case MOTOR_VAR_REF_CURRENT_FEEDBACK_FREQ:        value =                   break;
        case MOTOR_VAR_REF_V_RATED:                 value = Phase_Calibration_GetVRated_Fract16();                 break;
        case MOTOR_VAR_REF_I_RATED:                 value = Phase_Calibration_GetIRatedPeak_Fract16();             break;
        case MOTOR_VAR_REF_V_MAX:                   value = Phase_Calibration_GetVMaxVolts();                      break;
        case MOTOR_VAR_REF_I_MAX:                   value = Phase_Calibration_GetIMaxAmps();                       break;
        case MOTOR_VAR_REF_V_MAX_ADCU:              value = PHASE_ANALOG_V_MAX_ADCU;                            break;
        case MOTOR_VAR_REF_I_MAX_ADCU:              value = PHASE_ANALOG_I_MAX_ADCU;                            break;

        case MOTOR_VAR_REF_BOARD_V_RATED_VOLTS:     value = Phase_AnalogSensor_GetVRated();                         break;
        case MOTOR_VAR_REF_BOARD_I_RATED_AMPS:      value = Phase_AnalogSensor_GetIRatedRms();                      break;
        case MOTOR_VAR_REF_V_PHASE_R1:              value = PHASE_ANALOG_SENSOR_REF.V_PHASE_R1 / 10;       break;
        case MOTOR_VAR_REF_V_PHASE_R2:              value = PHASE_ANALOG_SENSOR_REF.V_PHASE_R2 / 10;       break;
        case MOTOR_VAR_REF_I_PHASE_R_BASE:          value = PHASE_ANALOG_SENSOR_REF.I_PHASE_R_BASE;        break;
        case MOTOR_VAR_REF_I_PHASE_R_MOSFETS:       value = PHASE_ANALOG_SENSOR_REF.I_PHASE_R_MOSFETS;     break;
        case MOTOR_VAR_REF_I_PHASE_GAIN:            value = PHASE_ANALOG_SENSOR_REF.I_PHASE_GAIN;          break;

    }
    return value;
}

/******************************************************************************/
/*
    [VarType]
    Typed Per SensorId handled by Motor_Sensor.h/c
*/
/******************************************************************************/
int Motor_VarType_Control_Get(const Motor_T * p_motor, Motor_VarType_Control_T typeId, int varId)
{
    if (p_motor == NULL) { return 0; }

    switch (typeId)
    {
        case MOTOR_VAR_TYPE_USER_OUT:           return _Motor_Var_UserOut_Get(p_motor->P_MOTOR_STATE, varId);
        case MOTOR_VAR_TYPE_ROTOR_OUT:          return _Motor_Var_Rotor_Get(p_motor->P_MOTOR_STATE, varId);
        case MOTOR_VAR_TYPE_FOC_OUT:            return _Motor_Var_Foc_Get(p_motor->P_MOTOR_STATE, varId);
        case MOTOR_VAR_TYPE_USER_CONTROL:       return _Motor_Var_UserControl_Get(p_motor, varId);
        case MOTOR_VAR_TYPE_STATE_CMD:          return 0;
        // case MOTOR_VAR_TYPE_USER_SETPOINT:      return 0;
    }
    return 0;
}

// access control use on/off flag, or caller handle
// mapping to statemachine requires multiple substate.
/* Var Access Control cannot use StateMachine directly */
void Motor_VarType_Control_Set(const Motor_T * p_motor, Motor_VarType_Control_T typeId, int varId, int varValue)
{
    if (p_motor == NULL) { return; }

    switch (typeId)
    {
        case MOTOR_VAR_TYPE_USER_OUT:            break;
        case MOTOR_VAR_TYPE_ROTOR_OUT:           break;
        case MOTOR_VAR_TYPE_FOC_OUT:             break;
        /* Access Control on */
        case MOTOR_VAR_TYPE_USER_CONTROL:       _Motor_Var_UserControl_Set(p_motor, varId, varValue);     break;
        case MOTOR_VAR_TYPE_USER_SETPOINT:      _Motor_Var_UserSetpoint_Set(p_motor, varId, varValue);    break;
        case MOTOR_VAR_TYPE_STATE_CMD:          _Motor_Var_StateCmd_Set(p_motor, varId, varValue);        break;
    }
}

int Motor_VarType_Config_Get(const Motor_T * p_motor, Motor_VarType_Config_T typeId, int varId)
{
    if (p_motor == NULL) { return 0; }

    switch (typeId)
    {
        case MOTOR_VAR_TYPE_CONFIG_CALIBRATION:         return _Motor_Var_ConfigCalibration_Get(p_motor->P_MOTOR_STATE, varId);
        case MOTOR_VAR_TYPE_CONFIG_CALIBRATION_ALIAS:   return _Motor_Var_ConfigCalibrationAlias_Get(p_motor->P_MOTOR_STATE, varId);
        case MOTOR_VAR_TYPE_CONFIG_ACTUATION:           return _Motor_Var_ConfigActuation_Get(p_motor->P_MOTOR_STATE, varId);
        case MOTOR_VAR_TYPE_CONFIG_PID:                 return _Motor_Var_ConfigPid_Get(p_motor->P_MOTOR_STATE, varId);
        case MOTOR_VAR_TYPE_CONFIG_CMD:                 return 0; // Write only, no read access
        case MOTOR_VAR_TYPE_CONFIG_SENSOR_CMD:          return 0; // Write only, no read access
    }
    return 0;
}

/* Config Access Control use StateMachine */
void Motor_VarType_Config_Set(const Motor_T * p_motor, Motor_VarType_Config_T typeId, int varId, int varValue)
{
    if (p_motor == NULL) { return; }
    if (!Motor_StateMachine_IsConfig(p_motor)) { return; }  // optionally as calibration substate

    switch (typeId)
    {
        case MOTOR_VAR_TYPE_CONFIG_CALIBRATION:         _Motor_Var_ConfigCalibration_Set(p_motor->P_MOTOR_STATE, varId, varValue);  break;
        case MOTOR_VAR_TYPE_CONFIG_ACTUATION:           _Motor_Var_ConfigActuation_Set(p_motor->P_MOTOR_STATE, varId, varValue);    break;
        case MOTOR_VAR_TYPE_CONFIG_PID:                 _Motor_Var_ConfigPid_Set(p_motor->P_MOTOR_STATE, varId, varValue);          break;
        case MOTOR_VAR_TYPE_CONFIG_CMD:                 _Motor_Var_ConfigCmd_Call(p_motor, varId, varValue);                        break;
        case MOTOR_VAR_TYPE_CONFIG_SENSOR_CMD:          Motor_Sensor_CalibrationCmd_Call(p_motor, varId, varValue);                 break;
        case MOTOR_VAR_TYPE_CONFIG_CALIBRATION_ALIAS:   break; // Read only, no set access
    }
}

int Motor_VarType_SubModule_Get(const Motor_T * p_motor, Motor_VarType_SubModule_T typeId, int varId)
{
    if (p_motor == NULL) { return 0; }

    switch (typeId)
    {
        case MOTOR_VAR_TYPE_HEAT_MONITOR_OUT:           return HeatMonitor_VarId_Get(&p_motor->HEAT_MONITOR_CONTEXT, varId);
        case MOTOR_VAR_TYPE_HEAT_MONITOR_CONFIG:        return HeatMonitor_ConfigId_Get(&p_motor->HEAT_MONITOR_CONTEXT, varId);
        case MOTOR_VAR_TYPE_THERMISTOR_CONFIG:          return HeatMonitor_Thermistor_ConfigId_Get(&p_motor->HEAT_MONITOR_CONTEXT, varId);
        case MOTOR_VAR_TYPE_STATIC_BOARD_REF:           return Motor_Var_StaticRef_Get(varId);
        case MOTOR_VAR_TYPE_PID_TUNING_IO:              return _Motor_Var_PidTuning_Get(p_motor->P_MOTOR_STATE, varId);
    }
    return 0;
}

void Motor_VarType_SubModule_Set(const Motor_T * p_motor, Motor_VarType_SubModule_T typeId, int varId, int varValue)
{
    if (p_motor == NULL) { return; }

    switch (typeId)
    {
        case MOTOR_VAR_TYPE_HEAT_MONITOR_OUT:       break;
        case MOTOR_VAR_TYPE_HEAT_MONITOR_CONFIG:
            if (Motor_StateMachine_IsConfig(p_motor)) { HeatMonitor_ConfigId_Set(&p_motor->HEAT_MONITOR_CONTEXT, varId, varValue); }
            break;
        case MOTOR_VAR_TYPE_THERMISTOR_CONFIG:
            if (Motor_StateMachine_IsConfig(p_motor)) { HeatMonitor_Thermistor_ConfigId_Set(&p_motor->HEAT_MONITOR_CONTEXT, varId, varValue); }
            break;
        case MOTOR_VAR_TYPE_PID_TUNING_IO:          _Motor_Var_PidTuning_Set(p_motor->P_MOTOR_STATE, varId, varValue);      break;
        case MOTOR_VAR_TYPE_STATIC_BOARD_REF:       break; // Read only, no set access
    }
}

/******************************************************************************/
/*
    Wrap
*/
/******************************************************************************/
int Motor_VarType_Sensor_Get(const Motor_T * p_motor, Motor_VarType_RotorSensor_T typeId, int varId)
{
    if (p_motor == NULL) return 0;

    switch (typeId)
    {
        // case MOTOR_VAR_TYPE_HALL_STATE:     return Hall_VarId_Get(&p_motor->SENSOR_TABLE.HALL.HALL, varId); break;
        case MOTOR_VAR_TYPE_ENCODER_STATE:  return Encoder_ModeDT_VarId_Get(p_motor->SENSOR_TABLE.ENCODER.ENCODER.P_STATE, varId);  break;
        case MOTOR_VAR_TYPE_HALL_CONFIG:    return _Hall_ConfigId_Get(p_motor->SENSOR_TABLE.HALL.HALL.P_STATE, varId);              break;
        case MOTOR_VAR_TYPE_ENCODER_CONFIG: return _Encoder_ConfigId_Get(p_motor->SENSOR_TABLE.ENCODER.ENCODER.P_STATE, varId);     break;
        // case MOTOR_VAR_TYPE_SIN_COS: return SinCos_VarId_Get(&p_motor->SENSOR_TABLE.SIN_COS.SIN_COS, varId); break;
        // case MOTOR_VAR_TYPE_SENSORLESS: return Sensorless_VarId_Get(&p_motor->SENSOR_TABLE.SENSORLESS.SENSORLESS, varId); break;
        default: return 0; // or some error value
    }
    // RotorSensor_Of(   p_table,   id) tood typed
}

void Motor_VarType_Sensor_Set(const Motor_T * p_motor, Motor_VarType_RotorSensor_T typeId, int varId, int varValue)
{
    if (p_motor == NULL) return;
    if (!Motor_StateMachine_IsConfig(p_motor)) return;

    switch (typeId)
    {
        case MOTOR_VAR_TYPE_HALL_CONFIG:      Hall_ConfigId_Set(&p_motor->SENSOR_TABLE.HALL.HALL, varId, varValue);            break;
        case MOTOR_VAR_TYPE_ENCODER_CONFIG:   Encoder_ConfigId_Set(&p_motor->SENSOR_TABLE.ENCODER.ENCODER, varId, varValue);   break;
        case MOTOR_VAR_TYPE_HALL_STATE:                  break;
        case MOTOR_VAR_TYPE_ENCODER_STATE:               break;
        // case ROTOR_SENSOR_ID_SENSORLESS: Sensorless_ConfigId_Set(&p_motor->SENSOR_TABLE.SENSORLESS.SENSORLESS, varId, varValue); break;
        // case ROTOR_SENSOR_ID_SIN_COS:    SinCos_ConfigId_Set(&p_motor->SENSOR_TABLE.SIN_COS.SIN_COS, varId, varValue);    break;
        default: break;
    }
}


// /******************************************************************************/
// /*
//     Input - Access control using State Flags
// */
// /******************************************************************************/
// // alternatively use substate
// static int Var_PidTuning_Get(const Motor_State_T * p_motor, int varId) { return _Motor_Var_PidTuning_Get(p_motor, varId); }
// static void Var_PidTuning_Set(Motor_State_T * p_motor, int varId, int varValue) { _Motor_Var_PidTuning_Set(p_motor, varId, varValue); }

// const VarAccess_VTable_T MOTOR_VAR_ACCESS_PID_TUNING =
// {
//     .GET_AT = (get_field_t)Var_PidTuning_Get,
//     .SET_AT = (set_field_t)Var_PidTuning_Set,
//     .TEST_SET = NULL,
// };

// static int Var_UserControl_Get(const Motor_T * p_motor, int varId) { return _Motor_Var_UserControl_Get(p_motor, varId); }
// static void Var_UserControl_Set(const Motor_T * p_motor, int varId, int varValue) { _Motor_Var_UserControl_Set(p_motor, varId, varValue); }

// const VarAccess_VTable_T MOTOR_VAR_ACCESS_USER_CONTROL =
// {
//     .GET_AT = (get_field_t)Var_UserControl_Get,
//     .SET_AT = (set_field_t)Var_UserControl_Set,
//     .TEST_SET = NULL,
// };

// static void Var_StateCmd_Set(const Motor_T * p_motor, int varId, int varValue) { _Motor_Var_StateCmd_Set(p_motor, varId, varValue); }

// const VarAccess_VTable_T MOTOR_VAR_ACCESS_STATE_CMD =
// {
//     .GET_AT = NULL,
//     .SET_AT = (set_field_t)Var_StateCmd_Set,
//     .TEST_SET = NULL,
// };


// void Motor_Var_PidTuning_Set(const Motor_T * p_motor, Motor_Var_ConfigPid_T varId, int varValue) { _VarAccess_SetAt(&p_motor->VAR_ACCESS.ACCESS_PID_TUNING, varId, varValue); }
// void Motor_Var_UserControl_Set(const Motor_T * p_motor, Motor_Var_UserControl_T varId, int varValue) { _VarAccess_SetAt(&p_motor->VAR_ACCESS.ACCESS_USER_CONTROL, varId, varValue); }
// void Motor_Var_StateCmd_Set(const Motor_T * p_motor, Motor_Var_StateCmd_T varId, int varValue) { _VarAccess_SetAt(&p_motor->VAR_ACCESS.ACCESS_STATE_CMD, varId, varValue); }

// void Motor_Var_DisableInput(Motor_State_T * p_motor) { _VarAccess_DisableSet(&p_motor->VarAccessInputState); }
// void Motor_Var_EnableInput(Motor_State_T * p_motor) { _VarAccess_EnableSet(&p_motor->VarAccessInputState); }

// /******************************************************************************/
// /*
//     Config - alternatively check state in handler
// */
// /******************************************************************************/
// static int VarConfig_Calibration_Get(const Motor_State_T * p_motor, int varId) { return _Motor_Var_ConfigCalibration_Get(p_motor, varId); }
// static int VarConfig_CalibrationAlias_Get(const Motor_State_T * p_motor, int varId) { return _Motor_Var_ConfigCalibrationAlias_Get(p_motor, varId); }
// static int VarConfig_Actuation_Get(const Motor_State_T * p_motor, int varId) { return _Motor_Var_ConfigActuation_Get(p_motor, varId); }
// static int VarConfig_Pid_Get(const Motor_State_T * p_motor, int varId) { return _Motor_Var_ConfigPid_Get(p_motor, varId); }

// static void VarConfig_Calibration_Set(Motor_State_T * p_motor, int varId, int varValue) { _Motor_Var_ConfigCalibration_Set(p_motor, varId, varValue); }
// static void VarConfig_Actuation_Set(Motor_State_T * p_motor, int varId, int varValue) { _Motor_Var_ConfigActuation_Set(p_motor, varId, varValue); }
// static void VarConfig_Pid_Set(Motor_State_T * p_motor, int varId, int varValue) { _Motor_Var_ConfigPid_Set(p_motor, varId, varValue); }

// const VarAccess_VTable_T MOTOR_VAR_CONFIG_CALIBRATION =
// {
//     .GET_AT = (get_field_t)VarConfig_Calibration_Get,
//     .SET_AT = (set_field_t)VarConfig_Calibration_Set,
//     .TEST_SET = (test_t)Motor_Config_IsConfigState,
// };

// const VarAccess_VTable_T MOTOR_VAR_CONFIG_CALIBRATION_ALIAS =
// {
//     .GET_AT = (get_field_t)VarConfig_CalibrationAlias_Get,
//     .SET_AT = NULL,
//     .TEST_SET = NULL,
// };

// const VarAccess_VTable_T MOTOR_VAR_CONFIG_ACTUATION =
// {
//     .GET_AT = (get_field_t)VarConfig_Actuation_Get,
//     .SET_AT = (set_field_t)VarConfig_Actuation_Set,
//     .TEST_SET = (test_t)Motor_Config_IsConfigState,
// };


// const VarAccess_VTable_T MOTOR_VAR_CONFIG_PID =
// {
//     .GET_AT = (get_field_t)VarConfig_Pid_Get,
//     .SET_AT = (set_field_t)VarConfig_Pid_Set,
//     .TEST_SET = (test_t)Motor_Config_IsConfigState,
// };


// /******************************************************************************/
// /*
//     Config Cmds
// */
// /******************************************************************************/
// static void VarConfig_Cmd_Call(const Motor_T * p_motor, int varId, int varValue) { _Motor_Var_ConfigCmd_Call(p_motor, varId, varValue); }

// const VarAccess_VTable_T MOTOR_VAR_CONFIG_CMD =
// {
//     .GET_AT = NULL,
//     .SET_AT = (set_field_t)VarConfig_Cmd_Call,
//     .TEST_SET = (test_t)Motor_StateMachine_IsConfig, /* Check [StopState] to transition to [CalibrationState] */
// };


// /******************************************************************************/
// /*
//     Output - optionally access control
// */
// /******************************************************************************/
// static int Var_UserOut_Get(const Motor_State_T * p_motor, int varId) { return _Motor_Var_UserOut_Get(p_motor, varId); }
// static int Var_Sensor_Get(const Motor_State_T * p_motor, int varId) { return _Motor_Var_Rotor_Get(p_motor, varId); }
// static int Var_Foc_Get(const Motor_State_T * p_motor, int varId) { return _Motor_Var_Foc_Get(p_motor, varId); }

// const VarAccess_VTable_T MOTOR_VAR_OUT_USER =
// {
//     .GET_AT = (get_field_t)Var_UserOut_Get,
//     .SET_AT = NULL,
//     .TEST_SET = NULL,
// };

// const VarAccess_VTable_T MOTOR_VAR_OUT_FOC =
// {
//     .GET_AT = (get_field_t)Var_Foc_Get,
//     .SET_AT = NULL,
//     .TEST_SET = NULL,
// };

// const VarAccess_VTable_T MOTOR_VAR_OUT_ROTOR =
// {
//     .GET_AT = (get_field_t)Var_Sensor_Get,
//     .SET_AT = NULL,
//     .TEST_SET = NULL,
// };


