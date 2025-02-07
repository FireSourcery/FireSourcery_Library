

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
    @version V0

    @brief
*/
/******************************************************************************/
#include "Motor_Var.h"


/******************************************************************************/
/*
    Var
*/
/******************************************************************************/
int32_t Motor_VarOutput_Get(const Motor_T * p_motor, Motor_VarOuput_T varId)
{
    int32_t value = 0;
    switch (varId)
    {
        case MOTOR_VAR_SPEED:                       value = Motor_User_GetSpeed_UFract16(p_motor);              break;
        case MOTOR_VAR_I_PHASE:                     value = Motor_User_GetIPhase_UFract16(p_motor);             break;
        case MOTOR_VAR_V_PHASE:                     value = Motor_User_GetVPhase_UFract16(p_motor);             break;
        case MOTOR_VAR_POWER:                       value = Motor_User_GetElectricalPower_UFract16(p_motor);    break;
        case MOTOR_VAR_STATE:                       value = Motor_User_GetStateId(p_motor);                     break;
        case MOTOR_VAR_STATUS_FLAGS:                value = Motor_User_GetStatusFlags(p_motor).Value;           break;
        // case MOTOR_VAR_STATUS_FLAGS:    value = Motor_User_GetStateFlags(p_motor).Word;       break;
        case MOTOR_VAR_FAULT_FLAGS:                 value = Motor_User_GetFaultFlags(p_motor).Value;            break;
        case MOTOR_VAR_HEAT:                        value = Motor_User_GetHeat_Adcu(p_motor);                   break;
        case MOTOR_VAR_EFFECTIVE_FEEDBACK_MODE:     value = Motor_User_GetFeedbackMode(p_motor).Value;          break;
        case MOTOR_VAR_EFFECTIVE_SET_POINT:         value = Motor_User_GetSetPoint(p_motor);                    break;
        case MOTOR_VAR_EFFECTIVE_SPEED_LIMIT:       value = Motor_User_GetSpeedLimit(p_motor);                  break;
        case MOTOR_VAR_EFFECTIVE_I_LIMIT:           value = Motor_User_GetILimit(p_motor);                      break;
        case MOTOR_VAR_V_SPEED_DEBUG:               value = Motor_User_GetVSpeedDebug_UFract16(p_motor);        break;
        case MOTOR_VAR_V_SPEED_EFFECTIVE:           value = Motor_User_GetVSpeedEffective_UFract16(p_motor);    break;
        case MOTOR_VAR_ELECTRICAL_ANGLE:            value = Motor_User_GetElectricalAngle(p_motor);             break;
        case MOTOR_VAR_MECHANICAL_ANGLE:            value = Motor_User_GetMechanicalAngle(p_motor);             break;
    }
    return value;
}

int32_t Motor_VarOutput_Foc_Get(const Motor_T * p_motor, Motor_VarOuput_Foc_T varId)
{
    int32_t value = 0;
    switch (varId)
    {
        case MOTOR_VAR_FOC_IA:    value = p_motor->Foc.Ia;        break;
        case MOTOR_VAR_FOC_IB:    value = p_motor->Foc.Ib;        break;
        case MOTOR_VAR_FOC_IC:    value = p_motor->Foc.Ic;        break;
        case MOTOR_VAR_FOC_IQ:    value = p_motor->Foc.Iq;        break;
        case MOTOR_VAR_FOC_ID:    value = p_motor->Foc.Id;        break;
        case MOTOR_VAR_FOC_VQ:    value = p_motor->Foc.Vq;        break;
        case MOTOR_VAR_FOC_VD:    value = p_motor->Foc.Vd;        break;
        case MOTOR_VAR_FOC_REQ_Q: value = p_motor->Foc.ReqQ;      break;
        case MOTOR_VAR_FOC_REQ_D: value = p_motor->Foc.ReqD;      break;
        case MOTOR_VAR_FOC_VA:    value = p_motor->Foc.Va;        break;
        case MOTOR_VAR_FOC_VB:    value = p_motor->Foc.Vb;        break;
        case MOTOR_VAR_FOC_VC:    value = p_motor->Foc.Vc;        break;
        case MOTOR_VAR_FOC_INTEGRAL_Q:    value = PID_GetIntegral(&p_motor->PidIq);        break;
        case MOTOR_VAR_FOC_INTEGRAL_D:    value = PID_GetIntegral(&p_motor->PidId);        break;
    }
    return value;
}

int32_t Motor_VarOutput_PositionSensor_Get(const Motor_T * p_motor, Motor_VarOutput_PositionSensor_T varId)
{
    int32_t value = 0;
    switch (varId)
    {
        case MOTOR_VAR_ENCODER_FREQ:  value = p_motor->Encoder.FreqD;     break;
        case MOTOR_VAR_ENCODER_RPM:   value = Encoder_ModeDT_GetRotationalSpeed_RPM(&p_motor->Encoder);     break;
    }
    return value;
}

void Motor_VarInput_Set(Motor_T * p_motor, Motor_VarInput_T varId, int32_t varValue)
{
    switch (varId)
    {
        case MOTOR_VAR_CLEAR_FAULT:     break;
        // case MOTOR_VAR_USER_CMD:        Motor_User_SetActiveCmdValue(p_motor, varValue);    break;
        case MOTOR_VAR_CMD_SPEED:       Motor_User_SetSpeedCmd(p_motor, varValue);     break;
        case MOTOR_VAR_CMD_CURRENT:     Motor_User_SetICmd(p_motor, varValue);         break;
        case MOTOR_VAR_CMD_VOLTAGE:     Motor_User_SetVoltageCmd(p_motor, varValue);   break;
        case MOTOR_VAR_CMD_ANGLE:       Motor_User_SetPositionCmd(p_motor, varValue);  break;
        case MOTOR_VAR_CMD_OPEN_LOOP:   Motor_User_SetOpenLoopCmd(p_motor, varValue);  break;
        case MOTOR_VAR_FORCE_DISABLE_CONTROL: Motor_User_ForceDisableControl(p_motor);  break;
            //temp  without state machine
        // case MOTOR_VAR_PHASE_ALIGN:         Phase_Align_ActivateDuty(&p_motor->Phase, (Phase_Align_T)varValue, p_motor->Config.AlignPower_UFract16);    break;
        // case MOTOR_VAR_FEED_FORWARD_ANGLE:  Motor_FOC_ProcAngleFeedforward(p_motor, 0, p_motor->Config.AlignPower_UFract16, 0);                         break;
    }
}

int32_t Motor_VarIO_Get(const Motor_T * p_motor, Motor_VarIO_T varId)
{
    int32_t value = 0;
    switch (varId)
    {
        case MOTOR_VAR_DIRECTION:           value = Motor_User_GetDirection(p_motor);             break;
        case MOTOR_VAR_USER_SET_POINT:      value = Motor_User_GetSetPoint(p_motor);              break;
        case MOTOR_VAR_USER_FEEDBACK_MODE:  value = Motor_User_GetFeedbackMode(p_motor).Value;    break;
        case MOTOR_VAR_USER_PHASE_STATE:    value = Motor_User_GetPhaseState(p_motor);            break;
        case MOTOR_VAR_USER_SPEED_LIMIT:    value = Motor_User_GetSpeedLimit(p_motor);            break;
        case MOTOR_VAR_USER_I_LIMIT:        value = Motor_User_GetILimit(p_motor);                break;
    }
    return value;
}

void Motor_VarIO_Set(Motor_T * p_motor, Motor_VarIO_T varId, int32_t varValue)
{
    switch (varId)
    {
        case MOTOR_VAR_DIRECTION:           Motor_User_SetDirection(p_motor, (Motor_Direction_T)varValue);  break; // use async polling for status
        case MOTOR_VAR_USER_SET_POINT:      Motor_User_SetActiveCmdValue(p_motor, varValue);                break;
        case MOTOR_VAR_USER_FEEDBACK_MODE:  Motor_User_SetFeedbackMode_Cast(p_motor, (uint8_t)varValue);    break;
        case MOTOR_VAR_USER_PHASE_STATE:    Motor_User_StartPhaseState(p_motor, (Phase_State_T)varValue);   break;
        case MOTOR_VAR_USER_SPEED_LIMIT:    Motor_User_TrySpeedLimit(p_motor, varValue);                    break;
        case MOTOR_VAR_USER_I_LIMIT:        Motor_User_TryILimit(p_motor, varValue);                        break;
    }
}

/******************************************************************************/
/*
    Config
*/
/******************************************************************************/
int32_t Motor_VarConfig_Calibration_Get(const Motor_T * p_motor, Motor_VarConfig_Calibration_T varId)
{
    int32_t value = 0;
    switch (varId)
    {
        case MOTOR_VAR_COMMUTATION_MODE:              value = Motor_Config_GetCommutationMode(p_motor);           break;
        case MOTOR_VAR_SENSOR_MODE:                   value = Motor_Config_GetSensorMode(p_motor);                break;
        case MOTOR_VAR_DIRECTION_CALIBRATION:         value = Motor_Config_GetDirectionCalibration(p_motor);      break;
        case MOTOR_VAR_POLE_PAIRS:                    value = Motor_Config_GetPolePairs(p_motor);                 break;
        case MOTOR_VAR_KV:                            value = Motor_Config_GetKv(p_motor);                        break;
        case MOTOR_VAR_V_SPEED_SCALAR:                value = Motor_Config_GetVSpeedScalar_UFract16(p_motor);     break;
        case MOTOR_VAR_SPEED_V_REF_RPM:               value = Motor_Config_GetSpeedVRef_Rpm(p_motor);             break;
        case MOTOR_VAR_SPEED_V_MATCH_REF_RPM:         value = Motor_Config_GetSpeedVMatchRef_Rpm(p_motor);        break;
        case MOTOR_VAR_IA_ZERO_REF_ADCU:              value = Motor_Config_GetIaZero_Adcu(p_motor);               break;
        case MOTOR_VAR_IB_ZERO_REF_ADCU:              value = Motor_Config_GetIbZero_Adcu(p_motor);               break;
        case MOTOR_VAR_IC_ZERO_REF_ADCU:              value = Motor_Config_GetIcZero_Adcu(p_motor);               break;
        case MOTOR_VAR_I_PEAK_REF_ADCU:               value = Motor_Config_GetIPeakRef_Adcu(p_motor);             break;
    }
    return value;
}

void Motor_VarConfig_Calibration_Set(Motor_T * p_motor, Motor_VarConfig_Calibration_T varId, int32_t varValue)
{
    switch (varId)
    {
        case MOTOR_VAR_COMMUTATION_MODE:              Motor_Config_SetCommutationMode(p_motor, varValue);           break;
        case MOTOR_VAR_SENSOR_MODE:                   Motor_Config_SetSensorMode(p_motor, varValue);                break;
        case MOTOR_VAR_DIRECTION_CALIBRATION:         Motor_Config_SetDirectionCalibration(p_motor, varValue);      break;
        case MOTOR_VAR_POLE_PAIRS:                    Motor_Config_SetPolePairs(p_motor, varValue);                 break;
        case MOTOR_VAR_KV:                            Motor_Config_SetKv(p_motor, varValue);                        break;
        case MOTOR_VAR_V_SPEED_SCALAR:                Motor_Config_SetVSpeedScalar_UFract16(p_motor, varValue);     break;
        // case MOTOR_VAR_SPEED_V_REF_RPM:               Motor_Config_SetSpeedVRef_Rpm(p_motor, varValue);             break;
        // case MOTOR_VAR_SPEED_V_MATCH_REF_RPM:         Motor_Config_SetSpeedVMatchRef_Rpm(p_motor, varValue);        break;
        case MOTOR_VAR_IA_ZERO_REF_ADCU:              Motor_Config_SetIaZero_Adcu(p_motor, varValue);               break;
        case MOTOR_VAR_IB_ZERO_REF_ADCU:              Motor_Config_SetIbZero_Adcu(p_motor, varValue);               break;
        case MOTOR_VAR_IC_ZERO_REF_ADCU:              Motor_Config_SetIcZero_Adcu(p_motor, varValue);               break;
        case MOTOR_VAR_I_PEAK_REF_ADCU:               Motor_Config_SetIPeakRef_Adcu(p_motor, varValue);             break;
    }
}

int32_t Motor_VarConfig_Actuation_Get(const Motor_T * p_motor, Motor_VarConfig_Actuation_T varId)
{
    int32_t value = 0;
    switch (varId)
    {
        case MOTOR_VAR_BASE_SPEED_LIMIT_FORWARD:    value = Motor_Config_GetSpeedLimitForward_Fract16(p_motor);     break;
        case MOTOR_VAR_BASE_SPEED_LIMIT_REVERSE:    value = Motor_Config_GetSpeedLimitReverse_Fract16(p_motor);     break;
        case MOTOR_VAR_BASE_I_LIMIT_MOTORING:       value = Motor_Config_GetILimitMotoring_Fract16(p_motor);        break;
        case MOTOR_VAR_BASE_I_LIMIT_GENERATING:     value = Motor_Config_GetILimitGenerating_Fract16(p_motor);      break;
        case MOTOR_VAR_RAMP_ACCEL_TIME:             value = Motor_Config_GetRampAccel_Millis(p_motor);              break;
        // case MOTOR_VAR_ALIGN_MODE:               value = Motor_Config_GetAlignMode(p_motor);                 break;
        case MOTOR_VAR_ALIGN_POWER:                 value = Motor_Config_GetAlignPower_Fract16(p_motor);            break;
        case MOTOR_VAR_ALIGN_TIME:                  value = Motor_Config_GetAlignTime_Millis(p_motor);              break;
        #if defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE) || defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)
        case MOTOR_VAR_OPEN_LOOP_POWER:             value = Motor_Config_GetOpenLoopPower_Fract16(p_motor);         break;
        case MOTOR_VAR_OPEN_LOOP_SPEED:             value = Motor_Config_GetOpenLoopSpeed_Fract16(p_motor);         break;
        case MOTOR_VAR_OPEN_LOOP_ACCEL_TIME:        value = Motor_Config_GetOpenLoopAccel_Millis(p_motor);          break;
        #endif
        #if defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
        case MOTOR_VAR_PHASE_PWM_MODE:              value = Motor_Config_GetPhaseModeParam(p_motor);                break;
        #endif
    }
    return value;
}

void Motor_VarConfig_Actuation_Set(Motor_T * p_motor, Motor_VarConfig_Actuation_T varId, int32_t varValue)
{
    switch (varId)
    {
        case MOTOR_VAR_BASE_SPEED_LIMIT_FORWARD:      Motor_Config_SetSpeedLimitForward_Fract16(p_motor, varValue);  break;
        case MOTOR_VAR_BASE_SPEED_LIMIT_REVERSE:      Motor_Config_SetSpeedLimitReverse_Fract16(p_motor, varValue);  break;
        case MOTOR_VAR_BASE_I_LIMIT_MOTORING:         Motor_Config_SetILimitMotoring_Fract16(p_motor, varValue);     break;
        case MOTOR_VAR_BASE_I_LIMIT_GENERATING:       Motor_Config_SetILimitGenerating_Fract16(p_motor, varValue);   break;
        case MOTOR_VAR_RAMP_ACCEL_TIME:               Motor_Config_SetRampAccel_Millis(p_motor, varValue);           break;
        case MOTOR_VAR_ALIGN_POWER:                   Motor_Config_SetAlignPower_Fract16(p_motor, varValue);         break;
        case MOTOR_VAR_ALIGN_TIME:                    Motor_Config_SetAlignTime_Millis(p_motor, varValue);           break;
        case MOTOR_VAR_OPEN_LOOP_POWER:               Motor_Config_SetOpenLoopPower_Fract16(p_motor, varValue);      break;
        case MOTOR_VAR_OPEN_LOOP_SPEED:               Motor_Config_SetOpenLoopSpeed_Fract16(p_motor, varValue);      break;
        case MOTOR_VAR_OPEN_LOOP_ACCEL_TIME:          Motor_Config_SetOpenLoopAccel_Millis(p_motor, varValue);       break;
            // case MOTOR_VAR_PHASE_PWM_MODE:             Motor_Config_SetPhaseModeParam(p_motor,  varValue);  break;
    }
}

int32_t Motor_VarConfig_Hall_Get(const Motor_T * p_motor, Motor_VarConfig_Hall_T varId)
{
    int32_t value = 0;
    switch (varId)
    {
        case MOTOR_VAR_HALL_SENSOR_TABLE_1: value = p_motor->Hall.Config.SensorsTable[1U]; break;
        case MOTOR_VAR_HALL_SENSOR_TABLE_2: value = p_motor->Hall.Config.SensorsTable[2U]; break;
        case MOTOR_VAR_HALL_SENSOR_TABLE_3: value = p_motor->Hall.Config.SensorsTable[3U]; break;
        case MOTOR_VAR_HALL_SENSOR_TABLE_4: value = p_motor->Hall.Config.SensorsTable[4U]; break;
        case MOTOR_VAR_HALL_SENSOR_TABLE_5: value = p_motor->Hall.Config.SensorsTable[5U]; break;
        case MOTOR_VAR_HALL_SENSOR_TABLE_6: value = p_motor->Hall.Config.SensorsTable[6U]; break;
    }

    return value;
}

void Motor_VarConfig_Hall_Set(Motor_T * p_motor, Motor_VarConfig_Hall_T varId, int32_t varValue)
{
    switch (varId)
    {
        case MOTOR_VAR_HALL_SENSOR_TABLE_1: p_motor->Hall.Config.SensorsTable[1U] = varValue; break;
        case MOTOR_VAR_HALL_SENSOR_TABLE_2: p_motor->Hall.Config.SensorsTable[2U] = varValue; break;
        case MOTOR_VAR_HALL_SENSOR_TABLE_3: p_motor->Hall.Config.SensorsTable[3U] = varValue; break;
        case MOTOR_VAR_HALL_SENSOR_TABLE_4: p_motor->Hall.Config.SensorsTable[4U] = varValue; break;
        case MOTOR_VAR_HALL_SENSOR_TABLE_5: p_motor->Hall.Config.SensorsTable[5U] = varValue; break;
        case MOTOR_VAR_HALL_SENSOR_TABLE_6: p_motor->Hall.Config.SensorsTable[6U] = varValue; break;
    }
}

// move to encoder
int32_t Motor_VarConfig_Encoder_Get(const Motor_T * p_motor, Motor_VarConfig_Encoder_T varId)
{
    int32_t value = 0;
    switch (varId)
    {
        case MOTOR_VAR_ENCODER_COUNTS_PER_REVOLUTION:             value = p_motor->Encoder.Config.CountsPerRevolution;            break;
        case MOTOR_VAR_ENCODER_IS_QUADRATURE_CAPTURE_ENABLED:     value = p_motor->Encoder.Config.IsQuadratureCaptureEnabled;     break;
        case MOTOR_VAR_ENCODER_IS_A_LEAD_B_POSITIVE:              value = p_motor->Encoder.Config.IsALeadBPositive;               break;
        case MOTOR_VAR_ENCODER_EXTENDED_TIMER_DELTA_T_STOP:       value = p_motor->Encoder.Config.ExtendedDeltaTStop;             break;
        case MOTOR_VAR_ENCODER_INTERPOLATE_ANGLE_SCALAR:          value = 0;    break;
    }
    return value;
}

void Motor_VarConfig_Encoder_Set(Motor_T * p_motor, Motor_VarConfig_Encoder_T varId, int32_t varValue)
{
    switch (varId)
    {
        case MOTOR_VAR_ENCODER_COUNTS_PER_REVOLUTION:             p_motor->Encoder.Config.CountsPerRevolution = varValue;            break;
        case MOTOR_VAR_ENCODER_IS_QUADRATURE_CAPTURE_ENABLED:     p_motor->Encoder.Config.IsQuadratureCaptureEnabled = varValue;     break;
        case MOTOR_VAR_ENCODER_IS_A_LEAD_B_POSITIVE:              p_motor->Encoder.Config.IsALeadBPositive = varValue;               break;
        case MOTOR_VAR_ENCODER_EXTENDED_TIMER_DELTA_T_STOP:       p_motor->Encoder.Config.ExtendedDeltaTStop = varValue;             break;
        case MOTOR_VAR_ENCODER_INTERPOLATE_ANGLE_SCALAR:          break;
    }
}

int32_t Motor_VarConfig_SinCos_Get(const Motor_T * p_motor, Motor_VarConfig_SinCos_T varId)
{
    int32_t value = 0;
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
    switch (varId)
    {
        case MOTOR_VAR_SIN_COS_ZERO_ADCU:               value = p_motor->SinCos.Config.Zero_Adcu;                           break;
        case MOTOR_VAR_SIN_COS_MAX_ADCU:                value = p_motor->SinCos.Config.Max_Adcu;                            break;
        case MOTOR_VAR_SIN_COS_MAX_MILLIV:              value = p_motor->SinCos.Config.Max_Milliv;                          break;
        case MOTOR_VAR_SIN_COS_ANGLE_OFFSET:            value = p_motor->SinCos.Config.AngleOffset;                         break;
        case MOTOR_VAR_SIN_COS_IS_B_POSITIVE:           value = p_motor->SinCos.Config.IsBPositive;                         break;
        case MOTOR_VAR_SIN_COS_ELECTRICAL_ROTATIONS_PER_CYCLE: value = p_motor->SinCos.Config.ElectricalRotationsPerCycle;  break;
    }
#endif
    return value;
}

void Motor_VarConfig_SinCos_Set(Motor_T * p_motor, Motor_VarConfig_SinCos_T varId, int32_t varValue)
{
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
    switch (varId)
    {
        case MOTOR_VAR_SIN_COS_ZERO_ADCU:               p_motor->SinCos.Config.Zero_Adcu = varValue;                           break;
        case MOTOR_VAR_SIN_COS_MAX_ADCU:                p_motor->SinCos.Config.Max_Adcu = varValue;                            break;
        case MOTOR_VAR_SIN_COS_MAX_MILLIV:              p_motor->SinCos.Config.Max_Milliv = varValue;                          break;
        case MOTOR_VAR_SIN_COS_ANGLE_OFFSET:            p_motor->SinCos.Config.AngleOffset = varValue;                         break;
        case MOTOR_VAR_SIN_COS_IS_B_POSITIVE:           p_motor->SinCos.Config.IsBPositive = varValue;                         break;
        case MOTOR_VAR_SIN_COS_ELECTRICAL_ROTATIONS_PER_CYCLE: p_motor->SinCos.Config.ElectricalRotationsPerCycle = varValue;  break;
    }
#endif
}

// remap thermistor todo
int32_t Motor_VarConfig_Thermistor_Get(const Motor_T * p_motor, Thermistor_ConfigId_T varId)
{
    return Thermistor_ConfigId_Get(&p_motor->Thermistor, varId);
}

void Motor_VarConfig_Thermistor_Set(Motor_T * p_motor, Thermistor_ConfigId_T varId, int32_t varValue)
{
    Thermistor_ConfigId_Set(&(p_motor->Thermistor), varId, varValue);
}

int32_t Motor_VarConfig_Pid_Get(const Motor_T * p_motor, Motor_VarConfig_Pid_T varId)
{
    int32_t value = 0;
    switch (varId)
    {
        case MOTOR_VAR_PID_SPEED_SAMPLE_FREQ:     value = PID_GetSampleFreq(&p_motor->PidSpeed);   break;
        case MOTOR_VAR_PID_SPEED_KP_FIXED16:      value = PID_GetKp_Fixed16(&p_motor->PidSpeed);   break;
        case MOTOR_VAR_PID_SPEED_KI_FIXED16:      value = PID_GetKi_Fixed16(&p_motor->PidSpeed);   break;
        // case MOTOR_VAR_PID_SPEED_KD_FIXED16:      value = PID_GetKd_Fixed16(&p_motor->PidSpeed);   break;
        case MOTOR_VAR_PID_FOC_IQ_SAMPLE_FREQ:    value = PID_GetSampleFreq(&p_motor->PidIq);     break;
        case MOTOR_VAR_PID_FOC_IQ_KP_FIXED16:     value = PID_GetKp_Fixed16(&p_motor->PidIq);     break;
        case MOTOR_VAR_PID_FOC_IQ_KI_FIXED16:     value = PID_GetKi_Fixed16(&p_motor->PidIq);     break;
        // case MOTOR_VAR_PID_FOC_IQ_KD_FIXED16:     value = PID_GetKd_Fixed16(&p_motor->PidIq);     break;

        // case MOTOR_VAR_PID_FOC_ID_SAMPLE_FREQ:    value = PID_GetSampleFreq(&p_motor->PidId);     break;
        // case MOTOR_VAR_PID_FOC_ID_KP_FIXED16:     value = PID_GetKp_Fixed16(&p_motor->PidId);     break;
        // case MOTOR_VAR_PID_FOC_ID_KI_FIXED16:     value = PID_GetKi_Fixed16(&p_motor->PidId);     break;
        // case MOTOR_VAR_PID_FOC_ID_KD_FIXED16:     value = PID_GetKd_Fixed16(&p_motor->PidId);     break;
    }
    return value;
}

void Motor_VarConfig_Pid_Set(Motor_T * p_motor, Motor_VarConfig_Pid_T varId, int32_t varValue)
{
    switch (varId)
    {
        // case MOTOR_VAR_PID_SPEED_SAMPLE_FREQ:  PID_SetSampleFreq(&p_motor->PidSpeed, varValue);          break;
        case MOTOR_VAR_PID_SPEED_KP_FIXED16:  PID_SetKp_Fixed16(&p_motor->PidSpeed, varValue);    break;
        case MOTOR_VAR_PID_SPEED_KI_FIXED16:  PID_SetKi_Fixed16(&p_motor->PidSpeed, varValue);    break;
        // case MOTOR_VAR_PID_SPEED_KD_FIXED16:  PID_SetKd_Fixed16(&p_motor->PidSpeed, varValue);       break;
        // case MOTOR_VAR_PID_FOC_IQ_SAMPLE_FREQ: PID_SetSampleFreq(&p_motor->PidIq, varValue);         break;
        case MOTOR_VAR_PID_FOC_IQ_KP_FIXED16: PID_SetKp_Fixed16(&p_motor->PidIq, varValue);       break;
        case MOTOR_VAR_PID_FOC_IQ_KI_FIXED16: PID_SetKi_Fixed16(&p_motor->PidIq, varValue);       break;
        // case MOTOR_VAR_PID_FOC_IQ_KD_FIXED16: PID_SetKd_Fixed16(&p_motor->PidIq, varValue);          break;

        // case MOTOR_VAR_PID_FOC_ID_KP_FIXED16: PID_SetKp_Fixed16(&p_motor->PidId, varValue);          break;
        // case MOTOR_VAR_PID_FOC_ID_KI_FIXED16: PID_SetKi_Fixed16(&p_motor->PidId, varValue);          break;
        // case MOTOR_VAR_PID_FOC_ID_KD_FIXED16: PID_SetKd_Fixed16(&p_motor->PidId, varValue);          break;
        // case MOTOR_VAR_PID_FOC_ID_SAMPLE_FREQ: PID_SetSampleFreq(&p_motor->PidId, varValue);         break;
    }
}
