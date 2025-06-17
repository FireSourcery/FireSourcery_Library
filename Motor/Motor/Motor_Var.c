

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

#include "Motor_User.h"
#include "Motor_Config.h"
// #include "Motor_FOC.h"
#include "Motor.h"


/******************************************************************************/
/*
    Var RealTime/RunTime
*/
/******************************************************************************/
int32_t _Motor_Var_UserOut_Get(const Motor_State_T * p_motor, Motor_Var_UserOut_T varId)
{
    int32_t value = 0;
    switch (varId)
    {
        case MOTOR_VAR_SPEED:                       value = Motor_User_GetSpeed_UFract16(p_motor);              break;
        // case MOTOR_VAR_SPEED:                       value = Motor_User_GetSpeed_DegPerCycle(p_motor);              break;
        case MOTOR_VAR_I_PHASE:                     value = Motor_User_GetIPhase_UFract16(p_motor);             break;
        case MOTOR_VAR_V_PHASE:                     value = Motor_User_GetVPhase_UFract16(p_motor);             break;
        case MOTOR_VAR_POWER:                       value = Motor_User_GetElectricalPower_UFract16(p_motor);    break;
        case MOTOR_VAR_STATE:                       value = Motor_User_GetStateId(p_motor);                     break;
        case MOTOR_VAR_STATUS_FLAGS:                value = Motor_User_GetStatusFlags(p_motor).Value;           break;
        case MOTOR_VAR_FAULT_FLAGS:                 value = Motor_User_GetFaultFlags(p_motor).Value;            break;
        case MOTOR_VAR_HEAT:                        value = Motor_User_GetHeat_Adcu(p_motor);                   break;
        case MOTOR_VAR_V_SPEED_EFFECTIVE:           value = Motor_User_GetVSpeedEffective_UFract16(p_motor);    break;
        case MOTOR_VAR_ELECTRICAL_ANGLE:            value = Motor_User_GetElectricalAngle(p_motor);             break;
        // case MOTOR_VAR_MECHANICAL_ANGLE:            value = Motor_User_GetMechanicalAngle(p_motor);             break;
        case MOTOR_VAR_SUB_STATE:                   value = Motor_User_GetSubStateId(p_motor);                  break;
    }
    return value;
}

int32_t _Motor_Var_Foc_Get(const Motor_State_T * p_motor, Motor_Var_FocOut_T varId)
{
    int32_t value = 0;
    switch (varId)
    {
        case MOTOR_VAR_FOC_IA:      value = p_motor->Foc.Ia;        break;
        case MOTOR_VAR_FOC_IB:      value = p_motor->Foc.Ib;        break;
        case MOTOR_VAR_FOC_IC:      value = p_motor->Foc.Ic;        break;
        // case MOTOR_VAR_FOC_IA:    value = Motor_Analog_GetIa(p_motor) - p_motor->Config.IaZeroRef_Adcu;  break;
        // case MOTOR_VAR_FOC_IB:    value = Motor_Analog_GetIb(p_motor) - p_motor->Config.IbZeroRef_Adcu;  break;
        // case MOTOR_VAR_FOC_IC:    value = Motor_Analog_GetIc(p_motor) - p_motor->Config.IcZeroRef_Adcu;  break;
        case MOTOR_VAR_FOC_IQ:      value = p_motor->Foc.Iq;        break;
        case MOTOR_VAR_FOC_ID:      value = p_motor->Foc.Id;        break;
        case MOTOR_VAR_FOC_VQ:      value = p_motor->Foc.Vq;        break;
        case MOTOR_VAR_FOC_VD:      value = p_motor->Foc.Vd;        break;
        case MOTOR_VAR_FOC_REQ_Q:   value = p_motor->Foc.ReqQ;      break;
        case MOTOR_VAR_FOC_REQ_D:   value = p_motor->Foc.ReqD;      break;
        case MOTOR_VAR_FOC_VA:      value = p_motor->Foc.Va;        break;
        case MOTOR_VAR_FOC_VB:      value = p_motor->Foc.Vb;        break;
        case MOTOR_VAR_FOC_VC:      value = p_motor->Foc.Vc;        break;
        case MOTOR_VAR_FOC_INTEGRAL_Q:    value = PID_GetIntegral(&p_motor->PidIq);        break;
        case MOTOR_VAR_FOC_INTEGRAL_D:    value = PID_GetIntegral(&p_motor->PidId);        break;
    }
    return value;
}

// int32_t _Motor_VarEffective_Get(const Motor_State_T * p_motor, Motor_VarOuput__T varId)
// {
// // case MOTOR_VAR_EFFECTIVE_FEEDBACK_MODE:     value = Motor_User_GetFeedbackMode(p_motor).Value;          break;
// // case MOTOR_VAR_EFFECTIVE_SET_POINT:         value = Motor_User_GetSetPoint(p_motor);                    break;
// // case MOTOR_VAR_EFFECTIVE_SPEED_LIMIT:       value = Motor_User_GetSpeedLimit(p_motor);                  break;
// // case MOTOR_VAR_EFFECTIVE_I_LIMIT:           value = Motor_User_GetILimit(p_motor);                      break;
// }


/*
    Runtime Tunning version
*/
int32_t _Motor_Var_PidTuning_Get(const Motor_State_T * p_motor, Motor_VarConfig_Pid_T varId)
{
    int32_t value = 0;
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

void _Motor_Var_PidTuning_Set(Motor_State_T * p_motor, Motor_VarConfig_Pid_T varId, int32_t varValue)
{
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



/******************************************************************************/
/*
    Input/Cmds
    Full context
*/
/******************************************************************************/
void _Motor_Var_Cmd_Set(const Motor_T * p_motor, Motor_Var_Cmd_T varId, int32_t varValue)
{
    switch (varId)
    {
        case MOTOR_VAR_CLEAR_FAULT:             Motor_StateMachine_ClearFault(p_motor, (Motor_FaultFlags_T) { .Value = varValue }); break; /* var unused for now */
        case MOTOR_VAR_FORCE_DISABLE_CONTROL:   Motor_User_ForceDisableControl(p_motor); break;

        case MOTOR_VAR_OPEN_LOOP_CONTROL:       Motor_OpenLoop_Enter(p_motor);                                      break;
        case MOTOR_VAR_OPEN_LOOP_PHASE_OUTPUT:  Motor_OpenLoop_SetPhaseOutput(p_motor, (Phase_Output_T)varValue);   break;
        case MOTOR_VAR_OPEN_LOOP_PHASE_ALIGN:   Motor_OpenLoop_SetPhaseAlign(p_motor, (Phase_Id_T)varValue);        break;
        case MOTOR_VAR_OPEN_LOOP_ANGLE_ALIGN:   Motor_OpenLoop_SetAngleAlign(p_motor, varValue);                    break;
        case MOTOR_VAR_OPEN_LOOP_JOG:           Motor_OpenLoop_SetJog(p_motor, varValue);                           break;
        /*  */
        case MOTOR_VAR_OPEN_LOOP_RUN:           Motor_OpenLoop_StartRunChain(p_motor);                              break;
        // case MOTOR_VAR_OPEN_LOOP_HOMING:     break;

        // case MOTOR_VAR_USER_CMD:        Motor_User_SetActiveCmdValue(p_motor, varValue);    break;
        // case MOTOR_VAR_CMD_SPEED:       Motor_User_SetSpeedCmd(p_motor, varValue);     break;
        // case MOTOR_VAR_CMD_CURRENT:     Motor_User_SetICmd(p_motor, varValue);         break;
        // case MOTOR_VAR_CMD_VOLTAGE:     Motor_User_SetVoltageCmd(p_motor, varValue);   break;
        // case MOTOR_VAR_CMD_ANGLE:       Motor_User_SetPositionCmd(p_motor, varValue);  break;
        // case MOTOR_VAR_CMD_OPEN_LOOP:   Motor_User_SetOpenLoopCmd(p_motor, varValue);  break;
    }
}

int32_t _Motor_Var_UserIO_Get(const Motor_T * p_motor, Motor_Var_UserIO_T varId)
{
    int32_t value = 0;
    switch (varId)
    {
        case MOTOR_VAR_USER_SET_POINT:          value = Motor_User_GetSetPoint(p_motor->P_ACTIVE);                break;
        case MOTOR_VAR_USER_DIRECTION:          value = Motor_User_GetDirectionSign(p_motor->P_ACTIVE);           break;
        case MOTOR_VAR_USER_FEEDBACK_MODE:      value = Motor_User_GetFeedbackMode(p_motor->P_ACTIVE).Value;      break;
        // case MOTOR_VAR_USER_CONTROL_STATE:   value = Motor_User_GetPhaseState(p_motor);              break; /* effective Stop/hold/run */
        case MOTOR_VAR_USER_SPEED_LIMIT:        value = Motor_User_GetSpeedLimit(p_motor->P_ACTIVE);              break;
        // case MOTOR_VAR_USER_I_LIMIT:         value = Motor_User_GetILimit(p_motor);                  break;
        case MOTOR_VAR_USER_I_LIMIT_MOTORING:        value = Motor_User_GetILimitMotoring(p_motor->P_ACTIVE);     break;
        case MOTOR_VAR_USER_I_LIMIT_GENERATING:      value = Motor_User_GetILimitGenerating(p_motor->P_ACTIVE);   break;
        /* RAMP_ON_OFF */
    }
    return value;
}

void _Motor_Var_UserIO_Set(const Motor_T * p_motor, Motor_Var_UserIO_T varId, int32_t varValue)
{
    switch (varId)
    {
        case MOTOR_VAR_USER_SET_POINT:      Motor_User_SetActiveCmdValue_Scalar(p_motor->P_ACTIVE, varValue);           break;
        case MOTOR_VAR_USER_DIRECTION:      Motor_User_ApplyDirectionSign(p_motor, math_sign(varValue));                break;
        case MOTOR_VAR_USER_FEEDBACK_MODE:  Motor_User_SetFeedbackMode_Cast(p_motor, (uint8_t)varValue);                    break;
        case MOTOR_VAR_USER_CONTROL_STATE:  Motor_User_ActivateControlState(p_motor, (Phase_Output_T)varValue);             break;
        // case MOTOR_VAR_USER_SPEED_LIMIT:    Motor_User_TrySpeedLimit(p_motor->P_ACTIVE, varValue);                        break;
        // case MOTOR_VAR_USER_I_LIMIT:        Motor_User_TryILimit(p_motor->P_ACTIVE, varValue);                            break;
    }
}


/******************************************************************************/
/*
    Config
*/
/******************************************************************************/
int32_t _Motor_VarConfig_Calibration_Get(const Motor_State_T * p_motor, Motor_VarConfig_Calibration_T varId)
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
        case MOTOR_VAR_SPEED_RATED_DEG:               value = Motor_Config_GetSpeedRated(p_motor);                break;
        case MOTOR_VAR_IA_ZERO_ADCU:                  value = Motor_Config_GetIaZero_Adcu(p_motor);               break;
        case MOTOR_VAR_IB_ZERO_ADCU:                  value = Motor_Config_GetIbZero_Adcu(p_motor);               break;
        case MOTOR_VAR_IC_ZERO_ADCU:                  value = Motor_Config_GetIcZero_Adcu(p_motor);               break;
        // case MOTOR_VAR_I_PEAK_REF_ADCU:               value = Motor_Config_GetIPeakRef_Adcu(p_motor);             break;
    }
    return value;
}

void _Motor_VarConfig_Calibration_Set(Motor_State_T * p_motor, Motor_VarConfig_Calibration_T varId, int32_t varValue)
{
    switch (varId)
    {
        case MOTOR_VAR_COMMUTATION_MODE:              Motor_Config_SetCommutationMode(p_motor, varValue);           break;
        case MOTOR_VAR_SENSOR_MODE:                   Motor_Config_SetSensorMode(p_motor, varValue);                break;
        case MOTOR_VAR_DIRECTION_CALIBRATION:         Motor_Config_SetDirectionCalibration(p_motor, varValue);      break;
        case MOTOR_VAR_POLE_PAIRS:                    Motor_Config_SetPolePairs(p_motor, varValue);                 break;
        case MOTOR_VAR_KV:                            Motor_Config_SetKv(p_motor, varValue);                        break;
        case MOTOR_VAR_V_SPEED_SCALAR:                Motor_Config_SetVSpeedScalar_UFract16(p_motor, varValue);     break;
        case MOTOR_VAR_SPEED_RATED_DEG:               Motor_Config_SetSpeedRated(p_motor, varValue);                break;
        case MOTOR_VAR_IA_ZERO_ADCU:                  Motor_Config_SetIaZero_Adcu(p_motor, varValue);               break;
        case MOTOR_VAR_IB_ZERO_ADCU:                  Motor_Config_SetIbZero_Adcu(p_motor, varValue);               break;
        case MOTOR_VAR_IC_ZERO_ADCU:                  Motor_Config_SetIcZero_Adcu(p_motor, varValue);               break;
        // case MOTOR_VAR_I_PEAK_REF_ADCU:               Motor_Config_SetIPeakRef_Adcu(p_motor, varValue);             break;
        // case MOTOR_VAR_SPEED_V_REF_RPM:               Motor_Config_SetSpeedVRef_Rpm(p_motor, varValue);             break;
        // case MOTOR_VAR_SPEED_V_MATCH_REF_RPM:         Motor_Config_SetSpeedVMatchRef_Rpm(p_motor, varValue);        break;

    }
}

int32_t _Motor_VarConfig_CalibrationAlias_Get(const Motor_State_T * p_motor, Motor_VarConfig_CalibrationAlias_T varId)
{
    int32_t value = 0;
    switch (varId)
    {
        case MOTOR_VAR_SPEED_RATED_RPM:               value = Motor_Config_GetSpeedRated_Rpm(p_motor);             break;
        case MOTOR_VAR_SPEED_V_REF_RPM:               value = Motor_Config_GetSpeedVRef_Rpm(p_motor);              break;
        case MOTOR_VAR_SPEED_V_MATCH_REF_RPM:         value = Motor_Config_GetSpeedVMatchRef_Rpm(p_motor);         break;
        case MOTOR_VAR_SPEED_V_SVPWM_REF_RPM:         value = Motor_Config_GetSpeedVSvpwmRef_Rpm(p_motor);         break;
        // case MOTOR_VAR_SPEED_RATED_ERPM:              value = Motor_GetSpeedRatedRef_Erpm(p_motor);                break;
        case MOTOR_VAR_SPEED_V_REF_DEG_PER_CYCLE:     value = Motor_GetSpeedVRef_DegPerCycle(p_motor);             break;
        // case MOTOR_VAR_SPEED_V_MATCH_REF_DEG_PER_CYCLE: value = Motor_GetSpeedVMatchRef_DegPerCycle(p_motor);       break;
        case MOTOR_VAR_SPEED_V_SVPWM_REF_DEG_PER_CYCLE: value = Motor_GetSpeedVSvpwmRef_DegPerCycle(p_motor);       break;
        case MOTOR_VAR_V_SPEED_RATED_FRACT16:           value = Motor_GetVSpeedRated_Fract16(p_motor);       break;

    }
    return value;
}

int32_t _Motor_VarConfig_Actuation_Get(const Motor_State_T * p_motor, Motor_VarConfig_Actuation_T varId)
{
    int32_t value = 0;
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
    // #if defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE) || defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)
        case MOTOR_VAR_OPEN_LOOP_RAMP_I_FINAL:      value = Motor_Config_GetOpenLoopI_Fract16(p_motor);             break;
        case MOTOR_VAR_OPEN_LOOP_RAMP_I_TIME:       value = Motor_Config_GetOpenLoopIRamp_Millis(p_motor);          break;
        case MOTOR_VAR_OPEN_LOOP_RAMP_SPEED_FINAL:  value = Motor_Config_GetOpenLoopSpeed_Fract16(p_motor);         break;
        case MOTOR_VAR_OPEN_LOOP_RAMP_SPEED_TIME:   value = Motor_Config_GetOpenLoopSpeedRamp_Millis(p_motor);      break;
    // #endif
    // #if defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
        // case MOTOR_VAR_PHASE_POLAR_MODE:         value = Motor_Config_GetPhasePolarMode(p_motor);                break;
    // #endif
    }
    return value;
}

void _Motor_VarConfig_Actuation_Set(Motor_State_T * p_motor, Motor_VarConfig_Actuation_T varId, int32_t varValue)
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


int32_t _Motor_VarConfig_Pid_Get(const Motor_State_T * p_motor, Motor_VarConfig_Pid_T varId)
{
    int32_t value = 0;
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
void _Motor_VarConfig_Pid_Set(Motor_State_T * p_motor, Motor_VarConfig_Pid_T varId, int32_t varValue)
{
    switch (varId)
    {
        case MOTOR_VAR_PID_SPEED_SAMPLE_FREQ:   break;
        case MOTOR_VAR_PID_SPEED_KP_FIXED16:
            p_motor->Config.PidSpeed.Kp_Fixed32 = varValue << 8;
            PID_SetKp_Fixed16(&p_motor->PidSpeed, varValue);
            break;
        case MOTOR_VAR_PID_SPEED_KI_FIXED16:
            p_motor->Config.PidSpeed.Ki_Fixed32 = varValue << 8;
            PID_SetKi_Fixed16(&p_motor->PidSpeed, varValue);
            break;
        // case MOTOR_VAR_PID_SPEED_KD_FIXED16:     PID_SetKd_Fixed16(&p_motor->PidSpeed, varValue);        break;
        case MOTOR_VAR_PID_CURRENT_SAMPLE_FREQ: break;
        case MOTOR_VAR_PID_CURRENT_KP_FIXED16:
            p_motor->Config.PidI.Kp_Fixed32 = varValue << 8;
            PID_SetKp_Fixed16(&p_motor->PidIq, varValue);
            PID_SetKp_Fixed16(&p_motor->PidId, varValue);
            break;
        case MOTOR_VAR_PID_CURRENT_KI_FIXED16:
            p_motor->Config.PidI.Ki_Fixed32 = varValue << 8;
            PID_SetKi_Fixed16(&p_motor->PidIq, varValue);
            PID_SetKi_Fixed16(&p_motor->PidId, varValue);
            break;

        // case MOTOR_VAR_PID_CURRENT_KD_FIXED16  PID_SetKd_Fixed16(&p_motor->PidSpeed, varValue);      break;
        // case MOTOR_VAR_PID_FOC_IQ_SAMPLE_FREQ: PID_SetSampleFreq(&p_motor->PidIq, varValue);         break;
        // case MOTOR_VAR_PID_FOC_IQ_KP_FIXED16:        //     break;
        // case MOTOR_VAR_PID_FOC_IQ_KI_FIXED16:        //     break;
        // case MOTOR_VAR_PID_FOC_IQ_KD_FIXED16: PID_SetKd_Fixed16(&p_motor->PidIq, varValue);          break;
        // case MOTOR_VAR_PID_FOC_ID_SAMPLE_FREQ: PID_SetSampleFreq(&p_motor->PidId, varValue);         break;
        // case MOTOR_VAR_PID_FOC_ID_KP_FIXED16: PID_SetKp_Fixed16(&p_motor->PidId, varValue);          break;
        // case MOTOR_VAR_PID_FOC_ID_KI_FIXED16: PID_SetKi_Fixed16(&p_motor->PidId, varValue);          break;
        // case MOTOR_VAR_PID_FOC_ID_KD_FIXED16: PID_SetKd_Fixed16(&p_motor->PidId, varValue);          break;
    }
}


/*

*/
void _Motor_VarConfig_Routine_Call(const Motor_T * p_motor, Motor_VarConfig_Routine_T varId, int32_t varValue)
{
    switch (varId)
    {
        // case MOTOR_VAR_CONFIG_RUN_SENSOR_CALIBRATION:   Motor_Hall_Calibrate(p_motor);              break;
        case MOTOR_VAR_CONFIG_RUN_ADC_CALIBRATION:      Motor_Analog_Calibrate(p_motor);            break;
        case MOTOR_VAR_CONFIG_RUN_VIRTUAL_HOME:         Motor_Calibration_StartHome(p_motor);       break;
        // case MOTOR_VAR_CONFIG_CMD_ENCODER_HOME:  Motor_Encoder_StartHoming(p_motor);         break;
        // case MOTOR_VAR_CONFIG_CMD_VIRTUAL_HOME:  Motor_Calibration_StartHome(p_motor);       break;
        // case ENCODER_CONFIG_RUN_HOMING:          Motor_Encoder_StartHoming(p_motor);        break;
    }
}

/*
    const
*/
int32_t Motor_VarRef_Get(Motor_VarRef_T varId)
{
    int32_t value = 0;
    switch (varId)
    {
        case MOTOR_VAR_REF_V_RATED:                 value = MotorAnalogRef_GetVRated_Fract16();                 break;
        case MOTOR_VAR_REF_I_RATED:                 value = MotorAnalogRef_GetIRatedPeak_Fract16();             break;
        case MOTOR_VAR_REF_V_MAX:                   value = MotorAnalogRef_GetVMaxVolts();                      break;
        case MOTOR_VAR_REF_I_MAX:                   value = MotorAnalogRef_GetIMaxAmps();                       break;
        case MOTOR_VAR_REF_V_MAX_ADCU:              value = MOTOR_ANALOG_V_MAX_ADCU;                            break;
        case MOTOR_VAR_REF_I_MAX_ADCU:              value = MOTOR_ANALOG_I_MAX_ADCU;                            break;

        case MOTOR_VAR_REF_BOARD_V_RATED_VOLTS:     value = MotorAnalogRef_GetVRated();                         break;
        case MOTOR_VAR_REF_BOARD_I_RATED_AMPS:      value = MotorAnalogRef_GetIRatedRms();                      break;
        case MOTOR_VAR_REF_V_PHASE_R1:              value = MOTOR_ANALOG_REFERENCE_BOARD.V_PHASE_R1 / 10;       break;
        case MOTOR_VAR_REF_V_PHASE_R2:              value = MOTOR_ANALOG_REFERENCE_BOARD.V_PHASE_R2 / 10;       break;
        case MOTOR_VAR_REF_I_PHASE_R_BASE:          value = MOTOR_ANALOG_REFERENCE_BOARD.I_PHASE_R_BASE;        break;
        case MOTOR_VAR_REF_I_PHASE_R_MOSFETS:       value = MOTOR_ANALOG_REFERENCE_BOARD.I_PHASE_R_MOSFETS;     break;
        case MOTOR_VAR_REF_I_PHASE_GAIN:            value = MOTOR_ANALOG_REFERENCE_BOARD.I_PHASE_GAIN;          break;
    }
    return value;
}





/*
    Generic Wrapper
*/
int Motor_Var_UserOut_Get(const Motor_State_T * p_motor, int varId) { return _Motor_Var_UserOut_Get(p_motor, varId); }
int Motor_Var_Foc_Get(const Motor_State_T * p_motor, int varId) { return _Motor_Var_Foc_Get(p_motor, varId); }


int Motor_Var_PidTunning_Get(const Motor_State_T * p_motor, int varId) { return _Motor_Var_PidTuning_Get(p_motor, varId); }
void Motor_Var_PidTunning_Set(Motor_State_T * p_motor, int varId, int varValue) { _Motor_Var_PidTuning_Set(p_motor, varId, varValue); }

void Motor_Var_Cmd_Set(const Motor_T * p_motor, int varId, int varValue) { _Motor_Var_Cmd_Set(p_motor, varId, varValue); }

int Motor_Var_UserIO_Get(const Motor_T * p_motor, int varId) { return _Motor_Var_UserIO_Get(p_motor, varId); }
void Motor_Var_UserIO_Set(const Motor_T * p_motor, int varId, int varValue) { _Motor_Var_UserIO_Set(p_motor, varId, varValue); }


int Motor_VarConfig_Calibration_Get(const Motor_State_T * p_motor, int varId) { return _Motor_VarConfig_Calibration_Get(p_motor, varId); }
int Motor_VarConfig_CalibrationAlias_Get(const Motor_State_T * p_motor, int varId) { return _Motor_VarConfig_CalibrationAlias_Get(p_motor, varId); }
int Motor_VarConfig_Actuation_Get(const Motor_State_T * p_motor, int varId) { return _Motor_VarConfig_Actuation_Get(p_motor, varId); }
int Motor_VarConfig_Pid_Get(const Motor_State_T * p_motor, int varId) { return _Motor_VarConfig_Pid_Get(p_motor, varId); }

void Motor_VarConfig_Calibration_Set(Motor_State_T * p_motor, int varId, int varValue) { _Motor_VarConfig_Calibration_Set(p_motor, varId, varValue); }
void Motor_VarConfig_Actuation_Set(Motor_State_T * p_motor, int varId, int varValue) { _Motor_VarConfig_Actuation_Set(p_motor, varId, varValue); }
void Motor_VarConfig_Pid_Set(Motor_State_T * p_motor, int varId, int varValue) { _Motor_VarConfig_Pid_Set(p_motor, varId, varValue); }

void Motor_VarConfig_Routine_Call(const Motor_T * p_motor, int varId, int varValue) { _Motor_VarConfig_Routine_Call(p_motor, varId, varValue); }

/*
    Submodule wrap
*/
int Motor_Var_Sensor_Get(const Motor_State_T * p_motor, int varId) { return MotorSensor_VarId_Get(p_motor->p_ActiveSensor, varId); }

// remap thermistor todo
// int32_t _Motor_VarConfig_HeatMonitor_Get(const Motor_State_T * p_motor, HeatMonitor_ConfigId_T varId) { return HeatMonitor_ConfigId_Get(&p_motor->Thermistor, varId); }
// void _Motor_VarConfig_HeatMonitor_Set(Motor_State_T * p_motor, HeatMonitor_ConfigId_T varId, int32_t varValue) { HeatMonitor_ConfigId_Set(&p_motor->Thermistor, varId, varValue); }

const VarAccess_VTable_T MOTOR_VAR_OUT_USER =
{
    .GET_AT = (get_at_t)Motor_Var_UserOut_Get,
    .SET_AT = NULL,
    .TEST_SET = NULL,
};

const VarAccess_VTable_T MOTOR_VAR_OUT_FOC =
{
    .GET_AT = (get_at_t)Motor_Var_Foc_Get,
    .SET_AT = NULL,
    .TEST_SET = NULL,
};

const VarAccess_VTable_T MOTOR_VAR_OUT_SENSOR =
{
    .GET_AT = (get_at_t)Motor_Var_Sensor_Get,
    .SET_AT = NULL,
    .TEST_SET = NULL,
};

const VarAccess_VTable_T MOTOR_VAR_IO_PID_TUNNING =
{
    .GET_AT = (get_at_t)Motor_Var_PidTunning_Get,
    .SET_AT = (set_at_t)Motor_Var_PidTunning_Set,
    .TEST_SET = NULL,
};

const VarAccess_VTable_T MOTOR_VAR_IN =
{
    .GET_AT = NULL,
    .SET_AT = (set_at_t)Motor_Var_Cmd_Set,
    .TEST_SET = NULL,
};

const VarAccess_VTable_T MOTOR_VAR_IO =
{
    .GET_AT = (get_at_t)Motor_Var_UserIO_Get,
    .SET_AT = (set_at_t)Motor_Var_UserIO_Set,
    .TEST_SET = NULL,
};

void Motor_Var_Cmd_Disable(Motor_State_T * p_motor) { _VarAccess_DisableSet(&p_motor->VarAccessInputState); }
void Motor_Var_Cmd_Enable(Motor_State_T * p_motor) { _VarAccess_EnableSet(&p_motor->VarAccessInputState); }

// void Motor_Var_UserIO_Set(const Motor_T * p_motor, Motor_Var_UserIO_T varId, int32_t varValue)
// {
//     VarAccess_SetAt(&p_motor->VAR_ACCESS.IO, varId, varValue);
// }

const VarAccess_VTable_T MOTOR_VAR_CONFIG_CALIBRATION =
{
    .GET_AT = (get_at_t)Motor_VarConfig_Calibration_Get,
    .SET_AT = (set_at_t)Motor_VarConfig_Calibration_Set,
    .TEST_SET = (test_t)Motor_Config_IsConfigState,
};

const VarAccess_VTable_T MOTOR_VAR_CONFIG_CALIBRATION_ALIAS =
{
    .GET_AT = (get_at_t)Motor_VarConfig_CalibrationAlias_Get,
    .SET_AT = NULL,
    .TEST_SET = NULL,
};

const VarAccess_VTable_T MOTOR_VAR_CONFIG_ACTUATION =
{
    .GET_AT = (get_at_t)Motor_VarConfig_Actuation_Get,
    .SET_AT = (set_at_t)Motor_VarConfig_Actuation_Set,
    .TEST_SET = (test_t)Motor_Config_IsConfigState,
};


const VarAccess_VTable_T MOTOR_VAR_CONFIG_PID =
{
    .GET_AT = (get_at_t)Motor_VarConfig_Pid_Get,
    .SET_AT = (set_at_t)Motor_VarConfig_Pid_Set,
    .TEST_SET = NULL,
};

const VarAccess_VTable_T MOTOR_VAR_CONFIG_ROUTINE =
{
    .GET_AT = NULL,
    .SET_AT = (set_at_t)Motor_VarConfig_Routine_Call,
    .TEST_SET = (test_t)Motor_Config_IsConfigState, /* Check [StopState] to transition to [CalibrationState] */
};

int VarRef_Get(void * p_void, int varId)
{
    (void)p_void; // unused
    return Motor_VarRef_Get(varId);
}

const VarAccess_VTable_T MOTOR_VAR_REF =
{
    .GET_AT = (get_at_t)VarRef_Get,
    .SET_AT = NULL,
    .TEST_SET = NULL,
};



