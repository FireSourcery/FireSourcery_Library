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
    @brief  User Get Set Vars by Id, key-value paradigm
    @version V0
*/
/******************************************************************************/
#include "MotorController_Var.h"

/******************************************************************************/
/*!
    Helpers
*/
/******************************************************************************/
static int32_t GetParameterThermistor(const Thermistor_T * p_thermistor, MotVarId_Config_Thermistor_T nameId)
{
    int32_t value = 0;
    if(p_thermistor != NULL)
    {
        switch(nameId)
        {
            case MOT_VAR_THERMISTOR_R_SERIES:                   value = p_thermistor->CONST.R_SERIES / 10U;                 break;
            case MOT_VAR_THERMISTOR_R_PARALLEL:                 value = p_thermistor->CONST.R_PARALLEL / 10U;               break;
            case MOT_VAR_THERMISTOR_R0:                         value = Thermistor_GetR0(p_thermistor) / 10U;               break;
            case MOT_VAR_THERMISTOR_T0:                         value = Thermistor_GetT0(p_thermistor);                     break;
            case MOT_VAR_THERMISTOR_B:                          value = Thermistor_GetB(p_thermistor);                      break;
            case MOT_VAR_THERMISTOR_FAULT_TRIGGER_ADCU:         value = Thermistor_GetFaultTrigger_Adcu(p_thermistor);      break;
            case MOT_VAR_THERMISTOR_FAULT_THRESHOLD_ADCU:       value = Thermistor_GetFaultThreshold_Adcu(p_thermistor);    break;
            case MOT_VAR_THERMISTOR_WARNING_TRIGGER_ADCU:       value = Thermistor_GetWarningTrigger_Adcu(p_thermistor);    break;
            case MOT_VAR_THERMISTOR_WARNING_THRESHOLD_ADCU:     value = Thermistor_GetWarningThreshold_Adcu(p_thermistor);  break;
            case MOT_VAR_THERMISTOR_IS_MONITOR_ENABLE:          value = Thermistor_IsMonitorEnable(p_thermistor);           break;
            // case MOT_VAR_THERMISTOR_TYPE:                       value = Thermistor_GetType(p_thermistor);                   break;
            default: break;
        }
    }
    return value;
}

/* Caller check instance */
static void SetParameterThermistor(Thermistor_T * p_thermistor, MotVarId_Config_Thermistor_T nameId, int32_t varValue)
{
    if(p_thermistor != NULL)
    {
        switch(nameId)
        {
            case MOT_VAR_THERMISTOR_R_SERIES:                   break;
            case MOT_VAR_THERMISTOR_R_PARALLEL:                 break;
            case MOT_VAR_THERMISTOR_R0:                         Thermistor_SetR0(p_thermistor, varValue);                     break;
            case MOT_VAR_THERMISTOR_T0:                         Thermistor_SetT0(p_thermistor, varValue);                     break;
            case MOT_VAR_THERMISTOR_B:                          Thermistor_SetB(p_thermistor, varValue);                      break;
            case MOT_VAR_THERMISTOR_FAULT_TRIGGER_ADCU:         Thermistor_SetFaultTrigger_Adcu(p_thermistor, varValue);      break;
            case MOT_VAR_THERMISTOR_FAULT_THRESHOLD_ADCU:       Thermistor_SetFaultThreshold_Adcu(p_thermistor, varValue);    break;
            case MOT_VAR_THERMISTOR_WARNING_TRIGGER_ADCU:       Thermistor_SetWarningTrigger_Adcu(p_thermistor, varValue);    break;
            case MOT_VAR_THERMISTOR_WARNING_THRESHOLD_ADCU:     Thermistor_SetWarningThreshold_Adcu(p_thermistor, varValue);  break;
            case MOT_VAR_THERMISTOR_IS_MONITOR_ENABLE:          Thermistor_SetIsMonitorEnable(p_thermistor, varValue);        break;
            // case MOT_VAR_THERMISTOR_TYPE:                       Thermistor_SetType(p_thermistor, varValue);                   break;
            default: break;
        }
    }
}

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
        case MOT_VAR_ID_TYPE_MONITOR_GENERAL:
            switch((MotVarId_Monitor_General_T)varId.NameBase)
            {
                case MOT_VAR_ZERO:                  value = 0;                                                      break;
                case MOT_VAR_MILLIS:                value = p_motor->ControlTimerBase;                              break;
                case MOT_VAR_DEBUG:                 value = p_motor->DebugTime[4];                                  break;
                // case MOT_VAR_DEBUG:                 value = Millis();                                               break;
                case MOT_VAR_MC_STATE:              value = MotorController_User_GetStateId(p_mc);                  break;
                case MOT_VAR_MC_STATE_FLAGS:        value = MotorController_User_GetStateFlags(p_mc).Word;          break;
                // case MOT_VAR_MC_STATUS_FLAGS:       value = MotorController_User_GetStatusFlags(p_mc).Word;          break;
                case MOT_VAR_MC_FAULT_FLAGS:        value = MotorController_User_GetFaultFlags(p_mc).Value;         break;
                case MOT_VAR_V_SOURCE:              value = MotorController_Analog_GetVSource(p_mc);                break;
                case MOT_VAR_V_SENSOR:              value = MotorController_Analog_GetVSense(p_mc);                 break;
                case MOT_VAR_V_ACCS:                value = MotorController_Analog_GetVAccs(p_mc);                  break;
                case MOT_VAR_HEAT_PCB:              value = MotorController_User_GetHeatPcb_Adcu(p_mc);             break;
                case MOT_VAR_HEAT_MOSFETS:          value = MotorController_User_GetHeatMosfets_Adcu(p_mc);         break;
                #ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
                // case MOT_VAR_HEAT_PCB_DEG_C:        value = MotorController_User_GetHeatPcb_DegC(p_mc, 1U);         break;
                // case MOT_VAR_HEAT_MOSFETS_DEG_C:    value = MotorController_User_GetHeatMosfets_DegC(p_mc, 1U);     break;
                case MOT_VAR_BATTERY_CHARGE:        value = MotorController_User_GetBatteryCharge_Percent16(p_mc);   break;
                #endif
                default: break;
            }
            break;

        case MOT_VAR_ID_TYPE_MONITOR_ANALOG_USER:
            switch((MotVarId_Monitor_AnalogUser_T)varId.NameBase)
            {
                case MOT_VAR_ANALOG_THROTTLE:       value = MotAnalogUser_GetThrottle(&p_mc->AnalogUser);               break;
                case MOT_VAR_ANALOG_BRAKE:          value = MotAnalogUser_GetBrake(&p_mc->AnalogUser);                  break;
                case MOT_VAR_ANALOG_THROTTLE_DIN:   value = p_mc->AnalogUser.ThrottleAIn.EdgePin.DebouncedState;        break;
                case MOT_VAR_ANALOG_BRAKE_DIN:      value = p_mc->AnalogUser.BrakeAIn.EdgePin.DebouncedState;           break;
                default: break;
            }
            break;

        case MOT_VAR_ID_TYPE_MONITOR_MOTOR:
            switch((MotVarId_Monitor_Motor_T)varId.NameBase)
            {
                case MOT_VAR_SPEED:                 value = Motor_User_GetSpeed_UFract16(p_motor);               break;
                case MOT_VAR_I_PHASE:               value = Motor_User_GetIPhase_UFract16(p_motor);              break;
                case MOT_VAR_V_PHASE:               value = Motor_User_GetVPhase_UFract16(p_motor);              break;
                case MOT_VAR_POWER:                 value = Motor_User_GetElectricalPower_UFract16(p_motor);     break;
                case MOT_VAR_MOTOR_STATE:           value = Motor_User_GetStateId(p_motor);                      break;
                case MOT_VAR_MOTOR_STATE_FLAGS:     value = Motor_User_GetStateFlags(p_motor).Word;              break;
                // case MOT_VAR_MOTOR_STATUS_FLAGS:    value = Motor_User_GetStateFlags(p_motor).Word;              break;
                case MOT_VAR_MOTOR_FAULT_FLAGS:     value = Motor_User_GetFaultFlags(p_motor).Value;             break;
                case MOT_VAR_MOTOR_HEAT:            value = Motor_User_GetHeat_Adcu(p_motor);                    break;
                    //Motor_User_GetHeat_DegC(p_motor, 1U);
                case MOT_VAR_MOTOR_EFFECTIVE_FEEDBACK_MODE:    value = Motor_User_GetFeedbackMode(p_motor).Word;  break;
                case MOT_VAR_MOTOR_EFFECTIVE_SPEED_LIMIT:      value = Motor_User_GetSpeedLimit(p_motor);         break;
                case MOT_VAR_MOTOR_EFFECTIVE_I_LIMIT:          value = Motor_User_GetILimit(p_motor);             break;
                case MOT_VAR_MOTOR_V_SPEED_DEBUG:               value = Motor_User_GetVSpeedDebug_UFract16(p_motor);             break;
                case MOT_VAR_MOTOR_V_SPEED_EFFECTIVE:           value = Motor_User_GetVSpeedEffective_UFract16(p_motor);    break;
                default: break;
            }
            break;

        case MOT_VAR_ID_TYPE_MONITOR_MOTOR_FOC:
            switch((MotVarId_Monitor_MotorFoc_T)varId.NameBase)
            {
                case MOT_VAR_FOC_IA:    value = p_motor->Foc.Ia;        break;
                case MOT_VAR_FOC_IB:    value = p_motor->Foc.Ib;        break;
                case MOT_VAR_FOC_IC:    value = p_motor->Foc.Ic;        break;
                case MOT_VAR_FOC_IQ:    value = p_motor->Foc.Iq;        break;
                case MOT_VAR_FOC_ID:    value = p_motor->Foc.Id;        break;
                case MOT_VAR_FOC_VQ:    value = p_motor->Foc.Vq;        break;
                case MOT_VAR_FOC_VD:    value = p_motor->Foc.Vd;        break;
                case MOT_VAR_FOC_REQ_Q: value = p_motor->Foc.ReqQ;      break;
                case MOT_VAR_FOC_REQ_D: value = p_motor->Foc.ReqD;      break;
                case MOT_VAR_FOC_VA:    value = p_motor->Foc.Va;        break;
                case MOT_VAR_FOC_VB:    value = p_motor->Foc.Vb;        break;
                case MOT_VAR_FOC_VC:    value = p_motor->Foc.Vc;        break;
                default: break;
            }
            break;

        case MOT_VAR_ID_TYPE_MONITOR_MOTOR_SENSOR:
            switch((MotVarId_Monitor_MotorSensor_T)varId.NameBase)
            {
                case MOT_VAR_ENCODER_FREQ:  value = p_motor->Encoder.FreqD;     break;
                default: break;
            }
            break;

        case MOT_VAR_ID_TYPE_CONTROL_GENERAL:
            switch((MotVarId_Control_General_T)varId.NameBase)
            {
                case MOT_VAR_DIRECTION:     value = MotorController_User_GetDirection(p_mc);    break;
            // case MOT_VAR_USER_SET_POINT: value = MotorController_User_GetCmdValue(p_mc);     break;
                default: break;
            }
            break;

        case MOT_VAR_ID_TYPE_CONTROL_MOTOR:
            switch((MotVarId_Control_Motor_T)varId.NameBase)
            {
                case MOT_VAR_MOTOR_DIRECTION:           value = Motor_User_GetDirection(p_motor);             break;
                case MOT_VAR_MOTOR_USER_SET_POINT:      value = Motor_User_GetSetPoint(p_motor);              break;
                case MOT_VAR_MOTOR_USER_FEEDBACK_MODE:  value = Motor_User_GetFeedbackMode(p_motor).Word;     break;
                case MOT_VAR_MOTOR_USER_SPEED_LIMIT:    value = Motor_User_GetSpeedLimit(p_motor);            break;
                case MOT_VAR_MOTOR_USER_I_LIMIT:        value = Motor_User_GetILimit(p_motor);                break;
                default: break;
            }
            break;

        case MOT_VAR_ID_TYPE_CMD_GENERAL:         value = 0; break;
        case MOT_VAR_ID_TYPE_CMD_MOTOR:   value = 0; break;
        default: value = 0; break;
    }

    return value;
}

/*!

*/
static inline MotVarId_Status_T SetRealTime(MotorController_T * p_mc, MotVarId_T varId, int32_t varValue)
{
   volatile MotVarId_Status_T status = MOT_VAR_STATUS_OK;
    bool isSuccess = true;
    Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, varId.Instance);

    switch((MotVarId_Type_RealTime_T)varId.NameType)
    {
        case MOT_VAR_ID_TYPE_CMD_MOTOR:
        // todo wrap motor cmd with MotorController StateMachine check
    //    if (MotorController_User_IsActiveState(p_mc))
    //    {

    //    }
            switch((MotVarId_Cmd_Motor_T)varId.NameBase)
            {
                // case MOT_VAR_MOTOR_USER_CMD:        Motor_User_SetActiveCmdValue(p_motor, varValue);    break;
                case MOT_VAR_MOTOR_CMD_SPEED:       Motor_User_SetSpeedCmd(p_motor, varValue);     break;
                case MOT_VAR_MOTOR_CMD_CURRENT:     Motor_User_SetICmd(p_motor, varValue);         break;
                case MOT_VAR_MOTOR_CMD_VOLTAGE:     Motor_User_SetVoltageCmd(p_motor, varValue);   break;
                case MOT_VAR_MOTOR_CMD_ANGLE:       Motor_User_SetPositionCmd(p_motor, varValue);  break;
                case MOT_VAR_MOTOR_CMD_OPEN_LOOP:   Motor_User_SetOpenLoopCmd(p_motor, varValue);  break;
                case MOT_VAR_MOTOR_FORCE_DISABLE_CONTROL:           break;
                case MOT_VAR_MOTOR_TRY_RELEASE:                     break;
                case MOT_VAR_MOTOR_TRY_HOLD:                        break;
                case MOT_VAR_MOTOR_CLEAR_FAULT:                     break;
                default: break;
            }
            break;

        case MOT_VAR_ID_TYPE_CONTROL_MOTOR:
            switch((MotVarId_Control_Motor_T)varId.NameBase)
            {
                case MOT_VAR_MOTOR_DIRECTION:               Motor_User_SetDirection(p_motor, (Motor_Direction_T)varValue);  break;
                case MOT_VAR_MOTOR_USER_SET_POINT:          Motor_User_SetActiveCmdValue(p_motor, varValue);                break;
                case MOT_VAR_MOTOR_USER_FEEDBACK_MODE:      Motor_User_SetFeedbackMode_Cast(p_motor, (uint8_t)varValue);    break;
                case MOT_VAR_MOTOR_USER_SPEED_LIMIT:        isSuccess = Motor_User_TrySpeedLimit(p_motor, varValue);        break;
                case MOT_VAR_MOTOR_USER_I_LIMIT:            isSuccess = Motor_User_TryILimit(p_motor, varValue);            break;
                default: break;
            }
            break;

        case MOT_VAR_ID_TYPE_CMD_GENERAL:
            status = MotorController_User_InputCmd(p_mc, (MotVarId_Cmd_General_T)varId.NameBase, varValue);
            break;

        case MOT_VAR_ID_TYPE_CONTROL_GENERAL:
            status = MotorController_User_InputControl(p_mc, (MotVarId_Control_General_T)varId.NameBase, varValue);
            break;

        case MOT_VAR_ID_TYPE_CMD_SYSTEM:
            status = MotorController_User_InputSystem(p_mc, (MotorController_User_System_T)varId.NameBase, varValue);
            break;

        case MOT_VAR_ID_TYPE_MONITOR_GENERAL:         status = MOT_VAR_STATUS_ERROR_READ_ONLY; break;
        case MOT_VAR_ID_TYPE_MONITOR_ANALOG_USER:     status = MOT_VAR_STATUS_ERROR_READ_ONLY; break;
        case MOT_VAR_ID_TYPE_MONITOR_MOTOR:           status = MOT_VAR_STATUS_ERROR_READ_ONLY; break;
        case MOT_VAR_ID_TYPE_MONITOR_MOTOR_FOC:       status = MOT_VAR_STATUS_ERROR_READ_ONLY; break;
        case MOT_VAR_ID_TYPE_MONITOR_MOTOR_SENSOR:    status = MOT_VAR_STATUS_ERROR_READ_ONLY; break;
        default: break;
    }

    if(isSuccess == false) { status = MOT_VAR_STATUS_ERROR; };

    return status;
}

/******************************************************************************/
/*!
    Parameter
*/
/******************************************************************************/
static int32_t GetConfig(const MotorController_T * p_mc, MotVarId_T varId)
{
    int32_t value = 0;
    const Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, varId.Instance);
    const Protocol_T * p_protocol = NULL;
    const Thermistor_T * p_thermistor = NULL;
    const VMonitor_T * p_vMonitor = NULL;

    // volatile int32_t id = varId.Word16;

    switch((MotVarId_Type_Config_T)varId.NameType)
    {
        case MOT_VAR_ID_TYPE_CONFIG_MOTOR_PRIMARY:
            switch((MotVarId_Config_MotorPrimary_T)varId.NameBase)
            {
                case MOT_VAR_COMMUTATION_MODE:              value = Motor_Config_GetCommutationMode(p_motor);           break;
                case MOT_VAR_SENSOR_MODE:                   value = Motor_Config_GetSensorMode(p_motor);                break;
                // case MOT_VAR_MOTOR_DEFAULT_FEEDBACK_MODE:   value = Motor_Config_GetDefaultFeedbackMode(p_motor).Word;  break;
                case MOT_VAR_DIRECTION_CALIBRATION:         value = Motor_Config_GetDirectionCalibration(p_motor);      break;
                case MOT_VAR_POLE_PAIRS:                    value = Motor_Config_GetPolePairs(p_motor);                 break;
                case MOT_VAR_KV:                            value = Motor_Config_GetKv(p_motor);                        break;
                case MOT_VAR_V_SPEED_SCALAR:                value = Motor_Config_GetVSpeedScalar_UFract16(p_motor);    break;
                case MOT_VAR_SPEED_V_REF_RPM:               value = Motor_Config_GetSpeedVRef_Rpm(p_motor);             break;
                case MOT_VAR_SPEED_V_MATCH_REF_RPM:         value = Motor_Config_GetSpeedVMatchRef_Rpm(p_motor);        break;
                case MOT_VAR_IA_ZERO_REF_ADCU:              value = Motor_Config_GetIaZero_Adcu(p_motor);               break;
                case MOT_VAR_IB_ZERO_REF_ADCU:              value = Motor_Config_GetIbZero_Adcu(p_motor);               break;
                case MOT_VAR_IC_ZERO_REF_ADCU:              value = Motor_Config_GetIcZero_Adcu(p_motor);               break;
                case MOT_VAR_I_PEAK_REF_ADCU:               value = Motor_Config_GetIPeakRef_Adcu(p_motor);             break;
                default: break;
            }
            break;

        case MOT_VAR_ID_TYPE_CONFIG_MOTOR_SECONDARY:
            switch((MotVarId_Config_MotorSecondary_T)varId.NameBase)
            {
                case MOT_VAR_BASE_SPEED_LIMIT_FORWARD:      value = Motor_Config_GetSpeedLimitForward_Fract16(p_motor);    break;
                case MOT_VAR_BASE_SPEED_LIMIT_REVERSE:      value = Motor_Config_GetSpeedLimitReverse_Fract16(p_motor);    break;
                case MOT_VAR_BASE_I_LIMIT_MOTORING:         value = Motor_Config_GetILimitMotoring_Fract16(p_motor);       break;
                case MOT_VAR_BASE_I_LIMIT_GENERATING:       value = Motor_Config_GetILimitGenerating_Fract16(p_motor);     break;
                case MOT_VAR_RAMP_ACCEL_TIME:               value = Motor_Config_GetRampAccel_Millis(p_motor);              break;
                // case MOT_VAR_ALIGN_MODE:                 value = Motor_Config_GetAlignMode(p_motor);                 break;
                case MOT_VAR_ALIGN_POWER:                   value = Motor_Config_GetAlignPower_Fract16(p_motor);           break;
                case MOT_VAR_ALIGN_TIME:                    value = Motor_Config_GetAlignTime_Millis(p_motor);              break;
                #if defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE) || defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)
                case MOT_VAR_OPEN_LOOP_POWER:               value = Motor_Config_GetOpenLoopPower_Fract16(p_motor);        break;
                case MOT_VAR_OPEN_LOOP_SPEED:               value = Motor_Config_GetOpenLoopSpeed_Fract16(p_motor);        break;
                case MOT_VAR_OPEN_LOOP_ACCEL_TIME:          value = Motor_Config_GetOpenLoopAccel_Millis(p_motor);          break;
                #endif
                // case MOT_VAR_PHASE_PWM_MODE:             value = Motor_Config_GetPhaseModeParam(p_motor);            break;
                default: break;
            }
            break;

        case MOT_VAR_ID_TYPE_CONFIG_MOTOR_HALL:
            switch((MotVarId_Config_MotorHall_T)varId.NameBase)
            {
                case MOT_VAR_HALL_SENSOR_TABLE_1: value = p_motor->Hall.Config.SensorsTable[1U]; break;
                case MOT_VAR_HALL_SENSOR_TABLE_2: value = p_motor->Hall.Config.SensorsTable[2U]; break;
                case MOT_VAR_HALL_SENSOR_TABLE_3: value = p_motor->Hall.Config.SensorsTable[3U]; break;
                case MOT_VAR_HALL_SENSOR_TABLE_4: value = p_motor->Hall.Config.SensorsTable[4U]; break;
                case MOT_VAR_HALL_SENSOR_TABLE_5: value = p_motor->Hall.Config.SensorsTable[5U]; break;
                case MOT_VAR_HALL_SENSOR_TABLE_6: value = p_motor->Hall.Config.SensorsTable[6U]; break;
                default: break;
            }
            break;

        case MOT_VAR_ID_TYPE_CONFIG_MOTOR_ENCODER:
            switch((MotVarId_Config_MotorEncoder_T)varId.NameBase)
            {
                case MOT_VAR_ENCODER_COUNTS_PER_REVOLUTION:             value = p_motor->Encoder.Config.CountsPerRevolution;            break;
                case MOT_VAR_ENCODER_IS_QUADRATURE_CAPTURE_ENABLED:     value = p_motor->Encoder.Config.IsQuadratureCaptureEnabled;     break;
                case MOT_VAR_ENCODER_IS_A_LEAD_B_POSITIVE:              value = p_motor->Encoder.Config.IsALeadBPositive;               break;
                // case MOT_VAR_ENCODER_EXTENDED_TIMER_DELTA_T_STOP:                            value = 0; break;
                // case MOT_VAR_ENCODER_INTERPOLATE_ANGLE_SCALAR:                               value = 0; break;
                default: break;
            }
            break;

        case MOT_VAR_ID_TYPE_CONFIG_MOTOR_THERMISTOR:
            if(p_motor != NULL) { value = GetParameterThermistor(&(p_motor->Thermistor), (MotVarId_Config_Thermistor_T)varId.NameBase); } break;
            break;

        case MOT_VAR_ID_TYPE_CONFIG_MOTOR_PID:
            switch((MotVarId_Config_MotorPid_T)varId.NameBase)
            {
                case MOT_VAR_PID_SPEED_KP_FIXED16:      value = PID_GetKp_Fixed16(&p_motor->PidSpeed);   break;
                case MOT_VAR_PID_SPEED_KI_FIXED16:      value = PID_GetKi_Fixed16(&p_motor->PidSpeed);   break;
                // case MOT_VAR_PID_SPEED_KD_FIXED16:      value = PID_GetKd_Fixed16(&p_motor->PidSpeed);   break;
                case MOT_VAR_PID_SPEED_SAMPLE_FREQ:     value = PID_GetSampleFreq(&p_motor->PidSpeed);  break;
                case MOT_VAR_PID_FOC_IQ_KP_FIXED16:     value = PID_GetKp_Fixed16(&p_motor->PidIq);     break;
                case MOT_VAR_PID_FOC_IQ_KI_FIXED16:     value = PID_GetKi_Fixed16(&p_motor->PidIq);     break;
                // case MOT_VAR_PID_FOC_IQ_KD_FIXED16:     value = PID_GetKd_Fixed16(&p_motor->PidIq);     break;
                case MOT_VAR_PID_FOC_IQ_SAMPLE_FREQ:    value = PID_GetSampleFreq(&p_motor->PidIq);     break;
                case MOT_VAR_PID_FOC_ID_KP_FIXED16:     value = PID_GetKp_Fixed16(&p_motor->PidId);     break;
                case MOT_VAR_PID_FOC_ID_KI_FIXED16:     value = PID_GetKi_Fixed16(&p_motor->PidId);     break;
                // case MOT_VAR_PID_FOC_ID_KD_FIXED16:     value = PID_GetKd_Fixed16(&p_motor->PidId);     break;
                case MOT_VAR_PID_FOC_ID_SAMPLE_FREQ:    value = PID_GetSampleFreq(&p_motor->PidId);     break;
                default: break;
            }
            break;


        case MOT_VAR_ID_TYPE_CONFIG_GENERAL:
            switch((MotVarId_Config_General_T)varId.NameBase)
            {
                case MOT_VAR_V_SOURCE_REF_VOLTS:        value = MotorController_User_GetVSourceRef(p_mc);               break;
                case MOT_VAR_USER_INPUT_MODE:           value = MotorController_User_GetInputMode(p_mc);                break;
                case MOT_VAR_USER_INIT_MODE:            value = MotorController_User_GetInitMode(p_mc);                 break;
                case MOT_VAR_THROTTLE_MODE:             value = MotorController_User_GetThrottleMode(p_mc);             break;
                case MOT_VAR_DEFAULT_FEEDBACK_MODE:     value = MotorController_User_GetDefaultFeedbackMode(p_mc).Word; break;
                case MOT_VAR_BRAKE_MODE:                value = MotorController_User_GetBrakeMode(p_mc);                break;
                case MOT_VAR_DRIVE_ZERO_MODE:           value = MotorController_User_GetDriveZeroMode(p_mc);            break;
                case MOT_VAR_I_LIMIT_LOW_V:             value = (p_mc->Config.VLowILimit_Fract16);                     break;
                case MOT_VAR_OPT_DIN_FUNCTION:          value = (p_mc->Config.OptDinMode);                              break;
                case MOT_VAR_OPT_SPEED_LIMIT:           value = (p_mc->Config.OptSpeedLimit_Fract16);                  break;
                // case MOT_VAR_BUZZER_FLAGS_ENABLE:    value = (p_mc->Config.Buzzer); break;
                // case MOT_VAR_CAN_SERVICES_ID:        value = (p_mc->Config.CanServicesId); break;
                // case MOT_VAR_CAN_IS_ENABLE:          value = (p_mc->Config.CanIsEnable); break;
                #ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
                case MOT_VAR_BATTERY_ZERO_ADCU:         value = (p_mc->Config.BatteryZero_Adcu);            break;
                case MOT_VAR_BATTERY_FULL_ADCU:         value = (p_mc->Config.BatteryFull_Adcu);            break;
                #endif
                default: break;
            }
            break;

        case MOT_VAR_ID_TYPE_CONFIG_ANALOG_USER:
            switch((MotVarId_Config_AnalogUser_T)varId.NameBase)
            {
                case MOT_VAR_ANALOG_THROTTLE_ZERO_ADCU:             value = p_mc->AnalogUser.Config.ThrottleZero_Adcu;          break;
                case MOT_VAR_ANALOG_THROTTLE_MAX_ADCU:              value = p_mc->AnalogUser.Config.ThrottleMax_Adcu;           break;
                case MOT_VAR_ANALOG_THROTTLE_EDGE_PIN_IS_ENABLE:    value = p_mc->AnalogUser.Config.UseThrottleEdgePin;         break;
                case MOT_VAR_ANALOG_BRAKE_ZERO_ADCU:                value = p_mc->AnalogUser.Config.BrakeZero_Adcu;             break;
                case MOT_VAR_ANALOG_BRAKE_MAX_ADCU:                 value = p_mc->AnalogUser.Config.BrakeMax_Adcu;              break;
                case MOT_VAR_ANALOG_BRAKE_EDGE_PIN_IS_ENABLE:       value = p_mc->AnalogUser.Config.UseBrakeEdgePin;            break;
                case MOT_VAR_ANALOG_DIN_BRAKE_VALUE:                value = p_mc->AnalogUser.Config.BistateBrakeValue_Percent16; break;
                case MOT_VAR_ANALOG_DIN_BRAKE_IS_ENABLE:            value = p_mc->AnalogUser.Config.UseBistateBrakePin;         break;
                case MOT_VAR_ANALOG_DIRECTION_PINS:                 value = p_mc->AnalogUser.Config.PinsSelect;                 break;
                default: break;
            }
            break;

        case MOT_VAR_ID_TYPE_CONFIG_BOARD_THERMISTOR:
            p_thermistor = MotorController_User_GetPtrThermistor(p_mc, varId.Instance);
            if(p_thermistor != NULL) { value = GetParameterThermistor(p_thermistor, (MotVarId_Config_Thermistor_T)varId.NameBase); }
            break;

        case MOT_VAR_ID_TYPE_CONFIG_VMONITOR:
            p_vMonitor = MotorController_User_GetPtrVMonitor(p_mc, varId.Instance);
            if(p_vMonitor != NULL)
            {
                switch((MotVarId_Config_VMonitor_T)varId.NameBase)
                {
                    case MOT_VAR_VMONITOR_R1:                   value = p_vMonitor->CONST.UNITS_R1 / 10U;       break;
                    case MOT_VAR_VMONITOR_R2:                   value = p_vMonitor->CONST.UNITS_R2 / 10U;       break;
                    case MOT_VAR_VMONITOR_FAULT_UPPER_ADCU:     value = VMonitor_GetFaultUpper(p_vMonitor);     break;
                    case MOT_VAR_VMONITOR_FAULT_LOWER_ADCU:     value = VMonitor_GetFaultLower(p_vMonitor);     break;
                    case MOT_VAR_VMONITOR_WARNING_UPPER_ADCU:   value = VMonitor_GetWarningUpper(p_vMonitor);   break;
                    case MOT_VAR_VMONITOR_WARNING_LOWER_ADCU:   value = VMonitor_GetWarningLower(p_vMonitor);   break;
                    case MOT_VAR_VMONITOR_IS_ENABLE:            value = VMonitor_IsEnable(p_vMonitor);          break;
                    default: break;
                }
            }
            break;

        case MOT_VAR_ID_TYPE_CONFIG_PROTOCOL:
            p_protocol = MotorController_User_GetPtrProtocol(p_mc, varId.Instance);
            switch((MotVarId_Config_Protocol_T)varId.NameBase)
            {
                case MOT_VAR_PROTOCOL_XCVR_ID:         value = p_protocol->Config.SpecsId; break;
                case MOT_VAR_PROTOCOL_SPECS_ID:        value = 0; break;
                case MOT_VAR_PROTOCOL_WATCHDOG_TIME:   value = 0; break;
                case MOT_VAR_PROTOCOL_BAUD_RATE:       value = 0; break;
                case MOT_VAR_PROTOCOL_IS_ENABLED:      value = 0; break;
                default: break;
            }
            break;

        case MOT_VAR_ID_TYPE_CONFIG_BOOT_REF:
            switch((MotVarId_Config_BootRef_T)varId.NameBase)
            {
                case MOT_VAR_BOOT_REF_FAST_BOOT:    MotorController_User_GetBootReg(p_mc).FastBoot;     break;
                case MOT_VAR_BOOT_REF_BEEP:         MotorController_User_GetBootReg(p_mc).Beep;         break;
                case MOT_VAR_BOOT_REF_BLINK:        MotorController_User_GetBootReg(p_mc).Blink;        break;
                default: break;
            }
            break;

        default: break;
    }

    return value;
}

static MotVarId_Status_T SetConfig(MotorController_T * p_mc, MotVarId_T varId, int32_t varValue)
{
    MotVarId_Status_T status = MOT_VAR_STATUS_OK;
    bool isSuccess = true;
    Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, varId.Instance); /* for convience, invalid if varId.NameBase is not Motor type */
    Protocol_T * p_protocol = NULL;
    Thermistor_T * p_thermistor = NULL;
    VMonitor_T * p_vMonitor = NULL;

    switch((MotVarId_Type_Config_T)varId.NameType)
    {
        case MOT_VAR_ID_TYPE_CONFIG_MOTOR_PRIMARY:
            if(p_motor != NULL)
            {
                switch((MotVarId_Config_MotorPrimary_T)varId.NameBase)
                {
                    case MOT_VAR_COMMUTATION_MODE:              Motor_Config_SetCommutationMode(p_motor, varValue);             break;
                    case MOT_VAR_SENSOR_MODE:                   Motor_Config_SetSensorMode(p_motor, varValue);                  break;
                    // case MOT_VAR_MOTOR_DEFAULT_FEEDBACK_MODE:   Motor_Config_SetDefaultFeedbackMode(p_motor, varValue);    break;
                    case MOT_VAR_DIRECTION_CALIBRATION:         Motor_Config_SetDirectionCalibration(p_motor, varValue);        break;
                    case MOT_VAR_POLE_PAIRS:                    Motor_Config_SetPolePairs(p_motor, varValue);                   break;
                    case MOT_VAR_KV:                            Motor_Config_SetKv(p_motor, varValue);                          break;
                    case MOT_VAR_V_SPEED_SCALAR:                Motor_Config_SetVSpeedScalar_UFract16(p_motor, varValue);           break;
                    case MOT_VAR_SPEED_V_REF_RPM:                   break;
                    case MOT_VAR_SPEED_V_MATCH_REF_RPM:             break;
                    case MOT_VAR_IA_ZERO_REF_ADCU:              Motor_Config_SetIaZero_Adcu(p_motor, varValue);                 break;
                    case MOT_VAR_IB_ZERO_REF_ADCU:              Motor_Config_SetIbZero_Adcu(p_motor, varValue);                 break;
                    case MOT_VAR_IC_ZERO_REF_ADCU:              Motor_Config_SetIcZero_Adcu(p_motor, varValue);                 break;
                    case MOT_VAR_I_PEAK_REF_ADCU:               Motor_Config_SetIPeakRef_Adcu(p_motor, varValue);               break;
                    default:  break;
                }
            }
            break;

        case MOT_VAR_ID_TYPE_CONFIG_MOTOR_SECONDARY:
            if(p_motor != NULL)
            {
                switch((MotVarId_Config_MotorSecondary_T)varId.NameBase)
                {
                    case MOT_VAR_BASE_SPEED_LIMIT_FORWARD:      Motor_Config_SetSpeedLimitForward_Fract16(p_motor, varValue);  break;
                    case MOT_VAR_BASE_SPEED_LIMIT_REVERSE:      Motor_Config_SetSpeedLimitReverse_Fract16(p_motor, varValue);  break;
                    case MOT_VAR_BASE_I_LIMIT_MOTORING:         Motor_Config_SetILimitMotoring_Fract16(p_motor, varValue);     break;
                    case MOT_VAR_BASE_I_LIMIT_GENERATING:       Motor_Config_SetILimitGenerating_Fract16(p_motor, varValue);   break;
                    case MOT_VAR_RAMP_ACCEL_TIME:               Motor_Config_SetRampAccel_Millis(p_motor, varValue);  break;
                    case MOT_VAR_ALIGN_POWER:                   Motor_Config_SetAlignPower_Fract16(p_motor, varValue);  break;
                    case MOT_VAR_ALIGN_TIME:                    Motor_Config_SetAlignTime_Millis(p_motor, varValue);  break;
                    case MOT_VAR_OPEN_LOOP_POWER:               Motor_Config_SetOpenLoopPower_Fract16(p_motor, varValue);  break;
                    case MOT_VAR_OPEN_LOOP_SPEED:               Motor_Config_SetOpenLoopSpeed_Fract16(p_motor, varValue);  break;
                    case MOT_VAR_OPEN_LOOP_ACCEL_TIME:          Motor_Config_SetOpenLoopAccel_Millis(p_motor, varValue);  break;
                    // case MOT_VAR_PHASE_PWM_MODE:             Motor_Config_SetPhaseModeParam(p_motor,  varValue);  break;
                    default: break;
                }
            }
            break;

        case MOT_VAR_ID_TYPE_CONFIG_MOTOR_HALL:
            if(p_motor != NULL)
            {
                switch((MotVarId_Config_MotorHall_T)varId.NameBase)
                {
                    // case MOT_VAR_HALL_SENSOR_TABLE_1: p_motor->Hall.Config.SensorsTable[1U];    break;
                    // case MOT_VAR_HALL_SENSOR_TABLE_2: p_motor->Hall.Config.SensorsTable[2U];    break;
                    // case MOT_VAR_HALL_SENSOR_TABLE_3: p_motor->Hall.Config.SensorsTable[3U];    break;
                    // case MOT_VAR_HALL_SENSOR_TABLE_4: p_motor->Hall.Config.SensorsTable[4U];    break;
                    // case MOT_VAR_HALL_SENSOR_TABLE_5: p_motor->Hall.Config.SensorsTable[5U];    break;
                    // case MOT_VAR_HALL_SENSOR_TABLE_6: p_motor->Hall.Config.SensorsTable[6U];    break;
                    default: break;
                }
            }
            break;

        case MOT_VAR_ID_TYPE_CONFIG_MOTOR_ENCODER:
            if(p_motor != NULL)
            {
                switch((MotVarId_Config_MotorEncoder_T)varId.NameBase)
                {
                    // case MOT_VAR_ENCODER_COUNTS_PER_REVOLUTION:              p_motor->Encoder.Config.CountsPerRevolution;            break;
                    // case MOT_VAR_ENCODER_IS_QUADRATURE_CAPTURE_ENABLED:      p_motor->Encoder.Config.IsQuadratureCaptureEnabled;     break;
                    // case MOT_VAR_ENCODER_IS_A_LEAD_B_POSITIVE:               p_motor->Encoder.Config.IsALeadBPositive;               break;
                    // case MOT_VAR_ENCODER_EXTENDED_TIMER_DELTA_T_STOP:                            0; break;
                    // case MOT_VAR_ENCODER_INTERPOLATE_ANGLE_SCALAR:                               0; break;
                    default: break;
                }
            }
            break;

        case MOT_VAR_ID_TYPE_CONFIG_MOTOR_THERMISTOR:
            if(p_motor != NULL) { SetParameterThermistor(&(p_motor->Thermistor), (MotVarId_Config_Thermistor_T)varId.NameBase, varValue); }
            break;

        case MOT_VAR_ID_TYPE_CONFIG_MOTOR_PID:
            if(p_motor != NULL)
            {
                switch((MotVarId_Config_MotorPid_T)varId.NameBase)
                {
                    case MOT_VAR_PID_SPEED_KP_FIXED16:  PID_SetKp_Fixed16(&p_motor->PidSpeed, varValue);    break;
                    case MOT_VAR_PID_SPEED_KI_FIXED16:  PID_SetKi_Fixed16(&p_motor->PidSpeed, varValue);    break;
                    // case MOT_VAR_PID_SPEED_KD_FIXED16:  PID_SetKd_Fixed16(&p_motor->PidSpeed, varValue);    break;
                    // case MOT_VAR_PID_SPEED_SAMPLE_FREQ:  PID_SetSampleFreq(&p_motor->PidSpeed, varValue);  break;
                    case MOT_VAR_PID_FOC_IQ_KP_FIXED16: PID_SetKp_Fixed16(&p_motor->PidIq, varValue);       break;
                    case MOT_VAR_PID_FOC_IQ_KI_FIXED16: PID_SetKi_Fixed16(&p_motor->PidIq, varValue);       break;
                    // case MOT_VAR_PID_FOC_IQ_KD_FIXED16: PID_SetKd_Fixed16(&p_motor->PidIq, varValue);       break;
                    // case MOT_VAR_PID_FOC_IQ_SAMPLE_FREQ: PID_SetSampleFreq(&p_motor->PidIq, varValue);     break;
                    case MOT_VAR_PID_FOC_ID_KP_FIXED16: PID_SetKp_Fixed16(&p_motor->PidId, varValue);       break;
                    case MOT_VAR_PID_FOC_ID_KI_FIXED16: PID_SetKi_Fixed16(&p_motor->PidId, varValue);       break;
                    // case MOT_VAR_PID_FOC_ID_KD_FIXED16: PID_SetKd_Fixed16(&p_motor->PidId, varValue);       break;
                    // case MOT_VAR_PID_FOC_ID_SAMPLE_FREQ: PID_SetSampleFreq(&p_motor->PidId, varValue);     break;
                    default: break;
                }
            }
            break;

        case MOT_VAR_ID_TYPE_CONFIG_GENERAL:
            switch((MotVarId_Config_General_T)varId.NameBase)
            {
                case MOT_VAR_V_SOURCE_REF_VOLTS:            MotorController_User_SetVSourceRef(p_mc, varValue);                                 break;
                case MOT_VAR_USER_INIT_MODE:                p_mc->Config.InitMode           = (MotorController_MainMode_T)varValue;             break;
                case MOT_VAR_USER_INPUT_MODE:               p_mc->Config.InputMode          = (MotorController_InputMode_T)varValue;            break;
                case MOT_VAR_DEFAULT_FEEDBACK_MODE:         MotorController_User_SetDefaultFeedbackMode_Cast(p_mc, varValue);                   break;
                case MOT_VAR_THROTTLE_MODE:                 p_mc->Config.ThrottleMode       = (MotorController_ThrottleMode_T)varValue;         break;
                case MOT_VAR_BRAKE_MODE:                    p_mc->Config.BrakeMode          = (MotorController_BrakeMode_T)varValue;            break;
                case MOT_VAR_DRIVE_ZERO_MODE:               p_mc->Config.DriveZeroMode      = (MotorController_DriveZeroMode_T)varValue;        break;
                case MOT_VAR_I_LIMIT_LOW_V:                 p_mc->Config.VLowILimit_Fract16 = varValue;    break;
                // case MOT_VAR_BUZZER_FLAGS_ENABLE:                        (p_mc->Config.Buzzer); break;
                // case MOT_VAR_OPT_DIN_FUNCTION:                           (p_mc->Config.OptDinMode);              break;
                // case MOT_VAR_OPT_SPEED_LIMIT:                        (p_mc->Config.OptSpeedLimit_Fract16);   break;
                // case MOT_VAR_CAN_SERVICES_ID:                            (p_mc->Config.CanServicesId); break;
                // case MOT_VAR_CAN_IS_ENABLE:                              (p_mc->Config.CanIsEnable); break;
                #ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
                // case MOT_VAR_BATTERY_ZERO_ADCU:                          (p_mc->Config.BatteryZero_Adcu);                break;
                // case MOT_VAR_BATTERY_FULL_ADCU:                          (p_mc->Config.BatteryFull_Adcu);                break;
                #endif
                default: break;
            }
            break;

        case MOT_VAR_ID_TYPE_CONFIG_ANALOG_USER:
            switch((MotVarId_Config_AnalogUser_T)varId.NameBase)
            {
                case MOT_VAR_ANALOG_THROTTLE_ZERO_ADCU:             p_mc->AnalogUser.Config.ThrottleZero_Adcu = varValue;           break;
                case MOT_VAR_ANALOG_THROTTLE_MAX_ADCU:              p_mc->AnalogUser.Config.ThrottleMax_Adcu = varValue;            break;
                case MOT_VAR_ANALOG_THROTTLE_EDGE_PIN_IS_ENABLE:    p_mc->AnalogUser.Config.UseThrottleEdgePin = varValue;          break;
                case MOT_VAR_ANALOG_BRAKE_ZERO_ADCU:                p_mc->AnalogUser.Config.BrakeZero_Adcu = varValue;              break;
                case MOT_VAR_ANALOG_BRAKE_MAX_ADCU:                 p_mc->AnalogUser.Config.BrakeMax_Adcu = varValue;               break;
                case MOT_VAR_ANALOG_BRAKE_EDGE_PIN_IS_ENABLE:       p_mc->AnalogUser.Config.UseBrakeEdgePin = varValue;             break;
                case MOT_VAR_ANALOG_DIN_BRAKE_VALUE:                p_mc->AnalogUser.Config.BistateBrakeValue_Percent16 = varValue;  break;
                case MOT_VAR_ANALOG_DIN_BRAKE_IS_ENABLE:            p_mc->AnalogUser.Config.UseBistateBrakePin = varValue;          break;
                // case MOT_VAR_ANALOG_DIRECTION_PINS:          p_mc->AnalogUser.Config.; break;
                default: break;
            }
            break;

        case MOT_VAR_ID_TYPE_CONFIG_BOARD_THERMISTOR:
            if((MotVarId_Config_Thermistor_T)varId.NameBase == MOT_VAR_THERMISTOR_R0) break;
            if((MotVarId_Config_Thermistor_T)varId.NameBase == MOT_VAR_THERMISTOR_T0) break;
            if((MotVarId_Config_Thermistor_T)varId.NameBase == MOT_VAR_THERMISTOR_B) break;
            p_thermistor = MotorController_User_GetPtrThermistor(p_mc, varId.Instance);
            if(p_thermistor != NULL) { SetParameterThermistor(p_thermistor, (MotVarId_Config_Thermistor_T)varId.NameBase, varValue); }
            break;

        case MOT_VAR_ID_TYPE_CONFIG_VMONITOR:
            p_vMonitor = MotorController_User_GetPtrVMonitor(p_mc, varId.Instance);
            if(p_vMonitor != NULL)
            {
                switch((MotVarId_Config_VMonitor_T)varId.NameBase)
                {
                    case MOT_VAR_VMONITOR_R1:                                                                       break;
                    case MOT_VAR_VMONITOR_R2:                                                                       break;
                    case MOT_VAR_VMONITOR_FAULT_UPPER_ADCU:     VMonitor_SetFaultUpper(p_vMonitor, varValue);       break;
                    case MOT_VAR_VMONITOR_FAULT_LOWER_ADCU:     VMonitor_SetFaultLower(p_vMonitor, varValue);       break;
                    case MOT_VAR_VMONITOR_WARNING_UPPER_ADCU:   VMonitor_SetWarningUpper(p_vMonitor, varValue);     break;
                    case MOT_VAR_VMONITOR_WARNING_LOWER_ADCU:   VMonitor_SetWarningLower(p_vMonitor, varValue);     break;
                    case MOT_VAR_VMONITOR_IS_ENABLE:            VMonitor_SetIsEnable(p_vMonitor, varValue);         break;
                    default: break;
                }
            }
            break;

        case MOT_VAR_ID_TYPE_CONFIG_PROTOCOL:
            p_protocol = MotorController_User_GetPtrProtocol(p_mc, varId.Instance);
            if(p_protocol != NULL)
            {
                switch((MotVarId_Config_Protocol_T)varId.NameBase)
                {
                    // case MOT_VAR_PROTOCOL0_XCVR_ID:                         MotorController_User_SetPtrProtocol(p_mc, 0)->Config.SpecsId; break;
                    // case MOT_VAR_PROTOCOL0_SPECS_ID:                        0; break;
                    // case MOT_VAR_PROTOCOL0_WATCHDOG_TIME:                   0; break;
                    // case MOT_VAR_PROTOCOL0_BAUD_RATE:                       0; break;
                    // case MOT_VAR_PROTOCOL0_IS_ENABLED:                      0; break;
                    default: break;
                }
            }
            break;

        case MOT_VAR_ID_TYPE_CONFIG_BOOT_REF:
            switch((MotVarId_Config_BootRef_T)varId.NameBase)
            {
                case MOT_VAR_BOOT_REF_FAST_BOOT:    MotorController_User_SetFastBoot(p_mc, varValue);   break;
                case MOT_VAR_BOOT_REF_BEEP:         MotorController_User_SetBeep(p_mc, varValue);       break;
                case MOT_VAR_BOOT_REF_BLINK:        MotorController_User_SetBlink(p_mc, varValue);      break;
                default: break;
            }
            break;

        default: break;
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
        case MOT_VAR_ID_TYPE_CONFIG:    value = GetConfig(p_mc, varId);  break;
        default: break;
    }
    return value;
}

MotVarId_Status_T MotorController_Var_Set(MotorController_T * p_mc, MotVarId_T varId, int32_t varValue)
{
    MotVarId_Status_T status = MOT_VAR_STATUS_OK;
    switch((MotVarId_TypeType_T)varId.NameTypeType)
    {
        case MOT_VAR_ID_TYPE_REAL_TIME:
            // status = (p_mc->Config.InputMode == MOTOR_CONTROLLER_INPUT_MODE_SERIAL) ?
            //     SetRealTime(p_mc, varId, varValue) : MOT_VAR_STATUS_ERROR_PROTOCOL_CONTROL_DISABLED;
            SetRealTime(p_mc, varId, varValue);
            break;
        case MOT_VAR_ID_TYPE_CONFIG:
            status = (MotorController_User_IsConfigState(p_mc) == true) ?
                SetConfig(p_mc, varId, varValue) : MOT_VAR_STATUS_ERROR_RUNNING;
            break;
        default: break;
    }
    return status;
}

// uint8_t MotorController_Var_GetSize(MotorController_T * p_mc, uint16_t varId)
// {

// }
