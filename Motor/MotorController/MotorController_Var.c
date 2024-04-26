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
    @brief  User Get Set Vars by Id
    @version V0
*/
/******************************************************************************/
#include "MotorController_Var.h"
#include "Motor/MotorController/MotorController_User.h"
#include "System/SysTime/SysTime.h"

/******************************************************************************/
/*!
    Helpers
*/
/******************************************************************************/
static int32_t GetParameterThermistor(const Thermistor_T * p_thermistor, MotVarId_Params_Thermistor_T nameId)
{
    int32_t value = 0;
    if(p_thermistor != NULL)
    {
        switch(nameId)
        {
            case MOT_VAR_THERMISTOR_R_SERIES:                   value = p_thermistor->CONFIG.R_SERIES / 10U;                break;
            case MOT_VAR_THERMISTOR_R_PARALLEL:                 value = p_thermistor->CONFIG.R_PARALLEL / 10U;              break;
            case MOT_VAR_THERMISTOR_R0:                         value = Thermistor_GetR0(p_thermistor) / 10U;               break;
            case MOT_VAR_THERMISTOR_T0:                         value = Thermistor_GetT0(p_thermistor);                     break;
            case MOT_VAR_THERMISTOR_B:                          value = Thermistor_GetB(p_thermistor);                      break;
            case MOT_VAR_THERMISTOR_FAULT_TRIGGER_ADCU:         value = Thermistor_GetFaultTrigger_Adcu(p_thermistor);      break;
            case MOT_VAR_THERMISTOR_FAULT_THRESHOLD_ADCU:       value = Thermistor_GetFaultThreshold_Adcu(p_thermistor);    break;
            case MOT_VAR_THERMISTOR_WARNING_TRIGGER_ADCU:       value = Thermistor_GetWarningTrigger_Adcu(p_thermistor);    break;
            case MOT_VAR_THERMISTOR_WARNING_THRESHOLD_ADCU:     value = Thermistor_GetWarningThreshold_Adcu(p_thermistor);  break;
            case MOT_VAR_THERMISTOR_IS_MONITOR_ENABLE:          value = Thermistor_IsMonitorEnable(p_thermistor);           break;
            // case MOT_VAR_THERMISTOR_TYPE:                       value = Thermistor_GetType(p_thermistor);                   break;
            case MOT_VAR_THERMISTOR_LINEAR_T0_ADCU:             value = Thermistor_GetLinearT0_Adcu(p_thermistor);          break;
            case MOT_VAR_THERMISTOR_LINEAR_T1_ADCU:             value = Thermistor_GetLinearT1_Adcu(p_thermistor);          break;
            case MOT_VAR_THERMISTOR_LINEAR_T0_DEG_C:            value = Thermistor_GetLinearT0_DegC(p_thermistor);          break;
            case MOT_VAR_THERMISTOR_LINEAR_T1_DEG_C:            value = Thermistor_GetLinearT1_DegC(p_thermistor);          break;
            default: break;
        }
    }
    return value;
}

/* Caller check instance */
static void SetParameterThermistor(Thermistor_T * p_thermistor, MotVarId_Params_Thermistor_T nameId, int32_t varValue)
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
            case MOT_VAR_THERMISTOR_LINEAR_T0_ADCU:             Thermistor_SetLinearT0_Adcu(p_thermistor, varValue);          break;
            case MOT_VAR_THERMISTOR_LINEAR_T0_DEG_C:            Thermistor_SetLinearT0_DegC(p_thermistor, varValue);          break;
            case MOT_VAR_THERMISTOR_LINEAR_T1_ADCU:             Thermistor_SetLinearT1_Adcu(p_thermistor, varValue);          break;
            case MOT_VAR_THERMISTOR_LINEAR_T1_DEG_C:            Thermistor_SetLinearT1_DegC(p_thermistor, varValue);          break;
            default: break;
        }
    }
}

/******************************************************************************/
/*!
    RealTime
*/
/******************************************************************************/
static inline int32_t GetRealTime(const MotorControllerPtr_T p_mc, MotVarId_T varId)
{
    int32_t value = 0;
    MotorPtr_T p_motor = MotorController_User_GetPtrMotor(p_mc, varId.Instance); /* for convience, invalid if varId.NameId is not Motor type */

    switch((MotVarId_Type_RealTime_T)varId.NameType)
    {
        case MOT_VAR_ID_TYPE_MONITOR_GENERAL:
            switch((MotVarId_Monitor_General_T)varId.NameId)
            {
                case MOT_VAR_ZERO:                  value = 0;                                                      break;
                case MOT_VAR_MILLIS:                value = Millis();                                               break;
                case MOT_VAR_DEBUG:                 value = Millis();                                               break;
                case MOT_VAR_MC_STATE:              value = MotorController_User_GetStateId(p_mc);                  break;
                case MOT_VAR_MC_STATUS_FLAGS:       value = MotorController_User_GetStatusFlags(p_mc).Word;         break;
                case MOT_VAR_MC_FAULT_FLAGS:        value = MotorController_User_GetFaultFlags(p_mc).Word;          break;
                case MOT_VAR_V_SOURCE:              value = p_mc->AnalogResults.VSource_Adcu;                       break;
                case MOT_VAR_V_SENSOR:              value = p_mc->AnalogResults.VSense_Adcu;                        break;
                case MOT_VAR_V_ACCS:                value = p_mc->AnalogResults.VAccs_Adcu;                         break;
                case MOT_VAR_HEAT_PCB:              value = MotorController_User_GetHeatPcb_Adcu(p_mc);             break;
                case MOT_VAR_HEAT_MOSFETS:          value = MotorController_User_GetHeatMosfets_Adcu(p_mc);         break;
                #ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
                // case MOT_VAR_HEAT_PCB_DEG_C:        value = MotorController_User_GetHeatPcb_DegC(p_mc, 1U);         break;
                // case MOT_VAR_HEAT_MOSFETS_DEG_C:    value = MotorController_User_GetHeatMosfets_DegC(p_mc, 1U);     break;
                case MOT_VAR_BATTERY_CHARGE:        value = MotorController_User_GetBatteryCharge_Scalar16(p_mc);   break;
                #endif
                default: break;
            }
            break;

        case MOT_VAR_ID_TYPE_MONITOR_ANALOG_USER:
            switch((MotVarId_Monitor_AnalogUser_T)varId.NameId)
            {
                case MOT_VAR_ANALOG_THROTTLE:       value = MotAnalogUser_GetThrottle(&p_mc->AnalogUser);               break;
                case MOT_VAR_ANALOG_BRAKE:          value = MotAnalogUser_GetBrake(&p_mc->AnalogUser);                  break;
                case MOT_VAR_ANALOG_THROTTLE_DIN:   value = p_mc->AnalogUser.ThrottleAIn.EdgePin.DebouncedState;        break;
                case MOT_VAR_ANALOG_BRAKE_DIN:      value = p_mc->AnalogUser.BrakeAIn.EdgePin.DebouncedState;           break;
                default: break;
            }
            break;

        case MOT_VAR_ID_TYPE_MONITOR_MOTOR:
            switch((MotVarId_Monitor_Motor_T)varId.NameId)
            {
                case MOT_VAR_SPEED:                 value = Motor_User_GetSpeed_UFrac16(p_motor);               break;
                case MOT_VAR_I_PHASE:               value = Motor_User_GetIPhase_UFrac16(p_motor);              break;
                case MOT_VAR_V_PHASE:               value = Motor_User_GetVPhase_UFrac16(p_motor);              break;
                case MOT_VAR_POWER:                 value = Motor_User_GetElectricalPower_UFrac16(p_motor);     break;
                case MOT_VAR_RAMP_SET_POINT:        value = Linear_Ramp_GetOutput(&p_motor->Ramp);              break;
                case MOT_VAR_MOTOR_STATE:           value = Motor_User_GetStateId(p_motor);                     break;
                case MOT_VAR_MOTOR_STATUS_FLAGS:    value = Motor_User_GetStatusFlags(p_motor).Word;            break;
                case MOT_VAR_MOTOR_FAULT_FLAGS:     value = Motor_User_GetFaultFlags(p_motor).Word;             break;
                case MOT_VAR_MOTOR_HEAT:            value = Motor_User_GetHeat_Adcu(p_motor);                   break;
                    //Motor_User_GetHeat_DegC(p_motor, 1U);
                case MOT_VAR_MOTOR_ACTIVE_SPEED_LIMIT:      value = Motor_User_GetActiveSpeedLimit(p_motor);    break;
                case MOT_VAR_MOTOR_ACTIVE_I_LIMIT:          value = Motor_User_GetActiveILimit(p_motor);        break;
                case MOT_VAR_MOTOR_V_SPEED:                 value = Motor_GetVSpeed_Frac16(p_motor);            break;

                default: break;
            }
            break;

        case MOT_VAR_ID_TYPE_MONITOR_MOTOR_FOC:
            switch((MotVarId_Monitor_MotorFoc_T)varId.NameId)
            {
                case MOT_VAR_FOC_IA:    value = p_motor->Foc.Ia;        break;
                case MOT_VAR_FOC_IB:    value = p_motor->Foc.Ib;        break;
                case MOT_VAR_FOC_IC:    value = p_motor->Foc.Ic;        break;
                case MOT_VAR_FOC_IQ:    value = p_motor->Foc.Iq;        break;
                case MOT_VAR_FOC_ID:    value = p_motor->Foc.Id;        break;
                case MOT_VAR_FOC_VQ:    value = p_motor->Foc.Vq;        break;
                case MOT_VAR_FOC_VD:    value = p_motor->Foc.Vd;        break;
                case MOT_VAR_FOC_Q_REQ: value = p_motor->Foc.QReq;      break;
                case MOT_VAR_FOC_D_REQ: value = p_motor->Foc.DReq;      break;
                case MOT_VAR_FOC_REQ_PRELIMIT:  value = p_motor->Foc.ReqMagnitude;  break;
                default: break;
            }
            break;

        case MOT_VAR_ID_TYPE_MONITOR_MOTOR_SENSOR:
            switch((MotVarId_Monitor_MotorSensor_T)varId.NameId)
            {
                case MOT_VAR_ENCODER_FREQ:    value = p_motor->Encoder.FreqD;   break;
                default: break;
            }
            break;

        case MOT_VAR_ID_TYPE_CONTROL:
            switch((MotVarId_Control_T)varId.NameId)
            {
                case MOT_VAR_DIRECTION:         value = MotorController_User_GetDirection(p_mc);                    break;
                default: break;
            }
            break;

        case MOT_VAR_ID_TYPE_CONTROL_MOTOR:
            switch((MotVarId_Control_Motor_T)varId.NameId)
            {
                case MOT_VAR_MOTOR_USER_SET_POINT:          value = Motor_User_GetSetPoint(p_motor);                            break;
                case MOT_VAR_MOTOR_DIRECTION:               value = Motor_User_GetDirection(p_motor);                           break;
                case MOT_VAR_MOTOR_ACTIVE_FEEDBACK_MODE:    value = Motor_User_GetActiveFeedbackMode(p_motor).Word;             break;
                case MOT_VAR_MOTOR_USER_SPEED_LIMIT:        value = Motor_User_GetActiveSpeedLimit(p_motor);                    break;
                case MOT_VAR_MOTOR_USER_I_LIMIT:            value = Motor_User_GetActiveILimit(p_motor);                        break;
                case MOT_VAR_MOTOR_USER_CMD:                value = Motor_User_GetCmd(p_motor);                                 break;
                default: break;
            }
            break;

        case MOT_VAR_ID_TYPE_CMD:         value = 0; break;
                // case MOT_VAR_USER_CMD:    value = MotorController_User_GetCmdValue(p_mc);                     break;
        case MOT_VAR_ID_TYPE_CMD_MOTOR:   value = 0; break;
        default: value = 0; break;
    }

    return value;
}

/*!

*/
static inline MotVarId_Status_T SetRealTime(MotorControllerPtr_T p_mc, MotVarId_T varId, int32_t varValue)
{
    MotVarId_Status_T status = MOT_VAR_STATUS_OK;
    bool boolStatus = true;
    MotorPtr_T p_motor = MotorController_User_GetPtrMotor(p_mc, varId.Instance);

    switch((MotVarId_Type_RealTime_T)varId.NameType)
    {
        case MOT_VAR_ID_TYPE_CONTROL:
            switch((MotVarId_Control_T)varId.NameId)
            {
                case MOT_VAR_DIRECTION:         boolStatus = MotorController_User_SetDirection(p_mc, (MotorController_Direction_T)varValue);    break;
                default: break;
            }
            break;

        case MOT_VAR_ID_TYPE_CONTROL_MOTOR:
            switch((MotVarId_Control_Motor_T)varId.NameId)
            {
                case MOT_VAR_MOTOR_USER_SET_POINT:          Motor_User_SetActiveCmdValue(p_motor, varValue);                                                break;
                case MOT_VAR_MOTOR_DIRECTION:               Motor_User_TryDirection(p_motor, (Motor_Direction_T)varValue);                                  break;
                case MOT_VAR_MOTOR_ACTIVE_FEEDBACK_MODE:    Motor_User_ActivateControl_Cast(p_motor, (uint8_t)varValue);                               break;
                case MOT_VAR_MOTOR_USER_SPEED_LIMIT:        Motor_User_SetSpeedLimitActive_Id(p_motor, varValue, MOTOR_SPEED_LIMIT_ACTIVE_USER);            break;
                case MOT_VAR_MOTOR_USER_I_LIMIT:            Motor_User_SetILimitActive_Id(p_motor, varValue, MOTOR_I_LIMIT_ACTIVE_USER);                    break;
                case MOT_VAR_MOTOR_USER_CMD:                Motor_User_SetActiveCmdValue(p_motor, varValue);                                                break;
                default: break;
            }
            break;

        case MOT_VAR_ID_TYPE_CMD:
            switch((MotVarId_Cmd_T)varId.NameId)
            {
                case MOT_VAR_BEEP:              MotorController_User_BeepN(p_mc, 500U, 500U, varValue); break;
                case MOT_VAR_USER_CMD:          MotorController_User_SetCmdValue(p_mc, varValue);       break;
                case MOT_VAR_THROTTLE:          MotorController_User_SetCmdThrottle(p_mc, varValue);    break;
                case MOT_VAR_BRAKE:             MotorController_User_SetCmdBrake(p_mc, varValue);       break;
                case MOT_VAR_RELEASE_CONTROL:         break;
                case MOT_VAR_DISABLE_CONTROL:         break;
                case MOT_VAR_CLEAR_FAULT:             break;
                default: break;
            }
            break;

        case MOT_VAR_ID_TYPE_CMD_MOTOR:
            switch((MotVarId_Cmd_Motor_T)varId.NameId)
            {
                case MOT_VAR_MOTOR_CMD_SPEED:       Motor_User_SetSpeedCmdValue(p_motor, varValue);     break;
                case MOT_VAR_MOTOR_CMD_CURRENT:     Motor_User_SetTorqueCmdValue(p_motor, varValue);    break;
                case MOT_VAR_MOTOR_CMD_VOLTAGE:     Motor_User_SetVoltageCmdValue(p_motor, varValue);   break;
                case MOT_VAR_MOTOR_CMD_ANGLE:       Motor_User_SetPositionCmdValue(p_motor, varValue);  break;
                case MOT_VAR_MOTOR_CMD_OPEN_LOOP:   Motor_User_SetOpenLoopCmdValue(p_motor, varValue);  break;
                // case MOT_VAR_MOTOR_RELEASE_CONTROL:          break;
                // case MOT_VAR_MOTOR_DISABLE_CONTROL:          break;
                // case MOT_VAR_MOTOR_CLEAR_FAULT:              break;
                default: break;
            }
            break;

        case MOT_VAR_ID_TYPE_MONITOR_GENERAL:         status = MOT_VAR_STATUS_ERROR_READ_ONLY; break;
        case MOT_VAR_ID_TYPE_MONITOR_ANALOG_USER:     status = MOT_VAR_STATUS_ERROR_READ_ONLY; break;
        case MOT_VAR_ID_TYPE_MONITOR_MOTOR:           status = MOT_VAR_STATUS_ERROR_READ_ONLY; break;
        case MOT_VAR_ID_TYPE_MONITOR_MOTOR_FOC:       status = MOT_VAR_STATUS_ERROR_READ_ONLY; break;
        case MOT_VAR_ID_TYPE_MONITOR_MOTOR_SENSOR:    status = MOT_VAR_STATUS_ERROR_READ_ONLY; break;
        default: break;
    }

    if(boolStatus == false) { status = MOT_VAR_STATUS_ERROR; };

    return status;
}

/******************************************************************************/
/*!
    Parameter
*/
/******************************************************************************/
static inline int32_t GetParameter(const MotorControllerPtr_T p_mc, MotVarId_T varId)
{
    int32_t value = 0;
    MotorPtr_T p_motor = MotorController_User_GetPtrMotor(p_mc, varId.Instance);
    Protocol_T * p_protocol;
    Thermistor_T * p_thermistor;
    VMonitor_T * p_vMonitor;

    switch((MotVarId_Type_Parameter_T)varId.NameType)
    {
        case MOT_VAR_ID_TYPE_PARAMS_MOTOR_PRIMARY:
            switch((MotVarId_Params_MotorPrimary_T)varId.NameId)
            {
                case MOT_VAR_COMMUTATION_MODE:              value = Motor_Params_GetCommutationMode(p_motor);           break;
                case MOT_VAR_SENSOR_MODE:                   value = Motor_Params_GetSensorMode(p_motor);                break;
                case MOT_VAR_MOTOR_DEFAULT_FEEDBACK_MODE:   value = Motor_Params_GetDefaultFeedbackMode(p_motor).Word;  break;
                case MOT_VAR_DIRECTION_CALIBRATION:         value = Motor_Params_GetDirectionCalibration(p_motor);      break;
                case MOT_VAR_POLE_PAIRS:                    value = Motor_Params_GetPolePairs(p_motor);                 break;
                case MOT_VAR_KV:                            value = Motor_Params_GetKv(p_motor);                        break;
                case MOT_VAR_SPEED_FEEDBACK_REF_RPM:        value = Motor_Params_GetSpeedFeedbackRef_Rpm(p_motor);      break;
                case MOT_VAR_V_SPEED_REF_RPM:               value = Motor_Params_GetVSpeedRef_Rpm(p_motor);             break;
                case MOT_VAR_IA_ZERO_REF_ADCU:              value = Motor_Params_GetIaZero_Adcu(p_motor);               break;
                case MOT_VAR_IB_ZERO_REF_ADCU:              value = Motor_Params_GetIbZero_Adcu(p_motor);               break;
                case MOT_VAR_IC_ZERO_REF_ADCU:              value = Motor_Params_GetIcZero_Adcu(p_motor);               break;
                case MOT_VAR_I_PEAK_REF_ADCU:               value = Motor_Params_GetIPeakRef_Adcu(p_motor);                 break;
                default: break;
            }
            break;

        case MOT_VAR_ID_TYPE_PARAMS_MOTOR_SECONDARY:
            switch((MotVarId_Params_MotorSecondary_T)varId.NameId)
            {
                case MOT_VAR_BASE_SPEED_LIMIT_FORWARD:      value = Motor_Params_GetSpeedLimitForward_Scalar16(p_motor);    break;
                case MOT_VAR_BASE_SPEED_LIMIT_REVERSE:      value = Motor_Params_GetSpeedLimitReverse_Scalar16(p_motor);    break;
                case MOT_VAR_BASE_I_LIMIT_MOTORING:         value = Motor_Params_GetILimitMotoring_Scalar16(p_motor);       break;
                case MOT_VAR_BASE_I_LIMIT_GENERATING:       value = Motor_Params_GetILimitGenerating_Scalar16(p_motor);     break;
                case MOT_VAR_RAMP_ACCEL_TIME:               value = Motor_Params_GetRampAccel_Millis(p_motor);              break;
                // case MOT_VAR_ALIGN_MODE:                 value = Motor_Params_GetAlignMode(p_motor);                 break;
                case MOT_VAR_ALIGN_POWER:                   value = Motor_Params_GetAlignPower_Scalar16(p_motor);           break;
                case MOT_VAR_ALIGN_TIME:                    value = Motor_Params_GetAlignTime_Millis(p_motor);              break;
                #if defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE) || defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)
                case MOT_VAR_OPEN_LOOP_POWER:               value = Motor_Params_GetOpenLoopPower_Scalar16(p_motor);        break;
                case MOT_VAR_OPEN_LOOP_SPEED:               value = Motor_Params_GetOpenLoopSpeed_Scalar16(p_motor);        break;
                case MOT_VAR_OPEN_LOOP_ACCEL_TIME:          value = Motor_Params_GetOpenLoopAccel_Millis(p_motor);          break;
                #endif
                // case MOT_VAR_PHASE_PWM_MODE:             value = Motor_Params_GetPhaseModeParam(p_motor);            break;
                default: break;
            }
            break;

        case MOT_VAR_ID_TYPE_PARAMS_MOTOR_HALL:
            switch((MotVarId_Params_MotorHall_T)varId.NameId)
            {
                case MOT_VAR_HALL_SENSOR_TABLE_1: value = p_motor->Hall.Params.SensorsTable[1U]; break;
                case MOT_VAR_HALL_SENSOR_TABLE_2: value = p_motor->Hall.Params.SensorsTable[2U]; break;
                case MOT_VAR_HALL_SENSOR_TABLE_3: value = p_motor->Hall.Params.SensorsTable[3U]; break;
                case MOT_VAR_HALL_SENSOR_TABLE_4: value = p_motor->Hall.Params.SensorsTable[4U]; break;
                case MOT_VAR_HALL_SENSOR_TABLE_5: value = p_motor->Hall.Params.SensorsTable[5U]; break;
                case MOT_VAR_HALL_SENSOR_TABLE_6: value = p_motor->Hall.Params.SensorsTable[6U]; break;
                default: break;
            }
            break;

        case MOT_VAR_ID_TYPE_PARAMS_MOTOR_ENCODER:
            switch((MotVarId_Params_MotorEncoder_T)varId.NameId)
            {
                case MOT_VAR_ENCODER_COUNTS_PER_REVOLUTION:             value = p_motor->Encoder.Params.CountsPerRevolution;            break;
                case MOT_VAR_ENCODER_IS_QUADRATURE_CAPTURE_ENABLED:     value = p_motor->Encoder.Params.IsQuadratureCaptureEnabled;     break;
                case MOT_VAR_ENCODER_IS_A_LEAD_B_POSITIVE:              value = p_motor->Encoder.Params.IsALeadBPositive;               break;
                // case MOT_VAR_ENCODER_EXTENDED_TIMER_DELTA_T_STOP:                            value = 0; break;
                // case MOT_VAR_ENCODER_INTERPOLATE_ANGLE_SCALAR:                               value = 0; break;
                default: break;
            }
            break;

        case MOT_VAR_ID_TYPE_PARAMS_MOTOR_PID:
            switch((MotVarId_Params_MotorPid_T)varId.NameId)
            {
                case MOT_VAR_PID_SPEED_KP_FIXED16:      value = PID_GetKp_Fixed16(&p_motor->PidSpeed);   break;
                case MOT_VAR_PID_SPEED_KI_FIXED16:      value = PID_GetKi_Fixed16(&p_motor->PidSpeed);   break;
                case MOT_VAR_PID_SPEED_KD_FIXED16:      value = PID_GetKd_Fixed16(&p_motor->PidSpeed);   break;
                case MOT_VAR_PID_SPEED_SAMPLE_FREQ:     value = PID_GetSampleFreq(&p_motor->PidSpeed);  break;
                case MOT_VAR_PID_FOC_IQ_KP_FIXED16:     value = PID_GetKp_Fixed16(&p_motor->PidIq);     break;
                case MOT_VAR_PID_FOC_IQ_KI_FIXED16:     value = PID_GetKi_Fixed16(&p_motor->PidIq);     break;
                case MOT_VAR_PID_FOC_IQ_KD_FIXED16:     value = PID_GetKd_Fixed16(&p_motor->PidIq);     break;
                case MOT_VAR_PID_FOC_IQ_SAMPLE_FREQ:    value = PID_GetSampleFreq(&p_motor->PidIq);     break;
                case MOT_VAR_PID_FOC_ID_KP_FIXED16:     value = PID_GetKp_Fixed16(&p_motor->PidId);     break;
                case MOT_VAR_PID_FOC_ID_KI_FIXED16:     value = PID_GetKi_Fixed16(&p_motor->PidId);     break;
                case MOT_VAR_PID_FOC_ID_KD_FIXED16:     value = PID_GetKd_Fixed16(&p_motor->PidId);     break;
                case MOT_VAR_PID_FOC_ID_SAMPLE_FREQ:    value = PID_GetSampleFreq(&p_motor->PidId);     break;
                default: break;
            }
            break;


        case MOT_VAR_ID_TYPE_PARAMS_GENERAL:
            switch((MotVarId_Params_General_T)varId.NameId)
            {
                case MOT_VAR_V_SOURCE_REF_VOLTS:        value = MotorController_User_GetVSourceRef(p_mc);               break;
                case MOT_VAR_USER_INPUT_MODE:           value = MotorController_User_GetInputMode(p_mc);                break;
                case MOT_VAR_USER_INIT_MODE:            value = MotorController_User_GetInitMode(p_mc);                 break;
                case MOT_VAR_THROTTLE_MODE:             value = MotorController_User_GetThrottleMode(p_mc);             break;
                case MOT_VAR_DEFAULT_FEEDBACK_MODE:     value = MotorController_User_GetDefaultFeedbackMode(p_mc).Word; break;
                case MOT_VAR_BRAKE_MODE:                value = MotorController_User_GetBrakeMode(p_mc);                break;
                case MOT_VAR_DRIVE_ZERO_MODE:           value = MotorController_User_GetDriveZeroMode(p_mc);            break;
                case MOT_VAR_I_LIMIT_LOW_V:             value = (p_mc->Parameters.ILimitLowV_Scalar16);                 break;
                case MOT_VAR_OPT_DIN_FUNCTION:          value = (p_mc->Parameters.OptDinMode);                          break;
                case MOT_VAR_OPT_DIN_SPEED_LIMIT:       value = (p_mc->Parameters.OptDinSpeedLimit_Scalar16);           break;
                // case MOT_VAR_BUZZER_FLAGS_ENABLE:    value = (p_mc->Parameters.Buzzer); break;
                // case MOT_VAR_CAN_SERVICES_ID:        value = (p_mc->Parameters.CanServicesId); break;
                // case MOT_VAR_CAN_IS_ENABLE:          value = (p_mc->Parameters.CanIsEnable); break;
                #ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
                case MOT_VAR_BATTERY_ZERO_ADCU:         value = (p_mc->Parameters.BatteryZero_Adcu);            break;
                case MOT_VAR_BATTERY_FULL_ADCU:         value = (p_mc->Parameters.BatteryFull_Adcu);            break;
                #endif
                default: break;
            }
            break;

        case MOT_VAR_ID_TYPE_PARAMS_ANALOG_USER:
            switch((MotVarId_Params_AnalogUser_T)varId.NameId)
            {
                case MOT_VAR_ANALOG_THROTTLE_ZERO_ADCU:             value = p_mc->AnalogUser.Params.ThrottleZero_Adcu;          break;
                case MOT_VAR_ANALOG_THROTTLE_MAX_ADCU:              value = p_mc->AnalogUser.Params.ThrottleMax_Adcu;           break;
                case MOT_VAR_ANALOG_THROTTLE_EDGE_PIN_IS_ENABLE:    value = p_mc->AnalogUser.Params.UseThrottleEdgePin;         break;
                case MOT_VAR_ANALOG_BRAKE_ZERO_ADCU:                value = p_mc->AnalogUser.Params.BrakeZero_Adcu;             break;
                case MOT_VAR_ANALOG_BRAKE_MAX_ADCU:                 value = p_mc->AnalogUser.Params.BrakeMax_Adcu;              break;
                case MOT_VAR_ANALOG_BRAKE_EDGE_PIN_IS_ENABLE:       value = p_mc->AnalogUser.Params.UseBrakeEdgePin;            break;
                case MOT_VAR_ANALOG_DIN_BRAKE_VALUE:                value = p_mc->AnalogUser.Params.BistateBrakeValue_Scalar16; break;
                case MOT_VAR_ANALOG_DIN_BRAKE_IS_ENABLE:            value = p_mc->AnalogUser.Params.UseBistateBrakePin;         break;
                case MOT_VAR_ANALOG_DIRECTION_PINS:                 value = p_mc->AnalogUser.Params.PinsSelect;                 break;
                default: break;
            }
            break;

        case MOT_VAR_ID_TYPE_PARAMS_THERMISTOR:
            switch((MotVarId_Instance_Prefix_T)varId.InstancePrefix)
            {
                case MOT_VAR_ID_INSTANCE_PREFIX_BOARD:  p_thermistor = MotorController_User_GetPtrThermistor(p_mc, varId.Instance);     break;
                case MOT_VAR_ID_INSTANCE_PREFIX_MOTOR:  if(p_motor != NULL) { p_thermistor = &(p_motor->Thermistor); }                  break;
                default: break;
            }
            if(p_thermistor != NULL) { value = GetParameterThermistor(p_thermistor, (MotVarId_Params_Thermistor_T)varId.NameId); }
            break;

        case MOT_VAR_ID_TYPE_PARAMS_VMONITOR:
            p_vMonitor = MotorController_User_GetPtrVMonitor(p_mc, varId.Instance);
            if(p_vMonitor != NULL)
            {
                switch((MotVarId_Params_VMonitor_T)varId.NameId)
                {
                    case MOT_VAR_VMONITOR_R1:                   value = p_vMonitor->CONFIG.UNITS_R1 / 10U;      break;
                    case MOT_VAR_VMONITOR_R2:                   value = p_vMonitor->CONFIG.UNITS_R2 / 10U;      break;
                    case MOT_VAR_VMONITOR_FAULT_UPPER_ADCU:     value = VMonitor_GetFaultUpper(p_vMonitor);     break;
                    case MOT_VAR_VMONITOR_FAULT_LOWER_ADCU:     value = VMonitor_GetFaultLower(p_vMonitor);     break;
                    case MOT_VAR_VMONITOR_WARNING_UPPER_ADCU:   value = VMonitor_GetWarningUpper(p_vMonitor);   break;
                    case MOT_VAR_VMONITOR_WARNING_LOWER_ADCU:   value = VMonitor_GetWarningLower(p_vMonitor);   break;
                    case MOT_VAR_VMONITOR_IS_ENABLE:            value = VMonitor_IsEnable(p_vMonitor);          break;
                    default: break;
                }
            }
            break;

        case MOT_VAR_ID_TYPE_PARAMS_PROTOCOL:
            p_protocol = MotorController_User_GetPtrProtocol(p_mc, varId.Instance);
            switch((MotVarId_Params_Protocol_T)varId.NameId)
            {
                case MOT_VAR_PROTOCOL_XCVR_ID:         value = p_protocol->Params.SpecsId; break;
                case MOT_VAR_PROTOCOL_SPECS_ID:        value = 0; break;
                case MOT_VAR_PROTOCOL_WATCHDOG_TIME:   value = 0; break;
                case MOT_VAR_PROTOCOL_BAUD_RATE:       value = 0; break;
                case MOT_VAR_PROTOCOL_IS_ENABLED:      value = 0; break;
                default: break;
            }
            break;

        default: break;
    }

    return value;
}

static inline MotVarId_Status_T SetParameter(MotorControllerPtr_T p_mc, MotVarId_T varId, int32_t varValue)
{
    MotVarId_Status_T status = MOT_VAR_STATUS_OK;
    bool boolStatus = true;
    MotorPtr_T p_motor = MotorController_User_GetPtrMotor(p_mc, varId.Instance); //todo as nullable to indicate invalid motor
    Protocol_T * p_protocol;
    Thermistor_T * p_thermistor;
    VMonitor_T * p_vMonitor;

    switch((MotVarId_Type_Parameter_T)varId.NameType)
    {
        case MOT_VAR_ID_TYPE_PARAMS_MOTOR_PRIMARY:
            if(p_motor != NULL)
            {
                switch((MotVarId_Params_MotorPrimary_T)varId.NameId)
                {
                    case MOT_VAR_COMMUTATION_MODE:              Motor_Params_SetCommutationMode(p_motor, varValue);             break;
                    case MOT_VAR_SENSOR_MODE:                   Motor_Params_SetSensorMode(p_motor, varValue);                  break;
                    case MOT_VAR_MOTOR_DEFAULT_FEEDBACK_MODE:   Motor_Params_SetDefaultFeedbackMode(p_motor, varValue);    break;
                    case MOT_VAR_DIRECTION_CALIBRATION:         Motor_Params_SetDirectionCalibration(p_motor, varValue);        break;
                    case MOT_VAR_SPEED_FEEDBACK_REF_RPM:        Motor_Params_SetSpeedFeedbackRef_Rpm(p_motor, varValue);        break;
                    case MOT_VAR_POLE_PAIRS:                    Motor_Params_SetPolePairs(p_motor, varValue);                   break;
                    case MOT_VAR_KV:                            Motor_Params_SetKv(p_motor, varValue);                          break;
                    case MOT_VAR_V_SPEED_REF_RPM:               Motor_Params_SetVSpeedRef_Rpm(p_motor, varValue);               break;
                    case MOT_VAR_IA_ZERO_REF_ADCU:              Motor_Params_SetIaZero_Adcu(p_motor, varValue);                 break;
                    case MOT_VAR_IB_ZERO_REF_ADCU:              Motor_Params_SetIbZero_Adcu(p_motor, varValue);                 break;
                    case MOT_VAR_IC_ZERO_REF_ADCU:              Motor_Params_SetIcZero_Adcu(p_motor, varValue);                 break;
                    case MOT_VAR_I_PEAK_REF_ADCU:               Motor_Params_SetIPeakRef_Adcu(p_motor, varValue);               break;
                    default:  break;
                }
            }
            break;

        case MOT_VAR_ID_TYPE_PARAMS_MOTOR_SECONDARY:
            if(p_motor != NULL)
            {
                switch((MotVarId_Params_MotorSecondary_T)varId.NameId)
                {
                    case MOT_VAR_BASE_SPEED_LIMIT_FORWARD:      Motor_Params_SetSpeedLimitForward_Scalar16(p_motor, varValue);  break;
                    case MOT_VAR_BASE_SPEED_LIMIT_REVERSE:      Motor_Params_SetSpeedLimitReverse_Scalar16(p_motor, varValue);  break;
                    case MOT_VAR_BASE_I_LIMIT_MOTORING:         Motor_Params_SetILimitMotoring_Scalar16(p_motor, varValue);     break;
                    case MOT_VAR_BASE_I_LIMIT_GENERATING:       Motor_Params_SetILimitGenerating_Scalar16(p_motor, varValue);   break;
                    case MOT_VAR_RAMP_ACCEL_TIME:               Motor_Params_SetRampAccel_Millis(p_motor, varValue);  break;
                    case MOT_VAR_ALIGN_POWER:                   Motor_Params_SetAlignPower_Scalar16(p_motor, varValue);  break;
                    case MOT_VAR_ALIGN_TIME:                    Motor_Params_SetAlignTime_Millis(p_motor, varValue);  break;
                    case MOT_VAR_OPEN_LOOP_POWER:               Motor_Params_SetOpenLoopPower_Scalar16(p_motor, varValue);  break;
                    case MOT_VAR_OPEN_LOOP_SPEED:               Motor_Params_SetOpenLoopSpeed_Scalar16(p_motor, varValue);  break;
                    case MOT_VAR_OPEN_LOOP_ACCEL_TIME:          Motor_Params_SetOpenLoopAccel_Millis(p_motor, varValue);  break;
                        // case MOT_VAR_PHASE_PWM_MODE:             Motor_Params_SetPhaseModeParam(p_motor,  varValue);  break;
                    default: break;
                }
            }
            break;

        case MOT_VAR_ID_TYPE_PARAMS_MOTOR_HALL:
            if(p_motor != NULL)
            {
                switch((MotVarId_Params_MotorHall_T)varId.NameId)
                {
                    // case MOT_VAR_HALL_SENSOR_TABLE_1: p_motor->Hall.Params.SensorsTable[1U];    break;
                    // case MOT_VAR_HALL_SENSOR_TABLE_2: p_motor->Hall.Params.SensorsTable[2U];    break;
                    // case MOT_VAR_HALL_SENSOR_TABLE_3: p_motor->Hall.Params.SensorsTable[3U];    break;
                    // case MOT_VAR_HALL_SENSOR_TABLE_4: p_motor->Hall.Params.SensorsTable[4U];    break;
                    // case MOT_VAR_HALL_SENSOR_TABLE_5: p_motor->Hall.Params.SensorsTable[5U];    break;
                    // case MOT_VAR_HALL_SENSOR_TABLE_6: p_motor->Hall.Params.SensorsTable[6U];    break;
                    default: break;
                }
            }
            break;

        case MOT_VAR_ID_TYPE_PARAMS_MOTOR_ENCODER:
            if(p_motor != NULL)
            {
                switch((MotVarId_Params_MotorEncoder_T)varId.NameId)
                {
                    // case MOT_VAR_ENCODER_COUNTS_PER_REVOLUTION:              p_motor->Encoder.Params.CountsPerRevolution;            break;
                    // case MOT_VAR_ENCODER_IS_QUADRATURE_CAPTURE_ENABLED:      p_motor->Encoder.Params.IsQuadratureCaptureEnabled;     break;
                    // case MOT_VAR_ENCODER_IS_A_LEAD_B_POSITIVE:               p_motor->Encoder.Params.IsALeadBPositive;               break;
                    // case MOT_VAR_ENCODER_EXTENDED_TIMER_DELTA_T_STOP:                            0; break;
                    // case MOT_VAR_ENCODER_INTERPOLATE_ANGLE_SCALAR:                               0; break;
                    default: break;
                }
            }
            break;

        case MOT_VAR_ID_TYPE_PARAMS_MOTOR_PID:
            if(p_motor != NULL)
            {
                switch((MotVarId_Params_MotorPid_T)varId.NameId)
                {
                    case MOT_VAR_PID_SPEED_KP_FIXED16:  PID_SetKp_Fixed16(&p_motor->PidSpeed, varValue);    break;
                    case MOT_VAR_PID_SPEED_KI_FIXED16:  PID_SetKi_Fixed16(&p_motor->PidSpeed, varValue);    break;
                    case MOT_VAR_PID_SPEED_KD_FIXED16:  PID_SetKd_Fixed16(&p_motor->PidSpeed, varValue);    break;
                    // case MOT_VAR_PID_SPEED_SAMPLE_FREQ:  PID_SetSampleFreq(&p_motor->PidSpeed, varValue);  break;
                    case MOT_VAR_PID_FOC_IQ_KP_FIXED16: PID_SetKp_Fixed16(&p_motor->PidIq, varValue);       break;
                    case MOT_VAR_PID_FOC_IQ_KI_FIXED16: PID_SetKi_Fixed16(&p_motor->PidIq, varValue);       break;
                    case MOT_VAR_PID_FOC_IQ_KD_FIXED16: PID_SetKd_Fixed16(&p_motor->PidIq, varValue);       break;
                    // case MOT_VAR_PID_FOC_IQ_SAMPLE_FREQ: PID_SetSampleFreq(&p_motor->PidIq, varValue);     break;
                    case MOT_VAR_PID_FOC_ID_KP_FIXED16: PID_SetKp_Fixed16(&p_motor->PidId, varValue);       break;
                    case MOT_VAR_PID_FOC_ID_KI_FIXED16: PID_SetKi_Fixed16(&p_motor->PidId, varValue);       break;
                    case MOT_VAR_PID_FOC_ID_KD_FIXED16: PID_SetKd_Fixed16(&p_motor->PidId, varValue);       break;
                    // case MOT_VAR_PID_FOC_ID_SAMPLE_FREQ: PID_SetSampleFreq(&p_motor->PidId, varValue);     break;
                    default: break;
                }
            }
            break;

        case MOT_VAR_ID_TYPE_PARAMS_GENERAL:
            switch((MotVarId_Params_General_T)varId.NameId)
            {
                case MOT_VAR_V_SOURCE_REF_VOLTS:            MotorController_User_SetVSourceRef(p_mc, varValue);                     break;
                case MOT_VAR_USER_INIT_MODE:                p_mc->Parameters.InitMode = (MotorController_InitMode_T)varValue;       break;
                case MOT_VAR_USER_INPUT_MODE:               p_mc->Parameters.InputMode = (MotorController_InputMode_T)varValue;     break;
                case MOT_VAR_DEFAULT_FEEDBACK_MODE:         MotorController_User_SetDefaultFeedbackMode(p_mc, varValue);            break;
                case MOT_VAR_THROTTLE_MODE:                 p_mc->Parameters.ThrottleMode = varValue;           break;
                case MOT_VAR_BRAKE_MODE:                    p_mc->Parameters.BrakeMode = varValue;              break;
                case MOT_VAR_DRIVE_ZERO_MODE:               p_mc->Parameters.DriveZeroMode = varValue;          break;
                case MOT_VAR_I_LIMIT_LOW_V:                 p_mc->Parameters.ILimitLowV_Scalar16 = varValue;    break;
                // case MOT_VAR_BUZZER_FLAGS_ENABLE:                        (p_mc->Parameters.Buzzer); break;
                // case MOT_VAR_OPT_DIN_FUNCTION:                           (p_mc->Parameters.OptDinMode);              break;
                // case MOT_VAR_OPT_DIN_SPEED_LIMIT:                        (p_mc->Parameters.OptDinSpeedLimit_Scalar16);   break;
                // case MOT_VAR_CAN_SERVICES_ID:                            (p_mc->Parameters.CanServicesId); break;
                // case MOT_VAR_CAN_IS_ENABLE:                              (p_mc->Parameters.CanIsEnable); break;
                #ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
                // case MOT_VAR_BATTERY_ZERO_ADCU:                          (p_mc->Parameters.BatteryZero_Adcu);                break;
                // case MOT_VAR_BATTERY_FULL_ADCU:                          (p_mc->Parameters.BatteryFull_Adcu);                break;
                #endif
                default: break;
            }
            break;

        case MOT_VAR_ID_TYPE_PARAMS_ANALOG_USER:
            switch((MotVarId_Params_AnalogUser_T)varId.NameId)
            {
                case MOT_VAR_ANALOG_THROTTLE_ZERO_ADCU:             p_mc->AnalogUser.Params.ThrottleZero_Adcu = varValue;           break;
                case MOT_VAR_ANALOG_THROTTLE_MAX_ADCU:              p_mc->AnalogUser.Params.ThrottleMax_Adcu = varValue;            break;
                case MOT_VAR_ANALOG_THROTTLE_EDGE_PIN_IS_ENABLE:    p_mc->AnalogUser.Params.UseThrottleEdgePin = varValue;          break;
                case MOT_VAR_ANALOG_BRAKE_ZERO_ADCU:                p_mc->AnalogUser.Params.BrakeZero_Adcu = varValue;              break;
                case MOT_VAR_ANALOG_BRAKE_MAX_ADCU:                 p_mc->AnalogUser.Params.BrakeMax_Adcu = varValue;               break;
                case MOT_VAR_ANALOG_BRAKE_EDGE_PIN_IS_ENABLE:       p_mc->AnalogUser.Params.UseBrakeEdgePin = varValue;             break;
                case MOT_VAR_ANALOG_DIN_BRAKE_VALUE:                p_mc->AnalogUser.Params.BistateBrakeValue_Scalar16 = varValue;  break;
                case MOT_VAR_ANALOG_DIN_BRAKE_IS_ENABLE:            p_mc->AnalogUser.Params.UseBistateBrakePin = varValue;          break;
                // case MOT_VAR_ANALOG_DIRECTION_PINS:          p_mc->AnalogUser.Params.; break;
                default: break;
            }
            break;

        case MOT_VAR_ID_TYPE_PARAMS_THERMISTOR:
            switch((MotVarId_Instance_Prefix_T)varId.InstancePrefix)
            {
                case MOT_VAR_ID_INSTANCE_PREFIX_BOARD:
                    if((MotVarId_Params_Thermistor_T)varId.NameId == MOT_VAR_THERMISTOR_R0) break;
                    if((MotVarId_Params_Thermistor_T)varId.NameId == MOT_VAR_THERMISTOR_T0) break;
                    if((MotVarId_Params_Thermistor_T)varId.NameId == MOT_VAR_THERMISTOR_B) break;
                    p_thermistor = MotorController_User_GetPtrThermistor(p_mc, varId.Instance);
                    break;
                case MOT_VAR_ID_INSTANCE_PREFIX_MOTOR:
                    if(p_motor != NULL) { p_thermistor = &(p_motor->Thermistor); }
                    break;
                default: break;
            }
            if(p_thermistor != NULL) { SetParameterThermistor(p_thermistor, (MotVarId_Params_Thermistor_T)varId.NameId, varValue); }
            break;

        case MOT_VAR_ID_TYPE_PARAMS_VMONITOR:
            p_vMonitor = MotorController_User_GetPtrVMonitor(p_mc, varId.Instance);
            if(p_vMonitor != NULL)
            {
                switch((MotVarId_Params_VMonitor_T)varId.NameId)
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

        case MOT_VAR_ID_TYPE_PARAMS_PROTOCOL:
            p_protocol = MotorController_User_GetPtrProtocol(p_mc, varId.Instance);
            if(p_protocol != NULL)
            {
                switch((MotVarId_Params_Protocol_T)varId.NameId)
                {
                    // case MOT_VAR_PROTOCOL0_XCVR_ID:                         MotorController_User_SetPtrProtocol(p_mc, 0)->Params.SpecsId; break;
                    // case MOT_VAR_PROTOCOL0_SPECS_ID:                        0; break;
                    // case MOT_VAR_PROTOCOL0_WATCHDOG_TIME:                   0; break;
                    // case MOT_VAR_PROTOCOL0_BAUD_RATE:                       0; break;
                    // case MOT_VAR_PROTOCOL0_IS_ENABLED:                      0; break;
                    default: break;
                }
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
int32_t MotorController_Var_Get(const MotorControllerPtr_T p_mc, MotVarId_T varId)
{
    int32_t value = 0;
    switch((MotVarId_TypeType_T)varId.NameTypeType)
    {
        case MOT_VAR_ID_TYPE_REAL_TIME: value = GetRealTime(p_mc, varId);   break;
        case MOT_VAR_ID_TYPE_PARAMETER: value = GetParameter(p_mc, varId);  break;
        default: break;
    }
    return value;
}

MotVarId_Status_T MotorController_Var_Set(MotorControllerPtr_T p_mc, MotVarId_T varId, int32_t varValue)
{
    MotVarId_Status_T status = MOT_VAR_STATUS_OK;
    switch((MotVarId_TypeType_T)varId.NameTypeType)
    {
        case MOT_VAR_ID_TYPE_REAL_TIME:
            status = (p_mc->Parameters.InputMode == MOTOR_CONTROLLER_INPUT_MODE_PROTOCOL) ?
                SetRealTime(p_mc, varId, varValue) : MOT_VAR_STATUS_ERROR_PROTOCOL_CONTROL_DISABLED;
            break;
        case MOT_VAR_ID_TYPE_PARAMETER:
            status = (MotorController_User_GetStateId(p_mc) == MCSM_STATE_ID_LOCK) ?
                SetParameter(p_mc, varId, varValue) : MOT_VAR_STATUS_ERROR_RUNNING;
            break;
        default: break;
    }
    return status;
}

// uint8_t MotorController_Var_GetSize(MotorControllerPtr_T p_mc, uint16_t varId)
// {

// }
