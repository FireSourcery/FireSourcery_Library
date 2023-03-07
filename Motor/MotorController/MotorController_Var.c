/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery / The Firebrand Forge Inc

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
    @brief
    @version V0
*/
/******************************************************************************/
#include "MotorController_Var.h"
#include "Motor/MotorController/MotorController_User.h"
#include "Math/Q/QFrac16.h"

/*
    Var Reg Read Write Interface
    Variables, Single-Argument Functions
*/
int32_t MotorController_Var_Get(const MotorController_T * p_mc, MotVarId_T varId)
{
    int32_t value = 0;
    Motor_T * p_motor = MotorController_GetPtrMotor(p_mc, 0U);
    Protocol_T * p_protocol = MotorController_User_GetPtrProtocol(p_mc, 1U);

    switch(varId)
    {
        case MOT_VAR_NULL:          value = 0;                                                  break;
        case MOT_VAR_SPEED:         value = Motor_User_GetSpeed_Frac16Abs(p_motor);             break;
        case MOT_VAR_I_PHASE:       value = Motor_User_GetIPhase_FracS16(p_motor);              break;
        case MOT_VAR_V_PHASE:       value = Motor_User_GetVPhase_FracS16(p_motor);              break;
        case MOT_VAR_POWER:         value = Motor_User_GetElectricalPower_FracS16(p_motor);     break;
        case MOT_VAR_RAMP_CMD:      value = Linear_Ramp_GetOutput(&p_motor->Ramp);              break;
        case MOT_VAR_MOTOR_STATE:   value = Motor_User_GetStateId(p_motor);                    break;

        case MOT_VAR_FOC_IA:    value = p_motor->Foc.Ia;        break;
        case MOT_VAR_FOC_IB:    value = p_motor->Foc.Ib;        break;
        case MOT_VAR_FOC_IC:    value = p_motor->Foc.Ic;        break;
        case MOT_VAR_FOC_IQ:    value = p_motor->Foc.Iq;        break;
        case MOT_VAR_FOC_ID:    value = p_motor->Foc.Id;        break;
        case MOT_VAR_FOC_Q_REQ: value = p_motor->Foc.VIqReq;    break;
        case MOT_VAR_FOC_D_REQ: value = p_motor->Foc.VIdReq;    break;
        case MOT_VAR_FOC_VQ:    value = p_motor->Foc.Vq;        break;
        case MOT_VAR_FOC_VD:    value = p_motor->Foc.Vd;        break;

        // case MOT_VAR_DEBUG:     value = qfrac16_sin(Millis());              break;
        case MOT_VAR_DEBUG:     value = p_motor->Foc.Vunlimited;            break;
        case MOT_VAR_MILLIS:    value = Millis();                           break;
        // case MOT_VAR_TX_PACKET_COUNT:   value = Protocol_GetTxPacketCount(p_protocol);      break;
        // case MOT_VAR_RX_PACKET_COUNT:   value = Protocol_GetRxPacketCount(p_protocol);      break;

        case MOT_VAR_MC_STATE:              value = MotorController_User_GetStateId(p_mc);                  break;
        case MOT_VAR_MC_STATUS_FLAGS:       value = 0;                                                      break;
        case MOT_VAR_BATTERY_CHARGE:        value = MotorController_User_GetBatteryCharge_Scalar16(p_mc);   break;
        case MOT_VAR_V_SOURCE:              value = MotorController_User_GetVSource(p_mc, 100U);            break;
        case MOT_VAR_V_SENSOR:              value = MotorController_User_GetVSense(p_mc, 100U);             break;
        case MOT_VAR_V_ACC:                 value = MotorController_User_GetVAcc(p_mc, 100U);               break;
        case MOT_VAR_HEAT_PCB:              value = MotorController_User_GetAdcu(p_mc, MOT_ANALOG_CHANNEL_HEAT_PCB);        break;
        case MOT_VAR_HEAT_MOSFETS:          value = MotorController_User_GetAdcu(p_mc, MOT_ANALOG_CHANNEL_HEAT_MOSFETS);    break;
        case MOT_VAR_HEAT_PCB_DEG_C:        value = MotorController_User_GetHeatPcb_DegC(p_mc, 1U);         break;
        case MOT_VAR_HEAT_MOSFETS_DEG_C:    value = MotorController_User_GetHeatMosfets_DegC(p_mc, 1U);     break;
        case MOT_VAR_ANALOG_THROTTLE:       value = MotAnalogUser_GetThrottle(&p_mc->AnalogUser);           break;  /* Value U16 */
        case MOT_VAR_ANALOG_BRAKE:          value = MotAnalogUser_GetBrake(&p_mc->AnalogUser);              break;  /* Value U16 */

        /* Read Write */
        case MOT_VAR_USER_CMD:                      value = MotorController_User_GetCmdValue(p_mc);                 break;
        case MOT_VAR_DIRECTION:                     value = MotorController_User_GetDirection(p_mc);       break;  /* MotorController_Direction_T */

        case MOT_VAR_ACTIVE_FEEDBACK_MODE:          value = (Motor_User_GetActiveFeedbackMode(p_motor).State);     break;
        case MOT_VAR_ACTIVE_SPEED_LIMIT_SCALAR16:   value = Motor_User_GetActiveSpeedLimit(p_motor);                        break;
        case MOT_VAR_ACTIVE_I_LIMIT_SCALAR16:       value = Motor_User_GetActiveILimit(p_motor);                            break;

        // case MOT_VAR_THROTTLE:      value = MotorController_User_GetCmdValue(p_mc);         break;  /* Write-only or May differ from Write */
        // case MOT_VAR_BRAKE:         value = MotorController_User_GetCmdValue(p_mc);         break;  /* Write-only or May differ from Write */

        /*
            Parameters
        */
        case MOT_VAR_DEFAULT_FEEDBACK_MODE:     value = Motor_User_GetDefaultFeedbackMode(p_motor);     break;
        case MOT_VAR_COMMUTATION_MODE:          value = Motor_User_GetCommutationMode(p_motor);         break;
        case MOT_VAR_SENSOR_MODE:               value = Motor_User_GetSensorMode(p_motor);              break;
        case MOT_VAR_DIRECTION_CALIBRATION:     value = Motor_User_GetDirectionCalibration(p_motor);    break;
        case MOT_VAR_POLE_PAIRS:                value = Motor_User_GetPolePairs(p_motor);               break;
        case MOT_VAR_KV:                        value = Motor_User_GetKv(p_motor);                      break;
        case MOT_VAR_SPEED_FEEDBACK_REF_RPM:    value = Motor_User_GetSpeedFeedbackRef_Rpm(p_motor);    break;
        case MOT_VAR_SPEED_V_REF_RPM:           value = Motor_User_GetSpeedVRef_Rpm(p_motor);           break;

        case MOT_VAR_I_REF_PEAK_ADCU:   value = GLOBAL_MOTOR.I_ZERO_TO_PEAK_ADCU;   break;
        case MOT_VAR_IA_REF_ZERO_ADCU:  value = Motor_User_GetIaZero_Adcu(p_motor); break;
        case MOT_VAR_IB_REF_ZERO_ADCU:  value = Motor_User_GetIbZero_Adcu(p_motor); break;
        case MOT_VAR_IC_REF_ZERO_ADCU:  value = Motor_User_GetIcZero_Adcu(p_motor); break;

        case MOT_VAR_BASE_SPEED_LIMIT_FORWARD_SCALAR16: value = Motor_User_GetSpeedLimitForward_ScalarU16(p_motor); break;
        case MOT_VAR_BASE_SPEED_LIMIT_REVERSE_SCALAR16: value = Motor_User_GetSpeedLimitReverse_ScalarU16(p_motor); break;
        case MOT_VAR_BASE_I_LIMIT_MOTORING_SCALAR16:    value = Motor_User_GetILimitMotoring_ScalarU16(p_motor);    break;
        case MOT_VAR_BASE_I_LIMIT_GENERATING_SCALAR16:  value = Motor_User_GetILimitGenerating_ScalarU16(p_motor);  break;

        case MOT_VAR_RAMP_ACCEL_TIME_CYCLES:            value = Motor_User_GetRampAccel_Cycles(p_motor); break;
        // case MOT_VAR_ALIGN_MODE:                        value = Motor_User_GetAlignMode(p_motor); break;
        case MOT_VAR_ALIGN_POWER_SCALAR16:              value = Motor_User_GetAlignPower_Scalar16(p_motor); break;
        case MOT_VAR_ALIGN_TIME_CYCLES:                 value = Motor_User_GetAlignTime_Cycles(p_motor); break;
        case MOT_VAR_OPEN_LOOP_POWER_SCALAR16:          value = Motor_User_GetOpenLoopPower_Scalar16(p_motor); break;
        case MOT_VAR_OPEN_LOOP_SPEED_SCALAR16:          value = Motor_User_GetOpenLoopSpeed_Scalar16(p_motor); break;
        case MOT_VAR_OPEN_LOOP_ACCEL_TIME_CYCLES:       value = Motor_User_GetOpenLoopAccel_Cycles(p_motor); break;
        // case MOT_VAR_PHASE_PWM_MODE:                    value = Motor_User_GetPhaseModeParam(p_motor); break;

        case MOT_VAR_HALL_SENSOR_TABLE_1: value = p_motor->Hall.Params.SensorsTable[1U];    break;
        case MOT_VAR_HALL_SENSOR_TABLE_2: value = p_motor->Hall.Params.SensorsTable[2U];    break;
        case MOT_VAR_HALL_SENSOR_TABLE_3: value = p_motor->Hall.Params.SensorsTable[3U];    break;
        case MOT_VAR_HALL_SENSOR_TABLE_4: value = p_motor->Hall.Params.SensorsTable[4U];    break;
        case MOT_VAR_HALL_SENSOR_TABLE_5: value = p_motor->Hall.Params.SensorsTable[5U];    break;
        case MOT_VAR_HALL_SENSOR_TABLE_6: value = p_motor->Hall.Params.SensorsTable[6U];    break;

        case MOT_VAR_ENCODER_COUNTS_PER_REVOLUTION:     value = p_motor->Encoder.Params.CountsPerRevolution;            break;
        case MOT_VAR_IS_QUADRATURE_CAPTURE_ENABLED:     value = p_motor->Encoder.Params.IsQuadratureCaptureEnabled;     break;
        case MOT_VAR_IS_A_LEAD_B_POSITIVE:              value = p_motor->Encoder.Params.IsALeadBPositive;               break;
        // case MOT_VAR_ENCODER_EXTENDED_TIMER_DELTA_T_STOP:                            value = 0; break;
        // case MOT_VAR_ENCODER_INTERPOLATE_ANGLE_SCALAR:                               value = 0; break;

        case MOT_VAR_PID_SPEED_KP_FIXED_16:     value = PID_GetKp_Fixed16(&p_motor->PidSpeed);   break;
        case MOT_VAR_PID_SPEED_KI_FIXED_16:     value = PID_GetKi_Fixed16(&p_motor->PidSpeed);   break;
        // case MOT_VAR_PID_SPEED_KD_FIXED_16:  value = PID_GetKd_Fixed16(&p_motor->PidSpeed);   break;
        case MOT_VAR_PID_SPEED_SAMPLE_FREQ:     value = PID_GetSampleFreq(&p_motor->PidSpeed);  break;
        case MOT_VAR_PID_FOC_IQ_KP_FIXED_16:    value = PID_GetKp_Fixed16(&p_motor->PidIq);     break;
        case MOT_VAR_PID_FOC_IQ_KI_FIXED_16:    value = PID_GetKi_Fixed16(&p_motor->PidIq);     break;
        // case MOT_VAR_PID_FOC_IQ_KD_FIXED_16:    value = PID_GetKd_Fixed16(&p_motor->PidIq);  break;
        case MOT_VAR_PID_FOC_IQ_SAMPLE_FREQ:    value = PID_GetSampleFreq(&p_motor->PidIq);     break;
        case MOT_VAR_PID_FOC_ID_KP_FIXED_16:    value = PID_GetKp_Fixed16(&p_motor->PidId);     break;
        case MOT_VAR_PID_FOC_ID_KI_FIXED_16:    value = PID_GetKi_Fixed16(&p_motor->PidId);     break;
        // case MOT_VAR_PID_FOC_ID_KD_FIXED_16:    value = PID_GetKd_Fixed16(&p_motor->PidId);  break;
        case MOT_VAR_PID_FOC_ID_SAMPLE_FREQ:    value = PID_GetSampleFreq(&p_motor->PidId);     break;

        case MOT_VAR_THERMISTOR_MOTOR_LINEAR_T0_ADCU:           value = Thermistor_GetLinearT0_Adcu(&p_motor->Thermistor);          break;
        case MOT_VAR_THERMISTOR_MOTOR_LINEAR_T0_DEG_C:          value = Thermistor_GetLinearT0_DegC(&p_motor->Thermistor);          break;
        case MOT_VAR_THERMISTOR_MOTOR_LINEAR_T1_ADCU:           value = Thermistor_GetLinearT1_Adcu(&p_motor->Thermistor);          break;
        case MOT_VAR_THERMISTOR_MOTOR_LINEAR_T1_DEG_C:          value = Thermistor_GetLinearT1_DegC(&p_motor->Thermistor);          break;
        case MOT_VAR_THERMISTOR_MOTOR_TYPE:                     value = Thermistor_GetType(&p_motor->Thermistor);          break;
        case MOT_VAR_THERMISTOR_MOTOR_R:                        value = Thermistor_GetR0(&p_motor->Thermistor);                     break;
        case MOT_VAR_THERMISTOR_MOTOR_T:                        value = Thermistor_GetT0(&p_motor->Thermistor);                     break;
        case MOT_VAR_THERMISTOR_MOTOR_B:                        value = Thermistor_GetB(&p_motor->Thermistor);                      break;
        case MOT_VAR_THERMISTOR_MOTOR_FAULT_ADCU:               value = Thermistor_GetShutdown_Adcu(&p_motor->Thermistor);          break;
        case MOT_VAR_THERMISTOR_MOTOR_FAULT_THRESHOLD_ADCU:     value = Thermistor_GetShutdownThreshold_Adcu(&p_motor->Thermistor); break;
        case MOT_VAR_THERMISTOR_MOTOR_WARNING_ADCU:             value = Thermistor_GetWarning_Adcu(&p_motor->Thermistor);           break;
        case MOT_VAR_THERMISTOR_MOTOR_WARNING_THRESHOLD_ADCU:   value = Thermistor_GetWarningThreshold_Adcu(&p_motor->Thermistor);  break;
        case MOT_VAR_THERMISTOR_MOTOR_IS_MONITOR_ENABLE:        value = Thermistor_GetIsMonitorEnable(&p_motor->Thermistor);        break;

        case MOT_VAR_V_SOURCE_VOLTS:                            value = MotorController_User_GetVSourceRef(p_mc);   break;
        case MOT_VAR_BATTERY_ZERO_ADCU:                         value = (p_mc->Parameters.BatteryZero_Adcu);        break;
        case MOT_VAR_BATTERY_FULL_ADCU:                         value = (p_mc->Parameters.BatteryFull_Adcu);        break;
        case MOT_VAR_USER_INPUT_MODE:                           value = MotorController_User_GetInputMode(p_mc);    break;
        case MOT_VAR_BRAKE_MODE:                                value = (p_mc->Parameters.BrakeMode);               break;
        case MOT_VAR_ZERO_CMD_MODE:                             value = (p_mc->Parameters.ZeroCmdMode);             break;
        // case MOT_VAR_BUZZER_FLAGS_ENABLE:                       value = (p_mc->Parameters.Buzzer); break;
        case MOT_VAR_OPT_DIN_FUNCTION:                          value = (p_mc->Parameters.OptDinFunction);              break;
        case MOT_VAR_OPT_DIN_SPEED_LIMIT_SCALAR16:              value = (p_mc->Parameters.OptDinSpeedLimit_Scalar16);   break;
        case MOT_VAR_I_LIMIT_LOW_V_SCALAR16:                    value = (p_mc->Parameters.ILimitLowV_Scalar16);         break;
        // case MOT_VAR_CAN_SERVICES_ID:                           value = (p_mc->Parameters.CanServicesId); break;
        // case MOT_VAR_CAN_IS_ENABLE:                             value = (p_mc->Parameters.CanIsEnable); break;

        case MOT_VAR_VMONITOR_SOURCE_LIMIT_UPPER_ADCU:          value = VMonitor_GetLimitUpper(&p_mc->VMonitorSource);      break;
        case MOT_VAR_VMONITOR_SOURCE_LIMIT_LOWER_ADCU:          value = VMonitor_GetLimitLower(&p_mc->VMonitorSource);      break;
        case MOT_VAR_VMONITOR_SOURCE_WARNING_UPPER_ADCU:        value = VMonitor_GetWarningUpper(&p_mc->VMonitorSource);    break;
        case MOT_VAR_VMONITOR_SOURCE_WARNING_LOWER_ADCU:        value = VMonitor_GetWarningLower(&p_mc->VMonitorSource);    break;
        case MOT_VAR_VMONITOR_SOURCE_IS_ENABLE:                 value = VMonitor_GetIsEnable(&p_mc->VMonitorSource);        break;
        case MOT_VAR_VMONITOR_SENSE_LIMIT_UPPER_ADCU:           value = VMonitor_GetLimitUpper(&p_mc->VMonitorSense);      break;
        case MOT_VAR_VMONITOR_SENSE_LIMIT_LOWER_ADCU:           value = VMonitor_GetLimitLower(&p_mc->VMonitorSense);      break;
        case MOT_VAR_VMONITOR_SENSE_WARNING_UPPER_ADCU:         value = VMonitor_GetWarningUpper(&p_mc->VMonitorSense);    break;
        case MOT_VAR_VMONITOR_SENSE_WARNING_LOWER_ADCU:         value = VMonitor_GetWarningLower(&p_mc->VMonitorSense);    break;
        case MOT_VAR_VMONITOR_SENSE_IS_ENABLE:                  value = VMonitor_GetIsEnable(&p_mc->VMonitorSense);        break;
        case MOT_VAR_VMONITOR_ACC_LIMIT_UPPER_ADCU:             value = VMonitor_GetLimitUpper(&p_mc->VMonitorAcc);      break;
        case MOT_VAR_VMONITOR_ACC_LIMIT_LOWER_ADCU:             value = VMonitor_GetLimitLower(&p_mc->VMonitorAcc);      break;
        case MOT_VAR_VMONITOR_ACC_WARNING_UPPER_ADCU:           value = VMonitor_GetWarningUpper(&p_mc->VMonitorAcc);    break;
        case MOT_VAR_VMONITOR_ACC_WARNING_LOWER_ADCU:           value = VMonitor_GetWarningLower(&p_mc->VMonitorAcc);    break;
        case MOT_VAR_VMONITOR_ACC_IS_ENABLE:                    value = VMonitor_GetIsEnable(&p_mc->VMonitorAcc);        break;
        case MOT_VAR_THERMISTOR_PCB_TYPE:                       value = Thermistor_GetLinearT0_Adcu(&p_mc->ThermistorPcb);          break;
        case MOT_VAR_THERMISTOR_PCB_R:                          value = Thermistor_GetLinearT0_DegC(&p_mc->ThermistorPcb);          break;
        case MOT_VAR_THERMISTOR_PCB_T:                          value = Thermistor_GetLinearT1_Adcu(&p_mc->ThermistorPcb);          break;
        case MOT_VAR_THERMISTOR_PCB_B:                          value = Thermistor_GetLinearT1_DegC(&p_mc->ThermistorPcb);          break;
        case MOT_VAR_THERMISTOR_PCB_LINEAR_T0_ADCU:             value = Thermistor_GetType(&p_mc->ThermistorPcb);          break;
        case MOT_VAR_THERMISTOR_PCB_LINEAR_T0_DEG_C:            value = Thermistor_GetR0(&p_mc->ThermistorPcb);                     break;
        case MOT_VAR_THERMISTOR_PCB_LINEAR_T1_ADCU:             value = Thermistor_GetT0(&p_mc->ThermistorPcb);                     break;
        case MOT_VAR_THERMISTOR_PCB_LINEAR_T1_DEG_C:            value = Thermistor_GetB(&p_mc->ThermistorPcb);                      break;
        case MOT_VAR_THERMISTOR_PCB_FAULT_ADCU:                 value = Thermistor_GetShutdown_Adcu(&p_mc->ThermistorPcb);          break;
        case MOT_VAR_THERMISTOR_PCB_FAULT_THRESHOLD_ADCU:       value = Thermistor_GetShutdownThreshold_Adcu(&p_mc->ThermistorPcb); break;
        case MOT_VAR_THERMISTOR_PCB_WARNING_ADCU:               value = Thermistor_GetWarning_Adcu(&p_mc->ThermistorPcb);           break;
        case MOT_VAR_THERMISTOR_PCB_WARNING_THRESHOLD_ADCU:     value = Thermistor_GetWarningThreshold_Adcu(&p_mc->ThermistorPcb);  break;
        case MOT_VAR_THERMISTOR_PCB_IS_MONITOR_ENABLE:          value = Thermistor_GetIsMonitorEnable(&p_mc->ThermistorPcb);        break;
        case MOT_VAR_THERMISTOR_MOSFETS_TYPE:                   value = Thermistor_GetLinearT0_Adcu(&p_mc->ThermistorMosfets);          break;
        case MOT_VAR_THERMISTOR_MOSFETS_R:                      value = Thermistor_GetLinearT0_DegC(&p_mc->ThermistorMosfets);          break;
        case MOT_VAR_THERMISTOR_MOSFETS_T:                      value = Thermistor_GetLinearT1_Adcu(&p_mc->ThermistorMosfets);          break;
        case MOT_VAR_THERMISTOR_MOSFETS_B:                      value = Thermistor_GetLinearT1_DegC(&p_mc->ThermistorMosfets);          break;
        case MOT_VAR_THERMISTOR_MOSFETS_LINEAR_T0_ADCU:         value = Thermistor_GetType(&p_mc->ThermistorMosfets);          break;
        case MOT_VAR_THERMISTOR_MOSFETS_LINEAR_T0_DEG_C:        value = Thermistor_GetR0(&p_mc->ThermistorMosfets);                     break;
        case MOT_VAR_THERMISTOR_MOSFETS_LINEAR_T1_ADCU:         value = Thermistor_GetT0(&p_mc->ThermistorMosfets);                     break;
        case MOT_VAR_THERMISTOR_MOSFETS_LINEAR_T1_DEG_C:        value = Thermistor_GetB(&p_mc->ThermistorMosfets);                      break;
        case MOT_VAR_THERMISTOR_MOSFETS_FAULT_ADCU:             value = Thermistor_GetShutdown_Adcu(&p_mc->ThermistorMosfets);          break;
        case MOT_VAR_THERMISTOR_MOSFETS_FAULT_THRESHOLD_ADCU:   value = Thermistor_GetShutdownThreshold_Adcu(&p_mc->ThermistorMosfets); break;
        case MOT_VAR_THERMISTOR_MOSFETS_WARNING_ADCU:           value = Thermistor_GetWarning_Adcu(&p_mc->ThermistorMosfets);           break;
        case MOT_VAR_THERMISTOR_MOSFETS_WARNING_THRESHOLD_ADCU: value = Thermistor_GetWarningThreshold_Adcu(&p_mc->ThermistorMosfets);  break;
        case MOT_VAR_THERMISTOR_MOSFETS_IS_MONITOR_ENABLE:      value = Thermistor_GetIsMonitorEnable(&p_mc->ThermistorMosfets);        break;

        case MOT_VAR_ANALOG_THROTTLE_ZERO_ADCU:                 value = p_mc->AnalogUser.Params.ThrottleZero_Adcu; break;
        case MOT_VAR_ANALOG_THROTTLE_MAX_ADCU:                  value = p_mc->AnalogUser.Params.ThrottleMax_Adcu; break;
        case MOT_VAR_ANALOG_THROTTLE_EDGE_PIN_IS_ENABLE:        value = p_mc->AnalogUser.Params.UseThrottleEdgePin; break;
        case MOT_VAR_ANALOG_BRAKE_ZERO_ADCU:                    value = p_mc->AnalogUser.Params.BrakeZero_Adcu; break;
        case MOT_VAR_ANALOG_BRAKE_MAX_ADCU:                     value = p_mc->AnalogUser.Params.BrakeMax_Adcu; break;
        case MOT_VAR_ANALOG_BRAKE_EDGE_PIN_IS_ENABLE:           value = p_mc->AnalogUser.Params.UseBrakeEdgePin; break;
        case MOT_VAR_ANALOG_DIN_BRAKE_VALUE_FRAC16:             value = p_mc->AnalogUser.Params.BistateBrakeValue_Frac16; break;
        case MOT_VAR_ANALOG_DIN_BRAKE_IS_ENABLE:                value = p_mc->AnalogUser.Params.UseBistateBrakePin; break;
        // case MOT_VAR_ANALOG_DIRECTION_PINS:                  value = p_mc->AnalogUser.Params.; break;

        case MOT_VAR_PROTOCOL0_XCVR_ID:                         value = MotorController_User_GetPtrProtocol(p_mc, 0)->Params.SpecsId; break;
        case MOT_VAR_PROTOCOL0_SPECS_ID:                        value = 0; break;
        case MOT_VAR_PROTOCOL0_WATCHDOG_TIME:                   value = 0; break;
        case MOT_VAR_PROTOCOL0_BAUD_RATE:                       value = 0; break;
        case MOT_VAR_PROTOCOL0_IS_ENABLED:                      value = 0; break;
        case MOT_VAR_PROTOCOL1_XCVR_ID:                         value = 0; break;
        case MOT_VAR_PROTOCOL1_SPECS_ID:                        value = 0; break;
        case MOT_VAR_PROTOCOL1_WATCHDOG_TIME:                   value = 0; break;
        case MOT_VAR_PROTOCOL1_BAUD_RATE:                       value = 0; break;
        case MOT_VAR_PROTOCOL1_IS_ENABLED:                      value = 0; break;
        case MOT_VAR_PROTOCOL2_XCVR_ID:                         value = 0; break;
        case MOT_VAR_PROTOCOL2_SPECS_ID:                        value = 0; break;
        case MOT_VAR_PROTOCOL2_WATCHDOG_TIME:                   value = 0; break;
        case MOT_VAR_PROTOCOL2_BAUD_RATE:                       value = 0; break;
        case MOT_VAR_PROTOCOL2_IS_ENABLED:                      value = 0; break;

        case MOT_VAR_I_MAX_AMP:             value = MotorController_User_GetIMax(p_mc);         break;
        case MOT_VAR_V_MAX_VOLTS:           value = MotorController_User_GetVMax(p_mc);         break;
        case MOT_VAR_VERSION_KMC_REG32:     value = MotorController_User_GetMainVersion(p_mc);  break;
        default: value = 0xAAU; break;
    }

    return value;
}


/*!
    @return Variable length
*/
uint8_t MotorController_Var_Set(MotorController_T * p_mc, MotVarId_T varId, int32_t varValue)
{
    uint8_t writeCount = 0U; // identify var size on write?
    Motor_T * p_motor = MotorController_GetPtrMotor(p_mc, 0U);

    volatile int a = 0;
    volatile int b = 0;
    a = b;

    switch(varId)
    {
        case MOT_VAR_USER_CMD:      MotorController_User_SetCmdValue(p_mc, (uint16_t)varValue);                         writeCount = 2U;    break;
        // case MOT_VAR_ACTIVE_FEEDBACK_MODE:  _Motor_User_ActivateControlMode(p_motor, (Motor_FeedbackModeId_T)varValue);  writeCount = 2U;   break;
        case MOT_VAR_THROTTLE:      MotorController_User_SetCmdThrottle(p_mc, (uint16_t)varValue);                      writeCount = 2U;    break;
        case MOT_VAR_BRAKE:         MotorController_User_SetCmdBrake(p_mc, (uint16_t)varValue);                         writeCount = 2U;    break;
        case MOT_VAR_DIRECTION:     MotorController_User_SetDirection(p_mc, (MotorController_Direction_T)varValue);     writeCount = 2U;    break;  /* Value 0: Neutral, 1: Reverse, 2: Forward */
        case MOT_VAR_BEEP:          MotorController_User_BeepN(p_mc, 500U, 500U, varValue);                             writeCount = 2U;    break;
        case MOT_VAR_CALIBRATE_SENSOR:  Motor_User_ActivateCalibrationSensor(p_motor);                                  writeCount = 2U;    break;

        /*
            Parameters
        */
        case MOT_VAR_DEFAULT_FEEDBACK_MODE:     Motor_User_SetDefaultFeedbackMode(p_motor, (Motor_FeedbackModeId_T)varValue);   writeCount = 2U;   break;

        case MOT_VAR_COMMUTATION_MODE:          writeCount = Motor_User_GetCommutationMode(p_motor);         break;
        case MOT_VAR_SENSOR_MODE:               writeCount = Motor_User_GetSensorMode(p_motor);              break;
        case MOT_VAR_DIRECTION_CALIBRATION:     writeCount = Motor_User_GetDirectionCalibration(p_motor);    break;

        case MOT_VAR_POLE_PAIRS:                Motor_User_SetPolePairs(p_motor, varValue); writeCount = 2U;    break;
        case MOT_VAR_KV:                        writeCount = Motor_User_GetKv(p_motor);                      break;
        case MOT_VAR_SPEED_FEEDBACK_REF_RPM:    writeCount = Motor_User_GetSpeedFeedbackRef_Rpm(p_motor);    break;
        case MOT_VAR_SPEED_V_REF_RPM:           writeCount = Motor_User_GetSpeedVRef_Rpm(p_motor);           break;

        case MOT_VAR_I_REF_PEAK_ADCU:   writeCount = GLOBAL_MOTOR.I_ZERO_TO_PEAK_ADCU;   break;
        case MOT_VAR_IA_REF_ZERO_ADCU:  writeCount = Motor_User_GetIaZero_Adcu(p_motor); break;
        case MOT_VAR_IB_REF_ZERO_ADCU:  writeCount = Motor_User_GetIbZero_Adcu(p_motor); break;
        case MOT_VAR_IC_REF_ZERO_ADCU:  writeCount = Motor_User_GetIcZero_Adcu(p_motor); break;

        case MOT_VAR_BASE_SPEED_LIMIT_FORWARD_SCALAR16: writeCount = Motor_User_GetSpeedLimitForward_ScalarU16(p_motor); break;
        case MOT_VAR_BASE_SPEED_LIMIT_REVERSE_SCALAR16: writeCount = Motor_User_GetSpeedLimitReverse_ScalarU16(p_motor); break;
        case MOT_VAR_BASE_I_LIMIT_MOTORING_SCALAR16:    writeCount = Motor_User_GetILimitMotoring_ScalarU16(p_motor);    break;
        case MOT_VAR_BASE_I_LIMIT_GENERATING_SCALAR16:  writeCount = Motor_User_GetILimitGenerating_ScalarU16(p_motor);  break;

        case MOT_VAR_RAMP_ACCEL_TIME_CYCLES:            writeCount = Motor_User_GetRampAccel_Cycles(p_motor); break;
        // case MOT_VAR_ALIGN_MODE:                        writeCount = Motor_User_GetAlignMode(p_motor); break;
        case MOT_VAR_ALIGN_POWER_SCALAR16:              writeCount = Motor_User_GetAlignPower_Scalar16(p_motor); break;
        case MOT_VAR_ALIGN_TIME_CYCLES:                 writeCount = Motor_User_GetAlignTime_Cycles(p_motor); break;
        case MOT_VAR_OPEN_LOOP_POWER_SCALAR16:          writeCount = Motor_User_GetOpenLoopPower_Scalar16(p_motor); break;
        case MOT_VAR_OPEN_LOOP_SPEED_SCALAR16:          writeCount = Motor_User_GetOpenLoopSpeed_Scalar16(p_motor); break;
        case MOT_VAR_OPEN_LOOP_ACCEL_TIME_CYCLES:       writeCount = Motor_User_GetOpenLoopAccel_Cycles(p_motor); break;
        // case MOT_VAR_PHASE_PWM_MODE:                    writeCount = Motor_User_GetPhaseModeParam(p_motor); break;

        case MOT_VAR_HALL_SENSOR_TABLE_1: writeCount = p_motor->Hall.Params.SensorsTable[1U];    break;
        case MOT_VAR_HALL_SENSOR_TABLE_2: writeCount = p_motor->Hall.Params.SensorsTable[2U];    break;
        case MOT_VAR_HALL_SENSOR_TABLE_3: writeCount = p_motor->Hall.Params.SensorsTable[3U];    break;
        case MOT_VAR_HALL_SENSOR_TABLE_4: writeCount = p_motor->Hall.Params.SensorsTable[4U];    break;
        case MOT_VAR_HALL_SENSOR_TABLE_5: writeCount = p_motor->Hall.Params.SensorsTable[5U];    break;
        case MOT_VAR_HALL_SENSOR_TABLE_6: writeCount = p_motor->Hall.Params.SensorsTable[6U];    break;

        case MOT_VAR_ENCODER_COUNTS_PER_REVOLUTION:     writeCount = p_motor->Encoder.Params.CountsPerRevolution;            break;
        case MOT_VAR_IS_QUADRATURE_CAPTURE_ENABLED:     writeCount = p_motor->Encoder.Params.IsQuadratureCaptureEnabled;     break;
        case MOT_VAR_IS_A_LEAD_B_POSITIVE:              writeCount = p_motor->Encoder.Params.IsALeadBPositive;               break;
        // case MOT_VAR_ENCODER_EXTENDED_TIMER_DELTA_T_STOP:                            writeCount = 0; break;
        // case MOT_VAR_ENCODER_INTERPOLATE_ANGLE_SCALAR:                               writeCount = 0; break;

        case MOT_VAR_PID_SPEED_KP_FIXED_16:     writeCount = PID_GetKp_Fixed16(&p_motor->PidSpeed);   break;
        case MOT_VAR_PID_SPEED_KI_FIXED_16:     writeCount = PID_GetKi_Fixed16(&p_motor->PidSpeed);   break;
        // case MOT_VAR_PID_SPEED_KD_FIXED_16:  writeCount = PID_GetKd_Fixed16(&p_motor->PidSpeed);   break;
        case MOT_VAR_PID_SPEED_SAMPLE_FREQ:     writeCount = PID_GetSampleFreq(&p_motor->PidSpeed);  break;
        case MOT_VAR_PID_FOC_IQ_KP_FIXED_16:    writeCount = PID_GetKp_Fixed16(&p_motor->PidIq);     break;
        case MOT_VAR_PID_FOC_IQ_KI_FIXED_16:    writeCount = PID_GetKi_Fixed16(&p_motor->PidIq);     break;
        // case MOT_VAR_PID_FOC_IQ_KD_FIXED_16:    writeCount = PID_GetKd_Fixed16(&p_motor->PidIq);  break;
        case MOT_VAR_PID_FOC_IQ_SAMPLE_FREQ:    writeCount = PID_GetSampleFreq(&p_motor->PidIq);     break;
        case MOT_VAR_PID_FOC_ID_KP_FIXED_16:    writeCount = PID_GetKp_Fixed16(&p_motor->PidId);     break;
        case MOT_VAR_PID_FOC_ID_KI_FIXED_16:    writeCount = PID_GetKi_Fixed16(&p_motor->PidId);     break;
        // case MOT_VAR_PID_FOC_ID_KD_FIXED_16:    writeCount = PID_GetKd_Fixed16(&p_motor->PidId);  break;
        case MOT_VAR_PID_FOC_ID_SAMPLE_FREQ:    writeCount = PID_GetSampleFreq(&p_motor->PidId);     break;

        case MOT_VAR_THERMISTOR_MOTOR_LINEAR_T0_ADCU:           writeCount = Thermistor_GetLinearT0_Adcu(&p_motor->Thermistor);          break;
        case MOT_VAR_THERMISTOR_MOTOR_LINEAR_T0_DEG_C:          writeCount = Thermistor_GetLinearT0_DegC(&p_motor->Thermistor);          break;
        case MOT_VAR_THERMISTOR_MOTOR_LINEAR_T1_ADCU:           writeCount = Thermistor_GetLinearT1_Adcu(&p_motor->Thermistor);          break;
        case MOT_VAR_THERMISTOR_MOTOR_LINEAR_T1_DEG_C:          writeCount = Thermistor_GetLinearT1_DegC(&p_motor->Thermistor);          break;
        case MOT_VAR_THERMISTOR_MOTOR_TYPE:                     writeCount = Thermistor_GetType(&p_motor->Thermistor);          break;
        case MOT_VAR_THERMISTOR_MOTOR_R:                        writeCount = Thermistor_GetR0(&p_motor->Thermistor);                     break;
        case MOT_VAR_THERMISTOR_MOTOR_T:                        writeCount = Thermistor_GetT0(&p_motor->Thermistor);                     break;
        case MOT_VAR_THERMISTOR_MOTOR_B:                        writeCount = Thermistor_GetB(&p_motor->Thermistor);                      break;
        case MOT_VAR_THERMISTOR_MOTOR_FAULT_ADCU:               writeCount = Thermistor_GetShutdown_Adcu(&p_motor->Thermistor);          break;
        case MOT_VAR_THERMISTOR_MOTOR_FAULT_THRESHOLD_ADCU:     writeCount = Thermistor_GetShutdownThreshold_Adcu(&p_motor->Thermistor); break;
        case MOT_VAR_THERMISTOR_MOTOR_WARNING_ADCU:             writeCount = Thermistor_GetWarning_Adcu(&p_motor->Thermistor);           break;
        case MOT_VAR_THERMISTOR_MOTOR_WARNING_THRESHOLD_ADCU:   writeCount = Thermistor_GetWarningThreshold_Adcu(&p_motor->Thermistor);  break;
        case MOT_VAR_THERMISTOR_MOTOR_IS_MONITOR_ENABLE:        writeCount = Thermistor_GetIsMonitorEnable(&p_motor->Thermistor);        break;

        case MOT_VAR_V_SOURCE_VOLTS:                            writeCount = MotorController_User_GetVSourceRef(p_mc);   break;
        case MOT_VAR_BATTERY_ZERO_ADCU:                         writeCount = (p_mc->Parameters.BatteryZero_Adcu);        break;
        case MOT_VAR_BATTERY_FULL_ADCU:                         writeCount = (p_mc->Parameters.BatteryFull_Adcu);        break;
        case MOT_VAR_USER_INPUT_MODE:                           MotorController_User_SetInputMode_Blocking(p_mc, (MotorController_InputMode_T)varValue);    writeCount = 2U;break;
        case MOT_VAR_BRAKE_MODE:                                writeCount = (p_mc->Parameters.BrakeMode);               break;
        case MOT_VAR_ZERO_CMD_MODE:                             writeCount = (p_mc->Parameters.ZeroCmdMode);             break;
        // case MOT_VAR_BUZZER_FLAGS_ENABLE:                       writeCount = (p_mc->Parameters.Buzzer); break;
        case MOT_VAR_OPT_DIN_FUNCTION:                          writeCount = (p_mc->Parameters.OptDinFunction);              break;
        case MOT_VAR_OPT_DIN_SPEED_LIMIT_SCALAR16:              writeCount = (p_mc->Parameters.OptDinSpeedLimit_Scalar16);   break;
        case MOT_VAR_I_LIMIT_LOW_V_SCALAR16:                    writeCount = (p_mc->Parameters.ILimitLowV_Scalar16);         break;
        // case MOT_VAR_CAN_SERVICES_ID:                           writeCount = (p_mc->Parameters.CanServicesId); break;
        // case MOT_VAR_CAN_IS_ENABLE:                             writeCount = (p_mc->Parameters.CanIsEnable); break;

        case MOT_VAR_VMONITOR_SOURCE_LIMIT_UPPER_ADCU:          writeCount = VMonitor_GetLimitUpper(&p_mc->VMonitorSource);      break;
        case MOT_VAR_VMONITOR_SOURCE_LIMIT_LOWER_ADCU:          writeCount = VMonitor_GetLimitLower(&p_mc->VMonitorSource);      break;
        case MOT_VAR_VMONITOR_SOURCE_WARNING_UPPER_ADCU:        writeCount = VMonitor_GetWarningUpper(&p_mc->VMonitorSource);    break;
        case MOT_VAR_VMONITOR_SOURCE_WARNING_LOWER_ADCU:        writeCount = VMonitor_GetWarningLower(&p_mc->VMonitorSource);    break;
        case MOT_VAR_VMONITOR_SOURCE_IS_ENABLE:                 writeCount = VMonitor_GetIsEnable(&p_mc->VMonitorSource);        break;
        case MOT_VAR_VMONITOR_SENSE_LIMIT_UPPER_ADCU:           writeCount = VMonitor_GetLimitUpper(&p_mc->VMonitorSense);      break;
        case MOT_VAR_VMONITOR_SENSE_LIMIT_LOWER_ADCU:           writeCount = VMonitor_GetLimitLower(&p_mc->VMonitorSense);      break;
        case MOT_VAR_VMONITOR_SENSE_WARNING_UPPER_ADCU:         writeCount = VMonitor_GetWarningUpper(&p_mc->VMonitorSense);    break;
        case MOT_VAR_VMONITOR_SENSE_WARNING_LOWER_ADCU:         writeCount = VMonitor_GetWarningLower(&p_mc->VMonitorSense);    break;
        case MOT_VAR_VMONITOR_SENSE_IS_ENABLE:                  writeCount = VMonitor_GetIsEnable(&p_mc->VMonitorSense);        break;
        case MOT_VAR_VMONITOR_ACC_LIMIT_UPPER_ADCU:             writeCount = VMonitor_GetLimitUpper(&p_mc->VMonitorAcc);      break;
        case MOT_VAR_VMONITOR_ACC_LIMIT_LOWER_ADCU:             writeCount = VMonitor_GetLimitLower(&p_mc->VMonitorAcc);      break;
        case MOT_VAR_VMONITOR_ACC_WARNING_UPPER_ADCU:           writeCount = VMonitor_GetWarningUpper(&p_mc->VMonitorAcc);    break;
        case MOT_VAR_VMONITOR_ACC_WARNING_LOWER_ADCU:           writeCount = VMonitor_GetWarningLower(&p_mc->VMonitorAcc);    break;
        case MOT_VAR_VMONITOR_ACC_IS_ENABLE:                    writeCount = VMonitor_GetIsEnable(&p_mc->VMonitorAcc);        break;
        case MOT_VAR_THERMISTOR_PCB_TYPE:                       writeCount = Thermistor_GetLinearT0_Adcu(&p_mc->ThermistorPcb);          break;
        case MOT_VAR_THERMISTOR_PCB_R:                          writeCount = Thermistor_GetLinearT0_DegC(&p_mc->ThermistorPcb);          break;
        case MOT_VAR_THERMISTOR_PCB_T:                          writeCount = Thermistor_GetLinearT1_Adcu(&p_mc->ThermistorPcb);          break;
        case MOT_VAR_THERMISTOR_PCB_B:                          writeCount = Thermistor_GetLinearT1_DegC(&p_mc->ThermistorPcb);          break;
        case MOT_VAR_THERMISTOR_PCB_LINEAR_T0_ADCU:             writeCount = Thermistor_GetType(&p_mc->ThermistorPcb);          break;
        case MOT_VAR_THERMISTOR_PCB_LINEAR_T0_DEG_C:            writeCount = Thermistor_GetR0(&p_mc->ThermistorPcb);                     break;
        case MOT_VAR_THERMISTOR_PCB_LINEAR_T1_ADCU:             writeCount = Thermistor_GetT0(&p_mc->ThermistorPcb);                     break;
        case MOT_VAR_THERMISTOR_PCB_LINEAR_T1_DEG_C:            writeCount = Thermistor_GetB(&p_mc->ThermistorPcb);                      break;
        case MOT_VAR_THERMISTOR_PCB_FAULT_ADCU:                 writeCount = Thermistor_GetShutdown_Adcu(&p_mc->ThermistorPcb);          break;
        case MOT_VAR_THERMISTOR_PCB_FAULT_THRESHOLD_ADCU:       writeCount = Thermistor_GetShutdownThreshold_Adcu(&p_mc->ThermistorPcb); break;
        case MOT_VAR_THERMISTOR_PCB_WARNING_ADCU:               writeCount = Thermistor_GetWarning_Adcu(&p_mc->ThermistorPcb);           break;
        case MOT_VAR_THERMISTOR_PCB_WARNING_THRESHOLD_ADCU:     writeCount = Thermistor_GetWarningThreshold_Adcu(&p_mc->ThermistorPcb);  break;
        case MOT_VAR_THERMISTOR_PCB_IS_MONITOR_ENABLE:          writeCount = Thermistor_GetIsMonitorEnable(&p_mc->ThermistorPcb);        break;
        case MOT_VAR_THERMISTOR_MOSFETS_TYPE:                   writeCount = Thermistor_GetLinearT0_Adcu(&p_mc->ThermistorMosfets);          break;
        case MOT_VAR_THERMISTOR_MOSFETS_R:                      writeCount = Thermistor_GetLinearT0_DegC(&p_mc->ThermistorMosfets);          break;
        case MOT_VAR_THERMISTOR_MOSFETS_T:                      writeCount = Thermistor_GetLinearT1_Adcu(&p_mc->ThermistorMosfets);          break;
        case MOT_VAR_THERMISTOR_MOSFETS_B:                      writeCount = Thermistor_GetLinearT1_DegC(&p_mc->ThermistorMosfets);          break;
        case MOT_VAR_THERMISTOR_MOSFETS_LINEAR_T0_ADCU:         writeCount = Thermistor_GetType(&p_mc->ThermistorMosfets);          break;
        case MOT_VAR_THERMISTOR_MOSFETS_LINEAR_T0_DEG_C:        writeCount = Thermistor_GetR0(&p_mc->ThermistorMosfets);                     break;
        case MOT_VAR_THERMISTOR_MOSFETS_LINEAR_T1_ADCU:         writeCount = Thermistor_GetT0(&p_mc->ThermistorMosfets);                     break;
        case MOT_VAR_THERMISTOR_MOSFETS_LINEAR_T1_DEG_C:        writeCount = Thermistor_GetB(&p_mc->ThermistorMosfets);                      break;
        case MOT_VAR_THERMISTOR_MOSFETS_FAULT_ADCU:             writeCount = Thermistor_GetShutdown_Adcu(&p_mc->ThermistorMosfets);          break;
        case MOT_VAR_THERMISTOR_MOSFETS_FAULT_THRESHOLD_ADCU:   writeCount = Thermistor_GetShutdownThreshold_Adcu(&p_mc->ThermistorMosfets); break;
        case MOT_VAR_THERMISTOR_MOSFETS_WARNING_ADCU:           writeCount = Thermistor_GetWarning_Adcu(&p_mc->ThermistorMosfets);           break;
        case MOT_VAR_THERMISTOR_MOSFETS_WARNING_THRESHOLD_ADCU: writeCount = Thermistor_GetWarningThreshold_Adcu(&p_mc->ThermistorMosfets);  break;
        case MOT_VAR_THERMISTOR_MOSFETS_IS_MONITOR_ENABLE:      writeCount = Thermistor_GetIsMonitorEnable(&p_mc->ThermistorMosfets);        break;

        case MOT_VAR_ANALOG_THROTTLE_ZERO_ADCU:                 writeCount = p_mc->AnalogUser.Params.ThrottleZero_Adcu; break;
        case MOT_VAR_ANALOG_THROTTLE_MAX_ADCU:                  writeCount = p_mc->AnalogUser.Params.ThrottleMax_Adcu; break;
        case MOT_VAR_ANALOG_THROTTLE_EDGE_PIN_IS_ENABLE:        writeCount = p_mc->AnalogUser.Params.UseThrottleEdgePin; break;
        case MOT_VAR_ANALOG_BRAKE_ZERO_ADCU:                    writeCount = p_mc->AnalogUser.Params.BrakeZero_Adcu; break;
        case MOT_VAR_ANALOG_BRAKE_MAX_ADCU:                     writeCount = p_mc->AnalogUser.Params.BrakeMax_Adcu; break;
        case MOT_VAR_ANALOG_BRAKE_EDGE_PIN_IS_ENABLE:           writeCount = p_mc->AnalogUser.Params.UseBrakeEdgePin; break;
        case MOT_VAR_ANALOG_DIN_BRAKE_VALUE_FRAC16:             writeCount = p_mc->AnalogUser.Params.BistateBrakeValue_Frac16; break;
        case MOT_VAR_ANALOG_DIN_BRAKE_IS_ENABLE:                writeCount = p_mc->AnalogUser.Params.UseBistateBrakePin; break;
        // case MOT_VAR_ANALOG_DIRECTION_PINS:                  writeCount = p_mc->AnalogUser.Params.; break;

        case MOT_VAR_PROTOCOL0_XCVR_ID:                         writeCount = MotorController_User_GetPtrProtocol(p_mc, 0)->Params.SpecsId; break;
        case MOT_VAR_PROTOCOL0_SPECS_ID:                        writeCount = 0; break;
        case MOT_VAR_PROTOCOL0_WATCHDOG_TIME:                   writeCount = 0; break;
        case MOT_VAR_PROTOCOL0_BAUD_RATE:                       writeCount = 0; break;
        case MOT_VAR_PROTOCOL0_IS_ENABLED:                      writeCount = 0; break;
        case MOT_VAR_PROTOCOL1_XCVR_ID:                         writeCount = 0; break;
        case MOT_VAR_PROTOCOL1_SPECS_ID:                        writeCount = 0; break;
        case MOT_VAR_PROTOCOL1_WATCHDOG_TIME:                   writeCount = 0; break;
        case MOT_VAR_PROTOCOL1_BAUD_RATE:                       writeCount = 0; break;
        case MOT_VAR_PROTOCOL1_IS_ENABLED:                      writeCount = 0; break;
        case MOT_VAR_PROTOCOL2_XCVR_ID:                         writeCount = 0; break;
        case MOT_VAR_PROTOCOL2_SPECS_ID:                        writeCount = 0; break;
        case MOT_VAR_PROTOCOL2_WATCHDOG_TIME:                   writeCount = 0; break;
        case MOT_VAR_PROTOCOL2_BAUD_RATE:                       writeCount = 0; break;
        case MOT_VAR_PROTOCOL2_IS_ENABLED:                      writeCount = 0; break;

        default: writeCount = 0U; break;
    }

    // return writeCount;
}


uint8_t MotorController_Var_GetSize(MotorController_T * p_mc, MotVarId_T varId)
{

}