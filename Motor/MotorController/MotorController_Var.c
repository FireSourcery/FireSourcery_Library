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
    @brief
    @version V0
*/
/******************************************************************************/
#include "MotorController_Var.h"
#include "Motor/MotorController/MotorController_User.h"
// #include "Math/Q/QFrac16.h"


/******************************************************************************/
/*!
    User Get Set Vars by Id
*/
/******************************************************************************/

/*
    Var Reg Read Write Interface
    Variables, Single-Argument Functions
*/
int32_t MotorController_Var_Get(const MotorController_T * p_mc, MotVarId_T varId)
{
    int32_t value = 0;
    Motor_T * p_motor = MotorController_GetPtrMotor(p_mc, 0U); //switch(arg.Motor)
    Protocol_T * p_protocol = MotorController_User_GetPtrProtocol(p_mc, 1U);

    // volatile
    MotVarId_Arg_T arg = (MotVarId_Arg_T)varId.Arg;

    switch((MotVarId_Prefix_T)arg.Prefix)
    {
        case MOT_VAR_ID_PREFIX_REAL_TIME_MONITOR:
            switch((MotVarId_RealTimeMonitor_T)varId.Id8)
            {
                case MOT_VAR_ZERO:                  value = 0;                                                  break;
                case MOT_VAR_MILLIS:                value = Millis();                                           break;
                // case MOT_VAR_DEBUG:              value = qfrac16_sin(Millis());                              break;

                case MOT_VAR_SPEED:                 value = Motor_User_GetSpeed_FracU16(p_motor);               break;
                case MOT_VAR_I_PHASE:               value = Motor_User_GetIPhase_FracS16(p_motor);              break;
                case MOT_VAR_V_PHASE:               value = Motor_User_GetVPhase_FracS16(p_motor);              break;
                case MOT_VAR_POWER:                 value = Motor_User_GetElectricalPower_FracS16(p_motor);     break;
                case MOT_VAR_RAMP_CMD:              value = Linear_Ramp_GetOutput(&p_motor->Ramp);              break;
                case MOT_VAR_MOTOR_STATE:           value = Motor_User_GetStateId(p_motor);                     break;
                case MOT_VAR_MOTOR_STATUS_FLAGS:    value = Motor_User_GetStatusFlags(p_motor).State;           break;
                case MOT_VAR_MOTOR_FAULT_FLAGS:     value = Motor_User_GetFaultFlags(p_motor).State;            break;


                case MOT_VAR_MOTOR_HEAT:
                    value = (arg.Alt == 0U) ? Motor_User_GetHeat_Adcu(p_motor) : Motor_User_GetHeat_DegC(p_motor, 1U);
                    break;

                // case MOT_VAR_MOTOR_HEAT_ADCU:       value = Motor_User_GetHeat_Adcu(p_motor);           break;
                // case MOT_VAR_MOTOR_HEAT_DEG_C:      value = Motor_User_GetHeat_DegC(p_motor, 1U);       break;
                case MOT_VAR_MOTOR_ACTIVE_SPEED_LIMIT:      value = Motor_User_GetActiveSpeedLimit(p_motor);                    break;
                case MOT_VAR_MOTOR_ACTIVE_I_LIMIT:          value = Motor_User_GetActiveILimit(p_motor);                        break;

                case MOT_VAR_FOC_IA:    value = p_motor->Foc.Ia;        break;
                case MOT_VAR_FOC_IB:    value = p_motor->Foc.Ib;        break;
                case MOT_VAR_FOC_IC:    value = p_motor->Foc.Ic;        break;
                case MOT_VAR_FOC_IQ:    value = p_motor->Foc.Iq;        break;
                case MOT_VAR_FOC_ID:    value = p_motor->Foc.Id;        break;
                case MOT_VAR_FOC_VQ:    value = p_motor->Foc.Vq;        break;
                case MOT_VAR_FOC_VD:    value = p_motor->Foc.Vd;        break;
                case MOT_VAR_FOC_Q_REQ: value = p_motor->Foc.QReq;      break;
                case MOT_VAR_FOC_D_REQ: value = p_motor->Foc.DReq;      break;
                // case MOT_VAR_FOC_:     value = p_motor->Foc.Vunlimited;     break;
                case MOT_VAR_ENCODER_FREQ:    value = p_motor->Encoder.FreqD;   break;


                case MOT_VAR_MC_STATE:              value = MotorController_User_GetStateId(p_mc);                  break;
                case MOT_VAR_MC_STATUS_FLAGS:       value = 0;                                                      break;
                case MOT_VAR_MC_ERROR_FLAGS:        value = 0;                                                      break;
                case MOT_VAR_BATTERY_CHARGE:        value = MotorController_User_GetBatteryCharge_Scalar16(p_mc);                   break;
                case MOT_VAR_V_SOURCE:              value = MotorController_User_GetVSource(p_mc, 10U);                             break;
                    // value = (arg.Units == 0U) ?
                case MOT_VAR_V_SENSOR:              value = MotorController_User_GetVSense(p_mc, 1000U);                            break;
                case MOT_VAR_V_ACC:                 value = MotorController_User_GetVAcc(p_mc, 1000U);                              break;
                case MOT_VAR_HEAT_PCB:              value = MotorController_User_GetAdcu(p_mc, MOT_ANALOG_CHANNEL_HEAT_PCB);        break;
                // case MOT_VAR_HEAT_PCB_DEG_C:        value = MotorController_User_GetHeatPcb_DegC(p_mc, 1U);         break;
                case MOT_VAR_HEAT_MOSFETS:          value = MotorController_User_GetAdcu(p_mc, MOT_ANALOG_CHANNEL_HEAT_MOSFETS);    break;
                // case MOT_VAR_HEAT_MOSFETS_DEG_C:    value = MotorController_User_GetHeatMosfets_DegC(p_mc, 1U);     break;

                case MOT_VAR_ANALOG_THROTTLE:       value = MotAnalogUser_GetThrottle(&p_mc->AnalogUser);           break;
                case MOT_VAR_ANALOG_BRAKE:          value = MotAnalogUser_GetBrake(&p_mc->AnalogUser);              break;
                case MOT_VAR_ANALOG_THROTTLE_DIN:   value = p_mc->AnalogUser.ThrottleAIn.EdgePin.DebouncedState;           break;
                case MOT_VAR_ANALOG_BRAKE_DIN:      value = p_mc->AnalogUser.BrakeAIn.EdgePin.DebouncedState;              break;
                // case MOT_VAR_TX_PACKET_COUNT:   value = Protocol_GetTxPacketCount(p_protocol);      break;
                // case MOT_VAR_RX_PACKET_COUNT:   value = Protocol_GetRxPacketCount(p_protocol);      break;
                default: value = 0U; break;
            }
            break;

        case MOT_VAR_ID_PREFIX_REAL_TIME_CONTROL:
            switch((MotVarId_RealTimeControl_T)varId.Id8)
            {
                /* Read Write */
                case MOT_VAR_MOTOR_USER_CMD:                value = Motor_User_GetCmd(p_motor);                                 break;
                case MOT_VAR_MOTOR_DIRECTION:               value = Motor_User_GetDirection(p_motor);                           break;
                case MOT_VAR_MOTOR_ACTIVE_FEEDBACK_MODE:    value = (Motor_User_GetActiveFeedbackMode(p_motor).State);          break;
                // case MOT_VAR_MOTOR_USER_SPEED_LIMIT:        value = Motor_User_GetUserSpeedLimit(p_motor);                    break;
                // case MOT_VAR_MOTOR_USER_I_LIMIT:            value = Motor_User_GetUserILimit(p_motor);                        break;
                case MOT_VAR_USER_CMD:                      value = MotorController_User_GetCmdValue(p_mc);                     break;
                case MOT_VAR_DIRECTION:                     value = MotorController_User_GetDirection(p_mc);                    break;
                // case MOT_VAR_ACTIVE_FEEDBACK_MODE:   value = (MotorController_User_GetActiveFeedbackMode(p_motor).State);     break;
                // case MOT_VAR_ACTIVE_SPEED_LIMIT:     value = MotorController_User_GetActiveSpeedLimit(p_motor);                        break;
                // case MOT_VAR_ACTIVE_I_LIMIT:         value = MotorController_User_GetActiveILimit(p_motor);                            break;
                case MOT_VAR_MOTOR_CMD_SPEED:       value = 0U;      break;
                case MOT_VAR_MOTOR_CMD_CURRENT:     value = 0U;      break;
                case MOT_VAR_MOTOR_CMD_VOLTAGE:     value = 0U;      break;
                case MOT_VAR_MOTOR_CMD_ANGLE:       value = 0U;      break;

                case MOT_VAR_THROTTLE:              value = 0U;      break;
                case MOT_VAR_BRAKE:                 value = 0U;      break;
                case MOT_VAR_RELEASE_CONTROL:       value = 0U;      break;
                case MOT_VAR_DISABLE_CONTROL:       value = 0U;      break;
                case MOT_VAR_CLEAR_FAULT:           value = 0U;      break;
                case MOT_VAR_SET_FAULT:             value = 0U;      break;
                case MOT_VAR_BEEP:                  value = 0U;      break;
                default: value = 0U; break;
            }
            break;

        /*
            Parameters
        */
        case MOT_VAR_ID_PREFIX_PARAMS_MOTOR:
            switch((MotVarId_ParamsMotor_T)varId.Id8)
            {
                case MOT_VAR_COMMUTATION_MODE:          value = Motor_User_GetCommutationMode(p_motor);         break;
                case MOT_VAR_SENSOR_MODE:               value = Motor_User_GetSensorMode(p_motor);              break;
                case MOT_VAR_DEFAULT_FEEDBACK_MODE:     value = Motor_User_GetDefaultFeedbackMode(p_motor);     break;
                case MOT_VAR_DIRECTION_CALIBRATION:     value = Motor_User_GetDirectionCalibration(p_motor);    break;
                case MOT_VAR_POLE_PAIRS:                value = Motor_User_GetPolePairs(p_motor);               break;
                case MOT_VAR_KV:                        value = Motor_User_GetKv(p_motor);                      break;
                case MOT_VAR_SPEED_FEEDBACK_REF_RPM:    value = Motor_User_GetSpeedFeedbackRef_Rpm(p_motor);    break;
                case MOT_VAR_SPEED_V_REF_RPM:           value = Motor_User_GetSpeedVRef_Rpm(p_motor);           break;

                case MOT_VAR_I_REF_PEAK_ADCU:   value = GLOBAL_MOTOR.I_MAX_ADCU;   break;
                case MOT_VAR_IA_REF_ZERO_ADCU:  value = Motor_User_GetIaZero_Adcu(p_motor); break;
                case MOT_VAR_IB_REF_ZERO_ADCU:  value = Motor_User_GetIbZero_Adcu(p_motor); break;
                case MOT_VAR_IC_REF_ZERO_ADCU:  value = Motor_User_GetIcZero_Adcu(p_motor); break;

                case MOT_VAR_BASE_SPEED_LIMIT_FORWARD: value = Motor_User_GetSpeedLimitForward_ScalarU16(p_motor); break;
                case MOT_VAR_BASE_SPEED_LIMIT_REVERSE: value = Motor_User_GetSpeedLimitReverse_ScalarU16(p_motor); break;
                case MOT_VAR_BASE_I_LIMIT_MOTORING:    value = Motor_User_GetILimitMotoring_ScalarU16(p_motor);    break;
                case MOT_VAR_BASE_I_LIMIT_GENERATING:  value = Motor_User_GetILimitGenerating_ScalarU16(p_motor);  break;

                case MOT_VAR_RAMP_ACCEL_TIME_CYCLES:    value = Motor_User_GetRampAccel_Cycles(p_motor); break;
                // case MOT_VAR_ALIGN_MODE:             value = Motor_User_GetAlignMode(p_motor); break;
                case MOT_VAR_ALIGN_POWER:               value = Motor_User_GetAlignPower_Scalar16(p_motor); break;
                case MOT_VAR_ALIGN_TIME_CYCLES:         value = Motor_User_GetAlignTime_Cycles(p_motor); break;
                case MOT_VAR_OPEN_LOOP_POWER:               value = Motor_User_GetOpenLoopPower_Scalar16(p_motor); break;
                case MOT_VAR_OPEN_LOOP_SPEED:               value = Motor_User_GetOpenLoopSpeed_Scalar16(p_motor); break;
                case MOT_VAR_OPEN_LOOP_ACCEL_TIME_CYCLES:   value = Motor_User_GetOpenLoopAccel_Cycles(p_motor); break;
                // case MOT_VAR_PHASE_PWM_MODE:                    value = Motor_User_GetPhaseModeParam(p_motor); break;

                case MOT_VAR_HALL_SENSOR_TABLE_1: value = p_motor->Hall.Params.SensorsTable[1U];    break;
                case MOT_VAR_HALL_SENSOR_TABLE_2: value = p_motor->Hall.Params.SensorsTable[2U];    break;
                case MOT_VAR_HALL_SENSOR_TABLE_3: value = p_motor->Hall.Params.SensorsTable[3U];    break;
                case MOT_VAR_HALL_SENSOR_TABLE_4: value = p_motor->Hall.Params.SensorsTable[4U];    break;
                case MOT_VAR_HALL_SENSOR_TABLE_5: value = p_motor->Hall.Params.SensorsTable[5U];    break;
                case MOT_VAR_HALL_SENSOR_TABLE_6: value = p_motor->Hall.Params.SensorsTable[6U];    break;

                case MOT_VAR_ENCODER_COUNTS_PER_REVOLUTION:             value = p_motor->Encoder.Params.CountsPerRevolution;            break;
                case MOT_VAR_ENCODER_IS_QUADRATURE_CAPTURE_ENABLED:     value = p_motor->Encoder.Params.IsQuadratureCaptureEnabled;     break;
                case MOT_VAR_ENCODER_IS_A_LEAD_B_POSITIVE:              value = p_motor->Encoder.Params.IsALeadBPositive;               break;
                // case MOT_VAR_ENCODER_EXTENDED_TIMER_DELTA_T_STOP:                            value = 0; break;
                // case MOT_VAR_ENCODER_INTERPOLATE_ANGLE_SCALAR:                               value = 0; break;

                case MOT_VAR_PID_SPEED_KP_FIXED16:     value = PID_GetKp_Fixed16(&p_motor->PidSpeed);   break;
                case MOT_VAR_PID_SPEED_KI_FIXED16:     value = PID_GetKi_Fixed16(&p_motor->PidSpeed);   break;
                // case MOT_VAR_PID_SPEED_KD_FIXED16:  value = PID_GetKd_Fixed16(&p_motor->PidSpeed);   break;
                case MOT_VAR_PID_SPEED_SAMPLE_FREQ:     value = PID_GetSampleFreq(&p_motor->PidSpeed);  break;
                case MOT_VAR_PID_FOC_IQ_KP_FIXED16:     value = PID_GetKp_Fixed16(&p_motor->PidIq);     break;
                case MOT_VAR_PID_FOC_IQ_KI_FIXED16:     value = PID_GetKi_Fixed16(&p_motor->PidIq);     break;
                // case MOT_VAR_PID_FOC_IQ_KD_FIXED16:    value = PID_GetKd_Fixed16(&p_motor->PidIq);  break;
                case MOT_VAR_PID_FOC_IQ_SAMPLE_FREQ:    value = PID_GetSampleFreq(&p_motor->PidIq);     break;
                case MOT_VAR_PID_FOC_ID_KP_FIXED16:     value = PID_GetKp_Fixed16(&p_motor->PidId);     break;
                case MOT_VAR_PID_FOC_ID_KI_FIXED16:     value = PID_GetKi_Fixed16(&p_motor->PidId);     break;
                // case MOT_VAR_PID_FOC_ID_KD_FIXED16:    value = PID_GetKd_Fixed16(&p_motor->PidId);  break;
                case MOT_VAR_PID_FOC_ID_SAMPLE_FREQ:    value = PID_GetSampleFreq(&p_motor->PidId);     break;

                case MOT_VAR_THERMISTOR_MOTOR_LINEAR_T0_ADCU:           value = Thermistor_GetLinearT0_Adcu(&p_motor->Thermistor);          break;
                case MOT_VAR_THERMISTOR_MOTOR_LINEAR_T0_DEG_C:          value = Thermistor_GetLinearT0_DegC(&p_motor->Thermistor);          break;
                case MOT_VAR_THERMISTOR_MOTOR_LINEAR_T1_ADCU:           value = Thermistor_GetLinearT1_Adcu(&p_motor->Thermistor);          break;
                case MOT_VAR_THERMISTOR_MOTOR_LINEAR_T1_DEG_C:          value = Thermistor_GetLinearT1_DegC(&p_motor->Thermistor);          break;
                case MOT_VAR_THERMISTOR_MOTOR_TYPE:                     value = Thermistor_GetType(&p_motor->Thermistor);                   break;
                case MOT_VAR_THERMISTOR_MOTOR_R:                        value = Thermistor_GetR0(&p_motor->Thermistor);                     break;
                case MOT_VAR_THERMISTOR_MOTOR_T:                        value = Thermistor_GetT0(&p_motor->Thermistor);                     break;
                case MOT_VAR_THERMISTOR_MOTOR_B:                        value = Thermistor_GetB(&p_motor->Thermistor);                      break;
                case MOT_VAR_THERMISTOR_MOTOR_FAULT_ADCU:               value = Thermistor_GetFault_Adcu(&p_motor->Thermistor);             break;
                case MOT_VAR_THERMISTOR_MOTOR_FAULT_THRESHOLD_ADCU:     value = Thermistor_GetFaultThreshold_Adcu(&p_motor->Thermistor);    break;
                case MOT_VAR_THERMISTOR_MOTOR_WARNING_ADCU:             value = Thermistor_GetWarning_Adcu(&p_motor->Thermistor);           break;
                case MOT_VAR_THERMISTOR_MOTOR_WARNING_THRESHOLD_ADCU:   value = Thermistor_GetWarningThreshold_Adcu(&p_motor->Thermistor);  break;
                case MOT_VAR_THERMISTOR_MOTOR_IS_MONITOR_ENABLE:        value = Thermistor_GetIsMonitorEnable(&p_motor->Thermistor);        break;
                default: value = 0U; break;
            }
            break;

        case MOT_VAR_ID_PREFIX_PARAMS_GLOBAL:
            switch((MotVarId_ParamsGlobal_T)varId.Id8)
            {
                case MOT_VAR_V_SOURCE_VOLTS:                            value = MotorController_User_GetVSourceRef(p_mc);   break;
                case MOT_VAR_BATTERY_ZERO_ADCU:                         value = (p_mc->Parameters.BatteryZero_Adcu);        break;
                case MOT_VAR_BATTERY_FULL_ADCU:                         value = (p_mc->Parameters.BatteryFull_Adcu);        break;
                case MOT_VAR_USER_INPUT_MODE:                           value = MotorController_User_GetInputMode(p_mc);    break;
                case MOT_VAR_BRAKE_MODE:                                value = (p_mc->Parameters.BrakeMode);               break;
                case MOT_VAR_ZERO_CMD_MODE:                             value = (p_mc->Parameters.ZeroCmdMode);             break;
                    // case MOT_VAR_BUZZER_FLAGS_ENABLE:                       value = (p_mc->Parameters.Buzzer); break;
                case MOT_VAR_OPT_DIN_FUNCTION:                          value = (p_mc->Parameters.OptDinFunction);              break;
                case MOT_VAR_OPT_DIN_SPEED_LIMIT:                       value = (p_mc->Parameters.OptDinSpeedLimit_Scalar16);   break;
                case MOT_VAR_I_LIMIT_LOW_V:                             value = (p_mc->Parameters.ILimitLowV_Scalar16);         break;
                    // case MOT_VAR_CAN_SERVICES_ID:                           value = (p_mc->Parameters.CanServicesId); break;
                    // case MOT_VAR_CAN_IS_ENABLE:                             value = (p_mc->Parameters.CanIsEnable); break;

                case MOT_VAR_VMONITOR_SOURCE_LIMIT_UPPER_ADCU:          value = VMonitor_GetFaultUpper(&p_mc->VMonitorSource);      break;
                case MOT_VAR_VMONITOR_SOURCE_LIMIT_LOWER_ADCU:          value = VMonitor_GetFaultLower(&p_mc->VMonitorSource);      break;
                case MOT_VAR_VMONITOR_SOURCE_WARNING_UPPER_ADCU:        value = VMonitor_GetWarningUpper(&p_mc->VMonitorSource);    break;
                case MOT_VAR_VMONITOR_SOURCE_WARNING_LOWER_ADCU:        value = VMonitor_GetWarningLower(&p_mc->VMonitorSource);    break;
                case MOT_VAR_VMONITOR_SOURCE_IS_ENABLE:                 value = VMonitor_GetIsEnable(&p_mc->VMonitorSource);        break;
                case MOT_VAR_VMONITOR_SENSE_LIMIT_UPPER_ADCU:           value = VMonitor_GetFaultUpper(&p_mc->VMonitorSense);      break;
                case MOT_VAR_VMONITOR_SENSE_LIMIT_LOWER_ADCU:           value = VMonitor_GetFaultLower(&p_mc->VMonitorSense);      break;
                case MOT_VAR_VMONITOR_SENSE_WARNING_UPPER_ADCU:         value = VMonitor_GetWarningUpper(&p_mc->VMonitorSense);    break;
                case MOT_VAR_VMONITOR_SENSE_WARNING_LOWER_ADCU:         value = VMonitor_GetWarningLower(&p_mc->VMonitorSense);    break;
                case MOT_VAR_VMONITOR_SENSE_IS_ENABLE:                  value = VMonitor_GetIsEnable(&p_mc->VMonitorSense);        break;
                case MOT_VAR_VMONITOR_ACC_LIMIT_UPPER_ADCU:             value = VMonitor_GetFaultUpper(&p_mc->VMonitorAcc);      break;
                case MOT_VAR_VMONITOR_ACC_LIMIT_LOWER_ADCU:             value = VMonitor_GetFaultLower(&p_mc->VMonitorAcc);      break;
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
                case MOT_VAR_THERMISTOR_PCB_FAULT_ADCU:                 value = Thermistor_GetFault_Adcu(&p_mc->ThermistorPcb);          break;
                case MOT_VAR_THERMISTOR_PCB_FAULT_THRESHOLD_ADCU:       value = Thermistor_GetFaultThreshold_Adcu(&p_mc->ThermistorPcb); break;
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
                case MOT_VAR_THERMISTOR_MOSFETS_FAULT_ADCU:             value = Thermistor_GetFault_Adcu(&p_mc->ThermistorMosfets);          break;
                case MOT_VAR_THERMISTOR_MOSFETS_FAULT_THRESHOLD_ADCU:   value = Thermistor_GetFaultThreshold_Adcu(&p_mc->ThermistorMosfets); break;
                case MOT_VAR_THERMISTOR_MOSFETS_WARNING_ADCU:           value = Thermistor_GetWarning_Adcu(&p_mc->ThermistorMosfets);           break;
                case MOT_VAR_THERMISTOR_MOSFETS_WARNING_THRESHOLD_ADCU: value = Thermistor_GetWarningThreshold_Adcu(&p_mc->ThermistorMosfets);  break;
                case MOT_VAR_THERMISTOR_MOSFETS_IS_MONITOR_ENABLE:      value = Thermistor_GetIsMonitorEnable(&p_mc->ThermistorMosfets);        break;

                case MOT_VAR_ANALOG_THROTTLE_ZERO_ADCU:                 value = p_mc->AnalogUser.Params.ThrottleZero_Adcu; break;
                case MOT_VAR_ANALOG_THROTTLE_MAX_ADCU:                  value = p_mc->AnalogUser.Params.ThrottleMax_Adcu; break;
                case MOT_VAR_ANALOG_THROTTLE_EDGE_PIN_IS_ENABLE:        value = p_mc->AnalogUser.Params.UseThrottleEdgePin; break;
                case MOT_VAR_ANALOG_BRAKE_ZERO_ADCU:                    value = p_mc->AnalogUser.Params.BrakeZero_Adcu; break;
                case MOT_VAR_ANALOG_BRAKE_MAX_ADCU:                     value = p_mc->AnalogUser.Params.BrakeMax_Adcu; break;
                case MOT_VAR_ANALOG_BRAKE_EDGE_PIN_IS_ENABLE:           value = p_mc->AnalogUser.Params.UseBrakeEdgePin; break;
                case MOT_VAR_ANALOG_DIN_BRAKE_VALUE:                    value = p_mc->AnalogUser.Params.BistateBrakeValue_Scalar16; break;
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

                case MOT_VAR_I_MAX_AMP:                                 value = MotorController_User_GetIMax(p_mc);         break;
                case MOT_VAR_V_MAX_VOLTS:                               value = MotorController_User_GetVMax(p_mc);         break;
                // case MOT_VAR_VERSION_KMC_REG32:     value = MotorController_User_GetMainVersion(p_mc);  break;
                default: value = 0U; break;
            }
            break;
    }

    return value;
}


/*!
    Limitation: Cannot set Brake and Throttle in the same packet
*/
MotVar_Status_T MotorController_Var_Set(MotorController_T * p_mc, MotVarId_T varId, int32_t varValue)
{
    Motor_T * p_motor = MotorController_GetPtrMotor(p_mc, 0U);
    volatile
    MotVarId_Arg_T arg = (MotVarId_Arg_T)varId.Arg;
    MotVar_Status_T status = MOT_VAR_STATUS_OK;

    switch((MotVarId_Prefix_T)arg.Prefix)
    {
        case MOT_VAR_ID_PREFIX_REAL_TIME_MONITOR: break;
        /*
            Cmd Vars - use input mode constraint
        */
        case MOT_VAR_ID_PREFIX_REAL_TIME_CONTROL:
            if(p_mc->Parameters.UserInputMode == MOTOR_CONTROLLER_INPUT_MODE_PROTOCOL)
            {
                switch((MotVarId_RealTimeControl_T)varId.Id8)
                {
                    /* Read/Write Cmds */
                    case MOT_VAR_MOTOR_USER_CMD:            Motor_User_SetDefaultModeCmd(p_motor, (uint16_t)varValue);              break;
                    case MOT_VAR_MOTOR_DIRECTION:           Motor_User_SetDirection(p_motor, (Motor_Direction_T)varValue);          break;
                    // case MOT_VAR_MOTOR_ACTIVE_FEEDBACK_MODE:    Motor_User_ActivateFeedbackMode(p_motor, (Motor_FeedbackModeId_T)varValue);                  break;

                    case MOT_VAR_MOTOR_USER_SPEED_LIMIT:    Motor_User_SetSpeedLimitActive(p_motor, (uint16_t)varValue);                                 break;
                    case MOT_VAR_MOTOR_USER_I_LIMIT:        Motor_User_SetILimitActive(p_motor, (uint16_t)varValue, MOTOR_I_LIMIT_ACTIVE_USER);          break;

                    case MOT_VAR_USER_CMD:                  MotorController_User_SetCmdValue(p_mc, (uint16_t)varValue);                              break;
                    case MOT_VAR_DIRECTION:                 MotorController_User_SetDirection(p_mc, (MotorController_Direction_T)varValue);          break;

                    /* Write-Only Cmds */
                    case MOT_VAR_MOTOR_CMD_SPEED:       Motor_User_SetSpeedCmdValue(p_motor, (uint16_t)varValue);                           break;
                    case MOT_VAR_MOTOR_CMD_CURRENT:     Motor_User_SetTorqueCmdValue(p_motor, (uint16_t)varValue);                          break;
                    case MOT_VAR_MOTOR_CMD_VOLTAGE:     Motor_User_SetVoltageCmdValue(p_motor, (uint16_t)varValue);                         break;
                    case MOT_VAR_MOTOR_CMD_ANGLE:       Motor_User_SetPositionCmdValue(p_motor, (uint16_t)varValue);                        break;
                        // case MOT_VAR_MOTOR_CMD_OPEN_LOOP:   Motor_User_SetOpenLoopCmdValue(p_motor, (uint16_t)varValue);                      break;

                    case MOT_VAR_THROTTLE:              MotorController_User_SetCmdThrottle(p_mc, (uint16_t)varValue);                       break;
                    case MOT_VAR_BRAKE:                 MotorController_User_SetCmdBrake(p_mc, (uint16_t)varValue);                          break;
                    case MOT_VAR_BEEP:                  MotorController_User_BeepN(p_mc, 500U, 500U, varValue);                              break;
                        // MOT_VAR_MOTOR_RELEASE_CONTROL,
                        // MOT_VAR_MOTOR_DISABLE,
                        // MOT_VAR_MOTOR_CLEAR_FAULT,
                        // MOT_VAR_MOTOR_SET_FAULT,
                    default: break;
                }
            }
            break;
        /*
            Parameters
        */
        case MOT_VAR_ID_PREFIX_PARAMS_MOTOR:
            switch((MotVarId_ParamsMotor_T)varId.Id8)
            {
                // case MOT_VAR_COMMUTATION_MODE:          Motor_User_SetCommutationMode(p_motor);         break;
                // case MOT_VAR_SENSOR_MODE:               Motor_User_SetSensorMode(p_motor);              break;
                // case MOT_VAR_DEFAULT_FEEDBACK_MODE:     Motor_User_SetDefaultFeedbackMode(p_motor, (Motor_FeedbackModeId_T)varValue);       break;
                // case MOT_VAR_DIRECTION_CALIBRATION:     Motor_User_SetDirectionCalibration(p_motor);    break;

                // case MOT_VAR_POLE_PAIRS:                Motor_User_SetPolePairs(p_motor, varValue);     break;
                // case MOT_VAR_KV:                        Motor_User_SetKv(p_motor);                      break;
                case MOT_VAR_SPEED_FEEDBACK_REF_RPM:    Motor_User_SetSpeedFeedbackRef_Rpm(p_motor, varValue);    break;
                // case MOT_VAR_SPEED_V_REF_RPM:           Motor_User_SetSpeedVRef_Rpm(p_motor);           break;

                // case MOT_VAR_I_REF_PEAK_ADCU:   GLOBAL_MOTOR.I_MAX_ADCU;   break;
                // case MOT_VAR_IA_REF_ZERO_ADCU:  Motor_User_SetIaZero_Adcu(p_motor); break;
                // case MOT_VAR_IB_REF_ZERO_ADCU:  Motor_User_SetIbZero_Adcu(p_motor); break;
                // case MOT_VAR_IC_REF_ZERO_ADCU:  Motor_User_SetIcZero_Adcu(p_motor); break;

                // case MOT_VAR_BASE_SPEED_LIMIT_FORWARD: Motor_User_SetSpeedLimitForward_ScalarU16(p_motor); break;
                // case MOT_VAR_BASE_SPEED_LIMIT_REVERSE: Motor_User_SetSpeedLimitReverse_ScalarU16(p_motor); break;
                // case MOT_VAR_BASE_I_LIMIT_MOTORING:    Motor_User_SetILimitMotoring_ScalarU16(p_motor);    break;
                // case MOT_VAR_BASE_I_LIMIT_GENERATING:  Motor_User_SetILimitGenerating_ScalarU16(p_motor);  break;

                // case MOT_VAR_RAMP_ACCEL_TIME_CYCLES:            Motor_User_SetRampAccel_Cycles(p_motor); break;
                //     // case MOT_VAR_ALIGN_MODE:                        Motor_User_SetAlignMode(p_motor); break;
                // case MOT_VAR_ALIGN_POWER:              Motor_User_SetAlignPower_Scalar16(p_motor); break;
                // case MOT_VAR_ALIGN_TIME_CYCLES:                 Motor_User_SetAlignTime_Cycles(p_motor); break;
                // case MOT_VAR_OPEN_LOOP_POWER:          Motor_User_SetOpenLoopPower_Scalar16(p_motor); break;
                // case MOT_VAR_OPEN_LOOP_SPEED:          Motor_User_SetOpenLoopSpeed_Scalar16(p_motor); break;
                // case MOT_VAR_OPEN_LOOP_ACCEL_TIME_CYCLES:       Motor_User_SetOpenLoopAccel_Cycles(p_motor); break;
                //     // case MOT_VAR_PHASE_PWM_MODE:                    Motor_User_SetPhaseModeParam(p_motor); break;

                // case MOT_VAR_HALL_SENSOR_TABLE_1: p_motor->Hall.Params.SensorsTable[1U];    break;
                // case MOT_VAR_HALL_SENSOR_TABLE_2: p_motor->Hall.Params.SensorsTable[2U];    break;
                // case MOT_VAR_HALL_SENSOR_TABLE_3: p_motor->Hall.Params.SensorsTable[3U];    break;
                // case MOT_VAR_HALL_SENSOR_TABLE_4: p_motor->Hall.Params.SensorsTable[4U];    break;
                // case MOT_VAR_HALL_SENSOR_TABLE_5: p_motor->Hall.Params.SensorsTable[5U];    break;
                // case MOT_VAR_HALL_SENSOR_TABLE_6: p_motor->Hall.Params.SensorsTable[6U];    break;

                // case MOT_VAR_ENCODER_COUNTS_PER_REVOLUTION:     p_motor->Encoder.Params.CountsPerRevolution;            break;
                // case MOT_VAR_ENCODER_IS_QUADRATURE_CAPTURE_ENABLED:     p_motor->Encoder.Params.IsQuadratureCaptureEnabled;     break;
                // case MOT_VAR_ENCODER_IS_A_LEAD_B_POSITIVE:              p_motor->Encoder.Params.IsALeadBPositive;               break;
                //     // case MOT_VAR_ENCODER_EXTENDED_TIMER_DELTA_T_STOP:                            0; break;
                //     // case MOT_VAR_ENCODER_INTERPOLATE_ANGLE_SCALAR:                               0; break;

                // case MOT_VAR_PID_SPEED_KP_FIXED16:     PID_SetKp_Fixed16(&p_motor->PidSpeed);   break;
                // case MOT_VAR_PID_SPEED_KI_FIXED16:     PID_SetKi_Fixed16(&p_motor->PidSpeed);   break;
                //     // case MOT_VAR_PID_SPEED_KD_FIXED16:  PID_SetKd_Fixed16(&p_motor->PidSpeed);   break;
                // case MOT_VAR_PID_SPEED_SAMPLE_FREQ:     PID_SetSampleFreq(&p_motor->PidSpeed);  break;
                // case MOT_VAR_PID_FOC_IQ_KP_FIXED16:    PID_SetKp_Fixed16(&p_motor->PidIq);     break;
                // case MOT_VAR_PID_FOC_IQ_KI_FIXED16:    PID_SetKi_Fixed16(&p_motor->PidIq);     break;
                //     // case MOT_VAR_PID_FOC_IQ_KD_FIXED16:    PID_SetKd_Fixed16(&p_motor->PidIq);  break;
                // case MOT_VAR_PID_FOC_IQ_SAMPLE_FREQ:    PID_SetSampleFreq(&p_motor->PidIq);     break;
                // case MOT_VAR_PID_FOC_ID_KP_FIXED16:    PID_SetKp_Fixed16(&p_motor->PidId);     break;
                // case MOT_VAR_PID_FOC_ID_KI_FIXED16:    PID_SetKi_Fixed16(&p_motor->PidId);     break;
                //     // case MOT_VAR_PID_FOC_ID_KD_FIXED16:    PID_SetKd_Fixed16(&p_motor->PidId);  break;
                // case MOT_VAR_PID_FOC_ID_SAMPLE_FREQ:    PID_SetSampleFreq(&p_motor->PidId);     break;

                // case MOT_VAR_THERMISTOR_MOTOR_LINEAR_T0_ADCU:           Thermistor_SetLinearT0_Adcu(&p_motor->Thermistor);          break;
                // case MOT_VAR_THERMISTOR_MOTOR_LINEAR_T0_DEG_C:          Thermistor_SetLinearT0_DegC(&p_motor->Thermistor);          break;
                // case MOT_VAR_THERMISTOR_MOTOR_LINEAR_T1_ADCU:           Thermistor_SetLinearT1_Adcu(&p_motor->Thermistor);          break;
                // case MOT_VAR_THERMISTOR_MOTOR_LINEAR_T1_DEG_C:          Thermistor_SetLinearT1_DegC(&p_motor->Thermistor);          break;
                // case MOT_VAR_THERMISTOR_MOTOR_TYPE:                     Thermistor_SetType(&p_motor->Thermistor);          break;
                // case MOT_VAR_THERMISTOR_MOTOR_R:                        Thermistor_SetR0(&p_motor->Thermistor);                     break;
                // case MOT_VAR_THERMISTOR_MOTOR_T:                        Thermistor_SetT0(&p_motor->Thermistor);                     break;
                // case MOT_VAR_THERMISTOR_MOTOR_B:                        Thermistor_SetB(&p_motor->Thermistor);                      break;
                // case MOT_VAR_THERMISTOR_MOTOR_FAULT_ADCU:               Thermistor_SetFault_Adcu(&p_motor->Thermistor);          break;
                // case MOT_VAR_THERMISTOR_MOTOR_FAULT_THRESHOLD_ADCU:     Thermistor_SetFaultThreshold_Adcu(&p_motor->Thermistor); break;
                // case MOT_VAR_THERMISTOR_MOTOR_WARNING_ADCU:             Thermistor_SetWarning_Adcu(&p_motor->Thermistor);           break;
                // case MOT_VAR_THERMISTOR_MOTOR_WARNING_THRESHOLD_ADCU:   Thermistor_SetWarningThreshold_Adcu(&p_motor->Thermistor);  break;
                // case MOT_VAR_THERMISTOR_MOTOR_IS_MONITOR_ENABLE:        Thermistor_SetIsMonitorEnable(&p_motor->Thermistor);        break;
                default: break;
            }
            break;

        case MOT_VAR_ID_PREFIX_PARAMS_GLOBAL:
            switch((MotVarId_ParamsGlobal_T)varId.Id8)
            {
                case MOT_VAR_V_SOURCE_VOLTS:                            MotorController_User_SetVSourceRef(p_mc, varValue);   break;
                // case MOT_VAR_BATTERY_ZERO_ADCU:                         (p_mc->Parameters.BatteryZero_Adcu);        break;
                // case MOT_VAR_BATTERY_FULL_ADCU:                         (p_mc->Parameters.BatteryFull_Adcu);        break;
                // case MOT_VAR_USER_INPUT_MODE:                           MotorController_User_SetInputMode_Blocking(p_mc, (MotorController_InputMode_T)varValue);    2U;break;
                // case MOT_VAR_BRAKE_MODE:                                (p_mc->Parameters.BrakeMode);               break;
                // case MOT_VAR_ZERO_CMD_MODE:                             (p_mc->Parameters.ZeroCmdMode);             break;
                //     // case MOT_VAR_BUZZER_FLAGS_ENABLE:                       (p_mc->Parameters.Buzzer); break;
                // case MOT_VAR_OPT_DIN_FUNCTION:                          (p_mc->Parameters.OptDinFunction);              break;
                // case MOT_VAR_OPT_DIN_SPEED_LIMIT:              (p_mc->Parameters.OptDinSpeedLimit_Scalar16);   break;
                // case MOT_VAR_I_LIMIT_LOW_V:                    (p_mc->Parameters.ILimitLowV_Scalar16);         break;
                //     // case MOT_VAR_CAN_SERVICES_ID:                           (p_mc->Parameters.CanServicesId); break;
                //     // case MOT_VAR_CAN_IS_ENABLE:                             (p_mc->Parameters.CanIsEnable); break;

                // case MOT_VAR_VMONITOR_SOURCE_LIMIT_UPPER_ADCU:          VMonitor_SetFaultUpper(&p_mc->VMonitorSource);      break;
                // case MOT_VAR_VMONITOR_SOURCE_LIMIT_LOWER_ADCU:          VMonitor_SetFaultLower(&p_mc->VMonitorSource);      break;
                // case MOT_VAR_VMONITOR_SOURCE_WARNING_UPPER_ADCU:        VMonitor_SetWarningUpper(&p_mc->VMonitorSource);    break;
                // case MOT_VAR_VMONITOR_SOURCE_WARNING_LOWER_ADCU:        VMonitor_SetWarningLower(&p_mc->VMonitorSource);    break;
                // case MOT_VAR_VMONITOR_SOURCE_IS_ENABLE:                 VMonitor_SetIsEnable(&p_mc->VMonitorSource);        break;
                // case MOT_VAR_VMONITOR_SENSE_LIMIT_UPPER_ADCU:           VMonitor_SetFaultUpper(&p_mc->VMonitorSense);      break;
                // case MOT_VAR_VMONITOR_SENSE_LIMIT_LOWER_ADCU:           VMonitor_SetFaultLower(&p_mc->VMonitorSense);      break;
                // case MOT_VAR_VMONITOR_SENSE_WARNING_UPPER_ADCU:         VMonitor_SetWarningUpper(&p_mc->VMonitorSense);    break;
                // case MOT_VAR_VMONITOR_SENSE_WARNING_LOWER_ADCU:         VMonitor_SetWarningLower(&p_mc->VMonitorSense);    break;
                // case MOT_VAR_VMONITOR_SENSE_IS_ENABLE:                  VMonitor_SetIsEnable(&p_mc->VMonitorSense);        break;
                // case MOT_VAR_VMONITOR_ACC_LIMIT_UPPER_ADCU:             VMonitor_SetFaultUpper(&p_mc->VMonitorAcc);      break;
                // case MOT_VAR_VMONITOR_ACC_LIMIT_LOWER_ADCU:             VMonitor_SetFaultLower(&p_mc->VMonitorAcc);      break;
                // case MOT_VAR_VMONITOR_ACC_WARNING_UPPER_ADCU:           VMonitor_SetWarningUpper(&p_mc->VMonitorAcc);    break;
                // case MOT_VAR_VMONITOR_ACC_WARNING_LOWER_ADCU:           VMonitor_SetWarningLower(&p_mc->VMonitorAcc);    break;
                // case MOT_VAR_VMONITOR_ACC_IS_ENABLE:                    VMonitor_SetIsEnable(&p_mc->VMonitorAcc);        break;
                // case MOT_VAR_THERMISTOR_PCB_TYPE:                       Thermistor_SetLinearT0_Adcu(&p_mc->ThermistorPcb);          break;
                // case MOT_VAR_THERMISTOR_PCB_R:                          Thermistor_SetLinearT0_DegC(&p_mc->ThermistorPcb);          break;
                // case MOT_VAR_THERMISTOR_PCB_T:                          Thermistor_SetLinearT1_Adcu(&p_mc->ThermistorPcb);          break;
                // case MOT_VAR_THERMISTOR_PCB_B:                          Thermistor_SetLinearT1_DegC(&p_mc->ThermistorPcb);          break;
                // case MOT_VAR_THERMISTOR_PCB_LINEAR_T0_ADCU:             Thermistor_SetType(&p_mc->ThermistorPcb);          break;
                // case MOT_VAR_THERMISTOR_PCB_LINEAR_T0_DEG_C:            Thermistor_SetR0(&p_mc->ThermistorPcb);                     break;
                // case MOT_VAR_THERMISTOR_PCB_LINEAR_T1_ADCU:             Thermistor_SetT0(&p_mc->ThermistorPcb);                     break;
                // case MOT_VAR_THERMISTOR_PCB_LINEAR_T1_DEG_C:            Thermistor_SetB(&p_mc->ThermistorPcb);                      break;
                // case MOT_VAR_THERMISTOR_PCB_FAULT_ADCU:                 Thermistor_SetFault_Adcu(&p_mc->ThermistorPcb);          break;
                // case MOT_VAR_THERMISTOR_PCB_FAULT_THRESHOLD_ADCU:       Thermistor_SetFaultThreshold_Adcu(&p_mc->ThermistorPcb); break;
                // case MOT_VAR_THERMISTOR_PCB_WARNING_ADCU:               Thermistor_SetWarning_Adcu(&p_mc->ThermistorPcb);           break;
                // case MOT_VAR_THERMISTOR_PCB_WARNING_THRESHOLD_ADCU:     Thermistor_SetWarningThreshold_Adcu(&p_mc->ThermistorPcb);  break;
                // case MOT_VAR_THERMISTOR_PCB_IS_MONITOR_ENABLE:          Thermistor_SetIsMonitorEnable(&p_mc->ThermistorPcb);        break;
                // case MOT_VAR_THERMISTOR_MOSFETS_TYPE:                   Thermistor_SetLinearT0_Adcu(&p_mc->ThermistorMosfets);          break;
                // case MOT_VAR_THERMISTOR_MOSFETS_R:                      Thermistor_SetLinearT0_DegC(&p_mc->ThermistorMosfets);          break;
                // case MOT_VAR_THERMISTOR_MOSFETS_T:                      Thermistor_SetLinearT1_Adcu(&p_mc->ThermistorMosfets);          break;
                // case MOT_VAR_THERMISTOR_MOSFETS_B:                      Thermistor_SetLinearT1_DegC(&p_mc->ThermistorMosfets);          break;
                // case MOT_VAR_THERMISTOR_MOSFETS_LINEAR_T0_ADCU:         Thermistor_SetType(&p_mc->ThermistorMosfets);          break;
                // case MOT_VAR_THERMISTOR_MOSFETS_LINEAR_T0_DEG_C:        Thermistor_SetR0(&p_mc->ThermistorMosfets);                     break;
                // case MOT_VAR_THERMISTOR_MOSFETS_LINEAR_T1_ADCU:         Thermistor_SetT0(&p_mc->ThermistorMosfets);                     break;
                // case MOT_VAR_THERMISTOR_MOSFETS_LINEAR_T1_DEG_C:        Thermistor_SetB(&p_mc->ThermistorMosfets);                      break;
                // case MOT_VAR_THERMISTOR_MOSFETS_FAULT_ADCU:             Thermistor_SetFault_Adcu(&p_mc->ThermistorMosfets);          break;
                // case MOT_VAR_THERMISTOR_MOSFETS_FAULT_THRESHOLD_ADCU:   Thermistor_SetFaultThreshold_Adcu(&p_mc->ThermistorMosfets); break;
                // case MOT_VAR_THERMISTOR_MOSFETS_WARNING_ADCU:           Thermistor_SetWarning_Adcu(&p_mc->ThermistorMosfets);           break;
                // case MOT_VAR_THERMISTOR_MOSFETS_WARNING_THRESHOLD_ADCU: Thermistor_SetWarningThreshold_Adcu(&p_mc->ThermistorMosfets);  break;
                // case MOT_VAR_THERMISTOR_MOSFETS_IS_MONITOR_ENABLE:      Thermistor_SetIsMonitorEnable(&p_mc->ThermistorMosfets);        break;

                // case MOT_VAR_ANALOG_THROTTLE_ZERO_ADCU:                 p_mc->AnalogUser.Params.ThrottleZero_Adcu; break;
                // case MOT_VAR_ANALOG_THROTTLE_MAX_ADCU:                  p_mc->AnalogUser.Params.ThrottleMax_Adcu; break;
                // case MOT_VAR_ANALOG_THROTTLE_EDGE_PIN_IS_ENABLE:        p_mc->AnalogUser.Params.UseThrottleEdgePin; break;
                // case MOT_VAR_ANALOG_BRAKE_ZERO_ADCU:                    p_mc->AnalogUser.Params.BrakeZero_Adcu; break;
                // case MOT_VAR_ANALOG_BRAKE_MAX_ADCU:                     p_mc->AnalogUser.Params.BrakeMax_Adcu; break;
                // case MOT_VAR_ANALOG_BRAKE_EDGE_PIN_IS_ENABLE:           p_mc->AnalogUser.Params.UseBrakeEdgePin; break;
                // case MOT_VAR_ANALOG_DIN_BRAKE_VALUE:           p_mc->AnalogUser.Params.BistateBrakeValue_Scalar16; break;
                // case MOT_VAR_ANALOG_DIN_BRAKE_IS_ENABLE:                p_mc->AnalogUser.Params.UseBistateBrakePin; break;
                //     // case MOT_VAR_ANALOG_DIRECTION_PINS:                  p_mc->AnalogUser.Params.; break;

                // case MOT_VAR_PROTOCOL0_XCVR_ID:                         MotorController_User_SetPtrProtocol(p_mc, 0)->Params.SpecsId; break;
                case MOT_VAR_PROTOCOL0_SPECS_ID:                        0; break;
                case MOT_VAR_PROTOCOL0_WATCHDOG_TIME:                   0; break;
                case MOT_VAR_PROTOCOL0_BAUD_RATE:                       0; break;
                case MOT_VAR_PROTOCOL0_IS_ENABLED:                      0; break;
                case MOT_VAR_PROTOCOL1_XCVR_ID:                         0; break;
                case MOT_VAR_PROTOCOL1_SPECS_ID:                        0; break;
                case MOT_VAR_PROTOCOL1_WATCHDOG_TIME:                   0; break;
                case MOT_VAR_PROTOCOL1_BAUD_RATE:                       0; break;
                case MOT_VAR_PROTOCOL1_IS_ENABLED:                      0; break;
                case MOT_VAR_PROTOCOL2_XCVR_ID:                         0; break;
                case MOT_VAR_PROTOCOL2_SPECS_ID:                        0; break;
                case MOT_VAR_PROTOCOL2_WATCHDOG_TIME:                   0; break;
                case MOT_VAR_PROTOCOL2_BAUD_RATE:                       0; break;
                case MOT_VAR_PROTOCOL2_IS_ENABLED:                      0; break;

                default: break;
            }
            break;
        default: break;
    }

    return status;
}


// uint8_t MotorController_Var_GetSize(MotorController_T * p_mc, uint16_t varId)
// {

// }