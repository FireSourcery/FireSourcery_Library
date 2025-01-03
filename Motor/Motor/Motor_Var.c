

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

int32_t Motor_VarOutput_Get(const Motor_T * p_motor, MotVarId_Monitor_Motor_T varId)
{
    int32_t value = 0;
    switch (varId)
    {
        case MOT_VAR_SPEED:                             value = Motor_User_GetSpeed_UFract16(p_motor);              break;
        case MOT_VAR_I_PHASE:                           value = Motor_User_GetIPhase_UFract16(p_motor);             break;
        case MOT_VAR_V_PHASE:                           value = Motor_User_GetVPhase_UFract16(p_motor);             break;
        case MOT_VAR_POWER:                             value = Motor_User_GetElectricalPower_UFract16(p_motor);    break;
        case MOT_VAR_MOTOR_STATE:                       value = Motor_User_GetStateId(p_motor);                     break;
        case MOT_VAR_MOTOR_STATE_FLAGS:                 value = Motor_User_GetStateFlags(p_motor).Value;             break;
        // case MOT_VAR_MOTOR_STATUS_FLAGS:    value = Motor_User_GetStateFlags(p_motor).Word;       break;
        case MOT_VAR_MOTOR_FAULT_FLAGS:                 value = Motor_User_GetFaultFlags(p_motor).Value;            break;
        case MOT_VAR_MOTOR_HEAT:                        value = Motor_User_GetHeat_Adcu(p_motor);                   break;
        case MOT_VAR_MOTOR_EFFECTIVE_FEEDBACK_MODE:     value = Motor_User_GetFeedbackMode(p_motor).Value;          break;
        case MOT_VAR_MOTOR_EFFECTIVE_SET_POINT:         value = Motor_User_GetSetPoint(p_motor);                    break;
        case MOT_VAR_MOTOR_EFFECTIVE_SPEED_LIMIT:       value = Motor_User_GetSpeedLimit(p_motor);                  break;
        case MOT_VAR_MOTOR_EFFECTIVE_I_LIMIT:           value = Motor_User_GetILimit(p_motor);                      break;
        case MOT_VAR_MOTOR_V_SPEED_DEBUG:               value = Motor_User_GetVSpeedDebug_UFract16(p_motor);        break;
        case MOT_VAR_MOTOR_V_SPEED_EFFECTIVE:           value = Motor_User_GetVSpeedEffective_UFract16(p_motor);    break;
    }
    return value;
}

int32_t Motor_VarOutput_Foc_Get(const Motor_T * p_motor, MotVarId_Monitor_MotorFoc_T varId)
{
    int32_t value = 0;
    switch (varId)
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
    }
    return value;
}


// case MOT_VAR_ID_TYPE_MONITOR_MOTOR_SENSOR:
//     switch ((MotVarId_Monitor_MotorSensor_T)varId.NameBase)
//     {
//         case MOT_VAR_ENCODER_FREQ:  value = p_motor->Encoder.FreqD;     break;
//         default: break;
//     }
//     break;