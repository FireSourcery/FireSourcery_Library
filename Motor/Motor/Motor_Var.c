

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
    }
    return value;
}

int32_t Motor_VarOutput_PositionSensor_Get(const Motor_T * p_motor, Motor_VarOutput_PositionSensor_T varId)
{
    int32_t value = 0;
    switch (varId)
    {
        case MOTOR_VAR_ENCODER_FREQ:  value = p_motor->Encoder.FreqD;     break;
    }
    return value;
}

int32_t Motor_VarIO_Get(const Motor_T * p_motor, Motor_VarIO_T varId)
{
    int32_t value = 0;
    switch (varId)
    {
        case MOTOR_VAR_DIRECTION:           value = Motor_User_GetDirection(p_motor);             break;
        case MOTOR_VAR_USER_SET_POINT:      value = Motor_User_GetSetPoint(p_motor);              break;
        case MOTOR_VAR_USER_FEEDBACK_MODE:  value = Motor_User_GetFeedbackMode(p_motor).Value;    break;
        case MOTOR_VAR_USER_SPEED_LIMIT:    value = Motor_User_GetSpeedLimit(p_motor);            break;
        case MOTOR_VAR_USER_I_LIMIT:        value = Motor_User_GetILimit(p_motor);                break;
        default: break;
    }
    return value;
}

void Motor_VarInput_Set(Motor_T * p_motor, Motor_VarInput_T varId, int32_t varValue)
{
    switch (varId)
    {
        // case MOTOR_VAR_USER_CMD:        Motor_User_SetActiveCmdValue(p_motor, varValue);    break;
        case MOTOR_VAR_CMD_SPEED:       Motor_User_SetSpeedCmd(p_motor, varValue);     break;
        case MOTOR_VAR_CMD_CURRENT:     Motor_User_SetICmd(p_motor, varValue);         break;
        case MOTOR_VAR_CMD_VOLTAGE:     Motor_User_SetVoltageCmd(p_motor, varValue);   break;
        case MOTOR_VAR_CMD_ANGLE:       Motor_User_SetPositionCmd(p_motor, varValue);  break;
        case MOTOR_VAR_CMD_OPEN_LOOP:   Motor_User_SetOpenLoopCmd(p_motor, varValue);  break;
        case MOTOR_VAR_FORCE_DISABLE_CONTROL:           break;
        case MOTOR_VAR_TRY_RELEASE:                     break;
        case MOTOR_VAR_TRY_HOLD:                        break;
        case MOTOR_VAR_CLEAR_FAULT:                     break;
    }
}

void Motor_VarIO_Set(Motor_T * p_motor, Motor_VarIO_T varId, int32_t varValue)
{
    switch (varId)
    {
        case MOTOR_VAR_DIRECTION:               Motor_User_SetDirection(p_motor, (Motor_Direction_T)varValue);  break; //use async polling for status for now
        case MOTOR_VAR_USER_SET_POINT:          Motor_User_SetActiveCmdValue(p_motor, varValue);                break;
        // case MOTOR_VAR_USER_FEEDBACK_MODE:      Motor_User_SetFeedbackMode_Cast(p_motor, (uint8_t)varValue);    break;
        case MOTOR_VAR_USER_SPEED_LIMIT:        Motor_User_TrySpeedLimit(p_motor, varValue);                    break;
        case MOTOR_VAR_USER_I_LIMIT:            Motor_User_TryILimit(p_motor, varValue);                        break;
    //     case MOTOR_VAR_USER_SPEED_LIMIT:        isSuccess = Motor_User_TrySpeedLimit(p_motor, varValue);        break;
    //     case MOTOR_VAR_USER_I_LIMIT:            isSuccess = Motor_User_TryILimit(p_motor, varValue);            break;
        default: break;
    }
}


