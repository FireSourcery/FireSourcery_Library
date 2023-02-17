/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2021 FireSourcery / The Firebrand Forge Inc

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
        case MOT_VAR_NULL:      value = 0;                                                  break;
        case MOT_VAR_SPEED:     value = Motor_User_GetSpeed_FracU16(p_motor);               break;
        case MOT_VAR_I_PHASE:   value = Motor_User_GetIPhase_FracS16(p_motor);              break;
        case MOT_VAR_V_PHASE:   value = Motor_User_GetVPhase_FracS16(p_motor);              break;
        case MOT_VAR_POWER:     value = Motor_User_GetElectricalPower_FracS16(p_motor);     break;

        case MOT_VAR_FOC_IQ:    value = p_motor->Foc.Iq;        break;
        case MOT_VAR_FOC_ID:    value = p_motor->Foc.Id;        break;
        case MOT_VAR_FOC_Q_REQ: value = p_motor->Foc.VIqReq;    break;
        case MOT_VAR_FOC_D_REQ: value = p_motor->Foc.VIdReq;    break;
        case MOT_VAR_FOC_IA:    value = p_motor->Foc.Ia;        break;
        case MOT_VAR_FOC_IB:    value = p_motor->Foc.Ib;        break;
        case MOT_VAR_FOC_IC:    value = p_motor->Foc.Ic;        break;

        case MOT_VAR_DEBUG:     value = qfrac16_sin(Millis());              break;
        case MOT_VAR_MILLIS:    value = Millis();                           break;

        case MOT_VAR_MC_STATE:          value = MotorController_User_GetStateId(p_mc);    break;
        case MOT_VAR_MC_STATUS_FLAGS:   value = 0;                  break;

        //todo change to adcu
        case MOT_VAR_V_SOURCE:      value = MotorController_User_GetVSource(p_mc, 1000U);           break;
        case MOT_VAR_V_SENSOR:      value = MotorController_User_GetVSense(p_mc, 1U);               break;
        case MOT_VAR_V_ACC:         value = MotorController_User_GetVAcc(p_mc, 1U);                 break;
        case MOT_VAR_HEAT_PCB:      value = MotorController_User_GetHeatPcb_DegC(p_mc, 1U);         break;
        case MOT_VAR_HEAT_MOSFETS:  value = MotorController_User_GetHeatMosfets_DegC(p_mc, 1U);     break;

        case MOT_VAR_TX_PACKET_COUNT:   value = Protocol_GetTxPacketCount(p_protocol);      break;
        case MOT_VAR_RX_PACKET_COUNT:   value = Protocol_GetRxPacketCount(p_protocol);      break;

        case MOT_VAR_ANALOG_THROTTLE:   value = MotAnalogUser_GetThrottle(&p_mc->AnalogUser);   break;  /* Value U16 */
        case MOT_VAR_ANALOG_BRAKE:      value = MotAnalogUser_GetBrake(&p_mc->AnalogUser);      break;  /* Value U16 */

        case MOT_VAR_USER_CMD:      value = MotorController_User_GetCmdValue(p_mc);         break;
        case MOT_VAR_THROTTLE:      value = MotorController_User_GetCmdValue(p_mc);         break;  /* Write-only or May differ from Write */
        case MOT_VAR_BRAKE:         value = MotorController_User_GetCmdValue(p_mc);         break;  /* Write-only or May differ from Write */
        case MOT_VAR_DIRECTION:     value = MotorController_User_GetDirection(p_mc);        break;   /* Value 0: Neutral, 1: Reverse, 2: Forward */

        case MOT_VAR_ACTIVE_FEEDBACK_MODE:   value = Motor_User_GetDefaultFeedbackMode(p_motor);    break; //todo change to active


        /* Parameters */
        case MOT_VAR_DEFAULT_FEEDBACK_MODE:   value = Motor_User_GetDefaultFeedbackMode(p_motor);    break; //todo change to active
        // case MOT_VAR_HEAT_PCB_DEG_C:    MotorController_User_GetHeatPcb_DegC(p_mc, 1U);             break;
        // case MOT_VAR_I_MAX_REF_AMP:  value = MotorController_User_GetIMax(p_mc); break;
        // case MOT_VAR_I_PEAK_AMP:              break;
        // case MOT_VAR_SPEED_GROUND_KMH:          break;
        // case MOT_VAR_HEAT_PCB_DEG_C:         break;
        // case MOT_VAR_POLE_PAIRS:                  value = Motor_User_GetPolePairs(p_motor);     break;
        case MOT_VAR_VERSION:   value =  MotorController_User_GetMainVersion(p_mc);         break;
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
    switch(varId)
    {
        case MOT_VAR_USER_CMD:      MotorController_User_SetCmdValue(p_mc, (uint16_t)varValue);                         writeCount = 2U;    break;
        case MOT_VAR_THROTTLE:      MotorController_User_SetCmdThrottle(p_mc, (uint16_t)varValue);                      writeCount = 2U;    break;
        case MOT_VAR_BRAKE:         MotorController_User_SetCmdBrake(p_mc, (uint16_t)varValue);                         writeCount = 2U;    break;
        case MOT_VAR_DIRECTION:     MotorController_User_SetDirection(p_mc, (MotorController_Direction_T)varValue);     writeCount = 2U;    break;  /* Value 0: Neutral, 1: Reverse, 2: Forward */
        case MOT_VAR_BEEP:          MotorController_User_BeepN(p_mc, 500U, 500U, varValue);                             writeCount = 2U;    break;

        case MOT_VAR_ACTIVE_FEEDBACK_MODE:  Motor_User_SetDefaultFeedbackMode(p_motor, (Motor_FeedbackModeId_T)varValue);  writeCount = 2U;   break; //todo change to active

        /* Parameters */
        case MOT_VAR_DEFAULT_FEEDBACK_MODE:  Motor_User_SetDefaultFeedbackMode(p_motor, (Motor_FeedbackModeId_T)varValue);  writeCount = 2U;   break; //todo change to active
        // case MOT_VAR_USER_INPUT_MODE: MotorController_User_SetInputMode_Blocking(p_mc, (MotorController_InputMode_T)varValue); writeCount = 2U;break;

        // case MOT_VAR_PARAM_X:           break;                    /*  */
        // case MOT_VAR_POLE_PAIRS:    Motor_User_SetPolePairs(MotorController_User_GetPtrMotor(p_mc, 0U), varValue); writeCount = 2U;    break;

        default: writeCount = 0U; break;
    }

    // return writeCount;
}



