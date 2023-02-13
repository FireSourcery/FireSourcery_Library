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
    int32_t value = 0U;
    Motor_T * p_motor = MotorController_GetPtrMotor(p_mc, 0U);
    Protocol_T * p_protocol = MotorController_User_GetPtrProtocol(p_mc, 1U);

    switch(varId)
    {
        case MOT_VAR_NULL:      value = 0;                                                  break;
        case MOT_VAR_SPEED:     value = Motor_User_GetSpeed_FracS16(p_motor);               break;
        case MOT_VAR_CURRENT:   value = Motor_User_GetIPhase_FracS16(p_motor);              break;
        case MOT_VAR_VOLTAGE:   value = Motor_User_GetVPhase_FracS16(p_motor);              break;
        case MOT_VAR_POWER:     value = Motor_User_GetElectricalPower_FracS16(p_motor);     break;
        case MOT_VAR_FOC_IQ:    value = p_motor->Foc.Iq;    break;
        case MOT_VAR_FOC_ID:    value = p_motor->Foc.Id;    break;
        case MOT_VAR_DEBUG:     value = qfrac16_sin(Millis());              break;
        case MOT_VAR_MILLIS:    value = Millis();                           break;
        case MOT_VAR_TX_PACKET_COUNT:   value = Protocol_GetTxPacketCount(p_protocol);      break;
        case MOT_VAR_RX_PACKET_COUNT:   value = Protocol_GetRxPacketCount(p_protocol);      break;
        case MOT_VAR_VERSION:   value =  MotorController_User_GetMainVersion(p_mc);         break;


        // // case MOT_VAR_THROTTLE:      value = MotAnalogUser_GetThrottle(&p_mc->AnalogUser);   break;
        // // case MOT_VAR_BRAKE:         value = MotAnalogUser_GetBrake(&p_mc->AnalogUser);      break;
        // case MOT_VAR_THROTTLE:   value = MotorController_User_GetCmdValue(p_mc);         break;  /* Write-Only */
        // case MOT_VAR_BRAKE:      value = MotorController_User_GetCmdValue(p_mc);         break;  /* Write-Only */
        //     // case MOT_VAR_ANALOG_THROTTLE:        value = MotAnalogUser_GetThrottle(&p_mc->AnalogUser);     break;                    /* Value 16-bit */
        //     // case MOT_VAR_ANALOG_BRAKE:            value = MotAnalogUser_GetBrake(&p_mc->AnalogUser);         break;                    /* Value 16-bit */

        // case MOT_VAR_DIRECTION:        value = (uint32_t)MotorController_User_GetDirection(p_mc);     break;                /* Value 0: Neutral, 1: Reverse, 2: Forward */
        // case MOT_VAR_USER_CMD:        value = MotorController_User_GetCmdValue(p_mc);                break;                /* Write-Only */

        // case MOT_VAR_SPEED_RPM:        value = Motor_User_GetSpeed_Rpm(p_motor);     break;
        // case MOT_VAR_ERROR_CODE:    value = p_mc->FaultFlags.State;                                break;
        // case MOT_VAR_MC_STATE:        value = (uint32_t)MotorController_User_GetStateId(p_mc);     break;

        //     // case MOT_VAR_HEAT_PCB_DEG_C:    MotorController_User_GetHeatPcb_DegC(p_mc, 1U);             break;

        // case MOT_VAR_I_PEAK_AMP:              break;
        // case MOT_VAR_SPEED_GROUND_KMH:          break;
        // case MOT_VAR_HEAT_PCB_DEG_C:         break;
        // case MOT_VAR_POLE_PAIRS:                  value = Motor_User_GetPolePairs(p_motor);     break;
        // case MOT_VAR_SPEED_FEEDBACK_REF_RPM:    break;
        // case MOT_VAR_I_MAX_REF_AMP:             value = MotorController_User_GetIMax(p_mc); break;
        default: value = 0U; break;
    }

    return value;
}


/*!
    @return Variable length
*/
uint8_t MotorController_Var_Set(MotorController_T * p_mc, MotVarId_T varId, uint32_t varValue)
{
    uint8_t writeCount = 0U; // identify var size on write?

    switch(varId)
    {
        case MOT_VAR_USER_CMD:      MotorController_User_SetCmdValue(p_mc, (uint16_t)varValue);                         writeCount = 2U;    break;
        case MOT_VAR_THROTTLE:      MotorController_User_SetCmdThrottle(p_mc, (uint16_t)varValue);                      writeCount = 2U;    break;
        case MOT_VAR_BRAKE:         MotorController_User_SetCmdBrake(p_mc, (uint16_t)varValue);                         writeCount = 2U;    break;
        case MOT_VAR_DIRECTION:     MotorController_User_SetDirection(p_mc, (MotorController_Direction_T)varValue);     writeCount = 2U;    break;  /* Value 0: Neutral, 1: Reverse, 2: Forward */
        case MOT_VAR_BEEP:          MotorController_User_BeepN(p_mc, 500U, 500U, varValue);                             writeCount = 2U;    break;

        /* Parameters */
        case MOT_VAR_USER_INPUT_MODE:
            switch((MotorController_InputMode_T)varValue)
            {
                case MOTOR_CONTROLLER_INPUT_MODE_ANALOG:    break;
                case MOTOR_CONTROLLER_INPUT_MODE_PROTOCOL:  break;
                case MOTOR_CONTROLLER_INPUT_MODE_DISABLE:   break;
                case MOTOR_CONTROLLER_INPUT_MODE_CAN:       break;
                default:     break;
            }
            // MotorController_User_ProcCalibration_Blocking(p_mc, MOTOR_CONTROLLER_TOGGLE_USER_INPUT_MODE);
            writeCount = 2U;
            break;
        // case MOT_VAR_PARAM_X:           break;                    /*  */
        // case MOT_VAR_POLE_PAIRS:    Motor_User_SetPolePairs(MotorController_User_GetPtrMotor(p_mc, 0U), varValue); writeCount = 2U;    break;

        default: writeCount = 0U; break;
    }

    // return writeCount;
}



