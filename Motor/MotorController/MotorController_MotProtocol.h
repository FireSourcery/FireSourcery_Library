#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery

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
    @file   MotorController_MotProtocol.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Framework/Protocol/Protocol.h"
#include "Motor/MotProtocol/MotPacket.h"
#include "Motor/MotProtocol/MotProtocol.h"
#include "Motor/MotorController/MotorController_Var.h"

#define MOTOR_CONTROLLER_MOT_PROTOCOL_REQ_TABLE_LENGTH (10U)

extern const Protocol_Req_T MOTOR_CONTROLLER_MOT_PROTOCOL_REQ_TABLE[MOTOR_CONTROLLER_MOT_PROTOCOL_REQ_TABLE_LENGTH];


static packet_size_t MotorController_BuildVarRead(const MotorController_T * p_dev, MotPacket_VarReadResp_T * p_vars, const MotPacket_VarReadReq_T * p_ids, uint8_t count)
{
    for (uint8_t iVar = 0U; iVar < count; iVar++) { p_vars->Value16[iVar] = (uint16_t)MotorController_Var_Get(p_dev, (MotVarId_T)p_ids->MotVarIds[iVar]); }
}





// static inline bool IsProtocolControlMode(const MotorController_T * p_dev)
// {
//     switch (p_dev->P_MC->Config.InputMode)
//     {
//         case MOTOR_CONTROLLER_INPUT_MODE_SERIAL:    return true;
//         case MOTOR_CONTROLLER_INPUT_MODE_CAN:       return true;
//         case MOTOR_CONTROLLER_INPUT_MODE_ANALOG:    return false;
//         default: return false;
//     }
// }
