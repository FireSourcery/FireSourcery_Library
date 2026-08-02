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

#define MOTOR_CONTROLLER_MOT_PROTOCOL_REQ_TABLE_LENGTH (12U)

extern const Protocol_Req_T MOTOR_CONTROLLER_MOT_PROTOCOL_REQ_TABLE[MOTOR_CONTROLLER_MOT_PROTOCOL_REQ_TABLE_LENGTH];


/*  */
static inline packet_size_t MotorController_ReadVar(MotorController_T * p_dev, const MotPacket_VarReadFixedReq_T * p_rx, MotPacket_VarReadFixedResp_T * p_tx)
{
    p_tx->Value = MotorController_Var_Get(p_dev, (MotVarId_T)p_rx->MotVarId);
    return sizeof(MotPacket_VarReadFixedResp_T);
}

static inline packet_size_t MotorController_WriteVar(MotorController_T * p_dev, const MotPacket_VarWriteFixedReq_T * p_rx, MotPacket_VarWriteFixedResp_T * p_tx)
{
    p_tx->Status = MotorController_Var_Set(p_dev, (MotVarId_T)p_rx->MotVarId, p_rx->Value);
    return sizeof(MotPacket_VarWriteFixedResp_T);
}

static packet_size_t MotorController_ReadVar32s(MotorController_T * p_dev, const MotPacket_Var32ReadReq_T * p_rx, MotPacket_Var32ReadResp_T * p_tx, uint8_t varCount)
{
    for (uint8_t index = 0U; index < varCount; index++) { p_tx->Values[index] = MotorController_Var_Get(p_dev, (MotVarId_T)p_rx->Read[index].MotVarId); }
    return varCount * sizeof(uint32_t);
}

static packet_size_t MotorController_WriteVar32s(MotorController_T * p_dev, const MotPacket_Var32WriteReq_T * p_rx, MotPacket_Var32WriteResp_T * p_tx, uint8_t varCount)
{
    for (uint8_t index = 0U; index < varCount; index++) { p_tx->VarStatus[index] = MotorController_Var_Set(p_dev, (MotVarId_T)p_rx->Write[index].MotVarId, p_rx->Write[index].Value); }
    return varCount * sizeof(uint8_t);
}

static packet_size_t MotorController_ReadVar16s(MotorController_T * p_dev, const MotPacket_VarReadReq_T * p_rx, MotPacket_VarReadResp_T * p_tx, uint8_t count)
{
    for (uint8_t index = 0U; index < count; index++) { p_tx->Value16[index] = (uint16_t)MotorController_Var_Get(p_dev, (MotVarId_T)p_rx->MotVarIds[index]); }
    return count * sizeof(uint16_t);
}

static packet_size_t MotorController_WriteVar16s(MotorController_T * p_dev, const MotPacket_VarWriteReq_T * p_rx, MotPacket_VarWriteResp_T * p_tx, uint8_t count)
{
    for (uint8_t index = 0U; index < count; index++) { p_tx->VarStatus[index] = MotorController_Var_Set(p_dev, (MotVarId_T)p_rx->Pairs[index].MotVarId, p_rx->Pairs[index].Value16); }
    return count * sizeof(uint8_t);
}

/*
    Status Flags for User Interface
    Combined boolean outputs for transport convenience
*/
// typedef union MotorController_StatusFlags
// {
//     struct
//     {
//         // uint16_t HeatWarning : 1U; // ILimit by Heat
//         // uint16_t VSourceLow : 1U; // ILimit by VSourceLow
//         // uint16_t SpeedLimit         : 1U;
//         // uint16_t ILimit             : 1U;
//         // uint16_t BuzzerEnable       : 1U;
//         // derive from thermistor functions
//         // uint16_t ILimitHeatMosfets  : 1U;
//         // uint16_t ILimitHeatPcb      : 1U;
//         // uint16_t ILimitHeatMotors   : 1U;
//     };
//     uint16_t Value;
// }
// MotorController_StatusFlags_T;

// static inline MotorController_StatusFlags_T MotorController_GetStatusFlags(MotorController_T * p_dev)
// {
//     return (MotorController_StatusFlags_T)
//     {
//         // .HeatWarning    = Monitor_GetStatus(p_dev->HEAT_PCB.P_STATE) == HEAT_MONITOR_STATUS_WARNING_OVERHEAT ||
//         //                   Monitor_GetStatus(p_dev->HEAT_MOSFETS.P_STATE) == HEAT_MONITOR_STATUS_WARNING_OVERHEAT,
//         // .HeatWarning    = p_dev->StateFlags.HeatWarning,
//         // .VSourceLow     = p_dev->StateFlags.VSourceLow,
//         // .BuzzerEnable   = p_dev->StateFlags.BuzzerEnable,
//     };
// }

// static inline bool IsProtocolControlMode(MotorController_T * p_dev)
// {
//     switch (p_dev->P_MC->Config.InputMode)
//     {
//         case MOTOR_CONTROLLER_INPUT_MODE_SERIAL:    return true;
//         case MOTOR_CONTROLLER_INPUT_MODE_CAN:       return true;
//         case MOTOR_CONTROLLER_INPUT_MODE_ANALOG:    return false;
//         default: return false;
//     }
// }
