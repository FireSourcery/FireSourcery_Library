#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2026 FireSourcery

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
    @file   MotorController_MotCan.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "MotCan.h"

#include "Motor/MotorController/MotorController.h"
#include "Motor/MotorController/MotorController_MotProtocol.h"
#include "Motor/MotorController/Traction/MotorController_Traction.h"
#include "Motor/Motor/Motor_User.h"
#include "Peripheral/CanBus/CanBus.h"
#include "Peripheral/CanBus/CanBus_Service.h"


#include <stdint.h>

/*!
    @brief  Broadcast primary motion telemetry.
            Speed, phase current & voltage, bus voltage — all as fract16.
*/
static inline void _BuildTelemetry1(MotorController_T * p_mc, MotCan_Telemetry1_T * p_data)
{
    const Motor_Context_T * p_motor = &p_mc->MOTORS.P_STATES[0U];

    *p_data = (const MotCan_Telemetry1_T)
    {
        .Speed = Motor_User_GetSpeed_Fract16(p_motor),
        .IPhase = Motor_GetIPhase_UFract16(p_motor),
        .VPhase = Motor_GetVPhase_UFract16(p_motor),
        .VBus = VBus_Fract16(p_mc->P_VBUS),
    };
}

static inline void BuildTelemetry1(MotorController_T * p_mc, CAN_Frame_T * p_tx)
{
    _BuildTelemetry1(p_mc, (MotCan_Telemetry1_T *)p_tx->Data);
    p_tx->DataLength = sizeof(MotCan_Telemetry1_T);
}

/*!
    @brief  Broadcast secondary system status.
            MOSFET heat, motor heat, system fault flags, motor state.
*/
static inline void _BuildTelemetry2(MotorController_T * p_mc, MotCan_Telemetry2_T * p_data)
{
    const Motor_Context_T * p_motor = &p_mc->MOTORS.P_STATES[0U];

    *p_data = (const MotCan_Telemetry2_T)
    {
        .ControllerHeat = (uint8_t)(Monitor_GetValue(p_mc->HEAT_MOSFETS.P_STATE) >> 4U),
        .MotorHeat = (uint8_t)(Motor_GetHeat_Adcu(p_motor) >> 4U),
        .FaultFlags = (uint8_t)(p_mc->P_MC->FaultFlags.Value),
        .MotorState = (uint8_t)Motor_GetStateId(p_motor),
        // .StatusFlags =
    };
}

static inline void BuildTelemetry2(MotorController_T * p_mc, CAN_Frame_T * p_tx)
{
    _BuildTelemetry2(p_mc, (MotCan_Telemetry2_T *)p_tx->Data);
    p_tx->DataLength = sizeof(MotCan_Telemetry2_T);
}


static const CanBus_BroadcastEntry_T MOT_CAN_BROADCAST_TABLE[] =
{
    [0] = {.ID = MOT_CAN_TX_TELEMETRY1_ID, .INTERVAL = 20U, .BUILD = (CanBus_BuildBroadcast_T)BuildTelemetry1, .P_STATE = &(CanBus_BroadcastState_T) { 0 } },
    [1] = {.ID = MOT_CAN_TX_TELEMETRY2_ID, .INTERVAL = 1000U, .BUILD = (CanBus_BuildBroadcast_T)BuildTelemetry2, .P_STATE = &(CanBus_BroadcastState_T) { 0 } },
};


/******************************************************************************/
/*! RX dispatch — register as CanBus_T REQ_CALLBACK */
/******************************************************************************/
/*!
    @brief  Dispatch received CAN frame to the appropriate handler.
            Signature matches CanBus_RxRequest_T.
            p_context must be MotorController_T *.
*/
/*
    handlers run as interrupt priority of Rx
*/
static inline void Req_Traction(MotorController_T * p_mc, const CAN_Frame_T * p_rx, CAN_Frame_T * p_tx)
{
    (void)p_tx;
    const MotCan_TractionControl_T * p_ctrl = (const MotCan_TractionControl_T *)p_rx->Data;
    MotorController_Traction_SetThrottleBrake(p_mc, p_ctrl->Throttle, p_ctrl->Brake);
}

/*
    Var access — read/write a single 32-bit var by MotVarId; reply on the request's own COB-ID.
    Read:  req {MotVarId, Flags} (4B)        -> resp {Value}  (4B)
    Write: req {MotVarId, Flags, Value} (8B) -> resp {Status} (1B)
*/
static inline void Req_VarRead(MotorController_T * p_mc, const CAN_Frame_T * p_rx, CAN_Frame_T * p_tx)
{
    p_tx->CanId = p_rx->CanId;
    p_tx->DataLength = MotorController_ReadVar(p_mc, (const MotPacket_VarReadFixedReq_T *)p_rx->Data, (MotPacket_VarReadFixedResp_T *)p_tx->Data);
}

static inline void Req_VarWrite(MotorController_T * p_mc, const CAN_Frame_T * p_rx, CAN_Frame_T * p_tx)
{
    p_tx->CanId = p_rx->CanId;
    p_tx->DataLength = MotorController_WriteVar(p_mc, (const MotPacket_VarWriteFixedReq_T *)p_rx->Data, (MotPacket_VarWriteFixedResp_T *)p_tx->Data);
}


static const CanBus_ReqRoute_T MOT_CAN_ROUTES[] =
{
    { MOT_CAN_RX_CONTROL_ID,   0x7FFU, Req_Traction }, /* 0x001 throttle/brake — no reply */
    { MOT_CAN_RX_VAR_READ_ID,  0x7FFU, Req_VarRead  }, /* 0x1A0 */
    { MOT_CAN_RX_VAR_WRITE_ID, 0x7FFU, Req_VarWrite }, /* 0x1A1 */
    /* Config-class vars are reachable through the same Var_Get/Set via the VAR ids above;
       enable dedicated CONFIG ids here if the host addresses them separately (e.g. persist-on-write):
    { MOT_CAN_RX_CONFIG_READ_ID,  0x7FFU, Req_VarRead  },
    { MOT_CAN_RX_CONFIG_WRITE_ID, 0x7FFU, Req_VarWrite }, */
};


static const CanBus_Service_T MOTOR_CONTROLLER_MOT_CAN_SERVICE =
{
    .P_ROUTES = MOT_CAN_ROUTES,
    .ROUTE_COUNT = sizeof(MOT_CAN_ROUTES) / sizeof(MOT_CAN_ROUTES[0]),
    .P_BROADCASTS = MOT_CAN_BROADCAST_TABLE,
    .BROADCAST_COUNT = sizeof(MOT_CAN_BROADCAST_TABLE) / sizeof(MOT_CAN_BROADCAST_TABLE[0]),
};
