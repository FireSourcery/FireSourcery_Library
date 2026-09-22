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
#include "Motor/MotProtocol/Cia402/MotorController_Cia402.h"
#include "Motor/MotorController/Traction/MotorController_Traction.h"
#include "Motor/Motor/Motor_User.h"
#include "Peripheral/CAN/CAN.h"
#include "Peripheral/CAN/CAN_Service.h"


#include <stdint.h>

/*!
    @brief  Broadcast primary motion telemetry.
            Speed, phase current & voltage, bus voltage — all as fract16.
*/
static inline void _BuildTelemetry1(MotorController_T * p_mc, MotCan_Telemetry1_T * p_data)
{
    const Motor_Context_T * p_motor = p_mc->MOTORS.P_DEVS[0U].P_MOTOR;

    *p_data = (const MotCan_Telemetry1_T)
    {
        .Speed = Motor_User_GetSpeed_Fract16(p_motor),
        .IPhase = Motor_GetIPhase_UFract16(p_motor),
        .VPhase = Motor_GetVPhase_UFract16(p_motor),
        .VBus = VBus_Fract16(p_mc->P_VBUS),
    };
}

static inline void _BuildTelemetry_Si(MotorController_T * p_mc, MotCan_Telemetry1_T * p_data)
{
    const Motor_Context_T * p_motor = p_mc->MOTORS.P_DEVS[0U].P_MOTOR;

    *p_data = (const MotCan_Telemetry1_T)
    {
        .Speed = Motor_GetSpeed_Rpm(p_motor),
        .IPhase = Motor_GetIPhase_Amps(p_motor),
        .VPhase = Motor_GetVPhase_Volts(p_motor),
        .VBus = VBus_Volts(p_mc->P_VBUS),
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
    const Motor_Context_T * p_motor = p_mc->MOTORS.P_DEVS[0U].P_MOTOR;

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


static const CAN_BroadcastEntry_T MOT_CAN_BROADCAST_TABLE[] =
{
    [0] = {.ID = MOT_CAN_TX_TELEMETRY1_ID, .INTERVAL = 20U,     .BUILD = (CAN_BuildBroadcast_T)BuildTelemetry1, .P_STATE = &(CAN_BroadcastState_T) { 0 } },
    [1] = {.ID = MOT_CAN_TX_TELEMETRY2_ID, .INTERVAL = 1000U,   .BUILD = (CAN_BuildBroadcast_T)BuildTelemetry2, .P_STATE = &(CAN_BroadcastState_T) { 0 } },
};


/******************************************************************************/
/*! RX dispatch — register as CAN_T REQ_CALLBACK */
/******************************************************************************/
/*!
    @brief  Dispatch received CAN frame to the appropriate handler.
            Signature matches CAN_RxRequest_T.
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

/******************************************************************************/
/*
    MotVar SDO server — manufacturer object range (0x2000..0x2FFF)

    Every var is a 32-bit RW object. Read-only and state-refusals are reported
    by MotorController_Var_Set's status, which MotCan_Od_StatusOf turns into the
    same abort code a per-id access table would have produced.
*/
/******************************************************************************/
/*
    OD_Interface_T callbacks — the context-bound shape of the pure
    MotCan_Od_* mapping in MotCan.h. Named apart so the wire-format layer and
    the bound callbacks do not collide on GetInfo.
*/
static inline OD_Info_T _MotCan_OdGetInfo(MotorController_T * p_mc, uint16_t index, uint8_t subindex)
{
    (void)p_mc; /* object metadata is static — no device state consulted */
    return MotCan_Od_GetInfo(index, subindex);
}

/* Var_Get has no status channel — an unmapped id reads 0. GetInfo already rejected out-of-range. */
static inline OD_Status_T _MotCan_OdGet(MotorController_T * p_mc, uint16_t index, uint8_t subindex, int32_t * p_value)
{
    *p_value = MotorController_Var_Get(p_mc, MotCan_Od_VarIdOf(index, subindex));
    return OD_OK;
}

static inline OD_Status_T _MotCan_OdSet(MotorController_T * p_mc, uint16_t index, uint8_t subindex, int32_t value)
{
    return MotCan_Od_StatusOf(MotorController_Var_Set(p_mc, MotCan_Od_VarIdOf(index, subindex), value));
}

static inline void MotCan_HandleSdo(MotorController_T * p_mc, const CAN_Frame_T * p_rx, CAN_Frame_T * p_tx)
{
    // OD_Interface_T od =
    // {
    //     .p_Context = (void *)p_mc, /* MotorController_T is a const typedef; the callbacks cast it back */
    //     .GetInfo   = (OD_GetInfoFn_T)_MotCan_OdGetInfo,
    //     .Get       = (OD_GetFn_T)_MotCan_OdGet,
    //     .Set       = (OD_SetFn_T)_MotCan_OdSet,
    // };

    static const OD_Interface_T od =
    {
        .GetInfo   = (OD_GetInfoFn_T)_MotCan_OdGetInfo,
        .Get       = (OD_GetFn_T)_MotCan_OdGet,
        .Set       = (OD_SetFn_T)_MotCan_OdSet,
    };

    /* p_adapter is unused by the engine for OD-interface-backed servers */
    p_tx->DataLength = SDO_HandleRequest(&od, (void *)p_mc, (const SDO_T *)p_rx->Data, (SDO_T *)p_tx->Data);
    if (p_tx->DataLength > 0U) { p_tx->CanId.Id32 = (COB_SDO_RSP_BASE | COB_NODE(p_rx->CanId.Id)); }
}

/*
    0x600 + node — one SDO server address, two object ranges.
        0x2000..0x2FFF  manufacturer : MotVar accessors
        otherwise       profile      : CiA 402 standard objects
*/
static inline void Req_HandleSdo(MotorController_T * p_mc, const CAN_Frame_T * p_rx, CAN_Frame_T * p_tx)
{
    if (MotCan_Od_IsVarIndex(((const SDO_T *)p_rx->Data)->Index)) { MotCan_HandleSdo(p_mc, p_rx, p_tx); }
    else { MotorController_Cia402_HandleSdo(p_mc, p_rx, p_tx); }
}


static const CAN_ReqRoute_T MOT_CAN_ROUTES[] =
{
    { COB_RXPDO1_BASE,  COB_FUNCTION_MASK, (CAN_RouteHandler_T)MotorController_Cia402_HandleRxPdo }, /* RxPDO3 stays free for MOT_CAN_RX_CONTROL_ID */
    { COB_RXPDO2_BASE,  COB_FUNCTION_MASK, (CAN_RouteHandler_T)MotorController_Cia402_HandleRxPdo },
    { COB_SDO_REQ_BASE, COB_FUNCTION_MASK, (CAN_RouteHandler_T)Req_HandleSdo },
    // { MOT_CAN_RX_CONTROL_ID,   0x7FFU, (CAN_RouteHandler_T)Req_Traction }, /* 0x001 throttle/brake — no reply */
};


static const CAN_Service_T MOTOR_CONTROLLER_MOT_CAN_SERVICE =
{
    .P_ROUTES = MOT_CAN_ROUTES,
    .ROUTE_COUNT = sizeof(MOT_CAN_ROUTES) / sizeof(MOT_CAN_ROUTES[0]),
    .P_BROADCASTS = MOT_CAN_BROADCAST_TABLE,
    .BROADCAST_COUNT = sizeof(MOT_CAN_BROADCAST_TABLE) / sizeof(MOT_CAN_BROADCAST_TABLE[0]),
};

/*
    Hardware acceptance filter for this service — node bits only.
    Accepts every function code addressed to the node and rejects every other node in hardware;
    MOT_CAN_ROUTES then fans out by function code (ID_MASK COB_FUNCTION_MASK).

    Node 0 matches the bare COB bases this service currently answers and broadcasts on.
    Moving to a CANopen node 1..127 also requires OR-ing the node into the Tx ids.
*/
#ifndef MOT_CAN_NODE_ID
#define MOT_CAN_NODE_ID (0U)
#endif

#define MOT_CAN_RX_FILTER_INIT(nodeId) { .Id = { .Id = (nodeId) }, .Mask = COB_NODE_MASK }

#define MOT_CAN_CONFIG_INIT(nodeId) { .IsEnabled = true, .RxFilterCount = 1U, .RxFilters = { MOT_CAN_RX_FILTER_INIT(nodeId) } }

/* #define MOT_CAN_RX_VAR_ID        (0x680U) */
// static inline void Req_VarRead(MotorController_T * p_mc, const CAN_Frame_T * p_rx, CAN_Frame_T * p_tx)
// {
//     p_tx->CanId = p_rx->CanId;
//     p_tx->DataLength = MotorController_ReadVar(p_mc, (const MotPacket_VarReadFixedReq_T *)p_rx->Data, (MotPacket_VarReadFixedResp_T *)p_tx->Data);
// }

// static inline void Req_VarWrite(MotorController_T * p_mc, const CAN_Frame_T * p_rx, CAN_Frame_T * p_tx)
// {
//     p_tx->CanId = p_rx->CanId;
//     p_tx->DataLength = MotorController_WriteVar(p_mc, (const MotPacket_VarWriteFixedReq_T *)p_rx->Data, (MotPacket_VarWriteFixedResp_T *)p_tx->Data);
// }