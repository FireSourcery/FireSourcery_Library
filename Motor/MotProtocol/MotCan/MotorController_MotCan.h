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
#include "Peripheral/CanBus/CanBus.h"
#include "Peripheral/CanBus/CanBus_Service.h"


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


static const CanBus_BroadcastEntry_T MOT_CAN_BROADCAST_TABLE[] =
{
    [0] = {.ID = MOT_CAN_TX_TELEMETRY1_ID, .INTERVAL = 20U,     .BUILD = (CanBus_BuildBroadcast_T)BuildTelemetry1, .P_STATE = &(CanBus_BroadcastState_T) { 0 } },
    [1] = {.ID = MOT_CAN_TX_TELEMETRY2_ID, .INTERVAL = 1000U,   .BUILD = (CanBus_BuildBroadcast_T)BuildTelemetry2, .P_STATE = &(CanBus_BroadcastState_T) { 0 } },
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

/******************************************************************************/
/*
    MotVar SDO server — manufacturer object range (0x2000..0x20FF)

    Kept separate from the CiA 402 profile server: this one owns only the
    MotVarId mapping and MotorController_Var access, and delegates all SDO
    framing, ccs dispatch and abort encoding to the generic engine
    (Cia402_Sdo_HandleRequest) via an OD callback interface.

    (Motor_Cia402_HandleSdo still carries its own copy of that dispatch rather
    than calling the engine — collapsing it onto the same entry point would
    leave exactly one SDO state machine in the tree.)

    Every var is a 32-bit RW object. There is no per-id access table to consult
    up front — read-only and state-refusals are reported by MotorController_Var_Set's
    status, which MotCan_Od_Status turns into the same abort code an access
    table would have produced.


    ---------------------------------------------------------------------------
    Wire shape of a MotVarId access — expedited SDO, always 8 data bytes
    ---------------------------------------------------------------------------

    COB-ID (11-bit standard)
        request   0x600 + nodeId      master -> drive
        response  0x580 + nodeId      drive  -> master

    The route matches the function code only (ID_MASK 0x780), so any nodeId in
    the low 7 bits is accepted and echoed back on the response.

     bit    10  9  8  7  6 | 5  4  3  2  1  0
            +--------------+-------------------+
            | function 0x600/0x580 |  nodeId   |
            +--------------+-------------------+

    Payload — 8 bytes, CiA 301 expedited layout

        byte   0        1        2        3        4     5     6     7
             +--------+--------+--------+--------+-----+-----+-----+-----+
             |  Cmd   |   Index (LE)    | SubIdx |     Data (LE, i32)    |
             +--------+--------+--------+--------+-----+-----+-----+-----+

    byte 0 — Cmd (Cia402_SdoCmd_T)

         bit  7  6  5 | 4 | 3  2 | 1 | 0
             +--------+---+------+---+---+
             |  ccs   |rsv|  n   | e | s |
             +--------+---+------+---+---+
              ccs = command code   n = unused data bytes
              e   = expedited      s = size indicated

        0x40  upload   init request    (read)         master -> drive
        0x23  download init request    (write, 4B)    master -> drive
        0x43  upload   init response   (read reply)   drive  -> master
        0x60  download init response   (write ack)    drive  -> master
        0x80  abort                    (either direction)

    bytes 1..2 — Index, little-endian, = 0x2000 | (Prefix << 4) | Type
    byte 3     — SubIndex,             = (Instance << 4) | Base

        MotVarId_T
             bit 15 14 | 13 12 | 11 10  9  8 | 7  6  5  4 | 3  2  1  0
                +------+-------+-------------+------------+------------+
                | Resv | Inst  |   Prefix    |    Type    |    Base    |
                +------+-------+-------------+------------+------------+
                   |       |          |            |            |
                   |       |          +-- Index ---+            |
                   |       +------------------- SubIndex -------+
                   +-- must be 0 (not carried on the wire)

    bytes 4..7 — Data, little-endian
        read  request   ignored (send zeros)
        read  response  int32 value
        write request   int32 value
        write ack       zeros
        abort           uint32 CiA 301 abort code (Cia402_OdStatus_T)


    Worked example — VBus charge level, node 1
        Prefix   = MOT_VAR_ID_PREFIX_V_MONITOR        (5)
        Type     = MOT_VAR_TYPE_VBUS_OUT              (0)
        Instance = 0
        Base     = VBUS_VAR_ID_CHARGE_LEVEL_FRACT16   (2)
        -> index 0x2050, subindex 0x02

        read  req  601  [8]  40  50 20  02  00 00 00 00
        read  resp 581  [8]  43  50 20  02  <---- i32 LE ---->

        This object is read-only, so a write is refused by Var_Set and the
        status becomes a CiA 301 abort rather than an ack:

        write req  601  [8]  23  50 20  02  <---- i32 LE ---->
        abort resp 581  [8]  80  50 20  02  02 00 01 06
                                            ^^ 0x06010002 LE, "write to RO object"


    Engine behaviour worth knowing when writing a host

      - On a write the engine decodes the data field per the object's OD type,
        ignoring the e/n/s bits. Every MotVar reports as i32, so all four data
        bytes are consumed — a host must send 4 data bytes (Cmd 0x23), never a
        width-tagged short form such as 0x2F.
      - An abort from the master (Cmd 0x80) is consumed with no reply.
      - Segmented and block transfers are not supported; they abort 0x08000000.
      - An index or subindex outside the mapped range aborts 0x06020000. An
        in-range id that no accessor backs is not detectable on read — it
        returns 0 rather than aborting (see MotCan_OdIf_Get).
*/
/******************************************************************************/
/*
    Cia402_OdInterface_T callbacks — the context-bound shape of the pure
    MotCan_Od_* mapping in MotCan.h. Named apart so the wire-format layer and
    the bound callbacks do not collide on GetInfo.
*/
static inline Cia402_OdInfo_T _MotCan_OdGetInfo(MotorController_T * p_mc, uint16_t index, uint8_t subindex)
{
    (void)p_mc; /* object metadata is static — no device state consulted */
    return MotCan_Od_GetInfo(index, subindex);
}

/* Var_Get has no status channel — an unmapped id reads 0. GetInfo already rejected out-of-range. */
static inline Cia402_OdStatus_T _MotCan_OdGet(MotorController_T * p_mc, uint16_t index, uint8_t subindex, int32_t * p_value)
{
    *p_value = MotorController_Var_Get(p_mc, MotCan_Od_ToVarId(index, subindex));
    return CIA402_OD_OK;
}

static inline Cia402_OdStatus_T _MotCan_OdSet(MotorController_T * p_mc, uint16_t index, uint8_t subindex, int32_t value)
{
    return MotCan_Od_StatusOf(MotorController_Var_Set(p_mc, MotCan_Od_ToVarId(index, subindex), value));
}

static inline void MotCan_HandleSdo(MotorController_T * p_mc, const CAN_Frame_T * p_rx, CAN_Frame_T * p_tx)
{
    Cia402_OdInterface_T od =
    {
        .p_Context = (void *)p_mc, /* MotorController_T is a const typedef; the callbacks cast it back */
        .GetInfo   = (Cia402_OdGetInfoFn_T)_MotCan_OdGetInfo,
        .Get       = (Cia402_OdGetFn_T)_MotCan_OdGet,
        .Set       = (Cia402_OdSetFn_T)_MotCan_OdSet,
    };

    /* p_adapter is unused by the engine for OD-interface-backed servers */
    p_tx->DataLength = Cia402_Sdo_HandleRequest(&od, NULL, (const Cia402_Sdo_T *)p_rx->Data, (Cia402_Sdo_T *)p_tx->Data);
    if (p_tx->DataLength > 0U) { p_tx->CanId.Id32 = (CIA402_COB_SDO_RSP_BASE | CIA402_COB_NODE(p_rx->CanId.Id)); }
}

/*
    0x600 + node — one SDO server address, two object ranges.
        0x2000..0x20FF  manufacturer : MotVar accessors
        otherwise       profile      : CiA 402 standard objects
*/
static inline void Req_HandleSdo(MotorController_T * p_mc, const CAN_Frame_T * p_rx, CAN_Frame_T * p_tx)
{
    if (MotCan_Od_IsVarIndex(((const Cia402_Sdo_T *)p_rx->Data)->Index)) { MotCan_HandleSdo(p_mc, p_rx, p_tx); }
    else { MotorController_Cia402_HandleSdo(p_mc, p_rx, p_tx); }
}


static const CanBus_ReqRoute_T MOT_CAN_ROUTES[] =
{
    { CIA402_COB_RXPDO1_BASE,  CIA402_COB_FUNCTION_MASK, (CanBus_RouteHandler_T)MotorController_Cia402_HandleRxPdo1 },
    { CIA402_COB_RXPDO2_BASE,  CIA402_COB_FUNCTION_MASK, (CanBus_RouteHandler_T)MotorController_Cia402_HandleRxPdo2 },
    { CIA402_COB_SDO_REQ_BASE, CIA402_COB_FUNCTION_MASK, (CanBus_RouteHandler_T)Req_HandleSdo },
    // { MOT_CAN_RX_CONTROL_ID,   0x7FFU, (CanBus_RouteHandler_T)Req_Traction }, /* 0x001 throttle/brake — no reply */
};


static const CanBus_Service_T MOTOR_CONTROLLER_MOT_CAN_SERVICE =
{
    .P_ROUTES = MOT_CAN_ROUTES,
    .ROUTE_COUNT = sizeof(MOT_CAN_ROUTES) / sizeof(MOT_CAN_ROUTES[0]),
    .P_BROADCASTS = MOT_CAN_BROADCAST_TABLE,
    .BROADCAST_COUNT = sizeof(MOT_CAN_BROADCAST_TABLE) / sizeof(MOT_CAN_BROADCAST_TABLE[0]),
};

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