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
    @file   MotorController_Cia402.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "MotorController_Cia402.h"
#include "Cia402.h"
#include "Motor_Cia402.h"
#include "Motor/MotorController/MotorController.h"
// #include "Motor/MotorController/MotorController_Var.h"

/******************************************************************************/
/*
    COB-ID dispatcher

    Routes one inbound CAN frame to the right SDO/RxPDO handler based on the
    function code (upper 4 bits of CobId) and the node id (lower 7 bits).
    Mode-aware for RxPDO2 — picks the typed overlay per ActiveMode.

    Returns true and fills *p_tx if a response should be transmitted.

    Build-side counterpart Motor_Cia402_BuildTxPdoN is called periodically
    (event-driven, SYNC, or timer) to produce TxPDO frames.

    Frames not addressed to this node, and frames in COB-ID classes the
    drive doesn't consume here (NMT 0x000, SYNC 0x080, NMT heartbeat 0x700,
    SDO response 0x580, our own TxPDOs 0x180/0x280/...), are ignored.
*/
/******************************************************************************/
/*
    Per-COB-ID Rx route handlers — mapped directly into CIA402_ROUTES (CanBus_RouteHandler_T shape:
    p_dev is the MotorController context). The route table fans out by COB-ID class; each handler
    resolves its adapter/motor and validates the node id before acting.
*/
static Cia402_Adapter_T * Cia402_Adapter(MotorController_T * p_mc, uint8_t index)
{
    // if (index >= p_mc->MOTORS.LENGTH) { return NULL; }
    return (Cia402_Adapter_T *)(p_mc->MOTORS.P_DEVS[0].P_MOTOR->AdapterBuffer);
}

/* Resolve the addressed adapter, or NULL when the frame is not for this node. */
static Cia402_Adapter_T * Cia402_AdapterIfAddressed(MotorController_T * p_mc, const CAN_Frame_T * p_rx)
{
    Cia402_Adapter_T * p_adapter = Cia402_Adapter(p_mc, 0);
    return (CIA402_COB_NODE(p_rx->CanId.Id) == p_adapter->Config.NodeId) ? p_adapter : NULL;
}

/* 0x200 RxPDO1 — Controlword only */
void MotorController_Cia402_HandleRxPdo1(MotorController_T * p_mc, const CAN_Frame_T * p_rx, CAN_Frame_T * p_tx)
{
    (void)p_tx; /* no reply */
    Cia402_Adapter_T * p_adapter = Cia402_AdapterIfAddressed(p_mc, p_rx);
    if (p_adapter == NULL) { return; }
    Motor_Cia402_HandleRxPdo_Cw(&p_mc->MOTORS.P_DEVS[0], p_adapter, (const Cia402_RxPdo_Control_T *)p_rx->Data);
}

/* 0x300 RxPDO2 — Controlword + setpoint, typed overlay per ActiveMode */
void MotorController_Cia402_HandleRxPdo2(MotorController_T * p_mc, const CAN_Frame_T * p_rx, CAN_Frame_T * p_tx)
{
    (void)p_tx; /* no reply */
    Cia402_Adapter_T * p_adapter = Cia402_AdapterIfAddressed(p_mc, p_rx);
    if (p_adapter == NULL) { return; }
    Motor_T * p_motor = &p_mc->MOTORS.P_DEVS[0];
    switch (p_adapter->Input.ActiveMode)
    {
        case CIA402_MODE_PROFILE_TORQUE:
        case CIA402_MODE_CYCLIC_SYNC_TORQUE:
            Motor_Cia402_HandleRxPdo_CwTorque(p_motor, p_adapter, (const Cia402_RxPdo_ControlTorque_T *)p_rx->Data);
            break;
        case CIA402_MODE_VELOCITY:
        case CIA402_MODE_PROFILE_VELOCITY:
        case CIA402_MODE_CYCLIC_SYNC_VELOCITY:
            Motor_Cia402_HandleRxPdo_CwVelocity(p_motor, p_adapter, (const Cia402_RxPdo_ControlVelocity_T *)p_rx->Data);
            break;
        case CIA402_MODE_PROFILE_POSITION:
        case CIA402_MODE_CYCLIC_SYNC_POSITION:
            Motor_Cia402_HandleRxPdo_CwPosition(p_motor, p_adapter, (const Cia402_RxPdo_ControlPosition_T *)p_rx->Data);
            break;
        default:
            /* No setpoint mapping for current mode — fall back to Controlword-only */
            Motor_Cia402_HandleRxPdo_Cw(p_motor, p_adapter, (const Cia402_RxPdo_Control_T *)p_rx->Data);
            break;
    }
}

/* 0x600 SDO download/upload request — fills p_tx; non-zero DataLength signals a reply to CanBus_ProcRequest */
void MotorController_Cia402_HandleSdo(MotorController_T * p_mc, const CAN_Frame_T * p_rx, CAN_Frame_T * p_tx)
{
    Cia402_Adapter_T * p_adapter = Cia402_AdapterIfAddressed(p_mc, p_rx);
    if (p_adapter == NULL) { return; }
    if (Motor_Cia402_HandleSdo(&p_mc->MOTORS.P_DEVS[0], p_adapter, (const Cia402_Sdo_T *)p_rx->Data, (Cia402_Sdo_T *)p_tx->Data) == true)
    {
        p_tx->CanId.CanId = (CIA402_COB_SDO_RSP_BASE | p_adapter->Config.NodeId);
        p_tx->DataLength = 8U;
    }
}

/*
    Outer dispatcher — one inbound CAN frame, switch on COB-ID

    Frames not addressed to this node, or in unconsumed COB-ID classes
    (NMT, SYNC, EMCY, our own TxPDOs, SDO response), are ignored.
*/
void MotorController_Cia402_HandleRxRequest(MotorController_T * p_mc, const CAN_Frame_T * p_rx, CAN_Frame_T * p_tx)
{
    Cia402_Adapter_T * p_adapter = Cia402_Adapter(p_mc, 0);
    Motor_T * p_motor = &p_mc->MOTORS.P_DEVS[0];

    if (CIA402_COB_NODE(p_rx->CanId.Id) != p_adapter->Config.NodeId) { return; }

    switch (CIA402_COB_FUNCTION(p_rx->CanId.Id))
    {
        case CIA402_COB_RXPDO1_BASE:  MotorController_Cia402_HandleRxPdo1(p_mc, p_rx, p_tx);       break;
        case CIA402_COB_RXPDO2_BASE:  MotorController_Cia402_HandleRxPdo2(p_mc, p_rx, p_tx);       break;
        case CIA402_COB_SDO_REQ_BASE: MotorController_Cia402_HandleSdo(p_mc, p_rx, p_tx);       break;
        /* Not consumed by this drive (NMT, SYNC, EMCY, our own TxPDOs, etc.) */
        default:            break;
    }
}

/*

*/
void MotorController_Cia402_BuildTxPdo1(MotorController_T * p_mc, CAN_Frame_T * p_tx)
{
    Cia402_Adapter_T * p_adapter = Cia402_Adapter(p_mc, 0);
    Motor_T * p_motor = &p_mc->MOTORS.P_DEVS[0];

    p_tx->CanId.CanId = CIA402_COB_TXPDO1_BASE | p_adapter->Config.NodeId;
    Motor_Cia402_BuildTxPdo_Sw(p_motor, (Cia402_TxPdo_Status_T *)p_tx->Data);
    p_tx->DataLength = sizeof(Cia402_TxPdo_Status_T);
}

void MotorController_Cia402_BuildTxPdo2(MotorController_T * p_mc, CAN_Frame_T * p_tx)
{
    Cia402_Adapter_T * p_adapter = Cia402_Adapter(p_mc, 0);
    Motor_T * p_motor = &p_mc->MOTORS.P_DEVS[0];

    p_tx->CanId.CanId = CIA402_COB_TXPDO2_BASE | p_adapter->Config.NodeId;

    switch (p_adapter->Input.ActiveMode)
    {
        case CIA402_MODE_PROFILE_TORQUE:
        case CIA402_MODE_CYCLIC_SYNC_TORQUE:
            Motor_Cia402_BuildTxPdo_SwTorque(p_motor, (Cia402_TxPdo_StatusTorque_T *)p_tx->Data);
            p_tx->DataLength = sizeof(Cia402_TxPdo_StatusTorque_T);
            break;
        case CIA402_MODE_VELOCITY:
        case CIA402_MODE_PROFILE_VELOCITY:
        case CIA402_MODE_CYCLIC_SYNC_VELOCITY:
            Motor_Cia402_BuildTxPdo_SwVelocity(p_motor, (Cia402_TxPdo_StatusVelocity_T *)p_tx->Data);
            p_tx->DataLength = sizeof(Cia402_TxPdo_StatusVelocity_T);
            break;
        case CIA402_MODE_PROFILE_POSITION:
        case CIA402_MODE_CYCLIC_SYNC_POSITION:
            Motor_Cia402_BuildTxPdo_SwPosition(p_motor, (Cia402_TxPdo_StatusPosition_T *)p_tx->Data);
            p_tx->DataLength = sizeof(Cia402_TxPdo_StatusPosition_T);
            break;
        default:
            Motor_Cia402_BuildTxPdo_Sw(p_motor, (Cia402_TxPdo_Status_T *)p_tx->Data);
            p_tx->DataLength = sizeof(Cia402_TxPdo_Status_T);
            break;
    }
}


/*
    One route per consumed COB-ID class → its handler. ID_MASK 0x780 matches the function code
    (upper 4 bits) for any node; each handler validates the node id against Config.NodeId.
*/
const CanBus_ReqRoute_T CIA402_ROUTES[] =
{
    { CIA402_COB_RXPDO1_BASE,  CIA402_COB_FUNCTION_MASK, MotorController_Cia402_HandleRxPdo1 },
    { CIA402_COB_RXPDO2_BASE,  CIA402_COB_FUNCTION_MASK, MotorController_Cia402_HandleRxPdo2 },
    { CIA402_COB_SDO_REQ_BASE, CIA402_COB_FUNCTION_MASK, MotorController_Cia402_HandleSdo },
};

const CanBus_BroadcastEntry_T CIA402_BROADCASTS[] = {
  { .ID = CIA402_COB_TXPDO1_BASE, MotorController_Cia402_BuildTxPdo1, 1000U /* 1 ms */, .P_STATE = &(CanBus_BroadcastState_T){ 0 } },
  { .ID = CIA402_COB_TXPDO2_BASE, MotorController_Cia402_BuildTxPdo2, 1000U /* 1 ms */, .P_STATE = &(CanBus_BroadcastState_T){ 0 } },
  /* Heartbeat, etc. */
};

CanBus_Service_T MOTOR_CONTROLLER_CIA402_SERVICE =
{
    .P_ROUTES = CIA402_ROUTES,
    .ROUTE_COUNT = sizeof(CIA402_ROUTES) / sizeof(CIA402_ROUTES[0]),
    .P_BROADCASTS = CIA402_BROADCASTS,
    .BROADCAST_COUNT = sizeof(CIA402_BROADCASTS) / sizeof(CIA402_BROADCASTS[0]),
};

