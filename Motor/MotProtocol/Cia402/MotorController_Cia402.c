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
#include "Cia402_Pdo.h"
#include "Motor/MotProtocol/CANopen/SDO.h"
#include "Motor_Cia402.h"
#include "Motor/MotorController/MotorController.h"
// #include "Motor/MotorController/MotorController_Var.h"

/******************************************************************************/
/*
    COB-ID dispatcher

    Routes one inbound CAN frame to the right SDO/RxPDO handler based on the
    function code (upper 4 bits of CobId) and the node id (lower 7 bits).
    RxPDOs share one handler; each channel's mapping says what its bytes are.

    Returns true and fills *p_tx if a response should be transmitted.

    TxPDOs share one builder, polled each millisecond; each channel's event
    timer sets its period. BuildTxPdo1/2 remain as the fixed alternative.

    Frames not addressed to this node, and frames in COB-ID classes the
    drive doesn't consume here (NMT 0x000, SYNC 0x080, NMT heartbeat 0x700,
    SDO response 0x580, our own TxPDOs 0x180/0x280/...), are ignored.
*/
/******************************************************************************/
/*
    Per-COB-ID Rx route handlers — mapped directly into CIA402_ROUTES
    (CAN_RouteHandler_T shape: p_dev is the MotorController context).
    The route table fans out by COB-ID class; the bus filter admits only this node.
*/
#if defined(MOTOR_CONTROLLER_CAN_ENABLE)
static Cia402_Adapter_T * Cia402_Adapter(MotorController_T * p_mc, uint8_t axis)
{
    assert(axis >= p_mc->MOTORS.LENGTH);
    return &p_mc->P_CIA402_ADAPTERS[axis];
}
#else
static Cia402_Adapter_T * Cia402_Adapter(MotorController_T * p_mc, uint8_t axis)
{
    (void)p_mc;    (void)axis;    return NULL;
}
#endif


/******************************************************************************/
/*
    Application dictionary — the objects SDO and RPDO both reach

    Device-profile objects, paged per axis: the index names the axis, and
    Motor_Cia402 sees the axis-0 index. RPDO mapping validation, RPDO
    writes, and SDO access all go through this one interface.
*/
/******************************************************************************/
static OD_Info_T _AppOd_GetInfo(MotorController_T * p_mc, uint16_t index, uint8_t subindex)
{
    return (Cia402_OdIndex_IsProfile(index) && (Cia402_OdIndex_DecodeAxis(index) < p_mc->MOTORS.LENGTH)) ?
        Cia402_Od_GetInfo(Cia402_OdDeviceIndex(index), subindex) : (OD_Info_T) { .Type = OD_TYPE_NONE };
}

static OD_Status_T _AppOd_Get(MotorController_T * p_mc, uint16_t index, uint8_t subindex, int32_t * p_value)
{
    uint8_t axis = Cia402_OdIndex_DecodeAxis(index);
    if (axis >= p_mc->MOTORS.LENGTH) { return OD_ERR_NO_OBJECT; }
    return Motor_Cia402_Od_Get(&p_mc->MOTORS.P_DEVS[axis], Cia402_Adapter(p_mc, axis), Cia402_OdDeviceIndex(index), subindex, p_value);
}

static OD_Status_T _AppOd_Set(MotorController_T * p_mc, uint16_t index, uint8_t subindex, int32_t value)
{
    uint8_t axis = Cia402_OdIndex_DecodeAxis(index);
    if (axis >= p_mc->MOTORS.LENGTH) { return OD_ERR_NO_OBJECT; }
    return Motor_Cia402_Od_Set(&p_mc->MOTORS.P_DEVS[axis], Cia402_Adapter(p_mc, axis), Cia402_OdDeviceIndex(index), subindex, value);
}

static const OD_Interface_T APP_OD =
{
    .GetInfo = (OD_GetInfoFn_T)_AppOd_GetInfo,
    .Get     = (OD_GetFn_T)_AppOd_Get,
    .Set     = (OD_SetFn_T)_AppOd_Set,
};


/******************************************************************************/
/*
    Route handlers
*/
/******************************************************************************/
/* 0x600 SDO — the communication area holds the PDO parameters; the rest is the application dictionary. */
void MotorController_Cia402_HandleSdo(MotorController_T * p_mc, const CAN_Frame_T * p_rx, CAN_Frame_T * p_tx)
{
    const SDO_T * p_req = (const SDO_T *)p_rx->Data;

    const OD_Interface_T * p_od = (OD_Area_Of(p_req->Index) == OD_AREA_COMM_PROFILE) ? &PDO_PARAM_OD : &APP_OD;
    const void * p_context = (OD_Area_Of(p_req->Index) == OD_AREA_COMM_PROFILE) ? (const void *)&CIA402_PDO_TABLES : (const void *)p_mc;

    p_tx->DataLength = SDO_HandleRequest(p_od, (void *)p_context, p_req, (SDO_T *)p_tx->Data);
    if (p_tx->DataLength > 0U) { p_tx->CanId.Id32 = (COB_SDO_RSP_BASE | COB_NODE(p_rx->CanId.Id)); }
}

/* Every RxPDO route — the channel table decides which PDO this is and what its bytes mean. */
void MotorController_Cia402_HandleRxPdo(MotorController_T * p_mc, const CAN_Frame_T * p_rx, CAN_Frame_T * p_tx)
{
    (void)p_tx; /* no reply */
    PDO_HandleRx(&APP_OD, (void *)p_mc, &CIA402_PDO_TABLES.RX, (uint16_t)p_rx->CanId.Id, (const PDO_T *)p_rx->Data, p_rx->DataLength);
}

static uint32_t TxPdoTimestamps[CIA402_PDO_COUNT]; /* last transmission, ms */

/*
    Every TPDO broadcast entry. The entry's seeded ID names the channel — TxPDO1..4's predefined base,
    0x180 + 0x100 * n — and the channel's own COB-ID replaces it. Called each millisecond; builds a frame only
    when the channel's event timer is due, so an empty frame means nothing to send.
    Channel COB-IDs keep node bits 0 until this node's id reaches the transmit path — as BuildTxPdo1/2 do today.
*/
void MotorController_Cia402_BuildTxPdo(MotorController_T * p_mc, CAN_Frame_T * p_tx)
{
    uint8_t n = (uint8_t)((p_tx->CanId.Id - COB_TXPDO1_BASE) >> 8U);
    uint32_t now = TimerT_Ticks(&p_mc->MILLIS_TIMER);

    if (n >= CIA402_PDO_TABLES.TX.COUNT) { return; }
    if (PDO_Channel_IsTxDue(&Cia402_PdoConfig.Tx[n], TxPdoTimestamps[n], now) == true)
    {
        p_tx->CanId.Id32 = Cia402_PdoConfig.Tx[n].CobId.CanId;
        p_tx->DataLength = PDO_BuildTx(&APP_OD, (void *)p_mc, &Cia402_PdoConfig.Tx[n], (PDO_T *)p_tx->Data);
        TxPdoTimestamps[n] = now;
    }
}



/*
    One route per consumed COB-ID class → its handler. ID_MASK 0x780 matches the function code
    (upper 4 bits); the bus filter admits only this node. A channel moved outside these four classes is not routed.
*/
/*
    Outer dispatcher — one inbound CAN frame, switch on COB-ID

    Frames not addressed to this node, or in unconsumed COB-ID classes
    (NMT, SYNC, EMCY, our own TxPDOs, SDO response), are ignored.
*/
const CAN_ReqRoute_T CIA402_ROUTES[] =
{
    { COB_RXPDO1_BASE,  COB_FUNCTION_MASK, (CAN_RouteHandler_T)MotorController_Cia402_HandleRxPdo },
    { COB_RXPDO2_BASE,  COB_FUNCTION_MASK, (CAN_RouteHandler_T)MotorController_Cia402_HandleRxPdo },
    { COB_RXPDO3_BASE,  COB_FUNCTION_MASK, (CAN_RouteHandler_T)MotorController_Cia402_HandleRxPdo },
    { COB_RXPDO4_BASE,  COB_FUNCTION_MASK, (CAN_RouteHandler_T)MotorController_Cia402_HandleRxPdo },
    { COB_SDO_REQ_BASE, COB_FUNCTION_MASK, (CAN_RouteHandler_T)MotorController_Cia402_HandleSdo },
};

const CAN_BroadcastEntry_T CIA402_BROADCASTS[] =
{
    /* One entry per TPDO channel, checked each millisecond; the channel's event timer sets the period. */
    { .ID = COB_TXPDO1_BASE, .BUILD = (CAN_BuildBroadcast_T)MotorController_Cia402_BuildTxPdo, .INTERVAL = 1U, .P_STATE = &(CAN_BroadcastState_T){ 0 } },
    { .ID = COB_TXPDO2_BASE, .BUILD = (CAN_BuildBroadcast_T)MotorController_Cia402_BuildTxPdo, .INTERVAL = 1U, .P_STATE = &(CAN_BroadcastState_T){ 0 } },
    { .ID = COB_TXPDO3_BASE, .BUILD = (CAN_BuildBroadcast_T)MotorController_Cia402_BuildTxPdo, .INTERVAL = 1U, .P_STATE = &(CAN_BroadcastState_T){ 0 } },
    { .ID = COB_TXPDO4_BASE, .BUILD = (CAN_BuildBroadcast_T)MotorController_Cia402_BuildTxPdo, .INTERVAL = 1U, .P_STATE = &(CAN_BroadcastState_T){ 0 } },
    /* Fixed alternative: { .ID = COB_TXPDO1_BASE, .BUILD = (CAN_BuildBroadcast_T)MotorController_Cia402_BuildTxPdo1, .INTERVAL = 10U, ... } */
    /* Heartbeat, etc. */
};

CAN_Service_T MOTOR_CONTROLLER_CIA402_SERVICE =
{
    .P_ROUTES = CIA402_ROUTES,
    .ROUTE_COUNT = sizeof(CIA402_ROUTES) / sizeof(CIA402_ROUTES[0]),
    .P_BROADCASTS = CIA402_BROADCASTS,
    .BROADCAST_COUNT = sizeof(CIA402_BROADCASTS) / sizeof(CIA402_BROADCASTS[0]),
};


/******************************************************************************/
/*

*/
/******************************************************************************/
void MotorController_Cia402_BuildTxPdo1(MotorController_T * p_mc, CAN_Frame_T * p_tx)
{
    // Cia402_Adapter_T * p_adapter = Cia402_Adapter(p_mc, 0);
    Motor_T * p_motor = &p_mc->MOTORS.P_DEVS[0];
    // p_tx->CanId.Id32 = COB_TXPDO1_BASE | p_adapter->Config.NodeId;

    Motor_Cia402_BuildTxPdo_Sw(p_motor, (Cia402_TxPdo_Status_T *)p_tx->Data);
    p_tx->DataLength = sizeof(Cia402_TxPdo_Status_T);
}

void MotorController_Cia402_BuildTxPdo2(MotorController_T * p_mc, CAN_Frame_T * p_tx)
{
    Cia402_Adapter_T * p_adapter = Cia402_Adapter(p_mc, 0);
    Motor_T * p_motor = &p_mc->MOTORS.P_DEVS[0];
    // p_tx->CanId.Id32 = COB_TXPDO2_BASE | p_adapter->Config.NodeId;

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

/* unused tx keeps the same signature */
/* 0x200 RxPDO1 — Controlword only */
void MotorController_Cia402_HandleRxPdoCw(MotorController_T * p_mc, const CAN_Frame_T * p_rx, CAN_Frame_T * p_tx)
{
    (void)p_tx; /* no reply */
    uint8_t axis = Cia402_OdIndex_DecodeAxis(((const SDO_T *)p_rx->Data)->Index);
    Cia402_Adapter_T * p_adapter = Cia402_Adapter(p_mc, axis);
    if (p_adapter == NULL) { return; }

    Motor_Cia402_HandleRxPdo_Cw(&p_mc->MOTORS.P_DEVS[0], p_adapter, (const Cia402_RxPdo_Control_T *)p_rx->Data);
}

/* 0x300 RxPDO2 — Controlword + setpoint, typed overlay per ActiveMode */
void MotorController_Cia402_HandleRxPdoCwMode(MotorController_T * p_mc, const CAN_Frame_T * p_rx, CAN_Frame_T * p_tx)
{
    (void)p_tx; /* no reply */
    uint8_t axis = Cia402_OdIndex_DecodeAxis(((const SDO_T *)p_rx->Data)->Index);
    Cia402_Adapter_T * p_adapter = Cia402_Adapter(p_mc, axis);
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

void MotorController_Cia402_HandleSdo_Direct(MotorController_T * p_mc, const CAN_Frame_T * p_rx, CAN_Frame_T * p_tx)
{
    uint8_t axis = Cia402_OdIndex_DecodeAxis(((const SDO_T *)p_rx->Data)->Index);
    Cia402_Adapter_T * p_adapter = Cia402_Adapter(p_mc, axis);
    if (p_adapter == NULL) { return; }

    if (Motor_Cia402_HandleSdo(&p_mc->MOTORS.P_DEVS[0], p_adapter, (const SDO_T *)p_rx->Data, (SDO_T *)p_tx->Data) == true)
    {
        p_tx->CanId.Id32 = (COB_SDO_RSP_BASE);
        p_tx->DataLength = 8U;
    }
}