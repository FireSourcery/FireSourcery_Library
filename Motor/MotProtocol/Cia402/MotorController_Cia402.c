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
#include "Motor/MotorController/MotorController_Var.h"

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
static Cia402_Adapter_T * Cia402_Adapter(MotorController_T * p_mc) { /* return p_mc->P_MC->Cia402Adapter; */ }

void MotorController_Cia402_HandleRxRequest(MotorController_T * p_mc, const CAN_Frame_T * p_rx, CAN_Frame_T * p_tx)
{
    Cia402_Adapter_T * p_adapter = Cia402_Adapter(p_mc);
    Motor_T * p_motor = &p_mc->MOTORS.P_DEVS[0];

    if (CIA402_COB_NODE(p_rx->CanId.Id) != p_adapter->Config.NodeId) { return false; }

    switch (CIA402_COB_FUNCTION(p_rx->CanId.Id))
    {
        case CIA402_COB_RXPDO1_BASE: /* 0x200 — Controlword only */
            Motor_Cia402_HandleRxPdo_Cw(p_motor, p_adapter, (const Cia402_RxPdo_Control_T *)p_rx->Data);
            return false;

        case CIA402_COB_RXPDO2_BASE: /* 0x300 — Controlword + setpoint, layout depends on mode */
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
            return false;

        case CIA402_COB_SDO_REQ_BASE: /* 0x600 — SDO download/upload request */
            {
                if (Motor_Cia402_HandleSdo(p_motor, p_adapter, (const Cia402_Sdo_T *)p_rx->Data, (Cia402_Sdo_T *)p_tx->Data) == true)
                {
                    p_tx->CanId.CanId = (CIA402_COB_SDO_RSP_BASE | p_adapter->Config.NodeId);
                    p_tx->DataLength = 8U;
                    return true;
                }
                return false;
            }

        default:
            /* Not consumed by this drive (NMT, SYNC, EMCY, our own TxPDOs, etc.) */
            return false;
    }
}

void MotorController_Cia402_BuildTxPdo1(MotorController_T * p_mc, CAN_Frame_T * p_tx)
{
    Cia402_Adapter_T * p_adapter = Cia402_Adapter(p_mc);
    Motor_T * p_motor = &p_mc->MOTORS.P_DEVS[0];

    p_tx->CanId.CanId = CIA402_COB_TXPDO1_BASE | p_adapter->Config.NodeId;
    Motor_Cia402_BuildTxPdo_Sw(p_motor, (Cia402_TxPdo_Status_T *)p_tx->Data);
    p_tx->DataLength = sizeof(Cia402_TxPdo_Status_T);
}

void MotorController_Cia402_BuildTxPdo2(MotorController_T * p_mc, CAN_Frame_T * p_tx)
{
    Cia402_Adapter_T * p_adapter = Cia402_Adapter(p_mc);
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


// /* Integration glue (application) */

// /* Protocol-side handlers — wrap the typed CiA 402 entry points
//    into the driver's full-frame callback shape. */
// static void OnSdoFrame(void * ctx, const CAN_Frame_T * f)
// {
//     Cia402_CanFrame_T rx = ToCia402(f), tx;
//     if (Motor_Cia402_HandleCanRx(ctx, &Adapter, NodeId, &rx, &tx)) { CanBus_SendFrame(p_can, FromCia402(&tx)); }
// }
// static void OnRxPdoFrame(void * ctx, const CAN_Frame_T * f) { /* same shape */ }

// /* Driver-side route table — pure COB-ID class routing */
// static const CanBus_RxRoute_T ROUTES[] = {
//     { CIA402_COB_RXPDO1_BASE | NODE_ID, 0x7FFU, OnRxPdoFrame },
//     { CIA402_COB_RXPDO2_BASE | NODE_ID, 0x7FFU, OnRxPdoFrame },
//     { CIA402_COB_SDO_REQ_BASE | NODE_ID, 0x7FFU, OnSdoFrame },
// };

// /* TxPDO build adapters — wrap CiA 402 build into the driver's BuildFrame_T shape */
// static void BuildTxPdo1(void * ctx, CAN_Frame_T * f)
// {
//     Cia402_CanFrame_T tx;
//      Motor_Cia402_BuildTxPdo1(ctx, NodeId, &tx);
//     *f = FromCia402(&tx);
// }

// static const CanBus_FrameBroadcast_T BROADCASTS[] = {
//     { BuildTxPdo1, 1000U /* 1 ms */ },
//     /* Heartbeat, TxPDO2, etc. */
// };

// static CanBus_FrameBroadcastState_T BROADCAST_STATE[ARRAY_LEN(BROADCASTS)];

// /* Hot loops */
// void OnRx_ISR(const CAN_Frame_T * f) { CanBus_Service_Dispatch(p_can, f, ROUTES, ARRAY_LEN(ROUTES)); }
// void OnTimer_1ms(void) { CanBus_Service_Tick(p_can, BROADCASTS, BROADCAST_STATE, ARRAY_LEN(BROADCASTS), 1000U); }