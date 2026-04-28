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
    @file   Motor_Cia402.h
    @author FireSourcery
    @brief  Adapter mapping CiA 402 Controlword/Statusword onto the Motor SM.

    Mapping summary:
      Controlword                 → Motor_User function
        DISABLE_VOLTAGE           → Motor_ReleaseVZ          (full coast / phases off)
        SHUTDOWN                  → Motor_ReleaseVZ          (no power-stage equivalent — closest)
        SWITCH_ON                 → Motor_ReleaseV0          (V0 hold, armed)
        DISABLE_OPERATION         → Motor_ReleaseV0          (V0 hold, armed)
        ENABLE_OPERATION          → Motor_ActivateControl    (PWM on)
        QUICK_STOP                → Motor_Disable               (controlled shutdown via INTERVENTION)
        FAULT_RESET (rising edge) → Motor_StateMachine_TryClearFaultAll

      Motor State                 → Statusword
        INIT                      → NOT_READY_TO_SWITCH_ON
        STOP                      → SWITCH_ON_DISABLED
        PASSIVE  (Phase=Z)        → READY_TO_SWITCH_ON
        PASSIVE  (Phase=V0)       → SWITCHED_ON
        RUN                       → OPERATION_ENABLED
        INTERVENTION              → QUICK_STOP_ACTIVE
        OPEN_LOOP                 → OPERATION_ENABLED + Manufacturer0 (manufacturer-specific marker)
        CALIBRATION               → SWITCH_ON_DISABLED + Manufacturer0
        FAULT                     → FAULT
*/
/******************************************************************************/
#include "Cia402.h"
#include "Motor/Motor/Motor_StateMachine.h"
#include "Motor/Motor/Motor_User.h"


/******************************************************************************/
/*
    Inner handlers map Cia402_Adapter_T * p_adapter
    Outer handler match handler signature
*/
/******************************************************************************/

/******************************************************************************/
/*
    Object Dictionary access — by (index, subindex)

    Subindex 0 only for the mandatory CiA 402 entries supported here.
    Returns CIA402_OD_OK on success, otherwise a CiA 301 SDO abort code.
*/
/******************************************************************************/
extern Cia402_OdStatus_T Motor_Cia402_Od_Get(const Motor_T * p_motor, const Cia402_Adapter_T * p_adapter, uint16_t index, uint8_t subindex, int32_t * p_value);
extern Cia402_OdStatus_T Motor_Cia402_Od_Set(const Motor_T * p_motor, Cia402_Adapter_T * p_adapter, uint16_t index, uint8_t subindex, int32_t value);

/******************************************************************************/
/*
    SDO server — process one inbound expedited request, fill the response.
    Returns true if the response should be transmitted; false on inbound abort
    (no response is sent for abort requests).
*/
/******************************************************************************/
extern bool Motor_Cia402_HandleSdo(const Motor_T * p_motor, Cia402_Adapter_T * p_adapter, const Cia402_Sdo_T * p_req, Cia402_Sdo_T * p_resp);


/******************************************************************************/
/*
    RxPDO consumers — apply pre-mapped process data.
    Caller dispatches by COB-ID per the configured PDO mapping.
*/
/******************************************************************************/
extern void Motor_Cia402_HandleRxPdo_Cw(const Motor_T * p_motor, Cia402_Adapter_T * p_adapter, const Cia402_RxPdo_Cw_T * p_pdo);
extern void Motor_Cia402_HandleRxPdo_CwTorque(const Motor_T * p_motor, Cia402_Adapter_T * p_adapter, const Cia402_RxPdo_CwTorque_T * p_pdo);
extern void Motor_Cia402_HandleRxPdo_CwVelocity(const Motor_T * p_motor, Cia402_Adapter_T * p_adapter, const Cia402_RxPdo_CwVelocity_T * p_pdo);
extern void Motor_Cia402_HandleRxPdo_CwPosition(const Motor_T * p_motor, Cia402_Adapter_T * p_adapter, const Cia402_RxPdo_CwPosition_T * p_pdo);


/******************************************************************************/
/*
    TxPDO producers — fill pre-mapped process data.
    Caller dispatches by COB-ID per the configured PDO mapping.
*/
/******************************************************************************/
extern void Motor_Cia402_BuildTxPdo_Sw(const Motor_T * p_motor, Cia402_TxPdo_Sw_T * p_pdo);
extern void Motor_Cia402_BuildTxPdo_SwTorque(const Motor_T * p_motor, Cia402_TxPdo_SwTorque_T * p_pdo);
extern void Motor_Cia402_BuildTxPdo_SwVelocity(const Motor_T * p_motor, Cia402_TxPdo_SwVelocity_T * p_pdo);
extern void Motor_Cia402_BuildTxPdo_SwPosition(const Motor_T * p_motor, Cia402_TxPdo_SwPosition_T * p_pdo);


/******************************************************************************/
/*
    Outer dispatcher — one inbound CAN frame, switch on COB-ID

    Returns true if a response should be transmitted (currently SDO only).
    Frames not addressed to this node, or in unconsumed COB-ID classes
    (NMT, SYNC, EMCY, our own TxPDOs, SDO response), are ignored.

    TxPDOs are produced periodically via the BuildTxPdoN counterparts —
    they are not driven by Rx events.
*/
/******************************************************************************/
extern bool Motor_Cia402_HandleCanRx(const Motor_T * p_motor, const CAN_Frame_T * p_rx, CAN_Frame_T * p_tx);

extern void Motor_Cia402_BuildTxPdo1(const Motor_T * p_motor, CAN_Frame_T * p_tx);
extern void Motor_Cia402_BuildTxPdo2(const Motor_T * p_motor, CAN_Frame_T * p_tx);
// bool Motor_Cia402_HandleCanRxRequest(const Motor_T * p_motor, const CAN_Frame_T * p_rx, CAN_Frame_T * p_tx);

// void Motor_Cia402_HandleCanRxRequest(Motor_T * p_motor, uint16_t cobId, uint8_t * p_data, uint8_t length)
// {
//     Cia402_Adapter_T * p_adapter = Motor_Cia402_Adapter(p_motor);
// }


extern void MotorController_Cia402_HandleRxRequest(const MotorController_T * p_mc, uint16_t cobId, uint8_t * p_data, uint8_t length);
extern void MotorController_Cia402_BuildTxPdo1(const MotorController_T * p_mc, const CAN_Frame_T * p_rx, CAN_Frame_T * p_tx);

