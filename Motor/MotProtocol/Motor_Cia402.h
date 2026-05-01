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
extern Cia402_OdStatus_T Motor_Cia402_Od_Get(Motor_T * p_motor, const Cia402_Adapter_T * p_adapter, uint16_t index, uint8_t subindex, int32_t * p_value);
extern Cia402_OdStatus_T Motor_Cia402_Od_Set(Motor_T * p_motor, Cia402_Adapter_T * p_adapter, uint16_t index, uint8_t subindex, int32_t value);

/******************************************************************************/
/*
    SDO server — process one inbound expedited request, fill the response.
    Returns true if the response should be transmitted; false on inbound abort
    (no response is sent for abort requests).
*/
/******************************************************************************/
extern bool Motor_Cia402_HandleSdo(Motor_T * p_motor, Cia402_Adapter_T * p_adapter, const Cia402_Sdo_T * p_req, Cia402_Sdo_T * p_resp);


/******************************************************************************/
/*
    RxPDO consumers — apply pre-mapped process data.
    Caller dispatches by COB-ID per the configured PDO mapping.
*/
/******************************************************************************/
extern void Motor_Cia402_HandleRxPdo_Cw(Motor_T * p_motor, Cia402_Adapter_T * p_adapter, const Cia402_RxPdo_Cw_T * p_pdo);
extern void Motor_Cia402_HandleRxPdo_CwTorque(Motor_T * p_motor, Cia402_Adapter_T * p_adapter, const Cia402_RxPdo_CwTorque_T * p_pdo);
extern void Motor_Cia402_HandleRxPdo_CwVelocity(Motor_T * p_motor, Cia402_Adapter_T * p_adapter, const Cia402_RxPdo_CwVelocity_T * p_pdo);
extern void Motor_Cia402_HandleRxPdo_CwPosition(Motor_T * p_motor, Cia402_Adapter_T * p_adapter, const Cia402_RxPdo_CwPosition_T * p_pdo);


/******************************************************************************/
/*
    TxPDO producers — fill pre-mapped process data.
    Caller dispatches by COB-ID per the configured PDO mapping.
*/
/******************************************************************************/
extern void Motor_Cia402_BuildTxPdo_Sw(Motor_T * p_motor, Cia402_TxPdo_Sw_T * p_pdo);
extern void Motor_Cia402_BuildTxPdo_SwTorque(Motor_T * p_motor, Cia402_TxPdo_SwTorque_T * p_pdo);
extern void Motor_Cia402_BuildTxPdo_SwVelocity(Motor_T * p_motor, Cia402_TxPdo_SwVelocity_T * p_pdo);
extern void Motor_Cia402_BuildTxPdo_SwPosition(Motor_T * p_motor, Cia402_TxPdo_SwPosition_T * p_pdo);



/*
    Outer handler directly on Motor
*/
// extern bool Motor_Cia402_HandleCanRx(Motor_T * p_motor, const CAN_Frame_T * p_rx, CAN_Frame_T * p_tx);
// extern void Motor_Cia402_BuildTxPdo1(Motor_T * p_motor, CAN_Frame_T * p_tx);
// extern void Motor_Cia402_BuildTxPdo2(Motor_T * p_motor, CAN_Frame_T * p_tx);

