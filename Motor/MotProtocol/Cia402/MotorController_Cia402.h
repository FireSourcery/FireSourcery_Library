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
#include "Motor/MotorController/MotorController_Var.h"
#include "Peripheral/CanBus/CanBus.h"
#include "Peripheral/CanBus/CanBus_Service.h"

/******************************************************************************/
/*
    Rx route handlers — one per consumed COB-ID class, mapped directly into CIA402_ROUTES.

    Each has the CanBus_RouteHandler_T shape (p_dev is the MotorController context), resolves
    its adapter, and validates the node id. Only the SDO handler fills p_tx (a non-zero
    DataLength signals a reply); the RxPDO handlers never respond.

    Frames in unconsumed COB-ID classes (NMT, SYNC, EMCY, our own TxPDOs, SDO response) simply
    have no route. TxPDOs are produced periodically via the BuildTxPdoN counterparts — they are
    not driven by Rx events.
*/
/******************************************************************************/

extern void MotorController_Cia402_HandleRxPdo1(MotorController_T * p_mc, const CAN_Frame_T * p_rx, CAN_Frame_T * p_tx);
extern void MotorController_Cia402_HandleRxPdo2(MotorController_T * p_mc, const CAN_Frame_T * p_rx, CAN_Frame_T * p_tx);
extern void MotorController_Cia402_HandleSdo(MotorController_T * p_mc, const CAN_Frame_T * p_rx, CAN_Frame_T * p_tx);

extern void MotorController_Cia402_BuildTxPdo1(MotorController_T * p_mc, CAN_Frame_T * p_tx);
extern void MotorController_Cia402_BuildTxPdo2(MotorController_T * p_mc, CAN_Frame_T * p_tx);

extern CanBus_Service_T MOTOR_CONTROLLER_CIA402_SERVICE;