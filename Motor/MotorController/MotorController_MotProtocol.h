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
#include "Utility/Protocol/Protocol.h"

#define MOTOR_CONTROLLER_MOT_PROTOCOL_REQ_TABLE_LENGTH (10U)

extern const Protocol_Req_T MOTOR_CONTROLLER_MOT_PROTOCOL_REQ_TABLE[MOTOR_CONTROLLER_MOT_PROTOCOL_REQ_TABLE_LENGTH];


/*
//move to alternate interface extension
    Status Flags for User Interface

    Combined boolean outputs for protocol convenience
*/
typedef union MotorController_StatusFlags
{
    struct
    {
        uint16_t HeatWarning : 1U; // ILimit by Heat
        uint16_t VSourceLow : 1U; // ILimit by VSourceLow
        // uint16_t SpeedLimit         : 1U;
        // uint16_t ILimit             : 1U;
        // uint16_t BuzzerEnable       : 1U;
        // derive from thermistor functions
        // uint16_t ILimitHeatMosfets  : 1U;
        // uint16_t ILimitHeatPcb      : 1U;
        // uint16_t ILimitHeatMotors   : 1U;
    };
    uint16_t Value;
}
MotorController_StatusFlags_T;

static inline MotorController_StatusFlags_T MotorController_GetStatusFlags(MotorController_T * p_context)
{
    return (MotorController_StatusFlags_T)
    {
        // .HeatWarning    = Monitor_GetStatus(p_context->HEAT_PCB.P_STATE) == HEAT_MONITOR_STATUS_WARNING_OVERHEAT ||
        //                   Monitor_GetStatus(p_context->HEAT_MOSFETS.P_STATE) == HEAT_MONITOR_STATUS_WARNING_OVERHEAT,
        // .HeatWarning    = p_context->StateFlags.HeatWarning,
        // .VSourceLow     = p_context->StateFlags.VSourceLow,
        // .BuzzerEnable   = p_context->StateFlags.BuzzerEnable,
    };
}
