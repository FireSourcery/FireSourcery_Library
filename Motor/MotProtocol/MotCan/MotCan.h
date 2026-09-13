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
    @file   MotCan.h
    @author FireSourcery
    @brief  CAN service layer for motor controller.
            RX: control commands (throttle, brake).
            TX: periodic telemetry broadcasts.

    Frame values use the library's internal fract16 representation.
    Scaling to engineering units (RPM, Amps, Volts) is done on the host
    using the rated values readable via the serial protocol.

    IDs follow CANopen Tx/Rx PDO conventions (standard 11-bit):
        0x001         Control   RX  (throttle, brake)
        0x181 = 0x180 + node    Telemetry1 TX  (speed, current, voltage, vbus)
        0x182 = 0x180 + node    Telemetry2 TX  (heat, faults, state)
*/
/******************************************************************************/
#include "Motor/MotProtocol/MotPacket.h"
#include "Motor/MotProtocol/MotVarId.h"
#include "Motor/MotProtocol/Cia402/Cia402.h"

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*! CAN IDs */
/******************************************************************************/
#define MOT_CAN_TX_TELEMETRY1_ID     (0x181U)   /* speed, IPhase, VPhase, VBus */
#define MOT_CAN_TX_TELEMETRY2_ID     (0x182U)   /* heat, fault flags, state */
#define MOT_CAN_RX_CONTROL_ID        (0x001U)


/******************************************************************************/
/*! Frame types */
/******************************************************************************/
/*!
    @brief  RX — throttle and brake commands from the host.
    Throttle and Brake: uint8 linear scale, 0x00 = 0%, 0xFF = 100%.
*/
typedef struct __attribute__((packed))
{
    uint16_t Throttle;
    uint16_t Brake;
    uint8_t Resv[4];
}
MotCan_TractionControl_T;

typedef struct __attribute__((packed))
{
    uint16_t CmdValue;
    uint8_t FeedbackMode;
    uint8_t Resv[5];
}
MotCan_MotorControl_T;

typedef struct __attribute__((packed))
{
    uint16_t ClearFaults;
    uint8_t StopAll;
    uint8_t Resv[5];
}
MotCan_StateControl_T;

/******************************************************************************/
/*!
    MotVar objects in the CANopen manufacturer-specific range (0x2000-0x5FFF)

    MotVarId_T is already a namespaced struct accessor: {Prefix, Type} names the
    struct type, {Instance, Base} names the member within it. That is precisely a
    CANopen record — so the type pair becomes the object index and the member pair
    its subindex:

        index    = 0x2000 | (Prefix << 4) | Type    -> 0x2000..0x20FF  (256 objects)
        subindex = (Instance << 4) | Base           -> 0x00..0x3F      (64 members)

    Every var is therefore reachable over the standard SDO route at 0x600 + node,
    with standard CiA 301 abort codes, and consumes only 256 of the 16384
    manufacturer indices.

    Note: subindex 0 is a real member here (Instance 0, Base 0), not the CiA 301
    "number of entries" count — these are flat accessor records, not arrays.
*/
/******************************************************************************/
#define MOT_CAN_OD_VAR_BASE     (0x2000U)
#define MOT_CAN_OD_VAR_LAST     (MOT_CAN_OD_VAR_BASE | 0x00FFU)

#define MOT_CAN_OD_SUBINDEX_MAX (0x3FU)     /* Instance 2 bits, Base 4 bits */

static inline bool MotCan_Od_IsVarIndex(uint16_t index) { return (index >= MOT_CAN_OD_VAR_BASE) && (index <= MOT_CAN_OD_VAR_LAST); }

static inline MotVarId_T MotCan_Od_ToVarId(uint16_t index, uint8_t subindex)
{
    return (MotVarId_T) { .Prefix = (index >> 4U) & 0x0FU, .Type = index & 0x0FU, .Instance = (subindex >> 4U) & 0x03U, .Base = subindex & 0x0FU };
}

/* Encode-form counterparts — host/EDS side of the same bijection. */
static inline uint16_t MotCan_Od_IndexOf(MotVarId_T varId)    { return MOT_CAN_OD_VAR_BASE | MOT_VAR_ID_TYPE_ID(varId.Prefix, varId.Type); }
static inline uint8_t  MotCan_Od_SubIndexOf(MotVarId_T varId) { return (uint8_t)((varId.Instance << 4U) | varId.Base); }

/*
    MotVarId_Status_T -> CiA 301 abort code.
    State-dependent write refusals all map to 0x08000022, the spec's
    "cannot be transferred because of the present device state".
*/
static inline Cia402_OdStatus_T MotCan_Od_StatusOf(MotVarId_Status_T status)
{
    switch (status)
    {
        case MOT_VAR_STATUS_OK:                      return CIA402_OD_OK;
        case MOT_VAR_STATUS_ERROR_INVALID_ID:        return CIA402_OD_ERR_NO_OBJECT;
        case MOT_VAR_STATUS_ERROR_READ_ONLY:         return CIA402_OD_ERR_READ_ONLY;
        case MOT_VAR_STATUS_ERROR_WRITE_ONLY:        return CIA402_OD_ERR_WRITE_ONLY;
        case MOT_VAR_STATUS_ERROR_ACCESS_DISABLED:   return CIA402_OD_ERR_DEVICE_STATE;
        case MOT_VAR_STATUS_ERROR_NOT_CONFIG_STATE:  return CIA402_OD_ERR_DEVICE_STATE;
        case MOT_VAR_STATUS_ERROR_NOT_RUNNING_STATE: return CIA402_OD_ERR_DEVICE_STATE;
        default:                                     return CIA402_OD_ERR_GENERAL;
    }
}

static inline Cia402_OdInfo_T MotCan_Od_GetInfo(uint16_t index, uint8_t subindex)
{
    return (MotCan_Od_IsVarIndex(index) && (subindex <= MOT_CAN_OD_SUBINDEX_MAX))
        ? (Cia402_OdInfo_T) { .Type = CIA402_OD_TYPE_I32, .Access = CIA402_OD_ACCESS_RW, .Size = sizeof(int32_t) }
    : (Cia402_OdInfo_T) { .Type = CIA402_OD_TYPE_NONE, .Access = CIA402_OD_ACCESS_NONE, .Size = 0U };
}

/******************************************************************************/
/*! TX broadcasts — call periodically (e.g. every 20 ms) */
/******************************************************************************/
/*!
    @brief  TX — primary motion telemetry.
    All values are fract16 fractions of rated quantities, little-endian.
        Speed:  fract16  [-32767, 32767] = [- , + ], direction signed.
        IPhase: fract16  [-32767, 32767] = [-I_calib, +I_calib], direction signed.
        VPhase: fract16  [-32767, 32767] = [-V_calib, +V_calib], direction signed.
        VBus:   fract16  [-32767, 32767] = [-V_calib, +V_calib], direction signed.
*/
typedef struct __attribute__((packed))
{
    int16_t Speed;
    int16_t IPhase;
    int16_t VPhase;
    int16_t VBus;
}
MotCan_Telemetry1_T;

/*!
    @brief  TX — secondary system status.
    Heat values: raw ADC counts scaled to uint8 (ADCU >> (ADC_BITS - 8)).
    FaultFlags: lower 8 bits of MotorController_FaultFlags_T.
    MotorState: active state machine state ID.
*/
typedef struct __attribute__((packed))
{
    uint8_t ControllerHeat;
    uint8_t MotorHeat;
    uint8_t FaultFlags;
    uint8_t MotorState;
    uint8_t StatusFlags;
    // uint8_t MotorFeedback;
    // uint8_t VOutState;
    uint8_t Resv;
}
MotCan_Telemetry2_T;

typedef struct __attribute__((packed))
{
    int16_t Id;
    int16_t Iq;
    int16_t Vd;
    int16_t Vq;
}
MotCan_TelemetryFoc_T;
