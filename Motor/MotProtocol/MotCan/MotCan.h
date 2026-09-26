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
#include "Motor/MotProtocol/MotVarId/MotVarId.h"
#include "Motor/MotProtocol/CANopen/CANopen.h"
#include "Motor/MotProtocol/CANopen/OD.h"
#include "Motor/MotProtocol/CANopen/SDO.h"

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*! CAN IDs */
/******************************************************************************/
#define MOT_CAN_TX_TELEMETRY1_ID     (COB_TXPDO3_BASE)   /* speed, IPhase, VPhase, VBus */
#define MOT_CAN_TX_TELEMETRY2_ID     (COB_TXPDO3_BASE)   /* heat, fault flags, state */
#define MOT_CAN_RX_CONTROL_ID        (COB_RXPDO3_BASE)
#define MOT_CAN_VAR_SDO_ID           (COB_SDO_REQ_BASE)

/******************************************************************************/
/*!
    MotVar objects in the CANopen manufacturer-specific area

    A MotVarId is its index/subindex pair shifted by one nibble:

        index    = 0x2000 | (MotVarId >> 4)     ->  0x2000..0x2FFF
        subindex = MotVarId & 0xF               ->  Base

    so {Resv, Instance, Prefix, Type} name the object and Base names the
    member. MotVar owns exactly one area nibble (0x2); 0x3..0x5 stay open.

    Subindex 0 is a real member (Base 0), not the CiA 301 entry count —
    these are flat accessor records, not arrays.
*/
/******************************************************************************/
#define MOT_CAN_OD_VAR_BASE     (0x2000U)
#define MOT_CAN_OD_VAR_AREA     (OD_AREA_ID(MOT_CAN_OD_VAR_BASE))  /* 0x2 */
#define MOT_CAN_OD_VAR_SHIFT    (4U)                                        /* id bits the subindex carries — Base */

static inline bool MotCan_Od_IsVarIndex(uint16_t index) { return (OD_Index(index).Area == MOT_CAN_OD_VAR_AREA); }

static inline MotVarId_T MotCan_Od_VarIdOf(uint16_t index, uint8_t subindex)
{
    return (MotVarId_T) { .Value = (uint16_t)((OD_Index(index).Offset << MOT_CAN_OD_VAR_SHIFT) | (subindex & 0x0FU)) };
}

/* Encode-form counterparts — host/EDS side of the same bijection. */
static inline uint16_t MotCan_Od_IndexOf(MotVarId_T varId)    { return (uint16_t)(MOT_CAN_OD_VAR_BASE | (varId.Value >> MOT_CAN_OD_VAR_SHIFT)); }
static inline uint8_t  MotCan_Od_SubIndexOf(MotVarId_T varId) { return (uint8_t)varId.Base; }

/*
    MotVarId_Status_T -> CiA 301 abort code.
    State-dependent write refusals all map to 0x08000022, the spec's
    "cannot be transferred because of the present device state".
*/
static inline OD_Status_T MotCan_Od_StatusOf(MotVarId_Status_T status)
{
    switch (status)
    {
        case MOT_VAR_STATUS_OK:                      return OD_OK;
        case MOT_VAR_STATUS_ERROR_INVALID_ID:        return OD_ERR_NO_OBJECT;
        case MOT_VAR_STATUS_ERROR_READ_ONLY:         return OD_ERR_READ_ONLY;
        case MOT_VAR_STATUS_ERROR_WRITE_ONLY:        return OD_ERR_WRITE_ONLY;
        case MOT_VAR_STATUS_ERROR_ACCESS_DISABLED:   return OD_ERR_DEVICE_STATE;
        case MOT_VAR_STATUS_ERROR_NOT_CONFIG_STATE:  return OD_ERR_DEVICE_STATE;
        case MOT_VAR_STATUS_ERROR_NOT_RUNNING_STATE: return OD_ERR_DEVICE_STATE;
        default:                                     return OD_ERR_GENERAL;
    }
}

/******************************************************************************/
/*!
    Typed wire overlay — the SDO payload with its address fields expanded into
    the MotVarId they carry. Same 8 bytes as SDO_T.

    Packed at every level: the overlay is cast onto CAN frame buffers, which
    are byte-aligned, and Cortex-M0+ faults on an unaligned word load.
*/
/******************************************************************************/
typedef union MotCan_VarSdo
{
    struct __attribute__((packed))
    {
        struct __attribute__((packed))
        {
            uint32_t Cmd        : 8;    /* byte 0 — typed view is .Sdo.Cmd */
            uint32_t Type       : 4;    /* index    [3:0]   */
            uint32_t Prefix     : 4;    /* index    [7:4]   */
            uint32_t Instance   : 2;    /* index    [9:8]   */
            uint32_t Flags      : 2;    /* index    [11:10] — MotVarId.Resv under the shift */
            uint32_t OdArea     : 4;    /* index    [15:12] — bit 12 is unused */
            uint32_t Base       : 4;    /* subindex [3:0]   */
            uint32_t Resv       : 4;    /* subindex [7:4]   */
        }
        VarCmd;
        OD_Data_T  Data;                /* bytes 4..7 */
    };
    SDO_T      Sdo;                     /* generic view — what the engine sees */
    uint8_t    Bytes[8];
}
MotCan_VarSdo_T;

static_assert(sizeof(MotCan_VarSdo_T) == sizeof(SDO_T), "MotVar overlay must be frame-sized");
static_assert(offsetof(MotCan_VarSdo_T, Data) == 4U, "Data must overlay bytes 4..7");
static_assert(alignof(MotCan_VarSdo_T) == 1U, "overlay is cast onto byte-aligned CAN buffers");
static_assert((MOT_CAN_OD_VAR_BASE & 0x0FFFU) == 0U, "MotVar owns exactly one area nibble");

static inline bool MotCan_VarSdo_IsVarId(const MotCan_VarSdo_T * p_frame) { return (p_frame->VarCmd.OdArea == MOT_CAN_OD_VAR_AREA); }

static inline MotVarId_T MotCan_VarSdo_VarId(const MotCan_VarSdo_T * p_frame) { return MotCan_Od_VarIdOf(p_frame->Sdo.Index, p_frame->Sdo.SubIndex); }

static inline OD_Info_T MotCan_Od_GetInfo(uint16_t index, uint8_t subindex)
{
    static const OD_Info_T VAR32 = { .Type = OD_TYPE_I32, .Access = OD_ACCESS_RW, .Size = sizeof(int32_t) };
    static const OD_Info_T ABSENT = { .Type = OD_TYPE_NONE, .Access = OD_ACCESS_NONE, .Size = 0U };
    const MotCan_VarSdo_T addr = { .Sdo = { .Index = index, .SubIndex = subindex } };
    /* Flags and subindex Resv have no meaning yet, so any address that sets them does not exist */
    return ((addr.VarCmd.OdArea == MOT_CAN_OD_VAR_AREA) && (addr.VarCmd.Flags == 0U) && (addr.VarCmd.Resv == 0U)) ? VAR32 : ABSENT;
}



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
