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
    @file   Cia402.h
    @author FireSourcery
    @brief  CiA 402 (CANopen drive profile) data interface
            — Controlword, Statusword, Modes of Operation, and standardized object indices.
*/
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "Motor/MotProtocol/CANopen/OD.h"


/******************************************************************************/
/*
    Controlword (0x6040) — Master → Drive

    Lower bits drive the state machine; upper bits are mode-specific.
    State transitions are triggered by the bit pattern of {SO, EV, QS, EO},
    not edges — except FaultReset which is rising-edge sensitive.

    Note: QuickStop is ACTIVE LOW (0 = quick stop requested).
*/
/******************************************************************************/
typedef union Cia402_Control
{
    struct __attribute__((packed))
    {
        uint16_t SwitchOn           : 1; /* [0]  SO   Enable power stage */
        uint16_t EnableVoltage      : 1; /* [1]  EV   Allow DC bus / contactor close */
        uint16_t QuickStop          : 1; /* [2]  QS   ACTIVE LOW: 0 = quick stop */
        uint16_t EnableOperation    : 1; /* [3]  EO   Enable PWM output */
        uint16_t OpModeSpecific0    : 1; /* [4]  oms  Mode-specific (PP: new setpoint, HM: start homing) */
        uint16_t OpModeSpecific1    : 1; /* [5]  oms  Mode-specific */
        uint16_t OpModeSpecific2    : 1; /* [6]  oms  Mode-specific */
        uint16_t FaultReset         : 1; /* [7]  FR   RISING EDGE clears fault */
        uint16_t Halt               : 1; /* [8]  h    Pause motion, hold position */
        uint16_t OpModeSpecific3    : 1; /* [9]  oms  Mode-specific (PP: change-set-immediately) */
        uint16_t OpModeSpecific4    : 1; /* [10] oms  Mode-specific (PP: abs/rel) */
        uint16_t Reserved           : 1; /* [11] */
        uint16_t Manufacturer0      : 1; /* [12] manufacturer-specific */
        uint16_t Manufacturer1      : 1; /* [13] manufacturer-specific */
        uint16_t Manufacturer2      : 1; /* [14] manufacturer-specific */
        uint16_t Manufacturer3      : 1; /* [15] manufacturer-specific */
    };
    uint8_t Bytes[2];
    uint16_t Word;
}
Cia402_Control_T;

/* Mask of state-machine command bits {SO, EV, QS, EO, FR} */
#define CIA402_CONTROL_CMD_MASK             (0x008FU)

/* Canonical command bit patterns (low byte, FR bit 7 cleared except where noted) */
typedef enum Cia402_ControlCmd
{
    CIA402_CMD_DISABLE_VOLTAGE      = 0x0000U, /* SO=0 EV=0 QS=x EO=x         → Switch On Disabled */
    CIA402_CMD_QUICK_STOP           = 0x0002U, /* SO=0 EV=1 QS=0 EO=x         → Quick Stop Active / Switch On Disabled */
    CIA402_CMD_SHUTDOWN             = 0x0006U, /* SO=0 EV=1 QS=1 EO=0         → Ready to Switch On */
    CIA402_CMD_SWITCH_ON            = 0x0007U, /* SO=1 EV=1 QS=1 EO=0         → Switched On */
    CIA402_CMD_SWITCH_ENABLE        = 0x000FU, /* SO=1 EV=1 QS=1 EO=1         → Operation Enabled */
    CIA402_CMD_DISABLE_OPERATION    = 0x0007U, /* SO=1 EV=1 QS=1 EO=0         → Switched On (from Op Enabled) */
    CIA402_CMD_ENABLE_OPERATION     = 0x000FU, /* SO=1 EV=1 QS=1 EO=1         → Operation Enabled */
    CIA402_CMD_FAULT_RESET          = 0x0080U, /* FR rising edge              → Switch On Disabled (if cleared) */
}
Cia402_ControlCmd_T;

/*
    Reduce raw Controlword to one canonical command using only state-machine bits.
    Returns one of Cia402_ControlCmd_T values, or the masked word if no match.
    FaultReset is detected separately by the caller (rising edge required).
*/
static inline Cia402_ControlCmd_T Cia402_DecodeControlCmd(Cia402_Control_T control)
{
    if (control.EnableVoltage == 0U)            return CIA402_CMD_DISABLE_VOLTAGE;
    if (control.QuickStop == 0U)                return CIA402_CMD_QUICK_STOP;
    if (control.SwitchOn == 0U)                 return CIA402_CMD_SHUTDOWN;
    if (control.EnableOperation == 0U)          return CIA402_CMD_SWITCH_ON; /* == DISABLE_OPERATION */
    return CIA402_CMD_ENABLE_OPERATION;
}

static inline bool Cia402_IsFaultResetEdge(Cia402_Control_T prev, Cia402_Control_T curr)
{
    return (prev.FaultReset == 0U) && (curr.FaultReset == 1U);
}


/******************************************************************************/
/*
    Statusword (0x6041) — Drive → Master

    Lower bits encode the state. Mask {SOD, QS, OE, SO, RTSO, F} — bits {6,5,2,1,0,3}.
    Note: QuickStop is ACTIVE LOW (0 = quick stop active).
*/
/******************************************************************************/
typedef union Cia402_Status
{
    struct __attribute__((packed))
    {
        uint16_t ReadyToSwitchOn    : 1; /* [0]  RTSO */
        uint16_t SwitchedOn         : 1; /* [1]  SO   */
        uint16_t OperationEnabled   : 1; /* [2]  OE   PWM active */
        uint16_t Fault              : 1; /* [3]  F    Fault present */
        uint16_t VoltageEnabled     : 1; /* [4]  VE   DC bus OK */
        uint16_t QuickStop          : 1; /* [5]  QS   ACTIVE LOW: 0 = quick stop active */
        uint16_t SwitchOnDisabled   : 1; /* [6]  SOD  In safe state */
        uint16_t Warning            : 1; /* [7]  WRN  Non-fatal alert */
        uint16_t Manufacturer0      : 1; /* [8]  manufacturer-specific */
        uint16_t Remote             : 1; /* [9]  REM  Controlword being honored */
        uint16_t TargetReached      : 1; /* [10] TR   Mode-specific (at setpoint) */
        uint16_t InternalLimit      : 1; /* [11] ILA  Torque/speed limit hit */
        uint16_t OpModeSpecific0    : 1; /* [12] oms  Mode-specific (PP: setpoint ack, HM: attained) */
        uint16_t OpModeSpecific1    : 1; /* [13] oms  Mode-specific (HM: error) */
        uint16_t Manufacturer1      : 1; /* [14] manufacturer-specific */
        uint16_t Manufacturer2      : 1; /* [15] manufacturer-specific */
    };
    uint8_t Bytes[2];
    uint16_t Word;
}
Cia402_Status_T;

/* State decode masks/values — apply to Statusword.Word */
#define CIA402_STATUS_MASK_NRSO             (0x004FU) /* {SOD, F, OE, SO, RTSO} */
#define CIA402_STATUS_MASK_GENERAL          (0x006FU) /* {SOD, QS, F, OE, SO, RTSO} */

#define CIA402_STATUS_NOT_READY_TO_SWITCH_ON    (0x0000U) /* mask NRSO */
#define CIA402_STATUS_SWITCH_ON_DISABLED        (0x0040U) /* mask NRSO */
#define CIA402_STATUS_READY_TO_SWITCH_ON        (0x0021U) /* mask GENERAL */
#define CIA402_STATUS_SWITCHED_ON               (0x0023U) /* mask GENERAL */
#define CIA402_STATUS_OPERATION_ENABLED         (0x0027U) /* mask GENERAL */
#define CIA402_STATUS_QUICK_STOP_ACTIVE         (0x0007U) /* mask GENERAL */
#define CIA402_STATUS_FAULT_REACTION_ACTIVE     (0x000FU) /* mask NRSO */
#define CIA402_STATUS_FAULT                     (0x0008U) /* mask NRSO */

/*
    Drive State (decoded enum form of Statusword)
*/
typedef enum Cia402_State
{
    CIA402_STATE_NOT_READY_TO_SWITCH_ON     = 0x00U,
    CIA402_STATE_SWITCH_ON_DISABLED         = 0x40U,
    CIA402_STATE_READY_TO_SWITCH_ON         = 0x21U,
    CIA402_STATE_SWITCHED_ON                = 0x23U,
    CIA402_STATE_OPERATION_ENABLED          = 0x27U,
    CIA402_STATE_QUICK_STOP_ACTIVE          = 0x07U,
    CIA402_STATE_FAULT_REACTION_ACTIVE      = 0x0FU,
    CIA402_STATE_FAULT                      = 0x08U,
    CIA402_STATE_UNKNOWN,
}
Cia402_State_T;

static inline Cia402_Status_T Cia402_EncodeStatus(Cia402_State_T status) { return (Cia402_Status_T) { .Word = (uint16_t)status }; }

static inline Cia402_State_T Cia402_DecodeStatus(Cia402_Status_T status)
{
    uint16_t general = status.Word & CIA402_STATUS_MASK_GENERAL;
    uint16_t nrso = status.Word & CIA402_STATUS_MASK_NRSO;

    if (nrso == CIA402_STATUS_NOT_READY_TO_SWITCH_ON)   return CIA402_STATE_NOT_READY_TO_SWITCH_ON;
    if (nrso == CIA402_STATUS_SWITCH_ON_DISABLED)       return CIA402_STATE_SWITCH_ON_DISABLED;
    if (nrso == CIA402_STATUS_FAULT_REACTION_ACTIVE)    return CIA402_STATE_FAULT_REACTION_ACTIVE;
    if (nrso == CIA402_STATUS_FAULT)                    return CIA402_STATE_FAULT;
    if (general == CIA402_STATUS_READY_TO_SWITCH_ON)    return CIA402_STATE_READY_TO_SWITCH_ON;
    if (general == CIA402_STATUS_SWITCHED_ON)           return CIA402_STATE_SWITCHED_ON;
    if (general == CIA402_STATUS_OPERATION_ENABLED)     return CIA402_STATE_OPERATION_ENABLED;
    if (general == CIA402_STATUS_QUICK_STOP_ACTIVE)     return CIA402_STATE_QUICK_STOP_ACTIVE;
    return CIA402_STATE_UNKNOWN;
}


/******************************************************************************/
/*
    Option Codes — per-transition shutdown profile selection
*/
/******************************************************************************/
/*
    Modes of Operation (0x6060 / 0x6061)
*/
typedef enum Cia402_OpMode
{
    CIA402_MODE_NONE                    = 0,
    CIA402_MODE_PROFILE_POSITION        = 1,  /* PP  */
    CIA402_MODE_VELOCITY                = 2,  /* vl  */
    CIA402_MODE_PROFILE_VELOCITY        = 3,  /* PV  */
    CIA402_MODE_PROFILE_TORQUE          = 4,  /* TQ  */
    CIA402_MODE_RESV                    = 5,
    CIA402_MODE_HOMING                  = 6,  /* HM  */
    CIA402_MODE_INTERPOLATED_POSITION   = 7,  /* IP  */
    CIA402_MODE_CYCLIC_SYNC_POSITION    = 8,  /* CSP */
    CIA402_MODE_CYCLIC_SYNC_VELOCITY    = 9,  /* CSV */
    CIA402_MODE_CYCLIC_SYNC_TORQUE      = 10, /* CST */
}
Cia402_OpMode_T;

/* Bitmask values for Supported Drive Modes (0x6502) */
#define CIA402_SUPPORTED_PP                 (1UL << 0)
#define CIA402_SUPPORTED_VL                 (1UL << 1)
#define CIA402_SUPPORTED_PV                 (1UL << 2)
#define CIA402_SUPPORTED_TQ                 (1UL << 3)
#define CIA402_SUPPORTED_HM                 (1UL << 5)
#define CIA402_SUPPORTED_IP                 (1UL << 6)
#define CIA402_SUPPORTED_CSP                (1UL << 7)
#define CIA402_SUPPORTED_CSV                (1UL << 8)
#define CIA402_SUPPORTED_CST                (1UL << 9)

typedef enum Cia402_QuickStopOption
{
    CIA402_QS_DISABLE                       = 0, /* Coast — disable drive function */
    CIA402_QS_DECEL_RAMP_KEEP_ENABLED       = 1, /* Decel via 0x6084, stay in Operation Enabled */
    CIA402_QS_QUICKSTOP_RAMP_KEEP_ENABLED   = 2, /* Decel via 0x6085, stay in Operation Enabled */
    CIA402_QS_CURRENT_LIMIT_KEEP_ENABLED    = 3, /* Current-limit decel, stay in Operation Enabled */
    CIA402_QS_VOLTAGE_LIMIT_KEEP_ENABLED    = 4, /* Voltage-limit decel, stay in Operation Enabled */
    CIA402_QS_DECEL_RAMP_HOLD               = 5, /* Decel via 0x6084, transition to Quick Stop Active, hold */
    CIA402_QS_QUICKSTOP_RAMP_HOLD           = 6, /* Decel via 0x6085, transition to Quick Stop Active, hold */
    CIA402_QS_CURRENT_LIMIT_HOLD            = 7, /* Current-limit decel, transition to Quick Stop Active, hold */
    CIA402_QS_VOLTAGE_LIMIT_HOLD            = 8, /* Voltage-limit decel, transition to Quick Stop Active, hold */
}
Cia402_QuickStopOption_T;

typedef enum Cia402_ShutdownOption
{
    CIA402_SHUTDOWN_DISABLE                 = 0, /* Coast */
    CIA402_SHUTDOWN_DECEL_RAMP              = 1, /* Decel via 0x6084 then disable */
}
Cia402_ShutdownOption_T;

typedef enum Cia402_DisableOpOption
{
    CIA402_DISABLE_OP_DISABLE               = 0, /* Coast */
    CIA402_DISABLE_OP_DECEL_RAMP            = 1, /* Decel via 0x6084 then Switched On */
}
Cia402_DisableOpOption_T;

typedef enum Cia402_HaltOption
{
    CIA402_HALT_DISABLE                     = 0, /* Coast */
    CIA402_HALT_DECEL_RAMP                  = 1, /* Decel via 0x6084, hold */
    CIA402_HALT_QUICKSTOP_RAMP              = 2, /* Decel via 0x6085, hold */
    CIA402_HALT_CURRENT_LIMIT               = 3,
    CIA402_HALT_VOLTAGE_LIMIT               = 4,
}
Cia402_HaltOption_T;

typedef enum Cia402_FaultReactionOption
{
    CIA402_FAULT_REACT_DISABLE              = 0, /* Coast immediately */
    CIA402_FAULT_REACT_DECEL_RAMP           = 1, /* Decel via 0x6084 then disable */
    CIA402_FAULT_REACT_QUICKSTOP_RAMP       = 2, /* Decel via 0x6085 then disable */
    CIA402_FAULT_REACT_CURRENT_LIMIT        = 3,
    CIA402_FAULT_REACT_VOLTAGE_LIMIT        = 4,
}
Cia402_FaultReactionOption_T;


/******************************************************************************/
/*
    Object Dictionary Indices (CiA 402)
*/
/******************************************************************************/
/*
    Index interpretation — the page view. CiA 402 pages the device profile area per axis.
*/
typedef union Cia402_OdIndex
{
    struct __attribute__((packed)) { uint16_t Object : 11; uint16_t Page : 5; };    /* page view — axis paging  */
    uint16_t Index;
}
Cia402_OdIndex_T;

#define CIA402_OD_OBJECT_BITS        (11U)
#define CIA402_OD_OBJECT_MASK        (0x07FFU)   /* [10:0]  object within a page */
#define CIA402_OD_PAGE_MASK          (0xF800U)   /* [15:11] area + axis */

static inline Cia402_OdIndex_T Cia402_OdIndex(uint16_t index) { return (Cia402_OdIndex_T) { .Index = index }; }

#define CIA402_OD_DEVICE_BASE        (0x6000U)
#define CIA402_OD_DEVICE_LAST        (0x9FFFU)

#define CIA402_OD_AXIS_OFFSET        (0x800U)    /* one page — axis N is N pages up */
#define CIA402_OD_AXIS_COUNT         (8U)        /* pages 12..19 */

typedef enum Cia402_OdDeviceAxis
{
    /* Profile areas for individual axes */
    CIA402_OD_DEVICE_AXIS0 = 0x6000U,
    CIA402_OD_DEVICE_AXIS1 = CIA402_OD_DEVICE_AXIS0 + CIA402_OD_AXIS_OFFSET,
    CIA402_OD_DEVICE_AXIS2 = CIA402_OD_DEVICE_AXIS1 + CIA402_OD_AXIS_OFFSET,
    CIA402_OD_DEVICE_AXIS3 = CIA402_OD_DEVICE_AXIS2 + CIA402_OD_AXIS_OFFSET,
    CIA402_OD_DEVICE_AXIS4 = CIA402_OD_DEVICE_AXIS3 + CIA402_OD_AXIS_OFFSET,
    CIA402_OD_DEVICE_AXIS5 = CIA402_OD_DEVICE_AXIS4 + CIA402_OD_AXIS_OFFSET,
    CIA402_OD_DEVICE_AXIS6 = CIA402_OD_DEVICE_AXIS5 + CIA402_OD_AXIS_OFFSET,
    CIA402_OD_DEVICE_AXIS7 = CIA402_OD_DEVICE_AXIS6 + CIA402_OD_AXIS_OFFSET,
}
Cia402_OdDeviceAxis_T;

/*
    Object dictionary entries for the device profile area (0x6000 - 0x67FF).
*/
typedef enum Cia402_OdDeviceIndex
{
    CIA402_OD_CONTROLWORD               = (0x6040U), /* RW  U16  Master command */
    CIA402_OD_STATUSWORD                = (0x6041U), /* RO  U16  Drive state report */
    CIA402_OD_QUICK_STOP_OPTION_CODE    = (0x605AU), /* RW  I16  Quick stop behavior */
    CIA402_OD_SHUTDOWN_OPTION_CODE      = (0x605BU), /* RW  I16  Shutdown behavior */
    CIA402_OD_DISABLE_OP_OPTION_CODE    = (0x605CU), /* RW  I16  Disable operation behavior */
    CIA402_OD_HALT_OPTION_CODE          = (0x605DU), /* RW  I16  Halt behavior */
    CIA402_OD_FAULT_REACTION_CODE       = (0x605EU), /* RW  I16  Fault reaction profile */
    CIA402_OD_MODES_OF_OPERATION        = (0x6060U), /* RW  I8   Requested mode */
    CIA402_OD_MODES_OF_OPERATION_DISP   = (0x6061U), /* RO  I8   Active mode */
    CIA402_OD_POSITION_ACTUAL           = (0x6064U), /* RO  I32  Position feedback */
    CIA402_OD_VELOCITY_ACTUAL           = (0x606CU), /* RO  I32  Velocity feedback */
    CIA402_OD_TARGET_TORQUE             = (0x6071U), /* RW  I16  Torque setpoint (per-mille rated) */
    CIA402_OD_TORQUE_ACTUAL             = (0x6077U), /* RO  I16  Torque feedback (per-mille rated) */
    CIA402_OD_CURRENT_ACTUAL            = (0x6078U), /* RO  I16  Current feedback (per-mille rated) */
    CIA402_OD_DC_LINK_VOLTAGE           = (0x6079U), /* RO  U32  DC bus voltage (mV) */
    CIA402_OD_TARGET_POSITION           = (0x607AU), /* RW  I32  Position setpoint (PP / CSP) */
    CIA402_OD_TARGET_VELOCITY           = (0x60FFU), /* RW  I32  Velocity setpoint */
    CIA402_OD_QUICK_STOP_DECELERATION   = (0x6085U), /* RW  U32  Quick-stop ramp rate */
    CIA402_OD_SUPPORTED_DRIVE_MODES     = (0x6502U), /* RO  U32  Bitmask of supported modes */
}
Cia402_OdDeviceIndex_T;

#define CIA402_OD_DEVICE_PAGE_OFFSET (CIA402_OD_DEVICE_BASE >> CIA402_OD_OBJECT_BITS)  /* 12 */

/* Axis helpers — meaningful only where Cia402_OdIndex_IsProfile holds. */
static inline bool Cia402_OdIndex_IsProfile(uint16_t index) { return (OD_Area_Of(index) == OD_AREA_DEVICE_PROFILE); }

/*
    Axis is a biased page: Bits [15:11] = Page 12..19
*/
static inline uint8_t Cia402_OdIndex_DecodeAxis(uint16_t index)
{
    return (uint8_t)(Cia402_OdIndex(index).Page - CIA402_OD_DEVICE_PAGE_OFFSET);
}
/* Re-page onto an axis, keeping the object. Taking only .Object is what normalizes — the input may name any axis. */
/* (index + axis * 0x0800) */
static inline Cia402_OdIndex_T Cia402_OdIndex_EncodeAxis(Cia402_OdDeviceIndex_T index, uint8_t axis)
{
    return (Cia402_OdIndex_T) { .Page = CIA402_OD_DEVICE_PAGE_OFFSET + axis, .Object = index };
}

/* Axis 0 is the canonical form — the value the Cia402_OdDeviceIndex_T constants name. */
static inline Cia402_OdDeviceIndex_T Cia402_OdDeviceIndex(uint16_t index)
{
    return (Cia402_OdDeviceIndex_T)Cia402_OdIndex_EncodeAxis((Cia402_OdDeviceIndex_T)index, 0U).Index;
}


/******************************************************************************/
/*
    Adapter context — one per CiA 402 axis.
    Motor_Cia402 / Cia402_MotorAdapter
*/
/******************************************************************************/
typedef struct Cia402_Input
{
    Cia402_Control_T              Control;
    Cia402_OpMode_T               ActiveMode;       /*  Index : 0x6060 / 0x6061 */
}
Cia402_Input_T;

typedef struct Cia402_Config
{
    Cia402_QuickStopOption_T      QuickStopOption;     /*  Index : 0x605A */
    Cia402_ShutdownOption_T       ShutdownOption;      /*  Index : 0x605B */
    Cia402_DisableOpOption_T      DisableOpOption;     /*  Index : 0x605C */
    Cia402_HaltOption_T           HaltOption;          /*  Index : 0x605D */
    Cia402_FaultReactionOption_T  FaultReactOption;    /*  Index : 0x605E */
    uint32_t                      QuickStopDecel;      /*  Index : 0x6085 */
}
Cia402_Config_T;

typedef struct Cia402_Adapter
{
    Cia402_Input_T Input;
    Cia402_Config_T Config;
}
Cia402_Adapter_T;


/******************************************************************************/
/*
    Object Dictionary metadata (CiA 402)
*/
/******************************************************************************/
typedef struct Cia402_OdMeta
{
    uint16_t    Index;
    uint8_t     SubIndex;
    OD_Type_T   Type;
    OD_Access_T Access;
}
Cia402_OdMeta_T;


/*
    Table Entry
*/
/* Generic accessor — pulls a typed value from a raw byte pointer */
typedef struct Cia402_OdEntry
{
    Cia402_OdMeta_T Meta;
    uint16_t AdapterOffset; /* offsetof(Cia402_Adapter_T, ...) — 0xFFFF if not adapter-backed */
    /* For non-adapter-backed entries, fall back to the function-pointer shape */
    int32_t(*Get)(const void *, const Cia402_Adapter_T *);
    OD_Status_T(*Set)(const void *, Cia402_Adapter_T *, int32_t);
}
Cia402_OdEntry_T;


#define OD_ADAPTER(idx, sub, ty, acc, field) \
    { (idx), (sub), (ty), (acc), offsetof(Cia402_Adapter_T, field), NULL, NULL }

#define OD_FN(idx, sub, ty, acc, get_fn, set_fn) \
    { (idx), (sub), (ty), (acc), 0xFFFFU, (get_fn), (set_fn) }


static const Cia402_OdEntry_T * Cia402_OdTable_Find(const Cia402_OdEntry_T * p_table, size_t length, uint16_t index, uint8_t subindex)
{
    /* Linear is fine for ~20 entries; binary search if it grows past ~50. */
    for (uint16_t i = 0U; i < length; i++)
    {
        const Cia402_OdEntry_T * e = &p_table[i];
        if ((e->Meta.Index == index) && (e->Meta.SubIndex == subindex)) { return e; }
    }
    return NULL;
}

/*
    Object Dictionary metadata — common-layer entry point.

    Returns spec-fixed type / access / size for the given (index, subindex).
    All sizes are compile-time constants (sizeof of the wire type).
    For unknown entries, returns Type = OD_TYPE_NONE.
*/
extern OD_Info_T Cia402_Od_GetInfo(uint16_t index, uint8_t subindex);


/******************************************************************************/
/*
    PDO (Process Data Object)
*/
/******************************************************************************/
/*
    Default PDO mappings per CiA 402 (predefined connection set).
    Each is a packed struct overlay over the PDO byte array.
    Mode-specific RxPDO/TxPDO variants share the same RxPDO/TxPDO COB-ID
    in the spec — pick the variant matching the active operating mode.

    RxPDO (master → drive)
*/
typedef struct __attribute__((packed)) Cia402_RxPdo_Control
{
    Cia402_Control_T Controlword;
}
Cia402_RxPdo_Control_T;

// typedef struct __attribute__((packed)) Cia402_RxPdo_ControlTarget
// {
//     Cia402_Control_T Controlword;
//     int32_t          Target;
// }
// Cia402_RxPdo_ControlTarget_T;

typedef struct __attribute__((packed)) Cia402_RxPdo_ControlTorque
{
    Cia402_Control_T Controlword;
    int16_t          TargetTorque;
}
Cia402_RxPdo_ControlTorque_T;

typedef struct __attribute__((packed)) Cia402_RxPdo_ControlVelocity
{
    Cia402_Control_T Controlword;
    int32_t          TargetVelocity;
}
Cia402_RxPdo_ControlVelocity_T;

typedef struct __attribute__((packed)) Cia402_RxPdo_ControlPosition
{
    Cia402_Control_T Controlword;
    int32_t          TargetPosition;
}
Cia402_RxPdo_ControlPosition_T;

/*  TxPDO (drive → master) */
typedef struct __attribute__((packed)) Cia402_TxPdo_Status
{
    Cia402_Status_T Statusword;
}
Cia402_TxPdo_Status_T;

// typedef struct __attribute__((packed)) Cia402_TxPdo_StatusActual
// {
//     Cia402_Status_T Statusword;
//     int32_t         Actual;
// }
// Cia402_TxPdo_StatusActual_T;

typedef struct __attribute__((packed)) Cia402_TxPdo_StatusTorque
{
    Cia402_Status_T Statusword;
    int16_t         TorqueActual;
}
Cia402_TxPdo_StatusTorque_T;

typedef struct __attribute__((packed)) Cia402_TxPdo_StatusVelocity
{
    Cia402_Status_T Statusword;
    int32_t         VelocityActual;
}
Cia402_TxPdo_StatusVelocity_T;

typedef struct __attribute__((packed)) Cia402_TxPdo_StatusPosition
{
    Cia402_Status_T Statusword;
    int32_t         PositionActual;
}
Cia402_TxPdo_StatusPosition_T;
