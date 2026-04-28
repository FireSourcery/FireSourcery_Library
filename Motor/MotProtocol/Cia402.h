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
    @brief  CiA 402 (CANopen drive profile) data interface — Controlword,
            Statusword, Modes of Operation, and standardized object indices.
*/
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>


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

// #define CIA402_STATUS_NOT_READY_TO_SWITCH_ON    (0x0000U) /* mask NRSO */
// #define CIA402_STATUS_SWITCH_ON_DISABLED        (0x0040U) /* mask NRSO */
// #define CIA402_STATUS_READY_TO_SWITCH_ON        (0x0021U) /* mask GENERAL */
// #define CIA402_STATUS_SWITCHED_ON               (0x0023U) /* mask GENERAL */
// #define CIA402_STATUS_OPERATION_ENABLED         (0x0027U) /* mask GENERAL */
// #define CIA402_STATUS_QUICK_STOP_ACTIVE         (0x0007U) /* mask GENERAL */
// #define CIA402_STATUS_FAULT_REACTION_ACTIVE     (0x000FU) /* mask NRSO */
// #define CIA402_STATUS_FAULT                     (0x0008U) /* mask NRSO */

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

// static inline Cia402_State_T Cia402_DecodeStatus(Cia402_Status_T status)
// {
//     uint16_t general = status.Word & CIA402_STATUS_MASK_GENERAL;
//     uint16_t nrso = status.Word & CIA402_STATUS_MASK_NRSO;

//     if (nrso == CIA402_STATUS_NOT_READY_TO_SWITCH_ON)   return CIA402_STATE_NOT_READY_TO_SWITCH_ON;
//     if (nrso == CIA402_STATUS_SWITCH_ON_DISABLED)       return CIA402_STATE_SWITCH_ON_DISABLED;
//     if (nrso == CIA402_STATUS_FAULT_REACTION_ACTIVE)    return CIA402_STATE_FAULT_REACTION_ACTIVE;
//     if (nrso == CIA402_STATUS_FAULT)                    return CIA402_STATE_FAULT;
//     if (general == CIA402_STATUS_READY_TO_SWITCH_ON)    return CIA402_STATE_READY_TO_SWITCH_ON;
//     if (general == CIA402_STATUS_SWITCHED_ON)           return CIA402_STATE_SWITCHED_ON;
//     if (general == CIA402_STATUS_OPERATION_ENABLED)     return CIA402_STATE_OPERATION_ENABLED;
//     if (general == CIA402_STATUS_QUICK_STOP_ACTIVE)     return CIA402_STATE_QUICK_STOP_ACTIVE;
//     return CIA402_STATE_UNKNOWN;
// }


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
#define CIA402_OD_CONTROLWORD               (0x6040U) /* RW  U16  Master command */
#define CIA402_OD_STATUSWORD                (0x6041U) /* RO  U16  Drive state report */
#define CIA402_OD_QUICK_STOP_OPTION_CODE    (0x605AU) /* RW  I16  Quick stop behavior */
#define CIA402_OD_SHUTDOWN_OPTION_CODE      (0x605BU) /* RW  I16  Shutdown behavior */
#define CIA402_OD_DISABLE_OP_OPTION_CODE    (0x605CU) /* RW  I16  Disable operation behavior */
#define CIA402_OD_HALT_OPTION_CODE          (0x605DU) /* RW  I16  Halt behavior */
#define CIA402_OD_FAULT_REACTION_CODE       (0x605EU) /* RW  I16  Fault reaction profile */
#define CIA402_OD_MODES_OF_OPERATION        (0x6060U) /* RW  I8   Requested mode */
#define CIA402_OD_MODES_OF_OPERATION_DISP   (0x6061U) /* RO  I8   Active mode */
#define CIA402_OD_POSITION_ACTUAL           (0x6064U) /* RO  I32  Position feedback */
#define CIA402_OD_VELOCITY_ACTUAL           (0x606CU) /* RO  I32  Velocity feedback */
#define CIA402_OD_TARGET_TORQUE             (0x6071U) /* RW  I16  Torque setpoint (per-mille rated) */
#define CIA402_OD_TORQUE_ACTUAL             (0x6077U) /* RO  I16  Torque feedback (per-mille rated) */
#define CIA402_OD_CURRENT_ACTUAL            (0x6078U) /* RO  I16  Current feedback (per-mille rated) */
#define CIA402_OD_DC_LINK_VOLTAGE           (0x6079U) /* RO  U32  DC bus voltage (mV) */
#define CIA402_OD_TARGET_POSITION           (0x607AU) /* RW  I32  Position setpoint (PP / CSP) */
#define CIA402_OD_TARGET_VELOCITY           (0x60FFU) /* RW  I32  Velocity setpoint */
#define CIA402_OD_QUICK_STOP_DECELERATION   (0x6085U) /* RW  U32  Quick-stop ramp rate */
#define CIA402_OD_SUPPORTED_DRIVE_MODES     (0x6502U) /* RO  U32  Bitmask of supported modes */

/*
    Controlword 0x6040 RW with the bit semantics above (master → drive command path)
    Statusword 0x6041 RO with the state-encoding bits above (drive → master state report)
    Modes of Operation 0x6060 / 0x6061 pair, with at least one supported mode declared in 0x6502
    The state machine itself — transitions match the table when control bits are written
    Quick stop, fault reaction, shutdown, disable, halt option codes (0x605A–E) — at minimum supported, even if only one value each
    Fault Reset edge detection on Controlword bit 7
    Cyclic update of Statusword (typ. ≤ PDO cycle time) so master sees state changes within one cycle
    Mandatory monitoring objects: Position actual (0x6064), Velocity actual (0x606C), Torque actual (0x6077), DC bus voltage (0x6079)
    EMCY message on fault entry per CiA 301 (CANopen base) with error code mapping per CiA 402
*/

typedef enum Cia402_OdType
{
    CIA402_OD_TYPE_NONE,
    CIA402_OD_TYPE_I8,
    CIA402_OD_TYPE_U8,
    CIA402_OD_TYPE_I16,
    CIA402_OD_TYPE_U16,
    CIA402_OD_TYPE_I32,
    CIA402_OD_TYPE_U32,
}
Cia402_OdType_T;

typedef enum Cia402_OdAccess
{
    CIA402_OD_ACCESS_NONE = 0U,
    CIA402_OD_ACCESS_RO   = 1U,
    CIA402_OD_ACCESS_WO   = 2U,
    CIA402_OD_ACCESS_RW   = 3U,
}
Cia402_OdAccess_T;

typedef struct Cia402_OdInfo
{
    Cia402_OdType_T   Type;
    Cia402_OdAccess_T Access;
    uint8_t           Size; /* in bytes */
}
Cia402_OdInfo_T;

extern Cia402_OdInfo_T Cia402_Od_GetInfo(uint16_t index, uint8_t subindex);


/* SDO abort codes per CiA 301 */
typedef enum Cia402_OdStatus
{
    CIA402_OD_OK                    = 0,
    CIA402_OD_ERR_TOGGLE_BIT        = (int)0x05030000, /* Toggle bit not alternated */
    CIA402_OD_ERR_TIMEOUT           = (int)0x05040000, /* SDO protocol timed out */
    CIA402_OD_ERR_INVALID_CCS       = (int)0x05040001, /* Client/server cmd specifier invalid */
    CIA402_OD_ERR_NO_OBJECT         = (int)0x06020000, /* Object does not exist */
    CIA402_OD_ERR_NOT_MAPPABLE      = (int)0x06040041, /* Object cannot be mapped to PDO */
    CIA402_OD_ERR_PDO_LENGTH        = (int)0x06040042, /* Mapped PDO length exceeds */
    CIA402_OD_ERR_GENERAL_PARAM     = (int)0x06040043, /* General parameter incompatibility */
    CIA402_OD_ERR_GENERAL_INTERNAL  = (int)0x06040047, /* General internal incompatibility */
    CIA402_OD_ERR_HARDWARE          = (int)0x06060000, /* Access failed due to hardware error */
    CIA402_OD_ERR_LENGTH_MISMATCH   = (int)0x06070010, /* Data type / length mismatch */
    CIA402_OD_ERR_LENGTH_HIGH       = (int)0x06070012, /* Length too high */
    CIA402_OD_ERR_LENGTH_LOW        = (int)0x06070013, /* Length too low */
    CIA402_OD_ERR_SUBINDEX          = (int)0x06090011, /* Subindex does not exist */
    CIA402_OD_ERR_READ_ONLY         = (int)0x06010002, /* Write to RO object */
    CIA402_OD_ERR_WRITE_ONLY        = (int)0x06010001, /* Read of WO object */
    CIA402_OD_ERR_VALUE_RANGE       = (int)0x06090030, /* Value out of range */
    CIA402_OD_ERR_VALUE_HIGH        = (int)0x06090031, /* Value too high */
    CIA402_OD_ERR_VALUE_LOW         = (int)0x06090032, /* Value too low */
    CIA402_OD_ERR_GENERAL           = (int)0x08000000, /* General error */
}
Cia402_OdStatus_T;


/******************************************************************************/
/*
    CAN packet parsing types (CiA 301 base)

    SDO frame layout (8-byte CAN payload):
      ┌────────┬─────────────┬──────────┬──────────────────────────┐
      │ Byte 0 │ Byte 1..2   │ Byte 3   │ Byte 4..7                │
      │ Cmd    │ Index (LE)  │ SubIdx   │ Data (LE, up to 4 bytes) │
      └────────┴─────────────┴──────────┴──────────────────────────┘

    PDO frame: 0..8 bytes of pre-mapped data — no header, layout is set
    via the PDO mapping objects (0x1A00..0x1603) at startup.

    COB-ID conventions for default connection set:
      SDO request   : 0x600 + nodeId   (master → slave)
      SDO response  : 0x580 + nodeId   (slave  → master)
      RxPDO1        : 0x200 + nodeId
      RxPDO2        : 0x300 + nodeId
      TxPDO1        : 0x180 + nodeId
      TxPDO2        : 0x280 + nodeId
      EMCY          : 0x080 + nodeId
*/
/******************************************************************************/
#define CIA402_COB_SDO_REQ_BASE     (0x600U)
#define CIA402_COB_SDO_RSP_BASE     (0x580U)
#define CIA402_COB_RXPDO1_BASE      (0x200U)
#define CIA402_COB_RXPDO2_BASE      (0x300U)
#define CIA402_COB_TXPDO1_BASE      (0x180U)
#define CIA402_COB_TXPDO2_BASE      (0x280U)
#define CIA402_COB_EMCY_BASE        (0x080U)

#define CIA402_COB_FUNCTION_MASK    (0x780U) /* upper 4 bits */
#define CIA402_COB_NODE_MASK        (0x07FU) /* lower 7 bits */

#define CIA402_COB_FUNCTION(cob)    ((cob) & CIA402_COB_FUNCTION_MASK)
#define CIA402_COB_NODE(cob)        ((cob) & CIA402_COB_NODE_MASK)

typedef enum Cia402_CobFunctionCode
{
    CIA402_COB_SDO_REQ   = 0x600U,
    CIA402_COB_SDO_RSP   = 0x580U,
    CIA402_COB_RXPDO1    = 0x200U,
    CIA402_COB_RXPDO2    = 0x300U,
    CIA402_COB_TXPDO1    = 0x180U,
    CIA402_COB_TXPDO2    = 0x280U,
    CIA402_COB_EMCY      = 0x080U,
}
Cia402_CobFunctionCode_T;

typedef struct Cia402_Cob
{
    uint16_t Node     : 7; /* lower 7 bits of COB-ID (node ID) */
    uint16_t Function : 4; /* upper 4 bits of COB-ID (function code) */
    uint16_t Reserved : 5; /* upper bits reserved, always 0 */
}
Cia402_Cob_T;

/*
    SDO Command Specifier — byte 0 of the SDO payload.
    GCC packs first-declared bitfield in LSB; layout below matches
    [bit7..5: ccs][bit4: rsv][bit3..2: n][bit1: e][bit0: s].
*/
typedef union Cia402_SdoCmd
{
    struct __attribute__((packed))
    {
        uint8_t Size      : 1; /* [0]  s    1 = data size indicated by N */
        uint8_t Expedited : 1; /* [1]  e    1 = data fits in bytes 4..7 */
        uint8_t N         : 2; /* [2:3] n   number of unused bytes in data field (0..3) */
        uint8_t Reserved  : 1; /* [4]       always 0 */
        uint8_t Ccs       : 3; /* [5:7] ccs command code (Cia402_SdoCcs_T) */
    };
    uint8_t Byte;
}
Cia402_SdoCmd_T;

/*
    Client/Server Command Specifier (CCS) — top 3 bits of byte 0.
    Distinguishes request kind (download = write, upload = read, etc).
*/
typedef enum Cia402_SdoCcs
{
    CIA402_SDO_CCS_DOWNLOAD_SEG_REQ     = 0U, /* segmented download request  (client → server) */
    CIA402_SDO_CCS_DOWNLOAD_INIT_REQ    = 1U, /* download initiate          (client → server) */
    CIA402_SDO_CCS_UPLOAD_INIT_REQ      = 2U, /* upload initiate            (client → server) */
    CIA402_SDO_CCS_UPLOAD_SEG_REQ       = 3U, /* segmented upload request   (client → server) */
    CIA402_SDO_CCS_ABORT                = 4U, /* abort transfer             (either direction) */
    CIA402_SDO_CCS_BLOCK_UPLOAD         = 5U, /* block upload               (either direction) */
    CIA402_SDO_CCS_BLOCK_DOWNLOAD       = 6U, /* block download             (either direction) */
    /* SCS (server response codes) reuse the same field — context distinguishes */
    CIA402_SDO_SCS_UPLOAD_INIT_RSP      = 2U, /* upload initiate response   (server → client) */
    CIA402_SDO_SCS_DOWNLOAD_INIT_RSP    = 3U, /* download initiate response (server → client) */
}
Cia402_SdoCcs_T;

/*
    SDO Data — bytes 4..7 of the SDO frame, viewed as the typed value it carries.
    Lets handlers read/write the value field by type without manual byte shifts:
        value = p_req->Data.I16;
        p_resp->Data.U32 = abortCode;
*/
typedef union Cia402_SdoData
{
    uint8_t  Bytes[4];
    int8_t   I8;
    uint8_t  U8;
    int16_t  I16;
    uint16_t U16;
    int32_t  I32;
    uint32_t U32;
    uint32_t AbortCode; /* for abort frames */
}
Cia402_SdoData_T;

/*
    Handles sign extension
*/
static int32_t Cia402_SdoData_Decode(Cia402_OdType_T type, Cia402_SdoData_T data)
{
    switch (type)
    {
        case CIA402_OD_TYPE_I8:  return (int32_t)data.I8;
        case CIA402_OD_TYPE_U8:  return (int32_t)data.U8;
        case CIA402_OD_TYPE_I16: return (int32_t)data.I16;
        case CIA402_OD_TYPE_U16: return data.U16;
        case CIA402_OD_TYPE_I32: return data.I32;
        case CIA402_OD_TYPE_U32: return data.U32;
        default:                 return 0;
    }
}

static Cia402_SdoData_T Cia402_SdoData_Encode(Cia402_OdType_T type, int32_t value)
{
    switch (type)
    {
        case CIA402_OD_TYPE_I8:  return (Cia402_SdoData_T) { .I8 = (int8_t)value };
        case CIA402_OD_TYPE_U8:  return (Cia402_SdoData_T) { .U8 = (uint8_t)value };
        case CIA402_OD_TYPE_I16: return (Cia402_SdoData_T) { .I16 = (int16_t)value };
        case CIA402_OD_TYPE_U16: return (Cia402_SdoData_T) { .U16 = (uint16_t)value };
        case CIA402_OD_TYPE_I32: return (Cia402_SdoData_T) { .I32 = value };
        case CIA402_OD_TYPE_U32: return (Cia402_SdoData_T) { .U32 = (uint32_t)value };
        default:                 return (Cia402_SdoData_T) { .U32 = 0 };
    }
}

/*
    SDO Frame — 8-byte CAN payload for SDO request and response.
    Fields are little-endian on the wire; the packed layout matches
    standard CiA 301 byte ordering.

    Use Data.<type> to read/write the value of the indexed object directly.
    For abort frames, Data.AbortCode holds the U32 abort reason.
*/
typedef union Cia402_Sdo
{
    struct __attribute__((packed))
    {
        Cia402_SdoCmd_T  Cmd;       /* byte 0     */
        uint16_t         Index;     /* bytes 1..2 little-endian */
        uint8_t          SubIndex;  /* byte 3     */
        Cia402_SdoData_T Data;      /* bytes 4..7 little-endian (typed) */
    };
    uint8_t Bytes[8];
}
Cia402_Sdo_T;

/* less than 2 registers */
static inline Cia402_Sdo_T Cia402_Sdo_EncodeAbort(uint16_t index, uint8_t subindex, Cia402_OdStatus_T abortCode)
{
    return (Cia402_Sdo_T) { .Cmd = { .Ccs = CIA402_SDO_CCS_ABORT, }, .Index = index, .SubIndex = subindex, .Data.AbortCode = (uint32_t)abortCode, };
}

static inline Cia402_Sdo_T Cia402_Sdo_EncodeDownloadAck(uint16_t index, uint8_t subindex)
{
    return (Cia402_Sdo_T) { .Cmd = { .Ccs = CIA402_SDO_SCS_DOWNLOAD_INIT_RSP }, .Index = index, .SubIndex = subindex };
}

static inline Cia402_Sdo_T Cia402_Sdo_EncodeUploadResponse(uint16_t index, uint8_t subindex, Cia402_OdInfo_T info, int32_t value)
{
    return (Cia402_Sdo_T)
    {
        .Cmd      = { .Ccs = CIA402_SDO_SCS_UPLOAD_INIT_RSP, .Expedited = 1U, .Size = 1U, .N = (uint8_t)(4U - info.Size) },
        .Index    = index,
        .SubIndex = subindex,
        .Data     = Cia402_SdoData_Encode(info.Type, value),
    };
}



/*
    Default PDO mappings per CiA 402 (predefined connection set).
    Each is a packed struct overlay over the PDO byte array.
    Mode-specific RxPDO/TxPDO variants share the same RxPDO/TxPDO COB-ID
    in the spec — pick the variant matching the active operating mode.

    RxPDO (master → drive)
*/
typedef struct __attribute__((packed)) Cia402_RxPdo_Cw
{
    Cia402_Control_T Controlword;
}
Cia402_RxPdo_Cw_T;

typedef struct __attribute__((packed)) Cia402_RxPdo_CwTorque
{
    Cia402_Control_T Controlword;
    int16_t          TargetTorque;
}
Cia402_RxPdo_CwTorque_T;

typedef struct __attribute__((packed)) Cia402_RxPdo_CwVelocity
{
    Cia402_Control_T Controlword;
    int32_t          TargetVelocity;
}
Cia402_RxPdo_CwVelocity_T;

typedef struct __attribute__((packed)) Cia402_RxPdo_CwPosition
{
    Cia402_Control_T Controlword;
    int32_t          TargetPosition;
}
Cia402_RxPdo_CwPosition_T;

/*  TxPDO (drive → master) */
typedef struct __attribute__((packed)) Cia402_TxPdo_Sw
{
    Cia402_Status_T Statusword;
}
Cia402_TxPdo_Sw_T;

typedef struct __attribute__((packed)) Cia402_TxPdo_SwTorque
{
    Cia402_Status_T Statusword;
    int16_t         TorqueActual;
}
Cia402_TxPdo_SwTorque_T;

typedef struct __attribute__((packed)) Cia402_TxPdo_SwVelocity
{
    Cia402_Status_T Statusword;
    int32_t         VelocityActual;
}
Cia402_TxPdo_SwVelocity_T;

typedef struct __attribute__((packed)) Cia402_TxPdo_SwPosition
{
    Cia402_Status_T Statusword;
    int32_t         PositionActual;
}
Cia402_TxPdo_SwPosition_T;


/*
    PDO Frame — up to 8 bytes of pre-mapped process data.
    Layout is set by the PDO mapping objects (0x1A00.., 0x1600..) at startup.
    Callers cast Bytes to typed PDO mapping structs below per the configured
    mapping for the COB-ID being received/transmitted.
*/
typedef union Cia402_Pdo
{
    uint8_t  Bytes[8];
}
Cia402_Pdo_T;

/******************************************************************************/
/*
    Adapter context — one per CiA 402 axis.

    Holds:
      - PrevControl       : previous Controlword for FaultReset rising-edge detection
      - ActiveMode        : current Modes of Operation (0x6060/0x6061)
      - QuickStopOption   : 0x605A
      - ShutdownOption    : 0x605B
      - DisableOpOption   : 0x605C
      - HaltOption        : 0x605D
      - FaultReactOption  : 0x605E
      - QuickStopDecel    : 0x6085
*/
/******************************************************************************/
typedef struct Cia402_Input
{
    Cia402_Control_T              PrevControl;
    Cia402_OpMode_T               ActiveMode;
}
Cia402_Input_T;

typedef struct Cia402_Config
{
    uint8_t                       NodeId;
    Cia402_QuickStopOption_T      QuickStopOption;
    Cia402_ShutdownOption_T       ShutdownOption;
    Cia402_DisableOpOption_T      DisableOpOption;
    Cia402_HaltOption_T           HaltOption;
    Cia402_FaultReactionOption_T  FaultReactOption;
    uint32_t                      QuickStopDecel;
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
    Object Dictionary metadata — common-layer entry point.

    Returns spec-fixed type / access / size for the given (index, subindex).
    All sizes are compile-time constants (sizeof of the wire type).
    For unknown entries, returns Type = CIA402_OD_TYPE_NONE.
*/
/******************************************************************************/
extern Cia402_OdInfo_T Cia402_Od_GetInfo(uint16_t index, uint8_t subindex);


/******************************************************************************/
/*
    Optional Interface
*/
/******************************************************************************/
/******************************************************************************/
/*
    Object Dictionary callback interface

    The protocol-layer SDO server is generic — it parses inbound frames and
    dispatches reads/writes through these callbacks. The application (e.g.
    Motor_Cia402) supplies the function pointers and a backing context.
*/
/******************************************************************************/
typedef Cia402_OdInfo_T(*Cia402_OdGetInfoFn_T)(void * p_context, uint16_t index, uint8_t subindex);
typedef Cia402_OdStatus_T(*Cia402_OdGetFn_T)  (void * p_context, uint16_t index, uint8_t subindex, int32_t * p_value);
typedef Cia402_OdStatus_T(*Cia402_OdSetFn_T)  (void * p_context, uint16_t index, uint8_t subindex, int32_t value);

typedef const struct Cia402_OdInterface
{
    void * p_Context;
    Cia402_OdGetInfoFn_T GetInfo;
    Cia402_OdGetFn_T Get;
    Cia402_OdSetFn_T Set;
}
Cia402_OdInterface_T;

extern uint8_t Cia402_Sdo_HandleRequest(const Cia402_OdInterface_T * p_od, const Cia402_Sdo_T * p_req, Cia402_Sdo_T * p_rsp);
extern void Cia402_Pdo_HandleRx(const Cia402_OdInterface_T * p_od, const Cia402_Adapter_T * p_adapter, uint16_t cob_id, const Cia402_Pdo_T * p_pdo, uint8_t dlc);

// /*
//     Table Entry
// */
// /* Generic accessor — pulls a typed value from a raw byte pointer */
// typedef struct Cia402_OdEntry
// {
//     Cia402_OdMeta_T Meta;
//     uint16_t          AdapterOffset; /* offsetof(Motor_Cia402_T, ...) — 0xFFFF if not adapter-backed */
//     /* For non-adapter-backed entries, fall back to the function-pointer shape */
//     int32_t(*Get)(const void *, const Cia402_Adapter_T *);
//     Cia402_OdStatus_T(*Set)(const void *, Cia402_Adapter_T *, int32_t);
// }
// Cia402_OdEntry_T;

// #define OD_ADAPTER(idx, sub, ty, acc, field) \
//     { (idx), (sub), (ty), (acc), offsetof(Motor_Cia402_T, field), NULL, NULL }

// #define OD_FN(idx, sub, ty, acc, get_fn, set_fn) \
//     { (idx), (sub), (ty), (acc), 0xFFFFU, (get_fn), (set_fn) }