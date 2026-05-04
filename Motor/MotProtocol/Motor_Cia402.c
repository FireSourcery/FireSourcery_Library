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
    @file   Motor_Cia402.c
    @author FireSourcery
    @brief  CiA 402 adapter implementation. Maps Controlword/Statusword and
            the mandatory object dictionary onto Motor_User / Motor_StateMachine,
            and parses inbound SDO / RxPDO frames into adapter actions.
*/
/******************************************************************************/
#include "Motor_Cia402.h"

#include "Motor/Motor/Motor_User.h"
#include "Motor/Motor/Motor_StateMachine.h"


/*
    Modes this driver actually supports — declare in object 0x6502.
    Update as Cia402-side operating modes are wired through.
*/
#define MOTOR_CIA402_SUPPORTED_DRIVE_MODES  (CIA402_SUPPORTED_PV | CIA402_SUPPORTED_TQ)

/* todo */
static inline Cia402_Adapter_T * Motor_Cia402_Adapter(Motor_T * p_motor) { return NULL; }


/******************************************************************************/
/*
    Controlword → Motor SM input
*/
/******************************************************************************/
/*
    Cyclic write — call every PDO cycle with the latest Controlword.
    Decodes the canonical command and detects the FaultReset rising edge
    against the previous value held in the adapter.
*/
static inline void Motor_Cia402_WriteControl(Motor_T * p_motor, Cia402_Adapter_T * p_adapter, Cia402_Control_T control)
{
    Cia402_Control_T prev = p_adapter->Input.PrevControl;

    switch (Cia402_DecodeControlCmd(control))
    {
        case CIA402_CMD_DISABLE_VOLTAGE:        Motor_Disable(p_motor);             break;
        case CIA402_CMD_SHUTDOWN:               Motor_ReleaseVZ(p_motor);           break;
        case CIA402_CMD_SWITCH_ON:              Motor_ReleaseV0(p_motor);           break;
        case CIA402_CMD_ENABLE_OPERATION:       Motor_ActivateControl(p_motor);     break;
        case CIA402_CMD_QUICK_STOP:             /* Motor_Disable(p_motor);  */      break;
        case CIA402_CMD_FAULT_RESET:
            if (Cia402_IsFaultResetEdge(prev, control) == true) { Motor_StateMachine_TryClearFaultAll(p_motor); }
            break;
        default:
            break;
    }
    p_adapter->Input.PrevControl = control;
}


/******************************************************************************/
/*
    Motor SM state → Statusword
*/
/******************************************************************************/
/*
    Build the cyclic Statusword from current Motor state.
    Uses the canonical state-encoding bits plus a few orthogonal flags:
      - VoltageEnabled  ← always 1 once past INIT (no DC contactor model here)
      - Remote          ← always 1 (SM honors inputs)
      - Warning         ← currently always 0 (hook a warning-flag accessor here)
      - Manufacturer0   ← 1 for OPEN_LOOP / CALIBRATION marker

    PASSIVE splits into RTSO (Phase=Z) and SO (Phase=V0). Read live phase output to disambiguate.
*/
static inline Cia402_Status_T Motor_Cia402_ReadStatus(Motor_T * p_motor)
{
    Cia402_Status_T status = { .Word = 0U };
    Motor_StateId_T id = Motor_GetStateId(p_motor->P_MOTOR);

    switch (id)
    {
        case MOTOR_STATE_ID_INIT:           status.Word = CIA402_STATE_NOT_READY_TO_SWITCH_ON;  break;
        case MOTOR_STATE_ID_DISABLED:       status.Word = CIA402_STATE_SWITCH_ON_DISABLED;      break;
        case MOTOR_STATE_ID_PASSIVE:        status.Word = (Motor_GetPhaseState(p_motor) == PHASE_VOUT_0) ? CIA402_STATE_SWITCHED_ON : CIA402_STATE_READY_TO_SWITCH_ON; break;
        case MOTOR_STATE_ID_RUN:            status.Word = CIA402_STATE_OPERATION_ENABLED;       break;
        case MOTOR_STATE_ID_INTERVENTION:   status.Word = CIA402_STATE_QUICK_STOP_ACTIVE;       break;
        case MOTOR_STATE_ID_OPEN_LOOP:      status.Word = CIA402_STATE_OPERATION_ENABLED;
                                            status.Manufacturer0 = 1U;                           break;
        case MOTOR_STATE_ID_CALIBRATION:    status.Word = CIA402_STATE_SWITCH_ON_DISABLED;
                                            status.Manufacturer0 = 1U;                           break;
        case MOTOR_STATE_ID_FAULT:          status.Word = CIA402_STATE_FAULT;                   break;
        default:                            status.Word = CIA402_STATE_NOT_READY_TO_SWITCH_ON;  break;
    }

    if (id != MOTOR_STATE_ID_INIT) { status.VoltageEnabled = 1U; }
    status.Remote = 1U;

    return status;
}



/******************************************************************************/
/*
    Object Dictionary
*/
/******************************************************************************/
Cia402_OdStatus_T Motor_Cia402_Od_Get(Motor_T * p_motor, const Cia402_Adapter_T * p_adapter, uint16_t index, uint8_t subindex, int32_t * p_value)
{
    Cia402_OdInfo_T info = Cia402_Od_GetInfo(index, subindex);

    if (info.Type   == CIA402_OD_TYPE_NONE)   { return (subindex != 0U) ? CIA402_OD_ERR_SUBINDEX : CIA402_OD_ERR_NO_OBJECT; }
    if (info.Access == CIA402_OD_ACCESS_WO)   { return CIA402_OD_ERR_WRITE_ONLY; }

    switch (index)
    {
        case CIA402_OD_CONTROLWORD:             *p_value = p_adapter->Input.PrevControl.Word;                                       break;
        case CIA402_OD_STATUSWORD:              *p_value = Motor_Cia402_ReadStatus(p_motor).Word;                                   break;
        case CIA402_OD_QUICK_STOP_OPTION_CODE:  *p_value = (int16_t)p_adapter->Config.QuickStopOption;                              break;
        case CIA402_OD_SHUTDOWN_OPTION_CODE:    *p_value = (int16_t)p_adapter->Config.ShutdownOption;                               break;
        case CIA402_OD_DISABLE_OP_OPTION_CODE:  *p_value = (int16_t)p_adapter->Config.DisableOpOption;                              break;
        case CIA402_OD_HALT_OPTION_CODE:        *p_value = (int16_t)p_adapter->Config.HaltOption;                                   break;
        case CIA402_OD_FAULT_REACTION_CODE:     *p_value = (int16_t)p_adapter->Config.FaultReactOption;                             break;
        case CIA402_OD_MODES_OF_OPERATION:      *p_value = (int8_t)p_adapter->Input.ActiveMode;                                     break;
        case CIA402_OD_MODES_OF_OPERATION_DISP: *p_value = (int8_t)p_adapter->Input.ActiveMode;                                     break;
        case CIA402_OD_POSITION_ACTUAL:         *p_value = (int32_t)RotorSensor_GetMechanicalAngle(p_motor->P_MOTOR->p_ActiveSensor); break;
        case CIA402_OD_VELOCITY_ACTUAL:         *p_value = (int32_t)Motor_GetSpeed_Fract16(p_motor->P_MOTOR);                       break;
        case CIA402_OD_TARGET_TORQUE:           *p_value = (int16_t)_Motor_GetTorqueSetpoint(p_motor->P_MOTOR);                     break;
        case CIA402_OD_TORQUE_ACTUAL:           *p_value = (int16_t)p_motor->P_MOTOR->Foc.Iq;                                       break;
        case CIA402_OD_CURRENT_ACTUAL:          *p_value = (int16_t)Motor_GetIPhase_Fract16(p_motor->P_MOTOR);                      break;
        case CIA402_OD_DC_LINK_VOLTAGE:         *p_value = (int32_t)Phase_VBus_Fract16();                                           break;
        case CIA402_OD_TARGET_VELOCITY:         *p_value = (int32_t)Motor_GetSpeedSetpoint(p_motor->P_MOTOR);                       break;
        case CIA402_OD_QUICK_STOP_DECELERATION: *p_value = (int32_t)p_adapter->Config.QuickStopDecel;                               break;
        case CIA402_OD_SUPPORTED_DRIVE_MODES:   *p_value = (int32_t)MOTOR_CIA402_SUPPORTED_DRIVE_MODES;                             break;
        default:                                return CIA402_OD_ERR_NO_OBJECT;
    }

    return CIA402_OD_OK;
}


Cia402_OdStatus_T Motor_Cia402_Od_Set(Motor_T * p_motor, Cia402_Adapter_T * p_adapter, uint16_t index, uint8_t subindex, int32_t value)
{
    Cia402_OdInfo_T info = Cia402_Od_GetInfo(index, subindex);
    if (info.Type   == CIA402_OD_TYPE_NONE)   { return (subindex != 0U) ? CIA402_OD_ERR_SUBINDEX : CIA402_OD_ERR_NO_OBJECT; }
    if (info.Access == CIA402_OD_ACCESS_RO)   { return CIA402_OD_ERR_READ_ONLY; }

    switch (index)
    {
        case CIA402_OD_CONTROLWORD:             Motor_Cia402_WriteControl(p_motor, p_adapter, (Cia402_Control_T){ .Word = (uint16_t)value });            break;
        case CIA402_OD_MODES_OF_OPERATION:      p_adapter->Input.ActiveMode = (Cia402_OpMode_T)(int8_t)value;             break; /* apply feedback mode */
        case CIA402_OD_QUICK_STOP_OPTION_CODE:  p_adapter->Config.QuickStopOption  = (Cia402_QuickStopOption_T)value;     break;
        case CIA402_OD_SHUTDOWN_OPTION_CODE:    p_adapter->Config.ShutdownOption   = (Cia402_ShutdownOption_T)value;      break;
        case CIA402_OD_DISABLE_OP_OPTION_CODE:  p_adapter->Config.DisableOpOption  = (Cia402_DisableOpOption_T)value;     break;
        case CIA402_OD_HALT_OPTION_CODE:        p_adapter->Config.HaltOption       = (Cia402_HaltOption_T)value;          break;
        case CIA402_OD_FAULT_REACTION_CODE:     p_adapter->Config.FaultReactOption = (Cia402_FaultReactionOption_T)value; break;
        case CIA402_OD_TARGET_TORQUE:           Motor_SetTorqueCmd(p_motor->P_MOTOR, (int16_t)value);            break;
        case CIA402_OD_TARGET_VELOCITY:         Motor_SetSpeedCmd(p_motor->P_MOTOR, (int16_t)value);             break;
        case CIA402_OD_QUICK_STOP_DECELERATION:            p_adapter->Config.QuickStopDecel = (uint32_t)value;            break;
        default:            return CIA402_OD_ERR_NO_OBJECT;
    }

    return CIA402_OD_OK;
}


/******************************************************************************/
/*
    Combined-context handle — one per CiA 402 axis.

    Bundles the two pointers the OD callbacks need (Motor_T for sensor /
    setpoint access, Cia402_Adapter_T for state and config). Lives on the
    caller's stack inside Motor_Cia402_HandleSdo, then is bound via
    Cia402_OdInterface_T.p_Context for the lifetime of one SDO transaction.
*/
/******************************************************************************/
typedef const struct Motor_Cia402_Ctx
{
    Motor_T * p_Motor;
    Cia402_Adapter_T * p_Adapter;
}
Motor_Cia402_Ctx_T;

static Cia402_OdInfo_T Od_GetInfo(Motor_Cia402_Ctx_T * p_ctx, uint16_t index, uint8_t subindex)
{
    (void)p_ctx;
    return Cia402_Od_GetInfo(index, subindex);
}

static Cia402_OdStatus_T Od_Get(Motor_Cia402_Ctx_T * p_ctx, uint16_t index, uint8_t subindex, int32_t * p_value)
{
    return Motor_Cia402_Od_Get(p_ctx->p_Motor, p_ctx->p_Adapter, index, subindex, p_value);
}

static Cia402_OdStatus_T Od_Set(Motor_Cia402_Ctx_T * p_ctx, uint16_t index, uint8_t subindex, int32_t value)
{
    return Motor_Cia402_Od_Set(p_ctx->p_Motor, p_ctx->p_Adapter, index, subindex, value);
}

/*
    Build the OD interface for one in-flight CAN transaction.
    Bound to a stack-allocated Motor_Cia402_Ctx_T owned by the caller.
*/
static inline Cia402_OdInterface_T _OdInterface(Motor_Cia402_Ctx_T * p_ctx)
{
    return (Cia402_OdInterface_T)
    {
        .p_Context = p_ctx,
        .GetInfo = (Cia402_OdGetInfoFn_T)Od_GetInfo,
        .Get = (Cia402_OdGetFn_T)Od_Get,
        .Set = (Cia402_OdSetFn_T)Od_Set,
    };
}


/******************************************************************************/
/*
    Statusword build-side helpers (TxPDO producers)
    Caller picks the variant matching the configured TxPDO mapping.
*/
/******************************************************************************/
void Motor_Cia402_BuildTxPdo_Sw(Motor_T * p_motor, Cia402_TxPdo_Sw_T * p_pdo)
{
    p_pdo->Statusword = Motor_Cia402_ReadStatus(p_motor);
}

void Motor_Cia402_BuildTxPdo_SwVelocity(Motor_T * p_motor, Cia402_TxPdo_SwVelocity_T * p_pdo)
{
    p_pdo->Statusword     = Motor_Cia402_ReadStatus(p_motor);
    p_pdo->VelocityActual = (int32_t)Motor_GetSpeed_Fract16(p_motor->P_MOTOR);
}

void Motor_Cia402_BuildTxPdo_SwTorque(Motor_T * p_motor, Cia402_TxPdo_SwTorque_T * p_pdo)
{
    p_pdo->Statusword   = Motor_Cia402_ReadStatus(p_motor);
    p_pdo->TorqueActual = (int16_t)p_motor->P_MOTOR->Foc.Iq;
}

void Motor_Cia402_BuildTxPdo_SwPosition(Motor_T * p_motor, Cia402_TxPdo_SwPosition_T * p_pdo)
{
    p_pdo->Statusword     = Motor_Cia402_ReadStatus(p_motor);
    p_pdo->PositionActual = (int32_t)RotorSensor_GetMechanicalAngle(p_motor->P_MOTOR->p_ActiveSensor);
}


/******************************************************************************/
/*
    RxPDO consumers — apply pre-mapped process data.

    Each variant overlays the typed struct on the inbound PDO bytes; no
    manual parsing. The caller selects the variant by COB-ID per the
    PDO mapping configured at startup (objects 0x1600..0x1603).

    Setpoint is applied BEFORE Controlword so an ENABLE_OPERATION transition
    consumes the new target rather than a stale one.
*/
/******************************************************************************/
void Motor_Cia402_HandleRxPdo_Cw(Motor_T * p_motor, Cia402_Adapter_T * p_adapter, const Cia402_RxPdo_Cw_T * p_pdo)
{
    Motor_Cia402_WriteControl(p_motor, p_adapter, p_pdo->Controlword);
}

void Motor_Cia402_HandleRxPdo_CwTorque(Motor_T * p_motor, Cia402_Adapter_T * p_adapter, const Cia402_RxPdo_CwTorque_T * p_pdo)
{
    Motor_SetTorqueCmd(p_motor->P_MOTOR, p_pdo->TargetTorque);
    Motor_Cia402_WriteControl(p_motor, p_adapter, p_pdo->Controlword);
}

void Motor_Cia402_HandleRxPdo_CwVelocity(Motor_T * p_motor, Cia402_Adapter_T * p_adapter, const Cia402_RxPdo_CwVelocity_T * p_pdo)
{
    Motor_SetSpeedCmd(p_motor->P_MOTOR, (int16_t)p_pdo->TargetVelocity);
    Motor_Cia402_WriteControl(p_motor, p_adapter, p_pdo->Controlword);
}

void Motor_Cia402_HandleRxPdo_CwPosition(Motor_T * p_motor, Cia402_Adapter_T * p_adapter, const Cia402_RxPdo_CwPosition_T * p_pdo)
{
    /* Motor_SetPositionCmd not yet wired through. Controlword still applied. */
    /* Motor_SetPositionCmd(p_motor->P_MOTOR, (uint16_t)p_pdo->TargetPosition); */
    Motor_Cia402_WriteControl(p_motor, p_adapter, p_pdo->Controlword);
}


/******************************************************************************/
/*
    SDO server — delegate to the protocol-layer engine.

    The generic engine (Cia402_Sdo_HandleRequest) parses the inbound frame,
    decodes the CCS, looks up OD metadata, and fills the response. It is
    bound to this drive's OD via three callback functions wrapped in a
    Cia402_OdInterface_T. The integration layer's only job is to construct
    that interface and invoke the engine.

    Single-context contract: the engine carries one void* per axis. We pass
    the Motor_T pointer; the callbacks resolve the adapter via
    Motor_Cia402_Adapter().
*/
/******************************************************************************/
bool Motor_Cia402_HandleSdo(Motor_T * p_motor, Cia402_Adapter_T * p_adapter, const Cia402_Sdo_T * p_req, Cia402_Sdo_T * p_resp)
{
    Motor_Cia402_Ctx_T ctx = { .p_Motor = (Motor_T *)p_motor, .p_Adapter = p_adapter };
    const Cia402_OdInterface_T od = _OdInterface(&ctx);
    return Cia402_Sdo_HandleRequest(&od, p_req, p_resp) != 0U;
}

/******************************************************************************/
/*
    TxPDO build dispatchers
*/
/******************************************************************************/
// size_t Motor_Cia402_BuildTxPdo1(Motor_T * p_motor, Cia402_Adapter_T * p_adapter, uint32_t * p_id, uint8_t * p_resp)
// {
//     Motor_Cia402_BuildTxPdo_Sw(p_motor, (Cia402_TxPdo_Sw_T *)p_resp);
//     SetCanFrameId(p_tx, CIA402_COB_TXPDO1_BASE | p_adapter->Config.NodeId);
//     p_tx->DataLength = (uint8_t)sizeof(Cia402_TxPdo_Sw_T);
// }

// void Motor_Cia402_BuildTxPdo2(Motor_T * p_motor, Cia402_Adapter_T * p_adapter, CAN_Frame_T * p_tx)
// {

//     SetCanFrameId(p_tx, CIA402_COB_TXPDO2_BASE | p_adapter->Config.NodeId);

//     switch (p_adapter->Input.ActiveMode)
//     {
//         case CIA402_MODE_PROFILE_TORQUE:
//         case CIA402_MODE_CYCLIC_SYNC_TORQUE:
//             Motor_Cia402_BuildTxPdo_SwTorque(p_motor, (Cia402_TxPdo_SwTorque_T *)p_tx->Data);
//             p_tx->DataLength = (uint8_t)sizeof(Cia402_TxPdo_SwTorque_T);
//             break;
//         case CIA402_MODE_VELOCITY:
//         case CIA402_MODE_PROFILE_VELOCITY:
//         case CIA402_MODE_CYCLIC_SYNC_VELOCITY:
//             Motor_Cia402_BuildTxPdo_SwVelocity(p_motor, (Cia402_TxPdo_SwVelocity_T *)p_tx->Data);
//             p_tx->DataLength = (uint8_t)sizeof(Cia402_TxPdo_SwVelocity_T);
//             break;
//         case CIA402_MODE_PROFILE_POSITION:
//         case CIA402_MODE_CYCLIC_SYNC_POSITION:
//             Motor_Cia402_BuildTxPdo_SwPosition(p_motor, (Cia402_TxPdo_SwPosition_T *)p_tx->Data);
//             p_tx->DataLength = (uint8_t)sizeof(Cia402_TxPdo_SwPosition_T);
//             break;
//         default:
//             Motor_Cia402_BuildTxPdo_Sw(p_motor, (Cia402_TxPdo_Sw_T *)p_tx->Data);
//             p_tx->DataLength = (uint8_t)sizeof(Cia402_TxPdo_Sw_T);
//             break;
//     }
// }



// bool Motor_Cia402_HandleCanRx(const Motor_T * p_motor, const CAN_Frame_T * p_rx, CAN_Frame_T * p_tx)
// {
//     Cia402_Adapter_T * p_adapter = Motor_Cia402_Adapter((Motor_T *)p_motor);

//     if (CIA402_COB_NODE(p_rx->CanId.Id) != p_adapter->Config.NodeId) { return false; }

//     Motor_Cia402_Ctx_T ctx = { .p_Motor = (Motor_T *)p_motor, .p_Adapter = p_adapter };
//     const Cia402_OdInterface_T od = _OdInterface(&ctx);

//     switch (CIA402_COB_FUNCTION(p_rx->CanId.Id))
//     {
//         case CIA402_COB_RXPDO1_BASE: /* 0x200 — Controlword only */
//         case CIA402_COB_RXPDO2_BASE: /* 0x300 — mode-aware Controlword + setpoint */
//             Cia402_Pdo_HandleRx(&od, p_adapter, p_rx->CanId.Id, (const Cia402_Pdo_T *)p_rx->Data, p_rx->DataLength);
//             return false;

//         case CIA402_COB_SDO_REQ_BASE: /* 0x600 — SDO download/upload request */
//             {
//                 const Cia402_Sdo_T * p_req = (const Cia402_Sdo_T *)p_rx->Data;
//                 Cia402_Sdo_T * p_resp = (Cia402_Sdo_T *)p_tx->Data;
//                 if (Cia402_Sdo_HandleRequest(&od, p_req, p_resp) != 0U)
//                 {
//                     p_tx->CanId.Id = (uint32_t)(CIA402_COB_SDO_RSP_BASE | p_adapter->Config.NodeId);
//                     p_tx->CanId.Eff = 0U;
//                     p_tx->DataLength = 8U;
//                     return true;
//                 }
//                 return false;
//             }

//         default:
//             /* Not consumed by this drive (NMT, SYNC, EMCY, our own TxPDOs, etc.) */
//             return false;
//     }
// }


// /* likely optimized over 2 virtual calls */
// bool Motor_Cia402_HandleCanRx(const Motor_T * p_motor, const CAN_Frame_T * p_rx, CAN_Frame_T * p_tx)
// {
//     if (CIA402_COB_NODE(p_rx->CobId) != nodeId) { return false; }

//     switch (CIA402_COB_FUNCTION(p_rx->CobId))
//     {
//         case CIA402_COB_RXPDO1_BASE: /* 0x200 — Controlword only */
//             Motor_Cia402_HandleRxPdo_Cw(p_motor, p_adapter, (const Cia402_RxPdo_Cw_T *)p_rx->Data);
//             return false;

//         case CIA402_COB_RXPDO2_BASE: /* 0x300 — Controlword + setpoint, layout depends on mode */
//             switch (p_adapter->Input.ActiveMode)
//             {
//                 case CIA402_MODE_PROFILE_TORQUE:
//                 case CIA402_MODE_CYCLIC_SYNC_TORQUE:
//                     Motor_Cia402_HandleRxPdo_CwTorque(p_motor, p_adapter, (const Cia402_RxPdo_CwTorque_T *)p_rx->Data);
//                     break;
//                 case CIA402_MODE_VELOCITY:
//                 case CIA402_MODE_PROFILE_VELOCITY:
//                 case CIA402_MODE_CYCLIC_SYNC_VELOCITY:
//                     Motor_Cia402_HandleRxPdo_CwVelocity(p_motor, p_adapter, (const Cia402_RxPdo_CwVelocity_T *)p_rx->Data);
//                     break;
//                 case CIA402_MODE_PROFILE_POSITION:
//                 case CIA402_MODE_CYCLIC_SYNC_POSITION:
//                     Motor_Cia402_HandleRxPdo_CwPosition(p_motor, p_adapter, (const Cia402_RxPdo_CwPosition_T *)p_rx->Data);
//                     break;
//                 default:
//                     /* No setpoint mapping for current mode — fall back to Controlword-only */
//                     Motor_Cia402_HandleRxPdo_Cw(p_motor, p_adapter, (const Cia402_RxPdo_Cw_T *)p_rx->Data);
//                     break;
//             }
//             return false;

//         case CIA402_COB_SDO_REQ_BASE: /* 0x600 — SDO download/upload request */
//             {
//                 const Cia402_Sdo_T * p_req = (const Cia402_Sdo_T *)p_rx->Data;
//                 Cia402_Sdo_T * p_resp = (Cia402_Sdo_T *)p_tx->Data;
//                 if (Motor_Cia402_HandleSdo(p_motor, p_adapter, p_req, p_resp) == true)
//                 {
//                     p_tx->CobId = (uint16_t)(CIA402_COB_SDO_RSP_BASE | nodeId);
//                     p_tx->Dlc = 8U;
//                     return true;
//                 }
//                 return false;
//             }

//         default:
//             /* Not consumed by this drive (NMT, SYNC, EMCY, our own TxPDOs, etc.) */
//             return false;
//     }
// }


/* Motor_Cia402_Od.c */

/* ---- Per-entry accessors ---- */

// static int32_t OdGet_Controlword(Motor_T * p, const Motor_Cia402_T * a)        { (void)p; return a->PrevControl.Word; }
// static int32_t OdGet_Statusword (Motor_T * p, const Motor_Cia402_T * a)        { (void)a; return Motor_Cia402_ReadStatus(p).Word; }
// static int32_t OdGet_Modes      (Motor_T * p, const Motor_Cia402_T * a)        { (void)p; return (int8_t)a->ActiveMode; }
// static int32_t OdGet_PosActual  (Motor_T * p, const Motor_Cia402_T * a)        { (void)a; return RotorSensor_GetMechanicalAngle(p->P_MOTOR->p_ActiveSensor); }
// static int32_t OdGet_VelActual  (Motor_T * p, const Motor_Cia402_T * a)        { (void)a; return Motor_GetSpeed_Fract16(p->P_MOTOR); }
// static int32_t OdGet_TorqueTgt  (Motor_T * p, const Motor_Cia402_T * a)        { (void)a; return _Motor_GetTorqueSetpoint(p->P_MOTOR); }
// static int32_t OdGet_TorqueAct  (Motor_T * p, const Motor_Cia402_T * a)        { (void)a; return p->P_MOTOR->Foc.Iq; }
// static int32_t OdGet_CurrentAct (Motor_T * p, const Motor_Cia402_T * a)        { (void)a; return Motor_GetIPhase_Fract16(p->P_MOTOR); }
// static int32_t OdGet_VBus       (Motor_T * p, const Motor_Cia402_T * a)        { (void)p; (void)a; return Phase_VBus_Fract16(); }
// static int32_t OdGet_VelTarget  (Motor_T * p, const Motor_Cia402_T * a)        { (void)a; return Motor_GetSpeedSetpoint(p->P_MOTOR); }
// static int32_t OdGet_QsDecel    (Motor_T * p, const Motor_Cia402_T * a)        { (void)p; return (int32_t)a->QuickStopDecel; }
// static int32_t OdGet_Supported  (Motor_T * p, const Motor_Cia402_T * a)        { (void)p; (void)a; return MOTOR_CIA402_SUPPORTED_DRIVE_MODES; }
// static int32_t OdGet_QsOption   (Motor_T * p, const Motor_Cia402_T * a)        { (void)p; return a->QuickStopOption; }
// static int32_t OdGet_ShdnOption (Motor_T * p, const Motor_Cia402_T * a)        { (void)p; return a->ShutdownOption; }
// static int32_t OdGet_DisOption  (Motor_T * p, const Motor_Cia402_T * a)        { (void)p; return a->DisableOpOption; }
// static int32_t OdGet_HaltOption (Motor_T * p, const Motor_Cia402_T * a)        { (void)p; return a->HaltOption; }
// static int32_t OdGet_FltReact   (Motor_T * p, const Motor_Cia402_T * a)        { (void)p; return a->FaultReactOption; }

// static Cia402_OdStatus_T OdSet_Controlword(Motor_T * p, Motor_Cia402_T * a, int32_t v) { Motor_Cia402_WriteControl(a, p, (Cia402_Control_T){ .Word = (uint16_t)v }); return CIA402_OD_OK; }
// static Cia402_OdStatus_T OdSet_Modes      (Motor_T * p, Motor_Cia402_T * a, int32_t v) { (void)p; Motor_Cia402_WriteOpMode(a, (Cia402_OpMode_T)(int8_t)v); return CIA402_OD_OK; }
// static Cia402_OdStatus_T OdSet_TorqueTgt  (Motor_T * p, Motor_Cia402_T * a, int32_t v) { (void)a; Motor_SetTorqueCmd(p->P_MOTOR, (int16_t)v); return CIA402_OD_OK; }
// static Cia402_OdStatus_T OdSet_VelTarget  (Motor_T * p, Motor_Cia402_T * a, int32_t v) { (void)a; Motor_SetSpeedCmd  (p->P_MOTOR, (int16_t)v); return CIA402_OD_OK; }
// static Cia402_OdStatus_T OdSet_QsDecel    (Motor_T * p, Motor_Cia402_T * a, int32_t v) { (void)p; a->QuickStopDecel  = (uint32_t)v;            return CIA402_OD_OK; }
// static Cia402_OdStatus_T OdSet_QsOption   (Motor_T * p, Motor_Cia402_T * a, int32_t v) { (void)p; a->QuickStopOption = (Cia402_QuickStopOption_T)v;     return CIA402_OD_OK; }
// static Cia402_OdStatus_T OdSet_ShdnOption (Motor_T * p, Motor_Cia402_T * a, int32_t v) { (void)p; a->ShutdownOption  = (Cia402_ShutdownOption_T)v;      return CIA402_OD_OK; }
// static Cia402_OdStatus_T OdSet_DisOption  (Motor_T * p, Motor_Cia402_T * a, int32_t v) { (void)p; a->DisableOpOption = (Cia402_DisableOpOption_T)v;     return CIA402_OD_OK; }
// static Cia402_OdStatus_T OdSet_HaltOption (Motor_T * p, Motor_Cia402_T * a, int32_t v) { (void)p; a->HaltOption      = (Cia402_HaltOption_T)v;          return CIA402_OD_OK; }
// static Cia402_OdStatus_T OdSet_FltReact   (Motor_T * p, Motor_Cia402_T * a, int32_t v) { (void)p; a->FaultReactOption= (Cia402_FaultReactionOption_T)v; return CIA402_OD_OK; }


// /* ---- The table (sorted by Index for binary search) ---- */

// const Cia402_OdEntry_T MOTOR_CIA402_OD_TABLE[] =
// {
//     { 0x6040, 0, CIA402_OD_TYPE_U16, CIA402_OD_ACCESS_RW, OdGet_Controlword, OdSet_Controlword },
//     { 0x6041, 0, CIA402_OD_TYPE_U16, CIA402_OD_ACCESS_RO, OdGet_Statusword,  NULL              },
//     { 0x605A, 0, CIA402_OD_TYPE_I16, CIA402_OD_ACCESS_RW, OdGet_QsOption,    OdSet_QsOption    },
//     { 0x605B, 0, CIA402_OD_TYPE_I16, CIA402_OD_ACCESS_RW, OdGet_ShdnOption,  OdSet_ShdnOption  },
//     { 0x605C, 0, CIA402_OD_TYPE_I16, CIA402_OD_ACCESS_RW, OdGet_DisOption,   OdSet_DisOption   },
//     { 0x605D, 0, CIA402_OD_TYPE_I16, CIA402_OD_ACCESS_RW, OdGet_HaltOption,  OdSet_HaltOption  },
//     { 0x605E, 0, CIA402_OD_TYPE_I16, CIA402_OD_ACCESS_RW, OdGet_FltReact,    OdSet_FltReact    },
//     { 0x6060, 0, CIA402_OD_TYPE_I8,  CIA402_OD_ACCESS_RW, OdGet_Modes,       OdSet_Modes       },
//     { 0x6061, 0, CIA402_OD_TYPE_I8,  CIA402_OD_ACCESS_RO, OdGet_Modes,       NULL              },
//     { 0x6064, 0, CIA402_OD_TYPE_I32, CIA402_OD_ACCESS_RO, OdGet_PosActual,   NULL              },
//     { 0x606C, 0, CIA402_OD_TYPE_I32, CIA402_OD_ACCESS_RO, OdGet_VelActual,   NULL              },
//     { 0x6071, 0, CIA402_OD_TYPE_I16, CIA402_OD_ACCESS_RW, OdGet_TorqueTgt,   OdSet_TorqueTgt   },
//     { 0x6077, 0, CIA402_OD_TYPE_I16, CIA402_OD_ACCESS_RO, OdGet_TorqueAct,   NULL              },
//     { 0x6078, 0, CIA402_OD_TYPE_I16, CIA402_OD_ACCESS_RO, OdGet_CurrentAct,  NULL              },
//     { 0x6079, 0, CIA402_OD_TYPE_U32, CIA402_OD_ACCESS_RO, OdGet_VBus,        NULL              },
//     { 0x6085, 0, CIA402_OD_TYPE_U32, CIA402_OD_ACCESS_RW, OdGet_QsDecel,     OdSet_QsDecel     },
//     { 0x60FF, 0, CIA402_OD_TYPE_I32, CIA402_OD_ACCESS_RW, OdGet_VelTarget,   OdSet_VelTarget   },
//     { 0x6502, 0, CIA402_OD_TYPE_U32, CIA402_OD_ACCESS_RO, OdGet_Supported,   NULL              },
// };

// const uint16_t MOTOR_CIA402_OD_TABLE_LENGTH = sizeof(MOTOR_CIA402_OD_TABLE) / sizeof(MOTOR_CIA402_OD_TABLE[0]);


// /* ---- Lookup ---- */

// static const Cia402_OdEntry_T * Find(uint16_t index, uint8_t subindex)
// {
//     /* Linear is fine for ~20 entries; binary search if it grows past ~50. */
//     for (uint16_t i = 0U; i < MOTOR_CIA402_OD_TABLE_LENGTH; i++)
//     {
//         const Cia402_OdEntry_T * e = &MOTOR_CIA402_OD_TABLE[i];
//         if ((e->Index == index) && (e->SubIndex == subindex)) { return e; }
//     }
//     return NULL;
// }


// /* ---- Public dispatchers ---- */

// Cia402_OdInfo_T Cia402_Od_GetInfo(uint16_t index, uint8_t subindex)
// {
//     const Cia402_OdEntry_T * e = Find(index, subindex);
//     if (e == NULL) { return (Cia402_OdInfo_T){ 0 }; }
//     return (Cia402_OdInfo_T){ .Type = e->Type, .Access = e->Access, .Size = TypeSize(e->Type) };
// }

// Cia402_OdStatus_T Motor_Cia402_Od_Get(Motor_T * p, const Motor_Cia402_T * a, uint16_t index, uint8_t subindex, int32_t * p_value)
// {
//     const Cia402_OdEntry_T * e = Find(index, subindex);
//     if (e == NULL)        { return CIA402_OD_ERR_NO_OBJECT; }
//     if (e->Get == NULL)   { return CIA402_OD_ERR_WRITE_ONLY; }
//     *p_value = e->Get(p, a);
//     return CIA402_OD_OK;
// }

// Cia402_OdStatus_T Motor_Cia402_Od_Set(Motor_T * p, Motor_Cia402_T * a, uint16_t index, uint8_t subindex, int32_t value)
// {
//     const Cia402_OdEntry_T * e = Find(index, subindex);
//     if (e == NULL)        { return CIA402_OD_ERR_NO_OBJECT; }
//     if (e->Set == NULL)   { return CIA402_OD_ERR_READ_ONLY; }
//     return e->Set(p, a, value);
// }

