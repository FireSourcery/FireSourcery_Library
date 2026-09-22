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


/*
    Modes this driver actually supports — declare in object 0x6502.
    Update as Cia402-side operating modes are wired through.
*/
#define MOTOR_CIA402_SUPPORTED_DRIVE_MODES  (CIA402_SUPPORTED_PV | CIA402_SUPPORTED_TQ)

static_assert(sizeof(Cia402_Adapter_T) <= MOTOR_ADAPTER_BUFFER_SIZE, "MOTOR_ADAPTER_BUFFER_SIZE must be large enough to hold the largest adapter struct");

static inline Cia402_Adapter_T * Motor_Cia402_Adapter(Motor_T * p_motor) { return (Cia402_Adapter_T *)(p_motor->P_MOTOR->AdapterBuffer); }

/*
    keep (Motor_T * p_motor, Cia402_Adapter_T * p_adapter, ...) signatures in case Adapter is move to outer context.
*/
// static inline Cia402_Adapter_T * Motor_Cia402_Adapter(Motor_Entity_T * p_motor) { return (Cia402_Adapter_T *)(p_motor->P_MOTOR->AdapterBuffer); }

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
// static void Motor_Cia402_WriteControl(Motor_T * p_motor, Cia402_Adapter_T * p_adapter, Cia402_Control_T control)
// {
//     Cia402_Control_T prev = p_adapter->Input.Control;

//     switch (Cia402_DecodeControlCmd(control))
//     {
//         case CIA402_CMD_DISABLE_VOLTAGE:        Motor_Disable(p_motor);             break;
//         case CIA402_CMD_SHUTDOWN:               Motor_ReleaseVZ(p_motor);           break;
//         case CIA402_CMD_SWITCH_ON:              Motor_ReleaseV0(p_motor);           break;
//         case CIA402_CMD_ENABLE_OPERATION:       Motor_ActivateControl(p_motor);     break;
//         case CIA402_CMD_QUICK_STOP:             /* Motor_Disable(p_motor);  */      break;
//         case CIA402_CMD_FAULT_RESET:
//             if (Cia402_IsFaultResetEdge(prev, control) == true) { Motor_StateMachine_TryClearFaultAll(p_motor); }
//             break;
//         default:
//             break;
//     }
//     p_adapter->Input.Control = control;
// }

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
static Cia402_Status_T Motor_Cia402_ReadStatus(Motor_T * p_motor)
{
    Cia402_Status_T status = { .Word = 0U };
    Motor_StateId_T id = Motor_GetStateId(p_motor->P_MOTOR);

    switch (id)
    {
        case MOTOR_STATE_ID_INIT:           status.Word = CIA402_STATE_NOT_READY_TO_SWITCH_ON;  break;
        case MOTOR_STATE_ID_DEACTIVATED:    status.Word = CIA402_STATE_SWITCH_ON_DISABLED;      break;
        case MOTOR_STATE_ID_PASSIVE:        status.Word = (Motor_GetPhaseState(p_motor) != PHASE_VOUT_Z) ? CIA402_STATE_SWITCHED_ON : CIA402_STATE_READY_TO_SWITCH_ON; break;
        case MOTOR_STATE_ID_RUN:            status.Word = CIA402_STATE_OPERATION_ENABLED;       break;
        case MOTOR_STATE_ID_INTERVENTION:   status.Word = CIA402_STATE_OPERATION_ENABLED;       break;
        case MOTOR_STATE_ID_OPEN_LOOP:      status.Word = CIA402_STATE_OPERATION_ENABLED;
                                            status.Manufacturer0 = 1U;                           break;
        case MOTOR_STATE_ID_CALIBRATION:    status.Word = (Motor_GetPhaseState(p_motor) != PHASE_VOUT_Z) ? CIA402_STATE_SWITCHED_ON : CIA402_STATE_SWITCH_ON_DISABLED; break;
                                            status.Manufacturer0 = 1U;                           break;
        case MOTOR_STATE_ID_FAULT:          status.Word = CIA402_STATE_FAULT;                    break;
        default:                            status.Word = CIA402_STATE_NOT_READY_TO_SWITCH_ON;   break;
    }

    if (id != MOTOR_STATE_ID_INIT) { status.VoltageEnabled = 1U; }
    status.Remote = 1U;

    return status;
}

static OD_Status_T Motor_Cia402_WriteOpMode(Motor_T * p_motor, Cia402_Adapter_T * p_adapter, Cia402_OpMode_T value)
{
    switch (value)
    {
        case CIA402_MODE_PROFILE_POSITION:      /* not supported */ break;
        case CIA402_MODE_VELOCITY:              Motor_SetFeedbackMode(p_motor, MOTOR_FEEDBACK_MODE_SPEED_CURRENT);  break;
        case CIA402_MODE_PROFILE_VELOCITY:      Motor_SetFeedbackMode(p_motor, MOTOR_FEEDBACK_MODE_SPEED_CURRENT);  break;
        case CIA402_MODE_PROFILE_TORQUE:        Motor_SetFeedbackMode(p_motor, MOTOR_FEEDBACK_MODE_CURRENT);        break;
        case CIA402_MODE_HOMING:                /* not supported */ break;
        case CIA402_MODE_INTERPOLATED_POSITION: /* not supported */ break;
        case CIA402_MODE_CYCLIC_SYNC_POSITION:  /* not supported */ break;
        case CIA402_MODE_CYCLIC_SYNC_VELOCITY: /* not supported */ break;
        case CIA402_MODE_CYCLIC_SYNC_TORQUE:   /* not supported */ break;
        default:                                return OD_ERR_VALUE_RANGE;
    }
    p_adapter->Input.ActiveMode = value;

    return OD_OK;
}


/******************************************************************************/
/*
    Statusword build-side helpers (TxPDO producers)
    Caller picks the variant matching the configured TxPDO mapping.
*/
/******************************************************************************/
void Motor_Cia402_BuildTxPdo_Sw(Motor_T * p_motor, Cia402_TxPdo_Status_T * p_pdo)
{
    p_pdo->Statusword = Motor_Cia402_ReadStatus(p_motor);
}

void Motor_Cia402_BuildTxPdo_SwVelocity(Motor_T * p_motor, Cia402_TxPdo_StatusVelocity_T * p_pdo)
{
    p_pdo->Statusword     = Motor_Cia402_ReadStatus(p_motor);
    p_pdo->VelocityActual = (int32_t)Motor_User_GetSpeed_Fract16(p_motor->P_MOTOR);
}

void Motor_Cia402_BuildTxPdo_SwTorque(Motor_T * p_motor, Cia402_TxPdo_StatusTorque_T * p_pdo)
{
    p_pdo->Statusword   = Motor_Cia402_ReadStatus(p_motor);
    p_pdo->TorqueActual = (int16_t)p_motor->P_MOTOR->Foc.Iq;
}

void Motor_Cia402_BuildTxPdo_SwPosition(Motor_T * p_motor, Cia402_TxPdo_StatusPosition_T * p_pdo)
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
void Motor_Cia402_HandleRxPdo_Cw(Motor_T * p_motor, Cia402_Adapter_T * p_adapter, const Cia402_RxPdo_Control_T * p_pdo)
{
    Motor_Cia402_WriteControl(p_motor, p_adapter, p_pdo->Controlword);
}

void Motor_Cia402_HandleRxPdo_CwTorque(Motor_T * p_motor, Cia402_Adapter_T * p_adapter, const Cia402_RxPdo_ControlTorque_T * p_pdo)
{
    Motor_SetTorqueCmd(p_motor->P_MOTOR, p_pdo->TargetTorque);
    Motor_Cia402_WriteControl(p_motor, p_adapter, p_pdo->Controlword);
}

void Motor_Cia402_HandleRxPdo_CwVelocity(Motor_T * p_motor, Cia402_Adapter_T * p_adapter, const Cia402_RxPdo_ControlVelocity_T * p_pdo)
{
    Motor_SetSpeedCmd(p_motor->P_MOTOR, (int16_t)p_pdo->TargetVelocity);
    Motor_Cia402_WriteControl(p_motor, p_adapter, p_pdo->Controlword);
}

void Motor_Cia402_HandleRxPdo_CwPosition(Motor_T * p_motor, Cia402_Adapter_T * p_adapter, const Cia402_RxPdo_ControlPosition_T * p_pdo)
{
    /* Motor_SetPositionCmd not yet wired through. Controlword still applied. */
    /* Motor_SetPositionCmd(p_motor->P_MOTOR, (uint16_t)p_pdo->TargetPosition); */
    Motor_Cia402_WriteControl(p_motor, p_adapter, p_pdo->Controlword);
}


/******************************************************************************/
/*
    SDO
*/
/******************************************************************************/
bool Motor_Cia402_HandleSdo(Motor_T * p_motor, Cia402_Adapter_T * p_adapter, const SDO_T * p_req, SDO_T * p_rsp)
{
    OD_Info_T info = Cia402_Od_GetInfo(p_req->Index, p_req->SubIndex);
    // Cia402_OdMeta_T info  ; /handle adapter-backed entries here if needed

    switch ((SDO_Ccs_T)p_req->Cmd.Ccs)
    {
        case SDO_CCS_DOWNLOAD_INIT_REQ: /* master writes object */
            {
                if (info.Type == OD_TYPE_NONE)
                {
                    *p_rsp = SDO_EncodeAbort(p_req->Index, p_req->SubIndex, (p_req->SubIndex != 0U) ? OD_ERR_SUBINDEX : OD_ERR_NO_OBJECT);
                    break;
                }
                if (info.Access == OD_ACCESS_RO)
                {
                    *p_rsp = SDO_EncodeAbort(p_req->Index, p_req->SubIndex, OD_ERR_READ_ONLY);
                    break;
                }
                int32_t value = OD_Data_Decode(info.Type, p_req->Data);
                OD_Status_T r = Motor_Cia402_Od_Set(p_motor, p_adapter, p_req->Index, p_req->SubIndex, value);
                *p_rsp = (r == OD_OK) ? SDO_EncodeDownloadAck(p_req->Index, p_req->SubIndex) : SDO_EncodeAbort(p_req->Index, p_req->SubIndex, r);
                break;
            }

        case SDO_CCS_UPLOAD_INIT_REQ: /* master reads object */
            {
                if (info.Type == OD_TYPE_NONE)
                {
                    *p_rsp = SDO_EncodeAbort(p_req->Index, p_req->SubIndex, (p_req->SubIndex != 0U) ? OD_ERR_SUBINDEX : OD_ERR_NO_OBJECT);
                    break;
                }
                if (info.Access == OD_ACCESS_WO)
                {
                    *p_rsp = SDO_EncodeAbort(p_req->Index, p_req->SubIndex, OD_ERR_WRITE_ONLY);
                    break;
                }
                int32_t value = 0;
                OD_Status_T r = Motor_Cia402_Od_Get(p_motor, p_adapter, p_req->Index, p_req->SubIndex, &value);
                *p_rsp = (r == OD_OK) ? SDO_EncodeUploadResponse(p_req->Index, p_req->SubIndex, info, value) : SDO_EncodeAbort(p_req->Index, p_req->SubIndex, r);
                break;
            }

        case SDO_CCS_ABORT: /* Master aborted — no response per CiA 301 */
            return false;

        default:
            /* Segmented and block transfers not supported by this minimal server */
            *p_rsp = SDO_EncodeAbort(p_req->Index, p_req->SubIndex, OD_ERR_GENERAL);
            break;
    }

    return true;
}


/******************************************************************************/
/*
    Object Dictionary
    via switch
*/
/******************************************************************************/
OD_Status_T Motor_Cia402_Od_Get(Motor_T * p_motor, const Cia402_Adapter_T * p_adapter, uint16_t index, uint8_t subindex, int32_t * p_value)
{
    OD_Info_T info = Cia402_Od_GetInfo(index, subindex);

    if (info.Type   == OD_TYPE_NONE)   { return (subindex != 0U) ? OD_ERR_SUBINDEX : OD_ERR_NO_OBJECT; }
    if (info.Access == OD_ACCESS_WO)   { return OD_ERR_WRITE_ONLY; }

    switch (index)
    {
        case CIA402_OD_CONTROLWORD:             *p_value = p_adapter->Input.Control.Word;                                       break;
        case CIA402_OD_STATUSWORD:              *p_value = Motor_Cia402_ReadStatus(p_motor).Word;                                   break;
        case CIA402_OD_QUICK_STOP_OPTION_CODE:  *p_value = (int16_t)p_adapter->Config.QuickStopOption;                              break;
        case CIA402_OD_SHUTDOWN_OPTION_CODE:    *p_value = (int16_t)p_adapter->Config.ShutdownOption;                               break;
        case CIA402_OD_DISABLE_OP_OPTION_CODE:  *p_value = (int16_t)p_adapter->Config.DisableOpOption;                              break;
        case CIA402_OD_HALT_OPTION_CODE:        *p_value = (int16_t)p_adapter->Config.HaltOption;                                   break;
        case CIA402_OD_FAULT_REACTION_CODE:     *p_value = (int16_t)p_adapter->Config.FaultReactOption;                             break;
        case CIA402_OD_MODES_OF_OPERATION:      *p_value = (int8_t)p_adapter->Input.ActiveMode;                                     break;
        case CIA402_OD_MODES_OF_OPERATION_DISP: *p_value = (int8_t)p_adapter->Input.ActiveMode;                                     break;
        case CIA402_OD_POSITION_ACTUAL:         *p_value = (int32_t)RotorSensor_GetMechanicalAngle(p_motor->P_MOTOR->p_ActiveSensor);    break; /* optional */
        case CIA402_OD_VELOCITY_ACTUAL:         *p_value = (int32_t)Motor_User_GetSpeed_Fract16(p_motor->P_MOTOR);                       break;
        case CIA402_OD_TARGET_TORQUE:           *p_value = (int16_t)_Motor_GetTorqueSetpoint(p_motor->P_MOTOR);                     break;
        case CIA402_OD_TORQUE_ACTUAL:           *p_value = (int16_t)p_motor->P_MOTOR->Foc.Iq;                                       break;
        case CIA402_OD_CURRENT_ACTUAL:          *p_value = (int16_t)Motor_GetIPhase_Fract16(p_motor->P_MOTOR);                      break;
        case CIA402_OD_DC_LINK_VOLTAGE:         *p_value = (int32_t)VBus_Fract16(p_motor->P_VBUS);                                   break;
        case CIA402_OD_TARGET_VELOCITY:         *p_value = (int32_t)Motor_GetSpeedSetpoint(p_motor->P_MOTOR);                       break;
        case CIA402_OD_QUICK_STOP_DECELERATION: *p_value = (int32_t)p_adapter->Config.QuickStopDecel;                               break;
        case CIA402_OD_SUPPORTED_DRIVE_MODES:   *p_value = (int32_t)MOTOR_CIA402_SUPPORTED_DRIVE_MODES;                             break;
        default:                                return OD_ERR_NO_OBJECT;
    }

    return OD_OK;
}

OD_Status_T Motor_Cia402_Od_Set(Motor_T * p_motor, Cia402_Adapter_T * p_adapter, uint16_t index, uint8_t subindex, int32_t value)
{
    OD_Info_T info = Cia402_Od_GetInfo(index, subindex);
    if (info.Type   == OD_TYPE_NONE)   { return (subindex != 0U) ? OD_ERR_SUBINDEX : OD_ERR_NO_OBJECT; }
    if (info.Access == OD_ACCESS_RO)   { return OD_ERR_READ_ONLY; }

    switch (index)
    {
        case CIA402_OD_CONTROLWORD:                 Motor_Cia402_WriteControl(p_motor, p_adapter, (Cia402_Control_T){ .Word = (uint16_t)value });            break;
        case CIA402_OD_MODES_OF_OPERATION:          Motor_Cia402_WriteOpMode(p_motor, p_adapter, (Cia402_OpMode_T)value);     break;
        case CIA402_OD_QUICK_STOP_OPTION_CODE:      p_adapter->Config.QuickStopOption  = (Cia402_QuickStopOption_T)value;     break;
        case CIA402_OD_SHUTDOWN_OPTION_CODE:        p_adapter->Config.ShutdownOption   = (Cia402_ShutdownOption_T)value;      break;
        case CIA402_OD_DISABLE_OP_OPTION_CODE:      p_adapter->Config.DisableOpOption  = (Cia402_DisableOpOption_T)value;     break;
        case CIA402_OD_HALT_OPTION_CODE:            p_adapter->Config.HaltOption       = (Cia402_HaltOption_T)value;          break;
        case CIA402_OD_FAULT_REACTION_CODE:         p_adapter->Config.FaultReactOption = (Cia402_FaultReactionOption_T)value; break;
        case CIA402_OD_TARGET_TORQUE:               Motor_SetTorqueCmd(p_motor->P_MOTOR, (int16_t)value);            break;
        case CIA402_OD_TARGET_VELOCITY:             Motor_SetSpeedCmd(p_motor->P_MOTOR, (int16_t)value);             break;
        case CIA402_OD_QUICK_STOP_DECELERATION:     p_adapter->Config.QuickStopDecel = (uint32_t)value;              break;
        default:                                    return OD_ERR_NO_OBJECT;
    }

    return OD_OK;
}

// get adapter from within motor
// static const OD_Interface_T APP_OD =
// {
//     .GetInfo = (OD_GetInfoFn_T)Cia402_Od_GetInfo,
//     .Get     = (OD_GetFn_T)Motor_Cia402_Od_Get,
//     .Set     = (OD_SetFn_T)Motor_Cia402_Od_Set,
// };


/******************************************************************************/
/*
    Object Dictionary by Table
*/
/******************************************************************************/
/* Motor_Cia402_Od.c */
typedef Cia402_Adapter_T Motor_Cia402_T;
/* ---- Per-entry accessors ---- */

static int32_t OdGet_Controlword(Motor_T * p, const Motor_Cia402_T * a)        { (void)p; return a->Input.Control.Word; }
static int32_t OdGet_Statusword (Motor_T * p, const Motor_Cia402_T * a)        { (void)a; return Motor_Cia402_ReadStatus(p).Word; }
static int32_t OdGet_Modes      (Motor_T * p, const Motor_Cia402_T * a)        { (void)p; return (int8_t)a->Input.ActiveMode; }
static int32_t OdGet_PosActual  (Motor_T * p, const Motor_Cia402_T * a)        { (void)a; return RotorSensor_GetMechanicalAngle(p->P_MOTOR->p_ActiveSensor); }
static int32_t OdGet_VelActual  (Motor_T * p, const Motor_Cia402_T * a)        { (void)a; return Motor_User_GetSpeed_Fract16(p->P_MOTOR); }
static int32_t OdGet_TorqueTgt  (Motor_T * p, const Motor_Cia402_T * a)        { (void)a; return _Motor_GetTorqueSetpoint(p->P_MOTOR); }
static int32_t OdGet_TorqueAct  (Motor_T * p, const Motor_Cia402_T * a)        { (void)a; return p->P_MOTOR->Foc.Iq; }
static int32_t OdGet_CurrentAct (Motor_T * p, const Motor_Cia402_T * a)        { (void)a; return Motor_GetIPhase_Fract16(p->P_MOTOR); }
static int32_t OdGet_VBus       (Motor_T * p, const Motor_Cia402_T * a)        { (void)a; return VBus_Fract16(p->P_VBUS); }
static int32_t OdGet_VelTarget  (Motor_T * p, const Motor_Cia402_T * a)        { (void)a; return Motor_GetSpeedSetpoint(p->P_MOTOR); }
static int32_t OdGet_QsDecel    (Motor_T * p, const Motor_Cia402_T * a)        { (void)p; return (int32_t)a->Config.QuickStopDecel; }
static int32_t OdGet_Supported  (Motor_T * p, const Motor_Cia402_T * a)        { (void)p; (void)a; return MOTOR_CIA402_SUPPORTED_DRIVE_MODES; }
static int32_t OdGet_QsOption   (Motor_T * p, const Motor_Cia402_T * a)        { (void)p; return a->Config.QuickStopOption; }
static int32_t OdGet_ShdnOption (Motor_T * p, const Motor_Cia402_T * a)        { (void)p; return a->Config.ShutdownOption; }
static int32_t OdGet_DisOption  (Motor_T * p, const Motor_Cia402_T * a)        { (void)p; return a->Config.DisableOpOption; }
static int32_t OdGet_HaltOption (Motor_T * p, const Motor_Cia402_T * a)        { (void)p; return a->Config.HaltOption; }
static int32_t OdGet_FltReact   (Motor_T * p, const Motor_Cia402_T * a)        { (void)p; return a->Config.FaultReactOption; }

static OD_Status_T OdSet_Controlword(Motor_T * p, Motor_Cia402_T * a, int32_t v) { Motor_Cia402_WriteControl(p, a, (Cia402_Control_T){ .Word = (uint16_t)v }); return OD_OK; }
static OD_Status_T OdSet_Modes      (Motor_T * p, Motor_Cia402_T * a, int32_t v) { (void)p; Motor_Cia402_WriteOpMode(p, a, (Cia402_OpMode_T)(int8_t)v); return OD_OK; }
static OD_Status_T OdSet_TorqueTgt  (Motor_T * p, Motor_Cia402_T * a, int32_t v) { (void)a; Motor_SetTorqueCmd(p->P_MOTOR, (int16_t)v); return OD_OK; }
static OD_Status_T OdSet_VelTarget  (Motor_T * p, Motor_Cia402_T * a, int32_t v) { (void)a; Motor_SetSpeedCmd(p->P_MOTOR, (int16_t)v); return OD_OK; }
static OD_Status_T OdSet_QsDecel    (Motor_T * p, Motor_Cia402_T * a, int32_t v) { (void)p; a->Config.QuickStopDecel  = (uint32_t)v;            return OD_OK; }
static OD_Status_T OdSet_QsOption   (Motor_T * p, Motor_Cia402_T * a, int32_t v) { (void)p; a->Config.QuickStopOption = (Cia402_QuickStopOption_T)v;     return OD_OK; }
static OD_Status_T OdSet_ShdnOption (Motor_T * p, Motor_Cia402_T * a, int32_t v) { (void)p; a->Config.ShutdownOption  = (Cia402_ShutdownOption_T)v;      return OD_OK; }
static OD_Status_T OdSet_DisOption  (Motor_T * p, Motor_Cia402_T * a, int32_t v) { (void)p; a->Config.DisableOpOption = (Cia402_DisableOpOption_T)v;     return OD_OK; }
static OD_Status_T OdSet_HaltOption (Motor_T * p, Motor_Cia402_T * a, int32_t v) { (void)p; a->Config.HaltOption      = (Cia402_HaltOption_T)v;          return OD_OK; }
static OD_Status_T OdSet_FltReact   (Motor_T * p, Motor_Cia402_T * a, int32_t v) { (void)p; a->Config.FaultReactOption= (Cia402_FaultReactionOption_T)v; return OD_OK; }


/* ---- The table (sorted by Index for binary search) ---- */

// const Cia402_OdEntry_T MOTOR_CIA402_OD_TABLE[] =
// {
//     { 0x6040, 0, OD_TYPE_U16, OD_ACCESS_RW, 0xFFFF, OdGet_Controlword, OdSet_Controlword },
//     { 0x6041, 0, OD_TYPE_U16, OD_ACCESS_RO, 0xFFFF, OdGet_Statusword,  NULL              },
//     { 0x605A, 0, OD_TYPE_I16, OD_ACCESS_RW, 0xFFFF, OdGet_QsOption,    OdSet_QsOption    },
//     { 0x605B, 0, OD_TYPE_I16, OD_ACCESS_RW, 0xFFFF, OdGet_ShdnOption,  OdSet_ShdnOption  },
//     { 0x605C, 0, OD_TYPE_I16, OD_ACCESS_RW, 0xFFFF, OdGet_DisOption,   OdSet_DisOption   },
//     { 0x605D, 0, OD_TYPE_I16, OD_ACCESS_RW, 0xFFFF, OdGet_HaltOption,  OdSet_HaltOption  },
//     { 0x605E, 0, OD_TYPE_I16, OD_ACCESS_RW, 0xFFFF, OdGet_FltReact,    OdSet_FltReact    },
//     { 0x6060, 0, OD_TYPE_I8,  OD_ACCESS_RW, 0xFFFF, OdGet_Modes,       OdSet_Modes       },
//     { 0x6061, 0, OD_TYPE_I8,  OD_ACCESS_RO, 0xFFFF, OdGet_Modes,       NULL              },
//     { 0x6064, 0, OD_TYPE_I32, OD_ACCESS_RO, 0xFFFF, OdGet_PosActual,   NULL              },
//     { 0x606C, 0, OD_TYPE_I32, OD_ACCESS_RO, 0xFFFF, OdGet_VelActual,   NULL              },
//     { 0x6071, 0, OD_TYPE_I16, OD_ACCESS_RW, 0xFFFF, OdGet_TorqueTgt,   OdSet_TorqueTgt   },
//     { 0x6077, 0, OD_TYPE_I16, OD_ACCESS_RO, 0xFFFF, OdGet_TorqueAct,   NULL              },
//     { 0x6078, 0, OD_TYPE_I16, OD_ACCESS_RO, 0xFFFF, OdGet_CurrentAct,  NULL              },
//     { 0x6079, 0, OD_TYPE_U32, OD_ACCESS_RO, 0xFFFF, OdGet_VBus,        NULL              },
//     { 0x6085, 0, OD_TYPE_U32, OD_ACCESS_RW, 0xFFFF, OdGet_QsDecel,     OdSet_QsDecel     },
//     { 0x60FF, 0, OD_TYPE_I32, OD_ACCESS_RW, 0xFFFF, OdGet_VelTarget,   OdSet_VelTarget   },
//     { 0x6502, 0, OD_TYPE_U32, OD_ACCESS_RO, 0xFFFF, OdGet_Supported,   NULL              },
// };

// const uint16_t MOTOR_CIA402_OD_TABLE_LENGTH = sizeof(MOTOR_CIA402_OD_TABLE) / sizeof(MOTOR_CIA402_OD_TABLE[0]);


// // /* ---- Lookup ---- */

// static const Cia402_OdEntry_T * Find(uint16_t index, uint8_t subindex)
// {
//     /* Linear is fine for ~20 entries; binary search if it grows past ~50. */
//     for (uint16_t i = 0U; i < MOTOR_CIA402_OD_TABLE_LENGTH; i++)
//     {
//         const Cia402_OdEntry_T * e = &MOTOR_CIA402_OD_TABLE[i];
//         if ((e->Meta.Index == index) && (e->Meta.SubIndex == subindex)) { return e; }
//     }
//     return NULL;
// }


// /* ---- Public dispatchers ---- */
// OD_Info_T Cia402_Od_GetInfo(uint16_t index, uint8_t subindex)
// {
//     const Cia402_OdEntry_T * e = Find(index, subindex);
//     if (e == NULL) { return (OD_Info_T){ 0 }; }
//     return (OD_Info_T){ .Type = e->Type, .Access = e->Access, .Size = TypeSize(e->Type) };
// }

// OD_Status_T Motor_Cia402_Od_Get(Motor_T * p, const Motor_Cia402_T * a, uint16_t index, uint8_t subindex, int32_t * p_value)
// {
//     const Cia402_OdEntry_T * e = Find(index, subindex);
//     if (e == NULL)        { return OD_ERR_NO_OBJECT; }
//     if (e->Get == NULL)   { return OD_ERR_WRITE_ONLY; }
//     *p_value = e->Get(p, a);
//     return OD_OK;
// }

// OD_Status_T Motor_Cia402_Od_Set(Motor_T * p, Motor_Cia402_T * a, uint16_t index, uint8_t subindex, int32_t value)
// {
//     const Cia402_OdEntry_T * e = Find(index, subindex);
//     if (e == NULL)        { return OD_ERR_NO_OBJECT; }
//     if (e->Set == NULL)   { return OD_ERR_READ_ONLY; }
//     return e->Set(p, a, value);
// }

