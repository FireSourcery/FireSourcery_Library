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
    @file   Cia402.c
    @author FireSourcery
    @brief  CiA 402 protocol-layer parser/dispatcher.

            SDO server: parses inbound SDO frames as Cia402_Sdo_T overlays,
                        dispatches via Cia402_OdInterface_T, builds the
                        response into the caller's buffer.

            PDO router: dispatches inbound PDO frames by COB-ID using typed
                        Cia402_RxPdo_*_T overlays for default mappings.
*/
/******************************************************************************/
#include "Cia402.h"

#include <stddef.h>

/******************************************************************************/
/*
    OD metadata — spec-fixed compile-time table.

    Rows are sorted by (Index, SubIndex) for readability and to permit
    binary search if the table grows past ~50 entries. Linear scan is
    fine at this size.

    To add an object: append a row. To remove: delete the row.
    No code change required in the SDO server.
*/
/******************************************************************************/
const Cia402_OdMeta_T CIA402_OD_META[] =
{
    { CIA402_OD_CONTROLWORD,             0U, CIA402_OD_TYPE_U16, CIA402_OD_ACCESS_RW, sizeof(uint16_t) },
    { CIA402_OD_STATUSWORD,              0U, CIA402_OD_TYPE_U16, CIA402_OD_ACCESS_RO, sizeof(uint16_t) },
    { CIA402_OD_QUICK_STOP_OPTION_CODE,  0U, CIA402_OD_TYPE_I16, CIA402_OD_ACCESS_RW, sizeof(int16_t)  },
    // { CIA402_OD_QUICK_STOP_OPTION_CODE,  0U, CIA402_OD_TYPE_I16, CIA402_OD_ACCESS_RW, offsetof(Cia402_Adapter_T, Config.QuickStopOption) },
    { CIA402_OD_SHUTDOWN_OPTION_CODE,    0U, CIA402_OD_TYPE_I16, CIA402_OD_ACCESS_RW, sizeof(int16_t)  },
    { CIA402_OD_DISABLE_OP_OPTION_CODE,  0U, CIA402_OD_TYPE_I16, CIA402_OD_ACCESS_RW, sizeof(int16_t)  },
    { CIA402_OD_HALT_OPTION_CODE,        0U, CIA402_OD_TYPE_I16, CIA402_OD_ACCESS_RW, sizeof(int16_t)  },
    { CIA402_OD_FAULT_REACTION_CODE,     0U, CIA402_OD_TYPE_I16, CIA402_OD_ACCESS_RW, sizeof(int16_t)  },
    { CIA402_OD_MODES_OF_OPERATION,      0U, CIA402_OD_TYPE_I8,  CIA402_OD_ACCESS_RW, sizeof(int8_t)   },
    { CIA402_OD_MODES_OF_OPERATION_DISP, 0U, CIA402_OD_TYPE_I8,  CIA402_OD_ACCESS_RO, sizeof(int8_t)   },
    { CIA402_OD_POSITION_ACTUAL,         0U, CIA402_OD_TYPE_I32, CIA402_OD_ACCESS_RO, sizeof(int32_t)  },
    { CIA402_OD_VELOCITY_ACTUAL,         0U, CIA402_OD_TYPE_I32, CIA402_OD_ACCESS_RO, sizeof(int32_t)  },
    { CIA402_OD_TARGET_TORQUE,           0U, CIA402_OD_TYPE_I16, CIA402_OD_ACCESS_RW, sizeof(int16_t)  },
    { CIA402_OD_TORQUE_ACTUAL,           0U, CIA402_OD_TYPE_I16, CIA402_OD_ACCESS_RO, sizeof(int16_t)  },
    { CIA402_OD_CURRENT_ACTUAL,          0U, CIA402_OD_TYPE_I16, CIA402_OD_ACCESS_RO, sizeof(int16_t)  },
    { CIA402_OD_DC_LINK_VOLTAGE,         0U, CIA402_OD_TYPE_U32, CIA402_OD_ACCESS_RO, sizeof(uint32_t) },
    { CIA402_OD_QUICK_STOP_DECELERATION, 0U, CIA402_OD_TYPE_U32, CIA402_OD_ACCESS_RW, sizeof(uint32_t) },
    { CIA402_OD_TARGET_VELOCITY,         0U, CIA402_OD_TYPE_I32, CIA402_OD_ACCESS_RW, sizeof(int32_t)  },
    { CIA402_OD_SUPPORTED_DRIVE_MODES,   0U, CIA402_OD_TYPE_U32, CIA402_OD_ACCESS_RO, sizeof(uint32_t) },
};


/* switch mapped */
const Cia402_OdMeta_T * _TableIndex(uint16_t index, uint8_t subindex)
{
    switch (index)
    {
        case CIA402_OD_CONTROLWORD:             return &CIA402_OD_META[0U];
        case CIA402_OD_STATUSWORD:              return &CIA402_OD_META[1U];
        case CIA402_OD_QUICK_STOP_OPTION_CODE:  return &CIA402_OD_META[2U];
        case CIA402_OD_SHUTDOWN_OPTION_CODE:    return &CIA402_OD_META[3U];
        case CIA402_OD_DISABLE_OP_OPTION_CODE:  return &CIA402_OD_META[4U];
        case CIA402_OD_HALT_OPTION_CODE:        return &CIA402_OD_META[5U];
        case CIA402_OD_FAULT_REACTION_CODE:     return &CIA402_OD_META[6U];
        case CIA402_OD_MODES_OF_OPERATION:      return &CIA402_OD_META[7U];
        case CIA402_OD_MODES_OF_OPERATION_DISP: return &CIA402_OD_META[8U];
        case CIA402_OD_POSITION_ACTUAL:         return &CIA402_OD_META[9U];
        case CIA402_OD_VELOCITY_ACTUAL:         return &CIA402_OD_META[10U];
        case CIA402_OD_TARGET_TORQUE:           return &CIA402_OD_META[11U];
        case CIA402_OD_TORQUE_ACTUAL:           return &CIA402_OD_META[12U];
        case CIA402_OD_CURRENT_ACTUAL:          return &CIA402_OD_META[13U];
        case CIA402_OD_DC_LINK_VOLTAGE:         return &CIA402_OD_META[14U];
        case CIA402_OD_TARGET_POSITION:         return &CIA402_OD_META[15U];
        case CIA402_OD_TARGET_VELOCITY:         return &CIA402_OD_META[16U];
        case CIA402_OD_QUICK_STOP_DECELERATION: return &CIA402_OD_META[17U];
        case CIA402_OD_SUPPORTED_DRIVE_MODES:   return &CIA402_OD_META[18U];
        default:                                return NULL;
    }
}

static const Cia402_OdInfo_T OD_ABSENT = { CIA402_OD_TYPE_NONE, CIA402_OD_ACCESS_NONE, 0U };

Cia402_OdInfo_T Cia402_Od_GetInfo(uint16_t index, uint8_t subindex)
{
    if (subindex != 0U) { return OD_ABSENT; }

    const Cia402_OdMeta_T * p_meta = _TableIndex(index, subindex);
    if (p_meta == NULL) { return OD_ABSENT; }
    return (Cia402_OdInfo_T) { .Type = p_meta->Type, .Access = p_meta->Access, .Size = p_meta->Size, };
}



/******************************************************************************/
/*
    Optional Interface
*/
/******************************************************************************/
/******************************************************************************/
/*
    SDO server entry point
*/
/******************************************************************************/
// uint8_t Cia402_Sdo_HandleRequest(const Cia402_OdInterface_T * p_od, const Cia402_Adapter_T * p_adapter, const Cia402_Sdo_T * p_req, Cia402_Sdo_T * p_rsp)
uint8_t Cia402_Sdo_HandleRequest(const Cia402_OdInterface_T * p_od, const Cia402_Sdo_T * p_req, Cia402_Sdo_T * p_rsp)
{
    if (p_od == NULL || p_req == NULL || p_rsp == NULL) { return 0U; }

    Cia402_OdInfo_T info = (p_od->GetInfo != NULL) ? p_od->GetInfo(p_od->p_Context, p_req->Index, p_req->SubIndex) : (Cia402_OdInfo_T) { 0 };

    switch ((Cia402_SdoCcs_T)p_req->Cmd.Ccs)
    {
        case CIA402_SDO_CCS_DOWNLOAD_INIT_REQ: /* master writes object */
            {
                if (info.Type == CIA402_OD_TYPE_NONE)
                {
                    *p_rsp = Cia402_Sdo_EncodeAbort(p_req->Index, p_req->SubIndex, (p_req->SubIndex != 0U) ? CIA402_OD_ERR_SUBINDEX : CIA402_OD_ERR_NO_OBJECT);
                    break;
                }
                if (info.Access == CIA402_OD_ACCESS_RO)
                {
                    *p_rsp = Cia402_Sdo_EncodeAbort(p_req->Index, p_req->SubIndex, CIA402_OD_ERR_READ_ONLY);
                    break;
                }
                // if (info.AdapterOffet != 0xFFFFU)

                int32_t value = Cia402_SdoData_Decode(info.Type, p_req->Data);
                Cia402_OdStatus_T r = (p_od->Set != NULL) ? p_od->Set(p_od->p_Context, p_req->Index, p_req->SubIndex, value) : CIA402_OD_ERR_GENERAL;
                *p_rsp = (r == CIA402_OD_OK) ? Cia402_Sdo_EncodeDownloadAck(p_req->Index, p_req->SubIndex) : Cia402_Sdo_EncodeAbort(p_req->Index, p_req->SubIndex, r);
                break;
            }

        case CIA402_SDO_CCS_UPLOAD_INIT_REQ: /* master reads object */
            {
                if (info.Type == CIA402_OD_TYPE_NONE)
                {
                    *p_rsp = Cia402_Sdo_EncodeAbort(p_req->Index, p_req->SubIndex, (p_req->SubIndex != 0U) ? CIA402_OD_ERR_SUBINDEX : CIA402_OD_ERR_NO_OBJECT);
                    break;
                }
                if (info.Access == CIA402_OD_ACCESS_WO)
                {
                    *p_rsp = Cia402_Sdo_EncodeAbort(p_req->Index, p_req->SubIndex, CIA402_OD_ERR_WRITE_ONLY);
                    break;
                }

                int32_t value = 0;
                Cia402_OdStatus_T r = (p_od->Get != NULL) ? p_od->Get(p_od->p_Context, p_req->Index, p_req->SubIndex, &value) : CIA402_OD_ERR_GENERAL;
                *p_rsp = (r == CIA402_OD_OK) ? Cia402_Sdo_EncodeUploadResponse(p_req->Index, p_req->SubIndex, info, value) : Cia402_Sdo_EncodeAbort(p_req->Index, p_req->SubIndex, r);
                break;
            }

        case CIA402_SDO_CCS_ABORT:
            /* Master aborted — no response per CiA 301 */
            return 0U;

        default:
            /* Segmented and block transfers not supported by this minimal server */
            *p_rsp = Cia402_Sdo_EncodeAbort(p_req->Index, p_req->SubIndex, CIA402_OD_ERR_GENERAL);
            break;
    }

    return 8U;
}


/******************************************************************************/
/*
    PDO router — generic framework, mode-aware.

    Protocol-layer state (ActiveMode) is read directly from the adapter —
    Cia402_Adapter_T is itself a common-layer type, so this isn't coupling
    to integration data. Integration effects (Controlword application,
    setpoint update) still route through the OD interface so the same
    state-machine transitions and range checks apply as for SDO.

    Default RxPDO mappings handled (CiA 402 predefined connection set):
      RxPDO1 (0x200 + nodeId) : Controlword
      RxPDO2 (0x300 + nodeId) : Controlword + setpoint, layout selected by
                                Adapter.Input.ActiveMode:
                                  Torque modes   → +TargetTorque   (i16)
                                  Velocity modes → +TargetVelocity (i32)
                                  Position modes → +TargetPosition (i32)

    Setpoint is written before Controlword so an ENABLE_OPERATION transition
    consumes the new target. Integrators with non-default mappings can
    supply a different Cia402_OdInterface_T instance whose Set callback
    dispatches the same OD indices to mode-specific cmd helpers — no
    change needed here.
*/
/******************************************************************************/
void Cia402_Pdo_HandleRx(const Cia402_OdInterface_T * p_od, const Cia402_Adapter_T * p_adapter, uint16_t cob_id, const Cia402_Pdo_T * p_pdo, uint8_t dlc)
{
    if (p_od == NULL || p_od->Set == NULL || p_adapter == NULL || p_pdo == NULL) { return; }
    (void)dlc; /* trusted upstream — typed overlays imply expected length per mode */

    switch (cob_id & CIA402_COB_FUNCTION_MASK)
    {
        case CIA402_COB_RXPDO1_BASE: /* Controlword only */
            {
                const Cia402_RxPdo_Control_T * p_map = (const Cia402_RxPdo_Control_T *)p_pdo->Bytes;
                (void)p_od->Set(p_od->p_Context, CIA402_OD_CONTROLWORD, 0U, (int32_t)p_map->Controlword.Word);
                break;
            }

        case CIA402_COB_RXPDO2_BASE: /* Controlword + mode-specific setpoint */
            switch (p_adapter->Input.ActiveMode)
            {
                case CIA402_MODE_PROFILE_TORQUE:
                case CIA402_MODE_CYCLIC_SYNC_TORQUE:
                    {
                        const Cia402_RxPdo_ControlTorque_T * p_map = (const Cia402_RxPdo_ControlTorque_T *)p_pdo->Bytes;
                        (void)p_od->Set(p_od->p_Context, CIA402_OD_TARGET_TORQUE, 0U, (int32_t)p_map->TargetTorque);
                        (void)p_od->Set(p_od->p_Context, CIA402_OD_CONTROLWORD, 0U, (int32_t)p_map->Controlword.Word);
                        break;
                    }

                case CIA402_MODE_VELOCITY:
                case CIA402_MODE_PROFILE_VELOCITY:
                case CIA402_MODE_CYCLIC_SYNC_VELOCITY:
                    {
                        const Cia402_RxPdo_ControlVelocity_T * p_map = (const Cia402_RxPdo_ControlVelocity_T *)p_pdo->Bytes;
                        (void)p_od->Set(p_od->p_Context, CIA402_OD_TARGET_VELOCITY, 0U, p_map->TargetVelocity);
                        (void)p_od->Set(p_od->p_Context, CIA402_OD_CONTROLWORD, 0U, (int32_t)p_map->Controlword.Word);
                        break;
                    }

                case CIA402_MODE_PROFILE_POSITION:
                case CIA402_MODE_CYCLIC_SYNC_POSITION:
                    {
                        const Cia402_RxPdo_ControlPosition_T * p_map = (const Cia402_RxPdo_ControlPosition_T *)p_pdo->Bytes;
                        (void)p_od->Set(p_od->p_Context, CIA402_OD_TARGET_POSITION, 0U, p_map->TargetPosition);
                        (void)p_od->Set(p_od->p_Context, CIA402_OD_CONTROLWORD, 0U, (int32_t)p_map->Controlword.Word);
                        break;
                    }

                default:
                    {
                        /* Unknown / NONE mode — Controlword only, drop setpoint */
                        const Cia402_RxPdo_Control_T * p_map = (const Cia402_RxPdo_Control_T *)p_pdo->Bytes;
                        (void)p_od->Set(p_od->p_Context, CIA402_OD_CONTROLWORD, 0U, (int32_t)p_map->Controlword.Word);
                        break;
                    }
            }
            break;

        default:
            /* Unmapped COB-ID — caller should filter before dispatch */
            break;
    }
}




