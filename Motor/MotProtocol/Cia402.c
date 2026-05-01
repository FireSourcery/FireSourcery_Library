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

/*
    Per-type expansion macros — each yields a compile-time const Cia402_OdInfo_T
    literal. Size resolves via sizeof() on the wire type, so the entire switch
    folds to a compare-and-load with no helper-function call.
*/
#define OD_I8(acc)   ((const Cia402_OdInfo_T){ CIA402_OD_TYPE_I8,  (acc), sizeof(int8_t)   })
#define OD_U8(acc)   ((const Cia402_OdInfo_T){ CIA402_OD_TYPE_U8,  (acc), sizeof(uint8_t)  })
#define OD_I16(acc)  ((const Cia402_OdInfo_T){ CIA402_OD_TYPE_I16, (acc), sizeof(int16_t)  })
#define OD_U16(acc)  ((const Cia402_OdInfo_T){ CIA402_OD_TYPE_U16, (acc), sizeof(uint16_t) })
#define OD_I32(acc)  ((const Cia402_OdInfo_T){ CIA402_OD_TYPE_I32, (acc), sizeof(int32_t)  })
#define OD_U32(acc)  ((const Cia402_OdInfo_T){ CIA402_OD_TYPE_U32, (acc), sizeof(uint32_t) })
#define OD_ABSENT    ((const Cia402_OdInfo_T){ CIA402_OD_TYPE_NONE, CIA402_OD_ACCESS_NONE, 0U })

Cia402_OdInfo_T Cia402_Od_GetInfo(uint16_t index, uint8_t subindex)
{
    if (subindex != 0U) { return OD_ABSENT; }

    switch (index)
    {
        case CIA402_OD_CONTROLWORD:             return OD_U16(CIA402_OD_ACCESS_RW);
        case CIA402_OD_STATUSWORD:              return OD_U16(CIA402_OD_ACCESS_RO);
        case CIA402_OD_QUICK_STOP_OPTION_CODE:  return OD_I16(CIA402_OD_ACCESS_RW);
        case CIA402_OD_SHUTDOWN_OPTION_CODE:    return OD_I16(CIA402_OD_ACCESS_RW);
        case CIA402_OD_DISABLE_OP_OPTION_CODE:  return OD_I16(CIA402_OD_ACCESS_RW);
        case CIA402_OD_HALT_OPTION_CODE:        return OD_I16(CIA402_OD_ACCESS_RW);
        case CIA402_OD_FAULT_REACTION_CODE:     return OD_I16(CIA402_OD_ACCESS_RW);
        case CIA402_OD_MODES_OF_OPERATION:      return OD_I8 (CIA402_OD_ACCESS_RW);
        case CIA402_OD_MODES_OF_OPERATION_DISP: return OD_I8 (CIA402_OD_ACCESS_RO);
        case CIA402_OD_POSITION_ACTUAL:         return OD_I32(CIA402_OD_ACCESS_RO);
        case CIA402_OD_VELOCITY_ACTUAL:         return OD_I32(CIA402_OD_ACCESS_RO);
        case CIA402_OD_TARGET_TORQUE:           return OD_I16(CIA402_OD_ACCESS_RW);
        case CIA402_OD_TORQUE_ACTUAL:           return OD_I16(CIA402_OD_ACCESS_RO);
        case CIA402_OD_CURRENT_ACTUAL:          return OD_I16(CIA402_OD_ACCESS_RO);
        case CIA402_OD_DC_LINK_VOLTAGE:         return OD_U32(CIA402_OD_ACCESS_RO);
        case CIA402_OD_TARGET_POSITION:         return OD_I32(CIA402_OD_ACCESS_RW);
        case CIA402_OD_TARGET_VELOCITY:         return OD_I32(CIA402_OD_ACCESS_RW);
        case CIA402_OD_QUICK_STOP_DECELERATION: return OD_U32(CIA402_OD_ACCESS_RW);
        case CIA402_OD_SUPPORTED_DRIVE_MODES:   return OD_U32(CIA402_OD_ACCESS_RO);
        default:                                return OD_ABSENT;
    }
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
                const Cia402_RxPdo_Cw_T * p_map = (const Cia402_RxPdo_Cw_T *)p_pdo->Bytes;
                (void)p_od->Set(p_od->p_Context, CIA402_OD_CONTROLWORD, 0U, (int32_t)p_map->Controlword.Word);
                break;
            }

        case CIA402_COB_RXPDO2_BASE: /* Controlword + mode-specific setpoint */
            switch (p_adapter->Input.ActiveMode)
            {
                case CIA402_MODE_PROFILE_TORQUE:
                case CIA402_MODE_CYCLIC_SYNC_TORQUE:
                    {
                        const Cia402_RxPdo_CwTorque_T * p_map = (const Cia402_RxPdo_CwTorque_T *)p_pdo->Bytes;
                        (void)p_od->Set(p_od->p_Context, CIA402_OD_TARGET_TORQUE, 0U, (int32_t)p_map->TargetTorque);
                        (void)p_od->Set(p_od->p_Context, CIA402_OD_CONTROLWORD, 0U, (int32_t)p_map->Controlword.Word);
                        break;
                    }

                case CIA402_MODE_VELOCITY:
                case CIA402_MODE_PROFILE_VELOCITY:
                case CIA402_MODE_CYCLIC_SYNC_VELOCITY:
                    {
                        const Cia402_RxPdo_CwVelocity_T * p_map = (const Cia402_RxPdo_CwVelocity_T *)p_pdo->Bytes;
                        (void)p_od->Set(p_od->p_Context, CIA402_OD_TARGET_VELOCITY, 0U, p_map->TargetVelocity);
                        (void)p_od->Set(p_od->p_Context, CIA402_OD_CONTROLWORD, 0U, (int32_t)p_map->Controlword.Word);
                        break;
                    }

                case CIA402_MODE_PROFILE_POSITION:
                case CIA402_MODE_CYCLIC_SYNC_POSITION:
                    {
                        const Cia402_RxPdo_CwPosition_T * p_map = (const Cia402_RxPdo_CwPosition_T *)p_pdo->Bytes;
                        (void)p_od->Set(p_od->p_Context, CIA402_OD_TARGET_POSITION, 0U, p_map->TargetPosition);
                        (void)p_od->Set(p_od->p_Context, CIA402_OD_CONTROLWORD, 0U, (int32_t)p_map->Controlword.Word);
                        break;
                    }

                default:
                    {
                        /* Unknown / NONE mode — Controlword only, drop setpoint */
                        const Cia402_RxPdo_Cw_T * p_map = (const Cia402_RxPdo_Cw_T *)p_pdo->Bytes;
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
// const Cia402_OdMeta_T CIA402_OD_META[] =
// {
//     { CIA402_OD_CONTROLWORD,             0U, CIA402_OD_TYPE_U16, CIA402_OD_ACCESS_RW, sizeof(uint16_t) },
//     { CIA402_OD_STATUSWORD,              0U, CIA402_OD_TYPE_U16, CIA402_OD_ACCESS_RO, sizeof(uint16_t) },
//     { CIA402_OD_QUICK_STOP_OPTION_CODE,  0U, CIA402_OD_TYPE_I16, CIA402_OD_ACCESS_RW, sizeof(int16_t)  },
//     { CIA402_OD_SHUTDOWN_OPTION_CODE,    0U, CIA402_OD_TYPE_I16, CIA402_OD_ACCESS_RW, sizeof(int16_t)  },
//     { CIA402_OD_DISABLE_OP_OPTION_CODE,  0U, CIA402_OD_TYPE_I16, CIA402_OD_ACCESS_RW, sizeof(int16_t)  },
//     { CIA402_OD_HALT_OPTION_CODE,        0U, CIA402_OD_TYPE_I16, CIA402_OD_ACCESS_RW, sizeof(int16_t)  },
//     { CIA402_OD_FAULT_REACTION_CODE,     0U, CIA402_OD_TYPE_I16, CIA402_OD_ACCESS_RW, sizeof(int16_t)  },
//     { CIA402_OD_MODES_OF_OPERATION,      0U, CIA402_OD_TYPE_I8,  CIA402_OD_ACCESS_RW, sizeof(int8_t)   },
//     { CIA402_OD_MODES_OF_OPERATION_DISP, 0U, CIA402_OD_TYPE_I8,  CIA402_OD_ACCESS_RO, sizeof(int8_t)   },
//     { CIA402_OD_POSITION_ACTUAL,         0U, CIA402_OD_TYPE_I32, CIA402_OD_ACCESS_RO, sizeof(int32_t)  },
//     { CIA402_OD_VELOCITY_ACTUAL,         0U, CIA402_OD_TYPE_I32, CIA402_OD_ACCESS_RO, sizeof(int32_t)  },
//     { CIA402_OD_TARGET_TORQUE,           0U, CIA402_OD_TYPE_I16, CIA402_OD_ACCESS_RW, sizeof(int16_t)  },
//     { CIA402_OD_TORQUE_ACTUAL,           0U, CIA402_OD_TYPE_I16, CIA402_OD_ACCESS_RO, sizeof(int16_t)  },
//     { CIA402_OD_CURRENT_ACTUAL,          0U, CIA402_OD_TYPE_I16, CIA402_OD_ACCESS_RO, sizeof(int16_t)  },
//     { CIA402_OD_DC_LINK_VOLTAGE,         0U, CIA402_OD_TYPE_U32, CIA402_OD_ACCESS_RO, sizeof(uint32_t) },
//     { CIA402_OD_QUICK_STOP_DECELERATION, 0U, CIA402_OD_TYPE_U32, CIA402_OD_ACCESS_RW, sizeof(uint32_t) },
//     { CIA402_OD_TARGET_VELOCITY,         0U, CIA402_OD_TYPE_I32, CIA402_OD_ACCESS_RW, sizeof(int32_t)  },
//     { CIA402_OD_SUPPORTED_DRIVE_MODES,   0U, CIA402_OD_TYPE_U32, CIA402_OD_ACCESS_RO, sizeof(uint32_t) },
// };