/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

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
    @file   MotorController_MotProtocol.c
    @author FireSourcery
    @brief  MotProtocol request handlers over MotorController_T.
*/
/******************************************************************************/
#include "MotorController_MotProtocol.h"
#include "Motor/MotProtocol/MotPacket.h"
#include "Motor/MotProtocol/MotProtocol.h"
#include "Motor/MotorController/MotorController_Var.h"
#include "Motor/MotorController/MotorController_User.h"

#include <string.h>

/******************************************************************************/
/*!
    MotProtocol implementation using MotorController_T.
    MotorController_T is the Protocol P_APP_CONTEXT directly, which avoids double buffering.

    Handler contract, per Protocol_Request.h:
        p_rxPayload / p_txPayload arrive already offset past the header - a handler never
        sees a delimiter, a length or a checksum, and never builds a header. It sets Id and
        Length on p_TxMeta and BUILD_TX_HEADER does the rest.

        Length on p_TxMeta is the PAYLOAD length, not the frame length.

        A stateless handler returns DONE on its first call. Anything that does not return
        DONE or ABORT leaves the request bound and the socket busy.
*/
/******************************************************************************/
/* The response payload can never exceed what one frame carries. */
#define MOT_PAYLOAD_MAX ((packet_size_t)(MOT_PACKET_LENGTH_MAX - sizeof(MotPacket_Header_T)))

/*
    Bound a count parsed from the request against what the response frame can hold.

    The request table carries no per-id length bounds yet, so a malformed count would
    otherwise size a write into the Tx payload. This is the handler standing in for a check
    that belongs in the table.
*/
static inline uint8_t CountMax(uint8_t count, size_t respElementSize)
{
    const uint8_t limit = (uint8_t)(MOT_PAYLOAD_MAX / respElementSize);
    return (count < limit) ? count : limit;
}

/******************************************************************************/
/*! Ping - answered with a bare sync frame, built by the codec from the id alone */
/******************************************************************************/
static Protocol_ReqCode_T Ping(MotorController_T * p_dev, Packet_Xfer_T * p_xfer, const MotPacket_PingReq_T * p_rxPayload, MotPacket_PingResp_T * p_txPayload)
{
    (void)p_rxPayload; (void)p_txPayload;
    MotBuzzer_Short(MotorController_Buzzer(p_dev));

    /* The reply is a bare sync frame, so the codec writes all of it from the id alone. */
    p_xfer->p_TxMeta->Id = MOT_PACKET_SYNC_ACK;
    p_xfer->p_TxMeta->Length = 0U;
    return PROTOCOL_REQ_DONE;
}

/******************************************************************************/
/*! Version */
/******************************************************************************/
static Protocol_ReqCode_T Version(MotorController_T * p_dev, Packet_Xfer_T * p_xfer, const MotPacket_VersionReq_T * p_rxPayload, MotPacket_VersionResp_T * p_txPayload)
{
    (void)p_rxPayload;

    p_txPayload->Protocol = MOT_PACKET_VERSION_WORD32;
    p_txPayload->Library = MOTOR_LIBRARY_VERSION;
    p_txPayload->Firmware = p_dev->MAIN_VERSION.Word32.Value32;

    p_xfer->p_TxMeta->Id = MOT_PACKET_VERSION;
    p_xfer->p_TxMeta->Length = sizeof(MotPacket_VersionResp_T);
    return PROTOCOL_REQ_DONE;
}

/******************************************************************************/
/*! Stop All */
/******************************************************************************/
static Protocol_ReqCode_T StopAll(MotorController_T * p_dev, Packet_Xfer_T * p_xfer, const MotPacket_StopReq_T * p_rxPayload, MotPacket_StopResp_T * p_txPayload)
{
    (void)p_rxPayload;
    MotorController_ForceDisableControl(p_dev);
    p_txPayload->Status = MOT_STATUS_SUCCESS;

    p_xfer->p_TxMeta->Id = MOT_PACKET_STOP_ALL;
    p_xfer->p_TxMeta->Length = sizeof(MotPacket_StopResp_T);
    return PROTOCOL_REQ_DONE;
}

/******************************************************************************/
/*! Call - May be Blocking */
/******************************************************************************/
static Protocol_ReqCode_T Call_Blocking(MotorController_T * p_dev, Packet_Xfer_T * p_xfer, const MotPacket_CallReq_T * p_rxPayload, MotPacket_CallResp_T * p_txPayload)
{
    p_txPayload->Id = p_rxPayload->Id;
    p_txPayload->Status = MotorController_CallSystemCmd(p_dev, (MotorController_SystemCmd_T)p_rxPayload->Id, p_rxPayload->Arg);

    p_xfer->p_TxMeta->Id = MOT_PACKET_CALL;
    p_xfer->p_TxMeta->Length = sizeof(MotPacket_CallResp_T);
    return PROTOCOL_REQ_DONE;
}

/******************************************************************************/
/*! Read / Write Single Var */
/******************************************************************************/
static Protocol_ReqCode_T ReadVar(MotorController_T * p_dev, Packet_Xfer_T * p_xfer, const MotPacket_VarReadFixedReq_T * p_rxPayload, MotPacket_VarReadFixedResp_T * p_txPayload)
{
    p_xfer->p_TxMeta->Length = MotorController_ReadVar(p_dev, p_rxPayload, p_txPayload);
    p_xfer->p_TxMeta->Id = MOT_PACKET_FIXED_VAR_READ;
    return PROTOCOL_REQ_DONE;
}

static Protocol_ReqCode_T WriteVar(MotorController_T * p_dev, Packet_Xfer_T * p_xfer, const MotPacket_VarWriteFixedReq_T * p_rxPayload, MotPacket_VarWriteFixedResp_T * p_txPayload)
{
    p_xfer->p_TxMeta->Length = MotorController_WriteVar(p_dev, p_rxPayload, p_txPayload);
    p_xfer->p_TxMeta->Id = MOT_PACKET_FIXED_VAR_WRITE;
    return PROTOCOL_REQ_DONE;
}

/******************************************************************************/
/*! Read / Write Var16s - resp truncates 32-bit vars */
/******************************************************************************/
static Protocol_ReqCode_T Var16Read(MotorController_T * p_dev, Packet_Xfer_T * p_xfer, const MotPacket_Var16ReadReq_T * p_rxPayload, MotPacket_Var16ReadResp_T * p_txPayload)
{
    uint8_t varCount = CountMax((uint8_t)(p_xfer->p_RxMeta->Length / sizeof(uint16_t)), sizeof(uint16_t));

    p_xfer->p_TxMeta->Length = MotorController_ReadVar16s(p_dev, p_rxPayload, p_txPayload, varCount);
    p_xfer->p_TxMeta->Id = MOT_PACKET_VAR16_READ;
    return PROTOCOL_REQ_DONE;
}

static Protocol_ReqCode_T Var16Write(MotorController_T * p_dev, Packet_Xfer_T * p_xfer, const MotPacket_Var16WriteReq_T * p_rxPayload, MotPacket_Var16WriteResp_T * p_txPayload)
{
    uint8_t varCount = CountMax((uint8_t)(p_xfer->p_RxMeta->Length / sizeof(p_rxPayload->Pairs[0U])), sizeof(uint8_t));

    p_xfer->p_TxMeta->Length = MotorController_WriteVar16s(p_dev, p_rxPayload, p_txPayload, varCount);
    p_xfer->p_TxMeta->Id = MOT_PACKET_VAR16_WRITE;
    return PROTOCOL_REQ_DONE;
}

/******************************************************************************/
/*! Read / Write Var32s */
/******************************************************************************/
static Protocol_ReqCode_T ReadVar32(MotorController_T * p_dev, Packet_Xfer_T * p_xfer, const MotPacket_Var32ReadReq_T * p_rxPayload, MotPacket_Var32ReadResp_T * p_txPayload)
{
    uint8_t varCount = CountMax((uint8_t)(p_xfer->p_RxMeta->Length / sizeof(p_rxPayload->Read[0U])), sizeof(uint32_t));

    p_xfer->p_TxMeta->Length = MotorController_ReadVar32s(p_dev, p_rxPayload, p_txPayload, varCount);
    p_xfer->p_TxMeta->Id = MOT_PACKET_VAR32_READ;
    return PROTOCOL_REQ_DONE;
}

static Protocol_ReqCode_T WriteVar32(MotorController_T * p_dev, Packet_Xfer_T * p_xfer, const MotPacket_Var32WriteReq_T * p_rxPayload, MotPacket_Var32WriteResp_T * p_txPayload)
{
    uint8_t varCount = CountMax((uint8_t)(p_xfer->p_RxMeta->Length / sizeof(p_rxPayload->Write[0U])), sizeof(uint8_t));

    p_xfer->p_TxMeta->Length = MotorController_WriteVar32s(p_dev, p_rxPayload, p_txPayload, varCount);
    p_xfer->p_TxMeta->Id = MOT_PACKET_VAR32_WRITE;
    return PROTOCOL_REQ_DONE;
}

/******************************************************************************/
/*! Mem

    NvMemory access is gated by a single MotorController_IsConfig check per request, so the
    outer StateMachine decides and the handler only reports.
*/
/******************************************************************************/
static Protocol_ReqCode_T ReadMem_Blocking(MotorController_T * p_dev, Packet_Xfer_T * p_xfer, const MotPacket_MemReadReq_T * p_rxPayload, MotPacket_MemReadResp_T * p_txPayload)
{
    uint8_t * p_buffer = p_txPayload->ByteData;
    uint8_t size = (p_rxPayload->Size < MOT_PAYLOAD_MAX) ? p_rxPayload->Size : (uint8_t)MOT_PAYLOAD_MAX;
    NvMemory_Status_T status;

    memset(p_buffer, 0U, size);

    if (MotorController_IsConfig(p_dev) == false) { status = NV_MEMORY_STATUS_ERROR_OTHER; }
    else switch ((MotProtocol_MemConfig_T)p_rxPayload->Config)
    {
        case MOT_PROTOCOL_MEM_CONFIG_RAM:  memcpy(p_buffer, (void *)p_rxPayload->Address, size); status = NV_MEMORY_STATUS_SUCCESS; break;
        case MOT_PROTOCOL_MEM_CONFIG_ONCE: status = MotNvm_ReadManufacture_Blocking(&p_dev->MOT_NVM, p_rxPayload->Address, size, p_buffer); break;
        case MOT_PROTOCOL_MEM_CONFIG_BOARD_REF_0: status = NV_MEMORY_STATUS_SUCCESS; memcpy(p_buffer, &PHASE_CALIBRATION, sizeof(Phase_Calibration_T));                 break;
        case MOT_PROTOCOL_MEM_CONFIG_BOARD_REF_1: status = NV_MEMORY_STATUS_SUCCESS; memcpy(p_buffer, &MOTOR_ELECTRICAL_CALIBRATION, sizeof(Motor_ElectricalCalib_T));  break;
        default: status = NV_MEMORY_STATUS_ERROR_NOT_IMPLEMENTED; break;
    }

    (void)status; /* MemRead header carries size only; status currently unused */

    p_xfer->p_TxMeta->Id = MOT_PACKET_MEM_READ;
    p_xfer->p_TxMeta->Length = size;
    return PROTOCOL_REQ_DONE;
}

/* Host handles the reboot */
static Protocol_ReqCode_T WriteMem_Blocking(MotorController_T * p_dev, Packet_Xfer_T * p_xfer, const MotPacket_MemWriteReq_T * p_rxPayload, MotPacket_MemWriteResp_T * p_txPayload)
{
    NvMemory_Status_T status;

    if (MotorController_IsConfig(p_dev) == false) { status = NV_MEMORY_STATUS_ERROR_OTHER; }
    else switch ((MotProtocol_MemConfig_T)p_rxPayload->Config)
    {
        case MOT_PROTOCOL_MEM_CONFIG_ONCE:        status = MotNvm_WriteManufacture_Blocking(&p_dev->MOT_NVM, p_rxPayload->Address, p_rxPayload->ByteData, p_rxPayload->Size); break;
        case MOT_PROTOCOL_MEM_CONFIG_BOARD_REF_0: status = MotNvm_WritePhaseCalibration(&p_dev->MOT_NVM, (const Phase_Calibration_T *)p_rxPayload->ByteData); break;
        case MOT_PROTOCOL_MEM_CONFIG_BOARD_REF_1: status = MotNvm_WriteMotorCalibration(&p_dev->MOT_NVM, (const Motor_ElectricalCalib_T *)p_rxPayload->ByteData); break;
        default: status = NV_MEMORY_STATUS_ERROR_NOT_IMPLEMENTED; break;
    }

    p_txPayload->Status = status;

    p_xfer->p_TxMeta->Id = MOT_PACKET_MEM_WRITE;
    p_xfer->p_TxMeta->Length = sizeof(MotPacket_MemWriteResp_T);
    return PROTOCOL_REQ_DONE;
}

/******************************************************************************/
/*!
    Stateful Data Mode - the engine's resumable path, delegated to MotProtocol

    These two keep void payload pointers where every handler above is typed, because their
    payload type is a function of Step rather than of the id: the opening frame carries a
    MotPacket_DataModeReq_T, every continuation carries raw bytes, and the response alternates
    between MotPacket_DataModeResp_T and a data chunk. A single declared type would be wrong
    on all but one call.
*/
/******************************************************************************/
#if defined(MOTOR_CONTROLLER_FLASH_LOADER_ENABLE)
/*
    The transfers are generic; only the binding is local. The interface is four pointers, so
    it is built per call rather than forcing the Flash instance to be a file-scope constant.

    Protocol_ProcReqResp_T fixes the context as void *, which cannot carry the interface's
    const, so the cast is at the boundary rather than hidden in the handler.
*/
static Protocol_ReqCode_T ReadData(MotorController_T * p_dev, Packet_Xfer_T * p_xfer, const void * p_rxPayload, void * p_txPayload)
{
    Protocol_DataModeInterface_T dataMode = MOT_PROTOCOL_FLASH_LOADER(p_dev->MOT_NVM.P_FLASH);
    return Protocol_DataMode_Read((void *)&dataMode, p_xfer, p_rxPayload, p_txPayload);
}

static Protocol_ReqCode_T WriteData_Blocking(MotorController_T * p_dev, Packet_Xfer_T * p_xfer, const void * p_rxPayload, void * p_txPayload)
{
    Protocol_DataModeInterface_T dataMode = MOT_PROTOCOL_FLASH_LOADER(p_dev->MOT_NVM.P_FLASH);
    return Protocol_DataMode_Write((void *)&dataMode, p_xfer, p_rxPayload, p_txPayload);
}
#endif


/******************************************************************************/
/*! Req Table

    Ack policy per handler:
        ACK_NONE        stateless exchange - the response is the only reply.
        ACK_ON_REQ      ack-paced - the request is acked on arrival and the response awaits
                        an ack, which is what drives a continuation.

    DataModeWrite is deliberately ACK_NONE: it is data-paced, so the handler's own
    ACCEPT / REJECT on each chunk is the acknowledgment and no ack round trip is inserted.
*/
/******************************************************************************/
const Protocol_Req_T MOTOR_CONTROLLER_MOT_PROTOCOL_REQ_TABLE[MOTOR_CONTROLLER_MOT_PROTOCOL_REQ_TABLE_LENGTH] =
{
    PROTOCOL_REQ(MOT_PACKET_PING,               Ping,               PROTOCOL_ACK_NONE),
    PROTOCOL_REQ(MOT_PACKET_STOP_ALL,           StopAll,            PROTOCOL_ACK_NONE),
    PROTOCOL_REQ(MOT_PACKET_VERSION,            Version,            PROTOCOL_ACK_NONE),
    PROTOCOL_REQ(MOT_PACKET_CALL,               Call_Blocking,      PROTOCOL_ACK_NONE),
    PROTOCOL_REQ(MOT_PACKET_VAR16_READ,         Var16Read,          PROTOCOL_ACK_NONE),
    PROTOCOL_REQ(MOT_PACKET_VAR16_WRITE,        Var16Write,         PROTOCOL_ACK_NONE),
    PROTOCOL_REQ(MOT_PACKET_VAR32_READ,         ReadVar32,          PROTOCOL_ACK_NONE),
    PROTOCOL_REQ(MOT_PACKET_VAR32_WRITE,        WriteVar32,         PROTOCOL_ACK_NONE),
    PROTOCOL_REQ(MOT_PACKET_FIXED_VAR_READ,     ReadVar,            PROTOCOL_ACK_NONE),
    PROTOCOL_REQ(MOT_PACKET_FIXED_VAR_WRITE,    WriteVar,           PROTOCOL_ACK_NONE),
    PROTOCOL_REQ(MOT_PACKET_MEM_READ,           ReadMem_Blocking,   PROTOCOL_ACK_NONE),
    PROTOCOL_REQ(MOT_PACKET_MEM_WRITE,          WriteMem_Blocking,  PROTOCOL_ACK_NONE),
#if defined(MOTOR_CONTROLLER_FLASH_LOADER_ENABLE)
    PROTOCOL_REQ(MOT_PACKET_DATA_MODE_READ,     ReadData,           PROTOCOL_ACK_ON_REQ),
    PROTOCOL_REQ(MOT_PACKET_DATA_MODE_WRITE,    WriteData_Blocking, PROTOCOL_ACK_NONE),
#endif
};
