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
    @brief

*/
/******************************************************************************/
#include "MotorController_MotProtocol.h"
#include "Motor/MotProtocol/MotPacket.h"
#include "Motor/MotProtocol/MotProtocol.h"
#include "Motor/MotorController/MotorController_Var.h"
#include "Motor/MotorController/MotorController_User.h"

/******************************************************************************/
/*!
    Notes
    MotProtocol implementation using MotorController_T
    Directly use MotorController_T as Protocol interface (P_APP_CONTEXT) avoids double buffering
*/
/******************************************************************************/

/******************************************************************************/
/*!
    Request Response
*/
/******************************************************************************/

/******************************************************************************/
/*! Ping */
/******************************************************************************/
static packet_size_t Ping(MotorController_T * p_dev, MotPacket_PingResp_T * p_txPacket, const MotPacket_PingReq_T * p_rxPacket)
{
    (void)p_rxPacket;
    MotBuzzer_Short(MotorController_Buzzer(p_dev));
    return MotPacket_Sync_Build((MotPacket_Sync_T *)p_txPacket, MOT_PACKET_SYNC_ACK);
}

/******************************************************************************/
/*! Version */
/******************************************************************************/
static packet_size_t Version(MotorController_T * p_dev, MotPacket_T * p_txPacket, const MotPacket_T * p_rxPacket)
{
    (void)p_rxPacket;
    MotPacket_VersionResp_T * p_resp = (MotPacket_VersionResp_T *)p_txPacket->Payload;
    p_resp->Protocol = MOT_PACKET_VERSION_WORD32;
    p_resp->Library  = MOTOR_LIBRARY_VERSION;
    p_resp->Firmware = p_dev->MAIN_VERSION.Word32.Value32;
    // p_resp->Flags = MOTOR_VERSION_FLAGS;

    return MotPacket_BuildHeader(p_txPacket, MOT_PACKET_VERSION, sizeof(MotPacket_VersionResp_T));
}

/******************************************************************************/
/*! Stop All */
/******************************************************************************/
static packet_size_t StopAll(MotorController_T * p_dev, MotPacket_T * p_txPacket, const MotPacket_T * p_rxPacket)
{
    (void)p_rxPacket;
    MotorController_ForceDisableControl(p_dev);
    ((MotPacket_StopResp_T *)p_txPacket->Payload)->Status = MOT_STATUS_SUCCESS;

    return MotPacket_BuildHeader(p_txPacket, MOT_PACKET_STOP_ALL, sizeof(MotPacket_StopResp_T));
}

/******************************************************************************/
/*! Call - May be Blocking */
/******************************************************************************/
/* Generic status response, type depending on input */
static packet_size_t Call_Blocking(MotorController_T * p_dev, MotPacket_T * p_txPacket, const MotPacket_T * p_rxPacket)
{
    const MotPacket_CallReq_T * p_req = (const MotPacket_CallReq_T *)p_rxPacket->Payload;
    MotPacket_CallResp_T * p_resp = (MotPacket_CallResp_T *)p_txPacket->Payload;
    uint16_t status = MotorController_CallSystemCmd(p_dev, (MotorController_SystemCmd_T)p_req->Id, p_req->Arg);
    p_resp->Id     = p_req->Id;
    p_resp->Status = status;

    return MotPacket_BuildHeader(p_txPacket, MOT_PACKET_CALL, sizeof(MotPacket_CallResp_T));
}

/******************************************************************************/
/*! Read Single Var */
/******************************************************************************/
static uint8_t ReadVar(MotorController_T * p_dev, MotPacket_T * p_tx, const MotPacket_T * p_rx)
{
    MotorController_ReadVar(p_dev, (const MotPacket_VarReadFixedReq_T *)p_rx->Payload, (MotPacket_VarReadFixedResp_T *)p_tx->Payload);

    MotProtocol_BuildTxHeader(p_tx, &(Protocol_HeaderMeta_T){.Id = MOT_PACKET_FIXED_VAR_READ, .Length = sizeof(MotPacket_VarReadFixedResp_T) + sizeof(MotPacket_Header_T) });
    return p_tx->Header.Length;
}

/******************************************************************************/
/*! Write Single Var */
/******************************************************************************/
static uint8_t WriteVar(MotorController_T * p_dev, MotPacket_T * p_tx, const MotPacket_T * p_rx)
{
    MotorController_WriteVar(p_dev, (const MotPacket_VarWriteFixedReq_T *)p_rx->Payload, (MotPacket_VarWriteFixedResp_T *)p_tx->Payload);

    MotProtocol_BuildTxHeader(p_tx, &(Protocol_HeaderMeta_T){.Id = MOT_PACKET_FIXED_VAR_WRITE, .Length = sizeof(MotPacket_VarWriteFixedResp_T) + sizeof(MotPacket_Header_T) });
    return p_tx->Header.Length;
}


/*
    todo if callback signiture changes to payload scope,
        header handler when MotProtocol_BuildTxHeader moves to Protocol handler
    multi var read use Protocol_ReqContext_T or Protocol_ReqMeta_T
    static uint8_t ReadVar32(MotorController_T * p_dev, Protocol_ReqContext_T )
*/

/******************************************************************************/
/*! Read Vars */
/******************************************************************************/
/* Resp Truncates 32-Bit Vars */
static packet_size_t Var16Read(MotorController_T * p_dev, MotPacket_T * p_txPacket, const MotPacket_T * p_rxPacket)
{
    uint8_t varCount = MotPacket_ParsePayloadLength(p_rxPacket) / sizeof(uint16_t);
    MotorController_ReadVar16s(p_dev, (const MotPacket_VarReadReq_T *)p_rxPacket->Payload, (MotPacket_VarReadResp_T *)p_txPacket->Payload, varCount);

    return MotPacket_BuildHeader(p_txPacket, MOT_PACKET_VAR_READ, varCount * sizeof(uint16_t));
}

/******************************************************************************/
/*! Write Vars */
/******************************************************************************/
static packet_size_t Var16Write(MotorController_T * p_dev, MotPacket_T * p_txPacket, const MotPacket_T * p_rxPacket)
{
    uint8_t varCount = MotPacket_ParsePayloadLength(p_rxPacket) / sizeof(uint16_t) / 2U;
    MotorController_WriteVar16s(p_dev, (const MotPacket_VarWriteReq_T *)p_rxPacket->Payload, (MotPacket_VarWriteResp_T *)p_txPacket->Payload, varCount);

    return MotPacket_BuildHeader(p_txPacket, MOT_PACKET_VAR_WRITE, varCount * sizeof(uint8_t));
}

/******************************************************************************/
/*! Read Var32s */
/******************************************************************************/
static uint8_t ReadVar32(MotorController_T * p_dev, MotPacket_T * p_tx, const MotPacket_T * p_rx)
{
    uint8_t varCount = MotPacket_Var32Read_ParseCount(p_rx);
    MotorController_ReadVar32s(p_dev, (const MotPacket_Var32ReadReq_T *)p_rx->Payload, (MotPacket_Var32ReadResp_T *)p_tx->Payload, varCount);

    MotProtocol_BuildTxHeader(p_tx, &(Protocol_HeaderMeta_T){.Id = MOT_PACKET_VAR32_READ, .Length = varCount * sizeof(uint32_t) + sizeof(MotPacket_Header_T) });
    return p_tx->Header.Length;
}

/******************************************************************************/
/*! Write Var32s */
/******************************************************************************/
static uint8_t WriteVar32(MotorController_T * p_dev, MotPacket_T * p_tx, const MotPacket_T * p_rx)
{
    uint8_t varCount = MotPacket_Var32WriteReq_ParseCount(p_rx);
    MotorController_WriteVar32s(p_dev, (const MotPacket_Var32WriteReq_T *)p_rx->Payload, (MotPacket_Var32WriteResp_T *)p_tx->Payload, varCount);

    MotProtocol_BuildTxHeader(p_tx, &(Protocol_HeaderMeta_T){.Id = MOT_PACKET_VAR32_WRITE, .Length = varCount * sizeof(uint8_t) + sizeof(MotPacket_Header_T) });
    return p_tx->Header.Length;
}


/******************************************************************************/
/*! Mem */
/******************************************************************************/
/*
    Multi variable StateMachine call
    handle StateMachine check and address virtualization
    Use outer layer StateMachine check, simplifies handling of signature type.
    NvMemory access gated by a single MotorController_IsConfig check per request.
*/
static packet_size_t ReadMem_Blocking(MotorController_T * p_dev, MotPacket_T * p_txPacket, const MotPacket_T * p_rxPacket)
{
    const MotPacket_MemReadReq_T * p_req = (const MotPacket_MemReadReq_T *)p_rxPacket->Payload;
    uint8_t * p_buffer = p_txPacket->Payload;
    NvMemory_Status_T status;

    memset(p_buffer, 0U, p_req->Size);

    if (MotorController_IsConfig(p_dev) == false) { status = NV_MEMORY_STATUS_ERROR_OTHER; }
    else switch ((MotProtocol_MemConfig_T)p_req->Config)
    {
        case MOT_PROTOCOL_MEM_CONFIG_RAM:  memcpy(p_buffer, (void *)p_req->Address, p_req->Size); status = NV_MEMORY_STATUS_SUCCESS; break;
        case MOT_PROTOCOL_MEM_CONFIG_ONCE: status = MotNvm_ReadManufacture_Blocking(&p_dev->MOT_NVM, p_req->Address, p_req->Size, p_buffer); break;
        case MOT_PROTOCOL_MEM_CONFIG_BOARD_REF_0: status = NV_MEMORY_STATUS_SUCCESS; memcpy(p_buffer, &PHASE_CALIBRATION, sizeof(Phase_Calibration_T));                 break;
        case MOT_PROTOCOL_MEM_CONFIG_BOARD_REF_1: status = NV_MEMORY_STATUS_SUCCESS; memcpy(p_buffer, &MOTOR_ELECTRICAL_CALIBRATION, sizeof(Motor_ElectricalCalib_T));  break;
        default: status = NV_MEMORY_STATUS_ERROR_NOT_IMPLEMENTED; break;
        // case MOT_PROTOCOL_MEM_CONFIG_FLASH: memcpy(p_buffer, (void *)address, size); status = NV_MEMORY_STATUS_SUCCESS; break;
    }

    (void)status; /* MemRead header carries size only; status currently unused */
    return MotPacket_BuildHeader(p_txPacket, MOT_PACKET_MEM_READ, p_req->Size);
}

/* Host handle reboot */
static packet_size_t WriteMem_Blocking(MotorController_T * p_dev, MotPacket_T * p_txPacket, const MotPacket_T * p_rxPacket)
{
    const MotPacket_MemWriteReq_T * p_req = (const MotPacket_MemWriteReq_T *)p_rxPacket->Payload;
    NvMemory_Status_T status;

    if (MotorController_IsConfig(p_dev) == false) { status = NV_MEMORY_STATUS_ERROR_OTHER; }
    else switch ((MotProtocol_MemConfig_T)p_req->Config)
    {
        // case MOT_PROTOCOL_MEM_CONFIG_RAM: memcpy((void *)address, p_data, size); status = NV_MEMORY_STATUS_SUCCESS; break;
        case MOT_PROTOCOL_MEM_CONFIG_ONCE:            status = MotNvm_WriteManufacture_Blocking(&p_dev->MOT_NVM, p_req->Address, p_req->ByteData, p_req->Size); break;
        case MOT_PROTOCOL_MEM_CONFIG_BOARD_REF_0:     status = MotNvm_WritePhaseCalibration(&p_dev->MOT_NVM, (const Phase_Calibration_T *)p_req->ByteData); break;
        case MOT_PROTOCOL_MEM_CONFIG_BOARD_REF_1:     status = MotNvm_WriteMotorCalibration(&p_dev->MOT_NVM, (const Motor_ElectricalCalib_T *)p_req->ByteData); break;
        default: status = NV_MEMORY_STATUS_ERROR_NOT_IMPLEMENTED; break;
        // case MOT_PROTOCOL_MEM_CONFIG_FLASH: status = Flash_Write_Blocking(p_flash, address, p_data, size); break;
    }

    ((MotPacket_MemWriteResp_T *)p_txPacket->Payload)->Status = status;
    return MotPacket_BuildHeader(p_txPacket, MOT_PACKET_MEM_WRITE, sizeof(MotPacket_MemWriteResp_T));
}



/******************************************************************************/
/*! Stateful Read Data */
/******************************************************************************/
static Protocol_ReqCode_T ReadData(MotorController_T * p_dev, Protocol_ReqContext_T * p_reqContext)
{
    // if (MotorController_IsLock(p_dev) == true)
    return MotProtocol_ReadData(NULL, p_reqContext);
}

/******************************************************************************/
/*! Stateful Write Data */
/******************************************************************************/
static Protocol_ReqCode_T WriteData_Blocking(MotorController_T * p_dev, Protocol_ReqContext_T * p_reqContext)
{
    return MotProtocol_Flash_WriteData_Blocking(p_dev->MOT_NVM.P_FLASH, p_reqContext);
}


/******************************************************************************/
/*! Req Table */
/******************************************************************************/
#define REQ_SYNC_DATA_MODE PROTOCOL_SYNC_EXT(1U, 1U, 1U, 1U, 3U)

const Protocol_Req_T MOTOR_CONTROLLER_MOT_PROTOCOL_REQ_TABLE[MOTOR_CONTROLLER_MOT_PROTOCOL_REQ_TABLE_LENGTH] =
{
    PROTOCOL_REQ(MOT_PACKET_PING,           Ping,               0U,     PROTOCOL_SYNC_DISABLE),
    PROTOCOL_REQ(MOT_PACKET_STOP_ALL,       StopAll,            0U,     PROTOCOL_SYNC_DISABLE),
    PROTOCOL_REQ(MOT_PACKET_VERSION,        Version,            0U,     PROTOCOL_SYNC_DISABLE),
    PROTOCOL_REQ(MOT_PACKET_CALL,           Call_Blocking,      0U,     PROTOCOL_SYNC_DISABLE),
    PROTOCOL_REQ(MOT_PACKET_VAR_READ,       Var16Read,            0U,     PROTOCOL_SYNC_DISABLE),
    PROTOCOL_REQ(MOT_PACKET_VAR_WRITE,      Var16Write,           0U,     PROTOCOL_SYNC_DISABLE),
    PROTOCOL_REQ(MOT_PACKET_VAR32_READ,     ReadVar32,            0U,     PROTOCOL_SYNC_DISABLE),
    PROTOCOL_REQ(MOT_PACKET_VAR32_WRITE,    WriteVar32,           0U,     PROTOCOL_SYNC_DISABLE),
    PROTOCOL_REQ(MOT_PACKET_FIXED_VAR_READ,        ReadVar,            0U,     PROTOCOL_SYNC_DISABLE),
    PROTOCOL_REQ(MOT_PACKET_FIXED_VAR_WRITE,       WriteVar,           0U,     PROTOCOL_SYNC_DISABLE),
    PROTOCOL_REQ(MOT_PACKET_MEM_READ,           ReadMem_Blocking,   0U,     PROTOCOL_SYNC_DISABLE),
    PROTOCOL_REQ(MOT_PACKET_MEM_WRITE,          WriteMem_Blocking,  0U,     PROTOCOL_SYNC_DISABLE),
#if defined(MOTOR_CONTROLLER_FLASH_LOADER_ENABLE)
    PROTOCOL_REQ(MOT_PACKET_DATA_MODE_READ,      0U,     ReadData,               REQ_SYNC_DATA_MODE),
    PROTOCOL_REQ(MOT_PACKET_DATA_MODE_WRITE,     0U,     WriteData_Blocking,     REQ_SYNC_DATA_MODE),
    // PROTOCOL_REQ(MOT_PACKET_DATA_MODE_DATA,      0U,     ReadData,               REQ_SYNC_DATA_MODE),
#endif
};

