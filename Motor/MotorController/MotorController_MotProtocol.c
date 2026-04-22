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
static packet_size_t Ping(const MotorController_T * p_dev, MotPacket_PingResp_T * p_txPacket, const MotPacket_PingReq_T * p_rxPacket)
{
    (void)p_rxPacket;
    // MotorController_BeepN(p_dev, 500U, 500U, 1U);
    MotorController_BeepShort(p_dev);
    return MotPacket_PingResp_Build(p_txPacket, MOT_PACKET_SYNC_ACK);
}

/******************************************************************************/
/*! Version */
/******************************************************************************/
// typedef struct PACKET_PACKED MotorController_Version
// {
//     Version_T MainVersion;
//     Version_T LibraryVersion;
//     Version_T ProtocolVersion;
//     // Version_T HardwareVersion;
//     // Version_T BootloaderVersion;
//     uint8_t MotorCount;
//     uint8_t VMonitorCount;
//     uint8_t ThermistorMosfetsCount;
//     uint8_t ProtocolSocketCount;
//     uint8_t CanSocketCount;
// }
// MotorController_Version_T;

// static packet_size_t Version(const MotorController_T * p_dev, MotPacket_VersionResp_T * p_txPayload, const MotPacket_VersionReq_T * p_rxPayload)
// {
//     (void)p_rxPayload;
//     MotorController_Version_T * p_payload = (MotorController_Version_T *)p_txPayload;

//     p_payload->MainVersion.Value = p_dev->MAIN_VERSION.Value;
//     p_payload->LibraryVersion.Value = MOTOR_LIBRARY_VERSION;
//     p_payload->ProtocolVersion.Value = MOT_PACKET_VERSION_WORD32;
//     // p_payload->HardwareVersion.Value = p_dev->HARDWARE_VERSION.Value;
//     // p_payload->BootloaderVersion.Value = p_dev->BOOTLOADER_VERSION.Value;
//     p_payload->MotorCount = p_dev->MOTORS.LENGTH;
//     p_payload->VMonitorCount = 3U;
//     p_payload->ThermistorMosfetsCount = p_dev->HEAT_MOSFETS.COUNT;
//     p_payload->ProtocolSocketCount = p_dev->PROTOCOL_COUNT;
// #if defined(MOTOR_CONTROLLER_CAN_BUS_ENABLE)
//     p_payload->CanSocketCount = p_dev->CAN_SOCKET_COUNT;
// #else
//     p_payload->CanSocketCount = 0U;
// #endif
// return MotPacket_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_VERSION, sizeof(MotPacket_VersionResp_T));
//     return sizeof(MotorController_Version_T);
// }

static packet_size_t Version(const MotorController_T * p_dev, MotPacket_T * p_txPacket, const MotPacket_T * p_rxPacket)
{
    (void)p_rxPacket;
    return MotPacket_VersionResp_Build(p_txPacket, p_dev->MAIN_VERSION.Word32.Value32);

    // with variable length
    // return MotPacket_VersionFlexResp_Build(p_txPacket, MotorController_GetLibraryVersion(), MotorController_GetMainVersion(p_dev), 0);
}

/******************************************************************************/
/*! Stop All */
/******************************************************************************/
static packet_size_t StopAll(const MotorController_T * p_dev, MotPacket_T * p_txPacket, const MotPacket_T * p_rxPacket)
{
    (void)p_rxPacket;
    MotorController_ForceDisableControl(p_dev);
    // ((MotPacket_StopResp_T *)p_txPacket->Payload)->Status = MOT_STATUS_SUCCESS;
    return MotPacket_StopResp_Build(p_txPacket, MOT_STATUS_SUCCESS);
}

/******************************************************************************/
/*! Call - May be Blocking */
/******************************************************************************/
/* Generic status response, type depending on input */
static packet_size_t Call_Blocking(const MotorController_T * p_dev, MotPacket_T * p_txPacket, const MotPacket_T * p_rxPacket)
{
    const MotPacket_CallReq_T * p_req = (const MotPacket_CallReq_T *)p_rxPacket->Payload;
    uint16_t status = MotorController_CallSystemCmd(p_dev, (MotorController_SystemCmd_T)p_req->Id, p_req->Arg);
    return MotPacket_CallResp_Build(p_txPacket, p_req->Id, status);
}

/******************************************************************************/
/*! Read Vars */
/******************************************************************************/
/* Resp Truncates 32-Bit Vars */
static packet_size_t VarRead(const MotorController_T * p_dev, MotPacket_T * p_txPacket, const MotPacket_T * p_rxPacket)
{
    // const MotPacket_VarReadReq_T * p_req = (const MotPacket_VarReadReq_T *)p_rxPacket->Payload;
    // MotPacket_VarReadResp_T * p_resp = (MotPacket_VarReadResp_T *)p_txPacket->Payload;
    uint8_t varCount = MotPacket_VarReadReq_ParseVarIdCount(p_rxPacket);

    // for (uint8_t index = 0U; index < varCount; index++)
    // {
    //     p_resp->Value16[index] = (uint16_t)MotorController_Var_Get(p_dev, (MotVarId_T)p_req->MotVarIds[index]);
    // }

    MotorController_BuildVarRead(p_dev, (MotPacket_VarReadResp_T *)p_txPacket->Payload, (const MotPacket_VarReadReq_T *)p_rxPacket->Payload, varCount);
    return MotPacket_VarReadResp_BuildHeader(p_txPacket, varCount);
}

/******************************************************************************/
/*! Write Vars */
/******************************************************************************/
static packet_size_t VarWrite(const MotorController_T * p_dev, MotPacket_T * p_txPacket, const MotPacket_T * p_rxPacket)
{
    const MotPacket_VarWriteReq_T * p_req = (const MotPacket_VarWriteReq_T *)p_rxPacket->Payload;
    MotPacket_VarWriteResp_T * p_resp = (MotPacket_VarWriteResp_T *)p_txPacket->Payload;
    uint8_t varCount = MotPacket_VarWriteReq_ParseVarCount(p_rxPacket);
    MotVarId_Status_T headerStatus = MOT_VAR_STATUS_OK;

    for (uint8_t index = 0U; index < varCount; index++)
    {
        p_resp->VarStatus[index] = MotorController_Var_Set(p_dev, (MotVarId_T)p_req->Pairs[index].MotVarId, p_req->Pairs[index].Value16);
        if (p_resp->VarStatus[index] != MOT_VAR_STATUS_OK) { headerStatus = MOT_VAR_STATUS_ERROR; }
    }
    return MotPacket_VarWriteResp_BuildHeader(p_txPacket, varCount);
}

/******************************************************************************/
/*! Mem */
/* with StateMachine check and address virtualization */
/******************************************************************************/
/******************************************************************************/
/* Manufacture */
/******************************************************************************/
/*
    Multi variable StateMachine call
    Use outer layer StateMachine check, simplifies handling of signature type.
*/
NvMemory_Status_T MotorController_ReadManufacture_Blocking(const MotorController_T * p_dev, uintptr_t onceAddress, uint8_t size, uint8_t * p_destBuffer)
{
    NvMemory_Status_T status = NV_MEMORY_STATUS_ERROR_OTHER;
    if (MotorController_IsConfig(p_dev) == true) { status = MotNvm_ReadManufacture_Blocking(&p_dev->MOT_NVM, onceAddress, size, p_destBuffer); }
    return status;
}

NvMemory_Status_T MotorController_WriteManufacture_Blocking(const MotorController_T * p_dev, uintptr_t onceAddress, const uint8_t * p_source, uint8_t size)
{
    NvMemory_Status_T status = NV_MEMORY_STATUS_ERROR_OTHER;
    if (MotorController_IsConfig(p_dev) == true) { status = MotNvm_WriteManufacture_Blocking(&p_dev->MOT_NVM, onceAddress, p_source, size); }
    return status;
}

/*
*/
static packet_size_t ReadMem_Blocking(const MotorController_T * p_dev, MotPacket_T * p_txPacket, const MotPacket_T * p_rxPacket)
{
    const MotPacket_MemReadReq_T * p_req = (const MotPacket_MemReadReq_T *)p_rxPacket->Payload;
    uint8_t * p_buffer = p_txPacket->Payload;
    NvMemory_Status_T status;

    memset(p_buffer, 0U, p_req->Size);

    switch ((MotProtocol_MemConfig_T)p_req->Config)
    {
        case MOT_MEM_CONFIG_RAM:  memcpy(p_buffer, (void *)p_req->Address, p_req->Size); status = NV_MEMORY_STATUS_SUCCESS; break;
        case MOT_MEM_CONFIG_ONCE: status = MotorController_ReadManufacture_Blocking(p_dev, p_req->Address, p_req->Size, p_buffer); break;
        default: status = NV_MEMORY_STATUS_ERROR_NOT_IMPLEMENTED; break;
            // case MOT_MEM_CONFIG_FLASH: memcpy(p_buffer, (void *)address, size); status = NV_MEMORY_STATUS_SUCCESS; break;
    }

    return MotPacket_MemReadResp_BuildHeader(p_txPacket, p_req->Size, status);
}

static packet_size_t WriteMem_Blocking(const MotorController_T * p_dev, MotPacket_T * p_txPacket, const MotPacket_T * p_rxPacket)
{
    const MotPacket_MemWriteReq_T * p_req = (const MotPacket_MemWriteReq_T *)p_rxPacket->Payload;
    NvMemory_Status_T status;

    switch ((MotProtocol_MemConfig_T)p_req->Config)
    {
        case MOT_MEM_CONFIG_ONCE: status = MotorController_WriteManufacture_Blocking(p_dev, p_req->Address, p_req->ByteData, p_req->Size); break;
        default: status = NV_MEMORY_STATUS_ERROR_NOT_IMPLEMENTED; break;
        // case MOT_MEM_CONFIG_RAM: memcpy((void *)address, p_data, size); status = NV_MEMORY_STATUS_SUCCESS; break;
        // case MOT_MEM_CONFIG_FLASH: status = Flash_Write_Blocking(p_flash, address, p_data, size); break;
    }

    return MotPacket_MemWriteResp_Build(p_txPacket, status);
}


// #if defined(MOTOR_CONTROLLER_FLASH_LOADER_ENABLE)
/******************************************************************************/
/*! Stateful Read Data */
/******************************************************************************/
static Protocol_ReqCode_T ReadData(const MotorController_T * p_dev, Protocol_ReqContext_T * p_reqContext)
{
    // if (MotorController_IsLock(p_dev) == true)
    return MotProtocol_ReadData(NULL, p_reqContext);
}

/******************************************************************************/
/*! Stateful Write Data */
/******************************************************************************/
static Protocol_ReqCode_T WriteData_Blocking(const MotorController_T * p_dev, Protocol_ReqContext_T * p_reqContext)
{
    return MotProtocol_Flash_WriteData_Blocking(p_dev->MOT_NVM.P_FLASH, p_reqContext);
}
// #endif

// /******************************************************************************/
// /*! Read Single Var */
// /******************************************************************************/
// static uint8_t ReadVar(const MotorController_T * p_dev, MotPacket_ReadVarResp_T * p_txPacket, const MotPacket_ReadVarReq_T * p_rxPacket)
// {
//     return MotPacket_ReadVarResp_Build(p_txPacket, MotorController_Var_Get(p_dev, (MotVarId_T)MotPacket_ReadVarReq_ParseVarId(p_rxPacket)));
// }
// /******************************************************************************/
// /*! Write Single Var */
// /******************************************************************************/
// static uint8_t WriteVar(const MotorController_T * p_dev, MotPacket_WriteVarResp_T * p_txPacket, const MotPacket_WriteVarReq_T * p_rxPacket)
// {
//     MotPacket_Status_T status = MOT_VAR_STATUS_OK;
//     if(MotorController_Var_Set(p_dev, (MotVarId_T)MotPacket_WriteVarReq_ParseVarId(p_rxPacket), MotPacket_WriteVarReq_ParseVarValue(p_rxPacket)) == 0U)
//         { status = MOT_VAR_STATUS_ERROR; }
//     return MotPacket_WriteVarResp_Build(p_txPacket, status);
// }

/******************************************************************************/
/*! Req Table */
/******************************************************************************/
#define REQ_SYNC_DATA_MODE PROTOCOL_SYNC_EXT(1U, 1U, 1U, 1U, 3U)

const Protocol_Req_T MOTOR_CONTROLLER_MOT_PROTOCOL_REQ_TABLE[] =
{
    PROTOCOL_REQ(MOT_PACKET_PING,           Ping,               0U,     PROTOCOL_SYNC_DISABLE),
    PROTOCOL_REQ(MOT_PACKET_STOP_ALL,       StopAll,            0U,     PROTOCOL_SYNC_DISABLE),
    PROTOCOL_REQ(MOT_PACKET_VERSION,        Version,            0U,     PROTOCOL_SYNC_DISABLE),
    PROTOCOL_REQ(MOT_PACKET_CALL,           Call_Blocking,      0U,     PROTOCOL_SYNC_DISABLE),
    PROTOCOL_REQ(MOT_PACKET_VAR_WRITE,      VarWrite,           0U,     PROTOCOL_SYNC_DISABLE),
    PROTOCOL_REQ(MOT_PACKET_VAR_READ,       VarRead,            0U,     PROTOCOL_SYNC_DISABLE),
    PROTOCOL_REQ(MOT_PACKET_MEM_READ,       ReadMem_Blocking,   0U,     PROTOCOL_SYNC_DISABLE),
    PROTOCOL_REQ(MOT_PACKET_MEM_WRITE,      WriteMem_Blocking,  0U,     PROTOCOL_SYNC_DISABLE),
    // PROTOCOL_REQ(MOT_PACKET_WRITE_VAR,       WriteVar,           0U,     PROTOCOL_SYNC_DISABLE),
    // PROTOCOL_REQ(MOT_PACKET_READ_VAR,        ReadVar,            0U,     PROTOCOL_SYNC_DISABLE),
    PROTOCOL_REQ(MOT_PACKET_DATA_MODE_READ,      0U,     ReadData,               REQ_SYNC_DATA_MODE),
    PROTOCOL_REQ(MOT_PACKET_DATA_MODE_WRITE,     0U,     WriteData_Blocking,     REQ_SYNC_DATA_MODE),
    // PROTOCOL_REQ(MOT_PACKET_DATA_MODE_DATA,      0U,     ReadData,               REQ_SYNC_DATA_MODE),
// #if defined(MOTOR_CONTROLLER_FLASH_LOADER_ENABLE)
// #endif
};

