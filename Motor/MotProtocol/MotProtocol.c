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
    @file   MotProtocol.c
    @author FireSourcery
    @brief  MotPacket bound to the Framework/Protocol engine.
*/
/******************************************************************************/
#include "MotProtocol.h"
#include "MotPacket.h"
#include "Peripheral/NvMemory/Flash/Flash.h"

#include <stddef.h>
#include <string.h>



/******************************************************************************/
/*!
    Flash loader

    The transfers themselves are Protocol_DataMode_Read and Protocol_DataMode_Write, bound to
    Flash through Protocol_FlashLoader.h. What remains Mot-specific is the chunk ceiling and
    which id carries a raw chunk; the integration layer supplies the Flash instance and passes
    PROTOCOL_FLASH_LOADER(...) as the handler context.

    MotProtocol_ReadData and MotProtocol_Flash_WriteData_Blocking are gone - they were a
    second copy of that staging.
*/
/******************************************************************************/
/******************************************************************************/
/*! Erase */
/******************************************************************************/
Protocol_ReqCode_T MotProtocol_EraseFlash_Blocking(Flash_T * p_flash, Packet_Xfer_T * p_xfer, const MotPacket_DataModeReq_T * p_req, MotPacket_DataModeResp_T * p_resp)
{
    p_resp->Status = Flash_Erase_Blocking(p_flash, p_req->AddressStart, p_req->SizeBytes);
    p_xfer->p_TxMeta->Id     = MOT_PACKET_DATA_MODE_ERASE;
    p_xfer->p_TxMeta->Length = sizeof(MotPacket_DataModeResp_T);
    return PROTOCOL_REQ_DONE;
}


/******************************************************************************/
/*!
    Mem
*/
/******************************************************************************/
NvMemory_Status_T ReadMem_Blocking(Flash_T * p_flash, uintptr_t address, uint8_t size, MotProtocol_MemConfig_T config, uint8_t * p_destBuffer)
{
    NvMemory_Status_T status = NV_MEMORY_STATUS_ERROR_OTHER;

    switch ((MotProtocol_MemConfig_T)config)
    {
        case MOT_PROTOCOL_MEM_CONFIG_RAM: memcpy(p_destBuffer, (void *)address, size);  status = NV_MEMORY_STATUS_SUCCESS; break;
        case MOT_PROTOCOL_MEM_CONFIG_FLASH: memcpy(p_destBuffer, (void *)address, size); status = NV_MEMORY_STATUS_SUCCESS; break;
        case MOT_PROTOCOL_MEM_CONFIG_ONCE: status = Flash_ReadOnce_Blocking(p_flash, address, size, p_destBuffer); break;
        default: status = NV_MEMORY_STATUS_ERROR_NOT_IMPLEMENTED; break;
    }

    return status;
}

packet_size_t MotProtocol_ReadMem_Blocking(Flash_T * p_flash, MotPacket_T * p_txPacket, const MotPacket_T * p_rxPacket)
{
    const MotPacket_MemReadReq_T * p_req = (const MotPacket_MemReadReq_T *)p_rxPacket->Payload;
    NvMemory_Status_T status = ReadMem_Blocking(p_flash, p_req->Address, p_req->Size, (MotProtocol_MemConfig_T)p_req->Config, p_txPacket->Payload);
    (void)status; /* MemRead header carries size only; status currently unused */

    return MotPacket_BuildHeader(p_txPacket, MOT_PACKET_MEM_READ, p_req->Size);
}

NvMemory_Status_T WriteMem_Blocking(Flash_T * p_flash, uintptr_t address, uint8_t size, MotProtocol_MemConfig_T config, const uint8_t * p_data)
{
    NvMemory_Status_T status = NV_MEMORY_STATUS_ERROR_OTHER;

    switch ((MotProtocol_MemConfig_T)config)
    {
        case MOT_PROTOCOL_MEM_CONFIG_RAM: memcpy((void *)address, p_data, size);  status = NV_MEMORY_STATUS_SUCCESS; break;
        case MOT_PROTOCOL_MEM_CONFIG_FLASH: status = Flash_Write_Blocking(p_flash, address, p_data, size); break;
        case MOT_PROTOCOL_MEM_CONFIG_ONCE: status = Flash_WriteOnce_Blocking(p_flash, address, p_data, size); break;
        default: status = NV_MEMORY_STATUS_ERROR_NOT_IMPLEMENTED; break;
    }

    return status;
}

packet_size_t MotProtocol_WriteMem_Blocking(Flash_T * p_flash, MotPacket_T * p_txPacket, const MotPacket_T * p_rxPacket)
{
    const MotPacket_MemWriteReq_T * p_req = (const MotPacket_MemWriteReq_T *)p_rxPacket->Payload;
    NvMemory_Status_T status = WriteMem_Blocking(p_flash, p_req->Address, p_req->Size, (MotProtocol_MemConfig_T)p_req->Config, p_req->ByteData);
    ((MotPacket_MemWriteResp_T *)p_txPacket->Payload)->Status = status;

    return MotPacket_BuildHeader(p_txPacket, MOT_PACKET_MEM_WRITE, sizeof(MotPacket_MemWriteResp_T));
}
