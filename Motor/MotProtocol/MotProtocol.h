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
    @file   MotProtocol.h
    @author FireSourcery
    @brief

*/
/******************************************************************************/
#ifndef MOT_PROTOCOL_H
#define MOT_PROTOCOL_H

#include "MotPacket.h"
#include "Framework/Protocol/Extension/Protocol_FlashLoader.h"
#include "Framework/Protocol/Protocol.h"
#include "Peripheral/NvMemory/Flash/Flash.h"

#define MOT_PROTOCOL_BAUD_RATE_DEFAULT  (19200U)
#define MOT_PROTOCOL_TIMEOUT_RX         (2000U)     /* Timeout Rx Packet */
#define MOT_PROTOCOL_TIMEOUT_REQ        (5000U)     /* Timeout Req */

/* Common status */
typedef enum MotProtocol_StatusCode
{
    MOT_STATUS_SUCCESS           = 0x00U,
    MOT_STATUS_INVALID_COMMAND   = 0x01U,
    MOT_STATUS_INVALID_PARAMETER = 0x02U,
    MOT_STATUS_DEVICE_BUSY       = 0x03U,
    MOT_STATUS_CHECKSUM_ERROR    = 0x04U,
    MOT_STATUS_TIMEOUT           = 0x05U,
    MOT_STATUS_ACCESS_DENIED     = 0x06U,
}
MotProtocol_StatusCode_T;

/* expandable to bitfields */
typedef enum MotProtocol_MemConfig
{
    MOT_PROTOCOL_MEM_CONFIG_RAM = 0x00U,
    MOT_PROTOCOL_MEM_CONFIG_FLASH = 0x01U,
    MOT_PROTOCOL_MEM_CONFIG_EEPROM = 0x02U,
    MOT_PROTOCOL_MEM_CONFIG_ONCE = 0x03U,
    MOT_PROTOCOL_MEM_CONFIG_BOARD_REF_0 = 0x04U,
    MOT_PROTOCOL_MEM_CONFIG_BOARD_REF_1 = 0x05U,
    MOT_PROTOCOL_MEM_CONFIG_BOARD_REF_2 = 0x06U,
    MOT_PROTOCOL_MEM_CONFIG_BOARD_REF_3 = 0x07U,
    MOT_PROTOCOL_MEM_CONFIG_RESERVED = 0xFFU,
}
MotProtocol_MemConfig_T;




/*
    Stateful handlers. Both are resumable: Step is the resume point, the sub-state is
    MotProtocol_DataModeState_T, and payload pointers arrive already offset past the header.

    Read is ack-paced   - RESPOND a chunk, wait for the ack, RESPOND the next.
    Write is data-paced - ACCEPT or REJECT each arriving chunk, no ack round trip.
*/
/*
    Bulk transfer is Protocol_DataMode_Read / Protocol_DataMode_Write, bound to Flash by
    Protocol_FlashLoader.h. Register those directly and hand them an interface built here -
    the integration layer supplies the Flash instance.
*/
#define MOT_DATA_MODE_CHUNK_MAX ((packet_size_t)(MOT_PACKET_LENGTH_MAX - sizeof(MotPacket_Header_T)))

#define MOT_PROTOCOL_FLASH_LOADER(p_Flash) PROTOCOL_FLASH_LOADER(p_Flash, MOT_PACKET_DATA_MODE_DATA, MOT_DATA_MODE_CHUNK_MAX)

extern Protocol_ReqCode_T MotProtocol_EraseFlash_Blocking(Flash_T * p_flash, Packet_Xfer_T * p_xfer, const MotPacket_DataModeReq_T * p_req, MotPacket_DataModeResp_T * p_resp);
// extern Protocol_ReqCode_T MotProtocol_Flash_Erase_Blocking(Flash_T * p_flash, Protocol_ReqContext_T * p_reqContext);
// extern packet_size_t MotProtocol_Flash_WriteOnce_Blocking(Flash_T * p_flash, MotPacket_OnceWriteResp_T * p_txPacket, const MotPacket_OnceWriteReq_T * p_rxPacket);
// extern packet_size_t MotProtocol_Flash_ReadOnce_Blocking(Flash_T * p_flash, MotPacket_OnceReadResp_T * p_txPacket, const MotPacket_OnceReadReq_T * p_rxPacket);

#endif

