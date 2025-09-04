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
#include "Utility/Protocol/Protocol.h"
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
    MOT_MEM_CONFIG_RAM = 0x00U,
    MOT_MEM_CONFIG_FLASH = 0x01U,
    MOT_MEM_CONFIG_EEPROM = 0x02U,
    MOT_MEM_CONFIG_ONCE = 0x03U,
    MOT_MEM_CONFIG_RESERVED = 0xFFU,
}
MotProtocol_MemConfig_T;

typedef enum MotProtocol_DataModeConfig
{
    MOT_PROTOCOL_DATA_MODE_CONFIG_NONE = 0x00U,
    MOT_PROTOCOL_DATA_MODE_CONFIG_ERASE = 0x01U, /* Erase first */
}
MotProtocol_DataModeConfig_T;

typedef enum MotProtocol_DataModeStateId
{
    MOT_PROTOCOL_DATA_MODE_INACTIVE,
    MOT_PROTOCOL_DATA_MODE_READ_ACTIVE,
    MOT_PROTOCOL_DATA_MODE_WRITE_ACTIVE,
}
MotProtocol_DataModeStateId_T;


/* For Stateful DataMode Read/Write */
typedef struct MotProtocol_DataModeState
{
    uintptr_t DataModeAddress;
    size_t DataModeSize;
    size_t DataIndex;
    // MotProtocol_DataModeStateId_T StateId;
    // uint8_t buffer[256];
}
MotProtocol_DataModeState_T;

// typedef enum MotProtocol_DataModeStateId { IDLE, REQUESTING, RECEIVING, COMPLETE, ERROR } MotProtocol_DataModeStateId_T;

// static inline void MotProtocol_ResetSubState(MotProtocol_DataModeState_T * p_subState) { p_subState->StateIndex = 0U; }
// extern void MotProtocol_ResetSubState(MotProtocol_DataModeState_T * p_subState);

extern const Packet_Class_T MOT_PROTOCOL_PACKET_CLASS;

extern void MotProtocol_BuildTxSync(MotPacket_Sync_T * p_txPacket, packet_size_t * p_txSize, Protocol_TxSyncId_T txId);
extern Protocol_RxCode_T MotProtocol_ParseRxMeta(Protocol_HeaderMeta_T * p_rxMeta, const MotPacket_T * p_rxPacket, packet_size_t rxCount);

extern Protocol_ReqCode_T MotProtocol_ReadData(void * p_app, Protocol_ReqContext_T * p_reqContext);
extern Protocol_ReqCode_T MotProtocol_Flash_WriteData_Blocking(Flash_T * const p_flash, Protocol_ReqContext_T * p_reqContext);
// extern packet_size_t MotProtocol_Flash_WriteOnce_Blocking(Flash_T * p_flash, MotPacket_OnceWriteResp_T * p_txPacket, const MotPacket_OnceWriteReq_T * p_rxPacket);
// extern packet_size_t MotProtocol_Flash_ReadOnce_Blocking(Flash_T * p_flash, MotPacket_OnceReadResp_T * p_txPacket, const MotPacket_OnceReadReq_T * p_rxPacket);

#endif
