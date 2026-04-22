#pragma once

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
    @file   HAL_CAN.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Peripheral/HAL/HAL_Peripheral.h"
#include HAL_PERIPHERAL_PATH(HAL_Types.h)



/* split data handling, unless sequence/launch constraints */
/*
    4 type combinations by function
    1. standard vs extended
    2. data vs remote
    HAL_CAN_WriteTxExtendedId(HAL_CAN_T * p_hal, uint32_t id);
    HAL_CAN_WriteTxStandardId(HAL_CAN_T * p_hal, uint32_t id);
    HAL_CAN_WriteTxExtendedIdRemote(HAL_CAN_T * p_hal, uint32_t id);
    HAL_CAN_WriteTxStandardIdRemote(HAL_CAN_T * p_hal, uint32_t id);
*/
static inline void HAL_CAN_WriteTxExtendedId(HAL_CAN_T * p_hal, uint32_t id);
static inline void HAL_CAN_WriteTxStandardId(HAL_CAN_T * p_hal, uint32_t id);
static inline void HAL_CAN_WriteTxRemote(HAL_CAN_T * p_hal, bool isRemote);
static inline void HAL_CAN_WriteTxData(HAL_CAN_T * p_hal, const uint8_t * p_data, uint8_t length);

static inline bool HAL_CAN_ReadRxExtendedFlag(HAL_CAN_T * p_hal);
static inline uint32_t HAL_CAN_ReadRxStandardId(HAL_CAN_T * p_hal);
static inline uint32_t HAL_CAN_ReadRxExtendedId(HAL_CAN_T * p_hal);
static inline bool HAL_CAN_ReadRxRemoteFlag(HAL_CAN_T * p_hal);
static inline uint8_t HAL_CAN_ReadRxData(HAL_CAN_T * p_hal, uint8_t * p_data);

static inline bool HAL_CAN_ReadTxEmptyFlag(HAL_CAN_T * p_hal);
static inline bool HAL_CAN_ReadRxFullFlag(HAL_CAN_T * p_hal);
static inline void HAL_CAN_EnableTxEmptyInterrupt(HAL_CAN_T * p_hal);
static inline void HAL_CAN_EnableRxFullInterrupt(HAL_CAN_T * p_hal);
static inline void HAL_CAN_DisableTxEmptyInterrupt(HAL_CAN_T * p_hal);
static inline void HAL_CAN_DisableRxFullInterrupt(HAL_CAN_T * p_hal);
static inline void HAL_CAN_ClearRxFullFlag(HAL_CAN_T * p_hal);

/*

*/
static inline uint8_t HAL_CAN_MapMessageBufferIndex(HAL_CAN_T * p_hal, uint8_t userId);
// static inline uint8_t HAL_CAN_MapTxMessageBufferIndex(HAL_CAN_T * p_hal, uint8_t userId);
// static inline uint8_t HAL_CAN_MapRxMessageBufferIndex(HAL_CAN_T * p_hal, uint8_t userId);

static inline bool HAL_CAN_ReadTxComplete(HAL_CAN_T * p_hal, uint8_t hwIndex);
static inline bool HAL_CAN_ReadTxRemoteRxEmpty(HAL_CAN_T * p_hal, uint8_t hwIndex);
static inline bool HAL_CAN_ReadTxRemoteRxFull(HAL_CAN_T * p_hal, uint8_t hwIndex);
static inline bool HAL_CAN_ReadRxComplete(HAL_CAN_T * p_hal, uint8_t hwIndex);

// static inline void HAL_CAN_ClearTxInterrupt(HAL_CAN_T * p_hal, uint8_t hwIndex);
// static inline void HAL_CAN_EnableTxInterrupt(HAL_CAN_T * p_hal, uint8_t hwIndex);
// static inline void HAL_CAN_DisableTxInterrupt(HAL_CAN_T * p_hal, uint8_t hwIndex);
// static inline void HAL_CAN_ClearRxInterrupt(HAL_CAN_T * p_hal, uint8_t hwIndex);
// static inline void HAL_CAN_EnableRxInterrupt(HAL_CAN_T * p_hal, uint8_t hwIndex);
// static inline void HAL_CAN_DisableRxInterrupt(HAL_CAN_T * p_hal, uint8_t hwIndex);

static inline bool HAL_CAN_LockRx(HAL_CAN_T * p_hal, uint8_t hwIndex);
static inline void HAL_CAN_UnlockRx(HAL_CAN_T * p_hal, uint8_t hwIndex);


static inline void HAL_CAN_InitBaudRate(HAL_CAN_T * p_hal, uint32_t baudRate);
static inline void HAL_CAN_Init(HAL_CAN_T * p_hal);

/* unmap status at hal layer if needed */
typedef enum
{
    CAN_BUS_MESSAGE_BUFFER_IDLE,
    CAN_BUS_MESSAGE_BUFFER_RX_BUSY,
    CAN_BUS_MESSAGE_BUFFER_TX_BUSY,
    CAN_BUS_MESSAGE_BUFFER_RX_FIFO_BUSY,
    CAN_BUS_MESSAGE_BUFFER_COMPLETE,
    CAN_BUS_MESSAGE_BUFFER_TX_REMOTE,
    CAN_BUS_MESSAGE_BUFFER_RX_REMOTE,
    CAN_BUS_MESSAGE_BUFFER_DMA_ERROR
}
HAL_CAN_DriverStatus_T;
// static inline HAL_CAN_DriverStatus_T HAL_CAN_ReadTxStatus(HAL_CAN_T * p_hal, uint8_t hwIndex);
// static inline HAL_CAN_DriverStatus_T HAL_CAN_ReadRxStatus(HAL_CAN_T * p_hal, uint8_t hwIndex);
// static inline HAL_CAN_DriverStatus_T HAL_CAN_ReadErrorStatus(HAL_CAN_T * p_hal, uint8_t hwIndex);

#include HAL_PERIPHERAL_PATH(HAL_CAN.h)



/* using data interface */
/*
    SocketCAN convention
*/
/* Flag bits in upper 3 bits of Id, following SocketCAN convention */
#define CAN_ID_FLAG_EXT     0x80000000U
#define CAN_ID_FLAG_RTR     0x40000000U
#define CAN_ID_FLAG_ERR     0x20000000U
#define CAN_ID_MASK_EXT     0x1FFFFFFFU  /* 29-bit */
#define CAN_ID_MASK_STD     0x000007FFU  /* 11-bit */

typedef union
{
    uint32_t CanId;
    struct
    {
        uint32_t Id : 29;
        uint32_t Err : 1;
        uint32_t Rtr : 1;
        uint32_t Eff : 1;
    };
}
can_id_t;

typedef struct __attribute__((packed))
{
    can_id_t CanId;
    uint8_t DataLength;
    uint8_t Opt;
    uint8_t Resv0;
    uint8_t Resv1;
    uint8_t Data[8];
}
CAN_Frame_T;

static inline void HAL_CAN_WriteTxId(HAL_CAN_T * p_hal, can_id_t id)
{
    if (id.Eff == true) { HAL_CAN_WriteTxExtendedId(p_hal, id.Id); } else { HAL_CAN_WriteTxStandardId(p_hal, id.Id); }
    if (id.Rtr == true) { HAL_CAN_WriteTxRemote(p_hal, true); }
}

static inline can_id_t HAL_CAN_ReadRxId(HAL_CAN_T * p_hal)
{
    can_id_t id;
    id.Eff = HAL_CAN_ReadRxExtendedFlag(p_hal);
    id.Rtr = HAL_CAN_ReadRxRemoteFlag(p_hal);
    id.Id = (id.Eff) ? HAL_CAN_ReadRxExtendedId(p_hal) : HAL_CAN_ReadRxStandardId(p_hal);
    return id;
}

static inline void HAL_CAN_WriteTxMessage(HAL_CAN_T * p_hal, const CAN_Frame_T * p_txFrame)
{
    HAL_CAN_WriteTxId(p_hal, p_txFrame->CanId);
    HAL_CAN_WriteTxData(p_hal, p_txFrame->Data, p_txFrame->DataLength); /* remote write 0, call to launch */
}

static inline void HAL_CAN_ReadRxMessage(HAL_CAN_T * p_hal, CAN_Frame_T * p_rxFrame)
{
    p_rxFrame->CanId = HAL_CAN_ReadRxId(p_hal);
    p_rxFrame->DataLength = HAL_CAN_ReadRxData(p_hal, &p_rxFrame->Data[0U]);
}

