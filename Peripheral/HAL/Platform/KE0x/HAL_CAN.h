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
    @brief  MSCAN HAL for KE0x (MKE06Z) - register-level access
            MSCAN has 3 Tx buffers (shared foreground register set) and 1 Rx buffer (FIFO-like).
            Unlike FlexCAN, there are no individual message buffer indices for Rx.
            Tx buffer selection is via CANTBSEL; Rx is always from the foreground buffer.
*/
/******************************************************************************/
#include "KE0x.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

typedef MSCAN_Type HAL_CanBus_T;

/******************************************************************************/
/*! MSCAN IDR bit layout helpers
    IDR0: [SID10:SID3] or [EID28:EID21]
    IDR1: [SID2:SID0 | RTR | IDE | 0 0 0] (std) or [EID20:EID18 | SRR=1 | IDE=1 | EID17:EID15] (ext)
    IDR2: (ext only) [EID14:EID7]
    IDR3: (ext only) [EID6:EID0 | RTR]
*/
/******************************************************************************/

/*
    Encode a 29-bit extended ID into the 4 IDR bytes.
    id29 bits [28:0] map to:
        IDR0 = id[28:21]
        IDR1 = id[20:18] | SRR=1 | IDE=1 | id[17:15]
        IDR2 = id[14:7]
        IDR3 = id[6:0] << 1 | RTR
*/
static inline void _HAL_CanBus_EncodeExtId(uint32_t id29, bool rtr, uint8_t * p_idr)
{
    p_idr[0] = (uint8_t)(id29 >> 21);
    p_idr[1] = (uint8_t)(((id29 >> 18) & 0x07U) << 5) | (1U << 4) | (1U << 3) | (uint8_t)((id29 >> 15) & 0x07U);
    p_idr[2] = (uint8_t)(id29 >> 7);
    p_idr[3] = (uint8_t)((id29 & 0x7FU) << 1) | (rtr ? 1U : 0U);
}

/*
    Encode an 11-bit standard ID into the 2 IDR bytes.
    id11 bits [10:0] map to:
        IDR0 = id[10:3]
        IDR1 = id[2:0] << 5 | RTR << 4 | IDE=0 | 0 0 0
*/
static inline void _HAL_CanBus_EncodeStdId(uint32_t id11, bool rtr, uint8_t * p_idr)
{
    p_idr[0] = (uint8_t)(id11 >> 3);
    p_idr[1] = (uint8_t)((id11 & 0x07U) << 5) | (rtr ? (1U << 4) : 0U);
}

/* Decode extended 29-bit ID from IDR registers */
static inline uint32_t _HAL_CanBus_DecodeExtId(uint8_t idr0, uint8_t idr1, uint8_t idr2, uint8_t idr3)
{
    return ((uint32_t)idr0 << 21) | (((uint32_t)(idr1 >> 5) & 0x07U) << 18) | (((uint32_t)idr1 & 0x07U) << 15) | ((uint32_t)idr2 << 7) | ((uint32_t)(idr3 >> 1) & 0x7FU);
}

/* Decode standard 11-bit ID from IDR registers */
static inline uint32_t _HAL_CanBus_DecodeStdId(uint8_t idr0, uint8_t idr1)
{
    return ((uint32_t)idr0 << 3) | ((uint32_t)(idr1 >> 5) & 0x07U);
}

/******************************************************************************/
/*! Unified HAL API using CanMessage_T
    hwBufferIndex is ignored for Rx (single foreground buffer).
    For Tx, hwBufferIndex is not directly used — MSCAN auto-selects via CANTBSEL.
*/
/******************************************************************************/

static inline void HAL_CanBus_WriteMessageBufferControl(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex, const CanMessage_T * p_message)
{
    (void)hwBufferIndex;
    bool rtr = (p_message->Type == CAN_FRAME_TYPE_REMOTE);

    /* Select an empty Tx buffer */
    uint8_t txEmpty = p_hal->CANTFLG & MSCAN_CANTFLG_TXE_MASK;
    p_hal->CANTBSEL = txEmpty;

    /* Write ID registers */
    if (p_message->Format == CAN_FRAME_FORMAT_EXTEND)
    {
        uint8_t idr[4];
        _HAL_CanBus_EncodeExtId(p_message->Id.Id, rtr, idr);
        p_hal->TEIDR0 = idr[0];
        p_hal->TEIDR1 = idr[1];
        p_hal->TEIDR2 = idr[2];
        p_hal->TEIDR3 = idr[3];
    }
    else
    {
        uint8_t idr[2];
        _HAL_CanBus_EncodeStdId(p_message->Id.Id, rtr, idr);
        p_hal->TSIDR0 = idr[0];
        p_hal->TSIDR1 = idr[1];
    }

    p_hal->TDLR = p_message->DataLength;
    p_hal->TBPR = p_message->Priority;
}

static inline void HAL_CanBus_WriteTxMessageBuffer(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex, const CanMessage_T * p_txMessage)
{
    (void)hwBufferIndex;
    bool rtr = (p_txMessage->Type == CAN_FRAME_TYPE_REMOTE);

    /* 1. Select an empty Tx buffer */
    uint8_t txEmpty = p_hal->CANTFLG & MSCAN_CANTFLG_TXE_MASK;
    p_hal->CANTBSEL = txEmpty;

    /* 2. Write ID registers */
    if (p_txMessage->Format == CAN_FRAME_FORMAT_EXTEND)
    {
        uint8_t idr[4];
        _HAL_CanBus_EncodeExtId(p_txMessage->Id.Id, rtr, idr);
        p_hal->TEIDR0 = idr[0];
        p_hal->TEIDR1 = idr[1];
        p_hal->TEIDR2 = idr[2];
        p_hal->TEIDR3 = idr[3];
    }
    else
    {
        uint8_t idr[2];
        _HAL_CanBus_EncodeStdId(p_txMessage->Id.Id, rtr, idr);
        p_hal->TSIDR0 = idr[0];
        p_hal->TSIDR1 = idr[1];
    }

    /* 3. Write data */
    for (uint8_t i = 0U; i < p_txMessage->DataLength; i++)
    {
        p_hal->TEDSR[i] = p_txMessage->Data[i];
    }

    /* 4. Write DLC and priority */
    p_hal->TDLR = p_txMessage->DataLength;
    p_hal->TBPR = p_txMessage->Priority;

    /* 5. Launch transmission — read back CANTBSEL to get the selected buffer, then clear its CANTFLG bit */
    p_hal->CANTFLG = p_hal->CANTBSEL & MSCAN_CANTFLG_TXE_MASK;
}

/*
    MSCAN Rx: single foreground buffer. Reading it dequeues the frame.
    hwBufferIndex is ignored.
*/
static inline void HAL_CanBus_ReadRxMessageBuffer(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex, CanMessage_T * p_rxMessage)
{
    (void)hwBufferIndex;

    uint8_t idr1 = p_hal->REIDR1;
    bool isExtended = ((idr1 >> 3) & 1U); /* IDE bit */

    if (isExtended)
    {
        p_rxMessage->Format = CAN_FRAME_FORMAT_EXTEND;
        p_rxMessage->Type = (p_hal->REIDR3 & 1U) ? CAN_FRAME_TYPE_REMOTE : CAN_FRAME_TYPE_DATA;
        p_rxMessage->Id.Id = _HAL_CanBus_DecodeExtId(p_hal->REIDR0, idr1, p_hal->REIDR2, p_hal->REIDR3);
    }
    else
    {
        p_rxMessage->Format = CAN_FRAME_FORMAT_STANDARD;
        p_rxMessage->Type = ((idr1 >> 4) & 1U) ? CAN_FRAME_TYPE_REMOTE : CAN_FRAME_TYPE_DATA;
        p_rxMessage->Id.Id = _HAL_CanBus_DecodeStdId(p_hal->RSIDR0, idr1);
    }

    p_rxMessage->DataLength = p_hal->RDLR & 0x0FU;
    for (uint8_t i = 0U; i < p_rxMessage->DataLength; i++)
    {
        p_rxMessage->Data[i] = p_hal->REDSR[i];
    }

    p_rxMessage->TimeStamp = ((uint16_t)p_hal->RTSRH << 8) | p_hal->RTSRL;

    /* Clear RXF flag to release the buffer */
    p_hal->CANRFLG = MSCAN_CANRFLG_RXF_MASK;
}

/*
    MSCAN has no per-buffer indexing like FlexCAN.
    userId maps 1:1 since there are only 3 Tx buffers (0-2) and Rx is implicit.
*/
static inline uint8_t HAL_CanBus_CalcMessageBufferIndex(HAL_CanBus_T * p_hal, uint8_t userId)
{
    (void)p_hal;
    return userId;
}

/*
    MSCAN interrupt flags:
    Rx: CANRFLG.RXF (bit 0)  — single Rx interrupt
    Tx: CANTFLG.TXE (bits 2:0) — per-buffer empty flags
    hwBufferIndex 0 = Rx, 1-3 = Tx buffers 0-2
    Convention: bufferIndex 0 => Rx, 1..3 => Tx buffer (bufferIndex - 1)
*/
static inline bool HAL_CanBus_ReadIsBufferInterrupt(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
{
    if (hwBufferIndex == 0U) { return (p_hal->CANRFLG & MSCAN_CANRFLG_RXF_MASK) && (p_hal->CANRIER & MSCAN_CANRIER_RXFIE_MASK); }
    else                     { return (p_hal->CANTFLG & (1U << (hwBufferIndex - 1U))) && (p_hal->CANTIER & (1U << (hwBufferIndex - 1U))); }
}

/*
    Write-1-to-clear semantics. For Rx: write RXF. For Tx: write the specific TXE bit.
    BSET must not be used — write the exact mask value.
*/
static inline void HAL_CanBus_ClearBufferInterrupt(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
{
    if (hwBufferIndex == 0U) { p_hal->CANRFLG = MSCAN_CANRFLG_RXF_MASK; }
    else                     { p_hal->CANTFLG = (1U << (hwBufferIndex - 1U)); }
}

static inline void HAL_CanBus_EnableBufferInterrupt(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
{
    if (hwBufferIndex == 0U) { p_hal->CANRIER |= MSCAN_CANRIER_RXFIE_MASK; }
    else                     { p_hal->CANTIER |= (1U << (hwBufferIndex - 1U)); }
}

static inline void HAL_CanBus_DisableBufferInterrupt(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
{
    if (hwBufferIndex == 0U) { p_hal->CANRIER &= ~MSCAN_CANRIER_RXFIE_MASK; }
    else                     { p_hal->CANTIER &= ~(1U << (hwBufferIndex - 1U)); }
}

/*
    MSCAN Tx: buffer is "empty" (ready for new frame) when CANTFLG.TXE bit is set.
    Rx: buffer has a frame when CANRFLG.RXF is set — "empty" is when RXF is clear.
    This matches FlexCAN semantics where TxRemote/RxEmpty means "ready to accept new work".
*/
static inline bool HAL_CanBus_ReadIsBufferTxRemoteRxEmpty(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
{
    if (hwBufferIndex == 0U) { return !(p_hal->CANRFLG & MSCAN_CANRFLG_RXF_MASK); } /* Rx: empty when no frame pending */
    else                     { return (p_hal->CANTFLG & (1U << (hwBufferIndex - 1U))) != 0U; } /* Tx: empty when TXE bit set */
}

/*
    MSCAN Rx locking: reading the foreground Rx registers locks them until RXF is cleared.
    The read in ReadRxMessageBuffer already performs the lock. These are no-ops/trivial.
*/
static inline bool HAL_CanBus_LockRxBuffer(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
{
    (void)p_hal; (void)hwBufferIndex;
    /* Rx foreground registers are automatically locked on first read until RXF is cleared */
    return true;
}

static inline void HAL_CanBus_UnlockRxBuffer(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
{
    (void)hwBufferIndex;
    /* Clear RXF to release the foreground buffer and allow next frame to load */
    p_hal->CANRFLG = MSCAN_CANRFLG_RXF_MASK;
}

/******************************************************************************/
/*! Global interrupt enable/disable via NVIC */
/******************************************************************************/
#define HAL_CAN_BUS_MSCAN_RX_IRQ_NUMBER    30U  /* MSCAN_1_IRQn */
#define HAL_CAN_BUS_MSCAN_TX_IRQ_NUMBER    31U  /* MSCAN_2_IRQn */

static inline void HAL_CanBus_DisableInterrupts(HAL_CanBus_T * p_hal)
{
    (void)p_hal;
    NVIC_DisableIRQ((IRQn_Type)HAL_CAN_BUS_MSCAN_RX_IRQ_NUMBER);
    NVIC_DisableIRQ((IRQn_Type)HAL_CAN_BUS_MSCAN_TX_IRQ_NUMBER);
}

static inline void HAL_CanBus_EnableInterrupts(HAL_CanBus_T * p_hal)
{
    (void)p_hal;
    NVIC_EnableIRQ((IRQn_Type)HAL_CAN_BUS_MSCAN_RX_IRQ_NUMBER);
    NVIC_EnableIRQ((IRQn_Type)HAL_CAN_BUS_MSCAN_TX_IRQ_NUMBER);
}

static inline void HAL_CanBus_ConfigBitRate(HAL_CanBus_T * p_hal, uint32_t bitRate)
{
    (void)p_hal; (void)bitRate;
}

static inline void HAL_CanBus_Init(HAL_CanBus_T * p_hal, uint32_t busFreq)
{
    (void)p_hal; (void)busFreq;
}
