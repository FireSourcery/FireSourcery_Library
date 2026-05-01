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
    @brief  MSCAN HAL for KE0x (MKE06Z) - register-level access
            MSCAN has 3 Tx buffers (shared foreground register set) and 1 Rx buffer (FIFO-like).
            Unlike FlexCAN, there are no individual message buffer indices for Rx.
            Tx buffer selection is via CANTBSEL; Rx is always from the foreground buffer.
*/
/******************************************************************************/
#include "KE0x.h"
// #include "../../../CanBus/CanBus.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>


#ifndef HAL_CAN_CLOCK_SOURCE_FREQ
#define HAL_CAN_CLOCK_SOURCE_FREQ CPU_FREQ
#endif

#ifndef HAL_CAN_INIT_BAUD_RATE
#define HAL_CAN_INIT_BAUD_RATE 250000U
#endif

typedef MSCAN_Type HAL_CAN_T;


/******************************************************************************/
/*!
    MSCAN IDR encode/decode
    IDR0: [ID28:ID21] (ext) or [ID10:ID3] (std)
    IDR1: [ID20:ID18 | SRR=1 | IDE=1 | ID17:ID15] (ext) or [ID2:ID0 | RTR | IDE=0 | 000] (std)
    IDR2: [ID14:ID7] (ext only)
    IDR3: [ID6:ID0 << 1 | RTR] (ext only)
*/
/******************************************************************************/
/*! @brief MSCAN IDR1 struct. */
typedef struct
{
    uint8_t EID17_15 : 3;           /*!< Extended Format Identifier 17-15*/
    uint8_t R_TEIDE : 1;            /*!< ID Extended */
    uint8_t R_TSRR : 1;             /*!< Substitute Remote Request */
    uint8_t EID20_18_OR_SID2_0 : 3; /*!< Extended Format Identifier 18-20 or standard format bit 0-2*/
} MSCAN_IDR1Type;

/*! @brief MSCAN IDR3 struct. */
typedef struct
{
    uint8_t ERTR : 1;   /*!< Remote Transmission Request*/
    uint8_t EID6_0 : 7; /*!< Extended Format Identifier 6-0*/
} MSCAN_IDR3Type;

/*! @brief MSCAN idr1 and idr3 union. */
typedef union
{
    MSCAN_IDR1Type IDR1; /*!< structure for identifier 1 */
    MSCAN_IDR3Type IDR3; /*!< structure for identifier 3 */
    uint8_t Bytes;       /*!< bytes */
} IDR1_3_UNION;

/*! @brief MSCAN extend ID struct. */
typedef struct
{
    uint32_t EID6_0 : 7;   /*!< ID[0:6] */
    uint32_t EID14_7 : 8;  /*!< ID[14:7] */
    uint32_t EID17_15 : 3; /*!< ID[17:15] */
    uint32_t EID20_18 : 3; /*!< ID[20:18] */
    uint32_t EID28_21 : 8; /*!< ID[28:21] */
    uint32_t rsvd : 3;
} MSCAN_ExtendIDType;

/*! @brief MSCAN standard ID struct. */
typedef struct
{
    uint32_t EID2_0 : 3;  /*!< ID[0:2] */
    uint32_t EID10_3 : 8; /*!< ID[10:3] */
    uint32_t rsvd : 21;
} MSCAN_StandardIDType;


static inline void _HAL_CAN_EncodeExtId(uint32_t id29, bool rtr, volatile uint8_t * p_idr0, volatile uint8_t * p_idr1, volatile uint8_t * p_idr2, volatile uint8_t * p_idr3)
{
    union { uint32_t Id; MSCAN_ExtendIDType Fields; }  id = { .Id = id29 };
    IDR1_3_UNION idr1 = { .IDR1 = {.EID17_15 = id.Fields.EID17_15, .R_TEIDE = 1U, .R_TSRR = 1U, .EID20_18_OR_SID2_0 = id.Fields.EID20_18 } };
    IDR1_3_UNION idr3 = { .IDR3 = {.ERTR = rtr, .EID6_0 = id.Fields.EID6_0 } };
    *p_idr0 = id.Fields.EID28_21;
    *p_idr1 = idr1.Bytes;
    *p_idr2 = id.Fields.EID14_7;
    *p_idr3 = idr3.Bytes;
}

static inline void _HAL_CAN_EncodeStdId(uint32_t id11, bool rtr, volatile uint8_t * p_idr0, volatile uint8_t * p_idr1)
{
    union { uint32_t Id; MSCAN_StandardIDType Fields; }  id = { .Id = id11 };
    IDR1_3_UNION idr1 = { .IDR1 = {.EID17_15 = 0U, .R_TEIDE = 0U, .R_TSRR = rtr, .EID20_18_OR_SID2_0 = id.Fields.EID2_0 } };
    *p_idr0 = id.Fields.EID10_3;
    *p_idr1 = idr1.Bytes;
}

static inline uint32_t _HAL_CAN_DecodeExtId(uint8_t idr0, uint8_t idr1, uint8_t idr2, uint8_t idr3)
{
    IDR1_3_UNION u1 = { .Bytes = idr1 };
    IDR1_3_UNION u3 = { .Bytes = idr3 };
    union { uint32_t Id; MSCAN_ExtendIDType Fields; } id = { .Fields = {
        .EID28_21 = idr0,
        .EID20_18 = u1.IDR1.EID20_18_OR_SID2_0,
        .EID17_15 = u1.IDR1.EID17_15,
        .EID14_7 = idr2,
        .EID6_0 = u3.IDR3.EID6_0,
    } };

    return id.Id;
}

static inline uint32_t _HAL_CAN_DecodeStdId(uint8_t idr0, uint8_t idr1)
{
    IDR1_3_UNION u1 = { .Bytes = idr1 };
    union { uint32_t Id; MSCAN_StandardIDType Fields; }  id = { .Fields = {.EID2_0 = u1.IDR1.EID20_18_OR_SID2_0, .EID10_3 = idr0 } };
    return id.Id;
}

/******************************************************************************/
/*!
    Tx/Rx Message
    MSCAN Tx: 3 buffers accessed through shared foreground registers.
        Select via CANTBSEL, load IDR/DSR/DLR/BPR, launch via CANTFLG.
    MSCAN Rx: Single foreground buffer (FIFO output). No hwIndex needed.
*/
/******************************************************************************/
/*
    Tx write sequence (caller order — driven by upper CanBus/HAL_CAN.h wrapper):
        WriteTxExtendedId | WriteTxStandardId   — selects buffer, encodes ID with RTR=0
        WriteTxRemote (optional)                 — sets RTR on the already-selected buffer
        WriteTxData                              — writes payload, DLC, launches
    CANTBSEL remains latched between calls, so the foreground register bank stays valid.
*/
static inline void HAL_CAN_WriteTxExtendedId(HAL_CAN_T * p_hal, uint32_t id)
{
    p_hal->CANTBSEL = p_hal->CANTFLG & MSCAN_CANTFLG_TXE_MASK;
    _HAL_CAN_EncodeExtId(id, false, &p_hal->TEIDR0, &p_hal->TEIDR1, &p_hal->TEIDR2, &p_hal->TEIDR3);
}

static inline void HAL_CAN_WriteTxStandardId(HAL_CAN_T * p_hal, uint32_t id)
{
    p_hal->CANTBSEL = p_hal->CANTFLG & MSCAN_CANTFLG_TXE_MASK;
    _HAL_CAN_EncodeStdId(id, false, &p_hal->TSIDR0, &p_hal->TSIDR1);
}

static inline void HAL_CAN_WriteTxRemote(HAL_CAN_T * p_hal, bool isRemote)
{
    IDR1_3_UNION idr1 = { .Bytes = p_hal->TEIDR1 };
    if (idr1.IDR1.R_TEIDE)
    {
        IDR1_3_UNION idr3 = { .Bytes = p_hal->TEIDR3 };
        idr3.IDR3.ERTR = isRemote;
        p_hal->TEIDR3 = idr3.Bytes;
    }
    else
    {
        idr1.IDR1.R_TSRR = isRemote;
        p_hal->TSIDR1 = idr1.Bytes;
    }
}

static inline void HAL_CAN_WriteTxData(HAL_CAN_T * p_hal, const uint8_t * p_data, uint8_t length)
{
    for (uint8_t i = 0U; i < length; i++) { p_hal->TEDSR[i] = p_data[i]; }
    p_hal->TDLR = length;
    p_hal->TBPR = 0U;
    /* Launch — clear the selected buffer's CANTFLG bit to schedule transmission */
    p_hal->CANTFLG = (p_hal->CANTBSEL & MSCAN_CANTBSEL_TX_MASK) & MSCAN_CANTFLG_TXE_MASK;
}

static inline bool HAL_CAN_ReadRxExtendedFlag(HAL_CAN_T * p_hal)
{
    IDR1_3_UNION idr1 = { .Bytes = p_hal->REIDR1 };
    return idr1.IDR1.R_TEIDE;
}

static inline uint32_t HAL_CAN_ReadRxStandardId(HAL_CAN_T * p_hal)
{
    return _HAL_CAN_DecodeStdId(p_hal->RSIDR0, p_hal->RSIDR1);
}

static inline uint32_t HAL_CAN_ReadRxExtendedId(HAL_CAN_T * p_hal)
{
    return _HAL_CAN_DecodeExtId(p_hal->REIDR0, p_hal->REIDR1, p_hal->REIDR2, p_hal->REIDR3);
}

static inline uint8_t HAL_CAN_ReadRxLength(HAL_CAN_T * p_hal) { return p_hal->RDLR & 0x0FU; }

static inline bool HAL_CAN_ReadRxRemoteFlag(HAL_CAN_T * p_hal)
{
    IDR1_3_UNION idr1 = { .Bytes = p_hal->REIDR1 };
    IDR1_3_UNION idr3 = { .Bytes = p_hal->REIDR3 };
    return (idr1.IDR1.R_TEIDE) ? idr3.IDR3.ERTR : idr1.IDR1.R_TSRR;
}

static inline uint32_t HAL_CAN_ReadRxTimeStamp(HAL_CAN_T * p_hal)
{
    return (p_hal->RTSRH << 8) | p_hal->RTSRL;
}

static inline uint8_t HAL_CAN_ReadRxData(HAL_CAN_T * p_hal, uint8_t * p_data)
{
    uint8_t length = p_hal->RDLR & 0x0FU;
    for (uint8_t i = 0U; i < length; i++) { p_data[i] = p_hal->REDSR[i]; }
    p_hal->CANRFLG = MSCAN_CANRFLG_RXF_MASK;
    return length;
}

/******************************************************************************/
/*!
    Interrupts
    Rx interrupt: CANRFLG.RXF flag + CANRIER.RXFIE enable
    Tx interrupt: CANTFLG.TXE flags + CANTIER.TXEIE enables (per-buffer via bit mask)

    NOTE: CANRFLG/CANTFLG are w1c — write the exact bit, do not use read-modify-write (BSET).
*/
/******************************************************************************/
/******************************************************************************/
/*! Convenience flag accessors (MSCAN-specific, flat — no hwIndex) */
/******************************************************************************/
static inline bool HAL_CAN_ReadTxEmptyFlag(HAL_CAN_T * p_hal) { return (p_hal->CANTFLG & MSCAN_CANTFLG_TXE_MASK) != 0U; }
static inline void HAL_CAN_ClearTxEmptyFlag(HAL_CAN_T * p_hal) { p_hal->CANTFLG |= MSCAN_CANTFLG_TXE_MASK; }
static inline void HAL_CAN_EnableTxEmptyInterrupt(HAL_CAN_T * p_hal) { p_hal->CANTIER |= MSCAN_CANTIER_TXEIE_MASK; }
static inline void HAL_CAN_DisableTxEmptyInterrupt(HAL_CAN_T * p_hal) { p_hal->CANTIER &= ~MSCAN_CANTIER_TXEIE_MASK; }

static inline bool HAL_CAN_ReadRxFullFlag(HAL_CAN_T * p_hal) { return (p_hal->CANRFLG & MSCAN_CANRFLG_RXF_MASK) != 0U; }
static inline void HAL_CAN_ClearRxFullFlag(HAL_CAN_T * p_hal) { p_hal->CANRFLG |= MSCAN_CANRFLG_RXF_MASK; }
static inline void HAL_CAN_EnableRxFullInterrupt(HAL_CAN_T * p_hal) { p_hal->CANRIER |= MSCAN_CANRIER_RXFIE_MASK; }
static inline void HAL_CAN_DisableRxFullInterrupt(HAL_CAN_T * p_hal) { p_hal->CANRIER &= ~MSCAN_CANRIER_RXFIE_MASK; }

/******************************************************************************/
/*!
    Buffer index mapping
    Rx always uses the single foreground buffer (no multiplexing)
    Rx is always index 0 (single foreground buffer).
    Tx has 3 buffers (0-2), each with its own interrupt enable bit
*/
/******************************************************************************/
static inline uint8_t HAL_CAN_MapMessageBufferIndex(HAL_CAN_T * p_hal, uint8_t userId) { (void)p_hal; return userId; }

/* MSCAN has only one Rx foreground buffer */
static inline uint8_t HAL_CAN_MapTxMessageBufferIndex(HAL_CAN_T * p_hal, uint8_t userId) { (void)p_hal; return userId; }
static inline uint8_t HAL_CAN_MapRxMessageBufferIndex(HAL_CAN_T * p_hal, uint8_t userId) { (void)p_hal; (void)userId; return 0U; }

/******************************************************************************/
/*!
    Completion / ready flags
    Tx complete: CANTFLG.TXE bit set for the buffer (empty = done transmitting).
    Rx complete: CANRFLG.RXF set (full = frame available).
*/
/******************************************************************************/
// static inline bool HAL_CAN_ReadTxComplete(HAL_CAN_T * p_hal, uint8_t hwIndex) { return (p_hal->CANTFLG & (1U << hwIndex)) != 0U; }
// /* Tx buffer empty = ready */
// static inline bool HAL_CAN_ReadTxRemoteRxEmpty(HAL_CAN_T * p_hal, uint8_t hwIndex) { return (p_hal->CANTFLG & (1U << hwIndex)) != 0U; }
// /* Rx buffer full = has data */
// static inline bool HAL_CAN_ReadTxRemoteRxFull(HAL_CAN_T * p_hal, uint8_t hwIndex) { (void)hwIndex; return (p_hal->CANRFLG & MSCAN_CANRFLG_RXF_MASK) != 0U; }

// static inline bool HAL_CAN_ReadRxComplete(HAL_CAN_T * p_hal, uint8_t hwIndex) { (void)hwIndex; return (p_hal->CANRFLG & MSCAN_CANRFLG_RXF_MASK) != 0U; }
// static inline bool _HAL_CAN_ReadRxInterruptEnable(HAL_CAN_T * p_hal, uint8_t hwIndex) { (void)hwIndex; return ((p_hal->CANRIER & MSCAN_CANRIER_RXFIE_MASK) != 0U); }
// // static inline bool HAL_CAN_ReadRxComplete(HAL_CAN_T * p_hal, uint8_t hwIndex) { return _HAL_CAN_ReadRxComplete(p_hal, hwIndex) && _HAL_CAN_ReadRxInterruptEnable(p_hal, hwIndex); }

// static inline void HAL_CAN_ClearTxRxInterrupt(HAL_CAN_T * p_hal, uint8_t hwIndex) {}
// static inline void HAL_CAN_EnableTxRxInterrupt(HAL_CAN_T * p_hal, uint8_t hwIndex) {}
// static inline void HAL_CAN_DisableTxRxInterrupt(HAL_CAN_T * p_hal, uint8_t hwIndex) {}

// static inline void HAL_CAN_ClearTxInterrupt(HAL_CAN_T * p_hal, uint8_t hwIndex) { p_hal->CANTFLG = (1U << hwIndex); }
// static inline void HAL_CAN_EnableTxInterrupt(HAL_CAN_T * p_hal, uint8_t hwIndex) { p_hal->CANTIER |= (1U << hwIndex); }
// static inline void HAL_CAN_DisableTxInterrupt(HAL_CAN_T * p_hal, uint8_t hwIndex) { p_hal->CANTIER &= ~(1U << hwIndex); }

// static inline void HAL_CAN_ClearRxInterrupt(HAL_CAN_T * p_hal, uint8_t hwIndex) { (void)hwIndex; p_hal->CANRFLG = MSCAN_CANRFLG_RXF_MASK; }
// static inline void HAL_CAN_EnableRxInterrupt(HAL_CAN_T * p_hal, uint8_t hwIndex) { (void)hwIndex; p_hal->CANRIER |= MSCAN_CANRIER_RXFIE_MASK; }
// static inline void HAL_CAN_DisableRxInterrupt(HAL_CAN_T * p_hal, uint8_t hwIndex) { (void)hwIndex; p_hal->CANRIER &= ~MSCAN_CANRIER_RXFIE_MASK; }

/******************************************************************************/
/*! Rx buffer lock/unlock
    MSCAN Rx foreground registers are implicitly locked once read.
    Clearing RXF releases the buffer and loads the next queued frame (if any).
*/
/******************************************************************************/
/* Foreground buffer auto-locks on first register read */
static inline bool HAL_CAN_LockRx(HAL_CAN_T * p_hal, uint8_t hwIndex) { (void)p_hal; (void)hwIndex; return true; }
/* Cleared on Read */
static inline void HAL_CAN_UnlockRx(HAL_CAN_T * p_hal, uint8_t hwIndex) { (void)hwIndex; }


/******************************************************************************/
/*! Status
    MSCAN Tx status: CANTFLG bits [2:0] — set when buffer is empty (transmission complete).
    MSCAN Rx status: CANRFLG.RXF — set when foreground buffer holds a valid frame.
*/
/******************************************************************************/
// static inline uint32_t HAL_CAN_ReadTxStatus(HAL_CAN_T * p_hal, uint8_t hwIndex) { (void)hwIndex; return p_hal->CANTFLG & MSCAN_CANTFLG_TXE_MASK; }
// static inline uint32_t HAL_CAN_ReadRxStatus(HAL_CAN_T * p_hal, uint8_t hwIndex) { (void)hwIndex; return p_hal->CANRFLG & MSCAN_CANRFLG_RXF_MASK; }
// static inline HAL_CAN_DriverStatus_T HAL_CAN_MapTxStatus(HAL_CAN_T * p_hal, uint32_t status) {
// static inline HAL_CAN_DriverStatus_T HAL_CAN_MapRxStatus(HAL_CAN_T * p_hal, uint32_t status) {


/******************************************************************************/
/*!
    Init
    MSCAN bit time = sync(1) + TSEG1+1 + TSEG2+1 TQ.
*/
/******************************************************************************/
#ifndef HAL_CAN_TIME_QUANTA_PER_BIT
#define HAL_CAN_TIME_QUANTA_PER_BIT     8U
#endif
#ifndef HAL_CAN_TSEG1
#define HAL_CAN_TSEG1                   3U      /* TSEG1 = 3 → 4 TQ */
#endif
#ifndef HAL_CAN_TSEG2
#define HAL_CAN_TSEG2                   2U      /* TSEG2 = 2 → 3 TQ */
#endif
#ifndef HAL_CAN_SJW
#define HAL_CAN_SJW                     0U      /* SJW = 0 → 1 TQ */
#endif
#ifndef HAL_CAN_SAMP
#define HAL_CAN_SAMP                    0U      /* single sample */
#endif

#ifndef HAL_CAN_CLK_SRC_BUS
#define HAL_CAN_CLK_SRC_BUS             0U      /* 1 = bus clock, 0 = oscillator */
#endif


static inline void _HAL_CAN_EnterInitMode(HAL_CAN_T * p_hal)
{
    p_hal->CANCTL0 |= MSCAN_CANCTL0_INITRQ_MASK;
    while ((p_hal->CANCTL1 & MSCAN_CANCTL1_INITAK_MASK) == 0U) {}
}

static inline void _HAL_CAN_ExitInitMode(HAL_CAN_T * p_hal)
{
    p_hal->CANCTL0 &= (uint8_t)~MSCAN_CANCTL0_INITRQ_MASK;
    while ((p_hal->CANCTL1 & MSCAN_CANCTL1_INITAK_MASK) != 0U) {}
}

static inline void _HAL_CAN_Enable(HAL_CAN_T * p_hal, bool enable)
{
    if (enable) { p_hal->CANCTL1 |= MSCAN_CANCTL1_CANE_MASK; }
    else { p_hal->CANCTL1 &= (uint8_t)~MSCAN_CANCTL1_CANE_MASK; }
}

static inline void HAL_CAN_InitBaudRate(HAL_CAN_T * p_hal, uint32_t baudRate)
{
    #define HAL_CAN_BRP_MAX                 0x3FU

    uint32_t tqClk = baudRate * HAL_CAN_TIME_QUANTA_PER_BIT;
    uint32_t brp = (tqClk != 0U && tqClk <= HAL_CAN_CLOCK_SOURCE_FREQ) ? (HAL_CAN_CLOCK_SOURCE_FREQ / tqClk) - 1U : 0U;
    if (brp > HAL_CAN_BRP_MAX) { brp = HAL_CAN_BRP_MAX; }

    _HAL_CAN_EnterInitMode(p_hal);

    p_hal->CANBTR0 = (uint8_t)(MSCAN_CANBTR0_BRP(brp) | MSCAN_CANBTR0_SJW(HAL_CAN_SJW));
    p_hal->CANBTR1 = (uint8_t)(MSCAN_CANBTR1_TSEG1(HAL_CAN_TSEG1) | MSCAN_CANBTR1_TSEG2(HAL_CAN_TSEG2) | MSCAN_CANBTR1_SAMP(HAL_CAN_SAMP));

    _HAL_CAN_ExitInitMode(p_hal);
}

static inline void HAL_CAN_Init(HAL_CAN_T * p_hal)
{
    /* Enable MSCAN module */
    _HAL_CAN_Enable(p_hal, true);

    /* Enter init mode to configure CTL/BTR/filter */
    _HAL_CAN_EnterInitMode(p_hal);

    /* CTL0: clear wake-up, time enable left off */
    p_hal->CANCTL0 = (uint8_t)(p_hal->CANCTL0 & ~(MSCAN_CANCTL0_WUPE_MASK | MSCAN_CANCTL0_TIME_MASK));

    /* CTL1: select clock source, disable loopback/listen */
    uint8_t ctl1 = p_hal->CANCTL1 & (uint8_t)~(MSCAN_CANCTL1_LOOPB_MASK | MSCAN_CANCTL1_LISTEN_MASK | MSCAN_CANCTL1_CLKSRC_MASK);
    if (HAL_CAN_CLK_SRC_BUS != 0U) { ctl1 |= MSCAN_CANCTL1_CLKSRC_MASK; }
    p_hal->CANCTL1 = ctl1;

    /* Acceptance filters: open (accept all) — 32-bit filter mode with mask = all-don't-care */
    p_hal->CANIDAC = (uint8_t)MSCAN_CANIDAC_IDAM(0U); /* 32-bit filter mode */
    for (uint8_t i = 0U; i < 4U; i++)
    {
        p_hal->CANIDAR_BANK_1[i] = 0x00U;
        p_hal->CANIDMR_BANK_1[i] = 0xFFU; /* all bits don't care */
        p_hal->CANIDAR_BANK_2[i] = 0x00U;
        p_hal->CANIDMR_BANK_2[i] = 0xFFU;
    }

    /* Baud rate — function handles its own init mode transition */
    _HAL_CAN_ExitInitMode(p_hal);
    HAL_CAN_InitBaudRate(p_hal, HAL_CAN_INIT_BAUD_RATE);
}

/******************************************************************************/
/*! Global NVIC interrupt enable/disable */
/******************************************************************************/
// #define HAL_CAN_MSCAN_RX_IRQ_NUMBER    30U  /* MSCAN_1_IRQn: Rx */
// #define HAL_CAN_MSCAN_TX_IRQ_NUMBER    31U  /* MSCAN_2_IRQn: Tx, Err, Wake-up */

// static inline void HAL_CAN_EnableInterrupts(HAL_CAN_T * p_hal)
// {
//     (void)p_hal;
//     EnableIRQ((IRQn_Type)HAL_CAN_MSCAN_RX_IRQ_NUMBER);
//     EnableIRQ((IRQn_Type)HAL_CAN_MSCAN_TX_IRQ_NUMBER);
// }

// static inline void HAL_CAN_DisableInterrupts(HAL_CAN_T * p_hal)
// {
//     (void)p_hal;
//     DisableIRQ((IRQn_Type)HAL_CAN_MSCAN_RX_IRQ_NUMBER);
//     DisableIRQ((IRQn_Type)HAL_CAN_MSCAN_TX_IRQ_NUMBER);
// }

// static inline void _HAL_CAN_EncodeExtId(uint32_t id29, bool rtr, volatile uint8_t * p_idr0, volatile uint8_t * p_idr1, volatile uint8_t * p_idr2, volatile uint8_t * p_idr3)
// {
//     *p_idr0 = (uint8_t)(id29 >> 21);
//     *p_idr1 = (uint8_t)(((id29 >> 18) & 0x07U) << 5) | (1U << 4) | (1U << 3) | (uint8_t)((id29 >> 15) & 0x07U);
//     *p_idr2 = (uint8_t)(id29 >> 7);
//     *p_idr3 = (uint8_t)((id29 & 0x7FU) << 1) | (rtr ? 1U : 0U);
// }

// static inline void _HAL_CAN_EncodeStdId(uint32_t id11, bool rtr, volatile uint8_t * p_idr0, volatile uint8_t * p_idr1)
// {
//     *p_idr0 = (uint8_t)(id11 >> 3);
//     *p_idr1 = (uint8_t)((id11 & 0x07U) << 5) | (rtr ? (1U << 4) : 0U);
// }

// static inline uint32_t _HAL_CAN_DecodeExtId(uint8_t idr0, uint8_t idr1, uint8_t idr2, uint8_t idr3)
// {
//     return ((uint32_t)idr0 << 21) | (((uint32_t)(idr1 >> 5) & 0x07U) << 18) | (((uint32_t)idr1 & 0x07U) << 15) | ((uint32_t)idr2 << 7) | ((uint32_t)(idr3 >> 1) & 0x7FU);
// }

// static inline uint32_t _HAL_CAN_DecodeStdId(uint8_t idr0, uint8_t idr1)
// {
//     return ((uint32_t)idr0 << 3) | ((uint32_t)(idr1 >> 5) & 0x07U);
// }
// static inline void HAL_CAN_WriteTxMessage(HAL_CAN_T * p_hal, const CAN_Frame_T * p_txMessage)
// {
//     bool rtr = (p_txMessage->CanId & CAN_ID_FLAG_RTR) != 0U;
//     bool ext = (p_txMessage->CanId & CAN_ID_FLAG_EXT) != 0U;
//     uint32_t id = p_txMessage->Id29;

//     /* 1. Select an empty Tx buffer */
//     /*  (p_hal->CANTFLG & MSCAN_CANTFLG_TXE_MASK) == 0, return error */
//     p_hal->CANTBSEL = p_hal->CANTFLG & MSCAN_CANTFLG_TXE_MASK;

//     /* 2. Write ID registers */
//     if (ext)
//         _HAL_CAN_EncodeExtId(id, rtr, &p_hal->TEIDR0, &p_hal->TEIDR1, &p_hal->TEIDR2, &p_hal->TEIDR3);
//     else
//         _HAL_CAN_EncodeStdId(id, rtr, &p_hal->TSIDR0, &p_hal->TSIDR1);

//     /* 3. Write data */
//     for (uint8_t i = 0U; i < p_txMessage->Dlc; i++) { p_hal->TEDSR[i] = p_txMessage->Data[i]; }

//     /* 4. Write DLC and priority */
//     p_hal->TDLR = p_txMessage->Dlc;
//     p_hal->TBPR = 0U;

//     /* 5. Launch — clear the selected buffer's CANTFLG bit to schedule transmission */
//     p_hal->CANTFLG = (p_hal->CANTBSEL & MSCAN_CANTBSEL_TX_MASK) & MSCAN_CANTFLG_TXE_MASK;
// }

// static inline void HAL_CAN_ReadRxMessage(HAL_CAN_T * p_hal, CAN_Frame_T * p_rxMessage)
// {
//     uint8_t idr1 = p_hal->REIDR1;
//     bool isExtended = ((idr1 >> 3) & 1U);
//     uint32_t canId;

//     if (isExtended)
//     {
//         canId = _HAL_CAN_DecodeExtId(p_hal->REIDR0, idr1, p_hal->REIDR2, p_hal->REIDR3);
//         canId |= CAN_ID_FLAG_EXT;
//         if (p_hal->REIDR3 & 1U) { canId |= CAN_ID_FLAG_RTR; }
//     }
//     else
//     {
//         canId = _HAL_CAN_DecodeStdId(p_hal->RSIDR0, idr1);
//         if ((idr1 >> 4) & 1U) { canId |= CAN_ID_FLAG_RTR; }
//     }

//     p_rxMessage->CanId = canId;
//     p_rxMessage->Dlc = p_hal->RDLR & 0x0FU;
//     p_rxMessage->Opt = 0U;
//     for (uint8_t i = 0U; i < p_rxMessage->Dlc; i++) { p_rxMessage->Data[i] = p_hal->REDSR[i]; }
//     /* RXF cleared by UnlockRx / ClearRxInterrupt — do not release buffer here */
// }


