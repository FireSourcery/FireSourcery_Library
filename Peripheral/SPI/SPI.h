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
    @file   SPI.h
    @author FireSourcery
    @brief  SPI master. Transaction driver.

    One write clocks one exchange, so Rx exists only as the echo of Tx. The unit of
    work is the [SPI_Transfer_T], not the byte: state is an index pair over caller
    memory, RxIndex trailing TxIndex by the shifter depth.
*/
/******************************************************************************/
#include "HAL_SPI.h"

#include "Peripheral/Pin/Pin.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Clocked out when a transfer supplies no Tx data */
#ifndef SPI_FILL_CHAR
#define SPI_FILL_CHAR 0xFFU
#endif

/* Bound on the blocking spin. A wedged bus must not hold the caller forever. */
#ifndef SPI_BLOCKING_TIMEOUT_LOOPS
#define SPI_BLOCKING_TIMEOUT_LOOPS 100000U
#endif

typedef enum SPI_Status
{
    SPI_STATUS_IDLE,                /* No transfer has run */
    SPI_STATUS_BUSY,
    SPI_STATUS_SUCCESS,
    SPI_STATUS_ERROR_MODE_FAULT,    /* Another master drove SS */
    SPI_STATUS_ERROR_TIMEOUT,
    SPI_STATUS_ERROR_BUSY,          /* Bus held by another transfer */
}
SPI_Status_T;

/******************************************************************************/
/*!
    Target - the device behind one chip select.
    Applied to the bus only when the active target changes.
*/
/******************************************************************************/
typedef const struct SPI_Target
{
    Pin_T CS_PIN;           /* IS_INVERT selects active low */
    uint32_t CLOCK_FREQ;
    uint8_t CLOCK_MODE;     /* 0..3 -> {CPOL, CPHA} */
    bool IS_LSB_FIRST;
}
SPI_Target_T;

/******************************************************************************/
/*!
    Transfer - the unit of work. Caller owns the buffers until completion.

    p_TxData == NULL    clock out SPI_FILL_CHAR, capture the echo     (read)
    p_RxData == NULL    discard the echo                              (write)
    both set            full duplex, paired by index                  (exchange)
*/
/******************************************************************************/
typedef void (*SPI_Callback_T)(void * p_context);

typedef struct SPI_Transfer
{
    const SPI_Target_T * p_Target;
    const uint8_t * p_TxData;
    uint8_t * p_RxData;
    size_t Size;
    SPI_Callback_T OnComplete;
    void * p_Context;
}
SPI_Transfer_T;

/******************************************************************************/
/*!
    Instance
*/
/******************************************************************************/
typedef struct SPI_State
{
    SPI_Transfer_T Active;
    size_t TxIndex;                         /* Handed to the shifter */
    size_t RxIndex;                         /* Echoes captured. Trails TxIndex */
    const SPI_Target_T * p_ActiveTarget;    /* Bus configuration currently applied */
    SPI_Status_T Status;
}
SPI_State_T;

#define SPI_STATE_ALLOC() (&(SPI_State_T){ .Status = SPI_STATUS_IDLE })

typedef const struct SPI
{
    HAL_SPI_T * P_HAL;
    SPI_State_T * P_STATE;
}
SPI_T;

#define SPI_INIT(p_Hal) { .P_HAL = (p_Hal), .P_STATE = SPI_STATE_ALLOC(), }

/******************************************************************************/
/*!
    Transfer step
*/
/******************************************************************************/
extern void _SPI_EndTransfer(const SPI_T * p_spi, SPI_Status_T status);

static inline uint8_t _SPI_TxCharAt(const SPI_Transfer_T * p_transfer, size_t index)
{
    return (p_transfer->p_TxData != NULL) ? p_transfer->p_TxData[index] : SPI_FILL_CHAR;
}

/* The D read is what clears SPRF, so it happens whether or not the echo is kept */
static inline void _SPI_CaptureRx(const SPI_T * p_spi)
{
    SPI_State_T * p_state = p_spi->P_STATE;
    uint8_t rxChar = HAL_SPI_ReadRxChar(p_spi->P_HAL);

    if (p_state->Active.p_RxData != NULL) { p_state->Active.p_RxData[p_state->RxIndex] = rxChar; }
    p_state->RxIndex++;
}

static inline void _SPI_FeedTx(const SPI_T * p_spi)
{
    SPI_State_T * p_state = p_spi->P_STATE;

    HAL_SPI_WriteTxChar(p_spi->P_HAL, _SPI_TxCharAt(&p_state->Active, p_state->TxIndex));
    p_state->TxIndex++;
}

/*
    Tx drives the clock, Rx is its echo. Feeding stops at Size; the transfer ends one
    exchange later, when the last echo lands.

    Interrupt entry point. The blocking path polls the same step with interrupts unarmed.
*/
static inline void SPI_ProcTransfer(const SPI_T * p_spi)
{
    SPI_State_T * p_state = p_spi->P_STATE;

    /* Active is only meaningful while busy. Guards a shared or spurious vector from dereferencing a stale target */
    if (p_state->Status != SPI_STATUS_BUSY) { return; }

    if (HAL_SPI_ReadModeFault(p_spi->P_HAL) == true) { _SPI_EndTransfer(p_spi, SPI_STATUS_ERROR_MODE_FAULT); return; }

    if (_HAL_SPI_ReadIsRxRegFull(p_spi->P_HAL) == true) { _SPI_CaptureRx(p_spi); }

    if (p_state->TxIndex < p_state->Active.Size)
    {
        if (_HAL_SPI_ReadIsTxRegEmpty(p_spi->P_HAL) == true) { _SPI_FeedTx(p_spi); }
    }
    else
    {
        /* SPTEF stays set once feeding stops. Leaving it armed re-enters forever */
        HAL_SPI_DisableTxInterrupt(p_spi->P_HAL);
        if (p_state->RxIndex == p_state->Active.Size) { _SPI_EndTransfer(p_spi, SPI_STATUS_SUCCESS); }
    }
}

/******************************************************************************/
/*!
    Query
*/
/******************************************************************************/
static inline SPI_Status_T SPI_GetStatus(const SPI_T * p_spi)   { return p_spi->P_STATE->Status; }
static inline bool SPI_IsBusy(const SPI_T * p_spi)              { return (p_spi->P_STATE->Status == SPI_STATUS_BUSY); }
// static inline size_t SPI_GetQueuedCount(const SPI_T * p_spi)    { return RingT_GetFullCount(RING_T_ARGS(p_spi->PENDING)); }

/******************************************************************************/
/*!
    Extern
*/
/******************************************************************************/
extern void SPI_Init(const SPI_T * p_spi);
extern void SPI_Deinit(const SPI_T * p_spi);

/* Interrupt driven. Completion reported by OnComplete, which may submit the next transfer */
extern SPI_Status_T SPI_Submit(const SPI_T * p_spi, const SPI_Transfer_T * p_transfer);

/* Polled. Rejects while the interrupt driven path holds the bus */
extern SPI_Status_T SPI_Transfer_Blocking(const SPI_T * p_spi, const SPI_Transfer_T * p_transfer);
extern SPI_Status_T SPI_Exchange_Blocking(const SPI_T * p_spi, const SPI_Target_T * p_target, const void * p_txData, void * p_rxData, size_t size);
