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
    @file   SPI.c
    @author FireSourcery
    @brief
*/
/******************************************************************************/
#include "SPI.h"

/******************************************************************************/
/*!
    Transfer lifecycle
*/
/******************************************************************************/
/* An aborted transfer can leave a byte in D. Its SPRF would be captured as index 0 of the next one. */
static void DrainRx(SPI_T * p_spi)
{
    if (_HAL_SPI_ReadIsRxRegFull(p_spi->P_HAL) == true) { (void)HAL_SPI_ReadRxChar(p_spi->P_HAL); }
}

/* Active must be loaded. CS is held for the whole transfer, which is why the transfer is the unit. */
static void StartTransfer(SPI_T * p_spi)
{
    SPI_State_T * p_state = p_spi->P_STATE;

    DrainRx(p_spi);
    p_state->TxIndex = 0U;
    p_state->RxIndex = 0U;
    p_state->Status = SPI_STATUS_BUSY;
    Pin_Output_On(&p_spi->CS_PIN);  /* Idempotent, so a held CS stays asserted across transfers */
}

/* SPTEF is already set, so arming Tx feeds byte 0. No first byte special case. */
static void StartTransfer_ISR(SPI_T * p_spi)
{
    StartTransfer(p_spi);
    HAL_SPI_EnableRxInterrupt(p_spi->P_HAL);
    HAL_SPI_EnableTxInterrupt(p_spi->P_HAL);
}

/*
    Status is left terminal, not cleared to idle, so a caller reads its own result.
    OnComplete runs after the bus is released, so it may submit the next transfer.
*/
void _SPI_EndTransfer(SPI_T * p_spi, SPI_Status_T status)
{
    SPI_State_T * p_state = p_spi->P_STATE;

    HAL_SPI_DisableTxInterrupt(p_spi->P_HAL);
    HAL_SPI_DisableRxInterrupt(p_spi->P_HAL);
    /* A failed transfer releases the bus whatever the caller asked: the transaction is over */
    if ((p_state->IsCsHold == false) || (status != SPI_STATUS_SUCCESS)) { Pin_Output_Off(&p_spi->CS_PIN); }
    if (status == SPI_STATUS_ERROR_MODE_FAULT) { HAL_SPI_ClearRxErrors(p_spi->P_HAL); }
    p_state->Status = status;

    if (p_state->Active.OnComplete != NULL) { p_state->Active.OnComplete(p_state->Active.p_Context); }
}

/******************************************************************************/
/*!
    Public
*/
/******************************************************************************/
/* One device, so the clock settles here rather than being reapplied per transfer */
void SPI_Init(SPI_T * p_spi)
{
    HAL_SPI_Init(p_spi->P_HAL);
    HAL_SPI_SetMaster(p_spi->P_HAL, true);
    HAL_SPI_SetClockMode(p_spi->P_HAL, p_spi->CLOCK_MODE);
    HAL_SPI_SetLsbFirst(p_spi->P_HAL, p_spi->IS_LSB_FIRST);
    HAL_SPI_InitBaudRate(p_spi->P_HAL, p_spi->CLOCK_FREQ);
    HAL_SPI_WriteSwitch(p_spi->P_HAL, true);
    Pin_Output_Off(&p_spi->CS_PIN);
    p_spi->P_STATE->IsCsHold = false;
    p_spi->P_STATE->Status = SPI_STATUS_IDLE;
}

void SPI_Deinit(SPI_T * p_spi)
{
    HAL_SPI_Deinit(p_spi->P_HAL);
}

/* Critical against a submit from OnComplete, which runs inside the transfer ISR */
SPI_Status_T SPI_Submit(SPI_T * p_spi, SPI_Transfer_T * p_transfer)
{
    SPI_Status_T status;
    if (SPI_IsBusy(p_spi) == true) { status = SPI_STATUS_ERROR_BUSY; }
    else
    {
        p_spi->P_STATE->Active = *p_transfer;
        StartTransfer_ISR(p_spi);
        status = SPI_STATUS_BUSY;
    }
    return status;
}

/* Drives the same step with interrupts unarmed */
SPI_Status_T SPI_Transfer_Blocking(SPI_T * p_spi, SPI_Transfer_T * p_transfer)
{
    uint32_t loops = 0U;

    if (SPI_IsBusy(p_spi) == true) { return SPI_STATUS_ERROR_BUSY; }

    p_spi->P_STATE->Active = *p_transfer;
    StartTransfer(p_spi);

    while (SPI_IsBusy(p_spi) == true)
    {
        SPI_ProcTransfer(p_spi);
        if (++loops > SPI_BLOCKING_TIMEOUT_LOOPS) { _SPI_EndTransfer(p_spi, SPI_STATUS_ERROR_TIMEOUT); }
    }

    return SPI_GetStatus(p_spi);
}

SPI_Status_T SPI_Exchange_Blocking(SPI_T * p_spi, const void * p_txData, void * p_rxData, size_t size)
{
    return SPI_Transfer_Blocking(p_spi, &(SPI_Transfer_T){ .p_TxData = p_txData, .p_RxData = p_rxData, .Size = size, });
}

/******************************************************************************/
/*!
    [Xcvr] shaped
*/
/******************************************************************************/
bool SPI_Tx(SPI_T * p_spi, const uint8_t * p_src, size_t length) { return (SPI_Exchange_Blocking(p_spi, p_src, NULL, length) == SPI_STATUS_SUCCESS); }
bool SPI_Rx(SPI_T * p_spi, uint8_t * p_dest, size_t length) { return (SPI_Exchange_Blocking(p_spi, NULL, p_dest, length) == SPI_STATUS_SUCCESS); }


