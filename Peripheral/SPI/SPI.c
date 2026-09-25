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

#include "System/Critical/Critical.h"

/******************************************************************************/
/*!
    Transfer lifecycle
*/
/******************************************************************************/
/* The baud rate search is expensive. Reapply only on target change. */
static void ApplyTarget(const SPI_T * p_spi, const SPI_Target_T * p_target)
{
    if (p_spi->P_STATE->p_ActiveTarget == p_target) { return; }

    HAL_SPI_SetClockMode(p_spi->P_HAL, p_target->CLOCK_MODE);
    HAL_SPI_SetLsbFirst(p_spi->P_HAL, p_target->IS_LSB_FIRST);
    HAL_SPI_InitBaudRate(p_spi->P_HAL, p_target->CLOCK_FREQ);
    p_spi->P_STATE->p_ActiveTarget = p_target;
}

/* An aborted transfer can leave a byte in D. Its SPRF would be captured as index 0 of the next one. */
static void DrainRx(const SPI_T * p_spi)
{
    if (_HAL_SPI_ReadIsRxRegFull(p_spi->P_HAL) == true) { (void)HAL_SPI_ReadRxChar(p_spi->P_HAL); }
}

/* Active must be loaded. CS is held for the whole transfer, which is why the transfer is the unit. */
static void StartTransfer(const SPI_T * p_spi)
{
    SPI_State_T * p_state = p_spi->P_STATE;

    ApplyTarget(p_spi, p_state->Active.p_Target);
    DrainRx(p_spi);
    p_state->TxIndex = 0U;
    p_state->RxIndex = 0U;
    p_state->Status = SPI_STATUS_BUSY;
    Pin_Output_On(&p_state->Active.p_Target->CS_PIN);
}

/* SPTEF is already set, so arming Tx feeds byte 0. No first byte special case. */
static void StartTransfer_ISR(const SPI_T * p_spi)
{
    StartTransfer(p_spi);
    HAL_SPI_EnableRxInterrupt(p_spi->P_HAL);
    HAL_SPI_EnableTxInterrupt(p_spi->P_HAL);
}

/*
    Status is left terminal, not cleared to idle, so a caller reads its own result.
    OnComplete runs after the bus is released, so it may submit the next transfer.
*/
void _SPI_EndTransfer(const SPI_T * p_spi, SPI_Status_T status)
{
    SPI_State_T * p_state = p_spi->P_STATE;

    HAL_SPI_DisableTxInterrupt(p_spi->P_HAL);
    HAL_SPI_DisableRxInterrupt(p_spi->P_HAL);
    Pin_Output_Off(&p_state->Active.p_Target->CS_PIN);
    if (status == SPI_STATUS_ERROR_MODE_FAULT) { HAL_SPI_ClearRxErrors(p_spi->P_HAL); }
    p_state->Status = status;

    if (p_state->Active.OnComplete != NULL) { p_state->Active.OnComplete(p_state->Active.p_Context); }
}

/******************************************************************************/
/*!
    Public
*/
/******************************************************************************/
void SPI_Init(const SPI_T * p_spi)
{
    HAL_SPI_Init(p_spi->P_HAL);
    HAL_SPI_SetMaster(p_spi->P_HAL, true);
    HAL_SPI_WriteSwitch(p_spi->P_HAL, true);
    p_spi->P_STATE->p_ActiveTarget = NULL;
    p_spi->P_STATE->Status = SPI_STATUS_IDLE;
}

void SPI_Deinit(const SPI_T * p_spi)
{
    HAL_SPI_Deinit(p_spi->P_HAL);
}

/* Critical against a submit from OnComplete, which runs inside the transfer ISR */
SPI_Status_T SPI_Submit(const SPI_T * p_spi, const SPI_Transfer_T * p_transfer)
{
    SPI_Status_T status;
    critical_state_t critical;

    _Critical_Enter(&critical);
    if (SPI_IsBusy(p_spi) == true) { status = SPI_STATUS_ERROR_BUSY; }
    else
    {
        p_spi->P_STATE->Active = *p_transfer;
        StartTransfer_ISR(p_spi);
        status = SPI_STATUS_BUSY;
    }
    _Critical_Exit(critical);

    return status;
}

/* Drives the same step with interrupts unarmed */
SPI_Status_T SPI_Transfer_Blocking(const SPI_T * p_spi, const SPI_Transfer_T * p_transfer)
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

SPI_Status_T SPI_Exchange_Blocking(const SPI_T * p_spi, const SPI_Target_T * p_target, const void * p_txData, void * p_rxData, size_t size)
{
    return SPI_Transfer_Blocking(p_spi, &(SPI_Transfer_T)
    {
        .p_Target = p_target,
        .p_TxData = p_txData,
        .p_RxData = p_rxData,
        .Size = size,
    });
}
