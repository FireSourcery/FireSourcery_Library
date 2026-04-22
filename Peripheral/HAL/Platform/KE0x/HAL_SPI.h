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
    @file   HAL_SPI.h
    @author FireSourcery
    @brief  SPI HAL for KE0x (MKE06Z/MKE02Z) — register-level access
            8-bit full-duplex SPI. Single SPE enable (no separate Tx/Rx switches).
            Baud rate = Bus Clock / ((SPPR+1) * 2^(SPR+1)), range 2..4096.
*/
/******************************************************************************/
#ifndef HAL_SPI_PLATFORM_H
#define HAL_SPI_PLATFORM_H

#include "KE0x.h"

#include <stdint.h>
#include <stdbool.h>

#ifndef HAL_SPI_CLOCK_SOURCE_FREQ
#define HAL_SPI_CLOCK_SOURCE_FREQ (CPU_FREQ / 2U)
#endif

typedef SPI_Type HAL_SPI_T;

/******************************************************************************/
/*!
    Data — 8-bit transfers.
    SPTEF clears on: read S with SPTEF=1 → write D.
    SPRF  clears on: read S with SPRF=1  → read  D.
*/
/******************************************************************************/
static inline void HAL_SPI_WriteTxChar(HAL_SPI_T * p_hal, uint8_t txChar) { p_hal->D = txChar; }
static inline uint8_t HAL_SPI_ReadRxChar(const HAL_SPI_T * p_hal) { return (uint8_t)p_hal->D; }

/******************************************************************************/
/*! Status */
/******************************************************************************/
static inline bool _HAL_SPI_ReadIsTxRegEmpty(const HAL_SPI_T * p_hal) { return ((p_hal->S & SPI_S_SPTEF_MASK) != 0U); }
static inline bool _HAL_SPI_ReadIsRxRegFull(const HAL_SPI_T * p_hal) { return ((p_hal->S & SPI_S_SPRF_MASK) != 0U); }
static inline uint8_t HAL_SPI_ReadTxEmptyCount(const HAL_SPI_T * p_hal) { return (uint8_t)_HAL_SPI_ReadIsTxRegEmpty(p_hal); }
static inline uint8_t HAL_SPI_ReadRxFullCount(const HAL_SPI_T * p_hal) { return (uint8_t)_HAL_SPI_ReadIsRxRegFull(p_hal); }

static inline bool HAL_SPI_ReadModeFault(const HAL_SPI_T * p_hal) { return ((p_hal->S & SPI_S_MODF_MASK) != 0U); }
static inline bool HAL_SPI_ReadMatchFlag(const HAL_SPI_T * p_hal) { return ((p_hal->S & SPI_S_SPMF_MASK) != 0U); }

/*
    KE06 SPI has no dedicated Rx overrun flag; callers poll SPRF and read D in time.
    ClearRxErrors: MODF is cleared by reading S with MODF=1 then writing C1. */
static inline bool HAL_SPI_ReadRxOverrun(HAL_SPI_T * p_hal) { (void)p_hal; return false; }
static inline void HAL_SPI_ClearRxErrors(HAL_SPI_T * p_hal)
{
    if ((p_hal->S & SPI_S_MODF_MASK) != 0U) { p_hal->C1 = p_hal->C1; }   /* MODF clear sequence */
}

/******************************************************************************/
/*!
    Interrupts
    SPTIE: Tx buffer empty interrupt.
    SPIE:  Rx buffer full (SPRF) and mode fault (MODF) interrupts share this enable.
    SPMIE: Match interrupt (optional).
*/
/******************************************************************************/
static inline void HAL_SPI_EnableTxInterrupt(HAL_SPI_T * p_hal) { p_hal->C1 |= SPI_C1_SPTIE_MASK; }
static inline void HAL_SPI_DisableTxInterrupt(HAL_SPI_T * p_hal) { p_hal->C1 &= (uint8_t)~SPI_C1_SPTIE_MASK; }
static inline void HAL_SPI_EnableRxInterrupt(HAL_SPI_T * p_hal) { p_hal->C1 |= SPI_C1_SPIE_MASK; }
static inline void HAL_SPI_DisableRxInterrupt(HAL_SPI_T * p_hal) { p_hal->C1 &= (uint8_t)~SPI_C1_SPIE_MASK; }

/******************************************************************************/
/*!
    Switch — KE06 SPI has one enable bit (SPE) for both directions.
    HAL_SPI_WriteTxSwitch / HAL_SPI_WriteRxSwitch both gate the same bit so the
    module can substitute the Serial-style API shape.
*/
/******************************************************************************/
static inline void HAL_SPI_WriteSwitch(HAL_SPI_T * p_hal, bool enable)
{
    p_hal->C1 = (p_hal->C1 & (uint8_t)~SPI_C1_SPE_MASK) | ((enable ? 1U : 0U) << SPI_C1_SPE_SHIFT);
    while (((p_hal->C1 & SPI_C1_SPE_MASK) != 0U) != enable) {}
}

static inline void HAL_SPI_WriteTxSwitch(HAL_SPI_T * p_hal, bool enable) { HAL_SPI_WriteSwitch(p_hal, enable); }
static inline void HAL_SPI_WriteRxSwitch(HAL_SPI_T * p_hal, bool enable) { HAL_SPI_WriteSwitch(p_hal, enable); }

/******************************************************************************/
/*! Mode / clock configuration */
/******************************************************************************/
static inline void HAL_SPI_SetMaster(HAL_SPI_T * p_hal, bool isMaster)
{
    p_hal->C1 = (p_hal->C1 & (uint8_t)~SPI_C1_MSTR_MASK) | ((isMaster ? 1U : 0U) << SPI_C1_MSTR_SHIFT);
}

/*! mode: 0..3 → {CPOL, CPHA} */
static inline void HAL_SPI_SetClockMode(HAL_SPI_T * p_hal, uint8_t mode)
{
    p_hal->C1 = (p_hal->C1 & (uint8_t)~(SPI_C1_CPOL_MASK | SPI_C1_CPHA_MASK)) | SPI_C1_CPOL((mode >> 1) & 1U) | SPI_C1_CPHA(mode & 1U);
}

static inline void HAL_SPI_SetLsbFirst(HAL_SPI_T * p_hal, bool lsbFirst)
{
    p_hal->C1 = (p_hal->C1 & (uint8_t)~SPI_C1_LSBFE_MASK) | ((lsbFirst ? 1U : 0U) << SPI_C1_LSBFE_SHIFT);
}

/******************************************************************************/
/*! Baud rate
    Bus / baud = (SPPR+1) * 2^(SPR+1)
    SPPR: 0..7 (prescaler 1..8), SPR: 0..8 (divisor 2,4,8,...,512).
    Search all 9 SPR values, pick best SPPR for each, track minimum |error|.
*/
/******************************************************************************/
static inline bool HAL_SPI_InitBaudRate(HAL_SPI_T * p_hal, uint32_t baudRate_Bps)
{
    if (baudRate_Bps == 0U) { return false; }

    uint32_t bestErr  = UINT32_MAX;
    uint8_t  bestSpr  = 0U;
    uint8_t  bestSppr = 0U;

    for (uint8_t spr = 0U; spr <= 8U; spr++)
    {
        uint32_t pow2   = (1UL << (spr + 1U));                                   /* 2, 4, 8, ..., 512 */
        uint32_t denom  = pow2 * baudRate_Bps;
        uint32_t sppr   = (HAL_SPI_CLOCK_SOURCE_FREQ + (denom / 2U)) / denom;    /* rounded prescaler */
        if (sppr == 0U) { sppr = 1U; }
        if (sppr > 8U)  { sppr = 8U; }
        uint32_t actual = HAL_SPI_CLOCK_SOURCE_FREQ / (sppr * pow2);
        uint32_t err    = (actual > baudRate_Bps) ? (actual - baudRate_Bps) : (baudRate_Bps - actual);
        if (err < bestErr) { bestErr = err; bestSpr = spr; bestSppr = (uint8_t)(sppr - 1U); }
    }

    p_hal->BR = (uint8_t)(SPI_BR_SPPR(bestSppr) | SPI_BR_SPR(bestSpr));
    return true;
}

/******************************************************************************/
/*! Init / Deinit */
/******************************************************************************/
static inline void HAL_SPI_Init(HAL_SPI_T * p_hal)
{
    p_hal->C1 = 0U;                                                      /* disable, clear interrupts */
    p_hal->C2 = 0U;                                                      /* normal pin mode, 7-bit addr, no match IRQ */
    p_hal->BR = 0U;
    (void)p_hal->S;                                                      /* clear sticky MODF via S→C1 sequence below if needed */
}

static inline void HAL_SPI_Deinit(HAL_SPI_T * p_hal)
{
    HAL_SPI_DisableTxInterrupt(p_hal);
    HAL_SPI_DisableRxInterrupt(p_hal);
    HAL_SPI_WriteSwitch(p_hal, false);
}

#endif
