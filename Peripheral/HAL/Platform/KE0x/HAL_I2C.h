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
    @file   HAL_I2C.h
    @author FireSourcery
    @brief  I2C HAL for KE0x (MKE06Z/MKE02Z) — register-level access
            SCL = Bus / (MULT * SCL_divider) where MULT ∈ {1,2,4}
            and SCL_divider comes from the 64-entry ICR lookup (non-linear).
            Note: IICIF, ARBL, STARTF, STOPF are w1c — write exact bit, not RMW.
*/
/******************************************************************************/
#ifndef HAL_I2C_PLATFORM_H
#define HAL_I2C_PLATFORM_H

#include "KE0x.h"

#include <stdint.h>
#include <stdbool.h>

#ifndef HAL_I2C_CLOCK_SOURCE_FREQ
#define HAL_I2C_CLOCK_SOURCE_FREQ (CPU_FREQ / 2U)
#endif

typedef I2C_Type HAL_I2C_T;

/******************************************************************************/
/*! Data — 8-bit.
    On Rx, reading D launches the next byte clock cycle while returning the previous one.
    Callers managing the last-byte-before-STOP/RSTART sequence must set TXAK / TX
    before reading the penultimate byte (MKE06 RM §Master Receiver).
*/
/******************************************************************************/
static inline void HAL_I2C_WriteTxChar(HAL_I2C_T * p_hal, uint8_t txChar) { p_hal->D = txChar; }
static inline uint8_t HAL_I2C_ReadRxChar(const HAL_I2C_T * p_hal)         { return (uint8_t)p_hal->D; }

/******************************************************************************/
/*! Status */
/******************************************************************************/
static inline bool HAL_I2C_ReadTransferComplete(const HAL_I2C_T * p_hal)  { return ((p_hal->S & I2C_S_TCF_MASK) != 0U); }
static inline bool HAL_I2C_ReadInterruptFlag(const HAL_I2C_T * p_hal)     { return ((p_hal->S & I2C_S_IICIF_MASK) != 0U); }
static inline bool HAL_I2C_ReadBusy(const HAL_I2C_T * p_hal)              { return ((p_hal->S & I2C_S_BUSY_MASK) != 0U); }
static inline bool HAL_I2C_ReadRxAck(const HAL_I2C_T * p_hal)             { return ((p_hal->S & I2C_S_RXAK_MASK) == 0U); }     /* bit=0 → ACK received */
static inline bool HAL_I2C_ReadArbitrationLost(const HAL_I2C_T * p_hal)   { return ((p_hal->S & I2C_S_ARBL_MASK) != 0U); }
static inline bool HAL_I2C_ReadAddressedAsSlave(const HAL_I2C_T * p_hal)  { return ((p_hal->S & I2C_S_IAAS_MASK) != 0U); }
static inline bool HAL_I2C_ReadSlaveReadRequest(const HAL_I2C_T * p_hal)  { return ((p_hal->S & I2C_S_SRW_MASK) != 0U); }

/* Equivalents of the Serial-style naming — TCF handles both directions */
static inline uint8_t HAL_I2C_ReadTxEmptyCount(const HAL_I2C_T * p_hal)   { return (uint8_t)HAL_I2C_ReadTransferComplete(p_hal); }
static inline uint8_t HAL_I2C_ReadRxFullCount(const HAL_I2C_T * p_hal)    { return (uint8_t)HAL_I2C_ReadTransferComplete(p_hal); }

/******************************************************************************/
/*! Flag clearing — w1c bits. Must write the exact bit, not RMW. */
/******************************************************************************/
static inline void HAL_I2C_ClearInterruptFlag(HAL_I2C_T * p_hal)          { p_hal->S = I2C_S_IICIF_MASK; }
static inline void HAL_I2C_ClearArbitrationLost(HAL_I2C_T * p_hal)        { p_hal->S = I2C_S_ARBL_MASK; }
static inline void HAL_I2C_ClearStartFlag(HAL_I2C_T * p_hal)              { p_hal->FLT = I2C_FLT_STARTF_MASK; }
static inline void HAL_I2C_ClearStopFlag(HAL_I2C_T * p_hal)               { p_hal->FLT = I2C_FLT_STOPF_MASK; }
static inline void HAL_I2C_ClearRxErrors(HAL_I2C_T * p_hal)               { p_hal->S = I2C_S_ARBL_MASK; }

/******************************************************************************/
/*! Interrupts */
/******************************************************************************/
static inline void HAL_I2C_EnableInterrupt(HAL_I2C_T * p_hal)             { p_hal->C1 |= I2C_C1_IICIE_MASK; }
static inline void HAL_I2C_DisableInterrupt(HAL_I2C_T * p_hal)            { p_hal->C1 &= (uint8_t)~I2C_C1_IICIE_MASK; }

/* Tx-empty and Rx-full share the single IICIF source on KE06 */
static inline void HAL_I2C_EnableTxInterrupt(HAL_I2C_T * p_hal)           { HAL_I2C_EnableInterrupt(p_hal); }
static inline void HAL_I2C_DisableTxInterrupt(HAL_I2C_T * p_hal)          { HAL_I2C_DisableInterrupt(p_hal); }
static inline void HAL_I2C_EnableRxInterrupt(HAL_I2C_T * p_hal)           { HAL_I2C_EnableInterrupt(p_hal); }
static inline void HAL_I2C_DisableRxInterrupt(HAL_I2C_T * p_hal)          { HAL_I2C_DisableInterrupt(p_hal); }

/******************************************************************************/
/*! Bus control — START / STOP / Repeated START
    START:  set MST while bus is idle.
    STOP:   clear MST.
    RSTART: write RSTA while master.
*/
/******************************************************************************/
static inline void HAL_I2C_SetTxDirection(HAL_I2C_T * p_hal)              { p_hal->C1 |= I2C_C1_TX_MASK; }
static inline void HAL_I2C_SetRxDirection(HAL_I2C_T * p_hal)              { p_hal->C1 &= (uint8_t)~I2C_C1_TX_MASK; }
static inline void HAL_I2C_EnableAck(HAL_I2C_T * p_hal)                   { p_hal->C1 &= (uint8_t)~I2C_C1_TXAK_MASK; }          /* TXAK=0 → ACK */
static inline void HAL_I2C_DisableAck(HAL_I2C_T * p_hal)                  { p_hal->C1 |= I2C_C1_TXAK_MASK; }                    /* TXAK=1 → NACK */

static inline void HAL_I2C_WriteStart(HAL_I2C_T * p_hal)                  { p_hal->C1 |= (uint8_t)(I2C_C1_MST_MASK | I2C_C1_TX_MASK); }
static inline void HAL_I2C_WriteStop(HAL_I2C_T * p_hal)                   { p_hal->C1 &= (uint8_t)~(I2C_C1_MST_MASK | I2C_C1_TX_MASK); }
static inline void HAL_I2C_WriteRepeatedStart(HAL_I2C_T * p_hal)          { p_hal->C1 |= (uint8_t)(I2C_C1_RSTA_MASK | I2C_C1_TX_MASK); }

/******************************************************************************/
/*! Switch — IICEN. Serial-parity aliases so module code can share call shape. */
/******************************************************************************/
static inline void HAL_I2C_WriteSwitch(HAL_I2C_T * p_hal, bool enable)
{
    p_hal->C1 = (p_hal->C1 & (uint8_t)~I2C_C1_IICEN_MASK) | ((enable ? 1U : 0U) << I2C_C1_IICEN_SHIFT);
    while (((p_hal->C1 & I2C_C1_IICEN_MASK) != 0U) != enable) {}
}

static inline void HAL_I2C_WriteTxSwitch(HAL_I2C_T * p_hal, bool enable) { HAL_I2C_WriteSwitch(p_hal, enable); }
static inline void HAL_I2C_WriteRxSwitch(HAL_I2C_T * p_hal, bool enable) { HAL_I2C_WriteSwitch(p_hal, enable); }

/******************************************************************************/
/*! Slave address (7-bit, shifted into A1[7:1]) */
/******************************************************************************/
static inline void HAL_I2C_SetSlaveAddress(HAL_I2C_T * p_hal, uint8_t addr7) { p_hal->A1 = I2C_A1_AD(addr7); }

/******************************************************************************/
/*! Baud rate
    SCL = Bus / (MULT * ICR_divider)
    ICR → divider lookup from KE06 RM Table "I2C divider and hold values" (64 entries).
    Routine picks the (mult, icr) combination whose actual SCL is ≤ baudRate
    and closest to it.
*/
/******************************************************************************/
static inline bool HAL_I2C_InitBaudRate(HAL_I2C_T * p_hal, uint32_t baudRate_Bps)
{
    static const uint16_t ICR_TO_DIV[64] = {
        20,   22,   24,   26,   28,   30,   34,   40,
        28,   32,   36,   40,   44,   48,   56,   68,
        48,   56,   64,   72,   80,   88,   104,  128,
        80,   96,   112,  128,  144,  160,  192,  240,
        160,  192,  224,  256,  288,  320,  384,  480,
        320,  384,  448,  512,  576,  640,  768,  960,
        640,  768,  896,  1024, 1152, 1280, 1536, 1920,
        1280, 1536, 1792, 2048, 2304, 2560, 3072, 3840,
    };
    static const uint8_t MULTS[3] = { 1U, 2U, 4U };

    if (baudRate_Bps == 0U) { return false; }

    uint32_t bestAbsErr = UINT32_MAX;
    uint8_t  bestMult   = 0U;
    uint8_t  bestIcr    = 0U;

    for (uint8_t m = 0U; m < 3U; m++)
    {
        for (uint8_t i = 0U; i < 64U; i++)
        {
            uint32_t actual = HAL_I2C_CLOCK_SOURCE_FREQ / ((uint32_t)MULTS[m] * (uint32_t)ICR_TO_DIV[i]);
            uint32_t err    = (actual > baudRate_Bps) ? (actual - baudRate_Bps) : (baudRate_Bps - actual);
            if (err < bestAbsErr) { bestAbsErr = err; bestMult = m; bestIcr = i; }
        }
    }

    p_hal->F = (uint8_t)(I2C_F_MULT(bestMult) | I2C_F_ICR(bestIcr));
    return true;
}

/******************************************************************************/
/*! Init / Deinit */
/******************************************************************************/
static inline void HAL_I2C_Init(HAL_I2C_T * p_hal)
{
    p_hal->C1  = 0U;
    p_hal->C2  = 0U;
    p_hal->FLT = 0U;
    p_hal->S   = (uint8_t)(I2C_S_IICIF_MASK | I2C_S_ARBL_MASK);          /* w1c any stale flags */
}

static inline void HAL_I2C_Deinit(HAL_I2C_T * p_hal)
{
    HAL_I2C_DisableInterrupt(p_hal);
    HAL_I2C_WriteSwitch(p_hal, false);
}

#endif
