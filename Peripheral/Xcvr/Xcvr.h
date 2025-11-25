#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery

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
    @file   Xcvr.h
    @author FireSourcery
    @brief
*/
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/*
    Serial Interface
*/
typedef bool    (*Xcvr_TxByte_T)    (void * p_xcvr, uint8_t txChar);
typedef bool    (*Xcvr_RxByte_T)    (void * p_xcvr, uint8_t * p_rxChar);
typedef size_t  (*Xcvr_TxMax_T)     (void * p_xcvr, const uint8_t * p_srcBuffer, size_t bufferSize);
typedef size_t  (*Xcvr_RxMax_T)     (void * p_xcvr, uint8_t * p_destBuffer, size_t bufferSize);
typedef bool    (*Xcvr_TxN_T)       (void * p_xcvr, const uint8_t * p_src, size_t length);
typedef bool    (*Xcvr_RxN_T)       (void * p_xcvr, uint8_t * p_dest, size_t length);
typedef size_t  (*Xcvr_GetCount_T)  (void * p_xcvr);
typedef bool    (*Xcvr_SetConfig_T) (void * p_xcvr, uint32_t value); // optionally split init and compare
// typedef void(*Xcvr_Init_T)        (void * p_xcvr );

/* Alternatively, length first on N */
// typedef bool    (*Xcvr_TxN_T)       (void * p_xcvr, size_t length, const uint8_t * p_src);
// typedef bool    (*Xcvr_RxN_T)       (void * p_xcvr, size_t length, uint8_t * p_dest);

typedef const struct Xcvr_VTable
{
    Xcvr_TxByte_T       TX_BYTE;
    Xcvr_RxByte_T       RX_BYTE;
    Xcvr_TxMax_T        TX_MAX; /* TxBytes */
    Xcvr_RxMax_T        RX_MAX; /* RxBytes */
    Xcvr_TxN_T          TX_N; /* TxFrame */
    Xcvr_RxN_T          RX_N; /* RxFrame */
    Xcvr_GetCount_T     GET_TX_EMPTY_COUNT;
    Xcvr_GetCount_T     GET_RX_FULL_COUNT;
    Xcvr_SetConfig_T    CONFIG_BAUD_RATE;
    // Xcvr_SetConfig_T    INIT_BAUD_RATE;
    // Xcvr_SetConfig_T    COMPARE_BAUD_RATE;
}
Xcvr_VTable_T;

/*
    Xcvr Instance
*/
typedef const struct Xcvr
{
    void * P_BASE; /* Xcvr data struct */
    const Xcvr_VTable_T * P_VTABLE;
}
Xcvr_T;

#define XCVR_INIT(p_XcvrBase, p_VTable) { .P_BASE = (void *)(p_XcvrBase), .P_VTABLE = (p_VTable), }

/*
    Inline wrap
*/
static inline bool Xcvr_TxByte(const Xcvr_T * p_xcvr, uint8_t txChar) { return p_xcvr->P_VTABLE->TX_BYTE(p_xcvr->P_BASE, txChar); }
static inline bool Xcvr_RxByte(const Xcvr_T * p_xcvr, uint8_t * p_rxChar) { return p_xcvr->P_VTABLE->RX_BYTE(p_xcvr->P_BASE, p_rxChar); }
static inline bool Xcvr_TxN(const Xcvr_T * p_xcvr, const uint8_t * p_src, size_t length) { return p_xcvr->P_VTABLE->TX_N(p_xcvr->P_BASE, p_src, length); }
static inline bool Xcvr_RxN(const Xcvr_T * p_xcvr, uint8_t * p_dest, size_t length) { return p_xcvr->P_VTABLE->RX_N(p_xcvr->P_BASE, p_dest, length); }
static inline size_t Xcvr_TxMax(const Xcvr_T * p_xcvr, const uint8_t * p_srcBuffer, size_t srcSize) { return p_xcvr->P_VTABLE->TX_MAX(p_xcvr->P_BASE, p_srcBuffer, srcSize); }
static inline size_t Xcvr_RxMax(const Xcvr_T * p_xcvr, uint8_t * p_destBuffer, size_t destSize) { return p_xcvr->P_VTABLE->RX_MAX(p_xcvr->P_BASE, p_destBuffer, destSize); }
static inline size_t Xcvr_GetRxFullCount(const Xcvr_T * p_xcvr) { return p_xcvr->P_VTABLE->GET_RX_FULL_COUNT(p_xcvr->P_BASE); }
static inline size_t Xcvr_GetTxEmptyCount(const Xcvr_T * p_xcvr) { return p_xcvr->P_VTABLE->GET_TX_EMPTY_COUNT(p_xcvr->P_BASE); }
static inline bool Xcvr_Tx(const Xcvr_T * p_xcvr, const uint8_t * p_src, size_t length)         { return Xcvr_TxN(p_xcvr, p_src, length); }
static inline size_t Xcvr_Rx(const Xcvr_T * p_xcvr, uint8_t * p_destBuffer, size_t destSize)    { return Xcvr_RxMax(p_xcvr, p_destBuffer, destSize); }

static inline bool Xcvr_ConfigBaudRate(const Xcvr_T * p_xcvr, uint32_t baudRate)
{
    bool isSuccess = true;
    if (p_xcvr->P_VTABLE->CONFIG_BAUD_RATE != NULL) { isSuccess = p_xcvr->P_VTABLE->CONFIG_BAUD_RATE(p_xcvr->P_BASE, baudRate); }
    return isSuccess;
}

// void Xcvr_InitFrom(const Xcvr_T ** pp_xcvr, Xcvr_Table_T * p_table, uint8_t xcvrIndex);
// bool Xcvr_AssignFrom(const Xcvr_T ** pp_xcvr, Xcvr_Table_T * p_table, uint8_t xcvrIndex);
// bool Xcvr_IsSet(const Xcvr_T * p_xcvr, Xcvr_Table_T * p_table, uint8_t xcvrIndex);
// bool Xcvr_IsValid(const Xcvr_Table_T * p_table, void * p_target);

// typedef const struct Xcvr_Table
// {
//     const Xcvr_T * const P_XCVRS;
//     const uint8_t LENGTH;
// }
// Xcvr_Table_T;

// #define XCVR_TABLE_INIT(p_XcvrTable, Count) { .P_XCVRS = p_XcvrTable, .LENGTH = Count, }
// #define XCVR_INIT_FIXED(p_XcvrTable) { .P_XCVRS = p_XcvrTable, .LENGTH = 1U, }

//static inline void Xcvr_EnableTx(const Xcvr_T * p_xcvr){}
//static inline void Xcvr_DisableTx(const Xcvr_T * p_xcvr){}
//static inline void Xcvr_EnableRxIsr(const Xcvr_T * p_xcvr){}
//static inline void Xcvr_DisableRxIsr(const Xcvr_T * p_xcvr){}
// size_t Xcvr_FlushRxBuffer(const Xcvr_T * p_xcvr)


