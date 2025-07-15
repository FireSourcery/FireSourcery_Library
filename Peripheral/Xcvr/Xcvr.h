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

#include "Config.h"

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
// typedef bool(*Xcvr_RxN_T)        (void * p_xcvr, size_t length, uint8_t * p_dest);

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
    // Xcvr_Type_T TYPE; // remove type and use vtable only
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


/*
    Switch impl
*/
// typedef enum Xcvr_Type
// {
//     XCVR_TYPE_SERIAL,
//     XCVR_TYPE_I2C,
//     XCVR_TYPE_SPI,
//     XCVR_TYPE_VIRTUAL,
//     XCVR_TYPE_INTERFACE,
// }
// Xcvr_Type_T;
// static inline bool Xcvr_TxByte(const Xcvr_T * p_xcvr, uint8_t txChar)
// {
//     return p_xcvr->p_Xcvr->P_VTABLE->TX_BYTE(p_xcvr->p_Xcvr->P_BASE, txChar);
//     // return p_xcvr->P_VTABLE->TX_BYTE(p_xcvr->P_BASE, txChar);
//     // return p_xcvr->P_VTABLE->TX_BYTE(p_xcvr, txChar);
// }

// static inline bool Xcvr_RxByte(const Xcvr_T * p_xcvr, uint8_t * p_rxChar)
// {
//     return p_xcvr->p_Xcvr->P_VTABLE->RX_BYTE(p_xcvr->p_Xcvr->P_BASE, p_rxChar);
// }

// static inline bool Xcvr_TxN(const Xcvr_T * p_xcvr, const uint8_t * p_src, size_t length)
// {
//     bool status;
// #if     defined(CONFIG_XCVR_INTERFACE_PERIPHERAL)
//     switch(p_xcvr->p_Xcvr->TYPE)
//     {
//         case XCVR_TYPE_SERIAL:      status = Serial_SendN(p_xcvr->p_Xcvr->P_BASE, p_src, length); break;
//         case XCVR_TYPE_I2C:         status = false;     break;
//         case XCVR_TYPE_SPI:         status = false;     break;
//         case XCVR_TYPE_VIRTUAL:     status = false;     break;
//         default:                    status = false;     break;
//     }
// #elif     defined(CONFIG_XCVR_INTERFACE_VTABLE_ONLY)
//     status = p_xcvr->p_Xcvr->P_VTABLE->TX_N(p_xcvr->p_Xcvr->P_BASE, p_src, length);
// #endif
//     return status;
// }

// static inline bool Xcvr_RxN(const Xcvr_T * p_xcvr, uint8_t * p_dest, size_t length)
// {
//     return p_xcvr->p_Xcvr->P_VTABLE->RX_N(p_xcvr->p_Xcvr->P_BASE, p_dest, length);
// }

// static inline size_t Xcvr_TxMax(const Xcvr_T * p_xcvr, const uint8_t * p_srcBuffer, size_t srcSize)
// {
//     return p_xcvr->p_Xcvr->P_VTABLE->TX_MAX(p_xcvr->p_Xcvr->P_BASE, p_srcBuffer, srcSize);
// }

// static inline size_t Xcvr_RxMax(const Xcvr_T * p_xcvr, uint8_t * p_destBuffer, size_t destSize)
// {
//     size_t rxCount;
// #if     defined(CONFIG_XCVR_INTERFACE_PERIPHERAL)
//     switch(p_xcvr->p_Xcvr->TYPE)
//     {
//         case XCVR_TYPE_SERIAL:      rxCount = Serial_RecvMax(p_xcvr->p_Xcvr->P_BASE, p_destBuffer, destSize); break;
//         case XCVR_TYPE_I2C:         rxCount = 0U; break;
//         case XCVR_TYPE_SPI:         rxCount = 0U; break;
//         case XCVR_TYPE_VIRTUAL:     rxCount = 0U; break;
//         default:                    rxCount = 0U; break;
//     }
// #elif     defined(CONFIG_XCVR_INTERFACE_VTABLE_ONLY)
//     rxCount = p_xcvr->p_Xcvr->P_VTABLE->RX_MAX(p_xcvr->p_Xcvr->P_BASE, p_destBuffer, destSize);
// #endif
//     return rxCount;
// }

// static inline size_t Xcvr_GetRxFullCount(const Xcvr_T * p_xcvr)
// {
//     size_t count;
// #if     defined(CONFIG_XCVR_INTERFACE_PERIPHERAL)
//     switch(p_xcvr->p_Xcvr->TYPE)
//     {
//         case XCVR_TYPE_SERIAL:      count = Serial_GetRxFullCount(p_xcvr->p_Xcvr->P_BASE); break;
//         case XCVR_TYPE_I2C:         count = 0U;    break;
//         case XCVR_TYPE_SPI:         count = 0U;    break;
//         case XCVR_TYPE_VIRTUAL:     count = 0U;    break;
//         default:                    count = 0U; break;
//     }
// #elif     defined(CONFIG_XCVR_INTERFACE_VTABLE_ONLY)
//     count = p_xcvr->p_Xcvr->P_VTABLE->GET_RX_FULL_COUNT(p_xcvr->p_Xcvr->P_BASE);
// #endif
//     return count;
// }

// static inline size_t Xcvr_GetTxEmptyCount(const Xcvr_T * p_xcvr)
// {
//     size_t count;
// #if     defined(CONFIG_XCVR_INTERFACE_PERIPHERAL)
//     switch(p_xcvr->p_Xcvr->TYPE)
//     {
//         case XCVR_TYPE_SERIAL:      count = Serial_GetTxEmptyCount(p_xcvr->p_Xcvr->P_BASE); break;
//         case XCVR_TYPE_I2C:         count = 0U;    break;
//         case XCVR_TYPE_SPI:         count = 0U;    break;
//         case XCVR_TYPE_VIRTUAL:     count = 0U;    break;
//         default:                    count = 0U; break;
//     }
// #elif     defined(CONFIG_XCVR_INTERFACE_VTABLE_ONLY)
//     count = p_xcvr->p_Xcvr->P_VTABLE->GET_TX_EMPTY_COUNT(p_xcvr->p_Xcvr->P_BASE);
// #endif
//     return count;
// }

// static inline bool Xcvr_ConfigBaudRate(const Xcvr_T * p_xcvr, uint32_t baudRate)
// {
//     bool isSuccess = true;
// #if     defined(CONFIG_XCVR_INTERFACE_PERIPHERAL)
//     switch (p_xcvr->p_Xcvr->TYPE)
//     {
//         case XCVR_TYPE_SERIAL:      isSuccess = Serial_ConfigBaudRate(p_xcvr->p_Xcvr->P_BASE, baudRate);    break;
//         case XCVR_TYPE_I2C:         break;
//         case XCVR_TYPE_SPI:         break;
//         case XCVR_TYPE_VIRTUAL:     break;
//         default: break;
//     }
// #elif     defined(CONFIG_XCVR_INTERFACE_VTABLE_ONLY)
//     if (p_xcvr->p_Xcvr->P_VTABLE->CONST_BAUD_RATE != 0U) { p_xcvr->p_Xcvr->P_VTABLE->CONST_BAUD_RATE(p_xcvr->p_Xcvr->P_BASE, baudRate); }
// #endif
//     return isSuccess;
// }

// static inline bool Xcvr_Tx(const Xcvr_T * p_xcvr, const uint8_t * p_src, size_t length)         { return Xcvr_TxN(p_xcvr, p_src, length); }
// static inline size_t Xcvr_Rx(const Xcvr_T * p_xcvr, uint8_t * p_destBuffer, size_t destSize)    { return Xcvr_RxMax(p_xcvr, p_destBuffer, destSize); }



// #if defined(CONFIG_XCVR_INTERFACE_PERIPHERAL)
// extern uint8_t * Xcvr_AcquireTxBuffer(const Xcvr_T * p_xcvr);
// extern void Xcvr_ReleaseTxBuffer(const Xcvr_T * p_xcvr, size_t writeSize);
// #endif

