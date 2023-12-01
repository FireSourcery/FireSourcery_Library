/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

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
    @brief  DataLink Peripheral
    @version V0
*/
/******************************************************************************/
#ifndef XCVR_H
#define XCVR_H

#include "Config.h"

#if defined(CONFIG_XCVR_INTERFACE_PERIPHERAL)
#include "Peripheral/Serial/Serial.h"
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef enum Xcvr_Type
{
    XCVR_TYPE_SERIAL,
    XCVR_TYPE_I2C,
    XCVR_TYPE_SPI,
    XCVR_TYPE_VIRTUAL,
    XCVR_TYPE_INTERFACE,
}
Xcvr_Type_T;

typedef bool(*Xcvr_Interface_TxByte_T)          (void * p_xcvr, uint8_t txChar);
typedef bool(*Xcvr_Interface_RxByte_T)          (void * p_xcvr, uint8_t * p_rxChar);
typedef size_t(*Xcvr_Interface_TxMax_T)         (void * p_xcvr, const uint8_t * p_srcBuffer, size_t bufferSize);
typedef size_t(*Xcvr_Interface_RxMax_T)         (void * p_xcvr, uint8_t * p_destBuffer, size_t bufferSize);
typedef bool(*Xcvr_Interface_TxN_T)             (void * p_xcvr, const uint8_t * p_src, size_t length);
typedef bool(*Xcvr_Interface_RxN_T)             (void * p_xcvr, uint8_t * p_dest, size_t length);
typedef size_t(*Xcvr_Interface_GetCount_T)      (void * p_xcvr);
typedef void(*Xcvr_Interface_ConfigBaudRate_T)  (void * p_xcvr, uint32_t baudRate);

typedef const struct Xcvr_Interface
{
    Xcvr_Interface_TxByte_T         TX_BYTE;
    Xcvr_Interface_RxByte_T         RX_BYTE;
    Xcvr_Interface_TxMax_T          TX_MAX;
    Xcvr_Interface_RxMax_T          RX_MAX;
    Xcvr_Interface_TxN_T            TX_N;
    Xcvr_Interface_RxN_T            RX_N;
    Xcvr_Interface_GetCount_T       GET_TX_EMPTY_COUNT;
    Xcvr_Interface_GetCount_T       GET_RX_FULL_COUNT;
    Xcvr_Interface_ConfigBaudRate_T CONFIG_BAUD_RATE;
}
Xcvr_Interface_T;

/*
    Xcvr Entry/Instance
    Entry in P_XCVR_TABLE
*/
typedef const struct Xcvr_Xcvr
{
    void * P_CONTEXT; /* Xcvr data struct */
    Xcvr_Type_T TYPE;
    Xcvr_Interface_T * P_INTERFACE; /* If applicable, user external defined */
}
Xcvr_Entry_T;

#define XCVR_ENTRY_INIT(p_Xcvr, Type)   \
{                                       \
    .P_CONTEXT  = (void *)p_Xcvr,       \
    .TYPE       = Type,                 \
}

#define XCVR_ENTRY_INIT_INTERFACE(p_Xcvr, p_Interface)  \
{                                                       \
    .P_CONTEXT      = (void *)p_Xcvr,                   \
    .TYPE           = XCVR_TYPE_INTERFACE,              \
    .P_INTERFACE    = p_Interface,                      \
}

typedef const struct Xcvr_Config
{
    const Xcvr_Entry_T * const P_XCVR_TABLE;
    const uint8_t XCVR_TABLE_LENGTH;
}
Xcvr_Config_T;

/*
   Todo  Xcvr_T, Xcvr_Table_T => 1 layer of dereference for operations
*/
typedef struct Xcvr
{
    const Xcvr_Config_T CONFIG;
    Xcvr_Entry_T * p_Xcvr;
}
Xcvr_T;

#define XCVR_INIT(p_XcvrTable, Count) { .CONFIG = { .P_XCVR_TABLE = p_XcvrTable, .XCVR_TABLE_LENGTH = Count, }, }

extern bool Xcvr_TxByte(const Xcvr_T * p_xcvr, uint8_t txChar);
extern bool Xcvr_RxByte(const Xcvr_T * p_xcvr, uint8_t * p_rxChar);
extern bool Xcvr_TxN(const Xcvr_T * p_xcvr, const uint8_t * p_src, size_t length);
extern bool Xcvr_RxN(const Xcvr_T * p_xcvr, uint8_t * p_dest, size_t length);
extern size_t Xcvr_TxMax(const Xcvr_T * p_xcvr, const uint8_t * p_srcBuffer, size_t srcSize);
extern size_t Xcvr_RxMax(const Xcvr_T * p_xcvr, uint8_t * p_destBuffer, size_t destSize);

static inline bool Xcvr_Tx(const Xcvr_T * p_xcvr, const uint8_t * p_src, size_t length)         { return Xcvr_TxN(p_xcvr, p_src, length); }
static inline size_t Xcvr_Rx(const Xcvr_T * p_xcvr, uint8_t * p_destBuffer, size_t destSize)    { return Xcvr_RxMax(p_xcvr, p_destBuffer, destSize); }

extern void Xcvr_Init(Xcvr_T * p_xcvr, uint8_t xcvrDefaultIndex);
extern bool Xcvr_SetXcvr(Xcvr_T * p_xcvr, uint8_t xcvrIndex);
extern bool Xcvr_CheckIsSet(const Xcvr_T * p_xcvr, uint8_t xcvrIndex);
extern bool Xcvr_CheckIsValid(const Xcvr_T * p_xcvr, void * p_target);
extern bool Xcvr_ConfigBaudRate(const Xcvr_T * p_xcvr, uint32_t baudRate);
extern size_t Xcvr_GetRxFullCount(const Xcvr_T * p_xcvr);
extern size_t Xcvr_GetTxEmptyCount(const Xcvr_T * p_xcvr);

#if defined(CONFIG_XCVR_INTERFACE_PERIPHERAL)
extern uint8_t * Xcvr_AcquireTxBuffer(const Xcvr_T * p_xcvr);
extern void Xcvr_ReleaseTxBuffer(const Xcvr_T * p_xcvr, size_t writeSize);
#endif

#endif



