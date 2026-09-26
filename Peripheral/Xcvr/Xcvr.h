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
    @brief  Message transport interface.
*/
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/*
    A whole message moves in one call, so the interface carries no byte level surface.
    Of (target, direction, buffer, length, flags), only buffer and length are arguments:
    direction is which function is called, and target and flags are set beforehand
    because only some transports have them.
*/
/*
    This message is part of a larger transaction: hold the bus open at its end.
    I2C suppresses STOP so a repeated START can follow, SPI keeps CS asserted.
*/
typedef enum Xcvr_Flags
{
    XCVR_FLAG_NONE          = 0U,
    XCVR_FLAG_XFER_PENDING  = 1U << 0U,
}
Xcvr_Flags_T;

/*
    Move one message. false when the message did not complete.
*/
typedef bool (*Xcvr_Tx_T)(void * p_xcvr, const uint8_t * p_src, size_t length);
typedef bool (*Xcvr_Rx_T)(void * p_xcvr, uint8_t * p_dest, size_t length);

/*
    Set one property of the next message. Absent on transports without it.
*/
typedef bool (*Xcvr_SetProperty_T)(void * p_xcvr, uint32_t value);

typedef const struct Xcvr_VTable
{
    Xcvr_Tx_T       TX;
    Xcvr_Rx_T       RX;
    Xcvr_SetProperty_T SET_TARGET;         /* Optional. I2C slave address, SPI chip select id */
    Xcvr_SetProperty_T SET_FLAGS;          /* Optional. [Xcvr_Flags_T] */
    Xcvr_SetProperty_T CONFIG_BAUD_RATE;   /* Optional. */
}
Xcvr_VTable_T;

/*
    Xcvr Instance
*/
typedef const struct Xcvr
{
    void * P_BASE;
    Xcvr_VTable_T * P_VTABLE;
}
Xcvr_T;

#define XCVR_INIT(p_XcvrBase, p_VTable) { .P_BASE = (void *)(p_XcvrBase), .P_VTABLE = (p_VTable), }

/*
    Inline wrap
*/
static inline bool Xcvr_TxN(Xcvr_T * p_xcvr, const uint8_t * p_src, size_t length) { return p_xcvr->P_VTABLE->TX(p_xcvr->P_BASE, p_src, length); }
static inline bool Xcvr_RxN(Xcvr_T * p_xcvr, uint8_t * p_dest, size_t length) { return p_xcvr->P_VTABLE->RX(p_xcvr->P_BASE, p_dest, length); }

/* An absent property is nothing to set, not a failure */
static inline bool _Xcvr_SetValue(Xcvr_T * p_xcvr, Xcvr_SetProperty_T set, uint32_t value) { return (set != NULL) ? set(p_xcvr->P_BASE, value) : true; }

static inline bool Xcvr_SetTarget(Xcvr_T * p_xcvr, uint32_t target)           { return _Xcvr_SetValue(p_xcvr, p_xcvr->P_VTABLE->SET_TARGET, target); }
static inline bool Xcvr_SetFlags(Xcvr_T * p_xcvr, Xcvr_Flags_T flags)         { return _Xcvr_SetValue(p_xcvr, p_xcvr->P_VTABLE->SET_FLAGS, (uint32_t)flags); }
static inline bool Xcvr_ConfigBaudRate(Xcvr_T * p_xcvr, uint32_t baudRate)    { return _Xcvr_SetValue(p_xcvr, p_xcvr->P_VTABLE->CONFIG_BAUD_RATE, baudRate); }
