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
    @file   SPI_Xcvr.h
    @author FireSourcery
    @brief  [SPI_T] as an [Xcvr_T].
*/
/******************************************************************************/
#include "SPI.h"
#include "../Xcvr/Xcvr.h"

// extern const Xcvr_VTable_T SPI_XCVR_VTABLE;

#define SPI_XCVR_INIT(p_Spi) XCVR_INIT((p_Spi), &SPI_XCVR_VTABLE)

/* Translates the generic flag word into the one thing SPI does with it */
static bool SetFlags(SPI_T * p_spi, uint32_t flags)
{
    SPI_SetCsHold(p_spi, ((flags & XCVR_FLAG_XFER_PENDING) != 0U));
    return true;
}

/*
    SET_TARGET is absent because SPI selects its peer with a wire, and one [SPI_T] is
    one device. CONFIG_BAUD_RATE is absent because the clock is const config, fixed by
    the device rather than negotiated at runtime.
*/
static const Xcvr_VTable_T SPI_XCVR_VTABLE =
{
    .TX       = (Xcvr_Tx_T)SPI_Tx,
    .RX       = (Xcvr_Rx_T)SPI_Rx,
    .SET_FLAGS  = (Xcvr_SetProperty_T)SetFlags,
};
