/******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSoucery / The Firebrand Forge Inc

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
	@file 	Xcvr.h
	@author FireSoucery
	@brief
	@version V0
*/
/******************************************************************************/
#ifndef XCVR_VIRTUAL_H
#define XCVR_VIRTUAL_H

#include "Utility/Ring/Ring.h"

typedef struct
{
	Ring_T RxRing;
	Ring_T TxRing;
}
XcvrVirtual_T;

#define XCVR_VIRTUAL_DEFINE(p_TxBuffer, p_RxBuffer, RingSize)	\
{																\
	.RxRing = ANALOG_INIT(p_RxBuffer, RingSize, 1U, 0U),		\
	.TxRing = ANALOG_INIT(p_TxBuffer, RingSize, 1U, 0U),		\
}

/*
	Use critical if multithreaded
*/
static inline bool XcvrVirtual_Tx(XcvrVirtual_T * p_xcvr, const uint8_t * p_srcBuffer, size_t length)
{
	return Ring_EnqueueN(&p_xcvr->TxRing, p_srcBuffer, length);
}

static inline uint32_t XcvrVirtual_Rx(XcvrVirtual_T * p_xcvr, uint8_t * p_destBuffer, size_t length)
{
	return Ring_DequeueMax(&p_xcvr->TxRing, p_destBuffer, length);
}

#endif
