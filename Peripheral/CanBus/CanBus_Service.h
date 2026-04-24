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
    @file   CanBus_Service.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "CanBus.h"


/******************************************************************************/
/*! Service callbacks */
/******************************************************************************/
typedef void (*CanBus_BuildData_T)(void * p_dev, uint8_t * p_txData);

typedef const struct
{
    CanBus_BuildData_T BROADCAST;
    uint32_t ID; /* with fields */
    uint32_t INTERVAL; /* Caller handle */
    // CanBus_ServiceInit_T INIT;
}
CanBus_Broadcast_T;

static inline void CanBus_ProcBroadcast(CanBus_T * p_can, CanBus_Broadcast_T * p_broadcast)
{
    uint8_t data[8U];
    p_broadcast->BROADCAST(p_can->P_CONTEXT, &data[0U]);
    CanBus_Send(p_can, ((can_id_t) {.CanId = p_broadcast->ID }), &data[0U], 8U);
}
