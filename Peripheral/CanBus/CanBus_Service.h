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
typedef void (*CanBus_BuildData_T)(void * p_context, uint8_t * p_txData);
// typedef size_t (*CanBus_BuildData_T)(void * p_context, uint8_t * p_txData);
typedef void (*CanBus_BuildFrame_T)(void * p_context, CAN_Frame_T * p_frame);
typedef void (*CanBus_Tick_T)(void * p_context, uint32_t dt_us);

// keep for interface
typedef struct
{
    uint32_t Elapsed; /* Millis or Micros */
}
CanBus_BroadcastState_T;

typedef const struct
{
    CanBus_BuildData_T BROADCAST;
    uint32_t ID; /* with fields */
    uint32_t INTERVAL; /* Caller handle */
    CanBus_BroadcastState_T * P_STATE;  /* allocate per entry */
    // CanBus_ServiceInit_T INIT;
}
CanBus_Broadcast_T;

/*!
    Frame-based broadcast — caller fills a full CAN_Frame_T (ID, DLC, data).
*/
// typedef const struct
// {
//     CanBus_BuildFrame_T BUILD;
//     uint32_t INTERVAL;
// }
// CanBus_FrameBroadcast_T;

static inline void CanBus_ProcBroadcast(CanBus_T * p_can, CanBus_Broadcast_T * p_broadcast)
{
    uint8_t data[8U];
    p_broadcast->BROADCAST(p_can->P_CONTEXT, &data[0U]);
    CanBus_Send(p_can, ((can_id_t) {.CanId = p_broadcast->ID }), &data[0U], 8U);
}

// typedef const struct
// {
//     CanBus_Broadcast_T * P_BROADCASTS;
//     // CanBus_BroadcastState_T * P_STATE;
//     uint8_t COUNT;
// }
// CanBus_BroadcastTable_T;

static inline void CanBus_ProcBroadcastTable(CanBus_T * p_can, CanBus_Broadcast_T * p_broadcasts, uint8_t count, uint32_t dt_us)
{
    for (uint8_t i = 0U; i < count; i++)
    {
        p_broadcasts[i].P_STATE->Elapsed += dt_us;
        if (p_broadcasts[i].P_STATE->Elapsed >= p_broadcasts[i].INTERVAL)
        {
            CanBus_ProcBroadcast(p_can, &p_broadcasts[i]);
            p_broadcasts[i].P_STATE->Elapsed = 0U;
        }
    }
}


/******************************************************************************/
/*!
    Inbound dispatch table — route a received frame to the right handler based on COB-ID range.
*/
/******************************************************************************/
typedef const struct
{
    uint32_t           ID_MATCH;     /* expected (id & ID_MASK) */
    uint32_t           ID_MASK;      /* bits to compare; 0x7FF for exact, 0x780 for COB-ID class */

    // CanBus_RxHandler_T   HANDLER;      /* called with full frame */
    // typedef void (*CanBus_RxRequest_T)(void * p_dev, uint32_t id, const uint8_t * p_data);
    // typedef void (*CanBus_RxFrame_T)(void * p_dev, const CAN_Frame_T * p_frame);
    // typedef void (*CanBus_RxFrame_T)(void * p_dev, const CAN_Frame_T * p_rxFrame, const CAN_Frame_T * p_txFrame);
}
CanBus_RxRoute_T;

static inline CanBus_RxRoute_T * CanBus_MapRxTable(CanBus_RxRoute_T * p_routes, uint8_t count, uint32_t id)
{
    for (uint8_t i = 0U; i < count; i++)
    {
        if ((id & p_routes[i].ID_MASK) == p_routes[i].ID_MATCH) { return &p_routes[i]; }
    }
    return NULL;
}

// direct write data to application buffer
typedef uint8_t * (*CanBus_RxDataMapper_T)(void * p_dev, uint32_t id);

// alternative to table
typedef CanBus_RxRoute_T * (*CanBus_RxRequestMapper_T)(void * p_dev, uint32_t id);
