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
typedef size_t (*CanBus_BuildData_T)(void * p_context, uint8_t * p_txData);
typedef void (*CanBus_BuildFrame_T)(void * p_context, CAN_Frame_T * p_frame);

typedef CanBus_BuildFrame_T CanBus_BuildBroadcast_T;
// #ifdef CAN_BUS_BROADCAST_FRAME_BASED
// typedef CanBus_BuildFrame_T CanBus_BuildBroadcast_T;
// #elif defined(CAN_BUS_BROADCAST_DATA_BASED)
// typedef CanBus_BuildData_T CanBus_BuildBroadcast_T;
// #endif

typedef void (*CanBus_Tick_T)(void * p_context, uint32_t dt_us);

// keep for interface
typedef struct
{
    // uint32_t Elapsed; /* Millis or Micros */
    uint32_t Timestamp; /* last broadcast */
    bool Enabled; /* optionally individual disable active */
}
CanBus_BroadcastState_T;

typedef const struct
{
    uint32_t ID; /*   */
    CanBus_BuildBroadcast_T BUILD;  /* Frame-based broadcast — caller fills a full CAN_Frame_T (ID, DLC, data). */
    uint32_t INTERVAL;
    CanBus_BroadcastState_T * P_STATE;  /* allocate per entry, alternatively collective handle */
    // CanBus_ServiceInit_T INIT;
}
CanBus_BroadcastEntry_T;

/*
    Disabled
*/
static void  CanBus_BuildEmpty(void * p_context, CAN_Frame_T * p_frame) { (void)p_context; (void)p_frame; }
static const CanBus_BroadcastEntry_T CAN_BUS_BROADCAST_EMPTY = { .BUILD = CanBus_BuildEmpty, .ID = 0U, .INTERVAL = 0U, .P_STATE = NULL };

/* App context is the driver's P_CONTEXT (single source). */
 /* Frame-form: callee fills ID, DLC, data (e.g. CiA402 TxPDO) */
static inline void CanBus_ProcBroadcast(CanBus_T * p_can, CanBus_BroadcastEntry_T * p_broadcast)
{
    CAN_Frame_T frame = { 0U };
    frame.CanId.CanId = p_broadcast->ID; /* seed default ID; frame builders (e.g. CiA402) may override */
    p_broadcast->BUILD(p_can->P_CONTEXT, &frame);
    HAL_CAN_WriteTxMessage(p_can->P_HAL, &frame);
}

/* Data-form: fixed ID from entry, 8 data bytes (e.g. MotCan telemetry) */
// static inline void _CanBus_ProcBroadcast_Data(CanBus_T * p_can, CanBus_BroadcastEntry_T * p_broadcast)
// {
//     uint8_t data[8U];
//     p_broadcast->BUILD(p_can->P_CONTEXT, &data[0U]);
//     HAL_CAN_WriteTx(p_can->P_HAL, ((can_id_t) {.CanId = p_broadcast->ID }), &data[0U], 8U);
// }

// static inline void _CanBus_ProcBroadcastService(CanBus_T * p_can, CanBus_BroadcastEntry_T * p_b, CanBus_BroadcastState_T * p_s, uint8_t count, uint32_t timer)
static inline void _CanBus_ProcBroadcastService(CanBus_T * p_can, CanBus_BroadcastEntry_T * p_table, uint8_t count, uint32_t timer)
{
    for (uint8_t i = 0U; i < count; i++)
    {
        if ((timer - p_table[i].P_STATE->Timestamp) >= p_table[i].INTERVAL)
        {
            CanBus_ProcBroadcast(p_can, &p_table[i]);
            p_table[i].P_STATE->Timestamp = timer;
        }
    }
}


/******************************************************************************/
/*!
    CAN-ID/COB-ID mapping, request semantics, periodic protocol frames
    Inbound dispatch table — route a received frame to the right handler based on COB-ID range.
*/
/******************************************************************************/
/*
    this layer handles request routing
*/
// typedef void (*CanBus_RouteHandler_T)(void * p_dev, const uint8_t * p_rx, uint8_t * p_tx);
// typedef void (*CanBus_RouteHandler_T)(void * p_dev, void * adapter, const uint8_t * p_rx, uint8_t * p_tx);
typedef void (*CanBus_RouteHandler_T)(void * p_dev, const CAN_Frame_T * p_rxFrame, CAN_Frame_T * p_txFrame);

typedef const struct
{
    uint32_t           ID_MATCH;     /* expected (id & ID_MASK) */
    uint32_t           ID_MASK;      /* bits to compare; 0x7FF for exact, 0x780 for COB-ID class */
    CanBus_RouteHandler_T HANDLER;
    // CanBus_RxHandler_T   HANDLER;      /* called with full frame */
}
CanBus_ReqRoute_T;

// alternative to table search
// typedef CanBus_ReqRoute_T * (*CanBus_RxRequestMapper_T)(void * p_dev, uint32_t id);

static inline CanBus_ReqRoute_T * CanBus_SearchRxTable(CanBus_ReqRoute_T * p_routes, uint8_t count, uint32_t id)
{
    for (uint8_t i = 0U; i < count; i++)
    {
        if ((id & p_routes[i].ID_MASK) == p_routes[i].ID_MATCH) { return &p_routes[i]; }
    }
    return NULL;
}

static inline void _CanBus_ProcRequestService(CanBus_T * p_can, CanBus_ReqRoute_T * p_table, uint8_t count, const CAN_Frame_T * p_rxFrame)
{
    CAN_Frame_T txFrame = { 0U };
    CanBus_ReqRoute_T * p_route = CanBus_SearchRxTable(p_table, count, p_rxFrame->CanId.Id);
    if (p_route != NULL) { p_route->HANDLER(p_can->P_CONTEXT, p_rxFrame, &txFrame); }
    if (txFrame.DataLength > 0U) { HAL_CAN_WriteTxMessage(p_can->P_HAL, &txFrame); }
}


/*
    CanBus_ReqHandler_T
    application layer handles request routing, or a single handler for all requests.
*/
typedef void (*CanBus_ReqHandler_T)(void * p_dev, const CAN_Frame_T * p_rxFrame, CAN_Frame_T * p_txFrame);
// typedef void (*CanBus_ReqHandler_T)(void * p_dev, void * adapter, const CAN_Frame_T * p_rxFrame, CAN_Frame_T * p_txFrame);

typedef const struct CanBus_Service
{
    CanBus_BroadcastEntry_T * P_BROADCASTS;  uint8_t BROADCAST_COUNT;
    CanBus_ReqRoute_T * P_ROUTES;             uint8_t ROUTE_COUNT;
    // CanBus_RxRequestMapper_T REQ_MAPPER;
    // CanBus_ReqHandler_T REQ_HANDLER;
    // const volatile uint32_t * P_TIMER;
}
CanBus_Service_T;


/*
    proc buffered frame, without isr prority
    poll rx buffer, or call form isr
    Dispatch one inbound frame: route-table match first, else the service-wide REQ_HANDLER.
    Handler fills txFrame (ID/DLC/data); a non-zero DataLength is transmitted as the reply.
*/
static inline void CanBus_ProcRequestService(CanBus_T * p_can)
{
    CAN_Frame_T * p_rxFrame = &p_can->P_STATE->Channel[0U].Frame; // todo handle selection
    CanBus_Service_T * p_service = p_can->P_STATE->p_Service;
    if (p_service == NULL) { return; } /* disabled — no active protocol */

    _CanBus_ProcRequestService(p_can, p_service->P_ROUTES, p_service->ROUTE_COUNT, p_rxFrame);
    // if (p_service->REQ_HANDLER != NULL) { p_service->REQ_HANDLER(p_can->P_CONTEXT, p_rxFrame, &txFrame); }
}

/*
    Proc in ISR context: poll rx buffer, if full, dispatch to service handler.
*/
static inline void CanBus_RxProcRequest_ISR(CanBus_T * p_can)
{
    CAN_Frame_T * p_rx = CanBus_PollRx(p_can);
    if (p_rx != NULL) { CanBus_ProcRequestService(p_can); }
}

static inline void CanBus_ProcBroadcastService(CanBus_T * p_can, uint32_t timer)
{
    CanBus_Service_T * p_service = p_can->P_STATE->p_Service;
    if (p_service == NULL) { return; } /* disabled — no active protocol */
    _CanBus_ProcBroadcastService(p_can, p_service->P_BROADCASTS, p_service->BROADCAST_COUNT, timer);
}

static inline void CanBus_Enable(CanBus_T * p_can, CanBus_Service_T * p_service)
{
    p_can->P_STATE->p_Service = p_service;
}

static inline void CanBus_Disable(CanBus_T * p_can)
{
    p_can->P_STATE->p_Service = NULL; // def empty to eliminate nullcheck
}

/*
    Runtime protocol swap — select the active service by index from the configured table.
    The active service is a single aligned pointer that the RX/broadcast paths reload each
    pass, so a repoint takes effect on the next frame/tick (atomic store on Cortex-M).
    TODO: on swap also reprogram HW acceptance filters from the new route table and rephase
        broadcast timestamps to avoid a startup burst on the newly selected service.
*/
static inline void CanBus_SelectService(CanBus_T * p_can, uint8_t index)
{
    if (index < p_can->SERVICE_COUNT) { CanBus_Enable(p_can, &p_can->P_SERVICE_TABLE[index]); }
}

