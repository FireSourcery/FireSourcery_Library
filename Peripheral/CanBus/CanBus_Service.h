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
    bool Enabled; /* optionally individual disable active */
}
CanBus_BroadcastState_T;


typedef const struct
{
    uint32_t ID; /* with fields */
    CanBus_BuildData_T BROADCAST;
    CanBus_BuildFrame_T BUILD;  /* Frame-based broadcast — caller fills a full CAN_Frame_T (ID, DLC, data). */
    uint32_t INTERVAL; /* Caller handle */
    CanBus_BroadcastState_T * P_STATE;  /* allocate per entry, alternatively collective handle */
    // CanBus_ServiceInit_T INIT;
}
CanBus_BroadcastEntry_T;

/*
    Disabled
*/
static void  CanBus_BuildEmpty(void * p_context, uint8_t * p_txData) { (void)p_context; (void)p_txData; }
static const CanBus_BroadcastEntry_T CAN_BUS_BROADCAST_EMPTY = { .BROADCAST = CanBus_BuildEmpty, .ID = 0U, .INTERVAL = 0U, .P_STATE = NULL };


static inline void CanBus_ProcBroadcast(CanBus_T * p_can, CanBus_BroadcastEntry_T * p_broadcast)
{
    // if ((p_broadcast == NULL)) { return; } /* empty/disabled service is a no-op */
    uint8_t data[8U];
    p_broadcast->BROADCAST(p_can->P_CONTEXT, &data[0U]);
    // CanBus_TxData(p_can, ((can_id_t) {.CanId = p_broadcast->ID }), &data[0U], 8U);
    HAL_CAN_WriteTx(p_can->P_HAL, ((can_id_t) {.CanId = p_broadcast->ID }), &data[0U], 8U);
}

// typedef const struct
// {
//     CanBus_BroadcastEntry_T * P_BROADCASTS;
//     // CanBus_BroadcastState_T * P_STATE;
//     uint8_t COUNT;
// P_TIMER;
// }
// CanBus_BroadcastService_T;

// static inline void CanBus_ProcBroadcastService(CanBus_T * p_can, CanBus_BroadcastService_T * p_service)
// {
//     for (uint8_t i = 0U; i < count; i++)
//     {
//         p_broadcasts[i].P_STATE->Elapsed += dt_us;
//         if (p_broadcasts[i].P_STATE->Elapsed >= p_broadcasts[i].INTERVAL)
//         {
//             CanBus_ProcBroadcast(p_can, &p_broadcasts[i]);
//             p_broadcasts[i].P_STATE->Elapsed = 0U;
//         }
//     }
// }


/******************************************************************************/
/*!
    Inbound dispatch table — route a received frame to the right handler based on COB-ID range.
*/
/******************************************************************************/
// typedef void (*CanBus_RxRequest_T)(void * p_dev, uint32_t id, const uint8_t * p_data);
// direct write data to application buffer
// typedef uint8_t * (*CanBus_RxDataMapper_T)(void * p_dev, uint32_t id);
// typedef void (*CanBus_RxFrame_T)(void * p_dev, const CAN_Frame_T * p_frame);


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
CanBus_RxRoute_T;

// alternative to table
typedef CanBus_RxRoute_T * (*CanBus_RxRequestMapper_T)(void * p_dev, uint32_t id);

static inline CanBus_RxRoute_T * CanBus_SearchRxTable(CanBus_RxRoute_T * p_routes, uint8_t count, uint32_t id)
{
    for (uint8_t i = 0U; i < count; i++)
    {
        if ((id & p_routes[i].ID_MASK) == p_routes[i].ID_MATCH) { return &p_routes[i]; }
    }
    return NULL;
}

// replace or redirect the default REQ_CALLBACK handler
// static inline void CanBus_ProcRx(CanBus_T * p_can, CanBus_ReqTable_T * p_)
// {
// }

/*
    CanBus_ReqHandler_T
    application layer handles request routing, or a single handler for all requests.
*/
typedef void (*CanBus_ReqHandler_T)(void * p_dev, const CAN_Frame_T * p_rxFrame, CAN_Frame_T * p_txFrame);
// typedef void (*CanBus_ReqHandler_T)(void * p_dev, void * adapter, const CAN_Frame_T * p_rxFrame, CAN_Frame_T * p_txFrame);

typedef struct CanBus_SocketConfig
{
    //  is enabled / serivce active/ resolve to empty
    bool IsEnabled;
}
CanBus_SocketConfig_T;

// static inline void CanBus_MapReqService(CanBus_T * p_can, CanBus_ReqHandler_T handler)
// {
//     p_can->P_STATE.RxCallback = handler;
// }

typedef const struct CanBus_Service
{
    CanBus_BroadcastEntry_T * P_BROADCASTS;  uint8_t BROADCAST_COUNT;
    CanBus_RxRoute_T * P_ROUTES;             uint8_t ROUTE_COUNT;
    // CanBus_RxRequestMapper_T REQ_MAPPER;
    CanBus_ReqHandler_T REQ_HANDLER;
    // const volatile uint32_t * P_TIMER;
}
CanBus_Service_T;


// poll rx buffer, or call form isr
static inline void CanBus_Service_ProcRx(CanBus_T * p_can, void *p_appContext, CanBus_Service_T * p_service, const CAN_Frame_T * p_rxFrame)
{
    CAN_Frame_T txFrame = { 0U };
    CanBus_RxRoute_T * p_route = CanBus_SearchRxTable(p_service->P_ROUTES, p_service->ROUTE_COUNT, p_rxFrame->CanId.Id);
    if (p_route != NULL)
    {
        p_route->HANDLER(p_appContext, p_rxFrame, &txFrame);
        if (txFrame.DataLength > 0U) { HAL_CAN_WriteTxMessage(p_can->P_HAL, &txFrame); }
    }
}

/*
    Outer wrap
*/
typedef const struct CanBus_Socket
{
    CanBus_T CAN;
    CanBus_Service_T * P_SERVICE;
    // CanBus_BroadcastEntry_T * P_BROADCASTS;  uint8_t BROADCAST_COUNT;
    // CanBus_RxRoute_T * P_ROUTES;             uint8_t ROUTE_COUNT;
    void * p_APP_CONTEXT; // or from  canbus
    // const volatile uint32_t * P_TIMER;
}
CanBus_Socket_T;


// static inline void CanBus_Socket_ProcBroadcast(CanBus_Socket_T * p_socket)
// {
//     CanBus_ProcBroadcast(&p_socket->CAN, p_socket->P_SERVICE->P_BROADCASTS);
// }

static inline void CanBus_Socket_ProcBroadcasts(CanBus_Socket_T * p_skt, uint32_t dt_ms)
{
    CanBus_Service_T * s = p_skt->P_SERVICE;
    for (uint8_t i = 0U; i < s->BROADCAST_COUNT; i++)
    {
        CanBus_BroadcastEntry_T * e = &s->P_BROADCASTS[i];
        e->P_STATE->Elapsed += dt_ms;
        if (e->P_STATE->Elapsed >= e->INTERVAL) { CanBus_ProcBroadcast(&p_skt->CAN, e); e->P_STATE->Elapsed = 0U; }
    }
}

static inline void CanBus_Socket_EnableRxReq(CanBus_Socket_T * p_socket)
{
    // p_socket->CAN.P_STATE->ServiceHandler = CanBus_Service_ProcRx;
    // p_socket->CAN.P_STATE->Service = p_socket->P_SERVICE;
}

/*
    directly on Socket rather than CanBus_T, so that the service can be selected per socket.
*/
static inline void CanBus_Socket_RxISR(CanBus_Socket_T * p_skt)
{
    // CAN_Frame_T * p_rx = CanBus_PollRx(&p_skt->CAN);   /* full frame or NULL */
    // if (p_rx == NULL) { return; }
    // CanBus_RouteHandler_T handler = (p_skt->P_SERVICE->P_ROUTES != NULL) ? CanBus_SearchRxTable(p_skt->P_SERVICE->P_ROUTES, p_skt->P_SERVICE->ROUTE_COUNT, p_rx->CanId.Id) : NULL;
    // if (handler == NULL) { return; }
    // CAN_Frame_T tx = { 0 };
    // handler(p_skt->CAN.P_CONTEXT, p_rx, &tx);          /* app from device */
    // if (tx.DataLength > 0U) { HAL_CAN_WriteTxMessage(p_skt->CAN.P_HAL, &tx); }
}