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
    @file   CAN_Service.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "CAN.h"


/******************************************************************************/
/*! Service callbacks */
/******************************************************************************/
typedef void (*CAN_BuildBroadcast_T)(void * p_context, CAN_Frame_T * p_frame);

// keep for interface
typedef struct
{
    // uint32_t Elapsed; /* Millis or Micros */
    uint32_t Timestamp; /* last broadcast */
    bool Enabled; /* optionally individual disable active */
}
CAN_BroadcastState_T;

typedef const struct
{
    uint32_t ID; /*   */
    CAN_BuildBroadcast_T BUILD;  /* Frame-based broadcast — caller fills a full CAN_Frame_T (ID, DLC, data). */
    uint32_t INTERVAL;
    CAN_BroadcastState_T * P_STATE;  /* allocate per entry, alternatively collective handle */
    // CAN_ServiceInit_T INIT;
}
CAN_BroadcastEntry_T;

// typedef const struct CAN_BroadcastService
// {
//     CAN_BroadcastEntry_T * P_BROADCASTS;
//     uint8_t BROADCAST_COUNT;
//     const volatile uint32_t * P_TIMER;
//     CAN_BroadcastState_T * P_STATES; /* Parallel array of broadcast states */
// }
// CAN_BroadcastService_T;

/*
    Disabled
*/
// static void  CAN_BuildEmpty(void * p_context, CAN_Frame_T * p_frame) { (void)p_context; (void)p_frame; }
// static const CAN_BroadcastEntry_T CAN_BROADCAST_EMPTY = { .BUILD = CAN_BuildEmpty, .ID = 0U, .INTERVAL = 0U, .P_STATE = NULL };

/* App context is the driver's P_CONTEXT (single source). */
/* Frame-form: callee fills ID, DLC, data (e.g. CiA402 TxPDO) */
static inline void CAN_ProcBroadcast(CAN_T * p_can, CAN_BroadcastEntry_T * p_broadcast)
{
    CAN_Frame_T frame = { 0U };
    frame.CanId.Id32  = p_broadcast->ID; /* seed default ID; frame builders (e.g. CiA402) may override */
    p_broadcast->BUILD(p_can->P_CONTEXT, &frame);
    if (frame.DataLength > 0U) { HAL_CAN_WriteTxMessage(p_can->P_HAL, &frame); } /* empty = nothing due, as the request path */
}

static inline void _CAN_ProcBroadcastService(CAN_T * p_can, CAN_BroadcastEntry_T * p_table, uint8_t count, uint32_t timer)
{
    for (uint8_t i = 0U; i < count; i++)
    {
        if ((timer - p_table[i].P_STATE->Timestamp) >= p_table[i].INTERVAL)
        {
            CAN_ProcBroadcast(p_can, &p_table[i]);
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
// typedef void (*CAN_ReqHandler_T)(void * p_dev, void * adapter, const void * p_rx, void * p_tx);
typedef void (*CAN_RouteHandler_T)(void * p_dev, const CAN_Frame_T * p_rx, CAN_Frame_T * p_tx);

typedef const struct
{
    uint32_t ID_MATCH;     /* expected (id & ID_MASK) */
    uint32_t ID_MASK;
    CAN_RouteHandler_T HANDLER;
}
CAN_ReqRoute_T;

// typedef const struct CAN_ReqService
// {
//     CAN_ReqRoute_T * P_ROUTES;
//     uint8_t ROUTE_COUNT;
//     // void * P_CONTEXT;
// }
// CAN_ReqService_T;

// alternative to table search
// typedef CAN_ReqRoute_T * (*CAN_RxRequestMapper_T)(void * p_dev, uint32_t id);
static inline CAN_ReqRoute_T * CAN_SearchRxTable(CAN_ReqRoute_T * p_routes, uint8_t count, uint32_t id)
{
    for (uint8_t i = 0U; i < count; i++) { if ((id & p_routes[i].ID_MASK) == p_routes[i].ID_MATCH) { return &p_routes[i]; } }
    return NULL;
}

// static inline void _CAN_ProcRequestService(CAN_T * p_can, CAN_ReqRoute_T * p_route, void * p_context)
// {
//     CAN_Frame_T txFrame = { 0U };
//     if (p_found != NULL) { p_found->HANDLER(p_context, p_rxFrame, &txFrame); }
//     if (txFrame.DataLength > 0U) { HAL_CAN_WriteTxMessage(p_can->P_HAL, &txFrame); }
// }

static inline void _CAN_ProcRequestService(CAN_T * p_can, CAN_ReqRoute_T * p_table, uint8_t count, const CAN_Frame_T * p_rxFrame)
{
    CAN_Frame_T txFrame = { 0U };
    CAN_ReqRoute_T * p_route = CAN_SearchRxTable(p_table, count, p_rxFrame->CanId.Id);
    if (p_route != NULL) { p_route->HANDLER(p_can->P_CONTEXT, p_rxFrame, &txFrame); }
    if (txFrame.DataLength > 0U) { HAL_CAN_WriteTxMessage(p_can->P_HAL, &txFrame); }
}


/*
 */


/* unit of selection, alternatively CAN holds separate tables */
typedef const struct CAN_Service
{
    CAN_BroadcastEntry_T * P_BROADCASTS;  uint8_t BROADCAST_COUNT;
    CAN_ReqRoute_T * P_ROUTES; uint8_t ROUTE_COUNT;
    // const volatile uint32_t * P_TIMER;
    // CAN_BroadcastState_T * P_STATES; /* Parallel array of broadcast states */
}
CAN_Service_T;

// CAN_Service_T CAN_SERVICE_EMPTY = { .P_BROADCASTS = NULL, .BROADCAST_COUNT = 0U, .P_ROUTES = NULL, .ROUTE_COUNT = 0U };

/*
    proc buffered frame, without isr priority
    poll rx buffer, or call form isr
    Dispatch one inbound frame: route-table match first, else the service-wide REQ_HANDLER.
    Handler fills txFrame (ID/DLC/data); a non-zero DataLength is transmitted as the reply.
*/
static inline void CAN_ProcRequestService(CAN_T * p_can)
{
    CAN_Frame_T * p_rxFrame = &p_can->P_STATE->Channel[0U].Frame;
    CAN_Service_T * p_service = p_can->P_STATE->p_Service;
    if (p_service == NULL) { return; } /* disabled — no active protocol */

    _CAN_ProcRequestService(p_can, p_service->P_ROUTES, p_service->ROUTE_COUNT, p_rxFrame);
}

/*
    Proc in ISR context: poll rx buffer, if full, dispatch to service handler.
*/
static inline void CAN_RxService_ISR(CAN_T * p_can)
{
    if (CAN_PollRx(p_can)) { CAN_ProcRequestService(p_can); }
}

// static inline void CAN_PollService(CAN_T * p_can)
// {
//     CAN_Frame_T * p_rxFrame = &p_can->P_STATE->Channel[0U].Frame;
//     if (p_rxFrame->DataLength > 0U) { CAN_ProcRequestService(p_can); }
// }

static inline void CAN_ProcBroadcastService(CAN_T * p_can, uint32_t timer)
{
    CAN_Service_T * p_service = p_can->P_STATE->p_Service;
    if (p_service == NULL) { return; } /* disabled — no active protocol */
    _CAN_ProcBroadcastService(p_can, p_service->P_BROADCASTS, p_service->BROADCAST_COUNT, timer);
}



static inline void CAN_Enable(CAN_T * p_can, CAN_Service_T * p_service) { p_can->P_STATE->p_Service = p_service; }
// todo def empty to eliminate nullcheck
static inline void CAN_Disable(CAN_T * p_can) { p_can->P_STATE->p_Service = NULL; }
static inline void CAN_DisableService(CAN_T * p_can)
{
    // p_can->P_STATE->ServiceHandler = CAN_ProcServiceDisabled;
}

/*
    TODO: on swap also reprogram HW acceptance filters from the new route table and rephase
        broadcast timestamps to avoid a startup burst on the newly selected service.
*/
static inline void CAN_SetService(CAN_T * p_can, uint8_t index)
{
    if (index < p_can->SERVICE_COUNT) { CAN_Enable(p_can, &p_can->P_SERVICE_TABLE[index]); }
}


static inline void CAN_TxService_ISR(CAN_T * p_can)
{
    (void)p_can;
}