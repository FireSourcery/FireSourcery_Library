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




// typedef void (*CanBus_ReqFunction_T)(void * p_dev, const uint8_t * p_rxData);
// // typedef void (*CanBus_ReqRespFunction_T)(void * p_dev, uint8_t * p_txData); /* remote only */
// // typedef void (*CanBus_ServiceInit_T)(CanBus_Buffer_T * p_buffers);

// typedef const struct
// {
//     CanBus_ReqFunction_T REQUEST;
//     // CanBus_ServiceInit_T INIT;
// }
// CanBus_Request_T;



// typedef CanBus_ReqFunction_T(*CanBus_RxRequestMapper_T)(void * p_dev, uint32_t id);


// typedef const struct CanBus_Service
// {
//     CanBus_T CAN;
//     CanBus_Broadcast_T * P_BROADCASTS;
//     uint8_t BROADCASTS_COUNT;
//     uint32_t * P_TIMER;

//     CanBus_RxRequestMapper_T REQUEST_MAPPER;
// }
// CanBus_Service_T;

// /* wire up callback  */
// // static inline void _CanBus_Service_RxReqCallback(CanBus_Service_T * p_service, uint32_t id, const uint8_t * p_data)
// // {
// //     p_service->REQUEST_MAPPER(p_service->CAN.P_CONTEXT, id)(p_service->CAN.P_CONTEXT, p_data);
// // }

// /*
//     Periodic service processing — called from main loop (1ms thread).
//     Handles broadcast scheduling, Rx service dispatch, and Tx submission.
// */
// static inline void CanBus_Service_ProcBroadcasts(CanBus_Service_T * p_can)
// {
//     uint8_t data[8U];
//     /* Periodic broadcast */
//     for (uint8_t i = 0U; i < p_can->BROADCASTS_COUNT; i++)
//     {
//         if ((*p_can->P_TIMER - p_can->CAN.P_STATE->Channel[0].BroadcastTime) >= p_can->P_BROADCASTS[i].INTERVAL)
//         {
//             p_can->P_BROADCASTS[i].BROADCAST(p_can->CAN.P_CONTEXT, &data[0U]);
//             p_can->CAN.P_STATE->Channel[0].BroadcastTime = *p_can->P_TIMER;
//         }
//     }
// }


// // static inline void CanBus_Service_PollRxReqs(CanBus_Service_T * p_can)
// // {
// //     CanBus_PollRxData(&p_can->CAN);

// //     // for (uint8_t i = 0U; i < CAN_BUS_MESSAGE_BUFFER_COUNT; i++)
// //     // {
// //     //     CanBus_Buffer_T * p_buf = &p_can->P_STATE->Buffers[i];

// //     //     if (p_buf->State == CAN_BUS_BUFFER_RX_WAIT_SERVICE)
// //     //     {
// //     //         if (HAL_CAN_ReadRxFullFlag(p_can->P_HAL))
// //     //         {
// //     //             *p_rxId = HAL_CAN_ReadRxId(p_can->P_HAL);
// //     //             return HAL_CAN_ReadRxData(p_can->P_HAL, p_rxData);
// //     //         }
// //     //     }
// //     // }
// // }