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
    @file   CanBus.h
    @author FireSourcery
    @brief  CAN Bus driver — manages message buffers, ISR dispatch, periodic services
*/
/******************************************************************************/
#ifndef CAN_BUS_H
#define CAN_BUS_H

#include "HAL_CAN.h"

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*! Message Buffer */
/******************************************************************************/
typedef enum
{
    CAN_BUS_BUFFER_IDLE,
    CAN_BUS_BUFFER_RX_WAIT_DATA,       /* Rx MB armed, waiting for data frame */
    CAN_BUS_BUFFER_RX_WAIT_REMOTE,     /* Rx MB armed, waiting for remote response */
    CAN_BUS_BUFFER_RX_WAIT_SERVICE,    /* Rx complete, awaiting application processing */
    CAN_BUS_BUFFER_TX_DATA,            /* Tx data frame pending */
    CAN_BUS_BUFFER_TX_REMOTE,          /* Tx remote request pending */
}
CanBus_BufferState_T;

typedef struct
{
    CAN_Frame_T Frame;
    // CAN_Frame_T TxBuffer;
    CanBus_BufferState_T State;
    uint32_t BroadcastTime;
    uint32_t HwIndex;
}
CanBus_Buffer_T;

/******************************************************************************/
/*! Rx Callbacks */
/******************************************************************************/
typedef void (*CanBus_RxRequest_T)(void * p_dev, uint32_t id, const uint8_t * p_data);
// typedef void (*CanBus_RxRequest_T)(void * p_dev, uint32_t id, const uint8_t * p_data, uint32_t length);

/*
    Full-frame Rx callback — preserves DLC, RTR, and ID metadata.
*/
typedef void (*CanBus_RxFrame_T)(void * p_dev, const CAN_Frame_T * p_frame);

/******************************************************************************/
/*! Runtime state */
/******************************************************************************/
#ifndef CAN_BUS_MESSAGE_BUFFER_COUNT
#define CAN_BUS_MESSAGE_BUFFER_COUNT 1U
#endif

typedef struct
{
    CanBus_Buffer_T Channel[1U];
}
CanBus_State_T;

/******************************************************************************/
/*! CanBus instance — const config + mutable state pointer */
/******************************************************************************/
typedef const struct CanBus
{
    HAL_CAN_T * P_HAL;
    CanBus_State_T * P_STATE;
    void * P_CONTEXT;
    CanBus_RxRequest_T REQ_CALLBACK;
    // const volatile uint32_t * P_TIMER;
}
CanBus_T;

#define CAN_BUS_INIT(p_Hal, p_State, p_App, Callback)  \
{                                                                                  \
    .P_HAL = p_Hal,                                                                \
    .P_STATE = p_State,                                                            \
    .P_CONTEXT = p_App,                                                            \
    .REQ_CALLBACK = Callback,                                                \
}




/******************************************************************************/
/*! ISR handlers */
/******************************************************************************/
static inline bool CanBus_PollRxData(CanBus_T * p_can)
{
    CanBus_Buffer_T * p_buf = &p_can->P_STATE->Channel[0U];
    bool hasData = HAL_CAN_ReadRxFullFlag(p_can->P_HAL);
    if (hasData)
    {
        HAL_CAN_ReadRxMessage(p_can->P_HAL, &p_buf->Frame);
        if (p_can->REQ_CALLBACK != NULL) { p_can->REQ_CALLBACK(p_can->P_CONTEXT, p_buf->Frame.CanId.Id, &p_buf->Frame.Data[0U]); }
    }
    return hasData;
}

/*
    without checking Remote
*/
static inline void CanBus_RxData_ISR(CanBus_T * p_can)
{
    CanBus_Buffer_T * p_buf = &p_can->P_STATE->Channel[0U];
    if (HAL_CAN_ReadRxFullFlag(p_can->P_HAL))
    {
        HAL_CAN_ReadRxMessage(p_can->P_HAL, &p_buf->Frame);

        if (p_can->REQ_CALLBACK != NULL)
        {
            p_can->REQ_CALLBACK(p_can->P_CONTEXT, p_buf->Frame.CanId.Id, &p_buf->Frame.Data[0U]);
        }
        else
        {
            p_buf->State = CAN_BUS_BUFFER_RX_WAIT_SERVICE;
            HAL_CAN_DisableRxFullInterrupt(p_can->P_HAL);
        }

        HAL_CAN_ClearRxFullFlag(p_can->P_HAL);
    }
}

/*
    Directly map to user buffer
*/
// typedef uint8_t * (*CanBus_RxDataMapper_T)(void * p_dev, uint32_t id);

// static inline bool _CanBus_PollRxData(CanBus_T * p_can, CanBus_RxDataMapper_T mapper)
// {
//     bool hasData = HAL_CAN_ReadRxFullFlag(p_can->P_HAL);
//     if (hasData) { HAL_CAN_ReadRxData(p_can->P_HAL, mapper(p_can->P_CONTEXT, HAL_CAN_ReadRxId(p_can->P_HAL).Id)); }
//     return hasData;
// }

/*
    Stateful
*/
/*
    Rx
    Rx data frame received
    Rx remote frame received
*/
static inline void _CanBus_Rx_ISR(CanBus_T * p_can, uint8_t bufferId, uint8_t hwIndex)
{
    CanBus_Buffer_T * p_buf = &p_can->P_STATE->Channel[bufferId];

    if (HAL_CAN_LockRx(p_can->P_HAL, hwIndex))
    {
        // CanBus_ReceiveData(p_can, (can_id_t *)&p_buf->Frame.CanId, &p_buf->Frame.Data[0U], 8U);
        HAL_CAN_ReadRxMessage(p_can->P_HAL, &p_buf->Frame);
        // HAL_CAN_ClearRxInterrupt(p_can->P_HAL, hwIndex);
        HAL_CAN_ClearRxFullFlag(p_can->P_HAL);
        HAL_CAN_UnlockRx(p_can->P_HAL, hwIndex);
    }

    switch (p_buf->State)
    {
        case CAN_BUS_BUFFER_RX_WAIT_REMOTE:
            p_buf->State = CAN_BUS_BUFFER_RX_WAIT_SERVICE;
            break;

        case CAN_BUS_BUFFER_RX_WAIT_DATA:
            p_buf->State = CAN_BUS_BUFFER_RX_WAIT_SERVICE;
            break;

        case CAN_BUS_BUFFER_RX_WAIT_SERVICE:
            /* over run or write to ring buffer */
            break;
        case CAN_BUS_BUFFER_IDLE:
            // HAL_CAN_ClearRxInterrupt(p_can->P_HAL, hwIndex);
            break;

        default:
            p_buf->State = CAN_BUS_BUFFER_IDLE;
            break;
    }

    if (p_can->REQ_CALLBACK != NULL)
    {
        p_can->REQ_CALLBACK(p_can->P_CONTEXT, p_buf->Frame.CanId.Id, &p_buf->Frame.Data[0U]);
        p_buf->State = CAN_BUS_BUFFER_IDLE;
    }
}

static inline void CanBus_Rx_ISR(CanBus_T * p_can)
{
    if (HAL_CAN_ReadRxFullFlag(p_can->P_HAL))
    {
        _CanBus_Rx_ISR(p_can, 0U, 0U);
        HAL_CAN_ClearRxFullFlag(p_can->P_HAL);
        HAL_CAN_DisableRxFullInterrupt(p_can->P_HAL);
    }
}

/*
    Tx
    Tx buffer completes transmission (transmit buffer becomes empty and available)
    Tx data frame sent
    Tx remote frame sent
*/
static inline void _CanBus_Tx_ISR(CanBus_T * p_can, uint8_t bufferId, uint8_t hwIndex)
{
    CanBus_Buffer_T * p_buf = &p_can->P_STATE->Channel[bufferId];

    switch (p_buf->State)
    {
        case CAN_BUS_BUFFER_TX_REMOTE:
            // if (HAL_CAN_ReadTxRemoteRxEmpty(p_can->P_HAL, hwIndex)) /* Tx remote request sent — now wait for data response */
            {
                HAL_CAN_EnableRxFullInterrupt(p_can->P_HAL);
                p_buf->State = CAN_BUS_BUFFER_RX_WAIT_REMOTE;
            }
            break;

        case CAN_BUS_BUFFER_TX_DATA:
            p_buf->State = CAN_BUS_BUFFER_IDLE;
            break;

        case CAN_BUS_BUFFER_IDLE: break;
        default: break;
    }

    HAL_CAN_ClearTxEmptyFlag(p_can->P_HAL);
    HAL_CAN_DisableTxEmptyInterrupt(p_can->P_HAL);

}

static inline void CanBus_Tx_ISR(CanBus_T * p_can)
{
    _CanBus_Tx_ISR(p_can, 0U, 0U);
}




/*
    Shared Tx/Rx ISR — iterates buffers to find which triggered the interrupt,
    then dispatches based on buffer state.
*/
// static inline void CanBus_TxRx_ISR(CanBus_T * p_can)
// {
//     uint8_t hwIndex;
//     uint8_t bufferId = 0xFFU;

//     // /* Find which buffer triggered the interrupt */
//     // for (uint8_t i = 0U; i < CAN_BUS_MESSAGE_BUFFER_COUNT; i++)
//     // {
//     //     hwIndex = HAL_CAN_MapMessageBufferIndex(p_can->P_HAL, i);
//     //     if (HAL_CAN_ReadRxComplete(p_can->P_HAL, hwIndex))
//     //     {
//     //         bufferId = i;
//     //         break;
//     //     }
//     //     if (HAL_CAN_ReadTxComplete(p_can->P_HAL, hwIndex))
//     //     {
//     //         bufferId = i;
//     //         break;
//     //     }
//     // }

//     // if (bufferId == 0xFFU) { return; } /* Spurious interrupt */

//     CanBus_Buffer_T * p_buf = &p_can->P_STATE->Channel[bufferId];

//     switch (p_buf->State)
//     {
//         case CAN_BUS_BUFFER_RX_WAIT_DATA:
//             if (HAL_CAN_LockRx(p_can->P_HAL, hwIndex))
//             {
//                 HAL_CAN_ReadRxMessage(p_can->P_HAL, hwIndex, &p_buf->Frame);
//                 HAL_CAN_ClearRxInterrupt(p_can->P_HAL, hwIndex);
//                 HAL_CAN_UnlockRx(p_can->P_HAL, hwIndex);
//                 p_buf->State = CAN_BUS_BUFFER_RX_WAIT_SERVICE;
//             }
//             break;

//         case CAN_BUS_BUFFER_TX_REMOTE:
//             if (HAL_CAN_ReadTxRemoteRxFull(p_can->P_HAL, hwIndex))
//             {
//                 /* Remote response received — read and transition to service */
//                 if (HAL_CAN_LockRx(p_can->P_HAL, hwIndex))
//                 {
//                     HAL_CAN_ReadRxMessage(p_can->P_HAL, hwIndex, &p_buf->Frame);
//                     HAL_CAN_ClearRxInterrupt(p_can->P_HAL, hwIndex);
//                     HAL_CAN_UnlockRx(p_can->P_HAL, hwIndex);
//                     p_buf->State = CAN_BUS_BUFFER_RX_WAIT_SERVICE;
//                 }
//             }
//             else if (HAL_CAN_ReadTxRemoteRxEmpty(p_can->P_HAL, hwIndex))
//             {
//                 /* Tx remote request sent — now wait for data response */
//                 HAL_CAN_ClearTxInterrupt(p_can->P_HAL, hwIndex);
//                 p_buf->State = CAN_BUS_BUFFER_RX_WAIT_DATA;
//             }
//             break;

//         case CAN_BUS_BUFFER_RX_WAIT_REMOTE:
//             break;

//         default: /* IDLE, TX_DATA complete, or unexpected */
//             HAL_CAN_ClearTxInterrupt(p_can->P_HAL, hwIndex);
//             HAL_CAN_DisableTxInterrupt(p_can->P_HAL, hwIndex);
//             p_buf->State = CAN_BUS_BUFFER_IDLE;
//             break;
//     }
// }

/******************************************************************************/
/*! Public API */
/******************************************************************************/
extern void CanBus_Init(CanBus_T * p_can);
// extern void CanBus_InitServices(CanBus_T * p_can);
// extern void CanBus_ProcServices(CanBus_T * p_can);
extern void CanBus_InitBaudRate(CanBus_T * p_can, uint32_t bitRate);

/* Tx — polling, no interrupt, fire-and-forget */
extern void CanBus_SendData(CanBus_T * p_can, can_id_t id, const uint8_t * p_txData, size_t length);
/* Tx — interrupt-driven, tracks buffer state; sets RX_WAIT_REMOTE on RTR frame */
extern void CanBus_Send(CanBus_T * p_can, can_id_t id, const uint8_t * p_txData, size_t length);
/* Tx — accept a fully-built frame. */
extern void CanBus_SendFrame(CanBus_T * p_can, const CAN_Frame_T * p_frame);

/* Tx — remote frame request. sets buffer state to RX_WAIT_REMOTE for the response. */
extern void CanBus_SendRemote(CanBus_T * p_can, can_id_t id, size_t length);

/* Rx — polling */
extern size_t CanBus_ReceiveData(CanBus_T * p_can, can_id_t * p_rxId, uint8_t * p_rxData, size_t length);
/* Arms Rx buffer and enables interrupt */
extern void CanBus_StartReceive(CanBus_T * p_can, can_id_t rxId);

size_t CanBus_ReceiveRequest(CanBus_T * p_can, uint32_t * p_rxId, uint8_t * p_rxData, size_t length);

#endif
