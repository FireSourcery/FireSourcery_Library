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
    @file   CAN.h
    @author FireSourcery
    @brief  CAN Bus driver — manages message buffers, ISR dispatch, periodic services
*/
/******************************************************************************/
#ifndef CAN_H
#define CAN_H

#include "HAL_CAN.h"

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*!
    Wrap additional runtime state and interface to Services structs.

    CAN layer start at higher level, include services layer.
    since register level hardware already handles transport
*/
/******************************************************************************/
/*
    hold the optional service handler
    alternatively,
    _CAN_ProcRequestService(CAN_T * p_can, CAN_Service_T * p_service, void *p_appContext, const CAN_Frame_T * p_rxFrame)
*/
struct CAN_Service;
typedef const struct CAN_Service CAN_Service_T;

/******************************************************************************/
/*! Message Buffer */
/******************************************************************************/
typedef enum
{
    // CAN_BUFFER_DISABLED,
    CAN_BUFFER_IDLE,
    CAN_BUFFER_RX_WAIT_DATA,       /* Rx MB armed, waiting for data frame */
    CAN_BUFFER_RX_WAIT_REMOTE,     /* Rx MB armed, waiting for remote response */
    CAN_BUFFER_RX_WAIT_SERVICE,    /* Rx complete, awaiting application processing */
    CAN_BUFFER_TX_DATA,            /* Tx data frame pending */
    CAN_BUFFER_TX_REMOTE,          /* Tx remote request pending */
}
CAN_BufferState_T;

typedef struct
{
    CAN_Frame_T Frame;
    // CAN_Frame_T TxBuffer;
    CAN_BufferState_T State;
    uint32_t TimeStamp;
    uint32_t HwIndex;
}
CAN_Buffer_T;

/******************************************************************************/
/*! Rx Callbacks */
/******************************************************************************/
// typedef void (*CAN_RxRequest_T)(void * p_dev, uint32_t id, const uint8_t * p_data); //, uint32_t length);
/*
    Full-frame Rx callback — preserves DLC, RTR, and ID metadata.
*/
// typedef void (*CAN_RxFrame_T)(void * p_dev, const CAN_Frame_T * p_frame);


/******************************************************************************/
/*! Runtime state */
/******************************************************************************/
#ifndef CAN_MESSAGE_BUFFER_COUNT
#define CAN_MESSAGE_BUFFER_COUNT 1U
#endif

typedef struct
{
    // CAN_Buffer_T ActiveChannel;
    CAN_Buffer_T Channel[CAN_MESSAGE_BUFFER_COUNT];
    CAN_Service_T * p_Service; /*  */
}
CAN_State_T;

// typedef struct CAN_SocketConfig
// {
//     //  is enabled / serivce active/ resolve to empty
//     bool IsEnabled;
// }
// CAN_Config_T;

/******************************************************************************/
/*!
    CAN instance — const config + mutable state pointer
*/
/******************************************************************************/
typedef const struct CAN
{
    HAL_CAN_T * P_HAL;
    CAN_State_T * P_STATE;
    void * P_CONTEXT;
    CAN_Service_T * P_SERVICE; /* default */
    CAN_Service_T * P_SERVICE_TABLE; /* Protocol selection */
    uint8_t SERVICE_COUNT;
    // CAN_RxRequest_T REQ_CALLBACK;
    // const volatile uint32_t * P_TIMER;
}
CAN_T;


/******************************************************************************/
/*! Call Poll status */
/******************************************************************************/
/* return null for empty buffer */
static inline bool CAN_PollRx(CAN_T * p_can)
{
    CAN_Frame_T * p_buffer = &p_can->P_STATE->Channel[0U].Frame;
    return HAL_CAN_PollRxMessage(p_can->P_HAL, p_buffer);
}

/******************************************************************************/
/*! ISR handlers */
/******************************************************************************/
/*
    buffer rx proc request on polling
    without checking Remote
    with callback optionally mapping to table
*/
static inline void CAN_RxData_ISR(CAN_T * p_can)
{
    (void)CAN_PollRx(p_can);
    // CAN_BUFFER_RX_WAIT_SERVICE
    // CAN_Frame_T txBuffer = { 0U };
    // if (p_rx != NULL)
    // {
    //     p_can->REQ_CALLBACK(p_can->P_CONTEXT, p_rx->CanId.Id, &p_rx->Data[0U]);
    // }
}


/******************************************************************************/
/*
    Stateful
*/
/******************************************************************************/
/*
    Rx
    Rx data frame received
    Rx remote frame received
*/
// static inline void _CAN_Rx_ISR(CAN_T * p_can, uint8_t bufferId, uint8_t hwIndex)
// {
//     CAN_Buffer_T * p_buf = &p_can->P_STATE->Channel[bufferId];

//     if (HAL_CAN_LockRx(p_can->P_HAL, hwIndex))
//     {
//         HAL_CAN_ReadRxMessage(p_can->P_HAL, &p_buf->Frame);
//         // HAL_CAN_ClearRxInterrupt(p_can->P_HAL, hwIndex);
//         HAL_CAN_ClearRxFullFlag(p_can->P_HAL);
//         HAL_CAN_UnlockRx(p_can->P_HAL, hwIndex);
//     }

//     switch (p_buf->State)
//     {
//         case CAN_BUFFER_RX_WAIT_REMOTE:
//             p_buf->State = CAN_BUFFER_RX_WAIT_SERVICE;
//             break;

//         case CAN_BUFFER_RX_WAIT_DATA:
//             p_buf->State = CAN_BUFFER_RX_WAIT_SERVICE;
//             break;

//         case CAN_BUFFER_RX_WAIT_SERVICE:
//             /* over run or write to ring buffer */
//             break;
//         case CAN_BUFFER_IDLE:
//             // HAL_CAN_ClearRxInterrupt(p_can->P_HAL, hwIndex);
//             break;

//         default:
//             p_buf->State = CAN_BUFFER_IDLE;
//             break;
//     }

//     // if (p_can->REQ_CALLBACK != NULL)
//     // {
//     //     p_can->REQ_CALLBACK(p_can->P_CONTEXT, p_buf->Frame.CanId.Id, &p_buf->Frame.Data[0U]);
//     //     p_buf->State = CAN_BUFFER_IDLE;
//     // }
// }

// static inline void CAN_Rx_ISR(CAN_T * p_can)
// {
//     if (HAL_CAN_ReadRxFullFlag(p_can->P_HAL))
//     {
//         _CAN_Rx_ISR(p_can, 0U, 0U);
//         HAL_CAN_ClearRxFullFlag(p_can->P_HAL);
//         HAL_CAN_DisableRxFullInterrupt(p_can->P_HAL);
//     }
// }

/*
    Tx
    Tx buffer completes transmission (transmit buffer becomes empty and available)
    Tx data frame sent
    Tx remote frame sent
*/
// static inline void _CAN_Tx_ISR(CAN_T * p_can, uint8_t bufferId, uint8_t hwIndex)
// {
//     CAN_Buffer_T * p_buf = &p_can->P_STATE->Channel[bufferId];

//     switch (p_buf->State)
//     {
//         case CAN_BUFFER_TX_REMOTE:
//             // if (HAL_CAN_ReadTxRemoteRxEmpty(p_can->P_HAL, hwIndex)) /* Tx remote request sent — now wait for data response */
//             {
//                 HAL_CAN_EnableRxFullInterrupt(p_can->P_HAL);
//                 p_buf->State = CAN_BUFFER_RX_WAIT_REMOTE;
//             }
//             break;

//         case CAN_BUFFER_TX_DATA:
//             p_buf->State = CAN_BUFFER_IDLE;
//             break;

//         case CAN_BUFFER_IDLE: break;
//         default: break;
//     }

//     HAL_CAN_ClearTxEmptyFlag(p_can->P_HAL);
//     HAL_CAN_DisableTxEmptyInterrupt(p_can->P_HAL);

// }

// static inline void CAN_Tx_ISR(CAN_T * p_can)
// {
//     _CAN_Tx_ISR(p_can, 0U, 0U);
// }




/*
    Shared Tx/Rx ISR — iterates buffers to find which triggered the interrupt,
    then dispatches based on buffer state.
*/
// static inline void CAN_TxRx_ISR(CAN_T * p_can)
// {
//     uint8_t hwIndex;
//     uint8_t bufferId = 0xFFU;

//     // /* Find which buffer triggered the interrupt */
//     // for (uint8_t i = 0U; i < CAN_MESSAGE_BUFFER_COUNT; i++)
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

//     CAN_Buffer_T * p_buf = &p_can->P_STATE->Channel[bufferId];

//     switch (p_buf->State)
//     {
//         case CAN_BUFFER_RX_WAIT_DATA:
//             if (HAL_CAN_LockRx(p_can->P_HAL, hwIndex))
//             {
//                 HAL_CAN_ReadRxMessage(p_can->P_HAL, hwIndex, &p_buf->Frame);
//                 HAL_CAN_ClearRxInterrupt(p_can->P_HAL, hwIndex);
//                 HAL_CAN_UnlockRx(p_can->P_HAL, hwIndex);
//                 p_buf->State = CAN_BUFFER_RX_WAIT_SERVICE;
//             }
//             break;

//         case CAN_BUFFER_TX_REMOTE:
//             if (HAL_CAN_ReadTxRemoteRxFull(p_can->P_HAL, hwIndex))
//             {
//                 /* Remote response received — read and transition to service */
//                 if (HAL_CAN_LockRx(p_can->P_HAL, hwIndex))
//                 {
//                     HAL_CAN_ReadRxMessage(p_can->P_HAL, hwIndex, &p_buf->Frame);
//                     HAL_CAN_ClearRxInterrupt(p_can->P_HAL, hwIndex);
//                     HAL_CAN_UnlockRx(p_can->P_HAL, hwIndex);
//                     p_buf->State = CAN_BUFFER_RX_WAIT_SERVICE;
//                 }
//             }
//             else if (HAL_CAN_ReadTxRemoteRxEmpty(p_can->P_HAL, hwIndex))
//             {
//                 /* Tx remote request sent — now wait for data response */
//                 HAL_CAN_ClearTxInterrupt(p_can->P_HAL, hwIndex);
//                 p_buf->State = CAN_BUFFER_RX_WAIT_DATA;
//             }
//             break;

//         case CAN_BUFFER_RX_WAIT_REMOTE:
//             break;

//         default: /* IDLE, TX_DATA complete, or unexpected */
//             HAL_CAN_ClearTxInterrupt(p_can->P_HAL, hwIndex);
//             HAL_CAN_DisableTxInterrupt(p_can->P_HAL, hwIndex);
//             p_buf->State = CAN_BUFFER_IDLE;
//             break;
//     }
// }

/******************************************************************************/
/*! Public API */
/******************************************************************************/
extern void CAN_Init(CAN_T * p_can);
extern void CAN_InitBaudRate(CAN_T * p_can, uint32_t bitRate);

/* Tx — polling, no interrupt, fire-and-forget */
// extern void CAN_TxData(CAN_T * p_can, can_id_t id, const uint8_t * p_txData, size_t length);
/* Tx — remote frame request. sets buffer state to RX_WAIT_REMOTE for the response. */
// extern void CAN_TxRemote(CAN_T * p_can, can_id_t id,  const uint8_t * p_txData,  size_t length);
/* Tx — accept a fully-built frame. */
// extern void CAN_Tx(CAN_T * p_can, CAN_Frame_T * p_frame);




#endif
