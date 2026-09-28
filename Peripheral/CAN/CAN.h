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
    CAN layer start at the socket level / services layer.
    Register level hardware already handles transport, frame state minus RTR
*/
/******************************************************************************/
/*
    hold the optional service handler
    alternatively,
    _CAN_ProcRequestService(CAN_T * p_can, CAN_Service_T * p_service, void *p_appContext, const CAN_Frame_T * p_rxFrame)
*/
struct CAN_Service;
typedef const struct CAN_Service CAN_Service_T;
struct CAN_ServiceState;

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
    CAN_BufferState_T State;
    uint32_t TimeStamp;
}
CAN_Buffer_T;

/******************************************************************************/
/*! Rx Callbacks */
/******************************************************************************/
/*
    Full-frame Rx callback — preserves DLC, RTR, and ID metadata.
*/
typedef void (*CAN_RxHandler_T)(void * p_context, const CAN_Frame_T * p_frame);
typedef void (*CAN_TxHandler_T)(void * p_context, CAN_Frame_T * p_frame);


/******************************************************************************/
/*! Runtime state */
/******************************************************************************/
#ifndef CAN_MESSAGE_BUFFER_COUNT
#define CAN_MESSAGE_BUFFER_COUNT 1U
#endif

/*
    Hardware Rx acceptance filter.
    Id.Eff selects a standard or extended filter; Mask uses 1 = bit must match — the same
    sense as CAN_Request_T.ID_MASK, so a route's (ID_MATCH, ID_MASK) pair is a valid filter.
*/
typedef struct CAN_RxFilter { can_id_t Id; uint32_t Mask; } CAN_RxFilter_T;

#ifndef CAN_RX_FILTER_COUNT
#define CAN_RX_FILTER_COUNT HAL_CAN_RX_FILTER_COUNT
#endif

typedef struct CAN_Config
{
    bool IsEnabled;
    uint8_t RxFilterCount;                          /* 0 = accept all */
    CAN_RxFilter_T RxFilters[CAN_RX_FILTER_COUNT];
    // bool IsExtendedId; /* true = use extended CAN IDs */
}
CAN_Config_T;

typedef struct
{
    // CAN_Buffer_T ActiveChannel;
    CAN_Buffer_T Channel[CAN_MESSAGE_BUFFER_COUNT];
    CAN_Config_T Config; /* configuration for this CAN instance */
}
CAN_State_T;

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
    struct CAN_ServiceState * P_SERVICE_STATE;  /* selection + broadcast timing, CAN_SERVICE_STATE_ALLOC() */
    CAN_Service_T * P_SERVICE;                  /* default */
    CAN_Service_T * const * P_SERVICE_TABLE;    /* selectable protocols. Array of pointers - defined by separate modules */
    uint8_t SERVICE_COUNT;
    const volatile uint32_t * P_TIMER;
    const CAN_Config_T * P_NVM_CONFIG; /* config source, copied into P_STATE->Config at init */
    // CAN_RxRequest_T REQ_CALLBACK;
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
/*! Public API */
/******************************************************************************/
extern void CAN_Init(CAN_T * p_can);
extern void CAN_InitBaudRate(CAN_T * p_can, uint32_t bitRate);
extern void CAN_SetRxFilters(CAN_T * p_can, const CAN_RxFilter_T * p_filters, uint8_t count);

/******************************************************************************/
/*!
    Var Id interface — Config fields.
    Writes land in P_STATE->Config and take effect on the next CAN_Init, as with the other
    config vars; persist through the NVM map. Use CAN_SetRxFilters for an immediate change.
*/
/******************************************************************************/
typedef enum CAN_ConfigId
{
    CAN_CONFIG_IS_ENABLED,              /* service active on init */
    CAN_CONFIG_RX_FILTER_COUNT,         /* 0 = accept all; clamped to CAN_RX_FILTER_COUNT */
    CAN_CONFIG_RX_FILTER0_ID,
    CAN_CONFIG_RX_FILTER0_MASK,         /* 1 = bit must match */
    CAN_CONFIG_RX_FILTER0_IS_EXTENDED,
    CAN_CONFIG_RX_FILTER1_ID,
    CAN_CONFIG_RX_FILTER1_MASK,
    CAN_CONFIG_RX_FILTER1_IS_EXTENDED,
}
CAN_ConfigId_T;

extern int CAN_ConfigId_Get(CAN_T * p_can, CAN_ConfigId_T id);
extern void CAN_ConfigId_Set(CAN_T * p_can, CAN_ConfigId_T id, int value);

/* Tx — polling, no interrupt, fire-and-forget */
// extern void CAN_TxData(CAN_T * p_can, can_id_t id, const uint8_t * p_txData, size_t length);
/* Tx — remote frame request. sets buffer state to RX_WAIT_REMOTE for the response. */
// extern void CAN_TxRemote(CAN_T * p_can, can_id_t id,  const uint8_t * p_txData,  size_t length);
/* Tx — accept a fully-built frame. */
// extern void CAN_Tx(CAN_T * p_can, CAN_Frame_T * p_frame);




#endif
