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
    @file   Protocol.c
    @author FireSourcery
    @brief  Protocol abstract class equivalent

*/
/******************************************************************************/
#include "Protocol.h"
#include "Socket.h"
#include <string.h>


/******************************************************************************/
/*!
    Init
*/
/******************************************************************************/
void Socket_Init(Socket_T * p_socket)
{
    Socket_State_T * p_state = p_socket->P_SOCKET_STATE;

    if (p_socket->P_NVM_CONFIG != NULL)
    {
        memcpy(&p_state->Config, p_socket->P_NVM_CONFIG, sizeof(Socket_Config_T));

        p_state->p_Xcvr = mux_or_default((entry_table_t)p_socket->P_XCVR_TABLE, p_socket->XCVR_COUNT, p_state->Config.XcvrId);
        Socket_ConfigXcvrBaudRate(p_socket, p_state->Config.BaudRate);
        Socket_SetSpecs(p_socket, p_state->Config.SpecsId);

        if (p_state->Config.IsEnableOnInit == true) { Socket_Enable(p_socket); } else { Socket_Disable(p_socket); }
    }
    else
    {
        Socket_Disable(p_socket);
    }
}

/******************************************************************************/
/*!
    Proc
*/
/******************************************************************************/
/*
    Receive into P_RX_PACKET_BUFFER and run PARSE_RX_META for RxMeta.Length and ReqCode / Rx completion
    Packet is complete => Req, ReqExt or Sync, or Error
*/
static inline Protocol_RxCode_T CaptureRx(const Socket_T * p_socket, Socket_State_T * p_state)
{
    Protocol_RxCode_T rxStatus = PROTOCOL_RX_CODE_AWAIT_PACKET;
    uint8_t nextRxIndex;
    // uint8_t xcvrRxLimit; /* PARSE_RX_META after this number is received */
    // uint8_t xcvrRxCount;

    /* Loop to empty Xcvr Rx buffer. Check for completetion, per Rx 1 byte, during unknown length, or up to known length */
    while (p_state->RxIndex < p_state->p_Specs->RX_LENGTH_MAX) /* RX_LENGTH_MAX <= CONFIG.PACKET_BUFFER_LENGTH */
    {
        if (rxStatus != PROTOCOL_RX_CODE_AWAIT_PACKET) { break; } /* Packet is complete, break */

        /*
            Set xcvrRxLimit for PARSE_RX_META. Prevent reading bytes from the following packet.
            RxMeta.Length unknown => Check 1 byte or a known const value, until RxMeta.Length is known
            RxMeta.Length known => Check Rx remaining
        */
        if (p_state->RxMeta.Length > 0U)  /* PacketLength is known. */
        {
            if (p_state->RxIndex < p_state->RxMeta.Length) { nextRxIndex = p_state->RxMeta.Length; }
            else { rxStatus = PROTOCOL_RX_CODE_ERROR_META; break; }
            /* (RxIndex == RxMeta.Length) => (xcvrRxLimit == 0), when rxStatus == PROTOCOL_RX_CODE_WAIT_PACKET erroneously i.e. received full packet without completion status */
        }
        /* PacketLength is unkown */
        else if (p_state->RxIndex < p_state->p_Specs->RX_LENGTH_MIN) { nextRxIndex = p_state->p_Specs->RX_LENGTH_MIN; }
        /* RX_LENGTH_INDEX > RX_LENGTH_MIN. Check for length determined by another property */
        // else if(p_state->RxIndex < p_state->p_Specs->RX_REQ_ID_INDEX) { nextRxIndex = p_state->p_Specs->RX_REQ_ID_INDEX + 1U; }
        // else if(p_state->RxIndex < p_state->p_Specs->RX_LENGTH_INDEX) { nextRxIndex = p_state->p_Specs->RX_LENGTH_INDEX + 1U; }
        else { nextRxIndex = p_state->RxIndex + 1U; }

        if (nextRxIndex > p_state->p_Specs->RX_LENGTH_MAX) { rxStatus = PROTOCOL_RX_CODE_ERROR_META; break; }
        /* Copy from Xcvr buffer to Protocol buffer, up to xcvrRxLimit */
        p_state->RxIndex += Xcvr_RxMax(p_state->p_Xcvr, &p_socket->P_RX_PACKET_BUFFER[p_state->RxIndex], nextRxIndex - p_state->RxIndex);

        if (p_state->RxIndex == nextRxIndex) /* (xcvrRxCount == xcvrRxLimit) */
        {
            /* Implicitly: (xcvrRxCount != 0), (xcvrRxLimit != 0). (RxIndex >= RX_LENGTH_MIN)  */
            /* returns PROTOCOL_RX_CODE_AWAIT_PACKET on successful set of meta data *//* more bytes in Xcvr Buffer, continue while loop */
            rxStatus = p_state->p_Specs->PARSE_RX_META(&p_state->RxMeta, p_socket->P_RX_PACKET_BUFFER, p_state->RxIndex);
        }
        else /* xcvrRxCount < xcvrRxLimit => Xcvr Rx Buffer empty, wait for Xcvr */
        {
            break;  /*  rxStatus == PROTOCOL_RX_CODE_AWAIT_PACKET; */
        }
    }

    return rxStatus;
}

static void TxSync(const Socket_T * p_socket, Socket_State_T * p_state, Protocol_TxSyncId_T txId)
{
    packet_size_t txLength = 0U; /* local length. not overwrite in case of retransmission */
    if (p_state->p_Specs->BUILD_TX_SYNC != 0U)
    {
        p_state->p_Specs->BUILD_TX_SYNC(p_socket->P_TX_PACKET_BUFFER, &txLength, txId);
        Xcvr_TxN(p_state->p_Xcvr, p_socket->P_TX_PACKET_BUFFER, txLength);
    }
}

/*
    ReqResp return 0, TxLength == 0, skip tx
    Error handle Xcvr TxBuffer full?
*/
static inline bool TxResp(const Socket_T * p_socket, Socket_State_T * p_state)
{
    return (p_state->TxLength > 0U) ? Xcvr_TxN(p_state->p_Xcvr, p_socket->P_TX_PACKET_BUFFER, p_state->TxLength) : false;
}

/*
    Tx Nack on Rx Error
    if client side needs to terminate nack loop. prefer implement on host side
*/
static bool ProcTxNackRxRepeat(const Socket_T * p_socket, Socket_State_T * p_state, uint8_t * p_nackCounter, uint8_t nackRepeatMax, Protocol_TxSyncId_T nackRepeatId, Protocol_TxSyncId_T nackFinalId)
{
    bool isFinal = false;

    if ((*p_nackCounter < nackRepeatMax) || (nackRepeatMax == 0U))
    {
        TxSync(p_socket, p_state, nackRepeatId);
        *p_nackCounter++;
        isFinal = false;
    }
    else
    {
        TxSync(p_socket, p_state, nackFinalId);
        *p_nackCounter = 0U;
        isFinal = true;
    }

    return isFinal;
}

// static inline bool IsRxLost(Socket_T * p_socket)
// {
//     Socket_State_T * p_state = p_socket->P_SOCKET_STATE;
//     return (*p_socket->P_TIMER - p_state->ReqTimeStart > p_state->Config.WatchdogTimeout);
// }

/*
    Handle Rx Packet
    Controls Rx Packet Parser, and Timeout Timer
    if run inside isr, need sync mechanism
*/
static inline Protocol_RxCode_T ProcRxState(const Socket_T * p_socket, Socket_State_T * p_state)
{
    Protocol_RxCode_T rxStatus = PROTOCOL_RX_CODE_AWAIT_PACKET;
    switch (p_state->RxState)
    {
        case PROTOCOL_RX_STATE_WAIT_BYTE_1: /* nonblocking wait state, no timer */
            if (Xcvr_RxMax(p_state->p_Xcvr, &p_socket->P_RX_PACKET_BUFFER[0U], 1U) > 0U)
            {
                /*
                    Use starting byte even if data is unencoded. first char can still be handled in separate state.
                    Unencoded still discards Rx chars until starting ID
                */
                if ((p_socket->P_RX_PACKET_BUFFER[0U] == p_state->p_Specs->RX_START_ID) || (p_state->p_Specs->RX_START_ID == 0x00U))
                {
                    p_state->RxIndex = 1U;
                    p_state->RxMeta.Length = 0U; /* RxMeta.Length is unknown */
                    p_state->RxTimeStart = *p_socket->P_TIMER;
                    p_state->RxState = PROTOCOL_RX_STATE_WAIT_PACKET;
                }
            }
            rxStatus = PROTOCOL_RX_CODE_AWAIT_PACKET; /* Should already be set */
            break;

        case PROTOCOL_RX_STATE_WAIT_PACKET: /* nonblocking wait state, timer started */
            rxStatus = CaptureRx(p_socket, p_state);
            switch (rxStatus)
            {
                case PROTOCOL_RX_CODE_AWAIT_PACKET:
                    /* check timer after reading buffer, to ensure timeout occurs when buffer is empty */
                    if (*p_socket->P_TIMER - p_state->RxTimeStart > p_state->p_Specs->RX_TIMEOUT)  /* No need to check for overflow if using millis */
                    {
                        // ProcNackRepeat(p_state, p_state->p_Specs->NACK_COUNT, PROTOCOL_TX_SYNC_NACK_RX_TIMEOUT, PROTOCOL_TX_SYNC_ABORT);
                        // Xcvr_FlushRxBuffer(p_state->p_Xcvr);
                        TxSync(p_socket, p_state, PROTOCOL_TX_SYNC_NACK_RX_TIMEOUT);
                        p_state->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1;
                        rxStatus = PROTOCOL_RX_CODE_ERROR_TIMEOUT;
                    }
                    break;

                case PROTOCOL_RX_CODE_PACKET_COMPLETE:
                    p_state->NackCount = 0U; // p_state->NackDataCount = 0U;
                    // p_state->RxPacketSuccessCount++;
                    break;

                case PROTOCOL_RX_CODE_ERROR_META:
                    /* May occur before a packet is complete. await byte 1 maybe erroneous if data is unencoded. flush buffers */
                    // Xcvr_FlushRxBuffer(p_state->p_Xcvr);
                    // ProcNackRepeat(p_state, p_state->p_Specs->NACK_COUNT, PROTOCOL_TX_SYNC_NACK_PACKET_META, PROTOCOL_TX_SYNC_ABORT);
                    TxSync(p_socket, p_state, PROTOCOL_TX_SYNC_NACK_PACKET_META);
                    // p_state->RxPacketErrorCount++;
                    break;

                case PROTOCOL_RX_CODE_ERROR_DATA:
                    // ProcNackRepeat(p_state, p_state->p_Specs->NACK_COUNT, PROTOCOL_TX_SYNC_NACK_PACKET_DATA, PROTOCOL_TX_SYNC_ABORT);
                    TxSync(p_socket, p_state, PROTOCOL_TX_SYNC_NACK_PACKET_DATA);
                    // p_state->RxPacketErrorCount++;
                    break;

                case PROTOCOL_RX_CODE_ACK: break;
                case PROTOCOL_RX_CODE_NACK: break;
                case PROTOCOL_RX_CODE_ABORT: break;
                default: break;
            }

            /*
                Continue CaptureRx during ReqExt processing
                Buffers will not be overwritten until user function returns. (No repeat process on same Rx)
                Req ensure packet data is processed, or copied
                Rx can queue out of sequence. Invalid Rx sequence until timeout buffer flush

                Alternatively, pause CaptureRx during ReqExt processing
                Incoming packet bytes wait in queue. Cannot miss packets (unless overflow)
                Cannot check for Abort without user signal, persistent wait process
            */
            if (rxStatus != PROTOCOL_RX_CODE_AWAIT_PACKET) { p_state->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1; }
            break;

        case PROTOCOL_RX_STATE_INACTIVE:
            break;

        default: break;
    }

    return rxStatus;
}

/*
    Handle user Req/Cmd function
    ReqTimeStart resets for all expected behaviors, in sequence packets

    Handle here
    PROTOCOL_RX_CODE_AWAIT_PACKET,
    PROTOCOL_RX_CODE_PACKET_COMPLETE,
    PROTOCOL_RX_CODE_ACK,
    PROTOCOL_RX_CODE_NACK,
    PROTOCOL_RX_CODE_ABORT,

    Handle Outside
    PROTOCOL_RX_CODE_ERROR_TIMEOUT,
    PROTOCOL_RX_CODE_ERROR_META,
    PROTOCOL_RX_CODE_ERROR_DATA,
*/
static inline Protocol_ReqCode_T ProcReqState(const Socket_T * p_socket, Socket_State_T * p_state, Protocol_RxCode_T rxCode)
{
    Protocol_ReqCode_T reqStatus = PROTOCOL_REQ_CODE_TX_CONTINUE;
    const Protocol_ReqContext_T reqContext =
    {
        .p_SubState = p_socket->P_REQ_STATE_BUFFER,
        .p_RxPacket = p_socket->P_RX_PACKET_BUFFER,
        .p_TxPacket = p_socket->P_TX_PACKET_BUFFER,
        .p_RxMeta = &p_state->RxMeta,
        .p_TxSize = &p_state->TxLength,
        .p_SubStateIndex = &p_state->ReqSubStateIndex,
    };

    switch (p_state->ReqState)
    {
        case PROTOCOL_REQ_STATE_WAIT_RX_ID: /* Initial packet containing Req Id */
            switch (rxCode)
            {
                case PROTOCOL_RX_CODE_AWAIT_PACKET: reqStatus = PROTOCOL_REQ_CODE_TX_CONTINUE; break;
                case PROTOCOL_RX_CODE_PACKET_COMPLETE:
                    p_state->p_ReqActive = _Protocol_SearchReqTable(p_socket->P_REQ_TABLE, p_socket->REQ_TABLE_LENGTH, p_state->RxMeta.ReqId);

                    if (p_state->p_ReqActive != NULL)
                    {
                        p_state->ReqTimeStart = *p_socket->P_TIMER; /* Reset timer on all non error packets, feed RxLost Timer */

                        if (p_state->p_ReqActive->SYNC.TX_ACK == true) { TxSync(p_socket, p_state, PROTOCOL_TX_SYNC_ACK_REQ); }

                        // disallow both types? PROC, PROC_EXT
                        if (p_state->p_ReqActive->PROC != NULL) /* Does not invoke state machine, no loop / nonblocking wait. */
                        {
                            // p_state->TxLength = 0U; /* in case it is not set by the user function */
                            p_state->TxLength = p_state->p_ReqActive->PROC(p_socket->P_APP_CONTEXT, p_socket->P_TX_PACKET_BUFFER, p_socket->P_RX_PACKET_BUFFER);
                            // p_state->p_Specs->BUILD_TX_HEADER(p_socket->P_TX_PACKET_BUFFER, &p_state->TxLength, txId, payloadLength);
                            if ((TxResp(p_socket, p_state) == true) && (p_state->p_ReqActive->SYNC.RX_ACK == true)) /* Tx resp if TxLength > 0 */
                            {
                                p_state->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_SYNC;
                                p_state->RxNackTxCount = 0U;
                                reqStatus = PROTOCOL_REQ_CODE_AWAIT_RX_SYNC;
                            }
                            else if (p_state->p_ReqActive->PROC_EXT == NULL)
                            {
                                reqStatus = PROTOCOL_REQ_CODE_PROCESS_COMPLETE;
                                /* p_state->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_ID; */
                            }
                        } //todo exclusive or determine sync

                        if ((p_state->p_ReqActive->PROC_EXT != NULL) && (reqStatus != PROTOCOL_REQ_CODE_AWAIT_RX_SYNC))
                            /* ((p_state->p_ReqActive->PROC == 0U) || (p_state->p_ReqActive->SYNC.RX_ACK == false || (p_state->TxLength == 0U))) */
                        {
                            // if(p_state->p_Specs->REQ_EXT_RESET != 0U) { p_state->p_Specs->REQ_EXT_RESET(p_socket->P_REQ_STATE_BUFFER); }
                            p_state->ReqSubStateIndex = 0U;
                            p_state->ReqState = PROTOCOL_REQ_STATE_PROCESS_REQ_EXT;
                            reqStatus = PROTOCOL_REQ_CODE_TX_CONTINUE;
                        }
                        /* if (p_state->p_ReqActive->PROC == 0U) && (p_state->p_ReqActive->PROC_EXT == 0U) reqStatus = PROTOCOL_REQ_CODE_PROCESS_COMPLETE; */
                    }
                    else
                    {
                        TxSync(p_socket, p_state, PROTOCOL_TX_SYNC_NACK_REQ);
                        reqStatus = PROTOCOL_REQ_CODE_ERROR_ID;
                    }
                    break;
                case PROTOCOL_RX_CODE_ACK:      reqStatus = PROTOCOL_REQ_CODE_ERROR_RX_UNEXPECTED; break;
                case PROTOCOL_RX_CODE_NACK:     reqStatus = PROTOCOL_REQ_CODE_ERROR_RX_UNEXPECTED; break;
                case PROTOCOL_RX_CODE_ABORT:    reqStatus = PROTOCOL_REQ_CODE_ERROR_RX_UNEXPECTED; break;

                    /* (rxCode == PROTOCOL_RX_CODE_ERROR_META) || (rxCode == PROTOCOL_RX_CODE_ERROR_DATA) || (rxCode == PROTOCOL_RX_CODE_ERROR_TIMEOUT) */
                default: reqStatus = PROTOCOL_REQ_CODE_PROCESS_COMPLETE; break;
            }
            break;


        case PROTOCOL_REQ_STATE_PROCESS_REQ_EXT: /* Proc once then transitions, non waiting */
            reqStatus = p_state->p_ReqActive->PROC_EXT(p_socket->P_APP_CONTEXT, &reqContext);

            switch (reqStatus)
            {

                case PROTOCOL_REQ_CODE_TX_CONTINUE: /* Tx then continue */
                    if ((TxResp(p_socket, p_state) == true) && (p_state->p_ReqActive->SYNC.RX_ACK_EXT == true))
                    {
                        p_state->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_SYNC;
                        p_state->RxNackTxCount = 0U;
                    }
                    else
                    {
                        p_state->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_CONTINUE; //override next rx id
                    }
                    break;

                case PROTOCOL_REQ_CODE_AWAIT_RX_CONTINUE:   p_state->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_CONTINUE;     break;
                case PROTOCOL_REQ_CODE_AWAIT_RX_SYNC:       p_state->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_SYNC;         break;

                    //split complete_clear_state and continue with id
                    // state saved
                    // case PROTOCOL_REQ_CODE_PROCESS_RETURN: p_state->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_ID;
                    //     break;
                    // need duplicate sync state, or track index/flag intenrally
                case PROTOCOL_REQ_CODE_PROCESS_COMPLETE:
                    p_state->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_ID;
                    p_state->ReqSubStateIndex = 0U;
                    // if(p_state->p_Specs->REQ_EXT_RESET != 0U) { p_state->p_Specs->REQ_EXT_RESET(p_socket->P_REQ_STATE_BUFFER); }
                    break;


                case PROTOCOL_REQ_CODE_ABORT: p_state->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_ID; break;

                    /* Granular function, manually initiated, if a custom sequence is needed  */
                    /* Tx Response and transfer control back to PROC_EXT */
                    // alternatively directly call helper
                case PROTOCOL_REQ_CODE_TX_RESPONSE:         TxResp(p_socket, p_state);                                             break;
                case PROTOCOL_REQ_CODE_TX_ACK:              TxSync(p_socket, p_state, PROTOCOL_TX_SYNC_ACK_REQ_EXT);               break;
                case PROTOCOL_REQ_CODE_TX_NACK:             TxSync(p_socket, p_state, PROTOCOL_TX_SYNC_NACK_REQ_EXT);              break;

                    /* ACK by PROC_EXT, after proc, instead of on reception */
                case PROTOCOL_REQ_CODE_PROCESS_ACK:
                    TxSync(p_socket, p_state, PROTOCOL_TX_SYNC_ACK_REQ_EXT);
                    p_state->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_CONTINUE;
                    break;

                    /* NACK by PROC_EXT */
                case PROTOCOL_REQ_CODE_PROCESS_NACK:
                    TxSync(p_socket, p_state, PROTOCOL_TX_SYNC_NACK_REQ_EXT);
                    p_state->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_CONTINUE;
                    // if(ProcTxNackRxRepeat(p_state, &p_state->TxNackRxCount, p_state->p_ReqActive->SYNC.NACK_REPEAT, PROTOCOL_TX_SYNC_NACK_REQ_EXT, PROTOCOL_TX_SYNC_NACK_REQ_EXT) == true)
                    // { p_state->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_ID; }
                    break;

                    // // noop
                    // /* PROC_EXT implements nonblocking wait, copy packet buffers*/
                    // case PROTOCOL_REQ_CODE_AWAIT_PROCESS:                                                                           break;
                    // case PROTOCOL_REQ_CODE_AWAIT_PROCESS_EXTEND_TIMER: p_state->ReqTimeStart = *p_socket->P_TIMER;      break;
                case PROTOCOL_REQ_CODE_ERROR_RX_UNEXPECTED:   break;

                default: break;
            }

            switch (rxCode)
            {
                case PROTOCOL_RX_CODE_AWAIT_PACKET: // handlke once before checking rx?
                    break;
                case PROTOCOL_RX_CODE_PACKET_COMPLETE:  reqStatus = PROTOCOL_REQ_CODE_ERROR_RX_UNEXPECTED;  break;
                case PROTOCOL_RX_CODE_ACK:              reqStatus = PROTOCOL_REQ_CODE_ERROR_RX_UNEXPECTED;  break;
                case PROTOCOL_RX_CODE_NACK:             reqStatus = PROTOCOL_REQ_CODE_ERROR_RX_UNEXPECTED;  break;
                case PROTOCOL_RX_CODE_ABORT: /* handle outside */ break;

                    /* (rxCode == PROTOCOL_RX_CODE_ERROR_META) || (rxCode == PROTOCOL_RX_CODE_ERROR_DATA) || (rxCode == PROTOCOL_RX_CODE_ERROR_TIMEOUT) */
                default: reqStatus = PROTOCOL_REQ_CODE_ERROR_RX_UNEXPECTED;  break;
            }
            break;

        case PROTOCOL_REQ_STATE_WAIT_RX_CONTINUE: /* Wait Rx Req Ext Continue - Treats next rx id as current req id */ // can combine iwth wait init, if a eclusive loop is not needed
            switch (rxCode)
            {
                case PROTOCOL_RX_CODE_AWAIT_PACKET: reqStatus = PROTOCOL_REQ_CODE_AWAIT_RX_CONTINUE; break;
                case PROTOCOL_RX_CODE_PACKET_COMPLETE:
                    p_state->ReqTimeStart = *p_socket->P_TIMER;
                    if (p_state->p_ReqActive->SYNC.TX_ACK_EXT == true) { TxSync(p_socket, p_state, PROTOCOL_TX_SYNC_ACK_REQ_EXT); } /* Ack on reception, perform Rx error checking only */
                    p_state->ReqState = PROTOCOL_REQ_STATE_PROCESS_REQ_EXT;
                    reqStatus = PROTOCOL_REQ_CODE_TX_CONTINUE;
                    break;
                case PROTOCOL_RX_CODE_ACK:  reqStatus = PROTOCOL_REQ_CODE_ERROR_RX_UNEXPECTED; break;
                case PROTOCOL_RX_CODE_NACK: reqStatus = PROTOCOL_REQ_CODE_ERROR_RX_UNEXPECTED; break;
                case PROTOCOL_RX_CODE_ABORT: break;

                    /* (rxCode == PROTOCOL_RX_CODE_ERROR_META) || (rxCode == PROTOCOL_RX_CODE_ERROR_DATA) || (rxCode == PROTOCOL_RX_CODE_ERROR_TIMEOUT) */
                default: reqStatus = PROTOCOL_REQ_CODE_AWAIT_RX_CONTINUE; break;
            }
            break;

        case PROTOCOL_REQ_STATE_WAIT_RX_SYNC:
            switch (rxCode)
            {
                case PROTOCOL_RX_CODE_AWAIT_PACKET: reqStatus = PROTOCOL_REQ_CODE_AWAIT_RX_SYNC; break;
                case PROTOCOL_RX_CODE_PACKET_COMPLETE: reqStatus = PROTOCOL_REQ_CODE_ERROR_RX_UNEXPECTED; break;
                case PROTOCOL_RX_CODE_ACK:
                    p_state->ReqTimeStart = *p_socket->P_TIMER;
                    if (p_state->p_ReqActive->PROC_EXT != NULL)
                    {
                        //flag for final?/* continue loop/ goto reqid split */
                        p_state->ReqState = PROTOCOL_REQ_STATE_PROCESS_REQ_EXT;
                        reqStatus = PROTOCOL_REQ_CODE_TX_CONTINUE;
                    }
                    else
                    {
                        p_state->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_ID;
                        reqStatus = PROTOCOL_REQ_CODE_PROCESS_COMPLETE;
                    }
                    break;
                case PROTOCOL_RX_CODE_NACK:
                    TxResp(p_socket, p_state); /* Retransmit Buffered Packet */
                    reqStatus = PROTOCOL_REQ_CODE_AWAIT_RX_SYNC;
                    // if(p_state->RxNackTxCount < p_state->p_ReqActive->SYNC.NACK_REPEAT)
                    // {
                    //     TxResp(p_socket, p_state); /* Retransmit Buffered Packet */
                    //     p_state->RxNackTxCount++;
                    //     reqStatus = PROTOCOL_REQ_CODE_AWAIT_RX_SYNC;
                    // }
                    // else
                    // {
                    //     p_state->RxNackTxCount = 0U;
                    //     p_state->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_ID;
                    //     reqStatus = PROTOCOL_REQ_CODE_PROCESS_COMPLETE;
                    // }
                    break;
                case PROTOCOL_RX_CODE_ABORT: break;

                    /* (rxCode == PROTOCOL_RX_CODE_ERROR_META) || (rxCode == PROTOCOL_RX_CODE_ERROR_DATA) || (rxCode == PROTOCOL_RX_CODE_ERROR_TIMEOUT) */
                default: reqStatus = PROTOCOL_REQ_CODE_AWAIT_RX_SYNC; break;
            }
            break;

        case PROTOCOL_REQ_STATE_INACTIVE: break;

        default: break;
    }

    // if (reqStatus == PROTOCOL_REQ_CODE_ERROR_RX_UNEXPECTED) { p_state->RxPacketErrorSync++; }
    /* States Common */
    if ((p_state->ReqState == PROTOCOL_REQ_STATE_WAIT_RX_CONTINUE) || (p_state->ReqState == PROTOCOL_REQ_STATE_WAIT_RX_SYNC) || (p_state->ReqState == PROTOCOL_REQ_STATE_PROCESS_REQ_EXT))
    {
        if (*p_socket->P_TIMER - p_state->ReqTimeStart > p_state->p_Specs->REQ_TIMEOUT) /* Timer restarts on Req packet and Ack */
        {
            TxSync(p_socket, p_state, PROTOCOL_TX_SYNC_NACK_REQ_TIMEOUT);
            p_state->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_ID;
            reqStatus = PROTOCOL_REQ_CODE_ERROR_TIMEOUT;
        }
        else if (rxCode == PROTOCOL_RX_CODE_ABORT)
        {
            if (p_state->p_ReqActive->SYNC.TX_ACK_ABORT == true) { TxSync(p_socket, p_state, PROTOCOL_TX_SYNC_ACK_ABORT); }
            p_state->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_ID;
            reqStatus = PROTOCOL_REQ_CODE_ABORT;
        }
    }

    return reqStatus;
}

/*! @return pointer to Req */
const Protocol_Req_T * _Protocol_SearchReqTable(Protocol_Req_T * p_reqTable, size_t tableLength, packet_id_t id)
{
    const Protocol_Req_T * p_req = NULL;
    for (uint8_t iChar = 0U; iChar < tableLength; iChar++) { if (p_reqTable[iChar].ID == id) { p_req = &p_reqTable[iChar]; break; } }
    return p_req;
}

/*
    Non blocking.
    Single threaded only, RxIndex not protected
    Controller Side, sequential rx req, proc, tx response
    Cmdr Side -

    Outputs RxStatus, ReqStatus - Updated every proc
*/
void Socket_Proc(Socket_T * p_socket)
{
    Socket_State_T * p_state = p_socket->P_SOCKET_STATE;
    p_state->RxStatus = ProcRxState(p_socket, p_state);
    p_state->ReqStatus = ProcReqState(p_socket, p_state, p_state->RxStatus);
    // ProcTxAsync(p_state);
    // ProcDatagram(p_state);  /* Enqueue Datagram for processing in parallel */
}

/******************************************************************************/
/*!
    Config
*/
/******************************************************************************/
void Socket_SetXcvr(Socket_T * p_socket, uint8_t xcvrId)
{
    Socket_State_T * p_state = p_socket->P_SOCKET_STATE;
    p_state->Config.XcvrId = xcvrId;
    p_state->p_Xcvr = mux_or_default((entry_table_t)p_socket->P_XCVR_TABLE, p_socket->XCVR_COUNT, xcvrId);
}

void Socket_ConfigXcvrBaudRate(Socket_T * p_socket, uint32_t baudRate)
{
    Socket_State_T * p_state = p_socket->P_SOCKET_STATE;
    if ((baudRate != 0U) && mux_is_valid(p_state->p_Xcvr, (entry_table_t)p_socket->P_XCVR_TABLE, p_socket->XCVR_COUNT, p_state->Config.XcvrId) == true)
    {
        Xcvr_ConfigBaudRate(p_state->p_Xcvr, baudRate);
    }
}

void Socket_SetSpecs(Socket_T * p_socket, uint8_t p_specsId)
{
    Socket_State_T * p_state = p_socket->P_SOCKET_STATE;
    const PacketClass_T * p_specs = (p_specsId < p_socket->PACKET_CLASS_COUNT) ? p_socket->P_PACKET_CLASS_TABLE[p_specsId] : NULL;

    if ((p_specs != NULL) && (p_specs->RX_LENGTH_MAX <= p_socket->PACKET_BUFFER_LENGTH))
    {
        p_state->p_Specs = p_specs;
        // if(p_state->Config.BaudRate == 0U) { Socket_ConfigXcvrBaudRate(p_state, p_state->p_Specs->BAUD_RATE_DEFAULT); }
    }
}

bool Socket_Enable(Socket_T * p_socket)
{
    Socket_State_T * p_state = p_socket->P_SOCKET_STATE;
    bool isEnable = ((mux_is_valid(p_state->p_Xcvr, (entry_table_t)p_socket->P_XCVR_TABLE, p_socket->XCVR_COUNT, p_state->Config.XcvrId) == true) && (p_state->p_Specs != NULL));

    if (isEnable == true)
    {
        p_state->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1;
        p_state->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_ID;
        p_state->RxIndex = 0U;
        p_state->TxLength = 0U;
        p_state->p_ReqActive = NULL;
    }

    return isEnable;
}

void Socket_Disable(Socket_T * p_socket)
{
    Socket_State_T * p_state = p_socket->P_SOCKET_STATE;
    p_state->RxState = PROTOCOL_RX_STATE_INACTIVE;
    p_state->ReqState = PROTOCOL_REQ_STATE_INACTIVE;
}

/******************************************************************************/
/*!
    Var Id interface
*/
/******************************************************************************/
int _Socket_ConfigId_Get(const Socket_State_T * p_socket, Protocol_ConfigId_T id)
{
    int value = 0;
    switch (id)
    {
        case PROTOCOL_CONFIG_XCVR_ID:         value = p_socket->Config.XcvrId;            break;
        case PROTOCOL_CONFIG_SPECS_ID:        value = p_socket->Config.SpecsId;           break;
        case PROTOCOL_CONFIG_WATCHDOG_TIME:   value = p_socket->Config.WatchdogTimeout;   break;
        case PROTOCOL_CONFIG_BAUD_RATE:       value = p_socket->Config.BaudRate;          break;
        case PROTOCOL_CONFIG_IS_ENABLED:      value = p_socket->Config.IsEnableOnInit;    break;
    }
    return value;
}


void _Socket_ConfigId_Set(Socket_State_T * p_socket, Protocol_ConfigId_T id, int value)
{
    switch (id)
    {
        case PROTOCOL_CONFIG_XCVR_ID:         p_socket->Config.XcvrId = value;            break;
        case PROTOCOL_CONFIG_SPECS_ID:        p_socket->Config.SpecsId = value;           break;
        case PROTOCOL_CONFIG_WATCHDOG_TIME:   p_socket->Config.WatchdogTimeout = value;   break;
        case PROTOCOL_CONFIG_BAUD_RATE:       p_socket->Config.BaudRate = value;          break;
        case PROTOCOL_CONFIG_IS_ENABLED:      p_socket->Config.IsEnableOnInit = value;    break;
    }
}

int Socket_ConfigId_Get(const Socket_T * p_socket, Protocol_ConfigId_T id)
{
    return (p_socket != NULL) ? _Socket_ConfigId_Get(p_socket->P_SOCKET_STATE, id) : 0;
}

void Socket_ConfigId_Set(const Socket_T * p_socket, Protocol_ConfigId_T id, int value)
{
    if (p_socket != NULL) { _Socket_ConfigId_Set(p_socket->P_SOCKET_STATE, id, value); }
}


//todo
//static void ProcDatagram(Socket_T * p_socket)
//{
//    //* save room for 1 req packet
//    if (Port_GetTxEmpty() > Datagram_GetPacketSize(&p_state->Datagram) +  p_socket->PACKET_BUFFER_LENGTH)
//    {
//        if (Datagram_Server_Proc(&p_state->Datagram))
//        {
//            PortTxString(p_state, p_state->Datagram.P_TX_BUFFER, p_state->Datagram.TxDataSizeActive + p_state->Datagram.HeaderSize);
//        }
//    }
//        //    if (Port_GetTxEmpty() > Datagram_GetPacketSize(&p_state->Datagram) +  p_socket->PACKET_BUFFER_LENGTH)
//        //    {
//                if (Datagram_Server_Proc(&p_state->Datagram))
//                {
//                    PortTxString(p_state, p_state->Datagram.P_TX_BUFFER, p_state->Datagram.TxDataSizeActive + p_state->Datagram.HeaderSize);
//                }
//        //    }
//}
