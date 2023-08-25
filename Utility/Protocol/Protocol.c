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
    @brief
    @version V0
*/
/******************************************************************************/
#include "Protocol.h"
#include <string.h>

/*! @return pointer to Req */
const Protocol_Req_T * _Protocol_SearchReqTable(Protocol_Req_T * p_reqTable, size_t tableLength, protocol_reqid_t id)
{
    const Protocol_Req_T * p_req = 0U;
    for(uint8_t iChar = 0U; iChar < tableLength; iChar++) { if(p_reqTable[iChar].ID == id) { p_req = &p_reqTable[iChar]; break; } }
    return p_req;
}

void Protocol_Init(Protocol_T * p_protocol)
{
    if(p_protocol->CONFIG.P_PARAMS != 0U)
    {
        memcpy(&p_protocol->Params, p_protocol->CONFIG.P_PARAMS, sizeof(Protocol_Params_T));

        Xcvr_Init(&p_protocol->Xcvr, p_protocol->Params.XcvrId);
        Protocol_ConfigXcvrBaudRate(p_protocol, p_protocol->Params.BaudRate);
        Protocol_SetSpecs(p_protocol, p_protocol->Params.SpecsId);
        if(p_protocol->Params.IsEnableOnInit == true) { Protocol_Enable(p_protocol); } else { Protocol_Disable(p_protocol); }
    }
    else
    {
        Protocol_Disable(p_protocol);
    }
}

/*
    Receive into P_RX_PACKET_BUFFER and run PARSE_RX_META for RxPacketLength and ReqCode / Rx completion
    Packet is complete => Req, ReqExt or Sync, or Error
*/
static inline Protocol_RxCode_T BuildRxPacket(Protocol_T * p_protocol)
{
    Protocol_RxCode_T rxStatus = PROTOCOL_RX_CODE_AWAIT_PACKET;
    uint8_t xcvrRxLimit; /* PARSE_RX_META after this number is received */
    uint8_t xcvrRxCount;

    while(p_protocol->RxIndex < p_protocol->p_Specs->RX_LENGTH_MAX) /* RX_LENGTH_MAX < CONFIG.PACKET_BUFFER_LENGTH */
    {
        /*
            Set xcvrRxLimit for PARSE_RX_META. Prevent reading byte from following packet
            RxPacketLength unknown => Check 1 byte or upto RX_LENGTH_MIN at a time, until RxPacketLength is known
            RxPacketLength known => Check Rx remaining
        */
        /* if(p_protocol->RxIndex > p_protocol->p_Specs->RX_LENGTH_MAX) should not occur. wait for time out */
        // if(p_protocol->RxIndex < p_protocol->p_Specs->RX_REQ_INDEX)      { xcvrRxLimit = p_protocol->p_Specs->RX_REQ_INDEX - p_protocol->RxIndex + 1; } //todo fixed req index
        // if(p_protocol->RxIndex < p_protocol->p_Specs->RX_LENGTH_INDEX)   { xcvrRxLimit = p_protocol->p_Specs->RX_LENGTH_INDEX - p_protocol->RxIndex + 1; } //todo fixed packet length index
        if(p_protocol->RxIndex < p_protocol->p_Specs->RX_LENGTH_MIN)    { xcvrRxLimit = p_protocol->p_Specs->RX_LENGTH_MIN - p_protocol->RxIndex; } /* PacketLength Index always > RX_LENGTH_MIN */
        else if(p_protocol->RxLength == 0U)                             { xcvrRxLimit = 1U; }
        else /* PacketLength is known */
        {
            if((p_protocol->RxIndex < p_protocol->RxLength) && (p_protocol->RxLength <= p_protocol->p_Specs->RX_LENGTH_MAX))
                { xcvrRxLimit = p_protocol->RxLength - p_protocol->RxIndex; }
            else
                { xcvrRxLimit = 0U; rxStatus = PROTOCOL_RX_CODE_ERROR_META; break; }
            /* xcvrRxLimit == 0, RxIndex == RxPacketLength when rxStatus == PROTOCOL_RX_CODE_WAIT_PACKET erroneously i.e. received full packet without completion status */
        }

        xcvrRxCount = Xcvr_RxMax(&p_protocol->Xcvr, &p_protocol->CONFIG.P_RX_PACKET_BUFFER[p_protocol->RxIndex], xcvrRxLimit); /* Copy from Xcvr buffer to Protocol buffer, up to xcvrRxLimit */
        p_protocol->RxIndex += xcvrRxCount; /* Always update state, used for error checking */

        if(xcvrRxCount == xcvrRxLimit) /* Implicitly: (xcvrRxCount != 0), (xcvrRxLimit != 0). (RxIndex >= RX_LENGTH_MIN)  */
        {
            /* returns PROTOCOL_RX_CODE_WAIT_PACKET on sucessfully set of meta data */
            rxStatus = p_protocol->p_Specs->PARSE_RX_META(&p_protocol->RxReqId, &p_protocol->RxLength, p_protocol->CONFIG.P_RX_PACKET_BUFFER, p_protocol->RxIndex);
            if(rxStatus != PROTOCOL_RX_CODE_AWAIT_PACKET) { break; } /* (rxStatus == PROTOCOL_RX_CODE_WAIT_PACKET) more bytes in Xcvr Buffer, continue while loop */
        }
        else /* xcvrRxCount < xcvrRxLimit => Xcvr Rx Buffer empty, wait for Xcvr */
        {
            rxStatus = PROTOCOL_RX_CODE_AWAIT_PACKET;
            break;
        }
    }

    return rxStatus;
}

static void TxSync(Protocol_T * p_protocol, Protocol_TxSyncId_T txId)
{
    size_t txLength = 0U; /* local length. not overwrite in case of retranmission */
    if(p_protocol->p_Specs->BUILD_TX_SYNC != 0U)
    {
        p_protocol->p_Specs->BUILD_TX_SYNC(p_protocol->CONFIG.P_TX_PACKET_BUFFER, &txLength, txId);
        Xcvr_TxN(&p_protocol->Xcvr, p_protocol->CONFIG.P_TX_PACKET_BUFFER, txLength);
    }
}

/*
    ReqResp return 0, TxLength == 0, skip tx
    Error handle Xcvr TxBuffer full?
*/
static inline bool TxResp(Protocol_T * p_protocol)
{
    bool status = (p_protocol->TxLength > 0U);
    if(status == true)
    {
        p_protocol->TxPacketCount++;
        status = Xcvr_TxN(&p_protocol->Xcvr, p_protocol->CONFIG.P_TX_PACKET_BUFFER, p_protocol->TxLength);
    }
    return status;
    // return (length > 0U) ? Xcvr_TxN(&p_protocol->Xcvr, p_txBuffer, length) : false;
}

/*
    Tx Nack on Rx Error
    if client side needs to terminate nack loop. prefer implement on host side
*/
static bool ProcTxNackRxRepeat(Protocol_T * p_protocol, uint8_t * p_nackCounter, uint8_t nackRepeatMax, Protocol_TxSyncId_T nackRepeatId, Protocol_TxSyncId_T nackFinalId)
{
    bool isFinal = false;

    if((*p_nackCounter < nackRepeatMax) || (nackRepeatMax == 0U))
    {
        TxSync(p_protocol, nackRepeatId);
        *p_nackCounter++;
        isFinal = false;
    }
    else
    {
        TxSync(p_protocol, nackFinalId);
        *p_nackCounter = 0U;
        isFinal = true;
    }

    return isFinal;
}

/*
    Handle Rx Packet
    Controls Rx Packet Parser, and Timeout Timer
    if run inside isr, need sync mechanism
*/
static inline Protocol_RxCode_T ProcRxState(Protocol_T * p_protocol)
{
    Protocol_RxCode_T rxStatus = PROTOCOL_RX_CODE_AWAIT_PACKET;

    switch(p_protocol->RxState)
    {
        case PROTOCOL_RX_STATE_WAIT_BYTE_1: /* nonblocking wait state, no timer */
            if(Xcvr_RxMax(&p_protocol->Xcvr, &p_protocol->CONFIG.P_RX_PACKET_BUFFER[0U], 1U) > 0U)
            {
                /*
                    Use starting byte even if data is unencoded. first char can still be handled in separate state.
                    Unencoded still discards Rx chars until starting ID
                */
                if((p_protocol->CONFIG.P_RX_PACKET_BUFFER[0U] == p_protocol->p_Specs->RX_START_ID) || (p_protocol->p_Specs->RX_START_ID == 0x00U))
                {
                    p_protocol->RxIndex = 1U;
                    p_protocol->RxLength = 0U; /* RxPacketLength is unknown */
                    p_protocol->RxTimeStart = *p_protocol->CONFIG.P_TIMER;
                    p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_PACKET;
                }
            }
            rxStatus = PROTOCOL_RX_CODE_AWAIT_PACKET; /* Should already be set */
            break;

        case PROTOCOL_RX_STATE_WAIT_PACKET: /* nonblocking wait state, timer started */
            rxStatus = BuildRxPacket(p_protocol);
            switch(rxStatus)
            {
                case PROTOCOL_RX_CODE_AWAIT_PACKET:
                    /* check timer after reading buffer, to ensure timeout occurs when buffer is empty */
                    if(*p_protocol->CONFIG.P_TIMER - p_protocol->RxTimeStart > p_protocol->p_Specs->RX_TIMEOUT)  /* No need to check for overflow if using millis */
                    {
                        // ProcNackRepeat(p_protocol, p_protocol->p_Specs->NACK_COUNT, PROTOCOL_TX_SYNC_NACK_RX_TIMEOUT, PROTOCOL_TX_SYNC_ABORT);
                        TxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_RX_TIMEOUT);
                        p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1;
                        rxStatus = PROTOCOL_RX_CODE_ERROR_TIMEOUT;
                    }
                    break;
                case PROTOCOL_RX_CODE_PACKET_COMPLETE:
                    // p_protocol->NackCount = 0U; // p_protocol->NackDataCount = 0U;
                    p_protocol->RxPacketSuccessCount++;
                    break;

                case PROTOCOL_RX_CODE_ERROR_META:
                    /* May occur before a packet is complete. await byte 1 maybe errornous if data is unencoded. flush buffers */
                    // Xcvr_FlushRxBuffer(&p_protocol->Xcvr);
                    // ProcNackRepeat(p_protocol, p_protocol->p_Specs->NACK_COUNT, PROTOCOL_TX_SYNC_NACK_PACKET_META, PROTOCOL_TX_SYNC_ABORT);
                    TxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_PACKET_META);
                    p_protocol->RxPacketErrorCount++;
                    break;

                case PROTOCOL_RX_CODE_ERROR_DATA:
                    // ProcNackRepeat(p_protocol, p_protocol->p_Specs->NACK_COUNT, PROTOCOL_TX_SYNC_NACK_PACKET_DATA, PROTOCOL_TX_SYNC_ABORT);
                    TxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_PACKET_DATA);
                    p_protocol->RxPacketErrorCount++;
                    break;

                default: break;
            }

            /*
                Continue BuildRxPacket during ReqExt processsing
                Rx can queue out of sequence. Invalid Rx sequence until timeout buffer flush
                Rx Buffer will be overwritten, Req ensure packet data is processed, or set Rx to wait signal

                Alternatively, pause BuildRxPacket during ReqExt processing
                Incoming packet bytes wait in queue. Cannot miss packets (unless overflow)
                Cannot check for Abort without user signal, persistent wait process
            */
            if(rxStatus != PROTOCOL_RX_CODE_AWAIT_PACKET) { p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1; }
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
static inline Protocol_ReqCode_T ProcReqState(Protocol_T * p_protocol, Protocol_RxCode_T rxCode)
{
    Protocol_ReqCode_T reqStatus = PROTOCOL_REQ_CODE_PROCESS_CONTINUE;

    switch(p_protocol->ReqState)
    {
        case PROTOCOL_REQ_STATE_WAIT_RX_INITIAL: /* Initial packet containing Req Id */
            switch(rxCode)
            {
                case PROTOCOL_RX_CODE_AWAIT_PACKET: reqStatus = PROTOCOL_REQ_CODE_PROCESS_CONTINUE; break;
                case PROTOCOL_RX_CODE_PACKET_COMPLETE:
                    p_protocol->p_ReqActive = _Protocol_SearchReqTable(p_protocol->p_Specs->P_REQ_TABLE, p_protocol->p_Specs->REQ_TABLE_LENGTH, p_protocol->RxReqId);

                    if(p_protocol->p_ReqActive != 0U)
                    {
                        p_protocol->ReqTimeStart = *p_protocol->CONFIG.P_TIMER; /* Reset timer on all non error packets, feed RxLost Timer */

                        if(p_protocol->p_ReqActive->SYNC.TX_ACK == true) { TxSync(p_protocol, PROTOCOL_TX_SYNC_ACK_REQ); }

                        if(p_protocol->p_ReqActive->PROC != 0U) /* Does not invoke state machine, no loop / nonblocking wait. */
                        {
                            p_protocol->TxLength = 0U; /* in case user does not set 0 */
                            p_protocol->p_ReqActive->PROC
                            (
                                p_protocol->CONFIG.P_APP_INTERFACE,
                                p_protocol->CONFIG.P_TX_PACKET_BUFFER, &p_protocol->TxLength,
                                p_protocol->CONFIG.P_RX_PACKET_BUFFER, p_protocol->RxIndex
                            );

                            if((TxResp(p_protocol) == true) && (p_protocol->p_ReqActive->SYNC.RX_ACK == true)) /* Tx resp if TxLength > 0 */
                            {
                                p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_SYNC;
                                p_protocol->RxNackTxCount = 0U;
                                reqStatus = PROTOCOL_REQ_CODE_AWAIT_RX_SYNC;
                            }
                            else if(p_protocol->p_ReqActive->PROC_EXT == 0U)
                            {
                                reqStatus = PROTOCOL_REQ_CODE_PROCESS_COMPLETE;
                            }
                        }

                        if(p_protocol->p_ReqActive->PROC_EXT != 0U)
                        {
                            if(p_protocol->p_Specs->REQ_EXT_RESET != 0U) { p_protocol->p_Specs->REQ_EXT_RESET(p_protocol->CONFIG.P_SUBSTATE_BUFFER); }
                            // if((p_protocol->p_ReqActive->PROC == 0U) || (p_protocol->p_ReqActive->SYNC.RX_ACK == false || (p_protocol->TxLength == 0U)))
                            if(reqStatus != PROTOCOL_REQ_CODE_AWAIT_RX_SYNC) /* Is there a better way to check this? */
                            {
                                p_protocol->ReqState = PROTOCOL_REQ_STATE_PROCESS_REQ_EXT;
                                reqStatus = PROTOCOL_REQ_CODE_PROCESS_CONTINUE;
                            }
                        }
                        /* if (p_protocol->p_ReqActive->PROC == 0U) && (p_protocol->p_ReqActive->PROC_EXT == 0U) reqStatus = PROTOCOL_REQ_CODE_PROCESS_COMPLETE; */
                    }
                    else
                    {
                        TxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_REQ);
                        reqStatus = PROTOCOL_REQ_CODE_ERROR_ID;
                    }
                    break;
                case PROTOCOL_RX_CODE_ACK:      p_protocol->RxPacketErrorSync++; reqStatus = PROTOCOL_REQ_CODE_ERROR_RX_UNEXPECTED; break;
                case PROTOCOL_RX_CODE_NACK:     p_protocol->RxPacketErrorSync++; reqStatus = PROTOCOL_REQ_CODE_ERROR_RX_UNEXPECTED; break;
                case PROTOCOL_RX_CODE_ABORT:    p_protocol->RxPacketErrorSync++; reqStatus = PROTOCOL_REQ_CODE_ERROR_RX_UNEXPECTED; break;
                /* (rxCode == PROTOCOL_RX_CODE_ERROR_META) || (rxCode == PROTOCOL_RX_CODE_ERROR_DATA) || (rxCode == PROTOCOL_RX_CODE_ERROR_TIMEOUT) */
                default: reqStatus = PROTOCOL_REQ_CODE_PROCESS_COMPLETE; break;
            }
            break;

        case PROTOCOL_REQ_STATE_PROCESS_REQ_EXT:
            switch(rxCode)
            {
                case PROTOCOL_RX_CODE_AWAIT_PACKET:
                    p_protocol->TxLength = 0U;
                    reqStatus = p_protocol->p_ReqActive->PROC_EXT
                    (
                        p_protocol->CONFIG.P_SUBSTATE_BUFFER, p_protocol->CONFIG.P_APP_INTERFACE,
                        p_protocol->CONFIG.P_TX_PACKET_BUFFER, &p_protocol->TxLength,
                        p_protocol->CONFIG.P_RX_PACKET_BUFFER, p_protocol->RxIndex
                    );

                    switch(reqStatus)
                    {
                        case PROTOCOL_REQ_CODE_PROCESS_CONTINUE:
                            if((TxResp(p_protocol) == true) && (p_protocol->p_ReqActive->SYNC.RX_ACK_EXT == true))
                            {
                                p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_SYNC;
                                p_protocol->RxNackTxCount = 0U;
                            }
                            else
                            {
                                p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_EXT;
                            }
                            break;

                        case PROTOCOL_REQ_CODE_PROCESS_COMPLETE: p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_INITIAL; break;

                        /* ACK by PROC_EXT instead of on reception */
                        case PROTOCOL_REQ_CODE_PROCESS_ACK:
                            TxSync(p_protocol, PROTOCOL_TX_SYNC_ACK_REQ_EXT);
                            p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_EXT;
                            break;

                        /* NACK by PROC_EXT */
                        case PROTOCOL_REQ_CODE_PROCESS_NACK:
                            TxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_REQ_EXT);
                            p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_EXT;
                            // if(ProcTxNackRxRepeat(p_protocol, &p_protocol->TxNackRxCount, p_protocol->p_ReqActive->SYNC.NACK_REPEAT, PROTOCOL_TX_SYNC_NACK_REQ_EXT, PROTOCOL_TX_SYNC_NACK_REQ_EXT) == true)
                            // { p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_INITIAL; }
                            break;

                        case PROTOCOL_REQ_CODE_ABORT: p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_INITIAL; break;

                        case PROTOCOL_REQ_CODE_AWAIT_PROCESS:                                                                           break;     /* PROC_EXT implements nonblocking wait */
                        case PROTOCOL_REQ_CODE_AWAIT_PROCESS_EXTEND_TIMER: p_protocol->ReqTimeStart = *p_protocol->CONFIG.P_TIMER;      break;

                        /* Granular funtcion, manually initiated, if a custom sequence is needed  */
                        case PROTOCOL_REQ_CODE_AWAIT_RX_SYNC:       p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_SYNC;         break;
                        case PROTOCOL_REQ_CODE_AWAIT_RX_EXT:        p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_EXT;          break;
                        /* Tx Response and transfer control back to PROC_EXT */
                        case PROTOCOL_REQ_CODE_TX_RESPONSE:         TxResp(p_protocol);                                             break;
                        case PROTOCOL_REQ_CODE_TX_ACK:              TxSync(p_protocol, PROTOCOL_TX_SYNC_ACK_REQ_EXT);               break;
                        case PROTOCOL_REQ_CODE_TX_NACK:             TxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_REQ_EXT);              break;

                        case PROTOCOL_REQ_CODE_ERROR_RX_UNEXPECTED:   break;

                        default: break;
                    }
                    break;
                case PROTOCOL_RX_CODE_PACKET_COMPLETE:  p_protocol->RxPacketErrorSync++; reqStatus = PROTOCOL_REQ_CODE_ERROR_RX_UNEXPECTED;  break;
                case PROTOCOL_RX_CODE_ACK:              p_protocol->RxPacketErrorSync++; reqStatus = PROTOCOL_REQ_CODE_ERROR_RX_UNEXPECTED;  break;
                case PROTOCOL_RX_CODE_NACK:             p_protocol->RxPacketErrorSync++; reqStatus = PROTOCOL_REQ_CODE_ERROR_RX_UNEXPECTED;  break;
                case PROTOCOL_RX_CODE_ABORT: /* handle outside */ break;
                /* (rxCode == PROTOCOL_RX_CODE_ERROR_META) || (rxCode == PROTOCOL_RX_CODE_ERROR_DATA) || (rxCode == PROTOCOL_RX_CODE_ERROR_TIMEOUT) */
                default: p_protocol->RxPacketErrorSync++; reqStatus = PROTOCOL_REQ_CODE_ERROR_RX_UNEXPECTED;  break;
            }
            break;

        case PROTOCOL_REQ_STATE_WAIT_RX_EXT: /* Wait Rx Req Ext Continue */
            switch(rxCode)
            {
                case PROTOCOL_RX_CODE_AWAIT_PACKET: reqStatus = PROTOCOL_REQ_CODE_AWAIT_RX_EXT; break;
                case PROTOCOL_RX_CODE_PACKET_COMPLETE:
                    p_protocol->ReqTimeStart = *p_protocol->CONFIG.P_TIMER;
                    if(p_protocol->p_ReqActive->SYNC.TX_ACK_EXT == true) { TxSync(p_protocol, PROTOCOL_TX_SYNC_ACK_REQ_EXT); } /* Ack on reception, perform Rx error checking only */
                    p_protocol->ReqState = PROTOCOL_REQ_STATE_PROCESS_REQ_EXT;
                    reqStatus = PROTOCOL_REQ_CODE_PROCESS_CONTINUE;
                    break;
                case PROTOCOL_RX_CODE_ACK:  p_protocol->RxPacketErrorSync++; reqStatus = PROTOCOL_REQ_CODE_ERROR_RX_UNEXPECTED; break;
                case PROTOCOL_RX_CODE_NACK: p_protocol->RxPacketErrorSync++; reqStatus = PROTOCOL_REQ_CODE_ERROR_RX_UNEXPECTED; break;
                case PROTOCOL_RX_CODE_ABORT: break;
                /* (rxCode == PROTOCOL_RX_CODE_ERROR_META) || (rxCode == PROTOCOL_RX_CODE_ERROR_DATA) || (rxCode == PROTOCOL_RX_CODE_ERROR_TIMEOUT) */
                default: reqStatus = PROTOCOL_REQ_CODE_AWAIT_RX_EXT; break;
            }
            break;

        case PROTOCOL_REQ_STATE_WAIT_RX_SYNC:
            switch(rxCode)
            {
                case PROTOCOL_RX_CODE_AWAIT_PACKET: reqStatus = PROTOCOL_REQ_CODE_AWAIT_RX_SYNC; break;
                case PROTOCOL_RX_CODE_PACKET_COMPLETE: p_protocol->RxPacketErrorSync++; reqStatus = PROTOCOL_REQ_CODE_ERROR_RX_UNEXPECTED; break;
                case PROTOCOL_RX_CODE_ACK:
                    p_protocol->ReqTimeStart = *p_protocol->CONFIG.P_TIMER;
                    if(p_protocol->p_ReqActive->PROC_EXT != 0U) { p_protocol->ReqState = PROTOCOL_REQ_STATE_PROCESS_REQ_EXT; reqStatus = PROTOCOL_REQ_CODE_PROCESS_CONTINUE; }
                    else                                        { p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_INITIAL; reqStatus = PROTOCOL_REQ_CODE_PROCESS_COMPLETE; }
                    break;
                case PROTOCOL_RX_CODE_NACK:
                    TxResp(p_protocol); /* Retransmit Buffered Packet */
                    reqStatus = PROTOCOL_REQ_CODE_AWAIT_RX_SYNC;
                    // if(p_protocol->RxNackTxCount < p_protocol->p_ReqActive->SYNC.NACK_REPEAT)
                    // {
                    //     TxResp(p_protocol); /* Retransmit Buffered Packet */
                    //     p_protocol->RxNackTxCount++;
                    //     reqStatus = PROTOCOL_REQ_CODE_AWAIT_RX_SYNC;
                    // }
                    // else
                    // {
                    //     p_protocol->RxNackTxCount = 0U;
                    //     p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_INITIAL;
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

    /* States Common */
    if((p_protocol->ReqState == PROTOCOL_REQ_STATE_WAIT_RX_EXT) || (p_protocol->ReqState == PROTOCOL_REQ_STATE_WAIT_RX_SYNC) || (p_protocol->ReqState == PROTOCOL_REQ_STATE_PROCESS_REQ_EXT))
    {
        if(*p_protocol->CONFIG.P_TIMER - p_protocol->ReqTimeStart > p_protocol->p_Specs->REQ_TIMEOUT) /* Timer restarts on Req packet and Ack */
        {
            TxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_REQ_TIMEOUT);
            p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_INITIAL;
            reqStatus = PROTOCOL_REQ_CODE_ERROR_TIMEOUT;
        }
        else if(rxCode == PROTOCOL_RX_CODE_ABORT)
        {
            if(p_protocol->p_ReqActive->SYNC.TX_ACK_ABORT == true) { TxSync(p_protocol, PROTOCOL_TX_SYNC_ACK_ABORT); }
            p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_INITIAL;
            reqStatus = PROTOCOL_REQ_CODE_ABORT;
        }
    }

    return reqStatus;
}

/*
    Non blocking.
    Single threaded only, RxIndex not protected
    Controller Side, sequential rx req, proc, tx response
    Cmdr Side -

    Outputs RxStatus, ReqStatus - Updated every proc
*/
void Protocol_Proc(Protocol_T * p_protocol)
{
    p_protocol->RxStatus = ProcRxState(p_protocol);
    p_protocol->ReqStatus = ProcReqState(p_protocol, p_protocol->RxStatus);
    // ProcDatagram(p_protocol);  /* Enqueue Datagram for processing in parallel */
}

/*!

    @return true if WatchdogTime reached, a sucessful Req has not occured
*/
bool Protocol_CheckRxLost(Protocol_T * p_protocol)
{
    /* Check one, RxState or ReqState != STATE_INACTIVE */
    return ((p_protocol->ReqState != PROTOCOL_REQ_STATE_INACTIVE) && (*p_protocol->CONFIG.P_TIMER - p_protocol->ReqTimeStart > p_protocol->Params.WatchdogTime));
}

void Protocol_SetXcvr(Protocol_T * p_protocol, uint8_t xcvrId)
{
    p_protocol->Params.XcvrId = xcvrId;
    Xcvr_Init(&p_protocol->Xcvr, p_protocol->Params.XcvrId);
}

void Protocol_ConfigXcvrBaudRate(Protocol_T * p_protocol, uint32_t baudRate)
{
    if((baudRate != 0U) && (Xcvr_CheckIsSet(&p_protocol->Xcvr, p_protocol->Params.XcvrId) == true))
    {
        Xcvr_ConfigBaudRate(&p_protocol->Xcvr, baudRate);
    }
}

void Protocol_SetSpecs(Protocol_T * p_protocol, uint8_t p_specsId)
{
    const Protocol_Specs_T * p_specs = (p_specsId < p_protocol->CONFIG.SPECS_COUNT) ? p_protocol->CONFIG.PP_SPECS_TABLE[p_specsId] : 0U;

    if((p_specs != 0U) && (p_specs->RX_LENGTH_MAX <= p_protocol->CONFIG.PACKET_BUFFER_LENGTH))
    {
        p_protocol->p_Specs = p_specs;
        if(p_protocol->Params.BaudRate == 0U) { Protocol_ConfigXcvrBaudRate(p_protocol, p_protocol->p_Specs->BAUD_RATE_DEFAULT); }
    }
}

bool Protocol_Enable(Protocol_T * p_protocol)
{
    bool isEnable = ((Xcvr_CheckIsSet(&p_protocol->Xcvr, p_protocol->Params.XcvrId) == true) && (p_protocol->p_Specs != 0U));

    if(isEnable == true)
    {
        p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1;
        p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_INITIAL;
        p_protocol->RxIndex = 0U;
        p_protocol->TxLength = 0U;
        p_protocol->p_ReqActive = 0U;
    }

    return isEnable;
}

void Protocol_Disable(Protocol_T * p_protocol)
{
    p_protocol->RxState = PROTOCOL_RX_STATE_INACTIVE;
    p_protocol->ReqState = PROTOCOL_REQ_STATE_INACTIVE;
}



//todo
//static void ProcDatagram(Protocol_T * p_protocol)
//{
//    //* save room for 1 req packet
//    if (Port_GetTxEmpty() > Datagram_GetPacketSize(&p_protocol->Datagram) +  p_protocol->CONFIG.PACKET_BUFFER_LENGTH)
//    {
//        if (Datagram_Server_Proc(&p_protocol->Datagram))
//        {
//            PortTxString(p_protocol, p_protocol->Datagram.P_TX_BUFFER, p_protocol->Datagram.TxDataSizeActive + p_protocol->Datagram.HeaderSize);
//        }
//    }
//        //    if (Port_GetTxEmpty() > Datagram_GetPacketSize(&p_protocol->Datagram) +  p_protocol->CONFIG.PACKET_BUFFER_LENGTH)
//        //    {
//                if (Datagram_Server_Proc(&p_protocol->Datagram))
//                {
//                    PortTxString(p_protocol, p_protocol->Datagram.P_TX_BUFFER, p_protocol->Datagram.TxDataSizeActive + p_protocol->Datagram.HeaderSize);
//                }
//        //    }
//}

// void Protocol_Debug(Protocol_T * p_protocol)
// {
//     static bool toggle = true;
//     p_protocol->p_ReqActive = (toggle == true) ? &p_protocol->p_Specs->P_REQ_TABLE[0] : &p_protocol->p_Specs->P_REQ_TABLE[1];
//     toggle = !toggle;

//     if(p_protocol->p_ReqActive != 0U)
//     {
//         if(p_protocol->p_ReqActive->PROC != 0U) /* Does not invoke state machine, no loop / nonblocking wait. */
//         {
//             p_protocol->TxLength = 0U;
//             p_protocol->p_ReqActive->PROC
//             (
//                 p_protocol->CONFIG.P_APP_INTERFACE,
//                 p_protocol->CONFIG.P_TX_PACKET_BUFFER, &p_protocol->TxLength,
//                 p_protocol->CONFIG.P_RX_PACKET_BUFFER, p_protocol->RxIndex
//             );
//             TxResp(p_protocol);
//         }
//     }
// }
