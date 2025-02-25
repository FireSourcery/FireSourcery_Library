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
    @version V0
*/
/******************************************************************************/
#include "Protocol.h"
#include <string.h>

void Protocol_Init(Protocol_T * p_protocol)
{
    if(p_protocol->CONST.P_CONFIG != NULL)
    {
        memcpy(&p_protocol->Config, p_protocol->CONST.P_CONFIG, sizeof(Protocol_Config_T));

        Xcvr_Init(&p_protocol->Xcvr, p_protocol->Config.XcvrId);
        Protocol_ConfigXcvrBaudRate(p_protocol, p_protocol->Config.BaudRate);
        Protocol_SetSpecs(p_protocol, p_protocol->Config.SpecsId);
        if (p_protocol->Config.IsEnableOnInit == true) { Protocol_Enable(p_protocol); } else { Protocol_Disable(p_protocol); }
    }
    else
    {
        Protocol_Disable(p_protocol);
    }
}

/*
    Receive into P_RX_PACKET_BUFFER and run PARSE_RX_META for RxMeta.Length and ReqCode / Rx completion
    Packet is complete => Req, ReqExt or Sync, or Error
*/
static inline Protocol_RxCode_T CaptureRx(Protocol_T * p_protocol)
{
    Protocol_RxCode_T rxStatus = PROTOCOL_RX_CODE_AWAIT_PACKET;
    uint8_t nextRxIndex;
    // uint8_t xcvrRxLimit; /* PARSE_RX_META after this number is received */
    // uint8_t xcvrRxCount;

    /* Loop to empty Xcvr Rx buffer. Check for completetion, per Rx 1 byte, during unknown length, or up to known length */
    while (p_protocol->RxIndex < p_protocol->p_Specs->RX_LENGTH_MAX) /* RX_LENGTH_MAX <= CONFIG.PACKET_BUFFER_LENGTH */
    {
        if(rxStatus != PROTOCOL_RX_CODE_AWAIT_PACKET) { break; } /* Packet is complete, break */

        /*
            Set xcvrRxLimit for PARSE_RX_META. Prevent reading bytes from the following packet.
            RxMeta.Length unknown => Check 1 byte or a known const value, until RxMeta.Length is known
            RxMeta.Length known => Check Rx remaining
        */
        if(p_protocol->RxMeta.Length > 0U)  /* PacketLength is known. */
        {
            if(p_protocol->RxIndex < p_protocol->RxMeta.Length) { nextRxIndex = p_protocol->RxMeta.Length; }
            else { rxStatus = PROTOCOL_RX_CODE_ERROR_META; break; }
            /* (RxIndex == RxMeta.Length) => (xcvrRxLimit == 0), when rxStatus == PROTOCOL_RX_CODE_WAIT_PACKET erroneously i.e. received full packet without completion status */
        }
        /* PacketLength is unkown */
        else if(p_protocol->RxIndex < p_protocol->p_Specs->RX_LENGTH_MIN)   { nextRxIndex = p_protocol->p_Specs->RX_LENGTH_MIN; }
        /* RX_LENGTH_INDEX > RX_LENGTH_MIN. Check for length determined by another property */
        // else if(p_protocol->RxIndex < p_protocol->p_Specs->RX_REQ_ID_INDEX) { nextRxIndex = p_protocol->p_Specs->RX_REQ_ID_INDEX + 1U; }
        // else if(p_protocol->RxIndex < p_protocol->p_Specs->RX_LENGTH_INDEX) { nextRxIndex = p_protocol->p_Specs->RX_LENGTH_INDEX + 1U; }
        else                                                                { nextRxIndex = p_protocol->RxIndex + 1U; }

        if (nextRxIndex > p_protocol->p_Specs->RX_LENGTH_MAX) { rxStatus = PROTOCOL_RX_CODE_ERROR_META; break; }
        /* Copy from Xcvr buffer to Protocol buffer, up to xcvrRxLimit */
        p_protocol->RxIndex += Xcvr_RxMax(&p_protocol->Xcvr, &p_protocol->CONST.P_RX_PACKET_BUFFER[p_protocol->RxIndex], nextRxIndex - p_protocol->RxIndex);

        if(p_protocol->RxIndex == nextRxIndex) /* (xcvrRxCount == xcvrRxLimit) */
        {
            /* Implicitly: (xcvrRxCount != 0), (xcvrRxLimit != 0). (RxIndex >= RX_LENGTH_MIN)  */
            /* returns PROTOCOL_RX_CODE_AWAIT_PACKET on successful set of meta data *//* more bytes in Xcvr Buffer, continue while loop */
            rxStatus = p_protocol->p_Specs->PARSE_RX_META(&p_protocol->RxMeta, p_protocol->CONST.P_RX_PACKET_BUFFER, p_protocol->RxIndex);
        }
        else /* xcvrRxCount < xcvrRxLimit => Xcvr Rx Buffer empty, wait for Xcvr */
        {
            break;  /*  rxStatus == PROTOCOL_RX_CODE_AWAIT_PACKET; */
        }
    }

    return rxStatus;
}

static void TxSync(Protocol_T * p_protocol, Protocol_TxSyncId_T txId)
{
    protocol_size_t txLength = 0U; /* local length. not overwrite in case of retransmission */
    if(p_protocol->p_Specs->BUILD_TX_SYNC != 0U)
    {
        p_protocol->p_Specs->BUILD_TX_SYNC(p_protocol->CONST.P_TX_PACKET_BUFFER, &txLength, txId);
        Xcvr_TxN(&p_protocol->Xcvr, p_protocol->CONST.P_TX_PACKET_BUFFER, txLength);
    }
}

/*
    ReqResp return 0, TxLength == 0, skip tx
    Error handle Xcvr TxBuffer full?
*/
static inline bool TxResp(Protocol_T * p_protocol)
{
    return (p_protocol->TxLength > 0U) ? Xcvr_TxN(&p_protocol->Xcvr, p_protocol->CONST.P_TX_PACKET_BUFFER, p_protocol->TxLength) : false;
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

static inline bool IsRxLost(Protocol_T * p_protocol)
{
    return (*p_protocol->CONST.P_TIMER - p_protocol->ReqTimeStart > p_protocol->Config.WatchdogTimeout);
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
            if(Xcvr_RxMax(&p_protocol->Xcvr, &p_protocol->CONST.P_RX_PACKET_BUFFER[0U], 1U) > 0U)
            {
                /*
                    Use starting byte even if data is unencoded. first char can still be handled in separate state.
                    Unencoded still discards Rx chars until starting ID
                */
                if((p_protocol->CONST.P_RX_PACKET_BUFFER[0U] == p_protocol->p_Specs->RX_START_ID) || (p_protocol->p_Specs->RX_START_ID == 0x00U))
                {
                    p_protocol->RxIndex = 1U;
                    p_protocol->RxMeta.Length = 0U; /* RxMeta.Length is unknown */
                    p_protocol->RxTimeStart = *p_protocol->CONST.P_TIMER;
                    p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_PACKET;
                }
            }
            rxStatus = PROTOCOL_RX_CODE_AWAIT_PACKET; /* Should already be set */
            break;

        case PROTOCOL_RX_STATE_WAIT_PACKET: /* nonblocking wait state, timer started */
            rxStatus = CaptureRx(p_protocol);
            switch(rxStatus)
            {
                case PROTOCOL_RX_CODE_AWAIT_PACKET:
                    /* check timer after reading buffer, to ensure timeout occurs when buffer is empty */
                    if(*p_protocol->CONST.P_TIMER - p_protocol->RxTimeStart > p_protocol->p_Specs->RX_TIMEOUT)  /* No need to check for overflow if using millis */
                    {
                        // ProcNackRepeat(p_protocol, p_protocol->p_Specs->NACK_COUNT, PROTOCOL_TX_SYNC_NACK_RX_TIMEOUT, PROTOCOL_TX_SYNC_ABORT);
                        // Xcvr_FlushRxBuffer(&p_protocol->Xcvr);
                        TxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_RX_TIMEOUT);
                        p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1;
                        rxStatus = PROTOCOL_RX_CODE_ERROR_TIMEOUT;
                    }
                    break;

                case PROTOCOL_RX_CODE_PACKET_COMPLETE:
                    p_protocol->NackCount = 0U; // p_protocol->NackDataCount = 0U;
                    // p_protocol->RxPacketSuccessCount++;
                    break;

                case PROTOCOL_RX_CODE_ERROR_META:
                    /* May occur before a packet is complete. await byte 1 maybe erroneous if data is unencoded. flush buffers */
                    // Xcvr_FlushRxBuffer(&p_protocol->Xcvr);
                    // ProcNackRepeat(p_protocol, p_protocol->p_Specs->NACK_COUNT, PROTOCOL_TX_SYNC_NACK_PACKET_META, PROTOCOL_TX_SYNC_ABORT);
                    TxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_PACKET_META);
                    // p_protocol->RxPacketErrorCount++;
                    break;

                case PROTOCOL_RX_CODE_ERROR_DATA:
                    // ProcNackRepeat(p_protocol, p_protocol->p_Specs->NACK_COUNT, PROTOCOL_TX_SYNC_NACK_PACKET_DATA, PROTOCOL_TX_SYNC_ABORT);
                    TxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_PACKET_DATA);
                    // p_protocol->RxPacketErrorCount++;
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
    Protocol_ReqCode_T reqStatus = PROTOCOL_REQ_CODE_TX_CONTINUE;
    const Protocol_ReqContext_T reqContext =
    {
        .p_SubState = p_protocol->CONST.P_REQ_STATE_BUFFER,
        .p_RxPacket = p_protocol->CONST.P_RX_PACKET_BUFFER,
        .p_TxPacket = p_protocol->CONST.P_TX_PACKET_BUFFER,
        .p_RxMeta = &p_protocol->RxMeta,
        .p_TxSize = &p_protocol->TxLength,
        .p_SubStateIndex = &p_protocol->ReqSubStateIndex,
    };

    switch(p_protocol->ReqState)
    {
        case PROTOCOL_REQ_STATE_WAIT_RX_ID: /* Initial packet containing Req Id */
            switch(rxCode)
            {
                case PROTOCOL_RX_CODE_AWAIT_PACKET: reqStatus = PROTOCOL_REQ_CODE_TX_CONTINUE; break;
                case PROTOCOL_RX_CODE_PACKET_COMPLETE:
                    p_protocol->p_ReqActive = _Protocol_SearchReqTable(p_protocol->p_Specs->P_REQ_TABLE, p_protocol->p_Specs->REQ_TABLE_LENGTH, p_protocol->RxMeta.ReqId);

                    if(p_protocol->p_ReqActive != NULL)
                    {
                        p_protocol->ReqTimeStart = *p_protocol->CONST.P_TIMER; /* Reset timer on all non error packets, feed RxLost Timer */

                        if(p_protocol->p_ReqActive->SYNC.TX_ACK == true) { TxSync(p_protocol, PROTOCOL_TX_SYNC_ACK_REQ); }

                        // disallow both types? PROC, PROC_EXT
                        if(p_protocol->p_ReqActive->PROC != NULL) /* Does not invoke state machine, no loop / nonblocking wait. */
                        {
                            // p_protocol->TxLength = 0U; /* in case it is not set by the user function */
                            p_protocol->TxLength = p_protocol->p_ReqActive->PROC(p_protocol->CONST.P_APP_INTERFACE, p_protocol->CONST.P_TX_PACKET_BUFFER, p_protocol->CONST.P_RX_PACKET_BUFFER);
                            // p_protocol->p_Specs->BUILD_TX_HEADER(p_protocol->CONST.P_TX_PACKET_BUFFER, &p_protocol->TxLength, txId, payloadLength);
                            if((TxResp(p_protocol) == true) && (p_protocol->p_ReqActive->SYNC.RX_ACK == true)) /* Tx resp if TxLength > 0 */
                            {
                                p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_SYNC;
                                p_protocol->RxNackTxCount = 0U;
                                reqStatus = PROTOCOL_REQ_CODE_AWAIT_RX_SYNC;
                            }
                            else if(p_protocol->p_ReqActive->PROC_EXT == 0U)
                            {
                                reqStatus = PROTOCOL_REQ_CODE_PROCESS_COMPLETE;
                                /* p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_ID; */
                            }
                        } //todo exclusive or determine sync

                        if((p_protocol->p_ReqActive->PROC_EXT != NULL) && (reqStatus != PROTOCOL_REQ_CODE_AWAIT_RX_SYNC))
                            /* ((p_protocol->p_ReqActive->PROC == 0U) || (p_protocol->p_ReqActive->SYNC.RX_ACK == false || (p_protocol->TxLength == 0U))) */
                        {
                            // if(p_protocol->p_Specs->REQ_EXT_RESET != 0U) { p_protocol->p_Specs->REQ_EXT_RESET(p_protocol->CONST.P_REQ_STATE_BUFFER); }
                            p_protocol->ReqSubStateIndex = 0U;
                            p_protocol->ReqState = PROTOCOL_REQ_STATE_PROCESS_REQ_EXT;
                            reqStatus = PROTOCOL_REQ_CODE_TX_CONTINUE;
                        }
                        /* if (p_protocol->p_ReqActive->PROC == 0U) && (p_protocol->p_ReqActive->PROC_EXT == 0U) reqStatus = PROTOCOL_REQ_CODE_PROCESS_COMPLETE; */
                    }
                    else
                    {
                        TxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_REQ);
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
            reqStatus = p_protocol->p_ReqActive->PROC_EXT(p_protocol->CONST.P_APP_INTERFACE, &reqContext);

            switch(reqStatus)
            {

                case PROTOCOL_REQ_CODE_TX_CONTINUE: /* Tx then continue */
                    if((TxResp(p_protocol) == true) && (p_protocol->p_ReqActive->SYNC.RX_ACK_EXT == true))
                    {
                        p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_SYNC;
                        p_protocol->RxNackTxCount = 0U;
                    }
                    else
                    {
                        p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_CONTINUE; //override next rx id
                    }
                    break;

                case PROTOCOL_REQ_CODE_AWAIT_RX_CONTINUE:   p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_CONTINUE;     break;
                case PROTOCOL_REQ_CODE_AWAIT_RX_SYNC:       p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_SYNC;         break;

                //split complete_clear_state and continue with id
                // state saved
                // case PROTOCOL_REQ_CODE_PROCESS_RETURN: p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_ID;
                //     break;
                // need duplicate sync state, or track index/flag intenrally
                case PROTOCOL_REQ_CODE_PROCESS_COMPLETE:
                    p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_ID;
                    p_protocol->ReqSubStateIndex = 0U;
                    // if(p_protocol->p_Specs->REQ_EXT_RESET != 0U) { p_protocol->p_Specs->REQ_EXT_RESET(p_protocol->CONST.P_REQ_STATE_BUFFER); }
                    break;


                case PROTOCOL_REQ_CODE_ABORT: p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_ID; break;

                /* Granular function, manually initiated, if a custom sequence is needed  */
                /* Tx Response and transfer control back to PROC_EXT */
                // alternatively directly call helper
                case PROTOCOL_REQ_CODE_TX_RESPONSE:         TxResp(p_protocol);                                             break;
                case PROTOCOL_REQ_CODE_TX_ACK:              TxSync(p_protocol, PROTOCOL_TX_SYNC_ACK_REQ_EXT);               break;
                case PROTOCOL_REQ_CODE_TX_NACK:             TxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_REQ_EXT);              break;

                /* ACK by PROC_EXT, after proc, instead of on reception */
                case PROTOCOL_REQ_CODE_PROCESS_ACK:
                    TxSync(p_protocol, PROTOCOL_TX_SYNC_ACK_REQ_EXT);
                    p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_CONTINUE;
                    break;

                /* NACK by PROC_EXT */
                case PROTOCOL_REQ_CODE_PROCESS_NACK:
                    TxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_REQ_EXT);
                    p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_CONTINUE;
                    // if(ProcTxNackRxRepeat(p_protocol, &p_protocol->TxNackRxCount, p_protocol->p_ReqActive->SYNC.NACK_REPEAT, PROTOCOL_TX_SYNC_NACK_REQ_EXT, PROTOCOL_TX_SYNC_NACK_REQ_EXT) == true)
                    // { p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_ID; }
                    break;

                // // noop
                // /* PROC_EXT implements nonblocking wait, copy packet buffers*/
                // case PROTOCOL_REQ_CODE_AWAIT_PROCESS:                                                                           break;
                // case PROTOCOL_REQ_CODE_AWAIT_PROCESS_EXTEND_TIMER: p_protocol->ReqTimeStart = *p_protocol->CONST.P_TIMER;      break;
                case PROTOCOL_REQ_CODE_ERROR_RX_UNEXPECTED:   break;

                default: break;
            }

            switch(rxCode)
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
            switch(rxCode)
            {
                case PROTOCOL_RX_CODE_AWAIT_PACKET: reqStatus = PROTOCOL_REQ_CODE_AWAIT_RX_CONTINUE; break;
                case PROTOCOL_RX_CODE_PACKET_COMPLETE:
                    p_protocol->ReqTimeStart = *p_protocol->CONST.P_TIMER;
                    if(p_protocol->p_ReqActive->SYNC.TX_ACK_EXT == true) { TxSync(p_protocol, PROTOCOL_TX_SYNC_ACK_REQ_EXT); } /* Ack on reception, perform Rx error checking only */
                    p_protocol->ReqState = PROTOCOL_REQ_STATE_PROCESS_REQ_EXT;
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
            switch(rxCode)
            {
                case PROTOCOL_RX_CODE_AWAIT_PACKET: reqStatus = PROTOCOL_REQ_CODE_AWAIT_RX_SYNC; break;
                case PROTOCOL_RX_CODE_PACKET_COMPLETE: reqStatus = PROTOCOL_REQ_CODE_ERROR_RX_UNEXPECTED; break;
                case PROTOCOL_RX_CODE_ACK:
                    p_protocol->ReqTimeStart = *p_protocol->CONST.P_TIMER;
                    if(p_protocol->p_ReqActive->PROC_EXT != NULL)
                    {
                        //flag for final?/* continue loop/ goto reqid split */
                        p_protocol->ReqState = PROTOCOL_REQ_STATE_PROCESS_REQ_EXT;
                        reqStatus = PROTOCOL_REQ_CODE_TX_CONTINUE;
                    }
                    else
                    {
                        p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_ID;
                        reqStatus = PROTOCOL_REQ_CODE_PROCESS_COMPLETE;
                    }
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
                    //     p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_ID;
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

    // if (reqStatus == PROTOCOL_REQ_CODE_ERROR_RX_UNEXPECTED) { p_protocol->RxPacketErrorSync++; }
    /* States Common */
    if((p_protocol->ReqState == PROTOCOL_REQ_STATE_WAIT_RX_CONTINUE) || (p_protocol->ReqState == PROTOCOL_REQ_STATE_WAIT_RX_SYNC) || (p_protocol->ReqState == PROTOCOL_REQ_STATE_PROCESS_REQ_EXT))
    {
        if(*p_protocol->CONST.P_TIMER - p_protocol->ReqTimeStart > p_protocol->p_Specs->REQ_TIMEOUT) /* Timer restarts on Req packet and Ack */
        {
            TxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_REQ_TIMEOUT);
            p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_ID;
            reqStatus = PROTOCOL_REQ_CODE_ERROR_TIMEOUT;
        }
        else if(rxCode == PROTOCOL_RX_CODE_ABORT)
        {
            if(p_protocol->p_ReqActive->SYNC.TX_ACK_ABORT == true) { TxSync(p_protocol, PROTOCOL_TX_SYNC_ACK_ABORT); }
            p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_ID;
            reqStatus = PROTOCOL_REQ_CODE_ABORT;
        }
    }

    return reqStatus;
}

/*! @return pointer to Req */
const Protocol_Req_T * _Protocol_SearchReqTable(Protocol_Req_T * p_reqTable, size_t tableLength, protocol_req_id_t id)
{
    const Protocol_Req_T * p_req = NULL;
    for(uint8_t iChar = 0U; iChar < tableLength; iChar++) { if(p_reqTable[iChar].ID == id) { p_req = &p_reqTable[iChar]; break; } }
    return p_req;
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
    // ProcTxAsync(p_protocol);
    // ProcDatagram(p_protocol);  /* Enqueue Datagram for processing in parallel */
}


void Protocol_SetXcvr(Protocol_T * p_protocol, uint8_t xcvrId)
{
    p_protocol->Config.XcvrId = xcvrId;
    Xcvr_Init(&p_protocol->Xcvr, p_protocol->Config.XcvrId);
}

void Protocol_ConfigXcvrBaudRate(Protocol_T * p_protocol, uint32_t baudRate)
{
    if((baudRate != 0U) && (Xcvr_CheckIsSet(&p_protocol->Xcvr, p_protocol->Config.XcvrId) == true))
    {
        Xcvr_ConfigBaudRate(&p_protocol->Xcvr, baudRate);
    }
}

void Protocol_SetSpecs(Protocol_T * p_protocol, uint8_t p_specsId)
{
    const Protocol_Specs_T * p_specs = (p_specsId < p_protocol->CONST.SPECS_COUNT) ? p_protocol->CONST.PP_SPECS_TABLE[p_specsId] : NULL;

    if((p_specs != NULL) && (p_specs->RX_LENGTH_MAX <= p_protocol->CONST.PACKET_BUFFER_LENGTH))
    {
        p_protocol->p_Specs = p_specs;
        // if(p_protocol->Config.BaudRate == 0U) { Protocol_ConfigXcvrBaudRate(p_protocol, p_protocol->p_Specs->BAUD_RATE_DEFAULT); }
    }
}

bool Protocol_Enable(Protocol_T * p_protocol)
{
    bool isEnable = ((Xcvr_CheckIsSet(&p_protocol->Xcvr, p_protocol->Config.XcvrId) == true) && (p_protocol->p_Specs != 0U));

    if(isEnable == true)
    {
        p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1;
        p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_ID;
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
//    if (Port_GetTxEmpty() > Datagram_GetPacketSize(&p_protocol->Datagram) +  p_protocol->CONST.PACKET_BUFFER_LENGTH)
//    {
//        if (Datagram_Server_Proc(&p_protocol->Datagram))
//        {
//            PortTxString(p_protocol, p_protocol->Datagram.P_TX_BUFFER, p_protocol->Datagram.TxDataSizeActive + p_protocol->Datagram.HeaderSize);
//        }
//    }
//        //    if (Port_GetTxEmpty() > Datagram_GetPacketSize(&p_protocol->Datagram) +  p_protocol->CONST.PACKET_BUFFER_LENGTH)
//        //    {
//                if (Datagram_Server_Proc(&p_protocol->Datagram))
//                {
//                    PortTxString(p_protocol, p_protocol->Datagram.P_TX_BUFFER, p_protocol->Datagram.TxDataSizeActive + p_protocol->Datagram.HeaderSize);
//                }
//        //    }
//}

int32_t _Protocol_ConfigId_Get(const Protocol_T * p_protocol, Protocol_ConfigId_T id)
{
    int32_t value = 0;
    switch(id)
    {
        case PROTOCOL_CONFIG_XCVR_ID:         value = p_protocol->Config.XcvrId;            break;
        case PROTOCOL_CONFIG_SPECS_ID:        value = p_protocol->Config.SpecsId;           break;
        case PROTOCOL_CONFIG_WATCHDOG_TIME:   value = p_protocol->Config.WatchdogTimeout;   break;
        case PROTOCOL_CONFIG_BAUD_RATE:       value = p_protocol->Config.BaudRate;          break;
        case PROTOCOL_CONFIG_IS_ENABLED:      value = p_protocol->Config.IsEnableOnInit;    break;
    }
    return value;
}

int32_t Protocol_ConfigId_Get(const Protocol_T * p_protocol, Protocol_ConfigId_T id)
{
    return (p_protocol != NULL) ? _Protocol_ConfigId_Get(p_protocol, id) : 0;
}

void _Protocol_ConfigId_Set(Protocol_T * p_protocol, Protocol_ConfigId_T id, uint32_t value)
{
    switch(id)
    {
        case PROTOCOL_CONFIG_XCVR_ID:         p_protocol->Config.XcvrId = value;            break;
        case PROTOCOL_CONFIG_SPECS_ID:        p_protocol->Config.SpecsId = value;           break;
        case PROTOCOL_CONFIG_WATCHDOG_TIME:   p_protocol->Config.WatchdogTimeout = value;   break;
        case PROTOCOL_CONFIG_BAUD_RATE:       p_protocol->Config.BaudRate = value;          break;
        case PROTOCOL_CONFIG_IS_ENABLED:      p_protocol->Config.IsEnableOnInit = value;    break;
    }
}

void Protocol_ConfigId_Set(Protocol_T * p_protocol, Protocol_ConfigId_T id, uint32_t value)
{
    if (p_protocol != NULL) { _Protocol_ConfigId_Set(p_protocol, id, value); }
}