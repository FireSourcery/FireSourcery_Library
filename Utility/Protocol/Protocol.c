/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2021 FireSourcery / The Firebrand Forge Inc

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
        if(p_protocol->Params.IsEnableOnInit == true)   { Protocol_Enable(p_protocol); }
        else                                            { Protocol_Disable(p_protocol); }
    }
    else
    {
        Protocol_Disable(p_protocol);
    }
}

/*
    Receive into P_RX_PACKET_BUFFER and run PARSE_RX_META for RxLength and ReqCode / Rx completion
*/
static inline Protocol_RxCode_T BuildRxPacket(Protocol_T * p_protocol)
{
    Protocol_RxCode_T rxStatus = PROTOCOL_RX_CODE_WAIT_PACKET;
    uint8_t xcvrRxLimit; /* PARSE_RX_META after this number is received */
    uint8_t xcvrRxLength;

    while(p_protocol->RxIndex < p_protocol->p_Specs->RX_LENGTH_MAX) /* p_Specs->RX_LENGTH_MAX < CONFIG.PACKET_BUFFER_LENGTH */
        // if(p_protocol->RxIndex < p_protocol->p_Specs->RX_REQ_INDEX)
    {
        /*
            Set xcvrRxLimit for PARSE_RX_META. Prevent reading byte from following packet
            RxLength unknown => Check 1 byte or upto RX_LENGTH_MIN at a time, until RxLength is known
            RxLength known => Check Rx remaining
        */
        if(p_protocol->RxIndex < p_protocol->p_Specs->RX_LENGTH_MIN) { xcvrRxLimit = p_protocol->p_Specs->RX_LENGTH_MIN - p_protocol->RxIndex; }
        else if(p_protocol->RxLength == 0U) { xcvrRxLimit = 1U; }
        // if(p_protocol->RxIndex < p_protocol->p_Specs->RX_LENGTH_INDEX)
        else { xcvrRxLimit = p_protocol->RxLength - p_protocol->RxIndex; }

        xcvrRxLength = Xcvr_RxMax(&p_protocol->Xcvr, &p_protocol->CONFIG.P_RX_PACKET_BUFFER[p_protocol->RxIndex], xcvrRxLimit); /* Rx up to xcvrRxLimit */
        p_protocol->RxIndex += xcvrRxLength; /* Always update state, used for error checking */

        if(xcvrRxLength == xcvrRxLimit) /* Implicitly (p_protocol->RxIndex >= p_protocol->p_Specs->RX_LENGTH_MIN) && (xcvrRxLength != 0). xcvrRxLimit will not be 0U */
        {
            rxStatus = p_protocol->p_Specs->PARSE_RX_META(&p_protocol->ReqIdActive, &p_protocol->RxLength, p_protocol->CONFIG.P_RX_PACKET_BUFFER, p_protocol->RxIndex);

            if(rxStatus != PROTOCOL_RX_CODE_WAIT_PACKET) { break; } /* Packet is complete => Req, ReqExt or Sync, or Error */

            if(p_protocol->RxLength != 0U)
            {
                /* RxLength Set Error or received full packet without completion */
                /* Implicitly checks if(xcvrRxLimit == 0U), p_protocol->RxLength - p_protocol->RxIndex */
                if((p_protocol->RxLength > p_protocol->p_Specs->RX_LENGTH_MAX) || (p_protocol->RxIndex >= p_protocol->RxLength))
                {
                    rxStatus = PROTOCOL_RX_CODE_SYNC_ERROR;
                    p_protocol->RxState = PROTOCOL_RX_STATE_INACTIVE;
                    break;
                }
            }
        }
        else /* xcvrRxLength < xcvrRxLimit, Xcvr Rx Buffer empty, wait for Xcvr */
        {
            break;
        }
    }
    /* if(p_protocol->RxIndex > p_protocol->p_Specs->RX_LENGTH_MAX) should not occur. wait for time out */

    return rxStatus;
}

static void TxSync(Protocol_T * p_protocol, Protocol_TxSyncId_T txId)
{
    if(p_protocol->p_Specs->BUILD_TX_SYNC != 0U)
    {
        p_protocol->p_Specs->BUILD_TX_SYNC(p_protocol->CONFIG.P_TX_PACKET_BUFFER, &p_protocol->TxLength, txId);
        Xcvr_TxN(&p_protocol->Xcvr, p_protocol->CONFIG.P_TX_PACKET_BUFFER, p_protocol->TxLength);
    }
}

/*
    Handle Rx Packet
    Controls Rx Packet Parser, and Timeout Timer
    if run inside isr, need sync mechanism
*/
static inline Protocol_RxCode_T ProcRxState(Protocol_T * p_protocol)
{
    Protocol_RxCode_T rxStatus = PROTOCOL_RX_CODE_WAIT_PACKET;

    switch(p_protocol->RxState)
    {
        case PROTOCOL_RX_STATE_AWAIT_BYTE_1: /* nonblocking wait state, no timer */
            if(Xcvr_RxMax(&p_protocol->Xcvr, &p_protocol->CONFIG.P_RX_PACKET_BUFFER[0U], 1U) == true)
            {
                /*
                    Use starting byte even if data is unencoded. first char can still be handled in separate state.
                    Unencoded still discards Rx chars until starting ID
                */
                if((p_protocol->CONFIG.P_RX_PACKET_BUFFER[0U] == p_protocol->p_Specs->RX_START_ID) || (p_protocol->p_Specs->RX_START_ID == 0x00U))
                {
                    p_protocol->RxIndex = 1U;
                    p_protocol->RxLength = 0U; /* RxLength is unknown */
                    p_protocol->RxTimeStart = *p_protocol->CONFIG.P_TIMER;
                    p_protocol->RxState = PROTOCOL_RX_STATE_AWAIT_PACKET;
                    // rxStatus = PROTOCOL_RX_CODE_WAIT_PACKET; /* Should already be set */
                }
            }
            break;

        case PROTOCOL_RX_STATE_AWAIT_PACKET: /* nonblocking wait state, timer started */
            if(*p_protocol->CONFIG.P_TIMER - p_protocol->RxTimeStart < p_protocol->p_Specs->RX_TIMEOUT)  /* No need to check for overflow if using millis */
            {
                rxStatus = BuildRxPacket(p_protocol);

                /*
                    Continue BuildRxPacket during ReqExt processsing
                    Rx can queue out of sequence. Invalid Rx sequence until timeout buffer flush
                    Rx Buffer will be overwritten, Req ensure packet data is processed, or set Rx to wait signal

                    Alternatively, pause BuildRxPacket during ReqExt processing
                    Incoming packet bytes wait in queue. Cannot miss packets (unless overflow)
                    Cannot check for Abort without user signal, persistent wait process
                */
                if(rxStatus != PROTOCOL_RX_CODE_WAIT_PACKET) { p_protocol->RxState = PROTOCOL_RX_STATE_AWAIT_BYTE_1; }

                if(rxStatus == PROTOCOL_RX_CODE_PACKET_ERROR)
                {
                    //pass error to req?
                    if(p_protocol->p_Specs->NACK_COUNT != 0U)
                    {
                        if(p_protocol->NackCount < p_protocol->p_Specs->NACK_COUNT)
                        {
                            TxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_PACKET_ERROR);
                            p_protocol->NackCount++; //tx nack count
                            // p_protocol->RxTimeStart = *p_protocol->CONFIG.P_TIMER;
                            // rxStatus = PROTOCOL_RX_CODE_PACKET_ERROR;
                        }
                        else /* wait for timeout */
                        {
                            // TxSync(p_protocol, PROTOCOL_TX_SYNC_ERROR_RESYNC);
                            // rxStatus = PROTOCOL_RX_CODE_ERROR_RESYNC;
                            // rxStatus = PROTOCOL_RX_CODE_PACKET_ERROR;
                            p_protocol->NackCount = 0U;
                        }
                    }
                    p_protocol->RxPacketErrorCount++;

                }
                else if(rxStatus != PROTOCOL_RX_CODE_WAIT_PACKET) /* !PROTOCOL_RX_CODE_WAIT_PACKET && !RX_CODE_PACKET_ERROR */
                {
                    p_protocol->NackCount = 0U;
                    p_protocol->RxPacketSuccessCount++;
                }
            }
            else
            {
                rxStatus = PROTOCOL_RX_CODE_PACKET_TIMEOUT;

                if(p_protocol->p_Specs->NACK_COUNT != 0U)
                {
                    if(p_protocol->NackCount < p_protocol->p_Specs->NACK_COUNT)
                    {
                        TxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_RX_TIMEOUT);
                        p_protocol->NackCount++;
                        // p_protocol->RxTimeStart = *p_protocol->CONFIG.P_TIMER;
                    }
                    else /* wait for timeout */
                    {
                        TxSync(p_protocol, PROTOCOL_TX_SYNC_ABORT);
                        p_protocol->NackCount = 0U;
                    }
                }
                // TxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_RX_TIMEOUT);
                // rxStatus = PROTOCOL_RX_CODE_PACKET_TIMEOUT;
                // p_protocol->NackCount = 0U;
            }
            break;

            // case PROTOCOL_RX_STATE_WAIT_REQ_SIGNAL:
            //     p_protocol->RxState = PROTOCOL_RX_STATE_AWAIT_BYTE_1;
            //     rxStatus = PROTOCOL_RX_CODE_WAIT_PACKET;
            //     // p_protocol->NackCount = 0U;
            //     break;

        case PROTOCOL_RX_STATE_INACTIVE:
            break;

        default: break;
    }

    return rxStatus;
}

/*
    ReqExtResp return 0, skip tx
    Error handle Xcvr TxBuffer full?
*/
static inline bool TxReqResp(Protocol_T * p_protocol, const uint8_t * p_txBuffer, uint8_t length)
{
    bool status = (length > 0U);
    if(status == true)
    {
        p_protocol->TxPacketCount++;
        Xcvr_TxN(&p_protocol->Xcvr, p_txBuffer, length);
    }
    // return (length > 0U) ? Xcvr_TxN(&p_protocol->Xcvr, p_txBuffer, length) : false;
    return status;
}

/*
    Common PROTOCOL_REQ_STATE_WAIT_RX_SYNC PROTOCOL_REQ_STATE_WAIT_RX_SYNC_FINAL
*/
static inline void ProcReqWaitRxSync(Protocol_T * p_protocol, Protocol_RxCode_T rxCode)
{
    if(rxCode == PROTOCOL_RX_CODE_ACK)
    {
        p_protocol->NackCount = 0U;
        p_protocol->ReqTimeStart = *p_protocol->CONFIG.P_TIMER;
    }
    else if(rxCode == PROTOCOL_RX_CODE_NACK)
    {
        if(p_protocol->NackCount < p_protocol->p_ReqActive->SYNC.NACK_REPEAT)
        {
            TxReqResp(p_protocol, p_protocol->CONFIG.P_TX_PACKET_BUFFER, p_protocol->TxLength); /* Retransmit Packet */
            p_protocol->NackCount++;
            p_protocol->ReqTimeStart = *p_protocol->CONFIG.P_TIMER;
        }
        else
        {
            p_protocol->NackCount = 0U;
            p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_COMPLETE_ID;
        }
    }
    //handle unexpected req packet
}


// static void ResetReqState(Protocol_T * p_protocol)
// {
//     p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_COMPLETE_ID;
//     // if(p_protocol->p_Specs->REQ_EXT_RESET != 0U) { p_protocol->p_Specs->REQ_EXT_RESET(p_protocol->CONFIG.P_SUBSTATE_BUFFER); }
//     p_protocol->NackCount = 0U;
// }
// static void ProcReqTxSyncNack(Protocol_T * p_protocol, Protocol_RxCode_T nackId)
// {
//     if(p_protocol->NackCount < p_protocol->p_ReqActive->SYNC.NACK_REPEAT)
//     {
//         p_protocol->NackCount++;
//         TxSync(p_protocol, nackId);
//     }
// }


/*
    Handle user Req/Cmd function
    ReqTimeStart resets for all expected behaviors, in sequence packets
*/
static inline Protocol_ReqCode_T ProcReqState(Protocol_T * p_protocol, Protocol_RxCode_T rxCode)
{
    Protocol_ReqCode_T reqStatus = PROTOCOL_REQ_CODE_WAIT_PROCESS;

    /* States Common */
    if((p_protocol->ReqState != PROTOCOL_REQ_STATE_INACTIVE) && (p_protocol->ReqState != PROTOCOL_REQ_STATE_WAIT_RX_COMPLETE_ID))
    {
        if(*p_protocol->CONFIG.P_TIMER - p_protocol->ReqTimeStart > p_protocol->p_Specs->REQ_TIMEOUT)
        {
            p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_COMPLETE_ID;
            TxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_REQ_TIMEOUT);
        }
        else if(rxCode == PROTOCOL_RX_CODE_ABORT)
        {
            p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_COMPLETE_ID;
            p_protocol->ReqTimeStart = *p_protocol->CONFIG.P_TIMER; /* Reset timer for feed RxLost Timer */
        }
    }

    switch(p_protocol->ReqState)
    {
        case PROTOCOL_REQ_STATE_WAIT_RX_COMPLETE_ID:
            if(rxCode == PROTOCOL_RX_CODE_PACKET_COMPLETE)
            {
                p_protocol->p_ReqActive = _Protocol_SearchReqTable(p_protocol->p_Specs->P_REQ_TABLE, p_protocol->p_Specs->REQ_TABLE_LENGTH, p_protocol->ReqIdActive);

                if(p_protocol->p_ReqActive != 0U)
                {
                    if(p_protocol->p_ReqActive->SYNC.TX_ACK == true) { TxSync(p_protocol, PROTOCOL_TX_SYNC_ACK_REQ_ID); }

                    if(p_protocol->p_ReqActive->PROC != 0U) /* Does not invoke state machine, no loop / nonblocking wait. */
                    {
                        p_protocol->TxLength = 0U;
                        p_protocol->p_ReqActive->PROC
                        (
                            p_protocol->CONFIG.P_APP_INTERFACE,
                            p_protocol->CONFIG.P_TX_PACKET_BUFFER, &p_protocol->TxLength,
                            p_protocol->CONFIG.P_RX_PACKET_BUFFER, p_protocol->RxIndex
                        );
                        TxReqResp(p_protocol, p_protocol->CONFIG.P_TX_PACKET_BUFFER, p_protocol->TxLength);
                    }

                    if(p_protocol->p_ReqActive->PROC_EXT != 0U)
                    {
                        if(p_protocol->p_Specs->REQ_EXT_RESET != 0U) { p_protocol->p_Specs->REQ_EXT_RESET(p_protocol->CONFIG.P_SUBSTATE_BUFFER); }
                        p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_PROCESS;
                    }
                    else if(p_protocol->p_ReqActive->SYNC.RX_ACK == true)
                    {
                        p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_SYNC_FINAL;
                    }

                    //  p_protocol->NackCount = 0U;  //check
                    p_protocol->ReqTimeStart = *p_protocol->CONFIG.P_TIMER; /* Reset timer on all non error packets, feed RxLost Timer */
                }
                else
                {
                    TxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_REQ_ID);
                }
            }
            else //move PROTOCOL_RX_CODE_PACKET_ERROR?
            {
                //handle out of sequence packet / handle special context
            }
            break;

        case PROTOCOL_REQ_STATE_WAIT_RX_COMPLETE_EXT: /* Wait Rx Req Ext Continue */
            if(rxCode == PROTOCOL_RX_CODE_PACKET_COMPLETE)
            {
                p_protocol->ReqTimeStart = *p_protocol->CONFIG.P_TIMER;
                p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_PROCESS;
            }
            else //move PROTOCOL_RX_CODE_PACKET_ERROR?
            {
                //handle out of sequence packet
            }
            break;

        case PROTOCOL_REQ_STATE_WAIT_RX_SYNC:
            ProcReqWaitRxSync(p_protocol, rxCode);
            if(rxCode == PROTOCOL_RX_CODE_ACK) { p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_PROCESS; }
            break;

        case PROTOCOL_REQ_STATE_WAIT_RX_SYNC_FINAL:
            ProcReqWaitRxSync(p_protocol, rxCode);
            if(rxCode == PROTOCOL_RX_CODE_ACK) { p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_COMPLETE_ID; }
            break;

        case PROTOCOL_REQ_STATE_WAIT_PROCESS:
            // if(rxCode != PROTOCOL_RX_CODE_WAIT_PACKET)
            // {
            //     OutOfSequencePacket++
            // }
            p_protocol->TxLength = 0U; /* in case user does not set 0 */

            reqStatus = p_protocol->p_ReqActive->PROC_EXT
            (
                p_protocol->CONFIG.P_SUBSTATE_BUFFER, p_protocol->CONFIG.P_APP_INTERFACE,
                p_protocol->CONFIG.P_TX_PACKET_BUFFER, &p_protocol->TxLength,
                p_protocol->CONFIG.P_RX_PACKET_BUFFER, p_protocol->RxIndex
            );

            TxReqResp(p_protocol, p_protocol->CONFIG.P_TX_PACKET_BUFFER, p_protocol->TxLength); /* always tx resp if TxLength > 0 */

            switch(reqStatus)
            {
                case PROTOCOL_REQ_CODE_WAIT_PROCESS:                break;     /* Timer ticking */
                case PROTOCOL_REQ_CODE_WAIT_PROCESS_EXTEND_TIMER:   break;     /* Timer reset */
                case PROTOCOL_REQ_CODE_PROCESS_COMPLETE:
                    if(p_protocol->p_ReqActive->SYNC.RX_ACK == true) { p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_SYNC_FINAL; }
                    else { p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_COMPLETE_ID; }
                    break;
                case PROTOCOL_REQ_CODE_AWAIT_RX_SYNC:       p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_SYNC;                break;
                case PROTOCOL_REQ_CODE_AWAIT_RX_REQ_EXT:    p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_COMPLETE_EXT;        break;
                    /*
                        Separate tx resp state vs always proc tx on user return txlength > 0
                        User will will have to manage more return codes
                        ?must save txlength in case of retransmit on nack?
                    */
                    // case PROTOCOL_REQ_CODE_TX_RESPONSE:     TxReqResp(p_protocol, p_protocol->CONFIG.P_TX_PACKET_BUFFER, p_protocol->TxLength);        break;
                case PROTOCOL_REQ_CODE_TX_ACK:  TxSync(p_protocol, PROTOCOL_TX_SYNC_ACK_REQ_EXT);                                  break;
                case PROTOCOL_REQ_CODE_TX_NACK: /* Shared Nack */
                    if(p_protocol->NackCount < p_protocol->p_ReqActive->SYNC.NACK_REPEAT)
                    {
                        TxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_REQ_EXT);
                        p_protocol->ReqTimeStart = *p_protocol->CONFIG.P_TIMER;
                        p_protocol->NackCount++;
                    }
                    else
                    {
                        p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_COMPLETE_ID;
                        p_protocol->NackCount = 0U;
                    }
                    break;

                    // case PROTOCOL_REQ_CODE_ERROR: break; reqid mismatch?
                    // case PROTOCOL_REQ_CODE_ERROR_TX_NACK: break;

                default: break;
            }

            if(reqStatus != PROTOCOL_REQ_CODE_WAIT_PROCESS) { p_protocol->ReqTimeStart = *p_protocol->CONFIG.P_TIMER; }
            break;

        case PROTOCOL_REQ_STATE_INACTIVE: break;

        default: break;

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
    //        ProcDatagram(p_protocol);  /* Enqueue Datagram for processing in parallel */
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
        p_protocol->RxState = PROTOCOL_RX_STATE_AWAIT_BYTE_1;
        p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_COMPLETE_ID;
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
//             TxReqResp(p_protocol, p_protocol->CONFIG.P_TX_PACKET_BUFFER, p_protocol->TxLength);
//         }
//     }
// }
