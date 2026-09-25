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
    @file   Datagram.c
    @author FireSourcery
    @brief  Datagram mode - mapping codec and the ack-paced run.
*/
/******************************************************************************/
#include "Datagram.h"

#include <string.h>

/******************************************************************************/
/*!
    Mapping codec - pure over (interface, state, payload)

    Two halves: what a mapping is allowed to be, and what it packs.
    Nothing here decides how the exchange continues.
*/
/******************************************************************************/
/*!
    A mapping the run could be given: at most DATAGRAM_MAP_MAX supported entries, within one
    payload. Read from the request rather than from the state, so a refusal never installs.

    Entries are indexed through the packed request, never through a [Datagram_Entry_T *] taken
    into it - the payload sits at whatever offset the header shape leaves, and a word load off
    an odd address faults on Cortex-M0+.
*/
static uint16_t _Datagram_StatusOfMap(const Datagram_ConfigReq_T * p_req, uint8_t count, packet_size_t lengthMax)
{
    packet_size_t length = 0U;

    if (count > DATAGRAM_MAP_MAX) { return DATAGRAM_STATUS_MAP_COUNT; }

    for (uint8_t i = 0U; i < count; i++)
    {
        if (Datagram_Entry_IsSupported(p_req->Map[i]) == false) { return DATAGRAM_STATUS_NOT_MAPPABLE; }
        length += p_req->Map[i].Size;
    }

    return (length <= lengthMax) ? DATAGRAM_STATUS_OK : DATAGRAM_STATUS_LENGTH;
}

/*! Rewrite the run whole. The opening request is the only thing that may do this. */
void Datagram_Begin(Datagram_State_T * p_state, const Packet_Meta_T * p_rxMeta, const Datagram_ConfigReq_T * p_req, packet_size_t lengthMax)
{
    uint8_t mapCount = Datagram_MapCountOf(p_rxMeta->Length);

    p_state->Status   = _Datagram_StatusOfMap(p_req, mapCount, lengthMax);
    p_state->MapCount = (p_state->Status == DATAGRAM_STATUS_OK) ? mapCount : 0U;
    p_state->Cycles   = p_req->Cycles;
    p_state->Count    = 0U;
    p_state->ReqId    = p_rxMeta->Id;

    memcpy(p_state->Map, p_req->Map, p_state->MapCount * sizeof(Datagram_Entry_T));
}

/*! Entry i: Get the value and pack its bytes at its offset. */
static void _Datagram_WriteEntry(Datagram_Interface_T * p_app, const Datagram_State_T * p_state, uint8_t i, uint8_t * p_payload)
{
    Datagram_Data_T data = { .Value = p_app->GET(p_app->P_CONTEXT, (datagram_id_t)p_state->Map[i].VarId) };

    memcpy(&p_payload[_Datagram_Offset(p_state, i)], data.Bytes, p_state->Map[i].Size);
}

/*! One datagram, in map order. @return the bytes written. */
packet_size_t Datagram_Build(Datagram_Interface_T * p_app, const Datagram_State_T * p_state, void * p_payload)
{
    for (uint8_t i = 0U; i < p_state->MapCount; i++) { _Datagram_WriteEntry(p_app, p_state, i, p_payload); }
    return Datagram_Length(p_state);
}

/******************************************************************************/
/*!
    Steps - [Protocol_ProcReqResp_T]'s shape with typed payloads

    Same arity, same order, same return as the outer handler, so the switch below dispatches
    without a cast and a step could become a table row unchanged. The run is reached through
    p_xfer->p_Substate rather than taken as an argument, which is what keeps the shape.
*/
/******************************************************************************/
static inline Datagram_State_T * _Datagram_SubstateOf(void * p_substate) { return (Datagram_State_T *)p_substate; }

/*!
    Stage a status reply and decide how the run continues. Every exit that carries a status
    goes through here, so the host always learns why a run stopped rather than waiting out
    REQ_TIMEOUT. A non-OK status always closes.
*/
static Protocol_ReqCode_T _Datagram_Reply(Packet_Meta_T * p_txMeta, Datagram_ConfigResp_T * p_resp, packet_id_t id, uint16_t status, Protocol_ReqCode_T onOk)
{
    p_resp->Status = status;
    p_txMeta->Id = id;
    p_txMeta->Length = sizeof(Datagram_ConfigResp_T);
    return (status == DATAGRAM_STATUS_OK) ? onOk : PROTOCOL_REQ_DONE;
}

/*! What the run will carry - the reply's account of the mapping just installed. */
static Protocol_ReqCode_T _Datagram_ReplyMap(Packet_Xfer_T * p_xfer, Datagram_ConfigResp_T * p_resp, const Datagram_State_T * p_state, Protocol_ReqCode_T onOk)
{
    p_resp->MapCount = p_state->MapCount;
    p_resp->Length = Datagram_Length(p_state);
    return _Datagram_Reply(p_xfer->p_TxMeta, p_resp, p_state->ReqId, p_state->Status, onOk);
}

/*! Install the mapping and answer with the result. An empty mapping is a stop, and completes here. */
static Protocol_ReqCode_T Datagram_Open(Datagram_Interface_T * p_app, Packet_Xfer_T * p_xfer, const Datagram_ConfigReq_T * p_req, Datagram_ConfigResp_T * p_resp)
{
    Datagram_State_T * p_state = _Datagram_SubstateOf(p_xfer->p_Substate);

    Datagram_Begin(p_state, p_xfer->p_RxMeta, p_req, p_app->LENGTH_MAX);

    return _Datagram_ReplyMap(p_xfer, p_resp, p_state, (p_state->MapCount == 0U) ? PROTOCOL_REQ_DONE : PROTOCOL_REQ_RESPOND);
}

/*! One datagram. The tx payload is raw values here, not a status. */
static Protocol_ReqCode_T Datagram_Data(Datagram_Interface_T * p_app, Packet_Xfer_T * p_xfer, const void * p_rx, uint8_t * p_payload)
{
    (void)p_rx;   /* the pacing ack carries nothing */
    Datagram_State_T * p_state = _Datagram_SubstateOf(p_xfer->p_Substate);

    p_xfer->p_TxMeta->Id = p_app->DATA_ID;
    p_xfer->p_TxMeta->Length = Datagram_Build(p_app, p_state, p_payload);
    p_state->Count++;

    return PROTOCOL_REQ_RESPOND;
}

/*! Cycle budget spent - the closing status, describing the run that just ended. */
static Protocol_ReqCode_T Datagram_Close(Datagram_Interface_T * p_app, Packet_Xfer_T * p_xfer, const void * p_rx, Datagram_ConfigResp_T * p_resp)
{
    (void)p_app; (void)p_rx;

    return _Datagram_ReplyMap(p_xfer, p_resp, _Datagram_SubstateOf(p_xfer->p_Substate), PROTOCOL_REQ_DONE);
}

/*! An opening request too short to carry its fixed fields. */
static Protocol_ReqCode_T Datagram_Malformed(Datagram_Interface_T * p_app, Packet_Xfer_T * p_xfer, const void * p_rx, Datagram_ConfigResp_T * p_resp)
{
    (void)p_app; (void)p_rx;

    /* Begin never ran, so the run's ReqId still belongs to the previous one. The id that just
       arrived is the only label this reply can honestly carry. */
    p_resp->MapCount = 0U;
    p_resp->Length = 0U;
    return _Datagram_Reply(p_xfer->p_TxMeta, p_resp, p_xfer->p_RxMeta->Id, DATAGRAM_STATUS_MALFORMED, PROTOCOL_REQ_DONE);
}

/*! Nothing mapped and nothing owed - the exchange stays open. */
static Protocol_ReqCode_T Datagram_Idle(Datagram_Interface_T * p_app, Packet_Xfer_T * p_xfer, const void * p_rx, void * p_tx)
{
    (void)p_app; (void)p_xfer; (void)p_rx; (void)p_tx;

    return PROTOCOL_REQ_AWAIT;
}

/******************************************************************************/
/*!
    Outer handler - [Protocol_ProcReqResp_T] exactly

    Classification and dispatch, nothing else. The payloads stay void * because their type is
    a function of the pass rather than of the id, and void * converts to an object pointer
    implicitly, so no arm casts.
*/
/******************************************************************************/
/*!
    Derived from the wire and the run, never from stored progress: a bound handler is
    re-entered for every frame of the exchange, the acks that pace it included, so "something
    arrived" carries no information of its own.

    Anything that is not the opening id is a pacing frame - the ack of the status reply, or
    the ack of the previous datagram. Both mean the floor is free, so neither is told apart.
*/
static Datagram_StateId_T _Datagram_StateIdOf(Datagram_Interface_T * p_app, const Datagram_State_T * p_state, const Packet_Meta_T * p_rxMeta)
{
    if (p_rxMeta->Id == p_app->CONFIG_ID)
    {
        return (p_rxMeta->Length < DATAGRAM_CONFIG_REQ_MIN) ? DATAGRAM_STATE_ERROR : DATAGRAM_STATE_OPEN;
    }

    if (p_state->MapCount == 0U) { return DATAGRAM_STATE_IDLE; }

    return Datagram_IsSpent(p_state) ? DATAGRAM_STATE_CLOSE : DATAGRAM_STATE_DATA;
}

Protocol_ReqCode_T Datagram_Proc(Datagram_Interface_T * p_app, Packet_Xfer_T * p_xfer, const void * restrict p_rxPayload, void * restrict p_txPayload)
{
    Datagram_State_T * p_state = _Datagram_SubstateOf(p_xfer->p_Substate);

    p_state->StateId = _Datagram_StateIdOf(p_app, p_state, p_xfer->p_RxMeta);

    switch (p_state->StateId)
    {
        case DATAGRAM_STATE_OPEN:   return Datagram_Open     (p_app, p_xfer, p_rxPayload, p_txPayload);
        case DATAGRAM_STATE_DATA:   return Datagram_Data     (p_app, p_xfer, p_rxPayload, p_txPayload);
        case DATAGRAM_STATE_CLOSE:  return Datagram_Close    (p_app, p_xfer, p_rxPayload, p_txPayload);
        case DATAGRAM_STATE_ERROR:  return Datagram_Malformed(p_app, p_xfer, p_rxPayload, p_txPayload);
        case DATAGRAM_STATE_IDLE:
        default:                    return Datagram_Idle     (p_app, p_xfer, p_rxPayload, p_txPayload);
    }
}

/******************************************************************************/
/*
    Pass trace - PROTOCOL_ACK_ON_REQ, ack-paced

        CONFIG_ID frame         OPEN    mapping installed, status reply     -> RESPOND
        ack                     DATA    one datagram staged                 -> RESPOND
        ...
        ack, cycles spent       CLOSE   closing status                      -> DONE

    A refused or empty mapping answers at OPEN and returns DONE, so a stop and a rejection
    cost the host one exchange each.

    A retransmitted opening request restarts the run from its declared mapping - the intended
    answer to a lost status reply, and the reason the mapping is rewritten whole rather than
    edited.

    While a datagram is outstanding the handshake is AWAIT_ACK, so a config frame sent without
    acking first is out of sequence and nacked. The host stops a run with an abort frame, then
    opens the next one.
*/
/******************************************************************************/
