// #pragma once

// /******************************************************************************/
// /*!
//     @file   Protocol_DataMode.h
//     @author FireSourcery
//     @brief  Stateful bulk transfer - Read and Write - over one resumable handler each.
// */
// /******************************************************************************/
// #include "../Protocol_Request.h"
// #include "../Packet.h"
// #include "Math/math_general.h"

// #include <stdint.h>
// #include <stdbool.h>
// #include <stddef.h>
// #include <string.h>

// /******************************************************************************/
// /*
//     Segmentation above the packet layer. The opening request declares an address and a size;
//     the bytes follow as a run of chunk frames, paced in whichever direction the data flows.

//         Read    device streams to host, paced by the host's acks
//         Write   host streams to device, paced by its own data frames

//     Both are one row in the request table, bound once by the opening request and re-entered
//     for every frame that arrives until the cursor runs out.

//     HOW A PASS IS CLASSIFIED

//     A bound handler is re-entered for every frame of the exchange, the acks that pace it
//     included, so "something arrived" carries no information. The engine keeps no step counter
//     on the handler's behalf - Packet_Xfer_T has no step field, and P_REQ_CONTEXT is never
//     cleared - so a stored "am I open yet" flag would start each transfer holding whatever the
//     previous one left behind.

//     The pass is therefore derived from the wire and the cursor, never from stored progress:

//         RxMeta.Id == READ_ID / WRITE_ID   the opening request. Sets the cursor.
//         RxMeta.Id == DATA_ID              a chunk. Only meaningful inbound, i.e. on a Write.
//         anything else                     a pacing ack. Nothing to absorb.

//     Two properties follow. A transfer cannot inherit stale progress, because the opening
//     request is recognised by its id and rewrites the cursor whole. And a host that reopens
//     mid-transfer - because our status reply was lost - restarts cleanly rather than resuming
//     at an index the two ends disagree about.

//     Sub-state is the cursor alone: { Address; Size; Count; ReqId; }
// */
// /******************************************************************************/
// /*
//     LAYERING

//     Three tiers, so that the shape of a step is independent of how steps are reached.

//         1  Outer handler    Protocol_ProcReqResp_T exactly - void * payloads, because one
//                             function serves every pass and the payload type differs per pass.
//                             Classifies, then dispatches. No transfer logic of its own.

//         2  Step handlers    The Protocol_ProcReqResp_T SHAPE, with typed payloads. Same
//                             arity, same argument order, same return - only the pointer types
//                             are narrowed to what that step actually reads and writes.

//         3  Utilities        Ordinary functions over (interface, cursor, packet). Not handler
//                             shaped, because they are not steps: Protocol_DataMode_ChunkOf,
//                             _Begin, _Reply, _ReadChunk, _WriteChunk.

//     Why tier 2 keeps the shape rather than taking whatever arguments it needs.

//     A step handler differs from Protocol_ProcReqResp_T only in pointer types, and void *
//     converts to an object pointer implicitly - so the outer handler dispatches with no casts,
//     and every arm of its switch is the same argument list. That makes the switch mechanical,
//     and it makes the promotion below a substitution rather than a rewrite:

//         typedef Protocol_ReqCode_T (*Protocol_DataMode_Step_T)(void *, Packet_Xfer_T *, const void * restrict, void * restrict);

//         static const Protocol_DataMode_Step_T READ_STEPS[] =
//         {
//             [PROTOCOL_DATA_MODE_STATE_IDLE]  = (Protocol_DataMode_Step_T)Protocol_DataMode_Idle,
//             [PROTOCOL_DATA_MODE_STATE_OPEN]  = (Protocol_DataMode_Step_T)Protocol_DataMode_Open,
//             [PROTOCOL_DATA_MODE_STATE_DATA]  = (Protocol_DataMode_Step_T)Protocol_DataMode_ReadData,
//             ...
//         };

//     That is the same cast PROTOCOL_REQ already performs on a row's PROC, and it is sound for
//     the same reason: the parameters differ only in what an object pointer points at.

//     A step reads the cursor from p_xfer->p_Substate rather than taking it as an argument.
//     That is the price of the shape, and it is the right price - the cursor is the handler's
//     own storage, so a step that took it as a parameter could not be reached through a table.
// */
// /******************************************************************************/

// /******************************************************************************/
// /*!
//     Wire payloads
// */
// /******************************************************************************/
// typedef struct PACKET_PACKED Protocol_DataMode_Req { uint32_t Address; uint32_t Size; uint32_t Config; } Protocol_DataMode_Req_T;
// typedef struct PACKET_PACKED Protocol_DataMode_Resp { uint16_t Status; }                                 Protocol_DataMode_Resp_T;

// #define PROTOCOL_DATA_MODE_STATUS_OK (0U)   /* Application status. Non-zero ends the transfer. */

// /*! Engine-side refusals, above anything the application ops can return. */
// #define PROTOCOL_DATA_MODE_STATUS_MALFORMED (0xE001U)   /* Opening request shorter than its payload type */
// #define PROTOCOL_DATA_MODE_STATUS_OVERRUN   (0xE002U)   /* A chunk past the declared size */

// /******************************************************************************/
// /*!
//     Application binding
// */
// /******************************************************************************/
// /*!
//     Memory operations. Each returns an application status, 0 for success.
//     OPEN prepares a transfer - bounds check, unlock, erase, arm a cursor - and is called for
//     both directions, with Config carrying whatever the opening request declared.
// */
// typedef const struct Protocol_DataMode_Ops
// {
//     uint16_t (*OPEN) (void * p_module, uintptr_t address, size_t size, uint32_t config);
//     uint16_t (*READ) (void * p_module, uintptr_t address, size_t size, void * p_dest);
//     uint16_t (*WRITE)(void * p_module, uintptr_t address, const void * p_src, size_t size);
// }
// Protocol_DataMode_Ops_T;

// /*!
//     One interface serves both directions, so the two rows share a context.

//     The three ids are how a pass is classified, which is why they are here rather than in the
//     codec: they are application vocabulary, and the codec's control ids are a different set -
//     an ack is recognised here only as "not one of mine".
// */
// typedef const struct Protocol_DataMode_Interface
// {
//     Protocol_DataMode_Ops_T * P_OPS;
//     void * P_MODULE;            /* Passed back to every op */
//     packet_id_t READ_ID;        /* Opens a read, and labels its status replies */
//     packet_id_t WRITE_ID;       /* Opens a write, and labels its status replies */
//     packet_id_t DATA_ID;        /* Carries a raw chunk in either direction */
//     packet_size_t CHUNK_MAX;    /* Bounded by the format's payload capacity */
// }
// Protocol_DataMode_Interface_T;

// /******************************************************************************/
// /*!
//     Sub-state - the cursor, held in the socket's P_REQ_CONTEXT buffer.
// */
// /******************************************************************************/
// /*!
//     What a pass is doing. Shared by both transfers so the two flows read side by side, and
//     the index a step table would be reached by.

//     Derived per pass from the inbound packet and the cursor - never an input. The copy kept
//     in Protocol_DataMode_State_T is for inspection only; deriving it is what keeps a stored
//     step counter from drifting out of sync with the transfer it describes.
// */
// typedef enum Protocol_DataModeStateId
// {
//     PROTOCOL_DATA_MODE_STATE_IDLE,      /* Nothing to absorb, nothing owed */
//     PROTOCOL_DATA_MODE_STATE_OPEN,      /* Opening request - set up the transfer, reply with status */
//     PROTOCOL_DATA_MODE_STATE_DATA,      /* Move one chunk, the transfer continues */
//     PROTOCOL_DATA_MODE_STATE_CLOSE,     /* Cursor spent, or the last chunk - reply and close */
//     PROTOCOL_DATA_MODE_STATE_ERROR,     /* Malformed for this transfer - report and close */
// }
// Protocol_DataMode_StateId_T;

// typedef struct Protocol_DataMode_State
// {
//     uintptr_t Address;      /* Transfer base */
//     size_t Size;            /* Total bytes declared by the opening request */
//     size_t Count;           /* Bytes transferred so far */

//     packet_id_t ReqId;      /* Echoed on the status replies. Taken from the opening request. */
//     uint16_t Status;        /* Last op result, for inspection */
//     Protocol_DataMode_StateId_T StateId;
// }
// Protocol_DataMode_State_T;

// /*! The cursor a step runs against. Every step reaches it this way, so the shape holds. */
// static inline Protocol_DataMode_State_T * _DataMode_StateOf(void * p_substate) { return (Protocol_DataMode_State_T *)p_substate; }

// /******************************************************************************/
// /*!
//     Tier 3 - utilities

//     Ordinary functions over (interface, cursor, packet). Nothing here decides what the
//     exchange does next, so nothing here returns a Protocol_ReqCode_T except _Reply, which
//     exists precisely to turn a status into one.
// */
// /******************************************************************************/

// // /*! Bytes still owed by the transfer. */
// // static inline size_t Protocol_DataMode_RemainingOf(const Protocol_DataMode_State_T * p_state) { return p_state->Size - p_state->Count; }

// // /*! Bytes left, clamped to one chunk. */
// // static inline packet_size_t Protocol_DataMode_ChunkOf(Protocol_DataMode_Interface_T * p_app, const Protocol_DataMode_State_T * p_state)
// // {
// //     return (packet_size_t)math_min(Protocol_DataMode_RemainingOf(p_state), (size_t)p_app->CHUNK_MAX);
// // }


// /*! Move one inbound chunk into memory and advance. Shared by the DATA and CLOSE steps. */
// static inline uint16_t Protocol_DataMode_WriteChunk(Protocol_DataMode_Interface_T * p_app, Protocol_DataMode_State_T * p_state, const Packet_Meta_T * p_rxMeta, const void * p_chunk)
// {
//     p_state->Status = p_app->P_OPS->WRITE(p_app->P_MODULE, p_state->Address + p_state->Count, p_chunk, p_rxMeta->Length);

//     if (p_state->Status == PROTOCOL_DATA_MODE_STATUS_OK) { p_state->Count += p_rxMeta->Length; }

//     return p_state->Status;
// }

// /*! Stage one outbound chunk and advance. Labels the tx frame as a chunk, not as a status. */
// static inline uint16_t Protocol_DataMode_ReadChunk(Protocol_DataMode_Interface_T * p_app, Protocol_DataMode_State_T * p_state, Packet_Meta_T * p_txMeta, void * p_chunk)
// {
//     packet_size_t chunk = math_min(p_state->Size - p_state->Count, p_app->CHUNK_MAX);

//     p_state->Status = p_app->P_OPS->READ(p_app->P_MODULE, p_state->Address + p_state->Count, chunk, p_chunk);

//     if (p_state->Status == PROTOCOL_DATA_MODE_STATUS_OK)
//     {
//         p_state->Count += chunk;
//         p_txMeta->Id = p_app->DATA_ID;
//         p_txMeta->Length = chunk;
//     }

//     return p_state->Status;
// }


// /*!
//     Stage a status reply and decide how the exchange continues.

//     Every exit that carries a status goes through here, so the remote always learns why a
//     transfer stopped rather than waiting out REQ_TIMEOUT. A non-OK status always closes.

//     convenience wrapper for sending a status reply.
// */
// static inline Protocol_ReqCode_T Protocol_DataMode_Reply(const Protocol_DataMode_State_T * p_state, Packet_Meta_T * p_txMeta, Protocol_DataMode_Resp_T * p_resp, uint16_t status, Protocol_ReqCode_T onOk)
// {
//     p_resp->Status = status;
//     p_txMeta->Id = p_state->ReqId;
//     p_txMeta->Length = sizeof(Protocol_DataMode_Resp_T);
//     return (status == PROTOCOL_DATA_MODE_STATUS_OK) ? onOk : PROTOCOL_REQ_DONE;
// }


// /******************************************************************************/
// /*!
//     Stage derivation

//     One function per transfer, stating in a single place how a pass is classified. Pure over
//     (interface, cursor, packet), so each is testable on its own and no step body has to
//     re-derive anything. This is what a step table would index by.
// */
// /******************************************************************************/
// /*! An opening request is only an opening request if it carries one. */
// static inline Protocol_DataMode_StateId_T _DataMode_OpenStateOf(const Packet_Xfer_T * p_xfer)
// {
//     return (p_xfer->p_RxMeta->Length < sizeof(Protocol_DataMode_Req_T)) ? PROTOCOL_DATA_MODE_STATE_ERROR : PROTOCOL_DATA_MODE_STATE_OPEN;
// }



// /******************************************************************************/
// /*!
//     Tier 2 - step handlers

//     Protocol_ProcReqResp_T's shape with typed payloads: (context, xfer, rx, tx) in, a
//     Protocol_ReqCode_T out. A step names what it actually reads and writes in its own
//     signature, so nothing inside one casts a payload.

//     Unused payload parameters are kept rather than dropped - the shape is the point, and a
//     step whose signature no longer matches cannot be reached through a table.
// */
// /******************************************************************************/
// /*! Rewrite the cursor whole. The opening request is the only thing that may do this. */
// static inline void Protocol_DataMode_Begin(Protocol_DataMode_State_T * p_state, const Packet_Meta_T * p_rxMeta, const Protocol_DataMode_Req_T * p_req)
// {
//     p_state->Address = (uintptr_t)p_req->Address;
//     p_state->Size    = (size_t)p_req->Size;
//     p_state->Count   = 0U;
//     p_state->ReqId   = p_rxMeta->Id;
//     p_state->Status  = PROTOCOL_DATA_MODE_STATUS_OK;
// }

// /*!
//     Set up a transfer and answer with the result. Shared by both directions.

//     A zero-size transfer is legal and completes here - there is nothing to stream, so the
//     opening status is also the closing one.
// */
// static inline Protocol_ReqCode_T Protocol_DataMode_Open(Protocol_DataMode_Interface_T * p_app, Packet_Xfer_T * p_xfer, const Protocol_DataMode_Req_T * p_req, Protocol_DataMode_Resp_T * p_resp)
// {
//     Protocol_DataMode_State_T * p_state = _DataMode_StateOf(p_xfer->p_Substate);

//     Protocol_DataMode_Begin(p_state, p_xfer->p_RxMeta, p_req);
//     p_state->Status = p_app->P_OPS->OPEN(p_app->P_MODULE, p_state->Address, p_state->Size, p_req->Config);

//     return Protocol_DataMode_Reply(p_state, p_xfer, p_resp, p_state->Status, (p_state->Size == 0U) ? PROTOCOL_REQ_DONE : PROTOCOL_REQ_RESPOND);
// }

// /*!
//     Read, continuing - stage the next chunk.

//     The tx payload is a raw chunk here, not a status, which is why it is typed uint8_t * and
//     the id is set by _ReadChunk rather than by _Reply. A fault swaps the frame for a status
//     and closes, so the remote learns why the stream stopped.
// */
// static inline Protocol_ReqCode_T Protocol_DataMode_ReadData(Protocol_DataMode_Interface_T * p_app, Packet_Xfer_T * p_xfer, const void * p_rx, uint8_t * p_chunk)
// {
//     (void)p_rx;   /* the pacing ack carries nothing */
//     Protocol_DataMode_State_T * p_state = _DataMode_StateOf(p_xfer->p_Substate);

//     return (Protocol_DataMode_ReadChunk(p_app, p_state, p_xfer->p_TxMeta, p_chunk) == PROTOCOL_DATA_MODE_STATUS_OK)
//          ? PROTOCOL_REQ_RESPOND
//          : Protocol_DataMode_Reply(p_state, p_xfer, (Protocol_DataMode_Resp_T *)p_chunk, p_state->Status, PROTOCOL_REQ_DONE);
// }

// /*! Read, cursor spent - the closing status. */
// static inline Protocol_ReqCode_T Protocol_DataMode_ReadClose(Protocol_DataMode_Interface_T * p_app, Packet_Xfer_T * p_xfer, const void * p_rxPayload, Protocol_DataMode_Resp_T * p_resp)
// {
//     (void)p_app; (void)p_rxPayload;

//     return Protocol_DataMode_Reply(_DataMode_StateOf(p_xfer->p_Substate), p_xfer, p_resp, PROTOCOL_DATA_MODE_STATUS_OK, PROTOCOL_REQ_DONE);
// }

// /*!
//     Write, continuing - absorb one chunk, silently.

//     ACCEPT is the silent path: the engine's ack is the whole reply, so nothing is staged.
//     A media fault reports and ends.
// */
// static inline Protocol_ReqCode_T Protocol_DataMode_WriteData(Protocol_DataMode_Interface_T * p_app, Packet_Xfer_T * p_xfer, const uint8_t * p_chunk, Protocol_DataMode_Resp_T * p_resp)
// {
//     Protocol_DataMode_State_T * p_state = _DataMode_StateOf(p_xfer->p_Substate);

//     return (Protocol_DataMode_WriteChunk(p_app, p_state, p_xfer->p_RxMeta, p_chunk) == PROTOCOL_DATA_MODE_STATUS_OK)
//          ? PROTOCOL_REQ_ACCEPT
//          : Protocol_DataMode_Reply(p_state, p_xfer, p_resp, p_state->Status, PROTOCOL_REQ_DONE);
// }

// /*! Write, last chunk - absorb it, then answer with the transfer's outcome. */
// static inline Protocol_ReqCode_T Protocol_DataMode_WriteClose(Protocol_DataMode_Interface_T * p_app, Packet_Xfer_T * p_xfer, const uint8_t * p_chunk, Protocol_DataMode_Resp_T * p_resp)
// {
//     Protocol_DataMode_State_T * p_state = _DataMode_StateOf(p_xfer->p_Substate);
//     uint16_t status = Protocol_DataMode_WriteChunk(p_app, p_state, p_xfer->p_RxMeta, p_chunk);

//     return Protocol_DataMode_Reply(p_state, p_xfer, p_resp, status, PROTOCOL_REQ_DONE);
// }

// /*! An opening request too short to be one. */
// static inline Protocol_ReqCode_T Protocol_DataMode_Malformed(Protocol_DataMode_Interface_T * p_app, Packet_Xfer_T * p_xfer, const void * p_rxPayload, Protocol_DataMode_Resp_T * p_resp)
// {
//     (void)p_app; (void)p_rxPayload;

//     return Protocol_DataMode_Reply(_DataMode_StateOf(p_xfer->p_Substate), p_xfer, p_resp, PROTOCOL_DATA_MODE_STATUS_MALFORMED, PROTOCOL_REQ_DONE);
// }

// /*! A chunk past the size the opening request declared. */
// static inline Protocol_ReqCode_T Protocol_DataMode_Overrun(Protocol_DataMode_Interface_T * p_app, Packet_Xfer_T * p_xfer, const void * p_rxPayload, Protocol_DataMode_Resp_T * p_resp)
// {
//     (void)p_app; (void)p_rxPayload;

//     return Protocol_DataMode_Reply(_DataMode_StateOf(p_xfer->p_Substate), p_xfer, p_resp, PROTOCOL_DATA_MODE_STATUS_OVERRUN, PROTOCOL_REQ_DONE);
// }

// /*! Nothing to absorb and nothing owed - the exchange stays open. */
// static inline Protocol_ReqCode_T Protocol_DataMode_Idle(Protocol_DataMode_Interface_T * p_app, Packet_Xfer_T * p_xfer, const void * p_rxPayload, void * p_txPayload)
// {
//     (void)p_app; (void)p_xfer; (void)p_rxPayload; (void)p_txPayload;

//     return PROTOCOL_REQ_AWAIT;
// }

// /******************************************************************************/
// /*!
//     Tier 1 - outer handlers

//     Protocol_ProcReqResp_T exactly. One function serves every pass of a transfer, and the
//     payload type differs per pass, so the payloads stay void * here and are narrowed at the
//     step boundary. void * converts to an object pointer implicitly, so no arm casts.

//     Classification and dispatch, nothing else. Every arm is the same argument list, which is
//     what makes the switch replaceable by a table without touching a step.
// */
// /******************************************************************************/



// /*!
//     Read - the opening request, then one chunk per pacing ack until the cursor runs out.

//     Anything that is not the opening id is a pacing frame: the ack of the status reply, or
//     the ack of the previous chunk. Both mean the same thing here - the floor is free, send
//     the next chunk - so neither needs to be told apart from the other.
// */
// static inline Protocol_DataMode_StateId_T _DataModeRead_StateOf(Protocol_DataMode_Interface_T * p_app, const Protocol_DataMode_State_T * p_state, const Packet_Meta_T * p_rxMeta)
// {
//     if (p_rxMeta->Id == p_app->READ_ID) { return _DataMode_OpenStateOf(p_rxMeta); }

//     return (p_state->Count < p_state->Size) ? PROTOCOL_DATA_MODE_STATE_DATA : PROTOCOL_DATA_MODE_STATE_CLOSE;
// }

// /*!
//     Read - device streams memory to the host.

//     Paced by the host's acks: every ack frees the floor and pulls the next chunk. Register
//     with PROTOCOL_ACK_ON_REQ so each chunk is acked and the next is pulled by that ack.
// */
// static inline Protocol_ReqCode_T Protocol_DataMode_Read(Protocol_DataMode_Interface_T * p_app, Packet_Xfer_T * p_xfer, const void * restrict p_rxPayload, void * restrict p_txPayload)
// {
//     Protocol_DataMode_State_T * p_state = _DataMode_StateOf(p_xfer->p_Substate);

//     p_state->StateId = _DataModeRead_StateOf(p_app, p_state, p_xfer);

//     switch (p_state->StateId)
//     {
//         case PROTOCOL_DATA_MODE_STATE_OPEN:     return Protocol_DataMode_Open     (p_app, p_xfer, p_rxPayload, p_txPayload);
//         case PROTOCOL_DATA_MODE_STATE_DATA:     return Protocol_DataMode_ReadData (p_app, p_xfer, p_rxPayload, p_txPayload);
//         case PROTOCOL_DATA_MODE_STATE_CLOSE:    return Protocol_DataMode_ReadClose(p_app, p_xfer, p_rxPayload, p_txPayload);
//         case PROTOCOL_DATA_MODE_STATE_ERROR:    return Protocol_DataMode_Malformed(p_app, p_xfer, p_rxPayload, p_txPayload);
//         case PROTOCOL_DATA_MODE_STATE_IDLE:
//         default:                                return Protocol_DataMode_Idle     (p_app, p_xfer, p_rxPayload, p_txPayload);
//     }
// }

// /*!
//     Write - the opening request, then a chunk per inbound DATA frame.

//     The ack answering the opening reply re-enters the handler with neither id; there is
//     nothing to absorb on that pass, so it classifies as IDLE rather than as a chunk.
// */
// static inline Protocol_DataMode_StateId_T _DataModeWrite_StateOf(Protocol_DataMode_Interface_T * p_app, const Protocol_DataMode_State_T * p_state, const Packet_Meta_T * p_rxMeta)
// {
//     if (p_rxMeta->Id == p_app->WRITE_ID) { return _DataMode_OpenStateOf(p_rxMeta); }

//     if (p_rxMeta->Id != p_app->DATA_ID) { return PROTOCOL_DATA_MODE_STATE_IDLE; }

//     /* A chunk past the declared size is the host misbehaving, not a memory fault. */
//     if ((size_t)p_rxMeta->Length > (p_state->Size - p_state->Count)) { return PROTOCOL_DATA_MODE_STATE_ERROR; }

//     return ((p_state->Count + p_rxMeta->Length) < p_state->Size) ? PROTOCOL_DATA_MODE_STATE_DATA : PROTOCOL_DATA_MODE_STATE_CLOSE;
// }

// /*!
//     Write - host streams memory to the device.

//     Paced by the host's data packets. Only the opening request and the last chunk earn a
//     reply; the rest are answered by ACCEPT, which is the engine's ack and nothing more.
// */
// static inline Protocol_ReqCode_T Protocol_DataMode_Write(Protocol_DataMode_Interface_T * p_app, Packet_Xfer_T * p_xfer, const void * restrict p_rxPayload, void * restrict p_txPayload)
// {
//     Protocol_DataMode_State_T * p_state = _DataMode_StateOf(p_xfer->p_Substate);

//     p_state->StateId = _DataModeWrite_StateOf(p_app, p_state, p_xfer);

//     switch (p_state->StateId)
//     {
//         case PROTOCOL_DATA_MODE_STATE_OPEN:     return Protocol_DataMode_Open      (p_app, p_xfer, p_rxPayload, p_txPayload);
//         case PROTOCOL_DATA_MODE_STATE_DATA:     return Protocol_DataMode_WriteData (p_app, p_xfer, p_rxPayload, p_txPayload);
//         case PROTOCOL_DATA_MODE_STATE_CLOSE:    return Protocol_DataMode_WriteClose(p_app, p_xfer, p_rxPayload, p_txPayload);
//         case PROTOCOL_DATA_MODE_STATE_ERROR:    return Protocol_DataMode_Overrun   (p_app, p_xfer, p_rxPayload, p_txPayload);
//         case PROTOCOL_DATA_MODE_STATE_IDLE:
//         default:                                return Protocol_DataMode_Idle      (p_app, p_xfer, p_rxPayload, p_txPayload);
//     }
// }

// /******************************************************************************/
// /*
//     Pass traces

//     Read - PROTOCOL_ACK_ON_REQ, ack-paced

//         READ_ID frame           OPEN    cursor set, OPEN(), status reply    -> RESPOND
//         ack                     DATA    one chunk staged                    -> RESPOND
//         ...
//         ack, cursor spent       CLOSE   closing status                      -> DONE

//     Write - data-paced

//         WRITE_ID frame          OPEN    cursor set, OPEN(), status reply    -> RESPOND
//         ack of that reply       IDLE    nothing to absorb                   -> AWAIT
//         DATA_ID frame           DATA    chunk written, silent               -> ACCEPT
//         ...
//         DATA_ID frame, last     CLOSE   chunk written, status reply         -> DONE

//     A fault in either direction stages the status and returns DONE, so the remote always
//     learns why a transfer stopped rather than waiting out REQ_TIMEOUT.

//     A retransmitted opening request restarts the transfer from its declared address. That is
//     the intended answer to a lost status reply - the alternative, resuming at an index the two
//     ends no longer agree on, is the one outcome a bulk transfer must not produce.

//     Note that OPEN is one step shared by both directions while DATA and CLOSE are not. That
//     asymmetry is the transfer's, not the layering's: opening is identical either way, and the
//     two middles differ in which payload they touch. Where the steps become table rows, the
//     two tables simply name the same OPEN.
// */
// /******************************************************************************/
