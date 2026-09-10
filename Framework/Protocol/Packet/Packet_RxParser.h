// #pragma once

// /******************************************************************************/
// /*!
//     @section LICENSE

//     Copyright (C) 2026 FireSourcery

//     This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).

//     This program is free software: you can redistribute it and/or modify
//     it under the terms of the GNU General Public License as published by
//     the Free Software Foundation, either version 3 of the License, or
//     (at your option) any later version.

//     This program is distributed in the hope that it will be useful,
//     but WITHOUT ANY WARRANTY; without even the implied warranty of
//     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//     GNU General Public License for more details.

//     You should have received a copy of the GNU General Public License
//     along with this program.  If not, see <https://www.gnu.org/licenses/>.
// */
// /******************************************************************************/
// /******************************************************************************/
// /*!
//     @file   Packet_RxParser.h
//     @author FireSourcery
//     @brief  Frame boundary detection. Bytes in, one Packet_RxCode_T out.
// */
// /******************************************************************************/
// #include "Packet.h"

// #include <stdint.h>
// #include <stdbool.h>

// /******************************************************************************/
// /*
//     Pure. No Xcvr, no timer, no transmit. The parser holds a frame's progress and nothing else,
//     so it can be exercised with a buffer and a struct.

//     Control is inverted: the parser says how many bytes it needs, the caller supplies them.
//     That keeps the byte source out of this header, and makes the byte target unambiguous in
//     every phase, so no index arithmetic is needed.

//         START       1 byte                  start delimiter
//         HEADER      LENGTH_MIN           enough for PARSE_RX_FRAMING to yield Id + Length
//         PAYLOAD     Header.Length           the whole frame, for PARSE_RX_HEADER to validate

//     Two-phase, per the Packet_Format_T contract documented in Packet.h.
// */
// /******************************************************************************/

// /******************************************************************************/
// /*!
//     Rx Code
// */
// /******************************************************************************/
// /*
//     User functions return status - communicate module handled behaviors
//     Handle status that can be determined by the framing/header alone.

//     include sync ids from framing/header
//     overload to eliminate need of addtional parser functions
// */
// typedef enum Packet_RxCode
// {
//     PACKET_RX_AWAIT,            /* Frame incomplete. Keep feeding. */
//     PACKET_RX_COMPLETE,         /* Valid data packet in the buffer */
//     // PACKET_RX_CONTROL,
//     PACKET_RX_ERROR_FRAME,      /* Unusable header - bad id, or a length that cannot describe the frame */
//     PACKET_RX_ERROR_DATA,       /* Payload failed validation */
//     PACKET_RX_TIMEOUT,          /* Frame stalled. Never returned by the parser - the caller owns the deadline. */
// }
// Packet_RxCode_T;

// /******************************************************************************/
// /*!
//     Parser State
// */
// /******************************************************************************/
// typedef enum Packet_RxState
// {
//     PACKET_RX_STATE_START,      /* Scanning for the delimiter */
//     PACKET_RX_PHASE_HEADER,     /* Collecting enough header to learn Id and Length. MIN_LENGTH or greater */
//     PACKET_RX_PHASE_PAYLOAD,    /* Collecting the rest of the frame */

//     // PROTOCOL_RX_STATE_INACTIVE,
//     // PROTOCOL_RX_STATE_WAIT_BYTE_1, /* SYNC */
//     // PROTOCOL_RX_STATE_WAIT_LENGTH, /* HEADER */
//     // PROTOCOL_RX_STATE_WAIT_PACKET, /* DATA */
//     // PROTOCOL_RX_STATE_WAIT_REQ_SIGNAL,
//     //     PROTOCOL_PARSE_STATE_SYNC,          /* Looking for sync/start marker */
//     //     PROTOCOL_PARSE_STATE_HEADER,        /* Parsing header */
//     //     PROTOCOL_PARSE_STATE_PAYLOAD,       /* Parsing payload */
//     //     PROTOCOL_PARSE_STATE_CHECKSUM,      /* Parsing checksum/CRC */
//     //     PROTOCOL_PARSE_STATE_COMPLETE,      /* Packet complete */
//     //     PROTOCOL_PARSE_STATE_ERROR          /* Error state */
// }
// Packet_RxState_T;

// typedef struct Packet_RxParser
// {
//     Packet_RxState_T StateId; //    Protocol_RxState_T RxState;
//     packet_size_t Index;            /* Bytes held in the buffer */ /* Index into P_RX_PACKET_BUFFER, number of bytes received */
//     // seperate parser state.
//     packet_size_t Length;           /* Total packet length */
//     packet_id_t Id;                 /* Packet type identifier. Index into P_REQ_TABLE */
//     uint32_t RxTimeStart;
// }
// Packet_RxParser_T;

// static inline bool _Packet_RxLengthRemaining(const Packet_RxParser_T * p_parser) { return (p_parser->Length - p_parser->Index); }

// /*!
//     Phase 1 — Framing / Boundary Detection
//     Called by CaptureRx when RxIndex >= LENGTH_MIN and Length is unknown.
//     Extracts Id and Length from partial header.
//     Sync packets return completion code directly (ACK/NACK/ABORT).
//     Data packets set p_meta->Length and return AWAIT_PACKET.
//     Phase 1 errors:  ERROR_META(invalid ID, nonsensical length)
//     Phase 1 control : ACK, NACK, ABORT(sync packets — no Phase 2)

//     do not pass Packet_RxParser_T to keep statemachine handler
// */
// // typedef Protocol_RxCode_T(*Packet_ParseRxHandler_T)(const void * p_buffer, packet_size_t rxCount );

// // force lenght first. return 0 for unknown, after min, per byte call.
// typedef packet_size_t(*Packet_ParseRxLength_T)(const void * p_buffer, packet_size_t rxCount);
// typedef packet_id_t(*Packet_ParseRxId_T) (const void * p_buffer);
// typedef bool (*Packet_ValidateRx_T) (const void * p_buffer, packet_size_t length);



// /*!
//     Validate start byte against format spec.
// */
// // static inline bool Packet_IsStartByte(Packet_Format_T * p_specs, uint8_t byte)
// // {
// //     return (byte == p_specs->START_ID) || (p_specs->START_ID == 0x00U);
// // }

// static inline void Packet_RxReset(Packet_RxParser_T * p_parser)
// {
//     p_parser->StateId = PACKET_RX_STATE_START;
//     p_parser->Index = 0U;
//     p_parser->Length = 0U;
// }

// /*! true once a delimiter has been accepted and the frame is still incomplete. */
// // static inline bool Packet_RxParser_IsInFrame(const Packet_RxParser_T * p_parser) { return (p_parser->StateId != PACKET_RX_STATE_START); }

// /*!
//     Reset framing state for a new packet (start byte already consumed).
// */
// static inline void Packet_RxBegin(Packet_RxParser_T * p_rx)
// {
//     p_rx->Index = 1U;
//     p_rx->Length = 0U;
//     p_rx->Id = 0U;
// }

// /******************************************************************************/
// /*!
//     Proc
// */
// /******************************************************************************/
// /*!
//     @brief  Bytes the buffer must hold before Advance may run.
//             Bounded by LENGTH_MAX, which Advance enforces on the length field.
// */
// static inline packet_size_t Packet_RxParser_TargetCount(Packet_Format_T * p_format, const Packet_RxParser_T * p_parser)
// {
//     switch (p_parser->StateId)
//     {
//         case PACKET_RX_PHASE_HEADER:    return p_format->LENGTH_MIN;
//         case PACKET_RX_PHASE_PAYLOAD:   return p_parser->Length;
//         case PACKET_RX_STATE_START:
//         default:                        return 1U;
//     }
// }


// /*!
//     @brief  Interpret a met target. Call only when Count >= TargetCount.
//     @return AWAIT to keep collecting. Anything else resolves the frame and resets the parser.
// */
// static inline Packet_RxCode_T Packet_RxParser_Advance(Packet_Format_T * p_format, const uint8_t * p_buffer, Packet_RxParser_T * p_parser)
// {
//     Packet_RxCode_T rxCode = PACKET_RX_AWAIT;

//     switch (p_parser->StateId)
//     {
//         case PACKET_RX_STATE_START:
//             /* Anything but the delimiter is dropped, one byte at a time. RX_START_ID of 0 accepts every byte. */
//             if ((p_buffer[0U] == p_format->START_ID) || (p_format->START_ID == 0x00U)) { p_parser->StateId = PACKET_RX_PHASE_HEADER; }
//             else { p_parser->Index = 0U; }
//             break;

//         case PACKET_RX_PHASE_HEADER:
//             /* Phase 1: Id + Length. Control packets carry no payload and resolve here. */
//             rxCode = Packet_RxCodeOf(p_format->PARSE_RX_FRAMING(p_buffer, p_parser->Index, &p_parser->Header));
//             if (rxCode == PACKET_RX_AWAIT)
//             {
//                 /* A length that cannot describe this frame makes everything after it meaningless. */
//                 if ((p_parser->Header.Length < p_parser->Count) || (p_parser->Header.Length > p_format->LENGTH_MAX)) { rxCode = PACKET_RX_ERROR_FRAME; }
//                 else { p_parser->StateId = PACKET_RX_PHASE_PAYLOAD; }
//             }
//             break;

//         case PACKET_RX_PHASE_PAYLOAD:
//             /* Phase 2: validate the whole frame. Always resolves. */
//             rxCode = Packet_RxCodeOf(p_format->PARSE_RX_HEADER(p_buffer, &p_parser->Header));
//             break;

//         default:
//             break;
//     }

//     if (rxCode != PACKET_RX_AWAIT) { Packet_RxParser_Reset(p_parser); }

//     return rxCode;
// }



// /*  */
// /* directly mapped to count */
// static inline Protocol_RxState_T _Packet_RxStateOf(Packet_Format_T * p_specs, size_t rxCount)
// {
//     if (rxCount == 0U) { return PROTOCOL_RX_STATE_WAIT_BYTE_1; }
//     else if (rxCount < p_specs->LENGTH_MIN) { return PROTOCOL_RX_STATE_WAIT_LENGTH; }
//     // else if (rxCount < p_state->RxMeta.Length) { return PROTOCOL_RX_STATE_WAIT_PACKET; }
//     else if (rxCount < p_specs->LENGTH_MAX) { return PROTOCOL_RX_STATE_WAIT_PACKET; }
//     else { return PROTOCOL_RX_STATE_WAIT_BYTE_1; } /* Invalid length, reset */
// }

// static inline Protocol_RxCode_T _Packet_ProcRxParser(Packet_Format_T * p_specs, const uint8_t * p_rxBuffer, packet_size_t * p_rxIndex, Protocol_HeaderMeta_T * p_rxMeta)
// {
//     Protocol_RxCode_T rxStatus = PROTOCOL_RX_CODE_AWAIT_PACKET;
//     switch (_Packet_RxStateOf(p_specs, *p_rxIndex))
//     {
//         case PROTOCOL_RX_STATE_WAIT_BYTE_1:
//             if (*p_rxIndex > 0U)
//             {
//                 if ((p_rxBuffer[0U] == p_specs->START_ID) || (p_specs->START_ID == 0x00U)) { p_rxMeta->Length = 0U; }
//                 else { *p_rxIndex = 0U; }                // reset and keep waiting
//             }
//             break;
//         case PROTOCOL_RX_STATE_WAIT_LENGTH:   if (*p_rxIndex >= p_specs->LENGTH_MIN) { rxStatus = p_specs->PARSE_RX_FRAMING(p_rxBuffer, *p_rxIndex, p_rxMeta); }  break;
//         case PROTOCOL_RX_STATE_WAIT_PACKET:   if (*p_rxIndex >= p_rxMeta->Length) { rxStatus = p_specs->PARSE_RX_HEADER(p_rxBuffer, p_rxMeta); } break; // PACKET_COMPLETE or ERROR_DATA
//         case PROTOCOL_RX_STATE_INACTIVE:    break;
//         default: break;
//     }
//     if (rxStatus != PROTOCOL_RX_CODE_AWAIT_PACKET) { *p_rxIndex = 0U; } //   p_state->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1;
//     return rxStatus;
// }



// /* ─── runs once per phase — three or four times per frame ─────────────────── */
// static void rx_advance(frame_rx_t * rx)
// {
//     if (rx->state == RX_HDR)
//     {
//         cur_t c = { rx->buf, rx->buf + rx->idx, false };
//         switch (rx->ops->parse_hdr(&c, &rx->meta, &rx->L))
//         {
//             case FR_NEED_MORE:                                   /* variable-length header */
//                 rx->need = rx->L.hdr_len - rx->idx;              /* codec said how much more */
//                 return;
//             case FR_NOMATCH:
//                 rx->stat.resync++;  rx_reset(rx);  return;
//             case FR_OK:
//                 if (rx->L.body_len > rx->cap - rx->idx)
//                 {        /* THE bounds check */
//                     rx->stat.oversize++;  rx_reset(rx);  return;
//                 }
//                 rx->state = RX_BODY;  rx->need = rx->L.body_len;  return;
//         }
//     }

//     /* RX_BODY complete: idx is now wire_len */
//     if (!rx->ops->verify(rx->buf, &rx->meta, &rx->L))
//     {
//         rx->stat.crc_err++;  rx_reset(rx);  return;
//     }
//     rx->stat.frames++;
//     rx->sink(rx->app, &rx->meta,
//         rx->buf + rx->L.hdr_len,
//         rx->L.body_len - rx->L.trailer_len);            /* payload_len, derived here */
//     rx_reset(rx);
// }

// /* ─── the drive loop: scan, or gather; never a per-byte switch ────────────── */
// void frame_feed(frame_rx_t * rx, const uint8_t * in, size_t n, uint32_t now_us)
// {
//     if (rx->state != RX_SCAN && (now_us - rx->last_us) > RX_IDLE_US)
//     {
//         rx->stat.timeout++;  rx_reset(rx);
//     }
//     rx->last_us = now_us;

//     while (n)
//     {
//         if (rx->state == RX_SCAN)
//         {
//             const uint8_t * s = memchr(in, rx->ops->sync, n);
//             if (!s) return;                                  /* whole chunk discarded */
//             n -= (size_t)(s - in);  in = s;
//             rx->state = RX_HDR;  rx->need = rx->ops->hdr_min;
//         }
//         size_t take = MIN(rx->need, n);
//         memcpy(rx->buf + rx->idx, in, take);                 /* provably in bounds */
//         rx->idx += take;  in += take;  n -= take;  rx->need -= take;
//         if (rx->need) return;                                /* chunk ended mid-phase */
//         rx_advance(rx);
//     }
// }


