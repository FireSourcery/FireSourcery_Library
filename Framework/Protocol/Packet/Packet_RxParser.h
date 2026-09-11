// static inline Packet_RxCode_T Packet_ProcRxParser(Packet_RxParser_T * p_parser, const Packet_Format_T * p_format, const uint8_t * p_buffer)
// {
//     Packet_RxCode_T rxCode = PACKET_RX_AWAIT;

//     switch (p_parser->StateId)
//     {
//         case PACKET_RX_STATE_START:
//             /*
//                 Anything but the delimiter is dropped, a byte at a time. START_ID of 0 accepts every byte.

//                 The reject branch rescans from the front. Index-- would not: the buffer does not slide,
//                 so RxN refills at &p_buffer[Index] and p_buffer[0] is retested against fresh data
//                 forever - one stray byte would desync the port permanently.

//                 The accept and reject branches set NextIndex separately. A single trailing
//                 NextIndex = LENGTH_MIN cannot serve both: bulk-reading LENGTH_MIN while scanning
//                 inside it needs the delimiter shifted to the front (memmove / memchr, and a non-const
//                 p_buffer). Scanning a byte at a time costs one extra RxN per frame on a clean line,
//                 where the delimiter is already byte 0.
//             */
//             if (p_parser->Index >= p_format->START_ID_LENGTH)
//             {
//                 /* START_ID first: with START_ID_LENGTH of 0 the buffer holds nothing to read yet. */
//                 if ((p_format->START_ID == 0x00U) || (p_buffer[0U] == p_format->START_ID))
//                 {
//                     p_parser->StateId = PACKET_RX_STATE_HEADER;
//                     p_parser->FrameLength = 0U;
//                     p_parser->NextIndex = p_format->LENGTH_MIN;
//                 }
//                 else
//                 {
//                     p_parser->Index = 0U;                                   /* rescan from the front */
//                     p_parser->NextIndex = p_format->START_ID_LENGTH;
//                 }
//             }
//             else
//             {
//                 p_parser->NextIndex = p_format->START_ID_LENGTH;
//             }
//             break;

//         case PACKET_RX_STATE_HEADER: /* Wait for Length */
//             p_parser->FrameLength = p_format->PARSE_RX_LENGTH(p_buffer, p_parser->Index);

//             if (p_parser->FrameLength > 0U) { p_parser->NextIndex = p_parser->FrameLength; } /* PacketLength is known. */
//             else { p_parser->NextIndex = p_parser->Index + 1U; }  /* PacketLength is unknown. p_format->LENGTH_MIN already reached */

//             /* Check Length through Target */
//             /* (RxIndex == nextRxIndex) => (RxSize == 0), when rxStatus == PROTOCOL_RX_CODE_WAIT_PACKET erroneously i.e. received full packet without completion status */
//             if (p_parser->NextIndex > p_format->LENGTH_MAX || p_parser->NextIndex < p_parser->Index) { rxCode = PACKET_RX_ERROR_FRAME; }
//             else if (p_parser->NextIndex == p_parser->FrameLength) { p_parser->StateId = PACKET_RX_STATE_PAYLOAD; } /* p_parser->Target != 0U  */

//             // /* Declined to answer. Grow the header by a byte, until it can no longer become a frame. */
//             // if (p_parser->Length == 0U)
//             // {
//             //     if (p_parser->Index >= p_format->LENGTH_MAX) { rxCode = PACKET_RX_ERROR_FRAME; }
//             //     else { p_parser->Target = p_parser->Index + 1U; }
//             // }
//             // else
//             // {
//                 // /* A length that cannot describe this frame makes everything after it meaningless. */
//             //     if ((p_parser->Length < p_parser->Index) || (p_parser->Length > p_format->LENGTH_MAX))
//             //     {
//             //         rxCode = PACKET_RX_ERROR_FRAME;
//             //     }
//             //     else
//             //     {
//             //         p_parser->StateId = PACKET_RX_STATE_PAYLOAD;
//             //         p_parser->Target = p_parser->Length;
//             //     }
//             // }
//             break;

//         case PACKET_RX_STATE_PAYLOAD:
//             // assert(p_parser->Index == p_parser->NextIndex); /* Ensure the whole payload has been received */
//             // assert(p_parser->Index == p_parser->Length); /* Ensure the whole payload has been received */
//             /* The whole frame is present, so this always resolves. */
//             /* Frame is complete. caller parse remaining meta with PARSE_RX_HEADER */
//             rxCode = (p_format->IS_RX_VALID(p_buffer, p_parser->FrameLength) == true) ? PACKET_RX_COMPLETE : PACKET_RX_ERROR_DATA;
//             break;

//         default:
//             break;
//     }
//     /*
//         Continue CaptureRx during ReqExt processing
//         Buffers may be overwritten after Req returns. (No repeat process on same Rx)
//         Req ensure packet data is processed, or copied
//         Rx can queue out of sequence. Invalid Rx sequence until timeout buffer flush

//         Alternatively, pause CaptureRx during ReqExt processing
//         Incoming packet bytes wait in queue. Cannot miss packets (unless overflow)
//         Cannot check for Abort without user signal, persistent wait process
//     */
//     /* Rewind. Index must clear with the state, or the next frame builds from a stale offset. */
//     if (rxCode != PACKET_RX_AWAIT) { Packet_ResetRx(p_parser); }

//     return rxCode;
// }
// static inline Packet_RxCode_T Protocol_CaptureRx(const Xcvr_T * p_xcvr, const Packet_Format_T * p_format, Packet_RxParser_T * p_parser, uint8_t * p_rxBuffer)
// {
//     Packet_RxCode_T rxCode = PACKET_RX_AWAIT;

//     while (rxCode == PACKET_RX_AWAIT)
//     {
//         Packet_RxState_T stateBefore = p_parser->StateId;
//         packet_size_t nextBefore = p_parser->NextIndex;

//         rxCode = Packet_ProcRxParser(p_parser, p_format, p_rxBuffer);
//         if (rxCode != PACKET_RX_AWAIT) { break; }   /* resolved - the buffer is the caller's now */

//         packet_size_t remaining = Packet_RxRemaining(p_parser);

//         if (remaining == 0U)
//         {
//             /* Nothing wanted and nothing moved: the format cannot make progress from here. */
//             if ((p_parser->StateId == stateBefore) && (p_parser->NextIndex == nextBefore)) { break; }
//             continue;   /* a phase boundary fell exactly on the frame end - run the next phase */
//         }

//         /* Nothing moves until the whole target is available. */
//         if (Xcvr_RxN(p_xcvr, &p_rxBuffer[p_parser->Index], remaining) == false) { break; }
//         p_parser->Index += remaining;
//     }

//     return rxCode;
// }
