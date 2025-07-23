// #include "Packet.h"
// /*
//     Receive into P_RX_PACKET_BUFFER and run PARSE_RX_META for RxMeta.Length and ReqCode / Rx completion
//     Packet is complete => Req, ReqExt or Sync, or Error
// */
// static inline Protocol_RxCode_T CaptureRx(const Socket_T * p_socket, Socket_State_T * p_state)
// {
//     Protocol_RxCode_T rxStatus = PROTOCOL_RX_CODE_AWAIT_PACKET;
//     uint8_t nextRxIndex;
//     // uint8_t xcvrRxLimit; /* PARSE_RX_META after this number is received */
//     // uint8_t xcvrRxCount;

//     /* Loop to empty Xcvr Rx buffer. Check for completetion, per Rx 1 byte, during unknown length, or up to known length */
//     while (p_state->RxIndex < p_state->p_Specs->RX_LENGTH_MAX) /* RX_LENGTH_MAX <= CONFIG.PACKET_BUFFER_LENGTH */
//     {
//         if (rxStatus != PROTOCOL_RX_CODE_AWAIT_PACKET) { break; } /* Packet is complete, break */

//         /*
//             Set xcvrRxLimit for PARSE_RX_META. Prevent reading bytes from the following packet.
//             RxMeta.Length unknown => Check 1 byte or a known const value, until RxMeta.Length is known
//             RxMeta.Length known => Check Rx remaining
//         */
//         if (p_state->RxMeta.Length > 0U)  /* PacketLength is known. */
//         {
//             if (p_state->RxIndex < p_state->RxMeta.Length) { nextRxIndex = p_state->RxMeta.Length; }
//             else { rxStatus = PROTOCOL_RX_CODE_ERROR_META; break; }
//             /* (RxIndex == RxMeta.Length) => (xcvrRxLimit == 0), when rxStatus == PROTOCOL_RX_CODE_WAIT_PACKET erroneously i.e. received full packet without completion status */
//         }
//         /* PacketLength is unkown */
//         else if (p_state->RxIndex < p_state->p_Specs->RX_LENGTH_MIN) { nextRxIndex = p_state->p_Specs->RX_LENGTH_MIN; }
//         /* RX_LENGTH_INDEX > RX_LENGTH_MIN. Check for length determined by another property */
//         // else if(p_state->RxIndex < p_state->p_Specs->RX_REQ_ID_INDEX) { nextRxIndex = p_state->p_Specs->RX_REQ_ID_INDEX + 1U; }
//         // else if(p_state->RxIndex < p_state->p_Specs->RX_LENGTH_INDEX) { nextRxIndex = p_state->p_Specs->RX_LENGTH_INDEX + 1U; }
//         else { nextRxIndex = p_state->RxIndex + 1U; }

//         if (nextRxIndex > p_state->p_Specs->RX_LENGTH_MAX) { rxStatus = PROTOCOL_RX_CODE_ERROR_META; break; }
//         /* Copy from Xcvr buffer to Protocol buffer, up to xcvrRxLimit */
//         p_state->RxIndex += Xcvr_RxMax(p_state->p_Xcvr, &p_socket->P_RX_PACKET_BUFFER[p_state->RxIndex], nextRxIndex - p_state->RxIndex);

//         if (p_state->RxIndex == nextRxIndex) /* (xcvrRxCount == xcvrRxLimit) */
//         {
//             /* Implicitly: (xcvrRxCount != 0), (xcvrRxLimit != 0). (RxIndex >= RX_LENGTH_MIN)  */
//             /* returns PROTOCOL_RX_CODE_AWAIT_PACKET on successful set of meta data *//* more bytes in Xcvr Buffer, continue while loop */
//             rxStatus = p_state->p_Specs->PARSE_RX_META(&p_state->RxMeta, p_socket->P_RX_PACKET_BUFFER, p_state->RxIndex);
//         }
//         else /* xcvrRxCount < xcvrRxLimit => Xcvr Rx Buffer empty, wait for Xcvr */
//         {
//             break;  /*  rxStatus == PROTOCOL_RX_CODE_AWAIT_PACKET; */
//         }
//     }

//     return rxStatus;
// }
