// #include "Packet.h"
// /*
//     Receive into P_RX_PACKET_BUFFER and run PARSE_RX_META for RxMeta.Length and ReqCode / Rx completion
//     Packet is complete => Req, ReqExt or Sync, or Error
// */
// static inline Protocol_RxCode_T CaptureRx(const Socket_T * p_socket, Socket_State_T * p_state)

// typedef struct PacketParser
// {
//     uint8_t * p_RxBuffer;
//     packet_size_t rxIndex;
//     packet_size_t rxLengthMax;
//     packet_size_t rxLengthMin;
//     Protocol_HeaderMeta_T rxMeta;
//     const Packet_ParseRxMeta_T parseRxMeta;
//     const Xcvr_T * p_Xcvr;
// } PacketParser_T;


// Protocol_RxCode_T PacketParser_CaptureRx(PacketParser_T * parser)
// {
//     Protocol_RxCode_T rxStatus = PROTOCOL_RX_CODE_AWAIT_PACKET;
//     packet_size_t nextRxIndex;

//     while (parser->rxIndex < parser->rxLengthMax)
//     {
//         if (rxStatus != PROTOCOL_RX_CODE_AWAIT_PACKET) break;

//         if (parser->rxMeta.Length > 0U)
//         {
//             nextRxIndex = (parser->rxIndex < parser->rxMeta.Length) ? parser->rxMeta.Length : parser->rxLengthMax; // Simplified error handling
//         }
//         else
//         {
//             nextRxIndex = (parser->rxIndex < parser->rxLengthMin) ? parser->rxLengthMin : parser->rxIndex + 1U;
//         }

//         if (nextRxIndex > parser->rxLengthMax)
//         {
//             rxStatus = PROTOCOL_RX_CODE_ERROR_META;
//             break;
//         }

//         parser->rxIndex += Xcvr_RxMax(parser->p_Xcvr, &parser->p_RxBuffer[parser->rxIndex], nextRxIndex - parser->rxIndex);

//         if (parser->rxIndex == nextRxIndex)
//         {
//             rxStatus = parser->parseRxMeta(&parser->rxMeta, parser->p_RxBuffer, parser->rxIndex);
//         }
//         else
//         {
//             break; // Buffer empty
//         }
//     }

//     return rxStatus;
// }