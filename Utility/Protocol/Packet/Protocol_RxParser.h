// #include "Packet.h"
// #include "Protocol.h"




// // Protocol_RxCode_T Packet_ParseRx(const Packet_Format_T * p_specs, uint8_t packet, uint8_t rxIndex, Protocol_HeaderMeta_T * p_rxMeta)


// typedef struct Packet_RxState
// {
//     Protocol_RxState_T StateId;
//     packet_size_t RxIndex;
//     Protocol_HeaderMeta_T RxMeta;
//     Protocol_RxCode_T rxStatus; /* Status of the parsing operation */
//     uint16_t Checksum;   /* Running checksum */
// }
// Packet_RxState_T;

// typedef const struct Packet_Context
// {
//     const Packet_Format_T * P_SPECS;
//     uint8_t * P_BUFFER;
//     Packet_RxState_T * P_RX_STATE;
//     Protocol_HeaderMeta_T * P_META;
// }
// Packet_Context_T;

// /*!
//     Delegate to protocol-specific parser once target bytes are available.
//     Caller must ensure p_rx->RxIndex >= target before calling.
// */
// static inline Protocol_RxCode_T Packet_ParseRxFraming(const Packet_Context_T * p_context)
// {
//     p_context->P_SPECS->PARSE_RX_META(p_context->P_BUFFER, p_context->P_RX_STATE->RxIndex, &p_context->P_RX_STATE->RxMeta);
// }

// /*!
//     Reset framing state for a new packet (start byte already consumed).
// */
// static inline void Packet_RxBegin(Packet_RxState_T * p_rx)
// {
//     p_rx->RxIndex = 1U;
//     p_rx->RxMeta.Length = 0U;
//     p_rx->RxMeta.Id = 0U;
// }

// /*!
//     Validate start byte against format spec.
// */
// static inline bool Packet_IsStartByte(const Packet_Format_T * p_specs, uint8_t byte)
// {
//     return (byte == p_specs->RX_START_ID) || (p_specs->RX_START_ID == 0x00U);
// }

// /*!
//     Compute the byte target for the next read.
//     Pure function — no transport, no side effects.
//     @return target index, or 0 on framing error
// */
// static inline packet_size_t Packet_RxTarget(const Packet_Format_T * p_specs, const Packet_RxState_T * p_rx)
// {
//     packet_size_t target;

//     if (p_rx->RxMeta.Length > 0U)
//     {
//         if (p_rx->RxIndex < p_rx->RxMeta.Length)
//             target = p_rx->RxMeta.Length;
//         else
//             return 0U; /* error: past expected length */
//     }
//     else if (p_rx->RxIndex < p_specs->RX_LENGTH_MIN)
//     {
//         target = p_specs->RX_LENGTH_MIN;
//     }
//     else
//     {
//         target = p_rx->RxIndex + 1U;
//     }

//     return (target <= p_specs->RX_LENGTH_MAX) ? target : 0U;
// }

// ///
