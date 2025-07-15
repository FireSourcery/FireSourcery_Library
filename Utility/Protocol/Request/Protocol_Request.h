
// /******************************************************************************/
// /*!
//     Request / Request Table

//     ProcReqResp - interface for Rx and Tx packets
//     User provide functions to convert between Packet format and appInterface format

//     User may config appInterface as
//         appInterface => buffer once, unrestricted scope
//         packetInterface => parse p_rxPacket to packetInterface, then call proc on buffered data => buffers twice
// */
// /******************************************************************************/
// /******************************************************************************/
// /*
//     Stateless Req, fit for simple read write.
//     Shared ReqResp function allows function to maintain temporary local state
//     @return txSize
// */
// /******************************************************************************/
// typedef packet_size_t(*Protocol_ProcReqResp_T)(void * p_appInterface, uint8_t * p_txPacket, const uint8_t * p_rxPacket);
// // alternatively,
// //req response
// // typedef packet_size_t (*MotProtocol_Fixed_T)(const void * p_context, MotPayload_T * p_txPayload, const MotPayload_T * p_rxPayload);
// // typedef packet_size_t (*MotProtocol_Variable_T)(const void * p_context, MotPayload_T * p_txPayload, const MotPayload_T * p_rxPayload, packet_size_t rxLength);

// // typedef packet_size_t(*Protocol_ParseProcReq_T)(void * p_appInterface, const uint8_t * p_rxPayload);
// // typedef packet_size_t(*Protocol_BuildResponse_T)(void * p_appInterface, uint8_t * p_txPayload);

// /******************************************************************************/
// /*
//     Extended/Stateful Request - Support wait, loop, dynamic ack/nack and additional processes
// */
// /******************************************************************************/
// /* Common Req from child protocol to supported general protocol control, predefined behaviors */
// typedef enum Protocol_ReqCode
// {
//     // PROTOCOL_REQ_CODE_AWAIT_RX_REQ_INITIAL,
//     PROTOCOL_REQ_CODE_TX_CONTINUE,             /* continue using default sync settings, wait for next packet *///option split with tx send
//     // PROTOCOL_REQ_CODE_PROCESS_AWAIT_RX,        /* Expecting Rx new packet */
//     PROTOCOL_REQ_CODE_PROCESS_COMPLETE,             /* Exit nonblocking wait processing state upon reception */
//     // PROTOCOL_REQ_CODE_PROCESS_COMPLETE_WITH_ERROR,
//     PROTOCOL_REQ_CODE_ABORT, /* Terminate */

//     /* ack after process */
//     PROTOCOL_REQ_CODE_PROCESS_ACK,
//     PROTOCOL_REQ_CODE_PROCESS_NACK,

//     /* User Function Manual select next step */
//     PROTOCOL_REQ_CODE_AWAIT_RX_CONTINUE,        /* Expecting Rx new packet */
//     PROTOCOL_REQ_CODE_AWAIT_RX_SYNC,            /* Expecting static ack nack */
//     PROTOCOL_REQ_CODE_TX_RESPONSE,
//     PROTOCOL_REQ_CODE_TX_ACK,
//     PROTOCOL_REQ_CODE_TX_NACK,

//     /* Error */
//     PROTOCOL_REQ_CODE_ERROR_ID,             /* ID not found */
//     PROTOCOL_REQ_CODE_ERROR_RX_UNEXPECTED,  /* Out of sequence packet */
//     PROTOCOL_REQ_CODE_ERROR_TIMEOUT,

//     //remove
//     // PROTOCOL_REQ_CODE_AWAIT_PROCESS,                 /* Child Protocol NonBlocking Wait ReqExt processing */
//     // PROTOCOL_REQ_CODE_AWAIT_PROCESS_EXTEND_TIMER,    /* Child Protocol NonBlocking Wait, Extend timer for RxLost and Timeout */
// }
// Protocol_ReqCode_T;

// /*
//     Stateful Req function, pass args collectively,
//     alternatively pass by struct and let compiler handle
// */
// typedef const struct Protocol_ReqContext
// {
//     void * const p_SubState;
//     uint32_t * const p_SubStateIndex;
//     const Protocol_HeaderMeta_T * const p_RxMeta;
//     const void * const p_RxPacket;
//     void * const p_TxPacket;
//     packet_size_t * const p_TxSize;
// }
// Protocol_ReqContext_T;

// typedef Protocol_ReqCode_T(*Protocol_ProcReqExt_T)(void * p_appInterface, Protocol_ReqContext_T * p_interface);

// typedef void (*Protocol_ResetReqState_T)(void * p_subState);

// /*
//     Sync Options - Configure per Req, uses common timeout
//     Stateless static string sync, ack, nack - Ack Nack Packet_BuildTxSync_T
//     Dynamic ack nack string implementation use Protocol_ReqExtFunction_T
// */
// typedef const union Protocol_ReqSync
// {
//     struct
//     {
//         uint32_t TX_ACK : 1U;   /* Tx Ack after Rx initial Request*/
//         uint32_t RX_ACK : 1U;   /* Wait for Rx Ack after Tx Response */
//         uint32_t NACK_REPEAT : 3U;   /* Common setting. Number of repeat TxPacket on Rx Nack, Tx Nack on RxPacket error */
//         uint32_t TX_ACK_EXT : 1U;   /* Use for All Stateful Ext Request  */
//         uint32_t RX_ACK_EXT : 1U;
//         uint32_t TX_ACK_ABORT : 1U;   /* Use for All Stateful Ext Request  */
//     };
//     uint32_t ID;
// }
// Protocol_ReqSync_T;

// #define PROTOCOL_SYNC_DISABLE \
//     { .TX_ACK = 0U, .RX_ACK = 0U, .TX_ACK_EXT = 0U, .RX_ACK_EXT = 0U, .NACK_REPEAT = 0U, }

// #define PROTOCOL_SYNC(UseTxAck, UseRxAck, NackRepeat) \
//     { .TX_ACK = UseTxAck, .RX_ACK = UseRxAck, .TX_ACK_EXT = 0U, .RX_ACK_EXT = 0U, .NACK_REPEAT = NackRepeat, }

// #define PROTOCOL_SYNC_EXT(UseTxAck, UseRxAck, UseTxAckExt, UseRxAckExt, NackRepeat) \
//     { .TX_ACK = UseTxAck, .RX_ACK = UseRxAck, .TX_ACK_EXT = UseTxAckExt, .RX_ACK_EXT = UseRxAckExt, .NACK_REPEAT = NackRepeat, }

// /*
//     Protocol_Req_T
// */
// typedef const struct Protocol_Req
// {
//     const packet_id_t         ID;
//     const Protocol_ProcReqResp_T    PROC;
//     const Protocol_ProcReqExt_T     PROC_EXT;
//     const Protocol_ReqSync_T        SYNC;
//     //    const uint32_t     TIMEOUT; /* overwrite common timeout */
// }
// Protocol_Req_T;

// #define PROTOCOL_REQ(ReqId, ProcReqResp, ProcExt, ReqSync) \
//     { .ID = (packet_id_t)ReqId, .PROC = (Protocol_ProcReqResp_T)ProcReqResp, .PROC_EXT = (Protocol_ProcReqExt_T)ProcExt, .SYNC = ReqSync, }

// #define PROTOCOL_REQ_EXT(ReqId, ProcReqResp, ProcExt, ReqSyncExt) \
//     { .ID = (packet_id_t)ReqId, .PROC = (Protocol_ProcReqResp_T)ProcReqResp, .PROC_EXT = (Protocol_ProcReqExt_T)ProcExt, .SYNC = ReqSyncExt, }
